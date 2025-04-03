#!/usr/bin/env python3
"""
PF Camera Rig Metadata Bridge for ROS2

This script creates a bridge between MQTT and ROS2 for the PF Camera Rig metadata.
It subscribes to MQTT messages from a Grover robot running ROS1, converts them to
ROS2 messages, and publishes them to ROS2 topics. It also logs the raw messages
to a JSON file for later analysis.

Author: Laurence Diack
Copyright (C) 2025, Peek Robotics
"""

import rclpy
from rclpy.node import Node
import json
from datetime import datetime
import os
import pathlib
import paho.mqtt.client as mqtt
import yaml
from grover_bridge.msg import PFCamRigMetadata
from rclpy_message_converter import message_converter


def convert_msg(data):
    """
    Convert ROS1 message format to ROS2 format.
    
    This function handles differences between ROS1 and ROS2 message formats,
    particularly in header fields and timestamp representations.
    
    Args:
        data: The message data to convert (dict, list, or primitive type)
        
    Returns:
        The converted data in ROS2-compatible format
    """
    if isinstance(data, dict):
        # Create a new dict to avoid modifying during iteration
        new_dict = {}
        
        # First pass: rename keys
        for key, value in data.items():
            # Convert key names for timestamp fields
            if key == 'secs':
                new_dict['sec'] = value
            elif key == 'nsecs':
                new_dict['nanosec'] = value
            else:
                new_dict[key] = value
                
        # Replace data with new_dict for further processing
        data = new_dict
        
        # Remove seq from header (ROS2 doesn't use sequence numbers in headers)
        if 'header' in data and isinstance(data['header'], dict):
            data['header'].pop('seq', None)
            
        # Handle standalone header-like dictionaries
        keys = set(data.keys())
        if data.get('seq') is not None and (
            keys == {'seq', 'stamp', 'frame_id'} or 
            keys == {'seq', 'stamp'} or
            keys == {'seq', 'sec', 'nanosec', 'frame_id'} or
            keys == {'seq', 'secs', 'nsecs', 'frame_id'} or
            keys == {'seq', 'sec', 'nanosec'} or
            keys == {'seq', 'secs', 'nsecs'}
        ):
            data.pop('seq', None)
            
        # Process all values recursively
        for key, value in list(data.items()):
            data[key] = convert_msg(value)
            
    elif isinstance(data, list):
        # Process all items in the list recursively
        data = [convert_msg(item) for item in data]
        
    return data


class GroverMetadataLogger(Node):
    """
    ROS2 Node that bridges MQTT messages from a Grover robot to ROS2 topics.
    
    This node:
    1. Connects to an MQTT broker
    2. Subscribes to configured topics (default: grover/cam_rig/metadata)
    3. Converts incoming messages from ROS1 to ROS2 format
    4. Publishes the converted messages to ROS2 topics
    5. Logs the raw messages to a JSON file
    """
    
    def __init__(self):
        """Initialize the GroverMetadataLogger node."""
        super().__init__('grover_metadata')
        
        # Load configuration from ROS parameter server
        self.config = self.load_config(None)
        
        # Set up output directory for message logging
        self.output_dir = self.config.get('output_dir', os.getcwd())
        pathlib.Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        
        # Create output file with timestamp in filename
        self.startup_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_file = os.path.join(self.output_dir, f"messages_{self.startup_time}.json")

        # Create ROS2 publisher for the converted messages
        self.publisher = self.create_publisher(
            PFCamRigMetadata,
            'grover/cam_rig/metadata',
            10
        )
        
        # Set up MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # Connect to MQTT broker
        self.mqtt_ip = self.config.get('mqtt_broker_ip', 'localhost')
        self.mqtt_port = self.config.get('mqtt_broker_port', 1883)
        self.mqtt_keepalive = self.config.get('mqtt_keepalive', 60)
        
        self.get_logger().info(f'Connecting to MQTT broker at {self.mqtt_ip}:{self.mqtt_port}')
        self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port, self.mqtt_keepalive)
        
        # Start the MQTT client loop in a non-blocking way
        self.mqtt_client.loop_start()
        
        self.get_logger().info(f'Message logger initialized. Logging to: {self.output_file}')
    
    def load_config(self, config_file):
        """
        Load configuration from ROS parameter server.
        
        Args:
            config_file: Unused, kept for backward compatibility
            
        Returns:
            dict: Configuration dictionary with MQTT and output settings
        """
        try:
            # Get MQTT configuration parameters from ROS param server
            config = {}
            
            # Declare and get parameters with defaults
            self.declare_parameter('mqtt.broker_ip', 'localhost')
            self.declare_parameter('mqtt.broker_port', 1883)
            self.declare_parameter('mqtt.keepalive', 60)
            self.declare_parameter('mqtt.topics', ['grover/cam_rig/metadata'])
            self.declare_parameter('output_dir', os.getcwd())
            
            # Build config dictionary from parameters
            config['mqtt_broker_ip'] = self.get_parameter('mqtt.broker_ip').value
            config['mqtt_broker_port'] = self.get_parameter('mqtt.broker_port').value
            config['mqtt_keepalive'] = self.get_parameter('mqtt.keepalive').value
            config['mqtt_topics'] = self.get_parameter('mqtt.topics').value
            config['output_dir'] = self.get_parameter('output_dir').value
            
            self.get_logger().info(f'Loaded MQTT configuration from ROS parameters')
            return config
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config from ROS parameters: {e}')
            # Return default configuration
            return {
                'mqtt_broker_ip': 'localhost',
                'mqtt_broker_port': 1883,
                'mqtt_keepalive': 60,
                'mqtt_topics': ['grover/cam_rig/metadata'],
                'output_dir': os.getcwd()
            }

    def dict_to_ros_message(self, message_type, dictionary):
        """
        Convert a Python dictionary to a ROS message object.
        
        Args:
            message_type (str): The ROS message type (e.g., 'grover_bridge/msg/PFCamRigMetadata')
            dictionary (dict): The dictionary to convert
            
        Returns:
            ROS message object or None if conversion fails
        """
        # First convert ROS1 format to ROS2 format
        dictionary = convert_msg(dictionary)
        try:
            # Use rclpy_message_converter to convert dict to ROS message
            return message_converter.convert_dictionary_to_ros_message(
                message_type, dictionary
            )
        except Exception as e:
            self.get_logger().error(f'Error converting dictionary to ROS message: {e}')
            # Fallback method: try direct conversion
            try:
                message = PFCamRigMetadata()
                for key, value in dictionary.items():
                    if hasattr(message, key):
                        setattr(message, key, value)
                return message
            except Exception as inner_e:
                self.get_logger().error(f'Fallback conversion failed: {inner_e}')
                return None
    
    def on_connect(self, client, userdata, flags, rc):
        """
        Callback when the client connects to the MQTT broker.
        
        Args:
            client: The MQTT client instance
            userdata: User data passed to the callback
            flags: Response flags from the broker
            rc: Response code from the broker
        """
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            # Subscribe to topics from configuration
            topics = self.config.get('mqtt_topics', ['grover/cam_rig/metadata'])
            for topic in topics:
                self.mqtt_client.subscribe(topic)
                self.get_logger().info(f'Subscribed to topic: {topic}')
        else:
            self.get_logger().error(f'Failed to connect to MQTT broker with code: {rc}')
    
    def on_message(self, client, userdata, msg):
        """
        Callback when a message is received from the broker.
        
        This method:
        1. Decodes the message payload
        2. Logs it to a file
        3. Converts it to a ROS2 message
        4. Publishes it to the ROS2 topic
        
        Args:
            client: The MQTT client instance
            userdata: User data passed to the callback
            msg: The received message
        """
        try:
            # Get the payload as a string
            payload = msg.payload.decode('utf-8')
            self.get_logger().info(f'Received message on topic {msg.topic}')
                        
            # Append to file
            with open(self.output_file, 'a') as f:
                f.write(payload + '\n')

            # Convert JSON to dictionary
            msg_dict = json.loads(payload)
            
            # Convert dictionary to ROS message
            ros_msg = self.dict_to_ros_message('grover_bridge/msg/PFCamRigMetadata', msg_dict)
            if ros_msg:
                self.publisher.publish(ros_msg)
                self.get_logger().debug('Published message to ROS2 topic')
            else:
                self.get_logger().warning('Failed to convert message to ROS2 format')
                
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

    def shutdown(self):
        """Clean shutdown of MQTT client."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        self.get_logger().info('MQTT client disconnected')


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    logger_node = GroverMetadataLogger()
    
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        logger_node.shutdown()
        logger_node.get_logger().info('Shutting down message logger')
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
