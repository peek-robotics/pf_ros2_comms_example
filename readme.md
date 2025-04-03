# PF Camera Rig ROS2 Integration Example

This repository demonstrates how to integrate with the PF Camera Rig attachment for Grover robots using ROS2. It provides a complete example of how to receive camera metadata from a ROS1-based Grover robot via MQTT and process it in a ROS2 environment.

## Overview

The PF Camera Rig is an attachment for Grover robots that captures images at specified intervals based on distance traveled. This example focuses on the receiving end of the system, showing how to:

1. Receive camera metadata from ROS1 via MQTT
2. Convert and process the metadata in ROS2
3. Store the metadata for further analysis

## Prerequisites

- ROS2 Humble or later
- Python 3.8+
- Paho MQTT client (`pip install paho-mqtt`)
- rclpy_message_converter package

## Repository Structure

```
├── docker-compose.yaml       # Docker configuration for ROS2 environment
├── ros2_ws/                  # ROS2 workspace
│   └── src/                  # Source packages
│       └── grover_bridge/    # MQTT bridge package
│           ├── config/       # Configuration files for MQTT connection
│           │   └── grover_metadata.yaml  # MQTT and output configuration
│           ├── launch/       # Launch files for the bridge
│           │   └── grover_mqtt_bridge.launch.xml  # Main launch file
│           ├── msg/          # Custom message definitions
│           │   └── PFCamRigMetadata.msg  # Message definition for camera metadata
│           └── scripts/      # Python scripts for metadata processing
│               └── grover_metadata.py  # MQTT to ROS2 bridge script
```

## Quick Start

1. Start the Docker container:
   ```bash
   docker compose up -d
   ```

2. Open a shell in the container:
   ```bash
   docker exec -it pf_ros2_comms_example-ros2_base-1 bash
   ```

4. Build the ROS2 workspace:
   ```bash
   # Update package lists
   apt update
   
   # Source ROS2 environment
   source /opt/ros/humble/setup.bash
   
   # Install dependencies
   cd /ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   
   # Build the workspace
   cd /ros2_ws
   colcon build
   
   # Source the built workspace
   source install/setup.bash
   ```

4. Launch the MQTT bridge:
   ```bash
   ros2 launch grover_bridge grover_mqtt_bridge.launch.xml
   ```

## Configuration

The MQTT bridge is configured in `ros2_ws/src/grover_bridge/config/grover_metadata.yaml`:

- **MQTT broker IP**: Default is `192.168.1.100` (IP address of the Grover robot)
- **MQTT port**: Default is `1883` (standard MQTT port)
- **MQTT topics**: Default is `grover/cam_rig/metadata` (topic where the Grover robot publishes metadata)
- **Output directory**: Default is `/tmp` (where JSON logs are stored)

You can modify these settings to match your environment by editing the YAML file or passing parameters at launch time:

```bash
ros2 launch grover_bridge grover_mqtt_bridge.launch.xml params_file:=/path/to/custom_config.yaml
```

## Message Format

The `PFCamRigMetadata` message contains:

- **Standard header**: Timestamp of when the image was captured
- **GPS coordinates**: Latitude, longitude, and altitude from the robot's GPS
- **Heading**: Direction the robot was facing when the image was captured
- **Odometry data**: Position and velocity information from the robot's odometry
- **Run number**: Identifies a specific capture session
- **Sequence number**: Identifies the image within a capture session

This information can be used to georeference the images and track the robot's path.

## How It Works

### Data Flow

1. **Grover Robot (ROS1)**:
   - The PF Camera Rig attachment on the Grover robot captures images at specified intervals
   - For each capture, it publishes metadata to MQTT topic `grover/cam_rig/metadata`

2. **MQTT Bridge (ROS2)**:
   - Subscribes to the MQTT topic
   - Receives JSON-formatted metadata
   - Converts the metadata from ROS1 format to ROS2 format
   - Publishes the converted messages to ROS2 topic `grover/cam_rig/metadata`
   - Logs the raw JSON messages to a file for later analysis

3. **Your ROS2 Application**:
   - Subscribe to the ROS2 topic `grover/cam_rig/metadata`
   - Process the metadata as needed for your application

### Key Components

- **grover_metadata.py**: The main script that bridges MQTT and ROS2
- **PFCamRigMetadata.msg**: Defines the message structure for the metadata
- **grover_metadata.yaml**: Configuration file for MQTT connection and output settings
- **grover_mqtt_bridge.launch.xml**: Launch file to start the bridge

## Troubleshooting

- **MQTT Connection Issues**:
  - Ensure the MQTT broker is running on the Grover robot
  - Verify that the IP address in the configuration matches your Grover robot
  - Check network connectivity between your system and the Grover robot
  - Use `mosquitto_sub -h 192.168.1.100 -t "grover/cam_rig/metadata"` to test MQTT directly

- **Message Conversion Issues**:
  - Check the logs for conversion errors
  - The bridge includes fallback conversion methods for handling format differences
  - Ensure the message definition matches what the Grover robot is sending

- **ROS2 Issues**:
  - Verify that the ROS2 workspace is properly built and sourced
  - Check ROS2 topics with `ros2 topic list` and `ros2 topic echo /grover/cam_rig/metadata`
  - Look for errors in the node output with `ros2 node info /grover_metadata`

## Extending the Example

To use this data in your own application:

1. Create a new ROS2 node that subscribes to the `grover/cam_rig/metadata` topic
2. Process the metadata as needed for your application
3. Optionally, correlate the metadata with the actual images (which would need to be transferred separately)

Example subscriber code:

```python
import rclpy
from rclpy.node import Node
from grover_bridge.msg import PFCamRigMetadata

class MetadataProcessor(Node):
    def __init__(self):
        super().__init__('metadata_processor')
        self.subscription = self.create_subscription(
            PFCamRigMetadata,
            'grover/cam_rig/metadata',
            self.metadata_callback,
            10
        )
        
    def metadata_callback(self, msg):
        # Process the metadata
        self.get_logger().info(f'Received metadata: run={msg.run_number}, seq={msg.sequence_number}')
        # Access GPS data
        lat = msg.gps.latitude
        lon = msg.gps.longitude
        # Access odometry data
        pos_x = msg.odometry.pose.pose.position.x
        pos_y = msg.odometry.pose.pose.position.y
        # Do something with the data...

def main():
    rclpy.init()
    node = MetadataProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## License

Copyright (C) 2025, Peek Robotics
