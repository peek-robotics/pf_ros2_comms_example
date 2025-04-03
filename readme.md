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
├── example.geojson           # Example GeoJSON file for automatic capture triggering
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
- **Run number**: Identifies a specific capture session (from trigger-on to trigger-off within a boot)
- **Sequence number**: Identifies the image within a capture run (sequential index within that run)

This information can be used to georeference the images and track the robot's path.

## Automatic Capture Triggering with GeoJSON

The repository includes an `example.geojson` file that demonstrates how to configure automatic capture triggering when the Grover robot enters specific geographic areas.

### How GeoJSON Triggering Works

1. The Grover robot can load GeoJSON files containing polygons with special metadata.
2. When the robot enters a polygon with camera rig trigger metadata, it automatically activates the camera capture system.
3. When the robot exits the polygon, it deactivates the capture system.

### Trigger Distance Calculation

The camera rig captures images based on distance traveled:

- **During autonomous navigation**: The trigger distance is calculated as the linear distance to the next target waypoint in the navigation path.
- **Without autonomous navigation**: The trigger distance is calculated as the linear distance traveled forward

### Run and Sequence Numbering

In the metadata messages:
- **Run number**: Represents a complete capture session from when the trigger is activated to when it's deactivated within a single boot of the robot. Each time the robot enters and exits a polygon (or manually triggers on/off), the run number increments.
- **Sequence number**: Represents the sequential index of an image within a specific run. This resets to 0 at the start of each new run.

This numbering system makes it easy to organize and process images from specific capture sessions.

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

## License

Copyright (C) 2025, Peek Robotics
