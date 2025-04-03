#!/bin/bash
# PF Camera Rig ROS2 Integration Setup Script
#
# This script performs the initial setup for the PF Camera Rig ROS2 integration.
# It updates package lists, installs dependencies, and builds the ROS2 workspace.
#
# Author: Peek Robotics
# Copyright (C) 2025, Peek Robotics

set -e  # Exit on error

echo "Starting PF Camera Rig ROS2 Integration Setup..."

# Update package lists
echo "Updating package lists..."
apt update

# Source ROS2 environment
echo "Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing dependencies..."
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
echo "Building ROS2 workspace..."
colcon build

# Source the built workspace
echo "Sourcing the built workspace..."
source install/setup.bash

echo "Setup complete! You can now run the MQTT bridge with:"
echo "ros2 launch grover_bridge grover_mqtt_bridge.launch.xml"
