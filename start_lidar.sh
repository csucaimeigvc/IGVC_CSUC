#!/bin/bash

# Source ROS 2
source /opt/ros/humble/setup.bash

# Set lidar permissions
sudo chmod 666 /dev/ttyUSB0

# Set ROS domain ID for networking (must match laptop)
export ROS_DOMAIN_ID=0

# Launch the RPLidar node
ros2 launch rplidar_ros rplidar_a1_launch.py
