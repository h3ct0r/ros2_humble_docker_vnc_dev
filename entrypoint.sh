#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/home/ros/ros2_ws/install/setup.bash" ]; then
    source /home/ros/ros2_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"
