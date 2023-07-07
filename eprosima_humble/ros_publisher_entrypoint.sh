#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
# source ~/app/install/setup.bash

# export LD_LIBRARY_PATH=/usr/local/lib/
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=5


exec "$@"
