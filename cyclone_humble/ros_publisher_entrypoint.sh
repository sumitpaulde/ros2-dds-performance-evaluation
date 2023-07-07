#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash


export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5
#/app/ros2_ws/zenoh-plugin-dds/target/release/zenoh-bridge-dds -d 5 -e tcp/10.5.0.6:7447

exec "$@"
