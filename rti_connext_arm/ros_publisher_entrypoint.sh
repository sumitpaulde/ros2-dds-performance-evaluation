#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/rti_workspace/y/rti_connext_dds-6.1.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash
source /root/rti_workspace/rmw/src/ros2/install/setup.bash
# source ~/app/install/setup.bash

export RTI_LICENSE_FILE="/root/rti_workspace/y/rti_connext_dds-6.1.1/rti_license.dat"
source /root/rti_workspace/rmw/src/ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_connextdds
export ROS_DOMAIN_ID=5


exec "$@"
