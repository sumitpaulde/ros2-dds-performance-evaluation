#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
#source /app/ros2_ws/install/setup.bash
source /app/rti_workspace/y/rti_connext_dds-6.1.1/resource/scripts/rtisetenv_x64Linux4gcc7.3.0.bash

# source ~/app/install/setup.bash

export RTI_LICENSE_FILE="/app/rti_workspace/y/rti_connext_dds-6.1.1/rti_license.dat"
source /app/rti_workspace/rmw/src/ros2/install/setup.bash

exec "$@"
