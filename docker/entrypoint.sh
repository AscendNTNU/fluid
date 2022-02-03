#!/bin/bash
set -e

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f /ws/install/setup.bash ]; then source "install/setup.bash"; fi
# run commands
exec "$@"