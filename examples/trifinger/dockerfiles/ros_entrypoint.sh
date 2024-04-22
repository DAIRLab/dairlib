#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/humble/setup.bash"

# build ros workspace
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DOS_VERSION=preempt-rt
source "install/local_setup.bash"

exec "$@"
