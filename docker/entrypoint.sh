#!/usr/bin/env bash
# Source ROS and the workspace overlay, then exec the requested command.
set -e

export DISABLE_ROS1_EOL_WARNINGS=1

source /opt/ros/noetic/setup.bash
source /root/hybrid_a_star_ws/devel/setup.bash

exec "$@"
