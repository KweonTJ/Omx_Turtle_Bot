#!/bin/bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/omx_turtle_ws}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

exec ros2 launch turtlebot3_control omx_turtle.launch.py "$@"
