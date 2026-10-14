#!/usr/bin/env bash
# Preview only: no hardware, manipulator controller, camera or policy nodes.
set -e
workspace="${OMX_ROBOT_WS:-/home/ktj/omx_turtle_ws}"
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=233
exec ros2 launch turtlebot3_manipulation_description leader_raised_shelf.launch.py "$@"
