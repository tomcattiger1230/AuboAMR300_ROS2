#!/bin/bash
#
# ROS 2 Lyrical (Lyrical uses Python 3.14; the local .venv must match it)

SETUP_FILE=install/setup.zsh
if [ -f "$SETUP_FILE" ]; then
    source $SETUP_FILE
else
    source /opt/ros/lyrical/setup.zsh
    colcon build
    source $SETUP_FILE
fi

source .venv/bin/activate
