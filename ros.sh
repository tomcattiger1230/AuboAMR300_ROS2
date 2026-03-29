#!/bin/bash
#

SETUP_FILE=install/setup.zsh
if [ -f "$SETUP_FILE" ]; then
    source $SETUP_FILE
else
    source /opt/ros/jazzy/setup.zsh
    colcon build 
    source $SETUP_FILE
fi

source .venv/bin/activate
