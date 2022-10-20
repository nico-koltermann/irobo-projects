#!/bin/bash

CATKIN_SHELL=bash

ROBOT_NAME=1
ROBOT_NUMBER=11

export LAB_ROOT=$( cd "$(dirname "${BASH_SOURCE[0]}" )" && pwd)

printf "Sourcing Lab Workspace"
printf "\n"

# check whether devel folder exists
if [ -f "${LAB_ROOT}/devel/setup.bash" ]; then
    # source setup.sh from same directory as this file
    source "${LAB_ROOT}/devel/setup.bash"
else
    source "/opt/ros/melodic/setup.bash"
    printf "You need to build first before you can source\n"
    printf "Run 'catkin build' in the skeleton_repo directory\n"
fi

[[ -f "${LAB_ROOT}/ipconfig" ]] && source "${LAB_ROOT}/ipconfig" || true


if [[ $1 == "robot" ]]; then
    # IP Config
    export ROS_IP=192.168.28.171
    export ROS_MASTER_URI=http://192.168.28.${ROBOT_NUMBER}:11311
    # TURTLE Info
    export TURTLEBOT3_MODEL=waffle_pi
    export TURTLEBOT3_NUMBER=${ROBOT_NUMBER}
    export TURTLEBOT3_NAME=waffle${ROBOT_NAME}
    export TURTLEBOT3_IP=192.168.28.${ROBOT_NUMBER}
    echo "Robot setup ready"
else 
    # IP Config
    unset ROS_IP
    export ROS_MASTER_URI=http://localhost:11311
fi


