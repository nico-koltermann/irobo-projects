#!/bin/bash

ROSBAG_NAME="vel_2022-10-19-14-48-37"
YAML_PATH="./yaml"

array=( "cmd_vel" )

rosbag info -y "bags/$ROSBAG_NAME.bag" >> "$YAML_PATH/$ROSBAG_NAME-info.yaml"

for i in "${array[@]}"
do
    python3 ros_readbagfile.py  ./bags/$ROSBAG_NAME.bag "/$i" | tee $YAML_PATH/$ROSBAG_NAME-$i.yaml
done

