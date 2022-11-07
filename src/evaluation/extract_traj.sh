#!/bin/bash

YAML_PATH="./yaml"

TOPIC="amcl_pose"

LAST_BAG=$(find bags/ -printf '%T+ %p\n' | sort -r | head -n1 | cut -d ' ' -f2 | cut -b 6-)

if [[ $1 == "-h" ]]; then
    echo "For the last bag, execute with argument -l"
    echo "For the specific bag, execute with argument -f <bagname>"
fi

if [[ $1 == "-l" ]]; then
    echo "Select Last Bag"
    ROSBAG_NAME=$LAST_BAG
fi

if [[ $1 == "-f" ]]; then
    echo "Select File"
    if [ -f "bags/$2" ]; then
        ROSBAG_NAME=$2
    else 
        echo "FILE NOT FOUND"
    fi
fi

if [ -f "bags/$ROSBAG_NAME" ]; then
    echo "FILE Ready"
else 
    echo "FILE NOT FOUND"
    exit
fi

FILENAME="${ROSBAG_NAME%.*}"

rosbag info -y "bags/$ROSBAG_NAME" >> "$YAML_PATH/$filename-info.yaml"

python3 ros_readbagfile.py  ./bags/$ROSBAG_NAME "/$TOPIC" | tee $YAML_PATH/$FILENAME-$TOPIC.yaml
# python3 scripts/plot_cmd.py -f $FILENAME-$TOPIC.yaml
# python3 scripts/plot_odom.py -f $FILENAME-$TOPIC.yaml


