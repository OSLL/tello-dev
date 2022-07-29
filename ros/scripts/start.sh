#!/bin/bash
source /opt/ros/foxy/setup.bash
source /builds/install/setup.bash

source ./install/setup.bash

if [[ -z $SOLUTION_NAME ]]
then
    echo "No solution name is provided, using 'marker_follower.launch' by default"
    SOLUTION_NAME="marker_follower"
else
    if [[ ! -f "$SOLUTION_NAME.launch" ]]
    then
        echo -e "\e[0;31mNo file $SOLUTION_NAME.launch. Abort\e[0m"
        exit 1
    fi
fi
LAUNCH_NAME="$SOLUTION_NAME.launch"
echo -e "\e[0;32mRunning $LAUNCH_NAME\e[0m"

ros2 launch ./$LAUNCH_NAME
