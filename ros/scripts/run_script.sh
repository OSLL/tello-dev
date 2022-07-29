#!/bin/bash
source /opt/ros/foxy/setup.bash
source /builds/install/setup.bash

source ./install/setup.bash

if [[ $# == 0 ]]
then
    echo "No script name is provided. Using 'manager_node' by default"
    script_name="manager_node"
else
    script_name=$1
fi

if [[ $script_name == "manager_node" ]]
then
    cmd="ros2 run tello_ros manager_node --ros-args --remap srv_tello_action:=/tello/tello_action --remap action_response:=/tello/tello_response"
else
    echo -e "\e[0;31mUnknown script '$script_name'. Abort\e[0m"
    exit 1
fi

$cmd
