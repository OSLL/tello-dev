#!/bin/bash
source /opt/ros/foxy/setup.bash
source /builds/install/setup.bash

source ./install/setup.bash

ros2 run tello_ros manager_node --ros-args --remap srv_tello_action:=/tello/tello_action --remap action_response:=/tello/tello_response
