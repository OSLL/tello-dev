#!/bin/bash
source /opt/ros/foxy/setup.bash
source /builds/install/setup.bash

source ./install/setup.bash

ros2 launch ./marker_follower.launch
