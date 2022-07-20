#!/bin/bash
source /opt/ros/foxy/setup.bash
source ./install/setup.bash
source /builds/install/setup.bash

ros2 launch ./test.launch.xml
