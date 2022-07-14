#!/bin/bash
source ~/.bashrc
source /opt/ros/noetic/setup.bash
ls /usr/bin/python3*
python -V
roslaunch ./test.launch.xml
# roscore & rosrun turtlesim turtlesim_node
# roslaunch tello_driver tello_node.launch
# roslaunch tello_driver joy_teleop.launch
