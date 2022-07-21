#!/bin/bash
source /opt/ros/foxy/setup.bash

ros2 topic pub --once /control std_msgs/msg/Bool "{data: true}"
