#!/bin/bash
mkdir -p /builds/src
cd /builds/src
git clone https://github.com/clydemcqueen/tello_ros.git
git clone https://github.com/ptrmu/ros2_shared.git
cd ..
source /opt/ros/foxy/setup.bash
# If you didn't intall Gazebo, skip tello_gazebo while building:
colcon build --event-handlers console_direct+ --packages-skip tello_gazebo
