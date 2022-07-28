#!/bin/bash
source /opt/ros/foxy/setup.bash
source /builds/install/setup.bash

source ./install/setup.bash

ros2 run tello_driver tello_driver_main & ros2 run camera_calibration cameracalibrator --size 5x6 --square 0.03 image:=/image_raw
