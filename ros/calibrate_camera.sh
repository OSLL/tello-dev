#!/bin/bash
xhost +local:docker
docker-compose -f docker-compose-camera-calibrate.yml up tello-ros-camera-calibration
xhost -local:docker
