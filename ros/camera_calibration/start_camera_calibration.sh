#!/bin/bash
xhost +local:docker
docker-compose up tello-ros-camera-calibration
xhost -local:docker
