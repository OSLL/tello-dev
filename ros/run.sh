#!/bin/bash
xhost +local:docker
docker-compose up tello-ros
xhost -local:docker
