#!/bin/bash
./stop_solutions.sh
docker-compose down
xhost -local:docker
