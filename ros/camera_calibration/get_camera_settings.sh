#!/bin/bash
cf=../camera_settings
mkdir -p $cf
docker cp tello-ros-camera-calibration:/tmp/calibrationdata.tar.gz $cf/../calibrationdata.tar.gz
cd $cf
tar -xzf ../calibrationdata.tar.gz ost.yaml
mv ost.yaml camera.yaml
