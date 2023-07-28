#!/bin/bash

cd /home/remote/aztec_ws/
url="http://192.168.0.196/skeleton"

# Usando curl
curl $url


gnome-terminal -- bash -c "python src/4kcamera/yolov8-pose/yolov8-pose.py; exec bash"
#gnome-terminal -- bash -c "ros2 launch synexens_package driver_launch.py ; exec bash"
