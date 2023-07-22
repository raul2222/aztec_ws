#!/bin/bash

cd /home/remote/aztec_ws/

# Ejecutar el primer archivo de launch en una nueva ventana de terminal
gnome-terminal -- bash -c "ros2 launch aztec_robot launch_robot.launch.py; exec bash"

# Esperar un poco para que el primer nodo comience a funcionar
sleep 3

# Ejecutar el segundo archivo de launch en una nueva ventana de terminal
gnome-terminal -- bash -c "ros2 launch oradar_lidar ms200_scan.launch.py; exec bash"
gnome-terminal -- bash -c "python src/4kcamera/yolov8-pose/main.py; exec bash"
gnome-terminal -- bash -c "ros2 launch synexens_package driver_launch.py ; exec bash"

sleep 3

gnome-terminal -- bash -c "ros2 launch aztec_robot localization_launch.py map:=/home/remote/aztec_ws/save4.yaml use_sim_time:=false; exec bash"

sleep 3

gnome-terminal -- bash -c "rviz2; exec bash"

sleep 16


gnome-terminal -- bash -c "ros2 launch aztec_robot navigation_launch.py use_sim_time:=false map_subscribe_trasient_local:=true; exec bash"

sleep 1

gnome-terminal -- bash -c "ros2 run aztec_robot minodo; exec bash"
