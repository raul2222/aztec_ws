#!/bin/bash

cd /home/raul/ws_AZTEC/

# Ejecutar el primer archivo de launch en una nueva ventana de terminal
gnome-terminal -- bash -c "ros2 launch aztec_robot launch_robot.launch.py; exec bash"

# Esperar un poco para que el primer nodo comience a funcionar
sleep 6

# Ejecutar el segundo archivo de launch en una nueva ventana de terminal
gnome-terminal -- bash -c "ros2 launch oradar_lidar ms200_scan.launch.py; exec bash"

sleep 6

gnome-terminal -- bash -c "ros2 launch aztec_robot localization_launch.py map:=/home/raul/ws_AZTEC/save.yaml use_sim_time:=false; exec bash"

sleep 5

gnome-terminal -- bash -c "rviz2; exec bash"

sleep 15

gnome-terminal -- bash -c "ros2 launch aztec_robot navigation_launch.py use_sim_time:=false map_subscribe_trasient_local:=true; exec bash"

sleep 6

gnome-terminal -- bash -c "ros2 run aztec_robot minodo; exec bash"
