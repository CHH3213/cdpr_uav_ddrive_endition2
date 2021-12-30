#!/bin/bash
# 运行后会打开仿真，加载mavros
gnome-terminal --tab --title="killall gzclient" -- bash -c "killall gzclient "
gnome-terminal --tab --title="killall gzserver" -- bash -c "killall gzserver "
gnome-terminal --tab --title="SITL-gazebo" -- bash -c "roslaunch cdpr_uav_ddrive multi_drone.launch;exec bash;"
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0 --out=tcpin:0.0.0.0:8000 "
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1 --out=tcpin:0.0.0.0:8100 "
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2 --out=tcpin:0.0.0.0:8200 "
# gnome-terminal --tab --title="multiMavros" -- bash -c "roslaunch cdpr_uav_ddrive multi-apm.launch;exec bash;"
