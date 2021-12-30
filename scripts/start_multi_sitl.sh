#!/bin/bash
gnome-terminal --tab --title="SITL-gazebo-node" -x bash -c "roslaunch iq_sim multi_drone.launch;exec bash;"
gnome-terminal --tab --title="MAVProxy1" -x bash -c "sleep 3s;cd ~/intelligent_quads/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-drone1 --console -I0 --out=tcpin:0.0.0.0:8100;exec bash;"
gnome-terminal --tab --title="MAVProxy2" -x bash -c "sleep 3s;cd ~/intelligent_quads/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-drone2 --console -I1 --out=tcpin:0.0.0.0:8200;exec bash;"
gnome-terminal --tab --title="MAVProxy3" -x bash -c "sleep 3s;cd ~/intelligent_quads/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-drone3 --console -I2 --out=tcpin:0.0.0.0:8300;exec bash;"
