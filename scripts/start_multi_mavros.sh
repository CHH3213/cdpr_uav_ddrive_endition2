#!/bin/bash
gnome-terminal --tab --title="mavros1" -x bash -c "roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14551@14555 mavros_ns:=/drone1 tgt_system:=1;exec bash;"
gnome-terminal --tab --title="mavros2" -x bash -c "roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2 tgt_system:=2;exec bash;"
gnome-terminal --tab --title="mavros3" -x bash -c "roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14571@14575 mavros_ns:=/drone3 tgt_system:=3;exec bash;"
gnome-terminal --tab -x bash -c "rosparam set /drone1/mavros/system_id 1;rosparam set /drone2/mavros/system_id 2;rosparam set /drone3/mavros/system_id 3;exec bash;"
