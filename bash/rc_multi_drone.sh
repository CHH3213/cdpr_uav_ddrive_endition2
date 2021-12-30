#!/bin/bash
gnome-terminal --tab --title="SITL-gazebo" -- bash -c "roslaunch cdpr_uav_ddrive multi_drone.launch;exec bash;"
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0 --out=tcpin:0.0.0.0:8000 "
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1 --out=tcpin:0.0.0.0:8100 "
gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2 --out=tcpin:0.0.0.0:8200 "

#为每个无人机实例创建MAVROS节点，采用UDP通信
gnome-terminal --tab --title="mavros1" -x bash -c "roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14551@14555 mavros_ns:=/drone1 tgt_system:=1;exec bash;"
gnome-terminal --tab --title="mavros2" -x bash -c "roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2 tgt_system:=2;exec bash;"
gnome-terminal --tab --title="mavros3" -x bash -c "roslaunch iq_sim apm.launch fcu_url:=udp://127.0.0.1:14571@14575 mavros_ns:=/drone3 tgt_system:=3;exec bash;"
#gnome-terminal --tab --title="multiMavros" -- bash -c "roslaunch cdpr_uav_ddrive multi-apm.launch;exec bash;"

#地面站编号与无人机编号对应，无线电控制才可以生效
gnome-terminal --tab -x bash -c "
sleep 10;
rosparam set /drone1/mavros/system_id 1;
sleep 0.5;
rosparam set /drone2/mavros/system_id 2;
sleep 0.5;
rosparam set /drone3/mavros/system_id 3;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set SYSID_MYGCS 1;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set SYSID_MYGCS 2;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set SYSID_MYGCS 3;
sleep 0.5;
"
#将无线电各通道重映射，1～4通道为四个电机PWM，5～8通道为无人机姿态与油门
gnome-terminal --tab -x bash -c "
sleep 15;
rosrun mavros mavparam -n /drone1/mavros set SERVO1_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set SERVO2_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set SERVO3_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set SERVO4_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set RCMAP_ROLL 5;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set RCMAP_PITCH 6;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set RCMAP_THROTTLE 7;
sleep 0.5;
rosrun mavros mavparam -n /drone1/mavros set RCMAP_YAW 8;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set SERVO1_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set SERVO2_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set SERVO3_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set SERVO4_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set RCMAP_ROLL 5;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set RCMAP_PITCH 6;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set RCMAP_THROTTLE 7;
sleep 0.5;
rosrun mavros mavparam -n /drone2/mavros set RCMAP_YAW 8;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set SERVO1_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set SERVO2_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set SERVO3_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set SERVO4_FUNCTION 1;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set RCMAP_ROLL 5;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set RCMAP_PITCH 6;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set RCMAP_THROTTLE 7;
sleep 0.5;
rosrun mavros mavparam -n /drone3/mavros set RCMAP_YAW 8;
"