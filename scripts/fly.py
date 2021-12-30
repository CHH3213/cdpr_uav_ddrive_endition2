# -*-coding:utf-8-*-
#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import print_function, absolute_import, division
import gym
import numpy
import time
import math
import datetime
# import qlearn
# ROS packages required
import rospy
import rospkg
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Twist
import numpy as np
import os
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetModeRequest
from mavros_msgs.srv import SetModeResponse
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBoolRequest
from mavros_msgs.srv import CommandBoolResponse
from mavros_msgs.srv import StreamRate, StreamRateRequest


from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState

from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest


import mavros.setpoint
from pidController import PID_posi
from pidController import PID_inc

import gym
from gym import spaces

from dronekit import connect, VehicleMode, LocationGlobalRelative
import scipy.io as sio
import math

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, GetModelState  # 设置模型状态、得到模型状态
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Pose, Twist
from PIDClass import PID

class CDPR(gym.Env):
    def __init__(self):
        ###  define ROS messages  ######
        self.current_state = State()
        self.imu_data = Imu()
        self.act_controls = ActuatorControl()
        self.pose = PoseStamped()
        self.mocap_pose = PoseStamped()
        self.local_velocity1 = TwistStamped()
        self.local_velocity2 = TwistStamped()
        self.local_velocity3 = TwistStamped()
        self.global_velocity = TwistStamped()
        self.battery = BatteryState()
        self.local_position1 = PoseStamped()
        self.local_position2 = PoseStamped()
        self.local_position3 = PoseStamped()
        self.model_states = ModelStates()
        self.rc_states1 = RCOut()
        self.rc_states2 = RCOut()
        self.rc_states3 = RCOut()
        self.rcIn_states1 = RCIn()
        self.rcIn_states2 = RCIn()
        self.rcIn_states3 = RCIn()



        # # 物理属性
        self.cable_length = 5
        self.payload_gravity = 2.5
        self.drone_gravity = 18.88
        self.Coeff_elasticity = 500
        self.state_logger0 = []
        self.state_logger1 = []
        self.state_logger2 = []
        self.state_logger3 = []
        self.state_drone1 = []
        self.state_drone2 = []
        self.state_drone3 = []
        self.state_payload = []
        self.dt = 0.015  ## 仿真频率



        self.saveData_dir = "/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/stateData"
        if not os.path.exists(self.saveData_dir):
            os.makedirs(self.saveData_dir)

        ## ROS Subscribers
        # self.state_sub1 = rospy.Subscriber("/drone1/mavros/state",State, self.state_cb,queue_size=1)
        # self.state_sub2 = rospy.Subscriber("/drone2/mavros/state",State, self.state_cb,queue_size=1)
        # self.state_sub3 = rospy.Subscriber("/drone3/mavros/state",State, self.state_cb,queue_size=1)
        # self.imu_sub1 = rospy.Subscriber("/drone1/mavros/imu/data",Imu, self.imu_cb, queue_size=1)
        # self.imu_sub2 = rospy.Subscriber("/drone2/mavros/imu/data",Imu, self.imu_cb, queue_size=1)
        # self.imu_sub3 = rospy.Subscriber("/drone3/mavros/imu/data",Imu, self.imu_cb, queue_size=1)
        self.local_pos_sub1 = rospy.Subscriber("/drone1/mavros/local_position/pose", PoseStamped, self.lp_cb1, queue_size=1)
        self.local_pos_sub2 = rospy.Subscriber("/drone2/mavros/local_position/pose", PoseStamped, self.lp_cb2, queue_size=1)
        self.local_pos_sub3 = rospy.Subscriber("/drone3/mavros/local_position/pose", PoseStamped, self.lp_cb3, queue_size=1)
        # self.local_vel_sub1 = rospy.Subscriber("/drone1/mavros/local_position/velocity_local", TwistStamped, self.lv_cb1, queue_size=1)
        # self.local_vel_sub2 = rospy.Subscriber("/drone2/mavros/local_position/velocity_local", TwistStamped, self.lv_cb2, queue_size=1)
        # self.local_vel_sub3 = rospy.Subscriber("/drone3/mavros/local_position/velocity_local", TwistStamped, self.lv_cb3, queue_size=1)

        # self.act_control_sub1 = rospy.Subscriber("/drone1/mavros/act_control/act_control_pub", ActuatorControl, self.act_cb,queue_size=1)
        # self.act_control_sub2 = rospy.Subscriber("/drone2/mavros/act_control/act_control_pub", ActuatorControl, self.act_cb,queue_size=1)
        # self.act_control_sub3 = rospy.Subscriber("/drone3/mavros/act_control/act_control_pub", ActuatorControl, self.act_cb,queue_size=1)

        # self.global_alt_sub1 = rospy.Subscriber("/drone1/mavros/global_position/rel_alt", Float64, self.ra_cb, queue_size=1)
        # self.global_alt_sub2 = rospy.Subscriber("/drone2/mavros/global_position/rel_alt", Float64, self.ra_cb, queue_size=1)
        # self.global_alt_sub3 = rospy.Subscriber("/drone3/mavros/global_position/rel_alt", Float64, self.ra_cb, queue_size=1)
        # self.global_pos_sub1 = rospy.Subscriber("/drone1/mavros/global_position/gp_vel", TwistStamped, self.gv_cb, queue_size=1)
        # self.global_pos_sub2 = rospy.Subscriber("/drone2/mavros/global_position/gp_vel", TwistStamped, self.gv_cb, queue_size=1)
        # self.global_pos_sub3 = rospy.Subscriber("/drone3/mavros/global_position/gp_vel", TwistStamped, self.gv_cb, queue_size=1)

        # self.battery_sub1 = rospy.Subscriber("/drone1/mavros/battery",BatteryState, self.bat_cb, queue_size=1)
        # self.battery_sub2 = rospy.Subscriber("/drone2/mavros/battery",BatteryState, self.bat_cb, queue_size=1)
        # self.battery_sub3 = rospy.Subscriber("/drone3/mavros/battery",BatteryState, self.bat_cb, queue_size=1)
        self.rc_out1 = rospy.Subscriber("/drone1/mavros/rc/out", RCOut, self.rc_cb1, queue_size=1)
        self.rc_out2 = rospy.Subscriber("/drone2/mavros/rc/out", RCOut, self.rc_cb2, queue_size=1)
        self.rc_out3 = rospy.Subscriber("/drone3/mavros/rc/out", RCOut, self.rc_cb3, queue_size=1)
        self.rc_in1 = rospy.Subscriber("/drone1/mavros/rc/in", RCIn, self.rcIn_cb1, queue_size=1)
        self.rc_in2 = rospy.Subscriber("/drone2/mavros/rc/in", RCIn, self.rcIn_cb2, queue_size=1)
        self.rc_in3 = rospy.Subscriber("/drone3/mavros/rc/in", RCIn, self.rcIn_cb3, queue_size=1)


        ## ROS Publishers
        # self.mocap_pos_pub = rospy.Publisher("/drone1/mavros/mocap/pose",PoseStamped,queue_size=1)
        self.acutator_control_pub1 = rospy.Publisher("/drone1/mavros/actuator_control",ActuatorControl,queue_size=1)
        self.acutator_control_pub2 = rospy.Publisher("/drone2/mavros/actuator_control",ActuatorControl,queue_size=1)
        self.acutator_control_pub3 = rospy.Publisher("/drone3/mavros/actuator_control",ActuatorControl,queue_size=1)
        self.setpoint_raw_pub1 = rospy.Publisher("/drone1/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)
        self.setpoint_raw_pub2 = rospy.Publisher("/drone2/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)
        self.setpoint_raw_pub3 = rospy.Publisher("/drone3/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=1)  # NED
        self.setposition_raw_pub1 = rospy.Publisher("/drone1/mavros/setpoint_raw/local",PositionTarget,queue_size=1)  # NED
        self.setposition_raw_pub2 = rospy.Publisher("/drone2/mavros/setpoint_raw/local",PositionTarget,queue_size=1)  # NED
        self.setposition_raw_pub3 = rospy.Publisher("/drone3/mavros/setpoint_raw/local",PositionTarget,queue_size=1)  # NED
        # self.setglobalpos_raw_pub1 = rospy.Publisher("/drone1/mavros/setpoint_raw/global",GlobalPositionTarget,queue_size=1)  # NED
        # self.setglobalpos_raw_pub2 = rospy.Publisher("/drone2/mavros/setpoint_raw/global",GlobalPositionTarget,queue_size=1)  # NED
        # self.setglobalpos_raw_pub3 = rospy.Publisher("/drone3/mavros/setpoint_raw/global",GlobalPositionTarget,queue_size=1)  # NED

        self.local_pos_pub1 = rospy.Publisher("/drone1/mavros/setpoint_position/local",PoseStamped,queue_size=1) #ENU
        self.local_pos_pub2 = rospy.Publisher("/drone2/mavros/setpoint_position/local",PoseStamped,queue_size=1)
        self.local_pos_pub3 = rospy.Publisher("/drone3/mavros/setpoint_position/local",PoseStamped,queue_size=1)
        # self.setcmd_vel_pub1 = rospy.Publisher("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
        # self.setcmd_vel_pub2 = rospy.Publisher("/drone2/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
        # self.setcmd_vel_pub3 = rospy.Publisher("/drone3/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
        self.thrust_pub1 = rospy.Publisher("/drone1/mavros/setpoint_attitude/thrust",Thrust,queue_size=1)
        self.thrust_pub2 = rospy.Publisher("/drone2/mavros/setpoint_attitude/thrust",Thrust,queue_size=1)
        self.thrust_pub3 = rospy.Publisher("/drone3/mavros/setpoint_attitude/thrust",Thrust,queue_size=1)
        # self.setattitude_pub1 = rospy.Publisher("/drone1/mavros/setpoint_attitude/attitude", PoseStamped,queue_size=1)
        # self.setattitude_pub2 = rospy.Publisher("/drone2/mavros/setpoint_attitude/attitude", PoseStamped,queue_size=1)
        # self.setattitude_pub3 = rospy.Publisher("/drone3/mavros/setpoint_attitude/attitude", PoseStamped,queue_size=1)  # ENU

        self.rc_override1 = rospy.Publisher("/drone1/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.rc_override2 = rospy.Publisher("/drone2/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.rc_override3 = rospy.Publisher("/drone3/mavros/rc/override", OverrideRCIn, queue_size=1)

        self.force_publisher0 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher1 = rospy.Publisher('/logger1/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher2 = rospy.Publisher('/logger2/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher3 = rospy.Publisher('/logger3/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher4 = rospy.Publisher('/drone1/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher5 = rospy.Publisher('/drone2/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher6 = rospy.Publisher('/drone3/testforce', Float32MultiArray, queue_size=1)


        # gazebo topic
        self.modestate_sub=rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb)
        # gazebo服务
        self.reset_world_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)  # 指定服务名来调用服务
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_physics_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_physics_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)



        ## ROS mavros Services
        rospy.wait_for_service('drone1/mavros/cmd/arming')
        self.arming_client1 = rospy.ServiceProxy('drone1/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('drone2/mavros/cmd/arming')
        self.arming_client2 = rospy.ServiceProxy('drone2/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('drone3/mavros/cmd/arming')
        self.arming_client3 = rospy.ServiceProxy('drone3/mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('drone1/mavros/cmd/takeoff')
        self.takeoff_service1 = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)
        rospy.wait_for_service('drone2/mavros/cmd/takeoff')
        self.takeoff_service2 = rospy.ServiceProxy('/drone2/mavros/cmd/takeoff', CommandTOL)
        rospy.wait_for_service('drone3/mavros/cmd/takeoff')
        self.takeoff_service3 = rospy.ServiceProxy('/drone3/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service('drone1/mavros/set_mode')
        self.set_mode_client1 = rospy.ServiceProxy('drone1/mavros/set_mode', SetMode)
        rospy.wait_for_service('drone2/mavros/set_mode')
        self.set_mode_client2 = rospy.ServiceProxy('drone2/mavros/set_mode', SetMode)
        rospy.wait_for_service('drone3/mavros/set_mode')
        self.set_mode_client3 = rospy.ServiceProxy('drone3/mavros/set_mode', SetMode)

        # rospy.wait_for_service('drone1/mavros/set_stream_rate')
        # set_stream_rate1=rospy.ServiceProxy("drone1/mavros/set_stream_rate",StreamRate)
        # set_stream_rate1(StreamRateRequest.STREAM_POSITION, 50, 1)
        # set_stream_rate1(StreamRateRequest.STREAM_ALL, 50, 1)
        #
        # rospy.wait_for_service('drone2/mavros/set_stream_rate')
        # set_stream_rate2=rospy.ServiceProxy("drone2/mavros/set_stream_rate",StreamRate)
        # set_stream_rate2(StreamRateRequest.STREAM_POSITION, 50, 1)
        # set_stream_rate2(StreamRateRequest.STREAM_ALL, 50, 1)
        #
        # rospy.wait_for_service('drone3/mavros/set_stream_rate')
        # set_stream_rate3=rospy.ServiceProxy("drone1/mavros/set_stream_rate",StreamRate)
        # set_stream_rate3(StreamRateRequest.STREAM_POSITION, 50, 1)
        # set_stream_rate3(StreamRateRequest.STREAM_ALL, 50, 1)


        ### Initiate ROS node
        print('-- Connecting to mavros')
        rospy.init_node('gym_apm_mavros',anonymous=True)
        print ('connected')

        id_logger0 = self.model_states.name.index("logger0")  #
        id_logger1 = self.model_states.name.index("logger1")  #
        id_logger2 = self.model_states.name.index("logger2")  #
        id_logger3 = self.model_states.name.index("logger3")  #
        id_payload = self.model_states.name.index("payload")  #
        id_drone1 = self.model_states.name.index("drone1")  #
        id_drone2 = self.model_states.name.index("drone2")  #
        id_drone3 = self.model_states.name.index("drone3")  #
        logger_pose0 = self.model_states.pose[id_logger0]
        logger_pose1 = self.model_states.pose[id_logger1]
        logger_pose2 = self.model_states.pose[id_logger2]
        logger_pose3 = self.model_states.pose[id_logger3]
        payload_pose = self.model_states.pose[id_payload]
        drone_pose1 = self.model_states.pose[id_drone1]
        drone_pose2 = self.model_states.pose[id_drone2]
        drone_pose3 = self.model_states.pose[id_drone3]
        # =====================初始位置=====================
        self.payload_pose = payload_pose
        self.drone_pose1=drone_pose1
        self.drone_pose2=drone_pose2
        self.drone_pose3=drone_pose3
        self.logger_pose0=logger_pose0
        # ==============================================


    def resetWorld(self):
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_world_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("/gazebo/reset_world service call failed")
    def resetSim(self):
        rospy.wait_for_service("/gazebo/reset_simulation")
        try:
            self.reset_simulation()
        except rospy.ServiceException as e:
            rospy.logerr("/gazebo/reset_simulation service call failed")

    def pausePhysics(self):
        rospy.wait_for_service("/gazebo/pause_physics")  # 等待服务器连接
        try:
            self.pause_physics_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("/gazebo/pause_physics service call failed")

    def unpausePhysics(self):
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause_physics_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("/gazebo/unpause_physics service call failed")
    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        self.set_mode_client1(custom_mode="9")
        self.arming_client1(False)
        self.set_mode_client2(custom_mode="9")
        self.arming_client2(False)
        self.set_mode_client3(custom_mode="9")
        self.arming_client3(False)

    def offb_arm(self):

        # print ('-- Enabling offboard mode and arming')
        # while not self.arm_cmd.value:
        #     pass
        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.set_mode_client1(custom_mode="4")
        # self.set_mode_client1(custom_mode="ALT_HOLD")
        self.arming_client1(True)

        self.set_mode_client2(custom_mode="4")
        # self.set_mode_client1(custom_mode="ALT_HOLD")
        self.arming_client2(True)

        self.set_mode_client3(custom_mode="4")
        # self.set_mode_client1(custom_mode="ALT_HOLD")
        self.arming_client3(True)
    def run(self):
        # logger
        multiarray0 = Float32MultiArray()
        multiarray1 = Float32MultiArray()
        multiarray2 = Float32MultiArray()
        multiarray3 = Float32MultiArray()
        # UAV
        multiarray4 = Float32MultiArray()
        multiarray5 = Float32MultiArray()
        multiarray6 = Float32MultiArray()

        multiarray0.data = [1000]
        multiarray1.data = [1000]
        multiarray2.data = [1000]
        multiarray3.data = [1000]
        target = [0,0,1]
        pid = apmPID(target)
         # vehicle = connect('127.0.0.1:14551', wait_ready=True)
        self.start = time.time()
        while not rospy.is_shutdown():
            # print('channels1', self.rc_states1.channels)
            # print('channels2', self.rc_states2.channels)
            # print('channels3', self.rc_states3.channels)
            # print('channelsRCIn1', self.rcIn_states1.channels)
            # print('channelsRCIn2', self.rcIn_states2.channels)
            # print('channelsRCIn3', self.rcIn_states3.channels)
            self.unpausePhysics()
            self.offb_arm()
            # publish
            # Override1 = OverrideRCIn()
            # Override1.channels = [1800, 1800, 1800, 1800, 1500, 1600, 1500, 1500]
            # self.rc_override1.publish(Override1.channels)
            # drone1_fly为z周距离是否大于0
            dist_cone2drone1, drone1_fly, dist_cone2drone2, drone2_fly, dist_cone2drone3, drone3_fly = self.get_dist()
            # print('dist_cone2drone1',dist_cone2drone1)
            # print('drone1_fly',drone1_fly)
            # print('dist_cone2drone2',dist_cone2drone2)
            # print('drone2_fly',drone2_fly)
            # print('dist_cone2drone3',dist_cone2drone3)
            # print('drone3_fly',drone3_fly)
            # self.takeoff_service1(altitude=3)
            # self.takeoff_service2(altitude=7)
            # self.takeoff_service3(altitude=7)
            #
            attitude = AttitudeTarget()
            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            x,y,z,w = self.euler_to_quaternion(roll,pitch,yaw)

            attitude.orientation.x = x
            attitude.orientation.y = y
            attitude.orientation.z = z
            attitude.orientation.w = w
            # attitude.thrust = 0.5005
            attitude.thrust = 0.55
            # self.setpoint_raw_pub1.publish(attitude)
            # self.setpoint_raw_pub2.publish(attitude)
            # self.setpoint_raw_pub3.publish(attitude)

            velocity = Twist()
            # if drone1_fly == True:
            #     if self.local_velocity1.twist.linear.x >2.5:
            #         velocity.linear.x = 2.5
            #         self.setcmd_vel_pub1.publish(velocity)
            #     elif self.local_velocity1.twist.linear.x <-2.5:
            #         velocity.linear.x = -2.5
            #         self.setcmd_vel_pub1.publish(velocity)
            #     if self.local_velocity1.twist.linear.y>2.5:
            #         velocity.linear.y = 2.5
            #         self.setcmd_vel_pub1.publish(velocity)
            #     elif self.local_velocity1.twist.linear.y<-2.5:
            #         velocity.linear.y = -2.5
            #         self.setcmd_vel_pub1.publish(velocity)
            if drone1_fly == True:
                # multiarray4.data = [dist_cone2drone1, self.cable_length, self.Coeff_elasticity] # 距离，绳长，弹性系数
                multiarray4.data = [20] # 距离，绳长，弹性系数
                self.force_publisher4.publish(multiarray4)
            # if drone2_fly == True:
            #     multiarray5.data = [dist_cone2drone2, self.cable_length, self.Coeff_elasticity]
            #     self.force_publisher5.publish(multiarray5)
            # if drone3_fly == True:
            #     multiarray6.data = [dist_cone2drone3, self.cable_length, self.Coeff_elasticity]
            #     self.force_publisher6.publish(multiarray6)
            self.rate.sleep()
            self.pausePhysics()
            # logger
            model_name = ['logger0', 'logger1', 'logger2', 'logger3', 'drone1', 'drone2', 'drone3', 'payload']
            [payload_position, payload_orientation, payload_linear, payload_angular] = self.get_state(self.model_states,  model_name[7])
            [logger0_position, logger0_orientation, logger0_linear, logger0_angular] = self.get_state(self.model_states,  model_name[0])
            [logger1_position, logger1_orientation, logger1_linear, logger1_angular] = self.get_state(self.model_states,  model_name[1])
            [logger2_position, logger2_orientation, logger2_linear, logger2_angular] = self.get_state(self.model_states,  model_name[2])
            [logger3_position, logger3_orientation, logger3_linear, logger3_angular] = self.get_state(self.model_states,  model_name[3])
            [drone1_position, drone1_orientation, drone1_linear, drone1_angular] = self.get_state(self.model_states,  model_name[4])
            #############################################################################################################
            # payload_position = [self.payload_pose.position.x,self.payload_pose.position.y,self.payload_pose.position.z]
            # logger0_position = [self.logger_pose0.position.x,self.logger_pose0.position.y,self.logger_pose0.position.z]
            # drone1_position = [self.drone_pose1.position.x,self.drone_pose1.position.y,self.drone_pose1.position.z]
            ############################################################################################################

            force0_error, force1_error,force2_error = pid.cal_actions(payload_position)
            #
            # delta_load2logger0 = [payload_position[0]-logger0_position[0],payload_position[1]-logger0_position[1],payload_position[2]-logger0_position[2]]
            # delta_load2logger1 = [payload_position[0]-logger1_position[0],payload_position[1]-logger1_position[1],payload_position[2]-logger1_position[2]]
            # delta_load2logger2 = [payload_position[0]-logger2_position[0],payload_position[1]-logger2_position[1],payload_position[2]-logger2_position[2]]
            # delta_load2logger3 = [payload_position[0]-logger3_position[0],payload_position[1]-logger3_position[1],payload_position[2]-logger3_position[2]]
            # delta_load2drone1 = [payload_position[0]-drone1_position[0],payload_position[1]-drone1_position[1],payload_position[2]-drone1_position[2]]
            # theta_load2logger0 = payload_position[2]/np.sqrt(np.sum(np.square(delta_load2logger0)))
            # theta_load2drone1 = payload_position[2]/np.sqrt(np.sum(np.square(delta_load2drone1)))
            # dir_z_force_drone1 = self.Coeff_elasticity * (dist_cone2drone1 - self.cable_length) * np.cos(theta_load2drone1)
            # force2 = (dir_z_force_drone1 - force2_error) / np.cos(theta_load2logger0)
            # force2 = (50 - force2_error) / np.cos(theta_load2logger0)

            print('payload_position_z',payload_position[2]-3)
            # multiarray0.data = [np.sqrt(np.sum(np.square(delta_load2logger0))) if np.sqrt(np.sum(np.square(delta_load2logger0))) < 100 else 100]
            # multiarray1.data = [np.sqrt(np.sum(np.square(delta_load2logger1))) if np.sqrt(np.sum(np.square(delta_load2logger1))) < 100 else 100]
            # multiarray2.data = [np.sqrt(np.sum(np.square(delta_load2logger2))) if np.sqrt(np.sum(np.square(delta_load2logger2))) < 100 else 100]
            # multiarray3.data = [np.sqrt(np.sum(np.square(delta_load2logger3))) if np.sqrt(np.sum(np.square(delta_load2logger3))) < 100 else 100]
            self.unpausePhysics()
            if drone1_fly and payload_position[2]>0.1:
                multiarray0.data = [force2_error]
            #     multiarray1.data = [force2_error]
            #     multiarray2.data = [force2_error]
            #     multiarray3.data = [force2_error]
            #     print('force',force2_error)
                # self.force_publisher0.publish(multiarray0)
                # self.force_publisher1.publish(multiarray1)
                # self.force_publisher2.publish(multiarray2)
                # self.force_publisher3.publish(multiarray3)
            # self.force_publisher0.publish(multiarray0)
            # self.force_publisher1.publish(multiarray1)
            # self.force_publisher2.publish(multiarray2)
            # self.force_publisher3.publish(multiarray3)
            self.rate.sleep()
            # print(time.time()-start)
            self.pausePhysics()
            # if dist_cone2drone1 < 2 or dist_cone2drone2 < 2 or dist_cone2drone3 < 2 or dist_cone2drone1 >10 or dist_cone2drone2 >10 or dist_cone2drone3 >10:
            #     self.pausePhysics()
            #     # self.resetWorld()
            #     self.resetSim()
            #     time.sleep(1)
            self.start = time.time()

            self.save2list()
        # print(np.shape(self.state_drone1))
        # print(np.shape(self.state_drone1[:]))
        # print(np.shape(self.state_drone1[0]))
        # print(self.state_drone1[0])
        draw_z = [self.state_drone1[i][0][2] for i in range(500)]
        # print(draw_z)

        # np.savetxt('state.txt',self.state_drone1,newline='\n')
        sio.savemat(self.saveData_dir+'/data.mat', {'logger0':self.state_logger0,
                                                    'logger1':self.state_logger1,
                                                    'logger2': self.state_logger2,
                                                    'logger3': self.state_logger3,
                                                    'drone1':self.state_drone1,
                                                    'drone2':self.state_drone2,
                                                    'drone3':self.state_drone3,
                                                    'payload':self.state_payload
                                                    })
        print('save success')
        plt.plot(draw_z)
        plt.show()

    def run_xyz(self):
        # logger
        multiarray0 = Float32MultiArray()
        multiarray1 = Float32MultiArray()
        multiarray2 = Float32MultiArray()
        multiarray3 = Float32MultiArray()
        # UAV
        multiarray4 = Float32MultiArray()
        multiarray5 = Float32MultiArray()
        multiarray6 = Float32MultiArray()

        target = [0,0,2.5]
        pid = apmPID(target)
         # vehicle = connect('127.0.0.1:14551', wait_ready=True)
        self.start = time.time()
        r = rospy.Rate(20)
        count = 1
        while not rospy.is_shutdown():
            # self.unpausePhysics()
            self.offb_arm()
            # publish
            # Override1 = OverrideRCIn()
            # Override1.channels = [1800, 1700, 1800, 1800, 1500, 1600, 1500, 1500]
            # self.rc_override1.publish(Override1.channels)
            print('channels1', self.rc_states1.channels)
            print('channels2', self.rc_states2.channels)
            print('channels3', self.rc_states3.channels)
            # print('channelsRCIn1', self.rcIn_states1.channels)
            # print('channelsRCIn2', self.rcIn_states2.channels)
            # print('channelsRCIn3', self.rcIn_states3.channels)

            # self.pausePhysics()
            # drone1_fly为z周距离是否大于0
            dist_cone2drone1, drone1_fly, dist_cone2drone2, drone2_fly, dist_cone2drone3, drone3_fly = self.get_dist()
            print('distance',dist_cone2drone1,dist_cone2drone2,dist_cone2drone3)

            model_name = ['logger0', 'logger1', 'logger2', 'logger3', 'drone1', 'drone2', 'drone3', 'payload']
            [payload_position, _, _, _] = self.get_state(self.model_states, model_name[7])
            [drone1_position, _, _, _] = self.get_state(self.model_states,  model_name[4])
            [drone2_position, _, _, _] = self.get_state(self.model_states,  model_name[5])
            [drone3_position, _, _, _] = self.get_state(self.model_states,  model_name[6])
            print('drone1_position',drone1_position)
            print('drone2_position',drone2_position)
            print('drone3_position',drone3_position)

            # self.unpausePhysics()
            self.takeoff_service1(altitude=7.5)
            self.takeoff_service2(altitude=7.5)
            self.takeoff_service3(altitude=7.5)
            if np.abs(drone1_position[2]-7.5)<=0.2 :
                self.set_mode_client1(custom_mode="ALT_HOLD")
            if np.abs(drone2_position[2]-7.5)<=0.2 :
                self.set_mode_client2(custom_mode="ALT_HOLD")
            if np.abs(drone3_position[2]-7.5)<=0.2 :
                self.set_mode_client3(custom_mode="ALT_HOLD")

            #
            # thrust = Thrust()
            # thrust.thrust = 0.7
            # self.thrust_pub3.publish(thrust)

            # self.pausePhysics()
            # attitude = AttitudeTarget()
            # roll = 0.0
            # pitch = 0.0
            # yaw = 0.0
            # x,y,z,w = self.euler_to_quaternion(roll,pitch,yaw)
            #
            # attitude.orientation.x = x
            # attitude.orientation.y = y
            # attitude.orientation.z = z
            # attitude.orientation.w = w
            # attitude.thrust = 0.5005

            # attitude.thrust = 0.55
            # self.unpausePhysics()
            # self.setpoint_raw_pub1.publish(attitude)
            # self.setpoint_raw_pub2.publish(attitude)
            # self.setpoint_raw_pub3.publish(attitude)

            # act_controls = ActuatorControl()
            # act_controls.controls[3] = 0.6
            # self.acutator_control_pub1.publish(act_controls)
            # pos = PositionTarget()
            # pos.position.x = 5
            # pos.position.y = 4
            # pos.position.z = 6

            # self.setposition_raw_pub1.publish(pos)
            # self.setposition_raw_pub2.publish(pos)
            # self.setposition_raw_pub3.publish(pos)

            # pose_stamped = PoseStamped()
            #
            # pose = Pose()
            # if count == 1:
            #     self.init_posdrone1 = drone1_position
            # print(self.init_posdrone1)
            # count =count+1
            # pose.position.x = 0-self.init_posdrone1[0]
            # pose.position.y = 0-self.init_posdrone1[1]
            # pose.position.z = 7
            # pose.position.x = 0
            # pose.position.y = 0
            # pose.position.z = 7
            # pose.orientation.x = x
            # pose.orientation.y = y
            # pose.orientation.z = z
            # pose.orientation.w = w
            # pose_stamped.pose = pose
            # self.unpausePhysics()
            # self.local_pos_pub1.publish(pose_stamped)
            # self.local_pos_pub2.publish(pose_stamped)
            # self.local_pos_pub3.publish(pose_stamped)

            # self.pausePhysics()
            force_logger0,force_logger1,force_logger2,force_logger3 = self.compute_force(22,22,22,self.payload_gravity,self.drone_gravity,pid)

            # print('force_logger0',force_logger0)
            # print('force_logger1',force_logger1)
            # print('force_logger2',force_logger2)
            # print('force_logger3',force_logger3)

            self.Coeff_elasticity = 0.5
            if drone1_fly == True and dist_cone2drone1>=5:
                # if drone1_fly == True:
                #     multiarray4.data = [dist_cone2drone1, self.cable_length, self.Coeff_elasticity] # 距离，绳长，弹性系数
                # 重物与绳子间拉力
                force_payload2cable = self.payload_gravity*50+self.Coeff_elasticity*(dist_cone2drone1-5)*50
                force_payload2cable = self.force_cable1
                self.force_cable1 = self.force_cable1+self.Coeff_elasticity*(dist_cone2drone1-5)*50
                multiarray4.data = [self.force_cable1,force_payload2cable] # 拉力大小
                # self.unpausePhysics()
                self.force_publisher4.publish(multiarray4)

            # # self.pausePhysics()
            if drone2_fly == True and dist_cone2drone2>=5:
                # if drone2_fly == True:
                #     multiarray5.data = [dist_cone2drone2, self.cable_length, self.Coeff_elasticity]
                # 重物与绳子间拉力
                force_payload2cable2 = self.payload_gravity*50+self.Coeff_elasticity*(dist_cone2drone2-5)*50
                force_payload2cable2 = self.force_cable2
                self.force_cable2 = self.force_cable2+self.Coeff_elasticity*(dist_cone2drone2-5)*50
                multiarray5.data = [self.force_cable2,force_payload2cable2] # 拉力大小
                # self.unpausePhysics()
                self.force_publisher5.publish(multiarray5)

            # self.pausePhysics()
            if drone3_fly == True and dist_cone2drone3>=5:
                # if drone3_fly == True:
                #     multiarray6.data = [dist_cone2drone3, self.cable_length, self.Coeff_elasticity]
                # 重物与绳子间拉力
                force_payload2cable3 = self.payload_gravity*50+self.Coeff_elasticity*(dist_cone2drone3-5)*50
                force_payload2cable3 = self.force_cable3
                self.force_cable3 = self.force_cable3+self.Coeff_elasticity*(dist_cone2drone3-5)*50
                multiarray6.data = [self.force_cable3,force_payload2cable3] # 拉力大小
                # self.unpausePhysics()
                self.force_publisher6.publish(multiarray6)

            # self.pausePhysics()


            # self.unpausePhysics()

            if drone1_fly and payload_position[2]>0.5:
                multiarray0.data = [force_logger0*50]
                multiarray1.data = [force_logger1*50]
                multiarray2.data = [force_logger2*50]
                multiarray3.data = [force_logger3*50]
                # multiarray0.data = [500]
                # multiarray1.data = [500]
                # multiarray2.data = [500]
                # multiarray3.data = [500]
                self.force_publisher0.publish(multiarray0)
                self.force_publisher1.publish(multiarray1)
                self.force_publisher2.publish(multiarray2)
                self.force_publisher3.publish(multiarray3)

            # self.unpausePhysics()
            r.sleep()
            print(time.time()-self.start)
            # self.pausePhysics()
            # if dist_cone2drone1 < 2 or dist_cone2drone2 < 2 or dist_cone2drone3 < 2 or dist_cone2drone1 >10 or dist_cone2drone2 >10 or dist_cone2drone3 >10:
            #     self.pausePhysics()
            #     # self.resetWorld()
            #     self.resetSim()
            #     time.sleep(1)
            self.start = time.time()



    def compute_force(self,throttle1,throttle2,throttle3, payload_gravity,drone_gravity, pid):
        '''

        :param throttle1: 无人机1推力
        :param throttle2: 无人机2推力
        :param throttle3: 无人机3推力
        :param payload_gravity: 负载重力
        :param pid: pid控制器
        :return: 返回小车拉力大小
        '''
        model_name = ['logger0', 'logger1', 'logger2', 'logger3', 'drone1', 'drone2', 'drone3', 'payload']
        [payload_position, _, _, _] = self.get_state(self.model_states,model_name[7])
        [logger0_position, _, _, _] = self.get_state(self.model_states,model_name[0])
        [logger1_position, _, _, _] = self.get_state(self.model_states,model_name[1])
        [logger2_position, _, _, _] = self.get_state(self.model_states,model_name[2])
        [logger3_position, _, _, _] = self.get_state(self.model_states,model_name[3])
        [drone1_position, _, _, _] = self.get_state(self.model_states,model_name[4])
        [drone2_position, _, _, _] = self.get_state(self.model_states,model_name[5])
        [drone3_position, _, _, _] = self.get_state(self.model_states,model_name[6])

        # 使用mavros获取无人机位置信息
        [drone1_position,drone2_position,drone3_position,ori_drone1,ori_drone2,ori_drone3] = self.get_posByMavros()

        # 负载与小车间的力的方向,指向小车
        delta_load2logger0 = [logger0_position[0] - payload_position[0],logger0_position[1]-payload_position[1],logger0_position[2]-(payload_position[2])]
        delta_load2logger1 = [logger1_position[0] - payload_position[0],logger1_position[1]-payload_position[1],logger1_position[2]-(payload_position[2])]
        delta_load2logger2 = [logger2_position[0] - payload_position[0],logger2_position[1]-payload_position[1],logger2_position[2]-(payload_position[2])]
        delta_load2logger3 = [logger3_position[0] - payload_position[0],logger3_position[1]-payload_position[1],logger3_position[2]-(payload_position[2])]

        # 皈依化
        delta_load2logger0 = delta_load2logger0 / np.sqrt(np.sum(np.square(delta_load2logger0)))
        delta_load2logger1 = delta_load2logger1 / np.sqrt(np.sum(np.square(delta_load2logger1)))
        delta_load2logger2 = delta_load2logger2 / np.sqrt(np.sum(np.square(delta_load2logger2)))
        delta_load2logger3 = delta_load2logger3 / np.sqrt(np.sum(np.square(delta_load2logger3)))

        # 绳子方向,指向负载
        delta_load2drone1 = [payload_position[0]-drone1_position[0],payload_position[1]-drone1_position[1],payload_position[2]-drone1_position[2]]
        delta_load2drone2 = [payload_position[0]-drone2_position[0],payload_position[1]-drone2_position[1],payload_position[2]-drone2_position[2]]
        delta_load2drone3 = [payload_position[0]-drone3_position[0],payload_position[1]-drone3_position[1],payload_position[2]-drone3_position[2]]
        #归一化
        delta_load2drone1 = delta_load2drone1/np.sqrt(np.sum(np.square(delta_load2drone1)))
        delta_load2drone2 = delta_load2drone2/np.sqrt(np.sum(np.square(delta_load2drone2)))
        delta_load2drone3 = delta_load2drone3/np.sqrt(np.sum(np.square(delta_load2drone3)))

        # 推力方向皈依化
        ori_drone1 = ori_drone1 / np.sqrt(np.sum(np.square(ori_drone1)))
        ori_drone2 = ori_drone2 / np.sqrt(np.sum(np.square(ori_drone2)))
        ori_drone3 = ori_drone3 / np.sqrt(np.sum(np.square(ori_drone3)))
        #绳子与重力方向夹角,向量积的形式计算
        theta_cableGravity1 = np.arccos(delta_load2drone1[2])
        theta_cableGravity2 = np.arccos(delta_load2drone2[2])
        theta_cableGravity3 = np.arccos(delta_load2drone3[2])
        # 绳子与推力方向夹角
        theta_cableThrottle1 = np.arccos(np.sum([a*b for a,b in zip(ori_drone1,delta_load2drone1)]))
        theta_cableThrottle2 = np.arccos(np.sum([a*b for a,b in zip(ori_drone2,delta_load2drone2)]))
        theta_cableThrottle3 = np.arccos(np.sum([a*b for a,b in zip(ori_drone3,delta_load2drone3)]))
        # 绳子方向上受的合力大小
        force_cable1 = drone_gravity*np.cos(theta_cableGravity1) - throttle1 *np.cos(theta_cableThrottle1)
        force_cable2 = drone_gravity*np.cos(theta_cableGravity2) - throttle2 *np.cos(theta_cableThrottle2)
        force_cable3 = drone_gravity*np.cos(theta_cableGravity3) - throttle3 *np.cos(theta_cableThrottle3)
        # 绳子方向上负载理论上受到 的绳子拉力(方向+大小)
        force_cableDrone1 = -np.abs(force_cable1)*delta_load2drone1
        force_cableDrone2 = -np.abs(force_cable2)*delta_load2drone2
        force_cableDrone3 = -np.abs(force_cable3)*delta_load2drone3
        print('force_cabledrone1', force_cableDrone1)
        print('force_cabledrone2', force_cableDrone2)
        print('force_cabledrone3', force_cableDrone3)
        # publish
        self.force_cable1 = np.abs(force_cable1)*50
        self.force_cable2 = np.abs(force_cable2)*50
        self.force_cable3 = np.abs(force_cable3)*50
        # print('force_cable',self.force_cable1,self.force_cable2,self.force_cable3)
        # pid
        force_Xerror, force_Yerror, force_Zerror = pid.cal_actions(payload_position)
        force_pid = [force_Xerror,force_Yerror,force_Zerror]
        print('force_pid',force_pid)
        print('payload_position',payload_position)
        # 目标小车合力
        # force_pid = [0,0,0]
        # 多架无人机
        target_forceLogger0123 = force_pid -  force_cableDrone1 - force_cableDrone2 - force_cableDrone3 - [0,0,-payload_gravity]

        # 一架无人机
        # target_forceLogger0123 = force_pid -  force_cableDrone1 - [0,0,-payload_gravity]

        print('target_forceLogger0123', target_forceLogger0123)
        from optimizer import minimizeForce
        allArgs = (target_forceLogger0123,delta_load2logger0,delta_load2logger1,delta_load2logger2,delta_load2logger3)
        force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3 = minimizeForce(allArgs)
        value = force_valueLogger0 * delta_load2logger0 + force_valueLogger1 * delta_load2logger1 + force_valueLogger2 * delta_load2logger2 + force_valueLogger3 * delta_load2logger3 - target_forceLogger0123
        # print('value',value)
        print('carforce',force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3)
        return force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3


    def run_logger(self):

        cmd1 = rospy.Publisher('/logger1/cmd_vel', Twist, queue_size=1)
        cmd0 = rospy.Publisher('/logger0/cmd_vel', Twist, queue_size=1)
        cmd2 = rospy.Publisher('/logger2/cmd_vel', Twist, queue_size=1)
        cmd3 = rospy.Publisher('/logger3/cmd_vel', Twist, queue_size=1)
        multiarray0 = Float32MultiArray()
        multiarray1 = Float32MultiArray()
        multiarray2 = Float32MultiArray()
        multiarray3 = Float32MultiArray()
        multiarray4 = Float32MultiArray()
        multiarray0.data = [20]
        multiarray1.data = [0]
        multiarray2.data = [0]
        multiarray3.data = [0]
        # multiarray4.data = [10]
        vel = Twist()
        vel.angular.z = 10
        while not rospy.is_shutdown():
            dist_cone2drone1, drone1_fly, dist_cone2drone2, drone2_fly, dist_cone2drone3, drone3_fly = self.get_dist()
            vel.linear.x = np.random.uniform(2)
            # vel.linear.x = 0
            # print('ssssss')
            self.force_publisher0.publish(multiarray0)
            self.force_publisher1.publish(multiarray1)
            self.force_publisher2.publish(multiarray2)
            self.force_publisher3.publish(multiarray3)
            # cmd1.publish(vel)
            # cmd0.publish(vel)
            # cmd2.publish(vel)
            # cmd3.publish(vel)
    def get_dist2(self):
        '''
        使用速度计算为位置
        :return:
        '''
        id_logger0 = self.model_states.name.index("logger0")  #
        id_logger1 = self.model_states.name.index("logger1")  #
        id_logger2 = self.model_states.name.index("logger2")  #
        id_logger3 = self.model_states.name.index("logger3")  #
        id_payload = self.model_states.name.index("payload")  #
        id_drone1 = self.model_states.name.index("drone1")  #
        id_drone2 = self.model_states.name.index("drone2")  #
        id_drone3 = self.model_states.name.index("drone3")  #
        payload_twist = self.model_states.twist[id_payload]
        drone_twist1 = self.model_states.twist[id_drone1]
        drone_twist2 = self.model_states.twist[id_drone2]
        drone_twist3 = self.model_states.twist[id_drone3]
        logger_twist0 = self.model_states.twist[id_logger0]
        self.dt = time.time() - self.start
        self.payload_pose.position.x = self.payload_pose.position.x + payload_twist.linear.x * self.dt
        self.payload_pose.position.y = self.payload_pose.position.y + payload_twist.linear.y * self.dt
        self.payload_pose.position.z = self.payload_pose.position.z + payload_twist.linear.z * self.dt
        self.drone_pose1.position.x = self.drone_pose1.position.x + drone_twist1.linear.x * self.dt
        self.drone_pose1.position.y = self.drone_pose1.position.y + drone_twist1.linear.y * self.dt
        self.drone_pose1.position.z = self.drone_pose1.position.z + drone_twist1.linear.z * self.dt
        self.drone_pose2.position.x = self.drone_pose2.position.x + drone_twist2.linear.x * self.dt
        self.drone_pose2.position.y = self.drone_pose2.position.y + drone_twist2.linear.y * self.dt
        self.drone_pose2.position.z = self.drone_pose2.position.z + drone_twist2.linear.z * self.dt
        self.drone_pose3.position.x = self.drone_pose3.position.x + drone_twist3.linear.x * self.dt
        self.drone_pose3.position.y = self.drone_pose3.position.y + drone_twist3.linear.y * self.dt
        self.drone_pose3.position.z = self.drone_pose3.position.z + drone_twist3.linear.z * self.dt
        self.logger_pose0.position.x = self.logger_pose0.position.x + logger_twist0.linear.x * self.dt
        self.logger_pose0.position.y = self.logger_pose0.position.y + logger_twist0.linear.y * self.dt
        self.logger_pose0.position.z = self.logger_pose0.position.z + logger_twist0.linear.z * self.dt
        dist_cone2drone1 = np.sqrt(np.sum(np.square([self.payload_pose.position.x - self.drone_pose1.position.x,
                                                     self.payload_pose.position.y - self.drone_pose1.position.y,
                                                     self.payload_pose.position.z - self.drone_pose1.position.z])))
        # dist_cone2drone1 = np.sqrt(np.sum(np.square([payload_pose.position.x-drone_pose1.position.x,payload_pose.position.y-drone_pose1.position.y,payload_pose.position.z-drone_pose1.position.z])))
        dist_cone2drone2 = np.sqrt(np.sum(np.square(
            [self.payload_pose.position.x - self.drone_pose2.position.x, self.payload_pose.position.y - self.drone_pose2.position.y,
             self.payload_pose.position.z - self.drone_pose2.position.z])))
        dist_cone2drone3 = np.sqrt(np.sum(np.square(
            [self.payload_pose.position.x - self.drone_pose3.position.x, self.payload_pose.position.y - self.drone_pose3.position.y,
             self.payload_pose.position.z - self.drone_pose3.position.z])))
        return dist_cone2drone1, self.drone_pose1.position.z > 0.05, dist_cone2drone2, self.drone_pose2.position.z > 0.05, dist_cone2drone3, self.drone_pose3.position.z > 0.05


    def get_dist(self):
        id_logger0 = self.model_states.name.index("logger0")  #
        id_logger1 = self.model_states.name.index("logger1")  #
        id_logger2 = self.model_states.name.index("logger2")  #
        id_logger3 = self.model_states.name.index("logger3")  #
        id_payload = self.model_states.name.index("payload")  #
        id_drone1 = self.model_states.name.index("drone1")  #
        id_drone2 = self.model_states.name.index("drone2")  #
        id_drone3 = self.model_states.name.index("drone3")  #
        logger_pose0 = self.model_states.pose[id_logger0]
        logger_pose1 = self.model_states.pose[id_logger1]
        logger_pose2 = self.model_states.pose[id_logger2]
        logger_pose3 = self.model_states.pose[id_logger3]
        payload_pose = self.model_states.pose[id_payload]
        drone_pose1 = self.model_states.pose[id_drone1]
        drone_pose2 = self.model_states.pose[id_drone2]
        drone_pose3 = self.model_states.pose[id_drone3]

        # 使用amvros返回无人机位置
        drone_pose1 = self.local_position1.pose
        drone_pose2 = self.local_position2.pose
        drone_pose3 = self.local_position3.pose



        dist_cone2drone1 = np.sqrt(np.sum(np.square([payload_pose.position.x-drone_pose1.position.x,payload_pose.position.y-drone_pose1.position.y,payload_pose.position.z-drone_pose1.position.z])))
        dist_cone2drone2 = np.sqrt(np.sum(np.square([payload_pose.position.x-drone_pose2.position.x,payload_pose.position.y-drone_pose2.position.y,payload_pose.position.z-drone_pose2.position.z])))
        dist_cone2drone3 = np.sqrt(np.sum(np.square([payload_pose.position.x-drone_pose3.position.x,payload_pose.position.y-drone_pose3.position.y,payload_pose.position.z-drone_pose3.position.z])))
        return dist_cone2drone1, drone_pose1.position.z>0.05, dist_cone2drone2, drone_pose2.position.z>0.05, dist_cone2drone3, drone_pose3.position.z>0.05

    def get_posByMavros(self):
        '''
        使用mavros话题获取无人机状态
        :return: 返回无人机的位置
        '''
        pos_drone1 = [self.local_position1.pose.position.x,self.local_position1.pose.position.y,self.local_position1.pose.position.z]
        pos_drone2 = [self.local_position2.pose.position.x,self.local_position2.pose.position.y,self.local_position2.pose.position.z]
        pos_drone3 = [self.local_position3.pose.position.x,self.local_position3.pose.position.y,self.local_position3.pose.position.z]
        ori_drone1 = self.quaternion_to_euler(self.local_position1.pose.orientation.x,self.local_position1.pose.orientation.y,self.local_position1.pose.orientation.z,self.local_position1.pose.orientation.w)
        ori_drone2 = self.quaternion_to_euler(self.local_position2.pose.orientation.x,self.local_position2.pose.orientation.y,self.local_position2.pose.orientation.z,self.local_position2.pose.orientation.w)
        ori_drone3 = self.quaternion_to_euler(self.local_position3.pose.orientation.x,self.local_position3.pose.orientation.y,self.local_position3.pose.orientation.z,self.local_position3.pose.orientation.w)

        return [pos_drone1,pos_drone2,pos_drone3,ori_drone1,ori_drone2,ori_drone3]

    def get_state(self,model_state, model_name):
        model = model_state.name.index(model_name)  #

        model_pose = model_state.pose[model]
        model_twist = model_state.twist[model]
        model_position = [model_pose.position.x,model_pose.position.y,model_pose.position.z]
        model_orientation = [model_pose.orientation.x,model_pose.orientation.y,model_pose.orientation.z,model_pose.orientation.w]
        model_linear = [model_twist.linear.x,model_twist.linear.y,model_twist.linear.z]
        model_angular = [model_twist.angular.x,model_twist.angular.y,model_twist.angular.z]
        # print([model_position,model_orientation,model_linear,model_angular])
        return [model_position,model_orientation,model_linear,model_angular]

    def save2list(self):
        models_name = ['logger0','logger1','logger2','logger3','drone1','drone2','drone3','payload']
        for model_name in models_name:
            model_state = self.get_state(self.model_states,model_name)
            # print(np.shape(model_state))
            # print(model_state[0])
            if model_name == 'logger0':
                self.state_logger0.append(model_state)

            if model_name == 'logger1':
                self.state_logger1.append(model_state)

            if model_name == 'logger2':
                self.state_logger2.append(model_state)

            if model_name == 'logger3':
                self.state_logger3.append(model_state)

            if model_name == 'drone1':
                self.state_drone1.append(model_state)
                # print(model_state[0][2])

            if model_name == 'drone2':
                self.state_drone2.append(model_state)

            if model_name == 'drone3':
                self.state_drone3.append(model_state)

            if model_name == 'payload':
                self.state_payload.append(model_state)



    def quaternion_to_euler(self, x, y, z, w):

        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # X = math.degrees(math.atan2(t0, t1))
        #
        # t2 = +2.0 * (w * y - z * x)
        # # t2 = +1.0 if t2 > +1.0 else t2
        # # t2 = -1.0 if t2 < -1.0 else t2
        # Y = math.degrees(math.asin(t2))
        #
        # t3 = +2.0 * (w * z + x * y)
        # t4 = +1.0 - 2.0 * (y * y + z * z)
        # Z = math.degrees(math.atan2(t3, t4))

            # 使用 tf 库
        import tf
        (X, Y, Z) = tf.transformations.euler_from_quaternion([x, y, z, w])
        return X, Y, Z

    def euler_to_quaternion(self, roll, pitch, yaw):
        # x=sin(pitch/2)sin(yaw/2)cos(roll/2)+cos(pitch/2)cos(yaw/2)sin(roll/2)
        # y=sin(pitch/2)cos(yaw/2)cos(roll/2)+cos(pitch/2)sin(yaw/2)sin(roll/2)
        # z=cos(pitch/2)sin(yaw/2)cos(roll/2)-sin(pitch/2)cos(yaw/2)sin(roll/2)
        # w=cos(pitch/2)cos(yaw/2)cos(roll/2)-sin(pitch/2)sin(yaw/2)sin(roll/2)
        import tf
        (x, y, z, w) = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        return x, y, z, w


    def lv_cb1(self,data):
        self.local_velocity1 = data
        # print(self.local_velocity1.twist.linear.x)
        # print(self.local_velocity1.twist.linear.y)
        # print(self.local_velocity1.twist.linear.z)
    def lv_cb2(self,data):
        self.local_velocity2 = data
    def lv_cb3(self,data):
        self.local_velocity3 = data

    def lp_cb1(self,data):
        self.local_position1 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)
    def lp_cb2(self,data):
        self.local_position2 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)
    def lp_cb3(self,data):
        self.local_position3 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)

    #
    # def state_cb(self,data):
    #     self.current_state = data

    # def imu_cb(self,data):
    #     self.imu_data = data

    # def act_cb(self,data):
    #     self.act_controls = data

    # def gv_cb(self,data):
    #     self.global_velocity = data

    # def ra_cb(self,data):
    #     self.relative_altitude = data
    # def bat_cb(self,data):
    #     self.battery = data

    def _model_states_cb(self,data):
        self.model_states  = data

    def rc_cb1(self,data):
        self.rc_states1 = data
    def rc_cb2(self,data):
        self.rc_states2 = data
    def rc_cb3(self,data):
        self.rc_states3 = data
    def rcIn_cb1(self,data):
        self.rcIn_states1 = data
    def rcIn_cb2(self,data):
        self.rcIn_states2 = data
    def rcIn_cb3(self,data):
        self.rcIn_states3 = data


class apmPID:
    def __init__(self,target):
        # 位置式
        # self.control_x = PID_posi(0.01, 0.001, 0.01, target[0], up=30, low=-30)  # control position x
        # self.control_y = PID_posi(0.01, 0.001, 0.02, target[1], up=30, low=-30)  # control position y
        # self.control_z = PID_posi(0.01, 0.005, 0.01, target[2], up=30, low= -30)  # control position z
        # 增量式
        # self.control_x = PID_inc(0.01, 0, 0.001, target[0], up=10, low=-10)  # control position x
        # self.control_y = PID_inc(0.03, 0., 0.002, target[1], up=10, low=-10)  # control position y
        # self.control_z = PID_inc(10, 0.0, 0.00, target[2], up=100, low=0)  # control position z

        ## 2
        self.control_x = PID([0.25, 0.00, 0.01], target[0], upper=2.5, lower=-2.5)  # control position x
        self.control_y = PID([0.25, 0.00, 0.01], target[1], upper=2.5, lower=-2.5)  # control position y
        self.control_z = PID([0.1, 0.00, 0.01], target[2], upper=0, lower= -1)  # control position z

        # self.control_x = PID([0.3, 0.00, 0.01], target[0], upper=5, lower=-5)  # control position x
        # self.control_y = PID([1, 0.00, 0.1], target[1], upper=5, lower=-5)  # control position y
        # self.control_z = PID([2, 0.00, 0.1], target[2], upper=5, lower= -5)  # control position z


    def cal_actions(self,state):
        # self.control_x.increase(state[0])
        # self.control_y.increase(state[1])
        # self.control_z.increase(state[2])
        # list = [self.control_x.value, self.control_y.value, self.control_z.value]

        ## 2
        u1 = self.control_x.cal_output(state[0])
        u2 = self.control_y.cal_output(state[1])
        u3 = self.control_z.cal_output(state[2])
        list = [u1, u2, u3]
        return list
    #
    # def reset(self):
    #     self.control_x.reset()
    #     self.control_y.reset()
    #     self.control_z.reset()


if __name__ == "__main__":
    env = CDPR()
    # env.run()
    env.run_xyz()
    # env.run_logger()

    # rospy.init_node('apm_mavros', anonymous=True)
    # cmd1 = rospy.Publisher('/logger1/cmd_vel', Twist, queue_size=1)
    # cmd0 = rospy.Publisher('/logger0/cmd_vel', Twist, queue_size=1)
    # cmd2 = rospy.Publisher('/logger2/cmd_vel', Twist, queue_size=1)
    # cmd3 = rospy.Publisher('/logger3/cmd_vel', Twist, queue_size=1)
    # multiarray0 = Float32MultiArray()
    # multiarray1 = Float32MultiArray()
    # multiarray2 = Float32MultiArray()
    # multiarray3 = Float32MultiArray()
    # multiarray4 = Float32MultiArray()
    # multiarray0.data = [6]
    # multiarray1.data = [0]
    # multiarray2.data = [0]
    # multiarray3.data = [0]
    # # multiarray4.data = [10]
    # vel = Twist()
    # vel.angular.z = 10
    # force_publisher0 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=1)
    # force_publisher1 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=1)
    # force_publisher2 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=1)
    # force_publisher3 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=1)
    # while not rospy.is_shutdown():
    #     vel.linear.x = np.random.uniform(2)
    #     # vel.linear.x = 0
    #     # print('ssssss')
    #     force_publisher0.publish(multiarray0)
    #     force_publisher1.publish(multiarray1)
    #     force_publisher2.publish(multiarray2)
    #     force_publisher3.publish(multiarray3)
        # cmd1.publish(vel)
        # cmd0.publish(vel)
        # cmd2.publish(vel)
        # cmd3.publish(vel)



