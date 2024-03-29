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
from std_msgs.msg import Float32MultiArray, Bool
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Twist
import numpy as np
import os
import tf
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
from PIDClass import PID
import matplotlib.pyplot as plt

import gym
from gym import spaces

from dronekit import connect, VehicleMode, LocationGlobalRelative
import scipy.io as sio
import math

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, GetModelState  # 设置模型状态、得到模型状态
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose, Twist


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
        self.global_velocity = TwistStamped()
        self.battery = BatteryState()
        self.local_position1 = PoseStamped()
        self.local_position2 = PoseStamped()
        self.model_states = ModelStates()
        self.rc_states1 = RCOut()
        self.rc_states2 = RCOut()
        self.rcIn_states1 = RCIn()
        self.rcIn_states2 = RCIn()

        # # 物理属性
        self.cable_length = 5
        self.payload_gravity = 0.25*9.8
        self.drone_gravity = 18.88
        self.Coeff_elasticity = 4
        # 数据保存
        self.state_logger0 = []
        self.state_logger1 = []
        self.state_logger2 = []
        self.state_logger3 = []
        self.state_drone1 = []
        self.state_drone2 = []
        self.state_payload = []
        self.cable_drone1 = []
        self.cable_drone2 = []
        self.force_logger0 = []
        self.force_logger1 = []
        self.force_logger2 = []
        self.force_logger3 = []
        self.thrust_drone1 = []
        self.thrust_drone2 = []
        self.target_loggerAll = []
        self.timestep = []
        self.pwm2thrust1 = []
        self.pwm2thrust2 = []
        # 画图
        self.dataPosx_payload = []
        self.dataPosy_payload = []
        self.dataPosz_payload = []
        self.data_force1 = []
        self.data_force2 = []
        self.data_force3 = []
        self.data_force4 = []
        self.data_droneforce1 = []
        self.data_droneforce2 = []
        self.dataPosz_drone1 = []
        self.dataPosz_drone2 = []
        self.dt_draw = []
        self.dt = 0.1  # 仿真频率

        self.saveData_dir = "/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data"
        if not os.path.exists(self.saveData_dir):
            os.makedirs(self.saveData_dir)

        ## ROS Subscribers
        self.local_pos_sub1 = rospy.Subscriber(
            "/drone1/mavros/local_position/pose", PoseStamped, self.lp_cb1, queue_size=1)
        self.local_pos_sub2 = rospy.Subscriber(
            "/drone2/mavros/local_position/pose", PoseStamped, self.lp_cb2, queue_size=1)

        self.rc_out1 = rospy.Subscriber(
            "/drone1/mavros/rc/out", RCOut, self.rc_cb1, queue_size=1)
        self.rc_out2 = rospy.Subscriber(
            "/drone2/mavros/rc/out", RCOut, self.rc_cb2, queue_size=1)
        self.rc_in1 = rospy.Subscriber(
            "/drone1/mavros/rc/in", RCIn, self.rcIn_cb1, queue_size=1)
        self.rc_in2 = rospy.Subscriber(
            "/drone2/mavros/rc/in", RCIn, self.rcIn_cb2, queue_size=1)

        ## ROS Publishers
        # self.mocap_pos_pub = rospy.Publisher("/drone1/mavros/mocap/pose",PoseStamped,queue_size=1)
        self.acutator_control_pub1 = rospy.Publisher(
            "/drone1/mavros/actuator_control", ActuatorControl, queue_size=1)
        self.acutator_control_pub2 = rospy.Publisher(
            "/drone2/mavros/actuator_control", ActuatorControl, queue_size=1)
        self.setpoint_raw_pub1 = rospy.Publisher(
            "/drone1/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        self.setpoint_raw_pub2 = rospy.Publisher(
            "/drone2/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)
        self.setposition_raw_pub1 = rospy.Publisher(
            "/drone1/mavros/setpoint_raw/local", PositionTarget, queue_size=1)  # NED
        self.setposition_raw_pub2 = rospy.Publisher(
            "/drone2/mavros/setpoint_raw/local", PositionTarget, queue_size=1)  # NED

        self.local_pos_pub1 = rospy.Publisher(
            "/drone1/mavros/setpoint_position/local", PoseStamped, queue_size=1)  # ENU
        self.local_pos_pub2 = rospy.Publisher(
            "/drone2/mavros/setpoint_position/local", PoseStamped, queue_size=1)

        self.thrust_pub1 = rospy.Publisher(
            "/drone1/mavros/setpoint_attitude/thrust", Thrust, queue_size=1)
        self.thrust_pub2 = rospy.Publisher(
            "/drone2/mavros/setpoint_attitude/thrust", Thrust, queue_size=1)

        self.rc_override1 = rospy.Publisher(
            "/drone1/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.rc_override2 = rospy.Publisher(
            "/drone2/mavros/rc/override", OverrideRCIn, queue_size=1)

        self.force_publisher0 = rospy.Publisher(
            '/loggerAll/force', Float32MultiArray, queue_size=1)

        self.force_publisher4 = rospy.Publisher(
            '/drone1/force', Float32MultiArray, queue_size=1)
        self.force_publisher5 = rospy.Publisher(
            '/drone2/force', Float32MultiArray, queue_size=1)
        self.run_cmd_pub = rospy.Publisher('/cmd_running', Bool, queue_size=1)

        # gazebo topic
        self.modestate_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._model_states_cb)
        # gazebo服务
        self.reset_world_proxy = rospy.ServiceProxy(
            '/gazebo/reset_world', Empty)  # 指定服务名来调用服务
        self.reset_payload_proxy = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.reset_simulation = rospy.ServiceProxy(
            '/gazebo/reset_simulation', Empty)
        self.unpause_physics_proxy = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)
        self.pause_physics_proxy = rospy.ServiceProxy(
            '/gazebo/pause_physics', Empty)

        ## ROS mavros Services
        rospy.wait_for_service('drone1/mavros/cmd/arming')
        self.arming_client1 = rospy.ServiceProxy(
            'drone1/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('drone2/mavros/cmd/arming')
        self.arming_client2 = rospy.ServiceProxy(
            'drone2/mavros/cmd/arming', CommandBool)

        rospy.wait_for_service('drone1/mavros/cmd/takeoff')
        self.takeoff_service1 = rospy.ServiceProxy(
            '/drone1/mavros/cmd/takeoff', CommandTOL)
        rospy.wait_for_service('drone2/mavros/cmd/takeoff')
        self.takeoff_service2 = rospy.ServiceProxy(
            '/drone2/mavros/cmd/takeoff', CommandTOL)

        rospy.wait_for_service('drone1/mavros/set_mode')
        self.set_mode_client1 = rospy.ServiceProxy(
            'drone1/mavros/set_mode', SetMode)
        rospy.wait_for_service('drone2/mavros/set_mode')
        self.set_mode_client2 = rospy.ServiceProxy(
            'drone2/mavros/set_mode', SetMode)

        ### Initiate ROS node
        print('-- Connecting to mavros')
        rospy.init_node('gym_apm_mavros', anonymous=True)
        print('connected')

        id_logger0 = self.model_states.name.index("logger0")  #
        id_logger1 = self.model_states.name.index("logger1")  #
        id_logger2 = self.model_states.name.index("logger2")  #
        id_logger3 = self.model_states.name.index("logger3")  #
        id_payload = self.model_states.name.index("payload")  #
        id_drone1 = self.model_states.name.index("drone1")  #
        id_drone2 = self.model_states.name.index("drone2")  #
        logger_pose0 = self.model_states.pose[id_logger0]
        logger_pose1 = self.model_states.pose[id_logger1]
        logger_pose2 = self.model_states.pose[id_logger2]
        logger_pose3 = self.model_states.pose[id_logger3]
        payload_pose = self.model_states.pose[id_payload]
        drone_pose1 = self.model_states.pose[id_drone1]
        drone_pose2 = self.model_states.pose[id_drone2]
        # =====================初始位置=====================
        self.payload_pose = payload_pose
        self.drone_pose1 = drone_pose1
        self.drone_pose2 = drone_pose2
        self.logger_pose0 = logger_pose0
        # ==============================================

    def resetPayload(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            payload = SetModelStateRequest()
            payload.model_state.model_name = "payload"
            payload.model_state.pose.position.x = 0.0
            payload.model_state.pose.position.y = 0.0
            payload.model_state.pose.position.z = 0.0
            payload.model_state.pose.orientation.x = 0
            payload.model_state.pose.orientation.y = 0
            payload.model_state.pose.orientation.z = 0
            payload.model_state.pose.orientation.w = 1
            payload.model_state.twist.linear.x = 0.0
            payload.model_state.twist.linear.y = 0.0
            payload.model_state.twist.linear.z = 0.0
            payload.model_state.twist.angular.x = 0.0
            payload.model_state.twist.angular.y = 0.0
            payload.model_state.twist.angular.z = 0.0
            payload.model_state.reference_frame = "world"
            self.reset_payload_proxy(payload)
        except rospy.ServiceException as e:
            rospy.logerr("/gazebo/reset_payload service call failed")

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

    def run_last(self):
        # logger
        multiarray0 = Float32MultiArray()

        # UAV
        multiarray4 = Float32MultiArray()
        multiarray5 = Float32MultiArray()


        self.target_z =3.
        target = [0, 0, self.target_z]
        self.altitude = 7.18
        self.Coeff_elasticity = 2
        pid = apmPID(target)
        # vehicle = connect('127.0.0.1:14551', wait_ready=True)
        self.start = time.time()
        r = rospy.Rate(10)
        count = 1
        count1 = 1
        count2 = 1
        count3 = 1
        count4 = 1
        count5 = 1
        count6 = 1
        count7 = 1
        flag = False
        count_draw = 1
        flag_drone1 = False
        flag_drone2 = False
        flag_drone3 = False
        # self.offb_arm()
        # self.takeoff_service1(altitude=7)
        # self.takeoff_service2(altitude=7)
        # time.sleep(5)
        running = Bool()
        pose_stamp1 = PoseStamped()
        pose_stamp2 = PoseStamped()
        pose_stamp1.pose.position.x = 0.0
        pose_stamp1.pose.position.y = 0.0
        pose_stamp1.pose.position.z = self.altitude
        pose_stamp2.pose.position.x = 0.0
        pose_stamp2.pose.position.y = 0.0
        pose_stamp2.pose.position.z = self.altitude
        self.resetPayload()
        while not rospy.is_shutdown():
            # self.unpausePhysics()
            self.offb_arm()
            # publish
            # Override1 = OverrideRCIn()
            # Override1.channels = [1800, 1700, 1800, 1800, 1500, 1600, 1500, 1500]
            # self.rc_override1.publish(Override1.channels)
            # print('channels1', self.rc_states1.channels)
            # print('channels2', self.rc_states2.channels)
            # print('channels3', self.rc_states3.channels)
            # print('channelsRCIn1', self.rcIn_states1.channels)
            # print('channelsRCIn2', self.rcIn_states2.channels)
            # print('channelsRCIn3', self.rcIn_states3.channels)
            if self.rc_states1.channels == [] or self.rc_states2.channels == []:
                self.thrust1 = 0
                self.thrust2 = 0
                continue
            pwm2_thrust1 = [0.01894*self.rc_states1.channels[0]-24.91, 0.01894*self.rc_states1.channels[1] -
                            24.91, 0.01894*self.rc_states1.channels[2]-24.91, 0.01894*self.rc_states1.channels[3]-24.91]
            pwm2_thrust2 = [0.01894*self.rc_states2.channels[0]-24.91, 0.01894*self.rc_states2.channels[1] -
                            24.91, 0.01894*self.rc_states2.channels[2]-24.91, 0.01894*self.rc_states2.channels[3]-24.91]

            self.pwm2thrust1.append(pwm2_thrust1)
            self.pwm2thrust2.append(pwm2_thrust2)
            self.thrust1 = 0.01894 * (self.rc_states1.channels[0] + self.rc_states1.channels[1] + self.rc_states1.channels[2] +self.rc_states1.channels[3]) - 99.63
            self.thrust2 = 0.01894 * (self.rc_states2.channels[0] + self.rc_states2.channels[1] + self.rc_states2.channels[2] +self.rc_states2.channels[3]) - 99.63

            self.takeoff_service1(altitude=self.altitude)
            self.takeoff_service2(altitude=self.altitude)

            # 3 新的弹力模型
            # 第二种弹力模型的绳长计算
            if count_draw > 0:
                model_name = ['logger0', 'logger1', 'logger2',
                              'logger3', 'drone1', 'drone2', 'drone3', 'payload']
                [self.payload_position, _, self.payload_linear,_] = self.get_state(self.model_states, model_name[7])
                [self.drone1_position, _, _, _] = self.get_state(self.model_states, model_name[4])
                [self.drone2_position, _, _, _] = self.get_state(self.model_states, model_name[5])
                # [self.drone1_position, self.drone2_position, ori_drone1, ori_drone2] = self.get_posByMavros()
            if count_draw > 100:
                self.drone1_position[2] = self.altitude-(self.target_z-self.payload_position[2])
                self.drone2_position[2] = self.altitude-(self.target_z-self.payload_position[2])
            delta_load2drone1_ = [self.payload_position[0] - self.drone1_position[0],
                                  self.payload_position[1] -
                                  self.drone1_position[1],
                                  self.payload_position[2] - self.drone1_position[2]]
            delta_load2drone2_ = [self.payload_position[0] - self.drone2_position[0],
                                  self.payload_position[1] -
                                  self.drone2_position[1],
                                  self.payload_position[2] - self.drone2_position[2]]
            self.dist_cable1 = np.sqrt(np.sum(np.square(delta_load2drone1_)))
            self.dist_cable2 = np.sqrt(np.sum(np.square(delta_load2drone2_)))
            force_payload2drone1 = self.Coeff_elasticity * \
                (self.dist_cable1 - 1-4)
            force_payload2drone2 = self.Coeff_elasticity * \
                (self.dist_cable2 - 1-4)

            if force_payload2drone1 < 0:
                force_payload2drone1 = 0
            if force_payload2drone2 < 0:
                force_payload2drone2 = 0
            if(self.drone1_position[2] > 5):
                self.local_pos_pub1.publish(pose_stamp1)
                self.local_pos_pub2.publish(pose_stamp2)
            # drone1
            if self.drone1_position[2] > 0:
                multiarray4.data = [force_payload2drone1,
                                    force_payload2drone1, count1]  # 拉力大小
                count += 1
            else:
                multiarray4.data = [0, 0, count1]
                flag = False

            # drone2
            if self.drone2_position[2] > 0:
                multiarray5.data = [force_payload2drone2,
                                    force_payload2drone2, count2]  # 拉力大小
            else:
                multiarray5.data = [0, 0, count2]
                flag = False

            ##################################################
            force_logger0, force_logger1, force_logger2, force_logger3 = self.compute_LoggerForce(
                force_payload2drone1, force_payload2drone2,
                self.payload_gravity, self.drone_gravity, pid)
            ########################################################

            if self.payload_position[2] > 0.0:
                multiarray0.data = [force_logger0, force_logger1, force_logger2, force_logger3,
                                    count4]

            else:
                multiarray0.data = [0, 0, 0, 0, count4]

            # 发布力的话题
            # time.sleep(0.002)
            self.force_publisher0.publish(multiarray0)
            count4 += 1

            count1 += 1
            self.force_publisher4.publish(multiarray4)
            # time.sleep(0.002)
            count2 += 1
            self.force_publisher5.publish(multiarray5)

            running.data = True
            self.run_cmd_pub.publish(running)

            # self.unpausePhysics()

            # # 画图数据
            self.dataPosx_payload.append(self.payload_position[0])
            self.dataPosy_payload.append(self.payload_position[1])
            self.dataPosz_payload.append(self.payload_position[2])
            self.dataPosz_drone1.append(self.drone1_position_mav[2])
            self.dataPosz_drone2.append(self.drone2_position_mav[2])
            # self.dataPosz_drone1.append(self.drone1_position[2])
            # self.dataPosz_drone2.append(self.drone2_position[2])
            self.data_droneforce1.append(force_payload2drone1)
            self.data_droneforce2.append(force_payload2drone2)
            self.data_force1.append(force_logger0)
            self.data_force2.append(force_logger1)
            self.data_force3.append(force_logger2)
            self.data_force4.append(force_logger3)

            self.timestep.append(count_draw)
            self.save_data()

            if count_draw == 1100:
                plt.figure(1)
                plt.title('force')
                plt.plot(self.data_force1)
                plt.plot(self.data_force2)
                plt.plot(self.data_force3)
                plt.plot(self.data_force4)
                plt.plot(self.data_droneforce1)
                plt.plot(self.data_droneforce2)
                plt.legend(labels=['logger0', 'logger1', 'logger2', 'logger3', 'drone1', 'drone2'],
                           loc='best')
                plt.savefig(
                    '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/figures/force_2dBA.png')
                count_draw = 1
                plt.figure(2)
                plt.title('PayloadPos_multi')
                plt.plot(self.dataPosx_payload)
                plt.plot(self.dataPosy_payload)
                plt.plot(self.dataPosz_payload)
                plt.axhline(y=target[2], color='r', linestyle='-')
                plt.axhline(y=target[0], color='r', linestyle='-')
                plt.legend(labels=['pos_x', 'pos_y', 'pos_z'], loc='best')
                plt.savefig(
                    '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/figures/PayloadPos_2dBA.png')
                plt.figure(3)
                plt.title('drone_pos')
                plt.plot(self.dataPosz_drone1)
                plt.plot(self.dataPosz_drone2)
                plt.axhline(y=self.altitude, color='r', linestyle='-')
                plt.savefig(
                    '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/figures/DronePos_2dBA.png')

                plt.figure(4)
                plt.title('dt')
                plt.plot(self.dt_draw)
                plt.axhline(y=0.1, color='r', linestyle='-')
                # plt.savefig('/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/figures/dt_2dBA.png')

                # plt.show()
                running.data = False
                self.run_cmd_pub.publish(running)
                break

            r.sleep()
            self.dt = time.time() - self.start
            self.dt_draw.append(self.dt)
            print(time.time() - self.start)
            count_draw += 1
            self.start = time.time()
        targetForce = [np.linalg.norm(self.target_loggerAll[i])
                       for i in range(count_draw-100, count_draw)]
        f_sum1 = np.sum(self.cable_drone1[1000:1100],0)/100
        f_sum2 = np.sum(self.cable_drone2[1000:1100], 0)/100
        thrust1 = np.sum(self.thrust_drone1[1000:1100], 0)/100
        thrust2 = np.sum(self.thrust_drone2[1000:1100], 0)/100
        # thrust1 = np.sum(np.array(self.cable_drone1[1000:1100])+np.array([0,0,18.88]), 0)/100
        # thrust2 = np.sum(np.array(self.cable_drone2[1000:1100])+np.array([0,0,18.88]), 0)/100
        norm_thrust = ((np.linalg.norm(thrust1))**1.5+(np.linalg.norm(thrust2))**1.5)*4.328
        print('target_force_vector',np.array(f_sum2)+np.array(f_sum1))
        print('target', np.mean(targetForce))
        print('norm',norm_thrust)
        sio.savemat(self.saveData_dir + '/2droneBAData.mat',
                    dict(timestep=self.timestep,
                         state_drone1=self.state_drone1, state_drone2=self.state_drone2,
                         state_payload=self.state_payload, force_cable1=self.cable_drone1,
                         force_cable2=self.cable_drone2, thrust_drone1=self.thrust_drone1,
                         thrust_drone2=self.thrust_drone2, force_logger0=self.force_logger0,
                         force_logger1=self.force_logger1, force_logger2=self.force_logger2,
                         force_logger3=self.force_logger3, force_loggerAll=self.target_loggerAll,
                         pwm2thrust1=self.pwm2thrust1, pwm2thrust2=self.pwm2thrust2))
        self.set_mode_client1(custom_mode="RTL")
        self.set_mode_client2(custom_mode="RTL")
        self.resetPayload()

    def compute_LoggerForce(self, cableForce1, cableForce2, payload_gravity, drone_gravity, pid):
        '''

        :param throttle1: 无人机1推力
        :param throttle2: 无人机2推力
        :param throttle3: 无人机3推力
        :param payload_gravity: 负载重力
        :param pid: pid控制器
        :return: 返回小车拉力大小
        '''
        model_name = ['logger0', 'logger1', 'logger2',
                      'logger3', 'drone1', 'drone2', 'drone3', 'payload']
        [self.payload_position, _, self.payload_linear,
            _] = self.get_state(self.model_states, model_name[7])
        [self.logger0_position, _, _, _] = self.get_state(
            self.model_states, model_name[0])
        [self.logger1_position, _, _, _] = self.get_state(
            self.model_states, model_name[1])
        [self.logger2_position, _, _, _] = self.get_state(
            self.model_states, model_name[2])
        [self.logger3_position, _, _, _] = self.get_state(
            self.model_states, model_name[3])
        # 使用mavros获取无人机位置信息
        # [self.drone1_position, self.drone2_position, ori_drone1, ori_drone2] = self.get_posByMavros()
        [self.drone1_position_mav, self.drone2_position_mav,
            ori_drone1, ori_drone2] = self.get_posByMavros()
        [self.drone1_position, ori_drone1, _, _] = self.get_state(
            self.model_states, model_name[4])
        [self.drone2_position, ori_drone2, _, _] = self.get_state(
            self.model_states, model_name[5])
        self.drone1_position[2]=self.altitude-(self.target_z-self.payload_position[2])
        self.drone2_position[2]=self.altitude-(self.target_z-self.payload_position[2])
        # 负载与小车间的力的方向,指向小车
        delta_load2logger0 = [self.logger0_position[0] - self.payload_position[0],
                              self.logger0_position[1] -
                              self.payload_position[1],
                              self.logger0_position[2] - (self.payload_position[2])]
        delta_load2logger1 = [self.logger1_position[0] - self.payload_position[0],
                              self.logger1_position[1] -
                              self.payload_position[1],
                              self.logger1_position[2] - (self.payload_position[2])]
        delta_load2logger2 = [self.logger2_position[0] - self.payload_position[0],
                              self.logger2_position[1] -
                              self.payload_position[1],
                              self.logger2_position[2] - (self.payload_position[2])]
        delta_load2logger3 = [self.logger3_position[0] - self.payload_position[0],
                              self.logger3_position[1] -
                              self.payload_position[1],
                              self.logger3_position[2] - (self.payload_position[2])]

        # 皈依化
        delta_load2logger0 = delta_load2logger0 / \
            np.sqrt(np.sum(np.square(delta_load2logger0)))
        delta_load2logger1 = delta_load2logger1 / \
            np.sqrt(np.sum(np.square(delta_load2logger1)))
        delta_load2logger2 = delta_load2logger2 / \
            np.sqrt(np.sum(np.square(delta_load2logger2)))
        delta_load2logger3 = delta_load2logger3 / \
            np.sqrt(np.sum(np.square(delta_load2logger3)))
        # print('guiyihua',np.sqrt(np.sum(np.square(delta_load2logger0))))

        # 绳子方向,指向负载
        delta_load2drone1 = [self.payload_position[0] - self.drone1_position[0],
                             self.payload_position[1] -
                             self.drone1_position[1],
                             self.payload_position[2] - self.drone1_position[2]]
        delta_load2drone2 = [self.payload_position[0] - self.drone2_position[0],
                             self.payload_position[1] -
                             self.drone2_position[1],
                             self.payload_position[2] - self.drone2_position[2]]

        # 距离
        self.dist_load2drone1 = np.sqrt(np.sum(np.square(delta_load2drone1)))
        self.dist_load2drone2 = np.sqrt(np.sum(np.square(delta_load2drone2)))

        # 归一化
        delta_load2drone1 = delta_load2drone1 / self.dist_load2drone1
        delta_load2drone2 = delta_load2drone2 / self.dist_load2drone2

        # 绳子方向上负载理论上受到 的绳子拉力(方向+大小)
        force_cableDrone1 = -np.abs(cableForce1) * delta_load2drone1
        force_cableDrone2 = -np.abs(cableForce2) * delta_load2drone2

        self.force_cable1 = np.abs(cableForce1)
        self.force_cable2 = np.abs(cableForce2)
        # print('force_cable',self.force_cable1,self.force_cable2,self.force_cable3)
        # 姿态的旋转矩阵zyx
        matrix1 = self.eulerAnglesToRotationMatrix(ori_drone1)
        matrix2 = self.eulerAnglesToRotationMatrix(ori_drone2)
        # print('mat',matrix1)
        # 推力向量
        # [thrust_vector1, thrust_vector2] = self.get_thrust_vector()
        # thrust_drone1 = thrust_vector1*self.thrust1
        # thrust_drone2 = thrust_vector2*self.thrust2
        # print('1', thrust_drone1)
        # print('2', thrust_drone2)
        thrust_drone1 = np.array(
            [matrix1[0][2], matrix1[1][2], matrix1[2][2]])*self.thrust1
        thrust_drone2 = np.array(
            [matrix2[0][2], matrix2[1][2], matrix2[2][2]])*self.thrust2
        # print('3', thrust_drone1)
        # print('4', thrust_drone2)
        # theta_cableThrottle1 = np.arccos(np.sum(
        #     [a*b for a, b in zip(thrust_drone1, delta_load2drone1)])/(np.sqrt(np.sum(np.square(thrust_drone1)))*1))
        # theta_cableThrottle2 = np.arccos(np.sum(
        #     [a*b for a, b in zip(thrust_drone2, delta_load2drone2)])/(np.sqrt(np.sum(np.square(thrust_drone2)))*1))
        # theta_cableThrottle3 = np.arccos(np.sum([a*b for a,b in zip(thrust_drone3,delta_load2drone3)])/(np.sqrt(np.sum(np.square(thrust_drone3)))*1))
        # print('l', theta_cableThrottle1)
        # print('l', theta_cableThrottle2)
        real_cable_force1 = thrust_drone1 + [0, 0, -drone_gravity]
        real_cable_force2 = thrust_drone2 + [0, 0, -drone_gravity]
        print('real_cable_force1', real_cable_force1)
        print('real_cable_force2', real_cable_force2)
        # pid
        force_Xerror, force_Yerror, force_Zerror = pid.cal_actions(
            self.payload_position)
        force_pid = [force_Xerror, force_Yerror, force_Zerror]
        # 目标小车合力
        # force_pid = [0,0,-1]
        # 多架无人机
        payload_gravityDir = [0, 0, -payload_gravity]
        target_forceLogger0123 = force_pid - force_cableDrone1 - \
            force_cableDrone2 - payload_gravityDir
        # if np.linalg.norm(target_forceLogger0123)<2:
        #     print('payloadposnow',self.payload_position[2])
        from optimizer import minimizeForce
        allArgs = (target_forceLogger0123, delta_load2logger0,
                   delta_load2logger1, delta_load2logger2, delta_load2logger3)
        x, result = minimizeForce(allArgs)

        force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3 = x[
            0], x[1], x[2], x[3]
        # value = force_valueLogger0 * delta_load2logger0 + force_valueLogger1 * delta_load2logger1 + force_valueLogger2 * delta_load2logger2 + force_valueLogger3 * delta_load2logger3 - target_forceLogger0123
        # if self.payload_linear[2]<0:
        print('===========================')
        # print('value', value)
        # print('force_cable', self.force_cable1, self.force_cable2)
        print('force_pid', force_pid)
        print('force_cabledrone1', force_cableDrone1)
        print('force_cabledrone2', force_cableDrone2)
        # print('force_cabledrone3', force_cableDrone3)
        print('payload_position', self.payload_position)
        # print('target_forceLogger0123', target_forceLogger0123)
        # print('thrust_drone1', thrust_drone1)
        # print('thrust_drone2', thrust_drone2)
        # print('carforce', force_valueLogger0, force_valueLogger1,
        #       force_valueLogger2, force_valueLogger3)
        # print('velocity',self.payload_linear[2])
        # print('***************************')
        # print('distance', self.dist_load2drone1, self.dist_load2drone2)
        # print('drone1', self.drone1_position[2])
        # print('drone2', self.drone2_position[2])
        # print('+++++++++++++++++++++++++++++')
        # 保存力的数据 向量
        self.cable_drone1.append(force_cableDrone1)  # 绳子拉力
        self.cable_drone2.append(force_cableDrone2)
        self.thrust_drone1.append(thrust_drone1)  # 无人机推力
        self.thrust_drone2.append(thrust_drone2)
        self.force_logger0.append(
            force_valueLogger0 * delta_load2logger0)  # 小车拉力
        self.force_logger1.append(force_valueLogger1 * delta_load2logger1)
        self.force_logger2.append(force_valueLogger2 * delta_load2logger2)
        self.force_logger3.append(force_valueLogger3 * delta_load2logger3)
        self.target_loggerAll.append(target_forceLogger0123)  # 小车拉力合力

        return force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3
    
    def get_thrust_vector(self):
        quat_drone1 =np.array( [self.local_position1.pose.orientation.x, self.local_position1.pose.orientation.y,
                       self.local_position1.pose.orientation.z, self.local_position1.pose.orientation.w])
        quat_drone2 = np.array([self.local_position2.pose.orientation.x, self.local_position1.pose.orientation.y,
                       self.local_position2.pose.orientation.z, self.local_position2.pose.orientation.w])
        thrust_origin_vector = np.array([0,0,1,0]) # 四元数形式，推力初始向上
        thrust_quat_vector1  = tf.transformations.quaternion_multiply(quat_drone1, tf.transformations.quaternion_multiply(thrust_origin_vector, tf.transformations.quaternion_inverse(quat_drone1)))
        thrust_quat_vector2  = tf.transformations.quaternion_multiply(quat_drone2, tf.transformations.quaternion_multiply(thrust_origin_vector, tf.transformations.quaternion_inverse(quat_drone2)))
        thrust_vector1 = np.delete(thrust_quat_vector1, 3)
        thrust_vector2 = np.delete(thrust_quat_vector2, 3)
        # print(np.linalg.norm(thrust_vector2))
        return [thrust_vector1, thrust_vector2]

    def get_posByMavros(self):
        '''
        使用mavros话题获取无人机状态
        :return: 返回无人机的位置
        '''
        pos_drone1 = [self.local_position1.pose.position.x-8.,
                      self.local_position1.pose.position.y, self.local_position1.pose.position.z]
        pos_drone2 = [self.local_position2.pose.position.x+8.,
                      self.local_position2.pose.position.y, self.local_position2.pose.position.z]

        ori_drone1 = self.quaternion_to_euler(self.local_position1.pose.orientation.x, self.local_position1.pose.orientation.y,
                                              self.local_position1.pose.orientation.z, self.local_position1.pose.orientation.w)
        ori_drone2 = self.quaternion_to_euler(self.local_position2.pose.orientation.x, self.local_position2.pose.orientation.y,
                                              self.local_position2.pose.orientation.z, self.local_position2.pose.orientation.w)

        return [pos_drone1, pos_drone2, ori_drone1, ori_drone2]

    def get_state(self, model_state, model_name):
        model = model_state.name.index(model_name)  #

        model_pose = model_state.pose[model]
        model_twist = model_state.twist[model]
        model_position = [model_pose.position.x,
                          model_pose.position.y, model_pose.position.z]
        roll, pitch, yaw = self.quaternion_to_euler(
            model_pose.orientation.x, model_pose.orientation.y, model_pose.orientation.z, model_pose.orientation.w)
        model_attitude = [roll, pitch, yaw]
        model_linear = [model_twist.linear.x,
                        model_twist.linear.y, model_twist.linear.z]
        model_angular = [model_twist.angular.x,
                         model_twist.angular.y, model_twist.angular.z]
        # print([model_position,model_orientation,model_linear,model_angular])
        # 位置，姿态，线速度，角速度
        return [model_position, model_attitude, model_linear, model_angular]
        # 位置与姿态
        # return [model_position,model_attitude]

    def save_data(self):
        models_name = ['logger0', 'logger1', 'logger2',
                       'logger3', 'drone1', 'drone2', 'payload']
        [self.drone1_position_mav, self.drone2_position_mav,ori_drone1, ori_drone2] = self.get_posByMavros()

        for model_name in models_name:
            model_state = self.get_state(self.model_states, model_name)
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
                model_state[0] = self.drone1_position_mav
                self.state_drone1.append(model_state)
                # print(model_state[0][2])

            if model_name == 'drone2':
                model_state[0] = self.drone2_position_mav
                self.state_drone2.append(model_state)

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

    def eulerAnglesToRotationMatrix(self, theta):

        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])

        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def lv_cb1(self, data):
        self.local_velocity1 = data
        # print(self.local_velocity1.twist.linear.x)
        # print(self.local_velocity1.twist.linear.y)
        # print(self.local_velocity1.twist.linear.z)

    def lv_cb2(self, data):
        self.local_velocity2 = data

    def lv_cb3(self, data):
        self.local_velocity3 = data

    def lp_cb1(self, data):
        self.local_position1 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)

    def lp_cb2(self, data):
        self.local_position2 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)

    def lp_cb3(self, data):
        self.local_position3 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)

    def _model_states_cb(self, data):
        self.model_states = data

    def rc_cb1(self, data):
        self.rc_states1 = data

    def rc_cb2(self, data):
        self.rc_states2 = data

    def rc_cb3(self, data):
        self.rc_states3 = data

    def rcIn_cb1(self, data):
        self.rcIn_states1 = data

    def rcIn_cb2(self, data):
        self.rcIn_states2 = data

    def rcIn_cb3(self, data):
        self.rcIn_states3 = data


class apmPID:
    def __init__(self, target):


        x_p,x_i,x_d = 1,1,0.5
        y_p,y_i,y_d = 1,1,0.5
        z_p,z_i,z_d = 1,1,0.5
        x, y, z = 5, 5, 30
        self.control_x = PID(np.asarray( [0.5*x_p, 0.0002*x_i, 2.0*x_d])*x, target[0], upper=250, lower=-250)  # control position x
        self.control_y = PID(np.asarray( [0.03*y_p, 0.0002*y_i, 0.3*y_d])*y, target[1], upper=250, lower=-250)  # control position y
        self.control_z = PID(np.asarray([0.03*z_p, 0.0002*z_i, 0.3*z_d])*z, target[2], upper=100, lower=-100)  # control position z


    def cal_actions(self, state):
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
    # print(np.arccos(np.sqrt(2)/2))
    # print(np.cos(np.arccos(np.sqrt(2)/2)))
    # print(np.cos(45))
    env = CDPR()

    env.run_last()
