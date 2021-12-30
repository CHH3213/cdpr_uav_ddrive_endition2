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
        self.local_velocity1 = TwistStamped()

        self.global_velocity = TwistStamped()
        self.local_position1 = PoseStamped()

        self.model_states = ModelStates()
        self.rc_states1 = RCOut()

        self.rcIn_states1 = RCIn()




        # # 物理属性
        self.cable_length = 5
        self.payload_gravity = 2.5
        self.drone_gravity = 18.88
        self.Coeff_elasticity = 500
        self.state_logger0 = []
        self.state_logger1 = []

        self.state_drone1 = []

        self.state_payload = []
        self.dt = 0.05  ## 仿真频率



        self.saveData_dir = "/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/stateData"
        if not os.path.exists(self.saveData_dir):
            os.makedirs(self.saveData_dir)
        ### Initiate ROS node
        print('-- Connecting to mavros')
        rospy.init_node('gym_apm_mavros',anonymous=True)
        print ('connected')
        ## ROS Subscribers

        self.local_pos_sub1 = rospy.Subscriber("/drone1/mavros/local_position/pose", PoseStamped, self.lp_cb1, queue_size=100)


        self.rc_out1 = rospy.Subscriber("/drone1/mavros/rc/out", RCOut, self.rc_cb1, queue_size=100)

        self.rc_in1 = rospy.Subscriber("/drone1/mavros/rc/in", RCIn, self.rcIn_cb1, queue_size=100)



        ## ROS Publishers
        # self.mocap_pos_pub = rospy.Publisher("/drone1/mavros/mocap/pose",PoseStamped,queue_size=100)
        self.acutator_control_pub1 = rospy.Publisher("/drone1/mavros/actuator_control",ActuatorControl,queue_size=100)

        self.setpoint_raw_pub1 = rospy.Publisher("/drone1/mavros/setpoint_raw/attitude",AttitudeTarget,queue_size=100)




        self.local_pos_pub1 = rospy.Publisher("/drone1/mavros/setpoint_position/local",PoseStamped,queue_size=100) #ENU


        self.thrust_pub1 = rospy.Publisher("/drone1/mavros/setpoint_attitude/thrust",Thrust,queue_size=100)


        self.rc_override1 = rospy.Publisher("/drone1/mavros/rc/override", OverrideRCIn, queue_size=100)


        self.force_publisher0 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=100)
        self.force_publisher1 = rospy.Publisher('/logger1/testforce', Float32MultiArray, queue_size=100)
        self.force_publisher2 = rospy.Publisher('/logger2/testforce', Float32MultiArray, queue_size=100)
        self.force_publisher3 = rospy.Publisher('/logger3/testforce', Float32MultiArray, queue_size=100)
        self.force_publisher4 = rospy.Publisher('/drone1/testforce', Float32MultiArray, queue_size=100)


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


        rospy.wait_for_service('drone1/mavros/cmd/takeoff')
        self.takeoff_service1 = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)


        rospy.wait_for_service('drone1/mavros/set_mode')
        self.set_mode_client1 = rospy.ServiceProxy('drone1/mavros/set_mode', SetMode)





        id_logger0 = self.model_states.name.index("logger0")  #
        id_logger1 = self.model_states.name.index("logger1")  #
        id_logger2 = self.model_states.name.index("logger2")  #
        id_logger3 = self.model_states.name.index("logger3")  #
        id_payload = self.model_states.name.index("payload")  #
        id_drone1 = self.model_states.name.index("drone1")  #

        logger_pose0 = self.model_states.pose[id_logger0]
        logger_pose1 = self.model_states.pose[id_logger1]
        logger_pose2 = self.model_states.pose[id_logger2]
        logger_pose3 = self.model_states.pose[id_logger3]
        payload_pose = self.model_states.pose[id_payload]
        drone_pose1 = self.model_states.pose[id_drone1]

        # =====================初始位置=====================
        self.payload_pose = payload_pose
        self.drone_pose1=drone_pose1

        self.logger_pose0=logger_pose0
        # ==============================================




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



    def run_xyz(self):
        # logger
        multiarray0 = Float32MultiArray()
        multiarray1 = Float32MultiArray()
        multiarray2 = Float32MultiArray()
        multiarray3 = Float32MultiArray()
        # UAV
        multiarray4 = Float32MultiArray()


        target = [0,0,2.5]
        pid = apmPID(target)
        # vehicle = connect('127.0.0.1:14551', wait_ready=True)
        self.start = time.time()
        # r = rospy.Rate(20)
        count = 1
        while not rospy.is_shutdown():
            # self.unpausePhysics()
            self.offb_arm()
            # publish
            # Override1 = OverrideRCIn()
            # Override1.channels = [1800, 1700, 1800, 1800, 1500, 1600, 1500, 1500]
            # self.rc_override1.publish(Override1.channels)
            print('channels1', self.rc_states1.channels)

            # print('channelsRCIn1', self.rcIn_states1.channels)
            # print('channelsRCIn2', self.rcIn_states2.channels)
            # print('channelsRCIn3', self.rcIn_states3.channels)

            # self.pausePhysics()
            # drone1_fly为z周距离是否大于0
            dist_cone2drone1, drone1_fly = self.get_dist()
            print('distance',dist_cone2drone1)

            model_name = ['logger0', 'logger1', 'logger2', 'logger3', 'drone1', 'drone2', 'drone3', 'payload']
            [payload_position, _, _, _] = self.get_state(self.model_states, model_name[7])
            [drone1_position, _, _, _] = self.get_state(self.model_states,  model_name[4])

            print('drone1_position',drone1_position)


            # self.unpausePhysics()
            if np.abs(drone1_position[2]-7.5)>0.2:
                self.takeoff_service1(altitude=7.5)
            # self.takeoff_service2(altitude=7.5)
            # self.takeoff_service3(altitude=7.5)
            if np.abs(drone1_position[2]-7.5)<=0.2 :
                self.set_mode_client1(custom_mode="ALT_HOLD")
            # if np.abs(drone2_position[2]-7.5)<=0.2 :
            #     self.set_mode_client2(custom_mode="ALT_HOLD")
            # if np.abs(drone3_position[2]-7.5)<=0.2 :
            #     self.set_mode_client3(custom_mode="ALT_HOLD")

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
            force_logger0,force_logger1,force_logger2,force_logger3 = self.compute_force(22,self.payload_gravity,self.drone_gravity,pid)
            # self.unpausePhysics()

            # print('force_logger0',force_logger0)
            # print('force_logger1',force_logger1)
            # print('force_logger2',force_logger2)
            # print('force_logger3',force_logger3)

            self.Coeff_elasticity = 1.1
            if drone1_fly == True and dist_cone2drone1>=5:
                # if drone1_fly == True:
                #     multiarray4.data = [dist_cone2drone1, self.cable_length, self.Coeff_elasticity] # 距离，绳长，弹性系数
                # 重物与绳子间拉力
                # force_payload2cable = self.payload_gravity*50+self.Coeff_elasticity*(dist_cone2drone1-5)*50
                force_payload2cable = self.force_cable1
                self.force_cable1 = self.force_cable1+self.Coeff_elasticity*(dist_cone2drone1-5)* int(self.dt*1000+1)
                # self.force_cable1 = self.force_cable1+self.Coeff_elasticity*(dist_cone2drone1-5)* 50
                multiarray4.data = [self.force_cable1,force_payload2cable] # 拉力大小
                self.force_publisher4.publish(multiarray4)




            # self.unpausePhysics()
            #
            # if drone1_fly and payload_position[2]>0.5:
            #     multiarray0.data = [force_logger0*50]
            #     multiarray1.data = [force_logger1*50]
            #     multiarray2.data = [force_logger2*50]
            #     multiarray3.data = [force_logger3*50]
            #     # multiarray0.data = [500]
            #     # multiarray1.data = [500]
            #     # multiarray2.data = [500]
            #     # multiarray3.data = [500]
            #     self.force_publisher0.publish(multiarray0)
            #     self.force_publisher1.publish(multiarray1)
            #     self.force_publisher2.publish(multiarray2)
            #     self.force_publisher3.publish(multiarray3)

            # self.unpausePhysics()
            # r.sleep()
            self.dt = time.time()-self.start
            print(time.time()-self.start)
            # self.pausePhysics()
            # if dist_cone2drone1 < 2 or dist_cone2drone2 < 2 or dist_cone2drone3 < 2 or dist_cone2drone1 >10 or dist_cone2drone2 >10 or dist_cone2drone3 >10:
            #     self.pausePhysics()
            #     # self.resetWorld()
            #     self.resetSim()
            #     time.sleep(1)
            self.start = time.time()



    def compute_force(self,throttle1, payload_gravity,drone_gravity, pid):
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


        # 使用mavros获取无人机位置信息
        [drone1_position,ori_drone1] = self.get_posByMavros()

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

        #归一化
        delta_load2drone1 = delta_load2drone1/np.sqrt(np.sum(np.square(delta_load2drone1)))


        # 推力方向皈依化
        ori_drone1 = ori_drone1 / np.sqrt(np.sum(np.square(ori_drone1)))

        #绳子与重力方向夹角,向量积的形式计算
        theta_cableGravity1 = np.arccos(delta_load2drone1[2])

        # 绳子与推力方向夹角
        theta_cableThrottle1 = np.arccos(np.sum([a*b for a,b in zip(ori_drone1,delta_load2drone1)]))

        # 绳子方向上受的合力大小
        force_cable1 = drone_gravity*np.cos(theta_cableGravity1) - throttle1 *np.cos(theta_cableThrottle1)

        # 绳子方向上负载理论上受到 的绳子拉力(方向+大小)
        force_cableDrone1 = -np.abs(force_cable1)*delta_load2drone1

        # print('force_cabledrone1', force_cableDrone1)
        # publish
        self.force_cable1 = np.abs(force_cable1)* int(self.dt*1000+1)


        # self.force_cable1 = np.abs(force_cable1)* 50
        # self.force_cable2 = np.abs(force_cable2)* 50
        # self.force_cable3 = np.abs(force_cable3)* 50
        print('force_cable',self.force_cable1)
        # pid
        force_Xerror, force_Yerror, force_Zerror = pid.cal_actions(payload_position)
        force_pid = [force_Xerror,force_Yerror,force_Zerror]
        print('force_pid',force_pid)
        print('payload_position',payload_position)
        # 目标小车合力
        # force_pid = [0,0,0]


        # 一架无人机
        target_forceLogger0123 = force_pid -  force_cableDrone1 - [0,0,-payload_gravity]

        print('target_forceLogger0123', target_forceLogger0123)
        from optimizer import minimizeForce
        allArgs = (target_forceLogger0123,delta_load2logger0,delta_load2logger1,delta_load2logger2,delta_load2logger3)
        force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3 = minimizeForce(allArgs)
        value = force_valueLogger0 * delta_load2logger0 + force_valueLogger1 * delta_load2logger1 + force_valueLogger2 * delta_load2logger2 + force_valueLogger3 * delta_load2logger3 - target_forceLogger0123
        # print('value',value)
        print('carforce',force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3)
        return force_valueLogger0, force_valueLogger1, force_valueLogger2, force_valueLogger3





    def get_dist(self):
        id_logger0 = self.model_states.name.index("logger0")  #
        id_logger1 = self.model_states.name.index("logger1")  #
        id_logger2 = self.model_states.name.index("logger2")  #
        id_logger3 = self.model_states.name.index("logger3")  #
        id_payload = self.model_states.name.index("payload")  #
        id_drone1 = self.model_states.name.index("drone1")  #

        logger_pose0 = self.model_states.pose[id_logger0]
        logger_pose1 = self.model_states.pose[id_logger1]
        logger_pose2 = self.model_states.pose[id_logger2]
        logger_pose3 = self.model_states.pose[id_logger3]
        payload_pose = self.model_states.pose[id_payload]
        drone_pose1 = self.model_states.pose[id_drone1]


        # 使用amvros返回无人机位置
        drone_pose1 = self.local_position1.pose




        dist_cone2drone1 = np.sqrt(np.sum(np.square([payload_pose.position.x-drone_pose1.position.x,payload_pose.position.y-drone_pose1.position.y,payload_pose.position.z-drone_pose1.position.z])))
        return dist_cone2drone1, drone_pose1.position.z>0.05

    def get_posByMavros(self):
        '''
        使用mavros话题获取无人机状态
        :return: 返回无人机的位置
        '''
        pos_drone1 = [self.local_position1.pose.position.x,self.local_position1.pose.position.y,self.local_position1.pose.position.z]

        ori_drone1 = self.quaternion_to_euler(self.local_position1.pose.orientation.x,self.local_position1.pose.orientation.y,self.local_position1.pose.orientation.z,self.local_position1.pose.orientation.w)

        return [pos_drone1,ori_drone1]

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


    def lp_cb1(self,data):
        self.local_position1 = data
        # print('GPSpos',self.local_position.pose.position.x,self.local_position.pose.position.y,self.local_position.pose.position.z)




    def _model_states_cb(self,data):
        self.model_states  = data

    def rc_cb1(self,data):
        self.rc_states1 = data

    def rcIn_cb1(self,data):
        self.rcIn_states1 = data



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
        self.control_z = PID([0.1, 0.00, 0.01], target[2], upper=1, lower= -1)  # control position z




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

    env.run_xyz()

    #