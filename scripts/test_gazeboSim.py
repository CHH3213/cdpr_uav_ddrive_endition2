# -*-coding:utf-8-*-
#!/usr/bin/env python
# -*- coding: UTF-8 -*-
'''
测试PWM以及测试频率问题
'''
from __future__ import print_function, absolute_import, division
import gym
import numpy
import time
import tf as rostf
import math
import datetime
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
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import RCOut

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



model_states = ModelStates()
def _model_states_cb(data):
    global model_states
    model_states = data

if __name__ == "__main__":


    rospy.init_node('apm_mavros', anonymous=True)
    cmd1 = rospy.Publisher('/logger1/cmd_vel', Twist, queue_size=10)
    cmd0 = rospy.Publisher('/logger0/cmd_vel', Twist, queue_size=10)
    cmd2 = rospy.Publisher('/logger2/cmd_vel', Twist, queue_size=10)
    cmd3 = rospy.Publisher('/logger3/cmd_vel', Twist, queue_size=10)
    multiarray0 = Float32MultiArray()
    multiarray1 = Float32MultiArray()
    multiarray2 = Float32MultiArray()
    multiarray3 = Float32MultiArray()
    multiarray4 = Float32MultiArray()
    multiarray = Float32MultiArray()
    multiarray0.data = [0]
    multiarray1.data = [0]
    multiarray2.data = [0]
    multiarray3.data = [0]

    force_publisher0 = rospy.Publisher('/logger0/testforce', Float32MultiArray, queue_size=10)
    force_publisher1 = rospy.Publisher('/logger1/testforce', Float32MultiArray, queue_size=10)
    force_publisher2 = rospy.Publisher('/logger2/testforce', Float32MultiArray, queue_size=10)
    force_publisher3 = rospy.Publisher('/logger3/testforce', Float32MultiArray, queue_size=10)
    force_publisher4 = rospy.Publisher('/drone1/testforce', Float32MultiArray, queue_size=10)
    force_publisher = rospy.Publisher('/loggerAll/testforce', Float32MultiArray, queue_size=10)

    r = rospy.Rate(100)  # 10hz
    # rospy.wait_for_service('drone1/mavros/cmd/arming')
    arming_client1 = rospy.ServiceProxy('drone1/mavros/cmd/arming', CommandBool)
    # rospy.wait_for_service('drone1/mavros/set_mode')
    set_mode_client1 = rospy.ServiceProxy('drone1/mavros/set_mode', SetMode)
    # rospy.wait_for_service('drone1/mavros/cmd/takeoff')
    takeoff_service1 = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)

    rospy.wait_for_service('drone2/mavros/cmd/arming')
    arming_client2 = rospy.ServiceProxy('drone2/mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('drone3/mavros/cmd/arming')
    arming_client3 = rospy.ServiceProxy('drone3/mavros/cmd/arming', CommandBool)
    rospy.wait_for_service('drone2/mavros/cmd/takeoff')
    takeoff_service2 = rospy.ServiceProxy('/drone2/mavros/cmd/takeoff', CommandTOL)
    rospy.wait_for_service('drone3/mavros/cmd/takeoff')
    takeoff_service3 = rospy.ServiceProxy('/drone3/mavros/cmd/takeoff', CommandTOL)


    rospy.wait_for_service('drone2/mavros/set_mode')
    set_mode_client2 = rospy.ServiceProxy('drone2/mavros/set_mode', SetMode)
    rospy.wait_for_service('drone3/mavros/set_mode')
    set_mode_client3 = rospy.ServiceProxy('drone3/mavros/set_mode', SetMode)


    rospy.Subscriber("/gazebo/model_states", ModelStates, _model_states_cb)
    start = time.time()
    # id_payload = model_states.name.index("payload")  #
    # logger_twist0 = model_states.twist[id_payload]

    # while time.time()-start<3:
    #     # vel.linear.x = np.random.uniform(2)
    #     # vel.linear.x = 0
    #
    #     force_publisher.publish(multiarray0)
    #     # force_publisher1.publish(multiarray1)
    #     # force_publisher2.publish(multiarray2)
    #     # force_publisher3.publish(multiarray3)
    #     # force_publisher4.publish(multiarray0)

def rc_cb1(data):
    rc_states1 = RCOut()
    rc_states1 = data
    print('PWM',rc_states1.channels)



# start = time.time()
# r.sleep()
# r = rospy.Rate(10)  # 10hz
count = 1

while True:
    start = time.time()
    multiarray0.data = [2.5,2.5] # [uav拉力，质点拉力]
    # 无人机起飞
    set_mode_client1(custom_mode="4")
    arming_client1(True)
    takeoff_service1(altitude=8)
    # rc_out1 = rospy.Subscriber("/drone1/mavros/rc/out", RCOut, rc_cb1, queue_size=1)

    # if count == 1:
    #     time.sleep(10)
    #     count +=1
    # count +=1
    START =time.time()
    force_publisher4.publish(multiarray0)
    # force_publisher.publish(multiarray0)
    # print(time.time()-START)
    # time.sleep(0.003)
    # force_publisher4.publish(multiarray0)
    # time.sleep(0.003)
    # force_publisher4.publish(multiarray0)
    # time.sleep(0.003)
    # force_publisher4.publish(multiarray0)
    # time.sleep(0.003)
    # force_publisher4.publish(multiarray0)

    for _ in range(10):
        force_publisher4.publish(multiarray0)
    # time.sleep(5)
    # if count == 500:
    #     set_mode_client1(custom_mode="RTL")
    #     break

    r.sleep()
    dt = time.time()-start
    # print(time.time()-start)
    # print('vel', logger_twist0.linear.z)




