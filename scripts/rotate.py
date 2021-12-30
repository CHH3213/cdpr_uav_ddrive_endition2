# -*-coding:utf-8-*-
# -*- coding: utf-8 -*-
"""
旋转矩阵
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns;sns.set()
import scipy.io as sio
import os
import math
saveData_dir = '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data'
if not os.path.exists(saveData_dir):
    os.makedirs(saveData_dir)
dataFile2BA = '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/2droneBAData_v3.mat'
dataFile2SA = '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/2droneSAData_v1.mat'
dataFile3 = '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/3droneData_v3.mat'


def rotate(theta):
    '''

    :param theta: o欧拉角
    :return:x*y*z*x^-1*y^-1*z^-1
    '''
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
    inv_R_x = np.linalg.inv(R_x)
    inv_R_y = np.linalg.inv(R_y)
    inv_R_z = np.linalg.inv(R_z)
    R = np.dot(np.dot(R_x, np.dot(R_y, R_z)),np.dot(inv_R_x,np.dot(inv_R_y,inv_R_z)))
    return R

def eulerAnglesToRotationMatrix(theta):

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
    R = np.dot(R_x, np.dot(R_y, R_z))
    return R


def get_data(dataFile):
    data = []
    data_raw = sio.loadmat(dataFile)
    # print(data_raw.keys())
    state_drone1 = data_raw.get('state_drone1')
    state_drone2 = data_raw.get('state_drone2')
    thrust_drone1 = data_raw.get('thrust_drone1')
    thrust_drone2 = data_raw.get('thrust_drone2')

    # print(np.shape(state_drone1))  # 1100*4*3
    # print(np.shape(thrust_drone1))  # 1100*3
    pwm2thrust1 = data_raw.get('pwm2thrust1')
    pwm2thrust2 = data_raw.get('pwm2thrust2')
    # print(np.shape(pwm2thrust1))  #1100*4
    euler_drone1 = state_drone1[:,1,:]
    euler_drone2 = state_drone2[:,1,:]
    # print(np.shape(euler_drone1)) #1100*3
    # print(euler_drone1)
    if 'state_drone3' in data_raw.keys():
        state_drone3 = data_raw.get('state_drone3')
        thrust_drone3 = data_raw.get('thrust_drone3')
        pwm2thrust3 = data_raw.get('pwm2thrust3')
        euler_drone3 = state_drone3[:, 1, :]
        return euler_drone1, euler_drone2, euler_drone3, thrust_drone1, thrust_drone2, thrust_drone3, pwm2thrust1, pwm2thrust2,pwm2thrust3

    return euler_drone1,euler_drone2,thrust_drone1,thrust_drone2,pwm2thrust1,pwm2thrust2

def convert2thrust1(euler_drone,pwm_thrust):
    '''
    直接使用四个螺旋桨推力合力来计算
    :param euler_drone:
    :param pwm_thrust:
    :return:
    '''
    thrust_convert = []
    for euler, thrust in zip(euler_drone, pwm_thrust):
        R = eulerAnglesToRotationMatrix(euler)
        thrust_all = np.sum(thrust)
        thrust_vector = np.array([0,0,thrust_all])
        thrust_true = np.dot(R, thrust_vector)
        thrust_convert.append(thrust_true)
    return thrust_convert
def convert2correctThrust(euler_drone, thrust_drone):
    '''
    通过已经计算好的推力向量反推出另一种变换形式的推力向量
    :param euler_drone:
    :param thrust_drone:
    :return:
    '''
    thrust_convert = []
    for euler,thrust in zip(euler_drone,thrust_drone):
        R = rotate(euler)
        thrust = np.array(thrust)
        thrust_true = np.dot(R,thrust)
        thrust_convert.append(thrust_true)
    return thrust_convert


def save(datafile,savename):
    if savename=='/Thrust_3drone.mat':
        euler_drone1, euler_drone2, euler_drone3, thrust_drone1, thrust_drone2, thrust_drone3, pwm2thrust1, pwm2thrust2,pwm2thrust3 = get_data(datafile)
        true_thrust_drone1 = convert2thrust1(euler_drone1, pwm2thrust1)
        true_thrust_drone2 = convert2thrust1(euler_drone2, pwm2thrust2)
        true_thrust_drone3 = convert2thrust1(euler_drone3, pwm2thrust3)
        print(np.shape(true_thrust_drone1))
        print(np.shape(true_thrust_drone2))
        print(np.shape(true_thrust_drone3))
        # print(true_thrust_drone1[1090:])
        # print(true_thrust_drone2[1090:])
        # print(true_thrust_drone3[1090:])
        # print(thrust_drone1[1090:])
        # print(thrust_drone2[1090:])
        # print(thrust_drone3[1090:])
        sio.savemat(saveData_dir + savename,
                    dict(true_thrust_drone1=true_thrust_drone1, true_thrust_drone2=true_thrust_drone2,true_thrust_drone3=true_thrust_drone3))

        # data = sio.loadmat(saveData_dir + '/Thrust_3drone.mat')
        # print(data.keys())
        # print(data.get('true_thrust_drone1'))
    else:
        euler_drone1, euler_drone2, thrust_drone1, thrust_drone2, pwm2thrust1, pwm2thrust2 = get_data(datafile)
        # true_thrust_drone1 = convert2correctThrust(euler_drone1,thrust_drone1)
        # true_thrust_drone2 = convert2correctThrust(euler_drone2,thrust_drone2)
        # 2
        true_thrust_drone1 = convert2thrust1(euler_drone1, pwm2thrust1)
        true_thrust_drone2 = convert2thrust1(euler_drone2, pwm2thrust2)
        # print(np.shape(true_thrust_drone1))
        # print(np.shape(true_thrust_drone2))
        # print(true_thrust_drone1)
        # print(true_thrust_drone2)
        # print(true_thrust_drone2[1099])
        # print(thrust_drone2[1099])
        sio.savemat(saveData_dir + savename,
                    dict(true_thrust_drone1=true_thrust_drone1, true_thrust_drone2=true_thrust_drone2))

        # data = sio.loadmat(saveData_dir + '/Thrust_2droneBA.mat')
        # print(data.get('true_thrust_drone1'))


if __name__ == "__main__":
    save(dataFile2BA,savename='/Thrust_2droneBA.mat')
    save(dataFile2SA,savename='/Thrust_2droneSA.mat')
    save(dataFile3,savename='/Thrust_3drone.mat')



