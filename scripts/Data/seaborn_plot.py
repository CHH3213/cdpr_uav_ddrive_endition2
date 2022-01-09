# -*- coding: utf-8 -*-
"""
used on  21 July  7 10:29:23 2021
@user: chh
画奖励值曲线
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns;sns.set()
import scipy.io as sio
import os




if __name__ == "__main__":
    dataFile = '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/3UAV-3/3droneData_v8.mat'

    data = sio.loadmat(dataFile)
    print(data.keys())
    # plt.plot(data['state_drone1'][2])
    # plt.show()
    print(data['state_drone1'][0][2])
