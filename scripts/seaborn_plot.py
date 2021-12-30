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

dataFile = [
    '/home/chh3213/ros_wc/src/cdpr_uav_ddrive/scripts/Data/2droneBAData.mat'
    ]

def get_data(dataFile):
    data = []
    for file in dataFile:
        # print(str(file))
        data.append(sio.loadmat(file))
    # print(np.size(data))
    print(data)
    # rewards = []
    # for d in data:
    #     rewards.append(d['episode_reward'])
    # # print(np.size(rewards))
    # a_rewards = [] # 总共轮数
    # for i in rewards:
    #     accumlated_rewards = []  # 每一轮
    #     for j in i[0]:  # 总的轮数的数据
    #         for k in j:  # 一轮中的episode
    #             accumlated_reward = np.sum(k)  # 累加
    #             # accumlated_reward = np.mean(k)  # 求均值
    #             accumlated_rewards.append(accumlated_reward)
    #     accumlated_rewards = np.array(accumlated_rewards)
    #     a_rewards.append(accumlated_rewards)
    #     # print(a_rewards)
    # return a_rewards

def smooth(data, sm=1):  # sm表示滑动窗口大小,为2*k+1, smoothed_y[t] = average(y[t-k], y[t-k+1], ..., y[t+k-1], y[t+k])
    if sm > 1:
        smooth_data = []
        for d in data:
            y = np.ones(sm)*1.0/sm
            d = np.convolve(y, d, "same")

            smooth_data.append(d)

        return smooth_data
    return data


if __name__ == "__main__":
    reward_w = get_data(dataFile)
    # #
    # reward_w = smooth(reward_w,sm=2)
    #
    # rewards_final_w = np.vstack(reward_w) # 合并数据
    # df_w = pd.DataFrame(rewards_final_w).melt(var_name='episode', value_name='reward')  # 推荐这种转换方法
    # label = ['wind']
    # df=[df_w]
    # for i in range(len(df)):
    #     df[i]['']= label[i]
    # df = pd.concat(df) # 合并
    # plt.axhline(y=2300, color='r', linestyle='--')
    # fig = sns.lineplot(x="episode", y="reward", hue="", style="",data=df)
    # plt.legend(loc='lower right')
    # reward_fig = fig.get_figure()
    # reward_fig.savefig('reward.png',dpi=400)
    # # # print(accumlated_rewards)
    # plt.show()


