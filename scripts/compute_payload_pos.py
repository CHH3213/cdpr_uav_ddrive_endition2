import numpy as np
import copy
from scipy.optimize import minimize


def compute_pos(pos_drones, gravity_payload, target_cables_force, Coeff_elasticity):
    """
    通过无人机的绳子合力(带方向)结算出质点的目标位置
    payload_gravityDir = [0, 0, -gravity_payload]
    """
    dir_gravity_payload = [0, 0, -gravity_payload]
    target_force = target_cables_force+dir_gravity_payload

    def fun(pos_payload):
        # 绳拉力向量,指向负载
        load2drones_force = []
        for pos_drone in pos_drones:
            dir_load2drone = np.array(pos_drone)-np.array(pos_payload)
            dist_load2drone = np.linalg.norm(copy.deepcopy(dir_load2drone))
            magnitude_cable_force = Coeff_elasticity*(dist_load2drone-1-4)
            norm_dir_load2drone = copy.deepcopy(dir_load2drone)/dist_load2drone
            load2drones_force.append(norm_dir_load2drone*magnitude_cable_force)
        composite_force = np.sum(load2drones_force,0)
        v = np.linalg.norm(np.array(target_force)-np.array(composite_force))#+np.sum(np.dot(np.array(target_force), np.array(composite_force)))
        return v
    return fun


def compute_pos2(pos_drones, gravity_payload, pos_payload, target_cables_force, Coeff_elasticity):
    """
    测试直接给定的负载位置,得到的拉力大小
    payload_gravityDir = [0, 0, -gravity_payload]
    """
    dir_gravity_payload = [0, 0, -gravity_payload]
    target_force = target_cables_force+dir_gravity_payload

    # 绳拉力向量,指向负载
    load2drones_force = []
    for pos_drone in pos_drones:
        dir_load2drone = np.array(pos_drone)-np.array(pos_payload)
        dist_load2drone = np.linalg.norm(copy.deepcopy(dir_load2drone))
        print("==========================")
        print("绳子长度:",dist_load2drone)
        magnitude_cable_force = Coeff_elasticity*(dist_load2drone-1-4)
        print("绳子拉力大小：", magnitude_cable_force)
        print("==========================")
        norm_dir_load2drone = copy.deepcopy(dir_load2drone)/dist_load2drone
        # print(norm_dir_load2drone)
        # print(np.linalg.norm(norm_dir_load2drone))
        load2drones_force.append(norm_dir_load2drone*magnitude_cable_force)
        print('绳子拉力向量', norm_dir_load2drone*magnitude_cable_force)

    composite_force = np.sum(load2drones_force,0)
    v = np.array(target_force)-np.array(composite_force)
    print(np.linalg.norm(v))
    print(v)
    
def con():
    cons = (
            {'type': 'ineq', 'fun': lambda x: x[2] - 0},
            {'type': 'ineq', 'fun': lambda x: -x[2] + 9.},
            )
    return cons


def minimizeForce(pos_drones, gravity_payload, target_cables_force, Coeff_elasticity):
    cons = con()
    x0=np.array([0,0,3])
    res = minimize(compute_pos(pos_drones, gravity_payload, target_cables_force,
                   Coeff_elasticity), x0, method="SLSQP", constraints=cons, options={'disp':True})
    pos = res.x
    result = res.fun
    print("负载位置：", pos)
    print('result', result)
    print(res.success)
    return pos



if __name__ == '__main__':
    # l = [np.array([1,2,3]),np.array([2,2,2]),np.array([3,3,3])]
    # print(l+np.array([0,0,1]))
    """通过调整无人机位置来使得解出来的负载位置固定在目标位置上,如这里的目标位置是【0,0,3】"""
    # pos_drones = np.array([[-8,0,20.03],[8,0,20.03]]) # 大间距
    pos_drones = np.array([[-0.5,0,20.5],[0.5,0,20.5]]) # 小间距
    # pos_drones = np.array([[0.0,0.5774,16.33],[-0.5,-0.2887,16.33],[0.5,-0.2887,16.33]]) # 3uavs小间距

    gravity_payload = 0  #2.45  赋为0，不需要，合力是所有小车的合力
    target_cables_force = np.array([0,0,50])
    Coeff_elasticity = 2
    pos = minimizeForce(pos_drones, gravity_payload, target_cables_force, Coeff_elasticity)

    # pos_payload = np.array([0,0,3])
    pos_payload = np.array(pos)
    compute_pos2(pos_drones, gravity_payload, pos_payload, target_cables_force, Coeff_elasticity)
