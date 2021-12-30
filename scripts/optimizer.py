import numpy as np
from scipy.optimize import minimize
from scipy.optimize import least_squares





def fun(args):
    target_forceLogger0123, force_dir0, force_dir1,force_dir2,force_dir3 = args
    target_forceLogger0123 = np.array(target_forceLogger0123)
    force_dir0 = np.array(force_dir0)
    force_dir1 = np.array(force_dir1)
    force_dir2 = np.array(force_dir2)
    force_dir3 = np.array(force_dir3)
    # 最小化模
    # v1 = lambda x: np.linalg.norm(x[0]*force_dir0 + x[1]*force_dir1 + x[2]*force_dir2 + x[3]*force_dir3 - target_forceLogger0123)
    # 最小化模与最大化余弦距离
    v1 = lambda x: np.linalg.norm(x[0]*force_dir0 + x[1]*force_dir1 + x[2]*force_dir2 + x[3]*force_dir3 - target_forceLogger0123)-((x[0]*force_dir0 + x[1]*force_dir1 + x[2]*force_dir2 + x[3]*force_dir3)[0]*target_forceLogger0123[0]+(x[0]*force_dir0 + x[1]*force_dir1 + x[2]*force_dir2 + x[3]*force_dir3)[1]*target_forceLogger0123[1]+(x[0]*force_dir0 + x[1]*force_dir1 + x[2]*force_dir2 + x[3]*force_dir3)[2]*target_forceLogger0123[2])/(np.linalg.norm((x[0]*force_dir0 + x[1]*force_dir1 + x[2]*force_dir2 + x[3]*force_dir3))*np.linalg.norm(target_forceLogger0123))
    return v1


def con1(args):
    vmin, vmax = args
    cons = ({'type': 'ineq', 'fun': lambda x: x[0] - vmin}, \
            {'type': 'ineq', 'fun': lambda x: x[1] - vmin}, \
            {'type': 'ineq', 'fun': lambda x: x[2] - vmin}, \
            {'type': 'ineq', 'fun': lambda x: x[3] - vmin}, \
            {'type': 'ineq', 'fun': lambda x: -x[0] + vmax}, \
            {'type': 'ineq', 'fun': lambda x: -x[1] + vmax}, \
            {'type': 'ineq', 'fun': lambda x: -x[2] + vmax}, \
            {'type': 'ineq', 'fun': lambda x: -x[3] + vmax}, \
            )
    return cons


def minimizeForce(allArgs):
    # target_forceLogger0123, force_dir0, force_dir1,force_dir2,force_dir3 = allArgs
    # 力的下限是大于0,因为是小车的拉力，方向指向小车
    args = [0,5000]
    cons = con1(args)

    x0 = np.asarray((1, 1,1,1))
    res = minimize(fun(allArgs), x0, method='SLSQP', constraints=cons)
    v = res.x  # 返回最小化后的变量
    result = res.fun # 返回最小化后的结果
    print('result',result)
    # print(res.x)
    # print(res.success)
    return v,result

if __name__ == "__main__":
    args = (1, 2, 1, 1, 1)
    v = minimizeForce(args)
