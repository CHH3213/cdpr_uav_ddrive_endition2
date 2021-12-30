import numpy as np
from scipy.optimize import minimize

def fun(args):
    target_forceLogger0123, force_dir0 = args
    target_forceLogger0123 = np.array(target_forceLogger0123)
    force_dir0 = np.array(force_dir0)

    v1 = lambda x: np.linalg.norm(x[0]*force_dir0  - target_forceLogger0123)
    return v1


def con1(args):
    vmin = args
    cons = ({'type': 'ineq', 'fun': lambda x: x[0] - vmin})
    return cons



def minimizeForce(allArgs):
    # target_forceLogger0123, force_dir0, force_dir1,force_dir2,force_dir3 = allArgs
    # 力的下限是大于0,因为是小车的拉力，方向指向小车
    args = 0
    cons = con1(args)

    x0 = np.asarray((5))
    res = minimize(fun(allArgs), x0, method='SLSQP', constraints=cons)
    v = res.x  # 返回最小化后的变量
    result = res.fun # 返回最小化后的结果
    # print('result',result)
    # print(res.x)
    # print(res.success)
    return v

if __name__ == "__main__":
    args = (1, 2)
    v = minimizeForce(args)
