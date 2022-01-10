## 程序功能解释
- `cmd_force.py`:从主程序中接收到力的话题，并且将数据发布给gazebo仿真环境，频率为1000hz
- `compute_payload_pos.py`:计算在给定无人机绳子合力的情况下，计算质点的目标位置
- `experi_2dBA.py`:2架无人机大间距下的实验主程序
- `experi_2dSA.py`:2架无人机小间距下的实验主程序
- `experi_3drone.py`:3架无人机下的实验主程序

- `pidController.py` 和 `PIDClass.py`:PID控制器
- `pwm2thrust.py`：PWM与推力的转换
- `rotate.py`：旋转相关的函数
