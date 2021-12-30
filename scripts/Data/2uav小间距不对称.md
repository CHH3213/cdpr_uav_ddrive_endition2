date：7/23 16：30

1. 所测实验为**两架无人机、小间距、拉力分配不平均**的情况。
2. 最新弹力模型
3. 弹性系数设为1.5，优化器没有上限约束，优化对象是模的大小+方向
4. 负载的目标位置设置为**[0,0,3]**,质量为0.25kg
5. 无人机1初始位置[0,-0.25],无人机2初始位置[0,0.45], 定高位置为7.5米
6. mat文件数据保存顺序

```python
dict(timestep = self.timestep,
     state_drone1=self.state_drone1, state_drone2=self.state_drone2,
     state_payload=self.state_payload, force_cable1=self.cable_drone1,
     force_cable2=self.cable_drone2, thrust_drone1 = self.thrust_drone1,
     thrust_drone2 = self.thrust_drone2, force_logger0=self.force_logger0,
     force_logger1=self.force_logger1, force_logger2=self.force_logger2,
     force_logger3=self.force_logger3,force_loggerAll = self.target_loggerAll,
     pwm2thrust1 = self.pwm2thrust1,pwm2thrust2=self.pwm2thrust2))
  state为状态量：包括：位置,欧拉角,线速度,角速度
  force为力向量。drone代表无人机，logger代表小车，cable表示绳子拉力，thrust为由PWM波转换过来的推力力向量，方向由欧拉角通过zyx的顺序得到。pwm2thrust是4个螺旋桨各自的推力大小。
  共保存了1100步的数据
```

