    单位为标准国际单位
    红色小圆锥尺寸：半径0.05，高0.1，质量0.5
    无人机质量：底座1.5kg，IMU0.15kg，里程计0.15kg，单个螺旋桨0.025kg。总重：1.9kg
    小车：位置为 横纵坐标上5的位置，半径为0.25米
    世界中每个单元格的长度为1米
    仿真频率real_time_update_rate：1000
    绳子长度：drone1：5米；drone2：6米；drone3：7米
    小球：0.25kg 半径0.05

ddrobot.world:只有小车的环境

multi_drone.world:只有无人机的环境

multi_drone_turtlebot.world:turtlebot、无人机环境

multi_drone_logger4_BACKUP.world:小车、无人机、立方体、草坪的环境

multi_drone_logger4_backup_cone.world:小车、无人机、大圆锥、草坪的环境

multi_drone_logger4.world:小车、无人机、小圆锥、草坪的环境

multi_drone_logger5.world:小车、无人机、红色小圆锥、灰色地面的环境,无人机碰撞属性已改

multi_drone_logger6.world:小车、无人机、红色小球、灰色地面的环境，无人机碰撞属性已改

multi_drone_logger6_test.world:小车、无人机、红色小球、灰色地面的环境，无人机碰撞属性已改,第一架无人机位置初始在小球附近，用来验证一架无人机情况下的稳定程度

multi_drone_logger6_changePos.world:小车、无人机、红色小球、灰色地面的环境，无人机碰撞属性已改,所有无人机位置初始在小球附近

testPWM.world:简单环境，测试pwm，与test_gazeboSim.py,libforce_noiris_testPWM.so配套

test_hz_no_iris.world:简单环境，测试；频率，与test_hz.py,libforce_plugin_testPWM.so配套,只有小车，没有无人机

testLoggerOnetopic.world:小车话题共用一个，fly_multi_onetopic.py,libforce_plugin_loggerOneTopic.so配套, 3架无人机拉力分配平均

最新使用环境： multi_drone_logger6.world

实验测试环境：
    experiment3drone.world：3架无人机拉力分配平均
    experiment2dSmallAverage.world：两架无人机，小间距，拉力分配平均
    experiment2dBigAverage.world：两架无人机，大间距，拉力分配平均
    experiment2dNOTAverage.world：两架无人机，小间距，拉力分配不平均
    
    