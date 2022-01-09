# cdpr_uav_ddrive

> 2021/12/30

## Environments
- 整个环境是在`apm`飞控下运行成功的。所以在运行本环境之前，需要安装好mavros和apm，[Intelligent Quads Tutorials](https://github.com/Intelligent-Quads/iq_tutorials)
- 安装好后，将本功能包从仓库上git clone下来：
```bash
git clone https://github.com/CHH3213/cdpr_uav_ddrive_endition2.git
```
- 将功能包名字修改为`cdpr_uav_ddrive`：

```bash
cp cdpr_uav_ddrive_endition2 cdpr_uav_ddrive
```

- 将整个`cdpr_uav_ddrive`功能包放到ros工作空间中，并编译和source

  ```bash
  cd ~/ros_ws/
  catkin_make
  source devel/setup.sh
  ```

- 进入bash文件夹，给bash脚本添加权限

  ```bash
  chmod a+x start_multi_drone.sh
  ```


- 运行bash脚本，正常可以打开环境：

  ```
  ./start_multi_drone.sh
  ```

- 打开环境，正常打开后应该会有如下界面：

<img src="./worlds/fig/env3.png" alt="world" style="zoom:150%;" />





## 运行

在`scripts`文件夹下，主要的main函数有`experi_2dBA`,`experi_2dNOTA`,`experi_2dSA`,`experi_3drone`分别表示2架无人机大间距，两架无人机无间距，两架无人机小间距，3架无人机。需要分别打开对应的环境才可以运行。
- 首先运行`cmd_force.py`
- 而后运行相应的main函数

