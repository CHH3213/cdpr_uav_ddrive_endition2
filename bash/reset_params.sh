#!/bin/bash
gnome-terminal --tab  --title="reset_params" -x bash -c "rosrun mavros mavparam load /home/chh3213/catkin_ws/src/intelligent-quads/default_params/default_mavros.parm;exec bash;"
# 重置参数时可以将该文件夹下的所有参数文件删除即可，只保留.sh文件