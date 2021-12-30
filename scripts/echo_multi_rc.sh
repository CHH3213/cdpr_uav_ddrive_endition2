#!/bin/bash
gnome-terminal --tab --title="1-rc-in" -x bash -c "rostopic echo /drone1/mavros/rc/in;exec bash;"
gnome-terminal --tab --title="1-rc-out" -x bash -c "rostopic echo /drone1/mavros/rc/out;exec bash;"
gnome-terminal --tab --title="2-rc-in" -x bash -c "rostopic echo /drone2/mavros/rc/in;exec bash;"
gnome-terminal --tab --title="2-rc-out" -x bash -c "rostopic echo /drone2/mavros/rc/out;exec bash;"
gnome-terminal --tab --title="3-rc-in" -x bash -c "rostopic echo /drone3/mavros/rc/in;exec bash;"
gnome-terminal --tab --title="3-rc-out" -x bash -c "rostopic echo /drone3/mavros/rc/out;exec bash;"
