#!/bin/bash

ulimit -r 99
ethercatctl start
source /home/rnd/hamal_ws/devel/setup.bash

export ROS_HOSTNAME=$(hostname)
export ROS_MASTER=$($(hostname -I) | cut -d " " -f1)
export ROS_HOME=${ROS_HOME:=$(echo ~root)/.ros}

which setpriv > /dev/null
if ["$?" != "0"];then
    exit 1
fi

setpriv --reuid root --regid root --init-groups roslaunch 