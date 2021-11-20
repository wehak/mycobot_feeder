#!/usr/bin/env bash

source /home/wehak/code/ACIT4280/catkin_ws/devel/setup.bash

export ROS_HOSTNAME=wehak-xps
export ROS_MASTER_URI=http://wehak-xps:11311

exec "$@"