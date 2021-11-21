#!/usr/bin/env bash

source /home/ubuntu/catkin_ws/devel/setup.bash

export ROS_HOSTNAME=ubuntu
export ROS_MASTER_URI=http://wehak-xps:11311

exec "$@"