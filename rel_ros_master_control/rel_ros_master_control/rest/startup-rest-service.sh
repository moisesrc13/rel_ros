#!/usr/bin/env bash

. /home/relant/.profile
cd /home/relant/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/

python rest/app.py > /dev/null 2>&1 &
