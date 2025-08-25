#!/usr/bin/env bash
source ~/.bashrc
cd ~/git/rel_ros/
python rel_ros_master_control/rel_ros_master_control/rest/app.py > /dev/null 2>&1 &
