#!/usr/bin/env bash

# just run a demo here
. /opt/ros/humble/setup.sh
. ~/colcon_venv/venv/bin/activate
ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker
