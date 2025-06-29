#!/usr/bin/env bash
source ~/.bashrc
RELANT_PACKAGE="rel_ros_hmi"
RELANT_NODE="rel_ros_hmi_node"
cd ~/ros2_ws
source install/setup.bash
echo "running node ✨"
ros2 run $RELANT_PACKAGE $RELANT_NODE
