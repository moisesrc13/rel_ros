#!/usr/bin/env bash
source ~/.bashrc
cd ~/ros2_ws
colcon build --packages-select rel_interfaces
source install/setup.bash
echo "------------------ HMI"
ros2 interface show rel_interfaces/msg/HMI
echo "------------------ IOLinkData"
ros2 interface show rel_interfaces/msg/IOLinkData
