#!/usr/bin/env bash
source ~/.bashrc
RELANT_PACKAGE="rel_ros_hmi"
RELANT_NODE="rel_ros_hmi_node"
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
echo "current python path $PYTHONPATH"
rs=$(echo $PYTHONPATH | grep "ros2_ws/src/${RELANT_PACKAGE}" | wc -l)
if [[ $rs -lt 1 ]];
then
  echo "📦 adding package to python path ..."
  export PYTHONPATH=$PYTHONPATH:/home/relant/ros2_ws/venv/lib/python3.10/site-packages
  export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/${RELANT_PACKAGE}"
fi
echo "running node ✨"
ros2 run $RELANT_PACKAGE $RELANT_NODE
