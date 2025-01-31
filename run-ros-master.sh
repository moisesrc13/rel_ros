#!/usr/bin/env bash
source ~/.bashrc
RELANT_PACKAGE="rel_ros_master_control"
RELANT_NODE="rel_ros_master_control_node"
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "current python path $PYTHONPATH"
cp -r src/rel_ros_master_control/rel_ros_master_control/config /home/relant/ros2_ws/install/rel_ros_master_control/lib/python3.10/site-packages/rel_ros_master_control
rs=$(echo $PYTHONPATH | grep "ros2_ws/src/${RELANT_PACKAGE}" | wc -l)
if [[ $rs -lt 1 ]];
then
  echo "📦 adding package to python path ..."
  export PYTHONPATH=$PYTHONPATH:/home/relant/ros2_ws/venv/lib/python3.10/site-packages
  export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/${RELANT_PACKAGE}"
fi
echo "running node ✨"
ros2 run $RELANT_PACKAGE $RELANT_NODE
