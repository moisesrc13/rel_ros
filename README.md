# rel_ros
Relant on ROS2


## Docker

>build
`docker build . -t rel-ros:0.1.0`

>run
`docker run --rm --name rel-ros -it rel-ros:0.1.0 /bin/bash`

>run with mounted volumes for all nodes
```bash
cd ~/vs-workspace/rel_ros

docker run --rm --name rel-ros -it \
 -v $(pwd)/rel_ros_master_control:/home/relant/ros2_ws/src/rel_ros_master_control \
 -v $(pwd)/rel_ros_hmi:/home/relant/ros2_ws/src/rel_ros_hmi \
 -v $(pwd)/rel_interfaces:/home/relant/ros2_ws/src/rel_interfaces \
 rel-ros:0.1.0 /bin/bash
```

## Run on local

> my local all
```bash
export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_master_control"
export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_hmi"
```

> RPi4
```bash
export PYTHONPATH="${PYTHONPATH}:/home/relant/git/rel_ros/rel_ros_master_control"
export PYTHONPATH="${PYTHONPATH}:/home/relant/git/rel_ros/rel_ros_hmi"
```

> for ROS image
This is important to execute before running (only once)
```bash
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_hmi"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_master_control"
```

export the `rel_ros_hmi` in the path is required to run the master modbus test from hmi package

## Run nodes

This is how we can run all nodes

```bash
export PYTHONPATH=$PYTHONPATH:/home/relant/ros2_ws/venv/lib/python3.10/site-packages
source install/setup.bash
ros2 run rel_ros_master_control rel_ros_master_control_node
ros2 run rel_ros_hmi rel_ros_hmi_node
```

## Run test HMI modbus master & slave

```bash

python rel_ros_hmi/rel_ros_hmi/modbus_slave.py

cd ~/ros2_ws/src
python rel_ros_hmi/rel_ros_hmi/modbus_master.py --action write --register 40010 --value 1200

# read all
python rel_ros_hmi/rel_ros_hmi/modbus_master.py --action read --register 0
```
