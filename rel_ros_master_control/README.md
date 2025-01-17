# rel_ros_master_control
Relant master control on ROS

## Docker

>build
`docker build . -t rel-ros-master-control:0.1.0`

>run
`docker run --rm --name rel-ros -it rel-ros-master-control:0.1.0 /bin/bash`

>run with mounted volume
$ cd ~/vs-workspace/rel_ros_master_control
`docker run --rm --name rel-ros -it -v $(pwd):/home/relant/ros2_ws/src/rel_ros_master_control rel-ros:0.1.0 /bin/bash`

## Local Env Vars in .envrc
```bash
export LOGLEVEL="DEBUG"
export USE_TEST_MODBUS="true"
export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_master_control"
export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_hmi"
```

## ROS

### create package

```bash
cd ~/ros2_ws/src
ros2 pkg create  --build-type ament_python --license Apache-2.0 rel_ros_master_control
```

**build**

```bash
source ~/.bashrc
cd ~/ros2_ws
colcon build --symlink-install
```

**run**

```bash
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/relant/ros2_ws/venv/lib/python3.10/site-packages
ros2 run rel_ros_master_control rel_ros_master_control_node
```

## Run on local

> my local all
`export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_master_control"`


> my local
`export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros_master_control"`

> RPi4
`export PYTHONPATH="${PYTHONPATH}:/home/relant/git/rel_ros_master_control"`



for ROS image
<mark>This is important to execute before running (only once)</mark>
`export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_master_control"`

# download IOlink IODD Viewer


## Environment Config
Testing with DEVICE_PORTS=5,8

Master IOLink
```bash
export MASTER_IO_LINK_IP=0.0.0.0
export MASTER_IO_LINK_PORT=8844
```

## Static IP config in Ubuntu

`nano /etc/netplan/99_config.yaml`

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.0.10/24
```
The run
`sudo netplan apply`

## CONTROL test examples
port 5 => tower, octect 1 5051
port 8 => sensor, 8000 status 8001 => data => 8002 - 8017

```bash
# READ
python rel_ros_master_control/rel_ros_master_control/control.py --action read --register 8001

# WRITE
python rel_ros_master_control/rel_ros_master_control/control.py --action write --register 5051 --value 1`

python rel_ros_master_control/rel_ros_master_control/control.py --action write --register 5052 --value 1`
```
