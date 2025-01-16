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
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/venv/lib/python3.10/site-packages"
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


## Run test modbus master control slave
```bash
export USE_TEST_MODBUS="true"
python rel_ros_master_control/rel_ros_master_control/modbus_slave.py
python rel_ros_master_control/rel_ros_master_control/rest/app.py
```


## Run test HMI modbus master & slave

### HMI

```bash
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_hmi" \
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_master_control" \
export USE_TEST_MODBUS="true"

python ~/ros2_ws/src/rel_ros_hmi/rel_ros_hmi/modbus_slave.py

cd ~/ros2_ws/src
python rel_ros_hmi/rel_ros_hmi/modbus_master.py --action write --register 40010 --value 1200

# read all
python rel_ros_hmi/rel_ros_hmi/modbus_master.py --action read --register 0
```

### IOLink test slave

```bash

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/modbus_slave.py

cd ~/ros2_ws/src
python rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40010 --value 1200

# read all
python rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0
```

## Build interfaces package
Make sure you install `numpy` in the virtual env

## Sym links in RPi

```bash
ln -s /home/relant/git/rel_ros/rel_ros_hmi /home/relant/ros2_ws/src/rel_ros_hmi && \
ln -s /home/relant/git/rel_ros/rel_ros_master_control /home/relant/ros2_ws/src/rel_ros_master_control && \
ln -s /home/relant/git/rel_ros/rel_interfaces /home/relant/ros2_ws/src/rel_interfaces
```

## Config REST Service
```bash
sudo cp rel_ros_master_control/rel_ros_master_control/rest/rel-iolink-rest.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable rel-iolink-rest.service
sudo systemctl status rel-iolink-rest.service
sudo systemctl rel-iolink-rest.service
```

### calling service
Replace `localhost` by host IP


curl "http://localhost:9080/read/8001"

curl -X POST "http://localhost:9080/write" -H "Content-Type:application/json" -d '{"register": 5051, "value": 1}'
