# rel_ros

Relant on ROS2

## Docker

> build
> `docker build . -t rel-ros:0.1.0`

> run
> `docker run --rm --name rel-ros -it rel-ros:0.1.0 /bin/bash`

> run with mounted volumes for all nodes

```bash
cd ~/vs-workspace/rel_ros

docker run --rm --name rel-ros -it \
 -v $(pwd)/rel_ros_master_control:/home/relant/ros2_ws/src/rel_ros_master_control \
 -v $(pwd)/rel_ros_hmi:/home/relant/ros2_ws/src/rel_ros_hmi \
 -v $(pwd)/rel_interfaces:/home/relant/ros2_ws/src/rel_interfaces \
 rel-ros:0.1.0 /bin/bash
```

## Environment Config

Testing with DEVICE_PORTS=5,8

Master IOLink

```bash
export MASTER_IO_LINK_IP=0.0.0.0
export MASTER_IO_LINK_PORT=8844
```


## Run on local

### my local all

```bash
export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_master_control"
export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_hmi"
export CONFIG_PATH="/home/moisesrc/vs-workspace/rel_ros/config"
```

### RPi4

```bash
export PYTHONPATH="${PYTHONPATH}:/home/relant/git/rel_ros/rel_ros_master_control"
export PYTHONPATH="${PYTHONPATH}:/home/relant/git/rel_ros/rel_ros_hmi"
export CONFIG_PATH="/home/relant/git/rel_ros/config"
```

### for ROS Docker image

> This is important to execute before running (only once)

```bash
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/venv/lib/python3.10/site-packages"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_hmi"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_master_control"
```

export the `rel_ros_hmi` in the path is required to run the master modbus test from hmi package

### ROS Interfaces

#### Requirements

```bash
pip install empy==3.3.4
pip install numpy
```

The interface messages should be defined in `./rel_interfaces/CMakeLists.txt`, then compile as:

edit `run-ros-build-interfaces` to add the correct interface message

```bash
cd ~/ros2_ws
./run-ros-build-interfaces.sh
```

## Run ROS Nodes

This is how we can run all nodes

```bash
export PYTHONPATH=$PYTHONPATH:/home/relant/ros2_ws/venv/lib/python3.10/site-packages
source install/setup.bash
ros2 run rel_ros_master_control rel_ros_master_control_node
ros2 run rel_ros_hmi rel_ros_hmi_node
```

### 1. Export env vars

```bash
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_hmi"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_master_control"
export USE_TEST_MODBUS="true"
export APP_MASTER_IOLINK_ID="0"
```

### 2. Run test modbus master control (IOLink) slave. The slave is a server, contains all register data

`python  ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/modbus_slave.py`

### 3. Run rest API app (optional)

`python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/rest/app.py`

### 4. Run test HMI modbus slave

`python ~/ros2_ws/src/rel_ros_hmi/rel_ros_hmi/modbus_slave.py`

### 5. Run IOLink Node

```bash
cd ~/ros2_ws
./run-ros-master.sh
```

---

### Master Commands for IOLink and HMI

#### HMI master commands

```bash

python ~/ros2_ws/src/rel_ros_hmi/rel_ros_hmi/modbus_master.py --action write --register 40010 --value 1200

python ~/ros2_ws/src/rel_ros_hmi/rel_ros_hmi/modbus_master.py --action read --register 40010

# read all
python ~/ros2_ws/src/rel_ros_hmi/rel_ros_hmi/modbus_master.py --action read --register 0
```

#### IOLink test slave with HMI too

```bash

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/modbus_slave.py

cd ~/ros2_ws/src
python rel_ros_master_control/rel_ros_master_control/control.py --action write --register 2002 --value 1200

# read all
# iolink
python rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0 -m iolink

# hmi read all hr
python rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0 -m hmi -x holiding

# hmi read all cr
python rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0 -m hmi -x coil

# hmi coil
python rel_ros_master_control/rel_ros_master_control/control.py --action write --register 5 -m hmi -x coil --value 1
python rel_ros_master_control/rel_ros_master_control/control.py --action read --register 5 -m hmi -x coil
```

---

## Sym links in RPi

```bash
ln -s /home/relant/git/rel_ros/rel_ros_hmi /home/relant/ros2_ws/src/rel_ros_hmi && \
ln -s /home/relant/git/rel_ros/rel_ros_master_control /home/relant/ros2_ws/src/rel_ros_master_control && \
ln -s /home/relant/git/rel_ros/rel_interfaces /home/relant/ros2_ws/src/rel_interfaces
```

### Run ROS on RPi4

```
cd /home/relant/ros2_ws
cp /home/relant/git/rel_ros/*.sh .
cp -r /home/relant/git/rel_ros/rel_ros_master_control/rel_ros_master_control/config /home/relant/ros2_ws/install/rel_ros_master_control/lib/python3.12/site-packages/rel_ros_master_control/
cp -r /home/relant/git/rel_ros/rel_ros_hmi/rel_ros_hmi/config /home/relant/ros2_ws/install/rel_ros_hmi/lib/python3.12/site-packages/rel_ros_hmi/

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

## PWM

https://www.electronicwings.com/raspberry-pi/raspberry-pi-pwm-generation-using-python-and-c

first, install `sudo apt-get install python3.11-dev`
need this lib https://pypi.org/project/RPi.GPIO/

Error running PWM
https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root

No access to /dev/mem

sudo apt install rpi.gpio-common
sudo adduser relant dialout
sudo groupadd gpio
sudo usermod -a -G gpio relant
sudo grep gpio /etc/group
sudo chown root:gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem

sudo reboot

## Static IP config in Ubuntu for RPi

`nano /etc/netplan/99_config.yaml`

```yaml
network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.0.10/24
```

Then run
`sudo netplan apply`
