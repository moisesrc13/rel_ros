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
 -v $(pwd)/config:/home/relant/config \
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


## ROS messages

### publish

```bash
source install/setup.bash  # -> important to run this always before
ros2 topic pub /rel/hmi_user_task_0 rel_interfaces/msg/HMIUserTask "{coil_address: enter_manual_mode_screen, value: 1}"
ros2 topic pub /rel/hmi_user_task_0 rel_interfaces/msg/HMIUserTask "{coil_address: action_pull_up_pistons_manual, value: 1}"
```

### info
```bash
ros2 topic info /rel/hmi_user_task_0
ros2 topic type /rel/hmi_user_task_0
ros2 interface show rel_interfaces/msg/HMIUserTask
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

### 2. Run HMI node

```bash
cd ~/ros2_ws
./run-ros-hmi.sh
```

`python  ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/modbus_slave.py`

### 3. Run rest API app (optional)

`python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/rest/app.py`

### 4. Run IOLink Node

```bash
cd ~/ros2_ws
./run-ros-master.sh
```

---

### Master Tester

```bash
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control_tester.py -t a
```

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

# Run IOLink on local for testing
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/modbus_slave.py

# iolink write
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 2002 --value 1200 -m iolink

# read all
# iolink
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0 -m iolink

# hmi read all hr
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0 -m hmi -x holding

# hmi read all cr
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action read --register 0 -m hmi -x coil

# hmi coil
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 5 -m hmi -x coil --value 1

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action read --register 5 -m hmi -x coil

# tower
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action tower --register 5 -m iolink -t "full"

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action tower --register 5 -m iolink -t "medium"

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action tower --register 5 -m iolink -t "bucket_change"

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action tower --register 5 -m iolink -t "toff"

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action tower --register 5 -m iolink -t "acoustic_alarm_on"
```

### Manual HMI Tasks

```bash
# manual on
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 8 -m hmi -x coil --value 1

# prefill, this will run a sub flow to check sensor laser. PWM activated on D) Sensor laser d>X && d<W
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 14 -m hmi -x coil --value 1
## simulate state D

  ### -> sensor laser
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 2002 --value 60 -m iolink

  ### -> param vacuum limit high
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40015 --value 50 -m hmi -x holding

  ### -> param bucket size selection (option 1)
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40033 --value 1 -m hmi -x holding

  ### -> param distance bucket 1
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40028 --value 100 -m hmi -x holding

  ### CONDITION
  ## sensor_distance > sensor_distance_params.high_vacuum_limit and sensor_distance < sensor_distance_params.bucket_distance

# -> ./prefill-test.sh

PARAM_DISTANCE_BUCKET_1

# pistons down
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 15 -m hmi -x coil --value 1

# pistons up
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 16 -m hmi -x coil --value 1

# vacuum air
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 17 -m hmi -x coil --value 1

# depressurize
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 18 -m hmi -x coil --value 1

# recycle retractil
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 23 -m hmi -x coil --value 1

```

---

## Run ROS on RPi4

### Sym links in RPi

```bash
ln -s /home/relant/git/rel_ros/rel_ros_hmi /home/relant/ros2_ws/src/rel_ros_hmi && \
ln -s /home/relant/git/rel_ros/rel_ros_master_control /home/relant/ros2_ws/src/rel_ros_master_control && \
ln -s /home/relant/git/rel_ros/rel_interfaces /home/relant/ros2_ws/src/rel_interfaces && \
ln -s /home/relant/git/rel_ros/config /home/relant/config
```

### Files and requirements

```bash
cd /home/relant/ros2_ws
cp /home/relant/git/rel_ros/*.sh .

# install requirements, ensure the venv is activated
pip install -r ~/git/rel_ros/requirements.txt
```

### 1. Export env vars (RPi)

```bash
export CONFIG_PATH="/home/relant/config"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/venv/lib/python3.12/site-packages"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_hmi"
export PYTHONPATH="${PYTHONPATH}:/home/relant/ros2_ws/src/rel_ros_master_control"
export LOGLEVEL="DEBUG"
export USE_TEST_MODBUS="true"  # set this to false to work with IOLink connected in the network
export APP_MASTER_IOLINK_ID="0"
```

### 2. (FOR TEST ONLY) Run test modbus master control (IOLink) slave. The slave is a server, contains all register data (RPi)

`python  ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/modbus_slave.py`

### 3. Run rest API app (optional - RPi)

`python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/rest/app.py`

### 4. Run test HMI modbus slave (RPi)

```bash
cd ~/ros2_ws
authbind --deep ./run-ros-hmi.sh
```

### 5. Run IOLink Node (RPi)

```bash
cd ~/ros2_ws
authbind --deep ./run-ros-master.sh
```

---

### Master Tester (Rpi)

```bash
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control_tester.py -t a
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

This is RPi IP Network

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

## Team Viewer
ID: 928154640
pwd: vm2sruv8

## Setup Relant
RPi: 192.168.0.10
IOLink: 192.168.0.21:502

- HMI connection in VT Studio > System Settings > PLC communication
- Start simulator > Tool > Start simulator
- Send Data, Communications > PC VT Send Data > all data

## Non Root privileged ports (for modbus)

Note: this is not longer required since the modbus HMI now runs on port >= 8845

```bash
sudo apt-get install authbind
sudo mkdir /etc/authbind/byport
sudo touch /etc/authbind/byport/502
sudo chmod 777 /etc/authbind/byport/502
authbind --deep  # script here
```
# check for error

```
 rel_ros_master_control.services.pwm - ERROR - error setting up PWM - No access to /dev/mem.  Try running as root!
```

`Reldosys1069`

## PWM update

We just need to use the lib [lgpio](http://abyz.me.uk/lg/py_lgpio.html) and install it with pip as normal dependency. We must call method `gpio_claim_output` in order to enable the pin as PWM

### commands

```bash
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/services/start_pwm.py

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/services/stop_pwm.py
```

## User Password

coil: 28H -> 40 decimal
password: 0
