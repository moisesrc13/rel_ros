#!/usr/bin/env bash

## simulate state D

  ### -> sensor laser
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 2002 --value 60 -m iolink

  ### -> param vacuum limit high
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40015 --value 50 -m hmi -x holding

  ### -> param bucket size selection (option 1)
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40033 --value 1 -m hmi -x holding

  ### -> param distance bucket 1
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 40028 --value 100 -m hmi -x holding

# manual on
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 8 -m hmi -x coil --value 1

# prefill task, this will run a sub flow to check sensor laser. PWM activated on D) Sensor laser d>X && d<W
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 14 -m hmi -x coil --value 1
