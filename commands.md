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
# unblock manual for password

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 28 -m hmi -x coil --value 1


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

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/control.py --action write --register 14 -m hmi -x coil --value 1

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

```bash
python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/services/pwm_start.py -o medium

python ~/ros2_ws/src/rel_ros_master_control/rel_ros_master_control/services/stop_pwm.py
```
