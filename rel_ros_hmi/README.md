# rel_ros_hmi
Relant ROS2 node for HMI




## Docker

>build
`docker build . -t rel-ros-hmi:0.1.0`

>run
`docker run --rm --name rel-ros-hmi -it rel-ros-hmi:0.1.0 /bin/bash`

>run with mounted volume
$ cd ~/vs-workspace/rel_ros_hmi
`docker run --rm --name rel-ros-hmi -it -v $(pwd):/home/relant/ros2_ws/src/rel_ros_hmi rel-ros-hmi:0.1.0 /bin/bash`



## Run on local

> my local all
`export PYTHONPATH="${PYTHONPATH}:/home/moisesrc/vs-workspace/rel_ros/rel_ros_hmi"`

## Messaging

For custom message you need to add `std_msgs` in `package.xml` file

msg/HMIData.msg
```bash
int32 type  # 0 = parameter, 1 = sensor
int32 value
string name
```
