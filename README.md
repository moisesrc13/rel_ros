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
docker run --rm --name rel-ros -it -v rel_ros_master_control:/home/relant/ros2_ws/src/rel_ros_master_control -v rel_ros_hmi:/home/relant/ros2_ws/src/rel_ros_hmi rel-ros:0.1.0 /bin/bash
```
