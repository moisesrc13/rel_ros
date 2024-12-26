FROM ros:humble

ENV RELANT_PACKAGE=rel_ros_master_control
ENV RELANT_NODE=rel_ros_master_control_node

USER root
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
# install ros package
RUN apt-get update && apt-get install -y \
      ros-humble-demo-nodes-cpp curl wget \
      virtualenv nano qt5-* \
      ros-humble-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*


RUN useradd -m relant

USER relant

WORKDIR /home/relant
COPY ./run.sh /home/relant/run.sh

# create ROS workspace and virutal env
RUN mkdir -p /home/relant/ros2_ws/src
COPY ./requirements.txt /home/relant/ros2_ws/requirements.txt
COPY ./rel-node-run.sh /home/relant/ros2_ws/rel-node-run.sh
RUN cd /home/relant/ros2_ws && virtualenv -p python3.10 ./venv && touch ./venv/COLCON_IGNORE


# activate venv and install dependencies
RUN source /opt/ros/humble/setup.bash && source /home/relant/ros2_ws/venv/bin/activate && pip install -r /home/relant/ros2_ws/requirements.txt

# create dev workspace for vscode
RUN mkdir -p /home/relant/vs-workspace
COPY . /home/relant/vs-workspace/rel-ros

USER root
RUN chmod -R g+r /home/relant
RUN chown -R relant:relant /home/relant
# install VS Code (code-server)
RUN curl -fsSL https://code-server.dev/install.sh | sh


# configure git
USER relant
RUN git config --global --add safe.directory /home/relant/vs-workspace/rel-ros
RUN git config --global user.email "jm.roco@gmail.com"
RUN git config --global user.name "Moises Romo"

# create ROS package
RUN cd ~/ros2_ws/src && source /opt/ros/humble/setup.bash && ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs  --license Apache-2.0 ${RELANT_PACKAGE}
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/relant/ros2_ws/venv/bin/activate" >> ~/.bashrc


# launch ros package
CMD ["/home/relant/run.sh"]
