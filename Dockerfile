FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO noetic
# Install necessary packages

RUN apt-get update && apt-get install -y \
    python3-pip \
    ipython3 \
    python3-dev \
    python3-rosdep \
    python3-rosinstall \
    python3-rpi.gpio \
    python3-vcstools \
    python3-catkin-tools \
    neovim \
    ros-noetic-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install --no-install-recommends -y \
    software-properties-common \
    build-essential \
    git
RUN add-apt-repository universe

RUN pip3 install pyserial \
	setuptools==49.4.0 \
	numpy==1.19.5 \
	opencv-contrib-python==4.6.0.66 \ 
	requests \
	colorama \
	ipdb \
	pyrealsense2

#RUN rosdep init && \
#  rosdep update --rosdistro $ROS_DISTRO

##########################################
## CATKIN CONFIG ##
##########################################
ENV CATKIN_WS=/root/catkin_ws
ENV ROS_PYTHON_VERSION=3

RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src


RUN git clone https://github.com/ReconCycle/digital_interface_msgs.git
COPY src $CATKIN_WS/src
#RUN git clone https://github.com/ReconCycle/cnc_manager.git
#RUN git clone --branch unified_cnc https://github.com/ReconCycle/cnc_manager.git
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd $CATKIN_WS \
    && catkin init \
    && catkin config --install

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin build

# Install ROS packages if needed
# Example: RUN apt-get install -y ros-noetic-PACKAGE
#RUN apt-get install -y ros-noetic-tf2-py ros-noetic-tf2-tools python3-tf2-geometry-msgs
# Set up ROS environment variables
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Set the entrypoint for the container
ENTRYPOINT ["/entrypoint.sh"]
CMD tail -f /dev/null
