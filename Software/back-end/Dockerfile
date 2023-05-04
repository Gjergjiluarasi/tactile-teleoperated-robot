# FROM ros:noetic

# SHELL ["/bin/bash", "-c"]

# RUN apt-get update && apt-get -y install apt-utils curl nano neofetch net-tools && apt-get -y upgrade && apt-get -y autoremove 

# COPY . /root/maxon

# RUN cd /root/maxon \ 
#     &&  chmod +x install_ros_humble.sh \
#     &&  ./install_ros_humble.sh

# # RUN locale #check for UTF-8 \ 
# #     && apt-get update && apt-get install locales \ 
# #     && locale-gen en_US en_US.UTF-8 \ 
# #     && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \ 
# #     && export LANG=en_US.UTF-8 \ 
# #     && locale # verify settings \ 
# #     && apt-get install software-properties-common \ 
# #     && add-apt-repository universe \ 
# #     && apt-get update && apt-get install curl -y \ 
# #     && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \ 
# #     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |tee /etc/apt/sources.list.d/ros2.list > /dev/null 

# # RUN apt-get update && apt-get -y upgrade && apt-get -y autoremove 

# # RUN apt-get install -y ros-humble-desktop 

# # RUN apt-get install ros-dev-tools 

# # RUN echo "source /opt/ros/humble/setup.bash" > ~/.bashrc \ 
#     # && source /opt/ros/humble/setup.bash 

# # ENTRYPOINT [ "./ros-entrypoint.sh" ] 

# WORKDIR /root 

FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# Make installation non interactive so that docker doesnt crash during build
ENV DEBIAN_FRONTEND noninteractive

# Define work directory
ARG workdir=/root
ARG projectdir=${workdir}/6G-life

# Set workdir
WORKDIR ${workdir}

# Install utilies
RUN apt-get update && \
    apt-get install -y apt-utils
RUN apt-get update && \
    apt-get install -y tree nano neofetch net-tools iputils-ping libxkbcommon-x11-0 qt5-default && \
    apt-get upgrade -y && \
    apt-get autoremove
RUN apt-get update &&\
    apt-get install -y\
    dpkg \
    cmake \
    libeigen3-dev \
    libpoco-dev \
    && rm -rf /var/lib/apt/lists/*

# Python setup
RUN apt-get update &&\
    apt-get install python3.10 -y &&\
    apt install -y \
    python3-pip \
    python3-tk \
    python3-gi-cairo &&\
    pip install --upgrade pip &&\
    pip install \ 
    numpy \
    matplotlib

# Install ROS1 packages
RUN apt-get update && \
    apt-get install -y build-essential ros-noetic-catkin python3-catkin-tools git
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp && mkdir build && cd build && cmake .. && make -j && make -j install
RUN mkdir -p ${workdir}/catkin_ws/src && cd ${workdir}/catkin_ws/src \
    && git clone https://github.com/leggedrobotics/message_logger.git \
    && git clone https://github.com/leggedrobotics/soem_interface.git \
    && git clone https://github.com/leggedrobotics/ethercat_sdk_master.git \
    && git clone https://github.com/leggedrobotics/maxon_epos_ethercat_sdk.git \
    && git clone https://github.com/leggedrobotics/ethercat_device_configurator.git

# Install ROS2 packages
RUN apt-get update && apt-get install -y curl && \
    apt-get update && apt-get install -y gnupg2 lsb-release && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt-get update && \
    apt-get install -y ros-foxy-ros-base
RUN apt-get update && \
    apt-get install -y ros-foxy-ros1-bridge ros-foxy-demo-* ros-foxy-turtle*
RUN apt-get install -y python3-colcon-common-extensions

# Change to workdir directory
RUN cd ${workdir}

# Copy project files
COPY . .

RUN ./scripts/install_libfranka.sh 

ENTRYPOINT bash && echo "Attaching to container" && exec "$@"

# # Build ethercat_device_configurator
# RUN cd ${workdir}/catkin_ws && \
#     ./scripts/build_ethercat_device_configurator.sh

# # Build ros_franka

# # Run ros_franka

# # Run ethercat_device_configurator
# RUN cd ${workdir}/catkin_ws && \
#     ./scripts/run_ethercat_device_configurator.sh

# Run bash
CMD bash

# Setup ROS1 environment
# RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Setup ROS2 environment
# RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

### Important: for every reboot: run xhost +local:docker
