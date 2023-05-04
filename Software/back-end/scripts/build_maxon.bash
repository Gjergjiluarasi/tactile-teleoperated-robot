#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /home
if [ ! -d "~/catkin_ws" ]
then
    git clone https://github.com/jbeder/yaml-cpp.git
    cd yaml-cpp
    cmake -S . -B build -DYAML_BUILD_SHARED_LIBS=on
    cd build 
    make install
    cd 
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone https://github.com/leggedrobotics/message_logger.git
    git clone https://github.com/leggedrobotics/soem_interface.git
    git clone https://github.com/leggedrobotics/ethercat_sdk_master.git
    git clone https://github.com/leggedrobotics/maxon_epos_ethercat_sdk.git
    git clone https://github.com/leggedrobotics/ethercat_device_configurator.git
    cd ~/catkin_ws/
    catkin build ethercat_device_configurator    # catkin_make --only-pkg-with-deps message_logger
    # catkin_make --only-pkg-with-deps soem_interface_examples
    # catkin_make --only-pkg-with-deps ethercat_sdk_master
    # catkin_make --only-pkg-with-deps maxon_epos_ethercat_sdk
    # catkin_make --only-pkg-with-deps ethercat_device_configurator
fi

exec "$@"