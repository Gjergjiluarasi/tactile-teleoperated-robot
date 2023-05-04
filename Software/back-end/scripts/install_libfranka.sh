#!/bin/bash

### LIBFRANKA
cd /root
if [ ! -d "libfranka" ]
then 
    git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0 
fi
cd libfranka

if [ ! -d "build" ]
then 
    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  ..
    cmake --build . -j$(nproc)
    cpack -G DEB
    sudo dpkg -i libfranka-*.deb
fi

### ROS 2
cd /root
if [ ! -d "ros2_ws" ]
then 
    source /opt/ros/foxy/setup.bash
    mkdir -p ros2_ws/src
    cd ros2_ws
    git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2
    # rosdep install --from-paths src --ignore-src
    # colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/root/libfranka/build
    # source install/setup.sh
else
    echo "franka_ros2 is installed" 
    # cd ros2_ws
    # source /opt/ros/foxy/setup.bash
    # source install/setup.sh
fi

# echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc