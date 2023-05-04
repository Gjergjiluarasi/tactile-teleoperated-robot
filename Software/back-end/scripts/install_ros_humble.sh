#!/bin/bash

# Summarized of http://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html
# Setup ROS 2 Humble on Ubuntu 22.04
# First step：setlocale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

#Second Step: Add ROS 2 apt repository 
#apt-cache policy | grep universe
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade

#Third Step: Install ros-humble-desktop， ros-humble-ros-base
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base

#Fourth Step: Config environment
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
printenv | grep -i ROS
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc