#!/bin/bash

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full -y
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional packages:
sudo apt install ros-noetic-mavlink -y
sudo apt install ros-noetic-mavros -y

# Install install geographiclib datasets:
wget -P /home/$USER/drone_simulator_ws -o install_geographiclib_datasets.sh https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo chmod +x /home/$USER/drone_simulator_ws/install_geographiclib_datasets.sh
sudo sh /home/$USER/drone_simulator_ws/install_geographiclib_datasets.sh

# Creating a ROS Workspace
cd /home/$USER/drone_simulator_ws/
catkin_make
