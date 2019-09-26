#!/bin/bash

# install boost
mkdir ~/boost
cd ~/boost
wget http://sourceforge.net/projects/boost/files/boost/1.58.0/boost_1_58_0.tar.bz2
tar xvfo boost_1_58_0.tar.bz2
cd boost_1_58_0
./bootstrap.sh
sudo ./b2 install

# increase swap size
sudo dphys-swapfile swapoff
sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=1024/g' /etc/dphys-swapfile
sudo dphys-swapfile swapon

# Setup ROS Repositories
sudo apt-get install dirmngr -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# update and upgrade the system
sudo apt update
sudo apt upgrade

# Install Bootstrap Dependencies
sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake -y

# Initializing rosdep
sudo rosdep init
rosdep update

# Installation
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

# ROS-Comm
rosinstall_generator ros_comm --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
wstool init src kinetic-ros_comm-wet.rosinstall

# Resolve Dependencies
mkdir -p ~/ros_catkin_ws/external_src
cd ~/ros_catkin_ws/external_src
wget http://sourceforge.net/projects/assimp/files/assimp-3.1/assimp-3.1.1_no_test_models.zip/download -O assimp-3.1.1_no_test_models.zip
unzip assimp-3.1.1_no_test_models.zip
cd assimp-3.1.1
cmake .
make
sudo make install

# Resolving Dependencies with rosdep
cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:buster

# Building the catkin Workspace
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2

# ROS environment variables
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

#### Install Ros Packages

sudo apt-get install git python-catkin-tools python-rosinstall-generator -y

# Install MAVLINK && MAVROS
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
rosinstall_generator --rosdistro kinetic --upstream mavros | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
#sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Install raspicam-node
cd ~/ros_catkin_ws/src
git clone https://github.com/UbiquityRobotics/raspicam_node.git
cd ~/ros_catkin_ws/
sudo touch /etc/ros/rosdep/sources.list.d/30-ubiquity.list
sudo echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" >> /etc/ros/rosdep/sources.list.d/30-ubiquity.list
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

# build all
catkin_make
