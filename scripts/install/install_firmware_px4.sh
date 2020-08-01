#!/bin/bash

# Requirements
sudo apt install libgstreamer1.0-dev -y
sudo apt install gstreamer1.0-plugins-good -y
sudo apt install gstreamer1.0-plugins-bad -y
sudo apt install gstreamer1.0-plugins-ugly -y
sudo apt install xmlstarlet -y

# setup dir
mkdir -p /home/$USER/drone_simulator_ws/multi_uav_dependencies/firmware
cd /home/$USER/drone_simulator_ws/multi_uav_dependencies/firmware

# clone repository of the firmware
git clone https://github.com/maikbasso/Firmware.git px4

# go to firmware folder
cd px4

# make px4 firmware
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
DONT_RUN=1 make px4_sitl_default gazebo

# return to ros workspace
cd /home/$USER/drone_simulator_ws/
