#!/bin/bash

# remove modem manager
sudo apt-get remove modemmanager

# add serial permissions
sudo usermod -a -G dialout $USER

# install px4 dependencies
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool -y

# Install xxd (package depends on version)
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y

# Required python packages
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus

# setup dir
mkdir -p /home/$USER/drone_simulator_ws/multi_uav_dependencies/firmware
cd /home/$USER/drone_simulator_ws/multi_uav_dependencies/firmware

# clone repository of the firmware
git clone https://github.com/maikbasso/Firmware.git px4

# go to firmware folder
cd px4

# make px4 firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo

# return to ros workspace
cd /home/$USER/drone_simulator_ws/
