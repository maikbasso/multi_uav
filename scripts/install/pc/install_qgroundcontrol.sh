#!/bin/bash

# setup dir
mkdir -p /home/$USER/drone_simulator_ws/multi_uav_dependencies/ground-station
cd /home/$USER/drone_simulator_ws/multi_uav_dependencies/ground-station

# download
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage

# https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav -y
sudo chmod +x QGroundControl.AppImage

# go back directory
cd /home/$USER/drone_simulator_ws/