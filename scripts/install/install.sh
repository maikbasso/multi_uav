#!/bin/bash
sh /home/$USER/drone_simulator_ws/src/multi_uav/scripts/install/install_qgroundcontrol.sh
sh /home/$USER/drone_simulator_ws/src/multi_uav/scripts/install/install_ros_neotic.sh
sh /home/$USER/drone_simulator_ws/src/multi_uav/scripts/install/install_firmware_px4.sh

# Compiling a ROS Workspace
cd /home/$USER/drone_simulator_ws/
catkin_make
