# Multi UAV

A ROS package for multiple UAVs Simulation in Gazebo simulator.

## Features
- Provide 3D simulation using Gazebo, ROS, MAVROS and PX4 firmware;
- Include a ROS Node to control each UAV use simple MAVROS commands;
- Include a ROS Node to control multiple UAVs in formation;
- The UAV model was set up with a camera;
- It is possible to visualize the UAV camera in RVIZ;

## Installation
Following the steps above you are able to perform a simple setup of a Multi UAV simulation.

### Install ROS and setup multi_uav package
This tutorial install on Ubuntu 20.04, the ROS Noetic version and current multi_uav package version.

### Creating a ROS Workspace folder (don't change the folder name!)
Run on terminal:
```sh
mkdir -p ~/drone_simulator_ws/src
```
### Installing ROS and multi_uav Package
Run on terminal:
```sh
cd ~/drone_simulator_ws/src
git clone https://github.com/maikbasso/multi_uav.git multi_uav
sh ~/drone_simulator_ws/src/multi_uav/scripts/install/install.sh
```
In the installation, the script can request your Linux user password and other user inputs.

## Running a simulation
Run on terminal:
```sh
source ~/drone_simulator_ws/src/multi_uav/scripts/init/init_ws.sh
roslaunch multi_uav multi_uav.launch
```

Optional parameters:

    - worldName: the name of the world in worlds folder;
    - modelType: xacro or sdf;
    - vehicle: iris, iris_fpv_camera, iris_depth_camera, solo, solo_fpv_camera, solo_depth_camera (only if using sdf model type);
    - numberUAVs: the number of drones (0 to N drones). N increases until your PC supports;

Example:
```sh
roslaunch multi_uav multi_uav.launch vehicle:=solo modelType:=sdf numberUAVs:=5 worldName:=empty
```

## Node examples
With the simulation running, run in a new terminal:
```sh
source ~/drone_simulator_ws/src/multi_uav/scripts/init/init_ws.sh
```
Then, run a example:
```sh
roslaunch multi_uav offboard_local_control.launch
	or
roslaunch multi_uav offboard_global_control.launch
```

## Configuring a Ground Station
- Run on terminal:

```sh
sh ~/drone_simulator_ws/src/multi_uav/scripts/run/run_qgroundcontrol.sh
```

- Click on QGroundControl Icon;
- Click on "Comm Links";
- Click in Add;
- Insert all informations following the image bellow:

![UDP PORT](others/tutorial/QGorundControlUDPPORT.png "UDP PORT")

- Click on OK;
- A connection link appears on "Comm Links" tab, as in the image below:

![COMM LINKS](others/tutorial/QGroundCommLink.png "COMM LINKS")

- Close and open a ground station again. If a simulation is running the ground station connects automatically in each UAV.

## Solving problems

### Compiling on Raspberry Pi 3 B+ with Ubuntu 16.04
Firstly, you need to increase the virtual memory:
```sh
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```
Now, install the additional ros packages:
```sh
sudo apt install ros-<version>-gazebo-*
```
Then, compile the project:
```sh
cd catkin_ws
catkin_make
```
