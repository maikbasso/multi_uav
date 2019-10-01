/*
@author Lucas Grazziotim <lucasgrazziotim@gmail.com>
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <multi_uav/Formation.h>
#include <thread>
#include <vector>
#include <string.h>
#include <multi_uav/utils/Math.h>

multi_uav::Formation *formation;

void rosLoop(){
  ros::Rate rate(50.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  formation->~Formation();
}

int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "joystick_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  std::vector<multi_uav::Drone*> drones;

  int droneNumber;

  std::cout << "Type a valid UAV id: ";
  std::cin >> droneNumber;

  multi_uav::Drone *drone = new multi_uav::Drone(nh, droneNumber, false);
  drones.push_back(drone);

  formation = new multi_uav::Formation(drones);

  //set new offsets
  formation->setMoveOffset(0.1); //10 cm
  formation->setUpDownOffset(0.1); //10 cm
  formation->setRotateOffset(1.0); // 1 degree

  std::thread *rosLoopThread = new std::thread(rosLoop);  
  std::thread *joystickModeThread = new std::thread(&multi_uav::Formation::joystickMode, formation);
  std::thread *droneThread = new std::thread(&multi_uav::Formation::droneLocalControl, formation, droneNumber);

  rosLoopThread->join();  
  joystickModeThread->join();
  droneThread->join();

  return 0;
}
