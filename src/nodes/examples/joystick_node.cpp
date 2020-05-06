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

int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "joystick_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  std::vector<multi_uav::Drone*> drones;

  //read drone number
  if(nh.hasParam("/joystick_node/droneId")){
      int droneNumber;
      nh.getParam("/joystick_node/droneId", droneNumber);

      multi_uav::Drone *drone = new multi_uav::Drone(nh, droneNumber, true);
      drones.push_back(drone);

      multi_uav::Formation *formation = new multi_uav::Formation(drones);

      //set new offsets
      formation->setMoveOffset(0.1); //10 cm
      formation->setUpDownOffset(0.1); //10 cm
      formation->setRotateOffset(1.0); // 1 degree

      std::thread *joystickModeThread = new std::thread(&multi_uav::Formation::joystickMode, formation);
      std::thread *droneThread = new std::thread(&multi_uav::Formation::droneLocalControl, formation, droneNumber);

      ros::Rate rate(20.0);

      while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
      }

      formation->~Formation();
  }
  else{
      std::cout << "droneId parameter not specified." << std::endl;
  }

  return 0;
}
