/*
@author Lucas Grazziotim <lucasgrazziotim@gmail.com>
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <multi_uav/Formation.h>
#include <thread>
#include <vector>
#include <string>
#include <multi_uav/utils/Math.h>

std::vector<int> splitInts(const std::string& s, char delimiter){
  std::vector<int> numbers;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)){
    try {
      int number = std::stoi(token);
      numbers.push_back(number);
    } catch (std::invalid_argument const &e){
      std::cout << "Bad input: std::invalid_argument thrown" << std::endl;
    } catch (std::out_of_range const &e){
      std::cout << "Integer overflow: std::out_of_range thrown" << std::endl;
    }
  }
  return numbers;
}

int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "joystick_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  //read drone number
  if(nh.hasParam("/joystick_node/droneIds")){
      std::string droneIdsString;
      nh.getParam("/joystick_node/droneIds", droneIdsString);

      // split ids
      std::vector<int> droneIds = splitInts(droneIdsString, ',');

      //create drones objects
      std::vector<multi_uav::Drone*> drones;
      for (int i=0; i<droneIds.size(); i++) {
        multi_uav::Drone *drone = new multi_uav::Drone(nh, droneIds[i], false);
        drones.push_back(drone);
      }

      if(drones.size() > 0){
        multi_uav::Formation *formation = new multi_uav::Formation(drones);

        //set new offsets
        formation->setMoveOffset(0.1); //10 cm
        formation->setUpDownOffset(0.1); //10 cm
        formation->setRotateOffset(1.0); // 1 degree

        std::thread *joystickModeThread = new std::thread(&multi_uav::Formation::joystickMode, formation);
        std::vector<std::thread *> droneThreads;

        for (int i=0; i<droneIds.size(); i++) {
          droneThreads.push_back(new std::thread(&multi_uav::Formation::droneLocalControl, formation, droneIds[i]));
        }

        ros::Rate rate(20.0);
        while(ros::ok()){
          ros::spinOnce();
          rate.sleep();
        }

        formation->~Formation();
      }
  }
  else{
      std::cout << "droneIds parameter not specified." << std::endl;
  }

  return 0;
}
