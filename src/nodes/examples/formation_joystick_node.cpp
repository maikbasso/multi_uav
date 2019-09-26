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
void cameraView(){

  ros::Duration(2).sleep();

  std::vector<std::string> windowNames;

  for (int i=0; i<formation->getDrones().size(); i++){
    std::stringstream ss;
    ss << "Drone " << formation->getDrones()[i]->parameters.id;
    std::string title = ss.str();
    windowNames.push_back(title);
    cv::namedWindow(title.c_str(), cv::WINDOW_NORMAL);
    cv::resizeWindow(title.c_str(), 480, 360);
  }

  // display the image
  while(ros::ok()){

    for (int i=0; i<formation->getDrones().size(); i++){
      cv::imshow(windowNames[i].c_str(), formation->getDrones()[i]->getOSDImage());
    }

    cv::waitKey(1);

  }

  // Closes all the frames
  cv::destroyAllWindows();
}


int  setNumberDrones()
{
  int numDrones;
  std::cout << std::endl << "Type number of UAVs in the simulation (min=1): ";
  std::cin >>  numDrones;
  if(numDrones < 1)    {
    numDrones = 1;
    std::cout << "Invalid numDrones" << std::endl << " numDrones set to 1" << std::endl;
  }
  std::cout << "Go take a cup of coffee. This may take a while" << std::endl;
  std::cout << "Wait for the UAVs to stabilize in the air" << std::endl;
  return numDrones;
}


int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "formation_joystick_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  std::vector<multi_uav::Drone*> drones;
  int numDrones = setNumberDrones();

  for(int i=0; i<numDrones; i++){
    multi_uav::Drone *drone = new multi_uav::Drone(nh, i, false);
    drones.push_back(drone);
  }

  formation = new multi_uav::Formation(drones);

  std::thread *rosLoopThread = new std::thread(rosLoop);  
  std::thread *joystickModeThread = new std::thread(&multi_uav::Formation::joystickMode, formation);
  //std::thread *cameraViewThread = new std::thread(cameraView);

  std::thread *droneThreads[numDrones];

  for(int i=0; i<numDrones; i++){
    droneThreads[i] = new std::thread(&multi_uav::Formation::droneLocalControl, formation, i);
  }

  rosLoopThread->join();  
  joystickModeThread->join();
  //cameraViewThread->join();
  for(int i=0; i<numDrones; i++){
    droneThreads[i]->join();
  }
  return 0;
}
