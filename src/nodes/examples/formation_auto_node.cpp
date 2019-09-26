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

void printAndWait(double time)
{
  std::system("clear");
  formation->printCurrentFormationInfo();
  ros::Duration(time).sleep();
}

void mission()
{
  double droneDistance=0.0, apertureAngle=0.0, yawAngle=0.0, height=0.0;
  double distance=0.0, angle=0.0;
  double time=5.0;
  ros::Duration(10.0).sleep();  

  droneDistance=2.0, apertureAngle=0.0, yawAngle=45.0, height=3.0;
  formation->line  (droneDistance, yawAngle, height);
  printAndWait(time);

  distance=4.0, angle=0.0;
  formation->moveBackwardFormation    (distance);
  printAndWait(time);

  distance=4.0, angle=0.0;
  formation->moveRightFormation       (distance);
  printAndWait(time);

  droneDistance=2.0, apertureAngle=90.0, yawAngle=45.0, height=3.0;
  formation->arrow (droneDistance, apertureAngle, yawAngle, height);
  printAndWait(time);

  apertureAngle=25.0;
  formation->setApertureAngleFormation(apertureAngle);
  printAndWait(time);

  droneDistance=4.0;
  formation->setDroneDistanceFormation(droneDistance);
  printAndWait(time);

  apertureAngle=120.0;
  formation->setApertureAngleFormation(apertureAngle);
  printAndWait(time);

  yawAngle=100.0;
  formation->setYawAngleFormation     (yawAngle);
  printAndWait(time);

  yawAngle=180.0;
  formation->setYawAngleFormation     (yawAngle);
  printAndWait(time);

  apertureAngle=180.0;
  formation->setApertureAngleFormation(apertureAngle);
  printAndWait(time);

  apertureAngle=270.0;
  formation->setApertureAngleFormation(apertureAngle);
  printAndWait(time);

  distance=4.0;
  formation->moveForwardFormation     (distance);
  printAndWait(time);

  distance=4.0;
  formation->moveLeftFormation    (distance);
  printAndWait(time);

  droneDistance=2.0, apertureAngle=90.0, yawAngle=180.0, height=5.0;
  formation->line  (droneDistance, yawAngle, height);
  printAndWait(time);

  droneDistance=2.0, apertureAngle=90.0, yawAngle=180.0, height=5.0;
  formation->cross (droneDistance, apertureAngle, yawAngle, height);
  printAndWait(time);

  distance=4.0;
  formation->moveLeftFormation        (distance);
  printAndWait(time);

  yawAngle=270.0;
  formation->setYawAngleFormation     (yawAngle);
  printAndWait(time);

  yawAngle=0.0;
  formation->setYawAngleFormation     (yawAngle);
  printAndWait(time);

  apertureAngle=130.0;
  formation->setApertureAngleFormation(apertureAngle);
  printAndWait(time);

  droneDistance=3.0;
  formation->setDroneDistanceFormation(droneDistance);
  printAndWait(time);

  droneDistance=3.0, apertureAngle=90.0, yawAngle=0.0, height=5.0;
  formation->line  (droneDistance, yawAngle, height);
  printAndWait(time);

  droneDistance=4.0, apertureAngle=90.0, yawAngle=0.0, height=2.0;
  formation->circle(droneDistance, yawAngle, height);
  printAndWait(time);

  droneDistance=4.0, apertureAngle=90.0, yawAngle=-45.0, height=2.0;
  formation->circle(droneDistance, yawAngle, height);
  printAndWait(time);



  droneDistance=4.0, apertureAngle=90.0, yawAngle=-90.0, height=2.0;
  formation->circle(droneDistance, yawAngle, height);
  printAndWait(time);

  droneDistance=2.0, apertureAngle=90.0, yawAngle=-90.0, height=2.0;
  formation->circle(droneDistance, yawAngle, height);
  printAndWait(time);

  distance=4.0;
  formation->moveRightFormation       (distance);
  printAndWait(time);

  distance=4.0;
  formation->moveBackwardFormation    (distance);
  printAndWait(time);

  distance=8.0;
  formation->moveUpFormation          (distance);
  printAndWait(time);

  ros::shutdown();

}

// all comands you can use in your mission:
// all functions are void and the parameters are double

//formation->circle(droneDistance, yawAngle, height);
//formation->line  (droneDistance, yawAngle, height);
//formation->cross (droneDistance, apertureAngle, yawAngle, height);
//formation->arrow (droneDistance, apertureAngle, yawAngle, height);

//formation->moveForwardFormation     (distance);
//formation->moveBackwardFormation    (distance);
//formation->moveLeftFormation        (distance);
//formation->moveRightFormation       (distance);
//formation->moveUpFormation          (distance);
//formation->moveDownFormation        (distance);

//formation->setDroneDistanceFormation(droneDistance);
//formation->setApertureAngleFormation(apertureAngle);
//formation->setYawAngleFormation     (yawAngle);


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
  std::thread *missionThread = new std::thread(mission);
  //std::thread *cameraViewThread = new std::thread(cameraView);

  std::thread *droneThreads[numDrones];

  for(int i=0; i<numDrones; i++){
    droneThreads[i] = new std::thread(&multi_uav::Formation::droneLocalControl, formation, i);
  }

  rosLoopThread->join();  
  missionThread->join();
  //cameraViewThread->join();
  for(int i=0; i<numDrones; i++){
    droneThreads[i]->join();
  }
  return 0;
}
