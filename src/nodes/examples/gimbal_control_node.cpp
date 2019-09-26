/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <thread>
#include <iostream>
#include <string>
#include <termios.h>

#define ANGLE_INCREMENT 1.0

std::vector<multi_uav::Drone*> drones;

void rosLoop(){
  ros::Rate rate(20.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}

void cameraView(){

  ros::Duration(2).sleep();

  std::vector<std::string> windowNames;

  for (int i=0; i<drones.size(); i++){
    std::stringstream ss;
    ss << "Drone " << drones[i]->parameters.id;
    std::string title = ss.str();
    windowNames.push_back(title);
    cv::namedWindow(title.c_str(), cv::WINDOW_NORMAL);
    cv::resizeWindow(title.c_str(), 480, 360);
  }

  // display the image
  while(ros::ok()){

    for (int i=0; i<drones.size(); i++){
      cv::imshow(windowNames[i].c_str(), drones[i]->getOSDImage());
    }

    cv::waitKey(1);

  }

  // Closes all the frames
  cv::destroyAllWindows();
}

int getch(){
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();  // read character (blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  usleep(100000);
  return c;
}

void printMenu(double pitch){

  system("clear");

  std::cout << "==========================" << std::endl;
  std::cout << "= Gimbal control example =" << std::endl;
  std::cout << "==========================" << std::endl;

  std::cout << "= Options:               =" << std::endl;
  std::cout << "= (+) : increase pitch   =" << std::endl;
  std::cout << "= (-) : decrease pitch   =" << std::endl;

  std::cout << "==========================" << std::endl;
  std::cout << std::fixed << std::setprecision(2) << "= pitch: " << pitch;
  if(pitch >= 10.0 || (pitch < 0.0 && pitch > -10.0)){
    std::cout << "           =" << std::endl;
  }
  else if(pitch <= -10.0){
    std::cout << "          =" << std::endl;
  }
  else{
    std::cout << "            =" << std::endl;
  }
  std::cout << "==========================" << std::endl;
}

void gimbalControl( ros::NodeHandle nh, int droneNumber ){
  multi_uav::Drone *d = new multi_uav::Drone(nh, droneNumber, true);

  drones.push_back(d);

  while(ros::ok()){

    printMenu(d->hardware.gimbal.pitch);

    switch(getch()){
      case 43:
        d->setGimbalOrientation(d->hardware.gimbal.pitch + ANGLE_INCREMENT);
        break;
      case 45:
        d->setGimbalOrientation(d->hardware.gimbal.pitch - ANGLE_INCREMENT);
        break;
      default:
        std::cout << "Invalid command!" << std::endl;
    }

  }

  d->~Drone();
}

int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "gimbal_control_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  std::thread *rosLoopThread = new std::thread(rosLoop);
  std::thread *cameraViewThread = new std::thread(cameraView);
  std::thread *drone0Thread = new std::thread(gimbalControl, nh, 0);

  rosLoopThread->join();
  cameraViewThread->join();
  drone0Thread->join();

  return 0;
}
