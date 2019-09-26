/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <multi_uav/utils/GlobalPosition.h>
#include <thread>

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

void droneLocalControl( ros::NodeHandle nh, int droneNumber ){
  multi_uav::Drone *d = new multi_uav::Drone(nh, droneNumber, true);

  drones.push_back(d);

  d->configureToUseLocalCoordinates();


  d->forceModeOffboard();

  d->arm();

  double posd = 2.0;
  double posh = 2.0;

  while(ros::ok()){

    d->setGimbalOrientation(45.0);

    d->goToLocalPosition(0.0, 0.0, posh, 0.0, true);
    d->goToLocalPosition(0.0, 0.0, posh, 90.0, true);

    d->goToLocalPosition(0.0, posd, posh, 90.0, true);
    d->goToLocalPosition(0.0, posd, posh, 0.0, true);

    d->goToLocalPosition(posd, posd, posh, 0.0, true);
    d->goToLocalPosition(posd, posd, posh, -90.0, true);

    d->setGimbalOrientation(0.0);

    d->goToLocalPosition(posd, 0.0, posh, -90.0, true);
    d->goToLocalPosition(posd, 0.0, posh, -180.0, true);

    d->goToLocalPosition(0.0, 0.0, posh, -180.0, true);
    d->goToLocalPosition(0.0, 0.0, posh, 0.0, true);

  }

  //d->disarm();

  d->~Drone();
}

int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "offboard_local_control_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  std::thread *rosLoopThread = new std::thread(rosLoop);
  std::thread *cameraViewThread = new std::thread(cameraView);
  std::thread *drone0Thread = new std::thread(droneLocalControl, nh, 0);
  std::thread *drone1Thread = new std::thread(droneLocalControl, nh, 1);
  std::thread *drone2Thread = new std::thread(droneLocalControl, nh, 2);

  rosLoopThread->join();
  cameraViewThread->join();
  drone0Thread->join();
  drone1Thread->join();
  drone2Thread->join();

  return 0;
}
