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

void droneGlobalControl( ros::NodeHandle nh, int droneNumber ){
  multi_uav::Drone *d = new multi_uav::Drone(nh, droneNumber, true);

  drones.push_back(d);

  d->configureToUseGlobalCoordinates();

  d->forceModeOffboard();

  d->arm();

  multi_uav::utils::GlobalPosition *gp = new multi_uav::utils::GlobalPosition(
    d->parameters.position.global.latitude,
    d->parameters.position.global.longitude,
    d->parameters.position.global.altitude + 5.0,
    d->parameters.orientation.global.yaw
  );

  while(ros::ok()){

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

    gp->addPositionOffsetInMeters(10.0, 0.0);
    gp->addDegreesToYaw(50.0);

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

    gp->addPositionOffsetInMeters(-10.0, 0.0);
    gp->addDegreesToYaw(-50.0);

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

    gp->setYaw(360.0);

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

    gp->setYaw(100.0);

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

    gp->setYaw(0.0);

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);
  }

  //d->land();

  //d->disarm();

  d->~Drone();
}

int main(int argc, char **argv) {
  // init os
  ros::init(argc, argv, "offboard_global_control_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  std::thread *rosLoopThread = new std::thread(rosLoop);
  std::thread *cameraViewThread = new std::thread(cameraView);
  std::thread *drone0Thread = new std::thread(droneGlobalControl, nh, 0);
//  std::thread *drone1Thread = new std::thread(droneGlobalControl, nh, 1);
//  std::thread *drone2Thread = new std::thread(droneGlobalControl, nh, 2);

  rosLoopThread->join();
  cameraViewThread->join();
  drone0Thread->join();
//  drone1Thread->join();
//  drone2Thread->join();

  return 0;
}
