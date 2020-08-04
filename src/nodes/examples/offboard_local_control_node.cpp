/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <multi_uav/utils/GlobalPosition.h>
#include <thread>

void droneLocalControl( ros::NodeHandle nh, int droneNumber ){
  multi_uav::Drone *d = new multi_uav::Drone(nh, droneNumber, true);

  d->configureToUseLocalCoordinates();

  d->forceModeOffboard();

  d->arm();

  double posd = 2.0;
  double posh = 2.0;

  while(ros::ok()){

    d->goToLocalPosition(0.0, 0.0, posh, 0.0, true);
    d->goToLocalPosition(0.0, 0.0, posh, 90.0, true);

    d->goToLocalPosition(0.0, posd, posh, 90.0, true);
    d->goToLocalPosition(0.0, posd, posh, 0.0, true);

    d->goToLocalPosition(posd, posd, posh, 0.0, true);
    d->goToLocalPosition(posd, posd, posh, -90.0, true);

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

  std::thread *drone0Thread = new std::thread(droneLocalControl, nh, 0);
  std::thread *drone1Thread = new std::thread(droneLocalControl, nh, 1);
  std::thread *drone2Thread = new std::thread(droneLocalControl, nh, 2);

  ros::Rate rate(20.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
