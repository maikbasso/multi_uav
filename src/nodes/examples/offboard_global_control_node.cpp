/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <multi_uav/utils/GlobalPosition.h>
#include <thread>

void droneGlobalControl( ros::NodeHandle nh, int droneNumber ){
  multi_uav::Drone *d = new multi_uav::Drone(nh, droneNumber, true);

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

  std::thread *drone0Thread = new std::thread(droneGlobalControl, nh, 0);

  ros::Rate rate(20.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
