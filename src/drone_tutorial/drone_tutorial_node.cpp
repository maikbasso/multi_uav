#include <ros/ros.h>
#include <multi_uav/Drone.h>
#include <thread>

void rosLoop(){
  ros::Rate rate(20.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}

void droneLocalControl( ros::NodeHandle nh){
  multi_uav::Drone *d = new multi_uav::Drone(nh, 0, true);

  d->configureToUseLocalCoordinates();

  d->forceModeOffboard();

  d->arm();

  d->setGimbalOrientation(0.0);

  d->goToLocalPosition(0.0, 0.0, 5.0, 0.0, true);

  d->setGimbalOrientation(45.0);

  d->goToLocalPosition(0.0, 0.0, 5.0, 0.0, true);

  d->disarm();

  d->~Drone();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "drone_tutorial_node");
  ros::NodeHandle nh;

  std::thread *rosLoopThread = new std::thread(rosLoop);
  std::thread *drone0Thread = new std::thread(droneLocalControl, nh);

  rosLoopThread->join();
  drone0Thread->join();

  return 0;
}
