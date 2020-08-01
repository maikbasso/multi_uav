/*
@author Maik Basso <maik@maikbasso.com.br>
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
  ros::init(argc, argv, "image_viewer_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  //parameters
  int droneNumber = 0;
  std::string camera = "rgb";

  //get user imput parameters
  if(nh.hasParam("/image_viewer_node/droneId")){
      nh.getParam("/image_viewer_node/droneId", droneNumber);
  }

  if(nh.hasParam("/image_viewer_node/camera")){
      nh.getParam("/image_viewer_node/camera", camera);
  }

  multi_uav::Drone *drone = new multi_uav::Drone(nh, droneNumber, false);

  std::stringstream ss;
  ss << "Drone " << drone->parameters.id << " [" << camera << "]";
  std::string title = ss.str();
  cv::namedWindow(title.c_str(), cv::WINDOW_NORMAL);
  cv::resizeWindow(title.c_str(), 480, 360);

  ros::Rate rate(10.0);
  while(ros::ok()){
    ros::spinOnce();

    if(camera == "rgb"){
      cv::imshow(title.c_str(), drone->sensors.camera.rgb.image);
    }
    else if(camera == "osd"){
      cv::imshow(title.c_str(), drone->getOSDImage());
    }
    else {
      cv::imshow(title.c_str(), drone->getOSDImage());
    }

    cv::waitKey(1);

    rate.sleep();
  }


  return 0;
}
