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
  ros::init(argc, argv, "osd_viewer_node");
  // ROS need to create a node handle first than other things
  ros::NodeHandle nh;

  //read drone number
  if(nh.hasParam("/osd_viewer_node/droneId")){
      int droneNumber;
      nh.getParam("/osd_viewer_node/droneId", droneNumber);

      multi_uav::Drone *drone = new multi_uav::Drone(nh, droneNumber, false);

      std::stringstream ss;
      ss << "Drone " << drone->parameters.id << " OSD";
      std::string title = ss.str();
      cv::namedWindow(title.c_str(), cv::WINDOW_NORMAL);
      cv::resizeWindow(title.c_str(), 480, 360);

      ros::Rate rate(10.0);
      while(ros::ok()){
        ros::spinOnce();

        cv::imshow(title.c_str(), drone->getOSDImage());
        cv::waitKey(1);

        rate.sleep();
      }

  }
  else{
      std::cout << "droneId parameter not specified." << std::endl;
  }

  return 0;
}
