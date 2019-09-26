/*
@author Maik Basso <maik@maikbasso.com.br>

Reference: http://gazebosim.org/tutorials/?tut=ros_plugins

*/
#ifndef GAZEBO_PLUGINS_GIMBALSMALL2DPLUGIN_HH_
#define GAZEBO_PLUGINS_GIMBALSMALL2DPLUGIN_HH_

#include <string>
#include <vector>
#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <multi_uav/RPY.h>
#include <multi_uav/utils/Math.h>

namespace gazebo{

// A plugin for controlling the angle of a gimbal joint
class GAZEBO_VISIBLE GimbalSmall2dPlugin : public ModelPlugin{

private:
  // A list of event connections
  std::vector<event::ConnectionPtr> connections;
  // Parent model of this plugin
  physics::ModelPtr model;
  // Joint for tilting the gimbal
  physics::JointPtr tiltJoint;
  // Pointer to the transport node
//  transport::NodePtr node;
  // PID controller for the gimbal
  common::PID pid;
  // Last update sim time
  common::Time lastUpdateTime;

  // Subscriber to the camera imu topic
//  transport::SubscriberPtr cameraIMUSubscriber;

//  void cameraIMUCallback(ConstIMUPtr &msg);
//  double currentCameraIMUPitch; // radians
  double currentDroneIMUPitch; // radians
  double lastDroneIMUPitch; // radians
//  double lastWishCameraPitch; // radians

  // Callback on world update
  void OnUpdate();

  //gimbal orientation
  multi_uav::RPY gimbalOrientation;
  ros::NodeHandle nodeHandle;
  ros::Subscriber gimbalOrientationSubscriber;
  ros::Subscriber droneIMUSubscriber;
  std::thread *rosLoopThread;

public:
  GimbalSmall2dPlugin();
  ~GimbalSmall2dPlugin();
  // Documentation Inherited.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  // Documentation Inherited.
  virtual void Init();
  //gimbal angle callback
  void gimbalOrientationCallback(const multi_uav::RPY::ConstPtr &msg);
  // drone imu callback
  void droneIMUCallback(const sensor_msgs::Imu::ConstPtr &msg);
  //ros loop
  void rosLoop();

};

// gazebo register plugin
GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPlugin)

}
#endif
