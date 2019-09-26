/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include "gazebo_plugins/GimbalSmall2dPlugin.h"

//exponential smooth filter
//#define EXPONETIAL_SMOOTH(A,X,Y) ((A*Y)+((1.0-A)*X))

namespace gazebo{

GimbalSmall2dPlugin::GimbalSmall2dPlugin() {

  // init PID
  this->pid.Init(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0);
  
  // init gimbal orientation
  this->gimbalOrientation.roll = 0.0;
  this->gimbalOrientation.pitch = 0.0;
  this->gimbalOrientation.yaw = 0.0;

  // init camera and gimbal imu orientation
//  this->currentCameraIMUPitch = 0.0; // radians
  this->currentDroneIMUPitch = 0.0; // radians
  this->lastDroneIMUPitch = 0.0; // radians
//  this->lastWishCameraPitch = 0.0; // radians

}

GimbalSmall2dPlugin::~GimbalSmall2dPlugin(){
  // close all
  this->rosLoopThread->~thread();
  this->gimbalOrientationSubscriber.shutdown();
}

void GimbalSmall2dPlugin::rosLoop(){
  // create a simple ros loop to get the RPY angle data
  ros::Rate rate(1.0);
  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}

void GimbalSmall2dPlugin::gimbalOrientationCallback(const multi_uav::RPY::ConstPtr &msg){
  // callback to be used to get RPY angle
  this->gimbalOrientation = *msg;
}

void GimbalSmall2dPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf){
  
  // get model data
  this->model = model;

  //set pitch joint name
  std::string jointName = "tilt_joint";

  //verify if model as any joint
  if (sdf->HasElement("joint")){
    jointName = sdf->Get<std::string>("joint");
  }

  // first try to get joint
  this->tiltJoint = this->model->GetJoint(jointName);

  // second try
  if (!this->tiltJoint){
    std::string scopedJointName = model->GetScopedName() + "::" + jointName;
    std::cout << "GimbalSmall2dPlugin::Load joint " << jointName << " not found, trying again with scoped joint name = " << scopedJointName << "\n";
    this->tiltJoint = this->model->GetJoint(scopedJointName);
  }

  // can't get joint
  if (!this->tiltJoint){
    std::cout << "GimbalSmall2dPlugin::Load ERROR! Can't get joint '" << jointName << "' " << std::endl;
  }
  
}

void GimbalSmall2dPlugin::Init(){

  // create a new node
//  this->node = transport::NodePtr(new transport::Node());

  // node init
//  this->node->Init(this->model->GetWorld()->GetName());

  // update sim time
#if GAZEBO_MAJOR_VERSION < 8
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
#else
  this->lastUpdateTime = this->model->GetWorld()->SimTime();
#endif

  //create a subscriber to get gimbal angle
  std::stringstream gimbalOrientationTopic;
  gimbalOrientationTopic << "/";

  gimbalOrientationTopic << this->model->GetName();

  gimbalOrientationTopic << "/gimbal/set_orientation";
  this->gimbalOrientationSubscriber = this->nodeHandle.subscribe(gimbalOrientationTopic.str(), 1, &GimbalSmall2dPlugin::gimbalOrientationCallback, this);

  // drone IMU topic
  std::stringstream droneIMUTopic;

  droneIMUTopic << std::string("/") <<  this->model->GetName() << "/mavros/imu/data";

  this->droneIMUSubscriber = this->nodeHandle.subscribe(droneIMUTopic.str(), 1, &GimbalSmall2dPlugin::droneIMUCallback, this);

  // create a ros loop thread
  this->rosLoopThread = new std::thread(&GimbalSmall2dPlugin::rosLoop, this);

//  //gazebo camera IMU topic
//  std::string cameraIMUTopic = std::string("~/") +  this->model->GetName() + "/tilt_link/camera_imu_sensor/imu";
//  this->cameraIMUSubscriber = this->node->Subscribe(cameraIMUTopic, &GimbalSmall2dPlugin::cameraIMUCallback, this);

  // bind update function and store the connection
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(std::bind(&GimbalSmall2dPlugin::OnUpdate, this)));

}

//void GimbalSmall2dPlugin::cameraIMUCallback(ConstIMUPtr &msg){
//  gazebo::msgs::Quaternion q = msg->orientation();
//  gazebo::math::Quaternion qm = gazebo::math::Quaternion(q.w(), q.x(), q.y(), q.z());
//  gazebo::math::Vector3 v = qm.GetAsEuler();
//// std::cout << "reading imu camera x " << v.x << std::endl;
////  std::cout << "reading imu camera y " << v.y << std::endl;
////  std::cout << "reading imu camera z " << v.z << std::endl << std::endl;

//  this->currentCameraIMUPitch = v.x; // radians

//}

void GimbalSmall2dPlugin::droneIMUCallback(const sensor_msgs::Imu::ConstPtr &msg){

//    gazebo::math::Quaternion qm = gazebo::math::Quaternion(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
//    gazebo::math::Vector3 v = qm.GetAsEuler();
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

//  std::cout << "reading imu gimbal x " << roll << std::endl;
//  std::cout << "reading imu gimbal y " << pitch << std::endl;
//  std::cout << "reading imu gimbal z " << yaw << std::endl << std::endl;

    this->currentDroneIMUPitch = pitch; // radians

}

void GimbalSmall2dPlugin::OnUpdate(){

  // update ROS information
  //ros::spinOnce();

  // get simulatio time
#if GAZEBO_MAJOR_VERSION < 8
  common::Time time = this->model->GetWorld()->GetSimTime();
#else
  common::Time time = this->model->GetWorld()->SimTime();
#endif

  // verify all required elements
  if (!this->tiltJoint || (time < this->lastUpdateTime)){
    this->lastUpdateTime = time;
    return;
  }

  // calculate tilt PID parameters
  double dt = (this->lastUpdateTime - time).Double();
  double currentIMU = this->currentDroneIMUPitch;

#if GAZEBO_MAJOR_VERSION < 8
  double error = this->tiltJoint->GetAngle(0).Radian() - (currentIMU - this->lastDroneIMUPitch) - multi_uav::utils::Math::degreesToRadians(gimbalOrientation.pitch);
#else
  double error = this->tiltJoint->Position(0) - (currentIMU - this->lastDroneIMUPitch) - multi_uav::utils::Math::degreesToRadians(gimbalOrientation.pitch);
#endif


//  std::cout << "e = " << e << std::endl;
//  std::cout << "error = " << error << std::endl;
//  std::cout << "joint = " << this->tiltJoint->GetAngle(0).Radian() << std::endl;
//  std::cout << "wish = " << wish << std::endl;

//  system("clear");

  // update tilt control
  double force = this->pid.Update(error, dt);
  
  // apply force on pitch joint
  this->tiltJoint->SetForce(0, force);

  // store last data
  this->lastUpdateTime = time;
  this->lastDroneIMUPitch = currentIMU;
  //this->lastWishCameraPitch = wish;

}

}
