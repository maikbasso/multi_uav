/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <multi_uav/Drone.h>

namespace multi_uav{

Drone::Drone(ros::NodeHandle nodeHandle, int droneNumber, bool debugMode){

  // node handle
  this->nodeHandle = nodeHandle;

  // save parameters
  this->droneNumber = droneNumber;

  // enable console messages
  this->debugMode = debugMode; 

  // initial global yaw
  this->globalYawInitRemap = 0.0;

  // default values of parameters
  this->setCommandTimeoutSeconds(45.0);

  // by default use local coordinates
  this->configureToUseLocalCoordinates();

  // init drone parameters
  this->initParameters();

  // init sensors
  this->initSensors();

  // init subscribers
  this->initRosSubscribers();

  // init service clients
  this->initRosServiceClients();

  // init publishers
  this->initRosPublishers();

  // ros data initialization
  int count = 40;
  ros::Rate rate(20.0);
  while(ros::ok() && count > 0){
    count--;
    ros::spinOnce();
    rate.sleep();
  }

  //FCU Connection
  this->waitFCUConnection();

}

Drone::~Drone(){

  // subscribers
  this->stateSubscriber.shutdown();
  this->globalPositionSubscriber.shutdown();
  this->localPositionPoseSubscriber.shutdown();
  this->globalPositionCompassHdg.shutdown();
  this->cameraRGBSubscriber.shutdown();
  this->batteryStateSubscriber.shutdown();

  // service clients
  this->serviceClientSetMode.shutdown();
  this->serviceClientArming.shutdown();
  this->serviceClientLand.shutdown();
  this->serviceClientTakeOff.shutdown();

  // publishers
  this->publisherSetPositionLocal.shutdown();
  this->publisherSetPositionGlobal.shutdown();

  this->print("success on shutdown drone interface");
}

void Drone::configureToUseLocalCoordinates(){
  this->useLocalCoordinates = true;
  this->print("using local coordinates");
}

void Drone::configureToUseGlobalCoordinates(){
  this->useLocalCoordinates = false;
  this->globalYawInitRemap = multi_uav::utils::Math::map(this->parameters.orientation.global.yaw, 0.0, 360.0, -180.0, 180.0);
  this->print("using global coordinates");
}

void Drone::setCommandTimeoutSeconds(double timeout){
  this->commandTimeoutSeconds = timeout;
  std::stringstream ss;
  ss << "command timeout is set to ";
  ss << this->commandTimeoutSeconds;
  ss << " seconds";
  this->print(ss.str());
}

void Drone::print(std::string text){
  if(this->debugMode){
    std::cout << "DRONE " << this->droneNumber << ": " << text << "." << std::endl;
  }
}

std::string Drone::formatTopicName(std::string topicName){
  std::stringstream tn;
  tn << "/uav";
  tn << this->droneNumber;
  tn << topicName;
  //this->print(tn.str());
  return tn.str();
}

void Drone::initSensors(){
  this->print("initializing sensors data");

  DRONE_CAMERA_RGB rgbCamera;
  rgbCamera.name = "Camera RGB";
  rgbCamera.image = cv::Mat::zeros(200, 200, CV_8UC3); //blank image

  DRONE_CAMERA camera;
  camera.rgb = rgbCamera;

  DRONE_SENSORS sensors;
  sensors.camera = camera;

  this->sensors = sensors;

  this->print("sensors OK");
}

void Drone::initParameters(){
  this->print("initializing parameters");

  VEHICLE_POSITION_LOCAL posLocal;
  posLocal.x = 0.0;
  posLocal.y = 0.0;
  posLocal.z = 0.0;

  VEHICLE_POSITION_GLOBAL posGlobal;
  posGlobal.latitude = 0.0;
  posGlobal.longitude = 0.0;
  posGlobal.altitude = 0.0;

  VEHICLE_ORIENTATION_LOCAL orienLocal;
  orienLocal.yaw = 0.0;

  VEHICLE_ORIENTATION_GLOBAL orienGlobal;
  orienGlobal.yaw = 0.0;

  VEHICLE_POSITION pos;
  pos.local = posLocal;
  pos.global = posGlobal;

  VEHICLE_ORIENTATION orien;
  orien.local = orienLocal;
  orien.global = orienGlobal;

  VEHICLE_BATTERY battery;
  battery.voltage = 0.0;
  battery.current = 0.0;
  battery.percentage = 0.0;
  battery.present = false;

  DRONE_PARAMETERS parameters;
  parameters.id = this->droneNumber;
  parameters.connected = false;
  parameters.armed = false;
  parameters.guided = false;
  parameters.mode = "";
  parameters.position = pos;
  parameters.orientation = orien;
  parameters.battery = battery;

  this->parameters = parameters;

  this->print("parameters OK");
}

void Drone::printParameters(){
  std::stringstream ss;
  ss << std::fixed;
  ss << std::endl;
  ss << "==========================================" << std::endl;
  ss << "DRONE " << this->droneNumber << std::endl;
  ss << "==========================================" << std::endl;
  ss << "connected: " << this->parameters.connected << std::endl;
  ss << "armed: " << this->parameters.armed << std::endl;
  ss << "mode: " << this->parameters.mode << std::endl;
  ss << "guided: " << this->parameters.guided << std::endl;
  ss << std::endl;
  ss << "voltage: " << this->parameters.battery.voltage << std::endl;
  ss << "current: " << this->parameters.battery.current << std::endl;
  ss << "percentage: " << this->parameters.battery.percentage << std::endl;
  ss << "present: " << this->parameters.battery.present << std::endl;
  ss << std::endl;
  ss << "position.global.latitude: " << this->parameters.position.global.latitude << std::endl;
  ss << "position.global.longitude: " << this->parameters.position.global.longitude << std::endl;
  ss << "position.global.altitude: " << this->parameters.position.global.altitude << std::endl;
  ss << "orientation.global.yaw: " << this->parameters.orientation.global.yaw << std::endl;
  ss << std::endl;
  ss << "position.relative.x: " << this->parameters.position.local.x << std::endl;
  ss << "position.relative.y: " << this->parameters.position.local.y << std::endl;
  ss << "position.relative.z: " << this->parameters.position.local.z << std::endl;
  ss << "orientation.relative.yaw: " << this->parameters.orientation.local.yaw << std::endl;
  ss << "==========================================" << std::endl;
  ss << std::endl;
  std::cout << ss.str();
}

cv::Mat Drone::getOSDImage(){
  cv::Mat temp = this->sensors.camera.rgb.image.clone();

  cv::Scalar color = cv::Scalar(255,255,255);
  double fontSize = 1.0;
  int width = temp.size[1];
  int height = temp.size[0];

#if CV_MAJOR_VERSION > 4
  int lineType = CV_AA;
#else
  int lineType = cv::LINE_AA;
#endif

  std::stringstream ssname;
  ssname << "Drone " << this->parameters.id;
  std::string name = ssname.str();
  cv::putText(temp, name.c_str(), cv::Point(30, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, fontSize, color, 1, lineType);

  std::stringstream ssbattery;
  ssbattery << "Battery: " << (int)(this->parameters.battery.percentage * 100.0) << "%";
  std::string battery = ssbattery.str();
  cv::putText(temp, battery.c_str(), cv::Point(width-200, 30), cv::FONT_HERSHEY_COMPLEX_SMALL, fontSize, color, 1, lineType);

  std::stringstream ssmode;
  ssmode << "FCU: " << this->parameters.connected << " ";
  ssmode << "Fly mode: " << this->parameters.mode << " ";
  ssmode << "Armed: " << this->parameters.armed << " ";
  ssmode << "Guided: " << this->parameters.guided << " ";
  std::string mode = ssmode.str();
  cv::putText(temp, mode.c_str(), cv::Point(200, height-100), cv::FONT_HERSHEY_COMPLEX_SMALL, fontSize, color, 1, lineType);

  std::stringstream ssxyz;
  ssxyz << std::fixed;
  ssxyz << std::setprecision(2) << "x: " << this->parameters.position.local.x << " ";
  ssxyz << std::setprecision(2) << "y: " << this->parameters.position.local.y <<  " ";
  ssxyz << std::setprecision(2) << "z: " << this->parameters.position.local.z <<  " ";
  ssxyz << std::setprecision(2) << "roll: " << this->parameters.orientation.local.roll <<  " ";
  ssxyz << std::setprecision(2) << "pitch: " << this->parameters.orientation.local.pitch <<  " ";
  ssxyz << std::setprecision(2) << "yaw: " << this->parameters.orientation.local.yaw;
  std::string xyz = ssxyz.str();
  cv::putText(temp, xyz.c_str(), cv::Point(160, height-60), cv::FONT_HERSHEY_COMPLEX_SMALL, fontSize, color, 1, lineType);

  std::stringstream ssgps;
  ssgps << std::fixed;
  ssgps << std::setprecision(8) << "lat: " << this->parameters.position.global.latitude << " ";
  ssgps << std::setprecision(8) << "long: " << this->parameters.position.global.longitude <<  " ";
  ssgps << std::setprecision(2) << "alt: " << this->parameters.position.global.altitude << " ";
  ssgps << std::setprecision(2) << "yaw: " << this->parameters.orientation.global.yaw;
  std::string gps = ssgps.str();
  cv::putText(temp, gps.c_str(), cv::Point(150, height-20), cv::FONT_HERSHEY_COMPLEX_SMALL, fontSize, color, 1, lineType);

  return temp;
}

void Drone::mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg){
  mavros_msgs::State current = *msg;
  this->parameters.connected = current.connected;
  this->parameters.mode = current.mode;
  this->parameters.armed = current.armed;
  this->parameters.guided = current.guided;

  //this->stateSubscriberIsOK = true;
}

void Drone::mavrosGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  sensor_msgs::NavSatFix current = *msg;
  this->parameters.position.global.latitude = current.latitude;
  this->parameters.position.global.longitude = current.longitude;
  this->parameters.position.global.altitude = current.altitude;

  //this->globalPositionSubscriberIsOK = true;
}

void Drone::mavrosLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  geometry_msgs::PoseStamped current = *msg;

  //position
  this->parameters.position.local.x = current.pose.position.x;
  this->parameters.position.local.y = current.pose.position.y;
  this->parameters.position.local.z = current.pose.position.z;

  // orientation
  //tf::Pose pose;
  //tf::poseMsgToTF(current.pose, pose);

  //double roll = multi_uav::utils::Math::radiansToDegrees(tf::getYaw(pose.getRotation()));
  //double pitch = multi_uav::utils::Math::radiansToDegrees(tf::getYaw(pose.getRotation()));
  //double yaw = multi_uav::utils::Math::radiansToDegrees(tf::getYaw(pose.getRotation()));

  tf::Quaternion q(current.pose.orientation.x, current.pose.orientation.y, current.pose.orientation.z, current.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);

  this->parameters.orientation.local.roll = multi_uav::utils::Math::radiansToDegrees(roll);
  this->parameters.orientation.local.pitch = multi_uav::utils::Math::radiansToDegrees(pitch);
  this->parameters.orientation.local.yaw = multi_uav::utils::Math::radiansToDegrees(yaw);

  //this->localPositionPoseSubscriberIsOK = true;
}


void Drone::mavrosglobalPositionCompassHdgCallback(const std_msgs::Float64::ConstPtr& msg){
  std_msgs::Float64 current = *msg;

  // global yaw
  double yaw = (double) current.data;
  this->parameters.orientation.global.yaw = yaw; // fix compass orientation on model

  //this->globalPositionCompassHdgIsOK = true;
}

void Drone::cameraRGBCallback(const sensor_msgs::Image::ConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  this->sensors.camera.rgb.image = cv_ptr->image;
}

void Drone::batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg){
  sensor_msgs::BatteryState current = *msg;

  this->parameters.battery.voltage = current.voltage;
  this->parameters.battery.current = current.current;
  this->parameters.battery.percentage = current.percentage;
  this->parameters.battery.present = current.present;

}

void Drone::initRosSubscribers(){

  this->print("initializing ROS subscribers");

  // state
  this->stateSubscriber = this->nodeHandle.subscribe<mavros_msgs::State>(this->formatTopicName("/mavros/state"), 1, &Drone::mavrosStateCallback, this);
  // global position
  this->globalPositionSubscriber = this->nodeHandle.subscribe<sensor_msgs::NavSatFix>(this->formatTopicName("/mavros/global_position/raw/fix"), 1, &Drone::mavrosGlobalPositionCallback, this);
  // local position
  this->localPositionPoseSubscriber = this->nodeHandle.subscribe<geometry_msgs::PoseStamped>(this->formatTopicName("/mavros/local_position/pose"), 1, &Drone::mavrosLocalPositionCallback, this);
  // global compass heading graus
  this->globalPositionCompassHdg = this->nodeHandle.subscribe<std_msgs::Float64>(this->formatTopicName("/mavros/global_position/compass_hdg"), 1, &Drone::mavrosglobalPositionCompassHdgCallback, this);
  // camera rbg
  this->cameraRGBSubscriber = this->nodeHandle.subscribe<sensor_msgs::Image>(this->formatTopicName("/camera/rgb/image_raw"), 1, &Drone::cameraRGBCallback, this);
  //battery state
  this->batteryStateSubscriber = this->nodeHandle.subscribe<sensor_msgs::BatteryState>(this->formatTopicName("/mavros/battery"), 1, &Drone::batteryStateCallback, this);

  this->print("subscribers are OK");
}

void Drone::initRosServiceClients(){
  // set mode service client
  this->serviceClientSetMode = this->nodeHandle.serviceClient<mavros_msgs::SetMode>(this->formatTopicName("/mavros/set_mode"));

  // arming service client
  this->serviceClientArming = this->nodeHandle.serviceClient<mavros_msgs::CommandBool>(this->formatTopicName("/mavros/cmd/arming"));

  // takeoff service client
  this->serviceClientTakeOff = this->nodeHandle.serviceClient<mavros_msgs::CommandTOL>(this->formatTopicName("/mavros/cmd/takeoff"));

  // land service client
  this->serviceClientLand = this->nodeHandle.serviceClient<mavros_msgs::CommandTOL>(this->formatTopicName("/mavros/cmd/land"));

}

void Drone::initRosPublishers(){
  // set position local publisher
  this->publisherSetPositionLocal = this->nodeHandle.advertise<geometry_msgs::PoseStamped>(this->formatTopicName("/mavros/setpoint_position/local"), 100);

  // set position global publisher
  this->publisherSetPositionGlobal = this->nodeHandle.advertise<geographic_msgs::GeoPoseStamped>(this->formatTopicName("/mavros/setpoint_position/global"), 100);
}

//void Drone::gimbalOrientationPublisher(){

//  ros::Rate rate(1.0);

//  while (ros::ok()) {
//    // create the message
//    multi_uav::RPY msg;
//    msg.roll = this->hardware.gimbal.roll;
//    //this implementation needs to be performed on gimbal plugin side
//    //double newPitch = this->hardware.gimbal.pitch - this->parameters.orientation.local.pitch;
//    msg.pitch = this->hardware.gimbal.pitch;
//    msg.yaw = this->hardware.gimbal.yaw;
//    //publish
//    this->publisherGimbalOrientation.publish(msg);
//    //sleep
//    rate.sleep();
//  }

//}

bool Drone::waitFCUConnection(){
  this->print("wait FCU connection");
  // wait FCU connection
  ros::Rate rate(20.0);
  while( ros::ok() ) {
    if(this->parameters.connected){
      this->print("FCU connected");
      return true;
    }
    ros::spinOnce();
    rate.sleep();
  }
  this->print("Could not connect to FCU");
  return false;
}

bool Drone::arm(){

  //create a ros commnad
  mavros_msgs::CommandBool armCmd;
  armCmd.request.value = true;

  this->print("arming vehicle");

  // wait for arm
  ros::Time initialTime = ros::Time::now();
  while( ros::ok() ){
    if( !this->parameters.armed && ros::Time::now() - initialTime < ros::Duration(this->commandTimeoutSeconds) ) {
      if(this->serviceClientArming.call(armCmd) && armCmd.response.success){
        this->print("vehicle armed");
        return true;
      }
    }
  }

  this->print("timeout exceeded on arm command");
  return false;
}

bool Drone::disarm(){

  //create a ros commnad
  mavros_msgs::CommandBool armCmd;
  armCmd.request.value = false;

  this->print("disarming vehicle");

  // wait for disarm
  ros::Time initialTime = ros::Time::now();
  while( ros::ok() ){
    if( !this->parameters.armed && ros::Time::now() - initialTime < ros::Duration(this->commandTimeoutSeconds) ) {
      if(this->serviceClientArming.call(armCmd) && armCmd.response.success){
        this->print("vehicle disarmed");
        return true;
      }
    }
  }

  this->print("timeout exceeded on disarm command");
  return false;
}

bool Drone::setMode(std::string mode){

  //create a mavros set mode commnad
  mavros_msgs::SetMode setModeCMD;
  setModeCMD.request.custom_mode = mode;

  std::stringstream ss1;
  ss1 << "setting vehicle mode to ";
  ss1 << mode;
  this->print(ss1.str());

  ros::Rate rate(20.0);
  // wait for change mode
  ros::Time initialTime = ros::Time::now();
  while( ros::ok() ){
    // try at each 5 seconds
    if( this->parameters.mode != mode){// && (ros::Time::now() - initialTime > ros::Duration(5.0)) ) {
      if(this->serviceClientSetMode.call(setModeCMD) && setModeCMD.response.mode_sent){
          //this->print("trying to set vehicle mode");
      }
      initialTime = ros::Time::now();
    }
    else{
      std::stringstream ss2;
      ss2 << "vehicle mode was set to ";
      ss2 << mode;
      this->print(ss2.str());
      return true;
    }
    ros::spinOnce();
    rate.sleep();
  }

  std::stringstream ss3;
  ss3 << "timeout exceeded on set mode to ";
  ss3 << mode;
  this->print(ss3.str());
  return false;
}

bool Drone::setModeWithTimeout(std::string mode){

  //create a mavros set mode commnad
  mavros_msgs::SetMode setModeCMD;
  setModeCMD.request.custom_mode = mode;

  std::stringstream ss1;
  ss1 << "setting vehicle mode to ";
  ss1 << mode;
  this->print(ss1.str());

  ros::Rate rate(20.0);

  // try to change mode for command timeout seconds
  ros::Time initialTime = ros::Time::now();
  while( ros::ok() &&
         this->parameters.mode != mode &&
         this->serviceClientSetMode.call(setModeCMD) &&
         setModeCMD.response.mode_sent &&
         ((ros::Time::now() - initialTime) < ros::Duration(this->commandTimeoutSeconds))
         ){

    ros::spinOnce();
    rate.sleep();
  }

  if( this->parameters.mode == mode){
    std::stringstream ss2;
    ss2 << "vehicle mode was set to ";
    ss2 << mode;
    this->print(ss2.str());
    return true;
  }
  else{
    std::stringstream ss3;
    ss3 << "timeout exceeded on set mode to ";
    ss3 << mode;
    this->print(ss3.str());
    return false;
  }
}

/*
Reference: http://wiki.ros.org/mavros/CustomModes
*/

bool Drone::setModeStabilized(){
  return this->setMode("STABILIZED");
}

bool Drone::setModeLoiter(){
  return this->setMode("AUTO.LOITER");
}

bool Drone::setModeOffboard(){

  //send a few setpoints before starting OFFBOARD mode
  this->print("send a few setpoints before starting");

  if(this->useLocalCoordinates == true){

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    for(int i = 200; ros::ok() && i > 0; --i){
      this->publisherSetPositionLocal.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }

  }
  else{

    geographic_msgs::GeoPoseStamped globalTargetMsg;
    globalTargetMsg.pose.position.latitude = this->parameters.position.global.latitude;
    globalTargetMsg.pose.position.longitude = this->parameters.position.global.longitude;
    globalTargetMsg.pose.position.altitude = this->parameters.position.global.altitude; // in meters, AMSL or above terrain
    tf::Quaternion q = tf::createQuaternionFromYaw(multi_uav::utils::Math::degreesToRadians(this->parameters.orientation.global.yaw));
    globalTargetMsg.pose.orientation.x = q.getX();
    globalTargetMsg.pose.orientation.y = q.getY();
    globalTargetMsg.pose.orientation.z = q.getZ();
    globalTargetMsg.pose.orientation.w = q.getW();

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    for(int i = 200; ros::ok() && i > 0; --i){
      globalTargetMsg.header.stamp = ros::Time::now(); // https://github.com/mavlink/mavros/issues/903
      this->publisherSetPositionGlobal.publish(globalTargetMsg);
      ros::spinOnce();
      rate.sleep();
    }

  }

  // change the fly mode
  return this->setMode("OFFBOARD");
}

void Drone::forceModeOffboard(){
  bool modeWasSet = false;

  this->print("force set mode offboard enabled");

  while (!modeWasSet && ros::ok()) {

    //send a few setpoints before starting OFFBOARD mode
    this->print("send a few setpoints before starting");

    if(this->useLocalCoordinates == true){

      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = 0;
      pose.pose.position.y = 0;
      pose.pose.position.z = 0;

      //the setpoint publishing rate MUST be faster than 2Hz
      ros::Rate rate(50.0);
      for(int i = 200; ros::ok() && i > 0; --i){
        this->publisherSetPositionLocal.publish(pose);
        ros::spinOnce();
        rate.sleep();
      }

    }
    else{

      geographic_msgs::GeoPoseStamped globalTargetMsg;
      globalTargetMsg.pose.position.latitude = this->parameters.position.global.latitude;
      globalTargetMsg.pose.position.longitude = this->parameters.position.global.longitude;
      globalTargetMsg.pose.position.altitude = this->parameters.position.global.altitude; // in meters, AMSL or above terrain
      tf::Quaternion q = tf::createQuaternionFromYaw(multi_uav::utils::Math::degreesToRadians(this->parameters.orientation.global.yaw));
      globalTargetMsg.pose.orientation.x = q.getX();
      globalTargetMsg.pose.orientation.y = q.getY();
      globalTargetMsg.pose.orientation.z = q.getZ();
      globalTargetMsg.pose.orientation.w = q.getW();


      //the setpoint publishing rate MUST be faster than 2Hz
      ros::Rate rate(50.0);
      for(int i = 200; ros::ok() && i > 0; --i){
        globalTargetMsg.header.stamp = ros::Time::now(); // https://github.com/mavlink/mavros/issues/903
        this->publisherSetPositionGlobal.publish(globalTargetMsg);
        ros::spinOnce();
        rate.sleep();
      }

    }

    // change the fly mode and update force condition
    modeWasSet = this->setModeWithTimeout("OFFBOARD");
  }

}

bool Drone::setModeRTL(){
  return this->setMode("AUTO.RTL");
}

bool Drone::setModeLand(){
  return this->setMode("AUTO.LAND");
}

bool Drone::setModeMission(){
  return this->setMode("AUTO.MISSION");
}

bool Drone::takeOff(double meters){

  if(meters < 2.50){
    meters = 3.0;
    this->print("Low altitude. Takeoff altitude set to 3.00");
  }

  //create a mavros commnad
  mavros_msgs::CommandTOL takeoffCMD;
  takeoffCMD.request.altitude = meters;
  takeoffCMD.request.latitude = this->parameters.position.global.latitude;
  takeoffCMD.request.longitude = this->parameters.position.global.longitude;
  takeoffCMD.request.min_pitch = 0.0;
  takeoffCMD.request.yaw = multi_uav::utils::Math::degreesToRadians(this->parameters.orientation.global.yaw);

  this->print("takeoff vehicle");

  // wait for arm
  ros::Time initialTime = ros::Time::now();
  while( ros::Time::now() - initialTime < ros::Duration(this->commandTimeoutSeconds) ) {
    if(this->serviceClientTakeOff.call(takeoffCMD) && takeoffCMD.response.success &&
       this->parameters.position.local.z >= (meters*0.95)){
      this->print("vehicle reached the altitude");
      return true;
    }
  }

  this->print("timeout exceeded on takeoff command");
  return false;
}

bool Drone::land(){

  double landOffset = 0.5;

  //create a mavros commnad
  mavros_msgs::CommandTOL landCMD;
  landCMD.request.altitude = 0.0; // go to land
  landCMD.request.latitude = this->parameters.position.global.latitude;
  landCMD.request.longitude = this->parameters.position.global.longitude;
  landCMD.request.min_pitch = 0.0;
  landCMD.request.yaw = multi_uav::utils::Math::degreesToRadians(this->parameters.orientation.global.yaw);

  this->print("vehicle land");

  // wait for arm
  ros::Time initialTime = ros::Time::now();
  while( ros::Time::now() - initialTime < ros::Duration(this->commandTimeoutSeconds) ) {
    if(this->serviceClientLand.call(landCMD) && landCMD.response.success ||
       (this->parameters.position.local.z <= landOffset && this->parameters.position.local.z >= -landOffset)){
      this->print("vehicle reached the altitude");
      return true;
    }
  }

  this->print("timeout exceeded on land command");
  return false;
}

bool Drone::goToLocalPosition(double x, double y, double z, double yaw, bool wait){

  double posOffset = 0.5;
  double angleOffset = 0.5;

  //create a mavros commnad
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  tf::Quaternion q = tf::createQuaternionFromYaw(multi_uav::utils::Math::degreesToRadians(yaw));
  pose.pose.orientation.x = q.getX();
  pose.pose.orientation.y = q.getY();
  pose.pose.orientation.z = q.getZ();
  pose.pose.orientation.w = q.getW();

  std::stringstream ssgoing;
  ssgoing << std::fixed;
  ssgoing << "vehicle going to ";
  ssgoing << std::setprecision(2) << "x: " << x << ", ";
  ssgoing << std::setprecision(2) << "y: " << y <<  ", ";
  ssgoing << std::setprecision(2) << "z: " << z <<  ", ";
  ssgoing << std::setprecision(2) << "yaw: " << yaw;
  std::string goingTo = ssgoing.str();

  this->print(goingTo);

  if(wait){

    this->print("waiting to vehicle reach the position");

    ros::Rate rate(100.0);
    while(ros::ok()){

      if(
          (this->parameters.position.local.x <= (x+posOffset) && this->parameters.position.local.x >= (x-posOffset)) &&
          (this->parameters.position.local.y <= (y+posOffset) && this->parameters.position.local.y >= (y-posOffset)) &&
          (this->parameters.position.local.z <= (z+posOffset) && this->parameters.position.local.z >= (z-posOffset)) &&
          //(this->parameters.orientation.local.yaw <= (yaw+angleOffset) && this->parameters.orientation.local.yaw >= (yaw-angleOffset))
         this->isAngleBetweenLimits(this->parameters.orientation.local.yaw, yaw, angleOffset, -180.0, 180.0)
          ) {
        this->print("vehicle reached the position");
        return true;
      }
      else{
        //this->print("publishing the new pose");
        this->publisherSetPositionLocal.publish(pose);
      }
      rate.sleep();
    }
  }
  else{
      this->publisherSetPositionLocal.publish(pose);
      return true;
  }

  this->print("timeout exceeded on goToLocalPosition command");
  return false;
}

bool Drone::goToGlobalPosition(double latitude, double longitude, double altitude, double yaw, bool wait){

  double posOffset = 0.5;
  double angleOffset = 5.0;

  //create a mavros commnad
  // http://docs.ros.org/jade/api/geographic_msgs/html/msg/GeoPoseStamped.html
  geographic_msgs::GeoPoseStamped msg;
  msg.pose.position.latitude = latitude;
  msg.pose.position.longitude = longitude;
  msg.pose.position.altitude = altitude; // in meters, AMSL or above terrain

  // wish yaw, convert global yaw to local yaw considering the offset on the initial position
  double wishLocalYaw = this->globalYawInitRemap - multi_uav::utils::Math::map(yaw, 0.0, 360.0, -180.0, 180.0);

  // final yaw
  tf::Quaternion q = tf::createQuaternionFromYaw(multi_uav::utils::Math::degreesToRadians(wishLocalYaw));
  msg.pose.orientation.x = q.getX();
  msg.pose.orientation.y = q.getY();
  msg.pose.orientation.z = q.getZ();
  msg.pose.orientation.w = q.getW();

  std::stringstream ssgoing;
  ssgoing << std::fixed;
  ssgoing << "vehicle going to ";
  ssgoing << std::setprecision(8) << "latitude: " << latitude << ", ";
  ssgoing << std::setprecision(8) << "longitude: " << longitude <<  ", ";
  ssgoing << std::setprecision(2) << "altitude: " << (altitude - this->parameters.position.global.altitude) <<  ", ";
  ssgoing << std::setprecision(2) << "yaw: " << yaw;
  std::string goingTo = ssgoing.str();

  this->print(goingTo);

  if(wait){

    this->print("waiting to vehicle reach the position");

    // offsets
    multi_uav::utils::GlobalPosition *positiveOffset = new multi_uav::utils::GlobalPosition(latitude, longitude, altitude, yaw);
    positiveOffset->addPositionOffsetInMeters(posOffset, posOffset);
    positiveOffset->addMetersToAltitude(posOffset);
    positiveOffset->addDegreesToYaw(angleOffset);

    multi_uav::utils::GlobalPosition *negativeOffset = new multi_uav::utils::GlobalPosition(latitude, longitude, altitude, yaw);
    negativeOffset->addPositionOffsetInMeters(-posOffset, -posOffset);
    negativeOffset->addMetersToAltitude(-posOffset);
    negativeOffset->addDegreesToYaw(angleOffset);

    ros::Rate rate(100.0);
    while(ros::ok()){

//      std::stringstream ssgps;
//      ssgps << std::fixed;
//      ssgps << std::setprecision(12) << "lat: " << negativeOffset->getLatitude() << " < " << this->parameters.position.global.latitude << " > " << positiveOffset->getLatitude() << std::endl;
//      ssgps << std::setprecision(12) << "long: " << negativeOffset->getLongitude() << " < " << this->parameters.position.global.longitude << " > " << positiveOffset->getLongitude() << std::endl;
//      ssgps << std::setprecision(6) << "alt: "  << negativeOffset->getAltitude() << " < " << this->parameters.position.global.altitude << " > " << positiveOffset->getAltitude() << std::endl;
//      ssgps << std::setprecision(6) << "yaw: "  << tlow << " < " << this->parameters.orientation.global.yaw << " > " << thigh << std::endl;
//      ssgps << std::endl;
//      this->print(ssgps.str());

      if(
          (this->parameters.position.global.latitude <= positiveOffset->getLatitude() && this->parameters.position.global.latitude >= negativeOffset->getLatitude()) &&
          (this->parameters.position.global.longitude <= positiveOffset->getLongitude() && this->parameters.position.global.longitude >= negativeOffset->getLongitude()) &&
          (this->parameters.position.global.altitude <= positiveOffset->getAltitude() && this->parameters.position.global.altitude >= negativeOffset->getAltitude()) &&
          this->isAngleBetweenLimits(this->parameters.orientation.global.yaw, yaw, angleOffset, 0.0, 360.0)
         ) {
        this->print("vehicle reached the position");
        return true;
      }
      else{
        //this->print("publishing the new coordinates");
        msg.header.stamp = ros::Time::now(); // https://github.com/mavlink/mavros/issues/903
        this->publisherSetPositionGlobal.publish(msg);
      }
      rate.sleep();
    }
  }
  else{
    msg.header.stamp = ros::Time::now(); // https://github.com/mavlink/mavros/issues/903
    this->publisherSetPositionGlobal.publish(msg);
    return true;
  }

  this->print("timeout exceeded on goToGlobalPosition command");
  return false;

}

bool Drone::isAngleBetweenLimits(double angle, double angleTarget, double offset, double limitLow, double limtHigh){
  double thigh = angleTarget + offset;
  double tlow = angleTarget - offset;

  if(thigh > limtHigh){
    thigh -= 360.0;
  }
  else if(thigh < limitLow){
    thigh += 360.0;
  }

  if(tlow > limtHigh){
    tlow -= 360.0;
  }
  else if(tlow < limitLow){
    tlow += 360.0;
  }

//  std::stringstream ssgps;
//  ssgps << std::fixed;
//  ssgps << std::setprecision(6) << "yaw: "  << tlow << " < " << angle << " > " << thigh << std::endl;
//  this->print(ssgps.str());

  if((thigh > tlow && angle < thigh && angle > tlow) ||
     (thigh < tlow && ( angle < thigh || angle > tlow))){
    return true;
  }

  return false;
}

}
