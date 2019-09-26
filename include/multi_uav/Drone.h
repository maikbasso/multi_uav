/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef MULTI_UAV_DRONE_H
#define MULTI_UAV_DRONE_H

// c/c++
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

// ros
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>

// opencv
#include <opencv2/opencv.hpp>

// cv bridge
#include <cv_bridge/cv_bridge.h>

// own
#include <multi_uav/utils/Math.h>
#include <multi_uav/utils/GlobalPosition.h>
#include <multi_uav/Structures.h>
#include <multi_uav/RPY.h>

namespace multi_uav{

class Drone {

	private:
		// ros
    ros::NodeHandle nodeHandle;
		ros::Subscriber stateSubscriber;
		ros::Subscriber globalPositionSubscriber;
		ros::Subscriber localPositionPoseSubscriber;
    ros::Subscriber globalPositionCompassHdg;
    ros::Subscriber cameraRGBSubscriber;
    ros::Subscriber batteryStateSubscriber;
    ros::ServiceClient serviceClientSetMode;
    ros::ServiceClient serviceClientTakeOff;
    ros::ServiceClient serviceClientArming;
    ros::ServiceClient serviceClientLand;
    ros::Publisher publisherSetPositionLocal;
    ros::Publisher publisherSetPositionGlobal;
    ros::Publisher publisherGimbalOrientation;

//    // subscriber is ready
//    bool stateSubscriberIsOK;
//    bool globalPositionSubscriberIsOK;
//    bool localPositionPoseSubscriberIsOK;
//    bool globalPositionCompassHdgIsOK;
//    bool cameraRGBSubscriberIsOK;
//    bool batteryStateSubscriberIsOK;

		//drone
		bool waitFCUConnection();
		bool setMode(std::string mode);
    bool setModeWithTimeout(std::string mode);

		// others
		int droneNumber;
		double commandTimeoutSeconds;
    bool useLocalCoordinates;
    double globalYawInitRemap;
    bool debugMode;
    std::vector<std::thread*> threads;
		std::string formatTopicName(std::string topicName);
    void initParameters();
    void initSensors();
    void initHardwareParameters();
		void initRosSubscribers();
    void initRosServiceClients();
    void initRosPublishers();
    void initThreads();
		void print(std::string txt);
    bool isAngleBetweenLimits(double angle, double angleTarget, double offset, double limitLow, double limtHigh);

	public:
    Drone(ros::NodeHandle nodeHandle, int droneNumber, bool debugMode);
		~Drone();

    // global parameters
    DRONE_PARAMETERS parameters;
    void printParameters();

    // global sensors data
    DRONE_SENSORS sensors;

    // optional hardware
    DRONE_HARDWARE hardware;

		//configs
    void setCommandTimeoutSeconds(double timeout);
    void configureToUseLocalCoordinates();
    void configureToUseGlobalCoordinates();
    void setGimbalOrientation(double pitch);

		// drone		
		bool setModeStabilized();
		bool setModeLoiter();
		bool setModeOffboard();
		bool setModeRTL();
		bool setModeLand();
		bool setModeMission();

    void forceModeOffboard();

		bool arm();
    bool disarm();
		bool takeOff(double metters);
    bool land();

    bool goToGlobalPosition(double latitude, double longitude, double altitude, double yaw, bool wait);
    bool goToLocalPosition(double x, double y, double z, double theta, bool wait);

		//callbacks
		void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
		void mavrosGlobalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void mavrosLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mavrosglobalPositionCompassHdgCallback(const std_msgs::Float64::ConstPtr& msg);
    void cameraRGBCallback(const sensor_msgs::Image::ConstPtr& msg);
    void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

    //publisher
//    void gimbalOrientationPublisher();

    //other
    cv::Mat getOSDImage();
};

}

#endif // MULTI_UAV_DRONE_H
