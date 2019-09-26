/*
@author Lucas Grazziotim <lucasgrazziotim@gmail.com>
*/

#ifndef MULTI_UAV_FORMATION_H
#define MULTI_UAV_FORMATION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <multi_uav/Drone.h>
#include <multi_uav/utils/Math.h>
#include <thread>
#include <vector>
#include <termios.h>
#include <string.h>

#define OFFSETORIGIN 1.0
#define TAKEOFFHEIGHT 2.0

//  Joystick Mode offset comands
#define MOVEJM 1
#define ROTATEJM 15
#define DISTANCEJM 1
#define APERTUREJM 10
#define UPDOWNJM 1

namespace multi_uav{


struct position{
  double x;
  double y;
  double z;
  double theta;
};

class Formation{

 private:
  int numDrones;
  double xmoved;
  double ymoved;
  double zmoved;
  double moveX;
  double moveY;

  double droneDistance;
  double apertureAngle;
  double yawAngle;
  double height;
  int currentFormation;

  bool waitEnable;

  std::vector<multi_uav::Drone*> drones;
  std::vector<position> posDrones;


  int getch();
  void printCoordinates();  
  void printJoystickControls();
  void printJoystickCommands(int command);

  void moveJoystick            (int command);
  void distanceApertureJoystick(int command);
  void rotateJoystick          (int command); 

  void updateFormation();


 public:
  Formation(std::vector<multi_uav::Drone*> drones);
  ~Formation();

  void printCurrentFormationInfo();
  std::vector<multi_uav::Drone*> getDrones();

  void setWaitEnable(bool enable);

  void circle(double droneDistance,                       double yawAngle, double height);
  void cross (double droneDistance, double apertureAngle, double yawAngle, double height);
  void arrow (double droneDistance, double apertureAngle, double yawAngle, double height);
  void line  (double droneDistance,                       double yawAngle, double height);

  void joystickMode();
  void droneLocalControl(int droneNumber );


  ////////////////////////////////////  Auto Functions  ////////////////////////////////////
  void moveAuto                 (double distance, int command);
  void moveForwardFormation     (double distance);
  void moveBackwardFormation    (double distance);
  void moveLeftFormation        (double distance);
  void moveRightFormation       (double distance);
  void moveUpFormation          (double distance);
  void moveDownFormation        (double distance);
  void setDroneDistanceFormation(double droneDistance);
  void setApertureAngleFormation(double apertureAngle);
  void setYawAngleFormation     (double yawAngle);

};


}
#endif // MULTI_UAV_FORMATION_H
