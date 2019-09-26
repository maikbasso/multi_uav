/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef STRUCTURES_H
#define STRUCTURES_H

// c/c++
#include <string>

// opencv
#include <opencv2/opencv.hpp>

namespace multi_uav{

typedef struct{
  double x;
  double y;
  double z;
}VEHICLE_POSITION_LOCAL;

typedef struct{
  double roll;
  double pitch;
  double yaw;
}VEHICLE_ORIENTATION_LOCAL;

typedef struct{
  double latitude;
  double longitude;
  double altitude;
}VEHICLE_POSITION_GLOBAL;

typedef struct{
  double yaw;
}VEHICLE_ORIENTATION_GLOBAL;

typedef struct{
  VEHICLE_POSITION_LOCAL local;
  VEHICLE_POSITION_GLOBAL global;
}VEHICLE_POSITION;

typedef struct{
  VEHICLE_ORIENTATION_LOCAL local;
  VEHICLE_ORIENTATION_GLOBAL global;
}VEHICLE_ORIENTATION;

typedef struct{
  double voltage; // Voltage in Volts (Mandatory)
  double current; // Current charge in Ah  (If unmeasured NaN)
  double percentage; // Charge percentage on 0 to 1 range  (If unmeasured NaN)
  bool present; // True if the battery is present
}VEHICLE_BATTERY;

typedef struct{
  int id;
  bool connected;
  bool armed;
  bool guided;
  std::string mode;
  VEHICLE_BATTERY battery;
  VEHICLE_POSITION position;
  VEHICLE_ORIENTATION orientation;
}DRONE_PARAMETERS;

typedef struct{
  std::string name;
  cv::Mat image;
}DRONE_CAMERA_RGB;

typedef struct{
  DRONE_CAMERA_RGB rgb;
}DRONE_CAMERA;

typedef struct{
  DRONE_CAMERA camera;
}DRONE_SENSORS;

typedef struct{
  double roll;
  double pitch;
  double yaw;
}VEHICLE_GIMBAL;

typedef struct{
  VEHICLE_GIMBAL gimbal;
}DRONE_HARDWARE;

}

#endif // STRUCTURES_H
