/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef MULTI_UAV_UTILS_GLOBALPOSITION_H
#define MULTI_UAV_UTILS_GLOBALPOSITION_H

#include <math.h>

namespace multi_uav{

namespace utils{

class GlobalPosition {
private:
  double latitude;
  double longitude;
  double altitude;
  double yaw;

public:
  GlobalPosition(double latitude, double longitude, double altitude, double yaw);

  double getLatitude();
  double getLongitude();
  double getAltitude();
  double getYaw();

  void setLatitude(double latitude);
  void setLongitude(double longitude);
  void setAltitude(double altitude);
  void setYaw(double yaw);

  void addPositionOffsetInMeters(double distanceNorth, double distanceEast);
  void addMetersToAltitude(double meters);
  void addDegreesToYaw(double degrees);

};

}
}

#endif // MULTI_UAV_UTILS_GLOBALPOSITION_H
