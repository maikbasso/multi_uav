/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <multi_uav/utils/GlobalPosition.h>

namespace multi_uav{

namespace utils{

GlobalPosition::GlobalPosition(double latitude, double longitude, double altitude, double yaw){
  this->latitude = latitude;
  this->longitude = longitude;
  this->altitude = altitude;
  this->yaw = yaw;
}

double GlobalPosition::getLatitude(){
  return this->latitude;
}

double GlobalPosition::getLongitude(){
  return this->longitude;
}

double GlobalPosition::getAltitude(){
  return this->altitude;
}

double GlobalPosition::getYaw(){
  return this->yaw;
}

void GlobalPosition::setLatitude(double latitude){
  this->latitude = latitude;
}

void GlobalPosition::setLongitude(double longitude){
  this->longitude = longitude;
}

void GlobalPosition::setAltitude(double altitude){
  this->altitude = altitude;
}

void GlobalPosition::setYaw(double yaw){
  this->yaw = yaw;
}

void GlobalPosition::addPositionOffsetInMeters(double distanceNorth, double distanceEast){
  //Radius of "spherical" earth
  double earthRadius = 6378137.0;
  //Coordinate offsets in radians
  double dLat = distanceNorth/earthRadius;
  double dLon = distanceEast/(earthRadius * cos(M_PI*this->latitude/180.0));
  //New position in decimal degrees
  this->latitude = this->latitude + (dLat * 180.0/M_PI);
  this->longitude = this->longitude + (dLon * 180.0/M_PI);
}

void GlobalPosition::addMetersToAltitude(double meters){
  this->altitude += meters;
}

void GlobalPosition::addDegreesToYaw(double degrees){
  this->yaw += degrees;

//  if(this->yaw > 360.0){
//    while(this->yaw > 360.0){
//      this->yaw -= 360.0;
//    }
//  }
//  else if(this->yaw < 0.0){
//    while(this->yaw < 0.0){
//      this->yaw = 360.0 - this->yaw;
//    }
//  }

    if(this->yaw > 360.0){
      this->yaw = 360.0;
    }
    else if(this->yaw < 0.0){
      this->yaw = 0.0;
    }
}

}

}
