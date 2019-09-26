/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <multi_uav/utils/Math.h>

namespace multi_uav{

namespace utils{

Math::Math(){

}

Math::~Math(){

}

float Math::radiansToDegrees(float radians){
  return radians * (180.0/M_PI);
}

double Math::radiansToDegrees(double radians){
  return radians * (180.0/M_PI);
}

float Math::degreesToRadians(float degrees){
  return degrees * M_PI/180.0;
}

double Math::degreesToRadians(double degrees){
  return degrees * M_PI/180.0;
}

int Math::map(int value, int low1, int high1, int low2, int high2){
  return (int) (low2 + (value - low1) * (high2 - low2) / (high1 - low1));
}

float Math::map(float value, float low1, float high1, float low2, float high2){
  return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
}

double Math::map(double value, double low1, double high1, double low2, double high2){
  return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
}

}

}
