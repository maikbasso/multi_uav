/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef MULTI_UAV_UTILS_MATH_H
#define MULTI_UAV_UTILS_MATH_H

#define _USE_MATH_DEFINES // to use M_PI
#include <math.h>


namespace multi_uav{

namespace utils{

class Math {

  private:

	public:
    Math();
    ~Math();

    static float radiansToDegrees(float radians);
    static double radiansToDegrees(double radians);
    static float degreesToRadians(float degrees);
    static double degreesToRadians(double degrees);

    static int map(int value, int low1, int high1, int low2, int high2);
    static float map(float value, float low1, float high1, float low2, float high2);
    static double map(double value, double low1, double high1, double low2, double high2);

};

}
}

#endif // MULTI_UAV_UTILS_MATH_H
