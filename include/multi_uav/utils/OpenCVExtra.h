#ifndef MULTI_UAV_UTILS_OPENCVEXTRA_H
#define MULTI_UAV_UTILS_OPENCVEXTRA_H

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace multi_uav{

namespace utils{

class OpenCVExtra{
private:
public:
  OpenCVExtra();
  void saveMatAsPNG(std::string path, cv::Mat mat);
  void saveMatAsFile(cv::Mat mat, std::string path);
  cv::Mat loadFileAsMat(std::string path);
};

}
}

#endif // MULTI_UAV_UTILS_OPENCVEXTRA_H
