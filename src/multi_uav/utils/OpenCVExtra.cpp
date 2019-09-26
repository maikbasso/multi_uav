/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <multi_uav/utils/OpenCVExtra.h>

namespace multi_uav{

namespace utils{

OpenCVExtra::OpenCVExtra(){
  // do nothing
}

void OpenCVExtra::saveMatAsPNG(std::string path, cv::Mat mat){
  cv::imwrite( path, mat );
}

void OpenCVExtra::saveMatAsFile(cv::Mat mat, std::string path){
  cv::FileStorage file(path, cv::FileStorage::WRITE);
  // Write to file!
  file << "matrix" << mat;
}

cv::Mat OpenCVExtra::loadFileAsMat(std::string path){
  
  cv::FileStorage file(path, cv::FileStorage::WRITE);

  cv::FileNode fn = file["matrix"];
  cv::Mat matrix = fn.mat();

  return matrix;
}

}

}
