/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <multi_uav/utils/SafePrint.h>

namespace multi_uav{

namespace utils{

SafePrint::~SafePrint(){
  std::lock_guard<std::mutex> guard(mutexPrint);
  std::cout << this->str();
}

std::mutex SafePrint::mutexPrint{};

}

}
