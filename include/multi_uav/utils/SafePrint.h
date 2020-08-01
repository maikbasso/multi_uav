/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#ifndef MULTI_UAV_UTILS_SAFE_PRINT_H
#define MULTI_UAV_UTILS_SAFE_PRINT_H

#include <iostream>
#include <sstream>
#include <mutex>

namespace multi_uav{

namespace utils{

/*

Thread safe cout class

Exemple of use:

SafePrint{} << "val = " << 33 << std::endl;

*/

class SafePrint: public std::ostringstream
{
public:
    SafePrint() = default;

    ~SafePrint();

private:
    static std::mutex mutexPrint;
};

}
}

#endif // MULTI_UAV_UTILS_SAFE_PRINT_H
