#ifndef BASE_CPP
#define BASE_CPP

#include <Eigen/Eigen>

struct Base{
    Eigen::Vector3d coordinates;
    bool is_visited = false;
};

#endif