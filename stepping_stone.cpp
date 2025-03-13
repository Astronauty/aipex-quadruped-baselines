#include "stepping_stone.h"


SteppingStone::SteppingStone(Eigen::Vector3d point, Eigen::Vector3d normal, double width, double height) 
{
    this->width = width;
    this->height = height;


    // Define the halfspace representation of the stepping stone
}

Eigen::Matrix<double, 6, 3> SteppingStone::getConstraints() 
{
    Eigen::Matrix<double, 6, 3> A;
    A << 1, 0, 0,
         -1, 0, 0,
         0, 1, 0,
         0, -1, 0,
         0, 0, 1,
         0, 0, -1;

    return A;
}

// Return matrix s.t. Ax <= b