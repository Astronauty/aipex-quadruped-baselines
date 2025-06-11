#include "plane.h"

#include <iostream>

using namespace std;

Plane::Plane(Eigen::Vector3d point, Eigen::Vector3d normal) 
{
    this->point = point;
    this->normal = normal;
}
