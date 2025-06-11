// Class for a plane in 3D space
#pragma once
#include<Eigen/Dense>


class Plane
{
    public:
        Plane(Eigen::Vector3d point, Eigen::Vector3d normal);
        ~Plane();
    private:

};
