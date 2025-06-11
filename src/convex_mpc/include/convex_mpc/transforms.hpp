#pragma once
#include <Eigen/Dense>
using namespace Eigen;

Matrix3d eul2rotm(double roll, double pitch, double yaw);
Matrix3d hatMap(const Eigen::Vector3d& a);