#pragma once
#include <Eigen/Dense>
using namespace Eigen;

Matrix3f eul2rotm(double roll, double pitch, double yaw);
Matrix3f hatMap(const Eigen::Vector3f& a);