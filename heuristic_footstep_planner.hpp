#pragma once
#include <Eigen/Dense>
using namespace Eigen;

Eigen::Vector3f desired_footstep_position(const Vector3f& p_ref, const Vector3f& v_CoM, const double& foot_index);
