#pragma once
#include <Eigen/Dense>
#include "state_space.hpp"


using namespace Eigen;

StateSpace quadruped_state_space_continuous(const double& yaw, const Eigen::Matrix<double,3,4>& foot_levers_W, const Eigen::Matrix3d& I_b, const double& mass);
StateSpace quadruped_state_space_discrete(const double& yaw, const Eigen::Matrix<double,3,4>& foot_levers_W, const Eigen::Matrix3d& I_b, const double& mass, const double& dt);
