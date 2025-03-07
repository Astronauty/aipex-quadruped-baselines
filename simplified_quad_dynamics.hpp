#pragma once
#include <Eigen/Dense>
#include "state_space.hpp"


using namespace Eigen;

StateSpace quadruped_state_space_continuous(const double&  yaw, Matrix<double, 4, 3>& foot_positions);