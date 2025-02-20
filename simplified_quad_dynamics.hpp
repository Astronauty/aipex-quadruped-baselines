#pragma once
#include <Eigen/Dense>
#include "state_space.hpp"


using namespace Eigen;

StateSpace quadruped_state_space_continuous(const float&  yaw, Matrix<float, 4, 3>& foot_positions);