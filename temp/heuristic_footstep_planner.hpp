#pragma once
#include <Eigen/Dense>
using namespace Eigen;

enum GaitType{Trot, Bound, Pace, Pronk};
struct Gait{
    int N_MODES; 
    double duration_s; // Gait duration in seconds_t
}
Eigen::Vector3f desired_footstep_position(const Vector3f& p_ref, const Vector3f& v_CoM, const double& foot_index);
