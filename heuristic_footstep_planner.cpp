#include "heuristic_footstep_planner.hpp"
#include <Eigen/Dense>
using namespace Eigen;

Eigen::Vector3f desired_footstep_position(const Vector3f& p_ref, const Vector3f& v_CoM, const double& dt)
{
    // Define the desired footstep position based on the reference CoM position and velocity
    Vector3f p_des;
    p_des = p_ref + v_CoM*dt/2;

    return p_des;
}