#pragma once

#include <Eigen/Dense>


// TODO: Add parsing support for storing quadruped parameters in a file
struct QuadrupedParams {
    Eigen::Matrix3d inertiaTensor;
    const double mass;
    const double g; // Gravity vector

    const double torque_limit;

    // Constructor to initialize parameters
    QuadrupedParams(const Eigen::Matrix3d& inertiaTensor, const double& mass, const double& gravity, const double& torque_limit)
        : inertiaTensor(inertiaTensor), mass(mass), g(gravity), torque_limit(torque_limit) {};
};