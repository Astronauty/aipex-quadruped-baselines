#pragma once

#include <Eigen/Dense>


// TODO: Add parsing support for storing quadruped parameters in a file
struct QuadrupedParams {
    Eigen::Matrix3d inertiaTensor;
    const double mass;
    const double g; // Gravity vector

    const double torque_limit;

    const double mu; // Static coefficient of friction between foot and ground


    const unordered_map<std::string, int> LEG_NAME_TO_INDEX = {
        {"FL", 0},
        {"FR", 1},
        {"RL", 2},
        {"RR", 3}
    };

    const unordered_map<int, std::string> LEG_INDEX_TO_NAME = {
        {0, "FL"},
        {1, "FR"},
        {2, "RL"},
        {3, "RR"}
    };


    // Constructor to initialize parameters
    QuadrupedParams(const Eigen::Matrix3d& inertiaTensor, const double& mass, const double& gravity, const double& torque_limit, const double& mu)
        : inertiaTensor(inertiaTensor), mass(mass), g(gravity), torque_limit(torque_limit), mu(mu) {};
};