#pragma once

#include <Eigen/Dense>


// TODO: Add parsing support for storing quadruped parameters in a file
struct QuadrupedParams {
    Eigen::Matrix3d inertiaTensor;
    const double mass;
    const double g; // Gravity vector

    // Scalar limit, kept for callers that want a single conservative number.
    const double torque_limit;

    // Per-joint torque limits, indexed [hip, thigh, calf] and applied to every
    // leg. The Go2's joints are NOT identical: the MuJoCo model gives the hip
    // (abduction) and thigh actuators +/-23.7 Nm and only the calf (knee)
    // +/-45.43 Nm. Clamping all twelve joints at 45 lets the controller command
    // nearly twice what the hip and thigh can actually deliver, so the commanded
    // torque and the achieved torque diverge silently.
    const Eigen::Vector3d joint_torque_limits;

    /** Torque limit for a joint within a leg. @param joint_idx 0=hip, 1=thigh, 2=calf. */
    double joint_torque_limit(int joint_idx) const { return joint_torque_limits[joint_idx]; }

    const double mu; // Static coefficient of friction between foot and ground

    // Reflected rotor inertia per leg joint, kg*m^2. go2.xml sets
    // armature="0.01" on every leg joint; the URDF carries no such term, so a
    // mass matrix built from the URDF alone understates the inertia the plant
    // actually has. Used by get_foot_operational_space_inertia(), where omitting
    // it made Lambda 70% low against MuJoCo's own mj_fullM. On hardware this is
    // the motor-plus-gearbox inertia seen at the joint.
    double joint_armature = 0.01;


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
    QuadrupedParams(const Eigen::Matrix3d& inertiaTensor, const double& mass, const double& gravity,
                    const double& torque_limit, const double& mu,
                    const Eigen::Vector3d& joint_torque_limits = Eigen::Vector3d(23.7, 23.7, 45.43))
        : inertiaTensor(inertiaTensor), mass(mass), g(gravity), torque_limit(torque_limit),
          joint_torque_limits(joint_torque_limits), mu(mu) {};
};