#include "single_rigid_body_dynamics.h"
#include <Eigen/Dense>
#include <iostream>
#using namespace Eigen;


SRBDynamics::SRBDynamics(params)
{
    // Constructor
    this->params = params;
}

SRBDynamics::~SRBDynamics()
{
    // Destructor
}

Eigen::Matrix3d extrinsicRotationMatrix(double roll, double pitch, double yaw) {
    Eigen::Matrix3d rotationMatrix;

    Eigen::Matrix3d rotationX; // Roll (about X-axis)
    rotationX << 1, 0, 0,
                 0, cos(roll), -sin(roll),
                 0, sin(roll), cos(roll);

    Eigen::Matrix3d rotationY; // Pitch (about Y-axis)
    rotationY << cos(pitch), 0, sin(pitch),
                 0, 1, 0,
                 -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d rotationZ; // Yaw (about Z-axis)
    rotationZ << cos(yaw), -sin(yaw), 0,
                 sin(yaw), cos(yaw), 0,
                 0, 0, 1;

    // Extrinsic rotation (ZYX): Yaw * Pitch * Roll
    rotationMatrix = rotationZ * rotationY * rotationX;

    return rotationMatrix;
}

// Function to create the hat map (skew-symmetric matrix) of a 3D vector
Eigen::Matrix3d hatMap(const Eigen::Vector3d& a) {
    Eigen::Matrix3d a_hat;

    v_hat << 0, -a(2), a(1),
             a(2), 0, -a(0),
             -a(1), a(0), 0;

    return a_hat;
}

/*
Computes the state derivative x_dot = f(x, u) for a 3D single rigid body. https://arxiv.org/pdf/2012.10002
The state is defined as x = [p p_dot theta w_b], where p is the position, p_dot is the velocity, theta are extrinsic rpy angles, and w_b is the angular velocity in the body frame.
*/
void srb_dynamics(Eigen::Matrix<float, 12, 1> x, Eigen::Matrix<float, 12, 1> u), Eigen::Matrix<float, 3, 4> p_f)
{
    I = params.inertiaTensor;
    m = params.mass;

    // Extract state variables
    Eigen::Vector3f p = x.block<3, 1>(0, 0);
    Eigen::Vector3f p_dot = x.block<3, 1>(3, 0);
    Eigen::Matrix3f theta = x.block<3, 1>(6, 0);
    Eigen::Vector3f w_b = x.block<3, 1>(9, 0);

    // Compute foot positions relative to COM
    // Eigen::Matrix<float, 4, 3> p_f; // Rows are foot positions, cols are foot indices
    for (int i=0; i < 4; i++)
    {
        r_i.col(i) = p_f.col(i) - p;
    }

    // Compute applied external wrench
    Eigen::Matrix<float, 6, 1> f_ext;
    for (int i=0; i < 4; i++) // Iterate through the force from each foot
    {
        f_ext.block<3, 1>(0, 0) += u.block<3, 1>(i, 0); // Force on rigid body
        f_ext.block<3, 1>(3, 0) += r_i.col(i).cross(u.block<3, 1>(i, 0)); // Torque on rigid body
    }


    // Extract control variables
    Eigen::Vector3f f = u.block<3, 1>(0, 0);
    Eigen::Vector3f tau = u.block<3, 1>(3, 0);

    // Compute the state derivative
    Eigen::Matrix<float, 12, 1> x_dot;
    x_dot.block<3, 1>(0, 0) = p_dot;
    x_dot.block<3, 1>(3, 0) = (1 / params.mass) * f - ;
    x_dot.block<3, 3>(6, 0) = R * skew(w_b);
    x_dot.block<3, 1>(9, 0) = params.inertiaTensor.inverse() * (tau - w_b.cross(params.inertiaTensor * w_b));

    return x_dot;
}
