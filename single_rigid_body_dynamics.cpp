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


/*
Computes the state derivative x_dot = f(x, u) for a 3D single rigid body. https://arxiv.org/pdf/2012.10002
The state is defined as x = [p p_dot R w_b], where p is the position, p_dot is the velocity, R is the rotation matrix, and w_b is the angular velocity in the body frame.


*/
void srb_dynamics(Eigen::Matrix<float, 18, 1> x, Eigen::Matrix<float, 12, 1> u), Eigen::Matrix<float, 3, 4> p_f)
{
    I = params.inertiaTensor;
    m = params.mass;

    // Extract state variables
    Eigen::Vector3f p = x.block<3, 1>(0, 0);
    Eigen::Vector3f p_dot = x.block<3, 1>(3, 0);
    Eigen::Matrix3f R = x.block<3, 3>(6, 0);
    Eigen::Vector3f w_b = x.block<3, 1>(9, 0);

    // Compute foot positions relative to COM
    Eigen::Matrix<float, 4, 3> p_f; // Rows are foot positions, cols are foot indices
    for (int i=0; i < 4; i++)
    {
        r_i.col(i) = p_f.col(i) - p;
    }

    // Compute applied external wrench


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
