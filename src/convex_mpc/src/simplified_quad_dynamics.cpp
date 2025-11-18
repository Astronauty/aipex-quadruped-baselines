#include <Eigen/Dense>
#include <iostream>
#include "convex_mpc/simplified_quad_dynamics.hpp"
#include "convex_mpc/transforms.hpp"
#include "convex_mpc/state_space.hpp"

#include <iostream>

using namespace Eigen;
using namespace std;

/* 
Simplified state space quadruped dynamics based on https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf
x = [theta, p, omega, p_dot, g]
*/
StateSpace quadruped_state_space_continuous(const double& yaw, Matrix<double, 3, 4>& foot_positions, const Matrix3d& I_b, const double& mass)
{   
    // Define relevant transforms and inertial properties
    Matrix3d R_z = eul2rotm(0, 0, yaw);
    Matrix3d I_w = R_z*I_b*R_z.transpose(); // Inertia tensor in world body frame for small angles 
    Matrix3d I_w_inv = I_w.inverse(); // Inverse of inertia tensor in world body frame
    // Vector3f g;
    // g << 0, 0, -9.81; // Gravity vector
    
    double g;
    g = 9.81;

    // Define the continuous state space model for a simplified quadruped
    MatrixXd A(13,13);
    A.setZero();
    A.block<3, 3>(0, 6) = R_z.transpose(); // Rotation matrix from world frame to body frame
    A.block<3, 3>(3, 9) = Matrix3d::Identity();
    // A.block<3, 1>(9, 0) = Matrix3d::iden
    A(11, 12) = -1; // Gravity influence on accel
    // A(12, 12) = 1; // Gravity state

    // Matrix<double, 13, 13> B;
    MatrixXd B(13, 12);
    B.setZero();
    for (int foot_index = 0; foot_index < 4; foot_index++)
    {
        // hatMap expects Vector3d (column vector), foot_positions.col() already returns column vector
        B.block<3, 3>(6, 3*foot_index) = I_w_inv*hatMap(foot_positions.col(foot_index));
        B.block<3, 3>(9, 3*foot_index) = Matrix3d::Identity()/mass;
    }
    
    // cout << "Continuous State Space Model" << endl;
    std::stringstream ssA, ssB;

    ssA << A;
    ssB << B;

    // cout << "A:\n" << ssA.str() << endl;
    // cout << "B:\n" << ssB.str() << endl;

    Matrix<double, 12, 13> C = Matrix<double, 12, 13>::Identity(); // Full state feedback, not including gravity state
    // Matrix<double, 12, 13> D = Matrix<double, 12, 13>::Identity();
    Matrix<double, 12, 13> D = Matrix<double, 12, 13>::Zero();

    return StateSpace(A,B,C,D);
};


StateSpace quadruped_state_space_discrete(const double& yaw, Matrix<double, 3, 4>& foot_positions, const Matrix3d& I_b, const double& mass, const double& dt)
{
    StateSpace ss = quadruped_state_space_continuous(yaw, foot_positions, I_b, mass);
    StateSpace dss = c2d(ss, dt);
    return dss;
};

// int main(int, char**)
// {
//     Matrix<double, 4, 3> foot_positions;
//     foot_positions << 0.1, 0.1, 0,
//                       0.1, -0.1, 0,
//                       -0.1, 0.1, 0,
//                       -0.1, -0.1, 0;
//     std::cout << "Foot positions:\n" << foot_positions << std::endl;

//     Vector<double, 13> x0 = Vector<double, 13>::Zero();
//     x0(12) = 9.81; // Set gravity state to 1
//     Vector<double, 12> u = Vector<double, 12>::Zero();

//     double yaw = 0.0f;
//     StateSpace ss = quadruped_state_space_continuous(yaw, foot_positions);

//     std::cout << "Continuous SS\n" << std::endl;
//     std::cout << "--------------\n" << std::endl;
//     std::cout << "A:\n" << ss.A << std::endl;
//     std::cout << "\nB:\n" << ss.B << std::endl;
//     std::cout << "\nC:\n" << ss.C << std::endl;
//     std::cout << "\nD:\n" << ss.D << std::endl;

//     StateSpace discrete_ss = c2d(ss, 0.01);
//     std::cout << "\nDiscrete SS using ZOH\n" << std::endl;
//     std::cout << "--------------\n" << std::endl;
//     std::cout << "A:\n" << discrete_ss.A << std::endl;
//     std::cout << "\nB:\n" << discrete_ss.B << std::endl;
//     std::cout << "\nC:\n" << discrete_ss.C << std::endl;
//     std::cout << "\nD:\n" << discrete_ss.D << std::endl;

//     Vector<double, 13> x1 = discrete_ss.A*x0 + discrete_ss.B*u;
//     std::cout << "\nNext state:\n" << x1 << std::endl;

//     Vector<double, 13> x2 = discrete_ss.A*x1 + discrete_ss.B*u;
//     std::cout << "\nNext state:\n" << x2 << std::endl;
//     return 0;
// }