#include <Eigen/Dense>
#include <iostream>
#include "simplified_quad_dynamics.hpp"
#include "transforms.hpp"
#include "state_space.hpp"

using namespace Eigen;

/* 
Simplified state space quadruped dynamics based on https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf
x = [theta, p, omega, p_dot, g]
*/
StateSpace quadruped_state_space_continuous(const float& yaw, Matrix<float, 4, 3>& foot_positions)
{   
    // Define relevant transforms and inertial properties
    Matrix3f R_z = eul2rotm(0, 0, yaw);
    float m = 1;
    Matrix3f I_b = Matrix3f::Identity(); // TODO: DEFINE GO2 INERTIA HERE
    Matrix3f I_w = R_z*I_b*R_z.transpose(); // Inertia tensor in world body frame for small angles 
    Matrix3f I_w_inv = I_w.inverse(); // Inverse of inertia tensor in world body frame
    // Vector3f g;
    // g << 0, 0, -9.81; // Gravity vector
    float g;
    g = 9.81;

    // Define the continuous state space model for a simplified quadruped
    MatrixXf A(13,13);
    A.setZero();
    A.block<3, 3>(0, 6) = R_z;
    A.block<3, 3>(3, 9) = Matrix3f::Identity();
    // A.block<3, 1>(9, 0) = Matrix3f::iden
    A(11, 12) = -1; // Gravity influence on accel
    A(12, 12) = 1; // Gravity state



    // Matrix<float, 13, 13> B;
    MatrixXf B(13, 13);
    B.setZero();
    for (int foot_index = 0; foot_index < 4; foot_index++)
    {
        B.block<3, 3>(6, 3*foot_index) = I_w_inv*hatMap(foot_positions.row(foot_index).transpose());
        B.block<3, 3>(9, 3*foot_index) = Matrix3f::Identity()/m;
    }
    
    Matrix<float, 12, 13> C = Matrix<float, 12, 13>::Identity(); // Full state feedback, not including gravity state
    // Matrix<float, 12, 13> D = Matrix<float, 12, 13>::Identity();
    Matrix<float, 12, 13> D = Matrix<float, 12, 13>::Zero();

    return StateSpace(A,B,C,D);
};


int main(int, char**)
{
    Matrix<float, 4, 3> foot_positions;
    foot_positions << 0.1, 0.1, 0,
                      0.1, -0.1, 0,
                      -0.1, 0.1, 0,
                      -0.1, -0.1, 0;
    std::cout << "Foot positions:\n" << foot_positions << std::endl;

    Vector<float, 13> x0 = Vector<float, 13>::Zero();
    x0(12) = 9.81; // Set gravity state to 1
    Vector<float, 12> u = Vector<float, 12>::Zero();

    float yaw = 0.0f;
    StateSpace ss = quadruped_state_space_continuous(yaw, foot_positions);

    std::cout << "Continuous SS\n" << std::endl;
    std::cout << "--------------\n" << std::endl;
    std::cout << "A:\n" << ss.A << std::endl;
    std::cout << "\nB:\n" << ss.B << std::endl;
    std::cout << "\nC:\n" << ss.C << std::endl;
    std::cout << "\nD:\n" << ss.D << std::endl;

    StateSpace discrete_ss = c2d(ss, 0.01);
    std::cout << "\nDiscrete SS using ZOH\n" << std::endl;
    std::cout << "--------------\n" << std::endl;
    std::cout << "A:\n" << discrete_ss.A << std::endl;
    std::cout << "\nB:\n" << discrete_ss.B << std::endl;
    std::cout << "\nC:\n" << discrete_ss.C << std::endl;
    std::cout << "\nD:\n" << discrete_ss.D << std::endl;



    return 0;
}