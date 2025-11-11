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
StateSpace quadruped_state_space_continuous(const double& yaw,
                                            const Eigen::Matrix<double,3,4>& foot_levers_W,  // const & rename: r_i^W
                                            const Eigen::Matrix3d& I_b,
                                            const double& mass)
{
    // Inertia about world-aligned body (roll/pitch small, yaw = yaw)
    Eigen::Matrix3d Rz = eul2rotm(0.0, 0.0, yaw);
    Eigen::Matrix3d I_w = Rz * I_b * Rz.transpose();
    Eigen::Matrix3d I_w_inv = I_w.inverse();

    // State order: x = [theta(0:2), p(3:5), omega(6:8), p_dot(9:11), g(12)]
    Eigen::MatrixXd A(13,13); A.setZero();

    // θ̇ = T(θ) ω. For small angles, T(θ) ≈ I_3 (don't use Rz)
    A.block<3,3>(0,6) = Eigen::Matrix3d::Identity();

    // ṗ = v
    A.block<3,3>(3,9) = Eigen::Matrix3d::Identity();

    // v̇_z has -g (g is the last state x(12))
    A(11,12) = -1.0;

    // Inputs are ground reaction forces u = [F_FL; F_FR; F_RL; F_RR] (each 3x1), WORLD frame
    Eigen::MatrixXd B(13,12); B.setZero();
    for (int i = 0; i < 4; ++i) {
        // ω̇ = I_w^{-1} * Σ (r_i^W × F_i)  => block to ω̇ is I_w_inv * hat(r_i^W)
        // NOTE: pass a 3x1 vector to hatMap, not a row (don't transpose).
        B.block<3,3>(6, 3*i) = I_w_inv * hatMap(foot_levers_W.col(i));
        // v̇ = (1/m) Σ F_i
        B.block<3,3>(9, 3*i) = Eigen::Matrix3d::Identity() / mass;
    }

    // Output matrices (unused, but keep consistent)
    Eigen::Matrix<double,12,13> C = Eigen::Matrix<double,12,13>::Zero();
    C.block<12,12>(0,0) = Eigen::Matrix<double,12,12>::Identity(); // observe first 12 states
    Eigen::Matrix<double,12,13> D = Eigen::Matrix<double,12,13>::Zero();

    return StateSpace(A,B,C,D);
};


StateSpace quadruped_state_space_discrete(const double& yaw,
                                          const Eigen::Matrix<double,3,4>& foot_levers_W, // const &
                                          const Eigen::Matrix3d& I_b,
                                          const double& mass,
                                          const double& dt)
{
    StateSpace ss = quadruped_state_space_continuous(yaw, foot_levers_W, I_b, mass);
    return c2d(ss, dt);
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