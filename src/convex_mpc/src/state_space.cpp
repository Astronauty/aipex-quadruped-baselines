#include "convex_mpc/state_space.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

using namespace std;
StateSpace c2d(const StateSpace& ss, double dt)
{
    const int n = ss.A.rows();        // state dim
    const int m = ss.B.cols();        // input dim

    // Augmented for ZOH: exp( [A B; 0 0] dt ) = [Ad Bd; 0 I]
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n + m, n + m);
    M.topLeftCorner(n, n)  = ss.A;
    M.topRightCorner(n, m) = ss.B;

    Eigen::MatrixXd Md = (M * dt).exp();

    Eigen::MatrixXd Ad = Md.topLeftCorner(n, n);
    Eigen::MatrixXd Bd = Md.topRightCorner(n, m);

    // C, D unchanged for ZOH
    return StateSpace(Ad, Bd, ss.C, ss.D);
}

// int main(int, char**)
// {   
//     Eigen::MatrixXd A(2, 2);
//     Eigen::MatrixXd B(2, 1);
//     Eigen::MatrixXd C(1, 2);
//     Eigen::MatrixXd D(1, 1);

//     A << 0, 1,
//          -2, -3;
//     B << 0,
//          1;
//     C << 1, 0;
//     D << 0;

//     StateSpace go2_ss(A, B, C, D);

//     double dt = 0.01;
//     StateSpace go2_ss_d = c2d(go2_ss, dt);

//     std::cout << "Ad matrix:\n" << ss_discrete.A << std::endl;
//     std::cout << "Bd matrix:\n" << ss_discrete.B << std::endl;
//     std::cout << "Cd matrix:\n" << ss_discrete.C << std::endl;
//     std::cout << "Dd matrix:\n" << ss_discrete.D << std::endl;

//     return 0;
// }