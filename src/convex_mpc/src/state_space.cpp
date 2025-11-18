#include "convex_mpc/state_space.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>

using namespace std;
StateSpace c2d(const StateSpace& ss, double dt)
{
    MatrixXd A = ss.A;
    MatrixXd B = ss.B;
    MatrixXd C = ss.C;
    MatrixXd D = ss.D;

    // Compute matrix exponential
    MatrixXd M(A.rows()+ B.cols(), A.cols() + B.cols()); // Augmented matrix for finding ZOH discretization

    M << A, B,
         MatrixXd::Zero(B.cols(), A.cols()), MatrixXd::Zero(B.cols(), B.cols());

    MatrixXd M_exp = (M*dt).exp();
    // cout << "M exp matrix:\n" << M_exp << endl;

    // cout << "M exp matrix:\n" << M_exp << endl;


    // Discretize the continuous state space model
    // MatrixXd Ad = (A*dt).exp();
    MatrixXd Ad = M_exp.topLeftCorner(A.rows(), A.cols());
    MatrixXd Bd = M_exp.topRightCorner(B.rows(), B.cols());
    MatrixXd Cd = C;
    MatrixXd Dd = D;

    return StateSpace(Ad, Bd, Cd, Dd);

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