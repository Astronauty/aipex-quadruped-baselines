#include <Eigen/Dense>
#pragma once

using namespace Eigen;

struct StateSpace
{
    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    MatrixXd D;

    // double dt;

    StateSpace(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D)
        : A(A), B(B), C(C), D(D) {}
    ~StateSpace() {}
};

/*
Discretization via ZOH
*/
StateSpace c2d(const StateSpace& ss, double dt);