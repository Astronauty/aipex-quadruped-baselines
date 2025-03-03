#include <Eigen/Dense>
#pragma once

using namespace Eigen;

struct StateSpace
{
    MatrixXf A;
    MatrixXf B;
    MatrixXf C;
    MatrixXf D;

    // float dt;

    StateSpace(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D)
        : A(A), B(B), C(C), D(D) {}
    ~StateSpace() {}
};

/*
Discretization via ZOH
*/
StateSpace c2d(const StateSpace& ss, float dt);