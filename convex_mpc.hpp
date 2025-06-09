#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "gurobi_c++.h"
#include "state_space.hpp"
#include "gurobi_utils.hpp"
#include "mpc_params.hpp"
#include "simplified_quad_dynamics.hpp"
#include "quad_params.hpp"

using namespace std;
using namespace Eigen;
// using namespace GRB;

// Enum for measurement mode
enum class StateMeasurementMode {
    LOWLEVEL_EKF,
    SPORTMODE,
    MOCAP
};



class ConvexMPC
{
    public:
        ConvexMPC(MPCParams mpc_params, QuadrupedParams quad_params);
        // ~ConvexMPC();

        tuple<MatrixXd, MatrixXd> create_state_space_prediction_matrices(const StateSpace& quad_dss);
        VectorXd predict_states(MatrixXd A_qp, MatrixXd B_qp);


        StateSpace get_default_dss_model();

        
        void update();
    private:
        GRBModel model;
        GRBEnv env;
        GRBVar* U;
        GRBQuadExpr quad_expr;
        GRBLinExpr lin_expr;

        MPCParams mpc_params;
        QuadrupedParams quad_params;
        
        Vector<double, 13> x0;
        Vector<double, 12> u;
        VectorXd x_ref;

        MatrixXd A_qp;
        MatrixXd B_qp;

        MatrixXd Q_bar; // Diagonal block matrix of quadratic state cost for N_MPC steps
        MatrixXd R_bar; // Diagonal block matrix of quadratic control cost for N_MPC-1 steps

        MatrixXd P; // Quadratic cost of MPC
        MatrixXd q; // Linear cost of MPC

        Vector3d foot_positions[4];


        MatrixXd R_bar;
        MatrixXd Q_bar;
        MatrixXd compute_R_bar();
        MatrixXd compute_Q_bar();
        // MatrixXd blkdiag(const vector<MatrixXd>& matrices);

        MatrixXd compute_P(MatrixXd R_bar, MatrixXd Q_bar, MatrixXd A_qp);
        VectorXd compute_q(MatrixXd Q_bar, MatrixXd A_qp, MatrixXd B_qp, VectorXd x0, VectorXd x_ref);

        void update_x0(Vector<double, 13> x0);

};
