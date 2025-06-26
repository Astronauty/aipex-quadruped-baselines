#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>

#include "gurobi_c++.h"

#include "rclcpp/rclcpp.hpp"

#include "convex_mpc/state_space.hpp"
#include "convex_mpc/gurobi_utils.hpp"
#include "convex_mpc/mpc_params.hpp"
#include "convex_mpc/simplified_quad_dynamics.hpp"
#include "convex_mpc/quad_params.hpp"

using namespace std;
using namespace Eigen;
// using namespace GRB;

class ConvexMPC
{
    public:
        ConvexMPC(MPCParams mpc_params, QuadrupedParams quad_params, const rclcpp::Logger& logger);
        // ~ConvexMPC();

        tuple<MatrixXd, MatrixXd> create_state_space_prediction_matrices(const StateSpace& quad_dss);
        VectorXd predict_states(MatrixXd A_qp, MatrixXd B_qp);


        StateSpace get_default_dss_model();
        

        
        // void update();
        void update_x0(Vector<double, 13> x0);

    private:
        MPCParams mpc_params;
        QuadrupedParams quad_params;
        std::unique_ptr<GRBEnv> env; //Using a unique pointer to delay model initialization until env is properly set, while keeping model a member variable
        std::unique_ptr<GRBModel> model;

        GRBVar* U;
        GRBQuadExpr quad_expr;
        GRBLinExpr lin_expr;

        rclcpp::Logger logger_;
        
        Vector<double, 13> x0;
        Vector<double, 12> u;
        VectorXd x_ref;

        MatrixXd A_qp;
        MatrixXd B_qp;

        MatrixXd P; // Quadratic cost of MPC
        MatrixXd q; // Linear cost of MPC

        Vector3d foot_positions[4];

        MatrixXd Q_bar; // Diagonal block matrix of quadratic state cost for N_MPC steps
        MatrixXd R_bar; // Diagonal block matrix of quadratic control cost for N_MPC-1 steps
        MatrixXd compute_R_bar();
        MatrixXd compute_Q_bar();
        // MatrixXd blkdiag(const vector<MatrixXd>& matrices);

        MatrixXd compute_P(MatrixXd R_bar, MatrixXd Q_bar, MatrixXd A_qp);
        VectorXd compute_q(MatrixXd Q_bar, MatrixXd A_qp, MatrixXd B_qp, VectorXd x0, VectorXd x_ref);
};
