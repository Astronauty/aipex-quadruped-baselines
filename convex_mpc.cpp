
#pragma once

#include <Eigen/Dense>
#include "convex_mpc.hpp"

using namespace std;
using namespace eigen;


ConvexMPC() 
{
    try
    {
        env = GRBEnv();
        model = GRBModel();
    } 
    catch(GRBException e) 
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
}


/**
 * @brief Creates state evolution matrices for the Convex Model Predictive Control (MPC).
 * 
 * This function generates the state prediction matrices A_qp and B_qp based on the given discrete state space 
 * representation of the quadruped and the specified MPC horizon length. These can be used to generate a vector of 
 * the predicted states X for the next N_MPC timesteps, where U = [u_1... u_N-1] and x0 is the current state:
 * X = A_qp * U + B_qp * x0
 *
 * @param quad_dss The discrete state space representation of the quadruped.
 * @param N_MPC Number of steps for the horizon length in MPC.
 * @return A tuple containing two Eigen::MatrixXd objects representing the state evolution matrices.
 */
tuple<MatrixXd, MatrixXd> ConvexMPC::create_state_space_prediciton_matrices(const& StateSpace quad_dss, int& N_MPC)
{
    int N_STATES = quad_dss.A.columns();
    int N_CONTROLS = quad_dss.B.columns();

    MatrixXd A_qp = MatrixXd::Zero(state_dim * horizon, input_dim * horizon);
    MatrixXd B_qp = MatrixXd::Zero(state_dim * horizon, input_dim * horizon); // Example of another matrix

    for (int i = 0; i < N_MPC; ++i) {
        for (int j = 0; j <= i; ++j) {
            A_qp.block<N_STATES, N_CONTROLS>(i * N_STATES, j * N_CONTROLS) = A.pow(i - j) * B;
        }
        B_qp.block<N_STATES, N_STATES>(i*N_STATES, 0) = A.pow(i+1); // Example of another matrix
    }

    return std::make_tuple(A_qp, B_qp);
}

