// #pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "convex_mpc.hpp"

using namespace std;
using namespace Eigen;


ConvexMPC::ConvexMPC(MPCParams mpc_params, QuadrupedParams quad_params)
    : mpc_params(mpc_params), quad_params(quad_params)
{
    try
    {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "convex_mpc.log");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);
        // env = GRBEnv();
        // model = GRBModel(env);
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

tuple<MatrixXd, MatrixXd> ConvexMPC::create_state_space_prediction_matrices(StateSpace const& quad_dss, int& N_MPC)
{
    const int& N_STATES = mpc_params.N_STATES;
    const int& N_CONTROLS = mpc_params.N_CONTROLS;

    MatrixXd A_qp = MatrixXd::Zero(N_STATES * N_MPC, N_CONTROLS * N_MPC);
    MatrixXd B_qp = MatrixXd::Zero(N_STATES * N_MPC, N_CONTROLS * N_MPC); // Example of another matrix

    for (int i = 0; i < mpc_params.N_MPC; ++i) {
        for (int j = 0; j <= i; ++j) {
            A_qp.block(i * N_STATES, j * N_CONTROLS, N_STATES, N_CONTROLS) = quad_dss.A.pow(i - j) * quad_dss.B;
        }
        B_qp.block(i*N_STATES, 0, N_STATES, N_STATES) = quad_dss.A.pow(i+1); // Example of another matrix
    }

    return std::make_tuple(A_qp, B_qp);
}

int main(int, char**)
{
    // Define the parameters for the MPC and the quadruped
    int N_MPC = 10;
    double dt = 0.1;
    int N_STATES = 13;
    int N_CONTROLS = 4;
    MatrixXd Q = MatrixXd::Identity(N_STATES, N_STATES);
    MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS);
    VectorXd u_lower = VectorXd::Constant(N_CONTROLS, -1.0);
    VectorXd u_upper = VectorXd::Constant(N_CONTROLS, 1.0);
    MPCParams mpc_params = MPCParams(N_MPC, N_CONTROLS, N_STATES,
         dt, Q, R, u_lower, u_upper);



    Matrix3d inertiaTensor = Eigen::Matrix3d::Identity();
    double mass = 1.0;
    double gravity = 9.81;
    QuadrupedParams quadruped_params = QuadrupedParams(inertiaTensor, mass, gravity);
    

    return 0;
}
