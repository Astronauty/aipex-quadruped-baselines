// #pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "convex_mpc.hpp"


using namespace std;
using namespace Eigen;

/*
QP Form:
min 0.5 x^T P x + q^T x

*/

ConvexMPC::ConvexMPC(MPCParams mpc_params, QuadrupedParams quad_params)
    : mpc_params(mpc_params), quad_params(quad_params)
{
    try
    {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "convex_mpc.log");
        env.start();

        // Create an empty Gurobi model
        GRBModel model = GRBModel(env);


        // env = GRBEnv();
        // model = GRBModel(env);

        // Initialize robot state
        x0 = Vector<double, 13>::Zero();
        u = Vector<double, 12>::Zero();
        StateSpace quad_dss = get_default_dss_model(); // TODO: proper initialization
        tie(A_qp, B_qp) = create_state_space_prediction_matrices(quad_dss);

        // Formulate the QP 
        GRBVar *U = model.addVars(mpc_params.N_CONTROLS, GRB_CONTINUOUS);
        model.setObjective(create_quad_obj(U, mpc_params.Q , mpc_params.N_CONTROLS) 
            + create_quad_obj(U, mpc_params.R, mpc_params.N_STATES));


        model.setObjective(create_quad_obj(U, A_qp ))

        model.optimize();

        std::cout << "Optimized variables:" << std::endl;
        for (int i = 0; i < mpc_params.N_MPC; ++i) {
            double x_value = U[i].get(GRB_DoubleAttr_X);
            std::cout << "x[" << i << "] = " << x_value << std::endl;
        }


    } 
    catch(GRBException e) 
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }

}

ConvexMPC::~ConvexMPC(){

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

tuple<MatrixXd, MatrixXd> ConvexMPC::create_state_space_prediction_matrices(const StateSpace& quad_dss)
{
    const int& N_STATES = this->mpc_params.N_STATES;
    const int& N_CONTROLS = this->mpc_params.N_CONTROLS;
    const int& N_MPC = this->mpc_params.N_MPC;

    MatrixXd A_qp = MatrixXd::Zero(N_STATES * N_MPC, N_CONTROLS * N_MPC);
    MatrixXd B_qp = MatrixXd::Zero(N_STATES * N_MPC, N_CONTROLS * N_MPC); 

    for (int i = 0; i < mpc_params.N_MPC; ++i) {
        for (int j = 0; j <= i; ++j) {
            A_qp.block(i * N_STATES, j * N_CONTROLS, N_STATES, N_CONTROLS) = quad_dss.A.pow(i - j) * quad_dss.B;
        }
        B_qp.block(i*N_STATES, 0, N_STATES, N_STATES) = quad_dss.A.pow(i+1); 
    }

    return std::make_tuple(A_qp, B_qp);
}

StateSpace ConvexMPC::get_default_dss_model()
{
    Matrix<double, 4, 3> foot_positions;
    foot_positions << 0.1, 0.1, 0,
                      0.1, -0.1, 0,
                      -0.1, 0.1, 0,
                      -0.1, -0.1, 0;

    double yaw = 0.0f;
    StateSpace quad_dss = quadruped_state_space_discrete(yaw, foot_positions, this->mpc_params.dt);

    return quad_dss;
}

MatrixXd blkdiag(const std::vector<MatrixXd>& matrices) {
    // Calculate the total size of the block diagonal matrix
    int totalRows = 0, totalCols = 0;
    for (const auto& mat : matrices) {
        totalRows += mat.rows();
        totalCols += mat.cols();
    }

    // Create the block diagonal matrix
    MatrixXd blockDiagonal = MatrixXd::Zero(totalRows, totalCols);

    // Place each matrix along the diagonal
    int currentRow = 0, currentCol = 0;
    for (const auto& mat : matrices) {
        blockDiagonal.block(currentRow, currentCol, mat.rows(), mat.cols()) = mat;
        currentRow += mat.rows();
        currentCol += mat.cols();
    }

    return blockDiagonal;
}

MatrixXd ConvexMPC::compute_R_bar()
{
    MatrixXd R = mpc_params.R;
    int N_CONTROLS = mpc_params.N_CONTROLS;
    int N_MPC = mpc_params.N_MPC;

    std::vector<MatrixXd> R_vec(N_MPC, R);
    MatrixXd R_bar = blkdiag(R_vec);
    return R_bar;
}

MatrixXd ConvexMPC::compute_Q_bar()
{
    MatrixXd Q = mpc_params.Q;
    int N_CONTROLS = mpc_params.N_CONTROLS;
    int N_MPC = mpc_params.N_MPC;

    std::vector<MatrixXd> Q_vec(N_MPC,Q);
    MatrixXd R_bar = blkdiag(Q_vec);
    return Q_bar;
}


/*
Computes the quadratic cost fo the unconstrained linear mpc problem

P = 2*R_bar + 2*A_qp'*Q_bar*A_qp
*/
MatrixXd ConvexMPC::compute_P(MatrixXd R_bar, MatrixXd Q_bar, MatrixXd A_qp)
{
    MatrixXd P = 2*R_bar + 2*A_qp.transpose() * Q_bar * A_qp;
    return P;   
}

VectorXd ConvexMPC::compute_q(VectorXd Q_bar, B_qp, x0)
{
    VectorXd q = 2*
}


void update()
{
    // TODO:
    /*
    1) Get updated joint angles/velocities from unitree ros topic
        a) Update rigid body robot state (using estimator?)
        b) Compute foot positions with FK
    2) Recompute quad_dss with new foot positions and yaw
    */

}

int main(int, char**)
{
    // Define the parameters for the MPC and the quadruped
    int N_MPC = 3;
    double dt = 0.01;
    int N_STATES = 13;
    int N_CONTROLS = 4;

    // MPC Params
    MatrixXd Q = MatrixXd::Identity(N_STATES, N_STATES);
    MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS);
    VectorXd u_lower = VectorXd::Constant(N_CONTROLS, -1.0);
    VectorXd u_upper = VectorXd::Constant(N_CONTROLS, 1.0);
    MPCParams mpc_params = MPCParams(N_MPC, N_CONTROLS, N_STATES,
         dt, Q, R, u_lower, u_upper);

    // Quadruped Params
    Matrix3d inertiaTensor = Eigen::Matrix3d::Identity();
    double mass = 1.0;
    double gravity = 9.81;
    QuadrupedParams quadruped_params = QuadrupedParams(inertiaTensor, mass, gravity);

    // Initialize the ConvexMPC object
    ConvexMPC convex_mpc = ConvexMPC(mpc_params, quadruped_params);

    // Example of creating state space prediction matrices
    StateSpace quad_dss = convex_mpc.get_default_dss_model(); // TODO: Implement actual updates to the dss based on yaw and foot position

    auto [A_qp, B_qp] = convex_mpc.create_state_space_prediction_matrices(quad_dss);

    // Print the matrices to verify

    cout << "A_qp: \n" << A_qp << endl;
    cout << "B_qp: \n" << B_qp << endl;
    // cout << "Size of A_qp: " << A_qp.rows() << " x " << A_qp.cols() << endl;
    // cout << "Size of B_qp: " << B_qp.rows() << " x " << B_qp.cols() << endl;

    return 0;
}
