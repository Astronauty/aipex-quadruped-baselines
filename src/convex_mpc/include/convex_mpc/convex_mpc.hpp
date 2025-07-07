#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <filesystem>

#include "gurobi_c++.h"
#include "rclcpp/rclcpp.hpp"

#include "convex_mpc/state_space.hpp"
#include "convex_mpc/gurobi_utils.hpp"
#include "convex_mpc/mpc_params.hpp"
#include "convex_mpc/simplified_quad_dynamics.hpp"
#include "convex_mpc/quad_params.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

using namespace std;
using namespace Eigen;
// using namespace GRB;


/** 
    @class ConvexMPC
    @brief Solves a convex MPC problem for a quadrupedal robot using the Gurobi QP solver.
*/
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
        void update_joint_angles(Vector<double, 12> theta);

        Vector<double, 12> solve_joint_torques(); // Returns joint torques based on MPC GRF solution
        Vector<double, 3> compute_swing_leg_tracking_torques(
            const Vector<double, 12>& q, const Vector<double, 12> q_i_dot,
            Matrix3d J_i_B, 
            Vector3d p_i_B, Vector3d v_i_B,
            Vector3d p_i_ref_B, Vector3d v_i_ref_B,
            Matrix3d Kp, Matrix3d Kd,
            Vector3d a_i_ref_B, 
            int foot_index);

        void update_foot_positions(const Matrix<double, 3, 4>& foot_positions);


    private:
        MPCParams mpc_params;
        QuadrupedParams quad_params;
        std::unique_ptr<GRBEnv> env; //Using a unique pointer to delay model initialization until env is properly set, while keeping model a member variable
        std::unique_ptr<GRBModel> model;

        // Pinocchio model and data for foot jacobian computation
        pinocchio::Model pinocchio_model;
        pinocchio::Data pinocchio_data;

        GRBVar* U; // Decision variables for MPC, GRFs for the next N_MPC-1 timesteps
        GRBQuadExpr quad_expr;
        GRBLinExpr lin_expr;

        rclcpp::Logger logger_;

        //Robot states (updated from the ROS2 node wrapper - see convex_mpc_node.hpp)
        Vector<double, 12> theta; // Joint angles of Go2
        // Vector<double, 13> x0;  // State vector containing information about the rigid body pose: [theta, p, omega, p_dot, g]
        VectorXd x0;
        VectorXd x_ref; // Desired rigid body pose of the quadruped, size N_STATES * N_MPC
        // Vector<double, 12> u;  // GRFs for the 4 feet of the quadruped robot, represented as a vector of 12 elements (3 for each foot: x, y, z)
        Matrix<double, 3, 4> ground_reaction_forces; // GRFs for the 4 feet of the quadruped robot, rows are x, y, z forces, columns are feet 0, 1, 2, 3

        MatrixXd A_qp;
        MatrixXd B_qp;

        MatrixXd P; // Quadratic cost of MPC
        MatrixXd q; // Linear cost of MPC

        // Vector3d foot_positions[4];
        Matrix<double, 3, 4> foot_positions; // Positions of the feet in the body frame
        // void update_foot_positions(const Vector<double, 12>& q);

        MatrixXd Q_bar; // Diagonal block matrix of quadratic state cost for N_MPC steps
        MatrixXd R_bar; // Diagonal block matrix of quadratic control cost for N_MPC-1 steps
        MatrixXd compute_R_bar();
        MatrixXd compute_Q_bar();
        // MatrixXd blkdiag(const vector<MatrixXd>& matrices);

        MatrixXd compute_P(MatrixXd R_bar, MatrixXd Q_bar, MatrixXd A_qp);
        VectorXd compute_q(MatrixXd Q_bar, MatrixXd A_qp, MatrixXd B_qp, VectorXd x0, VectorXd x_ref);

        Matrix3d Kp; // Proportional gain for swing leg tracking
        Matrix3d Kd; // Derivative gain for swing leg tracking
        // Vector3d get_joint_torques_for_foot()
        vector<Matrix<double, 3, 3>>  get_foot_jacobians(const Vector<double, 12>& q); // Returns the foot jacobians for each foot in the body frame
        Matrix3d get_foot_operation_space_inertia_matrix(const Vector<double, 12>& q, int foot_index);

        StateSpace get_quadruped_dss_model(const double& yaw, Matrix<double, 3, 4>& foot_positions, const double& dt);
};
