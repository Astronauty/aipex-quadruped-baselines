#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <filesystem>
#include <array>
#include <stdexcept>


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
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"


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

        /**
         * @brief Joint torques that track a swing-foot reference, by Jacobian transpose.
         *
         * tau = J^T * ( Kp*(p_ref - p) + Kd*(v_ref - v) )
         *
         * Cartesian PD in the leg's world-aligned frame. No operational-space
         * inertia term: the mass matrix needed for it is not computed here, and a
         * placeholder identity would produce a silently wrong feedforward.
         *
         * @param J_i Foot Jacobian, LOCAL_WORLD_ALIGNED, 3x3 for this leg's joints.
         * @param p_i Measured foot position (world-aligned, relative to the base).
         * @param v_i Measured foot velocity (world-aligned).
         * @param p_i_ref Reference foot position, same frame as p_i.
         * @param v_i_ref Reference foot velocity, same frame as v_i.
         * @return Joint torques for this leg's 3 joints (hip, thigh, calf).
         */
        Vector<double, 3> compute_swing_leg_tracking_torques(
            const Matrix3d& J_i,
            const Vector3d& p_i, const Vector3d& v_i,
            const Vector3d& p_i_ref, const Vector3d& v_i_ref) const;

        /**
         * @brief Swing torques with the paper's feedforward term, eq. (1) + (2).
         *
         * tau_i = J^T [Kp (p_ref - p) + Kd (v_ref - v)] + tau_ff
         * tau_ff = J^T Lambda_i a_ref + (C_i qdot_i + G_i)
         *
         * Lambda_i = (J_i M_i^-1 J_i^T)^-1 is the operational-space inertia for
         * this leg, so the feedforward supplies the force needed to accelerate
         * the foot along its arc instead of making the feedback term generate it
         * from tracking error. Without it the error grows with how fast the arc
         * is: measured 32 mm RMS on a 0.15 s swing against 11 mm on a 1.0 s one.
         *
         * The Jdot*qdot term of the paper's eq. (2) is omitted -- it needs the
         * Jacobian time derivative, which Pinocchio can provide but which is a
         * second-order correction next to Lambda*a_ref. Stated rather than
         * silently dropped.
         *
         * @param leg_index Foot index, FL=0 FR=1 RL=2 RR=3. Selects the mass
         *        matrix block and the gravity/Coriolis rows.
         * @param a_i_ref Reference foot acceleration, same frame as p_i.
         */
        Vector<double, 3> compute_swing_leg_torques_with_feedforward(
            int leg_index,
            const Matrix3d& J_i,
            const Vector3d& p_i, const Vector3d& v_i,
            const Vector3d& p_i_ref, const Vector3d& v_i_ref,
            const Vector3d& a_i_ref);

        /**
         * @brief Per-leg operational-space inertia, Lambda_i = (J M^-1 J^T)^-1.
         *
         * Exact here: the model is fixed-base, so the legs are kinematically
         * independent and the mass matrix is block diagonal with one 3x3 block
         * per leg. (On a floating-base model the legs couple through the body and
         * this would be an approximation.)
         */
        Matrix3d get_foot_operational_space_inertia(int leg_index, const Matrix3d& J_i);

        /**
         * @brief Per-leg gravity and Coriolis torques, the C_i qdot_i + G_i of
         *        the paper's eq. (2). Needs joint velocities; see
         *        update_joint_velocities().
         */
        Vector3d get_leg_gravity_coriolis(int leg_index);

        /**
         * @brief Jdot * qdot for this foot, the paper's eq. (2) correction term.
         *
         * The part of the foot's acceleration that comes purely from the leg
         * moving, present even at constant joint rates. Expressed in
         * LOCAL_WORLD_ALIGNED to match get_foot_jacobians().
         */
        Vector3d get_foot_jacobian_time_variation_term(int leg_index);

        /**
         * @brief Joint velocities, for the Coriolis term and nothing else.
         * @note Foot order FL, FR, RL, RR, matching update_joint_angles().
         */
        void update_joint_velocities(const Vector<double, 12>& theta_dot);

        /**
         * @brief Set swing Kp from a target natural frequency, the paper's eq. (3).
         *
         * Kp_ii = omega_i^2 * Lambda_ii, evaluated at the current configuration.
         * The paper notes this is needed "to ensure high-gain stability in a wide
         * range of leg configurations" -- a fixed Kp lets the closed-loop natural
         * frequency drift as the leg's apparent mass changes with pose, and the
         * apparent mass differs per axis, which is why a hand-tuned Kp ends up
         * needing very different values on y than on x and z.
         *
         * @param omega Target natural frequency per axis, rad/s.
         * @param leg_index Leg whose Lambda sets the scale.
         * @param J_i That leg's Jacobian.
         */
        Matrix3d swing_Kp_from_natural_frequency(const Vector3d& omega,
                                                 int leg_index,
                                                 const Matrix3d& J_i);

        /**
         * @brief Swing torques with configuration-dependent gains, paper eq. (3).
         *
         * Kp_ii = omega_i^2 * Lambda_ii and Kd_ii = 2*zeta*omega_i*Lambda_ii,
         * both recomputed from the current configuration, so the closed-loop
         * natural frequency AND damping ratio stay constant as the leg's
         * apparent mass changes with pose.
         *
         * @param omega Target natural frequency per axis, rad/s.
         * @param zeta Target damping ratio (1.0 is critically damped).
         * @param use_feedforward Also add the eq. (2) term.
         */
        Vector<double, 3> compute_swing_leg_torques_natural_freq(
            int leg_index,
            const Matrix3d& J_i,
            const Vector3d& p_i, const Vector3d& v_i,
            const Vector3d& p_i_ref, const Vector3d& v_i_ref,
            const Vector3d& a_i_ref,
            const Vector3d& omega,
            double zeta,
            bool use_feedforward);

        void update_foot_positions(const Matrix<double, 3, 4>& foot_positions);
        void update_reference_trajectory(const VectorXd& X_ref);

        /**
         * @brief Sets contact constraints in the MPC problem based on the provided contact states. Enforces friction cone constraints for legs in contact (pyramidal approximation of the friction cone) and zero forces for swing legs.
         *
         * @note Friction coefficient is set by quad_params.mu which is passed to the ConvexMPC constructor.
         * @param contact_states An unordered map mapping leg names ("FL", "FR", "RL", "RR") to contact states (1 for contact, 0 for swing).
         * @note Holds one contact state across the whole horizon; only valid for a
         *       static gait. Prefer the schedule overload for anything that steps.
         * @return void
         */
        void set_contact_constraints(unordered_map<std::string, int>& contact_states); // Allows nonzero GRFs for legs in contact, forces zero GRFs for swing legs

        /**
         * @brief Sets contact constraints from a per-horizon-step contact schedule.
         *
         * @param contact_schedule Indexed [k][leg] with leg order FL, FR, RL, RR;
         *        true means stance. Must cover at least N_MPC-1 steps.
         */
        void set_contact_constraints(const std::vector<std::array<bool, 4>>& contact_schedule);

        /**
         * @brief Foot Jacobians (LOCAL_WORLD_ALIGNED), one 3x3 per foot, FL/FR/RL/RR.
         * @note Public so the node's swing-leg controller can map Cartesian
         *       foot-space PD into joint torques.
         */
        vector<Matrix<double, 3, 3>> get_foot_jacobians();

        /**
         * @brief Foot positions from FK, relative to the body CoM, in base-frame axes.
         */
        Matrix<double, 3, 4> get_foot_positions_body();

        /**
         * @brief Base-origin to body-CoM offset, in base-frame axes.
         * @note Needed to put a world-frame target into the same CoM-anchored frame
         *       that get_foot_positions_body() reports in.
         */
        Vector3d get_com_offset_body() const { return com_B_; }

        /**
         * @brief Swing-leg Cartesian PD gains, shared by every leg.
         */
        void set_swing_gains(const Matrix3d& Kp_swing, const Matrix3d& Kd_swing);

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
        std::vector<GRBConstr> contact_constraints_; // Store constraint references

        rclcpp::Logger logger_;

        // Steady clock backing the throttled log lines. Held as a member rather
        // than a function-local static so the throttle state is per-instance.
        std::shared_ptr<rclcpp::Clock> clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

        //Robot states (updated from the ROS2 node wrapper - see convex_mpc_node.hpp)
        Vector<double, 12> theta; // Joint angles of Go2
        // Vector<double, 13> x0;  // State vector containing information about the rigid body pose: [theta, p, omega, p_dot, g]
        VectorXd x0;
        VectorXd X_ref; // Desired rigid body pose of the quadruped, size N_STATES * N_MPC
        // Vector<double, 12> u;  // GRFs for the 4 feet of the quadruped robot, represented as a vector of 12 elements (3 for each foot: x, y, z)
        Matrix<double, 3, 4> ground_reaction_forces; // GRFs for the 4 feet of the quadruped robot, rows are x, y, z forces, columns are feet 0, 1, 2, 3

        MatrixXd A_qp;
        MatrixXd B_qp;

        MatrixXd P; // Quadratic cost of MPC
        MatrixXd q; // Linear cost of MPC

        // Vector3d foot_positions[4];
        Matrix<double, 3, 4> foot_positions_world; // Positions of the feet in the world frame
  
        MatrixXd Q_bar; // Diagonal block matrix of quadratic state cost for N_MPC steps
        MatrixXd R_bar; // Diagonal block matrix of quadratic control cost for N_MPC-1 steps
        MatrixXd compute_R_bar();
        MatrixXd compute_Q_bar();
        // MatrixXd blkdiag(const vector<MatrixXd>& matrices);

        MatrixXd compute_P(const MatrixXd& R_bar, const MatrixXd& Q_bar, const MatrixXd& B_qp, double regularization = 0.0);
        VectorXd compute_q(const MatrixXd& Q_bar, const MatrixXd& A_qp, const MatrixXd& B_qp, const VectorXd& x0, const VectorXd& X_ref);
        // Which feet are in stance, refreshed by set_contact_constraints() before
        // each solve. Defaults to all four so a solve occurring before the first
        // contact update assumes a standing distribution.
        std::array<bool, 4> stance_ = {true, true, true, true};

        Matrix3d Kp; // Proportional gain for swing leg tracking
        Matrix3d Kd; // Derivative gain for swing leg tracking
        // get_foot_jacobians() is declared public above, for the node's swing controller.

        std::array<std::array<int, 3>, 4> joint_qidx_; // idx_q() per [foot][joint] - index for joint angles of a given foot within pinocchio q vector
        std::array<std::array<int, 3>, 4> joint_vidx_; // idx_v() per [foot][joint] - index for joint velocities of a given foot within pinocchio q vector
        std::array<pinocchio::FrameIndex, 4> foot_fid_; // Foot frame per foot

        Eigen::VectorXd q_pin_;
        Eigen::VectorXd v_pin_;   // Joint velocities, for the Coriolis term

        // Body CoM in base frame, resolved once from the URDF base link inertia.
        // Fixed by the massless-leg assumption; see get_foot_positions_body().
        Vector3d com_B_ = Vector3d::Zero();
        bool foot_kinematics_dirty_ = true;
        void resolve_model_indices();
        void update_kinematics();

        // get_foot_positions_body() is declared public above, for the node's swing controller.
        

        StateSpace get_quadruped_dss_model(const double& yaw, Matrix<double, 3, 4>& foot_positions, const double& dt);

        std::vector<Matrix<double, 3, 4>>  grf_from_mpc_solution(); // Extracts the GRFs from the Gurobi MPC solution and returns as an eigen matrix

        // void add_friction_cone_constraints(GRBModel& model, GRBVar* U, const double& mu);
        Vector<double,12> clamp_joint_torques(Vector<double, 12>& joint_torques);


        
    };

void print_eigen_matrix(const Eigen::MatrixXd& mat, string name, const rclcpp::Logger& logger); // Utility function to print Eigen matrices to the logger