#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <functional>
#include <deque>

#include "convex_mpc/swing_trajectory.hpp"
#include "convex_mpc/mpc_params.hpp"
#include "convex_mpc/state_indices.hpp"
#include "convex_mpc/trajectory_utils.hpp"


using namespace std;

enum class GaitType
{
    STAND,
    TROT,
    WALK,
    GALLOP,
    PRONK,
    BOUND
};

class GaitPlanner
{
    public:


        /** Define a gait planner for quadruped robots. Based on specified gait type, solves for footstep timings/locations in addition to generating swing leg trajectories.
         * @param gait_type The type of gait to use (e.g., TROT, WALK, GALLOP, PRONK, BOUND).
         * @param duty_factor The duty factor for the gait which describes the phase duration for stance, default is 0.5.
         * @param gait_duration_s The duration of the gait in seconds, default is 1.0.
         * @param swing_height_m The height of the swing trajectory in meters, default is 0.08.
         * @param footstep_planning_horizon_s The planning horizon in seconds for future footsteps, default is 2.0.
         */
        GaitPlanner(GaitType gait_type, double duty_factor = 0.5, double gait_duration_s = 1.0, double swing_height_m = 0.08, double footstep_planning_horizon_s = 2.0, double start_time_s = 0.0);

        /**
         * @brief Contact state at the planner's current phase (set by update_time_and_phase).
         * @return A map with leg names as keys and contact state (0 or 1) as values.
         * 0 indicates the leg is in swing, 1 indicates the leg is in stance.
         */
        std::unordered_map<std::string, int> get_contact_state();

        /**
         * @brief Contact state at an arbitrary gait phase.
         * @param phase The phase of the gait cycle, nominally in [0, 1).
         * @return A map with leg names as keys and contact state (0 or 1) as values.
         */
        std::unordered_map<std::string, int> get_contact_state_at_phase(double phase);

        /**
         * @brief Contact schedule sampled across an MPC horizon.
         *
         * Needed because the MPC constrains every horizon timestep, and contact
         * flips within the horizon for any real gait. Indexed [k][leg] with the
         * leg order FL, FR, RL, RR; true means stance.
         *
         * @param start_time_s Time in seconds of horizon step k = 0.
         * @param dt Horizon discretization in seconds.
         * @param N Number of horizon steps to sample.
         */
        std::vector<std::array<bool, 4>> get_contact_schedule(double start_time_s, double dt, int N) const;

        /**
         * @brief Set the gait type for the planner.
         * @param gait_type The type of gait to set.
         * 
         * This method allows changing the gait type after the planner has been initialized.
         * It will update the phase offsets accordingly.
         */
        void set_gait_type(GaitType gait_type);

        /**
         * @brief Switch gait and restart the gait clock, so the new gait begins at phase 0.
         * @param gait_type The gait to switch to.
         * @param duty_factor Duty factor for the new gait (STAND's 1.0 must not carry over).
         * @param start_time_s New phase origin, normally "now".
         */
        void start_gait(GaitType gait_type, double duty_factor, double start_time_s);
        double get_phase(double current_time) const;

        /** Update the current time and phase of the gait planner.
         * @param current_time_s The current time in seconds.
         */
        void update_time_and_phase(double current_time_s);


        /**
         * @brief Plan/refresh swing trajectories across the footstep planning horizon.
         * @param X_ref MPC reference trajectory, used to evaluate the Raibert heuristic at touchdown.
         * @param mpc_params MPC horizon parameters, for indexing into X_ref.
         * @param current_foot_positions Measured foot positions in the WORLD frame, keyed
         *        "FL"/"FR"/"RL"/"RR". These become the lift-off points of first swings, and
         *        must share a frame with the Raibert touchdown targets, which are world frame.
         *
         * X_ref is the latched reference in every slot; only its commanded velocity is
         * used here. Touchdown targets come from the measured body state set with
         * set_measured_body_state() (Cheetah 3 eq. 33), recomputed every cycle until
         * the swing lifts off and frozen after that.
         */
        std::unordered_map<std::string, std::deque<SwingLegTrajectory>> update_swing_leg_trajectories(const Eigen::VectorXd& X_ref, const MPCParams& mpc_params, const unordered_map<string, Vector3d> &current_foot_positions);
 
        /**
         * @brief Get the current footstep target position and velocity for a specific leg
         * @param leg The leg identifier ("FL", "FR", "RL", "RR")
         * @param current_time_s The current time in seconds
         * @param target_position Output parameter for the target position
         * @param target_velocity Output parameter for the target velocity
         * @return true if a valid trajectory is found and evaluated, false otherwise
         */
        bool get_current_footstep_target(const std::string& leg, double current_time_s,
                                        Eigen::Vector3d& target_position,
                                        Eigen::Vector3d& target_velocity) const;

        /**
         * @brief Get current footstep targets for all legs
         * @param current_time_s The current time in seconds
         * @return A map with leg names as keys and pairs of (position, velocity) as values
         */
        std::unordered_map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector3d>>
        get_all_current_footstep_targets(double current_time_s) const;

        /**
         * @brief The currently-planned swing trajectory for each swinging leg.
         *
         * One trajectory per leg -- the earliest unexpired one, which is the
         * swing in progress or the next to begin. Legs with no planned swing are
         * absent from the result.
         *
         * Exists so a caller running on another thread can take a snapshot and
         * evaluate the splines itself, instead of holding a reference into the
         * planner's deques while the planning thread mutates them. Returning the
         * trajectories rather than a sampled position keeps the caller free to
         * evaluate at its own, faster rate.
         */
        std::unordered_map<std::string, SwingLegTrajectory>
        get_active_swing_trajectories() const;

        /**
         * @brief Enable per-cycle diagnostic printing.
         *
         * Off by default: these paths run once per leg per control cycle at 40 Hz,
         * and unconditional printing here is enough to distort the loop timing
         * being measured.
         */
        void set_verbose(bool verbose) { verbose_ = verbose; }

        /**
         * @brief Diagnostic mode: place every touchdown back at its lift-off point.
         *
         * Disables the Raibert heuristic's contribution, so feet lift and set down
         * in place with no horizontal travel. Used to tell a footstep-placement
         * fault apart from a swing-tracking fault when a gait diverges. The robot
         * cannot translate in this mode.
         */
        void set_step_in_place(bool step_in_place) { step_in_place_ = step_in_place; }

        /**
         * @brief Measured body state for foot placement.
         *
         * Cheetah 3 eq. (33) uses the state estimate for the step being committed.
         * Call this every cycle before update_swing_leg_trajectories(); until it has
         * been called, placement falls back to the reference trajectory.
         * @param p_WBo_W Measured base-origin position, world frame.
         * @param yaw     Measured heading, rad.
         * @param v_WBo_W Measured base-origin velocity, world frame.
         */
        void set_measured_body_state(const Eigen::Vector3d& p_WBo_W, double yaw,
                                     const Eigen::Vector3d& v_WBo_W)
        {
            p_WBo_W_meas_ = p_WBo_W;
            yaw_meas_ = yaw;
            v_WBo_W_meas_ = v_WBo_W;
            have_measured_state_ = true;
        }

        /// Gain k on the velocity-error term k * (v_meas - v_cmd) in foot placement.
        void set_footstep_velocity_gain(double k) { k_v_footstep_ = k; }

        /// Scale on the Raibert v*T/2 term along (x) and across (y) the heading.
        void set_raibert_scale(double sx, double sy) { raibert_scale_ = Eigen::Vector2d(sx, sy); }

        /// Rotate the hip offset by the commanded yaw rate up to mid-stance when placing feet.
        void set_yaw_rate_correction(bool on) { yaw_rate_correction_ = on; }



    private:
        bool verbose_ = false; // Gates the per-cycle diagnostic printing
        bool step_in_place_ = false; // Diagnostic: hold touchdown at lift-off

        // Measured body state for foot placement; see set_measured_body_state().
        Eigen::Vector3d p_WBo_W_meas_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_WBo_W_meas_ = Eigen::Vector3d::Zero();
        double yaw_meas_ = 0.0;
        bool have_measured_state_ = false;
        double k_v_footstep_ = 0.03; // Cheetah-Software convexMPC uses 0.03
        Eigen::Vector2d raibert_scale_ = Eigen::Vector2d(1.0, 1.0);
        bool yaw_rate_correction_ = false;

        double start_time_s_; // Start time of the gait planning in seconds
        double current_time_s_; // Current time in seconds
        double gait_phase_; // Current phase of the gait, between 0 and 1

        double gait_duration_s_; // Duration of the gait in seconds

        double time_to_phase(double current_time_s);
        double phase_to_time(double phase);

        /**
         * @brief Stance test for one leg at a given phase, shared by every contact query.
         * @param phase Gait phase.
         * @param phase_offset This leg's offset within the gait cycle.
         * @return true if the leg is in stance.
         */
        bool leg_in_stance(double phase, double phase_offset) const;
        double swing_height_m_; // Height of the swing trajectory in meters
        GaitType gait_type_;
        double duty_factor_; // Duty factor for the gait, default is 0.5
        std::unordered_map<std::string, double> phase_offsets_; // Phase offsets for each leg in the gait
        // double gait_planning_phase_horizon_; // The horizon length (in phase) for which to compute future footsteps

        std::unordered_map<string, deque<SwingLegTrajectory>>  swing_leg_trajectories_;
 
        std::unordered_map<std::string, Eigen::MatrixXd> desired_footstep_locations_; // Desired footstep locations for each foot in world frame
        // Nominal STANCE FOOT positions in the body frame -- the point each leg
        // hangs its footstep target around. These are the feet, not the hip roll
        // joints.
        //
        // This table previously held y = +/-0.0465, which is the hip ROLL JOINT
        // offset from the body centerline. The foot sits at |y| ~ 0.142 because
        // the thigh is offset outboard of that joint, so every Raibert touchdown
        // target was placed ~0.095 m inboard of where the leg can actually stand:
        // all four feet commanded toward the centerline, unreachable and
        // destabilising. Measured on the pinned rig, the resulting references were
        // 0.23 m above the ground and 0.49 m forward, driving 39-45% torque
        // saturation.
        //
        // Expressed about the BASE ORIGIN Bo, in base axes B (monogram p_BoH_B, H the
        // nominal foothold beneath each hip), because compute_desired_footstep_position()
        // adds them to the body position x[IDX_X], which is Bo since /sportmodestate
        // is shifted from the IMU site to Bo.
        //
        // These previously came from get_foot_positions_body(), which is about the
        // CoM: X 0.1982/-0.1886, Y 0.1403/-0.1437. That is the URDF geometry
        // shifted by the CoM offset (-4.8, +1.7) mm, and added to Bo it placed every
        // foothold 4.8 mm forward and 1.7 mm toward -y -- the "real" y asymmetry
        // noted here before was that offset, not geometry. Values now come straight
        // from the URDF: hip joint (+/-0.1934, +/-0.0465) plus thigh +/-0.0955 in y,
        // which matches FK of the feet at the home keyframe to the millimetre.
        // z is 0: the target is projected to the ground plane.
        // UNUSED: footholds are now frozen at lift-off, so no in-flight rebuild
        // happens at any target excursion. Kept only to document the attempt --
        // this threshold was sized for a target driven by the COMMANDED velocity
        // and became ineffective once the heuristic read the MEASURED velocity,
        // whose within-stride swing moves the target ~130 mm. See the rationale in
        // GaitPlanner::update_swing_leg_trajectories().
        [[maybe_unused]] static constexpr double kRetargetDeadbandM = 0.02;

        const std::unordered_map<std::string, Eigen::Vector3d> p_BoH_B_ = {
            {"FL", Eigen::Vector3d(0.1934, 0.1420, 0.0)},   // Front Left
            {"FR", Eigen::Vector3d(0.1934, -0.1420, 0.0)},  // Front Right
            {"RL", Eigen::Vector3d(-0.1934, 0.1420, 0.0)},  // Rear Left
            {"RR", Eigen::Vector3d(-0.1934, -0.1420, 0.0)}  // Rear Right
        }; // TODO: parse from URDF
        
        /**
         * @brief Set the phase offsets for each leg based on the selected gait type.
         * 
         * @return A map with leg names as keys and contact state (0 or 1) as values.
         */
        void set_phase_offsets(); // Defines gaits based on their phase offsets


        /**
         * @brief Raibert/Cheetah-3 eq. (33) footstep target: p_des = p_hip + v * dt/2.
         * @param x State supplying BOTH the body pose the hip is projected from and the
         *        velocity to sweep by. Reference-based fallback, used only until
         *        set_measured_body_state() has been called; see
         *        compute_desired_footstep_position_measured().
         */
        Eigen::Vector3d compute_desired_footstep_position(const Eigen::VectorXd& x, const string& foot_index);

        /**
         * @brief Footstep target from the MEASURED body state (Cheetah 3 eq. 33, in the
         *        form Cheetah-Software's convexMPC implements):
         *
         *   p_WH_W = p_WBo_W_meas + R_z(yaw_meas [+ wz_cmd (t_td + T_stance/2)]) p_BoH_B
         *            + v_cmd * t_to_touchdown   (bracket: set_yaw_rate_correction)
         *   p_WF_W = p_WH_W + v_meas * T_stance / 2 + k * (v_meas - v_cmd)
         *
         * The first line predicts where the hip will be at touchdown; the second is
         * the Raibert term plus velocity feedback, so a body moving faster than
         * commanded steps further ahead and brakes.
         */
        Eigen::Vector3d compute_desired_footstep_position_measured(const Eigen::Vector3d& v_cmd_W,
                                                                   double wz_cmd,
                                                                   double t_to_touchdown_s,
                                                                   const string& foot_index);

        /**
         * @brief Update the swing leg trajectories based on the current time and footstep positions.
         * @param current_time_s The current time in seconds.
         * @param current_footstep_positions A map of the current footstep positions for each leg, where the key is the leg identifier ("FL", "FR", "RL", "RR") and the value is the 3D position of the foot in body coordinates.
         */
        // void update_swing_leg_trajectories(double current_time_s, unordered_map<string, Vector3d> current_footstep_positions);

        /**
         * @brief Computes the time in seconds until the next stance phase for a given leg.
         * @param current_time_s The current time in seconds.
         * @param leg The leg for which to compute the time to the next stance phase, specified as "FL", "FR", "RL", or "RR".
         * @return The time until the next stance phase in seconds.
         */
        double time_to_next_stance(double current_time_s, const std::string& leg);
        
        double footstep_planning_horizon_s_; // The horizon length (in seconds) for which to compute future footsteps
        double footstep_planning_horizon_phase_; // The horizon length (in phases) for which to compute future footsteps

        // /**
        //  * @brief Get the future stance times for each leg within the planning horizon.
        //  * @param current_time_s The current time in seconds.
        //  * @param footstep_planning_horizon_s The planning horizon in seconds.
        //  * @return A map with leg names ("FL", "FR", "RL", "RR") as keys and a vector of pairs (start_time, end_time) as values.
        //  */
        // std::unordered_map<std::string, std::vector<std::pair<double, double>>> get_future_stance_times(double current_time_s, double footstep_planning_horizon_s);
        
        /**
         * @brief Get the future stance times for each leg within the planning horizon.
         * @param current_time_s The current time in seconds.
         * @param footstep_planning_horizon_s The planning horizon in seconds.
         * @return A map with leg names ("FL", "FR", "RL", "RR") as keys and a vector of pairs (start_time, end_time) as values.
         */
        std::unordered_map<std::string, std::vector<std::pair<double, double>>> get_future_swing_times(double current_time_s, double footstep_planning_horizon_s);

        /**
         * Clears expired swing leg trajectories that have ended before the current time.
         */
        void clear_expired_swing_leg_trajectories();

};


