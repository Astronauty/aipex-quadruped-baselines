#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <Eigen/Dense>

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "convex_mpc/convex_mpc.hpp"
#include "convex_mpc/gait_planner.hpp"
#include "convex_mpc/leg_kinematics.hpp"
#include "convex_mpc/swing_trajectory.hpp"
#include "convex_mpc/quad_params.hpp"

using namespace std;
using namespace Eigen;

// Enum for measurement mode
enum class StateMeasurementMode {
    LOWLEVEL_EKF,
    SPORTMODE,
    MOCAP
};



/**
 * @class QuadConvexMPCNode
 * @brief ROS2 node for quadruped robot control using Convex MPC
 *
 * This node performs the following:
 * 1. Receives robot state information from SportMode and LowState
 * 2. Updates the MPC state representation
 * 3. Solves the convex optimization problem
 * 4. Publishes torque commands to control the robot
 */
class QuadConvexMPCNode : public rclcpp::Node
{
    public:
        QuadConvexMPCNode();
        explicit QuadConvexMPCNode(const rclcpp::NodeOptions & options);
    private:
        std::unique_ptr<ConvexMPC> convex_mpc;
        // Kinematics model owned by publish_cmd, with its own pinocchio::Data so
        // the swing loop shares no state with the planning callback and needs no
        // mutex against it. See LegKinematics for why.
        std::unique_ptr<LegKinematics> swing_kin;
        Eigen::Matrix3d swing_Kp_ = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d swing_Kd_ = Eigen::Matrix3d::Identity();
        std::unique_ptr<MPCParams> mpc_params; // Pointer to MPC parameters
        std::unique_ptr<QuadrupedParams> quadruped_params;

        double start_time_s;
        double elapsed_time_s;

        // MPC State Vars
        float theta[3] = {0.0f}; // Orientation in roll, pitch, yaw
        float p[3] = {0.0f}; // Position in x, y, z
        float omega[3] = {0.0f}; // Angular velocity in roll, pitch, yaw
        float p_dot[3] = {0.0f}; // Linear velocity in x, y, z

        // Standing position reference, latched from the first state message so the
        // stance holds a fixed point instead of being pulled to the world origin.
        // While the stick commands a velocity the latch is carried along with it,
        // so releasing the stick holds wherever the robot ended up.
        double p_ref[2] = {0.0, 0.0};

        // Commanded horizontal body velocity, from the joystick. The swing
        // reference subtracts this rather than the measured body velocity, so a
        // body transient does not propagate into the foot target.
        double cmd_vx_ = 0.0;
        double cmd_vy_ = 0.0;

        // TROTHEALTH trend accumulators, reset every report. Exists to make an
        // intermittent late destabilisation traceable to a drifting quantity
        // instead of a guess: one parseable line per second covering attitude,
        // height, translation, heading, measured support, airborne fraction and
        // swing tracking.
        // Scheduled vs actual touchdown tracking. foot_force is the simulator's
        // measured contact normal force, so this compares the gait schedule
        // against ground truth rather than against another copy of the schedule
        // (which is what STANCESTALE does).
        static constexpr double kContactForceThresholdN = 5.0;

        static constexpr double kTouchdownEarlyWarnMs = 10.0;
        std::array<bool, 4> td_sched_prev_{false, false, false, false};
        std::array<bool, 4> td_contact_prev_{false, false, false, false};
        std::array<double, 4> td_sched_time_{0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> td_contact_time_{0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> td_off_sum_{0.0, 0.0, 0.0, 0.0};
        std::array<double, 4> td_off_absmax_{0.0, 0.0, 0.0, 0.0};
        std::array<int64_t, 4> td_off_n_{0, 0, 0, 0};
        std::array<int64_t, 4> td_early_n_{0, 0, 0, 0};

        int64_t health_samples_ = 0;
        int64_t health_airborne_ = 0;
        double health_roll_sum_ = 0.0;
        double health_pitch_sum_ = 0.0;
        double health_z_sum_ = 0.0;
        double health_vxy_sum_ = 0.0;
        double health_yaw_last_ = 0.0;
        double health_fz_sum_ = 0.0;
        double health_fz_min_ = 1e9;
        double health_perr_sum_ = 0.0;
        double health_tau_pk_ = 0.0;
        bool p_ref_latched = false;
        // MPCParams mpc_params; // MPC parameters

        Matrix<double, 3, 4> foot_positions = Matrix<double, 3, 4>::Zero(); // Foot positions in x, y, z
        float joint_angles[12] = {0.0f}; // Joint angles of Go2
        float joint_velocities[12] = {0.0f}; // Joint velocities of Go2, foot order FL/FR/RL/RR

        // Publisher for joint torque commands
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr joint_torque_pub_;
        // Separate callback groups so the MultiThreadedExecutor can run the swing
        // timer and the planning timer concurrently; see main().
        rclcpp::CallbackGroup::SharedPtr plan_cb_group_;
        rclcpp::CallbackGroup::SharedPtr swing_cb_group_;
        rclcpp::CallbackGroup::SharedPtr state_cb_group_;   // Subscriptions
        rclcpp::TimerBase::SharedPtr publish_joint_torque_timer_; // Fast: swing PD feedback + publish
        rclcpp::TimerBase::SharedPtr update_plan_timer_;          // Slow: MPC solve + gait planning

        // Unitree sportmode vars
        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_sub_;
        float foot_pos[12] = {0.0f};
        float foot_vel[12] = {0.0f};
        
        // Unitree lowstate vars
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

        unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
        unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message
        int16_t foot_force[4] = {0};                 // External contact force value (int)
        int16_t foot_force_est[4] = {0};             // Estimated  external contact force value (int)
        float battery_voltage;                 // Battery voltage
        float battery_current;                 // Battery current

        rclcpp::TimerBase::SharedPtr update_mpc_state_timer_; // Rate to update x0 in MPC based on lowstate

        // Unitree lowcmd
        unitree_go::msg::LowCmd low_cmd;
        rclcpp::TimerBase::SharedPtr cmd_timer_; // Lowcmd publish frequency
        size_t count_;
        StateMeasurementMode state_measurement_mode_;
        bool has_low_state_ = false;
        bool has_sport_mode_state_ = false;

        // Joystick parsing
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        sensor_msgs::msg::Joy::SharedPtr joy_msg; // Joystick message

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg); 
        VectorXd reference_traj_from_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

        void init_cmd(); // Set motors to torque mode and initialize the low_cmd message with default values
        void low_state_callback(const unitree_go::msg::LowState::SharedPtr msg);
        void sport_mode_callback(const unitree_go::msg::SportModeState::SharedPtr data);
        void update_mpc_state();
        void publish_cmd(); // Fast loop: swing PD on fresh joint state, then publish
        void update_plan();  // Slow loop: gait planning and the MPC solve

        // Plan handed from update_plan() to publish_cmd(). Guarded because the two
        // run on separate timers; with a single-threaded executor they cannot
        // actually interleave, but the lock keeps that an implementation detail of
        // the executor rather than a correctness assumption.
        std::mutex plan_mutex_;
        Vector<double, 12> cached_mpc_torques_ = Vector<double, 12>::Zero();
        std::array<bool, 4> cached_stance_ = {true, true, true, true};
        // Swing splines as of the last plan, evaluated by the fast loop at its own
        // rate. Snapshotted rather than queried so the fast loop holds no
        // reference into the planner's deques, which the planning callback
        // mutates each cycle.
        std::unordered_map<std::string, SwingLegTrajectory> cached_swing_trajectories_;

        // (mpc_mutex_ removed: the two callbacks no longer share a ConvexMPC, so
        // there is nothing to guard. Holding it across the solve was exactly what
        // defeated the earlier multi-threaded attempt.)
        bool plan_valid_ = false;
        void log_low_state(const unitree_go::msg::LowState& msg);
        void log_sport_mode_state(const unitree_go::msg::SportModeState& msg);

        // Gait parameters
        // float gait_phase_;
        float gait_duration_s_;

        std::unique_ptr<GaitPlanner> gait_planner;
        void update_gait_phase(float dt);

        // --- Startup stance -> gait handover ---
        // The robot spawns with collapsed legs, so the MPC is given time to stand
        // up before any foot is allowed to leave the ground. Starting a gait from
        // the collapsed pose makes the body roll over within a few cycles.
        GaitType target_gait_type_ = GaitType::STAND; // Gait to switch to once settled
        double target_duty_factor_ = 1.0;             // Duty factor for that gait
        double startup_stance_s_ = 2.0;               // Minimum stance time before handover

        // Runtime gait release. wait_for_gait_trigger_ makes startup_stance_s_
        // advisory and holds STAND until a /start_gait message arrives, so the gait
        // can be started from a confirmed stance without restarting the process
        // (a restart drops /lowcmd, the robot falls, and the gait then begins from
        // a fallen pose).
        bool wait_for_gait_trigger_ = false;
        bool gait_trigger_received_ = false;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_gait_sub_;

        // Diagnostic: hold stance legs with a joint-space PD instead of applying the
        // MPC's GRFs, isolating the swing controller from the GRF-to-torque mapping.
        // Only valid with a welded base -- see the parameter declaration.
        bool freeze_stance_legs_ = false;
        double stance_hold_Kp_ = 80.0;
        double stance_hold_Kd_ = 4.0;
        std::array<double, 12> stance_hold_q_{};  // Posture latched at gait start

        // Cartesian stance hold: the swing legs' own PD law applied to stance feet,
        // with the reference latched at touchdown. See the parameter declaration.
        bool swing_feedforward_ = true;   // Paper eq. (2) feedforward term
        // Nested planning: publish_cmd runs the solve itself every
        // plan_decimation_ ticks instead of a second timer doing it. See
        // publish_cmd() for the rationale and LOOPJITTER for how it is checked.
        bool nest_plan_in_swing_tick_ = true;
        int plan_decimation_ = 5;        // Swing ticks per solve
        int ticks_since_plan_ = 0;
        bool tick_carried_plan_ = false;

        // Tick-spacing census, split by whether the tick carried a solve, so a
        // long-but-periodic plan tick can be told apart from irregular sampling.
        double last_tick_s_ = 0.0;
        double plan_tick_gap_sum_ = 0.0, plan_tick_gap_max_ = 0.0;
        double swing_tick_gap_sum_ = 0.0, swing_tick_gap_max_ = 0.0;
        long plan_tick_count_ = 0, swing_tick_count_ = 0;

        bool skip_mpc_solve_ = false;   // Diagnostic: unstarve the swing timer
        bool swing_natural_freq_gains_ = false;  // Paper eq. (3) gain scheduling
        Eigen::Vector3d swing_omega_{30.0, 30.0, 30.0};  // Target natural freq, rad/s
        double swing_zeta_ = 0.7;                        // Target damping ratio
        bool cartesian_stance_hold_ = false;
        std::array<Eigen::Vector3d, 4> stance_hold_p_{};   // Foot position at touchdown
        std::array<bool, 4> was_in_stance_{};              // Edge detect for re-latching
        std::array<bool, 4> have_held_once_{};             // First hold uses nominal geometry
        std::array<Eigen::Vector3d, 4> nominal_foot_p_{};  // Nominal stance feet, from FK
        bool gait_started_ = true;                    // False while still standing up

        // Commanded standing height, matching the z reference in
        // reference_traj_from_joy(). Reported in the handover log line so the
        // pose the gait inherits is on the record.
        //
        // The handover was previously also gated on roll, pitch and height being
        // within tolerance. The height condition could not be met: stance height
        // converges with a ~34 s time constant toward a residual offset (a
        // constant ~7.7 N force deficit, not a weight-tuning problem), so a 0.05 m
        // tolerance held the gait off for ~35 s and would never be met at all if
        // the residual exceeded it. Roll and pitch settle inside ~2 s and were
        // never the binding condition. The gate is now purely the startup_stance_s
        // timer; fixing the convergence rate is the real remedy.
        static constexpr double STANDING_HEIGHT_M = 0.312;

        // Swing torque saturation census, accumulated over every swing sample in
        // publish_cmd() and reported once a second. This is the measurement that
        // tells whether raising the swing feedback rate fixed the saturation the
        // standalone rig predicted; a throttled per-sample print aliases against
        // the gait period and cannot answer it.
        int JOINT_TORQUE_PUBLISH_RATE_MS_ = 25; // Planning period, kept for diagnostics
        double swing_log_t0_ = 0.0;  // Time origin for the SWINGARC trace
        long publish_cmd_count_ = 0;
        long stance_check_samples_ = 0;
        long stance_mismatch_samples_ = 0;
        long swing_sample_count_ = 0;
        long swing_saturated_count_ = 0;
        double swing_tau_peak_ = 0.0;
        double swing_perr_sum_ = 0.0;
        long swing_perr_samples_ = 0;
        double last_swing_report_s_ = 0.0;

        unordered_map<string, Vector3d> matrix_to_foot_positions_map(const Eigen::Matrix<double, 3, 4>& foot_matrix);

        /**
         * @brief Foot velocities from measured joint velocities, v = J * qdot.
         * @param foot_jacobians Per-foot 3x3 Jacobians, LOCAL_WORLD_ALIGNED, FL/FR/RL/RR.
         */
        Eigen::Matrix<double, 3, 4> compute_foot_velocities(
            const std::vector<Eigen::Matrix3d>& foot_jacobians) const;

        Eigen::VectorXd constrain_reference_trajectory_size(const Eigen::VectorXd& X_ref_input, MPCParams& mpc_params);
};