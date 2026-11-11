#include <chrono>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>
#include <array>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "unitree_hg/msg/low_cmd.hpp"
// #include "unitree_hg/msg/low_state.hpp"
// #include "unitree_hg/msg/motor_cmd.hpp"

#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"

#include "convex_mpc/convex_mpc_node.hpp"
#include "convex_mpc/mpc_params.hpp"
#include "convex_mpc/quad_params.hpp"
#include "convex_mpc/convex_mpc.hpp"
#include "convex_mpc/state_indices.hpp"

// #include "common/motor_crc_hg.h"

using namespace std::chrono_literals;
using namespace Eigen;

// using namespace unitree::common;
// using namespace unitree::robot;

// Helper function to convert string to MeasurementMode
StateMeasurementMode measurement_mode_from_string(const std::string& mode_str) {
    if (mode_str == "lowlevel_ekf") return StateMeasurementMode::LOWLEVEL_EKF;
    if (mode_str == "sportmode") return StateMeasurementMode::SPORTMODE;
    if (mode_str == "mocap") return StateMeasurementMode::MOCAP;
    throw std::invalid_argument("Invalid measurement mode: " + mode_str);
}

// Helper function to convert string to GaitType.
// Only the gaits GaitPlanner::set_phase_offsets() defines offsets for are accepted;
// the others exist in the enum but would throw from deep inside the planner, so
// they are rejected here with a message that names the valid options.
GaitType gait_type_from_string(const std::string& gait_str) {
    if (gait_str == "STAND") return GaitType::STAND;
    if (gait_str == "TROT") return GaitType::TROT;
    if (gait_str == "BOUND") return GaitType::BOUND;
    throw std::invalid_argument(
        "Invalid or unimplemented gait type: '" + gait_str + "'. Supported: STAND, TROT, BOUND.");
}


QuadConvexMPCNode::QuadConvexMPCNode()
    : Node("convex_mpc_controller"), count_(0)
{
    init_cmd();
    // Set measurement mode
    // TODO: ROS param implementation
    // std::string measurement_mode_str = this->declare_parameter<std::string>("measurement_mode", "lowlevel_ekf");
    start_time_s = this->now().seconds();

    state_measurement_mode_ = StateMeasurementMode::SPORTMODE; 

    joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
    joy_msg->axes.resize(8, 0.0);  // Initialize with 8 axes, all zero
    joy_msg->buttons.resize(11, 0); // Initialize with 11 buttons, all zero
    

    // Publisher for joint torque commands
    this->declare_parameter<int>("JOINT_TORQUE_PUBLISH_RATE_MS", 5);
    const int JOINT_TORQUE_PUBLISH_RATE_MS = this->get_parameter("JOINT_TORQUE_PUBLISH_RATE_MS").as_int();
    JOINT_TORQUE_PUBLISH_RATE_MS_ = JOINT_TORQUE_PUBLISH_RATE_MS;

    joint_torque_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

    // Two rates. The MPC solve and gait planning run at JOINT_TORQUE_PUBLISH_RATE_MS
    // (the horizon discretization); the swing PD feedback and torque publish run at
    // SWING_CONTROL_RATE_MS, which is faster.
    //
    // The swing controller is a cheap Cartesian PD law whose tracking is limited by
    // how stale its velocity feedback is, not by gains or by how often the spline
    // reference is sampled -- measured in scripts/swing_leg_rig.py, where raising
    // only the reference rate 40->500Hz changed nothing while raising only the
    // feedback rate 40->200Hz cut peak torque from 61 Nm to 9 Nm and eliminated
    // saturation. Re-solving the QP that fast is neither necessary nor affordable.
    this->declare_parameter<int>("SWING_CONTROL_RATE_MS", 5);
    const int SWING_CONTROL_RATE_MS = this->get_parameter("SWING_CONTROL_RATE_MS").as_int();

    // Must be declared BEFORE the branch below reads it. It used to be declared
    // ~300 lines further down with the other diagnostic params, so the branch ran
    // against the in-class initializer and nesting was always ON regardless of the
    // yaml -- the node logged "Plan nested in the swing tick" with the param false.
    this->declare_parameter("nest_plan_in_swing_tick", true);
    nest_plan_in_swing_tick_ = this->get_parameter("nest_plan_in_swing_tick").as_bool();

    // When nesting, publish_cmd drives the solve itself, so a second timer would
    // double-solve. plan_decimation_ sets how many swing ticks per solve, derived
    // from the two configured rates so the existing params keep their meaning.
    if (nest_plan_in_swing_tick_)
    {
        plan_decimation_ = std::max(1, JOINT_TORQUE_PUBLISH_RATE_MS / SWING_CONTROL_RATE_MS);
        RCLCPP_INFO(this->get_logger(),
            "Plan nested in the swing tick: solve every %d swing ticks (%d ms / %d ms)",
            plan_decimation_, JOINT_TORQUE_PUBLISH_RATE_MS, SWING_CONTROL_RATE_MS);
    }
    else
    {
        plan_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // The group must be PASSED to create_wall_timer. Creating it and leaving
        // the timer on the default argument puts the timer in the node's DEFAULT
        // group, which is MutuallyExclusive -- so both timers serialised against
        // each other no matter how many threads the executor had. Measured: the
        // 5 ms swing timer achieved 80 Hz, not 200 Hz, with max gap ~24 ms, the
        // full length of the 17 ms solve.
        update_plan_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(JOINT_TORQUE_PUBLISH_RATE_MS),
            std::bind(&QuadConvexMPCNode::update_plan, this),
            plan_cb_group_);
    }

    // Separate groups so the two timers can genuinely run concurrently under the
    // MultiThreadedExecutor in main(). Each group is itself mutually exclusive, so
    // neither callback can re-enter itself.
    swing_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    publish_joint_torque_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(SWING_CONTROL_RATE_MS),
        std::bind(&QuadConvexMPCNode::publish_cmd, this),
        swing_cb_group_);

    RCLCPP_INFO(this->get_logger(), "Plan/MPC at %d ms (%.0f Hz), swing PD at %d ms (%.0f Hz)",
                JOINT_TORQUE_PUBLISH_RATE_MS, 1000.0 / JOINT_TORQUE_PUBLISH_RATE_MS,
                SWING_CONTROL_RATE_MS, 1000.0 / SWING_CONTROL_RATE_MS);

    // Create a lowstate subscriber to update mpc states
    // Subscriptions get their own Reentrant group and the executor an extra
    // thread. Once both timers were moved into explicit callback groups, the
    // node's DEFAULT group contained only these subscriptions -- and with two
    // threads occupied by the timers, incoming state was never serviced: the
    // controller ran but has_low_state_ stayed false and it published nothing.
    // Reentrant rather than MutuallyExclusive so state intake is not queued
    // behind another subscription callback.
    // MutuallyExclusive, NOT Reentrant. Reentrant would let two /lowstate
    // messages -- which arrive at 500 Hz -- run low_state_callback concurrently
    // and interleave their writes into the single shared joint_angles /
    // joint_velocities buffers that both timers read, i.e. a torn state vector.
    // Serialising the callbacks costs nothing here (the callback is a remap and a
    // few copies) and the queue depth of 1 means a backlog is dropped, not built.
    state_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions state_sub_opts;
    state_sub_opts.callback_group = state_cb_group_;

    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "lowstate", 1, std::bind(&QuadConvexMPCNode::low_state_callback, this, std::placeholders::_1),
        state_sub_opts
    );

    // Sport mode state subscriber
    // TODO: switch between subscriber callbacks based on measurement_mode
    sport_mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "sportmodestate", 1, std::bind(&QuadConvexMPCNode::sport_mode_callback, this, std::placeholders::_1),
        state_sub_opts
    );

    //Joystick subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&QuadConvexMPCNode::joy_callback, this, std::placeholders::_1),
        state_sub_opts
    );

    // Visualisation outputs. Declared as a parameter so the whole path can be
    // switched off: it costs a serialise-and-publish on the 40 Hz plan timer, and
    // a logging call in this control path has already destabilised the gait once.
    this->declare_parameter<bool>("publish_viz", true);
    this->get_parameter("publish_viz", publish_viz_);
    if (publish_viz_)
    {
        swing_plan_pub_ = this->create_publisher<convex_mpc_msgs::msg::SwingPlanArray>("swing_plan", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(), "Visualisation enabled: /swing_plan, /joint_states, TF world->base");
    }

    // NOTE: update_mpc_state() is called directly at the top of publish_cmd(), which
    // guarantees x0 is fresh for the solve and avoids the timer-skew problem this
    // separate timer was working around. Running both meant doing the state update
    // (and its joystick/foot-position work) twice per control cycle, so the timer is
    // intentionally not created here. MPC_STATE_UPDATE_RATE_MS is still declared for
    // config compatibility but no longer drives a timer.
    this->declare_parameter<int>("MPC_STATE_UPDATE_RATE_MS", 5);


    // Define MPC Params
    this->declare_parameter<int>("N_MPC", 10);
    this->declare_parameter<double>("mpc_dt", 0.050);
    this->declare_parameter<std::vector<double>>(
        "Q_diag", 
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
    );
    this->declare_parameter<double>("Q_scale", 1.0);
    this->declare_parameter<double>("Q_n_scale", 1.0);
    this->declare_parameter<double>("min_vertical_grf", 10.0);
    this->declare_parameter<double>("max_vertical_grf", 400.0);



    const int N_STATES = 13;
    const int N_CONTROLS = 12;
    const int N_MPC = this->get_parameter("N_MPC").as_int(); // Number of steps for the horizon length in MPC
    const double dt = this->get_parameter("mpc_dt").as_double();

    // Specify the diagonal as a vector first
    VectorXd Q_diag(N_STATES);
    auto Q_diag_double = this->get_parameter("Q_diag").as_double_array();

    Q_diag = Map<VectorXd>(Q_diag_double.data(), Q_diag_double.size()) * this->get_parameter("Q_scale").as_double();
    double Q_n_scale = this->get_parameter("Q_n_scale").as_double();
    double min_vertical_grf = this->get_parameter("min_vertical_grf").as_double();
    double max_vertical_grf = this->get_parameter("max_vertical_grf").as_double();

    MatrixXd Q = Q_diag.asDiagonal();
    // R weight balances tracking vs control effort
    // With Q[5] = 50, R = 0.1 gives Q/R = 500, which prioritizes tracking but prevents excessive forces
    // R = 0.5 gives Q/R = 100, more balanced for steady-state operation
    this->declare_parameter<double>("R_weight", 0.1);
    double R_weight = this->get_parameter("R_weight").as_double();
    MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS) * R_weight;
    VectorXd u_lower = VectorXd::Constant(N_CONTROLS, -45.0);
    VectorXd u_upper = VectorXd::Constant(N_CONTROLS, 45.0);

    mpc_params = std::make_unique<MPCParams>(N_MPC, N_CONTROLS, N_STATES,
        dt, Q, Q_n_scale, R, u_lower, u_upper, min_vertical_grf, max_vertical_grf);

    // Per-cycle diagnostic logging and the QP condition-number eigendecomposition
    // together cost ~80ms/cycle, which alone pushes the control loop past mpc_dt.
    // Off by default; enable only when debugging.
    this->declare_parameter<bool>("verbose_diagnostics", false);
    mpc_params->verbose_diagnostics = this->get_parameter("verbose_diagnostics").as_bool();

    // Define Quadruped Params (from https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
    this->declare_parameter<double>("mass", 15.7);
    double mass = this->get_parameter("mass").as_double();

    // Per-joint torque limits [hip, thigh, calf], matching the MuJoCo actuator
    // ctrlranges: abduction/hip 23.7, thigh 23.7, knee/calf 45.43 Nm. The old
    // single 45.0 applied the knee's limit to all three joints.
    this->declare_parameter<std::vector<double>>("joint_torque_limits", {23.7, 23.7, 45.43});
    const std::vector<double> jtl = this->get_parameter("joint_torque_limits").as_double_array();
    if (jtl.size() != 3) {
        throw std::invalid_argument("joint_torque_limits must have 3 elements [hip, thigh, calf]");
    }
    const Eigen::Vector3d joint_torque_limits(jtl[0], jtl[1], jtl[2]);

    // Scalar fallback, kept for the few callers that want one number; use the
    // smallest joint limit so it is conservative rather than optimistic.
    const double torque_limit = joint_torque_limits.minCoeff();

    RCLCPP_INFO(this->get_logger(), "Joint torque limits: hip %.2f, thigh %.2f, calf %.2f Nm",
                jtl[0], jtl[1], jtl[2]);

    this->declare_parameter<double>("mu_static_friction", 1.0);
    double mu = this->get_parameter("mu_static_friction").as_double();


    // Composite inertia of the whole robot about its CoM at the nominal stance
    // (hip 0, thigh 0.9, calf -1.8 -- the MuJoCo "home" keyframe), in body frame.
    //
    // These were previously the base link's own inertia, taken straight from the
    // URDF's <link name="base"> block, while mass below is the whole robot's
    // 16.1 kg. That pairing made the model roughly 5x easier to rotate than its
    // weight implies, because more than half the mass sits in the legs and their
    // parallel-axis contribution dominates: the legs are far enough from the CoM
    // that m*d^2 swamps their own inertia. The MPC therefore under-applied every
    // corrective couple and the shortfall accumulated as a steady backward creep
    // of the body relative to the feet (~1.3 mm/s), which no cost weight touched.
    //
    // Computed two ways that agree to within 5%: summing the URDF links with
    // parallel-axis terms, and summing MuJoCo's body_inertia about subtree_com at
    // the same keyframe. The MuJoCo values are used since that is what the sim
    // integrates. Still a constant, consistent with the fixed-inertia assumption
    // the convex formulation needs -- just evaluated for the whole robot rather
    // than the trunk alone.
    double ixx=0.17029;
    double ixy=0.000122;
    double ixz=-0.016475;
    double iyy=0.483737;
    double iyz=-3.12E-05;
    double izz=0.535372;

    Matrix3d I_b;
    I_b << ixx, ixy, ixz,
            ixy, iyy, iyz,
            ixz, iyz, izz;
    double gravity = 9.81;


    // QuadrupedParams quadruped_params = QuadrupedParams(I_b, mass, gravity, torque_limit, mu);
    quadruped_params = std::make_unique<QuadrupedParams>(I_b, mass, gravity, torque_limit, mu,
                                                         joint_torque_limits);

    // Initialize the ConvexMPC object
    convex_mpc = std::make_unique<ConvexMPC>(*mpc_params, *quadruped_params, this->get_logger());


    // Gait configuration. Defaults reproduce the previous hardcoded standing
    // behaviour (STAND with duty factor 1.0 keeps every foot in stance forever),
    // so a config that omits these is unchanged from before parameterization.
    this->declare_parameter<std::string>("gait_type", "STAND");
    this->declare_parameter("duty_factor", 1.0);
    this->declare_parameter("gait_duration_s", 1.0);
    this->declare_parameter("swing_height_m", 0.08);
    this->declare_parameter("footstep_planning_horizon_s", 0.2);

    const std::string gait_type_str = this->get_parameter("gait_type").as_string();
    const double duty_factor = this->get_parameter("duty_factor").as_double();
    const double gait_duration_s = this->get_parameter("gait_duration_s").as_double();
    const double swing_height_m = this->get_parameter("swing_height_m").as_double();
    const double footstep_planning_horizon_s = this->get_parameter("footstep_planning_horizon_s").as_double();

    const GaitType gait_type = gait_type_from_string(gait_type_str);

    if (duty_factor <= 0.0 || duty_factor > 1.0) {
        throw std::invalid_argument("duty_factor must be in (0, 1]");
    }
    if (gait_duration_s <= 0.0) {
        throw std::invalid_argument("gait_duration_s must be positive");
    }

    // Swing trajectories have to exist before the foot leaves the ground, so the
    // planning horizon needs to reach at least one full gait period ahead.
    if (gait_type != GaitType::STAND && footstep_planning_horizon_s < gait_duration_s) {
        RCLCPP_WARN(this->get_logger(),
            "footstep_planning_horizon_s (%.3f) is shorter than gait_duration_s (%.3f); "
            "swings may be planned late. Consider raising it to at least one gait period.",
            footstep_planning_horizon_s, gait_duration_s);
    }

    // Startup stance: hold all four feet down until the body has settled onto the
    // commanded height and attitude, then hand over to the configured gait.
    //
    // Starting a trot immediately does not work. The robot spawns with its legs
    // collapsed (feet ~0.07 m below the CoM rather than the ~0.31 m of a nominal
    // stance), so the first swings begin before there is a stance to push
    // against: the body rolls over and drifts metres sideways within a few gait
    // cycles. Letting the MPC stand up first gives the gait a stable initial
    // condition, which is what the trot formulation assumes.
    this->declare_parameter("startup_stance_s", 5.0);
    startup_stance_s_ = this->get_parameter("startup_stance_s").as_double();

    // IMU site Io relative to base origin Bo, in B. See p_BoIo_B_ in the header.
    this->declare_parameter("p_BoIo_B",
        std::vector<double>{p_BoIo_B_[0], p_BoIo_B_[1], p_BoIo_B_[2]});
    const std::vector<double> p_BoIo_B = this->get_parameter("p_BoIo_B").as_double_array();
    if (p_BoIo_B.size() != 3) {
        throw std::invalid_argument("p_BoIo_B must have 3 elements");
    }
    p_BoIo_B_ = Eigen::Vector3d(p_BoIo_B[0], p_BoIo_B[1], p_BoIo_B[2]);

    // Reference bounding and yaw latch. See ref_max_pos_error_m_ in the header.
    ref_max_pos_error_m_   = this->declare_parameter("ref_max_pos_error_m", ref_max_pos_error_m_);
    cmd_max_accel_mps2_       = this->declare_parameter("cmd_max_accel_mps2", cmd_max_accel_mps2_);
    cmd_max_yaw_accel_radps2_ = this->declare_parameter("cmd_max_yaw_accel_radps2", cmd_max_yaw_accel_radps2_);
    ref_max_yaw_error_rad_ = this->declare_parameter("ref_max_yaw_error_rad", ref_max_yaw_error_rad_);
    yaw_stick_deadband_    = this->declare_parameter("yaw_stick_deadband", yaw_stick_deadband_);
    yaw_latch_hold_s_      = this->declare_parameter("yaw_latch_hold_s", yaw_latch_hold_s_);
    RCLCPP_INFO(this->get_logger(),
        "Body position: /sportmodestate reports Io; shifting to Bo by p_BoIo_B [%.5f %.5f %.5f]",
        p_BoIo_B_[0], p_BoIo_B_[1], p_BoIo_B_[2]);

    // Stash the target gait; the planner starts in STAND regardless.
    target_gait_type_ = gait_type;
    target_duty_factor_ = duty_factor;
    gait_started_ = (gait_type == GaitType::STAND); // Nothing to switch to

    // Hold in STAND until explicitly released, rather than on a timer alone.
    //
    // This exists because killing and restarting the controller to change gait
    // leaves /lowcmd unpublished for the gap, the robot goes limp and falls, and
    // the new process then starts its trot from a robot already on its side --
    // measured twice as roll -0.73 rad and 59 N of contact force at the first
    // sample, which looks exactly like a trot that diverged. Switching gait inside
    // one process keeps torque continuous across the transition.
    //
    //   ros2 topic pub --once /start_gait std_msgs/msg/Bool "{data: true}"
    //
    // With wait_for_gait_trigger true the timer is ignored and the gait starts only
    // on that message, so an operator can confirm the stance first. This is the
    // mechanism a high-level gait command should eventually drive.
    this->declare_parameter("wait_for_gait_trigger", false);
    wait_for_gait_trigger_ = this->get_parameter("wait_for_gait_trigger").as_bool();
    joy_start_button_ = this->declare_parameter("joy_start_button", joy_start_button_);
    joy_l2_axis_ = this->declare_parameter("joy_l2_axis", joy_l2_axis_);
    joy_damping_button_ = this->declare_parameter("joy_damping_button", joy_damping_button_);
    damping_kd_ = this->declare_parameter("damping_kd", damping_kd_);

    start_gait_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "start_gait", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg->data) return;
            if (gait_started_) {
                RCLCPP_INFO(this->get_logger(), "start_gait ignored: gait already running.");
                return;
            }
            gait_trigger_received_ = true;
            RCLCPP_INFO(this->get_logger(), "start_gait received - releasing stance hold.");
        },
        state_sub_opts);

    RCLCPP_INFO(this->get_logger(),
        "Gait: %s, duty_factor %.3f, period %.3f s, swing height %.3f m, planning horizon %.3f s",
        gait_type_str.c_str(), duty_factor, gait_duration_s, swing_height_m,
        footstep_planning_horizon_s);

    if (!gait_started_) {
        RCLCPP_INFO(this->get_logger(),
            "Holding stance for %.2f s before starting %s.",
            startup_stance_s_, gait_type_str.c_str());
    }

    gait_planner = std::make_unique<GaitPlanner>
    (
        GaitType::STAND, // Always stand first; start_gait() switches over below
        1.0,             // Duty factor 1.0 -> every foot stays in stance
        gait_duration_s,
        swing_height_m,
        footstep_planning_horizon_s,
        this->start_time_s  // Start time
    );
    gait_planner->set_verbose(mpc_params->verbose_diagnostics);

    // Diagnostic: step in place, ignoring the Raibert footstep target. Isolates
    // swing tracking from footstep planning when a gait is unstable. The robot
    // cannot translate while this is on.
    this->declare_parameter("step_in_place", false);
    const bool step_in_place = this->get_parameter("step_in_place").as_bool();
    gait_planner->set_step_in_place(step_in_place);

    // Velocity-error gain in measured-state foot placement: k * (v_meas - v_cmd).
    gait_planner->set_footstep_velocity_gain(
        this->declare_parameter("footstep_velocity_gain", 0.03));
    {
        const auto rs = this->declare_parameter("footstep_raibert_scale", std::vector<double>{1.0, 1.0});
        if (rs.size() != 2) throw std::invalid_argument("footstep_raibert_scale must have 2 elements");
        gait_planner->set_raibert_scale(rs[0], rs[1]);
    }

    if (step_in_place) {
        RCLCPP_WARN(this->get_logger(),
            "step_in_place is enabled: footstep targets are pinned to lift-off, "
            "so the robot will not translate. Diagnostic use only.");
    }

    // Diagnostic: hold stance legs at a fixed joint posture instead of applying the
    // MPC's ground reaction forces, so the swing controller can be tested with the
    // GRF-to-torque mapping completely out of the loop.
    //
    // Only meaningful with the base welded (scripts/make_pinned_scene.py): a free
    // base needs real GRFs to stay upright, and the robot simply collapses. With
    // the base pinned nothing has to be held up, so stance legs are mere anchors
    // and any remaining swing tracking error is the swing controller's alone.
    //
    // If swing tracking is good here but bad with GRFs enabled, the fault is in the
    // stance path; if it is bad here too, it is in the swing path.
    this->declare_parameter("freeze_stance_legs", false);
    freeze_stance_legs_ = this->get_parameter("freeze_stance_legs").as_bool();
    swing_ref_body_xy_measured_ = this->declare_parameter("swing_ref_body_xy_measured", false);
    swing_ref_body_measured_ = this->declare_parameter("swing_ref_body_measured", false);
    mpc_omega_world_frame_ = this->declare_parameter("mpc_omega_world_frame", true);
    swing_ref_meas_max_dev_m_ = this->declare_parameter("swing_ref_meas_max_dev_m", 0.06);

    // Diagnostic: hold stance feet with the swing legs' own Cartesian PD instead of
    // applying the MPC's GRFs, so all twelve joints run one control law and the GRF
    // path is out of the loop. Pairs with a welded base; on a free base the robot
    // has to support itself on position-holding legs, which is a much harder ask.
    // Paper eq. (1)+(2): add the feedforward term Lambda*a_ref + C*qdot + G to
    // the swing feedback. Without it the feedback has to generate the force that
    // accelerates the foot along its arc out of tracking error, which is why the
    // error scales with arc speed (32 mm RMS at a 0.15 s swing vs 11 mm at 1.0 s).
    // Paper eq. (3): schedule Kp/Kd from the operational-space inertia so the
    // closed-loop natural frequency is configuration-independent, instead of
    // using the fixed swing_Kp/swing_Kd. omega is then the thing to tune, and it
    // has physical units (rad/s) rather than being an arbitrary gain.
    // Diagnostic: skip the MPC solve entirely, so the swing timer is not starved
    // by it. Isolates "is the swing loop limited by the solve blocking it?" from
    // the executor refactor needed to fix that properly. Requires
    // freeze_stance_legs, since without the solve there are no stance GRFs.
    // Run the MPC solve inside the swing tick instead of on its own timer. See
    // publish_cmd() for why. Defaults ON: the async-timer arrangement it replaces
    // was measured to cost 37x in swing tracking error.
    this->declare_parameter("skip_mpc_solve", false);
    skip_mpc_solve_ = this->get_parameter("skip_mpc_solve").as_bool();

    this->declare_parameter("swing_natural_freq_gains", false);
    this->declare_parameter("swing_omega", std::vector<double>{30.0, 30.0, 30.0});
    this->declare_parameter("swing_zeta", 0.7);
    swing_natural_freq_gains_ = this->get_parameter("swing_natural_freq_gains").as_bool();
    const std::vector<double> omega_v = this->get_parameter("swing_omega").as_double_array();
    if (omega_v.size() != 3) {
        throw std::invalid_argument("swing_omega must have 3 elements");
    }
    swing_omega_ = Vector3d(omega_v[0], omega_v[1], omega_v[2]);
    swing_zeta_ = this->get_parameter("swing_zeta").as_double();

    if (swing_natural_freq_gains_) {
        RCLCPP_INFO(this->get_logger(),
            "Swing gains scheduled from Lambda (paper eq. 3): omega [%.1f %.1f %.1f] rad/s, zeta %.2f",
            swing_omega_[0], swing_omega_[1], swing_omega_[2], swing_zeta_);
    }

    this->declare_parameter("swing_feedforward", true);
    swing_feedforward_ = this->get_parameter("swing_feedforward").as_bool();
    RCLCPP_INFO(this->get_logger(), "Swing feedforward (Lambda*a_ref + C*qdot + G): %s",
                swing_feedforward_ ? "ON" : "OFF");

    this->declare_parameter("cartesian_stance_hold", false);
    cartesian_stance_hold_ = this->get_parameter("cartesian_stance_hold").as_bool();

    // Nominal stance foot geometry, taken from the same FK the controller uses so
    // it cannot drift from the model. This is the hold target for the first stance
    // entry; see the hold branch in publish_cmd().
    {
        Vector<double, 12> q_nominal;
        for (int leg = 0; leg < 4; leg++) {
            q_nominal[leg * 3 + 0] = 0.0;
            q_nominal[leg * 3 + 1] = 0.9;
            q_nominal[leg * 3 + 2] = -1.8;
        }
        convex_mpc->update_joint_angles(q_nominal);
        const Matrix<double, 3, 4> p_nom = convex_mpc->get_foot_positions_body();
        for (int leg = 0; leg < 4; leg++) {
            nominal_foot_p_[leg] = p_nom.col(leg);
        }
        RCLCPP_INFO(this->get_logger(),
            "Nominal stance hold targets (body frame): "
            "FL %.3f %.3f %.3f | FR %.3f %.3f %.3f | RL %.3f %.3f %.3f | RR %.3f %.3f %.3f",
            p_nom(0,0), p_nom(1,0), p_nom(2,0), p_nom(0,1), p_nom(1,1), p_nom(2,1),
            p_nom(0,2), p_nom(1,2), p_nom(2,2), p_nom(0,3), p_nom(1,3), p_nom(2,3));
    }

    if (cartesian_stance_hold_) {
        RCLCPP_WARN(this->get_logger(),
            "cartesian_stance_hold is enabled: stance feet are held by the swing "
            "Cartesian PD and the MPC's GRFs are NOT applied. Diagnostic use only.");
    }

    // Joint-space hold target: the nominal stance, not whatever pose the legs
    // happen to be in. See the handover block for why it is not latched live.
    for (int leg = 0; leg < 4; leg++) {
        stance_hold_q_[leg * 3 + 0] = 0.0;
        stance_hold_q_[leg * 3 + 1] = 0.9;
        stance_hold_q_[leg * 3 + 2] = -1.8;
    }

    this->declare_parameter("stance_hold_Kp", 80.0);
    this->declare_parameter("stance_hold_Kd", 4.0);
    stance_hold_Kp_ = this->get_parameter("stance_hold_Kp").as_double();
    stance_hold_Kd_ = this->get_parameter("stance_hold_Kd").as_double();

    if (skip_mpc_solve_ && !freeze_stance_legs_) {
        throw std::invalid_argument(
            "skip_mpc_solve requires freeze_stance_legs: with the solve disabled "
            "there are no stance ground reaction forces and the robot has nothing "
            "holding it up. Enable freeze_stance_legs (and use the pinned scene).");
    }

    if (skip_mpc_solve_) {
        RCLCPP_WARN(this->get_logger(),
            "skip_mpc_solve is enabled: the MPC is NOT solved. Stance legs are held "
            "by the joint-space PD only. Diagnostic use only -- this measures the "
            "swing loop without the solve starving its timer.");
    }

    if (freeze_stance_legs_) {
        RCLCPP_WARN(this->get_logger(),
            "freeze_stance_legs is enabled: stance legs are held by a joint-space PD "
            "(Kp %.1f, Kd %.1f) and the MPC's GRFs are NOT applied. Only valid with a "
            "welded base. Diagnostic use only.",
            stance_hold_Kp_, stance_hold_Kd_);
    }

    // Swing-leg Cartesian PD gains, per-axis so lateral and vertical tracking can
    // be weighted differently.
    this->declare_parameter("swing_Kp", std::vector<double>{200.0, 200.0, 200.0});
    this->declare_parameter("swing_Kd", std::vector<double>{10.0, 10.0, 10.0});

    const std::vector<double> swing_Kp = this->get_parameter("swing_Kp").as_double_array();
    const std::vector<double> swing_Kd = this->get_parameter("swing_Kd").as_double_array();

    if (swing_Kp.size() != 3 || swing_Kd.size() != 3) {
        throw std::invalid_argument("swing_Kp and swing_Kd must each have 3 elements");
    }

    convex_mpc->set_swing_gains(
        Vector3d(swing_Kp[0], swing_Kp[1], swing_Kp[2]).asDiagonal(),
        Vector3d(swing_Kd[0], swing_Kd[1], swing_Kd[2]).asDiagonal());

    // The fast control loop gets its own kinematics model, with its own
    // pinocchio::Data, so publish_cmd and update_plan share NOTHING and need no
    // mutex between them. That is what makes the multi-threaded executor work
    // here: the previous attempt failed because publish_cmd's first act was to
    // take a mutex that update_plan held across the entire ~12 ms solve, so the
    // swing timer blocked on the lock instead of the thread and the starvation it
    // was meant to cure simply moved.
    swing_kin = std::make_unique<LegKinematics>(
        std::string(PINOCCHIO_MODEL_DIR) + "/go2_description.urdf",
        quadruped_params->joint_armature, this->get_logger());

    swing_Kp_ = Vector3d(swing_Kp[0], swing_Kp[1], swing_Kp[2]).asDiagonal();
    swing_Kd_ = Vector3d(swing_Kd[0], swing_Kd[1], swing_Kd[2]).asDiagonal();

    RCLCPP_INFO(this->get_logger(), "Swing gains: Kp [%.1f %.1f %.1f], Kd [%.1f %.1f %.1f]",
        swing_Kp[0], swing_Kp[1], swing_Kp[2], swing_Kd[0], swing_Kd[1], swing_Kd[2]);
}


void QuadConvexMPCNode::init_cmd()
{
    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd[i].mode = 0x01; // Set torque mode, 0x00 is passive mode
        low_cmd.motor_cmd[i].q = 0;
        low_cmd.motor_cmd[i].kp = 0;
        low_cmd.motor_cmd[i].dq = 0;
        low_cmd.motor_cmd[i].kd = 0;
        low_cmd.motor_cmd[i].tau = 0;
    }
}

void QuadConvexMPCNode::low_state_callback(unitree_go::msg::LowState::SharedPtr data)
{
    // Info IMU states
    // RPY euler angle(ZYX order respected to body frame)
    // Quaternion
    // Gyroscope (raw data)
    // Accelerometer (raw data)
    imu = data->imu_state;

    // RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]);
        
    theta[0] = imu.rpy[0]; // Roll
    theta[1] = imu.rpy[1]; // Pitch
    theta[2] = imu.rpy[2]; // Yaw

    // RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);

    omega[0] = imu.gyroscope[0]; // Angular velocity in roll
    omega[1] = imu.gyroscope[1]; // Angular velocity in pitch
    omega[2] = imu.gyroscope[2]; // Angular velocity in yaw


    for (int i = 0; i < 12; i++)
    {
        motor[i] = data->motor_state[i];
    }
        // RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
        //             i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
    // Map motor indices to joint angles. Unitree orders the motors FR, FL, RR, RL
    // while this code uses foot order FL, FR, RL, RR, so both the front pair and
    // the rear pair are swapped.
    //
    // Verified empirically against the MuJoCo jointactuatorfrc sensors (which are
    // bound to named joints): commanding /lowcmd index k reports back on the
    // sensor for the joint listed below.
    //   Motors 0-2  = FR  -> joint_angles[3-5]  (Foot 1 / FR)
    //   Motors 3-5  = FL  -> joint_angles[0-2]  (Foot 0 / FL)
    //   Motors 6-8  = RR  -> joint_angles[9-11] (Foot 3 / RR)
    //   Motors 9-11 = RL  -> joint_angles[6-8]  (Foot 2 / RL)
    // Joint velocities follow the identical remap; they feed the swing-leg
    // controller's foot velocity via J * qdot.
    for (int i = 0; i < 3; i++) {
        joint_angles[0 + i] = motor[3 + i].q; // Foot 0 (FL) gets motors 3-5 (FL)
        joint_angles[3 + i] = motor[0 + i].q; // Foot 1 (FR) gets motors 0-2 (FR)
        joint_angles[6 + i] = motor[9 + i].q; // Foot 2 (RL) gets motors 9-11 (RL)
        joint_angles[9 + i] = motor[6 + i].q; // Foot 3 (RR) gets motors 6-8 (RR)

        joint_velocities[0 + i] = motor[3 + i].dq;
        joint_velocities[3 + i] = motor[0 + i].dq;
        joint_velocities[6 + i] = motor[9 + i].dq;
        joint_velocities[9 + i] = motor[6 + i].dq;
    }

    // Measured vertical contact force per foot. In simulation this is filled by a
    // local patch to unitree_mujoco's bridge, which sums mj_contactForce over the
    // contacts involving each foot geom; on hardware it is the Go2's own foot
    // force estimate. Foot order matches the motor remap above: FL, FR, RL, RR.
    for (int i = 0; i < 4; i++) {
        foot_force[i] = data->foot_force[i];
    }

    has_low_state_ = true;

    // Fires at 500Hz and builds a multi-line stringstream per message, so debug-only.
    if (mpc_params && mpc_params->verbose_diagnostics) {
        log_low_state(*data);
    }
    
}

    

void QuadConvexMPCNode::sport_mode_callback(const unitree_go::msg::SportModeState::SharedPtr data)
{
    // Info motion states
    // Gait type and foot raise height
    // Robot position (Odometry frame)
    // Robot velocity (Odometry frame)

    // RCLCPP_INFO(this->get_logger(), "Gait state -- gait type: %d; raise height: %f", data->gait_type, data->foot_raise_height);
    // RCLCPP_INFO(this->get_logger(), "Position -- x: %f; y: %f; z: %f; body height: %f",
    //             data->position[0], data->position[1], data->position[2], data->body_height);
    // RCLCPP_INFO(this->get_logger(), "Velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
    //             data->velocity[0], data->velocity[1], data->velocity[2], data->yaw_speed);


    // Info foot states (foot position and velocity in body frame)
    for (int i = 0; i < 12; i++)
    {
        foot_pos[i] = data->foot_position_body[i];
        foot_vel[i] = data->foot_speed_body[i];
    }

    has_sport_mode_state_ = true;

    // Fires at 500Hz and builds a multi-line stringstream per message, so debug-only.
    if (mpc_params && mpc_params->verbose_diagnostics) {
        log_sport_mode_state(*data);
    }

    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             0, foot_pos[0], foot_pos[1], foot_pos[2], foot_vel[0], foot_vel[1], foot_vel[2]);
    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             1, foot_pos[3], foot_pos[4], foot_pos[5], foot_vel[3], foot_vel[4], foot_vel[5]);
    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             2, foot_pos[6], foot_pos[7], foot_pos[8], foot_vel[6], foot_vel[7], foot_vel[8]);
    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             3, foot_pos[9], foot_pos[10], foot_pos[11], foot_vel[9], foot_vel[10], foot_vel[11]);


    // Update MPC state.
    //
    // /sportmodestate position and velocity are those of the IMU site origin Io,
    // not the base origin Bo: the sim's frame_pos / frame_vel sensors sit on site
    // "imu", p_BoIo_B_ from Bo. Everything downstream -- FK feet in world, the swing
    // references, the TF RViz draws -- treats p as Bo, so it is shifted here. Left
    // unshifted, every world foot read 4.3 cm high and 2.5 cm back (FK minus the
    // sim's own foot positions matched R_WB * p_BoIo_B to 0.1 mm), and the swing
    // references were biased ~4 cm into the ground.
    //
    // Attitude and body rate come from the latest LowState (theta, omega).
    const Eigen::Vector3d p_WIo_W(data->position[0], data->position[1], data->position[2]);
    const Eigen::Vector3d v_WIo_W(data->velocity[0], data->velocity[1], data->velocity[2]);
    const Eigen::Matrix3d R_WB = (Eigen::AngleAxisd(theta[2], Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(theta[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(theta[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d w_WB_B(omega[0], omega[1], omega[2]); // gyro: body axes
    const Eigen::Vector3d p_BoIo_W = R_WB * p_BoIo_B_;
    const Eigen::Vector3d p_WBo_W = p_WIo_W - p_BoIo_W;
    // Io and Bo are points on the same rigid body: v_WIo = v_WBo + w_WB x p_BoIo.
    const Eigen::Vector3d v_WBo_W = v_WIo_W - (R_WB * w_WB_B).cross(p_BoIo_W);

    for (int i = 0; i < 3; i++)
    {
        p[i] = static_cast<float>(p_WBo_W[i]);
        p_dot[i] = static_cast<float>(v_WBo_W[i]);
    }

    
}

void QuadConvexMPCNode::log_low_state(const unitree_go::msg::LowState& msg)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "LowState | IMU rpy: [" << msg.imu_state.rpy[0] << ", "
        << msg.imu_state.rpy[1] << ", " << msg.imu_state.rpy[2] << "]; gyro: ["
        << msg.imu_state.gyroscope[0] << ", " << msg.imu_state.gyroscope[1]
        << ", " << msg.imu_state.gyroscope[2] << "]; accel: ["
        << msg.imu_state.accelerometer[0] << ", " << msg.imu_state.accelerometer[1]
        << ", " << msg.imu_state.accelerometer[2] << "]";

    for (int i = 0; i < 12; ++i)
    {
        const auto& motor_state = msg.motor_state[i];
        oss << "\n  Motor " << i
            << " | q: " << motor_state.q
            << ", dq: " << motor_state.dq
            << ", ddq: " << motor_state.ddq
            << ", tau_est: " << motor_state.tau_est;
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void QuadConvexMPCNode::log_sport_mode_state(const unitree_go::msg::SportModeState& msg)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "SportModeState | pos: [" << msg.position[0] << ", "
        << msg.position[1] << ", " << msg.position[2] << "]; vel: ["
        << msg.velocity[0] << ", " << msg.velocity[1] << ", "
        << msg.velocity[2] << "]; yaw_speed: " << msg.yaw_speed
        << "; gait_type: " << msg.gait_type
        << "; foot_raise_height: " << msg.foot_raise_height
        << "; body_height: " << msg.body_height;

    oss << "\n  Foot forces: [";
    for (int i = 0; i < 4; ++i)
    {
        oss << msg.foot_force[i];
        if (i < 3) oss << ", ";
    }
    oss << "]";

    oss << "\n  Foot positions (body frame):";
    for (int leg = 0; leg < 4; ++leg)
    {
        oss << "\n    Leg " << leg << " : [";
        for (int axis = 0; axis < 3; ++axis)
        {
            int idx = leg * 3 + axis;
            oss << msg.foot_position_body[idx];
            if (axis < 2) oss << ", ";
        }
        oss << "]";
    }

    oss << "\n  Foot velocities (body frame):";
    for (int leg = 0; leg < 4; ++leg)
    {
        oss << "\n    Leg " << leg << " : [";
        for (int axis = 0; axis < 3; ++axis)
        {
            int idx = leg * 3 + axis;
            oss << msg.foot_speed_body[idx];
            if (axis < 2) oss << ", ";
        }
        oss << "]";
    }

    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

void QuadConvexMPCNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> jlock(joy_mutex_);
        this->joy_msg = msg; // Store the joystick message
    }

    // START releases the stance hold, the same as /start_gait. This follows the
    // Unitree remote, where START is "unlock / default gait". Rising edge only, so
    // holding the button does nothing more. joy_start_button is 7 for an
    // XInput pad such as the F710 in X mode; -1 disables it.
    const int b = joy_start_button_;
    const bool pressed = b >= 0 && b < static_cast<int>(msg->buttons.size()) && msg->buttons[b];
    if (pressed && !joy_start_prev_)
    {
        if (gait_started_)
            RCLCPP_INFO(this->get_logger(), "START ignored: gait already running.");
        else
        {
            gait_trigger_received_ = true;
            RCLCPP_INFO(this->get_logger(), "START pressed - releasing stance hold.");
        }
    }
    joy_start_prev_ = pressed;

    // L2 held + B pressed: damping, the Unitree remote's emergency stop. Every
    // joint goes to zero torque with only joint damping, so the robot sinks to the
    // ground instead of being driven. Latched until the controller restarts. On the
    // F710 (X mode) L2 is an analog axis that rests at +1 and reads -1 when fully
    // pressed; "held" is anything below 0.
    const int l2 = joy_l2_axis_;
    const int bb = joy_damping_button_;
    const bool l2_held = l2 >= 0 && l2 < static_cast<int>(msg->axes.size()) && msg->axes[l2] < 0.0f;
    const bool b_pressed = bb >= 0 && bb < static_cast<int>(msg->buttons.size()) && msg->buttons[bb];
    if (l2_held && b_pressed && !joy_damping_prev_ && !damping_active_)
    {
        damping_active_ = true;
        RCLCPP_WARN(this->get_logger(),
            "L2+B - DAMPING: zero torque, joint kd %.2f on all joints. Restart the controller to recover.",
            damping_kd_);
    }
    joy_damping_prev_ = b_pressed;
}

// Hand back a stable reference to the latest joystick message. Copying the
// shared_ptr under the lock means the caller keeps the referenced message alive
// for as long as it needs it, even if joy_callback swaps in a new one midway.
sensor_msgs::msg::Joy::SharedPtr QuadConvexMPCNode::latest_joy_msg()
{
    std::lock_guard<std::mutex> jlock(joy_mutex_);
    return joy_msg;
}

VectorXd QuadConvexMPCNode::reference_traj_from_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    VectorXd X_ref = VectorXd::Zero(mpc_params->N_MPC * mpc_params->N_STATES);

    // Capped for first-light testing of the footstep heuristic under a real
    // velocity command. The Raibert offset is v * t_stance/2, so at duty 0.6 and
    // a 0.8 s period (t_stance 0.48 s) full stick at 0.5 m/s asks for a 120 mm
    // step against a ~390 mm nominal foot spacing -- a large first step for a
    // term that has never been exercised with nonzero velocity. 0.15 m/s gives
    // 36 mm. Raise once translation is shown to be stable.
    double max_translation_vel = 0.15;
    double max_yaw_vel = 0.5;
    
    double height = STANDING_HEIGHT_M; // Desired base-origin height above ground
    double pitch = 0.0; 
    double roll = 0.0; 
    double current_yaw = theta[2]; // Current yaw angle from IMU

    // Extract axes from the joystick message
    // Snapshot the message once. joy_callback replaces this->joy_msg from the
    // state thread while this runs on the plan thread, so re-reading msg->axes
    // mid-function could see a different vector than the one just sized.
    std::vector<float> axes = msg->axes;

    // A published /joy message is NOT guaranteed to carry the axes this code
    // indexes. The constructor seeds joy_msg with 8 zeroed axes, so running the
    // controller standalone is safe, but once joy_node starts publishing it
    // replaces that with whatever the device reported -- and the F710 publishes
    // an EMPTY axes vector until the first stick or trigger event. Indexing
    // axes_double[3] then read out of bounds and segfaulted the controller
    // (exit -11) within seconds, but ONLY under ros2 launch, because standalone
    // runs never get a real joy message. Six standalone soak runs passed while
    // every launch run died.
    //
    // Treat a short message as zero command rather than guessing: a missing axis
    // is not a centred stick, but commanding zero is the safe interpretation.
    constexpr int kAxesNeeded = 4; // indices 0,1,3 are used below
    if (static_cast<int>(axes.size()) < kAxesNeeded)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "/joy carries %zu axes, need %d - commanding zero velocity",
            axes.size(), kAxesNeeded);
        axes.assign(kAxesNeeded, 0.0f);
    }

    Eigen::VectorXd axes_double = Eigen::Map<Eigen::VectorXf>(axes.data(), axes.size()).cast<double>();

    // Calculate the desired translation and rotation velocities
    if (mpc_params && mpc_params->verbose_diagnostics) {
        cout << "Axes: " << axes_double.transpose() << endl;
    }

    std::unordered_map<std::string, int> controller_axis_indices;
    controller_axis_indices["x"] = 1;
    controller_axis_indices["y"] = 0;
    controller_axis_indices["yaw"] = 3;

    // The stick commands a BODY-frame velocity: "forward" means the robot's nose,
    // whatever direction that happens to point. The reference trajectory, though,
    // is WORLD frame -- x0[3..5] is world position and x0[9..11] world velocity,
    // and the Raibert heuristic reads the same slots as world
    // (gait_planner.cpp: "CoM velocity in world frame"). So the command has to be
    // rotated by the current yaw before it goes in.
    //
    // This was previously written straight into the world slots. At yaw = 0 the
    // two frames coincide, which is why trotting in place never exposed it, but at
    // yaw = 90 deg pushing the stick forward drives the robot sideways, and at
    // 180 deg it drives backwards. Only the yaw term matters here: roll and pitch
    // are regulated to zero and a velocity command has no vertical component.
    // Command shaping: the stick is rate-limited before it reaches the reference.
    // A full-stick step (0 -> 0.15 m/s in one plan cycle) made the MPC slam up to
    // ~125 N of horizontal force, overshoot to 0.26-0.28 m/s and roll over, forward
    // and backward alike. Limiting the commanded acceleration bounds the force the
    // step can demand (0.5 m/s^2 on ~15 kg is ~7.5 N). Cheetah-Software smooths the
    // same input with a first-order filter. Called once per plan cycle, so dt is
    // the plan period.
    const double vx_body_raw = max_translation_vel * axes_double[controller_axis_indices["x"]];
    const double vy_body_raw = max_translation_vel * axes_double[controller_axis_indices["y"]];
    const double wz_raw = max_yaw_vel * axes_double[controller_axis_indices["yaw"]];
    {
        const double dv = cmd_max_accel_mps2_ * mpc_params->dt;
        const double dw = cmd_max_yaw_accel_radps2_ * mpc_params->dt;
        cmd_vx_body_shaped_ += std::clamp(vx_body_raw - cmd_vx_body_shaped_, -dv, dv);
        cmd_vy_body_shaped_ += std::clamp(vy_body_raw - cmd_vy_body_shaped_, -dv, dv);
        cmd_wz_shaped_      += std::clamp(wz_raw - cmd_wz_shaped_, -dw, dw);
    }
    const double vx_body = cmd_vx_body_shaped_;
    const double vy_body = cmd_vy_body_shaped_;
    double wz = cmd_wz_shaped_; // Yaw rotation

    const double cy = std::cos(current_yaw);
    const double sy = std::sin(current_yaw);
    double vx = cy * vx_body - sy * vy_body; // world-frame x velocity
    double vy = sy * vx_body + cy * vy_body; // world-frame y velocity

    // Latch the standing position on the first cycle, then advance it only by the
    // commanded velocity. Referencing absolute 0 made any drift from the world
    // origin a standing error the MPC fought; re-anchoring to p every cycle would
    // instead leave x/y with no restoring force at all. Latching gives a fixed
    // target to hold, which is what standing means.
    if (!p_ref_latched)
    {
        p_ref[0] = p[0];
        p_ref[1] = p[1];
        p_ref_latched = true;
        RCLCPP_INFO(this->get_logger(), "Latched standing position reference: x=%.4f y=%.4f",
                    p_ref[0], p_ref[1]);
    }
    else
    {
        // Carry the latch along while a velocity is commanded so that releasing
        // the stick holds the new position rather than pulling back to the old one.
        p_ref[0] += vx * mpc_params->dt;
        p_ref[1] += vy * mpc_params->dt;
    }

    // Keep the latched position within ref_max_pos_error_m_ of the measurement,
    // per axis, as Cheetah-Software does. Inside the box nothing changes, so a
    // centred stick still holds a fixed point; outside it the reference is dragged
    // along, bounding the error after a reset, a push or an estimator jump.
    for (int i = 0; i < 2; i++)
    {
        p_ref[i] = std::clamp(p_ref[i],
                              static_cast<double>(p[i]) - ref_max_pos_error_m_,
                              static_cast<double>(p[i]) + ref_max_pos_error_m_);
    }

    // Cache the commanded horizontal velocity for the swing reference, which
    // subtracts the COMMANDED body velocity rather than the measured one. See
    // the frame discussion in publish_cmd().
    cmd_vx_ = vx;
    cmd_vy_ = vy;

    // Latch the heading on the first cycle, then advance it only by the commanded
    // yaw rate -- the same treatment p_ref gets. With the stick centred this holds
    // the initial heading, so a yaw excursion is a real tracking error that Q[2]
    // can act on; with the stick deflected it sweeps at wz and commands a turn.
    if (!yaw_ref_latched_)
    {
        yaw_ref_ = current_yaw;
        yaw_ref_latched_ = true;
        RCLCPP_INFO(this->get_logger(), "Latched standing yaw reference: %.4f rad", yaw_ref_);
    }
    else
    {
        // Re-anchor while turning, latch once the stick has settled at centre.
        // Integrating wz into a latch let the heading reference run ahead of a
        // body that turns slowly; re-anchoring tracks the commanded rate from
        // wherever the body actually points. Holding the measured heading while
        // the stick is still settling avoids latching mid-turn.
        const double yaw_stick = axes_double[controller_axis_indices["yaw"]];
        const double now_s = this->now().seconds();
        if (std::abs(yaw_stick) > yaw_stick_deadband_)
        {
            yaw_centred_since_s_ = -1.0;
            yaw_ref_ = current_yaw + wz * mpc_params->dt;
        }
        else
        {
            if (yaw_centred_since_s_ < 0.0) yaw_centred_since_s_ = now_s;
            if (now_s - yaw_centred_since_s_ < yaw_latch_hold_s_)
            {
                yaw_ref_ = current_yaw; // settling: follow, then latch where it stops
            }
            // else: latched, hold yaw_ref_
        }
    }

    // Bound the heading error the same way as position. Wrapped so a reference
    // near +pi and a measurement near -pi are treated as close.
    {
        const double yaw_err = std::remainder(yaw_ref_ - current_yaw, 2.0 * M_PI);
        yaw_ref_ = current_yaw + std::clamp(yaw_err, -ref_max_yaw_error_rad_, ref_max_yaw_error_rad_);
    }

    // KNOWN LIMITATION: the yaw reference sweeps across the horizon (wz * k * dt)
    // but the position reference extrapolates along a straight line using the
    // heading at k = 0, so a simultaneous turn-and-translate command describes a
    // chord rather than the arc the robot will actually follow. The cross-track
    // error is bounded by |v| * |wz| * (N_MPC * dt)^2 / 2, which at the current
    // caps (max_translation_vel 0.15 m/s, max_yaw_vel 0.5 rad/s, horizon 0.25 s)
    // is ~1.9 mm at the far end of the horizon -- below the footstep accuracy the
    // Raibert heuristic achieves. It is exactly zero whenever either wz or the
    // translation command is zero, so pure-yaw and pure-translation commands are
    // unaffected. Worth revisiting if the velocity cap is raised or the horizon
    // lengthened; the fix is to integrate the heading per step and rotate the
    // body-frame command inside this loop rather than once before it.
    for (int k=0; k < mpc_params->N_MPC; k++)
    {
        X_ref.segment<13>(k * mpc_params->N_STATES) = (Eigen::Matrix<double, 13, 1>() <<
            0.0, 
            0.0,
            yaw_ref_ + wz * k * mpc_params->dt, // Latched heading + commanded yaw rate
            p_ref[0] + vx * k * mpc_params->dt, // x position (forward/backward movement)
            p_ref[1] + vy * k * mpc_params->dt, // y position (left/right movement)
            height, // z position (height above ground)
            0.0, // Roll rate (fixed for now)
            0.0, // Pitch rate (fixed for now)
            wz, // Yaw rate
            vx, // x velocity
            vy, // y velocity
            0.0, // z velocity (fixed for now)
            9.81 // Gravity state
        ).finished();
    }

    // Every slot, including slot 0, holds the latched reference. Slot 0 used to be
    // overwritten with the measured yaw, x, y, vx and vy, meant to feed Cheetah 3
    // eq. (33) ("the state estimator is used ... for the current time step").
    //
    // It never did: a foothold is frozen at lift-off, ~0.32 s before touchdown and
    // beyond the 0.25 s horizon, so get_state_at_time_from_ref_traj() extrapolates
    // from the LAST slot and never reads slot 0. A bag showed the touchdown target
    // fixed to 0.0000 m while the body drifted 5.5 cm.
    //
    // What it did do was weaken tracking. The condensed QP costs
    // err = A_qp*x0 - X_ref, whose slot 0 is the one-step prediction A*x0. With the
    // measurement in slot 0 that term penalised one step of motion, not distance from
    // the latched target, so one of the N_MPC steps exerted no pull on x, y or yaw.
    // Feeding the state estimate into foot placement belongs in the footstep
    // planner, at the moment a target is frozen, not in the MPC reference.

    return X_ref;
}

void QuadConvexMPCNode::update_mpc_state()
{
    if (!has_low_state_ || !has_sport_mode_state_)
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for initial low_state and sport_mode messages before updating MPC state.");
        return;
    }

    // x = [theta, p, omega, p_dot, g]
    float g = 9.81; // Gravity state

    Eigen::Vector<double, 13> x0;

    // The gyro measures w_WB in body axes (MuJoCo's gyro sensor is in the imu
    // site frame, which is unrotated from B; the Go2's IMU likewise). The MPC
    // model integrates omega as a WORLD-frame vector: theta_dot = R_z^T w_W and
    // w_W_dot = I_W^-1 (r x f). Fed the body-frame rate, the MPC saw roll and pitch
    // rates rotated by the heading psi -- negligible at psi ~ 0, which is where
    // stance and the heading-latched trot spend their time, but a 10-20% roll/pitch
    // cross-coupling at the 0.1-0.2 rad a turn reaches.
    const Eigen::Vector3d w_WB_B(omega[0], omega[1], omega[2]);
    const Eigen::Matrix3d R_WB = (Eigen::AngleAxisd(theta[2], Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(theta[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(theta[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d w_WB_mpc = mpc_omega_world_frame_ ? Eigen::Vector3d(R_WB * w_WB_B) : w_WB_B;

    x0 <<   theta[0], theta[1], theta[2], // Orientation in roll, pitch, yaw
            p[0], p[1], p[2], // Position in x, y, z
            w_WB_mpc(0), w_WB_mpc(1), w_WB_mpc(2), // Angular velocity w_WB, world axes (see above)
            p_dot[0], p_dot[1], p_dot[2], // Velocity in x, y, z
            g; // Gravity state

    // Compact, throttled state trace. Always on: it is a single short line at 20Hz
    // (negligible cost) and gives offline tooling a state trajectory without having
    // to enable the full verbose_diagnostics firehose, which would itself perturb
    // the control loop timing being measured.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 50,
        "STATE %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
        x0(0), x0(1), x0(2), x0(3), x0(4), x0(5),
        x0(6), x0(7), x0(8), x0(9), x0(10), x0(11));

    // Tracking error against the standing reference, for gain tuning. Ordered
    // roll, pitch, x, y, z so the dominant terms are easy to read off.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "ERR roll %.4f pitch %.4f | x %.4f y %.4f z %.4f",
        x0(0), x0(1), x0(3) - p_ref[0], x0(4) - p_ref[1], x0(5) - STANDING_HEIGHT_M);

    // Vertical force actually measured at the feet, against the weight it has to
    // hold. Pair this with the GRF line from the solver: the MPC commands its own
    // sum(fz), and if the measured total differs then the forces being realised
    // are not the ones that were solved for, which separates a force-solution
    // problem from a force-realisation one.
    {
        const double fz_meas = foot_force[0] + foot_force[1] + foot_force[2] + foot_force[3];
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "FMEAS fz %d %d %d %d | sum %.2f",
            foot_force[0], foot_force[1], foot_force[2], foot_force[3], fz_meas);

        // Accumulate for the TROTHEALTH trend line. Sampled here because this runs
        // once per plan cycle on freshly read state, so every term describes the
        // same instant.
        std::lock_guard<std::mutex> hlock(health_mutex_);
        health_samples_++;
        health_roll_sum_  += theta[0];
        health_pitch_sum_ += theta[1];
        health_z_sum_     += p[2];
        health_vxy_sum_   += std::hypot(p_dot[0], p_dot[1]);
        health_yaw_last_   = theta[2];
        health_fz_sum_    += fz_meas;
        health_fz_min_     = std::min(health_fz_min_, fz_meas);
        if (fz_meas <= 0.0) health_airborne_++;
    }

    if (mpc_params->verbose_diagnostics) {
        RCLCPP_INFO(this->get_logger(), "Setting x0 to [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
            x0(0), x0(1), x0(2), x0(3), x0(4), x0(5), x0(6), x0(7), x0(8), x0(9), x0(10), x0(11), x0(12));
    }

    this->convex_mpc->update_x0(x0); // Update rigid body pose state

    // Convert joint_angles array to Eigen::VectorXd
    Eigen::Map<Eigen::VectorXf> joint_angles_eigen(joint_angles, 12);
    Vector<double, 12> joint_angles_double = joint_angles_eigen.cast<double>(); // Convert to double for MPC compatibility
    this->convex_mpc->update_joint_angles(joint_angles_double); // Update joint angles (used for foot Jacobian computation)

    // Reshape foot_pos (12x1 vector) into a 3x4 matrix for MPC
    // foot_pos are in world frame (absolute positions from xrog/TF)
    Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::ColMajor>> mat_map(foot_pos, 3, 4);
    foot_positions = mat_map.cast<double>();
    this->convex_mpc->update_foot_positions(foot_positions); // Update foot positions (world frame, will be converted to COM-relative in MPC)


    // Update the reference trajectory
    // Reference state to repeat
    Eigen::Vector<double, 13> X_ref_single;
    X_ref_single << 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, STANDING_HEIGHT_M, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 9.810000;

    // Repeat X_ref_single N_MPC times to form X_ref
    int N_MPC = mpc_params->N_MPC;
    
    VectorXd X_ref = VectorXd::Zero(mpc_params->N_MPC * mpc_params->N_STATES);
    for (int i = 0; i < N_MPC; ++i) {
        X_ref.segment<13>(i * 13) = X_ref_single;
    }

    // Built once per plan cycle and cached: update_plan() reuses it for the swing
    // planner rather than calling reference_traj_from_joy() a second time, which
    // advanced the latched reference twice per cycle. See X_ref_cycle_.
    X_ref_cycle_ = reference_traj_from_joy(latest_joy_msg());
    const Eigen::VectorXd& X_ref_joy = X_ref_cycle_;
    if (mpc_params->verbose_diagnostics) {
        RCLCPP_INFO(this->get_logger(), "Reference trajectory from joystick: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
            X_ref_joy[0], X_ref_joy[1], X_ref_joy[2], X_ref_joy[3], X_ref_joy[4], X_ref_joy[5],
            X_ref_joy[6], X_ref_joy[7], X_ref_joy[8], X_ref_joy[9], X_ref_joy[10], X_ref_joy[11], X_ref_joy[12]);
    }



    X_ref = constrain_reference_trajectory_size(X_ref_joy, *mpc_params); // Truncate the reference trajectory passed to MPC if the joystick 
    convex_mpc->update_reference_trajectory(X_ref); // Update the reference trajectory in MPC
}


Eigen::VectorXd QuadConvexMPCNode::constrain_reference_trajectory_size(const Eigen::VectorXd& X_ref, MPCParams& mpc_params)
{
    int expected_size = mpc_params.N_MPC * mpc_params.N_STATES;
    if (X_ref.size() < expected_size)
    {
        throw std::invalid_argument("Reference trajectory is too short.");
    }
    else if (X_ref.size() > expected_size)
    {
        RCLCPP_WARN(this->get_logger(), "Reference trajectory is longer than expected. Truncating to fit MPC horizon.");
        return X_ref.head(expected_size);
    }
    return X_ref;
}

/**
 * @brief Slow half of the control loop: gait planning and the MPC solve.
 *
 * Runs at the MPC rate (mpc_dt). Everything here is expensive or inherently tied
 * to the horizon -- the QP, the swing trajectory planning, the contact schedule.
 * Its outputs are cached for publish_cmd(), which runs faster.
 */
void QuadConvexMPCNode::update_plan()
{
    if (!has_low_state_ || !has_sport_mode_state_)
    {
        return;
    }

    // Update MPC state first to ensure x0 is current before solving
    // This prevents solving with uninitialized state when timers are slightly out of sync
    update_mpc_state();

    // One timestamp for the whole cycle. Re-reading the clock between the phase
    // update, the schedule and the trajectory lookup would let them straddle a
    // tick and disagree about which legs are swinging.
    const double now_s = this->now().seconds();

    // Hand over from the startup stance to the walking gait once the startup
    // timer expires. The robot spawns with collapsed legs, so stance time is
    // needed before a foot may leave the ground. This is a fixed duration, not a
    // convergence test: the gait switch is going to become a high-level ROS
    // command, so the node only needs to not trot before the robot is standing.
    //
    // startup_stance_s must cover the measured rise. On the free base the body is
    // still mid-rise at 2.0 s -- pitch -0.0529 against a settled -0.0191, and z
    // 0.1333 still climbing to 0.1647 -- and settles around 4.5 s. Starting the
    // trot from that nose-down, still-moving pose is what put the robot on two
    // diagonal feet carrying minimum GRF. Default raised 2.0 -> 5.0 s.
    //
    // The pose at handover is logged so a failed gait can be read against the
    // stance it started from.
    const bool stance_time_elapsed = (now_s - start_time_s) >= startup_stance_s_;
    const bool release_gait = wait_for_gait_trigger_ ? gait_trigger_received_.load()
                                                     : stance_time_elapsed;
    if (!gait_started_ && release_gait)
    {
        gait_planner->start_gait(target_gait_type_, target_duty_factor_, now_s);
        gait_started_ = true;
        swing_log_t0_ = now_s;

        // The stance hold target is NOT re-latched here. On a welded base the legs
        // collapse before the controller is up (gravity clears the keyframe pose in
        // ~250 ms, Gurobi and Pinocchio take seconds), so latching the measured
        // pose at handover would anchor the hold to the collapsed one. The nominal
        // stance set at construction is the right target; see stance_hold_q_.

        RCLCPP_INFO(this->get_logger(),
            "Startup stance elapsed (%.2f s) - starting gait from "
            "roll %.4f, pitch %.4f, z err %.4f.",
            now_s - start_time_s, theta[0], theta[1], p[2] - STANDING_HEIGHT_M);
    }

    // Advance the gait clock and plan swings BEFORE the solve, so the contact
    // schedule the MPC is constrained against and the trajectories the swing
    // controller follows both describe the same instant.
    gait_planner->update_time_and_phase(now_s);

    // Foot state, measured once per cycle and shared by the planner and the swing
    // controller below so both describe the same geometry.
    //
    // These come from Pinocchio FK, NOT from SportModeState.foot_position_body.
    // The two disagree by ~4.4 cm in z at a settled stance (FK puts the feet at
    // body_z - 0.196 = 0.066 in world terms, while sportmode reports resting feet
    // near 0.022), and mixing them meant the swing controller servoed an FK
    // measurement against a sportmode-seeded reference, fighting a constant
    // offset on every step. FK is the right source here because it is what the
    // Jacobians -- and therefore the torques -- are computed from.
    //
    // Guarded: the fast swing loop drives the same Pinocchio model and would
    // otherwise be able to dirty the FK cache partway through this read.
    // No lock needed: convex_mpc is now private to this (planning) callback.
    // The swing loop owns swing_kin and never touches this instance.
    const std::vector<Matrix3d> foot_jacobians = convex_mpc->get_foot_jacobians();
    const Matrix<double, 3, 4> foot_positions_meas = convex_mpc->get_foot_positions_body();

    // Frames. FK is in BODY axes about the CoM; the planner works in absolute
    // world coordinates. Convert the measurement into world so the trajectories
    // it builds are world-frame, then convert back when tracking.
    const Vector3d p_body_W(p[0], p[1], p[2]);
    const Vector3d com_offset_B = convex_mpc->get_com_offset_body();
    const Matrix3d R_WB = (AngleAxisd(theta[2], Vector3d::UnitZ()) *
                           AngleAxisd(theta[1], Vector3d::UnitY()) *
                           AngleAxisd(theta[0], Vector3d::UnitX())).toRotationMatrix();

    unordered_map<string, Vector3d> foot_positions_map;
    for (int leg_index = 0; leg_index < 4; leg_index++) {
        foot_positions_map[quadruped_params->LEG_INDEX_TO_NAME.at(leg_index)] =
            p_body_W + R_WB * (foot_positions_meas.col(leg_index) + com_offset_B);
    }

    // Same reference the MPC was given this cycle (built in update_mpc_state()).
    if (X_ref_cycle_.size() == 0) return;
    const Eigen::VectorXd& X_ref_joy = X_ref_cycle_;
    // Foot placement reads the measured body state (Cheetah 3 eq. 33).
    gait_planner->set_measured_body_state(
        p_body_W, theta[2], Vector3d(p_dot[0], p_dot[1], p_dot[2]));
    const auto swing_plans = gait_planner->update_swing_leg_trajectories(X_ref_joy, *mpc_params, foot_positions_map);

    // Visualisation, from the plan that was just built so the picture and the
    // control decision describe the same instant.
    publish_visualization(now_s, swing_plans, foot_positions_meas,
                          p_body_W, R_WB, com_offset_B, X_ref_joy);

    // Contact sampled per horizon step, not frozen at k=0 -- see
    // ConvexMPC::set_contact_constraints(schedule).
    const auto contact_schedule = gait_planner->get_contact_schedule(
        now_s, mpc_params->dt, mpc_params->N_MPC - 1);

    // Solve GRFs from MPC, unless the solve is disabled.
    //
    // skip_mpc_solve exists to answer one question without refactoring the
    // executor: does the swing loop actually improve when it stops being starved?
    // The ~12 ms solve is what blocks the 5 ms swing timer for half of every
    // planning cycle, producing the 3/5/17 ms spacing. Skipping it leaves the
    // gait clock, the contact schedule and the swing trajectories running exactly
    // as before, so the swing path is unchanged and the ONLY difference is the
    // timer no longer competing with a 12 ms callback.
    //
    // Only valid with freeze_stance_legs (which supplies stance torques from a
    // joint-space PD instead of GRFs) -- otherwise there is nothing holding the
    // robot up. Guarded below.
    Vector<double, 12> mpc_joint_torques = Vector<double, 12>::Zero();
    double constraints_ms = 0.0, solve_ms = 0.0;
    if (!skip_mpc_solve_)
    {
        const double t_c0 = this->now().seconds();
        convex_mpc->set_contact_constraints(contact_schedule); // Enforce friction constraints or zero GRF constraints based on contact state
        const double t_c1 = this->now().seconds();
        mpc_joint_torques = convex_mpc->solve_joint_torques();
        const double t_c2 = this->now().seconds();
        constraints_ms = (t_c1 - t_c0) * 1000.0;
        solve_ms = (t_c2 - t_c1) * 1000.0;
    }

    // Cache what the fast control loop needs. It re-reads joint state itself, but
    // the plan -- which feet are in stance, what the stance torques are -- only
    // changes at this slower rate.
    //
    // The footstep targets are snapshotted here rather than queried from the fast
    // loop, so that loop holds no reference into gait_planner's trajectory
    // deques, which update_swing_leg_trajectories() mutates just above.
    // Snapshotting also keeps the swing reference consistent with the contact
    // schedule it was planned against. The spline is re-evaluated at the fast rate inside publish_cmd();
    // what is cached here is the trajectory set, not a frozen sample.
    auto swing_trajectories = gait_planner->get_active_swing_trajectories();
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        cached_mpc_torques_ = mpc_joint_torques;
        cached_stance_ = contact_schedule[0];
        cached_swing_trajectories_ = std::move(swing_trajectories);
        plan_valid_ = true;
    }

    // How long the planning callback occupies the single executor thread. If this
    // approaches or exceeds JOINT_TORQUE_PUBLISH_RATE_MS, the faster publish_cmd
    // timer cannot get a slot and its effective rate collapses.
    const double plan_ms = (this->now().seconds() - now_s) * 1000.0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "PLANTIME update_plan %.2f ms (budget %d ms) | constraints %.2f | solve %.2f | planning %.2f",
        plan_ms, JOINT_TORQUE_PUBLISH_RATE_MS_, constraints_ms, solve_ms,
        plan_ms - constraints_ms - solve_ms);
}

/**
 * @brief Foot velocities from measured joint velocities, v_i = J_i * qdot_i.
 *
 * Derived from the joint encoders rather than read from SportModeState's
 * foot_speed_body: that field is populated in the MuJoCo bridge from cvel, a
 * spatial velocity about the body origin, which is not the point velocity of
 * the foot that a Cartesian PD law needs. J * qdot uses the same Jacobians as
 * the torque mapping, so the derivative term is consistent with the gain it
 * multiplies.
 *
 * @param foot_jacobians Per-foot 3x3 Jacobians, LOCAL_WORLD_ALIGNED, FL/FR/RL/RR.
 * @return 3x4 matrix of foot velocities, one column per foot, same frame.
 */
Eigen::Matrix<double, 3, 4> QuadConvexMPCNode::compute_foot_velocities(
    const std::vector<Eigen::Matrix3d>& foot_jacobians) const
{
    Eigen::Matrix<double, 3, 4> foot_velocities;
    for (int foot = 0; foot < 4; foot++)
    {
        Eigen::Vector3d qdot_leg;
        for (int j = 0; j < 3; j++) {
            qdot_leg[j] = static_cast<double>(joint_velocities[foot * 3 + j]);
        }
        foot_velocities.col(foot) = foot_jacobians[foot] * qdot_leg;
    }
    return foot_velocities;
}

/**
 * @brief Fast half of the control loop: swing PD feedback and torque publish.
 *
 * Runs faster than the MPC. The swing controller is a cheap Cartesian PD law, and
 * in the standalone rig (scripts/swing_leg_rig.py) its tracking is limited by how
 * stale the velocity feedback is rather than by gains or by how often the spline
 * reference is sampled: at 40Hz, Kd=40 saturates the actuators (61 Nm, 55% of
 * samples) while at 200Hz the same gains use 4 Nm and track ~3x better. The
 * break is around Kd*dt ~ 1.
 *
 * So the reference and the stance torques are held from the last plan, and only
 * the feedback -- foot position and velocity from fresh joint encoders -- is
 * recomputed here.
 */
void QuadConvexMPCNode::publish_cmd()
{
    if (!has_low_state_ || !has_sport_mode_state_) return;

    // Nested planning: run the solve INSIDE this tick rather than on its own
    // timer. Mirrors the reference implementation
    // (github.com/elijah-waichong-chan/go2-convex-mpc), whose control loop is
    //
    //     if k % CTRL_DECIM == 0:            # control rate
    //         if ctrl_i % STEPS_PER_MPC == 0:
    //             solve_QP(...)              # nested, same tick
    //         compute_leg_torque(...)
    //
    // Two async ROS timers on one executor let the 12 ms solve land anywhere
    // inside the 5 ms timer's period, which produced the measured 3/5/17 ms
    // spacing. Nesting makes the plan/swing ratio an exact integer and
    // guarantees the swing law runs immediately after the solve, on the same
    // state, every time.
    //
    // It does NOT by itself make every tick equal length: the tick that carries
    // the solve is still ~12 ms longer. Whether that residual matters is exactly
    // what the LOOPJITTER diagnostic below measures -- it reports the spacing
    // split by whether the tick carried a solve, so a long-but-periodic tick can
    // be told apart from genuinely irregular sampling.
    if (nest_plan_in_swing_tick_)
    {
        if (ticks_since_plan_ >= plan_decimation_ || !plan_valid_)
        {
            ticks_since_plan_ = 0;
            update_plan();
            tick_carried_plan_ = true;
        }
        else
        {
            tick_carried_plan_ = false;
        }
        ticks_since_plan_++;
    }

    // Tick-spacing census, split by whether this tick also ran the solve. A
    // uniform loop shows one tight cluster; a starved one shows the plan ticks
    // long and the others short, which is the 3/5/17 ms pattern.
    {
        const double now_tick = this->now().seconds();
        if (last_tick_s_ > 0.0) {
            const double gap_ms = (now_tick - last_tick_s_) * 1000.0;
            if (tick_carried_plan_) {
                plan_tick_gap_sum_ += gap_ms;
                plan_tick_gap_max_ = std::max(plan_tick_gap_max_, gap_ms);
                plan_tick_count_++;
            } else {
                swing_tick_gap_sum_ += gap_ms;
                swing_tick_gap_max_ = std::max(swing_tick_gap_max_, gap_ms);
                swing_tick_count_++;
            }
        }
        last_tick_s_ = now_tick;
    }

    Vector<double, 12> mpc_joint_torques;
    std::array<bool, 4> stance;
    std::unordered_map<std::string, SwingLegTrajectory> swing_trajectories;
    {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        if (!plan_valid_) return; // No plan solved yet; stay quiet rather than command zeros
        mpc_joint_torques = cached_mpc_torques_;
        stance = cached_stance_;
        swing_trajectories = cached_swing_trajectories_;
    }

    // Fresh kinematics from the latest encoder values. update_joint_angles marks
    // the FK cache dirty, so the Jacobians and foot positions below are recomputed
    // against the joint state as of this tick, not as of the last MPC solve.
    const double now_s = this->now().seconds();

    // One critical section for every use of the shared Pinocchio model: setting
    // the joint angles dirties the FK cache, so the Jacobians and foot positions
    // must be read before the planning thread can set its own angles.
    // No lock: swing_kin is this loop's private model. The planning thread
    // uses convex_mpc and never touches this one, so there is nothing to guard.
    Eigen::Map<Eigen::VectorXf> ja(joint_angles, 12);
    swing_kin->set_joint_angles(ja.cast<double>());

    // Needed by the feedforward's Coriolis term; harmless otherwise.
    Eigen::Map<Eigen::VectorXf> jv(joint_velocities, 12);
    swing_kin->set_joint_velocities(jv.cast<double>());

    const std::vector<Matrix3d> foot_jacobians = swing_kin->foot_jacobians();
    const Matrix<double, 3, 4> foot_positions_meas = swing_kin->foot_positions_body();
    const Matrix<double, 3, 4> foot_velocities_meas = compute_foot_velocities(foot_jacobians);

    const Vector3d p_body_W(p[0], p[1], p[2]);
    const Vector3d com_offset_B = swing_kin->com_offset_body();
    const Matrix3d R_WB = (AngleAxisd(theta[2], Vector3d::UnitZ()) *
                           AngleAxisd(theta[1], Vector3d::UnitY()) *
                           AngleAxisd(theta[0], Vector3d::UnitX())).toRotationMatrix();

    // Swing references are evaluated from the cached trajectories at the current
    // time. This is cheap -- a spline evaluation -- and the rig showed the
    // reference rate itself does not matter, but evaluating it here costs nothing
    // and keeps position and velocity references consistent with this instant.
    // Evaluate the snapshotted splines at this instant. This is the part that
    // genuinely benefits from the fast rate: the reference and the measured foot
    // state are sampled together, so the PD error is not computed against a
    // stale reference.
    // Position, velocity AND acceleration: the acceleration feeds the paper's
    // eq. (2) feedforward term, Lambda*a_ref.
    struct SwingRef { Vector3d p, v, a; };
    std::unordered_map<std::string, SwingRef> footstep_targets;
    for (const auto& [leg, traj] : swing_trajectories)
    {
        if (now_s < traj.get_start_time() || now_s > traj.get_end_time()) continue;
        Vector3d pos, vel, acc;
        traj.evaluate(now_s, pos, vel, acc);
        footstep_targets[leg] = {pos, vel, acc};
    }

    // Loop-rate census, counted unconditionally so it does not depend on how
    // many legs happen to be swinging. Reported alongside SWINGSAT.
    publish_cmd_count_++;

    // Decide stance/swing from the gait clock at THIS instant, not from the flag
    // cached by the last plan. publish_cmd runs several times per planning cycle,
    // so the cached set can be a planning period out of date; measured at 5% of
    // leg-samples during healthy stepping. Across a phase flip that means a leg
    // which has just lifted still receives the MPC's stance torque -- a force
    // sized to push against the ground, applied to a foot now in the air -- and a
    // leg which has just landed is still driven by an expired swing spline.
    //
    // The gait schedule is a pure function of time, so evaluating it here is exact
    // and costs only the phase arithmetic. The cached set is still used as the
    // fallback before the gait starts, when the planner is in STAND and every leg
    // is in stance anyway.
    std::array<bool, 4> solved_stance = stance;  // What the cached GRFs assumed
    if (gait_started_)
    {
        const auto live = gait_planner->get_contact_schedule(now_s, mpc_params->dt, 1);
        for (int i = 0; i < 4; i++)
        {
            stance_check_samples_++;
            if (live[0][i] != stance[i]) stance_mismatch_samples_++;
            stance[i] = live[0][i];
        }


    }

    Vector<double, 12> joint_torques = Vector<double, 12>::Zero();
    for (int leg_index = 0; leg_index < 4; leg_index++)
    {
        const std::string& leg = quadruped_params->LEG_INDEX_TO_NAME.at(leg_index);
        const int joint_start_idx = leg_index * 3;

        if (stance[leg_index])
        {
            // Only hold stance legs once the gait has actually started. Before
            // handover the MPC is standing the robot up, and the hold posture has
            // not been latched yet -- servoing to an unlatched (zero) target would
            // command straight legs and tip the robot over during the settle.
            // Applies before handover too, not just once the gait starts: the MPC
            // would otherwise spend the startup stance walking the feet outward
            // (measured on a welded base, front feet sliding 0.124 -> 0.456 and
            // locking at full extension), and the swing splines would then be
            // seeded from that. With the hold active from the first tick the feet
            // stay where the keyframe put them.
            if (cartesian_stance_hold_)
            {
                // Diagnostic: hold the stance foot where it landed, using the SAME
                // Cartesian PD law the swing legs use. The MPC's GRF solution is not
                // applied at all, so every one of the twelve joints is driven by
                // tau = J^T (Kp (p_ref - p) + Kd (v_ref - v)) and nothing else.
                //
                // This tests the swing control law on all four legs at once. If a
                // leg tracks its swing arc well but cannot hold a fixed point while
                // bearing the body, the fault is in the law rather than in the
                // trajectory; if both work, the law is sound and the problem is in
                // the GRF path it replaces.
                //
                // p_ref is latched when the leg ENTERS stance, not at gait start, so
                // it is the actual touchdown point rather than a pose that drifts
                // further out of date every step.
                if (!was_in_stance_[leg_index])
                {
                    // First stance entry uses the NOMINAL stance geometry rather
                    // than the measured foot position. On a welded base the legs
                    // have always collapsed by the time the controller is up: with
                    // no body weight to hold them, gravity takes them off the
                    // keyframe pose within ~250 ms, well before Gurobi and
                    // Pinocchio finish loading. Latching the measured position
                    // would anchor the hold to that collapsed pose and the feet
                    // would never return. Later touchdowns latch the measurement,
                    // which by then is a real foothold.
                    stance_hold_p_[leg_index] =
                        have_held_once_[leg_index] ? foot_positions_meas.col(leg_index)
                                                   : nominal_foot_p_[leg_index];
                    have_held_once_[leg_index] = true;
                    was_in_stance_[leg_index] = true;
                }

                joint_torques.segment<3>(joint_start_idx) =
                    swing_leg_torques(
                        *swing_kin, leg_index, foot_jacobians[leg_index],
                        swing_Kp_, swing_Kd_,
                        foot_positions_meas.col(leg_index),
                        foot_velocities_meas.col(leg_index),
                        stance_hold_p_[leg_index], Vector3d::Zero(), Vector3d::Zero(),
                        /*use_feedforward=*/false);
                continue;
            }

            if (freeze_stance_legs_)
            {
                // Diagnostic: anchor the leg at the posture it was holding when the
                // gait started, with a plain joint-space PD. No Jacobian, no GRF, no
                // MPC -- so nothing from the stance path can reach the swing legs
                // through the plant.
                for (int j = 0; j < 3; j++)
                {
                    const int k = joint_start_idx + j;
                    joint_torques[k] = stance_hold_Kp_ * (stance_hold_q_[k] - joint_angles[k])
                                     - stance_hold_Kd_ * joint_velocities[k];
                }
                continue;
            }

            // Stance torques come from the MPC and are held between solves: the
            // GRF is a horizon quantity and does not become more correct by being
            // re-read at a higher rate.
            //
            // But only if that solve actually treated this leg as stance. A leg
            // that has just touched down is stance by the live gait clock while
            // the cached solve still had it in swing, where its GRF was
            // constrained to exactly zero -- so reusing that torque would command
            // ~0 Nm to a leg now expected to bear load, and the body drops onto
            // it. Hold the foot in place until the next solve covers it instead.
            if (!solved_stance[leg_index])
            {
                const Vector3d p_hold = foot_positions_meas.col(leg_index);
                const Vector3d v_hold = foot_velocities_meas.col(leg_index);
                joint_torques.segment<3>(joint_start_idx) =
                    swing_leg_torques(
                        *swing_kin, leg_index, foot_jacobians[leg_index],
                        swing_Kp_, swing_Kd_, p_hold, v_hold,
                        p_hold, Vector3d::Zero(), Vector3d::Zero(),
                        /*use_feedforward=*/false);
                continue;
            }

            joint_torques.segment<3>(joint_start_idx) =
                mpc_joint_torques.segment<3>(joint_start_idx);
            continue;
        }

        was_in_stance_[leg_index] = false;  // Swing: next touchdown re-latches the hold point

        // Everything below is in the BASE frame, because that is the frame the
        // Jacobian maps. get_foot_jacobians() asks Pinocchio for
        // LOCAL_WORLD_ALIGNED, but the model is FIXED BASE -- there is no floating
        // joint for Pinocchio to align to the world, so its "world" is the base
        // link and the Jacobian's axes are base axes. JACCHECK confirms this: J
        // matches a finite difference of oMf[].translation(), which is exactly
        // what get_foot_positions_body() returns.
        //
        // These were previously rotated into world axes by R_WB while J^T still
        // expected base axes. The two agreed only while the body was level, which
        // is why standing was unaffected. Tilted, a purely vertical commanded
        // error leaked sideways: at the observed roll of 0.17 rad a 5 cm z error
        // put -8.4 mm into y, a force on an axis the swing trajectory never
        // commands -- and self-reinforcing, since the resulting y motion feeds
        // back into roll.
        const Vector3d p_meas = foot_positions_meas.col(leg_index);
        const Vector3d v_meas = foot_velocities_meas.col(leg_index);

        auto target = footstep_targets.find(leg);
        if (target == footstep_targets.end())
        {
            // No trajectory for this leg: damp in place rather than go limp.
            joint_torques.segment<3>(joint_start_idx) =
                swing_leg_torques(
                    *swing_kin, leg_index, foot_jacobians[leg_index],
                    swing_Kp_, swing_Kd_, p_meas, v_meas,
                    p_meas, Vector3d::Zero(), Vector3d::Zero(),
                    /*use_feedforward=*/false);
            continue;
        }

        // The trajectory is planned in world coordinates, so bring it into the
        // base frame to match p_meas: subtract the body origin, rotate by R_WB^T,
        // then shift to the CoM-relative origin get_foot_positions_body() uses.
        //
        // The body origin used here is the COMMANDED one, not the measured one.
        //
        // Subtracting the measured body position makes the foot reference inherit
        // every body excursion: target_W is a fixed point on the ground, so
        // ref_z ~ -(body height). Measured on the free base, a body that rose to
        // z = 0.406 m produced ref_z = -0.4874, i.e. the foot was commanded 487 mm
        // below the body when the leg only reaches ~330 mm. The swing PD then drove
        // the foot down at Kp * 0.15-0.20 m = 60-80 N into the ground; single feet
        // registered 107-124 N at touchdown (near full bodyweight), which threw the
        // body higher, which raised the reference further. A positive feedback loop
        // that bounced the robot between 124 N and fully airborne, and the MPC's
        // stance force then correctly collapsed to min_vertical_grf because the body
        // was above its height reference.
        //
        // Referencing the commanded body pose breaks the loop: the swing arc is
        // tracked relative to where the body is SUPPOSED to be, so a body transient
        // no longer propagates into the foot target. This is also what the Cheetah
        // formulation assumes -- a well-regulated body -- and it is why the welded
        // rig never saw this: there the body genuinely is where it is commanded, so
        // the two forms coincide (perr 0.027 m, 0% saturation).
        //
        // Attitude still uses the measured R_WB: the legs do have to be aimed at
        // the real ground, and roll/pitch are regulated far better than height.
        // Option: horizontal body reference from the MEASURED body. At speed the
        // body trails the commanded one by up to the reference clamp (4 cm), and
        // referencing swing to the commanded body then steers every foot that far
        // from its world target. Height stays commanded either way -- that is the
        // part that broke the height feedback loop described above.
        const Vector3d p_body_cmd_W = swing_ref_body_xy_measured_
            ? Vector3d(p[0], p[1], STANDING_HEIGHT_M)
            : Vector3d(p_ref[0], p_ref[1], STANDING_HEIGHT_M);
        Vector3d p_ref_B =
            R_WB.transpose() * (target->second.p - p_body_cmd_W) - com_offset_B;

        // Option: the MEASURED body in all three axes, as Cheetah-Software does
        // (pDesLeg = rBody * (pDesFootWorld - seResult.position)). The foot then
        // tracks the world spline itself instead of the spline shifted by the
        // body's error: with the commanded body, a body riding 4 cm high flew every
        // foot ~4 cm above its spline (correlation 0.7-0.85) and landed it late.
        // The 10-09 loop above came from a body at 0.406 m, before the IMU-site,
        // mass and QP fixes. As a guard against it, the reference may differ from
        // the commanded-body one by at most swing_ref_meas_max_dev_m per axis.
        if (swing_ref_body_measured_)
        {
            const Vector3d p_ref_meas_B =
                R_WB.transpose() * (target->second.p - Vector3d(p[0], p[1], p[2])) - com_offset_B;
            const Vector3d dev = (p_ref_meas_B - p_ref_B)
                .cwiseMax(-swing_ref_meas_max_dev_m_).cwiseMin(swing_ref_meas_max_dev_m_);
            p_ref_B += dev;
        }
        const Vector3d p_ref = p_ref_B;

        // Likewise for velocity, against the COMMANDED body velocity. v_meas is
        // J*qdot, the foot's velocity relative to the base in base axes, so the
        // body's own velocity has to come out of the world-frame reference before
        // rotating it in. The commanded body velocity is the joystick command,
        // with zero vertical component -- a standing or trotting-in-place body is
        // not meant to be moving vertically, and feeding the measured vz in is the
        // velocity-domain half of the same feedback loop.
        // With swing_ref_body_measured, the full measured body velocity, as
        // Cheetah-Software's vDesLeg = rBody * (vDesFootWorld - seResult.vWorld).
        const Vector3d v_body_cmd_W = swing_ref_body_measured_
            ? Vector3d(p_dot[0], p_dot[1], p_dot[2])
            : swing_ref_body_xy_measured_
            ? Vector3d(p_dot[0], p_dot[1], 0.0)
            : Vector3d(cmd_vx_, cmd_vy_, 0.0);
        const Vector3d v_ref =
            R_WB.transpose() * (target->second.v - v_body_cmd_W);

        // Reference acceleration, rotated into the base frame like p_ref/v_ref.
        //
        // INCOMPLETE: unlike p_ref and v_ref, this does NOT subtract the body's own
        // acceleration. On the welded rig that is exact -- the base cannot
        // accelerate -- and swing tracked to perr 0.027 m at 0% saturation there.
        // On a free base it is wrong: the body term leaks into Lambda*a_ref, and
        // with swing_feedforward on, the trot showed 42% saturation at 100-109 Nm
        // peaks (limits are 23.7 hip/thigh, 45.43 calf). The swing legs drove into
        // the ground hard enough to throw the body from z 0.330 to 0.408 in 0.8 s,
        // after which the MPC correctly cut stance force to min_vertical_grf
        // because the body was 96 mm ABOVE its height reference.
        //
        // The missing term is measurable, not unavailable: LowState carries
        // imu_state.accelerometer (the bridge fills it from MuJoCo's imu_acc
        // sensor, and it exists on hardware), but the node currently only logs it
        // in low_state_callback. Wiring it in means subtracting the body's
        // proper acceleration -- accelerometer minus gravity, rotated into the base
        // frame -- from target->second.a here. Until then swing_feedforward must
        // stay OFF on a free base; eq. (2) is only valid with the full term.
        const Vector3d a_ref = R_WB.transpose() * target->second.a;

        // Three selectable laws:
        //   swing_natural_freq_gains: paper eq. (3), gains scheduled from Lambda
        //   swing_feedforward:        paper eq. (2) feedforward on fixed gains
        //   neither:                  plain fixed-gain Cartesian PD
        const Vector3d tau = swing_natural_freq_gains_
            ? swing_leg_torques_scheduled(
                  *swing_kin, leg_index, foot_jacobians[leg_index],
                  swing_omega_, swing_zeta_,
                  p_meas, v_meas, p_ref, v_ref, a_ref, swing_feedforward_)
            : swing_leg_torques(
                  *swing_kin, leg_index, foot_jacobians[leg_index],
                  swing_Kp_, swing_Kd_,
                  p_meas, v_meas, p_ref, v_ref, a_ref, swing_feedforward_);

        joint_torques.segment<3>(joint_start_idx) = tau;

        // Saturation census over every swing sample. Counted here, pre-clamp,
        // because this is the question the split-rate loop was built to answer:
        // the standalone rig showed swing tracking is feedback-rate limited, and
        // at 40 Hz 71% of swing samples were pinned to the actuator limit. A
        // throttled per-sample print cannot measure that -- it samples 1 in 20 and
        // aliases against the gait half-period -- so accumulate over all samples
        // and report the rate periodically instead.
        for (int j = 0; j < 3; j++) {
            swing_sample_count_++;
            if (std::abs(tau[j]) >= quadruped_params->joint_torque_limit(j)) {
                swing_saturated_count_++;
            }
        }
        swing_tau_peak_ = std::max(swing_tau_peak_, tau.cwiseAbs().maxCoeff());
        swing_perr_sum_ += (p_ref - p_meas).norm();
        {
            std::lock_guard<std::mutex> hlock(health_mutex_);
            health_tau_pk_ = std::max(health_tau_pk_, tau.cwiseAbs().maxCoeff());
            health_perr_sum_ += (p_ref - p_meas).norm();
            health_perr_n_++;
        }
        swing_perr_samples_++;

        // Reference breakdown. perr reaching values far larger than the robot
        // means the reference itself has run away, not that the PD is tracking
        // badly, so log the world-frame spline output and the body-frame target
        // it converts to alongside the measurement. Throttled per leg, otherwise
        // a single shared throttle starves whichever legs come later in the loop.
        // Per-axis swing trace for the arc visualisation. For a step in place with
        // zero commanded velocity the Raibert term is zero, so x and y should be
        // flat and only z should move -- the foot tip going straight up and back
        // down. Deviation in x or y is the thing to look for.
        RCLCPP_INFO(this->get_logger(),
            "SWINGARC %s t=%.3f | ref %+7.4f %+7.4f %+7.4f | meas %+7.4f %+7.4f %+7.4f"
            " | dx %+6.4f dy %+6.4f dz %+6.4f | q %+7.4f %+7.4f %+7.4f"
            " | dq %+8.3f %+8.3f %+8.3f | tau %+7.3f %+7.3f %+7.3f",
            leg.c_str(), now_s - swing_log_t0_,
            p_ref.x(), p_ref.y(), p_ref.z(),
            p_meas.x(), p_meas.y(), p_meas.z(),
            p_meas.x() - p_ref.x(), p_meas.y() - p_ref.y(), p_meas.z() - p_ref.z(),
            // Joint state and the torque actually commanded, so this trace can be
            // compared directly against scripts/swing_mujoco_pd.py, which drives
            // the same arc through joint-space position servos. If the Cartesian
            // law is mistuned the joint trajectories will diverge even where the
            // Cartesian reference agrees.
            joint_angles[joint_start_idx + 0], joint_angles[joint_start_idx + 1],
            joint_angles[joint_start_idx + 2],
            joint_velocities[joint_start_idx + 0], joint_velocities[joint_start_idx + 1],
            joint_velocities[joint_start_idx + 2],
            tau[0], tau[1], tau[2]);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "SWINGREF %s | spline_W %+7.3f %+7.3f %+7.3f | body_W %+7.3f %+7.3f %+7.3f"
            " | p_ref_B %+7.3f %+7.3f %+7.3f | p_meas_B %+7.3f %+7.3f %+7.3f | perr %.4f",
            leg.c_str(),
            target->second.p[0], target->second.p[1], target->second.p[2],
            p_body_W[0], p_body_W[1], p_body_W[2],
            p_ref[0], p_ref[1], p_ref[2],
            p_meas[0], p_meas[1], p_meas[2],
            (p_ref - p_meas).norm());
    }

    // Periodic saturation summary. Reported on a plain elapsed-time check rather
    // than RCLCPP_*_THROTTLE so the counters reset with the report and each line
    // covers one disjoint window.
    {
        const double since_report_s = now_s - last_swing_report_s_;
        if (since_report_s >= 1.0)
        {
            // One line per second carrying everything needed to see a slow
            // divergence coming, so an intermittent late failure can be traced to
            // a trend rather than guessed at. Printed unconditionally and in a
            // fixed column order so it can be parsed straight out of the log.
            //
            // The quantities are chosen to separate the candidate mechanisms:
            //   roll/pitch      - attitude drift, the axis that actually fails
            //   z               - height drift, which moves the swing aiming error
            //                     because p_ref uses the COMMANDED height
            //   |v_xy|          - unintended translation; the Raibert term feeds
            //                     commanded velocity, so real drift is not planned for
            //   yaw             - accumulated heading error, which rotates the
            //                     footstep targets away from the stance geometry
            //   fz_sum          - measured support; a sagging mean means the stance
            //                     is losing the body
            //   airborne        - fraction of samples with NO foot loaded, the
            //                     signature of the bouncing failure mode
            if (gait_started_)
            {
                std::lock_guard<std::mutex> hlock(health_mutex_);
                const double n = static_cast<double>(std::max<int64_t>(health_samples_, 1));
                RCLCPP_INFO(this->get_logger(),
                    "TROTHEALTH t=%.1f | roll %+.4f pitch %+.4f z %.4f | |v_xy| %.4f"
                    " yaw %+.4f | fz_sum mean %.1f min %.1f | airborne %.0f%%"
                    " | swing_perr %.4f | tau_pk %.1f",
                    now_s - swing_log_t0_,
                    health_roll_sum_ / n, health_pitch_sum_ / n, health_z_sum_ / n,
                    health_vxy_sum_ / n, health_yaw_last_,
                    health_fz_sum_ / n, health_fz_min_,
                    100.0 * static_cast<double>(health_airborne_) / n,
                    health_perr_n_ ? health_perr_sum_ / static_cast<double>(health_perr_n_) : 0.0,
                    health_tau_pk_);
                health_samples_ = 0;
                health_roll_sum_ = health_pitch_sum_ = health_z_sum_ = 0.0;
                health_vxy_sum_ = health_fz_sum_ = health_perr_sum_ = 0.0;
                health_perr_n_ = 0;
                health_fz_min_ = 1e9; health_airborne_ = 0; health_tau_pk_ = 0.0;
            }

            if (stance_check_samples_ > 0)
            {
                RCLCPP_INFO(this->get_logger(),
                    "STANCESTALE %.1f%% of leg-samples commanded with a stale stance flag (%ld/%ld)",
                    100.0 * static_cast<double>(stance_mismatch_samples_) /
                        static_cast<double>(stance_check_samples_),
                    stance_mismatch_samples_, stance_check_samples_);
                stance_check_samples_ = 0;
                stance_mismatch_samples_ = 0;
            }

            if (plan_tick_count_ > 0 || swing_tick_count_ > 0)
            {
                RCLCPP_INFO(this->get_logger(),
                    "LOOPJITTER swing-only ticks: mean %.2f ms max %.2f ms (n=%ld)"
                    " | plan-carrying ticks: mean %.2f ms max %.2f ms (n=%ld)",
                    swing_tick_count_ ? swing_tick_gap_sum_ / swing_tick_count_ : 0.0,
                    swing_tick_gap_max_, swing_tick_count_,
                    plan_tick_count_ ? plan_tick_gap_sum_ / plan_tick_count_ : 0.0,
                    plan_tick_gap_max_, plan_tick_count_);
                plan_tick_gap_sum_ = 0.0; plan_tick_gap_max_ = 0.0; plan_tick_count_ = 0;
                swing_tick_gap_sum_ = 0.0; swing_tick_gap_max_ = 0.0; swing_tick_count_ = 0;
            }

            RCLCPP_INFO(this->get_logger(),
                "LOOPRATE publish_cmd %.1f Hz (%ld calls in %.2f s) | swing samples %ld",
                publish_cmd_count_ / since_report_s, publish_cmd_count_,
                since_report_s, swing_sample_count_);

            if (swing_sample_count_ > 0)
            {
                RCLCPP_INFO(this->get_logger(),
                "SWINGSAT %.0f%% saturated (%ld/%ld joint-samples) | peak |tau| %.2f Nm | mean perr %.4f m",
                100.0 * static_cast<double>(swing_saturated_count_) /
                    static_cast<double>(swing_sample_count_),
                swing_saturated_count_, swing_sample_count_, swing_tau_peak_,
                swing_perr_samples_ > 0 ? swing_perr_sum_ / swing_perr_samples_ : 0.0);
            }

            publish_cmd_count_ = 0;
            swing_sample_count_ = 0;
            swing_saturated_count_ = 0;
            swing_tau_peak_ = 0.0;
            swing_perr_sum_ = 0.0;
            swing_perr_samples_ = 0;
            last_swing_report_s_ = now_s;
        }
    }

    // Per-joint clamp: hip/thigh 23.7 Nm, calf 45.43. Stance torques were already
    // clamped in solve_joint_torques(), but the swing torques are computed here.
    for (int i = 0; i < 12; i++) {
        const double limit = quadruped_params->joint_torque_limit(i % 3);
        joint_torques[i] = std::clamp(joint_torques[i], -limit, limit);
    }

    // Remap to Unitree motor order (FR, FL, RR, RL) from foot order (FL, FR, RL, RR).
    for (int i = 0; i < 3; i++) {
        low_cmd.motor_cmd[0 + i].tau = joint_torques[3 + i];
        low_cmd.motor_cmd[3 + i].tau = joint_torques[0 + i];
        low_cmd.motor_cmd[6 + i].tau = joint_torques[9 + i];
        low_cmd.motor_cmd[9 + i].tau = joint_torques[6 + i];
    }

    // Damping (L2+B, see joy_callback) overrides every joint command.
    if (damping_active_)
    {
        for (int i = 0; i < 12; i++)
        {
            low_cmd.motor_cmd[i].q = 0;
            low_cmd.motor_cmd[i].kp = 0;
            low_cmd.motor_cmd[i].dq = 0;
            low_cmd.motor_cmd[i].kd = damping_kd_;
            low_cmd.motor_cmd[i].tau = 0;
        }
    }

    joint_torque_pub_->publish(low_cmd);
}

unordered_map<string, Vector3d> QuadConvexMPCNode::matrix_to_foot_positions_map(const Eigen::Matrix<double, 3, 4>& foot_matrix)
{
    unordered_map<string, Vector3d> foot_map;
    const std::array<std::string, 4> leg_names = {"FL", "FR", "RL", "RR"};
    
    for (int i = 0; i < 4; ++i) {
        foot_map[leg_names[i]] = foot_matrix.col(i);
    }
    
    return foot_map;
}

int main(int argc, char * argv[])
{
    try{
        rclcpp::init(argc, argv);

        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        // auto node = std::make_shared<QuadConvexMPCNode>(options);
        auto node = std::make_shared<QuadConvexMPCNode>();

        // Multi-threaded, so the 5 ms swing timer runs while the ~12 ms solve is in
        // progress on the other thread. This works now only because the two
        // callbacks no longer share any state: update_plan owns `convex_mpc` and
        // publish_cmd owns `swing_kin`, a LegKinematics instance with its own
        // pinocchio::Data. The plan itself is handed over through plan_mutex_,
        // which is held for four assignments (microseconds), not across the solve.
        //
        // The earlier attempt at this failed for a specific reason worth recording:
        // publish_cmd's first act was to take mpc_mutex_, which update_plan held
        // across the ENTIRE solve. A std::mutex makes the second thread wait, it
        // does not let it skip, so the swing timer blocked on the lock for the full
        // 12 ms and the starvation simply moved from the thread to the mutex.
        // STAND degraded to max |roll| 3.14 as a result. Removing the shared state
        // is what fixes it; a better lock could not.
        // The node puts its callbacks in explicit callback groups, and a group is
        // only serviced by an executor the node is added to. rclcpp::spin() builds
        // a SingleThreadedExecutor and add_node() picks up only AUTOMATICALLY_ADD
        // groups -- i.e. the default group. With the swing timer in swing_cb_group_
        // and the subscriptions in state_cb_group_, spin() serviced neither: the
        // process ran, construction logged normally, and then nothing was ever
        // called. publish_cmd never emitted a single LOOPRATE line.
        //
        // Three threads: one per timer group plus one for state intake, so neither
        // timer can be delayed by the other or by a burst of 500 Hz lowstate.
        rclcpp::executors::MultiThreadedExecutor executor(
            rclcpp::ExecutorOptions(), 3);
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
    }
    catch(GRBException& e) {
        std::cerr << "Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << "Gurobi error message: " << e.getMessage() << std::endl;
        return 1;
    }
    catch(const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

/**
 * @brief Publish everything RViz needs to draw the gait: TF, joint states, and
 *        the planned swing splines with both measured foot sources.
 *
 * Deliberately output-only. The viz node does all marker construction, so this
 * function just serialises state the control path already computed -- no extra
 * kinematics, no logging, no blocking I/O. It runs on the 40 Hz plan timer and
 * never on the 200 Hz swing timer.
 *
 * Everything is published in the WORLD frame, which is the frame the whole swing
 * pipeline works in. That is a deliberate choice for a debugging tool: a frame
 * error should show up as visibly wrong geometry rather than being absorbed by a
 * transform on the way to the screen.
 */
void QuadConvexMPCNode::publish_visualization(
    double now_s,
    const std::unordered_map<std::string, std::deque<SwingLegTrajectory>>& plans,
    const Eigen::Matrix<double, 3, 4>& foot_positions_meas,
    const Eigen::Vector3d& p_body_W,
    const Eigen::Matrix3d& R_WB,
    const Eigen::Vector3d& com_offset_B,
    const Eigen::VectorXd& X_ref)
{
    if (!publish_viz_ || !swing_plan_pub_) return;

    const rclcpp::Time stamp = this->now();

    // --- TF: world -> base. Without this the URDF sits at the origin and the
    // swing markers (which are in world) appear detached from the robot.
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "world";
        tf.child_frame_id = "base";
        tf.transform.translation.x = p_body_W.x();
        tf.transform.translation.y = p_body_W.y();
        tf.transform.translation.z = p_body_W.z();
        const Eigen::Quaterniond q(R_WB);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);
    }

    // --- Joint states, for robot_state_publisher to pose the URDF.
    // joint_angles[] is in FL, FR, RL, RR order with hip/thigh/calf within each
    // leg, which is exactly the URDF's joint order, so this is a direct copy.
    {
        static const std::array<const char*, 12> kJointNames = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

        sensor_msgs::msg::JointState js;
        js.header.stamp = stamp;
        js.name.assign(kJointNames.begin(), kJointNames.end());
        js.position.resize(12);
        for (int i = 0; i < 12; ++i) js.position[i] = joint_angles[i];
        joint_state_pub_->publish(js);
    }

    // --- The gait plan itself.
    convex_mpc_msgs::msg::SwingPlanArray msg;
    msg.stamp = stamp;
    msg.controller_time_s = now_s;

    msg.body_pose.position.x = p_body_W.x();
    msg.body_pose.position.y = p_body_W.y();
    msg.body_pose.position.z = p_body_W.z();
    const Eigen::Quaterniond q_body(R_WB);
    msg.body_pose.orientation.x = q_body.x();
    msg.body_pose.orientation.y = q_body.y();
    msg.body_pose.orientation.z = q_body.z();
    msg.body_pose.orientation.w = q_body.w();

    msg.body_velocity.x = p_dot[0];
    msg.body_velocity.y = p_dot[1];
    msg.body_velocity.z = p_dot[2];

    // Reference, to see commanded against measured in the same picture.
    msg.reference_position.x = X_ref(IDX_X);
    msg.reference_position.y = X_ref(IDX_X + 1);
    msg.reference_position.z = X_ref(IDX_X + 2);
    msg.reference_velocity.x = X_ref(IDX_Xd);
    msg.reference_velocity.y = X_ref(IDX_Xd + 1);
    msg.reference_velocity.z = X_ref(IDX_Xd + 2);
    msg.reference_yaw = X_ref(IDX_Y);

    // Both foot sources, so their disagreement is visible rather than inferred.
    // FK is body axes about the CoM, so it needs the same conversion the planner
    // uses; sportmode already carries world-frame values despite its field name.
    // Solved ground reaction forces (previous cycle's solve), for the bag.
    {
        const auto& grf_W = convex_mpc->get_last_grf_world();
        msg.grf_world.resize(4);
        for (int leg = 0; leg < 4; leg++)
        {
            msg.grf_world[leg].x = grf_W(0, leg);
            msg.grf_world[leg].y = grf_W(1, leg);
            msg.grf_world[leg].z = grf_W(2, leg);
        }
    }
    msg.feet_from_fk.resize(4);
    msg.feet_from_sportmode.resize(4);
    msg.stance.resize(4);
    const auto contact = gait_planner->get_contact_state();
    for (int leg = 0; leg < 4; ++leg)
    {
        const Eigen::Vector3d fk_W =
            p_body_W + R_WB * (foot_positions_meas.col(leg) + com_offset_B);
        msg.feet_from_fk[leg].x = fk_W.x();
        msg.feet_from_fk[leg].y = fk_W.y();
        msg.feet_from_fk[leg].z = fk_W.z();

        msg.feet_from_sportmode[leg].x = foot_pos[3 * leg + 0];
        msg.feet_from_sportmode[leg].y = foot_pos[3 * leg + 1];
        msg.feet_from_sportmode[leg].z = foot_pos[3 * leg + 2];

        const std::string& name = quadruped_params->LEG_INDEX_TO_NAME.at(leg);
        const auto it = contact.find(name);
        msg.stance[leg] = (it != contact.end()) && (it->second != 0);
    }

    // Sample each planned spline. Sampling here rather than in the viz node keeps
    // the spline maths in one place -- a visualiser that re-derived the curve
    // could draw something the swing controller never actually tracked.
    constexpr int kSamplesPerSwing = 20;
    for (const auto& [leg, trajectories] : plans)
    {
        for (const auto& traj : trajectories)
        {
            convex_mpc_msgs::msg::SwingPlan plan;
            plan.leg = leg;
            plan.start_time_s = traj.get_start_time();
            plan.end_time_s = traj.get_end_time();
            plan.swing_height_m = traj.get_swing_height();

            const Eigen::Vector3d p0 = traj.get_start_position();
            const Eigen::Vector3d p2 = traj.get_end_position();
            plan.start_position.x = p0.x();
            plan.start_position.y = p0.y();
            plan.start_position.z = p0.z();
            plan.end_position.x = p2.x();
            plan.end_position.y = p2.y();
            plan.end_position.z = p2.z();

            const double t0 = traj.get_start_time();
            const double t2 = traj.get_end_time();
            plan.sampled_points.resize(kSamplesPerSwing);
            for (int i = 0; i < kSamplesPerSwing; ++i)
            {
                const double t = t0 + (t2 - t0) * static_cast<double>(i) /
                                          static_cast<double>(kSamplesPerSwing - 1);
                Eigen::Vector3d p, v, a;
                traj.evaluate(t, p, v, a);
                plan.sampled_points[i].x = p.x();
                plan.sampled_points[i].y = p.y();
                plan.sampled_points[i].z = p.z();
            }

            // Where the reference wants the foot right now versus where it is.
            // These two being far apart IS the swing tracking error -- the thing
            // that reads as swing_perr in the health line, made visible.
            plan.in_flight = (now_s > t0) && (now_s < t2);
            Eigen::Vector3d p_ref_now, v_ref_now, a_ref_now;
            traj.evaluate(plan.in_flight ? now_s : t0, p_ref_now, v_ref_now, a_ref_now);
            plan.reference_position.x = p_ref_now.x();
            plan.reference_position.y = p_ref_now.y();
            plan.reference_position.z = p_ref_now.z();

            const int leg_index = quadruped_params->LEG_NAME_TO_INDEX.at(leg);
            const Eigen::Vector3d meas_W =
                p_body_W + R_WB * (foot_positions_meas.col(leg_index) + com_offset_B);
            plan.measured_position.x = meas_W.x();
            plan.measured_position.y = meas_W.y();
            plan.measured_position.z = meas_W.z();

            msg.plans.push_back(plan);
        }
    }

    swing_plan_pub_->publish(msg);
}
