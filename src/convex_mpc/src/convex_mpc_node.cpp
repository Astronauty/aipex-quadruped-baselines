#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <stdexcept>
#include <Eigen/Dense>


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


QuadConvexMPCNode::QuadConvexMPCNode()
    : Node("convex_mpc_controller"), count_(0)
{
    init_cmd();
    // Set measurement mode
    // TODO: ROS param implementation
    // std::string measurement_mode_str = this->declare_parameter<std::string>("measurement_mode", "lowlevel_ekf");
    start_time_s = this->now().seconds();

    state_measurement_mode_ = StateMeasurementMode::SPORTMODE; 

    float theta[3] = {0.0, 0.0, 0.0}; // Orientation in roll, pitch, yaw
    float p[3] = {0.0, 0.0, 0.0}; // Position in x, y, z
    float omega[3] = {0.0, 0.0, 0.0}; // Angular velocity in roll, pitch, yaw
    float p_dot[3] = {0.0, 0.0, 0.0}; // Linear velocity in x, y, z

    joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
    joy_msg->axes.resize(8, 0.0);  // Initialize with 8 axes, all zero
    joy_msg->buttons.resize(11, 0); // Initialize with 11 buttons, all zero
    

    // Publisher for joint torque commands
    this->declare_parameter<int>("JOINT_TORQUE_PUBLISH_RATE_MS", 10);
    const int JOINT_TORQUE_PUBLISH_RATE_MS = this->get_parameter("JOINT_TORQUE_PUBLISH_RATE_MS").as_int();

    joint_torque_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);
    publish_joint_torque_timer_ = this->create_wall_timer(std::chrono::milliseconds(JOINT_TORQUE_PUBLISH_RATE_MS), std::bind(&QuadConvexMPCNode::publish_cmd, this));

    // Create a lowstate subscriber to update mpc states
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
        "lowstate", 1, std::bind(&QuadConvexMPCNode::low_state_callback, this, std::placeholders::_1)
    );

    // Sport mode state subscriber
    // TODO: switch between subscriber callbacks based on measurement_mode
    sport_mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "sportmodestate", 1, std::bind(&QuadConvexMPCNode::sport_mode_callback, this, std::placeholders::_1)
    );

    //Joystick subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&QuadConvexMPCNode::joy_callback, this, std::placeholders::_1)
    );

    // TODO: Set correct rate for the timer
    this->declare_parameter<int>("MPC_STATE_UPDATE_RATE_MS", 10);
    const int MPC_STATE_UPDATE_RATE_MS = this->get_parameter("MPC_STATE_UPDATE_RATE_MS").as_int();
    update_mpc_state_timer_ = this->create_wall_timer(std::chrono::milliseconds(MPC_STATE_UPDATE_RATE_MS), std::bind(&QuadConvexMPCNode::update_mpc_state, this));


    // Define MPC Params
    this->declare_parameter<int>("N_MPC", 10);
    this->declare_parameter<double>("mpc_dt", 0.050);
    this->declare_parameter<std::vector<double>>(
        "Q_diag", 
        {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
    );
    this->declare_parameter<double>("Q_scale", 1.0);


    const int N_STATES = 13;
    const int N_CONTROLS = 12;
    const int N_MPC = this->get_parameter("N_MPC").as_int(); // Number of steps for the horizon length in MPC
    const double dt = this->get_parameter("mpc_dt").as_double();

    // Specify the diagonal as a vector first
    VectorXd Q_diag(N_STATES);
    auto Q_diag_double = this->get_parameter("Q_diag").as_double_array();
    Q_diag = Map<VectorXd>(Q_diag_double.data(), Q_diag_double.size()) * this->get_parameter("Q_scale").as_double();
    MatrixXd Q = Q_diag.asDiagonal();

    MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS);

    VectorXd u_lower = VectorXd::Constant(N_CONTROLS, -45.0);
    VectorXd u_upper = VectorXd::Constant(N_CONTROLS, 45.0);

    mpc_params = std::make_unique<MPCParams>(N_MPC, N_CONTROLS, N_STATES,
        dt, Q, R, u_lower, u_upper);

    // Define Quadruped Params (from https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
    this->declare_parameter<double>("mass", 6.921);
    this->declare_parameter<double>("torque_limit", 45.0);

    double ixx=0.02448;
    double ixy=0.00012166;
    double ixz=0.0014849;
    double iyy=0.098077;
    double iyz=-3.12E-05;
    double izz=0.107;

    Matrix3d I_b; // TODO: extract from URDF
    I_b << ixx, ixy, ixz,
            ixy, iyy, iyz,
            ixz, iyz, izz;
    double mass = this->get_parameter("mass").as_double();
    double gravity = 9.81;
    double torque_limit = this->get_parameter("torque_limit").as_double();
    double mu = 0.5; // Static coefficient of friction between foot and ground
    // QuadrupedParams quadruped_params = QuadrupedParams(I_b, mass, gravity, torque_limit, mu);
    quadruped_params = std::make_unique<QuadrupedParams>(I_b, mass, gravity, torque_limit, mu);

    // Initialize the ConvexMPC object
    convex_mpc = std::make_unique<ConvexMPC>(*mpc_params, quadruped_params, this->get_logger());

    // TODO: implement parameter loading for gait planner
    gait_planner = std::make_unique<GaitPlanner>(
        GaitType::TROT, 
        0.5, // Duty factor
        1.0, // Gait duration in seconds
        0.08, // Swing height in meters
        2.0, // Footstep planning horizon
        0.0  // Start time
    );
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
        // RCLCPP_INFO(this->get_logger(), "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
        //             i, motor[i].q, motor[i].dq, motor[i].ddq, motor[i].tau_est);
        joint_angles[i] = motor[i].q; // Joint angles of Go2
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

    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             0, foot_pos[0], foot_pos[1], foot_pos[2], foot_vel[0], foot_vel[1], foot_vel[2]);
    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             1, foot_pos[3], foot_pos[4], foot_pos[5], foot_vel[3], foot_vel[4], foot_vel[5]);
    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             2, foot_pos[6], foot_pos[7], foot_pos[8], foot_vel[6], foot_vel[7], foot_vel[8]);
    // RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
    //             3, foot_pos[9], foot_pos[10], foot_pos[11], foot_vel[9], foot_vel[10], foot_vel[11]);


    // Update MPC state
    p[0] = data->position[0]; // Position in x
    p[1] = data->position[1]; // Position in y
    p[2] = data->position[2]; // Position in z
    p_dot[0] = data->velocity[0]; // Velocity in x
    p_dot[1] = data->velocity[1]; // Velocity in y
    p_dot[2] = data->velocity[2]; // Velocity in z

    
}

void QuadConvexMPCNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    this->joy_msg = msg; // Store the joystick message
}

VectorXd QuadConvexMPCNode::reference_traj_from_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    VectorXd X_ref = VectorXd::Zero(mpc_params->N_MPC * mpc_params->N_STATES);

    double max_translation_vel = 0.5;
    double max_yaw_vel = 0.5;

    double height = 0.312; // Desired height above ground

    double pitch = 0.0; 
    double roll = 0.0; 
    double current_yaw = theta[2]; // Current yaw angle from IMU

    // Extract axes from the joystick message
    std::vector<float> axes = msg->axes; // Use '->' to access members of the object pointed to by msg
    Eigen::VectorXd axes_double = Eigen::Map<Eigen::VectorXf>(axes.data(), axes.size()).cast<double>();

    // Calculate the desired translation and rotation velocities
    cout << "Axes: " << axes_double.transpose() << endl;

    std::unordered_map<std::string, int> controller_axis_indices;
    controller_axis_indices["x"] = 1;
    controller_axis_indices["y"] = 0;
    controller_axis_indices["yaw"] = 3;

    double vx = max_translation_vel * axes_double[controller_axis_indices["x"]]; // Forward/backward movement
    double vy = max_translation_vel * axes_double[controller_axis_indices["y"]]; // Left/right movement
    double wz = max_yaw_vel * axes_double[controller_axis_indices["yaw"]]; // Yaw rotation

    for (int k=0; k < mpc_params->N_MPC; k++)
    {
        X_ref.segment<13>(k * mpc_params->N_STATES) = (Eigen::Matrix<double, 13, 1>() <<
            roll, 
            pitch,
            current_yaw + wz * k * mpc_params->dt, // Fixed yaw rate interpolation
            // p[0] + vx * k * mpc_params->dt, // x position (forward/backward movement)
            // p[1] + vy * k * mpc_params->dt, // y position (left/right movement)
            -0.025570 + vx * k * mpc_params->dt, // x position (forward/backward movement)
            vy * k * mpc_params->dt, // y position (left/right movement)
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

    return X_ref;
}

void QuadConvexMPCNode::update_mpc_state()
{
    // x = [theta, p, omega, p_dot, g]
    float g = 9.81; // Gravity state

    Eigen::Vector<double, 13> x0;

    x0 <<   theta[0], theta[1], theta[2], // Orientation in roll, pitch, yaw
            p[0], p[1], p[2], // Position in x, y, z
            omega[0], omega[1], omega[2], // Angular velocity in roll, pitch, yaw
            p_dot[0], p_dot[1], p_dot[2], // Velocity in x, y, z
            g; // Gravity state

    RCLCPP_INFO(this->get_logger(), "Setting x0 to [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        x0(0), x0(1), x0(2), x0(3), x0(4), x0(5), x0(6), x0(7), x0(8), x0(9), x0(10), x0(11), x0(12));

    this->convex_mpc->update_x0(x0); // Update rigid body pose state

    // Convert joint_angles array to Eigen::VectorXd
    Eigen::Map<Eigen::VectorXf> joint_angles_eigen(joint_angles, 12);
    Vector<double, 12> joint_angles_double = joint_angles_eigen.cast<double>(); // Convert to double for MPC compatibility
    this->convex_mpc->update_joint_angles(joint_angles_double); // Update joint angles (used for foot Jacobian computation)

    // Reshape foot_pos (12x1 vector) into a 3x4 matrix for MPC
    Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::ColMajor>> mat_map(foot_pos, 3, 4);
    foot_positions = mat_map.cast<double>();
    this->convex_mpc->update_foot_positions(foot_positions); // Update foot positions in the body frame


    // Update the reference trajectory
    // Reference state to repeat
    Eigen::Vector<double, 13> X_ref_single;
    X_ref_single << 0.000000, 0.000000, 0.000000, -0.025570, 0.000000, 0.312320, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 9.810000;

    // Repeat X_ref_single N_MPC times to form X_ref
    int N_MPC = mpc_params->N_MPC;
    
    VectorXd X_ref = VectorXd::Zero(mpc_params->N_MPC * mpc_params->N_STATES);
    for (int i = 0; i < N_MPC; ++i) {
        X_ref.segment<13>(i * 13) = X_ref_single;
    }

    Eigen::VectorXd X_ref_joy = reference_traj_from_joy(joy_msg); // Get the reference trajectory from joystick input
    RCLCPP_INFO(this->get_logger(), "Reference trajectory from joystick: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        X_ref_joy[0], X_ref_joy[1], X_ref_joy[2], X_ref_joy[3], X_ref_joy[4], X_ref_joy[5],
        X_ref_joy[6], X_ref_joy[7], X_ref_joy[8], X_ref_joy[9], X_ref_joy[10], X_ref_joy[11], X_ref_joy[12]);

    convex_mpc->update_reference_trajectory(X_ref_joy); // Update the reference trajectory in MPC
}

void QuadConvexMPCNode::publish_cmd()
{
    // Check whether each leg is scheduled to be in swing or stance
    unordered_map<std::string, int> contact_states;

    gait_planner->update_time_and_phase(this->now().seconds());
    contact_states = gait_planner->get_contact_state();

    // Solve GRFs from MPC
    convex_mpc->set_contact_constraints(contact_states); // Enable complementarity constraints
    Vector<double, 12> mpc_joint_torques = convex_mpc->solve_joint_torques();

    unordered_map<string, Vector3d> foot_positions_map = matrix_to_foot_positions_map(foot_positions);

    // Solve for swing leg trajectories within the gait planning horizon
    Eigen::VectorXd X_ref_joy = reference_traj_from_joy(joy_msg); // Get the reference trajectory from joystick input
    gait_planner->update_swing_leg_trajectories(X_ref_joy, *mpc_params, foot_positions_map);
    

    // Finite state machine to command torques based on either GRF (MPC) or PD swing leg controller
    Vector<double, 12> joint_torques = Vector<double, 12>::Zero();
    for (const auto& [leg, contact_state] : contact_states)
    {
        if(contact_state == 1)
        {
            int leg_index = quadruped_params->LEG_NAME_TO_INDEX.at(leg);
            int joint_start_idx = leg_index * 3;

            joint_torques[joint_start_idx] = mpc_joint_torques[joint_start_idx];
            joint_torques[joint_start_idx+1] = mpc_joint_torques[joint_start_idx+1];
            joint_torques[joint_start_idx+2] = mpc_joint_torques[joint_start_idx+2];
        }
        else
        {
            // Apply PD control for swing legs
            
        }

    }

    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd[i].tau = joint_torques[i]; // Set the joint torque command
    }

    RCLCPP_INFO(this->get_logger(), "Publishing joint torques: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
        joint_torques[0], joint_torques[1], joint_torques[2], joint_torques[3], joint_torques[4], joint_torques[5],
        joint_torques[6], joint_torques[7], joint_torques[8], joint_torques[9], joint_torques[10], joint_torques[11]);
    joint_torque_pub_->publish(low_cmd); // Publish the joint torque commands

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
        
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    catch(GRBException& e) {
        std::cerr << "Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << "Gurobi error message: " << e.getMessage() << std::endl;
        return 1;
    }
    catch(std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    catch(...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown exception occurred");
        return 1;
    }

    return 0;
}