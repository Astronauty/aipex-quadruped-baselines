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
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "convex_mpc/convex_mpc.hpp"
#include "convex_mpc/gait_planner.hpp"


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
        std::unique_ptr<MPCParams> mpc_params; // Pointer to MPC parameters

        double start_time_s;
        double elapsed_time_s;

        // MPC State Vars
        float theta[3]; // Orientation in roll, pitch, yaw
        float p[3]; // Position in x, y, z
        float omega[3]; // Angular velocity in roll, pitch, yaw
        float p_dot[3]; // Linear velocity in x, y, z
        // MPCParams mpc_params; // MPC parameters

        Matrix<double, 3, 4> foot_positions; // Foot positions in x, y, z
        float joint_angles[12]; // Joint angles of Go2

        // Publisher for joint torque commands
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr joint_torque_pub_;
        rclcpp::TimerBase::SharedPtr publish_joint_torque_timer_; // Rate to update x0 in MPC based on lowstate

        // Unitree sportmode vars
        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_sub_;
        float foot_pos[12];
        float foot_vel[12];
        
        // Unitree lowstate vars
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

        unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
        unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message
        int16_t foot_force[4];                 // External contact force value (int)
        int16_t foot_force_est[4];             // Estimated  external contact force value (int)
        float battery_voltage;                 // Battery voltage
        float battery_current;                 // Battery current

        rclcpp::TimerBase::SharedPtr update_mpc_state_timer_; // Rate to update x0 in MPC based on lowstate

        // Unitree lowcmd
        unitree_go::msg::LowCmd low_cmd;
        rclcpp::TimerBase::SharedPtr cmd_timer_; // Lowcmd publish frequency
        size_t count_;
        StateMeasurementMode state_measurement_mode_;

        // Joystick parsing
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        sensor_msgs::msg::Joy::SharedPtr joy_msg; // Joystick message

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg); 
        VectorXd reference_traj_from_joy(const sensor_msgs::msg::Joy::SharedPtr msg);

        void init_cmd(); // Set motors to torque mode and initialize the low_cmd message with default values
        void low_state_callback(const unitree_go::msg::LowState::SharedPtr msg);
        void sport_mode_callback(const unitree_go::msg::SportModeState::SharedPtr data);
        void update_mpc_state();
        void publish_cmd();

        // Gait parameters
        // float gait_phase_;
        float gait_duration_s_;

        std::unique_ptr<GaitPlanner> gait_planner;
        void update_gait_phase(float dt);
};