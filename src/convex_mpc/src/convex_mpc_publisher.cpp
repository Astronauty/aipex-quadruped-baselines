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

#include "convex_mpc/mpc_params.hpp"
#include "convex_mpc/quad_params.hpp"
#include "convex_mpc/convex_mpc.hpp"

// #include "common/motor_crc_hg.h"


using namespace std::chrono_literals;
using namespace Eigen;

// using namespace unitree::common;
// using namespace unitree::robot;

// Enum for measurement mode
enum class StateMeasurementMode {
    LOWLEVEL_EKF,
    SPORTMODE,
    MOCAP
};

// Helper function to convert string to MeasurementMode
StateMeasurementMode measurement_mode_from_string(const std::string& mode_str) {
    if (mode_str == "lowlevel_ekf") return StateMeasurementMode::LOWLEVEL_EKF;
    if (mode_str == "sportmode") return StateMeasurementMode::SPORTMODE;
    if (mode_str == "mocap") return StateMeasurementMode::MOCAP;
    throw std::invalid_argument("Invalid measurement mode: " + mode_str);
}



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
        QuadConvexMPCNode()
        : Node("quad_convex_mpc_publisher"), count_(0)
        {
            // Set measurement mode
            // TODO: ROS param implementation
            // std::string measurement_mode_str = this->declare_parameter<std::string>("measurement_mode", "lowlevel_ekf");

            state_measurement_mode_ = StateMeasurementMode::SPORTMODE; 

            // Publisher for joint torque commands
            joint_torque_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

            // // Create a lowstate subscriber to update mpc states
            low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
                "lowstate", 10, std::bind(&QuadConvexMPCNode::low_state_callback, this, std::placeholders::_1)
            );

            // Sport mode state subscriber
            // TODO: switch between subscriber callbacks based on measurement_mode
            sport_mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
                "sportmodestate", 10, std::bind(&QuadConvexMPCNode::sport_mode_callback, this, std::placeholders::_1)
            );

            // TODO: Set correct rate for the timer
            timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&QuadConvexMPCNode::update_mpc_state, this));

            // Initialize unitree lowcmd#
            int N_STATES = 13;
            int N_CONTROLS = 12;
            int N_MPC = 5; // Number of steps for the horizon length in MPC
            double dt = 0.01; // Time step for the MPC

            // Define MPC Params
            MatrixXd Q = MatrixXd::Identity(N_STATES, N_STATES);
            MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS);
            VectorXd u_lower = VectorXd::Constant(N_CONTROLS, -1.0);
            VectorXd u_upper = VectorXd::Constant(N_CONTROLS, 1.0);
            MPCParams mpc_params = MPCParams(N_MPC, N_CONTROLS, N_STATES,
                dt, Q, R, u_lower, u_upper);

            // Define Quadruped Params (from https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
            double ixx=0.02448;
            double ixy=0.00012166;
            double ixz=0.0014849;
            double iyy=0.098077;
            double iyz=-3.12E-05;
            double izz=0.107;


            // Matrix3d inertiaTensor = Eigen::Matrix3d::Identity();
            Matrix3d inertiaTensor;
            inertiaTensor << ixx, ixy, ixz,
                             ixy, iyy, iyz,
                             ixz, iyz, izz;
            double mass = 6.921;
            double gravity = 9.81;
            QuadrupedParams quadruped_params = QuadrupedParams(inertiaTensor, mass, gravity);

            // Initialize the ConvexMPC object
             convex_mpc = std::make_unique<ConvexMPC>(mpc_params, quadruped_params, this->get_logger());
        }
        
    private:
        std::unique_ptr<ConvexMPC> convex_mpc;
        
        StateMeasurementMode state_measurement_mode_;
        // MPC State Vars
        float theta[3]; // Orientation in roll, pitch, yaw
        float p[3]; // Position in x, y, z
        float omega[3]; // Angular velocity in roll, pitch, yaw
        float p_dot[3]; // Linear velocity in x, y, z

        float foot_positions[4][3]; // Foot positions in x, y, z

        // Publisher for joint torque commands
        rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr joint_torque_pub_;

        // Sport mode vars
        rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_sub_;
        float foot_pos[12];
        float foot_vel[12];
        
        // Low state vars
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

        unitree_go::msg::IMUState imu;         // Unitree go2 IMU message
        unitree_go::msg::MotorState motor[12]; // Unitree go2 motor state message
        int16_t foot_force[4];                 // External contact force value (int)
        int16_t foot_force_est[4];             // Estimated  external contact force value (int)
        float battery_voltage;                 // Battery voltage
        float battery_current;                 // Battery current


        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;          

        unitree_go::msg::LowCmd low_cmd;

                
        void init_cmd()
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

        void low_state_callback(unitree_go::msg::LowState::SharedPtr data)
        {
            // Info IMU states
            // RPY euler angle(ZYX order respected to body frame)
            // Quaternion
            // Gyroscope (raw data)
            // Accelerometer (raw data)
            imu = data->imu_state;

            RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]);
                
            theta[0] = imu.rpy[0]; // Roll
            theta[1] = imu.rpy[1]; // Pitch
            theta[2] = imu.rpy[2]; // Yaw

            RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f", imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);

            omega[0] = imu.gyroscope[0]; // Angular velocity in roll
            omega[1] = imu.gyroscope[1]; // Angular velocity in pitch
            omega[2] = imu.gyroscope[2]; // Angular velocity in yaw

        }

    

        void sport_mode_callback(unitree_go::msg::SportModeState::SharedPtr data)
        {
            // Info motion states
            // Gait type and foot raise height
            // Robot position (Odometry frame)
            // Robot velocity (Odometry frame)

            RCLCPP_INFO(this->get_logger(), "Gait state -- gait type: %d; raise height: %f", data->gait_type, data->foot_raise_height);
            RCLCPP_INFO(this->get_logger(), "Position -- x: %f; y: %f; z: %f; body height: %f",
                        data->position[0], data->position[1], data->position[2], data->body_height);
            RCLCPP_INFO(this->get_logger(), "Velocity -- vx: %f; vy: %f; vz: %f; yaw: %f",
                        data->velocity[0], data->velocity[1], data->velocity[2], data->yaw_speed);


            // Info foot states (foot position and velocity in body frame)
            for (int i = 0; i < 12; i++)
            {
                foot_pos[i] = data->foot_position_body[i];
                foot_vel[i] = data->foot_speed_body[i];
            }

            RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                        0, foot_pos[0], foot_pos[1], foot_pos[2], foot_vel[0], foot_vel[1], foot_vel[2]);
            RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                        1, foot_pos[3], foot_pos[4], foot_pos[5], foot_vel[3], foot_vel[4], foot_vel[5]);
            RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                        2, foot_pos[6], foot_pos[7], foot_pos[8], foot_vel[6], foot_vel[7], foot_vel[8]);
            RCLCPP_INFO(this->get_logger(), "Foot position and velcity relative to body -- num: %d; x: %f; y: %f; z: %f, vx: %f; vy: %f; vz: %f",
                        3, foot_pos[9], foot_pos[10], foot_pos[11], foot_vel[9], foot_vel[10], foot_vel[11]);


            // Update MPC state
            p[0] = data->position[0]; // Position in x
            p[1] = data->position[1]; // Position in y
            p[2] = data->position[2]; // Position in z
            p_dot[0] = data->velocity[0]; // Velocity in x
            p_dot[1] = data->velocity[1]; // Velocity in y
            p_dot[2] = data->velocity[2]; // Velocity in z

            
        }

        void update_mpc_state()
        {
            // x = [theta, p, omega, p_dot, g]
            float g = 9.81; // Gravity state
            
            Eigen::Matrix<double, 13, 1> x0;

            x0 <<   theta[0], theta[1], theta[2], // Orientation in roll, pitch, yaw
                    p[0], p[1], p[2], // Position in x, y, z
                    omega[0], omega[1], omega[2], // Angular velocity in roll, pitch, yaw
                    p_dot[0], p_dot[1], p_dot[2], // Velocity in x, y, z
                    g; // Gravity state

            RCLCPP_INFO(this->get_logger(), "Setting x0 to [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
                x0(0), x0(1), x0(2), x0(3), x0(4), x0(5), x0(6), x0(7), x0(8), x0(9), x0(10), x0(11), x0(12));
            this->convex_mpc->update_x0(x0);
        }



        // void update_mpc_states(const unitree_go::msg::LowState::SharedPtr data)
        // {
        //     switch (measurement_mode_) {
        //         case MeasurementMode::LOWLEVEL_EKF:
        //             // Extract from lowlevel EKF (default)
        //             imu = data->imu_state;
        //             RCLCPP_INFO(this->get_logger(), "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0], imu.rpy[1], imu.rpy[2]);

        //             this->convex_mpc.update_x0(
        //                 Vector<double, 13> {
        //                     motor[0].q, motor[1].q, motor[2].q, motor[3].q,
        //                     motor[4].q, motor[5].q, motor[6].q, motor[7].q,
        //                     motor[8].q, motor[9].q, motor[10].q, motor[11].q,
        //                     imu.rpy[2] // Use yaw as the last state
        //                 }
        //             );
        //             RCLCPP_WARN(this->get_logger(), "SPORTMODE measurement not yet implemented");

        //             break;
        //         case MeasurementMode::SPORTMODE:
        //             // TODO: Extract state from sportmode data
        //             RCLCPP_WARN(this->get_logger(), "SPORTMODE measurement not yet implemented");
        //             break;
        //         case MeasurementMode::MOCAP:
        //             // TODO: Extract state from mocap data
        //             RCLCPP_WARN(this->get_logger(), "MOCAP measurement not yet implemented");
        //             break;
        //     }
        // }
};

int main(int argc, char * argv[])
{
    try{
        rclcpp::init(argc, argv);
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