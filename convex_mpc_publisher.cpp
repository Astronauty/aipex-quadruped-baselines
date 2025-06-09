#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "common/motor_crc_hg.h"

using namespace std::chrono_literals;

class QuadConvexMPCPublisher : public rclcpp::Node
{
    public:
        QuadConvexMPCPublisher()
        : Node("quad_convex_mpc_publisher"), count_(0)
        {
            joint_torque_pub_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", 10);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&low_level_cmd_sender::timer_callback, this));

            // Initialize unitree lowcmd
            init_cmd();

            // Initialize MPC solver
            // Define the parameters for the MPC and the quadruped
            int N_MPC = 5;
            double dt = 0.01;
            int N_STATES = 13;
            int N_CONTROLS = 12;

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
            ConvexMPC convex_mpc = ConvexMPC(mpc_params, quadruped_params);
        }
        
    private:
        void init_cmd()
        {
            for (int i = 0; i < 20; i++)
            {
                low_cmd.motor_cmd[i].mode = 0x01; // Set torque mode, 0x00 is passive mode
                low_cmd.motor_cmd[i].q = PosStopF;
                low_cmd.motor_cmd[i].kp = 0;
                low_cmd.motor_cmd[i].dq = VelStopF;
                low_cmd.motor_cmd[i].kd = 0;
                low_cmd.motor_cmd[i].tau = 0;
            }
        }
}