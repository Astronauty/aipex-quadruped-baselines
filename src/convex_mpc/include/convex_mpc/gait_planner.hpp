#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>
#include <deque>

using namespace std;

class GaitPlanner
{
    public:
        enum class GaitType
        {
            TROT,
            WALK,
            GALLOP,
            PRONK,
            BOUND
        };

        GaitPlanner(GaitType gait_type, double duty_factor = 0.5, double gait_duration_s = 1.0, double swing_height_m = 0.08, double footstep_planning_horizon = 2.0);

        std::unordered_map<std::string, int> get_contact_state(double phase);
        void set_gait_type(GaitType gait_type);

        double get_phase(double current_time);


    private:
        double start_time_s_; // Start time of the gait planning in seconds
        double gait_duration_s_; // Duration of the gait in seconds
        double time_to_phase(double current_time_s);
        double phase_to_time(double phase);
        double swing_height_m_; // Height of the swing trajectory in meters
        GaitType gait_type_;
        double duty_factor_; // Duty factor for the gait, default is 0.5
        std::unordered_map<std::string, double> phase_offsets_; // Phase offsets for each leg in the gait
        // double gait_planning_phase_horizon_; // The horizon length (in phase) for which to compute future footsteps

        std::unordered_map<string, deque<SwingTrajectory>>  swing_leg_trajectories_;
 
        std::unordered_map<std::string, Eigen::MatrixXd> desired_footstep_locations_; // Desired footstep locations for each foot in world frame
        const std::unordered_map<std::string, Eigen::Vector3d> HIP_POSITIONS = {
            {"FL", Eigen::Vector3d(0.1934, 0.0465, 0.0)},   // Front Left
            {"FR", Eigen::Vector3d(0.1934, -0.0465, 0.0)},  // Front Right
            {"RL", Eigen::Vector3d(-0.1934, 0.0465, 0.0)},  // Rear Left
            {"RR", Eigen::Vector3d(-0.1934, -0.0465, 0.0)}  // Rear Right
        }; // TODO: parse from URDF
        
        void set_phase_offsets(); // Defines gaits based on their phase offsets
        Eigen::Vector3d compute_desired_footstep_position(const Eigen::Vector3f& v_CoM, const string& foot_index);

        void update_swing_leg_trajectories(double current_time_s, unordered_map<string, Vector3d> current_footstep_positions);
        // double time_to_next_stance(double current_time_s, const std::string& leg);
        
        double footstep_planning_horizon_s_; // The horizon length (in seconds) for which to compute future footsteps
        double footstep_planning_horizon_phase_; // The horizon length (in phases) for which to compute future footsteps
        std::unordered_map<std::string, std::vector<std::pair<double, double>>> get_future_stance_times(double current_time_s, double footstep_planning_horizon_phases);

};


class SwingLegTrajectory
{
    public:
        SwingLegTrajectory


}