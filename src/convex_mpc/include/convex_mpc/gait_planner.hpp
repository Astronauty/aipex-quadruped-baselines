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

        std::unordered_map<std::string, int> get_contact_state();
        void set_gait_type(GaitType gait_type);

        /** Update the current time and phase of the gait planner.
         * @param current_time_s The current time in seconds.
         */
        void update_time_and_phase(double current_time_s);
        
        double get_phase(double current_time);


    private:
        double start_time_s_; // Start time of the gait planning in seconds
        double current_time_s_; // Current time in seconds
        double gait_phase_; // Current phase of the gait, between 0 and 1

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
        
        /**
         * @brief Set the phase offsets for each leg based on the selected gait type.
         * 
         * @return A map with leg names as keys and contact state (0 or 1) as values.
         */
        void set_phase_offsets(); // Defines gaits based on their phase offsets


        Eigen::Vector3d compute_desired_footstep_position(const Eigen::Vector3f& v_CoM, const string& foot_index);

        /**
         * @brief Update the swing leg trajectories based on the current time and footstep positions.
         * @param current_time_s The current time in seconds.
         * @param current_footstep_positions A map of the current footstep positions for each leg, where the key is the leg identifier ("FL", "FR", "RL", "RR") and the value is the 3D position of the foot in body coordinates.
         */
        void update_swing_leg_trajectories(double current_time_s, unordered_map<string, Vector3d> current_footstep_positions);

        /**
         * @brief Computes the time in seconds until the next stance phase for a given leg.
         * @param current_time_s The current time in seconds.
         * @param leg The leg for which to compute the time to the next stance phase, specified as "FL", "FR", "RL", or "RR".
         * @return The time until the next stance phase in seconds.
         */
        double time_to_next_stance(double current_time_s, const std::string& leg);
        
        double footstep_planning_horizon_s_; // The horizon length (in seconds) for which to compute future footsteps
        double footstep_planning_horizon_phase_; // The horizon length (in phases) for which to compute future footsteps

        /**
         * @brief Get the future stance times for each leg within the planning horizon.
         * @param current_time_s The current time in seconds.
         * @param footstep_planning_horizon_s The planning horizon in seconds.
         * @return A map with leg names ("FL", "FR", "RL", "RR") as keys and a vector of pairs (start_time, end_time) as values.
         */
        std::unordered_map<std::string, std::vector<std::pair<double, double>>> get_future_stance_times(double current_time_s, double footstep_planning_horizon_s);

};


