#include "convex_mpc/gait_planner.hpp"


using namespace std;
using namespace Eigen;

GaitPlanner::GaitPlanner(GaitType gait_type, double duty_factor, double gait_duration_s, double swing_height_m, double footstep_planning_horizon_s, double start_time_s)
    : gait_type_(gait_type),
      duty_factor_(duty_factor),
      gait_duration_s_(gait_duration_s),
      swing_height_m_(swing_height_m),
      footstep_planning_horizon_s_(footstep_planning_horizon_s),
      start_time_s_(start_time_s)
{
    this->set_phase_offsets();                 // Define gait types based on phase offsets
    this->update_time_and_phase(start_time_s); // Initialize the current time and phase

    // Initialize the swing leg trajectory queue for each leg
    swing_leg_trajectories_["FL"] = {};
    swing_leg_trajectories_["FR"] = {};
    swing_leg_trajectories_["RL"] = {};
    swing_leg_trajectories_["RR"] = {};
}

void GaitPlanner::set_phase_offsets()
{
    switch (gait_type_)
    {
    case GaitType::TROT:
        phase_offsets_ =
            {
                {"FL", 0.0},
                {"FR", 0.5},
                {"RL", 0.5},
                {"RR", 0.0}};
        break;
    case GaitType::BOUND:
        phase_offsets_ =
            {
                {"FL", 0.5},
                {"FR", 0.5},
                {"RL", 0.0},
                {"RR", 0.0}};
        break;

    default:
        throw std::invalid_argument("Selected gait type is not implemented.");
    }
}

void GaitPlanner::update_time_and_phase(double current_time_s)
{
    this->current_time_s_ = current_time_s;
    this->gait_phase_ = time_to_phase(current_time_s);
}

unordered_map<std::string, int> GaitPlanner::get_contact_state()
{
    unordered_map<std::string, int> contact_state;

    for (const auto &[leg, offset] : phase_offsets_)
    {
        double adjusted_phase = fmod(gait_phase_ + offset, 1.0);
        contact_state[leg] = (adjusted_phase < duty_factor_) ? 1 : 0;
    }

    return contact_state;
}

void GaitPlanner::set_gait_type(GaitType gait_type)
{
    gait_type_ = gait_type;
    set_phase_offsets(); // Update phase offsets based on the new gait type
}

/**
 * @brief Compute the desired footstep position based on the the hip position (projection onto ground plane), CoM velocity, and foot index.
 * @param x The state vector at the desired footstep time.
 * @param foot_index The index of the foot (FL, FR, RL, RR) for which to compute the desired position.
 * @return The desired footstep position in the world frame, projected onto the ground plane.
 */
Vector3d GaitPlanner::compute_desired_footstep_position(const Eigen::VectorXd &x, const string &foot_index)
{
    // Compute the desired footstep position based on the reference position, CoM velocity, and foot index
    Eigen::Vector3d hip_position_B = HIP_POSITIONS.at(foot_index); // Hip position in body frame

    double roll = x[0];
    double pitch = x[1];
    double yaw = x[2];
    Vector3d p_COM = x.segment<3>(3); // CoM position in world frame
    Vector3d v_COM = x.segment<3>(6); // CoM velocity in world frame

    Matrix3d R_WB = (AngleAxisd(yaw, Vector3d::UnitZ()) *
                     AngleAxisd(pitch, Vector3d::UnitY()) *
                     AngleAxisd(roll, Vector3d::UnitX()))
                        .toRotationMatrix();

    Vector3d hip_position_W = p_COM + R_WB * hip_position_B;        // Hip position in world frame
    Vector3d p_des_W = hip_position_W + v_COM * duty_factor_ / 2.0; // Raibert heuristic to define desired footstep position in world frame

    return p_des_W;
}

double GaitPlanner::time_to_phase(double current_time_s)
{
    double phase = fmod((current_time_s - start_time_s_), gait_duration_s_) / gait_duration_s_;
    return phase;
}

double GaitPlanner::phase_to_time(double phase)
{
    // Returns the time corresponding to a given phase in the gait cycle
    if (phase < 0.0 || phase > 1.0)
        throw std::out_of_range("Phase must be in the range [0, 1]");

    double gait_cycle_start_time_s_ = current_time_s_ - fmod((current_time_s_ - start_time_s_), gait_duration_s_);

    return phase * gait_duration_s_ + gait_cycle_start_time_s_;
}

std::unordered_map<std::string, std::deque<SwingLegTrajectory>> GaitPlanner::update_swing_leg_trajectories(const Eigen::VectorXd &X_ref, const MPCParams &mpc_params, const unordered_map<string, Vector3d> &current_foot_positions)
{
    cout << "\nUpdating swing leg trajectories..." << endl;
    cout << "Current time: " << current_time_s_ << ", Current phase: " << gait_phase_ << endl;
    cout << "Footstep planning horizon: " << footstep_planning_horizon_s_ << " seconds" << endl;

    clear_expired_swing_leg_trajectories();

    auto future_swing_times = get_future_swing_times(current_time_s_, footstep_planning_horizon_s_);
    unordered_map<std::string, int> contact_state = get_contact_state(gait_phase_);

    Vector3d p_des_start;
    Vector3d p_des_end;
    

    for (const auto &[leg, swing_times] : future_swing_times)            
    {
        for (const auto &[start_time, end_time] : swing_times)
        {
            cout << "\nLeg: " << leg << ", Swing start time: " << start_time << ", Swing end time: " << end_time << endl;

            VectorXd x = get_state_at_time_from_ref_traj(X_ref, mpc_params, current_time_s_, end_time); // End of swing corresponds to start of stance
            Vector3d p_des_end = compute_desired_footstep_position(x, leg); // Compute desired footstep position based on reference traj and Raibert heuristic

            if (swing_leg_trajectories_[leg].size() == 0)
            {
                // If no previous trajectory, use current foot position as the start for the first swing
                p_des_start = current_foot_positions.at(leg); 
                // start_time = current_time_s_; // The start time from get_future_swing_times returns the start of the swing phase by default, but use the current time if no previous trajectory exists
    
                SwingLegTrajectory trajectory = SwingLegTrajectory(start_time, end_time, p_des_start, p_des_end, swing_height_m_);

                swing_leg_trajectories_[leg].emplace_back(trajectory);

            }
            else 
            if (swing_leg_trajectories_[leg].back().get_start_time() < current_time_s_ && swing_leg_trajectories_[leg].back().get_end_time() > current_time_s_) 
            {
                // If a previously planned swing trajectory is active, only update the end position based on the raibert heuristic
                swing_leg_trajectories_[leg].back().set_end_position(p_des_end); // Update the end position of the last trajectory


            }
            else
            {
                p_des_start = swing_leg_trajectories_[leg].back().get_end_position(); // Use the last end position as the start position (stays the same in stance)
                SwingLegTrajectory trajectory = SwingLegTrajectory(start_time, end_time, p_des_start, p_des_end, swing_height_m_);
                swing_leg_trajectories_[leg].emplace_back(trajectory);

            }


        }
    }

    return swing_leg_trajectories_;
}

void GaitPlanner::clear_expired_swing_leg_trajectories()
{
    for (auto &[leg, trajectories] : swing_leg_trajectories_)
    {
        // Remove trajectories that have ended before the current time
        trajectories.erase(
            std::remove_if(trajectories.begin(), trajectories.end(),
                [this](const SwingLegTrajectory& traj) {
                    return traj.get_end_time() < current_time_s_;
                }),
            trajectories.end()
        );
    }
}

// Get the time in seconds corresponding to the next time the given leg enters stance
double GaitPlanner::time_to_next_stance(double current_time_s, const std::string &leg)
{
    double phase = time_to_phase(current_time_s);
    double leg_phase = fmod(phase + phase_offsets_[leg], 1.0);

    return (1.0 - leg_phase) * gait_duration_s_;
}

// double GaitPlanner::time_to_next_swing(double current_time_s, const std::string& leg)
// {
//     double phase = time_to_phase(current_time_s);
//     double leg_phase = fmod(phase + phase_offsets_[leg], 1.0);

//     if (leg_phase < duty_factor_)
//         return (duty_factor_ - leg_phase) * gait_duration_s_;
//     else
//         return 0.0;
// }

std::unordered_map<std::string, std::vector<std::pair<double, double>>> GaitPlanner::get_future_swing_times(double current_time_s, double footstep_planning_horizon_s)
{
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> future_swing_times;

    for (const auto &leg : {"FL", "FR", "RL", "RR"})
    {
        future_swing_times[leg] = std::vector<std::pair<double, double>>();
    }

    double end_time = current_time_s + footstep_planning_horizon_s; // End time to find future swing entry times

    for (const auto &[leg, leg_phase_offset] : phase_offsets_)
    {
        vector<pair<double, double>> &leg_swing_times = future_swing_times[leg];

        double phase = time_to_phase(current_time_s);
        double leg_phase = fmod(phase + leg_phase_offset, 1.0); // Adjust phase for the leg

        if (leg_phase < 0.0)
            leg_phase += 1.0;

        double time_cursor = current_time_s; // Get the time to the next stance phase for the leg

        // If currently in swing, add remaining portion of swing
        if (leg_phase >= duty_factor_)
        {
            double current_swing_start = current_time_s - (leg_phase - duty_factor_) * gait_duration_s_; // Start of current swing phase
            double current_swing_end = current_time_s + (1.0 - leg_phase) * gait_duration_s_;
            leg_swing_times.emplace_back(current_swing_start, current_swing_end);
            // double clipped_swing_end = min(current_swing_end, end_time); // If the planning horizon ends before swing phase ends, clip the end time
            // leg_swing_times.emplace_back(current_time_s, clipped_swing_end);

            // Move cursor to start of next swing
            time_cursor = current_time_s + (1.0 - leg_phase) * gait_duration_s_ + duty_factor_ * gait_duration_s_; // Move cursor to the end of the current swing phase
        }
        else
        {
            // Currently in stance, find start of next swing phase
            time_cursor = current_time_s + (duty_factor_ - leg_phase) * gait_duration_s_; // Start of next swing phase
        }

        while (time_cursor < end_time)
        {
            double swing_start = time_cursor;
            double swing_end = swing_start + (1.0 - duty_factor_) * gait_duration_s_;

            if (time_cursor < end_time)
            {
                double clipped_swing_end = min(swing_end, end_time); // If the planning horizon ends before swing phase ends, clip the end time
                leg_swing_times.emplace_back(swing_start, clipped_swing_end);
            }

            time_cursor += gait_duration_s_;
        }
    }
    return future_swing_times;
}

// int main(int, char **)
// {
//     // GaitPlanner gait_planner(GaitPlanner::GaitType::TROT, 0.5, 1.0);
//     double duty_factor = 0.5;
//     double gait_duration_s = 1.0;
//     double swing_height_m = 0.08;
//     double footstep_planning_horizon = 0.8;
//     double start_time_s = 0.0;

//     GaitPlanner gait_planner = GaitPlanner(GaitType::TROT, duty_factor, gait_duration_s, swing_height_m, footstep_planning_horizon, start_time_s);
//     gait_planner.update_time_and_phase(start_time_s);

//     vector<double> phases = {0.0};

//     for (double phase : phases)
//     {
//         cout << "Contact states at phase: " << phase << endl;
//         unordered_map<std::string, int> contact_state = gait_planner.get_contact_state(phase);
//         for (const auto &[leg, state] : contact_state)
//             cout << "Leg: " << leg << ", Contact State: " << state << endl;
//     }

//     // Dummy reference trajectory for testing
//     int MPC_HORIZON = 10;
//     int N_STATES = 12;

//     VectorXd X_ref = Eigen::VectorXd::Zero(N_STATES * MPC_HORIZON);

//     for (int i = 0; i < MPC_HORIZON; ++i)
//     {
//         VectorXd state = VectorXd::Zero(12); // Initialize with zeros
//         state(3) = 1.0 * i;                  // Add 1.0 to the x position at each step (x position is at index 3)
//         X_ref.segment<12>(i * 12) = state;
//     }

//     MPCParams mpc_params = MPCParams(
//         MPC_HORIZON,                            // N_MPC
//         12,                                     // N_CONTROLS
//         N_STATES,                               // N_STATES
//         0.1,                                    // dt
//         MatrixXd::Identity(N_STATES, N_STATES), // Q
//         MatrixXd::Identity(12, 12),             // R
//         VectorXd::Constant(12, -1.0),           // u_lower
//         VectorXd::Constant(12, 1.0)             // u_upper
//     );

//     unordered_map<string, Vector3d> current_footstep_positions = {
//         {"FL", Vector3d(0.3, 0.2, 0.0)},
//         {"FR", Vector3d(0.3, -0.2, 0.0)},
//         {"RL", Vector3d(-0.3, 0.2, 0.0)},
//         {"RR", Vector3d(-0.3, -0.2, 0.0)}};

//     std::unordered_map<std::string, std::deque<SwingLegTrajectory>> swing_leg_trajectories = gait_planner.update_swing_leg_trajectories(X_ref, mpc_params, current_footstep_positions);

//     for (const auto &[leg, trajectories] : swing_leg_trajectories)
//     {
//         cout << "\nSwing trajectories for leg: " << leg << endl;
//         for (const auto &traj : trajectories)
//         {
//             cout << "Start time: " << traj.get_start_time() << ", End time: " << traj.get_end_time() << endl;
//             cout << "Start position: " << traj.get_start_position().transpose() << ", End position: " << traj.get_end_position().transpose() << endl;
//             // cout << "Apex position: " << traj.apex_position.transpose() << endl;
//         }
//     }


//     return 0;
// }
