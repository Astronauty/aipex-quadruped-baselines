#include "convex_mpc/gait_planner.hpp"
#include "convex_mpc/swing_trajectory.hpp"
using namespace std;
using namespace Eigen;

GaitPlanner::GaitPlanner(GaitType gait_type, double duty_factor, double gait_duration_s, double swing_height_m, double footstep_planning_horizon, double start_time_s)
    : gait_type_(gait_type), duty_factor_(duty_factor), gait_duration_s_(gait_duration_s), swing_height_m_(swing_height_m), footstep_planning_horizon_s_(footstep_planning_horizon_s), start_time_s_(start_time_s)
{
    this->set_phase_offsets(); // Define gait types based on phase offsets

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
                {"RR", 0.0}
            };
            break;
        case GaitType::BOUND:
            phase_offsets_ =
            {
                {"FL", 0.5},
                {"FR", 0.5},
                {"RL", 0.0},
                {"RR", 0.0}
            };
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

unordered_map<std::string, int> GaitPlanner::get_contact_state(double phase)
{
    unordered_map<std::string, int> contact_state;

    for (const auto& [leg, offset] : phase_offsets_)
    {
        double adjusted_phase = fmod(phase + offset, 1.0);
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
 * @param v_CoM The center of mass velocity at the scheduled footstep placement time.
 * @param foot_index The index of the foot (FL, FR, RL, RR) for which to compute the desired position.
 */
Vector3d GaitPlanner::compute_desired_footstep_position(const Eigen::Vector3f& v_CoM, const string& foot_index)
{
    // Compute the desired footstep position based on the reference position, CoM velocity, and foot index
    Eigen::Vector3d hip_position = HIP_POSITIONS.at(foot_index);
    Eigen::Vector3d desired_position = hip_position + v_CoM*duty_factor_/2.0;

    return desired_position;
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

    gait_cycle_start_time_s_ = current_time_s_ - fmod((current_time_s_ - start_time_s_), gait_duration_s_);

    return phase * gait_duration_s_ + gait_cycle_start_time_s_;
}

void GaitPlanner::update_swing_leg_trajectories(double current_time_s, unordered_map<string, Vector3d> current_footstep_positions)
{
    // for each leg in swing_leg_trajectories_
    double phase = get_phase(current_time_s);
    unordered_map<std::string, int> contact_state = get_contact_state(phase);

    MatrixXd v_CoM = MatrixXd::Zero(3, 1); // TODO: Placeholder for CoM velocity, should be set based on the robot's state 

    for (auto& [leg, trajectory] : swing_leg_trajectories_)
    {
        if (contact_state[leg] == 1) // If the leg is in contact, plan for the next swing phase
        {
            double time_to_next_stance_s = time_to_next_stance(current_time_s, leg);

            Vector3d v_CoM; // Placeholder, should be based on v_COM(time_to_next_stance_s) from the reference traj
            Vector3d p_des = compute_desired_footstep_position(v_CoM, leg);

            double swing_start_time_s = current_time_s + time_to_next_stance_s; // Start of the swing phase
            double swing_end_time_s = swing_start_time_s + (1.0 - duty_factor_) * gait_duration_s_;

            // Create a new swing trajectory for the leg
            SwingTrajectory new_swing_traj(
                swing_start_time_s,
                swing_end_time_s, 
                current_footstep_positions[leg],
                p_des,
                swing_height_m_
            );
            swing_leg_trajectories_[leg].push_back(new_swing_traj);

        }
        else if (contact_state[leg] == 0) // If the leg is in swing
        {
            // Remove completed trajectories
            auto it = swing_leg_trajectories_[leg].begin();
            while (it != swing_leg_trajectories_[leg].end()) {
                if (current_time_s > it->end_time_s) {
                    it = swing_leg_trajectories_[leg].erase(it);
                } else {
                    ++it;
                }
            }
            
        };
    }
}

// Get the time in seconds corresponding to the next time the given leg enters stance
double GaitPlanner::time_to_next_stance(double current_time_s, const std::string& leg)
{
    double phase = get_phase(current_time_s);
    double leg_phase = fmod(phase + phase_offsets_[leg], 1.0);

    return (1.0 - leg_phase)*gait_duration_s_;
}

std::unordered_map<std::string, std::vector<std::pair<double, double>>> GaitPlanner::get_future_stance_times(double current_time_s, double footstep_planning_horizon_s)
{
    std::unordered_map<std::string, std::vector<std::pair<double, double>>> future_stance_times;

    for (const auto& leg : {"FL", "FR", "RL", "RR"})
    {
        future_stance_times[leg] = std::vector<std::pair<double, double>>();
    }

    double end_time = current_time_s + footstep_planning_horizon_s; // End time to find future stance entry times


    for (const auto& [leg, leg_phase_offset] : phase_offsets_)
    {
        vector<pair<double, double>>& leg_stance_times = future_stance_times[leg];

        double time_cursor = current_time_s + time_to_next_stance(current_time_s, leg); // Get the time to the next stance phase for the leg

        while (time_cursor < end_time)
        {
            double stance_start = time_cursor;
            double stance_end = stance_start + duty_factor_ * gait_duration_s_;
            
            if (time_cursor < end_time)
            {
                double clipped_stance_end = min(stance_end, end_time); // If the planning horizon ends before stance phase ends, clip the end time
                leg_stance_times.emplace_back(stance_start, clipped_stance_end);
            }

            time_cursor += gait_duration_s_;
        }
    }
    return future_stance_times;
}

int main(int, char**)
{
    GaitPlanner gait_planner(GaitPlanner::GaitType::TROT, 0.5, 1.0);
    vector<double> phases = {0.0, 0.25, 0.5, 0.75};

    for (double phase : phases)
    {
        cout << "Contact states at phase: " << phase << endl;
        unordered_map<std::string, int> contact_state = gait_planner.get_contact_state(phase);
        for (const auto& [leg, state] : contact_state)
            cout << "Leg: " << leg << ", Contact State: " << state << endl;
    }

    return 0;
}
