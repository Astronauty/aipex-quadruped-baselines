#include "convex_mpc/gait_planner.hpp"

GaitPlanner::GaitPlanner(GaitType gait_type, double duty_factor)
    : gait_type_(gait_type), duty_factor_(duty_factor)
{
    this->set_phase_offsets();
}

/**
 * @brief Set the phase offsets for each leg based on the selected gait type.
 * 
 * @param void The current phase of the gait (0 to 1).
 * @return A map with leg names as keys and contact state (0 or 1) as values.
 */
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
                {"RR", 0.5}
            }
            break;
        
        default:
            throw std::invalid_argument("Selected gait type is not implemented.");
            
    }   
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

