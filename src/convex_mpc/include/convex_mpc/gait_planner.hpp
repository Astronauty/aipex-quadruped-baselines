#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>


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

        GaitPlanner(GaitType gait_type, double duty_factor = 0.5);

        std::unordered_map<std::string, int> get_contact_state(double phase);
        void set_gait_type(GaitType gait_type);

    private:
        GaitType gait_type_;
        double duty_factor_; // Duty factor for the gait, default is 0.5
        std::unordered_map<std::string, double> phase_offsets_; // Phase offsets for each leg in the gait

        void set_phase_offsets();
};

