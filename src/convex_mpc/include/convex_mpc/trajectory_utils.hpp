#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <tuple>
#include <cassert>
#include "convex_mpc/mpc_params.hpp"

/** 
 *@param X_ref Reference trajectory vector for the entire horizon, size N_MPC * N_STATES.
 *@param N_STATES Number of states.
 *@param N_MPC Number of steps in the MPC horizon.
 *@param current_time_s Current time in seconds.
 *@param desired_time_s Desired time in seconds to get the COM position and velocity.
 *@return State vector (possibly interpolated) at the desired time.
 */
inline Eigen::VectorXd get_state_at_time_from_ref_traj(const Eigen::VectorXd& X_ref, const MPCParams& mpc_params, double current_time_s, double desired_time_s)
{
    Eigen::Vector3d p_COM;
    Eigen::Vector3d V_COM;

    assert(desired_time_s >= current_time_s && "Desired time must be greater than or equal to current time");
    assert(desired_time_s <= current_time_s + mpc_params.dt * mpc_params.N_STATES && "Desired time must be within the reference trajectory horizon.");

    double timesteps_to_go = (desired_time_s - current_time_s) / mpc_params.dt;

    cout << "timesteps_to_go: " << timesteps_to_go << endl;

    int lower_index = static_cast<int>(std::floor(timesteps_to_go));
    int upper_index = lower_index + 1;

    cout << "lower_index: " << lower_index << ", upper_index: " << upper_index << endl;

    double alpha = timesteps_to_go - static_cast<double>(lower_index);
    cout << "alpha: " << alpha << endl;

    // const int pos_state_index = 3; // Position state index
    // const int vel_state_index = 6; // Velocity state index

    // p_COM = (1 - alpha) * X_ref.block<3, 1>(lower_index * mpc_params.N_STATES + pos_state_index, 0) + alpha * X_ref.block<3, 1>(upper_index * mpc_params.N_STATES + pos_state_index, 0);
    // V_COM = (1 - alpha) * X_ref.block<3, 1>(lower_index * mpc_params.N_STATES + vel_state_index, 0) + alpha * X_ref.block<3, 1>(upper_index * mpc_params.N_STATES + vel_state_index, 0);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(mpc_params.N_STATES);
    // Eigen::VectorXd x = (1.0-alpha) * X_ref.segment<12>(lower_index * mpc_params.N_STATES) + alpha * X_ref.segment<12>(upper_index * mpc_params.N_STATES);

    x = (1.0-alpha) * X_ref.segment<12>(lower_index * mpc_params.N_STATES) + alpha * X_ref.segment<12>(upper_index * mpc_params.N_STATES);

    // return std::make_tuple(p_COM, V_COM);
    return x;
}

 