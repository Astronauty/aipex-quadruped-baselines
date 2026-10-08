#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <tuple>
#include <cassert>
#include "convex_mpc/mpc_params.hpp"
#include "convex_mpc/state_indices.hpp"

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

    // cout << "desired_time_s "

    const bool verbose = mpc_params.verbose_diagnostics;

    if (verbose) {
        cout << "desired_time_s: " << desired_time_s - current_time_s << ", current_time_s: " << current_time_s << endl;
        cout << "gait planning time limit" << mpc_params.dt * (mpc_params.N_MPC - 1)<< endl;
    }

    assert(desired_time_s >= current_time_s && "Desired time must be greater than or equal to current time");

    double timesteps_to_go = (desired_time_s - current_time_s) / mpc_params.dt;

    // Footstep planning has to look a full swing ahead, which is longer than the
    // MPC horizon (N_MPC * dt = 0.25 s by default). Rather than assert -- or, in
    // an NDEBUG build, silently read past the end of X_ref via upper_index --
    // extrapolate beyond the last reference state at its own velocity.
    //
    // This is exact for the reference this controller actually builds:
    // reference_traj_from_joy() lays out a constant-velocity ramp, so continuing
    // it in a straight line reproduces what the next samples would have been.
    // The MPC's own N_MPC and dt are untouched, so stance tuning is unaffected.
    const int last_index = mpc_params.N_MPC - 1;
    if (timesteps_to_go > static_cast<double>(last_index))
    {
        Eigen::VectorXd x_last = X_ref.segment<12>(last_index * mpc_params.N_STATES);

        const double extrapolation_s = (timesteps_to_go - static_cast<double>(last_index)) * mpc_params.dt;

        Eigen::VectorXd x = x_last;
        x.segment<3>(IDX_X) += x_last.segment<3>(IDX_Xd) * extrapolation_s; // position advances at the reference velocity
        x.segment<3>(IDX_R) += x_last.segment<3>(IDX_Rd) * extrapolation_s; // orientation advances at the reference rate

        return x;
    }

    int lower_index = static_cast<int>(std::floor(timesteps_to_go));
    int upper_index = lower_index + 1;

    double alpha = timesteps_to_go - static_cast<double>(lower_index);

    if (verbose) {
        cout << "timesteps_to_go: " << timesteps_to_go << endl;
        cout << "lower_index: " << lower_index << ", upper_index: " << upper_index << endl;
        cout << "alpha: " << alpha << endl;
    }

    // const int pos_state_index = 3; // Position state index
    // const int vel_state_index = 6; // Velocity state index

    // p_COM = (1 - alpha) * X_ref.block<3, 1>(lower_index * mpc_params.N_STATES + pos_state_index, 0) + alpha * X_ref.block<3, 1>(upper_index * mpc_params.N_STATES + pos_state_index, 0);
    // V_COM = (1 - alpha) * X_ref.block<3, 1>(lower_index * mpc_params.N_STATES + vel_state_index, 0) + alpha * X_ref.block<3, 1>(upper_index * mpc_params.N_STATES + vel_state_index, 0);

    Eigen::VectorXd x = Eigen::VectorXd::Zero(mpc_params.N_STATES);

    // The segment width is 12 while the stride is N_STATES (13): each reference
    // state is gravity-augmented, and the constant gravity slot is deliberately
    // dropped here since no consumer of this function reads it. Stride and width
    // are intentionally different -- do not "fix" the 12 to N_STATES.
    x = (1.0-alpha) * X_ref.segment<12>(lower_index * mpc_params.N_STATES) + alpha * X_ref.segment<12>(upper_index * mpc_params.N_STATES);

    // return std::make_tuple(p_COM, V_COM);
    return x;
}

 