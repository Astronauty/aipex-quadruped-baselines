#include <Eigen/Dense>

using namespace Eigen;
using namespace std;
#pragma once

struct MPCParams 
{
    const int N_MPC;      // Number of steps in the MPC horizon
    const int N_CONTROLS;   // Time horizon for the control
    const int N_STATES;
    const double dt;  // Sampling time interval
    MatrixXd Q; // Quadratic state weight
    MatrixXd R; //Quadratic control weight
    VectorXd u_lower; // Lower limits for controls
    VectorXd u_upper; // Upper limits for controls

    const double Q_n_scale; // Scaling factor for terminal state weight
    const double min_vertical_grf; // Minimum vertical ground reaction force

    // MPCParams(): 
    //     N_MPC(-1),
    //     N_CONTROLS(-1),
    //     N_STATES(-1),
    //     dt(0.0),
    //     Q(MatrixXd::Zero(0, 0)),
    //     R(MatrixXd::Zero(0, 0)),
    //     u_lower(VectorXd::Zero(0)),
    //     u_upper(VectorXd::Zero(0)) {};

    MPCParams(int n_mpc, int n_controls, int n_states, double dt, MatrixXd Q, double Q_n_scale, MatrixXd R, VectorXd u_lower, VectorXd u_upper, double min_vertical_grf)
    : N_MPC(n_mpc),
      N_CONTROLS(n_controls),
      N_STATES(n_states),
      dt(dt),
      Q(Q),
      Q_n_scale(Q_n_scale),
      R(R),
      u_lower(u_lower),
      u_upper(u_upper),
      min_vertical_grf(min_vertical_grf){}
};
