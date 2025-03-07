#pragma once

#include <Eigen/Dense>
#include <iostream>
#include "gurobi_c++.h"
#include "state_space.hpp"
#include "gurobi_utils.hpp"
#include "mpc_params.hpp"
#include "quad_params.hpp"

using namespace std;
using namespace Eigen;
// using namespace GRB;

class ConvexMPC
{
    public:
        ConvexMPC(MPCParams mpc_params, QuadrupedParams quad_params);
        ~ConvexMPC();

        
        void update();
    private:
        // GRBModel model;
        // GRBEnv env;
        MPCParams mpc_params;
        QuadrupedParams quad_params;

        tuple<MatrixXd, MatrixXd> create_state_space_prediction_matrices(StateSpace const& quad_dss, int& N_MPC);

};
