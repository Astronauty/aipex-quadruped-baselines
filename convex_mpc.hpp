#include <Eigen/Dense>
#include <iostream>
#include "gurobi_c++.h"
#include "state_space.hpp"
#include "gurobi_utils.hpp"

#pragma once

using namespace std;
using namespace Eigen;
using namespace GRB

class ConvexMPC
{
    public:
        ConvexMPC();
        ~ConvexMPC();

        
        void update();
    private:
        GRBModel model;
        GRBEnv env;

        tuple<MatrixXd, MatrixXd> create_state_space_prediciton_matrices(const& StateSpace quad_dss, int& N_MPC);


}
