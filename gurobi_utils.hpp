#pragma once
#include "gurobi_c++.h"
#include <Eigen/Dense>
// #include "gurobi_utils.hpp"



using namespace Eigen;
using namespace std;

// GRBQuadExpr createQuadObj(const vector<GRBVar>* x, const MatrixXd& Q);
GRBQuadExpr create_quad_obj(const GRBVar* x, const MatrixXd& Q, int n);
GRBLinExpr create_lin_obj(const GRBVar* x, const VectorXd& p, int n);
void add_lin_eq_constr(GRBModel model, const GRBCar* x, const MatrixXd& A, const VectorXd b);
