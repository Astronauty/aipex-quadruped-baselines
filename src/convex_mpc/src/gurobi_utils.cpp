#include "gurobi_c++.h"
#include "convex_mpc/gurobi_utils.hpp"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

/**
 * @brief Create a quadratic objective function for gurobi of the form J = (x-x_ref)'P(x-x_ref).
 * @param Q Quadratic cost matrix, expressed as a Eigen MatrixXd.
 * @param x Decision variable, expressed as a pointer to a GRBVar vector.
 */
GRBQuadExpr create_quad_obj(const GRBVar* x, const MatrixXd& P, int n) // Necessary to specify the number of variables since GRBVar is passed by pointer
{
    GRBQuadExpr obj = 0.0;
    // int n = static_cast<int>(Q.rows());

    // Dimension checks
    // if (Q.rows() != x.size()) {
    if (P.rows() != n){
        cout << "Size of P: " << P.rows() << "x" << P.cols() << endl;
        cout << "Value of n: " << n << endl;
        throw invalid_argument("gurobi_utils.cpp : The number of rows in Q must equal the length of x.");
    }

    if (P.rows() != P.cols()) {
        throw invalid_argument("gurobi_utils.cpp : Q must be a square matrix.");
    }

    // Sum the quadratic costs
    for(int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            // obj += Q(i, j) * (x_ref[i] - x[i]) * (x_ref[j] - x[j]);
            obj += P(i, j) * x[i] * x[j];

        }
    }
    return obj;
}

/**
 * @brief Create a quadratic objective function for gurobi of the form J = p'x.
 * @param Q Quadratic cost matrix, expressed as a Eigen MatrixXd.
 * @param x Reference state vector, expressed as a std::vector<double> of gurobi variables.
 */
GRBLinExpr create_lin_obj(const GRBVar* x, const VectorXd& p, int n)
{
    GRBLinExpr obj = 0.0;
    // int n = static_cast<int>(Q.rows());
    for(int i = 0; i < n; i++)
    {
        // obj += Q(i, 0) * (x_ref[i] - x[i]);
        obj += p[i] * x[i];

    }

    return obj;
}


/**
 * @brief Create a Gurobi linear equality constraint in the form:
 Ax = b;
 */
void add_lin_eq_constr(GRBModel model, const GRBVar* x, const MatrixXd& A, const VectorXd b)
{
    // int n = static_cast<int>(b.size());
    // if (A.rows() != b.size()) {
    //     throw invalid_argument("The number of rows in A must equal the length of b.");
    // }

    // if (A.cols() != n) {
    //     throw invalid_argument("The number of columns in A must equal the length of x.");
    // }

    GRBLinExpr lhs;
    for (int i = 0; i < A.rows(); i++) {
        lhs = 0.0;
        for (int j = 0; j < A.cols(); j++) {
            lhs += A(i, j) * x[j];
        }
        model.addConstr(lhs == b[i]);
    }

}