#include "gurobi_c++.h"
#include "convex_mpc/gurobi_utils.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

/**
 * @brief Create the quadratic objective J = 1/2 x'Px for gurobi.
 *
 * The 1/2 matters: the MPC builds P = 2(B'QB + R) and q = 2B'Q(Ax0 - X_ref), the
 * convention for min 1/2 x'Px + q'x. Without it the objective was x'Px + q'x, whose
 * minimiser is exactly half the intended one -- every tracking correction and the
 * gravity compensation were halved.
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

    // Sum the quadratic costs. P is symmetric, so the off-diagonal pairs (i,j) and
    // (j,i) are folded into a single 2*P(i,j) term, and everything is handed to
    // Gurobi in one addTerms() call rather than n^2 individual operator+= calls.
    std::vector<double> coeffs;
    std::vector<GRBVar> vars1, vars2;
    coeffs.reserve(static_cast<size_t>(n) * (n + 1) / 2);
    vars1.reserve(static_cast<size_t>(n) * (n + 1) / 2);
    vars2.reserve(static_cast<size_t>(n) * (n + 1) / 2);

    for (int i = 0; i < n; i++)
    {
        if (P(i, i) != 0.0) {
            coeffs.push_back(0.5 * P(i, i));
            vars1.push_back(x[i]);
            vars2.push_back(x[i]);
        }
        for (int j = i + 1; j < n; j++)
        {
            const double c = 0.5 * (P(i, j) + P(j, i)); // symmetric pair, with the 1/2
            if (c != 0.0) {
                coeffs.push_back(c);
                vars1.push_back(x[i]);
                vars2.push_back(x[j]);
            }
        }
    }

    if (!coeffs.empty()) {
        obj.addTerms(coeffs.data(), vars1.data(), vars2.data(), static_cast<int>(coeffs.size()));
    }
    return obj;
}

/**
 * @brief Create a linear objective function for gurobi of the form J = p'x.
 * @param x Decision variables, expressed as a pointer to a GRBVar vector.
 * @param p Linear cost vector, expressed as an Eigen VectorXd.
 * @param n Number of variables.
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