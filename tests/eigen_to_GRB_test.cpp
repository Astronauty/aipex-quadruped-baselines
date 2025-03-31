#include <Eigen/Dense>
#include <iostream>
#include "gurobi_c++.h"
#include <vector>


#include "../gurobi_utils.hpp"
using namespace Eigen;
using namespace std;
// nt main(int argc, char* argv[])
// {
//     GRBEnv *env = new GRBEnv();
//     GRBModel model = GRBModel(env);

//     // MPC Parameters
//     const int N_STATES = 18; // Number of states
//     const int N_CONTROLS = 12; // Number of control inputs (forces per foot)
//     const int N_MPC = 1; // Horizon length for MPC

//     MatrixXd Q = MatrixXd::Identity(N_STATES, N_STATES); // State cost matrix
//     MatrixXd Qn = MatrixXd::Identity(N_STATES, N_STATES); // Terminal state cost matrix
//     MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS); // Control cost matrix

//     std::vector<

//     // GRBVar z = model.addVars(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "z", n);
//     GRBVar *x = model.addVars(N_STATES*N_MPC, GRB_CONTINUOUS); // Trajectories z = [x, u]
//     std::vector<GRBVar> xVec(x, x + N_STATES);

//     GRBVar *u = model.addVars(N_CONTROLS*(N_MPC-1), GRB_CONTINUOUS); // Control inputs
//     std::vector<GRBVar> uVec(u, u + N_CONTROLS);


//     // Quadratic LQR cost function
//     GRBQuadExpr obj = createQuadObj(xVec, Q) + createQuadObj(uVec, R);
//     model.setObjective(obj, GRB_MINIMIZE);

//     // Primal constraints
//     model.addConstr(xVec[0] == 1.0, "c0");

//     // Dynamics constraints
//     for (int k=0; k < N_MPC-1; k++)
//     {
//         // x_next = f(x, u)
//         // model.addConstr(xVec[k+1] == f(xVec[k], uVec[k]), "dynamics");
//     }


//     // Optimize
//     model.optimize();

//     // Print outputs
//     std::cout << "\nOptimized state variables (x):" << std::endl;
//     for (int i = 0; i < N_STATES; ++i) {
//         double x_value = xVec[i].get(GRB_DoubleAttr_X);
//         std::cout << "x[" << i << "] = " << x_value << std::endl;
//     }

//     std::cout << "\nOptimized control input variables (u):" << std::endl;
//     for (int i = 0; i < N_CONTROLS; ++i) {
//         double u_value = uVec[i].get(GRB_DoubleAttr_X);
//         std::cout << "u[" << i << "] = " << u_value << std::endl;
//     }


    
//     // std::vector<GRBVar> x(n);

//     // // Add variables to the model and store them in the vector
//     // for (int i = 0; i < n; ++i) {
//     //     x[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x" + std::to_string(i));
//     // }

// }

int main(int argc, char* argv[])
{
    GRBEnv env = GRBEnv();

    GRBModel model = GRBModel(env);
    
    GRBVar *x = model.addVars(3, GRB_CONTINUOUS);
    
    MatrixXd Q = MatrixXd::Identity(3, 3); // Quadratic cost 
    // VectorXd p = VectorXd::Ones(3, 1); // Linear cost
    VectorXd p(3);
    p << 1, 5, -3;


    GRBQuadExpr quad_obj = create_quad_obj(x, Q, 3);
    GRBLinExpr lin_obj = create_lin_obj(x, p, 3);

    // GRBQuadExpr obj = quad_obj + lin_obj;
    // GRBQuadExpr obj = x[1]*x[1] + x[2]*x[1]+5;
    // model.setObjective(obj);


    model.setObjective(quad_obj + lin_obj, GRB_MINIMIZE);
    std::cout << "Objective function: " << model.getObjective() << std::endl;
    model.optimize();

    std::cout << "Optimized variables:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        double x_value = x[i].get(GRB_DoubleAttr_X);
        std::cout << "x[" << i << "] = " << x_value << std::endl;
    }


}