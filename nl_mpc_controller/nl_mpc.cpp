#include "gurobi_c++.h"
using namespace std;

/*
Create a quadratic objective function for GRB
*/
GRBQuadExpr createQuadObj(const std::vector<GRBVar> &z, const MatrixXd &Q)
{
    GRBQuadExpr obj = 0.0;
    int n = Q.rows();
    for(int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            obj += Q(i, j) * z[i] * z[j];
        }
    }

    return obj;
}

GRBConstr createConstr(const std::vector<GRBVar> &z, const MatrixXd &A, const std::vector<GRBVar> &z_next, const MatrixXd &b)
{
    GRBConstr constr = 0.0;
    int n = A.rows();
    for(int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            constr += A(i, j) * z[j];
        }
    }

    return constr;
}

int main(int argc, char* argv[])
{
    GRBEnv *env = new GRBEnv();
    GRBModel model = GRBModel(env);

    const int N_STATES = 18; // Number of states
    const int N_CONTROLS = 12; // Number of control inputs (forces per foot)
    const int N_MPC; // Horizon length for MPC

    MatrixXd Q = MatrixXd::Identity(N_STATES, N_STATES); // State cost matrix
    MatrixXd R = MatrixXd::Identity(N_CONTROLS, N_CONTROLS); // Control cost matrix

    // GRBVar z = model.addVars(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "z", n);
    GRBVar *x = model.addVars(N_STATES, GRB_CONTINUOUS); // Trajectories z = [x, u]
    std::vector<GRBVar> xVec(x, x + N_STATES);

    
    GRBVar *u = model.addVars(N_CONTROLS, GRB_CONTINUOUS); // Control inputs


    // Quadratic LQR cost function
    GRBQuadExpr obj = createQuadObj(x, Q) + createQuadObj(u, R);
    model.setObjective(obj, GRB_MINIMIZE);

    // Primal constraints

    model.optimize();



    std::cout << z << std::endl;

    // std::vector<GRBVar> x(n);

    // // Add variables to the model and store them in the vector
    // for (int i = 0; i < n; ++i) {
    //     x[i] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x" + std::to_string(i));
    // }

}