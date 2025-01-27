/* Copyright 2024, Gurobi Optimization, LLC

This example considers the following nonconvex nonlinear problem

 minimize   sin(x) + cos(2*x) + 1
 subject to  0.25*exp(x) - x <= 0
             -1 <= x <= 4

 We show you two approaches to solve it as a nonlinear model:

  1) Set the paramter FuncNonlinear = 1 to handle all general function
     constraints as true nonlinear functions.

  2) Set the attribute FuncNonlinear = 1 for each general function
     constraint to handle these as true nonlinear functions.
*/
#if defined (WIN32) || defined (WIN64)
#include <Windows.h>
#endif

#include "gurobi_c++.h"
using namespace std;

static void
printsol(GRBModel& m, GRBVar& x)
{
  cout << "x = " << x.get(GRB_DoubleAttr_X) << endl;
  cout << "Obj = " << m.get(GRB_DoubleAttr_ObjVal) << endl;
}

int
main(int argc, char* argv[])
{
  try {

    // Create environment

    GRBEnv env = GRBEnv();

    // Create a new model

    GRBModel m = GRBModel(env);

    // Create variables
    GRBVar x     = m.addVar(-1.0, 4.0, 0.0, GRB_CONTINUOUS, "x");
    GRBVar twox  = m.addVar(-2.0, 8.0, 0.0, GRB_CONTINUOUS, "twox");
    GRBVar sinx  = m.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "sinx");
    GRBVar cos2x = m.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "cos2x");
    GRBVar expx  = m.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "expx");

    // Set objective
    m.setObjective(sinx + cos2x + 1, GRB_MINIMIZE);

    // Add linear constraints

    m.addConstr(0.25*expx - x <= 0, "l1");
    m.addConstr(2*x - twox == 0, "l2");

    // Add general function constraints
    // sinx = sin(x)
    GRBGenConstr gcf1 = m.addGenConstrSin(x, sinx, "gcf1");
    // cos2x = cos(twox)
    GRBGenConstr gcf2 = m.addGenConstrCos(twox, cos2x, "gcf2");
    // expx = exp(x)
    GRBGenConstr gcf3 = m.addGenConstrExp(x, expx, "gcf3");

  // Approach 1) Set FuncNonlinear parameter

    m.set(GRB_IntParam_FuncNonlinear, 1);

    // Optimize the model and print solution

    m.optimize();
    printsol(m, x);

    // Restore unsolved state and reset FuncNonlinear parameter to its
    // default value
    m.reset();
    m.set(GRB_IntParam_FuncNonlinear, 0);

  // Approach 2) Set FuncNonlinear attribute for every
  //             general function constraint

    gcf1.set(GRB_IntAttr_FuncNonlinear, 1);
    gcf2.set(GRB_IntAttr_FuncNonlinear, 1);
    gcf3.set(GRB_IntAttr_FuncNonlinear, 1);

    // Optimize the model and print solution

    m.optimize();
    printsol(m, x);

  } catch(GRBException e) {
    cout << "Error code = " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch(...) {
    cout << "Exception during optimization" << endl;
  }

  return 0;
}