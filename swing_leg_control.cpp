

#include <Eigen/Dense>

double Kp;
double Kd;

using namespace std;
using namespace eigen;

Matrix<float, 3, 4> compute_joint_torques(vector<Vector3d>& p_ref, vector<Vector3d>& v_ref, vector<Vector3d>& p, vector<Vector3d>& v)
{

    % Compute feed forward torques
    vector<Vector3d> torque_feedforward = computeFeedForwardToruq

    return joint_torques
}


vector<Matrix3d> compute_foot_jacobians(Vector3d joint_angles)
{
    foot_jacobians = vector<Matrix3d>
    

    return foot_jacobians
}

vector< computeFeedForwardTorque>