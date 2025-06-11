#pragma once
#include<Eigen/Dense>
using namespace Eigen;

struct inertialParamsSRB {
    double mass;
    Matrix3d
    Matrix3d inertiaTensor;
};

class SRBDynamics
{
    public:
        SRBDynamics();
        ~SRBDynamics();

        Eigen::Matrix3d eul2rotm(double roll, double pitch, double yaw);
        Eigen::Matrix3d hatMap(const Eigen::Vector3d& a);
    
    private:
        // Matrix<double, 18, 1> x;
        // Matrix<double, 4, 3> u;
        // Matrix<double, 4, 3> p_f;
        // Matrix<double, 12, 1> x_dot;

        Matrix<double, 3, 4> r_i; // Foot positions relative to COM

        inertialParams params;

        void srb_dynamics(inertialParamsSRB params, Matrix<double, 18, 1> x, Matrix<double, 4, 3> u, Matrix<double, 4, 3> p_f);
};

