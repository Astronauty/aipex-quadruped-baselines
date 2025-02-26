#pragma once
#include<Eigen/Dense>
using namespace Eigen;

struct inertialParamsSRB {
    float mass;
    Matrix3f
    Matrix3f inertiaTensor;
};

class SRBDynamics
{
    public:
        SRBDynamics();
        ~SRBDynamics();

        Eigen::Matrix3d eul2rotm(double roll, double pitch, double yaw);
        Eigen::Matrix3d hatMap(const Eigen::Vector3d& a);
    
    private:
        // Matrix<float, 18, 1> x;
        // Matrix<float, 4, 3> u;
        // Matrix<float, 4, 3> p_f;
        // Matrix<float, 12, 1> x_dot;

        Matrix<float, 3, 4> r_i; // Foot positions relative to COM

        inertialParams params;

        void srb_dynamics(inertialParamsSRB params, Matrix<float, 18, 1> x, Matrix<float, 4, 3> u, Matrix<float, 4, 3> p_f);
};

