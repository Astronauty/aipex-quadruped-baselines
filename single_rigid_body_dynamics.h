#pragma once
#include<Eigen/Dense>
using namespace Eigen;

struct{
    float mass;
    Matrix3f inertiaTensor;
} inertialParamsSRB;

class SRBDynamics
{
    public:
        SRBDynamics();
        ~SRBDynamics();
    
    private:
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        
        float width;
        float height;
        

        Eigen::Matrix<double, 6, 3> A;
            
    

};

