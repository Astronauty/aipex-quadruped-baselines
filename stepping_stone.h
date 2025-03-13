#include "plane.h"
#include <Eigen/Dense>
#include <drake/math/rigid_transform.h>

// Derived class
class SteppingStone
{
    public:
        SteppingStone(drake::math, double width, double height);
        ~SteppingStone();


        // Halfspace representation for the stepping stone
        Eigen::Matrix<double, 6, 3> getConstraints();
    
    private:
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        double width;
        double height;
        

        Eigen::Matrix<double, 6, 3> A;
            
    

};