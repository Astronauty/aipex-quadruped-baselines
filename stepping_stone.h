#include "plane.h"
#include <Eigen/Dense>
#include <drake/math/rigid_transform.h>

// Derived class
class SteppingStone
{
    public:
        SteppingStone(drake::math, float width, float height);
        ~SteppingStone();


        // Halfspace representation for the stepping stone
        Eigen::Matrix<double, 6, 3> getConstraints();
    
    private:
        Eigen::Vector3d point;
        Eigen::Vector3d normal;
        float width;
        float height;
        

        Eigen::Matrix<double, 6, 3> A;
            
    

};