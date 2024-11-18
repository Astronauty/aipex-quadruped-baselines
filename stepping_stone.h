#include "plane.h"

// Derived class
class SteppingStone : public Plane 
{
    public:
        SteppingStone(Eigen::Vector3d point, Eigen::Vector3d normal, float width, float height);
        ~SteppingStone();

};