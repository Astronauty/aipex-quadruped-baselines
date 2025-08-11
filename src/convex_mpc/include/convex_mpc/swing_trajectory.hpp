#pragma once

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

// Class to define spline for swing trajectories
// class SwingTrajectorySpline
// {
// public:
//     SwingTrajectorySpline(const Eigen::Vector3d& start_pos, 
//                          const Eigen::Vector3d& end_pos,
//                          double swing_height,
//                          double start_time,
//                          double end_time);
    
//     Eigen::Vector3d get_position(double t) const;
//     Eigen::Vector3d get_velocity(double t) const;
//     Eigen::Vector3d get_acceleration(double t) const;
    
// private:
//     Eigen::Vector3d start_pos_;
//     Eigen::Vector3d end_pos_;
//     double swing_height_;
//     double start_time_;
//     double end_time_;
//     double duration_;
    
//     // Cubic polynomial coefficients for each dimension
//     std::vector<Eigen::Vector4d> coefficients_;  // [a, b, c, d] for each x, y, z
    

// };

// Update SwingTrajectory struct
struct SwingLegTrajectory
{
    double start_time_s;
    double end_time_s;
    Eigen::Vector3d start_position;
    Eigen::Vector3d end_position;
    double swing_height;
    // SwingTrajectorySpline spline;
    
    SwingLegTrajectory(double start_time, double end_time, 
                   const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos,
                   double height = 0.08)
        : start_time_s(start_time), end_time_s(end_time),
          start_position(start_pos), end_position(end_pos), swing_height(height) {}
        //   spline(start_pos, end_pos, height, start_time, end_time) {}
    
};

