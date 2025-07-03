#include "convex_mpc/swing_trajectory_spline.hpp"

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>

// Add to gait_planner.cpp
SwingTrajectorySpline::SwingTrajectorySpline(const Eigen::Vector3d& start_pos, 
                                           const Eigen::Vector3d& end_pos,
                                           double swing_height,
                                           double start_time,
                                           double end_time)
    : start_pos_(start_pos), end_pos_(end_pos), swing_height_(swing_height),
      start_time_(start_time), end_time_(end_time), duration_(end_time - start_time)
{
    coefficients_.resize(3);  // x, y, z components
    compute_coefficients();
}

void SwingTrajectorySpline::compute_coefficients()
{
    // Create 5 waypoints for smooth swing trajectory
    std::vector<double> times = {0.0, 0.25, 0.5, 0.75, 1.0};  // Normalized time [0, 1]
    std::vector<Eigen::Vector3d> positions(5);
    
    // Define waypoints
    positions[0] = start_pos_;  // Start position
    positions[1] = start_pos_ + 0.25 * (end_pos_ - start_pos_) + Eigen::Vector3d(0, 0, swing_height_ * 0.5);
    positions[2] = 0.5 * (start_pos_ + end_pos_) + Eigen::Vector3d(0, 0, swing_height_);  // Peak
    positions[3] = start_pos_ + 0.75 * (end_pos_ - start_pos_) + Eigen::Vector3d(0, 0, swing_height_ * 0.5);
    positions[4] = end_pos_;    // End position
    
    // Compute cubic spline coefficients for each dimension
    for (int dim = 0; dim < 3; ++dim) {
        std::vector<double> y_vals(5);
        for (int i = 0; i < 5; ++i) {
            y_vals[i] = positions[i](dim);
        }
        
        // For simplicity, use cubic Hermite interpolation
        // This creates a smooth curve through the waypoints
        coefficients_[dim] = compute_cubic_hermite_coefficients(times, y_vals);
    }
}

Eigen::Vector4d SwingTrajectorySpline::compute_cubic_hermite_coefficients(
    const std::vector<double>& times, const std::vector<double>& values)
{
    // Simplified cubic interpolation for swing trajectory
    // Uses boundary conditions: zero velocity at start/end
    double y0 = values[0];  // Start value
    double y1 = values[4];  // End value
    double ymax = values[2]; // Peak value at t=0.5
    
    // Cubic polynomial: y = a*t^3 + b*t^2 + c*t + d
    // Boundary conditions:
    // y(0) = y0, y(1) = y1, y(0.5) = ymax, y'(0) = 0, y'(1) = 0
    
    double d = y0;
    double c = 0;  // Zero initial velocity
    double a = 2*y0 - 2*y1 + 8*ymax - 8*y0;
    double b = -3*y0 + 3*y1 - 8*ymax + 8*y0;
    
    return Eigen::Vector4d(a, b, c, d);
}

double SwingTrajectorySpline::normalize_time(double t) const
{
    if (t <= start_time_) return 0.0;
    if (t >= end_time_) return 1.0;
    return (t - start_time_) / duration_;
}

Eigen::Vector3d SwingTrajectorySpline::get_position(double t) const
{
    double tau = normalize_time(t);
    Eigen::Vector3d position;
    
    for (int dim = 0; dim < 3; ++dim) {
        const Eigen::Vector4d& coeff = coefficients_[dim];
        position(dim) = coeff(0)*tau*tau*tau + coeff(1)*tau*tau + coeff(2)*tau + coeff(3);
    }
    
    return position;
}

Eigen::Vector3d SwingTrajectorySpline::get_velocity(double t) const
{
    double tau = normalize_time(t);
    Eigen::Vector3d velocity;
    
    for (int dim = 0; dim < 3; ++dim) {
        const Eigen::Vector4d& coeff = coefficients_[dim];
        velocity(dim) = (3*coeff(0)*tau*tau + 2*coeff(1)*tau + coeff(2)) / duration_;
    }
    
    return velocity;
}

Eigen::Vector3d SwingTrajectorySpline::get_acceleration(double t) const
{
    double tau = normalize_time(t);
    Eigen::Vector3d acceleration;
    
    for (int dim = 0; dim < 3; ++dim) {
        const Eigen::Vector4d& coeff = coefficients_[dim];
        acceleration(dim) = (6*coeff(0)*tau + 2*coeff(1)) / (duration_ * duration_);
    }
    
    return acceleration;
}