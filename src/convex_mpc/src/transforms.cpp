#include <Eigen/Dense>
#include "convex_mpc/transforms.hpp"

Eigen::Matrix3d eul2rotm(double roll, double pitch, double yaw) {
    Eigen::Matrix3d rotationMatrix;

    Eigen::Matrix3d rotationX; // Roll (about X-axis)
    rotationX << 1, 0, 0,
                 0, cos(roll), -sin(roll),
                 0, sin(roll), cos(roll);

    Eigen::Matrix3d rotationY; // Pitch (about Y-axis)
    rotationY << cos(pitch), 0, sin(pitch),
                 0, 1, 0,
                 -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d rotationZ; // Yaw (about Z-axis)
    rotationZ << cos(yaw), -sin(yaw), 0,
                 sin(yaw), cos(yaw), 0,
                 0, 0, 1;

    // Extrinsic rotation (ZYX): Yaw * Pitch * Roll
    rotationMatrix = rotationZ * rotationY * rotationX;

    return rotationMatrix;
}

// Function to create the hat map (skew-symmetric matrix) of a 3D vector
Eigen::Matrix3d hatMap(const Eigen::Vector3d& a) {
    Eigen::Matrix3d a_hat;

    a_hat << 0, -a(2), a(1),
             a(2), 0, -a(0),
             -a(1), a(0), 0;

    return a_hat;
}