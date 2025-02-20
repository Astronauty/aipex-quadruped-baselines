#include <Eigen/Dense>
#include "transforms.hpp"

Eigen::Matrix3f eul2rotm(double roll, double pitch, double yaw) {
    Eigen::Matrix3f rotationMatrix;

    Eigen::Matrix3f rotationX; // Roll (about X-axis)
    rotationX << 1, 0, 0,
                 0, cos(roll), -sin(roll),
                 0, sin(roll), cos(roll);

    Eigen::Matrix3f rotationY; // Pitch (about Y-axis)
    rotationY << cos(pitch), 0, sin(pitch),
                 0, 1, 0,
                 -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3f rotationZ; // Yaw (about Z-axis)
    rotationZ << cos(yaw), -sin(yaw), 0,
                 sin(yaw), cos(yaw), 0,
                 0, 0, 1;

    // Extrinsic rotation (ZYX): Yaw * Pitch * Roll
    rotationMatrix = rotationZ * rotationY * rotationX;

    return rotationMatrix;
}

// Function to create the hat map (skew-symmetric matrix) of a 3D vector
Eigen::Matrix3f hatMap(const Eigen::Vector3f& a) {
    Eigen::Matrix3f a_hat;

    a_hat << 0, -a(2), a(1),
             a(2), 0, -a(0),
             -a(1), a(0), 0;

    return a_hat;
}