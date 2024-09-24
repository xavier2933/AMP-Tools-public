#include "ManipulatorSkeleton.h"
#define _USE_MATH_DEFINES
#include<cmath>
#include <Eigen/Dense>

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0, 1.0}) // Default to a 2-link with all links of 1.0 length
{}

// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement forward kinematics to calculate the joint position given the manipulator state (angles)
    // std::vector<Eigen::Vector2d> joint_positions = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 1.0), Eigen::Vector2d(1.0, 1.0)};
    // std::cout << joint_positions[1] << std::endl;

    // Joint angles (Replace these with actual state variables)
    // double theta1 = M_PI / 6; // Replace with state(0)
    // double theta2 = M_PI / 3; // Replace with state(1)
    // double theta3 = 7 * M_PI / 4; // Replace with state(2)
    double theta1 = -0.505361; // Replace with state(0)
    double theta2 = 1.82348; // Replace with state(1)
    double theta3 = -1.31812; // Replace with state(2)


    // Link lengths
    // double a1 = 0.5;
    // double a2 = 1.0;
    // double a3 = 0.5;
    double a1 = 1.0;
    double a2 = 0.5;
    double a3 = 1.0;

    // Define transformation matrices using Eigen
    Eigen::Matrix3d T1, T2, T3, T4;

    // Transformation from base frame to first joint
    T1 << std::cos(theta1), -std::sin(theta1), 0,
          std::sin(theta1), std::cos(theta1), 0,
          0, 0, 1.0;

    // Transformation from first joint to second joint
    T2 << std::cos(theta2), -std::sin(theta2), a1,
          std::sin(theta2), std::cos(theta2), 0,
          0, 0, 1.0;

    // Transformation from second joint to third joint
    T3 << std::cos(theta3), -std::sin(theta3), a2,
          std::sin(theta3), std::cos(theta3), 0,
          0, 0, 1.0;

    // Transformation from third joint to end effector (translation only, no rotation)
    T4 << 1.0, 0, a3,
          0, 1.0, 0,
          0, 0, 1.0;

    // Compute cumulative transformations for each joint
    Eigen::Matrix3d T_joint1 = T1;
    Eigen::Matrix3d T_joint2 = T1 * T2;
    Eigen::Matrix3d T_joint3 = T1 * T2 * T3;
    Eigen::Matrix3d T_end_effector = T1 * T2 * T3 * T4;

    // Extract the (x, y) position of the base, joints, and end-effector
    Eigen::Vector3d base_position(0.0, 0.0, 1.0); // Homogeneous coordinates
    Eigen::Vector3d joint1_position = T_joint1 * base_position;
    Eigen::Vector3d joint2_position = T_joint2 * base_position;
    Eigen::Vector3d joint3_position = T_joint3 * base_position;
    Eigen::Vector3d end_effector_position = T_end_effector * base_position;

    // Extract the (x, y) part of the positions and store them
    std::vector<Eigen::Vector2d> joint_positions;
    joint_positions.push_back(joint1_position.head<2>()); // Position of joint 1
    joint_positions.push_back(joint2_position.head<2>()); // Position of joint 2
    joint_positions.push_back(joint3_position.head<2>()); // Position of joint 3
    joint_positions.push_back(end_effector_position.head<2>()); // Position of end effector

    // Print positions for debugging
    std::cout << "Joint 1 position: " << joint1_position.head<2>().transpose() << std::endl;
    std::cout << "Joint 2 position: " << joint2_position.head<2>().transpose() << std::endl;
    std::cout << "Joint 3 position: " << joint3_position.head<2>().transpose() << std::endl;
    std::cout << "End effector position: " << end_effector_position.head<2>().transpose() << std::endl;

    // Return the position of the requested joint
    return joint_positions[joint_index];
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    // Implement inverse kinematics here
amp::ManipulatorState joint_angles; 
joint_angles.resize(3);
joint_angles.setZero();

// Link lengths
double a1 = 1.0; // Length of first link
double a2 = 0.5; // Length of second link
double a3 = 1.0; // Length of third link
double xe = 2.0;  // End-effector x position
double ye = 0.0;  // End-effector y position
double gamma = atan2(ye, xe);

// Calculate the wrist position (P2)
double x3 = xe - a3 * cos(gamma);
double y3 = ye - a3 * sin(gamma);

// Distance from the base to the wrist position
double C = std::sqrt(x3 * x3 + y3 * y3);

// Use the law of cosines to find theta2
double cos_theta2 = (C * C - a1 * a1 - a2 * a2) / (2 * a1 * a2);
joint_angles(1) = acos(cos_theta2);  // Theta2

// Solve for theta1 using trigonometry
double theta1_part1 = atan2(y3, x3);
double theta1_part2 = atan2(a2 * sin(joint_angles(1)), a1 + a2 * cos(joint_angles(1)));
joint_angles(0) = theta1_part1 - theta1_part2;  // Theta1

// Solve for theta3
joint_angles(2) = gamma - joint_angles(0) - joint_angles(1);  // Theta3

std::cout << "joint angles: " << joint_angles(0) << " " << joint_angles(1) << " " << joint_angles(2) << std::endl;

// Return the computed joint angles
return joint_angles;



    // If you have different implementations for 2/3/n link manipulators, you can separate them here
    // if (nLinks() == 2) {

    //     return joint_angles;
    // } else if (nLinks() == 3) {

    //     return joint_angles;
    // } else {

    //     return joint_angles;
    // }

    // return joint_angles;
}