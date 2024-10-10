#include "ManipulatorSkeleton.h"
#define _USE_MATH_DEFINES
#include<cmath>
#include <Eigen/Dense>

MyManipulator2D::MyManipulator2D()
    : LinkManipulator2D({1.0,1.0}) // Default to a 2-link with all links of 1.0 length
{}

// MyManipulator2D::MyManipulator2D() :
//         LinkManipulator2D(const std::vector<double>& link_lengths) 


// MyManipulator2D::MyManipulator2D(const std::vector<double>& link_lengths): LinkManipulator2D({0.5,1.0,0.5});
// Override this method for implementing forward kinematics
Eigen::Vector2d MyManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {

    std::vector<double> link_lengths_ = getLinkLengths();

    // std::cout << "link lengths " << link_lengths_[0] << std::endl;
    // if (joint_index >= state.size() || joint_index >= link_lengths_.size()) {
    //     throw std::out_of_range("Invalid joint index");
    // }

    // Base matrix
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

    Eigen::Vector3d base_position(0.0, 0.0, 1.0);

    for (uint32_t i = 0; i <= joint_index; ++i) {
        double theta = state[i];        // Joint angle
        double link_length = (i == 0) ? 0.0 : link_lengths_[i - 1]; 

        Eigen::Matrix3d T_joint;
        T_joint << std::cos(theta), -std::sin(theta), link_length,
                   std::sin(theta),  std::cos(theta), 0,
                   0, 0, 1.0;

        T *= T_joint;
    }

    Eigen::Vector3d joint_position = T * base_position;
    // std::cout << "returning " << joint_position.head<2>() << std::endl;
    // Return the (x, y) part of the position
    return joint_position.head<2>();
}

// Override this method for implementing inverse kinematics
amp::ManipulatorState MyManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
        // Implement inverse kinematics here
    amp::ManipulatorState joint_angles; 
    std::vector<double> link_lengths_ = getLinkLengths();

    joint_angles.resize(link_lengths_.size());
    joint_angles.setZero();

    // Link lengths
    if(link_lengths_.size() == 3)
    {
        double a1 = link_lengths_[0]; // Length of first link
        double a2 = link_lengths_[1]; // Length of second link
        double a3 = link_lengths_[2]; // Length of third link
    } else {
        double a1 = 1.0; // Length of first link
        double a2 = 0.5; // Length of second link
        double a3 = 1.0; // Length of third link
    }
    double a1 = 1.0; // Length of first link
    double a2 = 0.5; // Length of second link
    double a3 = 1.0; // Length of third link
    // double xe = 2.0;
    // double ye = 0.0;
    double xe = end_effector_location.x();  // End-effector x position
    double ye = end_effector_location.y();  // End-effector y position
    double gamma = atan2(ye, xe);

    // Calculate the wrist position (P2)
    double x3 = xe - a3 * cos(gamma);
    double y3 = ye - a3 * sin(gamma);

    // Distance from the base to the wrist position
    double C = std::sqrt(x3 * x3 + y3 * y3);

    // Use the law of cosines to find theta2
    double cos_theta2 = (C * C - a1 * a1 - a2 * a2) / (2 * a1 * a2);
    cos_theta2 = std::min(1.0, std::max(-1.0, cos_theta2));
    joint_angles(1) = acos(cos_theta2);  // Theta2

    // Solve for theta1 using trigonometry
    double theta1_part1 = atan2(y3, x3);
    double theta1_part2 = atan2(a2 * sin(joint_angles(1)), a1 + a2 * cos(joint_angles(1)));
    joint_angles(0) = theta1_part1 - theta1_part2;  // Theta1

    // Solve for theta3
    joint_angles(2) = gamma - joint_angles(0) - joint_angles(1);  // Theta3

    std::cout << "IK Joint angles: " << "Joint 1: " << joint_angles(0) << " Joint 2: " << joint_angles(1) << " Joint 3: " << joint_angles(2) << std::endl;

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