#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        // bool is_point_inside_polygon(const amp::Problem2D& problem, Eigen::Vector2d q);
        bool is_point_inside_polygon(const amp::Polygon& polygon, const Eigen::Vector2d& point) const;

    // You can add other private methods, e.g., boundary following, etc.
        Eigen::Vector2d follow_boundary(const Eigen::Vector2d& q, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles);

    private:
        // Add any member variables here...
};