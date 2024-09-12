#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        virtual amp::Path2D planBug1(const amp::Problem2D& problem, amp::Path2D& path);

        virtual amp::Path2D planBug2(const amp::Problem2D& problem, amp::Path2D& path);


        

        // Add any other methods here...
        // bool is_point_inside_polygon(const amp::Problem2D& problem, Eigen::Vector2d q);
        bool is_point_inside_polygon(const amp::Polygon& polygon, const Eigen::Vector2d& point) const;
        Eigen::Vector2d closest_point_on_line(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& point) const;



    // You can add other private methods, e.g., boundary following, etc.
    private:
        // Add any member variables here...
        std::vector<amp::Obstacle2D> foundObstacles;
};