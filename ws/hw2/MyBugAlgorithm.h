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
        Eigen::Vector2d closest_point_on_line(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& point) const;
        amp::Obstacle2D get_nearest_obstacle(const Eigen::Vector2d& q, const amp::Problem2D& problem);
        std::vector<Eigen::Vector2d> get_nearest_vertex(const Eigen::Vector2d& q, const amp::Obstacle2D ob, const Eigen::Vector2d& q_goal);
        bool is_point_near_edge(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& point) const;

        Eigen::Vector2d follow_boundary(const Eigen::Vector2d& current_position, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles);


    // You can add other private methods, e.g., boundary following, etc.
        // Eigen::Vector2d follow_boundary(const Eigen::Vector2d& q, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles);


    private:
        // Add any member variables here...
        std::vector<amp::Obstacle2D> foundObstacles;
};