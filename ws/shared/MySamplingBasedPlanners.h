#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        Eigen::Vector2d getRandomConfig(const amp::Problem2D& problem);
        Eigen::Vector2d getNearestConfig(Eigen::Vector2d temp, std::vector<Eigen::Vector2d>);


        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, Eigen::Vector2d> nodes;
};
