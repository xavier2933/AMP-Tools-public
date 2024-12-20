#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        virtual amp::Path2D plan2(const amp::Problem2D& problem, int n, double r);
        void getGraph(const amp::Problem2D& problem, int n, double r);
        Eigen::Vector2d getRandomConfig(const amp::Problem2D& problem);

        amp::Path2D smoothPath(amp::Path2D& original_path, const amp::Problem2D& problem);

        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, Eigen::Vector2d> nodes;
        double pathLength = 0.0;


};

class MyRRT : public amp::GoalBiasRRT2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override; 
        Eigen::Vector2d getRandomConfig(const amp::Problem2D& problem);
        Eigen::Vector2d getNearestConfig(Eigen::Vector2d temp, std::vector<Eigen::Vector2d>);


        std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
        std::map<amp::Node, Eigen::Vector2d> nodes;
};
