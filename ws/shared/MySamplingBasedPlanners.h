#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW7.h"

class MyPRM : public amp::PRM2D {
public:
    virtual amp::Path2D plan(const amp::Problem2D& problem);
};

class MyRRT : public amp::GoalBiasRRT2D {
public:
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;
    virtual amp::MultiAgentPath2D planHigherD(const amp::MultiAgentProblem2D& problem);

    Eigen::VectorXd getRandomConfig(const amp::MultiAgentProblem2D& problem);
    Eigen::VectorXd getNearestConfig(const Eigen::VectorXd& temp, const std::vector<Eigen::VectorXd>& tree);

    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;
};
