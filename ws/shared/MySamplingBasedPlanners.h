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

    Eigen::Vector2d getRandomConfig2d(const amp::Problem2D& problem);
    Eigen::Vector2d getNearestConfig2d(Eigen::Vector2d temp, std::vector<Eigen::Vector2d> tree);

    // Define a type alias for vector2dCompare to be used as map comparator
    using Vector2dComparator = bool (*)(const Eigen::Vector2d&, const Eigen::Vector2d&);
    static Vector2dComparator vector2dCompare;

    std::vector<std::map<int, Eigen::Vector2d>> timeToPoint;
    std::vector<std::map<Eigen::Vector2d, int, Vector2dComparator>> pointToTime;

    bool checkPaths(Eigen::Vector2d point, int time, double radius);


    std::shared_ptr<amp::Graph<double>> graphPtr = std::make_shared<amp::Graph<double>>();
    std::map<amp::Node, Eigen::VectorXd> nodes;
    int nodeCount = 0;
};

