#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW9.h"

class MyKinoRRT : public amp::KinodynamicRRT {
    public:

        MyKinoRRT(int samplesToTake) : samplesToTake(samplesToTake) {}

        virtual amp::KinoPath plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) override;
        Eigen::VectorXd getRandomConfig(const amp::KinodynamicProblem2D& problem);
        Eigen::VectorXd getNearestConfig(const Eigen::VectorXd& temp, const std::vector<Eigen::VectorXd>& tree);
        bool subpathCollsionFree(Eigen::VectorXd rand, Eigen::VectorXd near, const amp::KinodynamicProblem2D& problem, double step);
        bool checkCarPath(const amp::KinodynamicProblem2D& problem,std::vector<Eigen::Vector2d> prev, std::vector<Eigen::Vector2d> curr);

    private:
        int samplesToTake;

};  

class MySingleIntegrator : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MyFirstOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySecondOrderUnicycle : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};

class MySimpleCar : public amp::DynamicAgent {
    public:
        virtual void propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) override;
};