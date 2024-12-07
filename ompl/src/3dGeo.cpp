 /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
  
 /* Author: Ioan Sucan */
  
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip> // For formatting numbers
#include <cmath>   // For sqrt and pow
#include <ctime>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

// #include "PostProcessing.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

enum class AutomatonState {
    T0_init,
    T0_S1,
    T0_S3,
    accept_all
};

struct SphereObstacle {
    double x, y, z, radius;
};

struct StationTask {
    public:
        std::string name;
        int taskComplete = 0;
        int serviced = 0;
        Eigen::VectorXd goal = Eigen::VectorXd(3);
};

struct TaskStates {
    public:
        int s1 = 0;
        int s2 = 0;
};

std::vector<SphereObstacle> obstacles = {
    {0.0, 0.0, 0.0, 0.2}, // Example obstacle at origin with radius 0.2
    {0.5, 0.5, 0.5, 0.1},  // Another obstacle
    {1.0, 1.0, 0.0, 1.0}  // Another obstacle

};

bool isStateValid(const ob::State *state)
{
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    double x = pos->values[0];
    double y = pos->values[1];
    double z = pos->values[2];

    // Check collision with each obstacle
    for (const auto &obstacle : obstacles) {
        double dist = std::sqrt(std::pow(x - obstacle.x, 2) +
                                std::pow(y - obstacle.y, 2) +
                                std::pow(z - obstacle.z, 2));
        if (dist <= obstacle.radius) {
            return false; // State is in collision
            std::cout << "invalid state found ==========" << std::endl;
        }
    }

    return true; // State is valid
}

  
 void plan3d()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE3StateSpace>());
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-2);
     bounds.setHigh(2);
  
     space->setBounds(bounds);
  
     // construct an instance of  space information from this state space
     auto si(std::make_shared<ob::SpaceInformation>(space));
  
     // set state validity checking for this space
     si->setStateValidityChecker(isStateValid);
  
     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();
  
     // create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  
     // set the start and goal states
     pdef->setStartAndGoalStates(start, goal);
  
     // create a planner for the defined space
     auto planner(std::make_shared<og::RRTConnect>(si));
  
     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);
  
     // perform setup steps for the planner
     planner->setup();
  
  
     // print the settings for this space
     si->printSettings(std::cout);
  
     // print the problem settings
     pdef->print(std::cout);
  
     // attempt to solve the problem within one second of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
  
     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef->getSolutionPath();
         std::cout << "Found solution:" << std::endl;
  
         // print the path to screen
         path->print(std::cout);
     }
     else
         std::cout << "No solution found" << std::endl;
 }

AutomatonState getNextState(AutomatonState currentState, bool s1, bool s2) {
    switch (currentState) {
        case AutomatonState::T0_init:
            if (s1 && s2) return AutomatonState::accept_all;
            if (s1 && !s2) return AutomatonState::T0_S1;
            if (!s1 && !s2) return AutomatonState::T0_init;
            if (!s1 && s2) return AutomatonState::T0_S3;
            break;

        case AutomatonState::T0_S1:
            if (s2) return AutomatonState::accept_all;
            if (!s2) return AutomatonState::T0_S1;
            break;

        case AutomatonState::T0_S3:
            if (s1) return AutomatonState::accept_all;
            if (!s1) return AutomatonState::T0_S3;
            break;

        case AutomatonState::accept_all:
            // Remain in accept state
            return AutomatonState::accept_all;
    }
    return currentState; // Default: remain in the same state
}

std::string stateToString(AutomatonState state) {
    switch (state) {
        case AutomatonState::T0_init: return "T0_init";
        case AutomatonState::T0_S1: return "T0_S1";
        case AutomatonState::T0_S3: return "T0_S3";
        case AutomatonState::accept_all: return "accept_all";
        default: return "Unknown State";
    }
}

Eigen::VectorXd stateToGoal(AutomatonState state)
{
    Eigen::VectorXd res(3); // Initialize a 3-element vector
    switch (state) {
        case AutomatonState::T0_init: 
            res << 3.0, -5.0, 0.0;
            break;
        case AutomatonState::T0_S1: 
            res << 9.0, 9.0, 9.0;
            break;
        case AutomatonState::T0_S3: 
            res << -9.0, -9.0, -9.0;
            break;
        case AutomatonState::accept_all: 
            res << -9.0, 0.0, 0.0;
            break;
        default:
            res.setZero(); // Default to a zero vector for safety
            break;
    }
    return res;
}




void planWithSimpleSetup(const std::string &output_file, Eigen::VectorXd startVec, Eigen::VectorXd goalVec)
{

    std::ofstream out_file(output_file, std::ios::app);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }

    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });



    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);

    for(int i = 0; i < 7; i++)
    {
        if(i == 3) 
        {
            start[i] = 1.0;
            goal[i] = 1.0;
        } else if (i > 3)
        {
            start[i] = 0.0;
            goal[i] = 0.0;
        } else {
            start[i] = startVec[i];
            goal[i] = goalVec[i];
        }
    }

    ss.setStartAndGoalStates(start, goal);
    ss.setup();

    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        ss.simplifySolution();
                 ss.getSolutionPath().print(std::cout);
        const og::PathGeometric &path = ss.getSolutionPath();

        // Get state count and iterate through each state
        size_t num_states = path.getStateCount();
        for (size_t i = 0; i < num_states; ++i)
        {
            const ob::State *state = path.getState(i);
            const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // Extract position
            double x = pos->values[0];
            double y = pos->values[1];
            double z = pos->values[2];

            // Write position
            out_file << std::fixed << std::setprecision(5) << x << " " << y << " " << z << std::endl;

        }
    }
    else
    {
        out_file << "No solution found" << std::endl;
    }

    out_file.close();
}

  
int main(int /*argc*/, char ** /*argv*/)
{
    std::string output_file = "SampleOut.txt";
    if (std::remove(output_file.c_str()) == 0) {
        std::cout << "Deleted existing file: " << output_file << std::endl;
    }

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::time_t now = std::time(nullptr);

    AutomatonState currentState = AutomatonState::T0_init;
    AutomatonState nextState;
    TaskStates states;

    planWithSimpleSetup(output_file, stateToGoal(currentState), stateToGoal(AutomatonState::T0_S1));
    states.s1 = 1;
    currentState = AutomatonState::T0_S1;

    while(currentState != AutomatonState::accept_all)
    {
        now = std::time(nullptr);
        nextState = getNextState(currentState, states.s1, states.s2);
        std::cout << "Next state: " << stateToString(nextState) << std::endl;

        if(nextState != currentState)
        {
            planWithSimpleSetup(output_file, stateToGoal(currentState), stateToGoal(nextState));
        }
        // map string to int
        if(now%3 == 1) 
        {
            states.s2 = 1;
            std::cout << "state 2 success " << std::endl;
        } else {
            std::cout << "RETRYING" << std::endl;
        }
        currentState = nextState;
    }

    std::cout << std::endl << std::endl;

    return 0;
}