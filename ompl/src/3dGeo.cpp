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
    T0_S2,
    T0_S3,
    T0_S4,
    T0_S5,
    T0_S6,
    accept_all
};

enum class Locations {
    init,
    s1,
    s2,
    healthy,
    goal
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
        int shark = 0;
        int healthy = 1;
};

std::vector<SphereObstacle> obstacles = {
    {0.0, 0.0, 0.0, 0.2}, // Example obstacle at origin with radius 0.2
    {0.5, 0.5, 0.5, 0.1},  // Another obstacle
    {1.0, 1.0, 0.0, 5.0},  // Another obstacle
    {5.0, 5.0, 6.0, 3.0},  // Another obstacle
    {-5.0, -6.0, -4.0, 3.0}  // Another obstacle


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

// F(s1)&F(s2)&(shark->(~s2 U healthy))
AutomatonState getNextState(AutomatonState currentState, bool s1, bool s2, bool shark, bool healthy) {
    switch (currentState) {
        case AutomatonState::T0_init:
            if (((s1 && s2 && !shark) || (healthy && s1 && s2))) 
                return AutomatonState::accept_all;
            else if (((!s1 && s2 && !shark) || (healthy && !s1 && s2)))
                return AutomatonState::T0_S2;
            else if (((s1 && !s2 && !shark) || (healthy && s1 && !s2)))
                return AutomatonState::T0_S3;
            else if (((!s1 && !s2 && !shark) || (healthy && !s1 && !s2)))
                return AutomatonState::T0_S4;
            else if ((!healthy && s1 && !s2 && shark))
                return AutomatonState::T0_S5;
            else if ((!healthy && !s1 && !s2 && shark))
                return AutomatonState::T0_S6;
            break;

        case AutomatonState::T0_S2:
            if (s1) 
                return AutomatonState::accept_all;
            else 
                return AutomatonState::T0_S2;

        case AutomatonState::T0_S3:
            if (s2) 
                return AutomatonState::accept_all;
            else 
                return AutomatonState::T0_S3;

        case AutomatonState::T0_S4:
            if (s1 && s2) 
                return AutomatonState::accept_all;
            else if (!s1 && s2) 
                return AutomatonState::T0_S2;
            else if (s1 && !s2) 
                return AutomatonState::T0_S3;
            else 
                return AutomatonState::T0_S4;

        case AutomatonState::T0_S5:
            if (healthy && s2) 
                return AutomatonState::accept_all;
            else if (healthy && !s2) 
                return AutomatonState::T0_S3;
            else 
                return AutomatonState::T0_S5;

        case AutomatonState::T0_S6:
            if (healthy && s1 && s2) 
                return AutomatonState::accept_all;
            else if (healthy && !s1 && s2) 
                return AutomatonState::T0_S2;
            else if (healthy && s1 && !s2) 
                return AutomatonState::T0_S3;
            else if (healthy && !s1 && !s2) 
                return AutomatonState::T0_S4;
            else if (!healthy && s1 && !s2) 
                return AutomatonState::T0_S5;
            else 
                return AutomatonState::T0_S6;

        case AutomatonState::accept_all:
            return AutomatonState::accept_all; // Remain in the accept state.

        default:
            return currentState; // Stay in the same state by default.
    }

    return currentState; // Safety return for unhandled cases.
}


std::string stateToString(AutomatonState state) {
    switch (state) {
        case AutomatonState::T0_init: return "T0_init";
        case AutomatonState::T0_S2: return "T0_S2";
        case AutomatonState::T0_S3: return "T0_S3";
        case AutomatonState::T0_S4: return "T0_S4";
        case AutomatonState::T0_S5: return "T0_S5";
        case AutomatonState::T0_S6: return "T0_S6";
        case AutomatonState::accept_all: return "accept_all";
        default: return "Unknown State";
    }
}

Eigen::VectorXd stateToGoal(Locations locations)
{
    Eigen::VectorXd res(3); // Initialize a 3-element vector
    switch (locations) {
        case Locations::init: 
            res << 3.0, -5.0, 0.0;
            break;
        case Locations::s1: 
            res << 9.0, 9.0, 9.0;
            break;
        case Locations::s2: 
            res << -9.0, -9.0, -9.0;
            break;
        case Locations::healthy: 
            res << -9.0, 0.0, 0.0;
            break;
        case Locations::goal: 
            res << 0.0, -9.0, 0.0;
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


    Locations locations;

    Eigen::VectorXd curr = stateToGoal(Locations::init);

    if(now%3 == 0)
    {
        planWithSimpleSetup(output_file, stateToGoal(Locations::init), stateToGoal(Locations::s1));
        states.s1 = 1;
        curr = stateToGoal(Locations::s1);
        
    } else if (now%3 == 1) {
        planWithSimpleSetup(output_file, stateToGoal(Locations::init), stateToGoal(Locations::s2));
        curr = stateToGoal(Locations::s2);
        states.s2 = 1;
    } else {
        states.healthy = 0;
        states.shark = 1;
        std::cout <<"SHARK ATTACK" << std::endl;
    }

    // std::cout << "Next state: " << stateToString(getNextState(AutomatonState::T0_init, states.s1,states.s2, states.shark, states.healthy)) << std::endl;
    // std::cout << "Next state: " << stateToString(getNextState(AutomatonState::T0_S6, states.s1,states.s2, states.shark, !states.healthy)) << std::endl;
    // std::cout << "Next state: " << stateToString(getNextState(AutomatonState::T0_S4, states.s1,!states.s2, states.shark, !states.healthy)) << std::endl;
    // std::cout << "Next state: " << stateToString(getNextState(AutomatonState::T0_S2, !states.s1,!states.s2, states.shark, !states.healthy)) << std::endl;

    std::string stateString = "init -> ";
    while(currentState != AutomatonState::accept_all)
    {
        now = std::time(nullptr);
        if(now%10 == 1) 
        {
            states.healthy = 0;
            states.shark = 1;
            std::cout <<"SHARK ATTACK" << std::endl;
            stateString += "SHARK ATTACK -> ";
        }
        nextState = getNextState(currentState, states.s1,states.s2, states.shark, states.healthy);
        std::cout << "Next state: " << stateToString(nextState) << std::endl;
        stateString += stateToString(nextState) + " -> ";

        if(!states.healthy)
        {
            planWithSimpleSetup(output_file, curr, stateToGoal(Locations::healthy));
            curr = stateToGoal(Locations::healthy);
            states.healthy = 1;
            states.shark = 0;
            continue;
        }

        if(!states.s2)
        {
            planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s2));
            curr = stateToGoal(Locations::s2);
            states.s2 = 1;
            continue; 
        }

        if(!states.s1)
        {
            planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s1));
            curr = stateToGoal(Locations::s1);
            states.s1 = 1;
            continue; 
        }

        currentState = nextState;
    }

    planWithSimpleSetup(output_file, curr, stateToGoal(Locations::goal));


    std::cout << std::endl << std::endl;

    std::cout << stateString << std::endl;
    return 0;
}