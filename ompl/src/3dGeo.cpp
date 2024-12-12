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
    State0,
    State1,
    State2,
    State3,
    State4,
    State5,
    State6,
    State7,
    State8,
    State9,
    State10,
    State11
};

enum class Locations {
    init,
    s1,
    s2,
    s3,
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
        int s3 = 0;
        int shark = 0;
        int healthy = 1;
};

std::vector<SphereObstacle> obstacles = {
    {0.0, 0.0, 0.0, 0.25}, // Example obstacle at origin with radius 0.2
    {0.5, 0.5, 0.5, 0.15},  // Another obstacle
    {1.0, 1.0, 0.0, 5.2},  // Another obstacle
    {5.0, 5.0, 6.0, 3.2},  // Another obstacle
    {-5.0, -6.0, -4.0, 3.2}  // Another obstacle


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

// F(s1) & F(s2)&F(s3) & (!s2 U healthy) 
AutomatonState getNextState(AutomatonState currentState, bool s1, bool s2, bool s3, bool healthy) {
    switch (currentState) {
        case AutomatonState::State0:
            if (s1 && s2 && s3 && healthy) return AutomatonState::State1;   // [0&1&2&3] 1
            if (s1 && !s2 && !s3) return AutomatonState::State2;             // [0&!1&!2] 2
            if (!s1 && s2 && !s3 && healthy) return AutomatonState::State3; // [!0&1&!2&3] 3
            if (!s1 && !s2 && s3) return AutomatonState::State4;             // [!0&!1&2] 4
            if (!s1 && s2 && s3 && healthy) return AutomatonState::State5;   // [!0&1&2&3] 5
            if (s1 && !s2 && s3) return AutomatonState::State6;              // [0&!1&2] 6
            if (s1 && s2 && !s3 && healthy) return AutomatonState::State7;   // [0&1&!2&3] 7
            return AutomatonState::State0;                                   // [!0&!1&!2] 0

        case AutomatonState::State1:
            if (healthy) return AutomatonState::State1;                      // [3] 1 {0}
            if (!s2 && !healthy) return AutomatonState::State8;              // [!1&!3] 8
            return AutomatonState::State1;

        case AutomatonState::State2:
            if (s1 && s2 && s3 && healthy) return AutomatonState::State1;   // [1&2&3] 1
            if (!s1 && !s2) return AutomatonState::State2;                   // [!1&!2] 2
            if (!s1 && s2) return AutomatonState::State6;                    // [!1&2] 6
            if (s1 && !s2 && s3 && healthy) return AutomatonState::State7;   // [1&!2&3] 7
            return AutomatonState::State2;

        case AutomatonState::State3:
            if ((s1 && !s2 && s3) || (s1 && s2 && s3)) return AutomatonState::State1;     // [0&!1&2 | 0&2&3] 1
            if ((!s1 && !s2 && !s3) || (!s1 && !s2 && healthy)) return AutomatonState::State3; // [!0&!1&!2 | !0&!2&3] 3
            if ((!s1 && s2 && s3) || (!s1 && s2 && healthy)) return AutomatonState::State5;    // [!0&!1&2 | !0&2&3] 5
            if ((s1 && !s2 && !s3) || (s1 && !s2 && healthy)) return AutomatonState::State7;   // [0&!1&!2 | 0&!2&3] 7
            return AutomatonState::State3;

        case AutomatonState::State4:
            if (s1 && s3 && healthy) return AutomatonState::State1;         // [0&1&3] 1
            if (!s1 && !s2) return AutomatonState::State4;                  // [!0&!1] 4
            if (!s1 && s3 && healthy) return AutomatonState::State5;        // [!0&1&3] 5
            if (s1 && !s2) return AutomatonState::State6;                   // [0&!1] 6
            return AutomatonState::State4;

        case AutomatonState::State5:
            if ((s1 && !s2) || (s1 && healthy)) return AutomatonState::State1;  // [0&!1 | 0&3] 1
            if ((!s1 && !s2) || (!s1 && healthy)) return AutomatonState::State5; // [!0&!1 | !0&3] 5
            return AutomatonState::State5;

        case AutomatonState::State6:
            if (s1 && s3 && healthy) return AutomatonState::State1;         // [1&3] 1
            if (!s1) return AutomatonState::State6;                          // [!1] 6
            return AutomatonState::State6;

        case AutomatonState::State7:
            if ((!s1 && s2) || (s2 && healthy)) return AutomatonState::State1; // [!1&2 | 2&3] 1
            if ((!s1 && !s2) || (!s2 && healthy)) return AutomatonState::State7; // [!1&!2 | !2&3] 7
            return AutomatonState::State7;

        case AutomatonState::State8:
            if (healthy) return AutomatonState::State1;                      // [3] 1
            if (!s2 && !healthy) return AutomatonState::State8;              // [!1&!3] 8
            return AutomatonState::State8;

        default:
            return currentState; // Stay in the same state by default
    }
}


std::string stateToString(AutomatonState state) {
    switch (state) {
        case AutomatonState::State0: return "State0";
        case AutomatonState::State1: return "State1 (Accepting)";
        case AutomatonState::State2: return "State2";
        case AutomatonState::State3: return "State3";
        case AutomatonState::State4: return "State4";
        case AutomatonState::State5: return "State5";
        case AutomatonState::State6: return "State6";
        case AutomatonState::State7: return "State7";
        case AutomatonState::State8: return "State8";
        case AutomatonState::State9: return "State9";
        case AutomatonState::State10: return "State10";
        case AutomatonState::State11: return "State11";

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
        case Locations::s3: 
            res << -9.0, 9.0, -9.0;
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


double planWithSimpleSetup(const std::string &output_file, Eigen::VectorXd startVec, Eigen::VectorXd goalVec)
{

    std::ofstream out_file(output_file, std::ios::app);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return 0.0;
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
    double len = 0.0;
    if (solved)
    {
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
        const og::PathGeometric &path = ss.getSolutionPath();
        len = path.length();
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
    return len;
}

double doEverything()
{
    double len = 0.0;
    std::string output_file = "SampleOut.txt";
    if (std::remove(output_file.c_str()) == 0) {
        std::cout << "Deleted existing file: " << output_file << std::endl;
    }

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::time_t now = std::time(nullptr);

    AutomatonState currentState = AutomatonState::State0; // Updated to start at State3.
    AutomatonState nextState;
    TaskStates states;

    Locations locations;
    int attacks = 0;
    int it = 0;
    Eigen::VectorXd curr = stateToGoal(Locations::init);

    if(now % 4 == 0)
    {
        planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s2));
        curr = stateToGoal(Locations::s2);
        if(it%3 != 1) states.s2 = 1;
        std::cout << "VISITED S2" << std::endl;
        nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
    } else if(now%4 == 1) {
        planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s2));
        curr = stateToGoal(Locations::s3);
        if(it%3 != 1) states.s3 = 1;
        std::cout << "VISITED S3" << std::endl;
        nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);

    } else if(now%4 == 2) {
        planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s1));
        curr = stateToGoal(Locations::s1);
        if(it%3!= 1) states.s1 = 1;
        std::cout << "VISITED S1" << std::endl;
        nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);

    } else {
        states.healthy = 0;
        attacks++;
        states.shark = 1;
        nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);

    }

    std::cout << "Current state: " << stateToString(currentState)
                  << " -> Next state: " << stateToString(nextState) << std::endl;

    std::string visited = "init -> ";
    while (currentState != AutomatonState::State1) { // Run until acceptance condition is met.
        now = std::time(nullptr);
        it++;
            std::cout << "s1: " << states.s1 << ", "
              << "s2: " << states.s2 << ", "
              << "s3: " << states.s3 << ", "
              << "healthy: " << states.healthy << std::endl;
        if (now % 2 == 1 && attacks < 2) {
            states.healthy = 0;
            attacks++;
            states.shark = 1;
            std::cout << "SHARK ATTACK" << std::endl;
        }

        // nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
        // std::cout << "Current state: " << stateToString(currentState)
        //           << " -> Next state: " << stateToString(nextState) << std::endl;
        currentState = nextState;

        if (!states.healthy) {
            len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::healthy));
            curr = stateToGoal(Locations::healthy);
            states.healthy = 1;
            nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
            std::cout << "Current state: " << stateToString(nextState) << std::endl;
            continue;
        }

        if(!states.s1 && !states.s2 && !states.s3)
        {
            if(now%3 == 0)
            {
                len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s2));
                curr = stateToGoal(Locations::s2);
                if(it%3 != 1) states.s2 = 1;
                nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
                std::cout << "VISITED S2 with state: " << std::endl;
                std::cout << "Current state: " << stateToString(nextState) << std::endl;
            } else if(now%3 == 1) {
                len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s2));
                curr = stateToGoal(Locations::s3);
                if(it%3 != 1) states.s3 = 1;
                nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
                std::cout << "VISITED S3" << std::endl;
                std::cout << "Current state: " << stateToString(nextState) << std::endl;
            } else {
                len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s1));
                curr = stateToGoal(Locations::s1);
                if(it%3!= 1) states.s1 = 1;
                std::cout << "VISITED S1" << std::endl;
                nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
                std::cout << "Current state: " << stateToString(nextState) << std::endl;
            }
            continue;
        }

        if (!states.s2) {
            len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s2));
            curr = stateToGoal(Locations::s2);
            states.s2 = 1;
            nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
            std::cout << "VISITED S2" << std::endl;
            std::cout << "Current state: " << stateToString(nextState) << std::endl;
            continue;
        }

        if (!states.s1) {
            len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s1));
            curr = stateToGoal(Locations::s1);
            states.s1 = 1;
            nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
            std::cout << "VISITED S1" << std::endl;
            std::cout << "Current state: " << stateToString(nextState) << std::endl;
            continue;
        }

        if (!states.s3) {
            len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::s1));
            curr = stateToGoal(Locations::s3);
            states.s3 = 1;
            nextState = getNextState(currentState, states.s1, states.s2, states.s3, states.healthy);
            std::cout << "VISITED S3" << std::endl;

            std::cout << "Current state: " << stateToString(nextState) << std::endl;

            continue;
        }

        currentState = nextState;
    }
    len += planWithSimpleSetup(output_file, curr, stateToGoal(Locations::goal));
    return len;
}
  
int main(int /*argc*/, char ** /*argv*/) {
    std::vector<double> lens;
    double len = 0.0;
    double localLen = 0.0;
    double avg = 0.0;
    len = doEverything();
    for(int j = 0; j < 10; j++)
    {
        len = 0;
        for(int i = 0; i < 100; i++)
        {
            localLen = doEverything();
            len += localLen;
            lens.push_back(localLen);
        }
        avg += len/100.0;
    }

    std::ofstream outFile("lens.csv");
    if (outFile.is_open()) {
        outFile << "Index,Value\n"; // Optional: Header row for CSV
        for (size_t i = 0; i < lens.size(); i++) {
            outFile << i << "," << lens[i] << "\n";
        }
        outFile.close();
        std::cout << "Data written to lens.csv successfully." << std::endl;
    } else {
        std::cerr << "Failed to open file for writing." << std::endl;
    }
    std::cout << "len " << avg/10.0 << std::endl;
    return 0;
}

// int main() {
//     AutomatonState currentState = AutomatonState::State3;
//     bool s1 = false, s2 = false, healthy = true; // Initial conditions

//     while (currentState != AutomatonState::State0) { // Loop until acceptance
//         std::cout << "Current state: " << stateToString(currentState) << std::endl;

//         // Simulate environment conditions
//         s1 = (std::rand() % 2) == 0;
//         s2 = (std::rand() % 2) == 0;
//         healthy = (std::rand() % 2) == 0;

//         // Update state
//         AutomatonState nextState = getNextState(currentState, s1, s2, healthy);
//         std::cout << "Conditions: s1=" << s1 << ", s2=" << s2 << ", healthy=" << healthy << std::endl;
//         std::cout << "Next state: " << stateToString(nextState) << std::endl;

//         currentState = nextState;
//     }

//     std::cout << "Reached accepting state: " << stateToString(currentState) << std::endl;

//     return 0;
// }