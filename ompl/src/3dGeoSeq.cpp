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
  
 #include <ompl/base/SpaceInformation.h>
 #include <ompl/base/spaces/SE3StateSpace.h>
 #include <ompl/geometric/planners/rrt/RRTConnect.h>
 #include <ompl/geometric/SimpleSetup.h>

#include <fstream>
#include <vector>
#include <iomanip> // For formatting numbers
#include <ctime>


//  #include "PostProcessing.h"
  
 #include <ompl/config.h>
 #include <iostream>
  
 namespace ob = ompl::base;
 namespace og = ompl::geometric;

#include <cmath> // For sqrt and pow

struct SphereObstacle {
    double x, y, z, radius;
};

struct StationTask {
    public:
        int taskComplete = 0;
        int serviced = 0;
        int index;
        Eigen::VectorXd goal = Eigen::VectorXd(3);
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


double planWithSimpleSetup2(const std::string &output_file, Eigen::VectorXd startVec, Eigen::VectorXd goalVec)
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
        start[i] = startVec[i];
        goal[i] = goalVec[i];
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

int getNearest(std::vector<StationTask> stations, Eigen::VectorXd curr)
{
    double minDist = std::numeric_limits<double>::max();
    int index = -1;

    Eigen::VectorXd currNew(3);
    if (curr.size() >= 3) {
        currNew << curr[0], curr[1], curr[2];
    } else {
        std::cerr << "Error: curr must have at least 3 elements!" << std::endl;
        return index;
    }

    for (int i = 0; i < stations.size(); i++)
    {
        if (stations[i].goal.size() != currNew.size()) {
            std::cerr << "Skipping station " << i 
                      << " due to size mismatch: goal size is " 
                      << stations[i].goal.size() << std::endl;
            continue;
        }

        if ((currNew - stations[i].goal).norm() < minDist &&
            stations[i].serviced != 1 && 
            !currNew.isApprox(stations[i].goal))
        {
            index = i;
            minDist = (currNew - stations[i].goal).norm();
        }
    }
    return index;
}

double doEverythingSeq()
{
    double pathLength = 0;
    std::string output_file = "SampleOut.txt";
    if (std::remove(output_file.c_str()) == 0) {
        std::cout << "Deleted existing file: " << output_file << std::endl;
    }

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    std::time_t now = std::time(nullptr);

    std::vector<StationTask> stations;
    std::vector<StationTask> serviced;

    Eigen::VectorXd startVec(7);
    Eigen::VectorXd goalVec(7);

    StationTask s1;
    s1.goal << 9.0, 9.0, 9.0;
    s1.index = 0;
    stations.push_back(s1);

    StationTask s2;
    s2.goal << -9.0, -9.0, -9.0;
    s2.index = 1;
    stations.push_back(s2);

    StationTask s3;

    s3.goal << -9.0, 9.0, -9.0;
    s3.index = 2;
    stations.push_back(s3);

    std::string visited = "init ";
    Eigen::VectorXd init(7);
    Eigen::VectorXd curr(7);
    Eigen::VectorXd finalGoal(7);

    init << 3.0, -5.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // w, x, y, z
    finalGoal << 0, -9.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // w, x, y, z
    goalVec << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;  // w, x, y, z

    curr = init;

    std::cout << "Closest is " << getNearest(stations, curr) << std::endl;

    // Randomly select a station to start
    int firstIndex = now % stations.size();

    while(serviced.size() != stations.size())
    {
        std::time_t loopTime = std::time(nullptr);
        firstIndex = getNearest(stations, curr);
        if(firstIndex == -1)
        {
            std::cout << "going to goal" << std::endl;
            break;
        }
        if(stations[firstIndex].serviced == 1)
        {
            continue;
        }
        std::cout << "Planning station " << firstIndex << std::endl;
        goalVec[0] = stations[firstIndex].goal[0];
        goalVec[1] = stations[firstIndex].goal[1];
        goalVec[2] = stations[firstIndex].goal[2];

        pathLength += planWithSimpleSetup2(output_file, curr, goalVec);
        visited = visited + " -> " + std::to_string(firstIndex);
        curr[0] = stations[firstIndex].goal[0];
        curr[1] = stations[firstIndex].goal[1];
        curr[2] = stations[firstIndex].goal[2];
        if(loopTime % 3 != 1)
        {
            serviced.push_back(stations[firstIndex]);
            stations[firstIndex].serviced = 1;
            std::cout << "SERVICED " << firstIndex << std::endl;
        } else {
            std::cout << "DID NOT service " << firstIndex << std::endl;
            std::cout << "Current location " << curr.transpose() << std::endl;
        }
    }
    pathLength += planWithSimpleSetup2(output_file, curr, finalGoal);

    std::cout << std::endl << std::endl;
    return pathLength;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::vector<double> lens;
    double len = 0.0;
    double localLen = 0.0;
    double avg = 0.0;
    len = doEverythingSeq();
    for(int j = 0; j < 10; j++)
    {
        len = 0;
        for(int i = 0; i < 100; i++)
        {
            localLen = doEverythingSeq();
            len+=localLen;
            lens.push_back(localLen);
        }
        avg += len/100.0;
    }

    std::ofstream outFile("lensSeq.csv");
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