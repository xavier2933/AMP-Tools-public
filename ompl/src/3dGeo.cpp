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

//  #include "PostProcessing.h"
  
 #include <ompl/config.h>
 #include <iostream>
  
 namespace ob = ompl::base;
 namespace og = ompl::geometric;
  
 bool isStateValid(const ob::State *state)
 {
     // cast the abstract state type to the type we expect
     const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
  
     // extract the first component of the state and cast it to what we expect
     const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
  
     // extract the second component of the state and cast it to what we expect
     const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
  
     // check validity of state defined by pos & rot
  
  
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return (const void*)rot != (const void*)pos;
 }
  
 void plan3d()
 {
     // construct the state space we are planning in
     auto space(std::make_shared<ob::SE3StateSpace>());
  
     // set the bounds for the R^3 part of SE(3)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(-1);
     bounds.setHigh(1);
  
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


void planWithSimpleSetup(const std::string &output_file)
{
    std::ofstream out_file(output_file, std::ios::app);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }

    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    ob::ScopedState<> start(space);
    start.random();

    ob::ScopedState<> goal(space);
    goal.random();

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
     std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  
    //  plan3d();
  
     std::cout << std::endl << std::endl;
  
     planWithSimpleSetup("SampleOut.txt");
  
     return 0;
 }