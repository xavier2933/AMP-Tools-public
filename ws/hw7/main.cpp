// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"
#include <chrono>

using namespace amp;

int main(int argc, char** argv) {
    // auto start = std::chrono::high_resolution_clock::now();
    HW7::hint(); // Consider implementing an N-dimensional planner 
    MyPRM prm;
    std::list<std::vector<double>> lengths;
    std::list<std::vector<double>> time;
    std::list<std::vector<double>> validPaths;


    std::vector<std::string> labels;
    std::vector<std::string> labelsTimes;

    amp::Path2D temp;

    // Example of creating a graph and adding nodes for visualization
    // prm.graphPtr = std::make_shared<amp::Graph<double>>();
    // std::vector<double> pathLengths;
    // std::vector<double> times;
    // Test PRM on Workspace1 of HW2
    // Problem2D problem = HW2::getWorkspace2();
    int n = 200;
    double r = 2.0;
    // std::vector<std::pair<int, double>> vals = {
    //     {200, 1.0},
    //     {200, 2.0},
    //     {500, 1.0},
    //     {500, 2.0},
    //     {1000, 1.0},
    //     {1000, 2.0}
    // };    
    // // prm.getGraph(problem);

    // for(auto & val : vals)
    // {
    //     n = val.first;
    //     r = val.second;
    //     std::vector<double> pathLengths;
    //     std::vector<double> times;
    //     std::vector<double> validVec;
    //     double invalidPathCount = 0.0;
    //     for(int i = 0; i < 100; i++)
    //     {
    //         auto start = std::chrono::high_resolution_clock::now();

    //         MyPRM tempPRM;
    //         temp = tempPRM.plan2(problem, n,r);
    //         if(tempPRM.pathLength != 0)
    //         {
    //             pathLengths.push_back(tempPRM.pathLength);

    //         }
    //         if(temp.waypoints.empty()) {
    //             invalidPathCount++; 
    //         }
    //         auto stop = std::chrono::high_resolution_clock::now();
    //         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //         times.push_back((double)duration.count());
    //     }
    //     validVec.push_back(100 - invalidPathCount);
    //     lengths.push_back(pathLengths);
    //     time.push_back(times);
    //     validPaths.push_back(validVec);
    //     std::string result = "N = " + std::to_string(n) + ", r = " + std::to_string(r);
    //     labels.push_back(result);
    //     labelsTimes.push_back(result);
    // }


    // Visualizer::makeBoxPlot(lengths, labels, "PATH LENGTHS", "Variables", "Lengths");
    // Visualizer::makeBoxPlot(time, labelsTimes, "TIMES", "Variables", "Time (us)");
    // Visualizer::makeBoxPlot(validPaths, labels, "VALID PATHS", "Variables", "Number of paths");

    // Visualizer::makeFigure(problem, prm.plan2(problem,n,r), *prm.graphPtr, prm.nodes);

    // Generate a random problem and test RRT

    MyRRT rrt;
    Path2D path;
    // prm.getGraph(problem);
    std::vector<Problem2D> problems;
        problems.push_back(HW2::getWorkspace1());

    problems.push_back(HW2::getWorkspace2());
        problems.push_back(HW5::getWorkspace1());


    std::vector<double> pathLengths;
    std::vector<double> times;
    std::vector<double> validVec;
    // for(auto & problem: problems)
    // {
        Problem2D problem = HW2::getWorkspace2();

        double invalidPathCount = 0.0;
        for(int i = 0; i < 100; i++)
        {
            auto start = std::chrono::high_resolution_clock::now();

            MyRRT tempRRT;
            path = tempRRT.plan(problem);
            if(path.length() != 0)
            {
                pathLengths.push_back(path.length());

            }
            if(path.waypoints.empty()) {
                invalidPathCount++; 
            }
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            times.push_back((double)duration.count());
        }
        validVec.push_back(100 - invalidPathCount);
        lengths.push_back(pathLengths);
        time.push_back(times);
        validPaths.push_back(validVec);
        std::string result = "N = " + std::to_string(n) + ", r = " + std::to_string(r);
        // labels.push_back(result);
        // labelsTimes.push_back(result);
 //   }

    // labels.push_back("HW2 WS1");
    labels.push_back("HW2 WS2");

    // labels.push_back("HW5 WS1");


    Visualizer::makeBoxPlot(lengths, labels, "PATH LENGTHS", "Exercise", "Lengths");
    Visualizer::makeBoxPlot(time, labels, "TIMES", "Exercise", "Time (us)");
    Visualizer::makeBoxPlot(validPaths, labels, "VALID PATHS", "Exercise", "Number of paths");

    // HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(HW5::getWorkspace1(), rrt.plan(HW5::getWorkspace1()), *rrt.graphPtr, prm.nodes);
    Visualizer::showFigures();

    // Grade method
    // HW7::grade<MyPRM, MyRRT>("xavier.okeefe@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
//     auto stop = std::chrono::high_resolution_clock::now();
// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
// std::cout << "N = 500, r = 2.0 " << std::endl;
//     std::cout << "Total compute time: " << duration.count() << std::endl;
//     std::cout << "Total path length: " << prm.pathLength << std::endl;
    return 0;
}