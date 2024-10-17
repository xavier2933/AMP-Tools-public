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
    Problem2D problem = HW5::getWorkspace1();
    int n = 200;
    double r = 2;
    std::vector<std::pair<int, double>> vals = {
        {200, 0.5},
        {200, 1.0},
        {200, 1.5},
        {200, 2.0},
        {500, 0.5},
        {500, 1.0},
        {500, 1.5},
        {500, 2.0}
    };    // prm.getGraph(problem);

    for(auto & val : vals)
    {
        n = val.first;
        r = val.second;
        std::vector<double> pathLengths;
        std::vector<double> times;
        std::vector<double> validVec;
        double invalidPathCount = 0.0;
        for(int i = 0; i < 100; i++)
        {
            auto start = std::chrono::high_resolution_clock::now();

            MyPRM tempPRM;
            temp = tempPRM.plan2(problem, n,r);
            pathLengths.push_back(tempPRM.pathLength);
            if(temp.waypoints.empty()) {
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
        labels.push_back(result);
        labelsTimes.push_back(result);
    }


    Visualizer::makeBoxPlot(lengths, labels, "PATH LENGTHS", "Variables", "Lengths");
    Visualizer::makeBoxPlot(time, labelsTimes, "TIMES", "Variables", "Time (us)");
    Visualizer::makeBoxPlot(validPaths, labels, "VALID PATHS", "Variables", "Number of paths");

    // Visualizer::makeFigure(problem, prm.plan(problem), *prm.graphPtr, prm.nodes);

    // Generate a random problem and test RRT

    MyRRT rrt;
    Path2D path;
    // prm.getGraph(problem);

    // HW7::generateAndCheck(rrt, path, problem);
    // Visualizer::makeFigure(problem, path, *rrt.graphPtr, prm.nodes);
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