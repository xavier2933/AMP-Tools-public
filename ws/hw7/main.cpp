// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MySamplingBasedPlanners.h"

using namespace amp;

int main(int argc, char** argv) {
    HW7::hint(); // Consider implementing an N-dimensional planner 
    MyPRM prm;

    // Example of creating a graph and adding nodes for visualization
    // prm.graphPtr = std::make_shared<amp::Graph<double>>();

    // Test PRM on Workspace1 of HW2
    Problem2D problem = HW2::getWorkspace2();
    // prm.getGraph(problem);

    Visualizer::makeFigure(problem, prm.plan(problem), *prm.graphPtr, prm.nodes);

    // Generate a random problem and test RRT

    MyRRT rrt;
    Path2D path;
    // prm.getGraph(problem);

    HW7::generateAndCheck(rrt, path, problem);
    Visualizer::makeFigure(problem, path, *rrt.graphPtr, prm.nodes);
    Visualizer::showFigures();

    // Grade method
    // HW7::grade<MyPRM, MyRRT>("xavier.okeefe@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}