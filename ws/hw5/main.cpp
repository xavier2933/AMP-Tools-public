// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"

#include "hw/HW2.h"


// Include any custom headers you created in your workspace
#include "MyGDAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Test your gradient descent algorithm on a random problem.
    MyGDAlgorithm algo(1.0, 1.0, 1.0, 1.0);
    amp::Path2D path;
    amp::Problem2D prob;
    amp::Problem2D problem = HW2::getWorkspace2();   
    amp::Path2D pathHW2 = algo.plan(problem);
    // bool success = HW5::generateAndCheck(algo, pathHW2, problem);
    Visualizer::makeFigure(problem, pathHW2);


    // Visualize your potential function
    // amp::Visualizer::makeFigure(MyPotentialFunction{}, prob.x_min, prob.x_max, prob.y_min, prob.y_max, 20);
    Visualizer::showFigures();
    
    // Arguments following argv correspond to the constructor arguments of MyGDAlgorithm:
    HW5::grade<MyGDAlgorithm>("xavier.okeefe@colorado.edu", argc, argv, 1.0, 1.0, 1.0, 1.0);
    return 0;
}