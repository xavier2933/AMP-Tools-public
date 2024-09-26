// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;


void getPolygonRotation(double theta)
{
    // take in theta
    // rotate triangle about vertex?
    // minkowski sum
    // add to c space with given theta parameter as "z" axis
    return;
}

double calculateAngle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    return atan2(p2.y() - p1.y(), p2.x() - p1.x());
}

std::vector<Eigen::Vector2d> minkowskiSum(const std::vector<Eigen::Vector2d>& V, const std::vector<Eigen::Vector2d>& W) {
    int i = 0, j = 0;
    std::vector<Eigen::Vector2d> result;

    // Add the first vertex (V1 + W1)
    // result.push_back(V[i] + W[j]);

    // Process vertices using the given algorithm
    while (i != V.size() + 1 && j != W.size()+ 1) {
        double angleV = calculateAngle(V[i], V[(i + 1) % V.size()]);
        double angleW = calculateAngle(W[j], W[(j + 1) % W.size()]);
        result.push_back(V[i% V.size()] + W[j% W.size()]);


        if (angleV < angleW) {
            i++;
        } else if (angleV > angleW) {
            j++;
        } else {
            i++;
            j++;
        }

    }

    return result;
}


void triangle() {
    auto obstacle = HW4::getEx1TriangleObstacle().verticesCCW();
    auto robot = HW4::getEx1TriangleObstacle().verticesCCW();

    for (auto& vertex : robot) {
        vertex = -vertex;  // Negating each vertex
    }
    std::vector<Eigen::Vector2d> cspaceObstacle = minkowskiSum(obstacle, robot);

    std::cout << "C-space obstacle vertices:\n";
    for (const auto& vertex : cspaceObstacle) {
        std::cout << "(" << vertex.x() << ", " << vertex.y() << ")\n";
    }
    return;
}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator;

    triangle();
    // Eigen::Vector2d test(2,0);

    // You can visualize your manipulator given an angle state like so:
    // amp::ManipulatorState test_state = manipulator.getConfigurationFromIK(test);

    amp::ManipulatorState test_state(3);
    test_state << M_PI / 6, M_PI / 3, 7 * M_PI / 4;
    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, test_state); 

    

    // Create the collision space constructor
    std::size_t n_cells = 500;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment
    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // You can visualize your cspace 
    Visualizer::makeFigure(*cspace);

    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "xavier.okeefe@colorado.edu", argc, argv);
    return 0;
}