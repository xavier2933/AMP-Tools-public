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


double computeAngle(const Eigen::Vector2d& pivot, const Eigen::Vector2d& point) {
    Eigen::Vector2d vec = point - pivot;
    return std::atan2(vec.y(), vec.x());
}

std::vector<Eigen::Vector2d> sortPts(std::vector<Eigen::Vector2d> points) {

    // Step 1: Find the point with the smallest x and y values (the pivot)
    Eigen::Vector2d pivot = *std::min_element(points.begin(), points.end(),
                                              [](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
                                                  if (p1.x() == p2.x()) return p1.y() < p2.y();
                                                  return p1.x() < p2.x();
                                              });

    // Step 2: Sort the remaining points counterclockwise based on their angle with the pivot
    std::sort(points.begin(), points.end(),
              [pivot](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
                  // Compute angles for each point relative to the pivot
                  double angle1 = computeAngle(pivot, p1);
                  double angle2 = computeAngle(pivot, p2);
                  return angle1 < angle2;
              });

    // Output the sorted points
    // std::cout << "Sorted points counterclockwise:\n";
    for (const auto& point : points) {
        // std::cout << "(" << point.x() << ", " << point.y() << ")\n";
    }

    return points;
}

double calculateAngle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    Eigen::Vector2d vector_ = p2-p1;
    double angle = std::atan2(vector_(1), vector_(0));
    if(angle<0) angle+=2*M_PI;
    return angle;
}

std::vector<Eigen::Vector2d> minkowskiSum(const std::vector<Eigen::Vector2d>& V, const std::vector<Eigen::Vector2d>& W) {
    int i = 0, j = 0;
    std::vector<Eigen::Vector2d> result;
    Eigen::Vector2d base(0,0);
    // V.push_back(base); 

    // Add the first vertex (V1 + W1)
    result.push_back(V[i] + W[j]);

    // Process vertices using the given algorithm
    while (i < V.size() && j < W.size()+ 2) {
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

Eigen::Vector2d rotatePoint(const Eigen::Vector2d& point, double theta) {
    Eigen::Matrix2d rotationMatrix;
    rotationMatrix << std::cos(theta), -std::sin(theta),
                      std::sin(theta),  std::cos(theta);

    return rotationMatrix * point;
}


std::vector<Eigen::Vector2d> triangleRotation(double theta) {
    auto obstacle = HW4::getEx1TriangleObstacle().verticesCCW();
    auto robot = HW4::getEx1TriangleObstacle().verticesCCW();

    for (auto& vertex : robot) {
        vertex = -vertex;  // Negating each vertex
    }

    std::vector<Eigen::Vector2d> points = sortPts(robot);
    // std::vector<Eigen::Vector2d> cspaceObstacle = minkowskiSum(obstacle, points);

    // double theta = M_PI / 4;  // Rotate by 45 degrees (pi/4 radians)

    // Step 1: Rotate each vertex around the origin
    for (auto& point : points) {
        point = rotatePoint(point, theta);
    }

    // Output the rotated triangle vertices
    // // std::cout << "Rotated triangle vertices:\n";
    // for (const auto& point : points) {
    //     // std::cout << "(" << point.x() << ", " << point.y() << ")\n";
    // }
    std::vector<Eigen::Vector2d> points2 = sortPts(points);
    std::vector<Eigen::Vector2d> cspaceObstacle2 = minkowskiSum(obstacle, points2);


    // // std::cout << "C-space obstacle vertices:\n";
    // for (const auto& vertex : cspaceObstacle2) {
    //     // std::cout << "(" << vertex.x() << ", " << vertex.y() << ")\n";
    // }

    return cspaceObstacle2;
}

std::vector<Eigen::Vector2d> triangle() {
    auto obstacle = HW4::getEx1TriangleObstacle().verticesCCW();
    auto robot = HW4::getEx1TriangleObstacle().verticesCCW();

    for (auto& vertex : robot) {
        vertex = -vertex;  // Negating each vertex
        // std::cout << "robot vertex " << vertex << std::endl;
    }

    std::vector<Eigen::Vector2d> points = sortPts(robot);
    std::vector<Eigen::Vector2d> cspaceObstacle = minkowskiSum(obstacle, points);

    double theta = M_PI / 4;  // Rotate by 45 degrees (pi/4 radians)

    // Step 1: Rotate each vertex around the origin
    for (auto& point : points) {
        point = rotatePoint(point, theta);
    }

    // Output the rotated triangle vertices
    // std::cout << "Rotated triangle vertices:\n";
    // for (const auto& point : points) {
    //     // std::cout << "(" << point.x() << ", " << point.y() << ")\n";
    // }
    std::vector<Eigen::Vector2d> points2 = sortPts(points);
    std::vector<Eigen::Vector2d> cspaceObstacle2 = minkowskiSum(obstacle, points2);


    std::cout << "C-space obstacle vertices:\n";
    for (const auto& vertex : cspaceObstacle) {
        std::cout << "(" << vertex.x() << ", " << vertex.y() << ")\n";
    }

    return cspaceObstacle;
}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator;
    std::vector<double> jointLengths2 = {1.0,0.5,1.0};
    // MyManipulator2D IKManipulator(jointLengths2);

    std::vector<Eigen::Vector2d> res = triangle(); // create 1a answer
    Polygon poly = res;
    
    std::vector<Eigen::Vector2d> tempPolygon;
    std::vector<Polygon> polygons;
    std::vector<double> thetas;

    for(double i = 0; i < 2*M_PI; i+=(M_PI/25))
    {
        tempPolygon = triangleRotation(i);
        Polygon caster = tempPolygon;
        polygons.push_back(caster);
        thetas.push_back(i);
    }

    // Visualizer::makeFigure({poly});
    // Visualizer::makeFigure({poly2});
    // Visualizer::makeFigure(polygons, thetas);

    amp::ManipulatorState test_state(3);
    test_state << M_PI / 6, M_PI / 3, 7 * M_PI / 4;

    // can't figure out how to plot both arms at the same time, hardcoded values into Forward kinematics algorithm to get plot on submission
    std::cout << "Forward Kinematics, Point 4 is end effector " << std::endl;
    // Visualizer::makeFigure(manipulator, test_state);

    Eigen::Vector2d tempVecIK(2,0); 

    amp::ManipulatorState IkManip = manipulator.getConfigurationFromIK(tempVecIK);

    // Visualizer::makeFigure(manipulator, IkManip);

    // Create the collision space constructor
    std::size_t n_cells = 500;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // Create the collision space using a given manipulator and environment

    std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator, HW4::getEx3Workspace1());
    std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace2());
    std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor.construct(manipulator, HW4::getEx3Workspace3());

    // use below to plot workspace

    // std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.constructWorkspace(manipulator, HW4::getEx3Workspace1());
    // std::unique_ptr<amp::GridCSpace2D> cspace2 = cspace_constructor.constructWorkspace(manipulator, HW4::getEx3Workspace2());
    // std::unique_ptr<amp::GridCSpace2D> cspace3 = cspace_constructor.constructWorkspace(manipulator, HW4::getEx3Workspace3());


    // You can visualize your cspace 
    // Visualizer::makeFigure(*cspace);
    // Visualizer::makeFigure(*cspace2);
    // Visualizer::makeFigure(*cspace3);


    Visualizer::showFigures();

    // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "xavier.okeefe@colorado.edu", argc, argv);
    return 0;
}