#include <iostream>
#include <vector>
#include <Eigen/Dense>

// Function to calculate the Minkowski sum of two sets of points
std::vector<Eigen::Vector2d> minkowskiSum(const std::vector<Eigen::Vector2d>& obstacle, const std::vector<Eigen::Vector2d>& robot) {
    std::vector<Eigen::Vector2d> cSpaceVertices;
    for (const auto& o : obstacle) {
        for (const auto& r : robot) {
            cSpaceVertices.push_back(o + r);
        }
    }
    return cSpaceVertices;
}

void printVertices(const std::vector<Eigen::Vector2d>& vertices) {
    for (const auto& v : vertices) {
        std::cout << "(" << v.x() << ", " << v.y() << ")\n";
    }
}

int main() {
    // Vertices of the obstacle (triangle)
    std::vector<Eigen::Vector2d> obstacle = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 2),
        Eigen::Vector2d(0, 2)
    };

    // Vertices of the robot (same shape as the obstacle)
    std::vector<Eigen::Vector2d> robot = {
        Eigen::Vector2d(0, 0),  // Lower-left vertex as the local reference point
        Eigen::Vector2d(1, 2),
        Eigen::Vector2d(0, 2)
    };

    // Compute the Minkowski sum (C-space vertices)
    std::vector<Eigen::Vector2d> cSpaceVertices = minkowskiSum(obstacle, robot);

    // Print the C-space vertices
    std::cout << "C-space obstacle vertices:\n";
    printVertices(cSpaceVertices);

    return 0;
}