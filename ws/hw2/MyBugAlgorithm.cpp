#include "MyBugAlgorithm.h"

auto DistanceToGoal(const Eigen::Vector2d q, const amp::Problem2D& problem)
{
    return (q - problem.q_goal).norm();
}

// auto isObstacle(const Eigen::Vector2d q, const amp::Problem2D& problem)
// {
//     amp::Polygon poly;
//     for (const auto& obstacle : problem.obstacles) {
//         if (amp::is_point_inside_polygon(obstacle, q)) {
//             return true;
//         }
//     }
//     return false;
// }

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    // auto isObstacle = [&problem, this](const Eigen::Vector2d& q) {
    //     for (const auto& obstacle : problem.obstacles) {
    //         // Now correctly call the member function
    //         if (this->is_point_inside_polygon(obstacle, q)) {
    //             return true;
    //         }
    //     }
    //     return false;
    // };
    // Eigen::Vector2d() q = problem.q_init;
    auto varz = DistanceToGoal(problem.q_init, problem);
    std::cout<< " distance " << varz << std::endl;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(Eigen::Vector2d(2.0, 5.0));
    path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(problem.q_goal);
    
    return path;
}
