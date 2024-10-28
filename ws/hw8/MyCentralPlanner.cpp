#include "MyMultiAgentPlanners.h"
#include "MySamplingBasedPlanners.h"


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

// Eigen::Vector2d computeEdgeNormal(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
//     Eigen::Vector2d edge_direction = end - start;
//     edge_direction.normalize();
//     // Outward normal is perpendicular (rotate 90 degrees counterclockwise)
//     return Eigen::Vector2d(-edge_direction.y(), edge_direction.x());
// }

// Function to expand polygon edges and compute new vertices
std::vector<amp::Polygon> expandObstacles(const amp::MultiAgentProblem2D& problem, double radius) {
    std::vector<amp::Polygon> expanded_obstacles;
    int ct = 0;
    for (const auto& obstacle : problem.obstacles) {
        std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        std::vector<Eigen::Vector2d> expanded_vertices;
        // if(ct>=1) break;
        ct++;
        // Expand each edge by moving it outwards by the robot's radius
        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d current_vertex = vertices[i];
            Eigen::Vector2d next_vertex = vertices[(i + 1) % vertices.size()];

            // Compute edge direction
            Eigen::Vector2d edge = next_vertex - current_vertex;
            Eigen::Vector2d edge_normal = Eigen::Vector2d(edge.y(), -edge.x()).normalized();  // Perpendicular to the edge
            // std::cout << "Edge: " << edge.transpose() << " Edge Normal: " << edge_normal.transpose() << " Between points " << current_vertex.transpose() << " and " << next_vertex.transpose() << std::endl;
            // Move both vertices outwards by the radius
            Eigen::Vector2d expanded_current = current_vertex + radius * edge_normal;
            Eigen::Vector2d expanded_next = next_vertex + radius * edge_normal;

            // Add expanded vertices
            expanded_vertices.push_back(expanded_current);
            expanded_vertices.push_back(expanded_next);
            // std::cout << "curr " << expanded_current.transpose() << " next " << expanded_next.transpose() << std::endl;

            Eigen::Vector2d prev_vertex = vertices[(i + vertices.size() - 1) % vertices.size()];
            Eigen::Vector2d prev_edge = current_vertex - prev_vertex;
            Eigen::Vector2d prev_edge_normal = Eigen::Vector2d(prev_edge.y(), -prev_edge.x()).normalized();

            // Calculate the bisector direction
            Eigen::Vector2d bisector = (prev_edge_normal + edge_normal).normalized();

            // Create a new point one radius away from the current vertex along the bisector
            Eigen::Vector2d new_corner_point = current_vertex + radius * bisector;
            expanded_vertices.push_back(new_corner_point);  // Add the new point

            // Optionally add circular arc for smooth transition between edges
            // Find the outward normals at both vertices and interpolate between them
            Eigen::Vector2d next_edge = vertices[(i + 2) % vertices.size()] - next_vertex;
            Eigen::Vector2d next_edge_normal = Eigen::Vector2d(-next_edge.y(), next_edge.x()).normalized();
        }

        // Store the expanded polygon
        amp::Polygon expanded_obstacle;
        expanded_obstacle.verticesCCW() = sortPts(expanded_vertices);  // Ensure vertices remain counterclockwise
        expanded_obstacles.push_back(expanded_obstacle);

        // Debug output for the expanded vertices
        // for (const auto& vertex : expanded_obstacle.verticesCCW()) {
        //     std::cout << "Vertex: " << vertex.transpose() << std::endl;
        // }
        // std::cout << "Next polygon" << std::endl;
    }

    return expanded_obstacles;
}


/*
Need to check that each robot is not in collision with obstacle
Then for every pair of robots check if they're in collsion
*/
amp::MultiAgentPath2D MyCentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    amp::MultiAgentProblem2D newProblem = problem;
    newProblem.obstacles = expandObstacles(newProblem, 0.6);
    // for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
    //     MyRRT rrt;
    //     amp::Path2D agent_path;
    //     amp::Problem2D prob;
    //     prob.obstacles = newProblem.obstacles;
    //     prob.x_max = problem.x_max;
    //     prob.x_min = problem.x_min;
    //     prob.y_max = problem.y_max;
    //     prob.y_min = problem.y_min;
    //     prob.q_init = agent.q_init;
    //     prob.q_goal = agent.q_goal;

    //     agent_path = rrt.plan(prob);

    //     path.agent_paths.push_back(agent_path);
    // }
    MyRRT rrt;
    path = rrt.planHigherD(newProblem);
    nodeCount = rrt.nodeCount;
    std::cout << "nodeCount outer " << nodeCount << std::endl;
    // amp::Path2D agent_path;
    // amp::Problem2D prob;
    // prob.obstacles = problem.obstacles;
    // prob.x_max = problem.x_max;
    // prob.x_min = problem.x_min;
    // prob.y_max = problem.y_max;
    // prob.y_min = problem.y_min;
    // prob.q_init = problem.agent_properties[0].q_init;
    // prob.q_goal = problem.agent_properties[0].q_goal;

    // agent_path = rrt.plan(prob);

    // path.agent_paths.push_back(agent_path);

    return path;
}

/*
New function for RRT while checking against other paths
As paths are made, add to map<double, Eigen::Vector2d>
Standardize time from 1->n iterations, one step to one node is 1 time step, everything after is at goal
Check obstacles, if in collision at that time step with any other path?
    - this doesn't seem right, do I need to parameterize wrt distance?
*/

amp::MultiAgentPath2D MyDecentralPlanner::plan(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    amp::MultiAgentProblem2D newProblem = problem;
    newProblem.obstacles = expandObstacles(newProblem, 0.53);
    // for (const amp::CircularAgentProperties& agent : problem.agent_properties) {
    //     amp::Path2D agent_path;
    //     agent_path.waypoints = {agent.q_init, agent.q_goal};
    //     path.agent_paths.push_back(agent_path);
    // }

    // for(int i = 0; i < problem.agent_properties.size(); i++)
    // {
    //     std::vector<Eigen::Vector2d> expanded_vertices;

    //     Eigen::Vector2d goal = problem.agent_properties[i].q_goal;

    //     double halfLength = 0.51 / 2.0;

    //     // Calculate the vertices based on the center point
    //     expanded_vertices.push_back(goal + Eigen::Vector2d(-halfLength, -halfLength)); // Bottom-left
    //     expanded_vertices.push_back(goal + Eigen::Vector2d( halfLength, -halfLength)); // Bottom-right
    //     expanded_vertices.push_back(goal + Eigen::Vector2d( halfLength,  halfLength)); // Top-right
    //     expanded_vertices.push_back(goal + Eigen::Vector2d(-halfLength,  halfLength)); // Top-left
    //     amp::Polygon expanded_obstacle;
    //     expanded_obstacle.verticesCCW() = sortPts(expanded_vertices);  // Ensure vertices remain counterclockwise
    //     newProblem.obstacles.push_back(expanded_obstacle);
    // }

    // for(int i = 0; i < problem.agent_properties.size(); i++)
    // {
    //     std::vector<Eigen::Vector2d> expanded_vertices;

    //     Eigen::Vector2d goal = problem.agent_properties[i].q_init;

    //     double halfLength = 0.6 / 2.0;

    //     // Calculate the vertices based on the center point
    //     expanded_vertices.push_back(goal + Eigen::Vector2d(-halfLength, -halfLength)); // Bottom-left
    //     expanded_vertices.push_back(goal + Eigen::Vector2d( halfLength, -halfLength)); // Bottom-right
    //     expanded_vertices.push_back(goal + Eigen::Vector2d( halfLength,  halfLength)); // Top-right
    //     expanded_vertices.push_back(goal + Eigen::Vector2d(-halfLength,  halfLength)); // Top-left
    //     amp::Polygon expanded_obstacle;
    //     expanded_obstacle.verticesCCW() = sortPts(expanded_vertices);  // Ensure vertices remain counterclockwise
    //     newProblem.obstacles.push_back(expanded_obstacle);
    // }
    MyRRT rrt;
    for(int i = 0; i < problem.agent_properties.size(); i++)
    {
        amp::Path2D agent_path;
        amp::Problem2D prob;
        prob.obstacles = newProblem.obstacles;
        prob.x_max = problem.x_max;
        prob.x_min = problem.x_min;
        prob.y_max = problem.y_max;
        prob.y_min = problem.y_min;
        prob.q_init = problem.agent_properties[i].q_init;
        prob.q_goal = problem.agent_properties[i].q_goal;
        agent_path = rrt.plan(prob);
        path.agent_paths.push_back(agent_path);
        std::cout << "node coumt " << rrt.nodeCount << std::endl;
    }
    nodeCount = rrt.nodeCount;
    // std::cout << "node count 2: " << nodeCount << std::endl;

    // amp::Path2D agent_path;
    // amp::Problem2D prob;

    // prob.obstacles = newProblem.obstacles;
    // prob.x_max = problem.x_max;
    // prob.x_min = problem.x_min;
    // prob.y_max = problem.y_max;
    // prob.y_min = problem.y_min;
    // prob.q_init = problem.agent_properties[0].q_init;
    // prob.q_goal = problem.agent_properties[0].q_goal;
    // agent_path = rrt.plan(prob);
    // path.agent_paths.push_back(agent_path);
    //     prob.q_init = problem.agent_properties[1].q_init;
    // prob.q_goal = problem.agent_properties[1].q_goal;
    // agent_path = rrt.plan(prob);
    // path.agent_paths.push_back(agent_path);
    // prob.q_init = problem.agent_properties[2].q_init;
    // prob.q_goal = problem.agent_properties[2].q_goal;
    // agent_path = rrt.plan(prob);
    // path.agent_paths.push_back(agent_path);
    // std::cout << "num agents " << problem.agent_properties.size() << std::endl;


    return path;
}

/*
Decentralized
std::vector<map<time step (node #), Eigen::Vector2d point>>

*/