#include "MyBugAlgorithm.h"
#include <limits>

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d q = problem.q_init;

    // Initial point
    path.waypoints.push_back(q);

    // Helper functions for distance and obstacle detection
    auto distance_to_goal = [this, &problem](const Eigen::Vector2d& q) {
        return (q - problem.q_goal).norm();
    };

    // Check if the point is inside any of the obstacles (using the Polygon class)
    auto obstacle_detected = [&problem, this](const Eigen::Vector2d& q) {
        for (const auto& obstacle : problem.obstacles) {
            // Now correctly call the member function
            if (this->is_point_inside_polygon(obstacle, q)) {
                return true;
            }
        }
        return false;
    };

    // Eigen::Vector2d test(1.5,4);
    // path.waypoints.push_back(test);
    // std::cout << obstacle_detected(test) << std::endl;

    int i = 0;
    Eigen::Vector2d qLi = q;

    while (true) {
        // Step 1: Move from qLi toward q_goal until obstacle encountered or goal reached
        while (!obstacle_detected(q) && distance_to_goal(q) > 0.1) {
            // Simple movement toward goal
            Eigen::Vector2d direction = (problem.q_goal - q).normalized();
            q += direction * 0.1;  // Move in small steps toward the goal
            path.waypoints.push_back(q);
            std::cout << "last points " << q.x() << " " << q.y() << std::endl;

            if (distance_to_goal(q) <= 0.1) {
                // Goal reached
                path.waypoints.push_back(problem.q_goal);
                return path;
            }
            
        }
        path.waypoints.pop_back();
        std::cout << "last points " << path.waypoints.back().x() << " " << path.waypoints.back().y() << std::endl;
        path.waypoints.push_back(problem.q_goal);
        return path;
    }

    //     // Step 2: Follow boundary when obstacle is detected
    //     if (obstacle_detected(q)) {
    //         Eigen::Vector2d qHi = q;
    //         std::cout<<" obstacle " << std::endl;
    //         // Store the point on the boundary with the shortest distance to the goal
    //         Eigen::Vector2d qLi_temp = qLi;
    //         double min_distance_to_goal = distance_to_goal(qLi);
    //         bool qHi_reencountered = false;

    //         // Boundary following
    //         do {
    //             q = follow_boundary(q, qHi, problem.obstacles);  // Example: implement your boundary following here
    //             path.waypoints.push_back(q);

    //             double current_distance = distance_to_goal(q);
    //             if (current_distance < min_distance_to_goal) {
    //                 qLi_temp = q;
    //                 min_distance_to_goal = current_distance;
    //             }

    //             if ((q - qHi).norm() < 0.1) {
    //                 qHi_reencountered = true;
    //             }

    //             if (distance_to_goal(q) <= 0.1) {
    //                 // Goal reached while following the boundary
    //                 return path;
    //             }

    //         } while (!qHi_reencountered);

    //         // Update qLi with the closest point found during boundary following
    //         qLi = qLi_temp;

    //         // Step 3: Go to qLi
    //         q = qLi;
    //         path.waypoints.push_back(qLi);

    //         // Step 4: If moving toward the goal moves into an obstacle, exit with failure
    //         if (obstacle_detected(q)) {
    //             amp::Path2D empty_path;
    //             return empty_path;
    //         }
    //     }

    //     i++;
    // }
    path.waypoints.push_back(problem.q_goal);
    return path;
}

// Helper function for checking if a point is inside a polygon
bool MyBugAlgorithm::is_point_inside_polygon(const amp::Polygon& polygon, const Eigen::Vector2d& point) const {
    const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCCW();
    int num_vertices = vertices.size();
    bool inside = false;

    // Ray-casting algorithm to check if point is inside polygon
    for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
        if ((vertices[i].y() > point.y()) != (vertices[j].y() > point.y()) &&
            (point.x() < (vertices[j].x() - vertices[i].x()) * (point.y() - vertices[i].y()) / 
            (vertices[j].y() - vertices[i].y()) + vertices[i].x())) {
            inside = !inside;
        }
    }

    return inside;
}

Eigen::Vector2d MyBugAlgorithm::follow_boundary(const Eigen::Vector2d& q, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles) {
    // Placeholder for the closest point on the boundary
    Eigen::Vector2d closest_point = qHi;

    // Iterate through all obstacles (though there may be just one)
    for (const auto& obstacle : obstacles) {
        const std::vector<Eigen::Vector2d>& vertices = obstacle.verticesCCW();
        int num_vertices = vertices.size();

        // Find the nearest edge or vertex to the robot's current position q
        double min_distance = std::numeric_limits<double>::max();
        int closest_vertex_index = -1;

        for (int i = 0; i < num_vertices; ++i) {
            double distance = (q - vertices[i]).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_vertex_index = i;
            }
        }

        // Now follow the boundary starting from the closest vertex in CCW direction
        if (closest_vertex_index != -1) {
            int next_vertex_index = (closest_vertex_index + 1) % num_vertices; // next vertex in CCW order

            // Move towards the next vertex
            Eigen::Vector2d direction = (vertices[next_vertex_index] - vertices[closest_vertex_index]).normalized();
            closest_point = vertices[closest_vertex_index] + direction * 0.1;  // Move a small step toward the next vertex
        }
    }

    // Return the next point along the boundary
    return closest_point;
}