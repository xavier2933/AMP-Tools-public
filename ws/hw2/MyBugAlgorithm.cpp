#include "MyBugAlgorithm.h"
#include <limits>

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d q = problem.q_init;

    path.waypoints.push_back(q);

    // get distance to goal
    auto distance_to_goal = [this, &problem](const Eigen::Vector2d& q) {
        return (q - problem.q_goal).norm();
    };

    // check obstacles
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
    // amp::Obstacle2D tempOb = get_nearest_obstacle(problem.q_init, problem);
    // std::cout << "init q " << problem.q_init << std::endl;
    // std::cout << "OBSTALCE ";
    // std::cout << tempOb.verticesCCW().front() << std::endl;
    // // Eigen::Vector2d tempVec()
    // Eigen::Vector2d closestVert = get_nearest_vertex(problem.q_init, tempOb);
    // std::cout << "VERT " << closestVert << std::endl;

    int i = 0;
    Eigen::Vector2d qLi = q;
    amp::Obstacle2D temp = get_nearest_obstacle(q,problem);

    while (true) {
        // WORKING - moves bug towards goal
        int outerLoop = 0;
        while (!obstacle_detected(q) && distance_to_goal(q) > 0.1) {
            Eigen::Vector2d direction = (problem.q_goal - q).normalized();
            q += direction * 0.1; 
            path.waypoints.push_back(q);
            std::cout << "last points " << q.x() << " " << q.y() << std::endl;

            if (distance_to_goal(q) <= 0.1) {
                // Goal reached
                path.waypoints.push_back(problem.q_goal);
                return path;
            }
            
        }
    //     path.waypoints.pop_back();
    //     std::cout << "last points " << path.waypoints.back().x() << " " << path.waypoints.back().y() << std::endl;
    //     path.waypoints.push_back(problem.q_goal);
    //     return path;
    // }

        // NOT WORKING - follow boundary
        if (obstacle_detected(q)) {
            Eigen::Vector2d qHi = q; // hit point
            Eigen::Vector2d qLi_temp = qLi;
            double min_distance_to_goal = distance_to_goal(qLi);
            bool qHi_reencountered = false;
            int count = 0;
            do {
                count++;
                if ( count > 20) {
                    std::cout << "returning" << std::endl;
                    path.waypoints.push_back(problem.q_goal);
                    return path;    
                }
                // q = follow_boundary(q, qHi, problem.obstacles);
                amp::Obstacle2D tempOb2 = get_nearest_obstacle(problem.q_init, problem);
                std::vector<Eigen::Vector2d> closestVerts = get_nearest_vertex(q, tempOb2, problem.q_goal);
                for(auto point : closestVerts)
                {
                    path.waypoints.push_back(point);
                    std::cout << "last points " << path.waypoints.back().x() << " " << path.waypoints.back().y() << std::endl;

                }



                // path.waypoints.push_back(q);
                // std::cout << "last points " << path.waypoints.back().x() << " " << path.waypoints.back().y() << std::endl;

                double current_distance = distance_to_goal(q);
                if (current_distance < min_distance_to_goal) {
                    qLi_temp = q;
                    min_distance_to_goal = current_distance;
                }

                if ((q - qHi).norm() < 0.1) {
                    qHi_reencountered = true;
                }

                if (distance_to_goal(q) <= 0.1) {
                    path.waypoints.push_back(problem.q_goal);
                    return path;
                }
                qHi_reencountered = true;

            } while (!qHi_reencountered);

            // qLi = qLi_temp;
            // q = qLi;
            // path.waypoints.push_back(qLi);
            q = path.waypoints.back();


            // if (obstacle_detected(q)) {
            //     amp::Path2D empty_path;
            //     return empty_path;
            // }
        }
        if ((q - problem.q_goal).norm() < 0.2) {
            break;
        }
        i++;
        if(i > 50)
        {
            break;
        }
    }

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

// std::vector<Eigen::Vector2d> traverse_obstacle(amp::Obstacle2D ob, const Eigen::Vector2d& q)
// {
//     std::vector<Eigen::Vector2d> res;
//     int index = 0;
//     int minDist = 1000000;
//     for(i = 0; i < ob.verticesCCW.size(); ++i)
//     {
//         if((ob.verticesCCW[i] - q).norm() < minDist)
//         {
//             index = i;
//         }
//     }

// }

std::vector<Eigen::Vector2d> MyBugAlgorithm::get_nearest_vertex(const Eigen::Vector2d& q, const amp::Obstacle2D ob, const Eigen::Vector2d& q_goal)
{
    Eigen::Vector2d res;
    double minDistance = 100000000;
    double holder;
    std::vector<Eigen::Vector2d> points;
    int i = 0;
    int index = 0;
    Eigen::Vector2d closest;
    double min_dist = 100000000;
    // Find the nearest vertex and store its index
    for(const auto& vertex: ob.verticesCCW())
    {
        holder = (q - vertex).norm();
        if (holder < minDistance)
        {
            index = i;
            minDistance = holder;
            res = vertex;
        }
        i++;
    }

    points.push_back(res);  
    double stepSize = 0.1;  
    
    const auto& vertices = ob.verticesCCW();
    int currentIndex = index;
    do {
        Eigen::Vector2d start = vertices[currentIndex];
        Eigen::Vector2d end = vertices[(currentIndex + 1) % vertices.size()];  // Next vertex (wrap around)

        // Calculate direction and number of steps for the line segment
        Eigen::Vector2d direction = end - start;
        double segmentLength = direction.norm();
        direction.normalize();
        int numSteps = static_cast<int>(segmentLength / stepSize);

        // Add intermediate points along the segment
        for (int j = 1; j <= numSteps; ++j) {
            Eigen::Vector2d intermediatePoint = start + j * stepSize * direction;
            points.push_back(intermediatePoint);  // Add to points
        }

        // Move to the next vertex
        currentIndex = (currentIndex + 1) % vertices.size();
    } while (currentIndex != index);

    for(auto point:points)
    {
        if((point - q_goal).norm() < min_dist)
        {
            closest = point;
            min_dist = (point - q_goal).norm();
        }
    }

    for(int i = 0; i < points.size(); i++)
    {
        if(points[i] != closest)
        {
            points.push_back(points[i]);
        }
        if(points[i] == closest)
        {
            break;
        }
    }

    return points;  // Return the vector of vertices
}

amp::Obstacle2D MyBugAlgorithm::get_nearest_obstacle(const Eigen::Vector2d& q, const amp::Problem2D& problem)
{
    amp::Obstacle2D ob;
    double minDistance = 100000000;
    double holder;

    for(const auto& obstacle:problem.obstacles)
    {
        const auto& vertices = obstacle.verticesCCW();
        for (int i = 0; i < vertices.size(); ++i) {
            holder = (q - vertices[i]).norm();
            if ((q - vertices[i]).norm() < minDistance)
            {
                minDistance = holder;
                ob = obstacle;
            }
        }
    }
    return ob;
}

Eigen::Vector2d MyBugAlgorithm::follow_boundary(const Eigen::Vector2d& q, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles) {
    Eigen::Vector2d next_q = q;  

    for (const auto& obstacle : obstacles) {
        const auto& vertices = obstacle.verticesCCW(); 

        // Find the nearest boundary point to q
        for (size_t i = 0; i < vertices.size(); ++i) {
            Eigen::Vector2d edge_start = vertices[i];
            Eigen::Vector2d edge_end = vertices[(i + 1) % vertices.size()];

            Eigen::Vector2d closest_point = closest_point_on_line_segment(q, edge_start, edge_end);
            
            if ((closest_point - q).norm() < (next_q - q).norm()) {
                next_q = closest_point;
            }
        }
    }

    return next_q;
}

// Helper function to find the closest point on a line segment
Eigen::Vector2d MyBugAlgorithm::closest_point_on_line_segment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    Eigen::Vector2d ab = b - a;
    double t = (p - a).dot(ab) / ab.dot(ab);  // Projection scalar
    t = std::max(0.0, std::min(1.0, t));  // Clamp t to the line segment
    return a + t * ab;
}

// Eigen::Vector2d MyBugAlgorithm::follow_boundary(const Eigen::Vector2d& q, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles) {
//     // Assume the robot is already at the obstacle boundary (q) and we're now following the boundary
//     Eigen::Vector2d next_point = q;



//     double min_distance = std::numeric_limits<double>::max();
//     static std::vector<Eigen::Vector2d> visited_points;
    
//     // Variable to track if we have returned to qHi
//     bool qHi_reencountered = false;

//     for (const auto& obstacle : obstacles) {
//         const auto& vertices = obstacle.verticesCCW();
//         int num_vertices = vertices.size();

//         // Iterate through each edge of the obstacle
//         for (int i = 0; i < num_vertices; ++i) {
//             Eigen::Vector2d p1 = vertices[i];
//             Eigen::Vector2d p2 = vertices[(i + 1) % num_vertices]; // Wrap around to the first vertex

//             // Follow along the edge from p1 to p2 in small steps
//             for (double t = 0.0; t <= 1.0; t += 0.1) {  // Move in small steps along the edge
//                 Eigen::Vector2d candidate_point = p1 + t * (p2 - p1);

//                 // Avoid revisiting the same point
//                 bool already_visited = std::any_of(visited_points.begin(), visited_points.end(), [&](const Eigen::Vector2d& point) {
//                     return (point - candidate_point).norm() < 0.01;  // Small tolerance to avoid floating-point issues
//                 });

//                 if (!already_visited && (candidate_point - q).norm() > 0.01) {
//                     double distance_to_goal = (candidate_point - qHi).norm();  // Distance to initial contact point (qHi)

//                     // Update the next point based on the closest one on the edge
//                     if (distance_to_goal < min_distance) {
//                         next_point = candidate_point;
//                         min_distance = distance_to_goal;
//                     }
//                 }
//             }
//         }

//         // Add current point to visited list
//         visited_points.push_back(next_point);

//         // Check if we have returned to qHi
//         if ((next_point - qHi).norm() < 0.1) {
//             qHi_reencountered = true;
//         }

//         // If we've encountered qHi again, we've completed the boundary traversal
//         if (qHi_reencountered) {
//             return next_point;
//         }
//     }

//     // Return the next point on the boundary, even if traversal isn't complete
//     return next_point;
// }
