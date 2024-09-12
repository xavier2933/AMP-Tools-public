#include "MyBugAlgorithm.h"
#include <algorithm>
#include <complex>
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

    auto getDirection = [this](const Eigen::Vector2d& lead, const Eigen::Vector2d& q)
    {
        Eigen::Vector2d direction5 = (lead - q).normalized();

        return direction5;
    };

    auto getRight = [this](const Eigen::Vector2d& direction3)
    {
        Eigen::Vector2d right_direction(direction3.y(), -direction3.x());

        return right_direction;
    };

    auto getLeft = [this](const Eigen::Vector2d& direction4)
    {
        Eigen::Vector2d left_direction(direction4.y(), -direction4.x());

        return left_direction;
    };

    /*
     you rotate point (px, py) around point (ox, oy) by angle theta you'll get:

p'x = cos(theta) * (px-ox) - sin(theta) * (py-oy) + ox

p'y = sin(theta) * (px-ox) + cos(theta) * (py-oy) + oy
    */
    auto rotate_point = [this](const Eigen::Vector2d& point, const Eigen::Vector2d& center, double angle) {
        double x = cos(angle) * (point.x() - center.x()) - sin(angle) * (point.y() - center.y()) + center.x();
        double y = sin(angle) * (point.x() - center.x()) + cos(angle) * (point.y() - center.y()) + center.y();  // Add center.y() here
        Eigen::Vector2d newVec(x, y);
        return newVec;
    };
       
    int i = 0;
    Eigen::Vector2d direction = (problem.q_goal - q).normalized();
    Eigen::Vector2d qLi = q;
    Eigen::Vector2d lead = q + (problem.q_goal - q).normalized()*.1;
    Eigen::Vector2d right_direction(direction.y(), -direction.x());
    Eigen::Vector2d right = q + right_direction * 0.1;


    int doWhileCounter = 0;

    while (true) {
        // WORKING - moves bug towards goal
        int outerLoop = 0;
        int count2 = 0;
        while (!obstacle_detected(lead) && distance_to_goal(q) > 0.1) {
            Eigen::Vector2d direction = (problem.q_goal - q).normalized();
            q += direction * 0.2;
            lead = q + direction *0.2;
            right = q + getRight(direction) * 0.2;
            // path.waypoints.push_back(right);
            // path.waypoints.push_back(lead);
            path.waypoints.push_back(q);
            if (distance_to_goal(q) <= 0.1) {
                // Goal reached
                std::cout << "returning from if 1" << std::endl;
                path.waypoints.push_back(problem.q_goal);
                return path;
            }            
        }

    

                // Step 2: Follow boundary when obstacle is detected
        if (obstacle_detected(lead)) {
            Eigen::Vector2d qHi = q;
            std::cout<<" obstacle " << std::endl;
            // Store the point on the boundary with the shortest distance to the goal
            Eigen::Vector2d qLi_temp = qLi;

            double min_distance_to_goal = distance_to_goal(qLi);
            bool qHi_reencountered = false;
            int j = 0;
            int z = 0;
            double angle_step = 0.05;  // Small angle step in radians
            Eigen::Vector2d tempDir;

            // lead += (problem.q_goal - q).normalized() * .2; 
            // Boundary following
            // for (int i = 0; i < 50; i++) {
            //     z = 0;
            //     std::cout << "new iteration " << std::endl;
            //     while (obstacle_detected(lead)) {
            //         z++;
            //         lead = rotate_point(lead, q, angle_step);  // Rotate lead around q

            //         // std::cout << "Iteration " << i << ": Lead = " << lead.transpose() << ", Right = " << right.transpose() << ", q = " << q.transpose() << std::endl;

            //         if (z > 100) {
            //             std::cout << "Break condition met" << std::endl;
            //             break;
            //         }

            //         // Update right based on lead's new direction
            //         Eigen::Vector2d direction2 = (lead - q).normalized();
            //         right = q + getRight(direction2) * 0.2;
            //     }

            //     // Direction adjustment: reverse the angle step if needed
            //     if (lead == q || z > 100) {
            //         angle_step = -angle_step;  // Reverse rotation direction
            //         z = 0;  // Reset the iteration counter
            //     }
            //     tempDir = getDirection(lead, q);  // Direction to lead
            //     // std::cout << q.transpose() << " lead: " << lead.transpose() <<" tempDir" << tempDir.transpose() <<std::endl;
            //     q += tempDir * 0.2;  // Move q forward
            //     lead = q + tempDir * 0.2;  // Update lead forward
            //     right = q + getRight(tempDir) * 0.2;  // Update right
            //     path.waypoints.push_back(lead);
            //     path.waypoints.push_back(right);
            //     path.waypoints.push_back(q);
            //     int s = 0;

            //     bool right = true;
            //     while(obstacle_detected(right) && !obstacle_detected(lead))
            //     {
            //         q += tempDir *0.2;
            //         lead = q + tempDir * 0.2;  // Update lead forward
            //         right = q + getRight(tempDir) * 0.2;   
            //         std::cout << "obs detected " << obstacle_detected(lead) << "at " << lead.transpose()<< std::endl;
   
            //         if(s > 50)
            //         {
            //             std::cout << "broke in new loop" << std::endl;
            //             break;
            //         }  
            //         s++;            
            //     }

            //     // Update positions after wall-following iteration

            //     // Add to path
            //     path.waypoints.push_back(lead);
            //     path.waypoints.push_back(right);
            //     path.waypoints.push_back(q);
            //     // if (close_to_start(q, start_point)) {
            //     //     break;  // Exit once full circle is completed
            //     // }
            //     // Break condition to stop after completing the loop
            //     // if (close_to_start(q, start_point)) {
            //     //     break;  // Exit once full circle is completed
            //     // }
            // }

            for(int i = 0; i < 50; i++)
            {
                tempDir = getDirection(lead, q);  // Direction to lead

                bool lead_on = obstacle_detected(lead);
                bool right_on = obstacle_detected(right);
            
                if(lead_on || (lead_on && right_on) || (!right_on &&!lead_on))
                {
                    lead = rotate_point(lead,q, 0.05);
                    Eigen::Vector2d direction2 = (lead - q).normalized();
                    right = q + getRight(direction2) * 0.2;
                    std::cout <<"new lead " << lead.transpose() << std::endl;
                    std::cout <<"new right" << right.transpose() << std::endl;
                    path.waypoints.push_back(lead);
                    path.waypoints.push_back(right);
                    std::cout << "after" << " lead: " << obstacle_detected(lead) << " right: " << obstacle_detected(right) << std::endl;

                }
                else if (!lead_on && right_on)
                {
                    q += tempDir *0.2;
                    lead = q + tempDir * 0.2;  // Update lead forward
                    right = q + getRight(tempDir) * 0.2;
                    path.waypoints.push_back(q);  
                }
                else if (!lead_on && right_on)
                {
                    rotate_point(lead,q, 0.05);
                } else{
                    std::cout << "breaking" << " lead: " << lead_on << " right: " << right_on << std::endl;
                }
                
            }
            path.waypoints.push_back(problem.q_goal);
            return path;

            
            do {

                // while(obstacle_detected(lead))
                // {
                //     z++;
                //     lead = rotate_point(lead, q, angle_step);
                //     // path.waypoints.push_back(lead);
                    
                //     // Update right to be based on the direction from q to lead
                //     Eigen::Vector2d direction = (lead - q).normalized();
                //     right = q + getRight(direction) * 0.1;
                //     break;
                // // Recalculate right after lead is rotated
                //     // Eigen::Vector2d direction = (problem.q_goal - q).normalized();
                //     // right = q + getRight(direction) * 0.1;
                // }
                // if (!obstacle_detected(lead)) {
                //     Eigen::Vector2d tempDir = (lead - q).normalized();
                //     q = lead;
                //     lead += 0.1 * tempDir;
                //     right = q + getRight(tempDir) * 0.1;
                //     path.waypoints.push_back(q);
                // }
                // path.waypoints.push_back(lead);
                // path.waypoints.push_back(right);
                
        //         q = MyBugAlgorithm::follow_boundary(q, qHi, problem.obstacles);  // Example: implement your boundary following here
        //         path.waypoints.push_back(q);

        //         double current_distance = distance_to_goal(q);
        //         if (current_distance < min_distance_to_goal) {
        //             qLi_temp = q;
        //             min_distance_to_goal = current_distance;
        //         }

        //         if ((q - qHi).norm() < 0.1) {
        //             qHi_reencountered = true;
        //         }

        //         if (distance_to_goal(q) <= 0.1) {
        //             // Goal reached while following the boundary
        //             return path;
        //         }

                j++;
                break;
             } while (!qHi_reencountered);

        //     // Update qLi with the closest point found during boundary following
        //     qLi = qLi_temp;

        //     // Step 3: Go to qLi
        //     q = qLi;
        //     path.waypoints.push_back(qLi);

        //     // Step 4: If moving toward the goal moves into an obstacle, exit with failure
        //     if (obstacle_detected(q)) {
        //         amp::Path2D empty_path;
        //         return empty_path;
                if(i > 50)
        {
            break;
        }

        i++;
            }
             }

        // if(i > 50)
        // {
        //     break;
        // }

        // i++;

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

bool MyBugAlgorithm::is_point_near_edge(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& point) const {
    // Check if point is near the line segment (p1, p2)
    Eigen::Vector2d line_vector = p2 - p1;
    Eigen::Vector2d point_vector = point - p1;
    double projection_length = point_vector.dot(line_vector.normalized());

    if (projection_length < 0 || projection_length > line_vector.norm()) {
        return false;
    }

    // Calculate the perpendicular distance from the point to the line
    Eigen::Vector2d closest_point_on_line = p1 + projection_length * line_vector.normalized();
    double distance_to_line = (point - closest_point_on_line).norm();

    // Define a threshold distance for being 'near' the edge
    double distance_threshold = 0.1; // Tune as needed
    return distance_to_line < distance_threshold;
}

Eigen::Vector2d MyBugAlgorithm::follow_boundary(const Eigen::Vector2d& current_position, const Eigen::Vector2d& qHi, const std::vector<amp::Obstacle2D>& obstacles) {
    // Start by getting the obstacle's boundary near the current position
    Eigen::Vector2d new_position = current_position;
    
    // Implement the right-hand rule boundary following
    // Move along the boundary by calculating a small step along the obstacle edge
    double step_size = 0.1; // Tune step size as needed

    // Find the closest edge of the obstacle
    for (const auto& obstacle : obstacles) {
        for (size_t i = 0; i < obstacle.verticesCCW().size(); ++i) {
            const Eigen::Vector2d& p1 = obstacle.verticesCCW()[i];
            const Eigen::Vector2d& p2 = obstacle.verticesCCW()[(i + 1) % obstacle.verticesCCW().size()];

            // Check if the robot is near this edge
            if (is_point_near_edge(p1, p2, current_position)) {
                // Move along the boundary edge with the right-hand rule:
                // The robot should turn right, so it follows the boundary keeping the obstacle on its right-hand side
                Eigen::Vector2d edge_direction = (p2 - p1).normalized();
                Eigen::Vector2d right_normal = Eigen::Vector2d(edge_direction.y(), -edge_direction.x());

                // Add both forward and right movements for smooth following
                new_position += (step_size * edge_direction) + (step_size * right_normal);
                
                // Break out once the new position is updated
                break;
            }
        }
    }

    return new_position;
}




