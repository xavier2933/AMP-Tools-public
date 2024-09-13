#include "MyBugAlgorithm.h"
#include <algorithm>
#include <complex>
#include <limits>

/*
This might be the worst code I've ever written.
*/
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem)
{
    amp::Path2D path;
    // change 1 to 2 to make bug 2
    path = MyBugAlgorithm::planBug1(problem, path);
    return path;
}

amp::Path2D MyBugAlgorithm::planBug1(const amp::Problem2D& problem, amp::Path2D& path) {
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
        double angle = M_PI / 4;  // 45 degrees in radians
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << std::cos(angle), std::sin(angle),
                        -std::sin(angle), std::cos(angle);
        
        Eigen::Vector2d right_direction = rotation_matrix * direction3.normalized();
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
    Eigen::Vector2d halfRight = q + right_direction * 0.05;



    int doWhileCounter = 0;

    while (true) {
        // WORKING - moves bug towards goal
        double stepSize = 0.01;
        int outerLoop = 0;
        int count2 = 0;
        while (!obstacle_detected(lead) && distance_to_goal(q) > 0.1) {
            Eigen::Vector2d direction = (problem.q_goal - q).normalized();
            q += direction * stepSize;
            lead = q + direction *stepSize;
            right = q + getRight(direction) * stepSize;
            halfRight = q + getRight(direction) * (stepSize/2.00);
            // path.waypoints.push_back(right);
            // path.waypoints.push_back(lead);
            path.waypoints.push_back(q);
            if (distance_to_goal(q) <= 0.1) {
                // Goal reached
                std::cout << "Found goal!" << std::endl;
                path.waypoints.push_back(problem.q_goal);
                return path;
            }            
        }

    

                // Step 2: Follow boundary when obstacle is detected
        if (obstacle_detected(lead)) {
            Eigen::Vector2d qHi = q;
            // Store the point on the boundary with the shortest distance to the goal
            Eigen::Vector2d qLi_temp = qLi;

            double min_distance_to_goal = distance_to_goal(qLi);
            bool qHi_reencountered = false;
            int j = 0;
            int z = 0;
            double angle_step = 0.05;  // Small angle step in radians
            Eigen::Vector2d tempDir;

        
            double angleTheta = 0.025;
            Eigen::Vector2d halfRight;
            double angleTraveled = 0.0;

            /*
            THoughts:
            Track total angle turned? if > 160 turn right?
            */
            for(int i = 0; i < 100000; i++)
            {
                if(distance_to_goal(q) < distance_to_goal(qLi))
                {
                    qLi = q;
                }
                if((q - qHi).norm() < 0.1 && i > 1000)
                {
                    qHi_reencountered = true;
                    break;
                }
                tempDir = getDirection(lead, q);  // Direction to lead

                bool lead_on = obstacle_detected(lead);
                bool right_on = obstacle_detected(right);
                bool half_right_on = obstacle_detected(halfRight);

                if(lead_on || (!right_on && !lead_on))
                {
                    lead = rotate_point(lead,q, angleTheta);
                    Eigen::Vector2d direction2 = (lead - q).normalized();
                    right = q + getRight(direction2) * stepSize;
                    halfRight = q + getRight(direction2) * (stepSize/2.00);
                    angleTraveled+=angleTheta;
                } else if (!lead_on && right_on) {
                    q += tempDir * stepSize;
                    lead = q + tempDir * stepSize;  // Update lead forward
                    right = q + getRight(tempDir) * stepSize;
                    path.waypoints.push_back(q);  
                } else {
                    continue;
                }
                
            }

            for(int i = 0; i < 100000; i++)
            {

                if((q - qLi).norm() < 0.1 && i > 1000)
                {
                    qHi_reencountered = true;
                    break;
                }
                tempDir = getDirection(lead, q);  // Direction to lead

                bool lead_on = obstacle_detected(lead);
                bool right_on = obstacle_detected(right);
                bool half_right_on = obstacle_detected(halfRight);
                if(lead_on || (!right_on && !lead_on))
                {
                    lead = rotate_point(lead,q, angleTheta);
                    Eigen::Vector2d direction2 = (lead - q).normalized();
                    right = q + getRight(direction2) * stepSize;
                    halfRight = q + getRight(direction2) * (stepSize/2.00);
                    angleTraveled+=angleTheta;
                 } else if (!lead_on && right_on) {
                    q += tempDir * stepSize;
                    lead = q + tempDir * stepSize;  // Update lead forward
                    right = q + getRight(tempDir) * stepSize;
                    // halfRight = q + getRight(tempDir) * (stepSize/2.00);

                    path.waypoints.push_back(q);  
                } else {
                    continue;
                }  
            }
            if(i > 50)
            {
                break;
            }

            i++;
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

bool isPointNearLine(const Eigen::Vector2d& point, const Eigen::Vector2d& start, const Eigen::Vector2d& goal, double threshold = 0.01) {
    Eigen::Vector2d lineVector = goal - start;
    Eigen::Vector2d pointVector = point - start;

    double t = pointVector.dot(lineVector) / lineVector.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));
    Eigen::Vector2d projection = start + t * lineVector;
    double distance = (point - projection).norm();
    return distance <= threshold;
}


amp::Path2D MyBugAlgorithm::planBug2(const amp::Problem2D& problem, amp::Path2D& path) {
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
        double angle = M_PI / 4;  // 45 degrees in radians
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << std::cos(angle), std::sin(angle),
                        -std::sin(angle), std::cos(angle);
        
        Eigen::Vector2d right_direction = rotation_matrix * direction3.normalized();
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
    Eigen::Vector2d halfRight = q + right_direction * 0.05;



    int doWhileCounter = 0;

    while (true) {
        // WORKING - moves bug towards goal
        double stepSize = 0.01;
        int outerLoop = 0;
        int count2 = 0;
        while (!obstacle_detected(lead) && distance_to_goal(q) > 0.1) {
            Eigen::Vector2d direction = (problem.q_goal - q).normalized();
            q += direction * stepSize;
            lead = q + direction *stepSize;
            right = q + getRight(direction) * stepSize;
            // path.waypoints.push_back(right);
            // path.waypoints.push_back(lead);
            if(obstacle_detected(lead))
            {
                break;
            }
            path.waypoints.push_back(q);
            // if(obstacle_detected(q)){
            //     std::cout << "BREAK 1" << std::endl;
            //     return path;
            // }
            if (distance_to_goal(q) <= 0.1) {
                // Goal reached
                std::cout << "found goal! " << std::endl;
                path.waypoints.push_back(problem.q_goal);
                return path;
            }            
        }

    

                // Step 2: Follow boundary when obstacle is detected
        if (obstacle_detected(lead)) {
            Eigen::Vector2d qHi = q;
            // Store the point on the boundary with the shortest distance to the goal
            Eigen::Vector2d qLi_temp = qLi;

            double min_distance_to_goal = distance_to_goal(qLi);
            bool qHi_reencountered = false;
            int j = 0;
            int z = 0;
            double angle_step = 0.05;  // Small angle step in radians
            Eigen::Vector2d tempDir;

        
            double angleTheta = 0.025;
            Eigen::Vector2d halfRight;
            double angleTraveled = 0.0;
            double hitDist = distance_to_goal(q);

            for(int i = 0; i < 100000; i++)
            {
            // if(obstacle_detected(q)){
            //     std::cout << "BREAK 2" << std::endl;
            //     return path;
            // }
                if(isPointNearLine(q, problem.q_init, problem.q_goal) && ((q - qHi).norm() > 0.01) && distance_to_goal(q) < hitDist)
                {
                    bool test = true;
                    Eigen::Vector2d temp = q;
                    Eigen::Vector2d testDir = getDirection(problem.q_goal, q);
                    for(double i = 0; i < 0.2; i+=0.01)
                    {
                        
                        if(obstacle_detected(q+(i*testDir)))
                        {
                            test = false;
                        }
                    }
                    if(test = true)
                    {
                        break;
                    }
                }
                
                tempDir = getDirection(lead, q);  // Direction to lead

                bool lead_on = obstacle_detected(lead);
                bool right_on = obstacle_detected(right);
                bool half_right_on = obstacle_detected(halfRight);

                if(lead_on || (!right_on && !lead_on))
                {
                    lead = rotate_point(lead,q, angleTheta);
                    Eigen::Vector2d direction2 = (lead - q).normalized();
                    right = q + getRight(direction2) * stepSize;
                    halfRight = q + getRight(direction2) * (stepSize/2.00);
                    angleTraveled+=angleTheta;
                 }
                else if (!lead_on && right_on)
                {
                    q += tempDir * stepSize;
                    lead = q + tempDir * stepSize;  // Update lead forward
                    right = q + getRight(tempDir) * stepSize;

                    path.waypoints.push_back(q);  
                }
                else{
                    continue;
                }
                
            }
            if(i > 50)
            {
                break;
            }
            i++;
            }
        }
    path.waypoints.push_back(problem.q_goal);
    return path;
    }

