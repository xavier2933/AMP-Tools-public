# include "MySamplingBasedPlanners.h"


auto vector2dCompare = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    if (a.x() == b.x()) {
        return a.y() < b.y();
    }
    return a.x() < b.x();
};
// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}

bool isInCollision(const amp::Problem2D& environment, const Eigen::Vector2d& point) {
    for (const auto& polygon : environment.obstacles) {
        const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCCW();
        int num_vertices = vertices.size();
        bool inside = false;

        // Ray-casting algorithm to check if point is inside polygon
        for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
            // Check if the ray crosses the edge between vertices[i] and vertices[j]
            if ((vertices[i].y() > point.y()) != (vertices[j].y() > point.y()) &&
                (point.x() < (vertices[j].x() - vertices[i].x()) * (point.y() - vertices[i].y()) / 
                (vertices[j].y() - vertices[i].y()) + vertices[i].x())) {
                // Flip the inside flag
                inside = !inside;
            }
        }

        if (inside) {
            return true;  // Point is inside this polygon
        }
    }

    return false;
}

bool hasCollision(Eigen::Vector2d j1, Eigen::Vector2d j2, const amp::Problem2D& env) 
{
    Eigen::Vector2d direction = (j2 - j1).normalized();
    // std::cout << " Joint 1 : " << j1 << " Joint 2: " << j2 << std::endl;
    
    // Use a floating-point value for 'i' to step along the segment
    for(double i = 0; i <= 1; i += 0.01) // i is now a double, stepping by 0.01
    {
        Eigen::Vector2d point = j1 + i * (j2 - j1); // Scale the entire vector, not just normalized direction

        if(isInCollision(env, point))
        {
            // std::cout << "point is inside polygon" << std::endl;
            return true;
        }
    }
    
    return false;
}

// Implement your RRT algorithm here
Eigen::Vector2d MyRRT::getNearestConfig(Eigen::Vector2d temp, std::vector<Eigen::Vector2d> tree)
{
    double minDist = std::numeric_limits<double>::infinity();
    double dist = 0.0;
    Eigen::Vector2d closest;
    for(auto& node : tree)
    {
        dist = (temp - node).norm();
        if (dist < minDist)
        {
            closest = node;
            minDist = dist;
        }
    }
    return closest;
}

Eigen::Vector2d MyRRT::getRandomConfig(const amp::Problem2D& problem) {
    double x = problem.x_min + (problem.x_max - problem.x_min) * ((double) rand() / RAND_MAX);
    double y = problem.y_min + (problem.y_max - problem.y_min) * ((double) rand() / RAND_MAX);
    // double y = -3 + (6) * ((double) rand() / RAND_MAX);
    return {x, y};
}

bool subpathCollsionFree(Eigen::Vector2d rand, Eigen::Vector2d near, const amp::Problem2D& problem, double step)
{
    Eigen::Vector2d newEnd = near + (rand - near).normalized() * step;
    bool res = hasCollision(rand, newEnd, problem);
    return res;
}   

// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    std::cout << "Running RRT " << std::endl;
    // path.waypoints.push_back(problem.q_init);
    nodes[0] = problem.q_init;
    
    Eigen::Vector2d temp;
    Eigen::Vector2d near;
    std::vector<Eigen::Vector2d> tree;
    std::map<Eigen::Vector2d, Eigen::Vector2d, decltype(vector2dCompare)> prevMap(vector2dCompare); // current, node before
    tree.push_back(problem.q_init);
    
    int count = 0;
    int n = 5000;
    double step = 0.25;
    int goalBiasCount = 0;
    bool goalFound = false;
    Eigen::Vector2d goalNode;

    while(count < n) {
        
        if(goalBiasCount == 20) {
            temp = problem.q_goal;
            goalBiasCount = 0;
        } else {
            temp = getRandomConfig(problem);

        }
        near = getNearestConfig(temp, tree);


        goalBiasCount++;
        
        if(!subpathCollsionFree(temp, near, problem, step)) {
            Eigen::Vector2d newEnd = near + (temp - near).normalized() * step;
            tree.push_back(newEnd);
            
            // Track the parent of the new node
            prevMap[newEnd] = near;

            // std::cout << "map " << newEnd.transpose() << " maps to " << near.transpose() << std::endl;
            
            // Check if we've reached the goal
            if((newEnd - problem.q_goal).norm() < 0.5) {
                std::cout << "goal found " << std::endl;
                goalFound = true;
                goalNode = newEnd;
                break;
            }
        }
        count++;
    }

    if (goalFound) {
        // Backtrack from the goal to the start using the map
        Eigen::Vector2d currentNode = goalNode;
        // path.waypoints.push_back(problem.q_goal);
        int newCount = 0;
        path.waypoints.push_back(problem.q_goal);
        while (currentNode != problem.q_init) {
            path.waypoints.push_back(currentNode);
            currentNode = prevMap[currentNode];  // Move to the parent node
            newCount++;
            // std::cout << "current Node " << currentNode << std::endl;
            if (newCount > n)
            {
                std::cout << "breaking early" << std::endl;
                break;
            }
        }
        path.waypoints.push_back(problem.q_init);  // Finally, add the start
        std::reverse(path.waypoints.begin(), path.waypoints.end());  // Reverse to get the path from start to goal
        std::cout << "Path length " << path.length();
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }

    return path;
}
