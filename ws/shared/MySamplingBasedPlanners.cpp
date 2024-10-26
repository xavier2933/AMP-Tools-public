# include "MySamplingBasedPlanners.h"


MyRRT::Vector2dComparator MyRRT::vector2dCompare = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    if (a.x() == b.x()) {
        return a.y() < b.y();
    }
    return a.x() < b.x();
};

auto vectorXdCompare = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    for (int i = 0; i < a.size(); ++i) {
        if (a[i] < b[i]) {
            return true;
        } else if (a[i] > b[i]) {
            return false;
        }
    }
    return false;  // If all components are equal
};



bool isInCollision(const amp::MultiAgentProblem2D& environment, const Eigen::VectorXd& point) {
    for (const auto& polygon : environment.obstacles) {
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

        if (inside) {
            return true;  // Point is inside this polygon
        }
    }

    return false;
}

bool hasCollision(Eigen::VectorXd j1, Eigen::VectorXd j2, const amp::MultiAgentProblem2D& env) {
    for (int i = 0; i < j1.size() / 2; ++i) {
        Eigen::Vector2d robot_j1(j1(2 * i), j1(2 * i + 1));
        Eigen::Vector2d robot_j2(j2(2 * i), j2(2 * i + 1));
        Eigen::Vector2d direction = (robot_j2 - robot_j1).normalized();
        
        for (double t = 0; t <= 1.0; t += 0.01) {
            Eigen::Vector2d point = robot_j1 + t * (robot_j2 - robot_j1); // Scale the entire vector, not just normalized direction
            if (isInCollision(env, point)) {
                return true;
            }
        }
    }
    
    return false;
}

// Nearest config function for multi-agent system
Eigen::VectorXd MyRRT::getNearestConfig(const Eigen::VectorXd& temp, const std::vector<Eigen::VectorXd>& tree) {
    double minDist = std::numeric_limits<double>::infinity();
    double dist = 0.0;
    Eigen::VectorXd closest;
    
    for (const auto& node : tree) {
        dist = (temp - node).norm();  // Compute Euclidean distance in combined space
        if (dist < minDist) {
            closest = node;
            minDist = dist;
        }
    }
    return closest;
}

// Random config for multi-agent system
Eigen::VectorXd MyRRT::getRandomConfig(const amp::MultiAgentProblem2D& problem) {
    Eigen::VectorXd q_random(2 * problem.numAgents());
    for (int i = 0; i < problem.numAgents(); ++i) {
        double x = problem.x_min + (problem.x_max - problem.x_min) * ((double)rand() / RAND_MAX);
        double y = problem.y_min + (problem.y_max - problem.y_min) * ((double)rand() / RAND_MAX);
        q_random(2 * i) = x;
        q_random(2 * i + 1) = y;
    }
    return q_random;
}

bool subpathCollisionFree(const Eigen::VectorXd& rand, const Eigen::VectorXd& near, const amp::MultiAgentProblem2D& problem, double step) {
    Eigen::VectorXd newEnd = near + (rand - near).normalized() * step;
    return hasCollision(rand, newEnd, problem);
}


bool GOOOOOOOOOL(Eigen::VectorXd curr, Eigen::VectorXd GOL, int numAgents, double epsilon)
{
    Eigen::Vector2d curr_i;
    Eigen::Vector2d GOL_i;
    bool messi = true;
    for(int i = 0; i < numAgents; i++)
    {

        Eigen::Vector2d curr_i(curr[2 * i], curr[2 * i + 1]);
        Eigen::Vector2d GOL_i(GOL[2 * i], GOL[2 * i + 1]);

        if((GOL_i - curr_i).norm() >= epsilon)
        {
            messi = false;
            return messi;
        }
    
    }
    return messi;
}

bool areRobotsInCollision(const Eigen::VectorXd& states, double radius) {
    int numRobots = states.size() / 2;
    double collisionDistance = 2 * radius;

    for (int i = 0; i < numRobots; ++i) {
        Eigen::Vector2d pos_i(states(2 * i), states(2 * i + 1));
        for (int j = i + 1; j < numRobots; ++j) {
            Eigen::Vector2d pos_j(states(2 * j), states(2 * j + 1));
            if ((pos_i - pos_j).norm() < collisionDistance) {
                return true;  // Collision detected
            }
        }
    }

    return false;  // No collision detected
}

// Main RRT planning function for multi-agent system
amp::MultiAgentPath2D MyRRT::planHigherD(const amp::MultiAgentProblem2D& problem) {
    amp::MultiAgentPath2D path;
    std::cout << "Running RRT " << std::endl;

    Eigen::VectorXd temp(2 * problem.numAgents());
    Eigen::VectorXd near(2 * problem.numAgents());
    std::vector<Eigen::VectorXd> tree;
    std::map<Eigen::VectorXd, Eigen::VectorXd, decltype(vectorXdCompare)> prevMap(vectorXdCompare); // current, node before

    Eigen::VectorXd q_init(2 * problem.numAgents());
    Eigen::VectorXd q_goal(2 * problem.numAgents());

    for (std::size_t i = 0; i < problem.numAgents(); ++i) {
        // Assign the x and y coordinates for q_init
        q_init(2 * i)     = problem.agent_properties[i].q_init.x();
        q_init(2 * i + 1) = problem.agent_properties[i].q_init.y();

        // Assign the x and y coordinates for q_goal
        q_goal(2 * i)     = problem.agent_properties[i].q_goal.x();
        q_goal(2 * i + 1) = problem.agent_properties[i].q_goal.y();
    }

    tree.push_back(q_init);  // Push initial configuration

    int count = 0;
    int n = 200000;
    double step = 0.5;
    int goalBiasCount = 0;
    double epsilon = 0.5;
    bool goalFound = false;
    Eigen::VectorXd goalNode(2 * problem.numAgents());

    while (count < n) {
        if (goalBiasCount == 20) {
            temp = q_goal;  // Use goal for biased sampling
            goalBiasCount = 0;
        } else {
            temp = getRandomConfig(problem);  // Sample random configuration
        }

        near = getNearestConfig(temp, tree);  // Find nearest node in the tree
        goalBiasCount++;
        Eigen::VectorXd newEnd = near + (temp - near).normalized() * step;

        if (!subpathCollisionFree(temp, near, problem, step) && !areRobotsInCollision(newEnd, 0.5)) {
            tree.push_back(newEnd);
            // std::cout << "New End " << newEnd.transpose() << std::endl;
            count++; // move to where we actually add node to graph

            
            prevMap[newEnd] = near;  // Track the parent of the new node
            // keep track of index of nodes from root

            if (GOOOOOOOOOL(newEnd, q_goal, problem.numAgents(), epsilon)) {  // Check if goal is reached
                std::cout << "goal found" << std::endl;
                goalFound = true;
                goalNode = newEnd;
                break;
            }
        }
    }

    if (goalFound) {
        // Backtrack from goal to start using the map
        Eigen::VectorXd currentNode = goalNode;
        std::vector<Eigen::VectorXd> pathVector;
        pathVector.push_back(q_goal);
        std::cout << "q goal " << q_goal.transpose() << std::endl;
        // path.waypoints.push_back(q_goal);
        while (currentNode != q_init) {
            pathVector.push_back(currentNode);

            // path.waypoints.push_back(currentNode);
            currentNode = prevMap[currentNode];  // Move to parent node
        }
        // path.waypoints.push_back(q_init);  // Add the start node
        pathVector.push_back(q_init);
        std::cout << "q init " << q_init.transpose() << std::endl;

        std::reverse(pathVector.begin(), pathVector.end());  // Reverse the path to go from start to goal

        for(int i = 0; i < problem.numAgents(); i++)
        {
            amp::Path2D holderPath;

            Eigen::Vector2d pathTempVec;
            for(auto& vec: pathVector)
            {
                Eigen::Vector2d pathTempVec(vec[2 * i], vec[2 * i + 1]);
                // std::cout << "path " << pathTempVec.transpose() << std::endl;

                holderPath.waypoints.push_back(pathTempVec);
            }
            holderPath.waypoints.push_back(problem.agent_properties[i].q_goal);
            holderPath.waypoints.insert(holderPath.waypoints.begin(), problem.agent_properties[i].q_init);
            std::cout << "Path length " << holderPath.length() << std::endl;

            path.agent_paths.push_back(holderPath);
        }

        // std::cout << "Path length " << path.length();
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }

    return path;
}


Eigen::Vector2d MyRRT::getNearestConfig2d(Eigen::Vector2d temp, std::vector<Eigen::Vector2d> tree)
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

Eigen::Vector2d MyRRT::getRandomConfig2d(const amp::Problem2D& problem) {
    double x = problem.x_min + (problem.x_max - problem.x_min) * ((double) rand() / RAND_MAX);
    double y = problem.y_min + (problem.y_max - problem.y_min) * ((double) rand() / RAND_MAX);
    // double y = -3 + (6) * ((double) rand() / RAND_MAX);
    return {x, y};
}


 bool isInCollision2d(const amp::Problem2D& environment, const Eigen::Vector2d& point) {
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

bool hasCollision2d(Eigen::Vector2d j1, Eigen::Vector2d j2, const amp::Problem2D& env) 
{
    Eigen::Vector2d direction = (j2 - j1).normalized();
    // std::cout << " Joint 1 : " << j1 << " Joint 2: " << j2 << std::endl;
    
    // Use a floating-point value for 'i' to step along the segment
    for(double i = 0; i <= 1; i += 0.01) // i is now a double, stepping by 0.01
    {
        Eigen::Vector2d point = j1 + i * (j2 - j1); // Scale the entire vector, not just normalized direction

        if(isInCollision2d(env, point))
        {
            // std::cout << "point is inside polygon" << std::endl;
            return true;
        }
    }
    
    return false;
}

bool subpathCollsionFree2d(Eigen::Vector2d rand, Eigen::Vector2d near, const amp::Problem2D& problem, double step)
{
    Eigen::Vector2d newEnd = near + (rand - near).normalized() * step;
    bool res = hasCollision2d(rand, newEnd, problem);
    return res;
} 

bool MyRRT::checkPaths(Eigen::Vector2d point, int time, double radius)
{
    for(auto& map:timeToPoint)
    {
        Eigen::Vector2d pointFromMap = map[time];
        if((point-pointFromMap).norm() < 2*radius)
        {
            return false;
        }
    }
    return true; // if path is valid
}

// Implement your PRM algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    std::cout << "Running RRT " << std::endl;

    // Initialize maps with the comparator
    std::map<int, Eigen::Vector2d> timeToPointMap;
    std::map<Eigen::Vector2d, int, decltype(vector2dCompare)> pointToTimeMap(vector2dCompare);

    nodes[0] = problem.q_init;
    timeToPointMap.insert({0, problem.q_init});
    pointToTimeMap.insert({problem.q_init, 0});

    Eigen::Vector2d temp;
    Eigen::Vector2d near;
    std::vector<Eigen::Vector2d> tree;
    std::map<Eigen::Vector2d, Eigen::Vector2d, decltype(vector2dCompare)> prevMap(vector2dCompare);

    tree.push_back(problem.q_init);

    int count = 0;
    int n = 5000;
    double step = 0.25;
    int goalBiasCount = 0;
    bool goalFound = false;
    Eigen::Vector2d goalNode;

    while (count < n) {
        if (goalBiasCount == 20) {
            temp = problem.q_goal;
            goalBiasCount = 0;
        } else {
            temp = getRandomConfig2d(problem);
        }
        near = getNearestConfig2d(temp, tree);
        goalBiasCount++;
        int val = pointToTimeMap[near];
        Eigen::Vector2d newEnd = near + (temp - near).normalized() * step;
        val = val + 1;
        if (!subpathCollsionFree2d(temp, near, problem, step) && checkPaths(newEnd, val, 0.5)) {
            // Eigen::Vector2d newEnd = near + (temp - near).normalized() * step;
            tree.push_back(newEnd);

            prevMap[newEnd] = near;
            pointToTimeMap.insert({newEnd, val});
            timeToPointMap.insert({val, newEnd});

            if ((newEnd - problem.q_goal).norm() < 0.5) {
                std::cout << "goal found " << std::endl;
                goalFound = true;
                goalNode = newEnd;
                break;
            }
        }
        count++;
    }

    if (goalFound) {
        Eigen::Vector2d currentNode = goalNode;
        path.waypoints.push_back(problem.q_goal);
        while (currentNode != problem.q_init) {
            path.waypoints.push_back(currentNode);
            currentNode = prevMap[currentNode];
        }
        path.waypoints.push_back(problem.q_init);
        std::reverse(path.waypoints.begin(), path.waypoints.end());

        int pathCount = 0;
        for (auto& point : path.waypoints) {
            pathCount++;
            timeToPointMap.insert({pathCount, point});
        }

        timeToPoint.push_back(timeToPointMap);
        pointToTime.push_back(pointToTimeMap);
        std::cout << "Path length " << path.length();
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }

    return path;
}