# include "MySamplingBasedPlanners.h"


auto vector2dCompare = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
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

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}
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

amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;

    return path;
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
    int n = 20000;
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

        if (!subpathCollisionFree(temp, near, problem, step)) {
            Eigen::VectorXd newEnd = near + (temp - near).normalized() * step;
            tree.push_back(newEnd);
            std::cout << "New End " << newEnd.transpose() << std::endl;
            
            prevMap[newEnd] = near;  // Track the parent of the new node

            if (GOOOOOOOOOL(newEnd, q_goal, problem.numAgents(), epsilon)) {  // Check if goal is reached
                std::cout << "goal found" << std::endl;
                goalFound = true;
                goalNode = newEnd;
                break;
            }
        }
        count++;
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
        amp::Path2D holderPath;

        for(int i = 0; i < problem.numAgents(); i++)
        {
            Eigen::Vector2d pathTempVec;
            for(auto& vec: pathVector)
            {
                Eigen::Vector2d pathTempVec(vec[2 * i], vec[2 * i + 1]);
                // std::cout << "path " << pathTempVec.transpose() << std::endl;

                holderPath.waypoints.push_back(pathTempVec);
            }
            holderPath.waypoints.push_back(problem.agent_properties[i].q_goal);
            holderPath.waypoints.insert(holderPath.waypoints.begin(), problem.agent_properties[i].q_init);
            path.agent_paths.push_back(holderPath);
        }

        // std::cout << "Path length " << path.length();
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }

    return path;
}
