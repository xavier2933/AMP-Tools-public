#include "MyKinoRRT.h"

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

void MySingleIntegrator::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    state += dt * control;
};

void MyFirstOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt) {
    auto dynamics = [this](const Eigen::VectorXd& state, const Eigen::VectorXd& control) -> Eigen::VectorXd {
        Eigen::VectorXd dxdt(3);
        double theta = state(2);
        double v = control(0);   // Linear velocity
        double omega = control(1); // Angular velocity

        // Define the unicycle model's dynamics
        dxdt(0) = v * cos(theta); // dx/dt = u_sigma * cos(theta)
        dxdt(1) = v * sin(theta); // dy/dt = u_sigma * sin(theta)
        dxdt(2) = omega;          // dtheta/dt = omega

        return dxdt;
    };

    // Calculate w1, w2, w3, and w4
    Eigen::VectorXd w1 = dynamics(state, control);
    Eigen::VectorXd w2 = dynamics(state + 0.5 * dt * w1, control);
    Eigen::VectorXd w3 = dynamics(state + 0.5 * dt * w2, control);
    Eigen::VectorXd w4 = dynamics(state + dt * w3, control);

    // Update the state using RK4 formula
    // std::cout << "State before " << state.transpose() << std::endl;
    state = state + (dt / 6.0) * (w1 + 2.0 * w2 + 2.0 * w3 + w4);
    // std::cout << "State after " << state.transpose() << std::endl << std::endl;
};

void MySecondOrderUnicycle::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt)
{

    auto dynamics2 = [this](const Eigen::VectorXd& state, const Eigen::VectorXd& control) -> Eigen::VectorXd {
        Eigen::VectorXd dxdt2(5);

        double sigma = state(3);
        double r = 0.25;
        Eigen::VectorXd dxdt(3);
        double theta = state(2);
        double v = control(0);   // Linear velocity
        double omega = control(1); // Angular velocity

        // Define the unicycle model's dynamics
        dxdt2(0) = state(3) * r * cos(theta);
        dxdt2(1) = state(3) * r * sin(theta); // dy/dt = v * sin(theta)
        dxdt2(2) = state(4);          // dtheta/dt = omega
        dxdt2(3) = control(1);
        dxdt2(4) = control(2);

        return dxdt;
    };

    // Calculate w1, w2, w3, and w4
    Eigen::VectorXd w1 = dynamics2(state, control);
    Eigen::VectorXd w2 = dynamics2(state + 0.5 * dt * w1, control);
    Eigen::VectorXd w3 = dynamics2(state + 0.5 * dt * w2, control);
    Eigen::VectorXd w4 = dynamics2(state + dt * w3, control);

    // Update the state using RK4 formula
    // std::cout << "State before " << state.transpose() << std::endl;
    state = state + (dt / 6.0) * (w1 + 2.0 * w2 + 2.0 * w3 + w4);
}

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt)
{

}

bool isInCollision(const amp::KinodynamicProblem2D& environment, const Eigen::Vector2d& point) {
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

bool hasCollision(Eigen::Vector2d j1, Eigen::Vector2d j2, const amp::KinodynamicProblem2D& env) 
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

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init;
    std::vector<Eigen::VectorXd> controls;
    std::vector<double> timeSteps;

    // use points in workspace b/c rotation doesn't really matter
    Eigen::Vector2d init(problem.q_init[0], problem.q_init[1]);
    double x = problem.q_goal[0].first + (problem.q_goal[0].second - problem.q_goal[0].first)/2;
    double y = problem.q_goal[1].first + (problem.q_goal[1].second - problem.q_goal[1].first)/2;
    Eigen::Vector2d goal(x, y);
    // std::cout << x << " y " << y << std::endl;

    int a = 0;
    for(auto& point : problem.q_goal)
    {
        a++;
        std::cout << "dimension " << a << " is " << point.first << ", " << point.second << std::endl;
    }
    Eigen::Vector2d temp;
    Eigen::VectorXd tempTemp = Eigen::VectorXd::Zero(problem.q_init.size()); // Initialize a 2-dimensional VectorXd

    Eigen::Vector2d near;
    std::vector<Eigen::VectorXd> tree;
    std::map<Eigen::Vector2d, Eigen::Vector2d, decltype(vector2dCompare)> prevMap(vector2dCompare); // current, node before
    std::map<Eigen::VectorXd, Eigen::VectorXd, decltype(vectorXdCompare)> controlMap(vectorXdCompare); // current, node before

    tree.push_back(init);
    // path.waypoints.push_back(state);
    int count = 0;
    int n = 5000;
    double step = 0.25;
    int goalBiasCount = 0;
    bool goalFound = false;
    Eigen::Vector2d goalNode;

    while(count < n) {
        
        if(goalBiasCount == 20) {
            temp = goal;
            goalBiasCount = 0;
        } else {
            temp = getRandomConfig(problem);

        }
        near = getNearestConfig(temp, tree);
        tempTemp[0] = temp[0];
        tempTemp[1] = temp[1];

        Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
        agent.propagate(tempTemp, control, 0.4);
        temp[0] = tempTemp[0];
        temp[1] = tempTemp[1];

        goalBiasCount++;
        
        if(!subpathCollsionFree(temp, near, problem, step)) {
            Eigen::Vector2d newEnd = near + (temp - near).normalized() * step;
            tree.push_back(newEnd);
            
            // Track the parent of the new node
            prevMap[newEnd] = near;
            controlMap[newEnd] = control;

            // std::cout << "map " << newEnd.transpose() << " maps to " << near.transpose() << std::endl;
            
            // Check if we've reached the goal
            if((newEnd - goal).norm() < 0.5) {
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
        path.waypoints.push_back(goal);
        timeSteps.push_back(0.0);
        controls.push_back(Eigen::VectorXd::Zero(problem.q_init.size()));

        while (currentNode != init) {

            path.waypoints.push_back(currentNode);
            controls.push_back(controlMap[currentNode]);
            timeSteps.push_back(0.4);
            currentNode = prevMap[currentNode];  // Move to the parent node
            newCount++;

            if(currentNode == init)
            {
                timeSteps.push_back(0.0);
                controls.push_back(Eigen::VectorXd::Zero(problem.q_init.size())); 
            }
            // std::cout << "current " << currentNode.transpose() << " with control " << controlMap[currentNode].transpose() << std::endl;
            // std::cout << "current Node " << currentNode << std::endl;
            if (newCount > n)
            {
                std::cout << "breaking early" << std::endl;
                break;
            }
        }

        path.waypoints.push_back(init);  // Finally, add the start
        path.controls = controls;

        path.durations = timeSteps;
        // std::cout << "waypoints " << path.waypoints.size() << " controls " << path.controls.size() << " durations " << path.durations.size() << std::endl;



        std::reverse(path.waypoints.begin(), path.waypoints.end());  // Reverse to get the path from start to goal
        std::reverse(path.controls.begin(), path.controls.end());  // Reverse to get the path from start to goal

        for(int i = 0; i < path.waypoints.size(); i++)
        {
            // std::cout << "Point " << path.waypoints[i].transpose() << " with control " << path.controls[i].transpose() << " and duration " << path.durations[i] << std::endl;
        }
        // std::cout << "Path length " << path.length();
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }


    // std::cout << "size " << problem.q_init.size() << std::endl;
    // for (int i = 0; i < 10; i++) {
    //     Eigen::VectorXd control = Eigen::VectorXd::Random(problem.q_init.size());
    //     agent.propagate(state, control, 1.0);
    //     std::cout << "State is " << state.transpose() << std::endl;
    //     path.waypoints.push_back(state);
    //     path.controls.push_back(control);
    //     path.durations.push_back(1.0);
    // }
    path.valid = true;
    return path;
}

Eigen::VectorXd MyKinoRRT::getNearestConfig(const Eigen::VectorXd& temp, const std::vector<Eigen::VectorXd>& tree) {
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


Eigen::Vector2d MyKinoRRT::getRandomConfig(const amp::KinodynamicProblem2D& problem) {
    double x = problem.x_min + (problem.x_max - problem.x_min) * ((double) rand() / RAND_MAX);
    double y = problem.y_min + (problem.y_max - problem.y_min) * ((double) rand() / RAND_MAX);
    // double y = -3 + (6) * ((double) rand() / RAND_MAX);
    return {x, y};
}

bool MyKinoRRT::subpathCollsionFree(Eigen::VectorXd rand, Eigen::VectorXd near, const amp::KinodynamicProblem2D& problem, double step)
{
    Eigen::VectorXd newEnd = near + (rand - near).normalized() * step;
    return hasCollision(rand, newEnd, problem);
}   
