#include "MyKinoRRT.h"
#include <Eigen/Geometry> // Include for Rotation2D
#include <ctime>

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
        double r = 0.25;
        // Define the unicycle model's dynamics
        dxdt(0) = v * r * cos(theta); // dx/dt = u_sigma * cos(theta)
        dxdt(1) = v * r * sin(theta); // dy/dt = u_sigma * sin(theta)
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
        state(2) = std::atan2(std::sin(state(2)), std::cos(state(2)));
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
        dxdt2(3) = control(0);
        dxdt2(4) = control(1);

        return dxdt2;
    };

    // Calculate w1, w2, w3, and w4
    Eigen::VectorXd w1 = dynamics2(state, control);
    Eigen::VectorXd w2 = dynamics2(state + 0.5 * dt * w1, control);
    Eigen::VectorXd w3 = dynamics2(state + 0.5 * dt * w2, control);
    Eigen::VectorXd w4 = dynamics2(state + dt * w3, control);

    // Update the state using RK4 formula
    // std::cout << "State before " << state.transpose() << std::endl;
    state = state + (dt / 6.0) * (w1 + 2.0 * w2 + 2.0 * w3 + w4);
            state(2) = std::atan2(std::sin(state(2)), std::cos(state(2)));

}

void MySimpleCar::propagate(Eigen::VectorXd& state, Eigen::VectorXd& control, double dt)
{
    auto dynamics3 = [this](const Eigen::VectorXd& state, const Eigen::VectorXd& control) -> Eigen::VectorXd {
        Eigen::VectorXd dxdt3(5);
        double l = 5.0;
        double w = 2.0;
        double sigma = state(3);
        double r = 0.25;
        Eigen::VectorXd dxdt(3);
        double theta = state(2);
        double v = control(0);   // Linear velocity
        double omega = control(1); // Angular velocity

        // Define the unicycle model's dynamics
        dxdt3(0) = state(3) * cos(theta);
        dxdt3(1) = state(3) * sin(theta); // dy/dt = v * sin(theta)
        dxdt3(2) = (state(3)/l) * tan(state(4));          // dtheta/dt = omega
        dxdt3(3) = control(0);
        dxdt3(4) = control(1);

        return dxdt3;
    };

    // Calculate w1, w2, w3, and w4
    Eigen::VectorXd w1 = dynamics3(state, control);
    Eigen::VectorXd w2 = dynamics3(state + 0.5 * dt * w1, control);
    Eigen::VectorXd w3 = dynamics3(state + 0.5 * dt * w2, control);
    Eigen::VectorXd w4 = dynamics3(state + dt * w3, control);

    // Update the state using RK4 formula
    // std::cout << "State before " << state.transpose() << std::endl;
    state = state + (dt / 6.0) * (w1 + 2.0 * w2 + 2.0 * w3 + w4);
}

double pointToSegmentDistance(const Eigen::Vector2d& p, const Eigen::Vector2d& v, const Eigen::Vector2d& w) {
    // Projection of point p onto the line segment vw
    Eigen::Vector2d segment = w - v;
    double segmentLengthSquared = segment.squaredNorm();
    if (segmentLengthSquared == 0.0) return (p - v).norm();  // v == w case

    // Projection factor t of point p onto the line vw
    double t = ((p - v).dot(segment)) / segmentLengthSquared;
    t = std::clamp(t, 0.0, 1.0);  // Clamp to segment boundaries

    // Find the projection point on the segment
    Eigen::Vector2d projection = v + t * segment;
    return (p - projection).norm();  // Distance from p to the segment
}

bool isInCollision(const amp::KinodynamicProblem2D& environment, const Eigen::Vector2d& point, double cushion = 0.05) {
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

            // Check if the point is within the cushion distance of this edge
            if (pointToSegmentDistance(point, vertices[i], vertices[j]) < cushion) {
                return true;  // Point is within the buffer zone of this edge
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
            return true; // collision
        }
    }
    
    return false;
}



// Helper function to check if two line segments (v1-v2 and o1-o2) intersect
bool checkEdgeIntersection(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2,
                           const Eigen::Vector2d& o1, const Eigen::Vector2d& o2) {
    auto crossProduct = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
        return a.x() * b.y() - a.y() * b.x();
    };

    Eigen::Vector2d r = v2 - v1;
    Eigen::Vector2d s = o2 - o1;
    Eigen::Vector2d w = o1 - v1;

    double rsCross = crossProduct(r, s);
    double wCrossR = crossProduct(w, r);
    double wCrossS = crossProduct(w, s);

    // Check if they are parallel and non-collinear
    if (std::abs(rsCross) < 1e-6) {
        return false; // Parallel or collinear segments
    }

    // Calculate intersection t and u values
    double t = crossProduct(w, s) / rsCross;
    double u = crossProduct(w, r) / rsCross;

    return (t >= 0 && t <= 1 && u >= 0 && u <= 1); // True if intersection exists within segment bounds
}
bool isPolygonInCollision(const amp::KinodynamicProblem2D& environment, const std::vector<Eigen::Vector2d>& vehicleVertices) {
    // Check each obstacle in the environment
    for (const auto& obstacle : environment.obstacles) {
        const std::vector<Eigen::Vector2d>& obstacleVertices = obstacle.verticesCCW();
        int obstacleSize = obstacleVertices.size();

        // 1. Check if any edge of the vehicle polygon intersects any edge of the obstacle polygon
        for (size_t i = 0; i < vehicleVertices.size(); i++) {
            Eigen::Vector2d v1 = vehicleVertices[i];
            Eigen::Vector2d v2 = vehicleVertices[(i + 1) % vehicleVertices.size()]; // Wrap to start

            for (int j = 0; j < obstacleSize; j++) {
                Eigen::Vector2d o1 = obstacleVertices[j];
                Eigen::Vector2d o2 = obstacleVertices[(j + 1) % obstacleSize];

                if (checkEdgeIntersection(v1, v2, o1, o2)) {
                    return true; // Edge intersection found
                }
            }
        }

        // 2. Check if any vertex of the vehicle polygon is inside the obstacle
        for (const auto& vertex : vehicleVertices) {
            if (isInCollision(environment, vertex)) {
                return true; // Vertex inside an obstacle
            }
        }
    }
    return false; // No collision found
}

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

std::vector<Eigen::Vector2d> getVertices(Eigen::VectorXd state, double length, double width, bool car) {
    double x = state[0];
    double y = state[1];
    double theta = state[2]; // Orientation in radians

    double half_width = width / 2.0;

    std::vector<Eigen::Vector2d> vertices(4);

    // Rotation matrix for the orientation theta
    Eigen::Rotation2D<double> rotation(theta);

    // Define vertices based on the rear axle position as the origin
    vertices[0] = rotation * Eigen::Vector2d(0, -half_width) + Eigen::Vector2d(x, y);         // Rear-left
    vertices[1] = rotation * Eigen::Vector2d(length, -half_width) + Eigen::Vector2d(x, y);    // Front-left
    vertices[2] = rotation * Eigen::Vector2d(length, half_width) + Eigen::Vector2d(x, y);     // Front-right
    vertices[3] = rotation * Eigen::Vector2d(0, half_width) + Eigen::Vector2d(x, y);          // Rear-right

    vertices = sortPts(vertices);
        // std::cout << "Vertices " << vertices[0].transpose() << ", " << vertices[1].transpose()  << ", " << vertices[2].transpose()  << ", "<< vertices[3].transpose()  << std::endl;
    return vertices;
}

        // std::cout << "Vertices " << vertices[0].transpose() << ", " << vertices[1].transpose()  << ", " << vertices[2].transpose()  << ", "<< vertices[3].transpose()  << std::endl;


bool checkBounds(Eigen::VectorXd state, std::vector<std::pair<double, double>> goal)
{
    for(int i = 0; i < state.size(); i++)
    {
        if(state[i] < goal[i].first || state[i] > goal[i].second) return false;
    }
    return true;
} 

bool checkGoalBounds(Eigen::VectorXd state, std::vector<std::pair<double, double>> goal)
{
    for(int i = 0; i < state.size(); i++)
    {
        if(state[i] < (goal[i].first + 0.1) || state[i] > (goal[i].second - 0.1)) return false;
    }
    return true;
} 
 
bool MyKinoRRT::checkCarPath(const amp::KinodynamicProblem2D& problem, std::vector<Eigen::Vector2d> prev, std::vector<Eigen::Vector2d> curr)
{
    bool check;
    if(prev.size() != curr.size()) return false;
    for(int i = 0; i < prev.size(); i++)
    {
        if(!subpathCollsionFree(curr[i], prev[i], problem, 0.0)) return false;
    }
    return true; // if no collision
}

amp::KinoPath MyKinoRRT::plan(const amp::KinodynamicProblem2D& problem, amp::DynamicAgent& agent) {
    amp::KinoPath path;
    Eigen::VectorXd state = problem.q_init; // inital state Xd
    std::vector<Eigen::VectorXd> controls;
    std::vector<double> timeSteps;
    Eigen::VectorXd tempState(3);
    tempState << 17,6,0;
    Eigen::VectorXd tempState2(3);
    tempState2 << 12.5,6,0;
    // std::cout << "Agent dimensions " << problem.agent_dim.length << "width " << problem.agent_dim.width << std::endl;
bool pathChill;
    std::cout << "U samples " << samplesToTake << std::endl;
    // std::vector<Eigen::Vector2d> vert2 = getVertices(tempState, 5, 2, false);
    // std::vector<Eigen::Vector2d> prevVert2 = getVertices(tempState2, 5, 2, false);


    // bool pathChill = checkCarPath(problem, vert2, prevVert2);

    // std::cout << "Path collisions? " << pathChill << std::endl;


    Eigen::VectorXd goal = Eigen::VectorXd::Zero(problem.q_init.size()); // Initialize a 2-dimensional VectorXd

    for(int i = 0; i < problem.q_goal.size(); i++)
    {
        goal(i) = problem.q_goal[i].first + (problem.q_goal[i].second - problem.q_goal[i].first)/2.0;
    }

    int a = 0;
    for(auto& point : problem.q_goal)
    {
        a++;
        // std::cout << "dimension " << a << " is " << point.first << ", " << point.second << std::endl;
    }
    Eigen::VectorXd temp;

    Eigen::Vector2d near;
    Eigen::VectorXd nearXd = Eigen::VectorXd::Zero(problem.q_init.size());
    Eigen::VectorXd newEndXd = Eigen::VectorXd::Zero(problem.q_init.size());

    std::vector<Eigen::VectorXd> tree;
    std::map<Eigen::VectorXd, Eigen::VectorXd, decltype(vectorXdCompare)> prevMap(vectorXdCompare); // current, node before
    std::map<Eigen::VectorXd, Eigen::VectorXd, decltype(vectorXdCompare)> controlMap(vectorXdCompare); // current, node before
    std::map<Eigen::VectorXd, Eigen::VectorXd, decltype(vectorXdCompare)> prevCarPoseMap(vectorXdCompare);
    tree.push_back(state);
    // path.waypoints.push_back(state);
    int count = 0;
    int n = 50000;
    double step = 0.25;
    int goalBiasCount = 0;
    bool goalFound = false;
    Eigen::VectorXd goalNode;
    double timeStep = 0.5;
    int u = 1;

    while(count < n) {

        if(goalBiasCount == 20) { // do sampling in workspace 2-d
            temp = goal;
            goalBiasCount = 0;
        } else {
            temp = getRandomConfig(problem); //statesample
        }
        nearXd = getNearestConfig(temp, tree); // get nearest state
        // std::cout << "temp " << temp.transpose() << "nearest " << nearXd.transpose(); 

        std::vector<Eigen::VectorXd> controlVector;
        std::vector<Eigen::VectorXd> stateVector;
        Eigen::VectorXd control;
        for(int i = 0; i < u; i++)
        {
            Eigen::VectorXd controlSample = 0.5 * Eigen::VectorXd::Random(problem.u_bounds.size());
            controlVector.push_back(controlSample);
            Eigen::VectorXd uSamplePose = nearXd;
            agent.propagate(uSamplePose, controlSample, timeStep);
            stateVector.push_back(uSamplePose);
            // std::cout << "Sample " << uSamplePose.transpose() << " from control " << controlSample.transpose() << std::endl;
        }
        int controlIndex = 0;
        double bestDist = 1000000000;
        for(int i = 0; i < u; i++)
        {
            if((goal - stateVector[i]).norm() < bestDist)
            {
                bestDist = (goal - stateVector[i]).norm();
                controlIndex = i;
            }
        }
        // Eigen::VectorXd newEnd = nearXd;
        // // std::cout << "nearXd " << nearXd.transpose();

        // agent.propagate(newEnd, control, timeStep); // generateLocalTrajectory

        // std::cout << " Controlled to " << newEnd.transpose() << std::endl;
        Eigen::VectorXd newEnd = stateVector[controlIndex];
        control = controlVector[controlIndex];
        // std::cout << "Choosing " << newEnd.transpose() << " With control " << control.transpose() << std::endl << std::endl;
        goalBiasCount++;
        Eigen::Vector2d temp2d(newEnd[0],newEnd[1]);
        Eigen::Vector2d near2d(nearXd[0], nearXd[1]);

        if(!checkBounds(newEnd, problem.q_bounds)) // outsid eof bounds this logic is weird
        {
            continue;
        }
        // Eigen::Vector2d temp2d(0.5,1.5);
        // Eigen::Vector2d near2d(2.5, 1.5);
        // std::cout << "subpathCollisionFree " << subpathCollsionFree(near2d, temp2d, problem, step) << std::endl;
        if(problem.agent_type == amp::AgentType::SimpleCar) 
        {
            std::vector<Eigen::Vector2d> vert = getVertices(newEnd, 5, 2, false);
            std::vector<Eigen::Vector2d> prevVert = getVertices(nearXd, 5, 2, false);

            bool carChill = isPolygonInCollision(problem, vert);

            pathChill = checkCarPath(problem, prevVert, vert);
            // std::cout << "collisons?" << isPolygonInCollision(problem, vert) << std::endl;
            if(!carChill && pathChill) { //check if subtrajectory is valid
                Eigen::Vector2d problemCorner(13,3);
                bool problemCar = false;
                for(auto& vertex:vert)
                {
                    if((vertex - problemCorner).norm() < 0.2)
                    {
                        problemCar = true;
                    }
                }
                if(problemCar) continue;
                tree.push_back(newEnd);

                prevMap[newEnd] = nearXd;
                prevCarPoseMap[newEnd] = nearXd;
                controlMap[newEnd] = control;
                Eigen::Vector2d goal2d(goal[0],goal[1]);
                Eigen::Vector2d newEnd2d(newEnd[0], newEnd[1]);
                
                // Check if we've reached the goal
                if(checkBounds(newEnd, problem.q_goal)) {
                // if((newEnd - goal).norm() < 0.25) {

                    std::cout << "goal found " << std::endl;
                    goalFound = true;
                    goalNode = newEnd;
                    break;
                }
            }
        } else {
            if(subpathCollsionFree(temp2d, near2d, problem, step)) { //check if subtrajectory is valid
                tree.push_back(newEnd);

                prevMap[newEnd] = nearXd;
                controlMap[newEnd] = control;
                Eigen::Vector2d goal2d(goal[0],goal[1]);
                Eigen::Vector2d newEnd2d(newEnd[0], newEnd[1]);
                
                // Check if we've reached the goal
                if(checkBounds(newEnd, problem.q_goal)) {
                // if((newEnd - goal).norm() < 0.25) {

                    std::cout << "goal found " << std::endl;
                    goalFound = true;
                    goalNode = newEnd;
                    break;
                }
            }
        }
        count++;
    }

    if (goalFound) {
        // Backtrack from the goal to the start using the map
        Eigen::VectorXd currentNode = goalNode;
        // path.waypoints.push_back(problem.q_goal);
        int newCount = 0;
        Eigen::VectorXd gol= Eigen::VectorXd::Zero(problem.q_init.size());
        gol[0] = goal[0];
        gol[1] = goal[1];
        // path.waypoints.push_back(goal);
        // timeSteps.push_back(0.0);
        // controls.push_back(Eigen::VectorXd::Zero(problem.q_init.size()));
        while (currentNode != problem.q_init) {

            path.waypoints.push_back(currentNode);
            // std::cout << "pushing out " << currentNode.transpose() << std::endl;
            controls.push_back(controlMap[currentNode]);
            timeSteps.push_back(timeStep);
            currentNode = prevMap[currentNode];  // Move to the parent node
            newCount++;
            if (newCount > n)
            {
                std::cout << "breaking early" << std::endl;
                break;
            }
        }

        path.waypoints.push_back(problem.q_init);  // Finally, add the start
        timeSteps.push_back(timeStep);
        controls.push_back(Eigen::VectorXd::Zero(problem.u_bounds.size()));
        path.controls = controls;

        path.durations = timeSteps;
        // std::cout << "waypoints " << path.waypoints.size() << " controls " << path.controls.size() << " durations " << path.durations.size() << std::endl;



        std::reverse(path.waypoints.begin(), path.waypoints.end());  // Reverse to get the path from start to goal
        std::reverse(path.controls.begin(), path.controls.end());  // Reverse to get the path from start to goal

        // std::cout << "final state " << path.waypoints.back() << std::endl;
            // std::cout << "path length" << path.length() << std::endl;

        for(int i = 0; i < path.waypoints.size(); i++)
        {
            // std::cout << "Point " << path.waypoints[i].transpose() << " with control " << path.controls[i].transpose() << " and duration " << path.durations[i] << std::endl;
        }
        // std::cout << "Path length " << path.length();
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }

    path.valid = true;
            // std::cout <<"seg fault" << std::endl;

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


Eigen::VectorXd MyKinoRRT::getRandomConfig(const amp::KinodynamicProblem2D& problem) {
    Eigen::VectorXd q_random(problem.q_bounds.size());
    for(int i = 0; i < problem.q_bounds.size(); i++)
    {
        q_random(i) = problem.q_bounds[i].first + (problem.q_bounds[i].second - problem.q_bounds[i].first) * ((double) rand() / RAND_MAX);
    }
    // double y = -3 + (6) * ((double) rand() / RAND_MAX);
    return q_random;
}

bool MyKinoRRT::subpathCollsionFree(Eigen::VectorXd newEnd, Eigen::VectorXd near, const amp::KinodynamicProblem2D& problem, double step)
{
    // Eigen::VectorXd newEnd = near + (rand - near).normalized() * step;
    if(newEnd[0] < problem.x_min || newEnd[0] > problem.x_max) return false;
    if(near[0] < problem.x_min || near[0] > problem.x_max) return false;

    if(newEnd[1] < problem.y_min || newEnd[1] > problem.y_max) return false;
    if(near[1] < problem.y_min || near[1] > problem.y_max) return false;

    return !hasCollision(near, newEnd, problem);
}   