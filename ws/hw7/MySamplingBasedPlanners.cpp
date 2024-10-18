# include "MySamplingBasedPlanners.h"

#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <functional>
#include <iostream>


// double MyPRM::pathLength = 0.0;

// Need this in order to use map with Eigen::Vector2d
auto vector2dCompare = [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    if (a.x() == b.x()) {
        return a.y() < b.y();
    }
    return a.x() < b.x();
};

std::vector<amp::Node> AStar(amp::Node start_node, amp::Node goal_node, std::shared_ptr<amp::Graph<double>> graphPtr)
{
    std::vector<amp::Node> nodez;
    using QueueEntry = std::pair<double, amp::Node>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open_set;

    std::unordered_map<amp::Node, double> costs;
    std::unordered_map<amp::Node, amp::Node> came_from;

    costs[start_node] = 0.0;

    open_set.push({0, start_node});
    int count = 0;
    double path_cost = 0.0;

    while(!open_set.empty())
    {
        count++;
        auto [current_f_cost, current_node] = open_set.top();
        open_set.pop();

        if (current_node == goal_node) {
            amp::Node node = goal_node;
            while (node != start_node) {
                nodez.push_back(node);
                node = came_from[node];
            }
            nodez.push_back(start_node);
            std::reverse(nodez.begin(), nodez.end());
            path_cost = costs[goal_node];
            // std::cout << "Iterations: " << count << std::endl;
            return nodez;
        }

        for (const auto& neighbor : graphPtr->children(current_node)) {
            double edge_cost = graphPtr->outgoingEdges(current_node)[&neighbor - &graphPtr->children(current_node)[0]];

            double temp_cost = costs[current_node] + edge_cost;

        if (costs.find(neighbor) == costs.end() || temp_cost < costs[neighbor]) {
            costs[neighbor] = temp_cost;
            came_from[neighbor] = current_node;

            // f_cost = costs + heuristic(neighbor)
            // double f_cost = temp_cost + heuristic(neighbor);
            double f_cost = temp_cost;

            // std::cout << "for node " << neighbor << " cost: " << f_cost << " edge cost " << edge_cost << std::endl;
            open_set.push({f_cost, neighbor});
        }
        }


    }

    // std::cout << "No path found to the goal node." << std::endl;

    return nodez;
}


amp::Path2D MyPRM::plan(const amp::Problem2D& problem)
{
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);
    return path;
}
// Implement your PRM algorithm here
amp::Path2D MyPRM::plan2(const amp::Problem2D& problem, int n, double r) {
    amp::Path2D path;
    getGraph(problem, n, r);
    amp::Node start_node = graphPtr->nodes()[0];
    amp::Node end_node = graphPtr->nodes().back();
    // std::cout << "START NODE " << nodes[start_node] << " END NODE " << nodes[end_node] << std::endl;
    // auto res = temp.outgoingEdges();
    // std::cout << "NODE 1 " << graphPtr->children(start_node)[0] << " WITH EDGES " << graphPtr->outgoingEdges(start_node)[0] << std::endl;
    std::vector<amp::Node> res = AStar(start_node, end_node, graphPtr);
    if(res.empty())
    {
        // std::cout << "empty path " << n << " r " << r << std::endl;
        return path;
    }
    path.waypoints.push_back(problem.q_init);

    for(auto & node : res)
    {
        path.waypoints.push_back(nodes[node]);
    }
    path.waypoints.push_back(problem.q_goal);
    // std::cout << "path length: " << path.length() << std::endl;
    pathLength += path.length();
    path = smoothPath(path, problem);
    // std::cout << "smoothed path length: " << path.length() << std::endl;

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

amp::Path2D MyPRM::smoothPath(amp::Path2D& original_path, const amp::Problem2D& problem)
{
    if(original_path.waypoints.empty()) return original_path;
    // std::cout << "Running smoothed path" << std::endl;
    amp::Path2D smoothedPath;
    smoothedPath.waypoints.push_back(original_path.waypoints[0]);
    int i = 0;
    while(i < original_path.waypoints.size()-1)
    {
        int j = original_path.waypoints.size() - 1;
        while(j> i+1)
        {
            if(!hasCollision(original_path.waypoints[i], original_path.waypoints[j], problem))
            {
                break;
            }
            j--;
        }
        smoothedPath.waypoints.push_back(original_path.waypoints[j]);
        i = j;
    }
    return smoothedPath;
    
}

Eigen::Vector2d MyPRM::getRandomConfig(const amp::Problem2D& problem) {
    double x = problem.x_min + (problem.x_max - problem.x_min) * ((double) rand() / RAND_MAX);
    double y = problem.y_min + (problem.y_max - problem.y_min) * ((double) rand() / RAND_MAX);
    // double x = -1 + (11 + 1) * ((double) rand() / RAND_MAX);

    // double y = -3 + (6) * ((double) rand() / RAND_MAX);

    return {x, y};
}

void MyPRM::getGraph(const amp::Problem2D& problem, int n, double r)
{

    // double r = 2.0;
    // int n = 500;

    std::vector<Eigen::Vector2d> validSamples;
    validSamples.push_back(problem.q_init);

    // for (int i = 0; i < n; ++i) {
    //     Eigen::Vector2d sample = getRandomConfig(problem);
    //     if (!isInCollision(problem, sample)) {
    //         validSamples.push_back(sample);
    //     } else {
    //         // std::cout << "NOT ADDINF POINT " << sample << std::endl; 
    //     }
    // }
    while(validSamples.size() < n)
    {
        Eigen::Vector2d sample = getRandomConfig(problem);
        if (!isInCollision(problem, sample)) {
            validSamples.push_back(sample);

        } else {
            // std::cout << "NOT ADDINF POINT " << sample << std::endl; 
        }
    }
    // std::cout << "VALID SAMPLES SIZE " << validSamples.size() << std::endl;
    validSamples.push_back(problem.q_goal);

    // Step 2: Add valid configurations as nodes in the graph
    for (amp::Node i = 0; i < validSamples.size(); ++i) {
        nodes[i] = validSamples[i];  // Map node index to its position
    }

    // Step 3: Connect valid nodes within radius r, checking for collisions along the path
    for (amp::Node i = 0; i < validSamples.size(); ++i) {
        for (amp::Node j = i + 1; j < validSamples.size(); ++j) {
            double dist = (validSamples[i] - validSamples[j]).norm();
            if (dist <= r && !hasCollision(validSamples[i], validSamples[j], problem)) {
                graphPtr->connect(i, j, dist);  // Connect nodes with no obstacle in between
                graphPtr->connect(j,i,dist);
            } else {
            // std::cout << "NOT ADDINF POINT " << validSamples[i] << std::endl; 
        }
        }
    }

    // Optional: Print the graph structure
    // graphPtr->print();
}

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
    int n = 10000;
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
            if((newEnd - problem.q_goal).norm() < 0.2) {
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
    } else {
        std::cout << "RRT did not find a solution" << std::endl;
    }

    return path;
}
