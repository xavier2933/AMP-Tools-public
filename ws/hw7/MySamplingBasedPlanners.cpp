# include "MySamplingBasedPlanners.h"

#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <functional>
#include <iostream>

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
            std::cout << "Iterations: " << count << std::endl;
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

    std::cout << "No path found to the goal node." << std::endl;

    return nodez;
}

// Implement your PRM algorithm here
amp::Path2D MyPRM::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    amp::Node start_node = graphPtr->nodes()[0];
    amp::Node end_node = graphPtr->nodes().back();
    std::cout << "START NODE " << nodes[start_node] << " END NODE " << nodes[end_node] << std::endl;
    // auto res = temp.outgoingEdges();
    std::cout << "NODE 1 " << graphPtr->children(start_node)[0] << " WITH EDGES " << graphPtr->outgoingEdges(start_node)[0] << std::endl;
    path.waypoints.push_back(problem.q_init);
    std::vector<amp::Node> res = AStar(start_node, end_node, graphPtr);
    for(auto & node : res)
    {
        path.waypoints.push_back(nodes[node]);
    }
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

Eigen::Vector2d MyPRM::getRandomConfig(const amp::Problem2D& problem) {
    double x = problem.x_min + (problem.x_max - problem.x_min) * ((double) rand() / RAND_MAX);
    double y = problem.y_min + (problem.y_max - problem.y_min) * ((double) rand() / RAND_MAX);
    // double y = -3 + (6) * ((double) rand() / RAND_MAX);

    return {x, y};
}

void MyPRM::getGraph(const amp::Problem2D& problem)
{

    double r = 1;
    int n = 200;

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
    std::cout << "VALID SAMPLES SIZE " << validSamples.size() << std::endl;
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
            } else {
            // std::cout << "NOT ADDINF POINT " << validSamples[i] << std::endl; 
        }
        }
    }

    // Optional: Print the graph structure
    // graphPtr->print();
}



// Implement your RRT algorithm here
amp::Path2D MyRRT::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    // nodes[0] = problem.q_init;

    path.waypoints.push_back(problem.q_goal);
    return path;
}