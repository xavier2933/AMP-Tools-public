#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <functional>
#include <iostream>

#include "MyAStar.h"


using PriorityQueue = std::priority_queue<std::pair<double, amp::Node>,
                                          std::vector<std::pair<double, amp::Node>>,
                                          std::greater<>>;
void printOpenSet(PriorityQueue open_set) {
    std::cout << "Contents of open_set: ";
    while (!open_set.empty()) {
        auto [f_score, node] = open_set.top();
        std::cout << "[Node: " << node << ", f-score: " << f_score << "] ";
        open_set.pop();
    }
    std::cout << std::endl;
}

MyAStarAlgo::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {
    std::cout << "Starting A* Graph Search: Init --> goal | " << problem.init_node << " --> " << problem.goal_node << std::endl;

    GraphSearchResult result = {false, {}, 0.0};
    using QueueEntry = std::pair<double, amp::Node>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open_set;

    std::unordered_map<amp::Node, double> costs;
    std::unordered_map<amp::Node, amp::Node> came_from;

    amp::Node start_node = problem.init_node;
    amp::Node goal_node = problem.goal_node;

    costs[start_node] = 0.0;

    // for A*
    open_set.push({heuristic(start_node), start_node});
    //For Dijkstra
    // open_set.push({0, start_node});

    int count = 0;

    while (!open_set.empty()) {
        // std::cout << "open set: " << std::endl;
        // printOpenSet(open_set);
        // std::cout << std::endl;
        count++;
         auto [current_f_cost, current_node] = open_set.top();
        // std::cout << "popping " << current_node << std::endl;
        open_set.pop();

        if (current_node == goal_node) {
            amp::Node node = goal_node;
            while (node != start_node) {
                result.node_path.push_back(node);
                node = came_from[node];
            }
            result.node_path.push_back(start_node);
            std::reverse(result.node_path.begin(), result.node_path.end());
            result.path_cost = costs[goal_node];
            result.print();
            result.success = true;
            std::cout << "Iterations: " << count << std::endl;
            return result;
        }

        for (const auto& neighbor : problem.graph->children(current_node)) {
            double edge_cost = problem.graph->outgoingEdges(current_node)[&neighbor - &problem.graph->children(current_node)[0]];

            double temp_cost = costs[current_node] + edge_cost;

        if (costs.find(neighbor) == costs.end() || temp_cost < costs[neighbor]) {
            costs[neighbor] = temp_cost;
            came_from[neighbor] = current_node;

            // f_cost = costs + heuristic(neighbor)
            double f_cost = temp_cost + heuristic(neighbor);
            // double f_cost = temp_cost;

            // std::cout << "for node " << neighbor << " cost: " << f_cost << " edge cost " << edge_cost << std::endl;
            open_set.push({f_cost, neighbor});
        }
        }
    }

    std::cout << "No path found to the goal node." << std::endl;
    result.print();
    return result;
}
// namespace amp
