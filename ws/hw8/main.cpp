// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyMultiAgentPlanners.h"

using namespace amp;

void timer_example() {
    double startTime;
    amp::Timer timer("timer");
    for (int i=0; i < 5; ++i) {
        startTime = timer.now(TimeUnit::ms);  
        std::cout << "Press any key to continue...\n";
        std::cin.get();
        std::cout << "Time since last run: " << timer.now(TimeUnit::ms) - startTime << std::endl;
    }
    timer.stop();
    std::cout << "Total time since last run: " << Profiler::getTotalProfile("timer") << std::endl;
}

int main(int argc, char** argv) {
    // Run timer example (useful for benchmarking)
    // timer_example();

    // Initialize Workspace 1 with 3 agents
    amp::RNG::seed(amp::RNG::randiUnbounded());
    MultiAgentProblem2D problem = HW8::getWorkspace1(2);
    std::vector<std::vector<Eigen::Vector2d>> collision_states;

    // Solve using a centralized approach
    MyCentralPlanner central_planner;
    path = central_planner.plan(problem);
    bool isValid = HW8::check(path, problem, collision_states);
    Visualizer::makeFigure(problem, path, collision_states);


    std::list<std::vector<double>> timeVec;
    std::list<std::vector<double>> treeSizeVec;
    std::vector<std::string> labels;
    std::vector<double> averageTimes;
    std::vector<double> averageNodes;

    int n = 7;
    // for(int i = 1; i < n; i++)
    // {
    //     std::vector<double> times;
    //     std::vector<double> treeSize;
    //     MultiAgentProblem2D newProblem = HW8::getWorkspace1(i);

    //     for(int j = 0; j < 100; j++)
    //     {
    //         MyDecentralPlanner new_decentral_planner;

    //         auto start = std::chrono::high_resolution_clock::now();
    //         MultiAgentPath2D newPath = new_decentral_planner.plan(newProblem);
    //         auto stop = std::chrono::high_resolution_clock::now();
    //         auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //         times.push_back((double)duration.count());
    //         treeSize.push_back(new_decentral_planner.nodeCount);
    //         std::cout << "Iteration " << j << std::endl;
    //     }
    //     double sum = std::accumulate(times.begin(), times.end(), 0.0);
    //     double avg = sum / times.size();
    //     averageTimes.push_back(avg);

    //     double nodeSum = std::accumulate(treeSize.begin(), treeSize.end(), 0.0);
    //     double nodeAvg = nodeSum / treeSize.size();
    //     averageNodes.push_back(nodeAvg);
    //     // std::cout << "m = " << i << ", avg time: " << avg << " us, avg nodes: " << nodeAvg << std::endl;

    //     timeVec.push_back(times);
    //     treeSizeVec.push_back(treeSize);
    //     std::string result = "m = " + std::to_string(i);
    //     labels.push_back(result);
    // }

    // for(int i = 1; i < n; i++)
    // {
    //     std::cout << "m = " << i << ", avg time: " << averageTimes[i-1] << " us, avg nodes: " << averageNodes[i-1] << std::endl;

    // }
    // Visualizer::makeBoxPlot(timeVec, labels, "TIMES", "m", "Times (us)");
    // Visualizer::makeBoxPlot(treeSizeVec, labels, "TREE SIZE", "m", "NODES");

    // Solve using a decentralized approach
    MyDecentralPlanner decentral_planner;
    collision_states = {{}};
    // HW8::generateAndCheck(decentral_planner, path, problem, collision_states);
    // Visualizer::makeFigure(problem, path, collision_states);

    MultiAgentPath2D path2 = decentral_planner.plan(problem);
    bool isValid2 = HW8::check(path2, problem, collision_states);
    Visualizer::makeFigure(problem, path2, collision_states);

    // Visualize and grade methods
    Visualizer::showFigures();
    HW8::grade<MyCentralPlanner, MyDecentralPlanner>("xavier.okeefe@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple());
    return 0;
}