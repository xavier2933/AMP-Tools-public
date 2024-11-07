// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

// Load problems and map agent for quick testing               0                    1                           2                           3                               4                           5                       6                       7               
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize
        amp::RNG::seed(amp::RNG::randiUnbounded());
    std::vector<double> vals = {1,5,10,15};
    int select = 3;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner(2);
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    std::cout << "seg fault 1" << std::endl;
    HW9::check(path, prob);
        std::cout << "seg fault 2" << std::endl;
    if (path.valid)
        Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation

        std::vector<std::string> labels;

    std::list<std::vector<double>> timeVec;
    std::list<std::vector<double>> validVec;
    std::list<std::vector<double>> lengthVec;
    for(auto& i:vals)
    {
        std::vector<double> times;
        std::vector<double> lengthss;
        std::vector<double> _validVec;
        int validStates = 0;

        for(int j = 0; j < 50; j++)
        {
            MyKinoRRT kino_planner(i);
            auto start = std::chrono::high_resolution_clock::now();
            KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            if(!path.durations.empty()){
                validStates++;
                amp::Path2D newPath;
                std::vector<Eigen::Vector2d> newPathOrSumShitIonEvenKno = path.getWaypoints2D();

                newPath.waypoints = newPathOrSumShitIonEvenKno;
                lengthss.push_back(newPath.length());
                std::cout << "new length " << newPath.length() << std::endl;
                times.push_back((double)duration.count());

            }
        }
        _validVec.push_back(validStates);
        timeVec.push_back(times);
        validVec.push_back(_validVec);
        lengthVec.push_back(lengthss);
                std::string result = "u = " + std::to_string(i);
        labels.push_back(result);
    }

    Visualizer::makeBoxPlot(timeVec, labels, "TIMES", "u", "Times (us)");
    Visualizer::makeBoxPlot(lengthVec, labels, "LENGTHS", "u", "Lengths");
    
    Visualizer::makeBoxPlot(validVec, labels, "VALID", "u", "# valid");

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


    std::vector<Eigen::Vector2d> newPathOrSumShitIonEvenKno = path.getWaypoints2D();
    amp::Path2D newPath;
    newPath.waypoints = newPathOrSumShitIonEvenKno;
    std::cout << "new length " << newPath.length() << std::endl;
    Visualizer::showFigures();
    // HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("xavier.okeefe@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}