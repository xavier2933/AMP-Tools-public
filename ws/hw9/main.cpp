// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW9.h"
#include "hw/HW2.h"
#include "MyKinoRRT.h"

using namespace amp;

KinodynamicProblem2D createMailDeliveryProblem() {
    KinodynamicProblem2D problem;

    // Agent type and configuration
    problem.agent_type = AgentType::SimpleCar;
    problem.isPointAgent = false; // Rectangle agent
    problem.agent_dim = {2.0, 1.0}; // Length = 2, Width = 1

    // Initial state
    problem.q_init = Eigen::VectorXd(5); // State = [x, y, θ, v]
    problem.q_init << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Goal region: [x_min, x_max], [y_min, y_max], [θ_min, θ_max], [v_min, v_max]
    problem.q_goal = {
        {8.0, 9.0},   // x range
        {8.0, 9.0},    // y range
        {4*M_PI / 9, 5*M_PI / 9}, // θ range
        {0.0, 0.5},     // v range
        {-M_PI / 6, M_PI / 6} // φ range
    };

    // State space bounds: [x_min, x_max], [y_min, y_max], [θ_min, θ_max], [v_min, v_max], [φ_min, φ_max]
    problem.q_bounds = {
        {-1.0, 11.0},    // x range
        {-1.0, 10.0},    // y range
        {-M_PI, M_PI},  // θ range
        {-1.0 / 6.0, 0.5}, // v range
        {-M_PI / 6, M_PI / 6} // φ range
    };

    // Control space bounds: [u1_min, u1_max], [u2_min, u2_max]
    problem.u_bounds = {
        {-1.0 / 6.0, 1.0 / 6.0}, // Acceleration bounds
        {-M_PI / 6, M_PI / 6}    // Steering turn rate bounds
    };

    // Control duration bounds
    problem.dt_bounds = {0.1, 0.5}; // Min and max control duration

    // Cartesian dimensions
    problem.isDimCartesian = {true, true, false, false}; // [x, y, θ, v]

    // Obstacles (workspace)
std::vector<Eigen::Matrix<double, 2, 1>> obstacle1{
    Eigen::Vector2d(2.75, -1.25), Eigen::Vector2d(5.25, -1.25),
    Eigen::Vector2d(5.25, 2.25), Eigen::Vector2d(2.75, 2.25)
};

std::vector<Eigen::Matrix<double, 2, 1>> obstacle2{
    Eigen::Vector2d(6.75, 2.75), Eigen::Vector2d(9.25, 2.75),
    Eigen::Vector2d(9.25, 5.25), Eigen::Vector2d(6.75, 5.25)
};

std::vector<Eigen::Matrix<double, 2, 1>> obstacle3{
    Eigen::Vector2d(0.75, 3.75), Eigen::Vector2d(4.25, 3.75),
    Eigen::Vector2d(4.25, 6.25), Eigen::Vector2d(0.75, 6.25)
};

std::vector<Eigen::Matrix<double, 2, 1>> obstacle4{
    Eigen::Vector2d(4.75, 6.75), Eigen::Vector2d(6.25, 6.75),
    Eigen::Vector2d(6.25, 10.25), Eigen::Vector2d(4.75, 10.25)
};

std::vector<Eigen::Matrix<double, 2, 1>> PUDDLE{
    Eigen::Vector2d(0.0, 7.0), Eigen::Vector2d(4.0, 7.0),
    Eigen::Vector2d(4.0, 11.0), Eigen::Vector2d(0.0, 11.0)
};

problem.obstacles = {
    amp::Polygon(obstacle1),
    amp::Polygon(obstacle2),
    amp::Polygon(obstacle3),
        // amp::Polygon(PUDDLE),

    amp::Polygon(obstacle4)
};


    return problem;
}


void writeDataToCSV(const std::vector<Eigen::VectorXd>& controls, const std::vector<double>& timeStepsVec, const std::string& filename) {
    // Check if controls and timeStepsVec have the same size
    if (controls.size() != timeStepsVec.size()) {
        std::cerr << "Error: controls and timeStepsVec must have the same length." << std::endl;
        return;
    }

    // Open the file stream
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }

    // Write CSV header (optional)
    file << "TimeStep";
    for (int i = 0; i < controls[0].size(); ++i) {
        file << ",Control" << i;
    }
    file << "\n";

    // Write data to CSV
    for (size_t i = 0; i < controls.size(); ++i) {
        file << timeStepsVec[i]; // Write time step
        for (int j = 0; j < controls[i].size(); ++j) {
            file << "," << controls[i][j]; // Write each element in the control vector
        }
        file << "\n";
    }

    // Close the file
    file.close();
    std::cout << "Data successfully written to " << filename << std::endl;
}

// Load problems and map agent for quick testing               0                    1                           2                           3                               4                           5                       6                       7               
std::vector<KinodynamicProblem2D> problems = {HW9::getStateIntProblemWS1(), HW9::getStateIntProblemWS2(), HW9::getFOUniProblemWS1(), HW9::getFOUniProblemWS2(), HW9::getSOUniProblemWS1(), HW9::getSOUniProblemWS2(), HW9::getCarProblemWS1(), HW9::getParkingProblem(), createMailDeliveryProblem()};
std::unordered_map<AgentType, std::function<std::shared_ptr<amp::DynamicAgent>()>> agentFactory = {
    {AgentType::SingleIntegrator, []() { return std::make_shared<MySingleIntegrator>(); }},
    {AgentType::FirstOrderUnicycle, []() { return std::make_shared<MyFirstOrderUnicycle>(); }},
    {AgentType::SecondOrderUnicycle, []() { return std::make_shared<MySecondOrderUnicycle>(); }},
    {AgentType::SimpleCar, []() { return std::make_shared<MySimpleCar>(); }}
};

int main(int argc, char** argv) {
    // Select problem, plan, check, and visualize
        amp::RNG::seed(amp::RNG::randiUnbounded());
            std::vector<double> vals = {5};

    int select = 8;
    KinodynamicProblem2D prob = problems[select];
    MyKinoRRT kino_planner(2);
    KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
    std::cout << "seg fault 1" << std::endl;
    HW9::check(path, prob);
        std::cout << "seg fault 2" << std::endl;
    if (path.valid)
        Visualizer::makeFigure(prob, path, false); // Set to 'true' to render animation

    // std::vector<Eigen::VectorXd> controls = path.controls;
    // std::vector<double> timeStepsVec = path.durations;
    // writeDataToCSV(controls, timeStepsVec, "/home/xavier/motion_planning/output.csv");


        std::vector<std::string> labels;

    std::list<std::vector<double>> timeVec;
    std::vector<double> temp;
    temp.push_back(0.0);

    std::list<std::vector<double>> validVec;
    std::list<std::vector<double>> lengthVec;

    // timeVec.push_back(temp);
    // validVec.push_back(temp);
    // lengthVec.push_back(temp);
//     for(auto& i:vals)
//     {
//         std::vector<double> times;
//         std::vector<double> lengthss;
//         std::vector<double> _validVec;
//         int validStates = 0;

//         for(int j = 0; j < 100; j++)
//         {
//             MyKinoRRT kino_planner(i);
//             auto start = std::chrono::high_resolution_clock::now();
//             KinoPath path = kino_planner.plan(prob, *agentFactory[prob.agent_type]());
//             auto stop = std::chrono::high_resolution_clock::now();
//             auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
//             if(!path.durations.empty()){
//                 validStates++;
//                 amp::Path2D newPath;
//                 std::vector<Eigen::Vector2d> newPathOrSumShitIonEvenKno = path.getWaypoints2D();

//                 newPath.waypoints = newPathOrSumShitIonEvenKno;
//                 lengthss.push_back(newPath.length());
//                 std::cout << "new length " << newPath.length() << std::endl;
//                 times.push_back((double)duration.count());

//             }
//         }
//         _validVec.push_back(validStates);
//         // timeVec.push_back(times);
//         // validVec.push_back(_validVec);
//         // lengthVec.push_back(lengthss);

//         if (!times.empty()) {
//     timeVec.push_back(times);
// } else {
//     timeVec.push_back(temp);
// }

// if (!lengthss.empty()) {
//     lengthVec.push_back(lengthss);
// } else {
//     lengthVec.push_back(temp);
// }

// if (!_validVec.empty()) {
//     validVec.push_back(_validVec);
// } else {
//     validVec.push_back(temp);
// }
//                 std::string result = "u = " + std::to_string(i);
//         labels.push_back(result);
//     }
//     std::cout << "times " << std::endl;

//     std::cout << "timeVec size: " << timeVec.size() << std::endl;
// std::cout << "lengthVec size: " << lengthVec.size() << std::endl;
// std::cout << "validVec size: " << validVec.size() << std::endl;
// std::cout << "labels size: " << labels.size() << std::endl;



//     Visualizer::makeBoxPlot(timeVec, labels, "TIMES", "u", "Times (us)");
//     std::cout << "1" << std::endl;
//     Visualizer::makeBoxPlot(lengthVec, labels, "LENGTHS", "u", "Lengths");
//         std::cout << "1" << std::endl;

//     Visualizer::makeBoxPlot(validVec, labels, "VALID", "u", "# valid");



    // std::vector<Eigen::Vector2d> newPathOrSumShitIonEvenKno = path.getWaypoints2D();
    //     std::cout << "1" << std::endl;

    // amp::Path2D newPath;
    // newPath.waypoints = newPathOrSumShitIonEvenKno;
    // std::cout << "new length " << newPath.length() << std::endl;
    //     std::cout << "1" << std::endl;

    Visualizer::showFigures();
    // HW9::grade<MyKinoRRT, MySingleIntegrator, MyFirstOrderUnicycle, MySecondOrderUnicycle, MySimpleCar>("xavier.okeefe@colorado.edu", argc, argv, std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple(), std::make_tuple());
    return 0;
}