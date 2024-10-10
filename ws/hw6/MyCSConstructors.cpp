#include "MyCSConstructors.h"
#include "ManipulatorSkeleton.h"

#define xCellNumber 100
#define yCellNumber 100

bool is_point_inside_polygon(const amp::Environment2D& environment, const Eigen::Vector2d& point){
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

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

// Eigen::Vector2d MyManipulatorCSConstructor::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {

//     std::vector<double> link_lengths_ = getLinkLengths();

//     // std::cout << "link lengths " << link_lengths_[0] << std::endl;
//     // if (joint_index >= state.size() || joint_index >= link_lengths_.size()) {
//     //     throw std::out_of_range("Invalid joint index");
//     // }

//     // Base matrix
//     Eigen::Matrix3d T = Eigen::Matrix3d::Identity();

//     Eigen::Vector3d base_position(0.0, 0.0, 1.0);

//     for (uint32_t i = 0; i <= joint_index; ++i) {
//         double theta = state[i];        // Joint angle
//         double link_length = (i == 0) ? 0.0 : link_lengths_[i - 1]; 

//         Eigen::Matrix3d T_joint;
//         T_joint << std::cos(theta), -std::sin(theta), link_length,
//                    std::sin(theta),  std::cos(theta), 0,
//                    0, 0, 1.0;

//         T *= T_joint;
//     }

//     Eigen::Vector3d joint_position = T * base_position;
//     // std::cout << "returning " << joint_position.head<2>() << std::endl;
//     // Return the (x, y) part of the position
//     return joint_position.head<2>();
// }

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    std::pair<std::size_t, std::size_t> size = this->size();
    // double numCells = xCellNumber;
    // double numCellsY = yCellNumber;

    std::pair<double, double> x_lims = x0Bounds();
    std::pair<double, double> y_lims = x1Bounds();

    double cellWidthX = (x_lims.second - x_lims.first) / size.first;
    double cellWidthY = (y_lims.second - y_lims.first) / size.second;
    // std::cout << "x: " << cellWidthX << " Y: " << cellWidthY << std::endl;

    std::size_t cell_x = static_cast<std::size_t>((x0 - x_lims.first) / cellWidthX);
    std::size_t cell_y = static_cast<std::size_t>((x1 - y_lims.first) / cellWidthY);

    if(cell_x >= size.first) cell_x = size.first - 1;
    if(cell_y >= size.second) cell_y = size.second - 1;

    return {cell_x, cell_y};
}

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPointManip(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
std::size_t cell_x = 0; // x index of cell
    std::size_t cell_y = 0; // x index of cell
    // return {cell_x, cell_y};

    int cells = xCellNumber;
    double cellWidth = 2 * M_PI / cells;
    int x = 0;
    int y = 0;
    double prev = 0;
    double next = 0 + cellWidth;
    for (int i = 0; i < cells; i++)
    {
        if(x0 <= next && x0 >= prev)
        {
            x = i;
            break;
        }
        prev+=cellWidth;
        next +=cellWidth;
    }
    prev = 0;
    next = 0 + cellWidth;
    for (int i = 0; i < cells; i++)
    {
        if(x1 <= next && x1 >= prev)
        {
            y = i;
            break;
        }
        prev+=cellWidth;
        next +=cellWidth;
    }
    cell_x = x;
    cell_y = y;
    return {cell_x, cell_y};
}

std::vector<Eigen::Vector2d> getPoints(double t1, double t2, double a1, double a2)
{
    std::vector<Eigen::Vector2d> res;
    double j1x = a1 * cos(t1);
    double j1y = a1 * sin(t1);
    double j2x = j1x + a2 * cos(t1 + t2);
    double j2y = j1y + a2 * sin(t1 + t2);

    Eigen::Vector2d j0(0.0, 0.0);
    Eigen::Vector2d j1(j1x , j1y);
    Eigen::Vector2d j2(j2x, j2y);

    res.push_back(j0);
    res.push_back(j1);
    res.push_back(j2);

    return res;
}

std::pair<std::size_t, std::size_t> getCellFromPointManipNoClass(double x0, double x1) {
    double numCells = xCellNumber;
    double numCellsY = yCellNumber;
    double cellWidthX = (2 * M_PI)/numCells;
    double cellWidthY = (2 * M_PI)/numCellsY;
    // std::cout << "x: " << cellWidthX << " Y: " << cellWidthY << std::endl;

    std::size_t cell_x = static_cast<std::size_t>((x0) / cellWidthX);
    std::size_t cell_y = static_cast<std::size_t>((x1) / cellWidthY);
    if(cell_x >= xCellNumber) cell_x = xCellNumber - 1;
    if(cell_y >= yCellNumber) cell_y = yCellNumber - 1;
    return {cell_x, cell_y};
}

bool MyManipulatorCSConstructor::checkLineSegment(Eigen::Vector2d j1, Eigen::Vector2d j2, const amp::Environment2D& env)
{
    Eigen::Vector2d direction = (j2 - j1).normalized();
    // std::cout << " Joint 1 : " << j1 << " Joint 2: " << j2 << std::endl;
    
    // Use a floating-point value for 'i' to step along the segment
    for(double i = 0; i <= 1; i += 0.05) // i is now a double, stepping by 0.01
    {
        Eigen::Vector2d point = j1 + i * (j2 - j1); // Scale the entire vector, not just normalized direction

        if(is_point_inside_polygon(env, point))
        {
            // std::cout << "point is inside polygon" << std::endl;
            return true;
        }
    }
    
    return false;
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2 * M_PI, 0, 2 * M_PI);
    std::cout << "m cells per dim" << m_cells_per_dim << "env limits " << env.x_min << std::endl;
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class

    double cellWidth = (2*M_PI) / xCellNumber;
    double y = cellWidth / 2;
    double x = cellWidth / 2;
    std::vector<double> links = manipulator.getLinkLengths();
    double a1 = links[0];
    double a2 = links[1];
    // Eigen::Vector2d temp(-3.6, -2);
    // std::cout << "cellwidth" << cellWidth << std::endl;


    std::vector<Eigen::Vector2d> res = getPoints(0, M_PI/4,1,1);
    // std::cout << "last point " << res[2] << std::endl;
    // bool val = checkLineSegment(res[2],res[1],env);
    std::pair<std::size_t, std::size_t> cellPoint;
    for(double t1 = 0; t1 < 2 * M_PI; t1+=0.01)
    {
        for(double t2 = 0; t2 < 2 * M_PI; t2+=0.01)
        {
            res = getPoints(t1,t2,a1,a2);
            if(checkLineSegment(res[2],res[1],env) || checkLineSegment(res[1], res[0],env))
            {
                auto [cellX,cellY] = cspace.getCellFromPointManip(t1,t2);
                cspace(cellX,cellY) = true;
            }
        }
    }
    

    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

//////////////////////////////////////////////////////////////

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyPointAgentCSConstructor::construct(const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for point agent" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class

    // Eigen::Vector2d temp(9.65, 2.72);
    // auto[cellX, cellY] = cspace.getCellFromPoint(5,-2.5);
    // std::cout << "point 0,5 is in obstacle? " << is_point_inside_polygon(env, temp) << " with cell " << cellX << " " << cellY << std::endl;

    // double cellWidthX = (env.x_max - env.x_min) / xCellNumber;
    // double cellWidthY = (env.y_max - env.y_min) / yCellNumber;
    double cellWidthX = (env.x_max - env.x_min) / xCellNumber;
    double cellWidthY = (env.y_max - env.y_min) / yCellNumber;


    // Eigen::Vector2d temp(-3.6, -2);
    // std::cout << "cellwidth" << cellWidth << std::endl;


    for (int i = 0; i < xCellNumber; ++i) {
        double x = env.x_min + (i + 0.5) * cellWidthX;  // Center of the current column of cells
        for (int j = 0; j < yCellNumber; ++j) {
            double y = env.y_min + (j + 0.5) * cellWidthY;  // Center of the current row of cells
            Eigen::Vector2d point(x, y);

            // Check if the point is inside any obstacle
            if (is_point_inside_polygon(env, point)) {
                // std::cout << "Point inside obstacle: (" << point.x() << ", " << point.y() << ")" << std::endl;
                // std::cout << "Marked cell x: " << x << " y: " << y << std::endl;
                // Mark the corresponding cell as occupied
                auto [cellX, cellY] = cspace.getCellFromPoint(x, y);

                cspace(cellX, cellY) = true;  // Mark cell as occupied
            }
        }
    }
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

Eigen::Vector2d getPointFromCell(int x, int y, std::pair<std::size_t, std::size_t> size, std::pair<double, double> x_lims, std::pair<double, double> y_lims)
{
    // std::pair<std::size_t, std::size_t> size = this->size();

    // std::pair<double, double> x_lims = x0Bounds();
    // std::pair<double, double> y_lims = x1Bounds();

    double newX, newY;
    int xCells = xCellNumber;
    int yCells = yCellNumber;
    double xWidth = (x_lims.second - x_lims.first) / size.first;
    double yWidth = (y_lims.second - y_lims.first) / size.second;
    newX = x_lims.first + x * xWidth + (xWidth * 0.5);
    newY = y_lims.first + y * yWidth + (yWidth * 0.5);
    Eigen::Vector2d res(newX, newY);
    return res;

}

Eigen::Vector2d getPointFromCellManip(int x, int y)
{
    double newX, newY;
    int xCells = xCellNumber;
    int yCells = yCellNumber;
    double xWidth = 2*M_PI/xCells;
    double yWidth = 2*M_PI/yCells;
    newX = x * xWidth + (xWidth * 0.5);
    newY = y * yWidth + (yWidth * 0.5);
    Eigen::Vector2d res(newX, newY);
    return res;

}


amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) {
    // Implement your WaveFront algorithm here
    // Get the start and goal cells from the continuous points
    amp::Path2D path;
    auto [goal_x, goal_y] = grid_cspace.getCellFromPoint(q_goal.x(), q_goal.y());
    auto [init_x, init_y] = grid_cspace.getCellFromPoint(q_init.x(), q_init.y());

    std::pair<std::size_t, std::size_t> size = grid_cspace.size();


    std::pair<double, double> x_lims = grid_cspace.x0Bounds();
    std::pair<double, double> y_lims = grid_cspace.x1Bounds();



    if(false) {
        std::cout <<"running manipulator " << std::endl;
        std::tie(goal_x, goal_y) = getCellFromPointManipNoClass(q_goal.x(), q_goal.y());  // reuse goal_x and goal_y
        std::tie(init_x, init_y) = getCellFromPointManipNoClass(q_init.x(), q_init.y());  // reuse init_x and init_y
        // std::tie(goal_x, goal_y) = getCellFromPointManipNoClass(M_PI, 0.0);  // reuse goal_x and goal_y
        // std::tie(init_x, init_y) = getCellFromPointManipNoClass(0.0, 0.0);  // reuse init_x and init_y
        // std::cout << q_init.x() << ", " << q_init.y() << " init in if statement " << std::endl;
        // std::cout << "Init " << init_x << ", " <<init_y << std::endl;
        // std::cout << "Goals " << goal_x << " , " << goal_y << std::endl;
    }

    // std::cout << "-5,-5 in collision? " << grid_cspace.inCollision(-5,-5) << " " << grid_cspace(0,1)<<std::endl;
    std::cout << "init " << q_init << " end: " << q_goal << std::endl;
    std::size_t numCellsX = xCellNumber;
    std::size_t numCellsY = yCellNumber;

    // std::cout << "Init " << init_x << ", " <<init_y <<std::endl;
    // std::cout << "Goals " << goal_x << " , " << goal_y << std::endl;

    std::vector<std::vector<int>> wavefront(numCellsX, std::vector<int>(numCellsY, -1));
    // Initialize current position as Eigen::Vector2d
    Eigen::Vector2d current(init_x, init_y);
    // path.waypoints.push_back(q_init);
    if(isManipulator)
    {
        double newXInit = q_init.x();
        double newYInit = q_init.y();
        if (q_init.x() < 0) 
        {
            newXInit = newXInit * -1;
        }
        if (q_init.y() < 0)
        {
            newYInit = newYInit * -1;
        }
        Eigen::Vector2d tempInit(newXInit, newYInit);
        path.waypoints.push_back(tempInit);
    } else {
        path.waypoints.push_back(q_init);
    }
    std::cout << "Q_INIT " << q_init << std::endl;

    // Possible movement directions: right, left, up, down, and diagonals
    std::vector<Eigen::Vector2d> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1} // cardinal directions
    };


    std::queue<std::pair<std::size_t, std::size_t>> queue;
    queue.push({goal_x, goal_y});
    wavefront[goal_x][goal_y] = 0;  // Wave value of goal cell is 0

    while (!queue.empty()) {
        auto [cx, cy] = queue.front();
        queue.pop();
        int current_wave_val = wavefront[cx][cy] + 1;

        // Check all 4 neighbors
        for (const auto& direction : directions) {
            int nx = cx + direction.x();
            int ny = cy + direction.y();

            // Make sure the neighbor is within bounds and not an obstacle
            if (nx >= 0 && nx < numCellsX && ny >= 0 && ny < numCellsY && wavefront[nx][ny] == -1 && !grid_cspace(nx, ny)) {
                wavefront[nx][ny] = current_wave_val;
                // std::cout << "nx: " << nx << " ny: " << ny << "wave vale " << current_wave_val << std::endl;
                queue.push({nx, ny});
            }
        }
    }
    std::pair<std::size_t, std::size_t> initPoint = grid_cspace.getCellFromPoint(q_init.x(), q_init.y ());
    // if(isManipulator)
    // {
    //     initPoint = getCellFromPointManipNoClass(q_init.x(), q_init.y ());
    // }
    // path.waypoints.push_back(q_init);

    // std::cout << "q_init is " << q_init.x() << ", ' " << q_init.y() << std::endl;

    while (current != Eigen::Vector2d(goal_x, goal_y)) {
        int current_wave_val = wavefront[init_x][init_y];
        // std::cout << "in while loop " << std::endl;
        Eigen::Vector2d next = current;
        int min_wave_val = current_wave_val;

        for (const auto& direction : directions) {
            Eigen::Vector2d neighbor = current + direction;
            int nx = static_cast<int>(std::round(neighbor.x()));
            int ny = static_cast<int>(std::round(neighbor.y()));

            if (nx >= 0 && nx < numCellsX && ny >= 0 && ny < numCellsY) {

                if (wavefront[nx][ny] < min_wave_val && wavefront[nx][ny] != -1) {
                    min_wave_val = wavefront[nx][ny];
                    next = neighbor;
                }
            }
        }

        // Move to the next cell with the smallest wave value
        if ((min_wave_val < current_wave_val)) {
            current = next;
            auto[X,Y] = grid_cspace.getCellFromPoint(current.x(), current.y());
            Eigen::Vector2d temp(current.x(), current.y());
            temp = getPointFromCell(current.x(),current.y(), size, x_lims, y_lims);
            path.waypoints.push_back(temp);
        } else {
            std::cout << " breaking " << std::endl;
            break;  // No valid path
        }
    }

    if (isManipulator) {
        Eigen::Vector2d tempGoal(M_PI,0);
        auto[tempX1, tempY1] = getCellFromPointManipNoClass(M_PI,0.0);
        tempGoal.x() = tempX1;
        tempGoal.y() = tempY1;
        // path.waypoints.push_back(tempGoal);  // Add the goal at the end of the path
        Eigen::Vector2d bounds0 = Eigen::Vector2d(0.0, 0.0);
        Eigen::Vector2d bounds1 = Eigen::Vector2d(2*M_PI, 2*M_PI);
        // amp::unwrapWaypoints(path.waypoints, bounds0, bounds1);
        return path;
    }
    path.waypoints.push_back(q_goal);  // Add the goal at the end of the path

    return path;
}

