#include "MyCSConstructors.h"

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

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    double numCells = xCellNumber;
    double cellWidthX = (43.0)/numCells;
    double cellWidthY = (14.0)/numCells;
    // std::cout << "x: " << cellWidthX << " Y: " << cellWidthY << std::endl;

    std::size_t cell_x = static_cast<std::size_t>((x0 +7) / cellWidthX);
    std::size_t cell_y = static_cast<std::size_t>((x1 +7) / cellWidthY);
    return {cell_x, cell_y};
}

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;
    std::cout << "Constructing C-space for manipulator" << std::endl;
    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    cspace(1, 3) = true;
    cspace(3, 3) = true;
    cspace(0, 1) = true;
    cspace(1, 0) = true;
    cspace(2, 0) = true;
    cspace(3, 0) = true;
    cspace(4, 1) = true;

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

    double cellWidthX = (env.x_max - env.x_min) / m_cells_per_dim;
        double cellWidthY = (env.y_max - env.y_min) / m_cells_per_dim;


    // Eigen::Vector2d temp(-3.6, -2);
    // std::cout << "cellwidth" << cellWidth << std::endl;


    for (int i = 0; i < m_cells_per_dim; ++i) {
        double x = env.x_min + (i + 0.5) * cellWidthX;  // Center of the current column of cells

        for (int j = 0; j < m_cells_per_dim; ++j) {
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

Eigen::Vector2d getPointFromCell(int x, int y)
{
    double newX, newY;
    int xCells = xCellNumber;
    int yCells = yCellNumber;
    double xWidth = 43.0/xCells;
    double yWidth = 14.0/yCells;
    newX = -7.0 + x * xWidth + (xWidth * 0.5);
    newY = -7.0 + y * yWidth + (yWidth * 0.5);
    Eigen::Vector2d res(newX, newY);
    return res;

}

    // Implement your WaveFront algorithm here
amp::Path2D MyWaveFrontAlgorithm::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) {
    // Get the start and goal cells from the continuous points
    amp::Path2D path;
    auto [goal_x, goal_y] = grid_cspace.getCellFromPoint(q_goal.x(), q_goal.y());
    auto [init_x, init_y] = grid_cspace.getCellFromPoint(q_init.x(), q_init.y());
    // std::cout << "-5,-5 in collision? " << grid_cspace.inCollision(-5,-5) << " " << grid_cspace(0,1)<<std::endl;

    std::size_t numCellsX = xCellNumber;
    std::size_t numCellsY = yCellNumber;


    std::vector<std::vector<int>> wavefront(numCellsX, std::vector<int>(numCellsY, -1));
    // Initialize current position as Eigen::Vector2d
    Eigen::Vector2d current(init_x, init_y);
    path.waypoints.push_back(q_init);

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
                queue.push({nx, ny});
            }
        }
    }
    // for(int i = 0; i < numCellsX; i++)
    // {
    //     for(int j = 0; j < numCellsY; j++)
    //     {
    //         std::cout << "cell " << i << ", " << j << "= " << wavefront[i][j] << std::endl;
    //     }
    // }

    auto[resX,resY] = grid_cspace.getCellFromPoint(q_init.x(), q_init.y ());
    path.waypoints.push_back(q_init);
    std::cout << init_x << " " << init_y << "In collision? " << grid_cspace(init_x, init_y) << " with val " << wavefront[init_x][init_y] << std::endl;


    current.x() = resX;
    current.y() = resY;
    while (current != Eigen::Vector2d(goal_x, goal_y)) {
        // auto[X1,Y1] = grid_cspace.getCellFromPoint(current.x(), current.y());

        int current_wave_val = wavefront[init_x][init_y];
        // std::cout << "Wave val " << current_wave_val << "for : " << init_x << ", " << init_y << std::endl;

        // Find the neighbor with the smallest wave value
        Eigen::Vector2d next = current;
        int min_wave_val = current_wave_val;
        // std::cout << "current vector " << current << std::endl;

        for (const auto& direction : directions) {
            Eigen::Vector2d neighbor = current + direction;
            // std::cout << "neighbor components " << neighbor << std::endl;
            int nx = static_cast<int>(std::round(neighbor.x()));
            int ny = static_cast<int>(std::round(neighbor.y()));
            // std::cout << "nx, ny " << nx << ", " << ny << std::endl;

            if (nx >= 0 && nx < numCellsX && ny >= 0 && ny < numCellsY) {
                std::cout << "Wave val " << wavefront[nx][ny] << "for : " << nx << ", " << ny << std::endl;

                if (wavefront[nx][ny] < min_wave_val && wavefront[nx][ny] != -1) {
                    min_wave_val = wavefront[nx][ny];
                    next = neighbor;
                    std::cout << "New min wave " << min_wave_val << std::endl;
                }
            }
        }

        // Move to the next cell with the smallest wave value
        std::cout << min_wave_val << " curr " << current_wave_val << std::endl;
        if (min_wave_val < current_wave_val) {
            current = next;
            auto[X,Y] = grid_cspace.getCellFromPoint(current.x(), current.y());
            Eigen::Vector2d temp(current.x(), current.y());
            temp = getPointFromCell(current.x(),current.y());
            path.waypoints.push_back(temp);
            std::cout << " new cell cspace coords "<< temp.x() << ", " <<temp.y() << std::endl;
        } else {
                        std::cout << " breaking " << std::endl;

            break;  // No valid path
        }
    }

    path.waypoints.push_back(q_goal);  // Add the goal at the end of the path
    return path;
}

