#include "CSpaceSkeleton.h"

/*
IF I NEED TO DO C SPACE

use forward kinematics to get point one and point two
check line segment between origin and point 1, point1 and point 2
if interesects with grid -> thetas not valid
else -> thetas valid, add to vector of valid thetas
fill in grid with these valid thetas
*/
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


// Override this method for returning whether or not a point is in collision
std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = 0; // x index of cell
    std::size_t cell_y = 0; // x index of cell
    // return {cell_x, cell_y};

    int cells = 500;
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

// Override this method for computing all of the boolean collision values for each cell in the cspace
std::unique_ptr<amp::GridCSpace2D> MyManipulatorCSConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) {
    // Create an object of my custom cspace type (e.g. MyGridCSpace2D) and store it in a unique pointer. 
    // Pass the constructor parameters to std::make_unique()
    // std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, 0, 2 * M_PI, 0, 2 * M_PI);

    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    double cellWidth = (env.x_max - env.x_min) / m_cells_per_dim;
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
                auto [cellX,cellY] = cspace.getCellFromPoint(t1,t2);
                cspace(cellX,cellY) = true;
            }
        }
    }


    // for (int i = 0; i < m_cells_per_dim; ++i) {
    //     double x = env.x_min + (i + 0.5) * cellWidth;  // Center of the current column of cells

    //     for (int j = 0; j < m_cells_per_dim; ++j) {
    //         double y = env.y_min + (j + 0.5) * cellWidth;  // Center of the current row of cells
    //         Eigen::Vector2d point(x, y);

    //         // Check if the point is inside any obstacle
    //         if (is_point_inside_polygon(env, point)) {
    //             // std::cout << "Point inside obstacle: (" << point.x() << ", " << point.y() << ")" << std::endl;
    //             // std::cout << "Marked cell x: " << x << " y: " << y << std::endl;

    //             // Mark the corresponding cell as occupied
    //             auto [cellX, cellY] = cspace.getCellFromPoint(x, y);
    //             cspace(cellX, cellY) = true;  // Mark cell as occupied
    //         }
    //     }
    // }

    // bool test = is_point_inside_polygon(env, temp);
    // if(test)
    // {
    //     auto[cellX,cellY] = cspace.getCellFromPoint(temp.x(), temp.y());
    //     cspace(cellX, cellY) = true;  // Mark cell as occupied
    //     std::cout << "Success " << std::endl;

    // }

    // Eigen::Vector2d temp2(1.5,-1.9);

    // test = is_point_inside_polygon(env, temp);
    // if(test)
    // {
    //     auto[cellX2,cellY2] = cspace.getCellFromPoint(temp2.x(), temp2.y());
    //     cspace(cellX2, cellY2) = true;  // Mark cell as occupied
    //     std::cout << "Success " << std::endl;

    // }
    
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

// works

bool MyManipulatorCSConstructor::is_point_inside_polygon(const amp::Environment2D& environment, const Eigen::Vector2d& point) const {
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
