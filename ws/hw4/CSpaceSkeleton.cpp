#include "CSpaceSkeleton.h"

// Override this method for returning whether or not a point is in collision

std::pair<std::size_t, std::size_t> MyGridCSpace2D::getCellFromPoint(double x0, double x1) const {
    // Implment your discretization procedure here, such that the point (x0, x1) lies within the returned cell
    std::size_t cell_x = 0; // x index of cell
    std::size_t cell_y = 0; // x index of cell
    // return {cell_x, cell_y};

    double cellWidth = 8.0 / 10.0;
    int x = 0;
    int y = 0;
    double prev = -4;
    double next = -4 + cellWidth;
    for (int i = 0; i < 10; i++)
    {
        if(x0 <= next && x0 >= prev)
        {
            x = i;
            break;
        }
        prev+=cellWidth;
        next +=cellWidth;
    }
    prev = -4;
    next = -4 + cellWidth;
    for (int i = 0; i < 10; i++)
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
    std::unique_ptr<MyGridCSpace2D> cspace_ptr = std::make_unique<MyGridCSpace2D>(m_cells_per_dim, m_cells_per_dim, env.x_min, env.x_max, env.y_min, env.y_max);
    // In order to use the pointer as a regular GridCSpace2D object, we can just create a reference
    MyGridCSpace2D& cspace = *cspace_ptr;

    // Determine if each cell is in collision or not, and store the values the cspace. This `()` operator comes from DenseArray base class
    cspace(1, 3) = true;
    cspace(3, 3) = true;
    cspace(0, 1) = true;
    cspace(1, 0) = true;
    cspace(2, 0) = true;
    cspace(3, 0) = true;
    cspace(4, 1) = true;
    cspace(5, 5) = true;

    std::pair<std::size_t, std::size_t> temp = cspace.getCellFromPoint(0,0.1);
    std::cout << "cells per dim? " << m_cells_per_dim << "env x min " << env.x_max << std::endl;
    std::cout << "cell? " << temp.first << " y: " << temp.second << std::endl;
    // cspace(temp.first, temp.second) = true;
    


    std::cout << cspace.inCollision(0,0.1) << " < " << cspace.inCollision(0,0) << std::endl;

    /*
    TO DO

    determine if cells are in collision with objects or not?
    Run through t1 and t2 and find which thetas are in collision, then mark this in the grid
    thats it?

    old stuff 
    convert robot into point 
    convert obects into c space objects
    mark which squares are in collision-
    */

    // for(int i = 0; i < m_cells_per_dim; i++)
    // {
    //     for(int j = 0; j < m_cells_per_dim; j++)
    //     {

    //     }
    // }
    
    // cspace(9,9) = true;
    // Returning the object of type std::unique_ptr<MyGridCSpace2D> can automatically cast it to a polymorphic base-class pointer of type std::unique_ptr<amp::GridCSpace2D>.
    // The reason why this works is not super important for our purposes, but if you are curious, look up polymorphism!
    return cspace_ptr;
}

// bool MyBugAlgorithm::is_point_inside_polygon(const amp::Polygon& polygon, const Eigen::Vector2d& point) const {
//     const std::vector<Eigen::Vector2d>& vertices = polygon.verticesCCW();
//     int num_vertices = vertices.size();
//     bool inside = false;

//     // Ray-casting algorithm to check if point is inside polygon
//     for (int i = 0, j = num_vertices - 1; i < num_vertices; j = i++) {
//         if ((vertices[i].y() > point.y()) != (vertices[j].y() > point.y()) &&
//             (point.x() < (vertices[j].x() - vertices[i].x()) * (point.y() - vertices[i].y()) / 
//             (vertices[j].y() - vertices[i].y()) + vertices[i].x())) {
//             inside = !inside;
//         }
//     }

//     return inside;
// }
