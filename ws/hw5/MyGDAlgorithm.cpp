#include "MyGDAlgorithm.h"

double distance(Eigen::Vector2d q, Eigen::Vector2d qGoal) {
    return (q-qGoal).norm();
}

// Implement your plan method here, similar to HW2:
amp::Path2D MyGDAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d tempGrad(0.0,0.0);
    Eigen::Vector2d closestToInit = GetURep(tempGrad, path, problem);
    double toGo = INT_MAX;
    int maxSteps = 10000;
    double epsilon = 0.2;
    while(distance(path.waypoints.back(), problem.q_goal)>epsilon)
    {
        Eigen::Vector2d q = path.waypoints.back();
        Eigen::Vector2d direction = (problem.q_goal - q).normalized();

        Eigen::Vector2d grad = GetNextStep(path,problem);
        if(grad(0) < 0.0001 && grad(1) < 0.0001)
        {  
            grad(0) = amp::RNG::randd(-1.0, 1.0);
            grad(1) = amp::RNG::randd(-1.0, 1.0);
            int count = 0;
            while(is_point_inside_polygon(problem, q+grad*0.1))
            {
                count++;
                grad(0) = amp::RNG::randd(-1.0, 1.0);
                grad(1) = amp::RNG::randd(-1.0, 1.0);
                if(count > 1000) return path;
            }
        }
        double alpha = 1.0;
        Eigen::Vector2d newPos = q + alpha * grad;

        while(is_point_inside_polygon(problem, newPos))
        {
            alpha*=0.7;
            newPos = q + alpha*grad;
            if(alpha< 0.000001)
            {
                path.waypoints.push_back(problem.q_goal);
                return path;
            }
        }
        path.waypoints.push_back(newPos);

        // std::cout << "Result: " << GetNextStep(path,problem) << std::endl;

        if (path.waypoints.size() > maxSteps) {
            break;
        }
    }


    // path.waypoints.push_back(Eigen::Vector2d(1.0, 5.0));
    // path.waypoints.push_back(Eigen::Vector2d(3.0, 9.0));
    path.waypoints.push_back(problem.q_goal);
    return path;
}

Eigen::Vector2d MyGDAlgorithm::GetUAtt(Eigen::Vector2d gradient, amp::Path2D& path, const amp::Problem2D& problem)
{
    double distToGoal = (problem.q_goal - path.waypoints.back()).norm();

    if(distToGoal > d_star)
    {
        gradient += 0.75*(d_star / distToGoal * (problem.q_goal - path.waypoints.back()));
    } else {
        gradient += 0.75*(problem.q_goal - path.waypoints.back());
    }
    return gradient;

}


// Pretty sure this is working
Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    Eigen::Vector2d ab = b - a;  // Vector from A to B
    double t = (p - a).dot(ab) / ab.dot(ab);  // Project AP onto AB

    // Clamp t to the range [0, 1] to ensure the closest point lies on the segment
    t = std::max(0.0, std::min(1.0, t));

    // Compute the closest point as a linear interpolation between A and B
    return a + t * ab;
}


Eigen::Vector2d MyGDAlgorithm::GetURep(Eigen::Vector2d gradient, amp::Path2D& path, const amp::Problem2D& problem)
{
    Eigen::Vector2d closest;
    Eigen::Vector2d temp;
    Eigen::Vector2d curr = path.waypoints.back();
    std::vector<Eigen::Vector2d> vertices;
    std::vector<Eigen::Vector2d> segment;
    double distance = INT_MAX;
    for(const auto& obs : problem.obstacles)
    {
        vertices = obs.verticesCCW();
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Eigen::Vector2d& a = vertices[i];  // Start of the segment
            const Eigen::Vector2d& b = vertices[(i + 1) % vertices.size()];  // End of the segment (wrap around)

            // Find the closest point on the current segment to the current point
            Eigen::Vector2d temp = closestPointOnSegment(curr, a, b);

            // Calculate the distance from the current point to the closest point on the segment
            double dist = (curr - temp).norm();

            // If this segment is closer than the previous ones, update the closest point and distance
            if (dist < distance) {
                distance = dist;
                closest = temp;
                // std::cout << "New closest point to " << curr << " is " << closest << std::endl;
                // path.waypoints.push_back(closest);
            }
        }
        if(distance > Q_star) continue;
        Eigen::Vector2d direction =  (curr - closest)/distance;  // Direction away from the closest point
        gradient += (0.5) * (0.5) * pow(((1/distance) - (1/Q_star)),2) * direction;
        // std::cout << "Gradient: " << gradient << std::endl;
    }
    return gradient;
}

Eigen::Vector2d MyGDAlgorithm::GetNextStep(amp::Path2D& path, const amp::Problem2D& problem)
{
    Eigen::Vector2d gradient(0.0,0.0);
    Eigen::Vector2d Uatt = GetUAtt(gradient, path, problem).normalized();
    Eigen::Vector2d Urep = GetURep(gradient, path, problem).normalized();


    gradient = Uatt + Urep;

    return gradient;
}

bool MyGDAlgorithm::is_point_inside_polygon(const amp::Problem2D& environment, const Eigen::Vector2d& point) const {
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