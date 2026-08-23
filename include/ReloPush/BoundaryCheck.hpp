#ifndef BOUNDARY_CHECK_HPP
#define BOUNDARY_CHECK_HPP

#include <cmath>
#include <array>

/**
 * @brief Check if all 4 robot corners are within the given workspace bounds.
 *
 * Computes the 4 corners of the robot footprint and verifies that all corners
 * satisfy: xMin - tol <= cx <= xMax + tol AND yMin - tol <= cy <= yMax + tol
 *
 * @param x Robot x position (world frame)
 * @param y Robot y position (world frame)
 * @param yaw Robot orientation (radians)
 * @param front Front extent from rear axle (e.g., LF or LF + pushObject extent)
 * @param rear Rear extent from rear axle (e.g., LB)
 * @param half_width Half width of robot (car_width / 2.0)
 * @param xMin Minimum x boundary
 * @param xMax Maximum x boundary
 * @param yMin Minimum y boundary
 * @param yMax Maximum y boundary
 * @param tol Tolerance margin (default 1e-2)
 *
 * @return true if all 4 corners are within bounds (with tolerance), false otherwise
 */
inline bool footprint_in_bounds(double x, double y, double yaw,
                                double front, double rear, double half_width,
                                double xMin, double xMax, double yMin, double yMax,
                                double tol = 1e-2)
{
    // Compute robot corners in world frame
    // Corner formulas from PathPlanningTools.h stateValid2 (lines 1174-1178):
    // Front-left:  (x + front*cos(yaw) - half_width*sin(yaw), y + front*sin(yaw) + half_width*cos(yaw))
    // Front-right: (x + front*cos(yaw) + half_width*sin(yaw), y + front*sin(yaw) - half_width*cos(yaw))
    // Rear-left:   (x - rear*cos(yaw) - half_width*sin(yaw), y - rear*sin(yaw) + half_width*cos(yaw))
    // Rear-right:  (x - rear*cos(yaw) + half_width*sin(yaw), y - rear*sin(yaw) - half_width*cos(yaw))

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    std::array<std::pair<double, double>, 4> corners = {{
        {x + front * cos_yaw - half_width * sin_yaw, y + front * sin_yaw + half_width * cos_yaw},  // front-left
        {x + front * cos_yaw + half_width * sin_yaw, y + front * sin_yaw - half_width * cos_yaw},  // front-right
        {x - rear * cos_yaw - half_width * sin_yaw, y - rear * sin_yaw + half_width * cos_yaw},    // rear-left
        {x - rear * cos_yaw + half_width * sin_yaw, y - rear * sin_yaw - half_width * cos_yaw}     // rear-right
    }};

    // Check all 4 corners against bounds
    for (const auto &corner : corners)
    {
        double cx = corner.first;
        double cy = corner.second;
        if (cx < xMin - tol || cx > xMax + tol || cy < yMin - tol || cy > yMax + tol)
        {
            return false;  // Corner is out of bounds
        }
    }

    return true;  // All corners are in bounds
}

#endif // BOUNDARY_CHECK_HPP
