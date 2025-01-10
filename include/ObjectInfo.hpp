#ifndef OBJECTINFO_HPP
#define OBJECTINFO_HPP
#include <string>
#include <cmath>

/**
 * @brief A simple struct to hold "object-level" info:
 *        - name
 *        - position (x,y)
 *        - nominal orientation
 *        - number of sides
 */
struct ObjectInfo
{
    std::string name;
    double x;
    double y;
    double nominalOrientation;
    int numberOfSides;
    double enclosingRadius;

    double getOrientation(int orientationIndex) const
    {
        // If numberOfSides <= 0, fallback or just return nominalOrientation
        if (numberOfSides <= 0)
            return nominalOrientation;

        double stepAngle = (2.0 * M_PI) / static_cast<double>(numberOfSides);
        return nominalOrientation + (orientationIndex * stepAngle);
    }
};

/**
 * @brief A similar struct for "goal-level" info,
 *        if we want discrete orientations for the goal as well.
 */
struct GoalInfo
{
    std::string name;
    double x;
    double y;
    double nominalOrientation;
    int numberOfSides;
    double enclosingRadius;

    double getOrientation(int orientationIndex) const
    {
        // If numberOfSides <= 0, fallback or just return nominalOrientation
        if (numberOfSides <= 0)
            return nominalOrientation;

        double stepAngle = (2.0 * M_PI) / static_cast<double>(numberOfSides);
        return nominalOrientation + (orientationIndex * stepAngle);
    }
};

#endif // OBJECTINFO_HPP
