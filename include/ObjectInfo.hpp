#ifndef OBJECTINFO_HPP
#define OBJECTINFO_HPP
#include <string>
#include <cmath>
#include <State.h>

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

    ObjectInfo()
    {
        x = 0;
        y = 0;
        nominalOrientation = 0;
        numberOfSides = 0;
        enclosingRadius = 0;
    }

    ObjectInfo(std::string name_in, double x_in, double y_in, double nominal_yaw, double radius)
        : name(name_in), x(x_in), y(y_in), nominalOrientation(nominal_yaw), enclosingRadius(radius)
    {
        numberOfSides = 4;
    }

    double getOrientation(int orientationIndex) const
    {
        // If numberOfSides <= 0, fallback or just return nominalOrientation
        if (numberOfSides <= 0)
            return nominalOrientation;

        double stepAngle = (2.0 * M_PI) / static_cast<double>(numberOfSides);
        return nominalOrientation + (orientationIndex * stepAngle);
    }

    void applyRotation(double angleChange)
    {
        nominalOrientation = fromOMPL::mod2pi(nominalOrientation + angleChange);
    }

    ReloPush::State getNominalPose()
    {
        return ReloPush::State(x,y,nominalOrientation);
    }

    ReloPush::State getPushingPose(int pushing_index)
    {
        return ReloPush::State(x,y, getOrientation(pushing_index));
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
