#ifndef OBJECTINFO_HPP
#define OBJECTINFO_HPP
#include <string>
#include <cmath>
#include <unordered_map>
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

    ObjectInfo(std::string name_in, double x_in, double y_in, double nominal_yaw, int nSide, double radius)
        : name(name_in), x(x_in), y(y_in), nominalOrientation(nominal_yaw), numberOfSides(nSide),enclosingRadius(radius)
    {
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

    std::vector<ReloPush::State> getPushingPoses()
    {
        std::vector<ReloPush::State> out_poses(numberOfSides);

        for(size_t n=0; n<numberOfSides; n++)
            out_poses[n] = getPushingPose(n);

        return out_poses;
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

    GoalInfo()
    {
    }

    GoalInfo(std::string name_in, double x_in, double y_in, double nominal_yaw, int nSide, double radius)
    : name(name_in), x(x_in), y(y_in), nominalOrientation(nominal_yaw), numberOfSides(nSide),enclosingRadius(radius)
    {
    }

    double getOrientation(int orientationIndex) const
    {
        // If numberOfSides <= 0, fallback or just return nominalOrientation
        if (numberOfSides <= 0)
            return nominalOrientation;

        double stepAngle = (2.0 * M_PI) / static_cast<double>(numberOfSides);
        return nominalOrientation + (orientationIndex * stepAngle);
    }
};

typedef std::unordered_map<std::string, ObjectInfo> ObjectMap;
typedef std::unordered_map<std::string, GoalInfo> GoalMap;

#endif // OBJECTINFO_HPP
