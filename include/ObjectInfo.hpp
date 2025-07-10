#ifndef OBJECTINFO_HPP
#define OBJECTINFO_HPP
#include <string>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <State.h>

/**
 * @brief A simple struct to hold "object-level" info:
 *        - name
 *        - position (x,y)
 *        - nominal orientation
 *        - number of sides
 */
struct ObjectInfo// : ReloPush::State
{
    std::string name;
    double x;
    double y;
    double nominalOrientation; // might be duplicate with State yaw
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

    ObjectInfo(std::string name_in, ReloPush::State& pose, int nSide, double radius)
        : name(name_in), numberOfSides(nSide), enclosingRadius(radius)
    {
        x = pose.x;
        y = pose.y;
        nominalOrientation = pose.yaw;
    }

    // use info from an existing object info, use pose from state
    ObjectInfo(const ObjectInfo& obj_info, ReloPush::State& pose)
    {
        name = obj_info.name;
        numberOfSides = obj_info.numberOfSides;
        enclosingRadius = obj_info.enclosingRadius;

        x = pose.x;
        y = pose.y;
        nominalOrientation = pose.yaw;
    }

    bool operator==(const ObjectInfo& other) const {
        return x == other.x && y == other.y && nominalOrientation == other.nominalOrientation
                && name == other.name, enclosingRadius == other.enclosingRadius;
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

typedef ObjectInfo GoalInfo;

/*
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
*/

//typedef std::unordered_map<std::string, ObjectInfo> ObjectMap, GoalMap;
//typedef std::unordered_map<std::string, GoalInfo> GoalMap;


class ObjectMap : public std::unordered_map<std::string, ObjectInfo> {
public:

    // Default constructor
    ObjectMap() = default;

    // Constructor from std::unordered_map
    ObjectMap(const std::unordered_map<std::string, ObjectInfo>& other)
        : std::unordered_map<std::string, ObjectInfo>(other.begin(), other.end()) {}

    std::vector<ObjectInfo> toList() const {
        std::vector<ObjectInfo> out;
        out.reserve(this->size()); // Use 'this->size()'
        for (const auto& pair : *this) { // Iterate over *this
            out.push_back(pair.second);
        }
        return out;
    }

    std::vector<ReloPush::State> toStateList() const {
        std::vector<ReloPush::State> out;
        out.reserve(this->size());
        for (const auto& pair : *this) {
            out.push_back(ReloPush::State(pair.second.x, pair.second.y, pair.second.nominalOrientation));
        }
        return out;
    }

    void append(const ObjectMap& other) {
        for (const auto& pair : other) {
            if (this->count(pair.first)) {
                std::cerr << "[Warning] ObjectMap: Key '" << pair.first << "' already exists. Overwriting value." << std::endl;
            }
            (*this)[pair.first] = pair.second;
        }
    }

    // Example custom member function
    void printObjectNames() const {
        for (const auto& pair : *this) {
            std::cout << pair.first << std::endl;
        }
    }

    // Add more member functions as needed
};


typedef ObjectMap GoalMap;


struct ObjectGoalPair
{
    std::string objectName;
    std::string goalName;

    ObjectGoalPair()
    {}

    ObjectGoalPair(std::string obj, std::string goal)
        : objectName(obj), goalName(goal)
    {}
};

#endif // OBJECTINFO_HPP
