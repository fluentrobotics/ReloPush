#ifndef OBJECTINFO_HPP
#define OBJECTINFO_HPP
#include <string>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>
#include <ReloPush/State.h>

/**
 * @brief A simple struct to hold "object-level" info:
 *        - name
 *        - position (x,y)
 *        - nominal orientation
 *        - number of sides
 */
struct ObjectInfo// : ReloPush::State
{
    static constexpr std::size_t unspecified_input_order =
        std::numeric_limits<std::size_t>::max();

    std::string name;
    double x;
    double y;
    double nominalOrientation; // might be duplicate with State yaw
    int numberOfSides;
    double enclosingRadius;
    std::size_t input_order;

    ObjectInfo()
    {
        x = 0;
        y = 0;
        nominalOrientation = 0;
        numberOfSides = 0;
        enclosingRadius = 0;
        input_order = unspecified_input_order;
    }

    ObjectInfo(std::string name_in, double x_in, double y_in, double nominal_yaw,
               double radius,
               std::size_t input_order_in = unspecified_input_order)
        : name(name_in), x(x_in), y(y_in), nominalOrientation(nominal_yaw),
          enclosingRadius(radius), input_order(input_order_in)
    {
        numberOfSides = 4;
    }

    ObjectInfo(std::string name_in, double x_in, double y_in, double nominal_yaw,
               int nSide, double radius,
               std::size_t input_order_in = unspecified_input_order)
        : name(name_in), x(x_in), y(y_in), nominalOrientation(nominal_yaw),
          numberOfSides(nSide), enclosingRadius(radius), input_order(input_order_in)
    {
    }

    ObjectInfo(std::string name_in, ReloPush::State& pose, int nSide, double radius,
               std::size_t input_order_in = unspecified_input_order)
        : name(name_in), numberOfSides(nSide), enclosingRadius(radius),
          input_order(input_order_in)
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
        input_order = obj_info.input_order;

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

    std::vector<ObjectInfo> toOrderedList() const {
        std::vector<ObjectInfo> out;
        out.reserve(this->size());
        for (const auto& pair : *this) {
            out.push_back(pair.second);
        }
        std::sort(out.begin(), out.end(), [](const ObjectInfo& a, const ObjectInfo& b) {
            if (a.input_order != b.input_order)
                return a.input_order < b.input_order;
            return a.name < b.name;
        });
        return out;
    }

    std::vector<ObjectInfo> toList() const {
        return toOrderedList();
    }

    std::vector<ReloPush::State> toStateList() const {
        std::vector<ReloPush::State> out;
        out.reserve(this->size());
        for (const auto& object : toOrderedList()) {
            out.push_back(ReloPush::State(object.x, object.y, object.nominalOrientation));
        }
        return out;
    }

    void append(const ObjectMap& other) {
        for (const auto& object : other.toOrderedList()) {
            if (this->count(object.name)) {
                std::cerr << "[Warning] ObjectMap: Key '" << object.name << "' already exists. Overwriting value." << std::endl;
            }
            (*this)[object.name] = object;
        }
    }

    // Example custom member function
    void printObjectNames() const {
        for (const auto& object : toOrderedList()) {
            std::cout << object.name << std::endl;
        }
    }

    void updateObjectPosition(std::string& obj_name, ReloPush::State& pose_in)
    {
        // it is supposed to be already exsiting in the map
        if ((*this).find(obj_name) == (*this).end()) {
            std::cerr << "[OBJ_UPDATE] Key '" << obj_name << "' does not exist in the map." << std::endl;
        }
        (*this)[obj_name].x = pose_in.x;
        (*this)[obj_name].y = pose_in.y;
    }

    // Add more member functions as needed
};


typedef ObjectMap GoalMap;


struct ObjectGoalPair
{
    std::string objectName;
    std::string goalName;
    std::size_t input_order;

    ObjectGoalPair()
        : input_order(ObjectInfo::unspecified_input_order)
    {}

    ObjectGoalPair(std::string obj, std::string goal,
                   std::size_t input_order_in = ObjectInfo::unspecified_input_order)
        : objectName(obj), goalName(goal), input_order(input_order_in)
    {}
};

#endif // OBJECTINFO_HPP
