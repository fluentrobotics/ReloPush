#ifndef GRAPHDATA_HPP
#define GRAPHDATA_HPP

#include <string>
#include <vector>
#include <memory>
#include <boost/graph/adjacency_list.hpp>
//#include <cmath>
#include <State.h>
#include <variant>
#include <DubinsTools.h>

using EdgePathTypes = std::variant<reloDubinsPath, ReloPush::StatePathPtr>; // for storing in edge

struct EdgePath
{
    bool is_pushing;
    EdgePathTypes path;

    EdgePath()
    {}

    EdgePath(bool is_pushing_in, EdgePathTypes path_in) : is_pushing(is_pushing_in), path(path_in)
    {}

    ReloPush::StatePathPtr toStatePath(double resolution = 0.1) const
    {
        ReloPush::StatePathPtr statePath;
        // Check if the variant holds a StatePathPtr
        if (std::holds_alternative<ReloPush::StatePathPtr>(path))
        {
            statePath = std::get<ReloPush::StatePathPtr>(path);
        }
        // If needed, handle reloDubinsPath here (currently ignored)
        else if(std::holds_alternative<reloDubinsPath>(path))
        {
            auto dubinsPath = std::get<reloDubinsPath>(path);
            statePath = dubinsPath.interpolate(resolution); // todo: parse map resolution
        }

        return statePath;
    }

    void print(bool add_lines = true) const
    {
        auto path_to_print = toStatePath();
        for(auto it : *path_to_print)
        {
            it.print(add_lines);
        }
    }

    ReloPush::State getFirstWaypoint()
    {
        ReloPush::State out;

        if (std::holds_alternative<ReloPush::StatePathPtr>(path))
        {
            auto tmp_path = std::get<ReloPush::StatePathPtr>(path);
            out = tmp_path->at(0);
        }
        // If needed, handle reloDubinsPath here (currently ignored)
        else if(std::holds_alternative<reloDubinsPath>(path))
        {
            auto tmp_path = std::get<reloDubinsPath>(path);
            out = tmp_path.startState;
        }

        return out;
    }

    ReloPush::State getLastWaypoint()
    {
        ReloPush::State out;

        if (std::holds_alternative<ReloPush::StatePathPtr>(path))
        {
            auto tmp_path = std::get<ReloPush::StatePathPtr>(path);
            out = tmp_path->back();
        }
        // If needed, handle reloDubinsPath here (currently ignored)
        else if(std::holds_alternative<reloDubinsPath>(path))
        {
            auto tmp_path = std::get<reloDubinsPath>(path);
            out = tmp_path.targetState;
        }

        return out;
    }

    double getLength(void)
    {
        double out_length;
        if (std::holds_alternative<ReloPush::StatePathPtr>(path))
        {
            auto statePath = std::get<ReloPush::StatePathPtr>(path);
            out_length = ReloPush::StatePathlength(*statePath);
        }
        else if(std::holds_alternative<reloDubinsPath>(path))
        {
            auto dubinsPath = std::get<reloDubinsPath>(path);
            out_length = dubinsPath.lengthCost();
        }
        return out_length;
    }
};

using EdgePathPtr = std::shared_ptr<EdgePath>;

/**
 * @brief Distinguish whether a vertex is for an object or a goal.
 */
enum class VertexType
{
    OBJECT_VERTEX,
    GOAL_VERTEX
};

enum class ConnectionMode
{
    NONE,           // no valid mode
    NORMAL_MODE,    // "normalMode"
    PRE_RELOCATION  // "preRelocation"
};

struct PreRelocationInfo
{
    bool used;               ///< true if we did pre-relocation
    double xRelocated_robot;       ///< relocated X
    double yRelocated_robot;       ///< relocated Y
    double yawReloacted_robot;
    double xRelocated_object;
    double yRelocated_object;
    double yawRelocated_object;

    double extraCost;        ///< cost of that relocation alone
    int relocatingIndex;     ///< which orientation axis we used, e.g. i in [0..nSides-1]
    StateValidity reason;   ///< reason for prerelocation
    // or double relocatingAngle; // if you prefer storing actual angle
};


/**
 * @brief The data stored at each vertex in the graph.
 *
 * We do NOT store "actualOrientation" directly.
 * Instead, we compute it as:
 *
 *    actualOrientation = nominalOrientation + orientationIndex * (2π / numberOfSides)
 */
struct VertexData
{
    VertexType type;            ///< Is this an OBJECT or GOAL?
    std::string name;           ///< Object name or Goal label

    int orientationIndex;       ///< The discrete orientation index (0..n-1)
    double nominalOrientation;  ///< The "offset" orientation
    double x;                   ///< Position X
    double y;                   ///< Position Y
    int numberOfSides;          ///< e.g., 4 for a box with 4 discrete sides
    double radius;

    /**
     * Default constructor
     */
    VertexData()
        : type(VertexType::OBJECT_VERTEX),
        name(""),
        orientationIndex(0),
        nominalOrientation(0.0),
        x(0.0),
        y(0.0),
        numberOfSides(0)
    {}

    /**
     * @brief Compute the actual orientation = nominal + index*(2π / numberOfSides).
     */
    double getActualOrientation() const
    {
        // If numberOfSides <= 0, fallback or just return nominalOrientation
        if (numberOfSides <= 0)
            return nominalOrientation;

        double stepAngle = (2.0 * M_PI) / static_cast<double>(numberOfSides);
        return nominalOrientation + (orientationIndex * stepAngle);
    }
};

// /**
//  * @brief The data stored at each edge in the graph.
//  */
// struct PathData
// {
//     // A list of (x, y, theta) states along the path
//     std::vector<EdgePathTypes> paths; // Dubins or Waypoints
// };

struct EdgeData
{
    double weight; // weight of the edge

    VertexData srcVertexData; // source vertex
    VertexData sinkVertexData; // sink vertex

    // Record which mode was used to create this edge (normal or prerelocation)
    ConnectionMode mode;

    // Additional info about pre-relocation (if used)
    PreRelocationInfo preRelo;

    //PathData paths;
    std::vector<EdgePathPtr> paths; // Dubins or Waypoints Segments

    EdgeData()
        : weight(0.0),
        mode(ConnectionMode::NONE)
    {
        preRelo.used        = false;
        preRelo.xRelocated_robot  = 0.0;
        preRelo.yRelocated_robot  = 0.0;
        preRelo.yawReloacted_robot = 0.0;
        preRelo.xRelocated_object = 0.0;
        preRelo.yRelocated_object = 0.0;
        preRelo.yawRelocated_object = 0.0;
        preRelo.extraCost   = 0.0;
        preRelo.relocatingIndex = -1;
    }

    void printPath(bool add_lines = true) const
    {
        for(auto& it : paths)
        {
            it->print(add_lines);
        }
    }
};

/**
 * @brief Our directed, weighted graph type using Boost.
 */
using Graph = boost::adjacency_list<
    boost::listS,               // Edge container type
    boost::vecS,                // Vertex container type
    boost::directedS,           // Directed graph
    VertexData,                 // Vertex property
    EdgeData                    // Edge property
    >;

/**
 * @brief Handy descriptors for vertices and edges.
 */
using Vertex = boost::graph_traits<Graph>::vertex_descriptor;
using Edge   = boost::graph_traits<Graph>::edge_descriptor;

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

#endif // GRAPHDATA_HPP
