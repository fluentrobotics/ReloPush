#ifndef GRAPHDATA_HPP
#define GRAPHDATA_HPP

#include <string>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
//#include <cmath>
#include <State.h>
#include <variant>
#include <DubinsTools.h>

using EdgePathTypes = std::variant<reloDubinsPath, StatePathPtr>; // for storing in edge

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
    bool used;              ///< Did we actually do a pre-relocation?
    double xRelocated;      ///< The relocated X of the start
    double yRelocated;      ///< The relocated Y of the start
    double extraCost;       ///< The cost (distance) of this relocation alone
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
    double nominalOrientation;  ///< The "base" orientation
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
    double weight;
    //PathData paths;
    std::vector<EdgePathTypes> paths; // Dubins or Waypoints

    // Record which mode was used to create this edge
    ConnectionMode mode;

    // Additional info about pre-relocation
    PreRelocationInfo preRelo;

    EdgeData()
        : weight(0.0),
        mode(ConnectionMode::NONE)
    {
        preRelo.used        = false;
        preRelo.xRelocated  = 0.0;
        preRelo.yRelocated  = 0.0;
        preRelo.extraCost   = 0.0;
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

#endif // GRAPHDATA_HPP
