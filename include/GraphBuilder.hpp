#ifndef GRAPHBUILDER_HPP
#define GRAPHBUILDER_HPP

#include "GraphData.hpp"
//#include "Parameters.hpp"
#include <PlanningContext.hpp>
#include <vector>
#include <PreReloOptimization.hpp>
#include <PlanHybridAstar.hpp>


/**
 * @brief Add a new vertex to the graph. This can be either OBJECT_VERTEX or GOAL_VERTEX.
 *
 * @param g    Graph
 * @param type Which type (OBJECT_VERTEX or GOAL_VERTEX)
 * @param name Name or label
 * @param orientationIdx  The discrete orientation index
 * @param nominalOri      Base orientation
 * @param x,y             Position
 * @param numberOfSides   For computing the discrete steps
 * @return The newly created vertex
 */
Vertex addVertexToGraph(
    Graph &g,
    VertexType type,
    const std::string &name,
    int orientationIdx,
    double nominalOri,
    double x,
    double y,
    int numberOfSides
    );

/**
 * @brief Feasibility check between two vertices.
 *
 * E.g., checks if distance < threshold, or collision, etc.
 */
bool canConnect(const VertexData &from, const VertexData &to);
bool canConnectNormal(const VertexData &from,
                      const VertexData &to);
bool canConnectPreRelocation(const VertexData &from, const VertexData &to);

// Pre-push to approach
ReloPush::State find_pre_push(ReloPush::State& goalState, float distance);


/**
 * @brief Add a directed edge from v1->v2 if feasible (one direction).
 */
//void addEdgeIfFeasible(Graph &g, Vertex v1, Vertex v2);

bool addEdge(Graph &g, Vertex v1, Vertex v2, PlanningContext& ctx);

/**
 * @brief Print info about each vertex and its outgoing edges.
 */
void printGraphInfo(const Graph &g);

/**
 * @brief Create multiple vertices for a single object, one per orientation index.
 *
 * For an object with numberOfSides = N, we'll create N vertices (orientationIndex = 0..N-1).
 * If numberOfSides <= 0, we just create 1 vertex (index=0) as a fallback.
 *
 * @param g   Graph reference
 * @param obj The object-level data
 * @return A vector of newly created vertex descriptors
 */
std::vector<Vertex> createVerticesForObject(Graph &g, const ObjectInfo &obj);

/**
 * @brief Create multiple vertices for a single goal, one per orientation index.
 *
 * Exactly analogous to createVerticesForObject but uses VertexType::GOAL_VERTEX.
 *
 * @param g    Graph reference
 * @param goal The goal-level data
 * @return A vector of newly created vertex descriptors
 */
std::vector<Vertex> createVerticesForGoal(Graph &g, const GoalInfo &goal);

// /**
//  * @brief Connect every vertex in fromSet to every vertex in toSet by calling addEdgeIfFeasible.
//  */
// void connectVertexSets(Graph &g,
//                        const std::vector<Vertex> &fromSet,
//                        const std::vector<Vertex> &toSet, const PlanningParameters &params);

// Init Graph
void initGraph(Graph& g, ObjectMap& objects, GoalMap& goals);
// void initGraph(Graph& g, ObjectList& objects, GoalList& goals);

/**
 * @brief Returns all vertices in the graph whose type == OBJECT_VERTEX.
 */
std::vector<Vertex> getAllObjectVertices(const Graph &g);

/**
 * @brief Returns all vertices in the graph whose type == GOAL_VERTEX.
 */
std::vector<Vertex> getAllGoalVertices(const Graph &g);

/**
 * @brief Returns all vertices in the graph that match
 *        - type == OBJECT_VERTEX
 *        - name == objectName
 */
std::vector<Vertex> getObjectVertices(const Graph &g, const std::string &objectName);

/**
 * @brief Returns all vertices in the graph that match
 *        - type == GOAL_VERTEX
 *        - name == goalName
 */
std::vector<Vertex> getGoalVertices(const Graph &g, const std::string &goalName);

/**
 * @brief Connect all objects among themselves and objects to goals (but not goal->anything).
 *
 * @param g             The graph
 * @param objectVerts   objectVerts[i] = the set of vertices for the i-th object
 * @param goalVerts     goalVerts[j]   = the set of vertices for the j-th goal

void buildAllEdges(
    Graph &g,
    const std::vector<std::vector<Vertex>> &objectVerts,
    const std::vector<std::vector<Vertex>> &goalVerts,
    const PlanningParameters &params);
*/

void buildAllEdges(Graph &g, PlanningContext ctx);

/**
 * @brief Write the graph to a .dot file for visualization.
 *
 * @param g         The graph to write
 * @param filename  The output .dot file
 */
void writeGraphToDot(const Graph &g, const std::string &filename);

void writeGraphWithCoordinates(const Graph &g, const std::string &filename);

#endif // GRAPHBUILDER_HPP
