#include <GraphBuilder.hpp>
#include "DubinsTools.h"
#include <PathPlanningTools.h>
#include <boost/graph/graphviz.hpp>
#include <fstream>
#include <cmath>     // std::sqrt
#include <iostream>  // std::cout

// -----------------------------------------------------------------
// Add a single vertex
// -----------------------------------------------------------------
Vertex addVertexToGraph(
    Graph &g,
    VertexType type,
    const std::string &name,
    int orientationIdx,
    double nominalOri,
    double x,
    double y,
    int numberOfSides)
{
    Vertex v = boost::add_vertex(g);
    g[v].type = type;
    g[v].name = name;
    g[v].orientationIndex = orientationIdx;
    g[v].nominalOrientation = nominalOri;
    g[v].x = x;
    g[v].y = y;
    g[v].numberOfSides = numberOfSides;

    return v;
}

// -----------------------------------------------------------------
// Feasibility check
// -----------------------------------------------------------------
bool canConnect(const VertexData &from, const VertexData &to)
{
    // Example: check distance < some threshold
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    const double THRESHOLD = 100.0;  // e.g. 100 units
    return (dist < THRESHOLD);
}

bool canConnectNormal(const VertexData &from, const VertexData &to)
{
    // Put your "normalMode" feasibility logic here.
    // Example: distance < threshold, no collision, etc.
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    // Some condition for normalMode
    const double NORMAL_THRESHOLD = 100.0;
    return (dist < NORMAL_THRESHOLD);
}

bool canConnectPreRelocation(const VertexData &from, const VertexData &to)
{
    // Put your "preRelocation" logic here.
    // Maybe it's more lenient or a different constraint.
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    // Some condition for preRelocation
    // e.g., bigger threshold or some alternative check
    const double PRE_RELOC_THRESHOLD = 150.0;
    return (dist < PRE_RELOC_THRESHOLD);
}


// // -----------------------------------------------------------------
// // Add an edge from v1->v2 if feasible
// // -----------------------------------------------------------------
// void addEdgeIfFeasible(Graph &g, Vertex v1, Vertex v2)
// {
//     const auto &data1 = g[v1];
//     const auto &data2 = g[v2];

//     if (!canConnect(data1, data2))
//         return;  // Not feasible

//     // Compute a simple weight as the Euclidean distance
//     double dx = data2.x - data1.x;
//     double dy = data2.y - data1.y;
//     double dist = std::sqrt(dx*dx + dy*dy);

//     // Minimal path data (just start & end)
//     //PathData pathData;
//     //pathData.waypoints.push_back({data1.x, data1.y});
//     //pathData.waypoints.push_back({data2.x, data2.y});

//     auto [e, inserted] = boost::add_edge(v1, v2, g);
//     if (inserted)
//     {
//         g[e].weight = dist;
//         //g[e].path   = pathData;
//     }
// }

// Check Dubins Validity
StatePathValidity check_dubins_validity(reloDubinsPath& dubins_in, PlanningContext& ctx)
{
    // For each path points
    // Check boundary
    // Check collision

    auto l = dubins_in.lengthCost(); // unit cost * turning rad
    auto num_pts = static_cast<size_t>(l/ctx.parameters.map_resolution);

    ompl::base::DubinsStateSpace dubinsSpace(ctx.parameters.turning_rad_pair.push);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(dubins_in.startState.x, dubins_in.startState.y);
    dubinsStart->setYaw(dubins_in.startState.yaw);
    OmplState *interState = (OmplState *)dubinsSpace.allocState();

    std::vector<State> main_push_path(num_pts);

    bool failed_flag = false;
    StateValidity out_validity = StateValidity::valid;

    // interpolate dubins path
    // Interpolate dubins path to check for collision on grid map
    //nav_msgs::Path single_path;
    //single_path.poses.resize(num_pts);
    if(num_pts>0){
        for (size_t np=0; np<num_pts; np++)
        {
            //auto start = std::chrono::steady_clock::now();
            jeeho_interpolate(dubinsStart, dubins_in.omplDubins, (double)np / (double)num_pts, interState, &dubinsSpace,
                              ctx.parameters.turning_rad_pair.push);

            State tempState(interState->getX(), interState->getY(),interState->getYaw());
            main_push_path[np] = tempState; // failed State at the last

            // check boundary
            auto tempValid = ctx.env.stateValid(tempState, ctx.parameters.car_width, ctx.parameters.obs_rad, ctx.parameters.LB);
            if(!tempValid)
            {
                // reason for failure
                out_validity = tempValid.get_validity();
                // raise failed flag
                failed_flag = true;

                break;
            }
        }
    }
    else{
        std::cout << "Path too short to interpolate" << std::endl;
        main_push_path.resize(1);
        main_push_path[0] = dubins_in.targetState;
    } // path is too short there is nothing to interpolate

    StatePathValidity out_pair(std::make_shared<StatePath>(main_push_path), out_validity);

    return out_pair;
}

//
// Normal Edge Connection (No Prerelocation)
//

StateValidity addEdgeNormalMode(
    Graph &g,
    Vertex v1,
    Vertex v2,
    PlanningContext& ctx
    )
{
    // 0) Extract vertex data
    const auto &data1 = g[v1];
    const auto &data2 = g[v2];

    // 1) Temporarily remove start from obstacle. If target is also an obstacle, remove it, too.
    std::vector<State> took_out(0);
    took_out.push_back(State(data1.x,data1.y,data1.nominalOrientation));
    ctx.env.remove_obs(State(data1.x,data1.y,data1.nominalOrientation));
    if(data2.type==VertexType::OBJECT_VERTEX)
    {
        took_out.push_back(State(data2.x,data2.y,data2.nominalOrientation));
        ctx.env.remove_obs(State(data2.x,data2.y,data2.nominalOrientation));
    }


    // Create start/goal states with (x, y, theta)
    // If 'State' is your class from attached references:
    State start(data1.x, data1.y, data1.getActualOrientation());
    State goal(data2.x, data2.y, data2.getActualOrientation());

    // 2) Call your Dubins planner.
    auto dubinsResult = PlanDubins(start, goal, ctx);

    // 3) Check if the path is valid (within boundary & collision-free)
    auto isPathValid = check_dubins_validity(dubinsResult.second, ctx);

    // put the obstacles back
    for(auto it : took_out)
        ctx.env.add_obs(it);


    // 4) If all waypoints are valid => add the edge to the graph
    //    We'll store the path + its length in EdgeData
    if(isPathValid)
    {
        Edge e;
        bool inserted;
        boost::tie(e, inserted) = boost::add_edge(v1, v2, g);
        if (inserted)
        {
            g[e].weight = dubinsResult.second.lengthCost();           // path length from planner
            g[e].mode   = ConnectionMode::NORMAL_MODE;
            g[e].paths = {dubinsResult.second};    // store entire path for reference
        }
        return StateValidity::valid;
    }
    else
    {
        return isPathValid.path_validity; //reason for failure
    }
}

bool addEdge(Graph &g, Vertex v1, Vertex v2, PlanningContext &ctx)
{
    const auto &data1 = g[v1];
    const auto &data2 = g[v2];

    if(data1.name != data2.name)
    {
        // try normal mode
        StateValidity normalEdge = addEdgeNormalMode(g, v1, v2, ctx);

        // Normal mode Failed
        if(normalEdge != StateValidity::valid)
        {
            // find pre-relocation
        }
    }
    // skip for same object


    // If both checks fail, we do NOT add any edge
    return false;
}

// -----------------------------------------------------------------
// Print graph info (vertices + edges)
// -----------------------------------------------------------------
void printGraphInfo(const Graph &g)
{
    using vertex_iter = boost::graph_traits<Graph>::vertex_iterator;
    vertex_iter vi, vi_end;
    for (boost::tie(vi, vi_end) = boost::vertices(g); vi != vi_end; ++vi)
    {
        Vertex v = *vi;
        const auto &vd = g[v];
        std::cout << "Vertex " << v << " (" << vd.name << ")\n";

        using out_edge_iter = boost::graph_traits<Graph>::out_edge_iterator;
        out_edge_iter ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(v, g); ei != ei_end; ++ei)
        {
            Edge e = *ei;
            const auto &ed = g[e];

            // Convert connection mode to a string
            std::string modeStr;
            switch(ed.mode)
            {
            case ConnectionMode::NORMAL_MODE:    modeStr = "normalMode"; break;
            case ConnectionMode::PRE_RELOCATION: modeStr = "preRelocation"; break;
            default:                             modeStr = "none";
            }

            Vertex tgt = boost::target(e, g);
            std::cout << "  -> Edge to vertex " << tgt
                      << ", weight=" << ed.weight
                      << ", mode=" << modeStr << "\n";
        }
        std::cout << std::endl;
    }
}


// -----------------------------------------------------------------
// Create multiple vertices for one object
// -----------------------------------------------------------------
std::vector<Vertex> createVerticesForObject(Graph &g, const ObjectInfo &obj)
{
    std::vector<Vertex> createdVertices;

    // If object has zero or negative sides, treat it as 1 (fallback)
    int sides = (obj.numberOfSides > 0) ? obj.numberOfSides : 1;

    for (int i = 0; i < sides; ++i)
    {
        Vertex v = addVertexToGraph(
            g,
            VertexType::OBJECT_VERTEX,
            obj.name,
            i,                         // orientationIndex
            obj.nominalOrientation,
            obj.x,
            obj.y,
            obj.numberOfSides
            );
        createdVertices.push_back(v);
    }

    return createdVertices;
}

// -----------------------------------------------------------------
// Create multiple vertices for one goal
// -----------------------------------------------------------------
std::vector<Vertex> createVerticesForGoal(Graph &g, const GoalInfo &goal)
{
    std::vector<Vertex> createdVertices;

    // If goal has zero or negative sides, treat it as 1 (fallback)
    int sides = (goal.numberOfSides > 0) ? goal.numberOfSides : 1;

    for (int i = 0; i < sides; ++i)
    {
        Vertex v = addVertexToGraph(
            g,
            VertexType::GOAL_VERTEX,
            goal.name,
            i,                          // orientationIndex
            goal.nominalOrientation,
            goal.x,
            goal.y,
            goal.numberOfSides
            );
        createdVertices.push_back(v);
    }

    return createdVertices;
}

// void connectVertexSets(Graph &g,
//                        const std::vector<Vertex> &fromSet,
//                        const std::vector<Vertex> &toSet, const PlanningParameters &params)
// {
//     for (auto v1 : fromSet)
//     {
//         for (auto v2 : toSet)
//         {
//             addEdge(g, v1, v2, params); // v1 -> v2
//         }
//     }
// }

void initGraph(Graph& g, ObjectMap& objects, GoalMap& goals)
{
    // We might store boundary in a global or pass it around as needed
    // For each object, create vertices (each orientation)
    std::vector<std::vector<Vertex>> objectVerts;
    objectVerts.reserve(objects.size());
    for (auto &oi : objects)
    {
        // Create vertices
        auto verts = createVerticesForObject(g, oi.second);
        // Also, if your VertexData has a 'radius' field, set it there
        // (In createVerticesForObject, you likely do it automatically)
        objectVerts.push_back(verts);
    }

    // Same for goals
    std::vector<std::vector<Vertex>> goalVerts;
    goalVerts.reserve(goals.size());
    for (auto &gi : goals)
    {
        auto verts = createVerticesForGoal(g, gi.second);
        // If createVerticesForGoal sets g[v].radius = gi.enclosingRadius,
        // then you have it in the graph
        goalVerts.push_back(verts);
    }
}

/*
void initGraph(Graph& g, ObjectList& objects, GoalList& goals)
{
    // We might store boundary in a global or pass it around as needed
    // For each object, create vertices (each orientation)
    std::vector<std::vector<Vertex>> objectVerts;
    objectVerts.reserve(objects.size());
    for (auto &oi : objects)
    {
        // Create vertices
        auto verts = createVerticesForObject(g, oi);
        // Also, if your VertexData has a 'radius' field, set it there
        // (In createVerticesForObject, you likely do it automatically)
        objectVerts.push_back(verts);
    }

    // Same for goals
    std::vector<std::vector<Vertex>> goalVerts;
    goalVerts.reserve(goals.size());
    for (auto &gi : goals)
    {
        auto verts = createVerticesForGoal(g, gi);
        // If createVerticesForGoal sets g[v].radius = gi.enclosingRadius,
        // then you have it in the graph
        goalVerts.push_back(verts);
    }
}
*/

/**
 * @brief Build edges among multiple objects and goals, ensuring
 *        goals have no outgoing edges.
 *
 * @param g             The graph
 * @param objectVerts   objectVerts[i] = all orientation-vertices for the i-th object
 * @param goalVerts     goalVerts[j]   = all orientation-vertices for the j-th goal
 */
/*
void buildAllEdges(
    Graph &g,
    const std::vector<std::vector<Vertex>> &objectVerts,
    const std::vector<std::vector<Vertex>> &goalVerts, const PlanningParameters &params)
{
    // 1) Connect objects among themselves (object -> object)
    //    For each pair of distinct sets i, j, connect both ways or one way, depending on your logic.
    //    If you want to allow object->object edges in both directions, you could call connectVertexSets
    //    twice (once from i->j, once from j->i).
    //    But let's assume you only want a single direction or you want to unify them. It's up to you.
    for (size_t i = 0; i < objectVerts.size(); i++)
    {
        for (size_t j = i + 1; j < objectVerts.size(); j++)
        {
            // object i -> object j
            connectVertexSets(g, objectVerts[i], objectVerts[j], params);
            // object j -> object i, if you want edges in both directions
            connectVertexSets(g, objectVerts[j], objectVerts[i], params);
        }
    }

    // 2) Connect objects to goals (object -> goal)
    //    But do NOT connect the other way around, because goals have no outgoing edges.
    for (auto &oSet : objectVerts)
    {
        for (auto &gSet : goalVerts)
        {
            connectVertexSets(g, oSet, gSet, params);
        }
    }

    // 3) No goal->goal edges (since goals have no outgoing edges).
    //    So we do not connect goal sets among themselves.
}
*/

/**
 * @brief Returns all vertices in the graph whose type == OBJECT_VERTEX.
 */
std::vector<Vertex> getAllObjectVertices(const Graph &g)
{
    std::vector<Vertex> objectVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::OBJECT_VERTEX)
        {
            objectVerts.push_back(v);
        }
    }
    return objectVerts;
}

/**
 * @brief Returns all vertices in the graph whose type == GOAL_VERTEX.
 */
std::vector<Vertex> getAllGoalVertices(const Graph &g)
{
    std::vector<Vertex> goalVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::GOAL_VERTEX)
        {
            goalVerts.push_back(v);
        }
    }
    return goalVerts;
}

/**
 * @brief Returns all vertices in the graph that match
 *        - type == OBJECT_VERTEX
 *        - name == objectName
 */
std::vector<Vertex> getObjectVertices(const Graph &g, const std::string &objectName)
{
    std::vector<Vertex> objectVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::OBJECT_VERTEX && vData.name == objectName)
        {
            objectVerts.push_back(v);
        }
    }
    return objectVerts;
}

/**
 * @brief Returns all vertices in the graph that match
 *        - type == GOAL_VERTEX
 *        - name == goalName
 */
std::vector<Vertex> getGoalVertices(const Graph &g, const std::string &goalName)
{
    std::vector<Vertex> goalVerts;

    auto vertsRange = boost::vertices(g);
    for (auto it = vertsRange.first; it != vertsRange.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::GOAL_VERTEX && vData.name == goalName)
        {
            goalVerts.push_back(v);
        }
    }
    return goalVerts;
}

/**
 * @brief Build edges by scanning the entire graph for object vertices and goal vertices,
 *        then connecting them.
 *
 *        - All object vertices are stored in 'objectList'
 *        - All goal vertices in 'goalList'
 *        - Then object->object and object->goal edges are created,
 *          but goals have no outgoing edges.
 *
 * @param g    The graph (already containing object and goal vertices).
 * @param ctx  Your PlannerContext (or environment + parameters).
 */
void buildAllEdges(Graph &g, PlanningContext ctx)
{
    // 0) Init env with obstacles
    std::unordered_set<State> obs;
    for(auto& it : ctx.mo_list)
    {
        obs.insert(State(it.second.x, it.second.y, it.second.nominalOrientation));
        //obs.insert(State(it.get_x(),it.get_y(),0));
    }

    for(auto& it : ctx.delivered_list)
    {
        obs.insert(State(it.second.x, it.second.y, it.second.nominalOrientation));
        //obs.insert(State(it.get_x(),it.get_y(),0));
    }
    ctx.updateObs(obs);


    // 1) Separate object vs. goal vertices
    std::vector<Vertex> objectVerts;
    std::vector<Vertex> goalVerts;

    // Get iterators for all vertices in g
    auto vertsPair = boost::vertices(g);  // returns (begin, end)
    for (auto it = vertsPair.first; it != vertsPair.second; ++it)
    {
        Vertex v = *it;
        const auto &vData = g[v];
        if (vData.type == VertexType::OBJECT_VERTEX)
        {
            objectVerts.push_back(v);
        }
        else if (vData.type == VertexType::GOAL_VERTEX)
        {
            goalVerts.push_back(v);
        }
    }

    // 2) Connect objects among themselves (object->object).
    //    You can do object->object in both directions or just one direction.
    for (size_t i = 0; i < objectVerts.size(); ++i)
    {
        for (size_t j = i + 1; j < objectVerts.size(); ++j)
        {

            // object i -> object j
            addEdge(g, objectVerts[i], objectVerts[j], ctx);
            // object j -> object i
            addEdge(g, objectVerts[j], objectVerts[i], ctx);
        }
    }

    // 3) Connect objects to goals (object->goal), but NOT goal->object
    for (auto vObj : objectVerts)
    {
        for (auto vGoal : goalVerts)
        {
            addEdge(g, vObj, vGoal, ctx);
        }
    }

    // 4) No goal->goal edges (since goals have no outgoing edges).
    //    So we do nothing for goal->goal.
}


// -----------------------------------------------------------------
// Visualize Graph
// -----------------------------------------------------------------
void writeGraphToDot(const Graph &g, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << " for writing.\n";
        return;
    }

    // We create lambdas that describe how to write properties for vertices/edges.
    auto vertexWriter = [&](std::ostream &out, const Vertex v)
    {
        const auto &vd = g[v];
        // Create a label string. For example:
        //   name (orientationIndex)
        //   position: (x,y)
        //   or anything else you’d like to display!
        out << "[label=\""
            << vd.name << "(" << vd.orientationIndex << ")\\n"
            << "pos=(" << vd.x << "," << vd.y << ")\\n"
            << "ori=" << vd.getActualOrientation()
            << "\"]";
    };

    auto edgeWriter = [&](std::ostream &out, const Edge e)
    {
        // Show the edge weight in the label
        out << "[label=\"" << g[e].weight << "\"]";
    };

    // Write to .dot
    boost::write_graphviz(file, g, vertexWriter, edgeWriter);

    file.close();
    std::cout << "Wrote graph to " << filename << "\n";
}

// A helper function to write a .dot file that includes node positions.
void writeGraphWithCoordinates(const Graph &g, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << "\n";
        return;
    }

    double scale = 100.0;  // Increase to spread nodes more

    auto vertexWriter = [&](std::ostream &out, Vertex v)
    {
        const auto &vd = g[v];
        out << "["
            << "label=\"" << vd.name << "\\n"
            << "ori=" << vd.getActualOrientation() << "\" "
            << "fontsize=\"10\" "
            << "shape=\"circle\" width=\"0.7\" fixedsize=\"true\" "
            << "style=\"filled\" fillcolor=\""
            << (vd.type == VertexType::OBJECT_VERTEX ? "lightblue" : "yellow") << "\" "
            << "pos=\"" << (scale * vd.x) << ","
            << (scale * vd.y) << "!\""
            << "]";
    };

    auto edgeWriter = [&](std::ostream &out, Edge e)
    {
        out << "[label=\"" << g[e].weight << "\"]";
    };

    auto graphWriter = [&](std::ostream &out)
    {
        out << "graph [layout=neato, overlap=false, splines=line];\n";
    };

    boost::write_graphviz(file, g, vertexWriter, edgeWriter, graphWriter);
    file.close();
    std::cout << "Wrote scaled graph to " << filename << "\n";
}
