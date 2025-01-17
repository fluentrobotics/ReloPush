#include "GraphBuilder.hpp"
#include "InputParser.hpp"
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>

int main(int argc, char *argv[])
{
    //if (argc < 2)
   // {
   //     std::cerr << "Usage: " << argv[0] << " <input_file>\n";
    //    return 1;
    //}

    //std::string filename = argv[1];
    std::string filename = "input2.txt";

    // We'll parse the boundary, plus objects & goals
    WorkspaceBoundary boundary;
    //std::vector<ObjectInfo> objects;
    //std::vector<GoalInfo> goals;
    std::unordered_map<std::string, ObjectInfo> objects;
    std::unordered_map<std::string, GoalInfo> goals;

    //std::vector<ObjectGoalPair> objGoalPairs;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs; // key: object name


    if (!parseInputFile(filename, boundary, objects, goals, objGoalPairs))
    {
        std::cerr << "Parse failed.\n";
        return 1;
    }

    // The final sequence of chosen allocations
    std::vector<FinalAllocation> finalSequence;

    /*
    // Create graph
    Graph g;
    // Init graph
    initGraph(g, objects, goals);

    // set up the parameters
    PlanningParameters params;
    params.boundary = boundary; // todo: parse in a separate method

    // Initialize Planning Context
    PlanningContext PlanCtx(params, objects);


    // Build edges, taking into account boundary or radius in your feasibility checks
    buildAllEdges(g, PlanCtx);

    // pick a task
    // pick_best(objGoalPairs,g);
    auto result = computeAndSortAllPairs(g, objGoalPairs);


    auto task_candidate = findAbsoluteLowestCost(result);
    */


    // 2) While we still have pairs left to allocate
    while (!objGoalPairs.empty())
    {
        std::cout << "\n============================\n"
                  << "Remaining pairs: " << objGoalPairs.size() << "\n";

        // (a) Build the graph from scratch
        Graph g;
        initGraph(g, objects, goals);

        // (b) Setup the PlanningParameters, boundary, etc.
        PlanningParameters params;   //?inspect
        params.boundary = boundary; // or fill as needed

        // (c) Initialize PlanningContext
        PlanningContext planCtx(params, objects);

        // (d) Build all edges
        buildAllEdges(g, planCtx);

        // (e) Compute cost matrices for all *remaining* pairs
        auto pairResults = computeAndSortAllPairs(g, objGoalPairs);

        // (f) Find the absolute lowest cost among them
        auto best = findAbsoluteLowestCost(pairResults);

        // Check if we found any feasible solution
        if (best.indexInArray < 0 || best.cost == std::numeric_limits<double>::infinity())
        {
            std::cout << "No feasible pair found (or no pairs left)!\n";
            break; // You can break or continue, depending on desired behavior
        }

        // (g) Print or store the best
        std::cout << "BEST PAIR => " << best.objectName
                  << " -> " << best.goalName
                  << ", cost=" << best.cost
                  << ", row=" << best.row
                  << ", col=" << best.col << "\n";

        // Insert into final sequence
        FinalAllocation chosen;
        chosen.object = objects[best.objectName];
        chosen.goal   = goals[best.goalName];
        chosen.cost       = best.cost;
        chosen.row        = best.row;
        chosen.col        = best.col;
        chosen.startPose = State(chosen.object.x,chosen.object.y,chosen.object.getOrientation(chosen.row));
        chosen.goalPose = State(chosen.goal.x, chosen.goal.y, chosen.goal.getOrientation(chosen.col));

        // Look up the actual edge in the graph & read pre-relocation info
        auto objVerts  = getObjectVertices(g, best.objectName);
        auto goalVerts = getGoalVertices(g, best.goalName);

        if (best.row < (int)objVerts.size() && best.col < (int)goalVerts.size())
        {
            Vertex vObj  = objVerts[best.row];
            Vertex vGoal = goalVerts[best.col];

            Edge e; bool hasEdge;
            boost::tie(e, hasEdge) = boost::edge(vObj, vGoal, g);
            if (hasEdge)
            {
                const auto &ed = g[e];
                if (ed.mode == ConnectionMode::PRE_RELOCATION && ed.preRelo.used)
                {
                    chosen.usedPreRelocation = true;
                    chosen.xRelocated        = ed.preRelo.xRelocated;
                    chosen.yRelocated        = ed.preRelo.yRelocated;
                    chosen.preReloCost       = ed.preRelo.extraCost;
                    chosen.relocatingIndex   = ed.preRelo.relocatingIndex;
                }
            }
        }

        finalSequence.push_back(chosen);

        // (h) Remove the chosen pair from objGoalPairs
        //     so we don't pick the same pair again in the next loop.
        //     best.indexInArray is the index in 'pairResults',
        //     but we need the *corresponding* index in objGoalPairs.
        //
        //  Because we returned 'pairResults' sorted by bestCost, the index
        //  might not match the original order in objGoalPairs. We must track that carefully.


        // or if you do not track original index, you can do a small loop to find matching
        // objectName/goalName in objGoalPairs and remove that one. For example:

        objGoalPairs.erase(best.objectName);
        objects.erase(best.objectName);
        goals.erase(best.goalName);

        // That's it for one iteration. The loop continues until objGoalPairs is empty or no feasible solution.

        // Print result
        //printGraphInfo(g);

        // Export to txt
        //exportToTxt(4,5,objects,goals,g,"test.txt");

    }

    std::cout << "\nFinal sequence of chosen tasks:\n";
    for (auto &fa : finalSequence)
    {
        std::cout << "Object = " << fa.object.name
                  << ", Goal = " << fa.goal.name
                  << ", cost = " << fa.cost << "\n"
                  << "  start yaw = " << fa.startPose.yaw
                  << ", goal yaw = " << fa.goalPose.yaw << "\n";

        if (fa.usedPreRelocation)
        {
            std::cout << "  **Used pre-relocation**: Moved start from ("
                      << fa.object.x << ", " << fa.object.y << ") to ("
                      << fa.xRelocated << ", " << fa.yRelocated << "), cost="
                      << fa.preReloCost << ", orientation index="
                      << fa.relocatingIndex << "\n";
        }
        std::cout << std::endl;
    }

    return 0;
}
