#include "GraphBuilder.hpp"
#include "InputParser.hpp"
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>

#include <QApplication>
#include <QMainWindow>
#include <Visualization/VisualizationWidget.h>
#include <Visualization/QtMainControlWindow.h>


// =====================================================================
// Helper functions declarations
// =====================================================================

/// Runs the replan loop for the current allocation. Returns true if an allocation was found.
/// The function encapsulates both the intermediate and final approach planning steps.
bool planAllocation(
    std::unordered_map<std::string, MatrixPairResult>& pairResults,
    LowestCostInfo &best,
    std::vector<EdgePath>& obsReloPathList,
    std::unordered_map<std::string, ReloPush::State>& toUpdate,
    PlanningContext &planCtx,
    const std::unordered_map<std::string, ObjectInfo>& objects,
    const std::unordered_map<std::string, GoalInfo>& goals,
    PlanningContext &ctxSnapshot);

/// Encapsulates the duplicated code for planning an "ObsRelo" edge.
/// Returns true if the planning succeeded, or false (so that the calling loop can replan).
bool planObsReloEdge(
    const ReloPush::State &fromState,
    const ReloPush::State &toState,
    const ReloPush::State &obsRemove,
    const ReloPush::State &obsAdd,
    PlanningContext &planCtx,
    std::vector<EdgePath>& obsReloPathList,
    std::string pivotObjName,
    std::unordered_map<std::string, ReloPush::State>& toUpdate);

/// Updates object positions using the provided update map.
void updateObjectPositions(
    std::unordered_map<std::string, ObjectInfo>& objects,
    const std::unordered_map<std::string, ReloPush::State>& toUpdate);

/// Prints the best allocation found.
void printBestPair(const LowestCostInfo &best);

/// Creates a FinalAllocation object from the best allocation.
FinalAllocation createFinalAllocation(
    const LowestCostInfo &best,
    const std::unordered_map<std::string, ObjectInfo>& objects,
    const std::unordered_map<std::string, GoalInfo>& goals,
    const std::unordered_map<std::string, MatrixPairResult>& pairResults,
    const std::vector<EdgePath>& obsReloPathList,
    const PlanningContext &ctxSnapshot);

/// Prints the final sequence of allocations.
void printFinalSequence(const std::vector<FinalAllocation>& finalSequence);

/// Sets up the visualization windows.
void setupVisualization(const std::vector<FinalAllocation>& finalSequence);

// =====================================================================
// Helper functions definitions
// =====================================================================

bool planAllocation(
    std::unordered_map<std::string, MatrixPairResult>& pairResults,
    LowestCostInfo &best,
    std::vector<EdgePath>& obsReloPathList,
    std::unordered_map<std::string, ReloPush::State>& toUpdate,
    PlanningContext &planCtx,
    const std::unordered_map<std::string, ObjectInfo>& objects,
    const std::unordered_map<std::string, GoalInfo>& goals,
    PlanningContext &ctxSnapshot)
{
    // This while loop encapsulates your replan loop.
    while (true)
    {
        // Save snapshot of planning context
        ctxSnapshot = PlanningContext(planCtx);

        auto best_pick = findAbsoluteLowestCost(pairResults);
        if (best_pick.row == -1 || best_pick.col == -1)
            break; // no pick found

        if (best_pick.cost == std::numeric_limits<double>::infinity())
        {
            std::cout << "No feasible pair found (or no pairs left)!\n";
            break;
        }

        // Retrieve the best pair and extract the best path entry.
        auto best_pair = pairResults[best_pick.objectName];
        auto temp = best_pair.matrixResult->getBestPathMatEntry();

        // --- Process intermediate ObsRelo edges (if there are multiple approaches) ---
        for (size_t obs = 1; obs < temp.obsReloList.size(); obs++)
        {
            // For each intermediate edge, plan from the previous state's post-approach to
            // the next state's pre-approach.
            auto pivotObj = temp.vertexChain[obs]; // used later to update object state
            auto prev_pair = temp.obsReloList[obs - 1];
            auto next_pair = temp.obsReloList[obs];

            auto fromState = prev_pair.second;
            auto toState = next_pair.first;

            // Compute pre-push states
            auto fromState_prepush = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
            auto toState_prepush   = find_pre_push(toState, planCtx.parameters.PrePush_dist);

            // Attempt planning the edge (note: planObsReloEdge encapsulates the obstacle update logic)
            if (!planObsReloEdge(fromState_prepush, toState_prepush,
                                 prev_pair.first, prev_pair.second,
                                 planCtx, obsReloPathList, pivotObj.name, toUpdate))
            {
                // Planning failed; update the cost matrix accordingly and try next best.
                best_pair.matrixResult->sortedEntries.erase(
                    best_pair.matrixResult->sortedEntries.begin());
                best_pair.matrixResult->costMat(best_pick.row, best_pick.col) = std::numeric_limits<double>::infinity();
                continue;
            }
        }

        // --- Process final ObsRelo edge to final push ---
        if (temp.obsReloList.size() > 0)
        {
            auto lastObsObj = temp.vertexChain[temp.vertexChain.size() - 2]; // second last in chain
            auto last_pair = temp.obsReloList.back();

            auto fromState_prepush = find_pre_push(last_pair.second, planCtx.parameters.PrePush_dist);

            // Get the object info to compute the final approach pose.
            auto best_obj = objects.at(best_pick.objectName);
            ReloPush::State final_approach(best_obj.x, best_obj.y, best_obj.getOrientation(best_pick.row));
            auto toState_prepush = find_pre_push(final_approach, planCtx.parameters.PrePush_dist);

            if (!planObsReloEdge(fromState_prepush, toState_prepush,
                                 last_pair.first, last_pair.second,
                                 planCtx, obsReloPathList, lastObsObj.name, toUpdate))
            {
                // Planning failed; update the cost matrix and try again.
                best_pair.matrixResult->sortedEntries.erase(
                    best_pair.matrixResult->sortedEntries.begin());
                best_pair.matrixResult->costMat(best_pick.row, best_pick.col) = std::numeric_limits<double>::infinity();
                continue;
            }
        }

        // If we reach here, planning was successful.
        best = best_pick;
        break;
    }

    // Return true if a valid best allocation was found.
    if (best.row == -1 || best.cost == std::numeric_limits<double>::infinity())
        return false;
    return true;
}


bool planObsReloEdge(
    const ReloPush::State &fromState,
    const ReloPush::State &toState,
    const ReloPush::State &obsRemove,
    const ReloPush::State &obsAdd,
    PlanningContext &planCtx,
    std::vector<EdgePath>& obsReloPathList,
    std::string pivotObjName,
    std::unordered_map<std::string, ReloPush::State>& toUpdate)
{
    // Update obstacles in the planning context.
    planCtx.env.remove_obs(obsRemove);
    planCtx.env.add_obs(obsAdd);

    // Attempt planning the edge using Hybrid A* (with pre-push states).
    auto res = planHybridAstar(fromState, toState, planCtx, true);
    if (!res->success)
    {
        // Revert obstacle update if planning fails.
        planCtx.env.add_obs(obsRemove);
        planCtx.env.remove_obs(obsAdd);
        return false;
    }

    // If planning succeeds, create the two-edge path:
    // 1) From pre-push of obsRemove to fromState.
    ReloPush::StatePath pre_pair_path = {
        find_pre_push(obsRemove, planCtx.parameters.PrePush_dist),
        fromState
    };
    obsReloPathList.push_back(EdgePath(true, std::make_shared<ReloPush::StatePath>(pre_pair_path)));
    // 2) The planned path.
    obsReloPathList.push_back(EdgePath(false, res->getPathPtr(true)));

    // Queue the update for the object state.
    toUpdate[pivotObjName] = fromState;
    return true;
}


void updateObjectPositions(
    std::unordered_map<std::string, ObjectInfo>& objects,
    const std::unordered_map<std::string, ReloPush::State>& toUpdate)
{
    for (const auto& pair : toUpdate)
    {
        std::cout << "Updating " << pair.first << " to state ("
                  << pair.second.x << ", " << pair.second.y << ")\n";
        objects[pair.first].x = pair.second.x;
        objects[pair.first].y = pair.second.y;
        // Update orientation if necessary.
    }
}


void printBestPair(const LowestCostInfo &best)
{
    std::cout << "BEST PAIR => " << best.objectName
              << " -> " << best.goalName
              << ", cost = " << best.cost
              << ", row = " << best.row
              << ", col = " << best.col << "\n";
}


FinalAllocation createFinalAllocation(
    const LowestCostInfo &best,
    const std::unordered_map<std::string, ObjectInfo>& objects,
    const std::unordered_map<std::string, GoalInfo>& goals,
    const std::unordered_map<std::string, MatrixPairResult>& pairResults,
    const std::vector<EdgePath>& obsReloPathList,
    const PlanningContext &ctxSnapshot)
{
    FinalAllocation alloc;
    alloc.object = objects.at(best.objectName);
    alloc.goal   = goals.at(best.goalName);
    alloc.cost   = best.cost;
    alloc.row    = best.row;
    alloc.col    = best.col;
    alloc.startPose = ReloPush::State(alloc.object.x, alloc.object.y, alloc.object.getOrientation(alloc.row));
    alloc.goalPose  = ReloPush::State(alloc.goal.x, alloc.goal.y, alloc.goal.getOrientation(alloc.col));

    // Retrieve the best path from the pair result.
    alloc.paths = pairResults.at(best.objectName).getBestPath().edgesInfo;
    alloc.obsReloPaths = std::make_shared<std::vector<EdgePath>>(obsReloPathList);
    alloc.snapshot = ctxSnapshot;
    return alloc;
}


void printFinalSequence(const std::vector<FinalAllocation>& finalSequence)
{
    std::cout << "\nFinal sequence of chosen tasks:\n";
    for (const auto &fa : finalSequence)
    {
        std::cout << "Object = " << fa.object.name
                  << ", Goal = " << fa.goal.name
                  << ", cost = " << fa.cost << "\n"
                  << "  start yaw = " << fa.startPose.yaw
                  << ", goal yaw = " << fa.goalPose.yaw;

        if(fa.paths.size()>1)
            std::cout << " *PreRelo-used " << std::endl;

        if (fa.obsReloPaths->size() > 0)
            std::cout << "  ObsRelo steps: " << fa.obsReloPaths->size() << "\n";
        else
            std::cout << "\n";
    }
}


void setupVisualization(const std::vector<FinalAllocation>& finalSequence)
{
    // Create two main windows for visualization.
    QMainWindow *window1 = new QMainWindow();
    QMainWindow *window2 = new QMainWindow();
    window1->setWindowTitle("Path Planner Visualization 1");
    window2->setWindowTitle("Path Planner Visualization 2");

    // Define custom colors.
    QColor customInitialColor(70, 130, 180);   // Steel Blue
    QColor customGoalColor(34, 139, 34);         // Forest Green
    QColor customPathColor(220, 20, 60);         // Crimson
    QColor customPathArrowColor(178, 34, 34);     // Firebrick

    // For illustration, we assume there are at least two allocations.
    auto vis_path0 = finalSequence[0].toSinglePathPtr();
    auto vis_path   = finalSequence[1].toSinglePathPtr();

    // (Optional) Print the first path.
    for (const auto &pt : *vis_path0)
        std::cout << pt.x << "," << pt.y << "," << pt.yaw << std::endl;

    // Create visualization widgets.
    VisualizationWidget *viz1 = new VisualizationWidget(nullptr,
                                                        customInitialColor,
                                                        customGoalColor,
                                                        customPathColor,
                                                        customPathArrowColor);
    VisualizationWidget *viz2 = new VisualizationWidget(nullptr,
                                                        customInitialColor,
                                                        customGoalColor,
                                                        customPathColor,
                                                        customPathArrowColor);

    // Define workspace dimensions.
    float workspace_width  = 4.0f;
    float workspace_height = 5.0f;
    viz1->setWorkspace(workspace_width, workspace_height);
    viz1->setInitialPose(vis_path0->at(0));
    viz1->setGoalPose(vis_path0->back());
    viz1->setPath(*vis_path0);

    viz2->setWorkspace(workspace_width, workspace_height);
    viz2->setInitialPose(vis_path->at(0));
    viz2->setGoalPose(vis_path->back());
    viz2->setPath(*vis_path);

    // Retrieve obstacles from the snapshots.
    auto obsSet1 = finalSequence[0].snapshot.env.get_obs();
    std::vector<ReloPush::State> obstacles1(obsSet1.begin(), obsSet1.end());
    auto obsSet2 = finalSequence[1].snapshot.env.get_obs();
    std::vector<ReloPush::State> obstacles2(obsSet2.begin(), obsSet2.end());
    viz1->setObstacles(obstacles1);
    viz2->setObstacles(obstacles2);

    // Set the visualization widgets into the windows.
    window1->setCentralWidget(viz1);
    window1->resize(workspace_width * 100, workspace_height * 100);
    window1->show();

    window2->setCentralWidget(viz2);
    window2->resize(workspace_width * 100, workspace_height * 100);
    window2->show();
}

// =====================================================================
// Main function
// =====================================================================

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // You can uncomment the following to use command-line arguments.
    // if (argc < 2) {
    //     std::cerr << "Usage: " << argv[0] << " <input_file>\n";
    //     return 1;
    // }
    // std::string filename = argv[1];
    std::string filename = "input4.txt";

    // Parse the input file.
    WorkspaceBoundary boundary;
    std::unordered_map<std::string, ObjectInfo> objects;
    std::unordered_map<std::string, GoalInfo> goals;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
    if (!parseInputFile(filename, boundary, objects, goals, objGoalPairs))
    {
        std::cerr << "Parse failed.\n";
        return 1;
    }

    // Containers for the final allocation sequence.
    std::vector<FinalAllocation> finalSequence;
    GoalMap delivered_objs;

    // Main planning loop: continue until all object-goal pairs have been allocated.
    while (!objGoalPairs.empty())
    {
        std::cout << "\n============================\n"
                  << "Remaining pairs: " << objGoalPairs.size() << "\n";

        // (a) Build the graph from scratch.
        Graph g;
        initGraph(g, objects, goals);

        // (b) Setup planning parameters and context.
        PlanningParameters params;
        params.boundary = boundary;
        PlanningContext planCtx(params, objects, delivered_objs);

        // (c) Build all edges.
        buildAllEdges(g, planCtx);

        // (d) Compute cost matrices for all remaining pairs.
        auto pairResults = computeMatrixPairs(g, objGoalPairs, planCtx);

        // (e) Run the planning/allocation cycle.
        LowestCostInfo best;
        std::vector<EdgePath> obsReloPathList;
        std::unordered_map<std::string, ReloPush::State> toUpdate;
        PlanningContext ctxSnapshot;
        if (!planAllocation(pairResults, best, obsReloPathList, toUpdate, planCtx, objects, goals, ctxSnapshot))
        {
            std::cerr << "No feasible allocation found. Exiting loop.\n";
            break;
        }

        // (f) Update object positions based on the planning results.
        updateObjectPositions(objects, toUpdate);

        // Mark the object as delivered.
        planCtx.delivered_list[best.objectName] = goals[best.goalName];
        delivered_objs = planCtx.delivered_list;

        // (g) Print and store the best allocation.
        printBestPair(best);
        FinalAllocation chosen = createFinalAllocation(best, objects, goals, pairResults, obsReloPathList, ctxSnapshot);
        finalSequence.push_back(chosen);

        // (h) Remove the chosen pair so it isn’t reselected.
        objGoalPairs.erase(best.objectName);
        objects.erase(best.objectName);
        goals.erase(best.goalName);
    }

    // Print the final sequence.
    printFinalSequence(finalSequence);

    // Set up visualization windows.
    setupVisualization(finalSequence);

    // Create and show the main control window.
    MainControlWindow controlWindow;
    controlWindow.show();

    return app.exec();
}
