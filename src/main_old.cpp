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




int main(int argc, char *argv[])
{
    QApplication app(argc, argv);



    //if (argc < 2)
   // {
   //     std::cerr << "Usage: " << argv[0] << " <input_file>\n";
    //    return 1;
    //}

    //std::string filename = argv[1];
    std::string filename = "input4.txt";

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
    GoalMap delivered_objs;

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
        PlanningContext planCtx(params, objects, delivered_objs);

        // (d) Build all edges
        buildAllEdges(g, planCtx);

        // todo: make replan loop here
        // (e) Compute cost matrices for all *remaining* pairs
        auto pairResults = computeMatrixPairs(g, objGoalPairs, planCtx); // list of matrices


        LowestCostInfo best = LowestCostInfo();
        std::vector<EdgePath> ObsReloPathList;

        std::unordered_map<std::string,ReloPush::State> ToUpdate;

        PlanningContext ctxSnapshot;

        while(1)
        {
            // save context snapshot
            ctxSnapshot = PlanningContext(planCtx);

            auto best_pick = findAbsoluteLowestCost(pairResults); // failed to find if row=-1
            if(best_pick.row == -1 || best_pick.col == -1)
                break; // failed to find any
            // (f) Find the absolute lowest cost among them

            // Check if we found any feasible solution
            if (best_pick.cost == std::numeric_limits<double>::infinity())
            {
                std::cout << "No feasible pair found (or no pairs left)!\n";
                break;
            }

            // get matrixResult for the object
            auto best_pair = pairResults[best_pick.objectName];

            // find approach for ObsRelo to next approach (next ObsRelo or Final push)
            auto temp = best_pair.matrixResult->getBestPathMatEntry();
            for(size_t obs=1; obs<temp.obsReloList.size(); obs++) // if multiple Obs Relo
            {
                // object
                auto pivotObj = temp.vertexChain[obs];
                auto prev_pair = temp.obsReloList[obs-1];
                auto next_pair = temp.obsReloList[obs];

                auto fromState = prev_pair.second;
                auto toState = next_pair.first;

                auto fromState_prepush = find_pre_push(fromState, planCtx.parameters.PrePush_dist);
                auto toState_prepush = find_pre_push(toState, planCtx.parameters.PrePush_dist);

                // update obs
                planCtx.env.remove_obs(prev_pair.first);
                planCtx.env.add_obs(prev_pair.second);
                auto res = planHybridAstar(fromState_prepush, toState_prepush, planCtx, true);

                if(!res->success)
                {
                    // approach failed. replan
                    // remove first pick
                    pairResults[best_pick.objectName].matrixResult->sortedEntries.erase(pairResults[best_pick.objectName].matrixResult->sortedEntries.begin());
                    // mark inf on  mat
                    pairResults[best_pick.objectName].matrixResult->costMat(best_pick.row, best_pick.col) = std::numeric_limits<double>::infinity();

                    // put obs back
                    planCtx.env.add_obs(prev_pair.first);
                    planCtx.env.remove_obs(prev_pair.second);
                    continue;
                }

                // successful approach plan
                ReloPush::StatePath pre_pair_path = {find_pre_push(prev_pair.first,planCtx.parameters.PrePush_dist),fromState_prepush};
                ObsReloPathList.push_back(EdgePath(true,  std::make_shared<ReloPush::StatePath>(pre_pair_path))); // two-points path
                ObsReloPathList.push_back(EdgePath(false,res->getPathPtr(true)));
                // queue on object update
                ToUpdate[pivotObj.name] = fromState;
            }
            best = best_pick; // todo: is it better to be somewhere else?

            // Last ObsRelo to Final push
            if(temp.obsReloList.size()>0)
            {
                auto lastObsObj = temp.vertexChain[temp.vertexChain.size()-2]; //second last
                auto last_pair = temp.obsReloList.back();
                auto fromState_prepush = find_pre_push(last_pair.second, planCtx.parameters.PrePush_dist);
                auto best_obj = objects[best.objectName];
                auto final_approach = ReloPush::State(best_obj.x,best_obj.y,best_obj.getOrientation(best.row));
                auto toState_prepush = find_pre_push(final_approach, planCtx.parameters.PrePush_dist);
                // Try planning final approach
                // update obs
                planCtx.env.remove_obs(last_pair.first);
                planCtx.env.add_obs(last_pair.second);

                auto res = planHybridAstar(fromState_prepush, toState_prepush,planCtx, true);
                if(!res->success)
                {
                    // approach failed. replan
                    // remove first pick
                    pairResults[best_pick.objectName].matrixResult->sortedEntries.erase(pairResults[best_pick.objectName].matrixResult->sortedEntries.begin());
                    // mark inf on  mat
                    pairResults[best_pick.objectName].matrixResult->costMat(best_pick.row, best_pick.col) = std::numeric_limits<double>::infinity();

                    // put obs back
                    planCtx.env.remove_obs(last_pair.first);
                    planCtx.env.add_obs(last_pair.second);
                    continue;
                }
                ReloPush::StatePath last_pair_path = {find_pre_push(last_pair.first,planCtx.parameters.PrePush_dist),fromState_prepush};
                ObsReloPathList.push_back(EdgePath(true,  std::make_shared<ReloPush::StatePath>(last_pair_path))); // two-points path
                ObsReloPathList.push_back(EdgePath(false,res->getPathPtr(true)));
                // queue on object update
                ToUpdate[lastObsObj.name] = last_pair.second;

            }

            // update ObsRelo positions
            for (const auto& pair : ToUpdate) {
                std::cout << "Key: " << pair.first << ", Value: " << pair.second << std::endl;
                objects[pair.first].x = pair.second.x;
                objects[pair.first].y = pair.second.y;
                // todo: also consider orientation if needed
            }

            // make the object delivered
            planCtx.delivered_list[best.objectName] = goals[best.goalName];
            delivered_objs = planCtx.delivered_list; // planCtx resets every iteration
            break;


            // todo: handle replan here
        }

        if(best.row==-1 || best.cost == std::numeric_limits<double>::infinity())
        {
            // handle failure
        }

        //}
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
        chosen.startPose = ReloPush::State(chosen.object.x,chosen.object.y,chosen.object.getOrientation(chosen.row));
        chosen.goalPose = ReloPush::State(chosen.goal.x, chosen.goal.y, chosen.goal.getOrientation(chosen.col));
        chosen.paths = pairResults[best.objectName].getBestPath().edgesInfo;
        chosen.obsReloPaths = std::make_shared<std::vector<EdgePath>>(ObsReloPathList);
        chosen.snapshot = ctxSnapshot;


        // Look up the actual edge in the graph & read pre-relocation info
        auto objVerts  = getObjectVertices(g, best.objectName);
        auto goalVerts = getGoalVertices(g, best.goalName);

        /*
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
*/

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
                  << ", goal yaw = " << fa.goalPose.yaw;

        if(fa.obsReloPaths->size()>0)
        {
            std::cout << "ObsRelo: " << fa.obsReloPaths->size() << std::endl;
        }
        else
            std::cout << std::endl;

        /*
        if (fa.usedPreRelocation)
        {
            std::cout << "  **Used pre-relocation**: Moved start from ("
                      << fa.object.x << ", " << fa.object.y << ") to ("
                      << fa.xRelocated << ", " << fa.yRelocated << "), cost="
                      << fa.preReloCost << ", orientation index="
                      << fa.relocatingIndex << "\n";
        }
        std::cout << std::endl;
        */
    }


    #pragma region visualization

    // Create main window
    QMainWindow window1, window2;
    window1.setWindowTitle("Path Planner Visualization 1");
    window2.setWindowTitle("Path Planner Visualization 2");

    // Define custom colors (optional)
    QColor customInitialColor = QColor(70, 130, 180); // Steel Blue
    QColor customGoalColor = QColor(34, 139, 34);     // Forest Green
    QColor customPathColor = QColor(220, 20, 60);     // Crimson
    QColor customPathArrowColor = QColor(178, 34, 34); // Firebrick

    // test path vis
    auto vis_path0 = finalSequence[0].toSinglePathPtr();
    auto vis_path = finalSequence[1].toSinglePathPtr();


    //print path
    for(auto& it : *vis_path0)
    {
        std::cout << it.x << "," << it.y << "," << it.yaw << std::endl;
    }

    // Create visualization widget with custom colors
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

    // Alternatively, use default colors by omitting color parameters
    // VisualizationWidget *viz = new VisualizationWidget();

    // Define workspace size
    float workspace_width = 4.0f;
    float workspace_height = 5.0f;
    viz1->setWorkspace(workspace_width, workspace_height);
    viz1->setInitialPose(vis_path0->at(0));
    viz1->setGoalPose(vis_path0->back());
    viz1->setPath(*vis_path0);

    viz2->setWorkspace(workspace_width, workspace_height);
    viz2->setInitialPose(vis_path->at(0));
    viz2->setGoalPose(vis_path->back());
    viz2->setPath(*vis_path);


    // Create obstacles
    auto uset = finalSequence[0].snapshot.env.get_obs();
    std::vector<ReloPush::State> obstacles1(uset.begin(), uset.end());
    auto uset2 = finalSequence[1].snapshot.env.get_obs();
    std::vector<ReloPush::State> obstacles2(uset2.begin(), uset2.end());

    //obstacles.emplace_back(3.0f, 3.0f, 0.0f);
    //obstacles.emplace_back(1.0f, 0.4f, 0.4f);
    viz1->setObstacles(obstacles1);
    viz2->setObstacles(obstacles2);

    // Set the widget as central widget
    window1.setCentralWidget(viz1);
    window1.resize(workspace_width*100, workspace_height*100);
    window1.show();

    window2.setCentralWidget(viz2);
    window2.resize(workspace_width*100, workspace_height*100);
    window2.show();

    ReloPush::StatePath obs;

    std::vector<std::pair<std::vector<ReloPush::State>, std::vector<ReloPush::State>>> initialPaths;
    initialPaths.emplace_back(std::make_pair(*vis_path0,obs));
    initialPaths.emplace_back(std::make_pair(*vis_path,obs));

    // Create and show the Main Control Window
    MainControlWindow controlWindow;
    controlWindow.show();


    return app.exec();
    #pragma engregion visualization


    return 0;
}
