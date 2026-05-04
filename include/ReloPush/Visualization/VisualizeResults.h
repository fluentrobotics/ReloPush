#ifndef VISUALIZERESULTS_H
#define VISUALIZERESULTS_H

#include <ReloPush/TaskAllocation.hpp>

#include <QApplication>
#include <QMainWindow>
#include <ReloPush/Visualization/VisualizationWidget.h>
#include <ReloPush/Visualization/QtMainControlWindow.h>

// ---------------------------------------------------------------------------
// Helper Function 6: Visualization setup and launch
// ---------------------------------------------------------------------------
void visualizeResults(std::vector<FinalAllocation> &finalSequence,
                      QApplication &app)
{
    // If no final allocations, nothing to show.
    if (finalSequence.empty())
    {
        std::cerr << "No allocations to visualize!\n";
        return;
    }

    // get goals
    GoalMap goals;
    for(auto& it : finalSequence)
    {
        goals[it.goal.name] = it.goal;
    }

    // Custom colors (example)
    QColor customInitialColor   = QColor(70, 130, 180,127);   // Steel Blue
    QColor customGoalColor      = QColor(34, 139, 34,127);    // Forest Green
    QColor customTransitPathColor  = QColor("#DC143C");   // Crimson
    QColor customTransferPathColor = QColor("#2ECC71");   // Green
    QColor customPathArrowColor = QColor("#B22222");   // Firebrick

    // Define workspace size (example)
    float workspace_width  = 4.0f;
    float workspace_height = 5.2f;

    // We store windows in a vector so they won't go out of scope
    // before the Qt event loop (`app.exec()`) finishes.
    std::vector<std::unique_ptr<QMainWindow>> windows;
    windows.reserve(finalSequence.size());

    // Create one window per FinalAllocation
    for (size_t i = 0; i < finalSequence.size(); ++i)
    {
        // 1) Create and configure a new QMainWindow
        auto window = std::make_unique<QMainWindow>();
        window->setWindowTitle(
            QString("Path Planner Visualization %1").arg(i + 1)
            );

        // 2) Create a new VisualizationWidget for this final allocation
        auto viz = new VisualizationWidget(nullptr,
                           customInitialColor,
                           customGoalColor,
                           customTransitPathColor,
                           customPathArrowColor);
        viz->setSegmentTypeColors(customTransitPathColor, customTransferPathColor);
        viz->setRobotFootprintDimensions(finalSequence[i].snapshot.parameters.car_width,
                         finalSequence[i].snapshot.parameters.LF_nonpush,
                         finalSequence[i].snapshot.parameters.LF_push,
                         finalSequence[i].snapshot.parameters.LB);

        // 3) Define the path to visualize
        //    (Here we assume toSinglePathPtr() gives you the entire path as a vector.)
        auto pathPtr_tuple = finalSequence[i].toSinglePathPtrWithTypes();
        auto &pathPtr = std::get<0>(pathPtr_tuple);
        auto &path_segment_lengths = std::get<1>(pathPtr_tuple);
        auto &path_segment_is_transfer = std::get<2>(pathPtr_tuple);
        if (pathPtr && !pathPtr->empty())
        {
            viz->setWorkspace(workspace_width, workspace_height);

            // The first and last states in the path define the initial/goal poses
            viz->setInitialPose(pathPtr->front());
            viz->setGoalPose(pathPtr->back());
            viz->setPath(*pathPtr, path_segment_lengths, path_segment_is_transfer);
        }
        else
        {
            // No path? Optionally handle that scenario
            std::cerr << "Warning: FinalAllocation[" << i << "] has empty path.\n";
        }

        // 4) Retrieve obstacles from snapshot
        ObjectMap obstaclesSet(finalSequence[i].snapshot.env_push.get_obs());
//        std::vector<ObjectInfo> obstacles(
//            obstaclesSet.begin(), obstaclesSet.end()
//            );
        std::vector<ObjectInfo> obstacles;
        for (auto& kv : obstaclesSet) {
            obstacles.push_back(std::move(kv.second));
        }
        viz->setObstacles(obstacles);

        // goals
        viz->setGoals(goals);

        // prerelocation if any
        std::vector<ReloPush::State> prerelocs;
        for(auto& it : finalSequence[i].paths)
        {
            if(it.preRelo.used)
            {
                prerelocs.push_back(ReloPush::State(it.preRelo.xRelocated_object,it.preRelo.yRelocated_object,it.preRelo.yawRelocated_object));
            }
        }
        viz->setPreRelocations(prerelocs);


        // 5) Attach the VisualizationWidget to the QMainWindow
        window->setCentralWidget(viz);

        // 6) Resize and show
        window->resize(workspace_width * 100, workspace_height * 100);
        window->show();

        // 7) Keep the window in our vector so it stays alive
        windows.push_back(std::move(window));
    }

    // Optionally show any additional UI, like your MainControlWindow
    MainControlWindow controlWindow;
    controlWindow.show();

    // 8) Start the Qt event loop. This call blocks until the user exits.
    app.exec();
}


#endif // VISUALIZERESULTS_H
