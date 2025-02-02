#ifndef VISUALIZERESULTS_H
#define VISUALIZERESULTS_H

#include <TaskAllocation.hpp>

#include <QApplication>
#include <QMainWindow>
#include <Visualization/VisualizationWidget.h>
#include <Visualization/QtMainControlWindow.h>

// ---------------------------------------------------------------------------
// Helper Function 6: Visualization setup and launch
// ---------------------------------------------------------------------------
void visualizeResults(std::vector<FinalAllocation> &finalSequence,
                      QApplication &app)
{
    // Create main windows
    QMainWindow window1, window2;
    window1.setWindowTitle("Path Planner Visualization 1");
    window2.setWindowTitle("Path Planner Visualization 2");

    // Custom colors
    QColor customInitialColor   = QColor(70, 130, 180);   // Steel Blue
    QColor customGoalColor      = QColor(34, 139, 34);    // Forest Green
    QColor customPathColor      = QColor(220, 20, 60);    // Crimson
    QColor customPathArrowColor = QColor(178, 34, 34);    // Firebrick

    // Just show the first two allocations for demo:
    // (Make sure finalSequence has at least 2 elements in your real code)
    auto vis_path0 = finalSequence[0].toSinglePathPtr();
    auto vis_path1 = finalSequence[1].toSinglePathPtr();

    // Create Visualization widgets
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

    // Define workspace size
    float workspace_width  = 4.0f;
    float workspace_height = 5.0f;

    // Setup viz1
    viz1->setWorkspace(workspace_width, workspace_height);
    viz1->setInitialPose(vis_path0->at(0));
    viz1->setGoalPose(vis_path0->back());
    viz1->setPath(*vis_path0);

    // Setup viz2
    viz2->setWorkspace(workspace_width, workspace_height);
    viz2->setInitialPose(vis_path1->at(0));
    viz2->setGoalPose(vis_path1->back());
    viz2->setPath(*vis_path1);

    // Retrieve obstacles from snapshots
    auto obstaclesSet1 = finalSequence[0].snapshot.env.get_obs();
    std::vector<ReloPush::State> obstacles1(obstaclesSet1.begin(), obstaclesSet1.end());
    auto obstaclesSet2 = finalSequence[1].snapshot.env.get_obs();
    std::vector<ReloPush::State> obstacles2(obstaclesSet2.begin(), obstaclesSet2.end());

    viz1->setObstacles(obstacles1);
    viz2->setObstacles(obstacles2);

    // Connect them to windows
    window1.setCentralWidget(viz1);
    window2.setCentralWidget(viz2);
    window1.resize(workspace_width*100, workspace_height*100);
    window2.resize(workspace_width*100, workspace_height*100);

    window1.show();
    window2.show();

    // Show Main Control Window
    MainControlWindow controlWindow;
    controlWindow.show();

    app.exec();
}


#endif // VISUALIZERESULTS_H
