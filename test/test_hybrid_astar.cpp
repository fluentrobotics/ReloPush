#include <hybrid_astar.hpp>
#include <PlanHybridAstar.hpp>

#include <QApplication>
#include <QMainWindow>
#include <Visualization/VisualizationWidget.h>



int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Create main window
    QMainWindow window;
    window.setWindowTitle("Path Planner Visualization");

    // Define custom colors (optional)
    QColor customInitialColor = QColor(70, 130, 180); // Steel Blue
    QColor customGoalColor = QColor(34, 139, 34);     // Forest Green
    QColor customPathColor = QColor(220, 20, 60);     // Crimson
    QColor customPathArrowColor = QColor(178, 34, 34); // Firebrick


    float workspace_width = 4.0f;
    float workspace_height = 5.0f;
    WorkspaceBoundary boundary(workspace_width,workspace_height);
    PlanningParameters params;   //?inspect
    params.boundary = boundary; // or fill as needed


    // Init obstacles
    ObjectMap objects = {
                     {"a",ObjectInfo("a",1,1,0,0.075)}
    };


    // (c) Initialize PlanningContext
    PlanningContext planCtx(params, objects);


    // init robot pose
    ReloPush::State start(1,1,1.5708);
    // init goal
    ReloPush::State goal(1.5,1.5,3.84159);

    // plan
    auto res = planHybridAstar(start,goal,planCtx,true);
    // path in StatePath
    ReloPush::StatePath plannedPath = res->getPath(true);
    //for(size_t n=0; n<plannedPath.size(); n++)
    //{
    //    plannedPath[n] = res->states[n].first;
    //}

    // Create visualization widget with custom colors
    VisualizationWidget *viz = new VisualizationWidget(nullptr,
                                                       customInitialColor,
                                                       customGoalColor,
                                                       customPathColor,
                                                       customPathArrowColor);

    // Alternatively, use default colors by omitting color parameters
    // VisualizationWidget *viz = new VisualizationWidget();

    // Define workspace size

    viz->setWorkspace(workspace_width, workspace_height);

    viz->setInitialPose(start);
    viz->setGoalPose(goal);

    viz->setPath(plannedPath);

    // Create obstacles
    std::vector<ReloPush::State> obstacles;
    obstacles.emplace_back(1.0f, 1.0f, 0.0f);
    obstacles.emplace_back(1.0f, 0.4f, 0.0f);
    viz->setObstacles(obstacles);

    // Set the widget as central widget
    window.setCentralWidget(viz);
    window.resize(workspace_width*100, workspace_height*100);
    window.show();

    return app.exec();
}
