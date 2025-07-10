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


    //float workspace_width = 4.0f;
    //float workspace_height = 5.0f;
    float workspace_width = 4.0f;
    float workspace_height = 5.0f;

    WorkspaceBoundary boundary(workspace_width,workspace_height);
    PlanningParameters params;   //?inspect
    params.boundary = boundary; // or fill as needed


    // Init obstacles
    ObjectMap objects = {
                     //{"a",ObjectInfo("a",1,1,0,0.075)}
    };


    // (c) Initialize PlanningContext
    PlanningContext planCtx(params, objects);
    // testing different rho
    std::unordered_set<ReloPush::State> obs_dummy;
    //planCtx.env_nonpush = Environment(workspace_width, workspace_height, obs_dummy, 3, Constants::LF_nonpush, true);


    // init robot pose
    ReloPush::State start(0,0,1);
    // init goal
    ReloPush::State goal(2,3,-0.6);

    // plan
    auto res = planHybridAstar(start,goal,planCtx,true);
    // path in StatePath
    ReloPush::StatePath plannedPath = res->getPath(true);
    for(size_t n=0; n<plannedPath.size(); n++)
    {
        //plannedPath[n] = res->states[n].first;
        plannedPath[n].print();
    }

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
    std::vector<ObjectInfo> obstacles;
    //obstacles.emplace_back(1.0f, 1.0f, 1.0f);
    //obstacles.emplace_back(1.0f, 10.4f, 1.0f);
    viz->setObstacles(obstacles);

    // Set the widget as central widget
    window.setCentralWidget(viz);
    window.resize(workspace_width*100, workspace_height*100);
    window.show();

    return app.exec();
}
