#include <ReloPush/hybrid_astar.hpp>
#include <ReloPush/PlanHybridAstar.hpp>
//#include "temp_hastr.hpp"

#include <QApplication>
#include <QMainWindow>
#include <ReloPush/Visualization/VisualizationWidget.h>



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
    float workspace_height = 5.2f;

    WorkspaceBoundary boundary(workspace_width,workspace_height);
    PlanningParameters params;   //?inspect
    params.boundary = boundary; // or fill as needed


    // Init obstacles
    double rad = 0.075;
    ObjectMap objects;
//    objects["b9"]  = ObjectInfo{"b9",  1.9649452797179674, 3.3151443159946345, -0.08311932780525295,rad};
//    objects["d2"]  = ObjectInfo{"d2",  3.6065165996551514, 3.4514808654785156,  0.7358155250549316,rad};
//    objects["b3"]  = ObjectInfo{"b3",  1.6911420683512188, 2.6256094907822454, -0.0920688504985566,rad};
//    objects["d8"]  = ObjectInfo{"d8",  2.1348116397857666, 0.29120028018951416, -0.009176576510071754,rad};
//    objects["d5"]  = ObjectInfo{"d5",  0.6259856820106506, 4.80679988861084,   -0.09028522670269012,rad};
//    objects["b7"]  = ObjectInfo{"b7",  0.26406906043677236, 2.8881048785158967, 0.8094720847761377,rad};
//    objects["d10"] = ObjectInfo{"d10", 3.6437253952026367, 1.717552661895752,   0.7029879689216614,rad};
//    objects["d4"]  = ObjectInfo{"d4",  3.5595109462738037, 0.2788912355899811,  0.7621512413024902,rad};
//    objects["d6"]  = ObjectInfo{"d6",  3.600428342819214,  4.779384136199951,  -0.04667137563228607,rad};
//    objects["d1"]  = ObjectInfo{"d1",  0.5892226696014404, 3.4944525228271484,  0.7799423336982727,rad};



    // (c) Initialize PlanningContext
    PlanningContext planCtx(params, objects);
    // testing different rho
    std::unordered_set<ReloPush::State> obs_dummy;
    //planCtx.env_nonpush = Environment(workspace_width, workspace_height, obs_dummy, 3, Constants::LF_nonpush, true);

    // init robot pose
    //ReloPush::State start(3.57752,4.288992,1.5241249511626105);
    // init goal
    //ReloPush::State goal(2.454177,3.27438579,3.058473);


    // init robot pose
    ReloPush::State start(1,1,0.7854);
    // init goal
    ReloPush::State goal(1.7071,1.7071,0.7854);



    //ReloPush::State goal(2.2,4.288992,3.141592);

    // plan
    auto res = planHybridAstar(start,goal,planCtx,true);

    ReloPush::StatePath plannedPath;
    if(res->validity == PlanValidity::success)
    {
        // path in StatePath
        plannedPath = res->getPath();
        for(size_t n=0; n<plannedPath.size(); n++)
        {
            //plannedPath[n] = res->states[n].first;
            plannedPath[n].print();
        }
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
