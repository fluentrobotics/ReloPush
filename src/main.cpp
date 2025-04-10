#include <GraphBuilder.hpp>
#include "InputParser.hpp"
#include <batchInstanceParcer.hpp>
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>
#include <Visualization/VisualizeResults.h>
#include <chrono>
#include <trajectory.hpp>
#ifdef __APPLE__
// Include the glog header when compiling on MacOS.
    #include <glog/logging.h>
#else
// Otherwise, include the Abseil logging header.
    #include "absl/log/initialize.h"
#endif

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QTimer>
#include <QPen>
#include <QBrush>

// ---------------------------------------------------------------------------
// TrajectoryView: Custom QGraphicsView for Visualizing the Trajectory
// ---------------------------------------------------------------------------
class TrajectoryView : public QGraphicsView {
    Q_OBJECT
public:
    TrajectoryView(QWidget* parent = nullptr)
        : QGraphicsView(parent), arrowItem(nullptr)
    {
        scene = new QGraphicsScene(this);
        setScene(scene);
        setRenderHint(QPainter::Antialiasing);
        scaleFactor = 100.0; // meters to pixels conversion factor
    }

    // Set the trajectory (with time stamps) and start the animation.
    void setTrajectory(const ReloPush::trajectory& traj) {
        scene->clear();
        auto& points = *(traj.trajectory_points);

        // Draw the complete trajectory as blue line segments.
        QPen linePen(QColor("#0000FF"));  // Blue color via hex code.
        linePen.setWidth(2);
        for (size_t i = 1; i < points.size(); i++) {
            const ReloPush::trajectory_elem& prev = points[i - 1];
            const ReloPush::trajectory_elem& curr = points[i];
            scene->addLine(prev.x * scaleFactor, -prev.y * scaleFactor,
                           curr.x * scaleFactor, -curr.y * scaleFactor,
                           linePen);
        }

        // Create the robot item as an arrow if it doesn't exist.
        if (!arrowItem) {
            QPolygonF arrow;
            // Define the arrow polygon so that a zero yaw means arrow points to the right.
            // Tip: (7.5, 0) and Base: (-7.5, -5) and (-7.5, 5)
            arrow << QPointF(7.5, 0) << QPointF(-7.5, -5) << QPointF(-7.5, 5);
            arrowItem = scene->addPolygon(arrow, QPen(QColor("#FF5733")), QBrush(QColor("#FF5733")));
            // Center the rotation about the arrow's centroid.
            arrowItem->setTransformOriginPoint(arrowItem->boundingRect().center());
        }

        // Set initial arrow position and start the animation.
        if (!points.empty()) {
            updateArrowPosition(points[0]);
            animateFromIndex(0, points);
        }
    }

private:
    // Recursively animate the arrow from one trajectory element to the next,
    // using the time differences as delays.
    void animateFromIndex(size_t index, const std::vector<ReloPush::trajectory_elem>& points) {
        if (index >= points.size() - 1) return;
        // Compute delay as difference between next and current time (assumed in ms).
        float currentTime = points[index].time;
        float nextTime = points[index + 1].time;
        int delay = static_cast<int>((nextTime - currentTime)*1000);
        QTimer::singleShot(delay, this, [=, &points]() {
            updateArrowPosition(points[index + 1]);
            animateFromIndex(index + 1, points);
        });
    }

    // Update arrow position (world-to-screen conversion) and rotation.
    void updateArrowPosition(const ReloPush::trajectory_elem& elem) {
        qreal x = elem.x * scaleFactor;
        qreal y = -elem.y * scaleFactor; // Invert y for screen coordinates.
        arrowItem->setPos(x, y);
        qreal angleDegrees = -elem.yaw * 180.0 / M_PI; // Conversion: radians to degrees.
        arrowItem->setRotation(angleDegrees);
    }

    QGraphicsScene* scene;
    QGraphicsPolygonItem* arrowItem;
    double scaleFactor;
};

#include "main.moc"

// ---------------------------------------------------------------------------
// Main Function
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    #ifdef __APPLE__
        // For macOS, initialize Google Logging with the program name.
        google::InitGoogleLogging(argv[0]);
    #else
        // For non-macOS systems, initialize Abseil Logging.
        absl::InitializeLog();
    #endif
    QApplication app(argc, argv);

    //std::string filename = "input_opt2obj.txt";
    std::string filename = "alpha_to_omega.txt";
    //std::string filename = "omega_to_alpha.txt";
    int instance_ind = 0;
    bool use_opt = false;
    bool vis = true;

    // Data to parse
    WorkspaceBoundary boundary(4,5.2); // todo: parse from file
    std::unordered_map<std::string, ObjectInfo> objects;
    std::unordered_map<std::string, GoalInfo>   goals;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
    std::vector<ReloPush::State> robots;


    if(argc > 3) // parse from arg
    {
        handle_args(argc, argv, filename, instance_ind, use_opt);
        vis = false; // disable for evaluations
    }

    Color::println("\n=== " + filename + " ind: " + std::to_string(instance_ind) + " ===",Color::GREEN);
    Color::println("Use Optimized PreRelocation? " + std::to_string(use_opt),Color::YELLOW);

    parse_instance_from_file(filename, instance_ind, objects, goals, robots, objGoalPairs);

    /*
    // 1) Parse and initialize
    if (!parseAndInitialize(filename, boundary, objects, goals, objGoalPairs))
    {
        return 1;
    }
    */

    auto start = std::chrono::high_resolution_clock::now();

    // 2) Perform the main planning/allocation loop
    std::vector<FinalAllocation> finalSequence;
    //bool ok = performAllocations(boundary, objects, goals, objGoalPairs, finalSequence, use_opt);
    GoalMap delivered_objs;
    bool ok = performAllocationsDFS(boundary, objects, goals, objGoalPairs, delivered_objs, robots[0] ,finalSequence, use_opt);

    auto end = std::chrono::high_resolution_clock::now();

    if(!ok)
    {
        // plan failed
        Color::println("Failed to find a solution",Color::YELLOW,Color::BG_RED);
        return -1;
    }

    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;


    // 3) Print the final sequence
    printFinalSequence(finalSequence);

    // 4) Visualization
    if (!finalSequence.empty() && vis)
    {
       //visualizeResults(finalSequence, app);
    }

    //writeFinalSequenceSummary(filename, instance_ind,
    //                          static_cast<double>(duration.count()), finalSequence, use_opt);

    // generate resulting trajectory

    auto finalTrajectory = FA2Trajectory(finalSequence);
    //QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Trajectory Visualization (Arrow Format)");
    window.resize(800, 600);

    TrajectoryView* view = new TrajectoryView();
    window.setCentralWidget(view);

    // Create a trajectory object and fill it with sample data.
    // Here we use similar data as before, with time stamps (in ms).

    view->setTrajectory(finalTrajectory);

    window.show();
    return app.exec();


}
