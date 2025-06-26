// main.cpp
// ---------------------------------------------------------------------------
// Includes & Qt Setup
// ---------------------------------------------------------------------------
#include <QApplication>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QTimer>
#include <QPen>
#include <QBrush>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// trajectory_elem and trajectory Classes
// ---------------------------------------------------------------------------
class trajectory_elem {
public:
    float x;
    float y;
    float yaw;
    float ref_vel; // Reference velocity
    float time;    // Time (in ms)
    bool is_pushing;

    trajectory_elem() : x(0), y(0), yaw(0), ref_vel(0), time(-1), is_pushing(false) {}

    trajectory_elem(float x_in, float y_in, float yaw_in, float ref_vel_in, float time_in, bool is_pushing_in)
        : x(x_in), y(y_in), yaw(yaw_in), ref_vel(ref_vel_in), time(time_in), is_pushing(is_pushing_in)
    {}

    void print() const {
        std::cout << "(x=" << x << ", y=" << y << ", yaw=" << yaw
                  << ", ref_vel=" << ref_vel << ", time=" << time << ")";
    }
};

class trajectory {
public:
    float time_zero = 0;
    std::string header_delim = "!";
    std::string elem_delim = ";";
    std::string var_delim = ",";
    std::string header = "t"; // header for trajectory

    std::shared_ptr<std::vector<trajectory_elem>> trajectory_points;
};

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
    void setTrajectory(const trajectory& traj) {
        scene->clear();
        auto& points = *(traj.trajectory_points);

        // Draw the complete trajectory as blue line segments.
        QPen linePen(QColor("#0000FF"));  // Blue color via hex code.
        linePen.setWidth(2);
        for (size_t i = 1; i < points.size(); i++) {
            const trajectory_elem& prev = points[i - 1];
            const trajectory_elem& curr = points[i];
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
    void animateFromIndex(size_t index, const std::vector<trajectory_elem>& points) {
        if (index >= points.size() - 1) return;
        // Compute delay as difference between next and current time (assumed in ms).
        float currentTime = points[index].time;
        float nextTime = points[index + 1].time;
        int delay = static_cast<int>(nextTime - currentTime);
        QTimer::singleShot(delay, this, [=, &points]() {
            updateArrowPosition(points[index + 1]);
            animateFromIndex(index + 1, points);
        });
    }

    // Update arrow position (world-to-screen conversion) and rotation.
    void updateArrowPosition(const trajectory_elem& elem) {
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

#include "test_trajectory_vis.moc"

// ---------------------------------------------------------------------------
// Main Function: Setting Up the Window and Visualizing the Trajectory Data
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Trajectory Visualization (Arrow Format)");
    window.resize(800, 600);

    TrajectoryView* view = new TrajectoryView();
    window.setCentralWidget(view);

    // Create a trajectory object and fill it with sample data.
    // Here we use similar data as before, with time stamps (in ms).
    trajectory myTraj;
    myTraj.trajectory_points = std::make_shared<std::vector<trajectory_elem>>();

    myTraj.trajectory_points->push_back(trajectory_elem(0, 0, 1, 1.2, 0, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.112092, 0.230238, 1.23544, 1.2, 100, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.196561, 0.472607, 1.23544, 1.2, 200, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.281031, 0.714976, 1.23544, 1.2, 300, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.3655, 0.957345, 1.23544, 1.2, 400, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.449969, 1.19971, 1.23544, 1.2, 500, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.534439, 1.44208, 1.23544, 1.2, 600, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.618908, 1.68445, 1.23544, 1.2, 700, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.703378, 1.92682, 1.23544, 1.2, 800, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.787847, 2.16919, 1.23544, 1.2, 900, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.872316, 2.41156, 1.23544, 1.2, 1000, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.956786, 2.65393, 1.23544, 1.2, 1100, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.04126, 2.8963, 1.23544, 1.2, 1200, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.15335, 3.12653, 1, 1.2, 1300, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.31605, 3.32427, 0.764555, 1.2, 1400, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.32358, 3.33142, 0.755034, 1.2, 1500, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.11777, 3.17905, 0.51959, 1.2, 1600, false));
    myTraj.trajectory_points->push_back(trajectory_elem(0.980477, 3.11279, 0.379634, 1.2, 1700, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.22782, 3.1791, 0.14419, 1.2, 1800, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.4838, 3.18587, 6.19193, 1.2, 1900, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.73431, 3.13275, 5.95649, 1.2, 2000, false));
    myTraj.trajectory_points->push_back(trajectory_elem(1.96551, 3.02265, 5.72104, 1.2, 2100, false));
    myTraj.trajectory_points->push_back(trajectory_elem(2, 3, 5.68318, 1.2, 2200, false));

    view->setTrajectory(myTraj);

    window.show();
    return app.exec();
}
