

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
#include <vector>

// ---------------------------------------------------------------------------
// ReloPush Namespace, State Structure, and Timing Functions
// ---------------------------------------------------------------------------
namespace ReloPush {
    struct State {
        double x;
        double y;
        double yaw;      // Orientation in radians.
        uint64_t time;   // Time in milliseconds.
        bool is_push;
        State(double _x, double _y, double _yaw, bool _is_push)
            : x(_x), y(_y), yaw(_yaw), time(0), is_push(_is_push) {}
    };
    typedef std::vector<State> StatePath;
}

// Normalize angle to [-pi, pi)
static double normalizeAngle(double theta) {
    while (theta > M_PI) theta -= 2.0 * M_PI;
    while (theta <= -M_PI) theta += 2.0 * M_PI;
    return theta;
}

// Compute difference between angles, normalized
static double angleDiff(double a, double b) {
    return normalizeAngle(a - b);
}

// Check if the segment from prev to curr is forward
bool isForwardSegment(const ReloPush::State &prev, const ReloPush::State &curr) {
    double dx = curr.x - prev.x;
    double dy = curr.y - prev.y;
    double seg_angle = std::atan2(dy, dx);
    double heading_diff = std::fabs(angleDiff(seg_angle, prev.yaw));
    const double THRESHOLD = M_PI / 2.0; // 90 degrees
    return (heading_diff <= THRESHOLD);
}

/**
 * @brief Generates a timed trajectory from an ordered list of States.
 *
 * Based on velocities (v_forward, v_backward, v_transition), it assigns time stamps.
 */
ReloPush::StatePath generateTimedTrajectory(const ReloPush::StatePath &path,
                                            double v_forward,
                                            double v_backward,
                                            double v_transition)
{
    ReloPush::StatePath timed_path;
    if (path.empty()) return timed_path;

    uint64_t current_time = 0;
    // Start with the first state.
    ReloPush::State first = path.front();
    first.time = current_time;
    timed_path.push_back(first);

    bool prev_forward = true;  // Initial assumption

    for (size_t i = 1; i < path.size(); ++i) {
        const ReloPush::State &prev = timed_path.back();
        ReloPush::State curr = path[i];

        // Compute distance between states.
        double dx = curr.x - prev.x;
        double dy = curr.y - prev.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Determine direction.
        bool curr_forward = isForwardSegment(prev, curr);
        if (i == 1) prev_forward = curr_forward;
        bool direction_changed = (curr_forward != prev_forward);

        // Choose appropriate velocity.
        double velocity = direction_changed ? v_transition :
                             (curr_forward ? v_forward : v_backward);

        // Compute time difference (ms)
        double delta_time_sec = (velocity > 1e-6) ? (distance / velocity) : 0.0;
        uint64_t delta_ms = static_cast<uint64_t>(delta_time_sec * 1000.0);
        current_time += delta_ms;
        curr.time = current_time;
        timed_path.push_back(curr);
        prev_forward = curr_forward;
    }
    return timed_path;
}

// ---------------------------------------------------------------------------
// TrajectoryView: Custom QGraphicsView for Visualization
// ---------------------------------------------------------------------------
class TrajectoryView : public QGraphicsView {
    Q_OBJECT
public:
    TrajectoryView(QWidget *parent = nullptr)
        : QGraphicsView(parent), robotItem(nullptr)
    {
        scene = new QGraphicsScene(this);
        setScene(scene);
        setRenderHint(QPainter::Antialiasing);
        scaleFactor = 100.0; // Conversion factor: meters to pixels.
    }

    /**
     * @brief Sets the trajectory (with time stamps) and starts the animation.
     */
    void setTrajectory(const ReloPush::StatePath &path) {
        trajectory = path;
        scene->clear();

        // Draw the entire trajectory as blue line segments.
        QPen linePen(QColor("#E8AE68"));
        linePen.setWidth(2);
        for (size_t i = 1; i < trajectory.size(); i++) {
            const auto &prev = trajectory[i - 1];
            const auto &curr = trajectory[i];
            scene->addLine(prev.x * scaleFactor, -prev.y * scaleFactor,
                           curr.x * scaleFactor, -curr.y * scaleFactor,
                           linePen);
        }

        // Create the robot item as an arrow if it doesn't exist.
        if (!robotItem) {
            // Define an arrow polygon that points right when rotation is 0.
            // The arrow is defined so that its centroid is at (0,0):
            // Tip: (7.5, 0), Base: (-7.5, -5) and (-7.5, 5)
            QPolygonF arrow;
            arrow << QPointF(7.5, 0) << QPointF(-7.5, -5) << QPointF(-7.5, 5);
            robotItem = scene->addPolygon(arrow, QPen(QColor("#00AFB9")), QBrush(QColor("#00AFB9")));
            // Center the rotation about the arrow's centroid.
            robotItem->setTransformOriginPoint(robotItem->boundingRect().center());
        }

        // Set initial robot position and start animation.
        if (!trajectory.empty()) {
            updateRobotPosition(trajectory[0]);
            animateFromIndex(0);
        }
    }

private:
    /**
     * @brief Recursively animates the robot from one state to the next using the time stamps.
     */
    void animateFromIndex(size_t index) {
        if (index >= trajectory.size() - 1) return;
        uint64_t currentTime = trajectory[index].time;
        uint64_t nextTime = trajectory[index + 1].time;
        int delay = static_cast<int>(nextTime - currentTime);
        QTimer::singleShot(delay, this, [=]() {
            updateRobotPosition(trajectory[index + 1]);
            animateFromIndex(index + 1);
        });
    }

    // Update robot item's position and rotation based on the state.
    void updateRobotPosition(const ReloPush::State &state) {
        qreal x = state.x * scaleFactor;
        qreal y = -state.y * scaleFactor; // Invert y for screen coordinates.
        robotItem->setPos(x, y);
        // Rotate to reflect yaw (0 yaw will point to the right).
        qreal angleDegrees = -state.yaw * 180.0 / M_PI;
        robotItem->setRotation(angleDegrees);
    }

    QGraphicsScene *scene;
    QGraphicsPolygonItem *robotItem;
    ReloPush::StatePath trajectory;
    double scaleFactor;
};

#include "test_trajectory_gen.moc"

// ---------------------------------------------------------------------------
// Main Function: Setting Up the Window and Trajectory
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Timed Trajectory Visualization (Arrow)");

    // Create the custom view and set it as the central widget.
    TrajectoryView *view = new TrajectoryView();
    window.setCentralWidget(view);
    window.resize(800, 600);

    // -----------------------------------------------------------------------
    // Create the Base Path using your provided data [x, y, yaw]
    // -----------------------------------------------------------------------
    ReloPush::StatePath basePath;
    basePath.push_back(ReloPush::State(0, 0, 1, false));
    basePath.push_back(ReloPush::State(0.112092, 0.230238, 1.23544, false));
    basePath.push_back(ReloPush::State(0.196561, 0.472607, 1.23544, false));
    basePath.push_back(ReloPush::State(0.281031, 0.714976, 1.23544, false));
    basePath.push_back(ReloPush::State(0.3655, 0.957345, 1.23544, false));
    basePath.push_back(ReloPush::State(0.449969, 1.19971, 1.23544, false));
    basePath.push_back(ReloPush::State(0.534439, 1.44208, 1.23544, false));
    basePath.push_back(ReloPush::State(0.618908, 1.68445, 1.23544, false));
    basePath.push_back(ReloPush::State(0.703378, 1.92682, 1.23544, false));
    basePath.push_back(ReloPush::State(0.787847, 2.16919, 1.23544, false));
    basePath.push_back(ReloPush::State(0.872316, 2.41156, 1.23544, false));
    basePath.push_back(ReloPush::State(0.956786, 2.65393, 1.23544, false));
    basePath.push_back(ReloPush::State(1.04126, 2.8963, 1.23544, false));
    basePath.push_back(ReloPush::State(1.15335, 3.12653, 1, false));
    basePath.push_back(ReloPush::State(1.31605, 3.32427, 0.764555, false));
    basePath.push_back(ReloPush::State(1.32358, 3.33142, 0.755034, false));
    basePath.push_back(ReloPush::State(1.11777, 3.17905, 0.51959, false));
    basePath.push_back(ReloPush::State(0.980477, 3.11279, 0.379634, false));
    basePath.push_back(ReloPush::State(1.22782, 3.1791, 0.14419, false));
    basePath.push_back(ReloPush::State(1.4838, 3.18587, 6.19193, false));
    basePath.push_back(ReloPush::State(1.73431, 3.13275, 5.95649, false));
    basePath.push_back(ReloPush::State(1.96551, 3.02265, 5.72104, false));
    basePath.push_back(ReloPush::State(2, 3, 5.68318, false));

    // -----------------------------------------------------------------------
    // Generate Timed Trajectory using Your Existing Logic
    // -----------------------------------------------------------------------
    double v_forward = 0.5;     // m/s for forward motion
    double v_backward = 0.3;    // m/s for backward motion
    double v_transition = 0.15;  // m/s when switching direction
    ReloPush::StatePath timedTrajectory = generateTimedTrajectory(basePath, v_forward, v_backward, v_transition);

    // -----------------------------------------------------------------------
    // Set the Trajectory in the View for Visualization
    // -----------------------------------------------------------------------
    view->setTrajectory(timedTrajectory);

    window.show();
    return app.exec();
}
