#include <Visualization/VisualizationWidget.h>
#include <QPainter>
#include <QMouseEvent>
#include <QToolTip>
#include <cmath>

// Define PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

VisualizationWidget::VisualizationWidget(QWidget *parent,
                                         QColor initialPoseColor,
                                         QColor goalPoseColor,
                                         QColor pathColor_,
                                         QColor pathArrowColor_,
                                         QColor obstacleColor_,
                                         QColor goalsColor_)
    : QWidget(parent),
    workspace_width(100.0f),
    workspace_height(100.0f),
    initial_pose(10.0f, 10.0f, 0.0f),
    goal_pose(90.0f, 90.0f, 0.0f),
    initial_pose_color(initialPoseColor),
    goal_pose_color(goalPoseColor),
    path_color(pathColor_),
    path_arrow_color(pathArrowColor_),
    obstacle_color(obstacleColor_),
    goals_color(goalsColor_)
{
    setMouseTracking(true); // Enable mouse tracking without pressing buttons
}


void VisualizationWidget::setWorkspace(float width, float height)
{
    if (width <= 0 || height <= 0)
    {
        // Handle invalid workspace dimensions
        return;
    }
    workspace_width = width;
    workspace_height = height;
    update();
}

void VisualizationWidget::setInitialPose(const ReloPush::State& state)
{
    initial_pose = state;
    initial_pose.yaw = normalizeYaw(initial_pose.yaw);
    update();
}

void VisualizationWidget::setGoalPose(const ReloPush::State& state)
{
    goal_pose = state;
    goal_pose.yaw = normalizeYaw(goal_pose.yaw);
    update();
}

void VisualizationWidget::setPath(const std::vector<ReloPush::State>& path_)
{
    path = path_;
    // Normalize yaw for all states
    for(auto &state : path){
        state.yaw = normalizeYaw(state.yaw);
    }
    update();
}

void VisualizationWidget::setObstacles(const std::vector<ReloPush::State> &obstacles_)
{
    obstacles = obstacles_;
    update();
}

void VisualizationWidget::setGoals(const GoalMap &goals_)
{
    goals = goals_;
    update();
}

// Setter methods for colors
void VisualizationWidget::setInitialPoseColor(const QColor& color)
{
    initial_pose_color = color;
    update();
}

void VisualizationWidget::setGoalPoseColor(const QColor& color)
{
    goal_pose_color = color;
    update();
}

void VisualizationWidget::setPathColor(const QColor& color)
{
    path_color = color;
    update();
}

void VisualizationWidget::setPathArrowColor(const QColor& color)
{
    path_arrow_color = color;
    update();
}

void VisualizationWidget::setObstacleColor(const QColor &color)
{
    obstacle_color = color;
    update();
}

void VisualizationWidget::setGoalsColor(const QColor &color)
{
    goals_color = color;
    update();
}

void VisualizationWidget::paintEvent(QPaintEvent * /* event */)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Fill background
    painter.fillRect(rect(), Qt::white);

    // Determine the square drawing area
    int widget_width = width();
    int widget_height = height();
    int side = qMin(widget_width, widget_height) - 20; // Subtract padding (10px on each side)
    if (side <= 0)
        side = qMin(widget_width, widget_height); // Prevent negative size

    // Calculate top-left corner to center the square
    int left = (widget_width - side) / 2;
    int top = (widget_height - side) / 2;

    QRectF draw_rect = QRectF(left, top, side, side);
    painter.setPen(Qt::black);
    painter.drawRect(draw_rect);

    // Scaling factors
    float scale_x = draw_rect.width() / workspace_width;
    float scale_y = draw_rect.height() / workspace_height;

    // Lambda to map workspace coordinates to widget coordinates
    auto mapCoord = [&](float x, float y) -> QPointF
    {
        float widget_x = draw_rect.left() + x * scale_x;
        // Invert y-axis to match traditional Cartesian coordinates
        float widget_y = draw_rect.bottom() - y * scale_y;
        return QPointF(widget_x, widget_y);
    };

    // Draw path next
    if (path.size() >= 2)
    {
        QPen path_pen(path_color, 2);
        painter.setPen(path_pen);
        for (size_t i = 1; i < path.size(); ++i)
        {
            QPointF p1 = mapCoord(path[i - 1].x, path[i - 1].y);
            QPointF p2 = mapCoord(path[i].x, path[i].y);
            painter.drawLine(p1, p2);
        }

        // Draw path arrows
        // Define interval for arrows (e.g., every N states)
        const size_t max_arrows = 20;
        size_t arrow_interval = path.size() > max_arrows ? path.size() / max_arrows : 1;

        for (size_t i = 0; i < path.size(); i += arrow_interval)
        {
            QPointF pos = mapCoord(path[i].x, path[i].y);
            float yaw = path[i].yaw;
            drawOrientedArrow(painter, pos, yaw, path_arrow_color, true);
        }

        // Ensure the last state has an arrow if not already drawn
        if ((path.size() - 1) % arrow_interval != 0)
        {
            QPointF pos = mapCoord(path.back().x, path.back().y);
            float yaw = path.back().yaw;
            drawOrientedArrow(painter, pos, yaw, path_arrow_color, true);
        }
    }

    // Draw initial pose as an oriented arrow
    drawOrientedArrow(painter, mapCoord(initial_pose.x, initial_pose.y), initial_pose.yaw, initial_pose_color, false);

    // Draw goal pose as an oriented arrow
    drawOrientedArrow(painter, mapCoord(goal_pose.x, goal_pose.y), goal_pose.yaw, goal_pose_color, false);

    if (!goals.empty())
    {
        QBrush obstacle_brush(goals_color);
        QPen obstacle_pen(Qt::black, 1);
        painter.setBrush(obstacle_brush);
        painter.setPen(obstacle_pen);
        for (const auto &obs : goals)
        {
            QPointF pos = mapCoord(obs.second.x, obs.second.y);
            drawGoals(painter, pos, obs.second.nominalOrientation, goals_color);
        }
    }

    // Draw obstacles first (so they appear below the path and arrows)
    if (!obstacles.empty())
    {
        QBrush obstacle_brush(obstacle_color);
        QPen obstacle_pen(Qt::black, 1);
        painter.setBrush(obstacle_brush);
        painter.setPen(obstacle_pen);
        for (const auto &obs : obstacles)
        {
            QPointF pos = mapCoord(obs.x, obs.y);
            drawObstacle(painter, pos, obs.yaw, obstacle_color);
        }
    }
}

void VisualizationWidget::mouseMoveEvent(QMouseEvent *event)
{
    // Map widget coordinates back to workspace coordinates
    QRectF draw_rect = QRectF(10, 10, width() - 20, height() - 20);
    float scale_x = workspace_width / draw_rect.width();
    float scale_y = workspace_height / draw_rect.height();

    float x = (event->pos().x() - draw_rect.left()) * scale_x;
    float y = (draw_rect.bottom() - event->pos().y()) * scale_y;

    // Clamp values to workspace
    if (x < 0) x = 0;
    if (x > workspace_width) x = workspace_width;
    if (y < 0) y = 0;
    if (y > workspace_height) y = workspace_height;

    QString coord_text = QString("X: %1, Y: %2").arg(x, 0, 'f', 2).arg(y, 0, 'f', 2);
    QToolTip::showText(event->globalPos(), coord_text, this);
}

void VisualizationWidget::resizeEvent(QResizeEvent * /* event */)
{
    update();
}

void VisualizationWidget::drawOrientedArrow(QPainter& painter, const QPointF& position, float yaw, QColor color, bool isPath) const
{
    // Define base arrow shape with tail pointing to the left
    // Size of the arrow
    float arrow_length = isPath ? 10.0f : 20.0f; // Smaller for path
    float arrow_width = isPath ? 5.0f : 10.0f;    // Smaller for path
    float tail_length = isPath ? 5.0f : 10.0f;    // Smaller for path

    // Define the arrow with tail: tail points to the left, head to the right
    QPolygonF arrow;
    arrow << QPointF(-tail_length, -arrow_width / 2) // Tail left-bottom
          << QPointF(0, -arrow_width / 2)           // Tail right-bottom
          << QPointF(0, -arrow_width)              // Bottom of tail
          << QPointF(arrow_length, 0)              // Head point
          << QPointF(0, arrow_width)               // Top of tail
          << QPointF(0, arrow_width / 2)           // Tail right-top
          << QPointF(-tail_length, arrow_width / 2); // Tail left-top

    // Create a QTransform to rotate and translate the arrow
    QTransform transform;
    // Convert yaw from radians to degrees
    float yaw_deg = yaw * 180.0f / M_PI;
    transform.translate(position.x(), position.y());
    transform.rotate(-yaw_deg); // Negative for counterclockwise rotation

    // Apply transformation
    QPolygonF transformed_arrow = transform.map(arrow);

    // Set brush and pen
    QBrush brush(color);
    QPen pen(Qt::black, 1);
    painter.setBrush(brush);
    painter.setPen(pen);

    // Draw the arrow
    painter.drawPolygon(transformed_arrow);
}

void VisualizationWidget::drawObstacle(QPainter &painter, const QPointF &position, const float yaw, QColor color) const
{
    // Calculate the diameter based on the fixed radius and scaling
    // Assuming uniform scaling (scale_x == scale_y)
    float scale = std::min(width(), height()) / (workspace_width > workspace_height ? workspace_width : workspace_height);
    float diameter = obstacle_radius * 2 * scale;
    float radius = obstacle_radius * scale;

    float offset = diameter/2;

    // Define the unrotated corners (relative to center)
    QVector<QPointF> corners;
    corners << QPointF(-radius, -radius)
            << QPointF(radius, -radius)
            << QPointF(radius, radius)
            << QPointF(-radius, radius);

    // Rotate and translate each corner
    QPolygonF polygon;
    for (const QPointF &pt : corners) {
        double rotatedX = pt.x() * std::cos(-yaw) - pt.y() * std::sin(-yaw);
        double rotatedY = pt.x() * std::sin(-yaw) + pt.y() * std::cos(-yaw);
        polygon << QPointF(rotatedX + position.x(), rotatedY + position.y());
    }

    painter.drawPolygon(polygon);
}

void VisualizationWidget::drawGoals(QPainter &painter, const QPointF &position, const float yaw, QColor color) const
{
    // Calculate the diameter based on the fixed radius and scaling
    // Assuming uniform scaling (scale_x == scale_y)
    float scale = std::min(width(), height()) / (workspace_width > workspace_height ? workspace_width : workspace_height);
    float diameter = obstacle_radius * 2 * scale;
    float radius = obstacle_radius * scale;

    float offset = diameter/2;

    float outer_margin = 1.2; // to make it visible when overlapped with obstacle
    // Define the unrotated corners (relative to center)
    QVector<QPointF> corners;
    corners << QPointF(-radius*outer_margin, -radius*outer_margin)
            << QPointF(radius*outer_margin, -radius*outer_margin)
            << QPointF(radius*outer_margin, radius*outer_margin)
            << QPointF(-radius*outer_margin, radius*outer_margin);

    // Rotate and translate each corner
    QPolygonF polygon;
    for (const QPointF &pt : corners) {
        double rotatedX = pt.x() * std::cos(-yaw) - pt.y() * std::sin(-yaw);
        double rotatedY = pt.x() * std::sin(-yaw) + pt.y() * std::cos(-yaw);
        polygon << QPointF(rotatedX + position.x(), rotatedY + position.y());
    }

    painter.drawPolygon(polygon);
}

float VisualizationWidget::normalizeYaw(float yaw) const
{
    while (yaw < 0) yaw += 2 * M_PI;
    while (yaw >= 2 * M_PI) yaw -= 2 * M_PI;
    return yaw;
}
