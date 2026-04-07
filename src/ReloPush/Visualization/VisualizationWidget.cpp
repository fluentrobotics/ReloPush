#include <ReloPush/Visualization/VisualizationWidget.h>
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

void VisualizationWidget::setPath(const std::vector<ReloPush::State>& path_, const std::vector<size_t>& path_segment_lengths_)
{
    path = path_;
    path_segment_lengths = path_segment_lengths_;
    // Normalize yaw for all states
    for(auto &state : path){
        state.yaw = normalizeYaw(state.yaw);
    }
    update();
}

void VisualizationWidget::setObstacles(const std::vector<ObjectInfo> &obstacles_)
{
    obstacles = obstacles_;
    update();
}

void VisualizationWidget::setPreRelocations(const std::vector<ReloPush::State>& prerelocs_)
{
    prerelocations = prerelocs_;
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

    // Determine available drawing area with padding
    int widget_width = width();
    int widget_height = height();
    int padding = 10;  // 10px padding on each side
    float availableWidth = widget_width - 2 * padding;
    float availableHeight = widget_height - 2 * padding;

    // Compute a drawing rectangle (draw_rect) that has the specified aspect ratio (aspect_ratio)
    // aspect_ratio is defined as (width / height) and is a member variable.
    float aspect_ratio = workspace_width/workspace_height;
    float availableRatio = availableWidth / availableHeight;
    QRectF draw_rect;
    if (availableRatio > aspect_ratio)
    {
        // The available area is wider than desired;
        // use full available height and compute width from aspect_ratio.
        float draw_height = availableHeight;
        float draw_width = aspect_ratio * draw_height;
        float left = (widget_width - draw_width) / 2.0f;
        draw_rect = QRectF(left, padding, draw_width, draw_height);
    }
    else
    {
        // The available area is taller than desired;
        // use full available width and compute height from aspect_ratio.
        float draw_width = availableWidth;
        float draw_height = draw_width / aspect_ratio;
        float top = (widget_height - draw_height) / 2.0f;
        draw_rect = QRectF(padding, top, draw_width, draw_height);
    }

    // Compute uniform scale factor so that scale_x == scale_y.
    float uniformScale = std::min(draw_rect.width() / workspace_width,
                                  draw_rect.height() / workspace_height);

    // Determine the actual drawn boundary dimensions using uniform scaling.
    float drawnWidth = workspace_width * uniformScale;
    float drawnHeight = workspace_height * uniformScale;

    // Center the drawn boundary within draw_rect.
    float offsetX = draw_rect.left() + (draw_rect.width() - drawnWidth) / 2.0f;
    float offsetY = draw_rect.top() + (draw_rect.height() - drawnHeight) / 2.0f;

    QRectF boundaryRect(offsetX, offsetY, drawnWidth, drawnHeight);
    painter.setPen(Qt::black);
    painter.drawRect(boundaryRect);

    // Lambda to map workspace coordinates to widget coordinates using the uniform scale.
    auto mapCoord = [&](float x, float y) -> QPointF
    {
        float widget_x = offsetX + x * uniformScale;
        // Invert y-axis so that larger y is higher (Cartesian coordinates)
        float widget_y = offsetY + drawnHeight - y * uniformScale;
        return QPointF(widget_x, widget_y);
    };

    // --- Your drawing code for paths, segments, goals, obstacles, etc. remains below ---
    // (For example, your code for drawing the segmented path goes here.)

    if (path.size() >= 2) {
        size_t total = std::accumulate(path_segment_lengths.begin(), path_segment_lengths.end(), size_t(0));
        if (!path_segment_lengths.empty() && total == path.size()) {
            size_t index = 0;
            // Predefined list of segment colors (customize as needed)
            std::vector<QColor> segmentColors = { QColor("#FCD0A1"), QColor("#E07A5F"),
                                                 QColor("#798086"), QColor("#81B29A"), Qt::darkCyan ,Qt::black, Qt::darkGreen};
            for (size_t seg = 0; seg < path_segment_lengths.size(); seg++) {
                size_t segLength = path_segment_lengths[seg];
                QColor segColor = segmentColors[seg % segmentColors.size()];
                QPen segPen(segColor, 2);
                painter.setPen(segPen);
                for (size_t i = index + 1; i < index + segLength; i++) {
                    QPointF p1 = mapCoord(path[i-1].x, path[i-1].y);
                    QPointF p2 = mapCoord(path[i].x, path[i].y);
                    painter.drawLine(p1, p2);
                }
                // Optionally, draw an arrow in the middle of the segment if segment length >= 2.
                if (segLength >= 2) {
                    size_t arrow_index = index + segLength / 2;
                    QPointF arrowPos = mapCoord(path[arrow_index].x, path[arrow_index].y);
                    float yaw = path[arrow_index].yaw;
                    drawOrientedArrow(painter, arrowPos, yaw, segColor, true);
                }
                index += segLength;
            }
        } else {
            // Fallback: draw entire path in global path_color as before.
            QPen path_pen(path_color, 2);
            painter.setPen(path_pen);
            for (size_t i = 1; i < path.size(); ++i) {
                QPointF p1 = mapCoord(path[i-1].x, path[i-1].y);
                QPointF p2 = mapCoord(path[i].x, path[i].y);
                painter.drawLine(p1, p2);
            }
            // Draw global path arrows.
            const size_t max_arrows = 20;
            size_t arrow_interval = path.size() > max_arrows ? path.size() / max_arrows : 1;
            for (size_t i = 0; i < path.size(); i += arrow_interval) {
                QPointF pos = mapCoord(path[i].x, path[i].y);
                float yaw = path[i].yaw;
                drawOrientedArrow(painter, pos, yaw, path_arrow_color, true);
            }
            if ((path.size() - 1) % arrow_interval != 0) {
                QPointF pos = mapCoord(path.back().x, path.back().y);
                float yaw = path.back().yaw;
                drawOrientedArrow(painter, pos, yaw, path_arrow_color, true);
            }
        }
    }

    // Draw initial and goal poses.
    //drawOrientedArrow(painter, mapCoord(initial_pose.x, initial_pose.y), initial_pose.yaw, initial_pose_color, false);
    //drawOrientedArrow(painter, mapCoord(goal_pose.x, goal_pose.y), goal_pose.yaw, goal_pose_color, false);

    // Draw additional elements (goals, obstacles, prerelocations) as needed...
    // For example, if you have goals stored in a container:
    if (!goals.empty())
    {
        QBrush goalBrush(goals_color);
        QPen goalPen(Qt::black, 1);
        painter.setBrush(goalBrush);
        painter.setPen(goalPen);
        for (const auto &g : goals)
        {
            QPointF pos = mapCoord(g.second.x, g.second.y);
            drawGoals(painter, pos, g.second.nominalOrientation, goals_color);
        }
    }

    // Draw obstacles (if any) after the path.
    if (!obstacles.empty())
    {
        QBrush obsBrush(obstacle_color);
        QPen obsPen(Qt::black, 1);
        painter.setBrush(obsBrush);
        painter.setPen(obsPen);
        for (const auto &obs : obstacles)
        {
            QPointF pos = mapCoord(obs.x, obs.y);
            // Assuming drawObstacle accepts (painter, position, color)
            drawObstacle(painter, pos, obs.nominalOrientation, obstacle_color);
        }
    }

    if (!prerelocations.empty())
    {
        QPen dottedPen(QColor(255, 165, 0));
        dottedPen.setStyle(Qt::DashLine);
        dottedPen.setWidth(2);
        painter.setPen(dottedPen);
        painter.setBrush(Qt::NoBrush);
        for (const auto &pre : prerelocations)
        {
            QPointF pos = mapCoord(pre.x,pre.y);
            drawPreRelocations(painter, pos, pre.yaw, QColor(255, 165, 0));
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

void VisualizationWidget::drawPreRelocations(QPainter &painter, const QPointF &position, const float yaw, QColor color) const
{
    // Calculate the scaling factor based on the widget size and workspace dimensions
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

    // Set up a dotted pen and no brush
    QPen pen(color);
    pen.setStyle(Qt::DashLine);
    pen.setWidth(1);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    // Draw the dotted box
    painter.drawPolygon(polygon);
}

float VisualizationWidget::normalizeYaw(float yaw) const
{
    while (yaw < 0) yaw += 2 * M_PI;
    while (yaw >= 2 * M_PI) yaw -= 2 * M_PI;
    return yaw;
}
