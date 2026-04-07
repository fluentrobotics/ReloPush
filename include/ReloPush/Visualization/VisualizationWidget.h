#ifndef VISUALIZATIONWIDGET_H
#define VISUALIZATIONWIDGET_H

#include <QWidget>
#include <vector>
#include <QColor>
#include <ReloPush/State.h>
#include <ReloPush/GraphData.hpp>

class VisualizationWidget : public QWidget
{
    //Q_OBJECT

public:
    explicit VisualizationWidget(QWidget *parent = nullptr,
                                 QColor initialPoseColor = Qt::blue,
                                 QColor goalPoseColor = Qt::green,
                                 QColor pathColor = Qt::red,
                                 QColor pathArrowColor = Qt::darkRed,
                                 QColor obstacleColor = Qt::gray,
                                 QColor goalsColor = Qt::red);


    // Set workspace dimensions
    void setWorkspace(float width, float height);

    // Set initial and goal poses
    void setInitialPose(const ReloPush::State& state);
    void setGoalPose(const ReloPush::State& state);

    // Set path
    void setPath(const std::vector<ReloPush::State>& path);
    void setPath(const std::vector<ReloPush::State>& path_, const std::vector<size_t>& path_segment_lengths_);

    // Set obstacles
    void setObstacles(const std::vector<ObjectInfo> &obstacles);

    // Set goals
    void setGoals(const GoalMap &goals);

    void setPreRelocations(const std::vector<ReloPush::State>& prerelocs);

    // Set colors
    void setInitialPoseColor(const QColor& color);
    void setGoalPoseColor(const QColor& color);
    void setPathColor(const QColor& color);
    void setPathArrowColor(const QColor& color);
    void setObstacleColor(const QColor &color);
    void setGoalsColor(const QColor &color);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    float workspace_width;
    float workspace_height;

    ReloPush::State initial_pose;
    ReloPush::State goal_pose;
    std::vector<ReloPush::State> path;
    std::vector<ObjectInfo> obstacles;
    std::vector<ReloPush::State> prerelocations;
    std::vector<size_t> path_segment_lengths;
    GoalMap goals;

    QString mouse_coord_text;

    // Color parameters
    QColor initial_pose_color;
    QColor goal_pose_color;
    QColor path_color;
    QColor path_arrow_color;
    QColor obstacle_color;
    QColor goals_color;

    // Obstacle parameters
    const float obstacle_radius = 0.075f;
    const float prereloc_line_size = 0.2;


    // Coordinate mapping functions
    QPointF workspaceToWidget(float x, float y) const;

    // Utility functions to draw oriented arrows
    void drawOrientedArrow(QPainter& painter, const QPointF& position, float yaw, QColor color, bool isPath = false) const;
    void drawObstacle(QPainter &painter, const QPointF &position, const float yaw, QColor color) const;
    void drawGoals(QPainter &painter, const QPointF &position, const float yaw, QColor color) const;
    void drawPreRelocations(QPainter &painter, const QPointF &position, const float yaw, QColor color) const;

    // Utility function to normalize yaw
    float normalizeYaw(float yaw) const;
};

#endif // VISUALIZATIONWIDGET_H
