/*************************************
 * Path Visualization using Qt
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
 *************************************/

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QSlider>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QDialog>
#include <QDebug>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QFontMetrics>
#include <QKeyEvent>
#include <QMouseEvent>

#include <algorithm>
#include <string>
#include <vector>

#include <PHAstar/Node.h>
#include <PHAstar/Point.h>
#include <PHAstar/Entities.h>
#include <PHAstar/Params.h>
#include <PHAstar/TimeTable.h>
#include <PHAstar/PlanningResult.h>

std::tuple<double, double, double> interpolate_timed_path(const std::vector<Waypoint> &waypoints, double t)
{
    if (t <= waypoints[0].time)
        return {waypoints[0].x, waypoints[0].y, waypoints[0].yaw};
    if (t >= waypoints.back().time)
        return {waypoints.back().x, waypoints.back().y, waypoints.back().yaw};

    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        auto &wp1 = waypoints[i];
        auto &wp2 = waypoints[i + 1];
        if (wp1.time <= t && t <= wp2.time)
        {
            double frac = (t - wp1.time) / (wp2.time - wp1.time);
            double x = wp1.x + frac * (wp2.x - wp1.x);
            double y = wp1.y + frac * (wp2.y - wp1.y);

            // FIX: Use pi_2_pi to get the shortest angular distance (range -PI to PI)
            double dyaw = pi_2_pi(wp2.yaw - wp1.yaw);

            double yaw = wp1.yaw + frac * dyaw;
            yaw = mod2pi(yaw); // Keep the final result in [0, 2PI)
            return {x, y, yaw};
        }
    }
    return {waypoints.back().x, waypoints.back().y, waypoints.back().yaw};
}

// to be depricated
class VizWidget_old : public QWidget
{
private:
    const TimeTable &timetable;
    const std::unordered_map<std::string, EntityMeta *> &entities;
    const std::vector<Trajectory> &trajectories;
    const Params &params;
    double current_t = 0.0;
    double max_t = 0.0;

    void drawFilledPolygon(QPainter &p, const Corners &corners, const QColor &color, const std::function<double(double)> &screen_x, const std::function<double(double)> &screen_y)
    {
        QPolygonF poly;
        for (const auto &c : corners)
            poly << QPointF(screen_x(c.x), screen_y(c.y));
        poly << QPointF(screen_x(corners[0].x), screen_y(corners[0].y));
        p.setPen(color);
        p.setBrush(color);
        p.drawPolygon(poly);
    }

public:
    VizWidget_old(const TimeTable &tt, const std::unordered_map<std::string, EntityMeta *> &ents, const std::vector<Trajectory> &trajs, const Params &p)
        : timetable(tt), entities(ents), trajectories(trajs), params(p)
    {
        for (const auto &traj : trajectories)
        {
            if (!traj.waypoints.empty())
                max_t = timetable.get_max_time();
        }
        setMinimumSize(600, 600);
    }

    void setTime(double t)
    {
        current_t = t;
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter p(this);
        p.fillRect(rect(), Qt::white);

        double scale_x = static_cast<double>(width()) / (params.max_x - params.min_x);
        double scale_y = static_cast<double>(height()) / (params.max_y - params.min_y);
        double sc = std::min(scale_x, scale_y);
        auto screen_x = [&](double x)
        { return (x - params.min_x) * sc; };
        auto screen_y = [&](double y)
        { return (params.max_y - y) * sc; };

        // Draw grid
        p.setPen(Qt::lightGray);
        for (double x = params.min_x; x <= params.max_x + 1e-6; x += params.xy_resolution * 5)
        {
            double sx = screen_x(x);
            double sy1 = screen_y(params.min_y);
            double sy2 = screen_y(params.max_y);
            p.drawLine(QPointF(sx, sy1), QPointF(sx, sy2));
        }
        for (double y = params.min_y; y <= params.max_y + 1e-6; y += params.xy_resolution * 5)
        {
            double sy = screen_y(y);
            double sx1 = screen_x(params.min_x);
            double sx2 = screen_x(params.max_x);
            p.drawLine(QPointF(sx1, sy), QPointF(sx2, sy));
        }

        // Draw boundary
        p.setPen(QPen(Qt::black, 3));
        p.drawRect(QRectF(screen_x(params.min_x) + 1, screen_y(params.max_y) + 1, sc * (params.max_x - params.min_x) - 3, sc * (params.max_y - params.min_y) - 3));

        // Draw paths
        p.setPen(QPen(QColor("#FFD6C2"), 4));
        for (const auto &traj : trajectories)
        {
            QPolygonF path_poly;
            for (const auto &wp : traj.waypoints)
            {
                path_poly << QPointF(screen_x(wp.x), screen_y(wp.y));
            }
            p.drawPolyline(path_poly);
        }

        // Draw entities
        auto poses = timetable.get_poses(current_t);
        for (const auto &[ent, pose] : poses)
        {
            QColor color = (ent->type == EntityType::ROBOT) ? QColor("#555B6E") : QColor("#89B0AE");
            auto corners = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            drawFilledPolygon(p, corners, color, screen_x, screen_y);

            // Indicate front with a red arrow for robots
            if (ent->type == EntityType::ROBOT)
            {
                double front_x = pose.x + ent->size.front_length * std::cos(pose.yaw);
                double front_y = pose.y + ent->size.front_length * std::sin(pose.yaw);
                p.setPen(QPen(QColor("FFD6BA"), 2));
                p.drawLine(screen_x(pose.x), screen_y(pose.y), screen_x(front_x), screen_y(front_y));
            }

            // Draw Name
            p.setPen(Qt::black);
            p.drawText(QPointF(screen_x(pose.x), screen_y(pose.y)), QString::fromStdString(ent->name));
        }
    }
};

class VizWidget : public QWidget
{
private:
    const TimeTable &timetable;
    const std::unordered_map<std::string, EntityMeta *> &entities;
    const Params &params;
    double current_t = 0.0;
    double max_t = 0.0;

    void drawFilledPolygon(QPainter &p, const Corners &corners, const QColor &color, const std::function<double(double)> &screen_x, const std::function<double(double)> &screen_y)
    {
        QPolygonF poly;
        for (const auto &c : corners)
            poly << QPointF(screen_x(c.x), screen_y(c.y));
        poly << QPointF(screen_x(corners[0].x), screen_y(corners[0].y));
        p.setPen(color);
        p.setBrush(color);
        p.drawPolygon(poly);
    }

public:
    VizWidget(const TimeTable &tt, const std::unordered_map<std::string, EntityMeta *> &ents, const Params &p)
        : timetable(tt), entities(ents), params(p)
    {
        max_t = tt.get_max_time();
        setMinimumSize(600, 600);
    }

    void setTime(double t)
    {
        current_t = t;
        update();
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter p(this);
        p.fillRect(rect(), Qt::white);

        double scale_x = static_cast<double>(width()) / (params.max_x - params.min_x);
        double scale_y = static_cast<double>(height()) / (params.max_y - params.min_y);
        double sc = std::min(scale_x, scale_y);
        auto screen_x = [&](double x)
        { return (x - params.min_x) * sc; };
        auto screen_y = [&](double y)
        { return (params.max_y - y) * sc; };

        // Draw grid
        p.setPen(Qt::lightGray);
        for (double x = params.min_x; x <= params.max_x + 1e-6; x += params.xy_resolution * 5)
        {
            double sx = screen_x(x);
            double sy1 = screen_y(params.min_y);
            double sy2 = screen_y(params.max_y);
            p.drawLine(QPointF(sx, sy1), QPointF(sx, sy2));
        }
        for (double y = params.min_y; y <= params.max_y + 1e-6; y += params.xy_resolution * 5)
        {
            double sy = screen_y(y);
            double sx1 = screen_x(params.min_x);
            double sx2 = screen_x(params.max_x);
            p.drawLine(QPointF(sx1, sy), QPointF(sx2, sy));
        }

        // Draw boundary
        p.setPen(QPen(Qt::black, 3));
        p.drawRect(QRectF(screen_x(params.min_x) + 1, screen_y(params.max_y) + 1, sc * (params.max_x - params.min_x) - 3, sc * (params.max_y - params.min_y) - 3));

        // Draw entities
        auto poses = timetable.get_poses(current_t);
        for (const auto &[ent, pose] : poses)
        {
            if (ent == nullptr)
            {
                break;
            } // temp fix
            QColor color = (ent->type == EntityType::ROBOT) ? QColor("#555B6E") : QColor("#89B0AE");
            auto corners = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            drawFilledPolygon(p, corners, color, screen_x, screen_y);

            // Indicate front with a red arrow for robots
            if (ent->type == EntityType::ROBOT)
            {
                double front_x = pose.x + ent->size.front_length * std::cos(pose.yaw);
                double front_y = pose.y + ent->size.front_length * std::sin(pose.yaw);
                p.setPen(QPen(QColor("FFD6BA"), 2));
                p.drawLine(screen_x(pose.x), screen_y(pose.y), screen_x(front_x), screen_y(front_y));
            }

            // Draw Name
            p.setPen(Qt::black);
            p.drawText(QPointF(screen_x(pose.x), screen_y(pose.y)), QString::fromStdString(ent->name));
        }
    }
};

void show_results(int argc, char **argv, const TimeTable &timetable, const std::unordered_map<std::string, EntityMeta *> &entities,
                  const Params &params)
{
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget *viz = new VizWidget(timetable, entities, params);
    win.setCentralWidget(viz);

    double max_t = timetable.get_max_time();
    int max_val = static_cast<int>(max_t * 100 + 0.5);

    QWidget *panel = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout(panel);

    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, max_val);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel *timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Compute minimum width to prevent layout reflow
    QFontMetrics fm(timeLabel->font());
    QString maxTimeStr = QString("Time: %1 s").arg(max_t, 0, 'f', 2);
    int labelWidth = fm.horizontalAdvance(maxTimeStr) + 20; // +20 for padding
    timeLabel->setMinimumWidth(labelWidth);

    QDoubleSpinBox *stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, qMax(0.01, max_t));
    stepSpin->setSingleStep(0.05);
    stepSpin->setDecimals(2);
    stepSpin->setValue(0.50); // Changed to 0.50 s as requested
    stepSpin->setSuffix(" s");

    QPushButton *prevBtn = new QPushButton("<<");
    QPushButton *nextBtn = new QPushButton(">>");

    // Enable auto-repeat
    prevBtn->setAutoRepeat(true);
    prevBtn->setAutoRepeatDelay(300);
    prevBtn->setAutoRepeatInterval(100);
    nextBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeatDelay(300);
    nextBtn->setAutoRepeatInterval(100);

    layout->addWidget(new QLabel("Step:"));
    layout->addWidget(stepSpin);
    layout->addWidget(prevBtn);
    layout->addWidget(slider);
    layout->addWidget(nextBtn);
    layout->addWidget(timeLabel);

    // Throttled live update during drag
    QTimer *dragUpdateTimer = new QTimer(&win);
    dragUpdateTimer->setInterval(50);

    QObject::connect(dragUpdateTimer, &QTimer::timeout, [=]()
                     {
        double t = slider->value() / 100.0;
        viz->setTime(t); });

    QObject::connect(slider, &QSlider::sliderPressed, [=]()
                     { dragUpdateTimer->start(); });

    QObject::connect(slider, &QSlider::sliderReleased, [=]()
                     {
        dragUpdateTimer->stop();
        double t = slider->value() / 100.0;
        viz->setTime(t); });

    QObject::connect(slider, &QSlider::valueChanged, [=](int val)
                     {
        double t = val / 100.0;
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
        if (!slider->isSliderDown()) {
            viz->setTime(t);
        } });

    // Step buttons
    QObject::connect(prevBtn, &QPushButton::clicked, [=]()
                     {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMax(0.0, curr_t - step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5)); });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]()
                     {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMin(max_t, curr_t + step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5)); });

    slider->setValue(0);

    QDockWidget *dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    win.show();
    app.exec();
}

void show_results_comparison(
    int argc, char **argv,
    const QString &left_title,
    const TimeTable &left_timetable,
    const std::unordered_map<std::string, EntityMeta *> &left_entities,
    const Params &left_params,
    const QString &right_title,
    const TimeTable &right_timetable,
    const std::unordered_map<std::string, EntityMeta *> &right_entities,
    const Params &right_params)
{
    QApplication app(argc, argv);
    QMainWindow win;

    QWidget *central = new QWidget;
    QHBoxLayout *compareLayout = new QHBoxLayout(central);
    compareLayout->setContentsMargins(8, 8, 8, 8);
    compareLayout->setSpacing(8);

    auto makePane = [&](const QString &title,
                        const TimeTable &timetable,
                        const std::unordered_map<std::string, EntityMeta *> &entities,
                        const Params &params)
    {
        QWidget *pane = new QWidget;
        QVBoxLayout *paneLayout = new QVBoxLayout(pane);
        paneLayout->setContentsMargins(0, 0, 0, 0);
        paneLayout->setSpacing(6);

        QLabel *titleLabel = new QLabel(title);
        titleLabel->setAlignment(Qt::AlignCenter);
        titleLabel->setWordWrap(true);
        QFont titleFont = titleLabel->font();
        titleFont.setBold(true);
        titleLabel->setFont(titleFont);

        VizWidget *viz = new VizWidget(timetable, entities, params);
        paneLayout->addWidget(titleLabel);
        paneLayout->addWidget(viz, 1);
        return std::make_pair(pane, viz);
    };

    auto leftPaneAndViz =
        makePane(left_title, left_timetable, left_entities, left_params);
    auto rightPaneAndViz =
        makePane(right_title, right_timetable, right_entities, right_params);
    QWidget *leftPane = leftPaneAndViz.first;
    VizWidget *leftViz = leftPaneAndViz.second;
    QWidget *rightPane = rightPaneAndViz.first;
    VizWidget *rightViz = rightPaneAndViz.second;

    compareLayout->addWidget(leftPane, 1);
    compareLayout->addWidget(rightPane, 1);
    win.setCentralWidget(central);

    double max_t = std::max(left_timetable.get_max_time(),
                            right_timetable.get_max_time());
    int max_val = static_cast<int>(max_t * 100 + 0.5);

    QWidget *panel = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout(panel);

    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, max_val);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel *timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    QFontMetrics fm(timeLabel->font());
    QString maxTimeStr = QString("Time: %1 s").arg(max_t, 0, 'f', 2);
    int labelWidth = fm.horizontalAdvance(maxTimeStr) + 20;
    timeLabel->setMinimumWidth(labelWidth);

    QDoubleSpinBox *stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, qMax(0.01, max_t));
    stepSpin->setSingleStep(0.05);
    stepSpin->setDecimals(2);
    stepSpin->setValue(0.50);
    stepSpin->setSuffix(" s");

    QPushButton *prevBtn = new QPushButton("<<");
    QPushButton *nextBtn = new QPushButton(">>");
    prevBtn->setAutoRepeat(true);
    prevBtn->setAutoRepeatDelay(300);
    prevBtn->setAutoRepeatInterval(100);
    nextBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeatDelay(300);
    nextBtn->setAutoRepeatInterval(100);

    layout->addWidget(new QLabel("Step:"));
    layout->addWidget(stepSpin);
    layout->addWidget(prevBtn);
    layout->addWidget(slider);
    layout->addWidget(nextBtn);
    layout->addWidget(timeLabel);

    auto updateViews = [=](double t)
    {
        leftViz->setTime(t);
        rightViz->setTime(t);
    };

    QTimer *dragUpdateTimer = new QTimer(&win);
    dragUpdateTimer->setInterval(50);

    QObject::connect(dragUpdateTimer, &QTimer::timeout, [=]()
                     {
        double t = slider->value() / 100.0;
        updateViews(t); });

    QObject::connect(slider, &QSlider::sliderPressed, [=]()
                     { dragUpdateTimer->start(); });

    QObject::connect(slider, &QSlider::sliderReleased, [=]()
                     {
        dragUpdateTimer->stop();
        double t = slider->value() / 100.0;
        updateViews(t); });

    QObject::connect(slider, &QSlider::valueChanged, [=](int val)
                     {
        double t = val / 100.0;
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
        if (!slider->isSliderDown()) {
            updateViews(t);
        } });

    QObject::connect(prevBtn, &QPushButton::clicked, [=]()
                     {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMax(0.0, curr_t - step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5)); });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]()
                     {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMin(max_t, curr_t + step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5)); });

    slider->setValue(0);

    QDockWidget *dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Replay Comparison - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(1280, 720);
    win.show();
    app.exec();
}

void show_results(int argc, char **argv, const TimeTable &timetable, const std::unordered_map<std::string, EntityMeta *> &entities, const std::vector<Trajectory> &all_trajectories, const Params &params)
{
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget_old *viz = new VizWidget_old(timetable, entities, all_trajectories, params);
    win.setCentralWidget(viz);

    double max_t = 0.0;
    for (const auto &traj : all_trajectories)
    {
        if (!traj.waypoints.empty())
            max_t = timetable.get_max_time();
    }
    int max_val = static_cast<int>(max_t * 100 + 0.5);

    QWidget *panel = new QWidget;
    QHBoxLayout *layout = new QHBoxLayout(panel);

    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, max_val);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel *timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Compute minimum width to prevent layout reflow
    QFontMetrics fm(timeLabel->font());
    QString maxTimeStr = QString("Time: %1 s").arg(max_t, 0, 'f', 2);
    int labelWidth = fm.horizontalAdvance(maxTimeStr) + 20;
    timeLabel->setMinimumWidth(labelWidth);

    QDoubleSpinBox *stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, qMax(0.01, max_t));
    stepSpin->setSingleStep(0.05);
    stepSpin->setDecimals(2);
    stepSpin->setValue(0.50); // Changed to 0.50 s
    stepSpin->setSuffix(" s");

    QPushButton *prevBtn = new QPushButton("<<");
    QPushButton *nextBtn = new QPushButton(">>");

    prevBtn->setAutoRepeat(true);
    prevBtn->setAutoRepeatDelay(300);
    prevBtn->setAutoRepeatInterval(100);
    nextBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeatDelay(300);
    nextBtn->setAutoRepeatInterval(100);

    layout->addWidget(new QLabel("Step:"));
    layout->addWidget(stepSpin);
    layout->addWidget(prevBtn);
    layout->addWidget(slider);
    layout->addWidget(nextBtn);
    layout->addWidget(timeLabel);

    QTimer *dragUpdateTimer = new QTimer(&win);
    dragUpdateTimer->setInterval(50);

    QObject::connect(dragUpdateTimer, &QTimer::timeout, [=]()
                     {
        double t = slider->value() / 100.0;
        viz->setTime(t); });

    QObject::connect(slider, &QSlider::sliderPressed, [=]()
                     { dragUpdateTimer->start(); });

    QObject::connect(slider, &QSlider::sliderReleased, [=]()
                     {
        dragUpdateTimer->stop();
        double t = slider->value() / 100.0;
        viz->setTime(t); });

    QObject::connect(slider, &QSlider::valueChanged, [=](int val)
                     {
        double t = val / 100.0;
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
        if (!slider->isSliderDown()) {
            viz->setTime(t);
        } });

    QObject::connect(prevBtn, &QPushButton::clicked, [=]()
                     {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMax(0.0, curr_t - step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5)); });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]()
                     {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMin(max_t, curr_t + step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5)); });

    slider->setValue(0);

    QDockWidget *dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    win.show();
    app.exec();
}

// In Visualization.h or a new DebugViz.h (include <QWidget>, <QPainter>, <QApplication>, <QDialog> if not already)
class DebugVisualizer : public QDialog
{
public:
    DebugVisualizer(const TimeTable &timetable, const std::unordered_map<std::string, EntityMeta *> &entities,
                    const Params &params, double query_time,
                    const Pose &start_pose = {}, const Pose &goal_pose = {},
                    const Trajectory *attempted_traj = nullptr,
                    double attempted_start_time = 0.0,
                    const CollisionInfo *failure_info = nullptr,
                    const std::string &context_text = "",
                    QWidget *parent = nullptr)
        : QDialog(parent), timetable_(timetable), entities_(entities), params_(params),
          query_time_(query_time), start_pose_(start_pose), goal_pose_(goal_pose),
          attempted_start_time_(attempted_start_time), context_text_(context_text)
    {
        if (attempted_traj)
        {
            attempted_traj_ = *attempted_traj;
            has_attempted_traj_ = true;
        }
        if (failure_info)
        {
            failure_info_ = *failure_info;
            has_failure_info_ = true;
        }

        setWindowTitle(QString("Debug State at t=%1").arg(query_time));
        resize(800, 600); // Adjust as needed

        // Debug print: Check entities
        qDebug() << "Debug Viz: " << entities_.size() << " entities at t=" << query_time;
        for (const auto &[name, ent] : entities_)
        {
            Pose p = timetable_.get_pose(ent, query_time_);
            qDebug() << " - " << QString::fromStdString(name) << ": (" << p.x << ", " << p.y << ", yaw=" << p.yaw << ")";
        }
        qDebug() << " - " << QString::fromStdString("goalPose") << ": (" << goal_pose.x << ", " << goal_pose.y << ", yaw=" << goal_pose.yaw << ")";
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), Qt::white);

        std::vector<QString> causing_movers;

        // Compute scale (fit workspace to window)
        double w = params_.max_x - params_.min_x;
        double h = params_.max_y - params_.min_y;
        double scale_x = (width() - 100) / w; // Add margins
        double scale_y = (height() - 100) / h;
        double scale = std::min(scale_x, scale_y);
        qDebug() << "Scale:" << scale << "Workspace:" << w << "x" << h;

        // Save original state
        painter.save();

        // Apply transform for world coords
        painter.translate(50, height() - 50); // Bottom-left origin with margin
        painter.scale(scale, -scale);         // Flip Y, scale

        // Draw bounds (with fixed device pen)
        painter.restore();                  // Draw boundary in device coords for visibility
        painter.setPen(QPen(Qt::black, 2)); // Fixed 2px width
        double dev_min_x = 50;
        double dev_min_y = 50;
        double dev_w = w * scale;
        double dev_h = h * scale;
        painter.drawRect(QRectF(dev_min_x, dev_min_y, dev_w, dev_h));
        painter.save(); // Re-apply transform for world drawing
        painter.translate(50, height() - 50);
        painter.scale(scale, -scale);

        // Pre-compute poses and corners for all entities
        std::unordered_map<std::string, Pose> poses;
        std::unordered_map<std::string, Corners> all_corners;
        for (const auto &[name, ent] : entities_)
        {
            Pose p = timetable_.get_pose(ent, query_time_);
            poses[name] = p;
            Corners corners = get_corners(p.x, p.y, p.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            all_corners[name] = corners;
        }

        // Detect collisions: Check pairwise intersections
        std::vector<std::pair<std::string, std::string>> collisions;
        auto entity_names = std::vector<std::string>{};
        for (const auto &[name, ent] : entities_)
            entity_names.push_back(name);
        for (size_t i = 0; i < entity_names.size(); ++i)
        {
            for (size_t j = i + 1; j < entity_names.size(); ++j)
            {
                const auto &name1 = entity_names[i];
                const auto &name2 = entity_names[j];
                if (rectangles_intersect(all_corners.at(name1), all_corners.at(name2)))
                {
                    collisions.emplace_back(name1, name2);
                    qDebug() << "Collision detected between" << QString::fromStdString(name1) << "and" << QString::fromStdString(name2);
                }
            }
        }

        // Draw entities (scaled)
        for (const auto &[name, ent] : entities_)
        {
            QColor color = (ent->type == EntityType::ROBOT) ? Qt::blue : Qt::green;
            painter.setBrush(color);
            painter.setPen(QPen(Qt::black, 0.02)); // Fixed small world-space width (adjust if needed)

            // Get corners and draw polygon
            const Corners &corners = all_corners.at(name);
            QPolygonF poly;
            for (const auto &c : corners)
            {
                poly << QPointF(c.x, c.y);
            }
            painter.drawPolygon(poly);
        }

        // Highlight collisions (red overlay or outline)
        painter.setPen(QPen(Qt::red, 0.05, Qt::SolidLine));
        painter.setBrush(QBrush(QColor(255, 0, 0, 50))); // Semi-transparent red
        for (const auto &[name1, name2] : collisions)
        {
            // Draw outlines around colliding pairs
            QPolygonF poly1, poly2;
            for (const auto &c : all_corners.at(name1))
                poly1 << QPointF(c.x, c.y);
            for (const auto &c : all_corners.at(name2))
                poly2 << QPointF(c.x, c.y);
            painter.drawPolygon(poly1);
            painter.drawPolygon(poly2);
            // Optional: Draw intersection area if needed (advanced: compute clip, but skip for simplicity)
        }

        // Highlight reported blocking entity (at failure time) if provided
        if (has_failure_info_ && !failure_info_.entity_name.empty() &&
            failure_info_.entity_name != "Boundary")
        {
            auto it = entities_.find(failure_info_.entity_name);
            if (it != entities_.end() && it->second)
            {
                Pose blocker_pose = timetable_.get_pose(it->second, failure_info_.time);
                Corners blocker_corners = get_corners(
                    blocker_pose.x, blocker_pose.y, blocker_pose.yaw,
                    it->second->size.front_length,
                    it->second->size.rear_length,
                    it->second->size.width);
                QPolygonF blocker_poly;
                for (const auto &c : blocker_corners)
                    blocker_poly << QPointF(c.x, c.y);
                painter.setPen(QPen(QColor(200, 0, 0), 0.08, Qt::SolidLine));
                painter.setBrush(QBrush(QColor(255, 0, 0, 40)));
                painter.drawPolygon(blocker_poly);
            }
        }

        // Draw start/goal (scaled)
        painter.setPen(QPen(Qt::black, 0.02));
        if (start_pose_.x != 0 || start_pose_.y != 0)
        {
            painter.setBrush(Qt::yellow);
            painter.drawEllipse(QPointF(start_pose_.x, start_pose_.y), 0.1, 0.1);
        }
        if (goal_pose_.x != 0 || goal_pose_.y != 0)
        {
            painter.setBrush(Qt::red);
            painter.drawEllipse(QPointF(goal_pose_.x, goal_pose_.y), 0.1, 0.1);
            if (start_pose_.x != 0 || start_pose_.y != 0)
            {
                painter.setPen(QPen(Qt::red, 0.05, Qt::DashLine));
                painter.drawLine(QPointF(start_pose_.x, start_pose_.y), QPointF(goal_pose_.x, goal_pose_.y));
            }
        }

        // Draw attempted trajectory and mark failure point along it
        bool has_failure_state_pose = false;
        Pose failure_robot_pose;
        Pose failure_object_pose;

        if (has_attempted_traj_ && attempted_traj_.waypoints.size() >= 2)
        {
            QPolygonF attempt_poly;
            for (const auto &wp : attempted_traj_.waypoints)
            {
                attempt_poly << QPointF(wp.x, wp.y);
            }
            painter.setPen(QPen(QColor(150, 0, 200), 0.05, Qt::SolidLine));
            painter.setBrush(Qt::NoBrush);
            painter.drawPolyline(attempt_poly);

            if (has_failure_info_)
            {
                double first_t = attempted_traj_.waypoints.front().time;
                double last_t = attempted_traj_.waypoints.back().time;

                double rel_t = failure_info_.time - attempted_start_time_;
                if (rel_t < first_t - 1e-6 || rel_t > last_t + 1e-6)
                {
                    rel_t = failure_info_.time;
                }
                rel_t = std::max(first_t, std::min(last_t, rel_t));

                auto [fx, fy, fyaw] = interpolate_timed_path(attempted_traj_.waypoints, rel_t);
                failure_robot_pose = Pose{fx, fy, fyaw};
                has_failure_state_pose = true;

                if (attempted_traj_.is_transfer && attempted_traj_.entity && attempted_traj_.transferred_object)
                {
                    failure_object_pose = TimeTable::compute_object_pose(
                        failure_robot_pose,
                        attempted_traj_.entity->size,
                        attempted_traj_.transferred_object->size);
                }

                painter.setPen(QPen(QColor(220, 0, 0), 0.08, Qt::SolidLine));
                painter.setBrush(Qt::NoBrush);
                painter.drawEllipse(QPointF(fx, fy), 0.12, 0.12);
                painter.setPen(QPen(QColor(220, 0, 0), 0.06, Qt::DashLine));
                painter.drawLine(QPointF(fx - 0.15, fy - 0.15), QPointF(fx + 0.15, fy + 0.15));
                painter.drawLine(QPointF(fx - 0.15, fy + 0.15), QPointF(fx + 0.15, fy - 0.15));
            }
        }

        // Draw attempted robot/object state exactly at failure time.
        if (has_failure_state_pose && attempted_traj_.entity)
        {
            bool robot_caused_failure = false;
            bool object_caused_failure = false;

            EntityMeta *blocking_entity_ptr = nullptr;
            Pose blocking_pose;
            bool has_blocking_pose = false;
            if (has_failure_info_ && !failure_info_.entity_name.empty() &&
                failure_info_.entity_name != "Boundary")
            {
                auto it = entities_.find(failure_info_.entity_name);
                if (it != entities_.end() && it->second)
                {
                    blocking_entity_ptr = it->second;
                    blocking_pose = timetable_.get_pose(blocking_entity_ptr, failure_info_.time);
                    has_blocking_pose = true;
                }
            }

            Corners rc = get_corners(
                failure_robot_pose.x,
                failure_robot_pose.y,
                failure_robot_pose.yaw,
                attempted_traj_.entity->size.front_length,
                attempted_traj_.entity->size.rear_length,
                attempted_traj_.entity->size.width);

            if (has_failure_info_)
            {
                if (failure_info_.entity_name == "Boundary" ||
                    failure_info_.reason.find("Boundary") != std::string::npos)
                {
                    CollisionGeometry robot_bounds_geom = setup_collision_geometry(
                        failure_robot_pose, attempted_traj_.entity->size, 1.0);
                    robot_caused_failure = check_robot_bounds_collision(
                        failure_robot_pose, robot_bounds_geom.corners, params_);
                }
                else if (has_blocking_pose && blocking_entity_ptr)
                {
                    Corners blocker_corners = get_corners(
                        blocking_pose.x,
                        blocking_pose.y,
                        blocking_pose.yaw,
                        blocking_entity_ptr->size.front_length,
                        blocking_entity_ptr->size.rear_length,
                        blocking_entity_ptr->size.width);
                    robot_caused_failure = rectangles_intersect(rc, blocker_corners);
                }
                else
                {
                    robot_caused_failure = true;
                }
            }

            QPolygonF robot_poly;
            for (const auto &c : rc)
                robot_poly << QPointF(c.x, c.y);
            painter.setPen(QPen(robot_caused_failure ? QColor(220, 0, 0) : QColor(0, 160, 240),
                                robot_caused_failure ? 0.11 : 0.08,
                                Qt::SolidLine));
            painter.setBrush(QBrush(QColor(0, 160, 240, 70)));
            painter.drawPolygon(robot_poly);

            // Heading cue for the attempted robot state.
            QPointF robot_center(failure_robot_pose.x, failure_robot_pose.y);
            QPointF robot_front(
                failure_robot_pose.x + 0.25 * std::cos(failure_robot_pose.yaw),
                failure_robot_pose.y + 0.25 * std::sin(failure_robot_pose.yaw));
            painter.setPen(QPen(QColor(0, 90, 180), 0.06, Qt::SolidLine));
            painter.drawLine(robot_center, robot_front);

            if (attempted_traj_.is_transfer && attempted_traj_.transferred_object)
            {
                Corners oc = get_corners(
                    failure_object_pose.x,
                    failure_object_pose.y,
                    failure_object_pose.yaw,
                    attempted_traj_.transferred_object->size.front_length,
                    attempted_traj_.transferred_object->size.rear_length,
                    attempted_traj_.transferred_object->size.width);

                if (has_failure_info_)
                {
                    if (failure_info_.entity_name == "Boundary" ||
                        failure_info_.reason.find("Boundary") != std::string::npos)
                    {
                        CollisionGeometry object_bounds_geom = setup_collision_geometry(
                            failure_object_pose, attempted_traj_.transferred_object->size, 1.0);
                        object_caused_failure = check_object_bounds_collision(
                            failure_object_pose,
                            object_bounds_geom.corners,
                            params_);
                    }
                    else if (has_blocking_pose && blocking_entity_ptr)
                    {
                        Corners blocker_corners = get_corners(
                            blocking_pose.x,
                            blocking_pose.y,
                            blocking_pose.yaw,
                            blocking_entity_ptr->size.front_length,
                            blocking_entity_ptr->size.rear_length,
                            blocking_entity_ptr->size.width);
                        object_caused_failure = rectangles_intersect(oc, blocker_corners);
                    }
                }

                QPolygonF object_poly;
                for (const auto &c : oc)
                    object_poly << QPointF(c.x, c.y);
                painter.setPen(QPen(object_caused_failure ? QColor(220, 0, 0) : QColor(240, 140, 0),
                                    object_caused_failure ? 0.11 : 0.08,
                                    Qt::SolidLine));
                painter.setBrush(QBrush(QColor(255, 180, 0, 75)));
                painter.drawPolygon(object_poly);
            }

            if (robot_caused_failure)
            {
                causing_movers.push_back(
                    QString::fromStdString(attempted_traj_.entity->name + " (robot)"));
            }
            if (object_caused_failure && attempted_traj_.transferred_object)
            {
                causing_movers.push_back(
                    QString::fromStdString(attempted_traj_.transferred_object->name + " (object)"));
            }
            if (causing_movers.empty() && has_failure_info_)
            {
                causing_movers.push_back(
                    QString::fromStdString(attempted_traj_.entity->name + " (robot)"));
            }
        }

        // Future overlays (scaled, semi-transparent)
        painter.setOpacity(0.3);
        for (double dt = 5.0; dt <= 15.0; dt += 5.0)
        {
            for (const auto &[name, ent] : entities_)
            {
                if (ent->type != EntityType::ROBOT)
                    continue;
                Pose p_future = timetable_.get_pose(ent, query_time_ + dt);
                painter.setBrush(Qt::gray);
                painter.setPen(QPen(Qt::black, 0.02));
                Corners corners = get_corners(p_future.x, p_future.y, p_future.yaw,
                                              ent->size.front_length, ent->size.rear_length, ent->size.width);
                QPolygonF poly;
                for (const auto &c : corners)
                    poly << QPointF(c.x, c.y);
                painter.drawPolygon(poly);
            }
        }
        painter.setOpacity(1.0);

        // Restore for unscaled drawing (e.g., labels)
        painter.restore();

        // Draw labels in device coordinates (unscaled)
        QFont font = painter.font();
        font.setPointSize(10); // Fixed small size
        painter.setFont(font);
        painter.setPen(Qt::black);
        for (const auto &[name, ent] : entities_)
        {
            Pose p = timetable_.get_pose(ent, query_time_);
            // World to device
            double dev_x = 50 + (p.x - params_.min_x) * scale;
            double dev_y = 50 + (params_.max_y - p.y) * scale;                          // Adjust for flipped Y (since bottom is max_y after flip?)
            painter.drawText(QPointF(dev_x, dev_y + 10), QString::fromStdString(name)); // Pixel offset
        }

        // Draw failure explanation banner
        if (has_failure_info_ || !context_text_.empty())
        {
            QRect info_rect(8, 8, width() - 16, 84);
            painter.fillRect(info_rect, QColor(255, 245, 245, 225));
            painter.setPen(QPen(QColor(90, 0, 0), 1));
            painter.drawRect(info_rect);

            painter.setPen(QPen(QColor(120, 0, 0), 1));
            QFont info_font = painter.font();
            info_font.setPointSize(10);
            info_font.setBold(true);
            painter.setFont(info_font);
            painter.drawText(info_rect.adjusted(8, 6, -8, -6), Qt::AlignTop | Qt::TextWordWrap,
                             "Failure Diagnostics");

            info_font.setPointSize(9);
            info_font.setBold(false);
            painter.setFont(info_font);

            QString details;
            if (has_failure_info_)
            {
                details += QString("Reason: %1")
                               .arg(QString::fromStdString(failure_info_.reason));
                if (!failure_info_.entity_name.empty())
                {
                    details += QString(", Entity: %1")
                                   .arg(QString::fromStdString(failure_info_.entity_name));
                }
                details += QString(", Time: %1 s")
                               .arg(failure_info_.time, 0, 'f', 2);
            }
            if (!context_text_.empty())
            {
                if (!details.isEmpty())
                    details += "\n";
                details += QString::fromStdString(context_text_);
            }

            if (has_failure_state_pose)
            {
                QString pose_line = QString("Attempt robot@fail: (%1, %2, %3)")
                                        .arg(failure_robot_pose.x, 0, 'f', 2)
                                        .arg(failure_robot_pose.y, 0, 'f', 2)
                                        .arg(failure_robot_pose.yaw, 0, 'f', 2);
                if (!details.isEmpty())
                    details += "\n";
                details += pose_line;

                if (attempted_traj_.is_transfer && attempted_traj_.transferred_object)
                {
                    details += QString("\nAttempt object@fail: (%1, %2, %3)")
                                   .arg(failure_object_pose.x, 0, 'f', 2)
                                   .arg(failure_object_pose.y, 0, 'f', 2)
                                   .arg(failure_object_pose.yaw, 0, 'f', 2);
                }
            }

            if (!causing_movers.empty())
            {
                QString cause_line = "Causing mover(s): ";
                for (size_t i = 0; i < causing_movers.size(); ++i)
                {
                    if (i > 0)
                        cause_line += ", ";
                    cause_line += causing_movers[i];
                }
                if (!details.isEmpty())
                    details += "\n";
                details += cause_line;
            }

            painter.drawText(info_rect.adjusted(8, 24, -8, -6),
                             Qt::AlignTop | Qt::TextWordWrap,
                             details);
        }
    }

private:
    const TimeTable &timetable_;
    const std::unordered_map<std::string, EntityMeta *> &entities_;
    const Params &params_;
    double query_time_;
    Pose start_pose_;
    Pose goal_pose_;
    bool has_attempted_traj_ = false;
    Trajectory attempted_traj_;
    double attempted_start_time_ = 0.0;
    bool has_failure_info_ = false;
    CollisionInfo failure_info_;
    std::string context_text_;
};

// The visualize_current_state function remains the same
void visualize_current_state(const TimeTable &timetable, const std::unordered_map<std::string, EntityMeta *> &entities,
                             const Params &params, double query_time,
                             const Pose &start_pose = {}, const Pose &goal_pose = {},
                             const Trajectory *attempted_traj = nullptr,
                             double attempted_start_time = 0.0,
                             const CollisionInfo *failure_info = nullptr,
                             const std::string &context_text = "")
{
    QApplication *app = qobject_cast<QApplication *>(QCoreApplication::instance());
    bool own_app = false;
    if (!app)
    {
        static int local_argc = 1;
        static char *local_argv[] = {const_cast<char *>("debug_viz")};
        app = new QApplication(local_argc, local_argv);
        own_app = true;
    }

    DebugVisualizer viz(timetable, entities, params, query_time,
                        start_pose, goal_pose,
                        attempted_traj, attempted_start_time,
                        failure_info, context_text);
    viz.exec(); // Blocks until closed

    if (own_app)
    {
        delete app;
    }
}

class SearchTreeViz : public QDialog
{
public:
    SearchTreeViz(const std::vector<Node> &nodes, const Params &params, QWidget *parent = nullptr)
        : QDialog(parent), nodes_(nodes), params_(params)
    {
        setWindowTitle("PHA* Search Tree Debug");
        resize(800, 600);

        // Debug log
        qDebug() << "Search Tree Viz: " << nodes.size() << " nodes";
    }

protected:
    void paintEvent(QPaintEvent *event) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Compute scale (same as DebugVisualizer)
        double w = params_.max_x - params_.min_x;
        double h = params_.max_y - params_.min_y;
        double scale_x = (width() - 100) / w;
        double scale_y = (height() - 100) / h;
        double scale = std::min(scale_x, scale_y);

        painter.translate(50, height() - 50); // Bottom-left origin
        painter.scale(scale, -scale);         // Flip Y

        // Draw bounds
        painter.setPen(QPen(Qt::black, 2.0 / scale)); // Visible width
        painter.drawRect(QRectF(params_.min_x, params_.min_y, w, h));

        // Draw tree: edges first (lines to parent), then nodes (dots)
        painter.setPen(QPen(Qt::black, 0.01)); // Thin lines
        for (const auto &node : nodes_)
        {
            if (node.parent)
            {
                painter.drawLine(QPointF(node.x, node.y), QPointF(node.parent->x, node.parent->y));
            }
        }

        // Nodes as small cyan dots
        painter.setBrush(Qt::cyan);
        painter.setPen(Qt::NoPen); // No outline for dots
        for (const auto &node : nodes_)
        {
            painter.drawEllipse(QPointF(node.x, node.y), 0.05, 0.05); // Small radius
        }
    }

private:
    const std::vector<Node> &nodes_;
    const Params &params_;
};

void visualize_search_tree(const std::vector<Node> &nodes, const Params &params)
{
    QApplication *app = qobject_cast<QApplication *>(QCoreApplication::instance());
    bool own_app = false;
    if (!app)
    {
        static int local_argc = 1;
        static char *local_argv[] = {const_cast<char *>("search_viz")};
        app = new QApplication(local_argc, local_argv);
        own_app = true;
    }

    SearchTreeViz viz(nodes, params);
    viz.exec(); // Blocks until closed

    if (own_app)
    {
        delete app;
    }
}

// ==========================================
// PLANNING DEBUG VISUALIZATION (Visualization.h)
// ==========================================

inline QString planning_status_to_qstring(PlanningStatus status)
{
    switch (status)
    {
    case PlanningStatus::SUCCESS:
        return "SUCCESS";
    case PlanningStatus::START_INVALID_COLLISION:
        return "START_INVALID_COLLISION";
    case PlanningStatus::START_OUT_OF_BOUNDS:
        return "START_OUT_OF_BOUNDS";
    case PlanningStatus::GOAL_INVALID_COLLISION:
        return "GOAL_INVALID_COLLISION";
    case PlanningStatus::GOAL_OUT_OF_BOUNDS:
        return "GOAL_OUT_OF_BOUNDS";
    case PlanningStatus::NO_PATH_FOUND:
        return "NO_PATH_FOUND";
    case PlanningStatus::TIMEOUT_EXCEEDED:
        return "TIMEOUT_EXCEEDED";
    case PlanningStatus::HIGH_COST_UNFEASIBLE:
        return "HIGH_COST_UNFEASIBLE";
    case PlanningStatus::BLOCKED_BY_ROBOT:
        return "BLOCKED_BY_ROBOT";
    case PlanningStatus::INTERNAL_ERROR:
    default:
        return "INTERNAL_ERROR";
    }
}

inline const std::map<double, Pose> *find_entity_timeline(
    const TimeTable &tt, EntityMeta *entity)
{
    if (!entity)
        return nullptr;

    const auto &database = tt.get_database();
    auto it = database.find(entity);
    if (it == database.end())
        return nullptr;

    return &it->second;
}

// Helper to get corners for drawing (Local calculation)
inline std::vector<GeometryPoint> get_corners_local(double x, double y, double yaw, double fl, double rl, double w)
{
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);
    double dx_fl = fl * cos_y;
    double dy_fl = fl * sin_y;
    double dx_rl = rl * cos_y;
    double dy_rl = rl * sin_y;
    double dx_w = (w / 2.0) * sin_y;
    double dy_w = (w / 2.0) * cos_y;

    return {
        {x + dx_fl - dx_w, y + dy_fl + dy_w}, // Front Left
        {x + dx_fl + dx_w, y + dy_fl - dy_w}, // Front Right
        {x - dx_rl + dx_w, y - dy_rl - dy_w}, // Rear Right
        {x - dx_rl - dx_w, y - dy_rl + dy_w}  // Rear Left
    };
}

class PlanningDebugWidget : public QDialog
{
public:
    PlanningDebugWidget(const TimeTable &tt,
                        RobotMeta *robot,
                        const PlanningResult &result,
                        double start_time,
                        const Pose &start,
                        const Pose &goal,
                        const std::string &attempt_trace,
                        const std::string &plan_kind,
                        const Params &p,
                        QWidget *parent = nullptr)
        : QDialog(parent), timetable(tt),
          active_robot(robot), plan(result),
          plan_start_time(start_time),
          start_pose(start), goal_pose(goal),
          attempt_trace_text(attempt_trace),
          plan_kind_text(plan_kind),
          params(p)
    {
        plan_duration = 0.0;
        if (!plan.waypoints.empty())
        {
            plan_duration = plan.waypoints.back().time;
        }
        else
        {
            // Default window if plan is empty/failed
            plan_duration = 10.0;
        }

        QString kind_suffix;
        if (!plan_kind_text.empty())
        {
            kind_suffix = QString(" [%1]").arg(QString::fromStdString(plan_kind_text));
        }

        setWindowTitle(QString("Debug: %1%2 (t=%3 to %4)")
                           .arg(QString::fromStdString(robot->name))
                           .arg(kind_suffix)
                           .arg(start_time, 0, 'f', 2)
                           .arg(start_time + plan_duration, 0, 'f', 2));

        initializeContextSliderIfNeeded();

        // Resize to accommodate side panel
        resize(1200, 800);
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QDialog::resizeEvent(event);
        layoutContextControls();
    }

    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), Qt::white);

        // --- Layout Configuration ---
        int legend_width = 260;
        int view_width = width() - legend_width;
        int view_height = height() - contextControlsHeight();

        // --- 1. Transform Setup ---
        double margin = 1.0;
        double w_real = (params.max_x - params.min_x) + 2 * margin;
        double h_real = (params.max_y - params.min_y) + 2 * margin;

        // Scale to fit ONLY in the view area
        double scale = std::min((double)view_width / w_real, (double)view_height / h_real);

        auto toScreen = [&](double x, double y)
        {
            return QPointF((x - params.min_x + margin) * scale,
                           (params.max_y - y + margin) * scale); // Flip Y
        };

        // --- 2. Draw Workspace Boundary ---
        p.setPen(QPen(Qt::black, 3, Qt::SolidLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(toScreen(params.min_x, params.max_y), toScreen(params.max_x, params.min_y)));

        // Vertical separator line
        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(view_width, 0, view_width, view_height);

        // --- 3. Draw Entities (Context) ---
        const double context_time = currentContextTime();
        const double trail_start = contextTrailStartTime();
        const double trail_end = contextTrailEndTime();
        for (const auto &[entity, timeline] : timetable.get_database())
        {
            if (entity == active_robot)
                continue;

            Pose current_pose = timetable.get_pose(entity, context_time);

            QColor bodyColor;
            if (entity->type == EntityType::OBJECT)
            {
                bodyColor = QColor(255, 140, 0, 150); // Orange for Objects
            }
            else
            {
                bodyColor = QColor(100, 100, 100, 150); // Dark Grey for Robots
            }

            drawShape(p, current_pose, entity, bodyColor, QPen(Qt::black, 1), toScreen, QString::fromStdString(entity->name));

            drawTrail(p, timeline, trail_start, trail_end, toScreen);
        }

        if (const auto *active_timeline =
                find_entity_timeline(timetable, active_robot))
        {
            drawTrail(p, *active_timeline, trail_start, trail_end, toScreen,
                      QPen(QColor(65, 105, 225, 180), 2, Qt::DashLine));
            Pose active_pose = timetable.get_pose(active_robot, context_time);
            drawShape(p, active_pose, active_robot,
                      QColor(65, 105, 225, 80),
                      QPen(QColor(25, 25, 112), 2),
                      toScreen, "ACTIVE");
        }

        // --- 4. Draw Active Robot Plan ---
        if (!plan.waypoints.empty())
        {
            QPolygonF path;
            for (const auto &wp : plan.waypoints)
            {
                path << toScreen(wp.x, wp.y);
            }
            p.setPen(QPen(QColor(0, 102, 255), 3, Qt::SolidLine));
            p.drawPolyline(path);

            if (enable_context_slider_ &&
                context_time >= plan_start_time - 1e-9 &&
                context_time <= plan_start_time + plan_duration + 1e-9)
            {
                const double rel_t = std::clamp(context_time - plan_start_time, 0.0, plan_duration);
                auto [x_now, y_now, yaw_now] = interpolate_timed_path(plan.waypoints, rel_t);
                drawShape(p, Pose{x_now, y_now, yaw_now}, active_robot,
                          QColor(30, 144, 255, 90), QPen(Qt::black, 2),
                          toScreen, "PLAN");
            }
        }

        // --- 5. Draw Active Start / Goal ---
        drawShape(p, start_pose, active_robot, QColor(50, 205, 50, 180), QPen(Qt::black, 2), toScreen, "START");
        drawShape(p, goal_pose, active_robot, QColor(220, 20, 60, 180), QPen(Qt::black, 2, Qt::DashLine), toScreen, "GOAL");

        // --- 6. Draw Legend (Side Panel) ---
        drawLegend(p, view_width + 10, 20, legend_width - 20, view_height - 40);
    }

private:
    static constexpr int kContextSliderScale = 10;
    static constexpr int kContextControlHeight = 64;

    bool shouldEnableContextSlider() const
    {
        return plan.status != PlanningStatus::SUCCESS &&
               plan_kind_text.find("Initial transit") != std::string::npos;
    }

    void initializeContextSliderIfNeeded()
    {
        enable_context_slider_ = shouldEnableContextSlider();
        if (!enable_context_slider_)
        {
            current_context_time_ = plan_start_time;
            return;
        }

        context_time_pivot_ = (plan.failure_time > 1e-6)
                                  ? plan.failure_time
                                  : plan_start_time;
        context_time_min_ = std::max(0.0, context_time_pivot_ - 30.0);
        context_time_max_ = std::max(context_time_min_ + 0.1, context_time_pivot_ + 30.0);
        current_context_time_ = context_time_pivot_;

        context_time_label_ = new QLabel(this);
        context_time_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        context_time_slider_ = new QSlider(Qt::Horizontal, this);
        context_time_slider_->setRange(
            0,
            static_cast<int>(std::round((context_time_max_ - context_time_min_) *
                                        kContextSliderScale)));
        context_time_slider_->setValue(
            static_cast<int>(std::round((current_context_time_ - context_time_min_) *
                                        kContextSliderScale)));

        QObject::connect(context_time_slider_, &QSlider::valueChanged,
                         this, [this](int value)
                         {
                             current_context_time_ =
                                 context_time_min_ +
                                 static_cast<double>(value) / kContextSliderScale;
                             updateContextTimeLabel();
                             update();
                         });

        updateContextTimeLabel();
        layoutContextControls();
    }

    void layoutContextControls()
    {
        if (!enable_context_slider_ || !context_time_slider_ || !context_time_label_)
            return;

        const int margin = 12;
        const int label_height = 18;
        const int slider_height = 24;
        const int base_y = height() - kContextControlHeight + 8;

        context_time_label_->setGeometry(
            margin, base_y, width() - 2 * margin, label_height);
        context_time_slider_->setGeometry(
            margin, base_y + label_height + 6, width() - 2 * margin, slider_height);
    }

    void updateContextTimeLabel()
    {
        if (!context_time_label_)
            return;

        context_time_label_->setText(
            QString("Context time: %1 s   window=[%2, %3]   pivot=%4")
                .arg(current_context_time_, 0, 'f', 1)
                .arg(context_time_min_, 0, 'f', 1)
                .arg(context_time_max_, 0, 'f', 1)
                .arg(context_time_pivot_, 0, 'f', 1));
    }

    int contextControlsHeight() const
    {
        return enable_context_slider_ ? kContextControlHeight : 0;
    }

    double currentContextTime() const
    {
        return enable_context_slider_ ? current_context_time_ : plan_start_time;
    }

    double contextTrailStartTime() const
    {
        return enable_context_slider_ ? context_time_min_ : plan_start_time;
    }

    double contextTrailEndTime() const
    {
        return enable_context_slider_ ? context_time_max_ : (plan_start_time + plan_duration);
    }

    Pose interpolate_pose(const std::map<double, Pose> &timeline, double t)
    {
        if (timeline.empty())
            return Pose();
        auto it = timeline.lower_bound(t);

        if (it == timeline.end())
            return timeline.rbegin()->second;
        if (it->first == t || it == timeline.begin())
            return it->second;

        auto prev = std::prev(it);
        double t1 = prev->first;
        double t2 = it->first;
        Pose p1 = prev->second;
        Pose p2 = it->second;

        double ratio = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + ratio * (p2.x - p1.x);
        p.y = p1.y + ratio * (p2.y - p1.y);

        double dyaw = p2.yaw - p1.yaw;
        while (dyaw > M_PI)
            dyaw -= 2 * M_PI;
        while (dyaw < -M_PI)
            dyaw += 2 * M_PI;
        p.yaw = p1.yaw + ratio * dyaw;
        return p;
    }

    template <typename Func>
    void drawTrail(QPainter &p, const std::map<double, Pose> &timeline, double t_start, double t_end, Func toScreen,
                   const QPen &pen = QPen(QColor(150, 150, 150), 2, Qt::DashLine))
    {
        QPolygonF path;
        auto it = timeline.lower_bound(t_start);

        if (it != timeline.begin())
        {
            Pose start_p = interpolate_pose(timeline, t_start);
            path << toScreen(start_p.x, start_p.y);
        }

        for (; it != timeline.end() && it->first <= t_end; ++it)
        {
            path << toScreen(it->second.x, it->second.y);
        }

        if (it != timeline.end())
        {
            Pose end_p = interpolate_pose(timeline, t_end);
            path << toScreen(end_p.x, end_p.y);
        }

        if (path.size() > 1)
        {
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(path);
        }
    }

    template <typename Func>
    void drawShape(QPainter &p, Pose pose, EntityMeta *ent, QColor brush, QPen pen, Func toScreen, QString label = "")
    {
        auto c = get_corners_local(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
        QPolygonF poly;
        for (auto &pt : c)
            poly << toScreen(pt.x, pt.y);

        p.setBrush(brush);
        p.setPen(pen);
        p.drawPolygon(poly);

        if (!label.isEmpty())
        {
            p.setPen(Qt::black);
            p.drawText(toScreen(pose.x, pose.y) + QPointF(0, -5), label);
        }

        QPointF center = toScreen(pose.x, pose.y);
        QPointF front = toScreen(pose.x + 0.3 * std::cos(pose.yaw), pose.y + 0.3 * std::sin(pose.yaw));
        p.setPen(Qt::black);
        p.drawLine(center, front);
    }

    void drawLegend(QPainter &p, int x, int y, int w, int available_height)
    {
        p.setBrush(QColor(245, 245, 245));
        p.setPen(Qt::black);
        const int panel_h = std::max(420, available_height);
        p.drawRect(x, y, w, panel_h);

        int cy = y + 25;
        auto item = [&](QString t, QColor c, Qt::PenStyle s, bool line)
        {
            p.setPen(Qt::black);
            p.drawText(x + 40, cy + 5, t);

            if (line)
            {
                p.setPen(QPen(c, 2, s));
                p.drawLine(x + 10, cy, x + 30, cy);
            }
            else
            {
                p.setBrush(c);
                p.setPen(Qt::black);
                p.drawRect(x + 10, cy - 7, 15, 15);
            }
            cy += 25;
        };

        p.setFont(QFont("Arial", 10, QFont::Bold));
        p.drawText(x + 10, y + 18, "Context (t=" + QString::number(currentContextTime(), 'f', 1) + ")");
        p.setFont(QFont("Arial", 9));

        cy += 5;
        item("Plan Path", QColor(0, 102, 255), Qt::SolidLine, true);
        if (enable_context_slider_)
        {
            item("Plan Pose @ Context", QColor(30, 144, 255, 90), Qt::SolidLine, false);
        }
        item("Active Robot Trail", QColor(65, 105, 225), Qt::DashLine, true);
        item("Active Robot Pose", QColor(65, 105, 225, 80), Qt::SolidLine, false);
        item("Start Pose", QColor(50, 205, 50, 180), Qt::SolidLine, false);
        item("Goal Pose", QColor(220, 20, 60, 180), Qt::DashLine, false);
        item("Object", QColor(255, 140, 0, 150), Qt::SolidLine, false);
        item("Other Robot", QColor(100, 100, 100, 150), Qt::SolidLine, false);
        item("Other Trail", QColor(150, 150, 150), Qt::DashLine, true);

        cy += 8;
        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(x + 8, cy, x + w - 8, cy);
        cy += 18;

        p.setPen(Qt::black);
        p.setFont(QFont("Arial", 10, QFont::Bold));
        p.drawText(x + 10, cy, "Planning Outcome");
        cy += 18;
        p.setFont(QFont("Arial", 9));

        if (!plan_kind_text.empty())
        {
            p.drawText(x + 10, cy,
                       "Plan Type: " + QString::fromStdString(plan_kind_text));
            cy += 18;
        }

        p.drawText(x + 10, cy,
                   "Status: " + planning_status_to_qstring(plan.status));
        cy += 18;

        p.drawText(x + 10, cy,
                   QString("Waypoints: %1, Explored: %2")
                       .arg(static_cast<int>(plan.waypoints.size()))
                       .arg(static_cast<int>(plan.explored_nodes.size())));
        cy += 18;

        if (!plan.colliding_entity.empty())
        {
            p.drawText(x + 10, cy,
                       "Blocking entity: " + QString::fromStdString(plan.colliding_entity));
            cy += 18;
        }

        if (plan.failure_time > 1e-6)
        {
            const double rel_t = plan.failure_time - plan_start_time;
            p.drawText(x + 10, cy,
                       QString("Failure time: abs=%1 s, rel=%2 s")
                           .arg(plan.failure_time, 0, 'f', 2)
                           .arg(rel_t, 0, 'f', 2));
            cy += 18;
        }

        if (enable_context_slider_)
        {
            p.drawText(x + 10, cy,
                       QString("Context window: [%1, %2]")
                           .arg(context_time_min_, 0, 'f', 1)
                           .arg(context_time_max_, 0, 'f', 1));
            cy += 18;
        }

        const QString detail_text =
            plan.failure_detail.empty()
                ? "Detail: (none)"
                : ("Detail: " + QString::fromStdString(plan.failure_detail));
        QRect detail_rect(x + 10, cy, w - 20, 100);
        p.drawText(detail_rect, Qt::TextWordWrap | Qt::AlignTop, detail_text);
        cy = detail_rect.bottom() + 20;

        if (!attempt_trace_text.empty())
        {
            p.setFont(QFont("Arial", 9, QFont::Bold));
            p.drawText(x + 10, cy, "Attempt Trace");
            cy += 16;
            p.setFont(QFont("Arial", 8));
            QRect trace_rect(x + 10, cy, w - 20, panel_h - (cy - y) - 12);
            p.drawText(trace_rect, Qt::TextWordWrap | Qt::AlignTop,
                       QString::fromStdString(attempt_trace_text));
        }
    }

    const TimeTable &timetable;
    RobotMeta *active_robot;
    PlanningResult plan;
    double plan_start_time;
    double plan_duration;
    Pose start_pose;
    Pose goal_pose;
    std::string attempt_trace_text;
    std::string plan_kind_text;
    Params params;
    bool enable_context_slider_ = false;
    double current_context_time_ = 0.0;
    double context_time_pivot_ = 0.0;
    double context_time_min_ = 0.0;
    double context_time_max_ = 0.0;
    QLabel *context_time_label_ = nullptr;
    QSlider *context_time_slider_ = nullptr;
};

struct PlanningDebugAttempt
{
    std::string attempt_label;
    PlanningResult result;
};

class PlanningAttemptDebugWidget : public QDialog
{
public:
    PlanningAttemptDebugWidget(const TimeTable &tt,
                               RobotMeta *robot,
                               double start_time,
                               const Pose &start,
                               const Pose &goal,
                               const std::vector<PlanningDebugAttempt> &attempts,
                               const std::string &plan_kind,
                               const Params &p,
                               QWidget *parent = nullptr)
        : QDialog(parent), timetable_(tt), active_robot_(robot),
          plan_start_time_(start_time), start_pose_(start), goal_pose_(goal),
          attempts_(attempts), plan_kind_text_(plan_kind), params_(p)
    {
        if (!attempts_.empty())
        {
            selected_attempt_ = static_cast<int>(attempts_.size()) - 1;
        }

        context_time_label_ = new QLabel(this);
        context_time_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        context_time_slider_ = new QSlider(Qt::Horizontal, this);
        QObject::connect(context_time_slider_, &QSlider::valueChanged,
                         this, [this](int value)
                         {
                             current_context_time_ =
                                 context_time_min_ +
                                 static_cast<double>(value) / kContextSliderScale;
                             updateContextTimeLabel();
                             update();
                         });

        syncContextWindowToSelectedAttempt();
        resize(1340, 860);
        updateWindowTitle();
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QDialog::resizeEvent(event);
        layoutContextControls();
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        if (attempts_.empty())
        {
            QDialog::keyPressEvent(event);
            return;
        }

        if (event->key() == Qt::Key_Up)
        {
            selectAttempt((selected_attempt_ == 0)
                              ? static_cast<int>(attempts_.size()) - 1
                              : selected_attempt_ - 1);
            event->accept();
            return;
        }

        if (event->key() == Qt::Key_Down)
        {
            selectAttempt((selected_attempt_ + 1) %
                          static_cast<int>(attempts_.size()));
            event->accept();
            return;
        }

        if (event->key() == Qt::Key_Home)
        {
            selectAttempt(0);
            event->accept();
            return;
        }

        if (event->key() == Qt::Key_End)
        {
            selectAttempt(static_cast<int>(attempts_.size()) - 1);
            event->accept();
            return;
        }

        QDialog::keyPressEvent(event);
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        for (const auto &entry : attempt_row_rects_)
        {
            if (entry.first.contains(event->pos()))
            {
                selectAttempt(entry.second);
                event->accept();
                return;
            }
        }

        QDialog::mousePressEvent(event);
    }

    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), Qt::white);

        if (attempts_.empty() || !active_robot_)
            return;

        const PlanningDebugAttempt &attempt = currentAttempt();
        const PlanningResult &plan = attempt.result;
        double plan_duration = 0.0;
        if (!plan.waypoints.empty())
        {
            plan_duration = std::max(0.1, plan.waypoints.back().time - plan_start_time_);
        }
        else
        {
            plan_duration = 10.0;
        }

        const int legend_width = 380;
        const int view_width = width() - legend_width;
        const int view_height = height() - kContextControlHeight;

        const double margin = 1.0;
        const double w_real = (params_.max_x - params_.min_x) + 2 * margin;
        const double h_real = (params_.max_y - params_.min_y) + 2 * margin;
        const double scale = std::min((double)view_width / w_real,
                                      (double)view_height / h_real);

        auto toScreen = [&](double x, double y)
        {
            return QPointF((x - params_.min_x + margin) * scale,
                           (params_.max_y - y + margin) * scale);
        };

        p.setPen(QPen(Qt::black, 3, Qt::SolidLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(toScreen(params_.min_x, params_.max_y),
                          toScreen(params_.max_x, params_.min_y)));

        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(view_width, 0, view_width, view_height);

        for (const auto &[entity, timeline] : timetable_.get_database())
        {
            if (entity == active_robot_)
                continue;

            Pose current_pose = timetable_.get_pose(entity, current_context_time_);
            QColor body_color =
                (entity->type == EntityType::OBJECT)
                    ? QColor(255, 140, 0, 150)
                    : QColor(100, 100, 100, 150);

            drawShape(p, current_pose, entity, body_color, QPen(Qt::black, 1),
                      toScreen, QString::fromStdString(entity->name));
            drawTrail(p, timeline, context_time_min_, context_time_max_, toScreen);
        }

        if (const auto *active_timeline =
                find_entity_timeline(timetable_, active_robot_))
        {
            drawTrail(p, *active_timeline, context_time_min_, context_time_max_, toScreen,
                      QPen(QColor(65, 105, 225, 180), 2, Qt::DashLine));
            Pose active_pose = timetable_.get_pose(active_robot_, current_context_time_);
            drawShape(p, active_pose, active_robot_,
                      QColor(65, 105, 225, 80),
                      QPen(QColor(25, 25, 112), 2),
                      toScreen, "ACTIVE");
        }

        if (!plan.waypoints.empty())
        {
            QPolygonF path;
            for (const auto &wp : plan.waypoints)
            {
                path << toScreen(wp.x, wp.y);
            }
            p.setPen(QPen(QColor(0, 102, 255), 3, Qt::SolidLine));
            p.drawPolyline(path);

            if (current_context_time_ >= plan_start_time_ - 1e-9 &&
                current_context_time_ <= plan_start_time_ + plan_duration + 1e-9)
            {
                const double rel_t =
                    std::clamp(current_context_time_ - plan_start_time_, 0.0, plan_duration);
                auto [x_now, y_now, yaw_now] =
                    interpolate_timed_path(plan.waypoints, rel_t);
                drawShape(p, Pose{x_now, y_now, yaw_now}, active_robot_,
                          QColor(30, 144, 255, 90), QPen(Qt::black, 2),
                          toScreen, "TRY");
            }
        }

        drawShape(p, start_pose_, active_robot_, QColor(50, 205, 50, 180),
                  QPen(Qt::black, 2), toScreen, "START");
        drawShape(p, goal_pose_, active_robot_, QColor(220, 20, 60, 180),
                  QPen(Qt::black, 2, Qt::DashLine), toScreen, "GOAL");

        drawLegend(p, view_width + 10, 20, legend_width - 20, view_height - 40);
    }

private:
    static constexpr int kContextSliderScale = 10;
    static constexpr int kContextControlHeight = 64;

    const PlanningDebugAttempt &currentAttempt() const
    {
        return attempts_[selected_attempt_];
    }

    void selectAttempt(int index)
    {
        if (index < 0 || index >= static_cast<int>(attempts_.size()) ||
            index == selected_attempt_)
        {
            return;
        }

        selected_attempt_ = index;
        syncContextWindowToSelectedAttempt();
        updateWindowTitle();
        update();
    }

    void syncContextWindowToSelectedAttempt()
    {
        if (attempts_.empty())
            return;

        const PlanningResult &plan = currentAttempt().result;
        context_time_pivot_ = (plan.failure_time > 1e-6)
                                  ? plan.failure_time
                                  : plan_start_time_;
        context_time_min_ = std::max(0.0, context_time_pivot_ - 30.0);
        context_time_max_ = std::max(context_time_min_ + 0.1, context_time_pivot_ + 30.0);
        current_context_time_ = context_time_pivot_;

        if (context_time_slider_)
        {
            context_time_slider_->blockSignals(true);
            context_time_slider_->setRange(
                0,
                static_cast<int>(std::round((context_time_max_ - context_time_min_) *
                                            kContextSliderScale)));
            context_time_slider_->setValue(
                static_cast<int>(std::round((current_context_time_ - context_time_min_) *
                                            kContextSliderScale)));
            context_time_slider_->blockSignals(false);
        }

        updateContextTimeLabel();
        layoutContextControls();
    }

    void updateWindowTitle()
    {
        if (attempts_.empty() || !active_robot_)
        {
            setWindowTitle("Transit Attempt Debug");
            return;
        }

        const auto &attempt = currentAttempt();
        QString kind_suffix;
        if (!plan_kind_text_.empty())
        {
            kind_suffix = QString(" [%1]").arg(QString::fromStdString(plan_kind_text_));
        }

        setWindowTitle(
            QString("Debug: %1%2 [%3/%4] %5")
                .arg(QString::fromStdString(active_robot_->name))
                .arg(kind_suffix)
                .arg(selected_attempt_ + 1)
                .arg(static_cast<int>(attempts_.size()))
                .arg(planning_status_to_qstring(attempt.result.status)));
    }

    void layoutContextControls()
    {
        if (!context_time_slider_ || !context_time_label_)
            return;

        const int margin = 12;
        const int label_height = 18;
        const int slider_height = 24;
        const int base_y = height() - kContextControlHeight + 8;

        context_time_label_->setGeometry(
            margin, base_y, width() - 2 * margin, label_height);
        context_time_slider_->setGeometry(
            margin, base_y + label_height + 6, width() - 2 * margin, slider_height);
    }

    void updateContextTimeLabel()
    {
        if (!context_time_label_)
            return;

        context_time_label_->setText(
            QString("Context time: %1 s   window=[%2, %3]   pivot=%4")
                .arg(current_context_time_, 0, 'f', 1)
                .arg(context_time_min_, 0, 'f', 1)
                .arg(context_time_max_, 0, 'f', 1)
                .arg(context_time_pivot_, 0, 'f', 1));
    }

    Pose interpolatePose(const std::map<double, Pose> &timeline, double t) const
    {
        if (timeline.empty())
            return Pose();

        auto it = timeline.lower_bound(t);
        if (it == timeline.end())
            return timeline.rbegin()->second;
        if (it->first == t || it == timeline.begin())
            return it->second;

        auto prev = std::prev(it);
        double t1 = prev->first;
        double t2 = it->first;
        Pose p1 = prev->second;
        Pose p2 = it->second;

        double ratio = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + ratio * (p2.x - p1.x);
        p.y = p1.y + ratio * (p2.y - p1.y);

        double dyaw = p2.yaw - p1.yaw;
        while (dyaw > M_PI)
            dyaw -= 2 * M_PI;
        while (dyaw < -M_PI)
            dyaw += 2 * M_PI;
        p.yaw = p1.yaw + ratio * dyaw;
        return p;
    }

    template <typename Func>
    void drawTrail(QPainter &p, const std::map<double, Pose> &timeline,
                   double t_start, double t_end, Func toScreen,
                   const QPen &pen = QPen(QColor(150, 150, 150), 2, Qt::DashLine)) const
    {
        QPolygonF path;
        auto it = timeline.lower_bound(t_start);

        if (it != timeline.begin())
        {
            Pose start_p = interpolatePose(timeline, t_start);
            path << toScreen(start_p.x, start_p.y);
        }

        for (; it != timeline.end() && it->first <= t_end; ++it)
        {
            path << toScreen(it->second.x, it->second.y);
        }

        if (it != timeline.end())
        {
            Pose end_p = interpolatePose(timeline, t_end);
            path << toScreen(end_p.x, end_p.y);
        }

        if (path.size() > 1)
        {
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(path);
        }
    }

    template <typename Func>
    void drawShape(QPainter &p, Pose pose, EntityMeta *ent, QColor brush,
                   QPen pen, Func toScreen, QString label = "") const
    {
        auto c = get_corners_local(pose.x, pose.y, pose.yaw,
                                   ent->size.front_length, ent->size.rear_length,
                                   ent->size.width);
        QPolygonF poly;
        for (auto &pt : c)
            poly << toScreen(pt.x, pt.y);

        p.setBrush(brush);
        p.setPen(pen);
        p.drawPolygon(poly);

        if (!label.isEmpty())
        {
            p.setPen(Qt::black);
            p.drawText(toScreen(pose.x, pose.y) + QPointF(0, -5), label);
        }

        QPointF center = toScreen(pose.x, pose.y);
        QPointF front = toScreen(pose.x + 0.3 * std::cos(pose.yaw),
                                 pose.y + 0.3 * std::sin(pose.yaw));
        p.setPen(Qt::black);
        p.drawLine(center, front);
    }

    void drawLegend(QPainter &p, int x, int y, int w, int available_height) const
    {
        attempt_row_rects_.clear();
        const auto &attempt = currentAttempt();
        const PlanningResult &plan = attempt.result;

        p.setBrush(QColor(245, 245, 245));
        p.setPen(Qt::black);
        const int panel_h = std::max(540, available_height);
        p.drawRect(x, y, w, panel_h);

        int cy = y + 22;
        p.setFont(QFont("Arial", 10, QFont::Bold));
        p.drawText(x + 10, cy, "Initial Transit Attempts");
        cy += 18;
        p.setFont(QFont("Arial", 8));
        p.drawText(x + 10, cy, "Use Up/Down or click a row");
        cy += 18;

        p.setFont(QFont("Arial", 9));
        p.drawText(x + 10, cy,
                   QString("Attempt %1 / %2")
                       .arg(selected_attempt_ + 1)
                       .arg(static_cast<int>(attempts_.size())));
        cy += 18;

        if (!plan_kind_text_.empty())
        {
            p.drawText(x + 10, cy,
                       "Plan Type: " + QString::fromStdString(plan_kind_text_));
            cy += 18;
        }

        p.drawText(x + 10, cy,
                   "Stage: " + QString::fromStdString(attempt.attempt_label));
        cy += 18;
        p.drawText(x + 10, cy,
                   "Status: " + planning_status_to_qstring(plan.status));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Waypoints: %1, Explored: %2")
                       .arg(static_cast<int>(plan.waypoints.size()))
                       .arg(static_cast<int>(plan.explored_nodes.size())));
        cy += 18;

        if (!plan.colliding_entity.empty())
        {
            p.drawText(x + 10, cy,
                       "Blocking entity: " +
                           QString::fromStdString(plan.colliding_entity));
            cy += 18;
        }

        if (plan.failure_time > 1e-6)
        {
            p.drawText(x + 10, cy,
                       QString("Failure time: abs=%1 s, rel=%2 s")
                           .arg(plan.failure_time, 0, 'f', 2)
                           .arg(plan.failure_time - plan_start_time_, 0, 'f', 2));
            cy += 18;
        }

        p.drawText(x + 10, cy,
                   QString("Context window: [%1, %2]")
                       .arg(context_time_min_, 0, 'f', 1)
                       .arg(context_time_max_, 0, 'f', 1));
        cy += 18;

        QRect detail_rect(x + 10, cy, w - 20, 108);
        QString detail_text = plan.failure_detail.empty()
                                  ? "Detail: (none)"
                                  : ("Detail: " +
                                     QString::fromStdString(plan.failure_detail));
        p.drawText(detail_rect, Qt::TextWordWrap | Qt::AlignTop, detail_text);
        cy = detail_rect.bottom() + 18;

        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(x + 8, cy, x + w - 8, cy);
        cy += 16;

        p.setPen(Qt::black);
        p.setFont(QFont("Arial", 9, QFont::Bold));
        p.drawText(x + 10, cy, "Attempt List");
        cy += 16;
        p.setFont(QFont("Arial", 8));

        const int max_visible = 14;
        int start_idx = std::max(0, selected_attempt_ - max_visible / 2);
        int end_idx = std::min(start_idx + max_visible,
                               static_cast<int>(attempts_.size()));
        if (end_idx - start_idx < max_visible)
        {
            start_idx = std::max(0, end_idx - max_visible);
        }

        for (int i = start_idx; i < end_idx; ++i)
        {
            const auto &entry = attempts_[i];
            QRect row_rect(x + 10, cy - 12, w - 20, 18);
            attempt_row_rects_.push_back({row_rect, i});
            if (i == selected_attempt_)
            {
                p.fillRect(row_rect, QColor(220, 235, 255));
            }

            QString row_text =
                QString("%1%2 [%3] %4")
                    .arg(i == selected_attempt_ ? "> " : "  ")
                    .arg(i + 1, 3)
                    .arg(QString::fromStdString(entry.attempt_label))
                    .arg(planning_status_to_qstring(entry.result.status));
            p.setPen(Qt::black);
            p.drawText(x + 12, cy, row_text);
            cy += 18;
        }
    }

    const TimeTable &timetable_;
    RobotMeta *active_robot_ = nullptr;
    double plan_start_time_ = 0.0;
    Pose start_pose_;
    Pose goal_pose_;
    std::vector<PlanningDebugAttempt> attempts_;
    std::string plan_kind_text_;
    int selected_attempt_ = 0;
    Params params_;
    double current_context_time_ = 0.0;
    double context_time_pivot_ = 0.0;
    double context_time_min_ = 0.0;
    double context_time_max_ = 0.0;
    QLabel *context_time_label_ = nullptr;
    QSlider *context_time_slider_ = nullptr;
    mutable std::vector<std::pair<QRect, int>> attempt_row_rects_;
};

struct SafeParkingDebugTrial
{
    int trial_index = 0;
    std::string candidate_mode;
    Pose candidate_pose;
    PlanningResult result;
    std::vector<Waypoint> blocked_path_hint_waypoints;
    TimeTable context_timetable;
};

class SafeParkingDebugWidget : public QDialog
{
public:
    SafeParkingDebugWidget(const TimeTable &tt,
                           RobotMeta *robot,
                           double start_time,
                           const Pose &start,
                           const std::vector<SafeParkingDebugTrial> &trials,
                           const Params &p,
                           QWidget *parent = nullptr)
        : QDialog(parent), timetable_(tt), active_robot_(robot),
          plan_start_time_(start_time), start_pose_(start),
          trials_(trials), params_(p)
    {
        context_time_label_ = new QLabel(this);
        context_time_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        context_time_slider_ = new QSlider(Qt::Horizontal, this);
        QObject::connect(context_time_slider_, &QSlider::valueChanged,
                         this, [this](int value)
                         {
                             current_context_time_ =
                                 context_time_min_ +
                                 static_cast<double>(value) / kContextSliderScale;
                             updateContextTimeLabel();
                             update();
                         });

        syncContextWindowToSelectedTrial();
        resize(1360, 900);
        updateWindowTitle();
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QDialog::resizeEvent(event);
        layoutContextControls();
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        if (trials_.empty())
        {
            QDialog::keyPressEvent(event);
            return;
        }

        if (event->key() == Qt::Key_Up)
        {
            selectTrial((selected_trial_ == 0)
                            ? static_cast<int>(trials_.size()) - 1
                            : selected_trial_ - 1);
            event->accept();
            return;
        }

        if (event->key() == Qt::Key_Down)
        {
            selectTrial((selected_trial_ + 1) %
                        static_cast<int>(trials_.size()));
            event->accept();
            return;
        }

        if (event->key() == Qt::Key_Home)
        {
            selectTrial(0);
            event->accept();
            return;
        }

        if (event->key() == Qt::Key_End)
        {
            selectTrial(static_cast<int>(trials_.size()) - 1);
            event->accept();
            return;
        }

        QDialog::keyPressEvent(event);
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        for (const auto &entry : trial_row_rects_)
        {
            if (entry.first.contains(event->pos()))
            {
                selectTrial(entry.second);
                event->accept();
                return;
            }
        }

        QDialog::mousePressEvent(event);
    }

    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), Qt::white);

        if (trials_.empty() || !active_robot_)
            return;

        const SafeParkingDebugTrial &trial = currentTrial();
        const PlanningResult &plan = trial.result;
        const double context_time = current_context_time_;
        const double trail_start = context_time_min_;
        const double trail_end = context_time_max_;

        int legend_width = 380;
        int view_width = width() - legend_width;
        int view_height = height() - kContextControlHeight;

        double margin = 1.0;
        double w_real = (params_.max_x - params_.min_x) + 2 * margin;
        double h_real = (params_.max_y - params_.min_y) + 2 * margin;
        double scale = std::min((double)view_width / w_real,
                                (double)view_height / h_real);

        auto toScreen = [&](double x, double y)
        {
            return QPointF((x - params_.min_x + margin) * scale,
                           (params_.max_y - y + margin) * scale);
        };

        p.setPen(QPen(Qt::black, 3, Qt::SolidLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(toScreen(params_.min_x, params_.max_y),
                          toScreen(params_.max_x, params_.min_y)));

        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(view_width, 0, view_width, height());

        const TimeTable &context_timetable = currentTimeTable();
        for (const auto &[entity, timeline] : context_timetable.get_database())
        {
            if (entity == active_robot_)
                continue;

            Pose current_pose = context_timetable.get_pose(entity, context_time);
            QColor bodyColor =
                (entity->type == EntityType::OBJECT)
                    ? QColor(255, 140, 0, 150)
                    : QColor(100, 100, 100, 150);

            drawShape(p, current_pose, entity, bodyColor, QPen(Qt::black, 1),
                      toScreen, QString::fromStdString(entity->name));
            drawTrail(p, timeline, trail_start, trail_end,
                      toScreen);
        }

        if (const auto *active_timeline =
                find_entity_timeline(context_timetable, active_robot_))
        {
            drawTrail(p, *active_timeline, trail_start, trail_end, toScreen,
                      QPen(QColor(65, 105, 225, 180), 2, Qt::DashLine));
            Pose active_pose = context_timetable.get_pose(active_robot_, context_time);
            drawShape(p, active_pose, active_robot_,
                      QColor(65, 105, 225, 80),
                      QPen(QColor(25, 25, 112), 2),
                      toScreen, "ACTIVE");
        }

        highlightBlockingEntity(p, toScreen, trial.result.colliding_entity, "BLOCK");

        if (!plan.waypoints.empty())
        {
            QPolygonF path;
            for (const auto &wp : plan.waypoints)
            {
                path << toScreen(wp.x, wp.y);
            }
            p.setPen(QPen(QColor(0, 102, 255), 3, Qt::SolidLine));
            p.drawPolyline(path);

            if (context_time >= plan.waypoints.front().time - 1e-9 &&
                context_time <= plan.waypoints.back().time + 1e-9)
            {
                auto [x_now, y_now, yaw_now] =
                    interpolate_timed_path(plan.waypoints, context_time);
                drawShape(p, Pose{x_now, y_now, yaw_now}, active_robot_,
                          QColor(30, 144, 255, 90), QPen(Qt::black, 2),
                          toScreen, "TRY");
            }
            else if (context_time > plan.waypoints.back().time + 1e-9)
            {
                drawShape(p, trial.candidate_pose, active_robot_,
                          QColor(30, 144, 255, 90), QPen(Qt::black, 2),
                          toScreen, "TRY PARK");
            }
        }

        if (!trial.blocked_path_hint_waypoints.empty())
        {
            QPolygonF blocked_path;
            for (const auto &wp : trial.blocked_path_hint_waypoints)
            {
                blocked_path << toScreen(wp.x, wp.y);
            }
            if (blocked_path.size() > 1)
            {
                p.setPen(QPen(QColor(186, 85, 211), 2, Qt::DashDotLine));
                p.setBrush(Qt::NoBrush);
                p.drawPolyline(blocked_path);
            }
        }

        drawShape(p, start_pose_, active_robot_, QColor(50, 205, 50, 180),
                  QPen(Qt::black, 2), toScreen, "START");
        drawShape(p, trial.candidate_pose, active_robot_,
                  QColor(255, 215, 0, 170), QPen(Qt::black, 2, Qt::DashLine),
                  toScreen, "PARK");

        drawLegend(p, view_width + 10, 20, legend_width - 20, view_height - 40);
    }

private:
    static constexpr int kContextSliderScale = 10;
    static constexpr int kContextControlHeight = 64;

    const TimeTable &currentTimeTable() const
    {
        const auto &trial_tt = currentTrial().context_timetable;
        if (!trial_tt.get_database().empty())
            return trial_tt;
        return timetable_;
    }

    const SafeParkingDebugTrial &currentTrial() const
    {
        return trials_[selected_trial_];
    }

    void selectTrial(int index)
    {
        if (index < 0 || index >= static_cast<int>(trials_.size()) ||
            index == selected_trial_)
        {
            return;
        }

        selected_trial_ = index;
        syncContextWindowToSelectedTrial();
        updateWindowTitle();
        update();
    }

    void syncContextWindowToSelectedTrial()
    {
        if (trials_.empty())
            return;

        const PlanningResult &plan = currentTrial().result;
        if (plan.failure_time > 1e-6)
        {
            context_time_pivot_ = plan.failure_time;
        }
        else if (!plan.waypoints.empty())
        {
            context_time_pivot_ = plan.waypoints.front().time;
        }
        else
        {
            context_time_pivot_ = plan_start_time_;
        }

        context_time_min_ = std::max(0.0, context_time_pivot_ - 30.0);
        context_time_max_ = std::max(context_time_min_ + 0.1,
                                     context_time_pivot_ + 30.0);
        current_context_time_ = context_time_pivot_;

        if (context_time_slider_)
        {
            context_time_slider_->blockSignals(true);
            context_time_slider_->setRange(
                0,
                static_cast<int>(std::round((context_time_max_ - context_time_min_) *
                                            kContextSliderScale)));
            context_time_slider_->setValue(
                static_cast<int>(std::round((current_context_time_ - context_time_min_) *
                                            kContextSliderScale)));
            context_time_slider_->blockSignals(false);
        }

        updateContextTimeLabel();
        layoutContextControls();
    }

    void updateWindowTitle()
    {
        if (trials_.empty() || !active_robot_)
        {
            setWindowTitle("Safe Parking Debug");
            return;
        }

        const auto &trial = currentTrial();
        setWindowTitle(
            QString("Safe Parking Debug: %1 [%2/%3] %4")
                .arg(QString::fromStdString(active_robot_->name))
                .arg(selected_trial_ + 1)
                .arg(static_cast<int>(trials_.size()))
                .arg(planning_status_to_qstring(trial.result.status)));
    }

    void layoutContextControls()
    {
        if (!context_time_slider_ || !context_time_label_)
            return;

        const int margin = 12;
        const int label_height = 18;
        const int slider_height = 24;
        const int base_y = height() - kContextControlHeight + 8;

        context_time_label_->setGeometry(
            margin, base_y, width() - 2 * margin, label_height);
        context_time_slider_->setGeometry(
            margin, base_y + label_height + 6, width() - 2 * margin, slider_height);
    }

    void updateContextTimeLabel()
    {
        if (!context_time_label_)
            return;

        context_time_label_->setText(
            QString("Context time: %1 s   window=[%2, %3]   pivot=%4")
                .arg(current_context_time_, 0, 'f', 1)
                .arg(context_time_min_, 0, 'f', 1)
                .arg(context_time_max_, 0, 'f', 1)
                .arg(context_time_pivot_, 0, 'f', 1));
    }

    Pose interpolatePose(const std::map<double, Pose> &timeline, double t) const
    {
        if (timeline.empty())
            return Pose();
        auto it = timeline.lower_bound(t);
        if (it == timeline.end())
            return timeline.rbegin()->second;
        if (it->first == t || it == timeline.begin())
            return it->second;

        auto prev = std::prev(it);
        double t1 = prev->first;
        double t2 = it->first;
        Pose p1 = prev->second;
        Pose p2 = it->second;

        double ratio = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + ratio * (p2.x - p1.x);
        p.y = p1.y + ratio * (p2.y - p1.y);

        double dyaw = p2.yaw - p1.yaw;
        while (dyaw > M_PI)
            dyaw -= 2 * M_PI;
        while (dyaw < -M_PI)
            dyaw += 2 * M_PI;
        p.yaw = p1.yaw + ratio * dyaw;
        return p;
    }

    template <typename Func>
    void drawTrail(QPainter &p, const std::map<double, Pose> &timeline,
                   double t_start, double t_end, Func toScreen,
                   const QPen &pen = QPen(QColor(150, 150, 150), 2, Qt::DashLine)) const
    {
        QPolygonF path;
        auto it = timeline.lower_bound(t_start);

        if (it != timeline.begin())
        {
            Pose start_p = interpolatePose(timeline, t_start);
            path << toScreen(start_p.x, start_p.y);
        }

        for (; it != timeline.end() && it->first <= t_end; ++it)
        {
            path << toScreen(it->second.x, it->second.y);
        }

        if (it != timeline.end())
        {
            Pose end_p = interpolatePose(timeline, t_end);
            path << toScreen(end_p.x, end_p.y);
        }

        if (path.size() > 1)
        {
            p.setPen(pen);
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(path);
        }
    }

    template <typename Func>
    void drawShape(QPainter &p, Pose pose, EntityMeta *ent, QColor brush,
                   QPen pen, Func toScreen, QString label = "") const
    {
        auto c = get_corners_local(pose.x, pose.y, pose.yaw,
                                   ent->size.front_length, ent->size.rear_length,
                                   ent->size.width);
        QPolygonF poly;
        for (auto &pt : c)
            poly << toScreen(pt.x, pt.y);

        p.setBrush(brush);
        p.setPen(pen);
        p.drawPolygon(poly);

        if (!label.isEmpty())
        {
            p.setPen(Qt::black);
            p.drawText(toScreen(pose.x, pose.y) + QPointF(0, -5), label);
        }

        QPointF center = toScreen(pose.x, pose.y);
        QPointF front = toScreen(pose.x + 0.3 * std::cos(pose.yaw),
                                 pose.y + 0.3 * std::sin(pose.yaw));
        p.setPen(Qt::black);
        p.drawLine(center, front);
    }

    template <typename Func>
    void highlightBlockingEntity(QPainter &p, Func toScreen,
                                 const std::string &entity_name,
                                 const QString &marker) const
    {
        if (entity_name.empty() || entity_name == "Boundary" ||
            entity_name == "blocked_path_hint")
            return;

        EntityMeta *entity = findEntityByName(entity_name);
        if (!entity)
            return;

        Pose pose = currentTimeTable().get_pose(entity, current_context_time_);
        auto corners = get_corners_local(pose.x, pose.y, pose.yaw,
                                         entity->size.front_length,
                                         entity->size.rear_length,
                                         entity->size.width);
        QPolygonF poly;
        for (const auto &corner : corners)
            poly << toScreen(corner.x, corner.y);

        p.setPen(QPen(QColor(220, 0, 0), 3, Qt::SolidLine));
        p.setBrush(QBrush(QColor(255, 0, 0, 35)));
        p.drawPolygon(poly);
        p.setPen(Qt::black);
        p.drawText(poly.boundingRect().topLeft() + QPointF(0, -6), marker);
    }

    EntityMeta *findEntityByName(const std::string &entity_name) const
    {
        for (const auto &[entity, timeline] : currentTimeTable().get_database())
        {
            (void)timeline;
            if (entity && entity->name == entity_name)
                return entity;
        }
        return nullptr;
    }

    bool activePathOverlapsBlockingEntityNow() const
    {
        const SafeParkingDebugTrial &trial = currentTrial();
        const PlanningResult &plan = trial.result;
        if (plan.waypoints.empty() || plan.colliding_entity.empty() ||
            plan.colliding_entity == "Boundary" ||
            plan.colliding_entity == "blocked_path_hint")
        {
            return false;
        }

        if (current_context_time_ < plan.waypoints.front().time - 1e-9)
        {
            return false;
        }

        EntityMeta *entity = findEntityByName(plan.colliding_entity);
        if (!entity || !active_robot_)
            return false;

        Pose robot_pose = trial.candidate_pose;
        if (current_context_time_ <= plan.waypoints.back().time + 1e-9)
        {
            auto [x_now, y_now, yaw_now] =
                interpolate_timed_path(plan.waypoints, current_context_time_);
            robot_pose = Pose{x_now, y_now, yaw_now};
        }
        Pose other_pose = currentTimeTable().get_pose(entity, current_context_time_);

        Corners robot_corners = get_corners(robot_pose.x, robot_pose.y, robot_pose.yaw,
                                            active_robot_->size.front_length,
                                            active_robot_->size.rear_length,
                                            active_robot_->size.width);
        Corners other_corners = get_corners(other_pose.x, other_pose.y, other_pose.yaw,
                                            entity->size.front_length,
                                            entity->size.rear_length,
                                            entity->size.width);
        return rectangles_intersect(robot_corners, other_corners);
    }

    void drawLegend(QPainter &p, int x, int y, int w, int available_height) const
    {
        trial_row_rects_.clear();
        const SafeParkingDebugTrial &trial = currentTrial();
        const PlanningResult &plan = trial.result;

        p.setBrush(QColor(245, 245, 245));
        p.setPen(Qt::black);
        const int panel_h = std::max(560, available_height);
        p.drawRect(x, y, w, panel_h);

        int cy = y + 22;
        p.setFont(QFont("Arial", 10, QFont::Bold));
        p.drawText(x + 10, cy, "Safe Parking Search");
        cy += 18;
        p.setFont(QFont("Arial", 8));
        p.drawText(x + 10, cy, "Use Up/Down to browse trials");
        cy += 18;

        p.setFont(QFont("Arial", 9));
        p.drawText(x + 10, cy,
                   QString("Trial %1 / %2")
                       .arg(selected_trial_ + 1)
                       .arg(static_cast<int>(trials_.size())));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Current time: %1 s").arg(current_context_time_, 0, 'f', 2));
        cy += 18;
        p.drawText(x + 10, cy,
                   "Mode: " + QString::fromStdString(trial.candidate_mode));
        cy += 18;
        p.drawText(x + 10, cy,
                   "Status: " + planning_status_to_qstring(plan.status));
        cy += 18;
        p.drawText(
            x + 10, cy,
            QString("Candidate: (%1, %2, %3)")
                .arg(trial.candidate_pose.x, 0, 'f', 2)
                .arg(trial.candidate_pose.y, 0, 'f', 2)
                .arg(trial.candidate_pose.yaw, 0, 'f', 2));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Window: [%1, %2]")
                       .arg(context_time_min_, 0, 'f', 1)
                       .arg(context_time_max_, 0, 'f', 1));
        cy += 18;

        if (!plan.colliding_entity.empty())
        {
            p.drawText(x + 10, cy,
                       "Blocking entity: " +
                           QString::fromStdString(plan.colliding_entity));
            cy += 18;
        }

        if (plan.failure_time > 1e-6)
        {
            p.drawText(x + 10, cy,
                       QString("Failure time: abs=%1 s, rel=%2 s")
                           .arg(plan.failure_time, 0, 'f', 2)
                           .arg(plan.failure_time - plan_start_time_, 0, 'f', 2));
            cy += 18;
        }

        p.drawText(x + 10, cy,
                   QString("Active path overlaps blocker now: %1")
                       .arg(activePathOverlapsBlockingEntityNow() ? "YES" : "NO"));
        cy += 18;
        p.drawText(x + 10, cy,
                   "Blue dashed = actual robot timeline, bright blue = trial path");
        cy += 18;

        if (!trial.blocked_path_hint_waypoints.empty())
        {
            p.drawText(x + 10, cy, "Blocked-path hint overlay: shown");
            cy += 18;
        }

        QRect detail_rect(x + 10, cy, w - 20, 128);
        QString detail_text = plan.failure_detail.empty()
                                  ? "Detail: (none)"
                                  : ("Detail: " +
                                     QString::fromStdString(plan.failure_detail));
        p.drawText(detail_rect, Qt::TextWordWrap | Qt::AlignTop, detail_text);
        cy = detail_rect.bottom() + 18;

        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(x + 8, cy, x + w - 8, cy);
        cy += 16;

        p.setPen(Qt::black);
        p.setFont(QFont("Arial", 9, QFont::Bold));
        p.drawText(x + 10, cy, "Trial List");
        cy += 16;
        p.setFont(QFont("Arial", 8));

        const int max_visible = 14;
        int start_idx = std::max(0, selected_trial_ - max_visible / 2);
        int end_idx = std::min(start_idx + max_visible,
                               static_cast<int>(trials_.size()));
        if (end_idx - start_idx < max_visible)
        {
            start_idx = std::max(0, end_idx - max_visible);
        }

        for (int i = start_idx; i < end_idx; ++i)
        {
            const auto &entry = trials_[i];
            QRect row_rect(x + 10, cy - 12, w - 20, 18);
            trial_row_rects_.push_back({row_rect, i});
            if (i == selected_trial_)
            {
                p.fillRect(row_rect, QColor(220, 235, 255));
            }

            QString row_text =
                QString("%1%2 [%3] %4")
                    .arg(i == selected_trial_ ? "> " : "  ")
                    .arg(entry.trial_index, 3)
                    .arg(QString::fromStdString(entry.candidate_mode))
                    .arg(planning_status_to_qstring(entry.result.status));
            p.setPen(Qt::black);
            p.drawText(x + 12, cy, row_text);
            cy += 18;
        }
    }

    const TimeTable &timetable_;
    RobotMeta *active_robot_ = nullptr;
    double plan_start_time_ = 0.0;
    Pose start_pose_;
    std::vector<SafeParkingDebugTrial> trials_;
    int selected_trial_ = 0;
    Params params_;
    double current_context_time_ = 0.0;
    double context_time_pivot_ = 0.0;
    double context_time_min_ = 0.0;
    double context_time_max_ = 0.0;
    QLabel *context_time_label_ = nullptr;
    QSlider *context_time_slider_ = nullptr;
    mutable std::vector<std::pair<QRect, int>> trial_row_rects_;
};

class PostTaskVerificationDebugWidget : public QDialog
{
public:
    PostTaskVerificationDebugWidget(const TimeTable &tt,
                                    RobotMeta *robot,
                                    EntityMeta *task_object,
                                    double verification_time,
                                    const std::string &verification_reason,
                                    const std::string &verification_entity_a,
                                    const std::string &verification_entity_b,
                                    const Params &params,
                                    const std::string &context_text = "",
                                    QWidget *parent = nullptr)
        : QDialog(parent), timetable_(tt), active_robot_(robot),
          task_object_(task_object),
          verification_time_(verification_time),
          verification_reason_(verification_reason),
          verification_entity_a_(verification_entity_a),
          verification_entity_b_(verification_entity_b),
          params_(params), context_text_(context_text)
    {
        context_time_label_ = new QLabel(this);
        context_time_label_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        context_time_slider_ = new QSlider(Qt::Horizontal, this);
        QObject::connect(context_time_slider_, &QSlider::valueChanged,
                         this, [this](int value)
                         {
                             current_context_time_ =
                                 context_time_min_ +
                                 static_cast<double>(value) / kContextSliderScale;
                             updateContextTimeLabel();
                             update();
                         });

        initializeContextWindow();
        resize(1340, 860);
        updateWindowTitle();
    }

protected:
    void resizeEvent(QResizeEvent *event) override
    {
        QDialog::resizeEvent(event);
        layoutContextControls();
    }

    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), Qt::white);

        const int legend_width = 380;
        const int view_width = width() - legend_width;
        const int view_height = height() - kContextControlHeight;

        const double margin = 1.0;
        const double w_real = (params_.max_x - params_.min_x) + 2 * margin;
        const double h_real = (params_.max_y - params_.min_y) + 2 * margin;
        const double scale = std::min((double)view_width / w_real,
                                      (double)view_height / h_real);

        auto toScreen = [&](double x, double y)
        {
            return QPointF((x - params_.min_x + margin) * scale,
                           (params_.max_y - y + margin) * scale);
        };

        p.setPen(QPen(Qt::black, 3, Qt::SolidLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(toScreen(params_.min_x, params_.max_y),
                          toScreen(params_.max_x, params_.min_y)));

        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(view_width, 0, view_width, view_height);

        for (const auto &[entity, timeline] : timetable_.get_database())
        {
            Pose current_pose = timetable_.get_pose(entity, current_context_time_);
            QColor body_color =
                (entity->type == EntityType::OBJECT)
                    ? QColor(255, 140, 0, 150)
                    : QColor(100, 100, 100, 150);

            if (entity == active_robot_)
            {
                body_color = QColor(30, 144, 255, 170);
            }
            else if (entity == task_object_)
            {
                body_color = QColor(255, 215, 0, 170);
            }

            drawShape(p, current_pose, entity, body_color, QPen(Qt::black, 1),
                      toScreen, QString::fromStdString(entity->name));
            drawTrail(p, timeline, context_time_min_, context_time_max_, toScreen);
        }

        highlightVerifiedEntity(p, toScreen, verification_entity_a_, "A");
        highlightVerifiedEntity(p, toScreen, verification_entity_b_, "B");

        drawLegend(p, view_width + 10, 20, legend_width - 20, view_height - 40);
    }

private:
    static constexpr int kContextSliderScale = 10;
    static constexpr int kContextControlHeight = 64;

    void initializeContextWindow()
    {
        context_time_pivot_ = std::max(0.0, verification_time_);
        context_time_min_ = std::max(0.0, context_time_pivot_ - 30.0);
        context_time_max_ = std::max(context_time_min_ + 0.1,
                                     context_time_pivot_ + 30.0);
        current_context_time_ = context_time_pivot_;

        context_time_slider_->setRange(
            0,
            static_cast<int>(std::round((context_time_max_ - context_time_min_) *
                                        kContextSliderScale)));
        context_time_slider_->setValue(
            static_cast<int>(std::round((current_context_time_ - context_time_min_) *
                                        kContextSliderScale)));
        updateContextTimeLabel();
        layoutContextControls();
    }

    void updateWindowTitle()
    {
        QString robot_name =
            active_robot_ ? QString::fromStdString(active_robot_->name)
                          : QString("(unknown robot)");
        setWindowTitle(
            QString("Post-Task Verification Debug: %1 @ t=%2")
                .arg(robot_name)
                .arg(verification_time_, 0, 'f', 2));
    }

    void layoutContextControls()
    {
        if (!context_time_slider_ || !context_time_label_)
            return;

        const int margin = 12;
        const int label_height = 18;
        const int slider_height = 24;
        const int base_y = height() - kContextControlHeight + 8;

        context_time_label_->setGeometry(
            margin, base_y, width() - 2 * margin, label_height);
        context_time_slider_->setGeometry(
            margin, base_y + label_height + 6, width() - 2 * margin, slider_height);
    }

    void updateContextTimeLabel()
    {
        if (!context_time_label_)
            return;

        context_time_label_->setText(
            QString("Timestamp: %1 s   window=[%2, %3]   collision=%4")
                .arg(current_context_time_, 0, 'f', 1)
                .arg(context_time_min_, 0, 'f', 1)
                .arg(context_time_max_, 0, 'f', 1)
                .arg(context_time_pivot_, 0, 'f', 1));
    }

    Pose interpolatePose(const std::map<double, Pose> &timeline, double t) const
    {
        if (timeline.empty())
            return Pose();

        auto it = timeline.lower_bound(t);
        if (it == timeline.end())
            return timeline.rbegin()->second;
        if (it->first == t || it == timeline.begin())
            return it->second;

        auto prev = std::prev(it);
        double t1 = prev->first;
        double t2 = it->first;
        Pose p1 = prev->second;
        Pose p2 = it->second;

        double ratio = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + ratio * (p2.x - p1.x);
        p.y = p1.y + ratio * (p2.y - p1.y);

        double dyaw = p2.yaw - p1.yaw;
        while (dyaw > M_PI)
            dyaw -= 2 * M_PI;
        while (dyaw < -M_PI)
            dyaw += 2 * M_PI;
        p.yaw = p1.yaw + ratio * dyaw;
        return p;
    }

    template <typename Func>
    void drawTrail(QPainter &p, const std::map<double, Pose> &timeline,
                   double t_start, double t_end, Func toScreen) const
    {
        QPolygonF path;
        auto it = timeline.lower_bound(t_start);

        if (it != timeline.begin())
        {
            Pose start_p = interpolatePose(timeline, t_start);
            path << toScreen(start_p.x, start_p.y);
        }

        for (; it != timeline.end() && it->first <= t_end; ++it)
        {
            path << toScreen(it->second.x, it->second.y);
        }

        if (it != timeline.end())
        {
            Pose end_p = interpolatePose(timeline, t_end);
            path << toScreen(end_p.x, end_p.y);
        }

        if (path.size() > 1)
        {
            p.setPen(QPen(QColor(150, 150, 150), 2, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(path);
        }
    }

    template <typename Func>
    void drawShape(QPainter &p, Pose pose, EntityMeta *ent, QColor brush,
                   QPen pen, Func toScreen, QString label = "") const
    {
        if (!ent)
            return;

        auto c = get_corners_local(pose.x, pose.y, pose.yaw,
                                   ent->size.front_length, ent->size.rear_length,
                                   ent->size.width);
        QPolygonF poly;
        for (auto &pt : c)
            poly << toScreen(pt.x, pt.y);

        p.setBrush(brush);
        p.setPen(pen);
        p.drawPolygon(poly);

        if (!label.isEmpty())
        {
            p.setPen(Qt::black);
            p.drawText(toScreen(pose.x, pose.y) + QPointF(0, -5), label);
        }

        QPointF center = toScreen(pose.x, pose.y);
        QPointF front = toScreen(pose.x + 0.3 * std::cos(pose.yaw),
                                 pose.y + 0.3 * std::sin(pose.yaw));
        p.setPen(Qt::black);
        p.drawLine(center, front);
    }

    template <typename Func>
    void highlightVerifiedEntity(QPainter &p, Func toScreen,
                                 const std::string &entity_name,
                                 const QString &marker) const
    {
        if (entity_name.empty() || entity_name == "Boundary")
            return;

        EntityMeta *entity = findEntityByName(entity_name);
        if (!entity)
            return;

        Pose pose = timetable_.get_pose(entity, current_context_time_);
        auto corners = get_corners_local(pose.x, pose.y, pose.yaw,
                                         entity->size.front_length,
                                         entity->size.rear_length,
                                         entity->size.width);
        QPolygonF poly;
        for (const auto &corner : corners)
            poly << toScreen(corner.x, corner.y);

        p.setPen(QPen(QColor(220, 0, 0), 3, Qt::SolidLine));
        p.setBrush(QBrush(QColor(255, 0, 0, 35)));
        p.drawPolygon(poly);
        p.setPen(Qt::black);
        p.drawText(poly.boundingRect().topLeft() + QPointF(0, -6), marker);
    }

    EntityMeta *findEntityByName(const std::string &entity_name) const
    {
        for (const auto &[entity, timeline] : timetable_.get_database())
        {
            (void)timeline;
            if (entity && entity->name == entity_name)
                return entity;
        }
        return nullptr;
    }

    bool verifiedPairOverlapsAtCurrentTime() const
    {
        if (verification_entity_a_.empty() || verification_entity_b_.empty() ||
            verification_entity_a_ == "Boundary" || verification_entity_b_ == "Boundary")
        {
            return false;
        }

        EntityMeta *a = findEntityByName(verification_entity_a_);
        EntityMeta *b = findEntityByName(verification_entity_b_);
        if (!a || !b)
            return false;

        Pose pa = timetable_.get_pose(a, current_context_time_);
        Pose pb = timetable_.get_pose(b, current_context_time_);
        Corners ca = get_corners(pa.x, pa.y, pa.yaw,
                                 a->size.front_length, a->size.rear_length, a->size.width);
        Corners cb = get_corners(pb.x, pb.y, pb.yaw,
                                 b->size.front_length, b->size.rear_length, b->size.width);
        return rectangles_intersect(ca, cb);
    }

    void drawLegend(QPainter &p, int x, int y, int w, int available_height) const
    {
        p.setBrush(QColor(245, 245, 245));
        p.setPen(Qt::black);
        const int panel_h = std::max(480, available_height);
        p.drawRect(x, y, w, panel_h);

        int cy = y + 22;
        p.setFont(QFont("Arial", 10, QFont::Bold));
        p.drawText(x + 10, cy, "Post-Task Verification");
        cy += 18;

        p.setFont(QFont("Arial", 9));
        if (active_robot_)
        {
            p.drawText(x + 10, cy,
                       "Assigned Robot: " + QString::fromStdString(active_robot_->name));
            cy += 18;
        }
        if (task_object_)
        {
            p.drawText(x + 10, cy,
                       "Task Object: " + QString::fromStdString(task_object_->name));
            cy += 18;
        }

        p.drawText(x + 10, cy,
                   QString("Current time: %1 s").arg(current_context_time_, 0, 'f', 2));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Collision time: %1 s").arg(verification_time_, 0, 'f', 2));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Entities: %1 vs %2")
                       .arg(QString::fromStdString(verification_entity_a_))
                       .arg(QString::fromStdString(verification_entity_b_)));
        cy += 18;
        p.drawText(x + 10, cy,
                   "Reason: " + QString::fromStdString(verification_reason_));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Pair overlaps now: %1")
                       .arg(verifiedPairOverlapsAtCurrentTime() ? "YES" : "NO"));
        cy += 18;
        p.drawText(x + 10, cy,
                   QString("Window: [%1, %2]")
                       .arg(context_time_min_, 0, 'f', 1)
                       .arg(context_time_max_, 0, 'f', 1));
        cy += 24;

        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(x + 8, cy, x + w - 8, cy);
        cy += 16;

        p.setPen(Qt::black);
        p.setFont(QFont("Arial", 9, QFont::Bold));
        p.drawText(x + 10, cy, "Legend");
        cy += 18;
        p.setFont(QFont("Arial", 8));

        auto legendItem = [&](const QString &text, const QColor &color)
        {
            p.setBrush(color);
            p.setPen(Qt::black);
            p.drawRect(x + 10, cy - 10, 14, 14);
            p.drawText(x + 34, cy + 1, text);
            cy += 18;
        };

        legendItem("Assigned robot", QColor(30, 144, 255, 170));
        legendItem("Task object", QColor(255, 215, 0, 170));
        legendItem("Other robot", QColor(100, 100, 100, 150));
        legendItem("Other object", QColor(255, 140, 0, 150));
        legendItem("Verified collision pair", QColor(255, 0, 0, 60));

        if (!context_text_.empty())
        {
            cy += 8;
            p.setPen(QPen(Qt::gray, 1));
            p.drawLine(x + 8, cy, x + w - 8, cy);
            cy += 16;

            p.setPen(Qt::black);
            p.setFont(QFont("Arial", 9, QFont::Bold));
            p.drawText(x + 10, cy, "Context");
            cy += 16;
            p.setFont(QFont("Arial", 8));
            QRect text_rect(x + 10, cy, w - 20, panel_h - (cy - y) - 12);
            p.drawText(text_rect, Qt::TextWordWrap | Qt::AlignTop,
                       QString::fromStdString(context_text_));
        }
    }

    const TimeTable &timetable_;
    RobotMeta *active_robot_ = nullptr;
    EntityMeta *task_object_ = nullptr;
    double verification_time_ = 0.0;
    std::string verification_reason_;
    std::string verification_entity_a_;
    std::string verification_entity_b_;
    const Params &params_;
    std::string context_text_;
    double current_context_time_ = 0.0;
    double context_time_pivot_ = 0.0;
    double context_time_min_ = 0.0;
    double context_time_max_ = 0.0;
    QLabel *context_time_label_ = nullptr;
    QSlider *context_time_slider_ = nullptr;
};

// Main entry point for visualization
void visualize_planning_debug(const TimeTable &tt,
                              RobotMeta *robot,
                              const PlanningResult &result,
                              double start_time,
                              const Pose &start,
                              const Pose &goal,
                              const Params &params,
                              const std::string &attempt_trace = "",
                              const std::string &plan_kind = "")
{
    if (!QApplication::instance())
    {
        static int argc = 1;
        static char arg[] = "viz";
        static char *argv[] = {arg};
        new QApplication(argc, argv);
    }
    PlanningDebugWidget w(tt, robot, result, start_time, start, goal,
                          attempt_trace, plan_kind, params);
    w.exec();
}

inline void visualize_safe_parking_debug(const TimeTable &tt,
                                         RobotMeta *robot,
                                         double start_time,
                                         const Pose &start,
                                         const std::vector<SafeParkingDebugTrial> &trials,
                                         const Params &params)
{
    if (!QApplication::instance())
    {
        static int argc = 1;
        static char arg[] = "viz";
        static char *argv[] = {arg};
        new QApplication(argc, argv);
    }

    SafeParkingDebugWidget w(tt, robot, start_time, start, trials, params);
    w.exec();
}

inline void visualize_planning_attempt_debug(
    const TimeTable &tt,
    RobotMeta *robot,
    double start_time,
    const Pose &start,
    const Pose &goal,
    const std::vector<PlanningDebugAttempt> &attempts,
    const Params &params,
    const std::string &plan_kind = "")
{
    if (!QApplication::instance())
    {
        static int argc = 1;
        static char arg[] = "viz";
        static char *argv[] = {arg};
        new QApplication(argc, argv);
    }

    PlanningAttemptDebugWidget w(tt, robot, start_time, start, goal,
                                 attempts, plan_kind, params);
    w.exec();
}

inline void visualize_post_task_verification_debug(
    const TimeTable &tt,
    RobotMeta *robot,
    EntityMeta *task_object,
    double verification_time,
    const std::string &verification_reason,
    const std::string &verification_entity_a,
    const std::string &verification_entity_b,
    const Params &params,
    const std::string &context_text = "")
{
    if (!QApplication::instance())
    {
        static int argc = 1;
        static char arg[] = "viz";
        static char *argv[] = {arg};
        new QApplication(argc, argv);
    }

    PostTaskVerificationDebugWidget w(tt, robot, task_object,
                                      verification_time, verification_reason,
                                      verification_entity_a, verification_entity_b,
                                      params, context_text);
    w.exec();
}
#endif // VISUALIZATION_H
