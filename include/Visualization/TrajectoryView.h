#ifndef TRAJECTORYVIEW_H
#define TRAJECTORYVIEW_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPolygonItem>
#include <QTimer>
#include <QPen>
#include <QBrush>
#include <trajectory.hpp>

class TrajectoryView : public QGraphicsView {
    Q_OBJECT
public:
    explicit TrajectoryView(QWidget* parent = nullptr);

    void setTrajectory(const ReloPush::trajectory& traj);

private:
    void animateFromIndex(size_t index, const std::vector<ReloPush::trajectory_elem>& points);
    void updateArrowPosition(const ReloPush::trajectory_elem& elem);

    QGraphicsScene* scene;
    QGraphicsPolygonItem* arrowItem;
    double scaleFactor;
};

#endif // TRAJECTORYVIEW_H
