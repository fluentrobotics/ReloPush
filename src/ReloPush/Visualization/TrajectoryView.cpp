#include <ReloPush/Visualization/TrajectoryView.h>
#include <QGraphicsPolygonItem>
#include <QTimer>
#include <QPen>
#include <QBrush>
#include <QPolygonF>
#include <QColor>
#include <cmath>

// Constructor
TrajectoryView::TrajectoryView(QWidget* parent)
    : QGraphicsView(parent), arrowItem(nullptr)
{
    scene = new QGraphicsScene(this);
    setScene(scene);
    setRenderHint(QPainter::Antialiasing);
    scaleFactor = 100.0; // meters to pixels conversion factor
}

// Implementation of setTrajectory
void TrajectoryView::setTrajectory(const ReloPush::trajectory& traj) {
    scene->clear();
    auto& points = *(traj.trajectory_points);

    QPen linePen(QColor("#0000FF"));  // Blue color via hex code.
    linePen.setWidth(2);
    for (size_t i = 1; i < points.size(); i++) {
        const ReloPush::trajectory_elem& prev = points[i - 1];
        const ReloPush::trajectory_elem& curr = points[i];
        scene->addLine(prev.x * scaleFactor, -prev.y * scaleFactor,
                       curr.x * scaleFactor, -curr.y * scaleFactor,
                       linePen);
    }

    if (!arrowItem) {
        QPolygonF arrow;
        arrow << QPointF(7.5, 0) << QPointF(-7.5, -5) << QPointF(-7.5, 5);
        arrowItem = scene->addPolygon(arrow, QPen(QColor("#FF5733")), QBrush(QColor("#FF5733")));
        arrowItem->setTransformOriginPoint(arrowItem->boundingRect().center());
    }

    if (!points.empty()) {
        updateArrowPosition(points[0]);
        animateFromIndex(0, points);
    }
}

void TrajectoryView::animateFromIndex(size_t index, const std::vector<ReloPush::trajectory_elem>& points) {
    if (index >= points.size() - 1) return;
    float currentTime = points[index].time;
    float nextTime = points[index + 1].time;
    // If your time is in seconds, multiply by 1000.
    int delay = static_cast<int>((nextTime - currentTime)*1000);
    QTimer::singleShot(delay, this, [=, &points]() {
        updateArrowPosition(points[index + 1]);
        animateFromIndex(index + 1, points);
    });
}

void TrajectoryView::updateArrowPosition(const ReloPush::trajectory_elem& elem) {
    qreal x = elem.x * scaleFactor;
    qreal y = -elem.y * scaleFactor;
    arrowItem->setPos(x, y);
    qreal angleDegrees = -elem.yaw * 180.0 / M_PI;
    arrowItem->setRotation(angleDegrees);
}

// If you are not using CMAKE_AUTOMOC, then add:
//#include "moc_TrajectoryView.cpp"
