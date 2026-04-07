#ifndef PATHMANAGER_H
#define PATHMANAGER_H

#include <QObject>
#include <vector>
#include <ReloPush/Visualization/QtPathWindow.h>
#include <ReloPush/State.h>

class PathManager : public QObject
{
    //Q_OBJECT

public:
    explicit PathManager(QObject *parent = nullptr);
    ~PathManager();

    void addPath(const std::vector<ReloPush::State>& path,
                 const std::vector<ObjectInfo>& obstacles,
                 QString windowTitle = "Path Visualization",
                 QColor initialPoseColor = Qt::blue,
                 QColor goalPoseColor = Qt::green,
                 QColor pathColor = Qt::red,
                 QColor pathArrowColor = Qt::darkRed,
                 QColor obstacleColor = Qt::gray,
                 QColor goalsColor = Qt::red);

    void removePath(PathWindow *window);

    // Optionally, provide access to the list of PathWindows
    std::vector<PathWindow*> getPathWindows() const;

private:
    std::vector<PathWindow*> pathWindows;
};

#endif // PATHMANAGER_H
