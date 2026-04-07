#ifndef PATHWINDOW_H
#define PATHWINDOW_H

#include <QMainWindow>
#include <ReloPush/Visualization/VisualizationWidget.h>
#include <ReloPush/State.h>

class PathWindow : public QMainWindow
{
    //Q_OBJECT

public:
    explicit PathWindow(const std::vector<ReloPush::State>& path,
                        const std::vector<ObjectInfo>& obstacles,
                        QWidget *parent = nullptr,
                        QString windowTitle = "Path Visualization",
                        QColor initialPoseColor = Qt::blue,
                        QColor goalPoseColor = Qt::green,
                        QColor pathColor = Qt::red,
                        QColor pathArrowColor = Qt::darkRed,
                        QColor obstacleColor = Qt::gray,
                        QColor goalsColor = Qt::red);

    ~PathWindow();

private:
    VisualizationWidget *vizWidget;

// Add export functionality
public slots:
    void exportVisualization();
};

#endif // PATHWINDOW_H
