#include <ReloPush/Visualization/QtPathWindow.h>
#include <QMenuBar>
#include <QFileDialog>

PathWindow::PathWindow(const std::vector<ReloPush::State>& path,
                       const std::vector<ObjectInfo>& obstacles,
                       QWidget *parent,
                       QString windowTitle,
                       QColor initialPoseColor,
                       QColor goalPoseColor,
                       QColor pathColor,
                       QColor pathArrowColor,
                       QColor obstacleColor,
                       QColor goalsColor)
    : QMainWindow(parent)
{
    setWindowTitle(windowTitle);

    // Instantiate VisualizationWidget with custom colors
    vizWidget = new VisualizationWidget(this,
                                        initialPoseColor,
                                        goalPoseColor,
                                        pathColor,
                                        pathArrowColor,
                                        obstacleColor);

    // Set workspace dimensions (assuming same for all paths)
    float workspace_width = 100.0f;
    float workspace_height = 100.0f;
    vizWidget->setWorkspace(workspace_width, workspace_height);

    // Set initial and goal poses (assuming default positions or customize as needed)
    ReloPush::State initial_pose(10.0f, 10.0f, 0.0f);           // Example values
    ReloPush::State goal_pose(90.0f, 90.0f, M_PI / 2);         // Example values
    vizWidget->setInitialPose(initial_pose);
    vizWidget->setGoalPose(goal_pose);

    // Set the path
    vizWidget->setPath(path);

    // Set obstacles
    vizWidget->setObstacles(obstacles);

    // Set the VisualizationWidget as the central widget
    setCentralWidget(vizWidget);

    // Add Export action to the menu
    QMenu *fileMenu = menuBar()->addMenu("File");
    QAction *exportAction = fileMenu->addAction("Export as Image");
    connect(exportAction, &QAction::triggered, this, &PathWindow::exportVisualization);
}

PathWindow::~PathWindow()
{
    // Qt handles the deletion of child widgets
}

void PathWindow::exportVisualization()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Export Visualization",
                                                    "", "PNG Images (*.png);;JPEG Images (*.jpg)");
    if (!fileName.isEmpty()) {
        // Render the VisualizationWidget to a QPixmap
        QPixmap pixmap(vizWidget->size());
        vizWidget->render(&pixmap);
        pixmap.save(fileName);
    }
}
