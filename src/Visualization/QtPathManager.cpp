#include <Visualization/QTPathManager.h>

PathManager::PathManager(QObject *parent)
    : QObject(parent)
{
}

PathManager::~PathManager()
{
    for(auto window : pathWindows){
        window->close();
        delete window;
    }
}

void PathManager::addPath(const std::vector<ReloPush::State>& path,
                          const std::vector<ReloPush::State>& obstacles,
                          QString windowTitle,
                          QColor initialPoseColor,
                          QColor goalPoseColor,
                          QColor pathColor,
                          QColor pathArrowColor,
                          QColor obstacleColor,
                          QColor goalsColor)
{
    PathWindow *window = new PathWindow(path, obstacles, nullptr, windowTitle,
                                        initialPoseColor, goalPoseColor,
                                        pathColor, pathArrowColor,
                                        obstacleColor, goalsColor);
    pathWindows.push_back(window);
    window->show();
}

void PathManager::removePath(PathWindow *window)
{
    auto it = std::find(pathWindows.begin(), pathWindows.end(), window);
    if(it != pathWindows.end()){
        (*it)->close();
        delete *it;
        pathWindows.erase(it);
    }
}

std::vector<PathWindow*> PathManager::getPathWindows() const
{
    return pathWindows;
}
