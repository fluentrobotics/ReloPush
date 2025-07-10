#include <Visualization/QtMainControlWindow.h>
#include <QInputDialog>
#include <QMessageBox>

MainControlWindow::MainControlWindow(QWidget *parent)
    : QMainWindow(parent),
    pathManager(new PathManager(this))
{
    setWindowTitle("Path Manager Control Panel");

    centralWidget = new QWidget(this);
    layout = new QVBoxLayout(centralWidget);

    addPathButton = new QPushButton("Add Path", centralWidget);
    removePathButton = new QPushButton("Remove Selected Path", centralWidget);
    pathListWidget = new QListWidget(centralWidget);

    layout->addWidget(addPathButton);
    layout->addWidget(removePathButton);
    layout->addWidget(pathListWidget);

    setCentralWidget(centralWidget);

    //connect(addPathButton, &QPushButton::clicked, this, &MainControlWindow::onAddPathClicked);
    //connect(removePathButton, &QPushButton::clicked, this, &MainControlWindow::onRemovePathClicked);
}

MainControlWindow::~MainControlWindow()
{
    // PathManager will handle the deletion of PathWindows
}

/*
void MainControlWindow::onAddPathClicked()
{
    // For demonstration, we'll add pre-defined paths. In a real application,
    // you might allow users to input path data dynamically.

    // Prompt user to choose which path to add
    bool ok;
    QStringList items;
    items << "Path 1" << "Path 2";
    QString item = QInputDialog::getItem(this, "Select Path to Add",
                                         "Path:", items, 0, false, &ok);
    if (ok && !item.isEmpty()) {
        if(item == "Path 1"){
            std::vector<ReloPush::State> path = generatePath1();
            std::vector<ReloPush::State> obstacles = generateObstacles1();
            pathManager->addPath(path, obstacles, "Path 1 Visualization",
                                 QColor(70, 130, 180), QColor(34, 139, 34),
                                 QColor(220, 20, 60), QColor(178, 34, 34),
                                 QColor(128, 128, 128));
            pathListWidget->addItem("Path 1 Visualization");
        }
        else if(item == "Path 2"){
            std::vector<ReloPush::State> path = generatePath2();
            std::vector<ReloPush::State> obstacles = generateObstacles2();
            pathManager->addPath(path, obstacles, "Path 2 Visualization",
                                 QColor(255, 165, 0), QColor(75, 0, 130),
                                 QColor(0, 128, 0), QColor(0, 100, 0),
                                 QColor(169, 169, 169));
            pathListWidget->addItem("Path 2 Visualization");
        }
    }
}
*/

/*
void MainControlWindow::onRemovePathClicked()
{
    QListWidgetItem *selectedItem = pathListWidget->currentItem();
    if(selectedItem){
        QString windowTitle = selectedItem->text();
        // Find and remove the corresponding PathWindow
        // This requires PathManager to provide a method to find PathWindow by title
        // For simplicity, let's assume window titles are unique

        // Iterate through PathManager's windows to find the matching one
        for(auto window : pathManager->getPathWindows()){
            if(window->windowTitle() == windowTitle){
                pathManager->removePath(window);
                break;
            }
        }

        // Remove from the list widget
        delete selectedItem;
    }
    else{
        QMessageBox::information(this, "Remove Path", "Please select a path to remove.");
    }
}

std::vector<ReloPush::State> MainControlWindow::generatePath1()
{
    std::vector<ReloPush::State> path;
    path.emplace_back(10.0f, 10.0f, 0.0f);             // Start
    path.emplace_back(20.0f, 15.0f, M_PI / 12);       // 15 degrees
    path.emplace_back(30.0f, 25.0f, M_PI / 6);        // 30 degrees
    path.emplace_back(40.0f, 35.0f, M_PI / 4);        // 45 degrees
    path.emplace_back(50.0f, 50.0f, M_PI / 3);        // 60 degrees
    path.emplace_back(60.0f, 65.0f, M_PI / 2);        // 90 degrees
    path.emplace_back(70.0f, 80.0f, 2 * M_PI / 3);    // 120 degrees
    path.emplace_back(80.0f, 85.0f, 5 * M_PI / 6);    // 150 degrees
    path.emplace_back(90.0f, 90.0f, M_PI / 2);        // Goal
    return path;
}

std::vector<ReloPush::State> MainControlWindow::generatePath2()
{
    std::vector<ReloPush::State> path;
    path.emplace_back(10.0f, 90.0f, M_PI);            // Start (facing left)
    path.emplace_back(20.0f, 85.0f, 11 * M_PI / 12);  // 165 degrees
    path.emplace_back(30.0f, 75.0f, 3 * M_PI / 4);    // 135 degrees
    path.emplace_back(40.0f, 65.0f, 5 * M_PI / 6);    // 150 degrees
    path.emplace_back(50.0f, 50.0f, M_PI / 2);        // 90 degrees
    path.emplace_back(60.0f, 35.0f, M_PI / 3);        // 60 degrees
    path.emplace_back(70.0f, 25.0f, M_PI / 4);        // 45 degrees
    path.emplace_back(80.0f, 15.0f, M_PI / 6);        // 30 degrees
    path.emplace_back(90.0f, 10.0f, 0.0f);            // Goal
    return path;
}

std::vector<ReloPush::State> MainControlWindow::generateObstacles1()
{
    std::vector<ReloPush::State> obstacles;
    obstacles.emplace_back(30.0f, 30.0f, 0.0f);
    obstacles.emplace_back(50.0f, 40.0f, 0.0f);
    obstacles.emplace_back(70.0f, 60.0f, 0.0f);
    obstacles.emplace_back(60.0f, 80.0f, 0.0f);
    obstacles.emplace_back(40.0f, 70.0f, 0.0f);
    return obstacles;
}

std::vector<ReloPush::State> MainControlWindow::generateObstacles2()
{
    std::vector<ReloPush::State> obstacles;
    obstacles.emplace_back(30.0f, 70.0f, 0.0f);
    obstacles.emplace_back(50.0f, 60.0f, 0.0f);
    obstacles.emplace_back(70.0f, 40.0f, 0.0f);
    obstacles.emplace_back(60.0f, 20.0f, 0.0f);
    obstacles.emplace_back(40.0f, 30.0f, 0.0f);
    return obstacles;
}
*/
