#ifndef MAINCONTROLWINDOW_H
#define MAINCONTROLWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QListWidget>
#include <QVBoxLayout>
#include <ReloPush/Visualization/QtPathManager.h>

class MainControlWindow : public QMainWindow
{
    //Q_OBJECT

public:
    explicit MainControlWindow(QWidget *parent = nullptr);
    ~MainControlWindow();

private slots:
//    void onAddPathClicked();
//    void onRemovePathClicked();

private:
    QWidget *centralWidget;
    QVBoxLayout *layout;
    QPushButton *addPathButton;
    QPushButton *removePathButton;
    QListWidget *pathListWidget;

    PathManager *pathManager;

    // Sample data generators for demonstration
//    std::vector<ReloPush::State> generatePath1();
//    std::vector<ReloPush::State> generatePath2();
//    std::vector<ReloPush::State> generateObstacles1();
//    std::vector<ReloPush::State> generateObstacles2();
};

#endif // MAINCONTROLWINDOW_H
