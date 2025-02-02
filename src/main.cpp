#include "GraphBuilder.hpp"
#include "InputParser.hpp"
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>
#include <Visualization/VisualizeResults.h>

// #include <QApplication>
// #include <QMainWindow>
// #include <Visualization/VisualizationWidget.h>
// #include <Visualization/QtMainControlWindow.h>

// ---------------------------------------------------------------------------
// Helper Function 1: Parse and Initialize
// ---------------------------------------------------------------------------
bool parseAndInitialize(const std::string &filename,
                        WorkspaceBoundary &boundary,
                        std::unordered_map<std::string, ObjectInfo> &objects,
                        std::unordered_map<std::string, GoalInfo> &goals,
                        std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs)
{
    if (!parseInputFile(filename, boundary, objects, goals, objGoalPairs))
    {
        std::cerr << "Parse failed.\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Helper Function 5: Print the final sequence
// ---------------------------------------------------------------------------
void printFinalSequence(const std::vector<FinalAllocation> &finalSequence)
{
    std::cout << "\nFinal sequence of chosen tasks:\n";
    for (auto &fa : finalSequence)
    {
        std::cout << "Object = " << fa.object.name
                  << ", Goal = " << fa.goal.name
                  << ", cost = " << fa.cost << "\n"
                  << "  start yaw = " << fa.startPose.yaw
                  << ", goal yaw = " << fa.goalPose.yaw;

        if (fa.obsReloPaths->size() > 0)
        {
            std::cout << ", ObsRelo steps: " << fa.obsReloPaths->size() << "\n";
        }
        else
        {
            std::cout << std::endl;
        }
    }
}


// ---------------------------------------------------------------------------
// Main Function
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // Example: You could allow a command-line argument or use a fixed filename
    // if (argc < 2) {
    //     std::cerr << "Usage: " << argv[0] << " <input_file>\n";
    //     return 1;
    // }
    // std::string filename = argv[1];
    std::string filename = "input4.txt";

    // Data structures
    WorkspaceBoundary boundary;
    std::unordered_map<std::string, ObjectInfo> objects;
    std::unordered_map<std::string, GoalInfo>   goals;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;

    // 1) Parse and initialize
    if (!parseAndInitialize(filename, boundary, objects, goals, objGoalPairs))
    {
        return 1;
    }

    // 2) Perform the main planning/allocation loop
    std::vector<FinalAllocation> finalSequence;
    performAllocations(boundary, objects, goals, objGoalPairs, finalSequence);

    // 3) Print the final sequence
    printFinalSequence(finalSequence);

    // 4) Visualization
    if (!finalSequence.empty())
    {
        visualizeResults(finalSequence, app);
    }

}
