#include <GraphBuilder.hpp>
#include "InputParser.hpp"
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>
#include <Visualization/VisualizeResults.h>


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
    std::string filename = "input5.txt";

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
