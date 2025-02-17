#include <GraphBuilder.hpp>
#include "InputParser.hpp"
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>
#include <Visualization/VisualizeResults.h>
#include <chrono>

// ---------------------------------------------------------------------------
// Main Function
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    QApplication app(argc, argv);

    // Example: You could allow a command-line argument or use a fixed filename
    // if (argc < 2) {
    //     std::cerr << "Usage: " << argv[0] << " <input_file>\n";
    //     return 1;
    // }
    // std::string filename = argv[1];
    std::string filename = "clear_to_corners.txt";
    filename = "input3.txt";

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

    auto start = std::chrono::high_resolution_clock::now();

    // 2) Perform the main planning/allocation loop
    std::vector<FinalAllocation> finalSequence;
    bool ok = performAllocations(boundary, objects, goals, objGoalPairs, finalSequence);

    auto end = std::chrono::high_resolution_clock::now();

    if(!ok)
    {
        // plan failed
        std::cout << "Failed to find a solution" << std::endl;
        return -1;
    }

    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;

    // 3) Print the final sequence
    printFinalSequence(finalSequence);

    // 4) Visualization
    if (!finalSequence.empty())
    {
        visualizeResults(finalSequence, app);
    }

}
