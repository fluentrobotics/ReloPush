#include <GraphBuilder.hpp>
#include "InputParser.hpp"
#include <batchInstanceParcer.hpp>
#include <iostream>
#include <vector>
#include <PlanningContext.hpp>
#include <FileWriter.hpp>
#include <TaskAllocation.hpp>
#include <Visualization/VisualizeResults.h>
#include <chrono>


#ifdef __APPLE__
// Include the glog header when compiling on MacOS.
    #include <glog/logging.h>
#else
// Otherwise, include the Abseil logging header.
    #include "absl/log/initialize.h"
#endif

// ---------------------------------------------------------------------------
// Main Function
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    #ifdef __APPLE__
        // For macOS, initialize Google Logging with the program name.
        google::InitGoogleLogging(argv[0]);
    #else
        // For non-macOS systems, initialize Abseil Logging.
        absl::InitializeLog();
    #endif
    QApplication app(argc, argv);

    //std::string filename = "input_opt2obj.txt";
    std::string filename = "alpha_to_omega.txt";
    //std::string filename = "omega_to_alpha.txt";
    int instance_ind = 0;
    bool use_opt = false;
    bool vis = true;

    // Data to parse
    WorkspaceBoundary boundary(4,5.2); // todo: parse from file
    std::unordered_map<std::string, ObjectInfo> objects;
    std::unordered_map<std::string, GoalInfo>   goals;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
    std::vector<ReloPush::State> robots;


    if(argc > 3) // parse from arg
    {
        handle_args(argc, argv, filename, instance_ind, use_opt);
        vis = false; // disable for evaluations
    }

    Color::println("\n=== " + filename + " ind: " + std::to_string(instance_ind) + " ===",Color::GREEN);
    Color::println("Use Optimized PreRelocation? " + std::to_string(use_opt),Color::YELLOW);

    parse_instance_from_file(filename, instance_ind, objects, goals, robots, objGoalPairs);


    /*
    // 1) Parse and initialize
    if (!parseAndInitialize(filename, boundary, objects, goals, objGoalPairs))
    {
        return 1;
    }
    */


    auto start = std::chrono::high_resolution_clock::now();

    // 2) Perform the main planning/allocation loop
    std::vector<FinalAllocation> finalSequence;
    //bool ok = performAllocations(boundary, objects, goals, objGoalPairs, finalSequence, use_opt);
    GoalMap delivered_objs;
    bool ok = performAllocationsDFS(boundary, objects, goals, objGoalPairs, delivered_objs, robots[0] ,finalSequence, use_opt);

    auto end = std::chrono::high_resolution_clock::now();

    if(!ok)
    {
        // plan failed
        Color::println("Failed to find a solution",Color::YELLOW,Color::BG_RED);
        return -1;
    }

    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;


    // 3) Print the final sequence
    printFinalSequence(finalSequence);

    // 4) Visualization
    if (!finalSequence.empty() && vis)
    {
       visualizeResults(finalSequence, app);
    }

    writeFinalSequenceSummary(filename, instance_ind,
                              static_cast<double>(duration.count()), finalSequence, use_opt);



}
