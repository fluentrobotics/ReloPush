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
#include <thread>
#include <trajectory.hpp>
#ifdef __APPLE__
// Include the glog header when compiling on MacOS.
    #include <glog/logging.h>
#else
// Otherwise, include the Abseil logging header.
    #include "absl/log/initialize.h"
#endif

#include <Visualization/TrajectoryView.h>
#include <base64.h>


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
    bool sim = false;

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

    // send via zeromq
    zeromp_object mqClient;
    #ifdef __APPLE__
            // For macOS, initialize Google Logging with the program name.
        mqClient.connect("tcp://192.168.1.13:5555");
    std::cout << "APPLE" << std::endl;
    #else
            // For non-macOS systems, initialize Abseil Logging.
        mqClient.connect();
    #endif

    if(sim)
    {
        // init robot init pose
        ReloPush::trajectory_elem robot(robots[0].x,robots[0].y,robots[0].yaw,-1,-1,false);
        auto robot_str = "r!!!"+robot.serialize();
        std::string encoded_data_robot = base64_encode(reinterpret_cast<const unsigned char*>(robot_str.c_str()), robot_str.length());
        mqClient.send_and_wait(encoded_data_robot); //todo: gen message properly
    }
    else // real robot. get pose from ros bridge
    {
        auto req = std::string("l!!!");
        auto req_msg = base64_encode(reinterpret_cast<const unsigned char*>(req.c_str()), req.length());
        auto r_str = mqClient.send_and_wait(req_msg);
        auto r_dec = base64_decode(r_str,false);
        auto robot = ReloPush::trajectory_elem(r_dec);
        // For now, assume there is only one robot
        robots[0].x = robot.x;
        robots[0].y = robot.y;
        robots[0].yaw = robot.yaw;
        std::cout << "Robot at: " << robot.x << ", " << robot.y << ", " << robot.yaw << std::endl;
    }


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
       //visualizeResults(finalSequence, app);
    }

    //writeFinalSequenceSummary(filename, instance_ind,
    //                          static_cast<double>(duration.count()), finalSequence, use_opt);

    // generate resulting trajectory
    auto finalTrajectory = FA2Trajectory(finalSequence);
    //QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Trajectory Visualization (Arrow Format)");
    window.resize(800, 600);

    TrajectoryView* view = new TrajectoryView();
    window.setCentralWidget(view);

    // Create a trajectory object and fill it with sample data.
    // Here we use similar data as before, with time stamps (in ms).

    view->setTrajectory(finalTrajectory);

    window.show();


    // Allow time for the previous request to end
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // send trajectory
    auto s = finalTrajectory.serialize();
    std::string encoded_data = base64_encode(reinterpret_cast<const unsigned char*>(s.c_str()), s.length());
    //for debug
    std::cout << encoded_data.size() << std::endl;
    auto res = mqClient.send_and_wait(encoded_data);
    std::cout << res << std::endl; // response from server

    return app.exec();
}
