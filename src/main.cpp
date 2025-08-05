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
#include <array>
#ifdef __APPLE__
// Include the glog header when compiling on MacOS.
    #include <glog/logging.h>
#else
// Otherwise, include the Abseil logging header.
    #include "absl/log/initialize.h"
#endif

#include <Visualization/TrajectoryView.h>
#include <base64.h>

enum planningSimOrReal {planOnly, sim, real};


// Assuming FinalAllocation, ReloPush::StatePath, etc. are visible
struct ActionRecord {
    int action_type;          // 0=transit, 1=push/obsRelo
    std::string object_name;
    std::string goal_name;
    double x, y, yaw;
};



// You can adapt this mapping code if you have existing indices in your objects/goals
std::unordered_map<std::string, int> make_index_map(const std::vector<FinalAllocation>& seq, bool object_map) {
    std::unordered_map<std::string, int> idx;
    int cur = 1; // visualizer expects indices starting at 1
    for (const auto& fa : seq) {
        const std::string& name = object_map ? fa.object.name : fa.goal.name;
        if (idx.count(name) == 0)
            idx[name] = cur++;
    }
    return idx;
}

void save_actions_for_visualizer(const std::vector<FinalAllocation>& finalSequence, const std::string& filename) {
    // Map names to indices
    auto obj_idx_map  = make_index_map(finalSequence, true);
    auto goal_idx_map = make_index_map(finalSequence, false);

    std::ofstream out(filename);
    out << "actions: [\n";


    for(const auto& fa : finalSequence)
    {
        int obj_idx = obj_idx_map[fa.object.name];
        int goal_idx = goal_idx_map[fa.goal.name];


        //first approach: type 0
        for (const auto& s : *(fa.firstApproachPath)) {
            out << "  [0," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
        }

        //obstacle relocation
        for (const auto& op : *(fa.obsReloPaths)) {
            std::string mode_str = "";
            if(op.is_pushing)
                mode_str = "1";
            else
                mode_str = "0";

            auto obsPath = op.toStatePath();

            for (const auto& s : *obsPath)
            {
                out << "  [" << mode_str <<"," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
            }
        }

        for (const auto& p : fa.paths)
        {
            // each edge
            // multiple if prerelocation
            for (size_t n=0; n<p.paths.size(); n++)
            {
                std::string mode_str = "";
                if(p.paths[n]->is_pushing)
                    mode_str = "1";
                else
                    mode_str = "0";

                auto edgePath = p.paths[n]->toStatePath();

                for (const auto& s : *edgePath)
                {
                    out << "  [" << mode_str <<"," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                }

                // transit between edges (exists sometimes)
                if(fa.edgeTransitPaths.size()>n && fa.edgeTransitPaths.size()>0)
                {
                    for (const auto& s : *(fa.edgeTransitPaths[n]))
                    {
                        out << "  [0," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                    }

                }
            }
        }

    }




    /*

    for (const auto& fa : finalSequence) {
        int obj_idx = obj_idx_map[fa.object.name];
        int goal_idx = goal_idx_map[fa.goal.name];

        // 1. firstApproachPath (action type 0)
        if (fa.firstApproachPath) {
            for (const auto& s : *(fa.firstApproachPath)) {
                out << "  [0," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
            }
        }

        // 2. obsReloPaths (action type 1)
        if (fa.obsReloPaths) {
            for (const auto& edgePath : *(fa.obsReloPaths)) {
                if (std::holds_alternative<ReloPush::StatePathPtr>(edgePath.path)) {
                    auto ptr = std::get<ReloPush::StatePathPtr>(edgePath.path);
                    for (const auto& s : *ptr) {
                        out << "  [1," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                    }
                }
            }
        }

        // 3. paths (action type 1)
        for (const auto& edgeData : fa.paths) {
            for (const auto& pathVariant : edgeData.paths) {
                if (std::holds_alternative<ReloPush::StatePathPtr>(pathVariant->path)) {
                    auto ptr = std::get<ReloPush::StatePathPtr>(pathVariant->path);
                    for (const auto& s : *ptr) {
                        out << "  [1," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                    }
                }
            }
        }

        // 4. transitPaths (action type 0)
        for (const auto& transitPtr : fa.transitPaths) {
            if (transitPtr) {
                for (const auto& s : *transitPtr) {
                    out << "  [0," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                }
            }
        }
    }

    */

    out << "]\n";
    out.close();
}


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

    //std::string filename = "alpha_to_omega_simp.txt";
    std::string filename = "iros_obj12.txt";

    int instance_ind = 78; // 63
    bool use_opt = true;
    bool vis = true;
    //bool sim = true;
    planningSimOrReal sim = planningSimOrReal::planOnly;

    // Data to parse
    WorkspaceBoundary boundary(4,5.2); // todo: parse from file
    ObjectMap objects, goals;
    //std::unordered_map<std::string, ObjectInfo>   goals;
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

    if(sim!=planningSimOrReal::real)
    {
        // init robot init pose
        ReloPush::trajectory_elem robot(robots[0].x,robots[0].y,robots[0].yaw,-1,-1,false);
        auto robot_str = "r!!!"+robot.serialize();
        std::string encoded_data_robot = base64_encode(reinterpret_cast<const unsigned char*>(robot_str.c_str()), robot_str.length());
        if(sim == planningSimOrReal::sim)
        {
            mqClient.send_and_wait(encoded_data_robot); //todo: gen message properly
        }
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

    if(sim!=planningSimOrReal::planOnly)
    {
        // send objects for vis
        auto obj_vis = std::string("o!!!");
        std::vector<std::string> strs;
        for(auto& it : objects)
        {
            std::string temp="";
            temp+=float2binarystr(it.second.x);
            temp+=",,,";
            temp+=float2binarystr(it.second.y);
            temp+=",,,";
            temp+=float2binarystr(it.second.nominalOrientation);
            strs.push_back(temp);
        }

        for(int n=0; n<strs.size(); n++)
        {
            obj_vis+=strs[n];
            if(n!=strs.size()-1)
                obj_vis+=";;;";
        }
        auto obj_vis_msg = base64_encode(reinterpret_cast<const unsigned char*>(obj_vis.c_str()), obj_vis.length());
        mqClient.send_and_wait(obj_vis_msg);

        // send goal for vis
        auto goal_vis = std::string("g!!!");
        strs.clear();
        for(auto& it : goals)
        {
            std::string temp="";
            temp+=float2binarystr(it.second.x);
            temp+=",,,";
            temp+=float2binarystr(it.second.y);
            temp+=",,,";
            temp+=float2binarystr(it.second.nominalOrientation);
            strs.push_back(temp);
        }

        for(int n=0; n<strs.size(); n++)
        {
            goal_vis+=strs[n];
            if(n!=strs.size()-1)
                goal_vis+=";;;";
        }
        auto goal_vis_msg = base64_encode(reinterpret_cast<const unsigned char*>(goal_vis.c_str()), goal_vis.length());
        mqClient.send_and_wait(goal_vis_msg);
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
    bool ok = performAllocationsDFS(boundary, objects, goals, objGoalPairs, delivered_objs, robots[0] ,finalSequence, use_opt, start);

    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;

    bool timeout = false;
    if(duration.count() > 120000)
        timeout = true;

    double total_path_length = 0.0;
    double total_pushing_length = 0.0;

    if(timeout)
        total_path_length = -1;

    if(ok)
    {
        // 3) Print the final sequence
        printFinalSequence(finalSequence);

        // 4) Visualization
        if (!finalSequence.empty() && vis)
        {
           visualizeResults(finalSequence, app);
        }

        //writeFinalSequenceSummary(filename, instance_ind,
        //                          static_cast<double>(duration.count()), finalSequence, use_opt);


        // 3.5) Print total path length and total pushing length for the entire solution


        // Use the combined path for each allocation to get the length.
        for (auto& fa : finalSequence) {
            // Reconstruct the whole path for this allocation
            ReloPush::StatePathPtr singlePathPtr;
            std::vector<size_t> si;

            /*
            if(fa.firstApproachPath && !fa.firstApproachPath->empty())
            {
                for(size_t i=1; i<fa.firstApproachPath->size(); ++i)
                {
                    const auto& prev = fa.firstApproachPath->at(i - 1);
                    const auto& curr = fa.firstApproachPath->at(i);
                    double dx = curr.x - prev.x;
                    double dy = curr.y - prev.y;
                    total_path_length += std::sqrt(dx * dx + dy * dy);
                }
            }*/


            std::tie(singlePathPtr, si) = fa.toSinglePathPtr(0.1); // Or use your default resolution



            if (singlePathPtr && !singlePathPtr->empty()) {
                // Sum up Euclidean distances
                for (size_t i = 1; i < singlePathPtr->size(); ++i) {
                    const auto& prev = singlePathPtr->at(i - 1);
                    const auto& curr = singlePathPtr->at(i);
                    double dx = curr.x - prev.x;
                    double dy = curr.y - prev.y;
                    total_path_length += std::sqrt(dx * dx + dy * dy);
                }
            }

            // Pushing length as reported by the object
            total_pushing_length += fa.getPushingLength();
        }
    }
    else
    {
        if(timeout)
            Color::println("Timed out",Color::YELLOW,Color::BG_RED);
        else
        // plan failed
            Color::println("Failed to find a solution",Color::YELLOW,Color::BG_RED);
        //return -1;
    }

    std::cout << "=== Solution Summary ===" << std::endl;
    std::cout << "Total path length (all movements): " << total_path_length << std::endl;
    std::cout << "Total pushing length: " << total_pushing_length << std::endl;
    std::cout << "Planning time(s): " << (float)duration.count()/1000 << std::endl;


    // Compose output filename
    std::string result_filename = std::string(CMAKE_SOURCE_DIR) + "/results/result_" + filename;
    if(use_opt)
        result_filename = std::string(CMAKE_SOURCE_DIR) + "/results/result_opt_" + filename;

    std::cout << "saving to: "<< result_filename << std::endl;

    // Open for appending (if you run many instances), or for writing (overwrite)
    std::ofstream outfile(result_filename.c_str(), std::ios::app);

    // Write in required format, with fixed precision
    outfile << "===\n";
    outfile << "index:" << instance_ind << "\n";
    outfile << "planning_time(s):" << (float)duration.count()/1000 << "\n"; // left empty
    outfile << "total_length(m):" << std::fixed << std::setprecision(6) << total_path_length << "\n";
    outfile << "pushing_length(m):" << std::fixed << std::setprecision(6) << total_pushing_length << "\n";
    outfile.close();


    // generate resulting trajectory
    auto finalTrajectory = FA2Trajectory(finalSequence);


   //  auto actions = extractActionSequence(finalSequence);
   // for (const auto& act : actions) {
   //     std::cout << "[" << act.action_type
   //               << "," << act.object_name
   //               << "," << act.goal_name
   //               << "," << act.x
   //               << "," << act.y
   //               << "," << act.yaw << "],\n"<< std::flush;;
   // }

    //finalTrajectory.print();



    save_actions_for_visualizer(finalSequence,std::string(CMAKE_SOURCE_DIR) + "/result_opt_actions_" + filename);


    /*
    //QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Trajectory Visualization (Arrow Format)");
    window.resize(800, 600);

    TrajectoryView* view = new TrajectoryView();
    window.setCentralWidget(view);

    // Create a trajectory object and fill it with sample data.
    // Here we use similar data as before, with time stamps (in ms).

    view->setTrajectory(finalTrajectory);

    //window.show();


    // Allow time for the previous request to end
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    */

    // send trajectory
    if(sim!=planningSimOrReal::planOnly)
    {
        auto s = finalTrajectory.serialize();
        std::string encoded_data = base64_encode(reinterpret_cast<const unsigned char*>(s.c_str()), s.length());
        //for debug
        //std::cout << encoded_data.size() << std::endl;
        auto res = mqClient.send_and_wait(encoded_data);
        //std::cout << res << std::endl; // response from server
    }


    //return app.exec();
    return 0;
}
