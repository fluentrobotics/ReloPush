#include <ReloPush/GraphBuilder.hpp>
#include "ReloPush/InputParser.hpp"
#include <cstdlib>
#include <ReloPush/batchInstanceParcer.hpp>
#include <iostream>
#include <vector>
#include <ReloPush/PlanningContext.hpp>
#include <ReloPush/FileWriter.hpp>
#include <ReloPush/TaskAllocation.hpp>
#include <ReloPush/Visualization/VisualizeResults.h>
#include <chrono>
#include <filesystem>
#include <thread>
#include <ReloPush/trajectory.hpp>
#include <array>
#include <SerializeFinalSequence.h>
#include <ReloPush/FinalSequenceHandoff.h>
#include <ReloPush/ReloPushBossDiagnostics.hpp>

#if defined(__APPLE__) || defined(__linux__)
// Include glog on platforms where it is available from the build dependencies.
#include <glog/logging.h>
#else
// Other platforms can use Abseil logging if available.
#include "absl/log/initialize.h"
#endif

#include <ReloPush/Visualization/TrajectoryView.h>
#include <ReloPush/base64.h>

enum planningSimOrReal
{
    planOnly,
    sim,
    real
};

namespace
{
    struct MarsIntegrationOptions
    {
        bool enabled = false;
        std::string endpoint = ReloPush::kDefaultMarsHandoffEndpoint;
    };

    bool is_option_arg(const char *arg)
    {
        return arg != nullptr && arg[0] == '-';
    }

    bool has_batch_instance_args(int argc, char *argv[])
    {
        return argc > 3 &&
               !is_option_arg(argv[1]) &&
               !is_option_arg(argv[2]) &&
               !is_option_arg(argv[3]);
    }

    struct WorkspaceOverrideOptions
    {
        bool has_x = false;
        bool has_y = false;
        double x = 0.0;
        double y = 0.0;
    };

    // Scans argv for optional --workspace-x=<float> / --workspace-y=<float>
    // flags, records any values found, and builds filtered_argv with those
    // flags removed so downstream positional-arg parsing (has_batch_instance_args
    // / handle_args) never sees them, regardless of where they appear on the
    // command line. When neither flag is present, filtered_argv is just a copy
    // of argv and behavior is unchanged.
    WorkspaceOverrideOptions parse_and_strip_workspace_override(
        int argc, char *argv[], std::vector<char *> &filtered_argv)
    {
        WorkspaceOverrideOptions options;
        filtered_argv.clear();
        filtered_argv.push_back(argv[0]);

        const std::string x_prefix = "--workspace-x=";
        const std::string y_prefix = "--workspace-y=";

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i] ? argv[i] : "";
            if (arg.rfind(x_prefix, 0) == 0)
            {
                options.has_x = true;
                options.x = std::atof(arg.substr(x_prefix.size()).c_str());
            }
            else if (arg.rfind(y_prefix, 0) == 0)
            {
                options.has_y = true;
                options.y = std::atof(arg.substr(y_prefix.size()).c_str());
            }
            else
            {
                filtered_argv.push_back(argv[i]);
            }
        }

        return options;
    }

    MarsIntegrationOptions parse_mars_integration_options(int argc, char *argv[])
    {
        MarsIntegrationOptions options;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i] ? argv[i] : "";
            if (arg == "--integrated-mode" ||
                arg == "--integrated-mars")
            {
                options.enabled = true;
            }
            else if (arg.rfind("--mars-endpoint=", 0) == 0)
            {
                options.endpoint = arg.substr(std::string("--mars-endpoint=").size());
            }
            else if (arg.rfind("--handoff-endpoint=", 0) == 0)
            {
                options.endpoint = arg.substr(std::string("--handoff-endpoint=").size());
            }
        }

        return options;
    }

    // Parse --robot-boundary-mode=corners|origin flag from argv
    // Returns "corners" or "origin" (default)
    std::string parse_robot_boundary_mode(int argc, char *argv[])
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i] ? argv[i] : "";
            if (arg.rfind("--robot-boundary-mode=", 0) == 0)
            {
                return arg.substr(std::string("--robot-boundary-mode=").size());
            }
        }
        return "origin";  // default
    }
} // namespace

static long relopush_timeout_ms() {
    static long v = [](){
        const char* s = std::getenv("RELOPUSH_TIMEOUT_MS");
        long x = s ? std::atol(s) : 0;
        return x > 0 ? x : 180000L;
    }();
    return v;
}

// Function to save finalSequence to a file using base64-encoded binary data
void saveFinalSequenceToFile(const std::vector<FinalAllocation> &finalSequence, const std::string &filename, bool diag_disabled = false)
{
    const std::string binaryData = serializeFinalSequence(finalSequence);
    const std::string base64Data =
        base64_encode(reinterpret_cast<const unsigned char *>(binaryData.data()),
                      binaryData.size());
    if (!diag_disabled)
    {
        ReloPushBossDiagnostics::log_serialized_final_sequence(
            finalSequence, binaryData, base64Data, filename);
    }

    std::ofstream outFile(filename);
    if (outFile)
    {
        outFile << base64Data;
        outFile.close();
        std::cout << "Saved finalSequence to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
    }
}

// Assuming FinalAllocation, ReloPush::StatePath, etc. are visible
struct ActionRecord
{
    int action_type; // 0=transit, 1=push/obsRelo
    std::string object_name;
    std::string goal_name;
    double x, y, yaw;
};

// You can adapt this mapping code if you have existing indices in your objects/goals
std::unordered_map<std::string, int> make_index_map(const std::vector<FinalAllocation> &seq, bool object_map)
{
    std::unordered_map<std::string, int> idx;
    int cur = 1; // visualizer expects indices starting at 1
    for (const auto &fa : seq)
    {
        const std::string &name = object_map ? fa.object.name : fa.goal.name;
        if (idx.count(name) == 0)
            idx[name] = cur++;
    }
    return idx;
}

void save_actions_for_visualizer(const std::vector<FinalAllocation> &finalSequence, const std::string &filename)
{
    // Map names to indices
    auto obj_idx_map = make_index_map(finalSequence, true);
    auto goal_idx_map = make_index_map(finalSequence, false);

    std::ofstream out(filename);
    out << "actions: [\n";

    for (const auto &fa : finalSequence)
    {
        int obj_idx = obj_idx_map[fa.object.name];
        int goal_idx = goal_idx_map[fa.goal.name];

        // first approach: type 0
        for (const auto &s : *(fa.firstApproachPath))
        {
            out << "  [0," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
        }

        // obstacle relocation
        for (const auto &op : *(fa.obsReloPaths))
        {
            std::string mode_str = "";
            if (op.is_pushing)
                mode_str = "1";
            else
                mode_str = "0";

            auto obsPath = op.toStatePath();

            for (const auto &s : *obsPath)
            {
                out << "  [" << mode_str << "," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
            }
        }

        // for (const auto& p : fa.paths)
        for (size_t i = 0; i < fa.paths.size(); i++)
        {
            auto &p = fa.paths[i];
            // each edge
            // multiple if prerelocation
            for (size_t n = 0; n < p.paths.size(); n++)
            {
                std::string mode_str = "";
                if (p.paths[n]->is_pushing)
                    mode_str = "1";
                else
                    mode_str = "0";

                auto edgePath = p.paths[n]->toStatePath();

                for (const auto &s : *edgePath)
                {
                    out << "  [" << mode_str << "," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                }
            }
            // transit between edges (exists sometimes)
            if (fa.edgeTransitPaths.size() > i && fa.edgeTransitPaths.size() > 0)
            {
                for (const auto &s : *(fa.edgeTransitPaths[i]))
                {
                    out << "  [0," << obj_idx << "," << goal_idx << "," << s.x << "," << s.y << "," << s.yaw << "],\n";
                }
            }
        }
    }

    out << "]\n";
    out.close();
}

// ---------------------------------------------------------------------------
// Main Function
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // Use glog on Linux and macOS; the installed Abseil package here does not
    // ship the logging headers this code path previously relied on.
    google::InitGoogleLogging(argv[0]);
    QApplication app(argc, argv);

    // Check if diagnostics are disabled via environment variable
    bool diag_disabled = (std::getenv("RELOPUSH_DISABLE_DIAGNOSTICS") &&
                         std::string(std::getenv("RELOPUSH_DISABLE_DIAGNOSTICS")) == "1");

    std::string filename = "ReloPush-BOSS_12_objects.txt";

    int instance_ind = 4; // 32 // 63 //6 //40 //8
    bool use_opt = true;
    bool vis = true;
    bool no_init_guess = false;

    bool use_dfs = true;

    // bool sim = true;
    planningSimOrReal sim = planningSimOrReal::planOnly;

    // Strip optional --workspace-x=/--workspace-y= flags before any positional
    // or other flag parsing sees argv, so their presence/position cannot
    // affect existing argument handling.
    std::vector<char *> filtered_argv;
    const WorkspaceOverrideOptions workspace_override =
        parse_and_strip_workspace_override(argc, argv, filtered_argv);
    int filtered_argc = static_cast<int>(filtered_argv.size());

    const MarsIntegrationOptions mars_integration =
        parse_mars_integration_options(filtered_argc, filtered_argv.data());
    ReloPush::HandoffInstanceInfo handoff_instance_info;

    // Parse robot boundary mode (corners or origin)
    std::string boundary_mode = parse_robot_boundary_mode(filtered_argc, filtered_argv.data());
    if (boundary_mode == "corners")
    {
        params::robot_boundary_corners = true;
    }
    else
    {
        params::robot_boundary_corners = false;
    }

    // Data to parse
    ObjectMap objects, goals;
    // std::unordered_map<std::string, ObjectInfo>   goals;
    std::unordered_map<std::string, ObjectGoalPair> objGoalPairs;
    std::vector<ReloPush::State> robots;

    if (has_batch_instance_args(filtered_argc, filtered_argv.data())) // parse legacy positional batch args
    {
        handle_args(filtered_argc, filtered_argv.data(), filename, instance_ind, use_opt, no_init_guess, use_dfs); // 3rd arg: mode. 'f'=ReloPush-F 'd' = ReloPush-D 'u'=no-init-opt 'o'=ReloPush
        vis = false;                                                                                               // disable for evaluations
    }

    if (mars_integration.enabled)
    {
        std::cout << "[Integration] MARS handoff enabled at "
                  << mars_integration.endpoint << std::endl;
    }

    Color::println("\n=== " + filename + " ind: " + std::to_string(instance_ind) + " ===", Color::GREEN);
    Color::println("Use Optimized PreRelocation? " + std::to_string(use_opt), Color::YELLOW);
    handoff_instance_info.file_name = filename;
    handoff_instance_info.instance_index = instance_ind;

    std::string selected_instance_line;
    try
    {
        const auto input_lines = read_file(filename);
        if (instance_ind >= 0 &&
            static_cast<std::size_t>(instance_ind) < input_lines.size())
        {
            selected_instance_line =
                input_lines[static_cast<std::size_t>(instance_ind)];
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "[ReloPushBossDiag] Could not pre-read selected input line: "
                  << ex.what() << std::endl;
    }
    const std::string input_abs_path =
        std::string(CMAKE_SOURCE_DIR) + "/input/" + filename;
    if (!diag_disabled)
    {
        ReloPushBossDiagnostics::initialize(
            filename,
            instance_ind,
            use_opt,
            no_init_guess,
            use_dfs,
            input_abs_path,
            selected_instance_line);
    }

    bool file_has_ws = false;
    double file_ws_x = 0.0;
    double file_ws_y = 0.0;
    parse_instance_from_file(filename, instance_ind, objects, goals, robots, objGoalPairs,
                              &file_has_ws, &file_ws_x, &file_ws_y);
    if (!diag_disabled)
    {
        ReloPushBossDiagnostics::log_parsed_input(objects, goals, robots, objGoalPairs);
    }

    // Resolve effective workspace boundary: CLI flag > file `ws:` section > legacy default.
    const double workspace_x = workspace_override.has_x
        ? workspace_override.x
        : (file_has_ws ? file_ws_x : 4.0);
    const double workspace_y = workspace_override.has_y
        ? workspace_override.y
        : (file_has_ws ? file_ws_y : 5.2);
    const char *workspace_source =
        (workspace_override.has_x || workspace_override.has_y) ? "CLI override" :
        (file_has_ws ? "instance file" : "default");
    std::cout << "[Workspace] " << workspace_x << " x " << workspace_y
              << " (from " << workspace_source << ")" << std::endl;
    std::cout << "[Boundary] robot boundary mode: " << boundary_mode << std::endl;
    WorkspaceBoundary boundary(workspace_x, workspace_y);

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

    if (sim != planningSimOrReal::real)
    {
        // init robot init pose
        ReloPush::trajectory_elem robot(robots[0].x, robots[0].y, robots[0].yaw, -1, -1, false);
        auto robot_str = "r!!!" + robot.serialize();
        std::string encoded_data_robot = base64_encode(reinterpret_cast<const unsigned char *>(robot_str.c_str()), robot_str.length());
        if (sim == planningSimOrReal::sim)
        {
            mqClient.send_and_wait(encoded_data_robot); // todo: gen message properly
        }
    }
    else // real robot. get pose from ros bridge
    {
        auto req = std::string("l!!!");
        auto req_msg = base64_encode(reinterpret_cast<const unsigned char *>(req.c_str()), req.length());
        auto r_str = mqClient.send_and_wait(req_msg);
        auto r_dec = base64_decode(r_str, false);
        auto robot = ReloPush::trajectory_elem(r_dec);
        // For now, assume there is only one robot
        robots[0].x = robot.x;
        robots[0].y = robot.y;
        robots[0].yaw = robot.yaw;
        std::cout << "Robot at: " << robot.x << ", " << robot.y << ", " << robot.yaw << std::endl;
    }

    if (sim != planningSimOrReal::planOnly)
    {
        // send objects for vis
        auto obj_vis = std::string("o!!!");
        std::vector<std::string> strs;
        for (auto &it : objects)
        {
            std::string temp = "";
            temp += float2binarystr(it.second.x);
            temp += ",,,";
            temp += float2binarystr(it.second.y);
            temp += ",,,";
            temp += float2binarystr(it.second.nominalOrientation);
            strs.push_back(temp);
        }

        for (int n = 0; n < strs.size(); n++)
        {
            obj_vis += strs[n];
            if (n != strs.size() - 1)
                obj_vis += ";$;";
        }
        auto obj_vis_msg = base64_encode(reinterpret_cast<const unsigned char *>(obj_vis.c_str()), obj_vis.length());
        mqClient.send_and_wait(obj_vis_msg);

        // send goal for vis
        auto goal_vis = std::string("g!!!");
        strs.clear();
        for (auto &it : goals)
        {
            std::string temp = "";
            temp += float2binarystr(it.second.x);
            temp += ",,,";
            temp += float2binarystr(it.second.y);
            temp += ",,,";
            temp += float2binarystr(it.second.nominalOrientation);
            strs.push_back(temp);
        }

        for (int n = 0; n < strs.size(); n++)
        {
            goal_vis += strs[n];
            if (n != strs.size() - 1)
                goal_vis += ";$;";
        }
        auto goal_vis_msg = base64_encode(reinterpret_cast<const unsigned char *>(goal_vis.c_str()), goal_vis.length());
        mqClient.send_and_wait(goal_vis_msg);
    }

    auto start = std::chrono::high_resolution_clock::now();

    // 2) Perform the main planning/allocation loop
    std::vector<FinalAllocation> finalSequence;
    // bool ok = performAllocations(boundary, objects, goals, objGoalPairs, finalSequence, use_opt);
    GoalMap delivered_objs;
    bool ok;

    if (use_dfs)
    {
        ok = performAllocationsDFS(boundary, objects, goals, objGoalPairs, delivered_objs, robots[0],
                                   finalSequence, use_opt, no_init_guess, start);
    }
    else
    {
        ok = performAllocations(boundary, objects, goals, objGoalPairs, delivered_objs, robots[0],
                                finalSequence, use_opt, no_init_guess, start);
    }
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the elapsed time in milliseconds
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;

    bool timeout = false;
    if (duration.count() > (2 * relopush_timeout_ms()) / 3) // default 120 seconds (2/3 of the 180s solve deadline), overridable via RELOPUSH_TIMEOUT_MS
        timeout = true;

    double total_path_length = 0.0;
    double total_pushing_length = 0.0;

    if (timeout)
        total_path_length = -1;

    if (ok)
    {
        // 3) Print the final sequence
        printFinalSequence(finalSequence);

        // 4) Visualization
        if (!finalSequence.empty() && vis)
        {
            visualizeResults(finalSequence, app);
        }

        // Use the combined path for each allocation to get the length.
        for (auto &fa : finalSequence)
        {
            // Reconstruct the whole path for this allocation
            ReloPush::StatePathPtr singlePathPtr;
            std::vector<size_t> si;

            std::tie(singlePathPtr, si) = fa.toSinglePathPtr(0.1); // Or use your default resolution

            if (singlePathPtr && !singlePathPtr->empty())
            {
                // Sum up Euclidean distances
                for (size_t i = 1; i < singlePathPtr->size(); ++i)
                {
                    const auto &prev = singlePathPtr->at(i - 1);
                    const auto &curr = singlePathPtr->at(i);
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
        if (timeout)
            Color::println("Timed out", Color::YELLOW, Color::BG_RED);
        else
            // plan failed
            Color::println("Failed to find a solution", Color::YELLOW, Color::BG_RED);
        // return -1;
    }

    std::cout << "=== Solution Summary ===" << std::endl;
    std::cout << "Total path length (all movements): " << total_path_length << std::endl;
    std::cout << "Total pushing length: " << total_pushing_length << std::endl;
    std::cout << "Planning time(s): " << (float)duration.count() / 1000 << std::endl;

    int n_obsRelo = 0;
    int n_preRelo = 0;
    for (auto &it : finalSequence)
    {
        n_obsRelo += it.countObsRelo();
        n_preRelo += it.countPreRelo();
    }

    // Compose output filename
    std::string result_filename = std::string(CMAKE_SOURCE_DIR) + "/results/result_" + filename;
    if (!use_dfs) // not using dfs
    {
        result_filename = std::string(CMAKE_SOURCE_DIR) + "/results/result_prevReloPush_" + filename;
    }
    if (use_opt)
    {
        if (!no_init_guess)
            result_filename = std::string(CMAKE_SOURCE_DIR) + "/results/result_opt_" + filename;
        else
            result_filename = std::string(CMAKE_SOURCE_DIR) + "/results/result_opt_no_init_" + filename;
    }

    std::filesystem::create_directories(std::string(CMAKE_SOURCE_DIR) + "/results");
    std::cout << "saving to: " << result_filename << std::endl;

    // Open for appending (if you run many instances), or for writing (overwrite)
    std::ofstream outfile(result_filename.c_str(), std::ios::app);

    // Write in required format, with fixed precision
    outfile << "===\n";
    outfile << "index:" << instance_ind << "\n";
    outfile << "planning_time(s):" << (float)duration.count() / 1000 << "\n"; // left empty
    outfile << "total_length(m):" << std::fixed << std::setprecision(6) << total_path_length << "\n";
    outfile << "pushing_length(m):" << std::fixed << std::setprecision(6) << total_pushing_length << "\n";
    outfile << "obs_relocations:" << std::fixed << n_obsRelo << "\n";
    outfile << "pre_relocations:" << std::fixed << n_preRelo << "\n";
    outfile.close();

    if (!diag_disabled)
    {
        ReloPushBossDiagnostics::log_planning_outcome(
            ok,
            timeout,
            duration.count(),
            total_path_length,
            total_pushing_length,
            n_obsRelo,
            n_preRelo,
            finalSequence);
    }

    if (!ok)
    {
        if (mars_integration.enabled)
        {
            try
            {
                ReloPush::FinalSequenceHandoffClient handoff_client;
                handoff_client.connect(mars_integration.endpoint);
                const std::string reply = handoff_client.sendAbortAndWaitForReply(
                    handoff_instance_info,
                    "ReloPush planning failed before a final sequence was produced");
                std::cout << "[Integration] MARS reply: " << reply << std::endl;
            }
            catch (const std::exception &ex)
            {
                std::cerr << "[Integration] Failed to notify MARS about the planning failure: "
                          << ex.what() << std::endl;
            }
        }

        if (!diag_disabled)
        {
            ReloPushBossDiagnostics::finish();
        }
        return 1;
    }

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

    // finalTrajectory.print();

    // save_actions_for_visualizer(finalSequence,std::string(CMAKE_SOURCE_DIR) + "/result_actions_" + filename);

    // QApplication app(argc, argv);
    const bool show_trajectory_window = false; // vis && !mars_integration.enabled;

    QMainWindow window;
    if (show_trajectory_window)
    {
        window.setWindowTitle("Trajectory Visualization (Arrow Format)");
        window.resize(800, 600);

        TrajectoryView *view = new TrajectoryView();
        window.setCentralWidget(view);
        view->setTrajectory(finalTrajectory);
        window.show();
    }

    // Allow time for the previous request to end
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // send trajectory
    if (sim != planningSimOrReal::planOnly)
    {
        auto s = finalTrajectory.serialize();
        std::string encoded_data = base64_encode(reinterpret_cast<const unsigned char *>(s.c_str()), s.length());
        // for debug
        // std::cout << encoded_data.size() << std::endl;
        auto res = mqClient.send_and_wait(encoded_data);
        // std::cout << res << std::endl; // response from server
    }

    if (mars_integration.enabled)
    {
        try
        {
            const std::string handoff_binary = serializeFinalSequence(finalSequence);
            const std::string handoff_base64 =
                base64_encode(reinterpret_cast<const unsigned char *>(handoff_binary.data()),
                              handoff_binary.size());
            if (!diag_disabled)
            {
                ReloPushBossDiagnostics::log_serialized_final_sequence(
                    finalSequence, handoff_binary, handoff_base64, "MARS handoff");
            }

            ReloPush::FinalSequenceHandoffClient handoff_client;
            handoff_client.connect(mars_integration.endpoint);
            std::cout << "[Integration] Sending final sequence to MARS at "
                      << mars_integration.endpoint << std::endl;

            const std::string reply =
                handoff_client.sendFinalSequenceAndWaitForReply(
                    handoff_instance_info,
                    finalSequence);
            std::cout << "[Integration] MARS reply: " << reply << std::endl;

            if (!ReloPush::isMarsSuccessReply(reply))
            {
                std::cerr << "[Integration] MARS reported a failure while processing the handed-off sequence."
                          << std::endl;
                if (!diag_disabled)
                {
                    ReloPushBossDiagnostics::finish();
                }
                return 1;
            }
        }
        catch (const std::exception &ex)
        {
            std::cerr << "[Integration] Failed to hand off the final sequence to MARS: "
                      << ex.what() << std::endl;
            if (!diag_disabled)
            {
                ReloPushBossDiagnostics::finish();
            }
            return 1;
        }
    }
    else
    {
        const std::string relopush_out_dir =
            std::string(CMAKE_SOURCE_DIR) + "/results/relopush-out";
        std::filesystem::create_directories(relopush_out_dir);
        saveFinalSequenceToFile(
            finalSequence,
            relopush_out_dir + "/result_seq_" + filename + "_ind" + std::to_string(instance_ind) + ".b64",
            diag_disabled);
    }

    if (show_trajectory_window)
    {
        if (!diag_disabled)
        {
            ReloPushBossDiagnostics::finish();
        }
        return app.exec();
    }

    if (!diag_disabled)
    {
        ReloPushBossDiagnostics::finish();
    }
    return 0;
}
