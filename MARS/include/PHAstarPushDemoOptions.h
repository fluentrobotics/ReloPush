#pragma once

#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

enum class ParkingCandidateMode
{
    RANDOM,
    EXPAND,
    CONNECTED,
    CONNECTED_VFH,
    REVERSE_RECENT,
    REVERSE_RECENT_SHORTER,
};

struct RuntimeOptions
{
    // visualization options
    bool debug_vis = true;
    bool visualize_relopush_plan = true;
    bool enable_visualization = true; // show results

    // Default planning parameters
    double default_xy_resolution = 0.2;
    double default_yaw_resolution = 0.523598775598298873; // pi / 6
    double default_time_step = 2.5;
    double default_rs_step_size = 0.3;
    int default_collision_steps = 2;
    double default_collision_check_time_step = 0.05;
    double default_max_steer = 0.523598775598298873; // pi / 6
    double default_analytic_threshold_scale = 5.0;
    double default_turn_penalty = 1.25;
    double default_reverse_penalty = 10.0; // 50
    double default_switch_penalty = 1.2;
    double default_wait_penalty = 10.0;
    double default_max_time = 1000.0;
    double default_final_push_distance = 0.025;
    double default_inflation = 1.0;
    double default_safety_margin = 0.03;
    double default_robot_collision_inflation = 1.005;
    double retraction_distance = 0.11;
    bool enable_initial_transit_fallbacks = false;
    int planner_expansion_threads = 2;
    int max_search_iterations = 500;
    bool robot_boundary_origin_only = true;

    // Fine segment options (for replanning)
    int fine_segment_max_search_iterations = 0;
    double fine_segment_xy_resolution = 0.1;
    double fine_segment_yaw_resolution = 0.523598775598298873; // pi / 6
    double fine_segment_time_step = 1.6;
    double fine_segment_rs_step_size = 0.16;
    double fine_segment_collision_check_time_step = 0.05;
    bool enable_reverse_escape_retries = false;
    bool enable_contact_boundary_geometric_retry = true;
    int contact_boundary_max_search_iterations = 500;
    double contact_boundary_xy_resolution = 0.08;   // 0.08
    double contact_boundary_yaw_resolution = 0.235; // 0.26; // 0.235
    double contact_boundary_time_step = 1.0;
    double contact_boundary_rs_step_size = 0.10;
    double contact_boundary_reverse_penalty = 2.0;
    double contact_boundary_holonomic_resolution = 0.10;

    // E-Graph
    bool enable_reference_egraph_transit = true; // E-Graphs-inspired heuristic
    bool anchor_first_contact_segment_transit = true;
    double reference_egraph_epsilon = 10.0;
    double reference_egraph_waypoint_spacing = 0.10;
    double reference_egraph_snap_radius = 0.25;
    double reference_egraph_snap_yaw = M_PI / 3.0;
    int reference_egraph_successor_lookahead = 3;
    int reference_egraph_max_nodes = 120;

    // Safe Parking
    ParkingCandidateMode parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT_SHORTER;
    bool enable_failed_candidate_idle_parking = true;
    int failed_candidate_initial_transit_failure_threshold = 2;
    bool enable_order_constraint_learning = true;
    bool has_fixed_random_seed = false;
    std::uint32_t base_random_seed = 0;

    // Assignment Refinement options
    int assignment_search_iterations = 0; // for random assignment
    int local_sequence_search_iterations = 0;
    int shuffle_sequence_search_iterations = 0;
    int lns_iterations = 0;
    int lns_threads = 5;
    bool enable_fine_segment_retry = true;     // in case initial plan fails in greedy assignment
    bool enable_lns_fine_segment_retry = true; // for LNS

    // for figure generation
    bool enable_result_summary_figure = false;
    std::string result_summary_output_path;
    double result_summary_subplot_gap = 36.0;
    std::vector<std::string> robot_trace_colors = {
        "#70A288",
        "#DAB785",
        "#D5896F",
        "#CC79A7",
        "#E69F00",
        "#56B4E9",
    };

    // ZeroMQ
    bool integrated_mode = false;
    std::string handoff_endpoint = "tcp://127.0.0.1:5566";
    std::string input_sequence_path;
};
