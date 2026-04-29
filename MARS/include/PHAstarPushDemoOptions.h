#pragma once

#include <cstdint>
#include <string>

enum class ParkingCandidateMode
{
    RANDOM,
    EXPAND,
    CONNECTED,
    CONNECTED_VFH,
    REVERSE_RECENT,
};

struct RuntimeOptions
{
    bool debug_vis = false;
    bool robot_boundary_origin_only = true;
    ParkingCandidateMode parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT;
    bool enable_order_constraint_learning = true;
    bool has_fixed_random_seed = false;
    std::uint32_t base_random_seed = 0;
    int lns_threads = 5;
    int planner_expansion_threads = 4;
    int max_search_iterations = 2000;

    double default_xy_resolution = 0.1;
    double default_yaw_resolution = 0.523598775598298873; // pi / 6
    double default_time_step = 2.0;
    double default_rs_step_size = 0.2;
    int default_collision_steps = 2;
    double default_collision_check_time_step = 0.05;
    double default_max_steer = 0.523598775598298873; // pi / 6
    double default_analytic_threshold_scale = 5.0;
    double default_turn_penalty = 1.25;
    double default_reverse_penalty = 50.0;
    double default_switch_penalty = 1.2;
    double default_wait_penalty = 10.0;
    double default_max_time = 1000.0;
    double default_final_push_distance = 0.025;
    double default_inflation = 1.0;
    double default_safety_margin = 0.03;
    double default_robot_collision_inflation = 1.005;

    int fine_segment_max_search_iterations = 2000;
    double fine_segment_xy_resolution = 0.05;
    double fine_segment_yaw_resolution = 0.523598775598298873; // pi / 6
    double fine_segment_time_step = 1.0;
    double fine_segment_rs_step_size = 0.05;
    double fine_segment_collision_check_time_step = 0.05;

    int assignment_search_iterations = 0;
    int local_sequence_search_iterations = 0;
    int shuffle_sequence_search_iterations = 0;
    int lns_iterations = 0;
    bool enable_fine_segment_retry = true;
    bool enable_initial_transit_fallbacks = false;
    bool enable_lns_fine_segment_retry = true;
    bool enable_visualization = true;
    bool visualize_relopush_plan = false;
    bool integrated_mode = false;
    std::string handoff_endpoint = "tcp://127.0.0.1:5566";
    std::string input_sequence_path;
};
