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
    bool debug_vis = true;
    bool robot_boundary_origin_only = true;
    ParkingCandidateMode parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT;
    bool enable_order_constraint_learning = true;
    bool has_fixed_random_seed = false;
    std::uint32_t base_random_seed = 0;
    int search_threads = 5;
    int max_search_iterations = 50000;
    int assignment_search_iterations = 0;
    int local_sequence_search_iterations = 0;
    int shuffle_sequence_search_iterations = 0;
    int lns_iterations = 0;
    bool enable_visualization = true;
    bool integrated_mode = false;
    std::string handoff_endpoint = "tcp://127.0.0.1:5566";
    std::string input_sequence_path;
};
