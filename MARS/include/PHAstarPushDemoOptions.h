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

enum class TransitPlannerMethod
{
    PrimaryHybridAStar,
    FineHybridAStar,
    ContactBoundaryGeometricHybridAStar,
    GhostHybridAStar,
    GeometryFallbackHybridAStar,
    ReedSheppFallback,
    AllCandidateReedShepp,
    ReverseStraightEscapeFineHybridAStar,
    ReverseLeftEscapeFineHybridAStar,
    ReverseRightEscapeFineHybridAStar,
    ReverseStraightEscapeReedShepp,
    ReverseLeftEscapeReedShepp,
    ReverseRightEscapeReedShepp,
};

enum class ReferenceEGraphUse
{
    Disabled,
    EnabledWhenAvailable,
};

enum class TransitPlannerApplicability
{
    Always,
    TightOrContactOnly,
    ContactBoundaryOnly,
    StartContactOnly,
    NonStartContactOnly,
    StartContactTightOrContactOnly,
    NonStartContactTightOrContactOnly,
};

struct TransitPlannerStep
{
    TransitPlannerMethod method;
    ReferenceEGraphUse reference_egraph = ReferenceEGraphUse::Disabled;
    TransitPlannerApplicability applicability = TransitPlannerApplicability::Always;
};

inline const char *transit_planner_method_name(TransitPlannerMethod method)
{
    switch (method)
    {
    case TransitPlannerMethod::PrimaryHybridAStar:
        return "primary Hybrid A*";
    case TransitPlannerMethod::FineHybridAStar:
        return "fine Hybrid A*";
    case TransitPlannerMethod::ContactBoundaryGeometricHybridAStar:
        return "contact-boundary geometric Hybrid A*";
    case TransitPlannerMethod::GhostHybridAStar:
        return "ghost Hybrid A*";
    case TransitPlannerMethod::GeometryFallbackHybridAStar:
        return "geometry fallback Hybrid A*";
    case TransitPlannerMethod::ReedSheppFallback:
        return "Reed-Shepp fallback";
    case TransitPlannerMethod::AllCandidateReedShepp:
        return "all-candidate Reed-Shepp";
    case TransitPlannerMethod::ReverseStraightEscapeFineHybridAStar:
        return "reverse-straight escape + fine Hybrid A*";
    case TransitPlannerMethod::ReverseLeftEscapeFineHybridAStar:
        return "reverse-left escape + fine Hybrid A*";
    case TransitPlannerMethod::ReverseRightEscapeFineHybridAStar:
        return "reverse-right escape + fine Hybrid A*";
    case TransitPlannerMethod::ReverseStraightEscapeReedShepp:
        return "reverse-straight escape + Reed-Shepp";
    case TransitPlannerMethod::ReverseLeftEscapeReedShepp:
        return "reverse-left escape + Reed-Shepp";
    case TransitPlannerMethod::ReverseRightEscapeReedShepp:
        return "reverse-right escape + Reed-Shepp";
    }
    return "unknown";
}

inline const char *reference_egraph_use_name(ReferenceEGraphUse use)
{
    return use == ReferenceEGraphUse::EnabledWhenAvailable
               ? "Reference E-Graph when available"
               : "no Reference E-Graph";
}

inline const char *transit_planner_applicability_name(
    TransitPlannerApplicability applicability)
{
    switch (applicability)
    {
    case TransitPlannerApplicability::Always:
        return "always";
    case TransitPlannerApplicability::TightOrContactOnly:
        return "tight/contact only";
    case TransitPlannerApplicability::ContactBoundaryOnly:
        return "contact-boundary only";
    case TransitPlannerApplicability::StartContactOnly:
        return "start-contact only";
    case TransitPlannerApplicability::NonStartContactOnly:
        return "non-start-contact only";
    case TransitPlannerApplicability::StartContactTightOrContactOnly:
        return "start-contact tight/contact only";
    case TransitPlannerApplicability::NonStartContactTightOrContactOnly:
        return "non-start-contact tight/contact only";
    }
    return "unknown";
}

struct RuntimeOptions
{
    // visualization options
    bool debug_vis = false;
    bool visualize_relopush_plan = false;
    bool enable_visualization = true; // show results
    bool print_planning_status = true;

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
    int planner_expansion_threads = 1;
    int max_search_iterations = 300;
    bool robot_boundary_origin_only = true;

    // Fine segment options (for replanning)
    int fine_segment_max_search_iterations = 0;
    double fine_segment_xy_resolution = 0.1;
    double fine_segment_yaw_resolution = 0.523598775598298873; // pi / 6
    double fine_segment_time_step = 1.6;
    double fine_segment_rs_step_size = 0.16;
    double fine_segment_collision_check_time_step = 0.05;
    int contact_boundary_max_search_iterations = 500;
    double contact_boundary_xy_resolution = 0.08;   // 0.08
    double contact_boundary_yaw_resolution = 0.235; // 0.26; // 0.235
    double contact_boundary_time_step = 1.0;
    double contact_boundary_rs_step_size = 0.10;
    double contact_boundary_reverse_penalty = 2.0;
    double contact_boundary_holonomic_resolution = 0.10;

    // E-Graph
    double reference_egraph_epsilon = 10.0;
    double reference_egraph_waypoint_spacing = 0.10;
    double reference_egraph_snap_radius = 0.25;
    double reference_egraph_snap_yaw = M_PI / 3.0;
    int reference_egraph_successor_lookahead = 3;
    int reference_egraph_max_nodes = 120;

    // Safe Parking
    ParkingCandidateMode parking_candidate_mode = ParkingCandidateMode::REVERSE_RECENT_SHORTER;
    int safe_parking_max_search_iterations = 25; // 0 inherits max_search_iterations
    bool enable_failed_candidate_idle_parking = true;
    int failed_candidate_initial_transit_failure_threshold = 2;
    bool enable_order_constraint_learning = true;
    bool has_fixed_random_seed = false;
    std::uint32_t base_random_seed = 0;

    // Assignment Refinement options
    int assignment_search_iterations = 0; // for random assignment
    int local_sequence_search_iterations = 0;
    int shuffle_sequence_search_iterations = 0;
    int lns_iterations = 10;
    int lns_threads = 5;
    bool enable_lns_fine_segment_retry = false; // for LNS

    std::vector<TransitPlannerStep> initial_transit_methods = {
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
    };

    std::vector<TransitPlannerStep> segment_transit_methods = {
        /*
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::StartContactOnly},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::StartContactTightOrContactOnly},
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::NonStartContactOnly},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::NonStartContactTightOrContactOnly},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::ContactBoundaryOnly},
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::StartContactOnly},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable,
         TransitPlannerApplicability::StartContactTightOrContactOnly},
        {TransitPlannerMethod::AllCandidateReedShepp,
         ReferenceEGraphUse::Disabled,
         TransitPlannerApplicability::Always},
         */
        {TransitPlannerMethod::PrimaryHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::FineHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar,
         ReferenceEGraphUse::EnabledWhenAvailable},
    };

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
