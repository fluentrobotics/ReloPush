#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H


#include <PHAstar/Entities.h>
#include <vector>
#include <string>
#include <limits>
#include <PHAstar/Node.h>

enum class PlanningStatus
{
    SUCCESS,
    START_INVALID_COLLISION, // Start pose collides with entity
    START_OUT_OF_BOUNDS,     // Start pose outside map bounds
    GOAL_INVALID_COLLISION,  // Goal pose collides with entity
    GOAL_OUT_OF_BOUNDS,      // Goal pose outside map bounds
    NO_PATH_FOUND,           // Search exhausted without reaching goal
    TIMEOUT_EXCEEDED,        // Max iterations/time limit hit
    HIGH_COST_UNFEASIBLE,    // Path cost too high (e.g., excessive reverses)
    BLOCKED_BY_ROBOT,        // Valid path found but blocked by relocatable robot
    INTERNAL_ERROR           // Generic (e.g., empty graph)
};

struct PlanningDebugStats
{
    std::size_t iterations = 0;
    std::size_t generated_nodes = 0;
    std::size_t accepted_nodes = 0;
    std::size_t closed_nodes = 0;
    std::size_t peak_open_size = 0;
    std::size_t reject_collision = 0;
    std::size_t reject_closed = 0;
    std::size_t reject_worse_g = 0;
    std::size_t analytic_collision = 0;
    std::size_t analytic_post_arrival_collision = 0;
    std::size_t reference_egraph_nodes = 0;
    std::size_t reference_egraph_snap_accepted = 0;
    std::size_t reference_egraph_snap_rejected = 0;
    std::size_t reference_egraph_successor_accepted = 0;
    std::size_t reference_egraph_successor_rejected = 0;
    double best_dist = std::numeric_limits<double>::infinity();
    double best_yaw_error = std::numeric_limits<double>::infinity();
    Pose best_pose;
    int planner_expansion_threads = 1;
    double analytic_validation_time_sec = 0.0;
    double primitive_collision_time_sec = 0.0;
    double heuristic_time_sec = 0.0;
    double serial_merge_time_sec = 0.0;
    std::string final_failure_reason;
};

struct PlanningResult
{
    std::vector<Waypoint> waypoints;
    PlanningStatus status = PlanningStatus::INTERNAL_ERROR;
    std::string failure_detail = "";   // Human-readable message
    std::string colliding_entity = ""; // Name of entity causing collision (if applicable)
    double failure_time = 0.0;         // Timestamp where collision/failure occurred (if relevant)
    std::vector<Node> explored_nodes;
    PlanningDebugStats debug_stats;
};

struct CollisionInfo {
    bool is_valid = true;          // True if safe, False if collision/OOB
    std::string reason = "Valid";  // "Robot OOB", "Obj Collision", etc.
    std::string entity_name = "";  // Name of the obstacle hit ("wall", "robot2")
    double time = 0.0;             // Time of collision
};

#endif
