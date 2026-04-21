#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H


#include <PHAstar/Entities.h>
#include <vector>
#include <string>
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

struct PlanningResult
{
    std::vector<Waypoint> waypoints;
    PlanningStatus status = PlanningStatus::INTERNAL_ERROR;
    std::string failure_detail = "";   // Human-readable message
    std::string colliding_entity = ""; // Name of entity causing collision (if applicable)
    double failure_time = 0.0;         // Timestamp where collision/failure occurred (if relevant)
    std::vector<Node> explored_nodes;
};

struct CollisionInfo {
    bool is_valid = true;          // True if safe, False if collision/OOB
    std::string reason = "Valid";  // "Robot OOB", "Obj Collision", etc.
    std::string entity_name = "";  // Name of the obstacle hit ("wall", "robot2")
    double time = 0.0;             // Time of collision
};

#endif
