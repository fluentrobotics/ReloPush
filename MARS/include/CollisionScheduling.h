#ifndef COLLISION_SCHEDULING_H
#define COLLISION_SCHEDULING_H

#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <string>
#include <unordered_map>
#include <vector>

// ==========================================
// Collision Checking & Scheduling
// ==========================================

// Lazily-built memo for find_safe_start_time's terminal-hold scan (see
// check_terminal_hold_detailed in CollisionScheduling.cpp for the full
// design/invalidation notes and the backward-scan build algorithm, which
// stays private to that translation unit). The struct itself is exposed here
// -- rather than just forward-declared -- so callers (find_safe_start_time,
// and unit tests exercising the cache directly) can own an instance and pass
// its address across a whole candidate loop, exactly like
// find_safe_start_time does. Every field is an implementation detail owned
// by check_terminal_hold_detailed; callers should only default-construct an
// instance, optionally call invalidate(), and pass a pointer to it -- never
// read or set the fields directly.
struct TerminalHoldCache
{
  bool valid = false;
  bool empty = true;
  double max_colliding_sample = 0.0;
  std::string reason;
  std::string entity_name;

  void invalidate() { valid = false; }
};

// Contact Validation Functions (now in CollisionScheduling.cpp)
bool is_valid_transfer_contact(EntityMeta *e1, const Pose &p1,
                               EntityMeta *e2, const Pose &p2);
bool is_valid_terminal_approach_contact(RobotMeta *robot,
                                        const Pose &robot_pose,
                                        const Pose &goal_pose,
                                        EntityMeta *collider,
                                        const Pose &collider_pose,
                                        EntityMeta *terminal_approach_entity,
                                        const Params &params);
bool has_active_transfer_contact(EntityMeta *e1, EntityMeta *e2, double t,
                                 const std::vector<TransferContactWindow> &windows);
bool has_transfer_pair(EntityMeta *e1, EntityMeta *e2,
                       const std::vector<TransferContactWindow> &windows);
RobotMeta *find_robot_transferring_object_at_time(
    EntityMeta *object,
    double t,
    TimeTable &timetable);

// Timetable verification
TimeTableVerificationResult verify_timetable_collision_free(
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> &transfer_windows,
    double from_time = 0.0,
    double to_time = -1.0,
    double step = -1.0);

// Collision step size
double shared_collision_check_step(const Params &params);

// Trajectory collision checking
CollisionInfo check_collision_trajectory_detailed(
    const Trajectory &traj, double start_time,
    TimeTable &timetable, const Params &params,
    bool verbose,
    EntityMeta *terminal_approach_entity = nullptr);
bool check_collision_trajectory(
    const Trajectory &traj, double start_time,
    TimeTable &timetable, const Params &params,
    bool verbose = false);
// `hold_cache`, when non-null (and the compile-time kUseTerminalHoldCache
// escape hatch in CollisionScheduling.cpp is left at its default true),
// turns the per-sample forward scan below into a lazily-built O(1) lookup;
// see CollisionScheduling.cpp for the full design/invalidation notes. Every
// caller except find_safe_start_time passes the default nullptr and gets
// exactly the old per-call forward-scan behavior.
CollisionInfo check_terminal_hold_detailed(
    const Trajectory &traj, double start_time,
    TimeTable &timetable, const Params &params,
    EntityMeta *terminal_approach_entity = nullptr,
    TerminalHoldCache *hold_cache = nullptr);
// `plan_stats`, when non-null, times the two inner checks separately into
// PlanTimingStats::traj_scan_wall_s (check_collision_trajectory_detailed) and
// PlanTimingStats::terminal_hold_wall_s (check_terminal_hold_detailed, only
// reached when the motion check passes). Only find_safe_start_time's own
// calls to this function pass a real pointer; other callers keep the default
// (no timing, no behavior change). `hold_cache` is forwarded as-is to
// check_terminal_hold_detailed (see above); default nullptr = old behavior.
CollisionInfo check_trajectory_motion_and_terminal_hold_detailed(
    const Trajectory &traj, double start_time,
    TimeTable &timetable, const Params &params,
    EntityMeta *terminal_approach_entity = nullptr,
    PlanTimingStats *plan_stats = nullptr,
    TerminalHoldCache *hold_cache = nullptr);
CollisionInfo check_collision_trajectory_against_entity(
    const Trajectory &traj, double start_time,
    EntityMeta *other_entity,
    TimeTable &timetable, const Params &params);

// Wait-only start time search
double find_wait_only_start_time(
    const Trajectory &traj, double earliest_start,
    TimeTable &timetable, const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);
double find_wait_only_start_time_avoiding_entity(
    const Trajectory &traj, double earliest_start,
    EntityMeta *blocking_entity,
    TimeTable &timetable, const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

// Stationary pose checks
CollisionInfo check_stationary_pose_collision_at_time(
    EntityMeta *entity, const Pose &pose, double query_time,
    TimeTable &timetable, const Params &params);
CollisionInfo check_stationary_pose_collision_at_time_ignoring(
    EntityMeta *entity, const Pose &pose, double query_time,
    TimeTable &timetable, const Params &params,
    EntityMeta *ignored_entity);
CollisionInfo find_stationary_pose_conflict_until_last_timestamp(
    EntityMeta *entity, const Pose &pose, double from_time,
    TimeTable &timetable, const Params &params,
    double step = 0.5);
CollisionInfo find_stationary_pose_conflict_until_last_timestamp_ignoring(
    EntityMeta *entity, const Pose &pose, double from_time,
    TimeTable &timetable, const Params &params,
    double step, EntityMeta *ignored_entity);
CollisionInfo find_stationary_pose_conflict_in_interval(
    EntityMeta *entity, const Pose &pose,
    double from_time, double to_time,
    TimeTable &timetable, const Params &params,
    double step = 0.5);
CollisionInfo find_stationary_pose_conflict_in_interval_ignoring(
    EntityMeta *entity, const Pose &pose,
    double from_time, double to_time,
    TimeTable &timetable, const Params &params,
    double step, EntityMeta *ignored_entity);
CollisionInfo find_robot_waiting_pose_conflict_over_interval(
    RobotMeta *robot, const Pose &pose,
    double from_time, double to_time,
    TimeTable &timetable, const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    EntityMeta *preferred_ignored_entity = nullptr);
double delayed_segment_start_after_stationary_conflict(
    double current_start, double segment_duration,
    double conflict_time, double step = 0.5);

// Delay horizon
double timetable_delay_search_horizon(
    double earliest_start, const TimeTable &timetable, double step);

// Parking hint collision checks
bool parking_pose_conflicts_with_blocked_hint(
    const Pose &candidate_pose, RobotMeta *blocker,
    const Trajectory *blocked_traj_hint);
// `hint_reference_time` is the real scheduling-window time the caller wants
// blocked_traj_hint validated against. It is only used when
// blocked_traj_hint->start_time is unassigned (<= 0.0); the legacy
// waypoint-front-time fallback is used only if hint_reference_time is also
// unavailable (<= 0.0).
bool parking_candidate_clears_blocked_hint(
    const Trajectory *blocked_traj_hint,
    RobotMeta *blocker,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr,
    double hint_reference_time = -1.0);

// Transfer contact windows
std::vector<TransferContactWindow> transfer_windows_with_candidate(
    const std::vector<TransferContactWindow> &existing_windows,
    const Trajectory &candidate_traj);
void append_transfer_window_if_needed(
    const Trajectory &traj,
    std::vector<TransferContactWindow> *transfer_windows);

// Pre-commit validation
bool validate_pre_commit_trajectory(
    TimeTable &trial_timetable,
    const Trajectory &traj,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> *transfer_windows,
    CollisionInfo *out_collision,
    std::string *out_failure_reason);

// Commit trajectory
bool reserve_and_commit_trajectory(
    Trajectory *traj,
    RobotMeta *robot,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows = nullptr,
    TaskExecutionStats *stats = nullptr,
    std::string *out_failure_reason = nullptr,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr,
    PlanTimingStats *plan_stats = nullptr);

// Collision classification
bool is_soft_robot_collision(
    const CollisionInfo &info,
    const std::unordered_map<std::string, EntityMeta *> &entities);
std::string find_valid_start_contact_entity(
    RobotMeta *robot, const Pose &start_pose,
    double start_time, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities);

// Blocker resolution
RobotMeta *find_resolvable_robot_blocker(
    const CollisionInfo &col_info,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities);
RobotMeta *find_robot_transferring_object_at_time(
    EntityMeta *object, double query_time,
    TimeTable &timetable);
bool has_active_transfer_contact(
    EntityMeta *entity1, EntityMeta *entity2, double query_time,
    const std::vector<TransferContactWindow> &windows);
bool has_transfer_pair(
    EntityMeta *entity1, EntityMeta *entity2,
    const std::vector<TransferContactWindow> &windows);

// Full conflict-resolution scheduler. `plan_stats`, when non-null,
// accumulates: total wall time into PlanTimingStats::sched_wall_s, one
// PlanTimingStats::n_find_safe_start_calls per call, one
// PlanTimingStats::n_start_candidates_tried per candidate start-time
// evaluated in the internal search loop, and (via the pointer forwarded to
// check_trajectory_motion_and_terminal_hold_detailed and to
// relocate_blocking_robot) the traj-scan / terminal-hold / safe-parking
// breakdowns.
double find_safe_start_time(
    Trajectory *traj, double earliest_start,
    TimeTable &timetable, const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    double *out_wait_added = nullptr,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr,
    IdleBlockerRelocationPolicy idle_blocker_policy =
        IdleBlockerRelocationPolicy::RelocateAnyIdle,
    PlanTimingStats *plan_stats = nullptr);

#endif // COLLISION_SCHEDULING_H
