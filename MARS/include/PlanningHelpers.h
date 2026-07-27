#ifndef PLANNING_HELPERS_H
#define PLANNING_HELPERS_H

#include <PHAstar/Point.h>    // for Pose
#include <PHAstar/Entities.h> // for EntityMeta, RobotMeta, ObjectMeta, Trajectory, Waypoint
#include <PHAstar/Params.h>   // for Params
#include <PHAstar/PlanningResult.h> // for PlanningResult, PlanningDebugStats
#include <PHAstar/Visualization.h> // for PlanningDebugAttempt
#include <PHAstarPushDemoOptions.h> // for RuntimeOptions, ReferenceExperienceGraphOptions
#include <PHAstarPushDemoTypes.h> // for TaskExecutionStats, SegmentReplanContext
#include <Task.h> // for Task
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

// Forward declarations
class TimeTable;

// ==========================================
// Pose / Diagnostic Utilities
// ==========================================

bool poses_approximately_equal(const Pose &a, const Pose &b,
                               double pos_tol = 1e-3,
                               double yaw_tol = 1e-3);

std::string format_pose_compact(const Pose &pose);

std::string describe_entity_pose_context(const TimeTable &timetable,
                                         EntityMeta *entity,
                                         double t);

std::string summarize_pair_collision_window(const TimeTable &timetable,
                                            EntityMeta *entity_a,
                                            EntityMeta *entity_b,
                                            const Params &params,
                                            double center_time,
                                            double horizon = 2.0,
                                            double step = 0.05);

std::string compare_pair_collision_checks(const TimeTable &timetable,
                                          EntityMeta *entity_a,
                                          EntityMeta *entity_b,
                                          const Params &params,
                                          double t);

// ==========================================
// Contact and Approach Pose Helpers
// ==========================================

double contact_offset_for_object(const RobotMeta *robot,
                                 const EntityMeta *object,
                                 double extra_clearance = 0.0);

Pose recover_object_centric_pose_from_raw_terminal(const Pose &raw_terminal_pose,
                                                   double source_pre_push_distance);

Pose compute_adjusted_prepush_goal(const Pose &object_pose,
                                   double pushing_yaw,
                                   const RobotMeta *robot,
                                   const EntityMeta *object,
                                   double extra_clearance);

Pose compute_adjusted_task_start_pose(const Task &task,
                                      const RobotMeta *robot,
                                      double query_time,
                                      const TimeTable &timetable);

bool rewrite_transfer_terminal_pose(Trajectory *traj, RobotMeta *robot);

// ==========================================
// Planning Diagnostics
// ==========================================

void diagnose_planning_failure(RobotMeta *robot, const Pose &start,
                               const Pose &goal, double time,
                               TimeTable &timetable,
                               const std::unordered_map<std::string, EntityMeta *> &entities,
                               const Params &params);

// ==========================================
// Trajectory and Waypoint Utilities
// ==========================================

void project_waypoints_inside_bounds(std::vector<Waypoint> &waypoints,
                                     RobotMeta *robot,
                                     const Params &params,
                                     EntityMeta *transferred_object = nullptr);

void shift_waypoint_times(std::vector<Waypoint> &waypoints, double delta);

void make_waypoint_times_relative(std::vector<Waypoint> &waypoints,
                                  double reference_time);

Params make_relaxed_fallback_params(const Params &params);

Params make_fine_segment_params(const Params &params,
                                const RuntimeOptions &options);

Params make_contact_boundary_segment_params(const Params &params,
                                            const RuntimeOptions &options);

ReferenceExperienceGraphOptions reference_egraph_options_from_runtime(
    const RuntimeOptions &options);

std::vector<Waypoint> waypoints_from_relopush_state_path(
    const ReloPush::StatePathPtr &path);

void accumulate_wait_stats(TaskExecutionStats *stats, double wait_added);

// ==========================================
// Main Planning Functions (Blocks C, D, E)
// ==========================================

bool plan_initial_transit(
    RobotMeta *robot, const Pose &target_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    double *out_abs_start_time = nullptr,
    double *out_abs_end_time = nullptr,
    const std::function<Pose(double)> &target_pose_provider = {},
    const std::vector<Waypoint> *reference_waypoints = nullptr,
    PlanTimingStats *plan_stats = nullptr);

bool append_retraction(RobotMeta *robot, const Trajectory &previous_traj,
                       TimeTable &timetable, const Params &params,
                       const std::unordered_map<std::string, EntityMeta *> &entities,
                       const RuntimeOptions &options,
                       TaskExecutionStats *stats = nullptr,
                       std::string *out_failure_reason = nullptr,
                       PlanTimingStats *plan_stats = nullptr);

struct SegmentCandidateValidation
{
  bool hard_valid = false;
  bool has_soft_robot_conflict = false;
  CollisionInfo first_hard_collision;
  CollisionInfo first_soft_robot_collision;
};

struct SegmentCandidateReport
{
  std::string stage;
  PlanningResult planning;
  std::vector<Waypoint> debug_waypoints;
  SegmentCandidateValidation validation;
  bool candidate_tested = false;
  bool accepted_for_scheduling = false;
  bool selected = false;
};

SegmentCandidateValidation validate_segment_candidate(
    const std::vector<Waypoint> &candidate_rel,
    RobotMeta *robot,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    EntityMeta *terminal_approach_entity = nullptr);

std::vector<Waypoint> combine_prefix_and_tail(
    const std::vector<Waypoint> &prefix_rel,
    std::vector<Waypoint> tail_abs,
    double original_start_time);

std::string format_planner_stats(const PlanningDebugStats &stats);

void write_segment_replan_diagnostics(
    const SegmentReplanContext &context,
    RobotMeta *robot,
    const Pose &start_pose,
    const Pose &goal_pose,
    double start_time,
    const std::vector<SegmentCandidateReport> &reports,
    bool success,
    const std::string &selected_stage);

std::vector<PlanningDebugAttempt> segment_reports_to_debug_attempts(
    const std::vector<SegmentCandidateReport> &reports);

bool replan_transit_segment(
    RobotMeta *robot, const Pose &goal_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<Waypoint> &out_waypoints_rel,
    const SegmentReplanContext &context = SegmentReplanContext{},
    EntityMeta *terminal_approach_entity = nullptr,
    const std::vector<Waypoint> *reference_waypoints = nullptr,
    PlanTimingStats *plan_stats = nullptr);

// ==========================================
// Stage 3: failure triage (RuntimeOptions::enable_tier_triage) and the
// holonomic feasibility pre-gate (RuntimeOptions::enable_tier_gate).
//
// Both are opt-in, default-off refinements to the tier-escalation cascades
// in plan_initial_transit's method loop and replan_transit_segment's
// cascade (see PlanningHelpers.cpp for the full design note and wiring).
// The predicate/gate functions below are factored out as pure, free
// functions so they can be unit-tested directly against hand-built
// PlanningResult/debug_stats fixtures (see MARS/tests/phastar_unit_tests.cpp)
// without needing to drive a real search to failure.
// ==========================================

// Rule 2's progress threshold: a primary-tier search that hit its iteration
// cap is retried once (doubled cap) only if its best node got at least this
// close, as a fraction of the straight-line start-to-goal distance.
constexpr double kTriageProgressFraction = 0.5;

// Rule 3's collision-rejection threshold: a tier whose best node reached the
// goal's analytic-threshold neighborhood is treated as a contact-geometry
// -shaped failure only if collision rejections account for at least this
// fraction of generated_nodes.
constexpr double kTriageCollisionRejectFraction = 0.5;

// Rule 1 (BLOCKED-BY-ROBOT SHORT-CIRCUIT): true iff `result` carries a
// robot-blocked backup path. PlanningStatus::BLOCKED_BY_ROBOT is only ever
// produced by PHAStar::Planning_with_res together with a non-empty extracted
// backup path (see PHAstar.h's backup_result handling); the waypoints check
// here is a defensive double-check, not load-bearing. When true, the caller
// should stop escalating to later tiers and use `result` as the cascade's
// final outcome -- a robot in the way is not something a finer search
// resolves.
bool triage_blocked_by_robot_shortcut_applies(const PlanningResult &result);

// Rule 2 (CAP-HIT-WITH-PROGRESS RETRY, primary tier only): true iff `result`
// hit its search's iteration cap AND made real progress (best_dist within
// kTriageProgressFraction of `straight_line_distance`, the straight-line
// start-to-goal distance for that search).
bool triage_primary_retry_applies(const PlanningResult &result,
                                  double straight_line_distance);

// Rule 3 (NEAR-GOAL-COLLISION SKIP-TO-CONTACT): true iff `result`'s best node
// already reached the goal's analytic-threshold neighborhood but was
// repelled by collision rejections (a contact-geometry-shaped failure, where
// a finer free-space search -- Fine -- is unlikely to help and
// ContactBoundary's holonomic heuristic / tighter contact handling is the
// better next step).
bool triage_skip_to_contact_applies(const PlanningResult &result);

// True iff `methods` contains a FineHybridAStar entry followed later (not
// necessarily immediately) by a ContactBoundaryGeometricHybridAStar entry.
// Gates rule 3's applicability per the Stage 3 spec ("only when their method
// list literally contains Fine followed by Contact").
bool method_list_has_fine_before_contact(
    const std::vector<TransitPlannerStep> &methods);

// The largest circle centered at an entity's pose origin that stays fully
// inside its collision rectangle at every yaw -- an UNDER-approximation of
// its footprint. Uses get_corners's local-frame convention (rectangle spans
// [-rear_length, front_length] x [-width/2, width/2]) and the raw, uninflated
// size. Used by the holonomic feasibility gate so that any corridor the true
// (rectangular, possibly inflated-for-safety-elsewhere) robot could traverse
// is also open to the gate's disc approximation.
double entity_inscribed_radius(const OccuRect &size);

// Stage 3 HOLONOMIC FEASIBILITY PRE-GATE. Returns true iff a cheap, sound,
// any-yaw (holonomic) reachability check PROVES `goal_pose` cannot be
// reached from `start_pose` at all, on a grid at
// `gate_params.holonomic_heuristic_resolution` over `gate_params`'s
// workspace bounds. Parallels PHAStar::ensure_holonomic_heuristic's
// goal-rooted Dijkstra grid (see PHAstar.h) but as a completely separate,
// throwaway build: it never reads or mutates any PHAStar instance's cached
// heuristic state or Params.
//
// Soundness (the gate may only return true when infeasibility is
// guaranteed):
//  - Obstacles are STATIC only: the workspace boundary (via
//    check_robot_bounds_collision, respecting Params::robot_boundary_origin_only
//    exactly like the real planner) plus OBJECT entities with no scheduled
//    motion after `search_start_time` (TimeTable::is_entity_static_after).
//    ROBOTS are never treated as obstacles -- they can always move away.
//  - The robot's footprint is UNDER-approximated as a disc of radius
//    entity_inscribed_radius(robot->size) centered at the query pose (never
//    the inflated/circumscribed footprint used elsewhere for safety
//    margins), so the disc is a geometric subset of the true rectangle at
//    every yaw: any path the disc cannot traverse, the true (larger) robot
//    cannot traverse either.
//  - If the grid cannot be built (degenerate/empty bounds), returns false
//    (the gate passes; it never skips on an inconclusive check).
bool holonomic_gate_unreachable(
    const Pose &start_pose, const Pose &goal_pose,
    RobotMeta *robot, const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &gate_params, double search_start_time);

// Wraps holonomic_gate_unreachable for one tier attempt: times the check
// into plan_stats->gate_wall_s, always increments plan_stats->n_gate_checks
// (when plan_stats is non-null and the gate actually runs), and -- only when
// the goal is proven unreachable -- fills `out_result` with a synthetic
// NO_PATH_FOUND / empty-waypoints PlanningResult (as if the tier had run and
// failed) and increments the tier-specific skip counter
// (n_gate_skips_fine / n_gate_skips_contact). `tier` must be
// PlanSearchTier::Fine or PlanSearchTier::Contact. Returns true iff the tier
// should be skipped (out_result was filled).
bool apply_tier_gate(
    PlanningResult &out_result, PlanSearchTier tier,
    const Pose &start_pose, const Pose &goal_pose,
    RobotMeta *robot, const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &tier_params, double search_start_time,
    PlanTimingStats *plan_stats);

#endif // PLANNING_HELPERS_H
