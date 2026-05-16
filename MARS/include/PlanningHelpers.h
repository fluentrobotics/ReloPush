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
    const std::vector<Waypoint> *reference_waypoints = nullptr);

bool append_retraction(RobotMeta *robot, const Trajectory &previous_traj,
                       TimeTable &timetable, const Params &params,
                       const std::unordered_map<std::string, EntityMeta *> &entities,
                       const RuntimeOptions &options,
                       TaskExecutionStats *stats = nullptr,
                       std::string *out_failure_reason = nullptr);

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
    const std::vector<Waypoint> *reference_waypoints = nullptr);

#endif // PLANNING_HELPERS_H
