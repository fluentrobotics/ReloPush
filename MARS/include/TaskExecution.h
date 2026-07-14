#ifndef TASK_EXECUTION_H
#define TASK_EXECUTION_H

#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <PlanningHelpers.h>
#include <Task.h>
#include <PHAstar/Entities.h>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

// Thread-local state for tracking initial transit failures per robot/pose
extern thread_local std::unordered_map<std::string, int> g_initial_transit_failure_counts;
std::string rounded_pose_key(const Pose &pose);
std::string initial_transit_failure_key(RobotMeta *robot, const Pose &pose);
void clear_initial_transit_failures_for_robot(RobotMeta *robot);

// ==========================================
// Robot Candidate Selection
// ==========================================

RobotMeta *find_earliest_robot(const std::vector<RobotMeta *> &robots,
                                TimeTable &timetable, double &out_free_time);

std::vector<std::pair<RobotMeta *, double>> get_sorted_candidate_robots(
    const std::vector<RobotMeta *> &robots,
    TimeTable &timetable,
    const Task &task,
    const Params &params);

// ==========================================
// Segment Preparation & Repair
// ==========================================

bool prepare_segment_waypoints_for_scheduling(
    Trajectory *traj, RobotMeta *robot, double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params, const RuntimeOptions &options,
    const std::string &fallback_message,
    const SegmentReplanContext &replan_context = SegmentReplanContext{});

bool is_empty_noop_connector_between_pushes(
    const std::vector<TrajectoryPtr> &edge_paths, std::size_t connector_idx,
    double pos_tol = 1e-2, double yaw_tol = 1e-2);

bool try_local_transfer_path_repair(
    Trajectory *traj, RobotMeta *robot, double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params);

bool replan_transfer_segment_after_failed_schedule(
    Trajectory *traj, RobotMeta *robot, double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params, const RuntimeOptions &options,
    std::string *out_failure_reason = nullptr);

bool replan_transit_segment_after_failed_schedule(
    Trajectory *traj, RobotMeta *robot, double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params, const RuntimeOptions &options,
    std::string *out_failure_reason = nullptr);

// ==========================================
// Path Segment Scheduling
// ==========================================

bool schedule_path_segment(
    const EdgePath &edge_path, EntityMeta *obj_meta,
    RobotMeta *robot, TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    double source_pre_push_distance = 0.0,
    EntityMeta *approach_goal_entity = nullptr,
    std::vector<TransferContactWindow> *transfer_windows = nullptr,
    TaskExecutionStats *stats = nullptr,
    std::string *out_failure_reason = nullptr,
    double *out_scheduled_start_time = nullptr,
    Trajectory *out_scheduled_trajectory = nullptr);

// ==========================================
// Full Task Execution Pipeline
// ==========================================

bool process_task_execution(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows = nullptr,
    TaskExecutionStats *out_stats = nullptr,
    std::string *out_failure_reason = nullptr,
    double task_start_delay = 0.0,
    int task_id = -1);

// ==========================================
// Task Loop & Allocation
// ==========================================

bool attempt_task_with_candidate(
    Task &task,
    RobotMeta *cand_robot,
    double free_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> &transfer_windows,
    TaskCsvRow &row,
    std::string &last_failed_robot,
    std::string &last_failure_reason);

bool maybe_safe_park_repeated_initial_transit_failure(
    RobotMeta *robot,
    double free_time,
    const std::string &failure_reason,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options);

TaskCsvRow execute_single_task_with_candidates(
    Task &task,
    int task_counter,
    const std::vector<RobotMeta *> &all_robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> &transfer_windows);

std::vector<TaskCsvRow> execute_task_allocation_loop(
    std::vector<Task> &tasks,
    const std::vector<RobotMeta *> &all_robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    bool abort_on_first_failure = false);

#endif // TASK_EXECUTION_H
