/*****************************************************************
 * Task Execution Pipeline
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <TaskExecution.h>
#include <CsvLogging.h>
#include <PlanningHelpers.h>

#include <PHAstar/Visualization.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/Reeds_Shepp.h>
#include <PHAstar/CollisionUtils.h>
#include <ReloPush/TaskAllocation.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>

// Global debug flag (defined in PHAstar_push_demo.cpp)
extern bool DEBUG_VIS;

// Thread-local state for task execution failure tracking
thread_local std::unordered_map<std::string, int> g_initial_transit_failure_counts;

// Thread-local relocation cache
thread_local std::unordered_map<std::string, double> g_recent_failed_relocations;

// Helpers for thread-local state

std::unordered_map<std::string, double> &recent_failed_relocation_cache()
{
  return g_recent_failed_relocations;
}

std::string rounded_pose_key(const Pose &pose)
{
  const int xi = static_cast<int>(std::round(pose.x * 10.0));
  const int yi = static_cast<int>(std::round(pose.y * 10.0));
  const int ai = static_cast<int>(std::round(mod2pi(pose.yaw) * 10.0));
  return std::to_string(xi) + ":" + std::to_string(yi) + ":" +
         std::to_string(ai);
}

std::string initial_transit_failure_key(RobotMeta *robot, const Pose &pose)
{
  return (robot ? robot->name : std::string("unknown")) + "@" +
         rounded_pose_key(pose);
}

void clear_initial_transit_failures_for_robot(RobotMeta *robot)
{
  if (!robot)
    return;
  const std::string prefix = robot->name + "@";
  for (auto it = g_initial_transit_failure_counts.begin();
       it != g_initial_transit_failure_counts.end();)
  {
    if (it->first.rfind(prefix, 0) == 0)
      it = g_initial_transit_failure_counts.erase(it);
    else
      ++it;
  }
}

// Forward declarations for functions in monolith
void make_waypoint_times_relative(std::vector<Waypoint> &waypoints,
                                   double reference_time);
// plan_initial_transit, append_retraction, replan_transit_segment now declared in PlanningHelpers.h

// ==========================================
// Forward declarations: SafeParking still in monolith
// ==========================================

bool relocate_blocking_robot(
    RobotMeta *blocker, TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    const Trajectory *blocked_traj_hint = nullptr);

// ==========================================
// Forward declarations: CollisionScheduling still in monolith
// ==========================================

CollisionInfo check_collision_trajectory_detailed(
    const Trajectory &traj, double start_time,
    TimeTable &timetable, const Params &params, bool verbose,
    EntityMeta *terminal_approach_entity = nullptr);

CollisionInfo check_collision_trajectory_against_entity(
    const Trajectory &traj, double start_time,
    EntityMeta *other_entity, TimeTable &timetable,
    const Params &params);

TimeTableVerificationResult verify_timetable_collision_free(
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> &transfer_windows,
    double from_time = 0.0,
    double to_time = -1.0,
    double step = -1.0);

bool reserve_and_commit_trajectory(
    Trajectory *traj, RobotMeta *robot, double earliest_start,
    TimeTable &timetable, const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows = nullptr,
    TaskExecutionStats *stats = nullptr,
    std::string *out_failure_reason = nullptr,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

CollisionInfo find_robot_waiting_pose_conflict_over_interval(
    RobotMeta *robot, const Pose &wait_pose,
    double from_t, double to_t,
    TimeTable &timetable, const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    EntityMeta *preferred_ignored_entity = nullptr);

static const double INF = std::numeric_limits<double>::infinity();

// ==========================================
// Robot Candidate Selection
// ==========================================

RobotMeta *find_earliest_robot(const std::vector<RobotMeta *> &robots,
                                TimeTable &timetable, double &out_free_time)
{
  RobotMeta *best_robot = nullptr;
  out_free_time = std::numeric_limits<double>::infinity();

  for (auto *robot : robots)
  {
    double t = timetable.get_entity_max_time(robot);
    if (t < out_free_time)
    {
      out_free_time = t;
      best_robot = robot;
    }
  }
  return best_robot;
}

// Returns list of (robot, free_time) sorted by earliest free time
std::vector<std::pair<RobotMeta *, double>> get_sorted_candidate_robots(
    const std::vector<RobotMeta *> &robots,
    TimeTable &timetable,
    const Task &task,
    const Params &params)
{
  std::vector<std::pair<RobotMeta *, double>> candidates;
  for (auto *robot : robots)
  {
    double t = timetable.get_entity_max_time(robot);
    candidates.emplace_back(robot, t);
  }

  auto rs_length_to_task = [&](RobotMeta *robot, double free_time)
  {
    Pose start_pose = timetable.get_pose(robot, free_time);
    Pose task_start_pose =
        compute_adjusted_task_start_pose(task, robot, free_time, timetable);
    double maxc = 1.0 / std::max(robot->transit_turning_radius(), 1e-6);
    double step_size = params.rs_step_size;
    auto [xs, ys, yaws, ctypes, lengths, steers, directions] =
        ReedShepp::reeds_shepp_path_planning(start_pose.x, start_pose.y, start_pose.yaw,
                                             task_start_pose.x, task_start_pose.y, task_start_pose.yaw,
                                             maxc, step_size, robot->wheel_base);
    if (xs.empty())
    {
      return INF;
    }
    double total = 0.0;
    for (double len : lengths)
    {
      total += std::abs(len);
    }
    return total;
  };

  std::sort(candidates.begin(), candidates.end(),
            [&](const auto &a, const auto &b)
            {
              constexpr double EPS = 1e-6;
              if (std::abs(a.second - b.second) > EPS)
              {
                return a.second < b.second;
              }
              double ra = rs_length_to_task(a.first, a.second);
              double rb = rs_length_to_task(b.first, b.second);
              if (std::abs(ra - rb) > EPS)
                return ra < rb;
              return a.first->name < b.first->name;
            });
  return candidates;
}

// ==========================================
// Segment Preparation & Repair
// ==========================================

bool prepare_segment_waypoints_for_scheduling(
    Trajectory *traj,
    RobotMeta *robot,
    double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    const std::string &fallback_message,
    const SegmentReplanContext &replan_context)
{
  if (!traj)
    return false;

  traj->entity = robot;
  if (!traj->is_transfer)
  {
    if (traj->waypoints.empty())
      return false;

    auto original_waypoints = traj->waypoints;
    Pose start_pose = timetable.get_pose(robot, segment_ready_time);
    Pose segment_goal = traj->waypoints.back();
    EntityMeta *approach_goal_entity = traj->approach_goal_entity;
    SegmentReplanContext segment_context = replan_context;
    if (robot && approach_goal_entity)
    {
      const Pose live_object_pose =
          timetable.get_pose(approach_goal_entity, segment_ready_time);
      const double mars_prepush_distance =
          contact_offset_for_object(robot, approach_goal_entity, 0.01);
      const double source_prepush_distance = traj->source_pre_push_distance;
      const double effective_prepush_distance =
          std::max(mars_prepush_distance,
                   source_prepush_distance > 1e-9
                       ? source_prepush_distance
                       : mars_prepush_distance);
      ReloPush::State object_centric_pose(
          live_object_pose.x, live_object_pose.y, mod2pi(segment_goal.yaw));
      segment_goal = PoseFromReloPushState(
          object_centric_pose.get_prePush(effective_prepush_distance));

      segment_context.has_live_object_pose = true;
      segment_context.live_object_pose = live_object_pose;
      segment_context.mars_prepush_distance = mars_prepush_distance;
      segment_context.source_prepush_distance = source_prepush_distance;
      segment_context.source_clearance_goal = PoseFromReloPushState(
          object_centric_pose.get_prePush(
              source_prepush_distance > 1e-9
                  ? source_prepush_distance
                  : mars_prepush_distance));
    }
    std::vector<Waypoint> reference_waypoints = original_waypoints;
    if (!reference_waypoints.empty())
    {
      reference_waypoints.front().x = start_pose.x;
      reference_waypoints.front().y = start_pose.y;
      reference_waypoints.front().yaw = start_pose.yaw;
      reference_waypoints.back().x = segment_goal.x;
      reference_waypoints.back().y = segment_goal.y;
      reference_waypoints.back().yaw = segment_goal.yaw;
    }
    std::vector<Waypoint> replanned_rel;
    if (!replan_transit_segment(robot, segment_goal, segment_ready_time,
                                timetable, entities, params, options, replanned_rel,
                                segment_context,
                                approach_goal_entity,
                                &reference_waypoints))
    {
      std::cout << fallback_message << std::endl;
      traj->waypoints = original_waypoints;
      return false;
    }
    else
    {
      traj->waypoints = replanned_rel;
      traj->transferred_object = nullptr;
    }
  }
  else
  {
    rewrite_transfer_terminal_pose(traj, robot);
    traj->CalcualteTimeStamps(robot);
  }

  return true;
}

bool is_empty_noop_connector_between_pushes(
    const std::vector<TrajectoryPtr> &edge_paths,
    std::size_t connector_idx,
    double pos_tol,
    double yaw_tol)
{
  if (connector_idx == 0 || connector_idx + 1 >= edge_paths.size())
    return false;

  const auto &prev = edge_paths[connector_idx - 1];
  const auto &connector = edge_paths[connector_idx];
  const auto &next = edge_paths[connector_idx + 1];
  if (!prev || !connector || !next)
    return false;
  if (!prev->is_transfer || connector->is_transfer || !next->is_transfer)
    return false;
  if (!connector->waypoints.empty() || prev->waypoints.empty() ||
      next->waypoints.empty())
  {
    return false;
  }

  return poses_approximately_equal(prev->waypoints.back(),
                                   next->waypoints.front(),
                                   pos_tol, yaw_tol);
}

bool try_local_transfer_path_repair(
    Trajectory *traj,
    RobotMeta *robot,
    double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params)
{
  if (!traj || !robot || !traj->is_transfer || !traj->transferred_object ||
      traj->waypoints.size() < 4)
  {
    return false;
  }

  CollisionInfo original_collision =
      check_collision_trajectory_detailed(*traj, segment_ready_time,
                                          timetable, params, false);
  if (original_collision.is_valid || original_collision.entity_name.empty())
    return false;

  auto collider_it = entities.find(original_collision.entity_name);
  if (collider_it == entities.end() || !collider_it->second ||
      collider_it->second == traj->transferred_object)
  {
    return false;
  }

  const double collision_rel_time =
      std::max(0.0, original_collision.time - segment_ready_time);
  std::size_t center_idx = 1;
  double best_time_error = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1; i + 1 < traj->waypoints.size(); ++i)
  {
    const double err = std::abs(traj->waypoints[i].time - collision_rel_time);
    if (err < best_time_error)
    {
      best_time_error = err;
      center_idx = i;
    }
  }

  const std::vector<double> offsets = {0.015, 0.025, 0.04, 0.06, 0.08};
  const std::vector<int> radii = {2, 3, 4};
  const auto original_waypoints = traj->waypoints;

  for (double offset : offsets)
  {
    for (int sign : {-1, 1})
    {
      for (int radius : radii)
      {
        std::vector<Waypoint> candidate = original_waypoints;
        for (std::size_t i = 1; i + 1 < candidate.size(); ++i)
        {
          const int dist = static_cast<int>(
              std::abs(static_cast<long long>(i) -
                       static_cast<long long>(center_idx)));
          if (dist > radius)
            continue;

          const double phase =
              static_cast<double>(dist) / static_cast<double>(radius + 1);
          const double weight = 0.5 * (1.0 + std::cos(M_PI * phase));
          const double lateral = static_cast<double>(sign) * offset * weight;
          const double nx = -std::sin(candidate[i].yaw);
          const double ny = std::cos(candidate[i].yaw);
          candidate[i].x += lateral * nx;
          candidate[i].y += lateral * ny;
        }

        Trajectory repaired = *traj;
        repaired.waypoints = std::move(candidate);
        repaired.CalcualteTimeStamps(robot);

        CollisionInfo collider_check =
            check_collision_trajectory_against_entity(
                repaired, segment_ready_time, collider_it->second,
                timetable, params);
        if (!collider_check.is_valid)
          continue;

        CollisionInfo full_check =
            check_collision_trajectory_detailed(repaired, segment_ready_time,
                                                timetable, params, false);
        if (!full_check.is_valid &&
            full_check.entity_name == original_collision.entity_name)
        {
          continue;
        }

        traj->waypoints = std::move(repaired.waypoints);
        traj->start_time = segment_ready_time;
        traj->kind = TrajectoryKind::TRANSFER;
        traj->is_transfer = true;

        std::cout << "  [Segment] Local transfer repair accepted near t="
                  << std::fixed << std::setprecision(2)
                  << original_collision.time << "s using "
                  << (sign < 0 ? "right" : "left")
                  << " lateral bump " << offset << "m over "
                  << radius << " waypoints." << std::endl;
        return true;
      }
    }
  }

  std::cout << "  [Segment] Local transfer repair found no tiny detour; "
            << "falling back to full transfer replanning." << std::endl;
  return false;
}

bool replan_transfer_segment_after_failed_schedule(
    Trajectory *traj,
    RobotMeta *robot,
    double segment_ready_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::string *out_failure_reason)
{
  if (!traj || !robot || !traj->is_transfer || !traj->transferred_object ||
      traj->waypoints.empty())
  {
    if (out_failure_reason)
      *out_failure_reason = "transfer replanning skipped: invalid segment";
    return false;
  }

  const Pose start_pose = timetable.get_pose(robot, segment_ready_time);
  const Pose goal_pose = traj->waypoints.back();
  robot->initial_pose = start_pose;

  std::cout << "  [Segment] Given transfer path is not schedulable; "
            << "attempting forward-only transfer replanning for "
            << traj->transferred_object->name << "." << std::endl;

  if (try_local_transfer_path_repair(traj, robot, segment_ready_time,
                                     timetable, entities, params))
  {
    return true;
  }

  PHAStar planner(robot, goal_pose, &timetable, &entities, params,
                  true, traj->transferred_object->name, segment_ready_time,
                  "Transfer recovery");
  planner.max_search_iterations = options.max_search_iterations;
  planner.set_planner_expansion_threads(options.planner_expansion_threads);

  PlanningResult replanned = planner.Planning_with_res(segment_ready_time);
  if (replanned.status != PlanningStatus::SUCCESS ||
      replanned.waypoints.empty())
  {
    if (out_failure_reason)
    {
      std::ostringstream oss;
      oss << "transfer replanning failed: "
          << planning_status_name(replanned.status);
      if (!replanned.failure_detail.empty())
        oss << " (" << replanned.failure_detail << ")";
      *out_failure_reason = oss.str();
    }
    std::cerr << "  [Segment] Transfer replanning failed." << std::endl;
    return false;
  }

  make_waypoint_times_relative(replanned.waypoints, segment_ready_time);
  traj->waypoints = std::move(replanned.waypoints);
  traj->start_time = segment_ready_time;
  traj->kind = TrajectoryKind::TRANSFER;
  traj->is_transfer = true;

  std::cout << "  [Segment] Transfer replanning produced "
            << traj->waypoints.size()
            << " forward-only waypoints; retrying scheduling." << std::endl;
  return true;
}

// ==========================================
// Path Segment Scheduling
// ==========================================

bool schedule_path_segment(const EdgePath &edge_path, EntityMeta *obj_meta,
                           RobotMeta *robot, TimeTable &timetable,
                           const Params &params,
                           const std::unordered_map<std::string, EntityMeta *> &entities,
                           const RuntimeOptions &options,
                           double source_pre_push_distance,
                           EntityMeta *approach_goal_entity,
                           std::vector<TransferContactWindow> *transfer_windows,
                           TaskExecutionStats *stats,
                           std::string *out_failure_reason,
                           double *out_scheduled_start_time,
                           Trajectory *out_scheduled_trajectory)
{
  double current_avail_time = timetable.get_entity_max_time(robot);

  TrajectoryPtr traj =
      ReloPushPath2TrajPtr(edge_path, robot, obj_meta, current_avail_time,
                           source_pre_push_distance, approach_goal_entity);

  if (!prepare_segment_waypoints_for_scheduling(
          traj.get(), robot, current_avail_time, timetable, entities, params,
          options,
          "  [Segment] Transit replanning failed; using original transit path with conflict-resolution scheduling."))
  {
    if (out_failure_reason)
      *out_failure_reason = "empty transit segment";
    return false;
  }

  CollisionInfo segment_failure_collision;
  double segment_failure_start = current_avail_time;
  bool success = reserve_and_commit_trajectory(
      traj.get(), robot, current_avail_time, timetable, params, entities,
      options, transfer_windows, stats, out_failure_reason,
      &segment_failure_collision, &segment_failure_start);

  if (!success && traj->is_transfer && traj->transferred_object)
  {
    std::string transfer_replan_failure;
    if (replan_transfer_segment_after_failed_schedule(
            traj.get(), robot, current_avail_time, timetable, entities,
            params, options, &transfer_replan_failure))
    {
      success = reserve_and_commit_trajectory(
          traj.get(), robot, current_avail_time, timetable, params, entities,
          options, transfer_windows, stats, out_failure_reason,
          &segment_failure_collision, &segment_failure_start);
    }
    else if (out_failure_reason && !transfer_replan_failure.empty())
    {
      *out_failure_reason = transfer_replan_failure;
    }
  }

  if (success)
  {
    if (out_scheduled_start_time)
      *out_scheduled_start_time = traj->start_time;
    if (out_scheduled_trajectory)
      *out_scheduled_trajectory = *traj;
  }
  return success;
}

// ==========================================
// Full Task Execution Pipeline
// ==========================================

bool process_task_execution(
    RobotMeta *robot, Task &task, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows,
    TaskExecutionStats *out_stats,
    std::string *out_failure_reason,
    double task_start_delay,
    int task_id)
{
  if (out_stats)
    *out_stats = TaskExecutionStats{};

  auto set_failure = [&](const std::string &reason)
  {
    if (out_failure_reason)
      *out_failure_reason = reason;
  };

  auto build_task_debug_summary = [&](int highlight_segment_idx) -> std::string
  {
    std::ostringstream oss;
    oss << "Task execution failure summary";
    if (task.targetObject)
      oss << "\nTarget object: " << task.targetObject->name;
    if (robot)
      oss << "\nAssigned robot: " << robot->name;
    oss << "\nTask start pose (robot): ("
        << std::fixed << std::setprecision(2)
        << task.TaskStartPoseRobot.x << ", "
        << task.TaskStartPoseRobot.y << ", "
        << task.TaskStartPoseRobot.yaw << ")";
    oss << "\nTask goal pose (object): ("
        << task.GoalPoseObj.x << ", "
        << task.GoalPoseObj.y << ", "
        << task.GoalPoseObj.yaw << ")";
    oss << "\nVertex chain:";
    for (const auto &vertex : task.vertexChain)
    {
      oss << " " << vertex.name;
    }
    oss << "\nObs relocation paths: "
        << (task.obsReloPaths ? static_cast<int>(task.obsReloPaths->size()) : 0);
    oss << "\nEdge paths: " << static_cast<int>(task.EdgePaths.size());
    for (std::size_t idx = 0; idx < task.EdgePaths.size(); ++idx)
    {
      const auto &edge = task.EdgePaths[idx];
      oss << "\n  [" << (idx + 1) << "] "
          << ((edge && edge->is_transfer) ? "transfer" : "transit")
          << ", waypoints="
          << (edge ? static_cast<int>(edge->waypoints.size()) : -1);
      if (static_cast<int>(idx + 1) == highlight_segment_idx)
        oss << "  <-- failing segment";
    }
    return oss.str();
  };

  // 1. Plan Transit to Task Start
  double robot_avail_time = timetable.get_entity_max_time(robot) +
                            std::max(0.0, task_start_delay);
  if (out_stats && task_start_delay > 1e-9)
  {
    out_stats->total_waiting += task_start_delay;
  }
  task.TaskStartPoseRobot =
      compute_adjusted_task_start_pose(task, robot, robot_avail_time, timetable);
  const Pose initial_transit_start_pose = timetable.get_pose(robot, robot_avail_time);
  const Pose initial_transit_target_pose = task.TaskStartPoseRobot;
  std::vector<Waypoint> initial_transit_reference =
      waypoints_from_relopush_state_path(task.firstApproachPath);
  double initial_transit_abs_start = -1.0;
  double initial_transit_abs_end = -1.0;
  if (!plan_initial_transit(robot, task.TaskStartPoseRobot, robot_avail_time, timetable, entities, params, options, &initial_transit_abs_start, &initial_transit_abs_end, [&](double query_time)
                            { return compute_adjusted_task_start_pose(
                                  task, robot, query_time, timetable); }, &initial_transit_reference))
  {
    set_failure("initial transit planning failed");
    std::cerr << "Aborting task due to transit failure." << std::endl;
    return false;
  }
  if (out_stats && initial_transit_abs_start >= 0.0)
  {
    out_stats->has_initial_transit = true;
    out_stats->initial_transit_start_pose = initial_transit_start_pose;
    out_stats->initial_transit_target_pose = initial_transit_target_pose;
    out_stats->initial_transit_requested_start_time = robot_avail_time;
    out_stats->initial_transit_start_time = initial_transit_abs_start;
    out_stats->initial_transit_end_time = initial_transit_abs_end;
    out_stats->initial_transit_delay_scheduled =
        initial_transit_abs_start > robot_avail_time + 1e-6;
  }
  const Pose initial_wait_pose = timetable.get_pose(robot, initial_transit_abs_end);
  bool initial_wait_gap_checked = false;
  auto record_waiting_pose_conflict = [&](const Pose &wait_pose,
                                          const CollisionInfo &wait_conflict,
                                          const std::string &stage_label,
                                          bool mark_as_initial_gap) -> void
  {
    if (out_stats)
    {
      out_stats->has_waiting_pose_conflict = true;
      out_stats->waiting_pose = wait_pose;
      out_stats->waiting_conflict_time = wait_conflict.time;
      out_stats->waiting_conflict_entity = wait_conflict.entity_name;
      out_stats->waiting_conflict_stage = stage_label;
      if (mark_as_initial_gap)
      {
        out_stats->has_initial_wait_conflict = true;
        out_stats->initial_wait_pose = wait_pose;
        out_stats->initial_wait_conflict_time = wait_conflict.time;
        out_stats->initial_wait_conflict_entity = wait_conflict.entity_name;
      }
    }
  };

  auto validate_wait_gap = [&](const Pose &wait_pose,
                               double from_t,
                               double to_t,
                               const std::string &segment_label,
                               EntityMeta *ignored_entity = nullptr,
                               bool mark_as_initial_gap = false) -> bool
  {
    if (to_t <= from_t + 1e-9)
      return true;

    CollisionInfo wait_conflict = find_robot_waiting_pose_conflict_over_interval(
        robot, wait_pose, from_t, to_t, timetable, params, entities,
        ignored_entity);
    if (wait_conflict.is_valid)
      return true;

    record_waiting_pose_conflict(wait_pose, wait_conflict, segment_label,
                                 mark_as_initial_gap);

    std::ostringstream oss;
    oss << "waiting pose conflicts with reserved occupancy before "
        << segment_label;
    if (!wait_conflict.entity_name.empty())
      oss << " (" << wait_conflict.entity_name << ")";
    set_failure(oss.str());

    std::cerr << "  [Wait] Waiting pose is not safe before "
              << segment_label << ". Conflict: " << wait_conflict.reason;
    if (!wait_conflict.entity_name.empty())
      std::cerr << " with " << wait_conflict.entity_name;
    if (wait_conflict.time > 1e-6)
      std::cerr << " at t=" << std::fixed << std::setprecision(2)
                << wait_conflict.time;
    std::cerr << std::endl;

    if (DEBUG_VIS)
    {
      Trajectory wait_hint;
      wait_hint.entity = robot;
      wait_hint.start_time = from_t;
      wait_hint.is_transfer = false;
      wait_hint.kind = TrajectoryKind::TRANSIT;
      Waypoint wait_start;
      wait_start.x = wait_pose.x;
      wait_start.y = wait_pose.y;
      wait_start.yaw = wait_pose.yaw;
      wait_start.time = 0.0;
      Waypoint wait_end = wait_start;
      wait_end.time = std::max(0.0, to_t - from_t);
      wait_hint.waypoints = {wait_start, wait_end};

      std::ostringstream context;
      context << "Waiting pose conflict before " << segment_label
              << ". Robot " << (robot ? robot->name : "unknown")
              << " waits at this pose from t=" << std::fixed
              << std::setprecision(2) << from_t << " to " << to_t
              << " while reserved occupancy collides at t="
              << wait_conflict.time << ".";

      visualize_current_state(
          timetable, entities, params,
          wait_conflict.time > 1e-6 ? wait_conflict.time : from_t,
          wait_pose, wait_pose,
          &wait_hint, from_t,
          &wait_conflict,
          context.str());
    }
    return false;
  };

  auto validate_initial_wait_gap = [&](double next_segment_start_time,
                                       const std::string &segment_label,
                                       EntityMeta *ignored_entity = nullptr) -> bool
  {
    if (initial_wait_gap_checked)
      return true;
    initial_wait_gap_checked = true;

    if (next_segment_start_time <= initial_transit_abs_end + 1e-9)
      return true;

    return validate_wait_gap(initial_wait_pose, initial_transit_abs_end,
                             next_segment_start_time,
                             "initial transit -> " + segment_label,
                             ignored_entity, true);
  };

  // 2. ObsRelo (if exists)
  if (task.vertexChain.size() > 2)
  {
    size_t obs_path_base_idx = 0;
    for (size_t obs_ind = 1; obs_ind < task.vertexChain.size() - 1; obs_ind++)
    {
      std::cout << "Obstacle Relocation" << std::endl;

      std::string obs_name = task.vertexChain[obs_ind].name;
      auto obs_meta = entities.at(obs_name);

      size_t push_path_idx = obs_path_base_idx;
      size_t post_path_idx = obs_path_base_idx + 1;

      if (task.obsReloPaths->size() > post_path_idx)
      {
        const double obs_push_wait_start_time =
            timetable.get_entity_max_time(robot, 0.0);
        const Pose obs_push_wait_pose =
            timetable.get_pose(robot, obs_push_wait_start_time);
        double obs_push_start_time = -1.0;
        Trajectory obs_push_traj;
        if (!schedule_path_segment(task.obsReloPaths->at(push_path_idx), obs_meta,
                                   robot, timetable, params, entities, options,
                                   task.sourcePrePushDistance,
                                   nullptr,
                                   transfer_windows,
                                   out_stats, out_failure_reason,
                                   &obs_push_start_time,
                                   &obs_push_traj))
        {
          set_failure("obs relocation push segment failed");
          return false;
        }
        if (!validate_initial_wait_gap(obs_push_start_time,
                                       "obstacle relocation push",
                                       obs_meta))
        {
          return false;
        }
        if (!validate_wait_gap(obs_push_wait_pose, obs_push_wait_start_time,
                               obs_push_start_time,
                               "obstacle relocation push", obs_meta))
        {
          return false;
        }

        if (!append_retraction(robot, obs_push_traj, timetable, params,
                               entities, options, out_stats,
                               out_failure_reason))
        {
          std::cout << "  [Retract] Skipping retraction after obstacle relocation push "
                    << "(no collision-free slot)." << std::endl;
        }

        EntityMeta *return_approach_entity = task.targetObject;
        if (obs_ind + 1 < task.vertexChain.size() - 1)
        {
          auto next_obs_it = entities.find(task.vertexChain[obs_ind + 1].name);
          if (next_obs_it != entities.end())
            return_approach_entity = next_obs_it->second;
        }

        const double obs_return_wait_start_time =
            timetable.get_entity_max_time(robot, 0.0);
        const Pose obs_return_wait_pose =
            timetable.get_pose(robot, obs_return_wait_start_time);
        double obs_return_start_time = -1.0;
        if (!schedule_path_segment(task.obsReloPaths->at(post_path_idx), nullptr,
                                   robot, timetable, params, entities, options,
                                   task.sourcePrePushDistance,
                                   return_approach_entity,
                                   transfer_windows,
                                   out_stats, out_failure_reason,
                                   &obs_return_start_time))
        {
          set_failure("obs relocation return segment failed");
          return false;
        }
        if (!validate_wait_gap(obs_return_wait_pose, obs_return_wait_start_time,
                               obs_return_start_time,
                               "obstacle relocation return"))
        {
          return false;
        }
        obs_path_base_idx += 2;
      }
      else
      {
        std::cerr << "Error: obsReloPaths missing required paths for index "
                  << obs_ind << std::endl;
        return false;
      }
    }
  }

  // 3. Execute Edge Paths
  std::vector<bool> empty_noop_connectors(task.EdgePaths.size(), false);
  for (std::size_t idx = 0; idx < task.EdgePaths.size(); ++idx)
  {
    empty_noop_connectors[idx] =
        is_empty_noop_connector_between_pushes(task.EdgePaths, idx);
  }

  int segment_idx = 0;
  for (auto &path_ptr : task.EdgePaths)
  {
    segment_idx++;
    const std::size_t edge_path_idx = static_cast<std::size_t>(segment_idx - 1);
    double segment_ready_time = timetable.get_entity_max_time(robot);
    double segment_wait_start_time = timetable.get_entity_max_time(robot, 0.0);
    Pose segment_wait_pose = timetable.get_pose(robot, segment_wait_start_time);

    if (edge_path_idx < empty_noop_connectors.size() &&
        empty_noop_connectors[edge_path_idx])
    {
      std::cout << "  [Segment " << segment_idx
                << "] Empty transit between continuous pushes; treating as no-op."
                << std::endl;
      continue;
    }

    std::ostringstream fallback_msg;
    fallback_msg << "  [Segment " << segment_idx
                 << "] Transit replanning failed; no MARS-generated segment path accepted.";
    SegmentReplanContext replan_context;
    replan_context.task_id = task_id;
    replan_context.segment_id = segment_idx;
    replan_context.object_name =
        task.targetObject ? task.targetObject->name : "unknown";
    if (!prepare_segment_waypoints_for_scheduling(path_ptr.get(), robot,
                                                  segment_ready_time, timetable,
                                                  entities, params, options,
                                                  fallback_msg.str(),
                                                  replan_context))
    {
      set_failure("segment transit replanning failed");
      std::cerr << " [Error] Segment " << segment_idx
                << " transit replanning failed." << std::endl;
      if (DEBUG_VIS)
      {
        Pose start_pose = timetable.get_pose(robot, segment_ready_time);
        std::ostringstream context;
        context << "Segment " << segment_idx
                << " is empty before scheduling.\n";
        context << build_task_debug_summary(segment_idx);
        visualize_current_state(
            timetable, entities, params, segment_ready_time,
            start_pose, task.GoalPoseObj,
            nullptr, segment_ready_time,
            nullptr, context.str());
      }
      return false;
    }

    Pose segment_goal = path_ptr->waypoints.back();

    std::cout << "  [Segment " << segment_idx << "] Checking schedule..."
              << std::endl;
    CollisionInfo segment_failure_collision;
    double segment_failure_start = segment_ready_time;
    bool segment_committed = reserve_and_commit_trajectory(
        path_ptr.get(), robot, segment_ready_time, timetable, params,
        entities, options, transfer_windows, out_stats,
        out_failure_reason, &segment_failure_collision,
        &segment_failure_start);

    if (!segment_committed && path_ptr->is_transfer &&
        path_ptr->transferred_object)
    {
      std::string transfer_replan_failure;
      if (replan_transfer_segment_after_failed_schedule(
              path_ptr.get(), robot, segment_ready_time, timetable, entities,
              params, options, &transfer_replan_failure))
      {
        segment_failure_collision =
            CollisionInfo{true, "Not evaluated", "", segment_ready_time};
        segment_failure_start = segment_ready_time;
        segment_committed = reserve_and_commit_trajectory(
            path_ptr.get(), robot, segment_ready_time, timetable, params,
            entities, options, transfer_windows, out_stats,
            out_failure_reason, &segment_failure_collision,
            &segment_failure_start);
      }
      else if (out_failure_reason && !transfer_replan_failure.empty())
      {
        *out_failure_reason = transfer_replan_failure;
      }
    }

    if (!segment_committed)
    {
      std::ostringstream oss;
      oss << "segment " << segment_idx << " failed (no safe start time)";
      set_failure(oss.str());
      std::cerr << " [Error] Segment " << segment_idx
                << " failed (permanent blockage or empty path)" << std::endl;
      if (DEBUG_VIS)
      {
        Pose start_pose = timetable.get_pose(robot, segment_ready_time);

        double debug_query_time = segment_ready_time;
        if (segment_failure_collision.time > 1e-6)
          debug_query_time = segment_failure_collision.time;

        std::ostringstream context;
        context << "Segment " << segment_idx << " scheduling failed. "
                << "Attempted start=" << std::fixed << std::setprecision(2)
                << segment_failure_start << "s";

        visualize_current_state(timetable, entities, params, debug_query_time,
                                start_pose, segment_goal,
                                path_ptr.get(), segment_failure_start,
                                &segment_failure_collision,
                                context.str());
      }
      return false;
    }
    EntityMeta *allowed_contact_entity =
        path_ptr->is_transfer ? path_ptr->transferred_object : nullptr;
    if (!validate_initial_wait_gap(path_ptr->start_time,
                                   "segment " + std::to_string(segment_idx),
                                   allowed_contact_entity))
    {
      return false;
    }
    if (!validate_wait_gap(segment_wait_pose, segment_wait_start_time,
                           path_ptr->start_time,
                           "segment " + std::to_string(segment_idx),
                           allowed_contact_entity))
    {
      return false;
    }

    if (path_ptr->is_transfer)
    {
      const bool next_connector_is_noop =
          edge_path_idx + 1 < empty_noop_connectors.size() &&
          empty_noop_connectors[edge_path_idx + 1];
      if (next_connector_is_noop)
      {
        std::cout << "  [Retract] Skipping retraction after segment "
                  << segment_idx
                  << " because the next empty connector continues directly into another push."
                  << std::endl;
        continue;
      }
      if (!append_retraction(robot, *path_ptr, timetable, params, entities,
                             options,
                             out_stats, out_failure_reason))
      {
        std::cout << "  [Retract] Skipping retraction after segment "
                  << segment_idx << " (no collision-free slot)." << std::endl;
      }
    }
  }
  return true;
}

// ==========================================
// Task Loop Helpers
// ==========================================

static std::vector<std::pair<RobotMeta *, double>>
prepare_task_candidates(Task &task,
                        const std::vector<RobotMeta *> &all_robots,
                        TimeTable &timetable,
                        const Params &params)
{
  auto candidates =
      get_sorted_candidate_robots(all_robots, timetable, task, params);

  if (task.assignedRobot)
  {
    auto it = std::find_if(candidates.begin(), candidates.end(),
                           [&](const auto &p)
                           { return p.first == task.assignedRobot; });
    if (it != candidates.end())
    {
      std::rotate(candidates.begin(), it, it + 1);
    }
  }

  return candidates;
}

static TaskCsvRow initialize_task_row(
    int task_id,
    const Task &task,
    const std::vector<std::pair<RobotMeta *, double>> &candidates)
{
  TaskCsvRow row;
  row.task_id = task_id;
  row.object_name = task.targetObject ? task.targetObject->name : "unknown";
  row.status = "FAILED";
  row.robot_name = "";
  row.start_time = candidates.empty() ? -1.0 : candidates.front().second;
  row.end_time = -1.0;
  row.total_waiting = 0.0;
  row.attempts = 0;
  row.failure_reason = "";
  return row;
}

// ==========================================
// Candidate Attempt & Recovery
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
    std::string &last_failure_reason)
{
  TimeTable timetable_before_attempt = timetable;
  auto transfer_windows_before_attempt = transfer_windows;
  TimeTable retry_base_timetable = timetable_before_attempt;
  auto retry_base_transfer_windows = transfer_windows_before_attempt;
  task.assignedRobot = cand_robot;
  std::cout << "[Assign] Attempting " << cand_robot->name
            << " (Free at t=" << std::fixed << std::setprecision(2) << free_time << "s)" << std::endl;

  constexpr double kPostValidationRetryDelay = 0.5;
  constexpr int kMaxPostValidationDelayRetries = 4;
  double task_start_delay = 0.0;
  bool self_safe_parking_retry_applied = false;

  auto make_single_pose_hint = [&](const Pose &pose, double abs_time) -> Trajectory
  {
    Trajectory hint;
    hint.entity = cand_robot;
    hint.start_time = abs_time;
    hint.is_transfer = false;

    Waypoint wp;
    wp.x = pose.x;
    wp.y = pose.y;
    wp.yaw = pose.yaw;
    wp.time = 0.0;
    wp.linear_velocity = 0.0;
    wp.steering_angle = 0.0;
    hint.waypoints.push_back(wp);
    return hint;
  };

  for (int retry_idx = 0; retry_idx <= kMaxPostValidationDelayRetries; ++retry_idx)
  {
    timetable = retry_base_timetable;
    transfer_windows = retry_base_transfer_windows;

    if (retry_idx > 0)
    {
      std::cout << "         Retrying " << cand_robot->name
                << " with extra task-start delay of "
                << std::fixed << std::setprecision(2)
                << task_start_delay << "s." << std::endl;
    }

    TaskExecutionStats attempt_stats;
    std::string attempt_failure_reason;
    if (process_task_execution(cand_robot, task, timetable, entities, params,
                               options,
                               &transfer_windows,
                               &attempt_stats, &attempt_failure_reason,
                               task_start_delay, row.task_id))
    {
      auto verify = verify_timetable_collision_free(timetable, entities, params,
                                                    transfer_windows,
                                                    0.0);
      if (!verify.is_valid)
      {
        std::ostringstream vmsg;
        vmsg << "post-task verification collision at t="
             << std::fixed << std::setprecision(2)
             << verify.time << " between " << verify.entity_a
             << " and " << verify.entity_b;
        attempt_failure_reason = vmsg.str();

        EntityMeta *entity_a = nullptr;
        EntityMeta *entity_b = nullptr;
        auto entity_a_it = entities.find(verify.entity_a);
        if (entity_a_it != entities.end())
          entity_a = entity_a_it->second;
        auto entity_b_it = entities.find(verify.entity_b);
        if (entity_b_it != entities.end())
          entity_b = entity_b_it->second;

        const bool candidate_involved =
            (verify.entity_a == cand_robot->name ||
             verify.entity_b == cand_robot->name);
        const std::string other_entity_name =
            (verify.entity_a == cand_robot->name)
                ? verify.entity_b
                : ((verify.entity_b == cand_robot->name) ? verify.entity_a
                                                         : "");
        const bool collision_during_initial_transit =
            attempt_stats.has_initial_transit &&
            candidate_involved &&
            verify.time + 1e-6 >= attempt_stats.initial_transit_start_time &&
            verify.time <= attempt_stats.initial_transit_end_time + 1e-6;
        const bool later_transit_priority_inversion =
            collision_during_initial_transit &&
            !other_entity_name.empty() &&
            (!task.targetObject || other_entity_name != task.targetObject->name);

        std::cout << "         [VerifyDebug] "
                  << describe_entity_pose_context(timetable, entity_a, verify.time)
                  << std::endl;
        std::cout << "         [VerifyDebug] "
                  << describe_entity_pose_context(timetable, entity_b, verify.time)
                  << std::endl;
        std::cout << "         [VerifyDebug] "
                  << summarize_pair_collision_window(timetable, entity_a, entity_b,
                                                     params, verify.time)
                  << std::endl;
        std::cout << "         [VerifyDebug] "
                  << compare_pair_collision_checks(timetable, entity_a, entity_b,
                                                   params, verify.time)
                  << std::endl;

        if (DEBUG_VIS)
        {
          std::ostringstream context;
          context << "Task " << row.task_id;
          if (task.targetObject)
            context << " (" << task.targetObject->name << ")";
          context << "\nCandidate robot: " << cand_robot->name;
          context << "\nVerification result recorded after successful task execution";
          visualize_post_task_verification_debug(
              timetable, cand_robot, task.targetObject,
              verify.time, verify.reason, verify.entity_a, verify.entity_b, params,
              context.str());
        }

        if (later_transit_priority_inversion &&
            retry_idx < kMaxPostValidationDelayRetries)
        {
          task_start_delay += kPostValidationRetryDelay;
          std::cout << "         Earlier reserved occupancy keeps priority over "
                    << cand_robot->name
                    << "'s later initial transit. Adding "
                    << std::fixed << std::setprecision(2)
                    << kPostValidationRetryDelay
                    << "s task-start delay and retrying the same robot."
                    << std::endl;
          continue;
        }

        if (later_transit_priority_inversion &&
            !self_safe_parking_retry_applied)
        {
          TimeTable self_park_base_timetable = retry_base_timetable;
          auto self_park_base_transfer_windows = retry_base_transfer_windows;
          Pose conflict_pose = timetable.get_pose(cand_robot, verify.time);
          Trajectory blocked_hint = make_single_pose_hint(conflict_pose, verify.time);

          std::cout << "         Earlier relocation kept priority after delay retries."
                    << " Attempting self safe parking for " << cand_robot->name
                    << " before replanning the task." << std::endl;

          if (relocate_blocking_robot(cand_robot, self_park_base_timetable,
                                      params, entities, options, &blocked_hint))
          {
            retry_base_timetable = std::move(self_park_base_timetable);
            retry_base_transfer_windows = std::move(self_park_base_transfer_windows);
            self_safe_parking_retry_applied = true;
            task_start_delay = 0.0;
            retry_idx = -1;
            std::cout << "         " << cand_robot->name
                      << " reached safe parking. Re-running the same task while preserving earlier reservations."
                      << std::endl;
            continue;
          }

          std::cout << "         Self safe parking retry failed for "
                    << cand_robot->name << "." << std::endl;
        }

        if (!later_transit_priority_inversion &&
            retry_idx < kMaxPostValidationDelayRetries)
        {
          task_start_delay += kPostValidationRetryDelay;
          std::cout << "         Verification collision remained after planning."
                    << " Adding " << std::fixed << std::setprecision(2)
                    << kPostValidationRetryDelay
                    << "s more task-start delay and retrying the same robot."
                    << std::endl;
          continue;
        }

        timetable = timetable_before_attempt;
        transfer_windows = transfer_windows_before_attempt;
        last_failed_robot = cand_robot->name;
        last_failure_reason = attempt_failure_reason;
        std::cout << "[Assign] FAILED with " << cand_robot->name
                  << " (Post-Task Verification Failed) - Trying next..."
                  << std::endl;
        std::cout << "         Verification: collision at t="
                  << std::fixed << std::setprecision(2) << verify.time
                  << " between " << verify.entity_a
                  << " and " << verify.entity_b << std::endl;
        return false;
      }

      row.status = "SUCCESS";
      row.robot_name = cand_robot->name;
      row.start_time = attempt_stats.has_initial_transit
                           ? attempt_stats.initial_transit_start_time
                           : (free_time + task_start_delay);
      row.end_time = timetable.get_entity_max_time(cand_robot);
      row.total_waiting = attempt_stats.total_waiting;
      row.failure_reason = "";
      std::cout << "[Assign] SUCCESS with " << cand_robot->name << std::endl;
      return true;
    }

    if (attempt_stats.has_waiting_pose_conflict)
    {
      const bool delay_only_recovery_was_used =
          attempt_stats.initial_transit_delay_scheduled ||
          attempt_stats.delayed_segments > 0;

      if (delay_only_recovery_was_used)
      {
        std::cout << "         Reusing delayed initial transit; no full PHAStar rerun.";
      }
      else
      {
        std::cout << "         No reusable delayed initial transit was recorded; "
                  << "using self safe parking rather than small delay retries.";
      }

      if (!attempt_stats.waiting_conflict_stage.empty())
        std::cout << " Conflict before " << attempt_stats.waiting_conflict_stage;
      std::cout << "." << std::endl;
    }

    if (attempt_stats.has_waiting_pose_conflict &&
        !self_safe_parking_retry_applied)
    {
      TimeTable self_park_base_timetable = retry_base_timetable;
      auto self_park_base_transfer_windows = retry_base_transfer_windows;
      const double conflict_time =
          attempt_stats.waiting_conflict_time > 0.0
              ? attempt_stats.waiting_conflict_time
              : attempt_stats.initial_transit_end_time;
      Trajectory blocked_hint =
          make_single_pose_hint(attempt_stats.waiting_pose, conflict_time);

      std::cout << "         Repeated waiting conflict after reuse; switching to self safe parking. "
                << "Delay-only scheduling already reached the conflicting wait pose while "
                << cand_robot->name
                << " waits before continuing the task."
                << (attempt_stats.waiting_conflict_stage.empty()
                        ? ""
                        : (" Problem stage: " +
                           attempt_stats.waiting_conflict_stage + "."))
                << " Attempting self safe parking before replanning the task."
                << std::endl;

      if (relocate_blocking_robot(cand_robot, self_park_base_timetable,
                                  params, entities, options, &blocked_hint))
      {
        retry_base_timetable = std::move(self_park_base_timetable);
        retry_base_transfer_windows = std::move(self_park_base_transfer_windows);
        self_safe_parking_retry_applied = true;
        task_start_delay = 0.0;
        retry_idx = -1;
        std::cout << "         " << cand_robot->name
                  << " reached safe parking. Re-running the same task while preserving earlier reservations."
                  << std::endl;
        continue;
      }

      std::cout << "         Self safe parking retry failed for "
                << cand_robot->name << "." << std::endl;
    }

    timetable = timetable_before_attempt;
    transfer_windows = transfer_windows_before_attempt;
    last_failed_robot = cand_robot->name;
    last_failure_reason = attempt_failure_reason.empty() ? "task execution failed" : attempt_failure_reason;
    std::cout << "[Assign] FAILED with " << cand_robot->name << " (Transit Blocked) - Trying next..." << std::endl;
    if (!last_failure_reason.empty())
    {
      std::cout << "         Reason: " << last_failure_reason << std::endl;
    }
    return false;
  }

  timetable = timetable_before_attempt;
  transfer_windows = transfer_windows_before_attempt;
  last_failed_robot = cand_robot->name;
  last_failure_reason = "unexpected exhausted retry loop";
  return false;
}

bool maybe_safe_park_repeated_initial_transit_failure(
    RobotMeta *robot,
    double free_time,
    const std::string &failure_reason,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options)
{
  if (!options.enable_failed_candidate_idle_parking || !robot)
    return false;

  if (failure_reason.find("initial transit planning failed") ==
      std::string::npos)
  {
    return false;
  }

  const double robot_ready_time = timetable.get_entity_max_time(robot);
  if (std::abs(robot_ready_time - free_time) > 1e-3)
  {
    clear_initial_transit_failures_for_robot(robot);
    return false;
  }

  const Pose idle_pose = timetable.get_pose(robot, robot_ready_time);
  const std::string fail_key = initial_transit_failure_key(robot, idle_pose);
  int &failure_count = g_initial_transit_failure_counts[fail_key];
  failure_count += 1;

  const int threshold = std::max(
      1, options.failed_candidate_initial_transit_failure_threshold);
  std::cout << "         [CandidateRecovery] " << robot->name
            << " initial transit failed from idle pose "
            << rounded_pose_key(idle_pose) << " (" << failure_count
            << "/" << threshold << ")." << std::endl;

  if (failure_count < threshold)
    return false;

  Trajectory blocked_hint;
  blocked_hint.entity = robot;
  blocked_hint.start_time = robot_ready_time;
  blocked_hint.is_transfer = false;
  blocked_hint.kind = TrajectoryKind::TRANSIT;
  Waypoint wait_pose;
  wait_pose.x = idle_pose.x;
  wait_pose.y = idle_pose.y;
  wait_pose.yaw = idle_pose.yaw;
  wait_pose.time = 0.0;
  wait_pose.linear_velocity = 0.0;
  wait_pose.steering_angle = 0.0;
  blocked_hint.waypoints.push_back(wait_pose);

  std::cout << "         [CandidateRecovery] " << robot->name
            << " failed initial transit from this idle pose " << failure_count
            << " times; safe-parking before next candidate." << std::endl;

  if (!relocate_blocking_robot(robot, timetable, params, entities, options,
                               &blocked_hint))
  {
    std::cout << "         [CandidateRecovery] Safe parking failed for "
              << robot->name << "; keeping existing pose." << std::endl;
    return false;
  }

  clear_initial_transit_failures_for_robot(robot);
  std::cout << "         [CandidateRecovery] " << robot->name
            << " safe-parked after repeated initial-transit failures."
            << std::endl;
  return true;
}

TaskCsvRow execute_single_task_with_candidates(
    Task &task,
    int task_counter,
    const std::vector<RobotMeta *> &all_robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> &transfer_windows)
{
  std::cout << "\n=== Processing Task " << task_counter << " ("
            << task.targetObject->name << ") ===" << std::endl;

  auto candidates = prepare_task_candidates(task, all_robots, timetable, params);
  TaskCsvRow row = initialize_task_row(task_counter, task, candidates);

  bool task_success = false;
  std::string last_failed_robot = "";
  std::string last_failure_reason = "";

  for (auto &[cand_robot, free_time] : candidates)
  {
    row.attempts += 1;
    if (attempt_task_with_candidate(task, cand_robot, free_time,
                                    timetable, entities, params, options,
                                    transfer_windows, row,
                                    last_failed_robot, last_failure_reason))
    {
      clear_initial_transit_failures_for_robot(cand_robot);
      task_success = true;
      break;
    }

    maybe_safe_park_repeated_initial_transit_failure(
        cand_robot, free_time, last_failure_reason, timetable, entities,
        params, options);
  }

  if (!task_success)
  {
    row.status = "FAILED";
    row.robot_name = last_failed_robot;
    row.failure_reason = last_failure_reason.empty() ? "all candidate robots failed" : last_failure_reason;
    std::cerr << "[Critical] Task " << task_counter << " failed with ALL available robots." << std::endl;
  }

  return row;
}

std::vector<TaskCsvRow> execute_task_allocation_loop(
    std::vector<Task> &tasks,
    const std::vector<RobotMeta *> &all_robots,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options)
{
  std::vector<TaskCsvRow> task_rows;
  std::vector<TransferContactWindow> transfer_windows;

  int task_counter = 0;
  for (auto &task : tasks)
  {
    task_counter++;
    auto row = execute_single_task_with_candidates(
        task, task_counter, all_robots, timetable, entities, params,
        options,
        transfer_windows);
    task_rows.push_back(std::move(row));
  }

  return task_rows;
}
