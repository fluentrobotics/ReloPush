/*****************************************************************
 * Planning Diagnostic / Utility Helpers
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <PlanningHelpers.h>
#include <PHAstarPushDemoTypes.h>
#include <PHAstar/CollisionUtils.h>
#include <PHAstar/TimeTable.h>
#include <PHAstar/Utils.h>
#include <PHAstar/Visualization.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/Reeds_Shepp.h>
#include <SafeParking.h>
#include <CsvLogging.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <filesystem>
#include <fstream>

// Enum definitions moved to PHAstarPushDemoTypes.h

// Forward declarations for functions still in the monolith
struct Node;

// Forward declarations for functions still in the monolith that are called by extracted functions
double find_safe_start_time(Trajectory *traj, double earliest_start,
                            TimeTable &timetable, const Params &params,
                            const std::unordered_map<std::string, EntityMeta *> &entities,
                            const RuntimeOptions &options,
                            double *out_wait_added,
                            CollisionInfo *out_last_collision = nullptr,
                            double *out_last_check_time = nullptr,
                            IdleBlockerRelocationPolicy idle_blocker_policy =
                                IdleBlockerRelocationPolicy::RelocateAnyIdle,
                            PlanTimingStats *plan_stats = nullptr);

CollisionInfo check_collision_trajectory_detailed(const Trajectory &traj, double start_time,
                                                  TimeTable &timetable, const Params &params,
                                                  bool verbose,
                                                  EntityMeta *terminal_approach_entity = nullptr);

CollisionInfo check_trajectory_motion_and_terminal_hold_detailed(
    const Trajectory &traj, double start_time, TimeTable &timetable,
    const Params &params, EntityMeta *terminal_approach_entity);

bool reserve_and_commit_trajectory(
    Trajectory *traj, TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params, TaskExecutionStats *stats,
    std::string *out_failure_reason);

RobotMeta *find_resolvable_robot_blocker(
    const CollisionInfo &col_info,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities);

double shared_collision_check_step(const Params &params);

double timetable_delay_search_horizon(double earliest_start,
                                     TimeTable &timetable,
                                     double step);

bool validate_pre_commit_trajectory(
    TimeTable &trial_timetable,
    const Trajectory &traj,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> *transfer_windows,
    CollisionInfo *out_collision,
    std::string *out_failure_reason);

TimeTableVerificationResult verify_timetable_collision_free(
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params);

// visualize_planning_attempt_debug and visualize_planning_debug - defined in monolith
// sanitize_filename_component - defined in monolith

bool is_valid_transfer_contact(EntityMeta *e1, const Pose &p1,
                               EntityMeta *e2, const Pose &p2);

bool is_valid_terminal_approach_contact(RobotMeta *robot,
                                        const Pose &robot_pose,
                                        const Pose &goal_pose,
                                        EntityMeta *collider,
                                        const Pose &collider_pose,
                                        EntityMeta *terminal_approach_entity,
                                        const Params &params);

std::tuple<double, double, double> interpolate_timed_path(
    const std::vector<Waypoint> &path, double relative_time);

std::string find_valid_start_contact_entity(
    RobotMeta *robot,
    const Pose &start_pose,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities);

bool poses_approximately_equal(const Pose &a, const Pose &b,
                               double pos_tol,
                               double yaw_tol)
{
  return std::hypot(a.x - b.x, a.y - b.y) <= pos_tol &&
         std::abs(pi_2_pi(a.yaw - b.yaw)) <= yaw_tol;
}

std::string format_pose_compact(const Pose &pose)
{
  std::ostringstream oss;
  oss << "("
      << std::fixed << std::setprecision(2)
      << pose.x << ", " << pose.y << ", " << pose.yaw << ")";
  return oss.str();
}

std::string describe_entity_pose_context(const TimeTable &timetable,
                                         EntityMeta *entity,
                                         double t)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);

  if (!entity)
  {
    oss << "<null entity>";
    return oss.str();
  }

  const Pose pose_at_t = timetable.get_pose(entity, t);
  oss << entity->name << " @ t=" << t
      << " pose=" << format_pose_compact(pose_at_t);

  const auto &db = timetable.get_database();
  auto ent_it = db.find(entity);
  if (ent_it == db.end() || ent_it->second.empty())
  {
    oss << " | no timetable samples";
    return oss.str();
  }

  const auto &samples = ent_it->second;
  auto next_it = samples.lower_bound(t);

  if (next_it != samples.begin())
  {
    auto prev_it = std::prev(next_it);
    oss << " | prev=(" << prev_it->first << ", "
        << format_pose_compact(prev_it->second) << ")";
  }
  else
  {
    oss << " | prev=(none)";
  }

  if (next_it != samples.end())
  {
    oss << " | next=(" << next_it->first << ", "
        << format_pose_compact(next_it->second) << ")";
  }
  else
  {
    auto last_it = std::prev(samples.end());
    oss << " | next=(end, " << last_it->first << ", "
        << format_pose_compact(last_it->second) << ")";
  }

  if (next_it != samples.begin() && next_it != samples.end())
  {
    const auto prev_it = std::prev(next_it);
    const double dt = next_it->first - prev_it->first;
    const double dist = std::hypot(next_it->second.x - prev_it->second.x,
                                   next_it->second.y - prev_it->second.y);
    if (dt > 1e-6)
    {
      oss << " | local_dt=" << dt
          << " | local_move=" << dist;
    }

    if (poses_approximately_equal(prev_it->second, next_it->second))
    {
      oss << " | stationary-window";
    }
  }

  return oss.str();
}

std::string summarize_pair_collision_window(const TimeTable &timetable,
                                            EntityMeta *entity_a,
                                            EntityMeta *entity_b,
                                            const Params &params,
                                            double center_time,
                                            double horizon,
                                            double step)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);

  if (!entity_a || !entity_b)
  {
    oss << "pair window unavailable";
    return oss.str();
  }

  const double start_t = std::max(0.0, center_time - horizon);
  const double end_t = center_time + horizon;

  bool in_collision = false;
  double collision_start = -1.0;
  double collision_end = -1.0;
  int collision_samples = 0;

  for (double t = start_t; t <= end_t + 1e-9; t += step)
  {
    const Pose pose_a = timetable.get_pose(entity_a, t);
    const Pose pose_b = timetable.get_pose(entity_b, t);
    const Corners corners_a =
        get_collision_corners_for_type(pose_a, entity_a->type, entity_a->size, params);
    const Corners corners_b =
        get_collision_corners_for_type(pose_b, entity_b->type, entity_b->size, params);
    const bool colliding = rectangles_intersect(corners_a, corners_b);

    if (colliding)
    {
      if (!in_collision)
      {
        in_collision = true;
        collision_start = t;
      }
      collision_end = t;
      collision_samples += 1;
    }
    else if (in_collision)
    {
      break;
    }
  }

  if (collision_samples == 0)
  {
    oss << "No repeated pair collision found in ["
        << start_t << ", " << end_t << "]";
    return oss.str();
  }

  oss << "Pair collision window ~[" << collision_start
      << ", " << collision_end << "]"
      << " sampled every " << step << "s"
      << " (" << collision_samples << " samples)";
  return oss.str();
}

std::string compare_pair_collision_checks(const TimeTable &timetable,
                                          EntityMeta *entity_a,
                                          EntityMeta *entity_b,
                                          const Params &params,
                                          double t)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);

  if (!entity_a || !entity_b)
  {
    oss << "pair check unavailable";
    return oss.str();
  }

  const Pose pose_a = timetable.get_pose(entity_a, t);
  const Pose pose_b = timetable.get_pose(entity_b, t);
  const CollisionGeometry geom_a =
      setup_collision_geometry_for_type(pose_a, entity_a->type, entity_a->size, params);
  const auto fast_check = check_entity_collision(geom_a, pose_a, entity_b, pose_b, params);
  const Corners corners_a =
      get_collision_corners_for_type(pose_a, entity_a->type, entity_a->size, params);
  const Corners corners_b =
      get_collision_corners_for_type(pose_b, entity_b->type, entity_b->size, params);
  const bool direct_overlap = rectangles_intersect(corners_a, corners_b);

  const double dist = std::hypot(pose_a.x - pose_b.x, pose_a.y - pose_b.y);
  const double other_inflation =
      collision_inflation_for_type(entity_b->type, params);
  const double other_diag =
      collision_origin_radius(entity_b->size, other_inflation);
  const double fast_threshold =
      geom_a.diagonal_radius + other_diag + params.safety_margin;

  oss << "pair direct_overlap=" << (direct_overlap ? "true" : "false")
      << ", fast_check=" << (fast_check.has_collision ? "true" : "false")
      << ", center_dist=" << dist
      << ", fast_threshold=" << fast_threshold;
  return oss.str();
}

// ==========================================
// Contact and Approach Pose Helpers (Block A)
// ==========================================

double contact_offset_for_object(const RobotMeta *robot,
                                 const EntityMeta *object,
                                 double extra_clearance)
{
  if (!robot || !object)
    return extra_clearance;

  return robot->size.front_length + object->size.rear_length + extra_clearance;
}

Pose recover_object_centric_pose_from_raw_terminal(const Pose &raw_terminal_pose,
                                                   double source_pre_push_distance)
{
  ReloPush::State raw_state(raw_terminal_pose.x, raw_terminal_pose.y,
                            mod2pi(raw_terminal_pose.yaw));
  return PoseFromReloPushState(raw_state.get_postPush(source_pre_push_distance));
}

Pose compute_adjusted_prepush_goal(const Pose &object_pose,
                                   double pushing_yaw,
                                   const RobotMeta *robot,
                                   const EntityMeta *object,
                                   double extra_clearance)
{
  if (!robot || !object)
  {
    Pose fallback = object_pose;
    fallback.yaw = mod2pi(pushing_yaw);
    return fallback;
  }

  ReloPush::State object_centric_pose(object_pose.x, object_pose.y,
                                      mod2pi(pushing_yaw));
  const double pre_push_distance =
      contact_offset_for_object(robot, object, extra_clearance);
  return PoseFromReloPushState(
      object_centric_pose.get_prePush(pre_push_distance));
}

Pose compute_adjusted_task_start_pose(const Task &task,
                                      const RobotMeta *robot,
                                      double query_time,
                                      const TimeTable &timetable)
{
  Pose fallback = task.TaskStartPoseRobot;
  if (!robot || !task.initialApproachEntity)
    return fallback;

  const Pose object_pose =
      timetable.get_pose(task.initialApproachEntity, query_time);
  return compute_adjusted_prepush_goal(
      object_pose, fallback.yaw, robot, task.initialApproachEntity, 0.01);
}

bool rewrite_transfer_terminal_pose(Trajectory *traj, RobotMeta *robot)
{
  if (!traj || !traj->is_transfer || !robot || !traj->transferred_object ||
      traj->waypoints.empty())
  {
    return false;
  }

  const double source_pre_push_distance =
      (traj->source_pre_push_distance > 1e-9)
          ? traj->source_pre_push_distance
          : contact_offset_for_object(robot, traj->transferred_object, 0.01);

  const Pose raw_terminal_pose = traj->waypoints.back();
  const Pose object_goal_pose = recover_object_centric_pose_from_raw_terminal(
      raw_terminal_pose, source_pre_push_distance);

  // Keep transfer endpoints at exact contact distance. Adding extra gap here
  // can be a problem if the robot is too short because retraction happens
  // after the push segment is committed.
  const Pose adjusted_terminal_pose = compute_adjusted_prepush_goal(
      object_goal_pose, raw_terminal_pose.yaw, robot, traj->transferred_object,
      0.0);

  Waypoint &last_waypoint = traj->waypoints.back();
  last_waypoint.x = adjusted_terminal_pose.x;
  last_waypoint.y = adjusted_terminal_pose.y;
  last_waypoint.yaw = adjusted_terminal_pose.yaw;
  traj->CalcualteTimeStamps(robot);
  return true;
}

// ==========================================
// Planning Diagnostics (Block A)
// ==========================================

void diagnose_planning_failure(RobotMeta *robot, const Pose &start,
                               const Pose &goal, double time,
                               TimeTable &timetable,
                               const std::unordered_map<std::string, EntityMeta *> &entities,
                               const Params &params)
{
  std::cerr << "\n  [Diagnostics] Analyzing failure for " << robot->name
            << " at t=" << time << "s..." << std::endl;

  auto others_map = timetable.get_poses(time);
  std::vector<std::pair<EntityMeta *, Pose>> others(others_map.begin(),
                                                    others_map.end());

  // --- Helper Lambda: Is this a valid transfer? ---
  // ... (existing lambda) ...

  std::cout << "    Start: (" << start.x << ", " << start.y << ", " << start.yaw << ")" << std::endl;
  std::cout << "    Goal:  (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")" << std::endl;

  // Cross-check with Planner's internal check
  // NOTE: PHAStar is forward-declared; cannot instantiate here
  // if (diag_planner.start) { ... }

  auto is_valid_transfer = [](EntityMeta *e1, const Pose &p1, EntityMeta *e2,
                              const Pose &p2) -> bool
  {
    // 1. Identify Robot and Object
    RobotMeta *r =
        dynamic_cast<RobotMeta *>(e1->type == EntityType::ROBOT ? e1 : e2);
    ObjectMeta *o =
        dynamic_cast<ObjectMeta *>(e1->type == EntityType::OBJECT ? e1 : e2);
    if (!r || !o)
      return false; // Not a Robot-Object pair

    const Pose &r_pose = (e1 == r) ? p1 : p2;
    const Pose &o_pose = (e1 == o) ? p1 : p2;

    // 2. Check Orientation Alignment (should be similar for pushing)
    double yaw_diff = std::abs(r_pose.yaw - o_pose.yaw);
    while (yaw_diff > M_PI)
      yaw_diff -= 2 * M_PI;
    while (yaw_diff < -M_PI)
      yaw_diff += 2 * M_PI;
    if (std::abs(yaw_diff) > 0.5)
      return false; // Angle mismatch > ~30 deg

    // 3. Check Relative Position (Object should be in front)
    // Simple check: Distance should be roughly sum of half-lengths
    double dx = o_pose.x - r_pose.x;
    double dy = o_pose.y - r_pose.y;
    double dist = std::hypot(dx, dy);

    // Expected distance center-to-center approx (Front_R + Rear_O)
    // Allowing some tolerance (e.g., 0.2m)
    double expected_dist = r->size.front_length + o->size.rear_length;
    if (dist > expected_dist + 0.3 || dist < expected_dist - 0.3)
      return false;

    return true;
  };
  // ------------------------------------------------

  // 1. Check Start Pose Validity
  Corners start_c =
      get_corners(start.x, start.y, start.yaw, robot->size.front_length,
                  robot->size.rear_length, robot->size.width);
  bool start_ok = true;
  for (const auto &[ent, pose] : others)
  {
    if (ent == robot)
      continue;

    Corners ent_c =
        get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length,
                    ent->size.rear_length, ent->size.width);
    if (rectangles_intersect(start_c, ent_c))
    {
      std::cerr << "    [FAIL] Start Pose COLLIDES with " << ent->name
                << " (Dist: " << std::hypot(start.x - pose.x, start.y - pose.y)
                << "m)" << std::endl;
      start_ok = false;
    }
  }
  if (start_ok)
    std::cerr << "    [PASS] Start Pose is collision-free." << std::endl;

  // 2. Check Goal Pose Validity
  Corners goal_c =
      get_corners(goal.x, goal.y, goal.yaw, robot->size.front_length,
                  robot->size.rear_length, robot->size.width);
  bool goal_ok = true;
  for (const auto &[ent, pose] : others)
  {
    if (ent == robot)
      continue;

    Corners ent_c =
        get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length,
                    ent->size.rear_length, ent->size.width);
    if (rectangles_intersect(goal_c, ent_c))
    {
      std::cerr << "    [FAIL] Goal Pose COLLIDES with " << ent->name
                << std::endl;
      goal_ok = false;
    }
  }
  if (goal_ok)
    std::cerr << "    [PASS] Goal Pose is collision-free." << std::endl;

  // 3. Check Global Consistency
  std::cerr << "    [Info] Checking other entities for consistency..."
            << std::endl;
  bool global_issue = false;

  for (size_t i = 0; i < others.size(); ++i)
  {
    for (size_t j = i + 1; j < others.size(); ++j)
    {
      auto [ent1, p1] = others[i];
      auto [ent2, p2] = others[j];

      if (ent1 == robot || ent2 == robot)
        continue;

      Corners c1 = get_corners(p1.x, p1.y, p1.yaw, ent1->size.front_length,
                               ent1->size.rear_length, ent1->size.width);
      Corners c2 = get_corners(p2.x, p2.y, p2.yaw, ent2->size.front_length,
                               ent2->size.rear_length, ent2->size.width);

      if (rectangles_intersect(c1, c2))
      {
        // Check if this is a valid transfer (Robot pushing Object)
        if (is_valid_transfer(ent1, p1, ent2, p2))
        {
          // Valid transfer - ignore
          // std::cout << "    [Info] Ignoring contact between " << ent1->name
          // << " and " << ent2->name << " (Transferring)" << std::endl;
        }
        else
        {
          std::cerr << "    [WARN] Global Consistency: " << ent1->name
                    << " intersects " << ent2->name << "!" << std::endl;
          global_issue = true;
        }
      }
    }
  }
  if (!global_issue)
    std::cerr << "    [PASS] Global scene is consistent (ignoring transfers)."
              << std::endl;
}

// ==========================================
// Trajectory and Waypoint Utilities (Block A)
// ==========================================

static const double INF = std::numeric_limits<double>::infinity();

void project_waypoints_inside_bounds(std::vector<Waypoint> &waypoints,
                                     RobotMeta *robot,
                                     const Params &params,
                                     EntityMeta *transferred_object)
{
  if (!robot)
    return;

  for (auto &wp : waypoints)
  {
    if (params.robot_boundary_origin_only)
    {
      double lower_dx = params.min_x - wp.x;
      double upper_dx = params.max_x - wp.x;
      double lower_dy = params.min_y - wp.y;
      double upper_dy = params.max_y - wp.y;

      if (transferred_object)
      {
        const Pose object_pose =
            TimeTable::compute_object_pose(wp, robot->size,
                                           transferred_object->size);
        lower_dx = std::max(lower_dx, params.min_x - object_pose.x);
        upper_dx = std::min(upper_dx, params.max_x - object_pose.x);
        lower_dy = std::max(lower_dy, params.min_y - object_pose.y);
        upper_dy = std::min(upper_dy, params.max_y - object_pose.y);
      }

      if (lower_dx <= upper_dx)
      {
        if (lower_dx > 0.0)
          wp.x += lower_dx;
        else if (upper_dx < 0.0)
          wp.x += upper_dx;
      }
      if (lower_dy <= upper_dy)
      {
        if (lower_dy > 0.0)
          wp.y += lower_dy;
        else if (upper_dy < 0.0)
          wp.y += upper_dy;
      }
      continue;
    }

    Corners corners = get_corners(wp.x, wp.y, wp.yaw,
                                  robot->size.front_length,
                                  robot->size.rear_length,
                                  robot->size.width);

    double min_cx = corners[0].x, max_cx = corners[0].x;
    double min_cy = corners[0].y, max_cy = corners[0].y;
    for (const auto &pt : corners)
    {
      min_cx = std::min(min_cx, pt.x);
      max_cx = std::max(max_cx, pt.x);
      min_cy = std::min(min_cy, pt.y);
      max_cy = std::max(max_cy, pt.y);
    }

    double dx = 0.0;
    if (min_cx < params.min_x)
      dx += (params.min_x - min_cx);
    if (max_cx > params.max_x)
      dx += (params.max_x - max_cx);

    double dy = 0.0;
    if (min_cy < params.min_y)
      dy += (params.min_y - min_cy);
    if (max_cy > params.max_y)
      dy += (params.max_y - max_cy);

    wp.x += dx;
    wp.y += dy;
  }
}

void shift_waypoint_times(std::vector<Waypoint> &waypoints, double delta)
{
  if (std::abs(delta) < 1e-12)
    return;

  for (auto &wp : waypoints)
    wp.time += delta;
}

Params make_relaxed_fallback_params(const Params &params)
{
  Params relaxed = params;
  relaxed.xy_resolution = std::max(params.xy_resolution, 0.3);
  relaxed.yaw_resolution = std::max(params.yaw_resolution, M_PI / 2.0);
  relaxed.time_step = std::max(params.time_step, 3.0);
  relaxed.time_resolution = relaxed.time_step;
  relaxed.collision_steps = std::max(2, params.collision_steps);
  relaxed.rs_step_size = std::max(params.rs_step_size, 0.3);
  return relaxed;
}

Params make_fine_segment_params(const Params &params,
                                const RuntimeOptions &options)
{
  Params fine = params;
  fine.xy_resolution = std::min(params.xy_resolution,
                                std::max(1e-6, options.fine_segment_xy_resolution));
  fine.yaw_resolution = std::min(params.yaw_resolution,
                                 std::max(1e-6, options.fine_segment_yaw_resolution));
  fine.time_step = std::min(params.time_step,
                            std::max(1e-6, options.fine_segment_time_step));
  fine.time_resolution = fine.time_step;
  fine.rs_step_size = std::min(params.rs_step_size,
                               std::max(1e-6, options.fine_segment_rs_step_size));
  fine.collision_check_time_step =
      std::min(params.collision_check_time_step,
               std::max(1e-6, options.fine_segment_collision_check_time_step));
  fine.collision_steps = std::max(
      params.collision_steps,
      static_cast<int>(std::ceil(fine.time_step /
                                 std::max(1e-3, fine.collision_check_time_step))));
  return fine;
}

Params make_contact_boundary_segment_params(const Params &params,
                                            const RuntimeOptions &options)
{
  Params contact = make_fine_segment_params(params, options);
  contact.xy_resolution = std::min(contact.xy_resolution,
                                   std::max(1e-6, options.contact_boundary_xy_resolution));
  contact.yaw_resolution = std::min(contact.yaw_resolution,
                                    std::max(1e-6, options.contact_boundary_yaw_resolution));
  contact.time_step = std::min(contact.time_step,
                               std::max(1e-6, options.contact_boundary_time_step));
  contact.time_resolution = contact.time_step;
  contact.rs_step_size = std::min(contact.rs_step_size,
                                  std::max(1e-6, options.contact_boundary_rs_step_size));
  contact.reverse_penalty =
      std::max(0.0, options.contact_boundary_reverse_penalty);
  contact.spatial_only_index = true;
  contact.disable_wait_primitive = true;
  contact.enable_holonomic_heuristic = true;
  contact.holonomic_heuristic_resolution =
      std::max(1e-3, options.contact_boundary_holonomic_resolution);
  contact.collision_steps = std::max(
      contact.collision_steps,
      static_cast<int>(std::ceil(contact.time_step /
                                 std::max(1e-3, contact.collision_check_time_step))));
  return contact;
}

ReferenceExperienceGraphOptions reference_egraph_options_from_runtime(
    const RuntimeOptions &options)
{
  ReferenceExperienceGraphOptions egraph_options;
  egraph_options.enabled = true;
  egraph_options.epsilon = options.reference_egraph_epsilon;
  egraph_options.waypoint_spacing = options.reference_egraph_waypoint_spacing;
  egraph_options.snap_radius = options.reference_egraph_snap_radius;
  egraph_options.snap_yaw = options.reference_egraph_snap_yaw;
  egraph_options.successor_lookahead =
      options.reference_egraph_successor_lookahead;
  egraph_options.max_nodes = options.reference_egraph_max_nodes;
  return egraph_options;
}

bool transit_step_uses_reference_egraph(const TransitPlannerStep &step)
{
  return step.reference_egraph == ReferenceEGraphUse::EnabledWhenAvailable;
}

bool transit_step_is_fine_repair(const TransitPlannerStep &step)
{
  return step.method == TransitPlannerMethod::FineHybridAStar ||
         step.method == TransitPlannerMethod::ContactBoundaryGeometricHybridAStar ||
         step.method == TransitPlannerMethod::ReverseStraightEscapeFineHybridAStar ||
         step.method == TransitPlannerMethod::ReverseLeftEscapeFineHybridAStar ||
         step.method == TransitPlannerMethod::ReverseRightEscapeFineHybridAStar ||
         step.method == TransitPlannerMethod::AllCandidateReedShepp ||
         step.method == TransitPlannerMethod::ReverseStraightEscapeReedShepp ||
         step.method == TransitPlannerMethod::ReverseLeftEscapeReedShepp ||
         step.method == TransitPlannerMethod::ReverseRightEscapeReedShepp;
}

// ==========================================
// Stage 3: failure triage predicates + holonomic feasibility pre-gate
// ==========================================
//
// DESIGN NOTE -- rule 1 (BLOCKED-BY-ROBOT SHORT-CIRCUIT) current-behavior
// finding, per file:
//
//  * plan_initial_transit's method loop (run_initial_transit_method_list,
//    below) ALREADY stops on a BLOCKED_BY_ROBOT tier result: its only
//    escalate-vs-stop condition is `!result.waypoints.empty()`, and
//    PlanningStatus::BLOCKED_BY_ROBOT is only ever produced by
//    PHAStar::Planning_with_res together with a non-empty extracted backup
//    path (include/PHAstar/PHAstar.h's backup_result is seeded from
//    extract_path(...) before its status is ever set to BLOCKED_BY_ROBOT --
//    see the analytic-candidate handlers around PHAstar.h:2100-2168 -- and
//    both places Planning_with_res can return it, :1968 and :2515, return
//    that same backup_result object unchanged). So this rule is a no-op here
//    by construction; nothing is implemented for it in this function.
//
//  * replan_transit_segment's cascade (below) has NO status-based check at
//    all: a tier is "accepted" (cascade stops) purely via
//    evaluate_candidate()'s independent re-validation
//    (validate_segment_candidate), which happens to also treat a pure robot
//    collision as soft/non-blocking -- but if that independent re-check
//    finds hard_valid == false for ANY reason (including simply disagreeing
//    with the search's own analytic collision check, since the two use
//    different discretizations), the loop escalates to the next tier
//    regardless of the fact that the underlying failure was "just a robot in
//    the way". This IS a real escalate-past-BLOCKED_BY_ROBOT path, so rule 1
//    is implemented there (see replan_transit_segment).

// Rule 1 predicate (see design note above): does `result` carry a
// robot-blocked backup path at all (regardless of what any later,
// independent revalidation of its waypoints might conclude)?
bool triage_blocked_by_robot_shortcut_applies(const PlanningResult &result)
{
  return result.status == PlanningStatus::BLOCKED_BY_ROBOT &&
         !result.waypoints.empty();
}

bool triage_primary_retry_applies(const PlanningResult &result,
                                  double straight_line_distance)
{
  if (result.debug_stats.search_iteration_limit_hit == 0)
    return false;
  return result.debug_stats.best_dist <=
         kTriageProgressFraction * straight_line_distance;
}

bool triage_skip_to_contact_applies(const PlanningResult &result)
{
  const PlanningDebugStats &stats = result.debug_stats;
  // A zero-generated-node result (e.g. an immediate start/goal validation
  // failure) isn't "collision-rejection dominated" in any meaningful sense;
  // guard the ratio's denominator rather than let 0 >= 0 misfire.
  if (stats.generated_nodes == 0)
    return false;
  return stats.best_dist <= stats.analytic_threshold &&
         static_cast<double>(stats.reject_collision) >=
             kTriageCollisionRejectFraction *
                 static_cast<double>(stats.generated_nodes);
}

bool method_list_has_fine_before_contact(
    const std::vector<TransitPlannerStep> &methods)
{
  bool seen_fine = false;
  for (const auto &step : methods)
  {
    if (step.method == TransitPlannerMethod::FineHybridAStar)
      seen_fine = true;
    else if (seen_fine &&
             step.method ==
                 TransitPlannerMethod::ContactBoundaryGeometricHybridAStar)
      return true;
  }
  return false;
}

double entity_inscribed_radius(const OccuRect &size)
{
  return std::max(0.0, std::min({size.front_length, size.rear_length,
                                 size.width / 2.0}));
}

namespace
{
// Exact circle-vs-oriented-rectangle overlap test: transforms the circle
// center into the box's local frame (get_corners's convention: the box
// spans [-rear_length, front_length] along local +x, [-width/2, width/2]
// across local y) and compares the distance to the closest point on the box
// against the radius. Used (instead of rectangles_intersect, which needs
// both shapes to already be rectangles) so the gate's under-approximated
// robot disc is checked exactly, not via a bounding-square stand-in that
// would over-block relative to the true circle and risk unsound skips.
bool circle_intersects_oriented_box(double cx, double cy, double radius,
                                    const Pose &box_pose, const OccuRect &size)
{
  const double dx = cx - box_pose.x;
  const double dy = cy - box_pose.y;
  const double cos_yaw = std::cos(box_pose.yaw);
  const double sin_yaw = std::sin(box_pose.yaw);
  const double local_x = dx * cos_yaw + dy * sin_yaw;
  const double local_y = -dx * sin_yaw + dy * cos_yaw;
  const double half_width = size.width / 2.0;
  const double closest_x =
      std::clamp(local_x, -size.rear_length, size.front_length);
  const double closest_y = std::clamp(local_y, -half_width, half_width);
  return std::hypot(local_x - closest_x, local_y - closest_y) <= radius;
}
} // namespace

bool holonomic_gate_unreachable(
    const Pose &start_pose, const Pose &goal_pose,
    RobotMeta *robot, const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &gate_params, double search_start_time)
{
  if (!robot)
    return false; // Can't build a footprint; gate passes (never skips).

  const double resolution =
      std::max(1e-3, gate_params.holonomic_heuristic_resolution);
  const int width = static_cast<int>(std::ceil(
                        (gate_params.max_x - gate_params.min_x) / resolution)) +
                    1;
  const int height = static_cast<int>(std::ceil(
                         (gate_params.max_y - gate_params.min_y) / resolution)) +
                     1;
  if (width <= 0 || height <= 0)
    return false; // Degenerate bounds: gate passes, never skips.
  const std::size_t total =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (total == 0)
    return false;

  const double radius = entity_inscribed_radius(robot->size);

  // Static-only obstacle snapshot at the search's start time: the workspace
  // boundary is handled directly (via check_robot_bounds_collision) inside
  // cell_blocked below, so only OBJECT entities with no scheduled motion
  // after search_start_time are collected here. Robots are NEVER gate
  // obstacles -- they can always move out of the way (wait primitives
  // exist).
  struct StaticObstacle
  {
    Pose pose;
    OccuRect size;
  };
  std::vector<StaticObstacle> obstacles;
  obstacles.reserve(entities.size());
  for (const auto &[name, ent] : entities)
  {
    if (!ent || ent == robot || ent->type != EntityType::OBJECT)
      continue;
    if (!timetable.is_entity_static_after(search_start_time, ent))
      continue;
    obstacles.push_back({timetable.get_pose(ent, search_start_time), ent->size});
  }

  auto clamp_x = [&](double x)
  {
    return std::max(0, std::min(width - 1,
                                static_cast<int>(std::floor(
                                    (x - gate_params.min_x) / resolution))));
  };
  auto clamp_y = [&](double y)
  {
    return std::max(0, std::min(height - 1,
                                static_cast<int>(std::floor(
                                    (y - gate_params.min_y) / resolution))));
  };
  auto cell_index = [&](int ix, int iy)
  { return iy * width + ix; };
  auto cell_center = [&](int ix, int iy)
  {
    return Pose{gate_params.min_x + (static_cast<double>(ix) + 0.5) * resolution,
               gate_params.min_y + (static_cast<double>(iy) + 0.5) * resolution,
               0.0};
  };

  const int gx = clamp_x(goal_pose.x);
  const int gy = clamp_y(goal_pose.y);
  const int sx = clamp_x(start_pose.x);
  const int sy = clamp_y(start_pose.y);
  const int goal_idx = cell_index(gx, gy);
  const int start_idx = cell_index(sx, sy);

  auto cell_blocked = [&](int ix, int iy) -> bool
  {
    const Pose center = cell_center(ix, iy);
    // Boundary check: reuse check_robot_bounds_collision (the exact rule the
    // real planner applies, including the robot_boundary_origin_only
    // origin-only/strict-corners switch) via a synthetic axis-aligned square
    // bounding the under-approximated disc -- get_corners with
    // front=rear=radius, width=2*radius at yaw 0 gives exactly that square.
    const Corners disc_bound =
        get_corners(center.x, center.y, 0.0, radius, radius, 2.0 * radius);
    if (check_robot_bounds_collision(center, disc_bound, gate_params))
      return true;
    for (const auto &obstacle : obstacles)
    {
      if (circle_intersects_oriented_box(center.x, center.y, radius,
                                         obstacle.pose, obstacle.size))
        return true;
    }
    return false;
  };

  std::vector<double> cost(total, std::numeric_limits<double>::infinity());
  using CostCell = std::pair<double, int>;
  std::priority_queue<CostCell, std::vector<CostCell>, std::greater<CostCell>>
      open;
  cost[static_cast<std::size_t>(goal_idx)] = 0.0;
  open.emplace(0.0, goal_idx);

  constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!open.empty())
  {
    const auto [current_cost, idx] = open.top();
    open.pop();
    if (current_cost > cost[static_cast<std::size_t>(idx)] + 1e-9)
      continue;
    if (idx == start_idx)
      return false; // Reachable (Dijkstra: first non-stale pop is optimal).

    const int ix = idx % width;
    const int iy = idx / width;
    for (int k = 0; k < 8; ++k)
    {
      const int nx = ix + kDx[k];
      const int ny = iy + kDy[k];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height)
        continue;
      const int nidx = cell_index(nx, ny);
      // The goal and start cells are always treated as passable -- mirrors
      // ensure_holonomic_heuristic's own goal-cell exemption (PHAstar.h),
      // extended here to the start cell too, so grid quantization can never
      // manufacture a false "blocked" verdict at either query pose.
      if (nidx != goal_idx && nidx != start_idx && cell_blocked(nx, ny))
        continue;
      const double step = ((k < 4) ? 1.0 : std::sqrt(2.0)) * resolution;
      const double next_cost = current_cost + step;
      if (next_cost + 1e-9 < cost[static_cast<std::size_t>(nidx)])
      {
        cost[static_cast<std::size_t>(nidx)] = next_cost;
        open.emplace(next_cost, nidx);
      }
    }
  }

  return !std::isfinite(cost[static_cast<std::size_t>(start_idx)]);
}

bool apply_tier_gate(
    PlanningResult &out_result, PlanSearchTier tier,
    const Pose &start_pose, const Pose &goal_pose,
    RobotMeta *robot, const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &tier_params, double search_start_time,
    PlanTimingStats *plan_stats)
{
  const auto t0 = std::chrono::steady_clock::now();
  const bool unreachable = holonomic_gate_unreachable(
      start_pose, goal_pose, robot, timetable, entities, tier_params,
      search_start_time);
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  if (plan_stats)
  {
    ++plan_stats->n_gate_checks;
    plan_stats->gate_wall_s += elapsed;
  }
  if (!unreachable)
    return false;

  out_result = PlanningResult{};
  out_result.status = PlanningStatus::NO_PATH_FOUND;
  out_result.failure_detail =
      std::string("tier-gate: goal provably unreachable at ") +
      (tier == PlanSearchTier::Contact ? "contact" : "fine") +
      " holonomic resolution (static obstacles, inscribed-radius footprint)";
  if (plan_stats)
  {
    if (tier == PlanSearchTier::Contact)
      ++plan_stats->n_gate_skips_contact;
    else
      ++plan_stats->n_gate_skips_fine;
  }
  return true;
}

std::string format_planning_status_line(const std::string &stage,
                                        const PlanningResult &res)
{
  std::ostringstream oss;
  oss << stage << " => " << planning_status_name(res.status);
  if (!res.colliding_entity.empty())
    oss << ", blocker=" << res.colliding_entity;
  if (res.failure_time > 1e-6)
    oss << ", t=" << std::fixed << std::setprecision(2) << res.failure_time;
  if (!res.failure_detail.empty())
    oss << ", detail=" << res.failure_detail;
  if (!res.waypoints.empty())
    oss << ", waypoints=" << res.waypoints.size();
  return oss.str();
}

std::vector<Waypoint> waypoints_from_relopush_state_path(
    const ReloPush::StatePathPtr &path)
{
  std::vector<Waypoint> waypoints;
  if (!path)
    return waypoints;

  waypoints.reserve(path->size());
  for (const auto &state : *path)
    waypoints.push_back(WaypointFromReloPushState(state));
  return waypoints;
}

void accumulate_wait_stats(TaskExecutionStats *stats, double wait_added)
{
  if (stats && wait_added > 1e-9)
  {
    stats->total_waiting += wait_added;
    stats->delayed_segments += 1;
  }
}
bool plan_initial_transit(
    RobotMeta *robot, const Pose &target_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    double *out_abs_start_time,
    double *out_abs_end_time,
    const std::function<Pose(double)> &target_pose_provider,
    const std::vector<Waypoint> *reference_waypoints,
    PlanTimingStats *plan_stats)
{
  if (out_abs_start_time)
    *out_abs_start_time = -1.0;
  if (out_abs_end_time)
    *out_abs_end_time = -1.0;

  double planning_start_time = start_time;
  double chosen_start_time = planning_start_time;
  Pose active_target_pose = target_pose;
  Pose current_pose = timetable.get_pose(robot, planning_start_time);
  robot->initial_pose = current_pose; // Update meta for planner

  std::cout << "  [Transit] Planning " << robot->name << " -> Target ("
            << active_target_pose.x << ", " << active_target_pose.y << ", " << active_target_pose.yaw
            << ") from Start (" << current_pose.x << ", " << current_pose.y << ", " << current_pose.yaw
            << ") at " << planning_start_time << "s" << std::endl;

  std::vector<std::string> attempt_log;
  std::vector<PlanningDebugAttempt> debug_attempts;
  auto append_attempt = [&](const std::string &stage, const PlanningResult &res)
  {
    attempt_log.push_back(format_planning_status_line(stage, res));
    if (options.print_planning_status)
    {
      std::cout << "  [TransitStatus] " << attempt_log.size() << ") "
                << attempt_log.back() << std::endl;
    }
    debug_attempts.push_back({stage, res});
  };

  auto build_attempt_trace = [&]() -> std::string
  {
    if (attempt_log.empty())
      return "";

    std::ostringstream oss;
    oss << "Planning attempts:";
    for (std::size_t idx = 0; idx < attempt_log.size(); ++idx)
    {
      oss << "\n"
          << (idx + 1) << ") " << attempt_log[idx];
    }
    return oss.str();
  };

  auto refresh_target_pose = [&](double query_time,
                                 const std::string &reason_label) -> bool
  {
    if (!target_pose_provider)
      return false;

    const Pose refreshed = target_pose_provider(query_time);
    const double position_delta =
        std::hypot(refreshed.x - active_target_pose.x,
                   refreshed.y - active_target_pose.y);
    const double yaw_delta =
        std::abs(pi_2_pi(refreshed.yaw - active_target_pose.yaw));
    if (position_delta < 1e-3 && yaw_delta < 1e-3)
      return false;

    std::cout << "  [Transit] Refreshing initial transit target after "
              << reason_label << ": ("
              << std::fixed << std::setprecision(2)
              << active_target_pose.x << ", " << active_target_pose.y
              << ", " << active_target_pose.yaw << ") -> ("
              << refreshed.x << ", " << refreshed.y << ", "
              << refreshed.yaw << ") at t=" << query_time << "s."
              << std::endl;
    active_target_pose = refreshed;
    return true;
  };

  auto run_initial_transit_planner = [&](const std::string &debug_label,
                                         const Params &plan_params,
                                         int max_iterations,
                                         bool allow_reference_egraph,
                                         PlanSearchTier tier) -> PlanningResult
  {
    current_pose = timetable.get_pose(robot, planning_start_time);
    robot->initial_pose = current_pose;
    PHAStar retry_planner(robot, active_target_pose, &timetable, &entities,
                          plan_params, false, "", planning_start_time, debug_label);
    retry_planner.set_debug_popup_enabled(false);
    retry_planner.max_search_iterations = max_iterations;
    retry_planner.set_planner_expansion_threads(options.planner_expansion_threads);
    if (allow_reference_egraph && reference_waypoints &&
        reference_waypoints->size() >= 2)
    {
      retry_planner.set_reference_experience_graph(
          *reference_waypoints, reference_egraph_options_from_runtime(options));
      if (retry_planner.has_reference_experience_graph())
      {
        std::cout << "  [Transit] Reference E-Graph guide active for "
                  << debug_label << " (input waypoints="
                  << reference_waypoints->size() << ")." << std::endl;
      }
    }
    const auto search_t0 = std::chrono::steady_clock::now();
    PlanningResult res = retry_planner.Planning_with_res(planning_start_time);
    if (plan_stats)
    {
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - search_t0)
                                 .count();
      plan_stats->record_search(
          tier, elapsed, res.debug_stats.iterations,
          res.debug_stats.search_iteration_limit_hit,
          res.debug_stats.heuristic_time_sec,
          res.debug_stats.primitive_collision_time_sec,
          res.debug_stats.analytic_validation_time_sec,
          res.debug_stats.holonomic_heuristic_time_sec);
    }
    return res;
  };

  // Stage 3 holonomic feasibility pre-gate (RuntimeOptions::enable_tier_gate):
  // checked immediately before launching Fine/Contact below. Uses the
  // robot's live pose at planning_start_time (not the possibly-stale outer
  // `current_pose`, since this can run before run_initial_transit_planner's
  // own refresh) as the gate's start pose.
  auto apply_initial_transit_tier_gate =
      [&](PlanningResult &out_gated, PlanSearchTier tier,
          const Params &tier_params) -> bool
  {
    const Pose gate_start_pose = timetable.get_pose(robot, planning_start_time);
    return apply_tier_gate(out_gated, tier, gate_start_pose, active_target_pose,
                           robot, timetable, entities, tier_params,
                           planning_start_time, plan_stats);
  };

  auto replan_initial_transit_step =
      [&](const TransitPlannerStep &step,
          const std::string &debug_label) -> PlanningResult
  {
    const bool use_reference_egraph = transit_step_uses_reference_egraph(step);
    switch (step.method)
    {
    case TransitPlannerMethod::PrimaryHybridAStar:
      return run_initial_transit_planner(
          debug_label, params, options.max_search_iterations,
          use_reference_egraph, PlanSearchTier::Primary);
    case TransitPlannerMethod::FineHybridAStar:
    {
      Params fine_params = make_fine_segment_params(params, options);
      const int fine_max_iter = std::max(
          options.max_search_iterations,
          options.fine_segment_max_search_iterations);
      if (options.enable_tier_gate)
      {
        PlanningResult gated;
        if (apply_initial_transit_tier_gate(gated, PlanSearchTier::Fine,
                                            fine_params))
          return gated;
      }
      return run_initial_transit_planner(
          debug_label, fine_params, fine_max_iter, use_reference_egraph,
          PlanSearchTier::Fine);
    }
    case TransitPlannerMethod::ContactBoundaryGeometricHybridAStar:
    {
      Params contact_params = make_contact_boundary_segment_params(params, options);
      const int contact_max_iter = std::max(
          options.max_search_iterations,
          options.contact_boundary_max_search_iterations);
      if (options.enable_tier_gate)
      {
        PlanningResult gated;
        if (apply_initial_transit_tier_gate(gated, PlanSearchTier::Contact,
                                            contact_params))
          return gated;
      }
      return run_initial_transit_planner(
          debug_label, contact_params, contact_max_iter, use_reference_egraph,
          PlanSearchTier::Contact);
    }
    default:
    {
      PlanningResult unsupported;
      unsupported.status = PlanningStatus::NO_PATH_FOUND;
      unsupported.failure_detail = "unsupported initial-transit method in ordered list";
      return unsupported;
    }
    }
  };

  // Stage 3 failure triage (RuntimeOptions::enable_tier_triage). Rule 1
  // (BLOCKED-BY-ROBOT SHORT-CIRCUIT) is intentionally NOT implemented here:
  // this loop already stops on any BLOCKED_BY_ROBOT tier result today (see
  // the design note above triage_blocked_by_robot_shortcut_applies), so
  // there is nothing to short-circuit. Rules 2 and 3 are wired below.
  const bool triage_fine_then_contact_initial =
      method_list_has_fine_before_contact(options.initial_transit_methods);

  bool tried_fine_initial_transit = false;
  auto run_initial_transit_method_list =
      [&](const std::string &debug_prefix,
          bool fine_only = false) -> PlanningResult
  {
    PlanningResult result;
    result.status = PlanningStatus::NO_PATH_FOUND;
    result.failure_detail = "no initial-transit methods configured";

    // Mirrors this loop's own fine_only filtering so "the next tier" means
    // the next one this loop will actually attempt.
    auto next_step_is_fine = [&](std::size_t from_idx) -> bool
    {
      for (std::size_t j = from_idx + 1;
           j < options.initial_transit_methods.size(); ++j)
      {
        const auto &candidate = options.initial_transit_methods[j];
        if (fine_only && !transit_step_is_fine_repair(candidate))
          continue;
        return candidate.method == TransitPlannerMethod::FineHybridAStar;
      }
      return false;
    };

    bool skip_next_fine = false;
    for (std::size_t idx = 0; idx < options.initial_transit_methods.size(); ++idx)
    {
      const auto &step = options.initial_transit_methods[idx];
      if (fine_only && !transit_step_is_fine_repair(step))
        continue;

      if (options.enable_tier_triage && skip_next_fine &&
          step.method == TransitPlannerMethod::FineHybridAStar)
      {
        skip_next_fine = false;
        if (plan_stats)
          ++plan_stats->n_triage_skips_to_contact;
        continue;
      }
      skip_next_fine = false;

      if (transit_step_is_fine_repair(step))
        tried_fine_initial_transit = true;

      const std::string stage = debug_prefix + " " +
                                transit_planner_method_name(step.method);
      result = replan_initial_transit_step(step, stage);
      append_attempt(transit_planner_method_name(step.method), result);
      if (!result.waypoints.empty())
        return result;

      if (options.enable_tier_triage)
      {
        // Rule 2 (CAP-HIT-WITH-PROGRESS RETRY, primary tier only).
        if (step.method == TransitPlannerMethod::PrimaryHybridAStar)
        {
          const double straight_line_dist = std::hypot(
              active_target_pose.x - current_pose.x,
              active_target_pose.y - current_pose.y);
          if (triage_primary_retry_applies(result, straight_line_dist))
          {
            if (plan_stats)
              ++plan_stats->n_triage_primary_retries;
            const bool use_reference_egraph =
                transit_step_uses_reference_egraph(step);
            PlanningResult retry_result = run_initial_transit_planner(
                stage + " (triage retry)", params,
                2 * options.max_search_iterations, use_reference_egraph,
                PlanSearchTier::Primary);
            append_attempt(transit_planner_method_name(step.method) +
                              std::string(" (triage retry)"),
                          retry_result);
            // The retry's result replaces the original failure for any
            // subsequent triage rule (never retried twice; if this retry
            // also carries a BLOCKED_BY_ROBOT backup, the unconditional
            // !result.waypoints.empty() check below already implements
            // rule 1's "stop and use the backup" for it too).
            result = retry_result;
            if (!result.waypoints.empty())
            {
              if (plan_stats)
                ++plan_stats->n_triage_retry_successes;
              return result;
            }
          }
        }

        // Rule 3 (NEAR-GOAL-COLLISION SKIP-TO-CONTACT).
        if (triage_fine_then_contact_initial && next_step_is_fine(idx) &&
            triage_skip_to_contact_applies(result))
        {
          skip_next_fine = true;
        }
      }
    }
    return result;
  };

  std::function<bool(PlanningResult &, const std::string &, bool)>
      try_schedule_time_aware_path =
          [&](PlanningResult &candidate_res,
              const std::string &stage_label,
              bool allow_target_refresh) -> bool
  {
    if (candidate_res.waypoints.empty())
    {
      return false;
    }

    Trajectory candidate_traj;
    candidate_traj.entity = robot;
    candidate_traj.is_transfer = false;
    candidate_traj.waypoints = candidate_res.waypoints;

    const double candidate_start_time = candidate_traj.waypoints.front().time;
    candidate_traj.start_time = candidate_start_time;
    make_waypoint_times_relative(candidate_traj.waypoints, candidate_start_time);

    double wait_added = 0.0;
    CollisionInfo last_collision;
    double last_check_time = candidate_start_time;
    double safe_start = find_safe_start_time(&candidate_traj, candidate_start_time,
                                             timetable, params, entities,
                                             options,
                                             &wait_added, &last_collision,
                                             &last_check_time,
                                             IdleBlockerRelocationPolicy::RelocateAnyIdle,
                                             plan_stats);
    if (safe_start < 0.0)
    {
      PlanningResult sched_fail;
      sched_fail.status = PlanningStatus::NO_PATH_FOUND;
      sched_fail.colliding_entity = last_collision.entity_name;
      sched_fail.failure_time = last_check_time;
      std::ostringstream oss;
      oss << "time-aware scheduling failed";
      if (!last_collision.reason.empty())
      {
        oss << " (" << last_collision.reason;
        if (!last_collision.entity_name.empty())
        {
          oss << " with " << last_collision.entity_name;
        }
        oss << ")";
      }
      sched_fail.failure_detail = oss.str();
      append_attempt(stage_label, sched_fail);
      return false;
    }

    const double delta = safe_start - candidate_start_time;

    const double candidate_duration =
        candidate_res.waypoints.back().time - candidate_start_time;
    const double refreshed_target_query_time =
        safe_start + std::max(0.0, candidate_duration);
    if (allow_target_refresh && delta > 1e-6 &&
        refresh_target_pose(refreshed_target_query_time,
                            "delay recovery predicted arrival"))
    {
      planning_start_time = safe_start;
      chosen_start_time = safe_start;

      PlanningResult refreshed_res =
          run_initial_transit_method_list(
              "Initial transit after delay target refresh");

      if (!refreshed_res.waypoints.empty() &&
          try_schedule_time_aware_path(
              refreshed_res,
              "time-aware schedule of refreshed delayed path",
              false))
      {
        candidate_res = refreshed_res;
        return true;
      }

      candidate_res = refreshed_res;
      return false;
    }

    shift_waypoint_times(candidate_res.waypoints, delta);
    chosen_start_time = safe_start;
    candidate_res.status = PlanningStatus::SUCCESS;
    candidate_res.failure_detail.clear();
    candidate_res.colliding_entity.clear();
    candidate_res.failure_time = 0.0;
    append_attempt(stage_label, candidate_res);

    if (wait_added > 1e-9)
    {
      std::cout << "  [Transit] Dynamic blocker handled by waiting; scheduling path at "
                << std::fixed << std::setprecision(2) << safe_start
                << "s (wait " << wait_added << "s)." << std::endl;
    }
    else
    {
      std::cout << "  [Transit] Using time-aware scheduled path at "
                << std::fixed << std::setprecision(2) << safe_start
                << "s." << std::endl;
    }
    return true;
  };

  auto make_blocked_pose_hint = [&](const Pose &pose) -> Trajectory
  {
    Trajectory hint;
    hint.entity = robot;
    hint.start_time = planning_start_time;
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

  auto attempt_self_safe_parking = [&](PlanningResult &res,
                                       const Trajectory &blocked_hint,
                                       const std::string &reason_label,
                                       const std::string &retry_stage) -> bool
  {
    std::cout << "  [Transit] Preserving earlier reserved occupancy while "
              << robot->name << " adapts. " << reason_label
              << " Trying self safe parking before replanning initial transit."
              << std::endl;

    if (!relocate_blocking_robot(robot, timetable, params, entities,
                                 options, &blocked_hint, planning_start_time,
                                 "self-park", plan_stats))
    {
      std::cerr << "  [Transit] Self safe parking FAILED for " << robot->name
                << "." << std::endl;
      return false;
    }

    planning_start_time = timetable.get_entity_max_time(robot);
    chosen_start_time = planning_start_time;
    current_pose = timetable.get_pose(robot, planning_start_time);
    robot->initial_pose = current_pose;
    refresh_target_pose(planning_start_time, "self safe parking");

    std::cout << "  [Transit] " << robot->name
              << " parked safely. Replanning initial transit from ("
              << std::fixed << std::setprecision(2)
              << current_pose.x << ", " << current_pose.y << ", "
              << current_pose.yaw << ") at t=" << planning_start_time
              << "s." << std::endl;

    res = run_initial_transit_method_list(
        "Initial transit after self safe parking");
    if (!res.waypoints.empty())
      append_attempt(retry_stage, res);
    return true;
  };

  auto attempt_validation_blocker_recovery = [&](PlanningResult &res,
                                                 const std::string &retry_stage) -> bool
  {
    if ((res.status != PlanningStatus::GOAL_INVALID_COLLISION &&
         res.status != PlanningStatus::START_INVALID_COLLISION) ||
        res.colliding_entity.empty())
    {
      return false;
    }

    const bool is_start_blocker =
        (res.status == PlanningStatus::START_INVALID_COLLISION);
    const Pose &blocked_pose = is_start_blocker ? current_pose : target_pose;
    const char *blocked_label = is_start_blocker ? "Start" : "Goal";
    const double blocker_time =
        (res.failure_time > 1e-6) ? res.failure_time : planning_start_time;

    CollisionInfo blocker_info;
    blocker_info.is_valid = false;
    blocker_info.reason =
        is_start_blocker ? "Start validation collision" : "Goal validation collision";
    blocker_info.entity_name = res.colliding_entity;
    blocker_info.time = blocker_time;

    RobotMeta *blocker = find_resolvable_robot_blocker(
        blocker_info, timetable, entities);
    if (!blocker)
    {
      return false;
    }

    const bool blocker_waiting = timetable.is_waiting(blocker, blocker_time);
    const bool blocker_static_after =
        timetable.is_entity_static_after(blocker_time, blocker);
    if (!blocker_waiting && !blocker_static_after)
    {
      std::cout << "  [Transit] " << blocked_label
                << " blocked by moving robot " << blocker->name
                << ". Relocation deferred; waiting-based recovery must handle it."
                << std::endl;
      return false;
    }

    Trajectory blocked_pose_hint = make_blocked_pose_hint(blocked_pose);
    // For a goal block, planning_start_time (the candidate's departure time)
    // understates when blocked_pose actually needs to be clear; the real
    // window is when the candidate arrives/collides there, i.e. blocker_time.
    // Override so downstream clearance checks validate the correct window.
    blocked_pose_hint.start_time = blocker_time;
    std::cout << "  [Transit] " << blocked_label << " blocked by idle robot "
              << blocker->name;
    if (res.colliding_entity != blocker->name)
    {
      std::cout << " (reported as " << res.colliding_entity << ")";
    }
    std::cout << ". Keeping earlier reservation priority." << std::endl;

    if (!is_start_blocker)
    {
      // The blocker, not the candidate, is sitting on the target/staging
      // pose: try moving the actual blocker out of the way first, so the
      // candidate can still reach its intended target. Self safe parking
      // (below) would otherwise replan to the SAME pose the blocker still
      // occupies.
      std::cout << "  [Transit] Attempting to relocate blocking robot "
                << blocker->name << " off the " << blocked_label
                << " pose before falling back to self safe parking."
                << std::endl;
      if (relocate_blocking_robot(blocker, timetable, params, entities,
                                  options, &blocked_pose_hint, blocker_time,
                                  "blocker", plan_stats))
      {
        std::cout << "  [Transit] " << blocker->name << " relocated off the "
                  << blocked_label << " pose. Replanning " << robot->name
                  << " initial transit to (" << std::fixed
                  << std::setprecision(2) << active_target_pose.x << ", "
                  << active_target_pose.y << ", " << active_target_pose.yaw
                  << ")." << std::endl;
        res = run_initial_transit_method_list(
            "Initial transit after blocker relocation");
        if (!res.waypoints.empty())
          append_attempt(retry_stage, res);
        return true;
      }
      std::cout << "  [Transit] Could not relocate " << blocker->name
                << "; falling back to self safe parking." << std::endl;
    }

    return attempt_self_safe_parking(
        res, blocked_pose_hint,
        std::string(blocked_label) + " is occupied by higher-priority idle robot " +
            blocker->name + ".",
        retry_stage);
  };

  auto attempt_delayed_replan_after_blocked_backup =
      [&](PlanningResult &res,
          const std::string &reason_label) -> bool
  {
    const double original_start_time = planning_start_time;
    const double blocked_time =
        (res.failure_time > original_start_time + 1e-6)
            ? res.failure_time
            : original_start_time;

    std::cout << "  [Transit] " << reason_label
              << " Trying delayed replanning before self safe parking."
              << std::endl;

    constexpr int kMaxDelayedReplans = 2;
    constexpr double kDelayBuffer = 0.5;
    for (int delayed_attempt = 0; delayed_attempt < kMaxDelayedReplans;
         ++delayed_attempt)
    {
      planning_start_time =
          blocked_time + kDelayBuffer * static_cast<double>(delayed_attempt + 1);
      chosen_start_time = planning_start_time;
      refresh_target_pose(planning_start_time,
                          "blocked backup delayed replan");

      current_pose = timetable.get_pose(robot, planning_start_time);
      robot->initial_pose = current_pose;
      std::cout << "  [Transit] Delayed replan attempt "
                << (delayed_attempt + 1) << "/" << kMaxDelayedReplans
                << " for " << robot->name << " at t="
                << std::fixed << std::setprecision(2)
                << planning_start_time << "s from ("
                << current_pose.x << ", " << current_pose.y << ", "
                << current_pose.yaw << ")." << std::endl;

      PlanningResult delayed_res = run_initial_transit_method_list(
          "Initial transit delayed replan after blocked backup");

      if (delayed_res.waypoints.empty())
      {
        delayed_res = run_initial_transit_method_list(
            "Initial transit delayed replan after blocked backup", true);
        if (delayed_res.waypoints.empty())
          continue;
      }

      if (try_schedule_time_aware_path(
              delayed_res,
              "time-aware schedule of delayed blocked-backup replan",
              true))
      {
        res = delayed_res;
        std::cout << "  [Transit] Delayed replanning resolved the blocked "
                     "initial transit while preserving earlier reservations."
                  << std::endl;
        return true;
      }
    }

    planning_start_time = original_start_time;
    chosen_start_time = original_start_time;
    current_pose = timetable.get_pose(robot, planning_start_time);
    robot->initial_pose = current_pose;
    return false;
  };

  auto path_res = run_initial_transit_method_list("Initial transit");

  for (int relocation_attempt = 0; relocation_attempt < 3; ++relocation_attempt)
  {
    if (!attempt_validation_blocker_recovery(
            path_res,
            "retry after self safe parking"))
    {
      break;
    }
  }

  // If blocked by robot, we already have a "ghost" path in path_res.waypoints
  if (path_res.status == PlanningStatus::BLOCKED_BY_ROBOT)
  {
    std::cout << "  [Transit] Path blocked by robot " << path_res.colliding_entity
              << ". Evaluating delay-only recovery before any self safe parking..."
              << std::endl;

    if (try_schedule_time_aware_path(path_res,
                                     "time-aware schedule of blocked path",
                                     true))
    {
      // Waiting already resolved the moving blocker, or the scheduler handled an idle blocker.
    }
    else
    {
      std::cout << "  [Transit] Time-aware scheduling could not clear the blocked path. "
                   "Checking whether the later initial transit must adapt..."
                << std::endl;

      if (attempt_delayed_replan_after_blocked_backup(
              path_res,
              "The blocked backup path could not be scheduled by waiting alone."))
      {
        // Fresh planning at a later start found a schedulable path.
      }
      else
      {
        Trajectory ghost_traj_abs;
        ghost_traj_abs.waypoints = path_res.waypoints;
        ghost_traj_abs.entity = robot;

        if (ghost_traj_abs.waypoints.empty())
        {
          std::cerr << " [Error] Blocked path has no waypoints! Cannot relocate." << std::endl;
          path_res.status = PlanningStatus::NO_PATH_FOUND;
        }
        else
        {
          Trajectory ghost_traj = ghost_traj_abs;
          double initial_t = ghost_traj.waypoints.front().time;
          ghost_traj.start_time = initial_t;
          ghost_traj_abs.start_time = initial_t;
          for (auto &wp : ghost_traj.waypoints)
            wp.time -= initial_t;

          CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);

          if (!col_info.is_valid && !col_info.entity_name.empty())
          {
            EntityMeta *collider = nullptr;
            auto collider_it = entities.find(col_info.entity_name);
            if (collider_it != entities.end())
              collider = collider_it->second;

            bool should_self_park = false;
            std::string blocker_label = col_info.entity_name;

            if (auto *blocking_robot = find_resolvable_robot_blocker(
                    col_info, timetable, entities))
            {
              should_self_park =
                  timetable.is_waiting(blocking_robot, col_info.time) ||
                  timetable.is_entity_static_after(col_info.time, blocking_robot);
              blocker_label = blocking_robot->name;
            }
            else if (collider && collider->type == EntityType::OBJECT)
            {
              should_self_park = true;
            }

            if (should_self_park &&
                attempt_self_safe_parking(
                    path_res, ghost_traj_abs,
                    "Higher-priority reserved occupancy from " + blocker_label +
                        " remains on the initial-transit corridor after delay-only scheduling.",
                    "retry after self safe parking from blocked path"))
            {
              std::cout << "  [Transit] Self safe parking recovery successful. Retrying plan..."
                        << std::endl;
            }
            else
            {
              std::cerr << "  [Transit] Preserved earlier reservation, but no self safe parking recovery was available."
                        << std::endl;
              path_res.waypoints.clear();
              path_res.status = PlanningStatus::NO_PATH_FOUND;
              path_res.failure_detail =
                  "delay-only scheduling failed and self safe parking recovery failed";
              append_attempt("self safe parking recovery", path_res);
            }
          }
        }
      }
    }
  }

  const auto has_initial_method = [&](TransitPlannerMethod method)
  {
    return std::any_of(options.initial_transit_methods.begin(),
                       options.initial_transit_methods.end(),
                       [&](const TransitPlannerStep &step)
                       { return step.method == method; });
  };
  const bool has_expensive_initial_fallback =
      has_initial_method(TransitPlannerMethod::GhostHybridAStar) ||
      has_initial_method(TransitPlannerMethod::GeometryFallbackHybridAStar) ||
      has_initial_method(TransitPlannerMethod::ReedSheppFallback);

  if (path_res.waypoints.empty() && !has_expensive_initial_fallback)
  {
    PlanningResult disabled_res;
    disabled_res.status = PlanningStatus::NO_PATH_FOUND;
    disabled_res.failure_detail = "initial-transit method list exhausted";
    append_attempt("initial-transit method list exhausted", disabled_res);
  }

  // Fallback for cases where standard planning finds nothing (Search exhausted)
  if (path_res.waypoints.empty() &&
      has_initial_method(TransitPlannerMethod::GhostHybridAStar))
  {
    std::cerr << " [Transit] Standard planning failed. Attempting to resolve blocking robots with full Ghost Planning..." << std::endl;

    // 1. Attempt Ghost Planning (Ignore other robots)
    PHAStar ghost_planner(robot, target_pose, &timetable, &entities, params,
                          false, "", planning_start_time,
                          "Initial transit ghost planning");
    ghost_planner.set_ignore_other_robots(true);
    ghost_planner.set_debug_popup_enabled(false);
    ghost_planner.max_search_iterations = options.max_search_iterations;
    ghost_planner.set_planner_expansion_threads(options.planner_expansion_threads);
    const auto ghost_search_t0 = std::chrono::steady_clock::now();
    auto ghost_res = ghost_planner.Planning_with_res(planning_start_time);
    if (plan_stats)
    {
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - ghost_search_t0)
                                 .count();
      plan_stats->record_search(
          PlanSearchTier::Other, elapsed, ghost_res.debug_stats.iterations,
          ghost_res.debug_stats.search_iteration_limit_hit,
          ghost_res.debug_stats.heuristic_time_sec,
          ghost_res.debug_stats.primitive_collision_time_sec,
          ghost_res.debug_stats.analytic_validation_time_sec,
          ghost_res.debug_stats.holonomic_heuristic_time_sec);
    }
    append_attempt("ghost planner", ghost_res);

    if (ghost_res.status == PlanningStatus::SUCCESS)
    {
      if (try_schedule_time_aware_path(ghost_res,
                                       "time-aware schedule of ghost path",
                                       true))
      {
        path_res = ghost_res;
      }
      else
      {
        // 2. Identify blockers on the ghost path
        Trajectory ghost_traj;
        ghost_traj.waypoints = ghost_res.waypoints;
        ghost_traj.entity = robot; // FIX: Prevent segfault in collision check
        double initial_t = ghost_traj.waypoints.front().time;
        ghost_traj.start_time = initial_t;
        make_waypoint_times_relative(ghost_traj.waypoints, initial_t);

        CollisionInfo col_info = check_collision_trajectory_detailed(ghost_traj, initial_t, timetable, params, false);

        if (!col_info.is_valid && !col_info.entity_name.empty())
        {
          RobotMeta *blocker = find_resolvable_robot_blocker(
              col_info, timetable, entities);
          EntityMeta *collider = nullptr;
          auto collider_it = entities.find(col_info.entity_name);
          if (collider_it != entities.end())
            collider = collider_it->second;

          bool should_self_park = false;
          std::string blocker_label = col_info.entity_name;
          if (blocker)
          {
            should_self_park =
                timetable.is_waiting(blocker, col_info.time) ||
                timetable.is_entity_static_after(col_info.time, blocker);
            blocker_label = blocker->name;
          }
          else if (collider && collider->type == EntityType::OBJECT)
          {
            should_self_park = true;
          }

          shift_waypoint_times(ghost_traj.waypoints, initial_t);
          ghost_traj.start_time = initial_t;

          if (should_self_park &&
              attempt_self_safe_parking(
                  path_res, ghost_traj,
                  "Ghost path shows higher-priority reserved occupancy from " +
                      blocker_label + " that delay-only scheduling cannot clear.",
                  "retry after self safe parking from ghost path"))
          {
            std::cout << "  [Transit] Self safe parking recovery successful on ghost path."
                      << std::endl;
          }
          else
          {
            std::cerr << "  [Transit] Ghost path blocked by unknown entity/boundary (" << col_info.entity_name << "). Cannot relocate." << std::endl;
          }
        }
      }
    }
  }

  if (path_res.waypoints.empty() &&
      has_initial_method(TransitPlannerMethod::GeometryFallbackHybridAStar))
  {
    Params relaxed = make_relaxed_fallback_params(params);

    PHAStar geometry_planner(robot, target_pose, &timetable, &entities, relaxed,
                             false, "", planning_start_time,
                             "Initial transit geometry fallback");
    geometry_planner.set_ignore_other_robots(true);
    geometry_planner.set_debug_popup_enabled(false);
    geometry_planner.max_search_iterations = options.max_search_iterations;
    geometry_planner.set_planner_expansion_threads(options.planner_expansion_threads);
    const auto geometry_search_t0 = std::chrono::steady_clock::now();
    auto geometry_res = geometry_planner.Planning_with_res(planning_start_time);
    if (plan_stats)
    {
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - geometry_search_t0)
                                 .count();
      plan_stats->record_search(
          PlanSearchTier::Other, elapsed, geometry_res.debug_stats.iterations,
          geometry_res.debug_stats.search_iteration_limit_hit,
          geometry_res.debug_stats.heuristic_time_sec,
          geometry_res.debug_stats.primitive_collision_time_sec,
          geometry_res.debug_stats.analytic_validation_time_sec,
          geometry_res.debug_stats.holonomic_heuristic_time_sec);
    }
    append_attempt("geometry-first fallback", geometry_res);

    if (!geometry_res.waypoints.empty())
    {
      Trajectory geom_traj;
      geom_traj.entity = robot;
      geom_traj.is_transfer = false;
      geom_traj.start_time = planning_start_time;
      geom_traj.waypoints = geometry_res.waypoints;
      make_waypoint_times_relative(geom_traj.waypoints, planning_start_time);

      double wait_added = 0.0;
      double safe_start = find_safe_start_time(&geom_traj, planning_start_time, timetable,
                                               params, entities, options, &wait_added,
                                               nullptr, nullptr,
                                               IdleBlockerRelocationPolicy::RelocateAnyIdle,
                                               plan_stats);
      if (safe_start >= 0.0)
      {
        double delta = safe_start - planning_start_time;
        shift_waypoint_times(geometry_res.waypoints, delta);
        path_res = geometry_res;
        chosen_start_time = safe_start;
        std::cout << "  [Transit] Using geometry-first fallback transit with scheduled start at "
                  << std::fixed << std::setprecision(2) << safe_start << "s" << std::endl;
      }
    }
  }

  if (path_res.waypoints.empty() &&
      has_initial_method(TransitPlannerMethod::ReedSheppFallback))
  {
    double maxc = 1.0 / std::max(robot->transit_turning_radius(), 1e-6);
    auto [rs_x, rs_y, rs_yaw, rs_ctypes, rs_lengths, rs_steers, rs_dirs] =
        ReedShepp::reeds_shepp_path_planning(current_pose.x, current_pose.y, current_pose.yaw,
                                             target_pose.x, target_pose.y, target_pose.yaw,
                                             maxc, params.rs_step_size, robot->wheel_base);

    if (!rs_x.empty())
    {
      std::vector<Waypoint> rs_waypoints;
      rs_waypoints.reserve(rs_x.size());
      double t_abs = planning_start_time;
      for (size_t i = 0; i < rs_x.size(); ++i)
      {
        if (i > 0)
        {
          double d = std::hypot(rs_x[i] - rs_x[i - 1], rs_y[i] - rs_y[i - 1]);
          t_abs += d / std::max(robot->speed_transit, 1e-3);
        }
        Waypoint wp;
        wp.x = rs_x[i];
        wp.y = rs_y[i];
        wp.yaw = rs_yaw[i];
        wp.time = t_abs;
        wp.linear_velocity = rs_dirs.empty() ? robot->speed_transit : rs_dirs[i] * robot->speed_transit;
        wp.steering_angle = rs_steers.empty() ? 0.0 : rs_steers[i];
        rs_waypoints.push_back(wp);
      }

      Trajectory rs_traj;
      rs_traj.entity = robot;
      rs_traj.is_transfer = false;
      rs_traj.start_time = planning_start_time;
      rs_traj.waypoints = rs_waypoints;
      make_waypoint_times_relative(rs_traj.waypoints, planning_start_time);

      double wait_added = 0.0;
      CollisionInfo last_collision;
      double last_check_time = planning_start_time;
      double safe_start = find_safe_start_time(&rs_traj, planning_start_time, timetable,
                                               params, entities, options, &wait_added,
                                               &last_collision, &last_check_time,
                                               IdleBlockerRelocationPolicy::RelocateAnyIdle,
                                               plan_stats);
      if (safe_start >= 0.0)
      {
        path_res.waypoints = rs_waypoints;
        double delta = safe_start - planning_start_time;
        shift_waypoint_times(path_res.waypoints, delta);
        chosen_start_time = safe_start;
        path_res.status = PlanningStatus::SUCCESS;
        path_res.failure_detail.clear();
        path_res.colliding_entity.clear();
        path_res.failure_time = 0.0;
        append_attempt("RS fallback", path_res);
        std::cout << "  [Transit] Using RS fallback transit with scheduled start at "
                  << std::fixed << std::setprecision(2) << safe_start << "s" << std::endl;
      }
      else
      {
        PlanningResult rs_fail;
        rs_fail.status = PlanningStatus::NO_PATH_FOUND;
        rs_fail.failure_detail = "RS fallback found path but no collision-free start slot";
        rs_fail.waypoints = rs_waypoints;
        rs_fail.failure_time = last_check_time;
        rs_fail.colliding_entity = last_collision.entity_name;
        append_attempt("RS fallback", rs_fail);
      }
    }
    else
    {
      PlanningResult rs_fail;
      rs_fail.status = PlanningStatus::NO_PATH_FOUND;
      rs_fail.failure_detail = "RS fallback failed to find geometric path";
      append_attempt("RS fallback", rs_fail);
    }
  }

  const std::string attempt_trace = build_attempt_trace();

  if (path_res.waypoints.empty())
  {
    if (!attempt_trace.empty())
    {
      if (path_res.failure_detail.empty())
      {
        path_res.failure_detail = attempt_trace;
      }
      else
      {
        path_res.failure_detail += "\n" + attempt_trace;
      }
    }

    std::cerr << " [Error] Transit planning failed for " << robot->name
              << " - Status: " << static_cast<int>(path_res.status)
              << ", Detail: " << path_res.failure_detail << std::endl;
    diagnose_planning_failure(robot, current_pose, target_pose, planning_start_time, timetable, entities, params);
    if (DEBUG_VIS)
    {
      if (!debug_attempts.empty())
      {
        visualize_planning_attempt_debug(
            timetable,
            robot,
            planning_start_time,
            current_pose,
            target_pose,
            debug_attempts,
            params,
            "Initial transit");
      }
      else
      {
        visualize_planning_debug(
            timetable,
            robot,
            path_res,
            planning_start_time,
            current_pose,
            target_pose,
            params,
            attempt_trace,
            "Initial transit");
      }
    }
    return false;
  }

  auto validate_initial_transit_candidate =
      [&](PlanningResult &candidate_res,
          const std::string &stage_label) -> bool
  {
    if (candidate_res.waypoints.empty())
      return false;

    const double candidate_start_time = candidate_res.waypoints.front().time;
    Trajectory candidate_traj;
    candidate_traj.entity = robot;
    candidate_traj.start_time = candidate_start_time;
    candidate_traj.waypoints = candidate_res.waypoints;
    candidate_traj.is_transfer = false;
    candidate_traj.kind = TrajectoryKind::TRANSIT;
    make_waypoint_times_relative(candidate_traj.waypoints,
                                 candidate_start_time);

    CollisionInfo validation_info =
        check_collision_trajectory_detailed(candidate_traj,
                                            candidate_start_time,
                                            timetable, params, false);
    if (validation_info.is_valid)
      return true;

    PlanningResult validation_fail;
    validation_fail.status = PlanningStatus::NO_PATH_FOUND;
    validation_fail.failure_detail =
        "initial transit replay validation failed (" +
        validation_info.reason;
    if (!validation_info.entity_name.empty())
      validation_fail.failure_detail += " with " + validation_info.entity_name;
    validation_fail.failure_detail += ")";
    validation_fail.colliding_entity = validation_info.entity_name;
    validation_fail.failure_time = validation_info.time;
    validation_fail.waypoints = candidate_res.waypoints;
    append_attempt(stage_label, validation_fail);

    candidate_res.status = PlanningStatus::NO_PATH_FOUND;
    candidate_res.failure_detail = validation_fail.failure_detail;
    candidate_res.colliding_entity = validation_fail.colliding_entity;
    candidate_res.failure_time = validation_fail.failure_time;
    candidate_res.waypoints.clear();
    return false;
  };

  if (!validate_initial_transit_candidate(path_res,
                                          "initial transit replay validation"))
  {
    if (!tried_fine_initial_transit)
    {
      std::cout << "  [Transit] Initial transit replay validation failed. "
                   "Trying configured fine repair before committing."
                << std::endl;
      path_res = run_initial_transit_method_list(
          "Initial transit after replay validation", true);
      validate_initial_transit_candidate(
          path_res,
          "initial transit repair replay validation");
    }
  }

  if (path_res.waypoints.empty())
  {
    const std::string updated_attempt_trace = build_attempt_trace();
    if (!updated_attempt_trace.empty())
    {
      if (path_res.failure_detail.empty())
        path_res.failure_detail = updated_attempt_trace;
      else
        path_res.failure_detail += "\n" + updated_attempt_trace;
    }

    std::cerr << " [Error] Transit planning failed for " << robot->name
              << " during replay validation - Status: "
              << static_cast<int>(path_res.status)
              << ", Detail: " << path_res.failure_detail << std::endl;
    if (DEBUG_VIS)
    {
      visualize_planning_attempt_debug(
          timetable,
          robot,
          planning_start_time,
          current_pose,
          target_pose,
          debug_attempts,
          params,
          "Initial transit");
    }
    return false;
  }

  if (DEBUG_VIS)
  {
    std::cout << "[Debug] Visualizing Plan..." << std::endl;

    if (!debug_attempts.empty())
    {
      visualize_planning_attempt_debug(
          timetable,
          robot,
          planning_start_time,
          current_pose,
          target_pose,
          debug_attempts,
          params,
          "Initial transit");
    }
    else
    {
      visualize_planning_debug(
          timetable,           // Global history/future of others
          robot,               // The robot executing this plan
          path_res,            // The output plan (waypoints)
          planning_start_time, // Absolute start time for this plan
          current_pose,        // Start
          target_pose,         // Goal
          params,
          attempt_trace,
          "Initial transit");
    }
  }

  // Adjust relative time and register
  make_waypoint_times_relative(path_res.waypoints, chosen_start_time);

  Trajectory transit_traj;
  transit_traj.entity = robot;
  transit_traj.start_time = chosen_start_time;
  transit_traj.waypoints = path_res.waypoints;
  transit_traj.is_transfer = false;
  timetable.add_trajectory(transit_traj);

  if (out_abs_start_time)
    *out_abs_start_time = chosen_start_time;
  if (out_abs_end_time)
  {
    *out_abs_end_time =
        path_res.waypoints.empty()
            ? chosen_start_time
            : (chosen_start_time + path_res.waypoints.back().time);
  }

  return true;
}

bool append_retraction(RobotMeta *robot, const Trajectory &previous_traj,
                       TimeTable &timetable, const Params &params,
                       const std::unordered_map<std::string, EntityMeta *> &entities,
                       const RuntimeOptions &options,
                       TaskExecutionStats *stats,
                       std::string *out_failure_reason,
                       PlanTimingStats *plan_stats)
{
  if (!robot || previous_traj.waypoints.size() < 2)
    return true;

  const double retract_dist = std::max(0.0, options.retraction_distance);
  if (retract_dist <= 1e-9)
    return true;
  const double speed = std::max(robot->speed_transit, 1e-3);
  const size_t num_wp = previous_traj.waypoints.size();
  double actual_retract_dist = 0.0;

  // Backtrack along the just-executed push path. The previous implementation
  // often kept only the terminal waypoint, producing a 0s "retraction".
  std::vector<Waypoint> retract_wp;
  retract_wp.reserve(num_wp);
  Waypoint start_wp = previous_traj.waypoints.back();
  start_wp.time = 0.0;
  start_wp.linear_velocity = 0.0;
  retract_wp.push_back(start_wp);

  for (int i = static_cast<int>(num_wp) - 2;
       i >= 0 && actual_retract_dist < retract_dist - 1e-9;
       --i)
  {
    const Waypoint &toward_start = previous_traj.waypoints[i];
    const Waypoint &toward_end = previous_traj.waypoints[i + 1];
    const double dx = toward_start.x - toward_end.x;
    const double dy = toward_start.y - toward_end.y;
    const double segment_dist = std::hypot(dx, dy);
    if (segment_dist <= 1e-9)
      continue;

    const double remaining_dist = retract_dist - actual_retract_dist;
    const double frac = std::min(1.0, remaining_dist / segment_dist);

    Waypoint wp = toward_end;
    wp.x = toward_end.x + frac * dx;
    wp.y = toward_end.y + frac * dy;
    wp.yaw = mod2pi(toward_end.yaw +
                    frac * pi_2_pi(toward_start.yaw - toward_end.yaw));
    wp.linear_velocity = -speed;
    wp.steering_angle = -toward_end.steering_angle;
    retract_wp.push_back(wp);
    actual_retract_dist += frac * segment_dist;
  }

  const std::vector<Waypoint> path_only_retract_wp = retract_wp;
  const double path_only_retract_dist = actual_retract_dist;
  bool extended_beyond_previous_path = false;

  if (!retract_wp.empty() && actual_retract_dist < retract_dist - 1e-9)
  {
    const double remaining_dist = retract_dist - actual_retract_dist;
    Waypoint wp = retract_wp.back();
    wp.x -= remaining_dist * std::cos(wp.yaw);
    wp.y -= remaining_dist * std::sin(wp.yaw);
    wp.yaw = mod2pi(wp.yaw);
    wp.linear_velocity = -speed;
    wp.steering_angle = 0.0;
    retract_wp.push_back(wp);
    actual_retract_dist += remaining_dist;
    extended_beyond_previous_path = true;
  }

  if (retract_wp.size() < 2 || actual_retract_dist <= 1e-6)
    return true;

  auto assign_retraction_timing =
      [&](std::vector<Waypoint> &waypoints) -> double
  {
    double current_time = 0.0;
    if (waypoints.empty())
      return current_time;

    waypoints[0].time = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
      double d = std::hypot(waypoints[i].x - waypoints[i - 1].x,
                            waypoints[i].y - waypoints[i - 1].y);
      current_time += (d / speed);
      waypoints[i].time = current_time;
    }
    return current_time;
  };

  auto try_schedule_retraction =
      [&](std::vector<Waypoint> candidate_wp,
          double candidate_dist,
          bool used_extension,
          const std::string &failure_prefix) -> bool
  {
    if (candidate_wp.size() < 2 || candidate_dist <= 1e-6)
      return false;

    const double current_time = assign_retraction_timing(candidate_wp);

    Trajectory retract_traj;
    retract_traj.entity = robot;
    double earliest_start = timetable.get_entity_max_time(robot);
    retract_traj.start_time = earliest_start;
    retract_traj.waypoints = candidate_wp;
    retract_traj.is_transfer = false;
    retract_traj.kind = TrajectoryKind::RETRACTION;
    retract_traj.transferred_object = nullptr;
    retract_traj.approach_goal_entity = nullptr;

    double wait_added = 0.0;
    double safe_start = find_safe_start_time(&retract_traj, earliest_start,
                                             timetable, params, entities,
                                             options,
                                             &wait_added, nullptr, nullptr,
                                             IdleBlockerRelocationPolicy::RelocateAnyIdle,
                                             plan_stats);
    if (safe_start < 0.0)
    {
      if (out_failure_reason)
        *out_failure_reason =
            failure_prefix + "retraction has no collision-free slot";
      std::cerr << "  [Retract] Failed to schedule collision-free retraction."
                << std::endl;
      return false;
    }

    retract_traj.start_time = safe_start;
    if (stats && wait_added > 1e-9)
    {
      stats->total_waiting += wait_added;
      stats->delayed_segments += 1;
    }

    TimeTable trial_timetable = timetable;
    trial_timetable.add_trajectory(retract_traj);

    CollisionInfo precommit_collision;
    std::string precommit_failure_reason;
    if (!validate_pre_commit_trajectory(
            trial_timetable, retract_traj, entities, params, nullptr,
            &precommit_collision, &precommit_failure_reason))
    {
      if (out_failure_reason)
        *out_failure_reason = failure_prefix + precommit_failure_reason;
      std::cerr << "  [Retract] Failed strict pre-commit validation."
                << std::endl;
      return false;
    }

    timetable = std::move(trial_timetable);
    if (out_failure_reason)
      out_failure_reason->clear();

    std::ostringstream retract_msg;
    retract_msg << "  [Retract] Backing up " << std::fixed
                << std::setprecision(2) << candidate_dist << "m ("
                << current_time << "s)";
    if (used_extension)
    {
      retract_msg << " by extending "
                  << std::setprecision(2)
                  << (candidate_dist - path_only_retract_dist)
                  << "m beyond the stored push path";
    }
    retract_msg << ".";
    std::cout << retract_msg.str() << std::endl;
    return true;
  };

  if (try_schedule_retraction(retract_wp, actual_retract_dist,
                              extended_beyond_previous_path, ""))
  {
    return true;
  }

  if (extended_beyond_previous_path && path_only_retract_dist > 1e-6 &&
      try_schedule_retraction(path_only_retract_wp, path_only_retract_dist,
                              false, "path-only fallback: "))
  {
    std::cout << "  [Retract] Requested " << std::fixed << std::setprecision(2)
              << retract_dist << "m but only the stored push path was schedulable."
              << std::endl;
    return true;
  }

  return false;
}

SegmentCandidateValidation validate_segment_candidate(
    const std::vector<Waypoint> &candidate_rel,
    RobotMeta *robot,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    EntityMeta *terminal_approach_entity)
{
  SegmentCandidateValidation validation;
  validation.first_hard_collision = {true, "Valid", "", start_time};
  validation.first_soft_robot_collision = {true, "Valid", "", start_time};

  if (candidate_rel.empty() || !robot)
  {
    validation.first_hard_collision =
        {false, "Empty or invalid candidate", "", start_time};
    return validation;
  }

  const double duration = std::max(0.0, candidate_rel.back().time);
  const double dt = shared_collision_check_step(params);

  auto validate_sample = [&](double rel_t) -> bool
  {
    const double abs_t = start_time + rel_t;
    auto pose_tuple = interpolate_timed_path(candidate_rel, rel_t);
    Pose robot_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                       std::get<2>(pose_tuple)};

    CollisionGeometry robot_geom = setup_collision_geometry_for_type(
        robot_pose, EntityType::ROBOT, robot->size, params);
    CollisionGeometry robot_bounds_geom =
        setup_collision_geometry(robot_pose, robot->size, 1.0);

    if (check_robot_bounds_collision(robot_pose, robot_bounds_geom.corners,
                                     params))
    {
      validation.first_hard_collision =
          {false, "Boundary Collision", "Boundary", abs_t};
      return false;
    }

    // Swept directly off the timetable instead of building a fresh
    // get_poses(abs_t) map every sample (planner_opt Stage 2). NOTE
    // (tie-break): if multiple entities collide simultaneously at one
    // sample, which one gets reported can change vs. the old fresh-map
    // iteration order -- see TimeTable::for_each_pose.
    auto collision_result = check_multiple_entities_collision_visit(
        robot_geom, robot_pose,
        nullptr, nullptr,
        [&](auto &&per_entity)
        { timetable.for_each_pose(abs_t, per_entity); },
        params,
        robot,
        nullptr,
        nullptr);

    if (!collision_result.has_collision)
      return true;

    EntityMeta *collider = collision_result.colliding_entity;
    const std::string collider_name = collider ? collider->name : "";
    // colliding_pose was captured directly during the sweep above, so no
    // second others.find(collider) lookup is needed for either excuse check.
    if (rel_t <= 0.3 && collider && collider->type == EntityType::OBJECT)
    {
      if (is_valid_transfer_contact(robot, robot_pose, collider, collision_result.colliding_pose))
      {
        return true;
      }
    }
    if (terminal_approach_entity && collider == terminal_approach_entity &&
        collider->type == EntityType::OBJECT)
    {
      if (is_valid_terminal_approach_contact(
              robot, robot_pose, candidate_rel.back(), collider, collision_result.colliding_pose,
              terminal_approach_entity, params))
      {
        return true;
      }
    }

    CollisionInfo info{false, collision_result.collision_type + " Collision",
                       collider_name, abs_t};
    if (collider && collider->type == EntityType::ROBOT)
    {
      if (!validation.has_soft_robot_conflict)
      {
        validation.has_soft_robot_conflict = true;
        validation.first_soft_robot_collision = info;
      }
      return true;
    }

    validation.first_hard_collision = info;
    return false;
  };

  if (!validate_sample(0.0))
    return validation;

  double first_abs_t = start_time;
  if (dt > 1e-9 && start_time > 0.0)
  {
    first_abs_t = std::ceil((start_time - 1e-9) / dt) * dt;
  }

  for (double abs_t = first_abs_t; abs_t < start_time + duration - 1e-9;
       abs_t += dt)
  {
    const double rel_t = std::max(0.0, abs_t - start_time);
    if (!validate_sample(rel_t))
      return validation;
  }
  if (!validate_sample(duration))
    return validation;

  validation.hard_valid = true;
  return validation;
}

std::vector<Waypoint> waypoints_from_rs_path(const ReedShepp::Path &path,
                                             RobotMeta *robot)
{
  std::vector<Waypoint> rel;
  rel.reserve(path.x.size());
  double t_rel = 0.0;
  for (std::size_t i = 0; i < path.x.size(); ++i)
  {
    if (i > 0)
    {
      const double d = std::hypot(path.x[i] - path.x[i - 1],
                                  path.y[i] - path.y[i - 1]);
      t_rel += d / std::max(robot->speed_transit, 1e-3);
    }

    Waypoint wp;
    wp.x = path.x[i];
    wp.y = path.y[i];
    wp.yaw = mod2pi(path.yaw[i]);
    wp.time = t_rel;
    const int direction = (i < path.directions.size()) ? path.directions[i] : 1;
    wp.linear_velocity = static_cast<double>(direction) * robot->speed_transit;
    wp.steering_angle = (i < path.steers.size()) ? path.steers[i] : 0.0;
    rel.push_back(wp);
  }
  return rel;
}

std::vector<Waypoint> make_reverse_escape_prefix(const Pose &start_pose,
                                                 RobotMeta *robot,
                                                 double steer,
                                                 double escape_distance)
{
  std::vector<Waypoint> prefix;
  if (!robot || escape_distance <= 1e-9)
    return prefix;

  constexpr int kEscapeSamples = 4;
  const double speed = std::max(robot->speed_transit, 1e-3);
  for (int i = 0; i <= kEscapeSamples; ++i)
  {
    const double frac = static_cast<double>(i) / kEscapeSamples;
    const double d = -escape_distance * frac;
    double x = start_pose.x;
    double y = start_pose.y;
    double yaw = start_pose.yaw;
    const double steer_adjusted = -steer;
    if (std::abs(steer_adjusted) < 1e-5)
    {
      x += d * std::cos(yaw);
      y += d * std::sin(yaw);
    }
    else
    {
      const double R = robot->wheel_base / std::tan(steer_adjusted);
      const double beta = d / R;
      x += R * (std::sin(yaw + beta) - std::sin(yaw));
      y += R * (std::cos(yaw) - std::cos(yaw + beta));
      yaw = mod2pi(yaw + beta);
    }

    Waypoint wp;
    wp.x = x;
    wp.y = y;
    wp.yaw = mod2pi(yaw);
    wp.time = escape_distance * frac / speed;
    wp.linear_velocity = (i == 0) ? 0.0 : -speed;
    wp.steering_angle = steer;
    prefix.push_back(wp);
  }
  return prefix;
}

std::vector<Waypoint> combine_prefix_and_tail(
    const std::vector<Waypoint> &prefix_rel,
    std::vector<Waypoint> tail_abs,
    double original_start_time)
{
  std::vector<Waypoint> combined = prefix_rel;
  make_waypoint_times_relative(tail_abs, original_start_time);
  for (std::size_t i = 0; i < tail_abs.size(); ++i)
  {
    if (!combined.empty() && i == 0)
      continue;
    combined.push_back(tail_abs[i]);
  }
  return combined;
}

std::string format_planner_stats(const PlanningDebugStats &stats)
{
  std::ostringstream oss;
  oss << "iterations=" << stats.iterations
      << ", generated=" << stats.generated_nodes
      << ", accepted=" << stats.accepted_nodes
      << ", closed=" << stats.closed_nodes
      << ", peak_open=" << stats.peak_open_size
      << ", reject_collision=" << stats.reject_collision
      << ", reject_closed=" << stats.reject_closed
      << ", reject_worse_g=" << stats.reject_worse_g
      << ", analytic_collision=" << stats.analytic_collision
      << " (boundary=" << stats.analytic_boundary_collision
      << ", object=" << stats.analytic_object_collision
      << ", robot_soft=" << stats.analytic_robot_soft_collision
      << ", other=" << stats.analytic_other_collision << ")"
      << ", analytic_post_arrival_collision="
      << stats.analytic_post_arrival_collision
      << " (boundary="
      << stats.analytic_post_arrival_boundary_collision
      << ", object=" << stats.analytic_post_arrival_object_collision
      << ", robot_soft="
      << stats.analytic_post_arrival_robot_soft_collision
      << ", other=" << stats.analytic_post_arrival_other_collision << ")"
      << ", egraph_nodes=" << stats.reference_egraph_nodes
      << ", egraph_snap_acc=" << stats.reference_egraph_snap_accepted
      << ", egraph_snap_rej=" << stats.reference_egraph_snap_rejected
      << ", egraph_succ_acc=" << stats.reference_egraph_successor_accepted
      << ", egraph_succ_rej=" << stats.reference_egraph_successor_rejected
      << ", mha_anchor_exp=" << stats.mha_anchor_expansions
      << ", mha_ref_exp=" << stats.mha_reference_expansions
      << ", mha_ref_queued=" << stats.mha_reference_queued
      << ", mha_ref_skip_far=" << stats.mha_reference_skipped_far
      << ", iteration_limit_hit=" << stats.search_iteration_limit_hit
      << ", spatial_index=" << (stats.spatial_index_collapses ? "true" : "false")
      << ", wait_skipped=" << stats.wait_primitives_skipped
      << ", expansion_threads=" << stats.planner_expansion_threads
      << ", analytic_threshold=" << stats.analytic_threshold
      << ", analytic_validation_time="
      << std::fixed << std::setprecision(4)
      << stats.analytic_validation_time_sec
      << "s, primitive_collision_time="
      << stats.primitive_collision_time_sec
      << "s, heuristic_time=" << stats.heuristic_time_sec
      << "s, holonomic_build_time=" << stats.holonomic_heuristic_time_sec
      << "s, serial_merge_time=" << stats.serial_merge_time_sec << "s";
  if (std::isfinite(stats.best_dist))
  {
    oss << ", best_dist=" << std::fixed << std::setprecision(3)
        << stats.best_dist
        << ", best_yaw_error=" << stats.best_yaw_error
        << ", best_pose=(" << stats.best_pose.x << ", "
        << stats.best_pose.y << ", " << stats.best_pose.yaw << ")";
  }
  if (!stats.final_failure_reason.empty())
    oss << ", final_failure=" << stats.final_failure_reason;
  return oss.str();
}

void write_segment_replan_diagnostics(
    const SegmentReplanContext &context,
    RobotMeta *robot,
    const Pose &start_pose,
    const Pose &goal_pose,
    double start_time,
    const std::vector<SegmentCandidateReport> &reports,
    bool success,
    const std::string &selected_stage)
{
  std::filesystem::create_directories(diagnostics_dir());
  std::ostringstream filename;
  filename << diagnostics_path("planner_diagnostics_task"
           + std::to_string(context.task_id) + "_"
           + sanitize_filename_component(context.object_name)
           + "_segment" + std::to_string(context.segment_id) + "_"
           + sanitize_filename_component(robot ? robot->name : "unknown")
           + ".txt");

  std::ofstream ofs(filename.str());
  if (!ofs.is_open())
  {
    std::cerr << "[PlannerDiag] Failed to write " << filename.str() << std::endl;
    return;
  }

  ofs << "[PlannerDiag] Segment transit diagnostics\n";
  ofs << "[PlannerDiag] Result: " << (success ? "SUCCESS" : "FAILED") << "\n";
  ofs << "[PlannerDiag] Selected stage: " << selected_stage << "\n";
  ofs << "[PlannerDiag] Task: " << context.task_id
      << ", object: " << context.object_name
      << ", segment: " << context.segment_id << "\n";
  ofs << "[PlannerDiag] Robot: " << (robot ? robot->name : "unknown") << "\n";
  ofs << std::fixed << std::setprecision(3);
  if (robot)
  {
    ofs << "[PlannerDiag] Planner mode: transit, turning_radius="
        << robot->transit_turning_radius()
        << "m, transfer_turning_radius="
        << robot->transfer_turning_radius()
        << "m, legacy_radius=" << robot->min_turning_radius << "m\n";
  }
  ofs << "[PlannerDiag] Start: (" << start_pose.x << ", " << start_pose.y
      << ", " << start_pose.yaw << ") at t=" << start_time << "\n";
  ofs << "[PlannerDiag] Goal:  (" << goal_pose.x << ", " << goal_pose.y
      << ", " << goal_pose.yaw << ")\n";
  if (context.has_live_object_pose)
  {
    ofs << "[PlannerDiag] Live object pose: ("
        << context.live_object_pose.x << ", " << context.live_object_pose.y
        << ", " << context.live_object_pose.yaw << ")\n";
    ofs << "[PlannerDiag] Prepush distances: mars="
        << context.mars_prepush_distance
        << ", source=" << context.source_prepush_distance << "\n";
    ofs << "[PlannerDiag] Source-clearance goal: ("
        << context.source_clearance_goal.x << ", "
        << context.source_clearance_goal.y << ", "
        << context.source_clearance_goal.yaw << ")\n";
  }
  ofs << "[PlannerDiag] Start contact entity: "
      << (context.start_contact_entity.empty() ? "none"
                                               : context.start_contact_entity)
      << "\n";

  for (const auto &report : reports)
  {
    ofs << "[PlannerDiag] " << report.stage << ": "
        << planning_status_name(report.planning.status)
        << ", waypoints=" << report.planning.waypoints.size();
    if (!report.planning.failure_detail.empty())
      ofs << ", detail=" << report.planning.failure_detail;
    if (!report.planning.colliding_entity.empty())
      ofs << ", blocker=" << report.planning.colliding_entity;
    ofs << ", accepted_for_scheduling="
        << (report.accepted_for_scheduling ? "true" : "false")
        << ", selected=" << (report.selected ? "true" : "false")
        << "\n";
    ofs << "[PlannerDiag] " << report.stage
        << " stats: " << format_planner_stats(report.planning.debug_stats)
        << "\n";
    if (report.candidate_tested)
    {
      ofs << "[PlannerDiag] " << report.stage
          << " validation: hard_valid="
          << (report.validation.hard_valid ? "true" : "false")
          << ", soft_robot="
          << (report.validation.has_soft_robot_conflict ? "true" : "false");
      if (!report.validation.first_soft_robot_collision.is_valid)
      {
        ofs << ", first_soft="
            << report.validation.first_soft_robot_collision.reason << " with "
            << report.validation.first_soft_robot_collision.entity_name
            << " at t=" << report.validation.first_soft_robot_collision.time;
      }
      if (!report.validation.first_hard_collision.is_valid)
      {
        ofs << ", first_hard="
            << report.validation.first_hard_collision.reason << " with "
            << report.validation.first_hard_collision.entity_name
            << " at t=" << report.validation.first_hard_collision.time;
      }
      ofs << "\n";
    }
  }

  std::cout << "[PlannerDiag] Wrote segment diagnostics: "
            << filename.str() << std::endl;
}

std::vector<PlanningDebugAttempt> segment_reports_to_debug_attempts(
    const std::vector<SegmentCandidateReport> &reports)
{
  std::vector<PlanningDebugAttempt> attempts;
  attempts.reserve(reports.size());
  for (const auto &report : reports)
  {
    PlanningResult result = report.planning;
    if (!report.debug_waypoints.empty())
      result.waypoints = report.debug_waypoints;

    if (report.candidate_tested && !report.validation.hard_valid)
    {
      result.status = PlanningStatus::NO_PATH_FOUND;
      result.colliding_entity =
          report.validation.first_hard_collision.entity_name;
      result.failure_time = report.validation.first_hard_collision.time;
      std::ostringstream detail;
      detail << "MARS candidate validation rejected this path: "
             << report.validation.first_hard_collision.reason;
      if (!report.validation.first_hard_collision.entity_name.empty())
      {
        detail << " with "
               << report.validation.first_hard_collision.entity_name;
      }
      if (report.validation.has_soft_robot_conflict)
      {
        detail << "; soft robot conflict: "
               << report.validation.first_soft_robot_collision.reason;
        if (!report.validation.first_soft_robot_collision.entity_name.empty())
        {
          detail << " with "
                 << report.validation.first_soft_robot_collision.entity_name;
        }
      }
      result.failure_detail = detail.str();
    }

    attempts.push_back({report.stage, result});
  }
  return attempts;
}

bool replan_transit_segment(
    RobotMeta *robot, const Pose &goal_pose, double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const RuntimeOptions &options,
    std::vector<Waypoint> &out_waypoints_rel,
    const SegmentReplanContext &context,
    EntityMeta *terminal_approach_entity,
    const std::vector<Waypoint> *reference_waypoints,
    PlanTimingStats *plan_stats)
{
  Pose start_pose = timetable.get_pose(robot, start_time);
  robot->initial_pose = start_pose;
  SegmentReplanContext diag_context = context;
  diag_context.start_contact_entity = find_valid_start_contact_entity(
      robot, start_pose, start_time, timetable, entities);
  const double start_goal_dist =
      std::hypot(goal_pose.x - start_pose.x, goal_pose.y - start_pose.y);
  const double start_goal_yaw =
      std::fabs(pi_2_pi(goal_pose.yaw - start_pose.yaw));
  diag_context.tight_or_contact_case =
      !diag_context.start_contact_entity.empty() ||
      (start_goal_dist < 1.5 && start_goal_yaw > M_PI / 2.0);
  const auto near_boundary = [&](const Pose &pose)
  {
    constexpr double kBoundaryMargin = 0.35;
    return pose.x <= params.min_x + kBoundaryMargin ||
           pose.x >= params.max_x - kBoundaryMargin ||
           pose.y <= params.min_y + kBoundaryMargin ||
           pose.y >= params.max_y - kBoundaryMargin;
  };
  const bool contact_boundary_case =
      diag_context.tight_or_contact_case && near_boundary(goal_pose);

  std::vector<SegmentCandidateReport> reports;
  std::string selected_stage;

  auto evaluate_candidate = [&](const std::string &stage,
                                PlanningResult planning,
                                std::vector<Waypoint> candidate_rel) -> bool
  {
    SegmentCandidateReport report;
    report.stage = stage;
    report.planning = std::move(planning);

    if (candidate_rel.empty())
    {
      if (options.print_planning_status)
      {
        std::cout << "  [SegmentStatus] "
                  << format_planning_status_line(stage, report.planning)
                  << std::endl;
      }
      reports.push_back(std::move(report));
      return false;
    }

    report.debug_waypoints = candidate_rel;
    report.candidate_tested = true;
    report.validation = validate_segment_candidate(
        candidate_rel, robot, start_time, timetable, entities, params,
        terminal_approach_entity);
    report.accepted_for_scheduling = report.validation.hard_valid;
    if (!report.validation.hard_valid)
    {
      if (options.print_planning_status)
      {
        std::cout << "  [SegmentStatus] " << stage
                  << " => " << planning_status_name(report.planning.status)
                  << ", validation=FAILED";
        if (!report.validation.first_hard_collision.reason.empty())
        {
          std::cout << ", detail="
                    << report.validation.first_hard_collision.reason;
          if (!report.validation.first_hard_collision.entity_name.empty())
            std::cout << " with "
                      << report.validation.first_hard_collision.entity_name;
        }
        std::cout << std::endl;
      }
      reports.push_back(std::move(report));
      return false;
    }

    out_waypoints_rel = std::move(candidate_rel);
    selected_stage = stage;
    report.selected = true;
    if (options.print_planning_status)
    {
      std::cout << "  [SegmentStatus] " << stage
                << " => " << planning_status_name(report.planning.status)
                << ", validation=PASS"
                << ", waypoints=" << report.debug_waypoints.size()
                << std::endl;
    }
    reports.push_back(std::move(report));
    return true;
  };

  auto run_hybrid_plan = [&](const Pose &plan_start_pose,
                             double plan_start_time,
                             const Params &plan_params,
                             int max_iter,
                             const std::string &stage,
                             PlanSearchTier tier,
                             bool allow_reference_egraph = true) -> PlanningResult
  {
    Color::println("[PHAStar] Attempting search method: " + stage, Color::CYAN);
    robot->initial_pose = plan_start_pose;
    const std::string terminal_contact_name =
        terminal_approach_entity ? terminal_approach_entity->name : "";
    PHAStar planner(robot, goal_pose, &timetable, &entities, plan_params,
                    false, "", plan_start_time, stage, terminal_contact_name);
    planner.set_ignore_other_robots(true);
    planner.max_search_iterations = max_iter;
    planner.set_planner_expansion_threads(options.planner_expansion_threads);
    if (allow_reference_egraph && reference_waypoints &&
        reference_waypoints->size() >= 2)
    {
      planner.set_reference_experience_graph(
          *reference_waypoints, reference_egraph_options_from_runtime(options));
      if (planner.has_reference_experience_graph())
      {
        std::cout << "[PHAStar] Reference E-Graph guide active for "
                  << stage << " (input waypoints="
                  << reference_waypoints->size() << ")." << std::endl;
      }
    }
    const auto search_t0 = std::chrono::steady_clock::now();
    PlanningResult res = planner.Planning_with_res(plan_start_time);
    if (plan_stats)
    {
      const double elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - search_t0)
                                 .count();
      plan_stats->record_search(
          tier, elapsed, res.debug_stats.iterations,
          res.debug_stats.search_iteration_limit_hit,
          res.debug_stats.heuristic_time_sec,
          res.debug_stats.primitive_collision_time_sec,
          res.debug_stats.analytic_validation_time_sec,
          res.debug_stats.holonomic_heuristic_time_sec);
    }
    return res;
  };

  auto evaluate_hybrid_stage = [&](const std::string &stage,
                                   const Pose &plan_start_pose,
                                   double plan_start_time,
                                   const Params &plan_params,
                                   int max_iter,
                                   PlanSearchTier tier,
                                   const std::vector<Waypoint> *prefix = nullptr,
                                   bool allow_reference_egraph = true,
                                   PlanningResult *out_res = nullptr) -> bool
  {
    PlanningResult res = run_hybrid_plan(plan_start_pose, plan_start_time,
                                         plan_params, max_iter, stage, tier,
                                         allow_reference_egraph);
    if (out_res)
      *out_res = res;
    if (res.waypoints.empty())
      return evaluate_candidate(stage, res, {});

    std::vector<Waypoint> candidate_rel;
    if (prefix)
      candidate_rel = combine_prefix_and_tail(*prefix, res.waypoints, start_time);
    else
    {
      candidate_rel = res.waypoints;
      make_waypoint_times_relative(candidate_rel, start_time);
    }
    return evaluate_candidate(stage, res, std::move(candidate_rel));
  };

  // Stage 3 holonomic feasibility pre-gate wrapper for this cascade's Fine
  // and Contact dispatch (see run_gateable_hybrid_stage below): when the
  // gate is enabled and proves the goal unreachable at `plan_params`'s
  // holonomic resolution, synthesizes the same "tier ran and failed" shape
  // (a NO_PATH_FOUND/empty-waypoints PlanningResult run through
  // evaluate_candidate for reporting) that a real failed search would
  // produce, without spending time on the real search.
  auto run_gateable_hybrid_stage = [&](const std::string &stage,
                                       const Params &plan_params,
                                       int max_iter,
                                       PlanSearchTier tier,
                                       bool use_reference_egraph,
                                       PlanningResult &out_tier_result) -> bool
  {
    if (options.enable_tier_gate)
    {
      PlanningResult gated;
      if (apply_tier_gate(gated, tier, start_pose, goal_pose, robot, timetable,
                          entities, plan_params, start_time, plan_stats))
      {
        out_tier_result = gated;
        return evaluate_candidate(stage, gated, {});
      }
    }
    return evaluate_hybrid_stage(stage, start_pose, start_time, plan_params,
                                 max_iter, tier, nullptr, use_reference_egraph,
                                 &out_tier_result);
  };

  const bool start_contact_case = !diag_context.start_contact_entity.empty();
  const Params fine_params = make_fine_segment_params(params, options);
  const int fine_max_iter = std::max(
      options.max_search_iterations,
      options.fine_segment_max_search_iterations);
  const Params contact_params =
      make_contact_boundary_segment_params(params, options);
  const int contact_max_iter = std::max(
      fine_max_iter, options.contact_boundary_max_search_iterations);

  auto step_applies = [&](const TransitPlannerStep &step) -> bool
  {
    switch (step.applicability)
    {
    case TransitPlannerApplicability::Always:
      return true;
    case TransitPlannerApplicability::TightOrContactOnly:
      return diag_context.tight_or_contact_case;
    case TransitPlannerApplicability::ContactBoundaryOnly:
      return contact_boundary_case;
    case TransitPlannerApplicability::StartContactOnly:
      return start_contact_case;
    case TransitPlannerApplicability::NonStartContactOnly:
      return !start_contact_case;
    case TransitPlannerApplicability::StartContactTightOrContactOnly:
      return start_contact_case && diag_context.tight_or_contact_case;
    case TransitPlannerApplicability::NonStartContactTightOrContactOnly:
      return !start_contact_case && diag_context.tight_or_contact_case;
    }
    return false;
  };

  auto evaluate_rs_candidates = [&](const Pose &rs_start_pose,
                                    double rs_start_time,
                                    const Params &rs_params,
                                    const std::string &stage_prefix,
                                    const std::vector<Waypoint> *prefix = nullptr) -> bool
  {
    Color::println("[PHAStar] Attempting search method: " + stage_prefix, Color::CYAN);
    const double maxc =
        1.0 / std::max(robot->transit_turning_radius(), 1e-6);
    auto paths = ReedShepp::calc_paths(rs_start_pose.x, rs_start_pose.y,
                                       rs_start_pose.yaw,
                                       goal_pose.x, goal_pose.y, goal_pose.yaw,
                                       maxc, rs_params.rs_step_size,
                                       robot->wheel_base);
    std::sort(paths.begin(), paths.end(),
              [](const ReedShepp::Path &a, const ReedShepp::Path &b)
              { return a.L < b.L; });

    for (std::size_t i = 0; i < paths.size(); ++i)
    {
      PlanningResult rs_res;
      rs_res.status = PlanningStatus::SUCCESS;
      rs_res.waypoints = waypoints_from_rs_path(paths[i], robot);
      shift_waypoint_times(rs_res.waypoints, rs_start_time);

      std::vector<Waypoint> candidate_rel;
      if (prefix)
        candidate_rel = combine_prefix_and_tail(*prefix, rs_res.waypoints,
                                                start_time);
      else
      {
        candidate_rel = rs_res.waypoints;
        make_waypoint_times_relative(candidate_rel, start_time);
      }

      std::ostringstream stage;
      stage << stage_prefix << " RS candidate " << i << " "
            << paths[i].ctypes << " L=" << std::fixed << std::setprecision(3)
            << paths[i].L;
      if (evaluate_candidate(stage.str(), rs_res, std::move(candidate_rel)))
        return true;
    }
    return false;
  };

  const double escape_distance = 0.12;
  const double escape_steer =
      robot->transit_turning_radius() > 1e-9
          ? std::atan(robot->wheel_base / robot->transit_turning_radius())
          : 0.0;

  auto evaluate_escape_hybrid = [&](const std::string &stage,
                                    double steer_angle) -> bool
  {
    if (!start_contact_case)
      return false;
    auto prefix = make_reverse_escape_prefix(start_pose, robot,
                                             steer_angle, escape_distance);
    if (prefix.empty())
      return false;
    const Waypoint &escape_end = prefix.back();
    Pose escape_pose{escape_end.x, escape_end.y, escape_end.yaw};
    const double escape_abs_time = start_time + escape_end.time;
    return evaluate_hybrid_stage(stage, escape_pose, escape_abs_time,
                                 fine_params, fine_max_iter,
                                 PlanSearchTier::Fine, &prefix);
  };

  auto evaluate_escape_rs = [&](const std::string &stage,
                                double steer_angle) -> bool
  {
    if (!start_contact_case)
      return false;
    auto prefix = make_reverse_escape_prefix(start_pose, robot,
                                             steer_angle, escape_distance);
    if (prefix.empty())
      return false;
    const Waypoint &escape_end = prefix.back();
    Pose escape_pose{escape_end.x, escape_end.y, escape_end.yaw};
    const double escape_abs_time = start_time + escape_end.time;
    return evaluate_rs_candidates(escape_pose, escape_abs_time, fine_params,
                                  stage, &prefix);
  };

  // Stage 3 failure triage (RuntimeOptions::enable_tier_triage). Rule 1
  // (BLOCKED-BY-ROBOT SHORT-CIRCUIT) IS implemented in this cascade: unlike
  // plan_initial_transit's method loop, escalation here is governed by
  // evaluate_candidate()'s independent hard-collision revalidation
  // (validate_segment_candidate), not by PlanningStatus, so a
  // BLOCKED_BY_ROBOT tier result can otherwise fall through to the next
  // tier -- see the design note above triage_blocked_by_robot_shortcut_applies.
  const bool triage_fine_then_contact_segment =
      method_list_has_fine_before_contact(options.segment_transit_methods);

  // Mirrors this loop's own step_applies filtering so "the next tier" means
  // the next one this loop will actually attempt.
  auto next_segment_step_is_fine = [&](std::size_t from_idx) -> bool
  {
    for (std::size_t j = from_idx + 1;
         j < options.segment_transit_methods.size(); ++j)
    {
      const auto &candidate = options.segment_transit_methods[j];
      if (!step_applies(candidate))
        continue;
      return candidate.method == TransitPlannerMethod::FineHybridAStar;
    }
    return false;
  };

  bool skip_next_fine = false;
  for (std::size_t idx = 0; idx < options.segment_transit_methods.size(); ++idx)
  {
    const auto &step = options.segment_transit_methods[idx];
    if (!step_applies(step))
      continue;

    if (options.enable_tier_triage && skip_next_fine &&
        step.method == TransitPlannerMethod::FineHybridAStar)
    {
      skip_next_fine = false;
      if (plan_stats)
        ++plan_stats->n_triage_skips_to_contact;
      continue;
    }
    skip_next_fine = false;

    const bool use_reference_egraph = transit_step_uses_reference_egraph(step);
    const std::string stage = transit_planner_method_name(step.method);
    bool accepted = false;
    PlanningResult tier_result;
    bool has_tier_result = false;
    switch (step.method)
    {
    case TransitPlannerMethod::PrimaryHybridAStar:
      accepted = evaluate_hybrid_stage(stage, start_pose, start_time, params,
                                       options.max_search_iterations,
                                       PlanSearchTier::Primary, nullptr,
                                       use_reference_egraph, &tier_result);
      has_tier_result = true;
      break;
    case TransitPlannerMethod::FineHybridAStar:
      accepted = run_gateable_hybrid_stage(
          stage, fine_params, fine_max_iter, PlanSearchTier::Fine,
          use_reference_egraph, tier_result);
      has_tier_result = true;
      break;
    case TransitPlannerMethod::ContactBoundaryGeometricHybridAStar:
      accepted = run_gateable_hybrid_stage(
          stage, contact_params, contact_max_iter, PlanSearchTier::Contact,
          use_reference_egraph, tier_result);
      has_tier_result = true;
      break;
    case TransitPlannerMethod::AllCandidateReedShepp:
      accepted = evaluate_rs_candidates(start_pose, start_time, fine_params,
                                        stage);
      break;
    case TransitPlannerMethod::ReverseStraightEscapeFineHybridAStar:
      accepted = evaluate_escape_hybrid(stage, 0.0);
      break;
    case TransitPlannerMethod::ReverseLeftEscapeFineHybridAStar:
      accepted = evaluate_escape_hybrid(stage, escape_steer);
      break;
    case TransitPlannerMethod::ReverseRightEscapeFineHybridAStar:
      accepted = evaluate_escape_hybrid(stage, -escape_steer);
      break;
    case TransitPlannerMethod::ReverseStraightEscapeReedShepp:
      accepted = evaluate_escape_rs(stage, 0.0);
      break;
    case TransitPlannerMethod::ReverseLeftEscapeReedShepp:
      accepted = evaluate_escape_rs(stage, escape_steer);
      break;
    case TransitPlannerMethod::ReverseRightEscapeReedShepp:
      accepted = evaluate_escape_rs(stage, -escape_steer);
      break;
    default:
      break;
    }

    bool stop_cascade_failed = false;
    if (options.enable_tier_triage && has_tier_result && !accepted)
    {
      if (triage_blocked_by_robot_shortcut_applies(tier_result))
      {
        // Rule 1: stop escalating entirely and let this BLOCKED_BY_ROBOT
        // result stand as the cascade's outcome, exactly as if the later
        // tiers had also failed with the same backup -- the existing
        // post-loop write_segment_replan_diagnostics()/return false below
        // already handles "cascade exhausted" uniformly via `reports`.
        if (plan_stats)
          ++plan_stats->n_triage_blocked_shortcuts;
        stop_cascade_failed = true;
      }
      else
      {
        // Rule 2 (CAP-HIT-WITH-PROGRESS RETRY, primary tier only).
        if (step.method == TransitPlannerMethod::PrimaryHybridAStar)
        {
          const double straight_line_dist =
              std::hypot(goal_pose.x - start_pose.x, goal_pose.y - start_pose.y);
          if (triage_primary_retry_applies(tier_result, straight_line_dist))
          {
            if (plan_stats)
              ++plan_stats->n_triage_primary_retries;
            const std::string retry_stage = stage + " (triage retry)";
            PlanningResult retry_res = run_hybrid_plan(
                start_pose, start_time, params,
                2 * options.max_search_iterations, retry_stage,
                PlanSearchTier::Primary, use_reference_egraph);
            bool retry_accepted;
            if (retry_res.waypoints.empty())
            {
              retry_accepted = evaluate_candidate(retry_stage, retry_res, {});
            }
            else
            {
              std::vector<Waypoint> retry_rel = retry_res.waypoints;
              make_waypoint_times_relative(retry_rel, start_time);
              retry_accepted = evaluate_candidate(retry_stage, retry_res,
                                                  std::move(retry_rel));
            }
            // The retry's result replaces the original failure for any
            // subsequent triage rule (never retried twice).
            tier_result = retry_res;
            if (retry_accepted)
            {
              if (plan_stats)
                ++plan_stats->n_triage_retry_successes;
              accepted = true;
            }
            else if (triage_blocked_by_robot_shortcut_applies(tier_result))
            {
              // Rule 1, re-applied to the retry's own failure.
              if (plan_stats)
                ++plan_stats->n_triage_blocked_shortcuts;
              stop_cascade_failed = true;
            }
          }
        }

        // Rule 3 (NEAR-GOAL-COLLISION SKIP-TO-CONTACT).
        if (!accepted && !stop_cascade_failed &&
            triage_fine_then_contact_segment &&
            next_segment_step_is_fine(idx) &&
            triage_skip_to_contact_applies(tier_result))
        {
          skip_next_fine = true;
        }
      }
    }

    if (accepted)
    {
      if (diag_context.tight_or_contact_case)
        write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                         goal_pose, start_time, reports, true,
                                         selected_stage);
      return true;
    }
    if (stop_cascade_failed)
      break;
  }

  write_segment_replan_diagnostics(diag_context, robot, start_pose,
                                   goal_pose, start_time, reports, false,
                                   selected_stage.empty() ? "none"
                                                          : selected_stage);
  if (DEBUG_VIS && !reports.empty())
  {
    auto debug_attempts = segment_reports_to_debug_attempts(reports);
    visualize_planning_attempt_debug(
        timetable, robot, start_time, start_pose, goal_pose,
        debug_attempts, params,
        "Segment " + std::to_string(context.segment_id) +
            " transit replanning");
  }
  return false;
}

// ==========================================
// Waypoint Time Helpers (Extracted)
// ==========================================

void make_waypoint_times_relative(std::vector<Waypoint> &waypoints,
                                  double reference_time)
{
  shift_waypoint_times(waypoints, -reference_time);
}
