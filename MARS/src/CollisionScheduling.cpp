/*****************************************************************
 * Collision Checking & Scheduling
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <CollisionScheduling.h>
#include <SafeParking.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/CollisionUtils.h>
#include <PHAstar/Visualization.h>
#include <CsvLogging.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

// Forward declarations for functions still in the monolith
// relocate_blocking_robot is in SafeParking.h, don't forward declare
void accumulate_wait_stats(TaskExecutionStats *stats, double wait_added);
void project_waypoints_inside_bounds(std::vector<Waypoint> &waypoints, RobotMeta *robot,
                                     const Params &params, EntityMeta *transferred_object = nullptr);
std::tuple<double, double, double> interpolate_timed_path(
    const std::vector<Waypoint> &path, double relative_time);
void make_waypoint_times_relative(std::vector<Waypoint> &waypoints, double reference_time);

RobotMeta *find_resolvable_robot_blocker(
    const CollisionInfo &col_info,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto ent_it = entities.find(col_info.entity_name);
  if (ent_it == entities.end())
    return nullptr;

  EntityMeta *collider = ent_it->second;
  if (!collider)
    return nullptr;

  if (collider->type == EntityType::ROBOT)
  {
    return dynamic_cast<RobotMeta *>(collider);
  }

  if (collider->type == EntityType::OBJECT)
  {
    return find_robot_transferring_object_at_time(
        collider, col_info.time, timetable);
  }

  return nullptr;
}

// ==========================================
// Timetable Verification
// ==========================================

double shared_collision_check_step(const Params &params);

TimeTableVerificationResult verify_timetable_collision_free(
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> &transfer_windows,
    double from_time,
    double to_time,
    double step)
{
  TimeTableVerificationResult result;
  double max_t = timetable.get_max_time();
  if (to_time >= from_time)
  {
    max_t = std::min(max_t, to_time);
  }
  if (step <= 0.0)
  {
    step = shared_collision_check_step(params);
  }

  double first_sample_time = from_time;
  if (from_time > 0.0)
  {
    first_sample_time = std::ceil((from_time - 1e-9) / step) * step;
  }

  for (double t = first_sample_time; t <= max_t + 1e-9; t += step)
  {
    // Swept directly off the timetable instead of building a fresh
    // get_poses(t) map and then re-copying it into `entries` (planner_opt
    // Stage 2): for_each_pose fills the same reusable vector in one pass.
    //
    // NOTE (tie-break): if multiple entities are simultaneously out of
    // bounds at the same sample, the old code reported whichever one
    // get_poses(t)'s fresh-map iteration order visited first; this reports
    // whichever one per_entity_table's own iteration order visits first.
    // Those orders are not guaranteed identical (see
    // TimeTable::for_each_pose), so which entity/reason gets reported can
    // change in that rare case.
    std::vector<std::pair<EntityMeta *, Pose>> entries;
    bool out_of_bounds_found = false;
    timetable.for_each_pose(t, [&](EntityMeta *ent, const Pose &pose)
    {
      if (!out_of_bounds_found)
      {
        CollisionGeometry geom = setup_collision_geometry(pose, ent->size, 1.0);
        if (check_entity_bounds_collision(ent->type, pose, geom.corners, params))
        {
          out_of_bounds_found = true;
          result.is_valid = false;
          result.time = t;
          result.reason = "Boundary collision";
          result.entity_a = ent->name;
          result.entity_b = "Boundary";
        }
      }
      entries.emplace_back(ent, pose);
    });

    if (out_of_bounds_found)
    {
      return result;
    }

    for (size_t i = 0; i < entries.size(); ++i)
    {
      auto [e1, p1] = entries[i];
      CollisionGeometry g1 =
          setup_collision_geometry_for_type(p1, e1->type, e1->size, params);

      for (size_t j = i + 1; j < entries.size(); ++j)
      {
        auto [e2, p2] = entries[j];
        auto collision = check_entity_collision(g1, p1, e2, p2, params);
        if (!collision.has_collision)
          continue;

        if (has_active_transfer_contact(e1, e2, t, transfer_windows))
          continue;

        if (is_valid_transfer_contact(e1, p1, e2, p2))
          continue;

        bool robot_object_pair =
            (e1->type == EntityType::ROBOT && e2->type == EntityType::OBJECT) ||
            (e2->type == EntityType::ROBOT && e1->type == EntityType::OBJECT);
        if (robot_object_pair && has_transfer_pair(e1, e2, transfer_windows))
          continue;

        result.is_valid = false;
        result.time = t;
        result.reason = "Entity collision";
        result.entity_a = e1->name;
        result.entity_b = e2->name;
        return result;
      }
    }
  }

  return result;
}

namespace
{
  double positive_or(double value, double fallback)
  {
    return value > 0.0 ? value : fallback;
  }

  int positive_or(int value, int fallback)
  {
    return value > 0 ? value : fallback;
  }
}

// ==========================================
// Collision Check Step Size
// ==========================================

double shared_collision_check_step(const Params &params)
{
  const double planner_step =
      params.time_step / static_cast<double>(std::max(1, params.collision_steps));
  return std::max(1e-3, std::min(params.collision_check_time_step,
                                 planner_step));
}

// ==========================================
// Transfer Window Management
// ==========================================

void append_transfer_window_if_needed(
    const Trajectory &traj,
    std::vector<TransferContactWindow> *transfer_windows)
{
  if (traj.is_transfer && traj.entity && traj.transferred_object &&
      transfer_windows && !traj.waypoints.empty())
  {
    double st = traj.start_time + traj.waypoints.front().time;
    double et = traj.start_time + traj.waypoints.back().time;
    transfer_windows->push_back({traj.entity, traj.transferred_object, st, et});
  }
}

std::vector<TransferContactWindow> transfer_windows_with_candidate(
    const std::vector<TransferContactWindow> *transfer_windows,
    const Trajectory &traj)
{
  std::vector<TransferContactWindow> windows;
  if (transfer_windows)
    windows = *transfer_windows;

  if (traj.is_transfer && traj.entity && traj.transferred_object &&
      !traj.waypoints.empty())
  {
    TransferContactWindow w;
    w.robot = traj.entity;
    w.object = traj.transferred_object;
    w.start_time = traj.start_time;
    w.end_time = traj.start_time + traj.waypoints.back().time;
    windows.push_back(w);
  }

  return windows;
}

// ==========================================
// Trajectory Collision Checking
// ==========================================

CollisionInfo check_collision_trajectory_detailed(const Trajectory &traj, double start_time,
                                                  TimeTable &timetable, const Params &params,
                                                  bool verbose,
                                                  EntityMeta *terminal_approach_entity)
{
  if (traj.waypoints.empty())
    return {true, "Empty Trajectory", "", start_time};

  double dt = shared_collision_check_step(params);
  double duration = traj.waypoints.back().time;

  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  ObjectMeta *object = dynamic_cast<ObjectMeta *>(traj.transferred_object);

  if (!robot)
    return {false, "Invalid Trajectory", "", start_time};

  for (double t = 0; t <= duration; t += dt)
  {
    double abs_t = start_time + t;
    auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
    Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                   std::get<2>(pose_tuple)};

    CollisionGeometry r_geom = setup_collision_geometry_for_type(
        r_pose, EntityType::ROBOT, robot->size, params);

    CollisionGeometry r_geom_bounds = setup_collision_geometry(r_pose, robot->size, 1.0);

    if (check_robot_bounds_collision(r_pose, r_geom_bounds.corners, params))
    {
      return {false, "Boundary Collision", "Boundary", abs_t};
    }

    CollisionGeometry obj_geom;
    Pose obj_pose;
    const CollisionGeometry *obj_geom_ptr = nullptr;
    const Pose *obj_pose_ptr = nullptr;

    if (traj.is_transfer && object)
    {
      double offset = robot->size.front_length + object->size.rear_length;
      obj_pose = {r_pose.x + offset * std::cos(r_pose.yaw),
                  r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
      obj_geom = setup_collision_geometry_for_type(
          obj_pose, EntityType::OBJECT, object->size, params);

      CollisionGeometry obj_geom_bounds = setup_collision_geometry(obj_pose, object->size, 1.0);

      if (check_object_bounds_collision(obj_pose, obj_geom_bounds.corners, params))
      {
        return {false, "Boundary Collision", "Boundary", abs_t};
      }

      obj_geom_ptr = &obj_geom;
      obj_pose_ptr = &obj_pose;
    }

    // Swept directly off the timetable instead of building a fresh
    // get_poses(abs_t) map every sample (planner_opt Stage 2). See
    // TimeTable::for_each_pose for the tie-break caveat: this can only
    // change which entity is reported in the rare case of multiple
    // simultaneous colliders at one sample.
    auto collision_result = check_multiple_entities_collision_visit(
        r_geom, r_pose,
        obj_geom_ptr, obj_pose_ptr,
        [&](auto &&per_entity)
        { timetable.for_each_pose(abs_t, per_entity); },
        params,
        robot,
        object,
        nullptr);
    if (collision_result.has_collision)
    {
      EntityMeta *collider = collision_result.colliding_entity;
      // colliding_pose was captured directly during the sweep above, so no
      // second others.find(collider) lookup is needed for either excuse
      // check below.
      if (t <= 0.3 && collider &&
          collider->type == EntityType::OBJECT)
      {
        if (is_valid_transfer_contact(robot, r_pose, collider, collision_result.colliding_pose))
        {
          continue;
        }
      }
      if (!traj.is_transfer && terminal_approach_entity && collider &&
          collider == terminal_approach_entity &&
          collider->type == EntityType::OBJECT)
      {
        const Pose terminal_pose = traj.waypoints.back();
        if (is_valid_terminal_approach_contact(
                robot, r_pose, terminal_pose, collider, collision_result.colliding_pose,
                terminal_approach_entity, params))
        {
          continue;
        }
      }

      return {false, collision_result.collision_type + " Collision",
              collider ? collider->name : "",
              abs_t};
    }
  }
  return {true, "Valid", "", 0.0};
}

bool check_collision_trajectory(const Trajectory &traj, double start_time,
                                TimeTable &timetable, const Params &params,
                                bool verbose)
{
  if (traj.waypoints.empty())
    return true;
  return check_collision_trajectory_detailed(
             traj, start_time, timetable, params, verbose,
             traj.approach_goal_entity)
      .is_valid;
}

// Debug-only A/B escape hatch (planner_opt Stage 2): flip to false to force
// every check_terminal_hold_detailed call back onto the pre-Stage-2
// per-candidate forward scan, even when a caller supplies a hold_cache.
// Callers that pass hold_cache=nullptr (every call site except
// find_safe_start_time) are completely unaffected by this constant either
// way, since the cache branch below is only entered when hold_cache is
// non-null.
constexpr bool kUseTerminalHoldCache = true;

// TerminalHoldCache itself is defined in CollisionScheduling.h (needs to be
// a complete type there so find_safe_start_time and unit tests can own an
// instance). Design summary for the build algorithm below: for a FIXED
// (trajectory terminal pose, timetable) pair, the set of dt-grid samples
// from just above 0 through the timetable horizon at which a parked robot
// would see an unexcused hold collision is fixed; only *which* candidate
// arrival times end up probing into that set changes across
// find_safe_start_time's candidate loop. Building it once -- a single
// backward scan from the horizon, early-exiting at the first (i.e. largest)
// colliding sample -- turns every candidate's hold decision into an O(1)
// comparison against max_colliding_sample, instead of a fresh O(horizon/dt)
// forward scan per candidate.
//
// The cached reason/entity_name come from whichever sample the backward
// scan happened to stop at (the largest colliding one), not necessarily the
// smallest colliding sample >= a given candidate's first_sample_time (the
// one the old per-candidate scan would report). This can only make the
// *reported* CollisionInfo fields differ from the old path for a FAILING
// candidate when multiple, differently-caused hold conflicts exist at
// different times; the pass/fail decision itself is always exact (see
// check_terminal_hold_detailed). See the Stage 2 report for why this is safe
// in practice: a hold failure's exact entity/reason only ever surfaces in
// diagnostic text (find_safe_start_time only branches on is_valid for hold
// failures, via terminal_hold_conflict), and time is always recomputed fresh
// per query rather than cached.
//
// MUST be invalidated (invalidate()) whenever the timetable or the candidate
// trajectory's terminal pose/duration can have changed -- find_safe_start_time
// has exactly two such in-loop mutation points; see its body.

namespace
{
// Per-sample body shared by check_terminal_hold_detailed's original forward
// scan and TerminalHoldCache's one-time backward build scan, so the two
// paths can never disagree on what counts as a hold failure. Returns
// {true, "Valid", "", abs_t} when abs_t is clear (including "excused"
// terminal-approach contacts), or {false, reason, entity_name, abs_t} for a
// genuine (unexcused) collision -- exactly check_terminal_hold_detailed's old
// per-sample loop body, just factored out and driven by
// TimeTable::for_each_pose instead of a fresh get_poses(abs_t) map.
CollisionInfo evaluate_terminal_hold_sample(
    RobotMeta *robot, const Pose &goal_pose, bool is_transfer,
    EntityMeta *transferred_object, EntityMeta *terminal_approach_entity,
    TimeTable &timetable, const Params &params, double abs_t)
{
  CollisionGeometry r_geom = setup_collision_geometry_for_type(
      goal_pose, EntityType::ROBOT, robot->size, params);
  CollisionGeometry r_geom_bounds =
      setup_collision_geometry(goal_pose, robot->size, 1.0);

  if (check_robot_bounds_collision(goal_pose, r_geom_bounds.corners, params))
  {
    return {false, "Boundary Collision", "Boundary", abs_t};
  }

  auto collision_result = check_multiple_entities_collision_visit(
      r_geom, goal_pose,
      nullptr, nullptr,
      [&](auto &&per_entity)
      { timetable.for_each_pose(abs_t, per_entity); },
      params,
      robot,
      is_transfer ? transferred_object : nullptr,
      nullptr);
  if (!collision_result.has_collision)
  {
    return {true, "Valid", "", abs_t};
  }

  EntityMeta *collider = collision_result.colliding_entity;
  if (terminal_approach_entity && collider == terminal_approach_entity &&
      collider && collider->type == EntityType::OBJECT &&
      is_valid_terminal_approach_contact(
          robot, goal_pose, goal_pose, collider, collision_result.colliding_pose,
          terminal_approach_entity, params))
  {
    return {true, "Valid", "", abs_t};
  }

  return {false, collision_result.collision_type + " Collision",
          collider ? collider->name : "", abs_t};
}
} // namespace

CollisionInfo check_terminal_hold_detailed(
    const Trajectory &traj,
    double start_time,
    TimeTable &timetable,
    const Params &params,
    EntityMeta *terminal_approach_entity,
    TerminalHoldCache *hold_cache)
{
  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  if (!robot || traj.waypoints.empty())
    return {true, "No terminal hold to evaluate", "", start_time};

  const double duration = std::max(0.0, traj.waypoints.back().time);
  const double arrival_time = start_time + duration;
  const double horizon = timetable.get_max_time();
  if (horizon <= arrival_time + 1e-9)
    return {true, "Valid", "", arrival_time};

  const Pose goal_pose = traj.waypoints.back();
  const double dt = shared_collision_check_step(params);
  double first_sample_time = arrival_time + dt;
  if (first_sample_time > 0.0)
  {
    first_sample_time = std::ceil((first_sample_time - 1e-9) / dt) * dt;
  }

  if (hold_cache && kUseTerminalHoldCache)
  {
    if (!hold_cache->valid)
    {
      hold_cache->empty = true;
      hold_cache->max_colliding_sample = 0.0;
      hold_cache->reason.clear();
      hold_cache->entity_name.clear();

      // Largest dt-grid index k (abs_t = k*dt) that a forward scan could
      // ever reach for any arrival time, i.e. the same top-of-range value
      // the loop below reaches via abs_t <= horizon + 1e-9.
      const long top_index = static_cast<long>(std::floor((horizon + 1e-9) / dt));
      for (long k = top_index; k >= 1; --k)
      {
        const double abs_t = static_cast<double>(k) * dt;
        CollisionInfo sample = evaluate_terminal_hold_sample(
            robot, goal_pose, traj.is_transfer, traj.transferred_object,
            terminal_approach_entity, timetable, params, abs_t);
        if (!sample.is_valid)
        {
          hold_cache->empty = false;
          hold_cache->max_colliding_sample = abs_t;
          hold_cache->reason = sample.reason;
          hold_cache->entity_name = sample.entity_name;
          break;
        }
      }
      hold_cache->valid = true;
    }

    // pass iff first_sample_time is strictly past the largest colliding
    // sample (mirrors the forward scan's abs_t <= horizon + 1e-9 inclusive
    // upper bound / first_sample_time inclusive lower bound with the same
    // 1e-9 tolerance).
    if (hold_cache->empty || first_sample_time > hold_cache->max_colliding_sample + 1e-9)
    {
      return {true, "Valid", "", 0.0};
    }
    return {false, hold_cache->reason, hold_cache->entity_name, first_sample_time};
  }

  for (double abs_t = first_sample_time; abs_t <= horizon + 1e-9; abs_t += dt)
  {
    CollisionInfo sample = evaluate_terminal_hold_sample(
        robot, goal_pose, traj.is_transfer, traj.transferred_object,
        terminal_approach_entity, timetable, params, abs_t);
    if (!sample.is_valid)
    {
      return sample;
    }
  }

  return {true, "Valid", "", 0.0};
}

CollisionInfo check_trajectory_motion_and_terminal_hold_detailed(
    const Trajectory &traj,
    double start_time,
    TimeTable &timetable,
    const Params &params,
    EntityMeta *terminal_approach_entity,
    PlanTimingStats *plan_stats,
    TerminalHoldCache *hold_cache)
{
  const auto motion_t0 = std::chrono::steady_clock::now();
  CollisionInfo motion = check_collision_trajectory_detailed(
      traj, start_time, timetable, params, false, terminal_approach_entity);
  if (plan_stats)
  {
    plan_stats->traj_scan_wall_s +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - motion_t0)
            .count();
  }
  if (!motion.is_valid)
    return motion;

  const auto hold_t0 = std::chrono::steady_clock::now();
  CollisionInfo hold = check_terminal_hold_detailed(
      traj, start_time, timetable, params, terminal_approach_entity, hold_cache);
  if (plan_stats)
  {
    plan_stats->terminal_hold_wall_s +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - hold_t0)
            .count();
  }
  return hold;
}

CollisionInfo check_collision_trajectory_against_entity(
    const Trajectory &traj,
    double start_time,
    EntityMeta *other_entity,
    TimeTable &timetable,
    const Params &params)
{
  if (traj.waypoints.empty() || !other_entity)
    return {true, "Valid", "", start_time};

  double dt = shared_collision_check_step(params);
  double duration = traj.waypoints.back().time;

  RobotMeta *robot = dynamic_cast<RobotMeta *>(traj.entity);
  ObjectMeta *object = dynamic_cast<ObjectMeta *>(traj.transferred_object);

  if (!robot)
    return {false, "Invalid Trajectory", "", start_time};

  for (double t = 0; t <= duration + 1e-9; t += dt)
  {
    double abs_t = start_time + t;
    auto pose_tuple = interpolate_timed_path(traj.waypoints, t);
    Pose r_pose = {std::get<0>(pose_tuple), std::get<1>(pose_tuple),
                   std::get<2>(pose_tuple)};

    CollisionGeometry r_geom = setup_collision_geometry_for_type(
        r_pose, EntityType::ROBOT, robot->size, params);

    CollisionGeometry obj_geom;
    Pose obj_pose;
    const CollisionGeometry *obj_geom_ptr = nullptr;
    const Pose *obj_pose_ptr = nullptr;

    if (traj.is_transfer && object)
    {
      double offset = robot->size.front_length + object->size.rear_length;
      obj_pose = {r_pose.x + offset * std::cos(r_pose.yaw),
                  r_pose.y + offset * std::sin(r_pose.yaw), r_pose.yaw};
      obj_geom = setup_collision_geometry_for_type(
          obj_pose, EntityType::OBJECT, object->size, params);
      obj_geom_ptr = &obj_geom;
      obj_pose_ptr = &obj_pose;
    }

    std::unordered_map<EntityMeta *, Pose> other_pose_map;
    other_pose_map[other_entity] = timetable.get_pose(other_entity, abs_t);

    auto collision_result = check_multiple_entities_collision(
        r_geom, r_pose,
        obj_geom_ptr, obj_pose_ptr,
        other_pose_map,
        params,
        robot,
        object,
        nullptr);

    if (collision_result.has_collision)
    {
      if (t <= 0.3 && collision_result.colliding_entity &&
          collision_result.colliding_entity->type == EntityType::OBJECT)
      {
        auto it = other_pose_map.find(collision_result.colliding_entity);
        if (it != other_pose_map.end() &&
            is_valid_transfer_contact(robot, r_pose, it->first, it->second))
        {
          continue;
        }
      }

      return {false,
              collision_result.collision_type + " Collision",
              collision_result.colliding_entity ? collision_result.colliding_entity->name : "",
              abs_t};
    }
  }

  return {true, "Valid", "", 0.0};
}

// ==========================================
// Wait-only Start Time Search
// ==========================================

double timetable_delay_search_horizon(double earliest_start,
                                      TimeTable &timetable,
                                      double step)
{
  return std::max(earliest_start, timetable.get_max_time()) +
         std::max(step, 1e-3);
}

double find_wait_only_start_time_avoiding_entity(
    const Trajectory &traj,
    double earliest_start,
    EntityMeta *blocking_entity,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision,
    double *out_last_check_time)
{
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  double check_time = earliest_start;
  constexpr double step = 0.5;
  CollisionInfo last_collision{true, "Not evaluated", "", earliest_start};
  double last_check_time = earliest_start;

  while (check_time <=
         timetable_delay_search_horizon(earliest_start, timetable, step) +
             1e-9)
  {
    CollisionInfo col_info = check_collision_trajectory_against_entity(
        traj, check_time, blocking_entity, timetable, params);
    last_collision = col_info;
    last_check_time = check_time;

    if (col_info.is_valid)
    {
      if (out_last_collision)
        *out_last_collision = col_info;
      if (out_last_check_time)
        *out_last_check_time = check_time;
      return check_time;
    }

    check_time += step;
  }

  if (out_last_collision)
    *out_last_collision = last_collision;
  if (out_last_check_time)
    *out_last_check_time = last_check_time;
  return -1.0;
}

double find_wait_only_start_time(
    const Trajectory &traj,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision,
    double *out_last_check_time)
{
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  double check_time = earliest_start;
  constexpr double step = 0.5;
  CollisionInfo last_collision{true, "Not evaluated", "", earliest_start};
  double last_check_time = earliest_start;

  while (check_time <=
         timetable_delay_search_horizon(earliest_start, timetable, step) +
             1e-9)
  {
    CollisionInfo col_info =
        check_trajectory_motion_and_terminal_hold_detailed(
            traj, check_time, timetable, params, traj.approach_goal_entity);
    last_collision = col_info;
    last_check_time = check_time;

    if (col_info.is_valid)
    {
      if (out_last_collision)
        *out_last_collision = col_info;
      if (out_last_check_time)
        *out_last_check_time = check_time;
      return check_time;
    }

    check_time += step;
  }

  if (out_last_collision)
    *out_last_collision = last_collision;
  if (out_last_check_time)
    *out_last_check_time = last_check_time;
  return -1.0;
}

// ==========================================
// Stationary Pose Collision Checks
// ==========================================

CollisionInfo check_stationary_pose_collision_at_time(EntityMeta *entity,
                                                      const Pose &pose,
                                                      double t,
                                                      TimeTable &timetable,
                                                      const Params &params)
{
  if (!entity)
    return {false, "Invalid Entity", "", t};

  CollisionGeometry geom = setup_collision_geometry_for_type(
      pose, entity->type, entity->size, params);
  CollisionGeometry geom_bounds = setup_collision_geometry(pose, entity->size, 1.0);
  if (check_entity_bounds_collision(entity->type, pose, geom_bounds.corners, params))
  {
    return {false, "Boundary Collision", "Boundary", t};
  }

  // Swept directly off the timetable instead of building a fresh
  // get_poses(t) map (planner_opt Stage 2). NOTE (tie-break): if multiple
  // entities collide with `pose` simultaneously, which one gets reported can
  // change vs. the old fresh-map iteration order -- see
  // TimeTable::for_each_pose.
  CollisionInfo result{true, "Valid", "", t};
  bool collision_found = false;
  timetable.for_each_pose(t, [&](EntityMeta *ent, const Pose &ent_pose)
  {
    if (collision_found || ent == entity)
      return;

    auto collision = check_entity_collision(geom, pose, ent, ent_pose, params);
    if (collision.has_collision)
    {
      collision_found = true;
      result = {false,
                std::string(ent->type == EntityType::ROBOT ? "Robot Collision"
                                                           : "Object Collision"),
                ent ? ent->name : "",
                t};
    }
  });

  return result;
}

CollisionInfo check_stationary_pose_collision_at_time_ignoring(
    EntityMeta *entity,
    const Pose &pose,
    double t,
    TimeTable &timetable,
    const Params &params,
    EntityMeta *ignored_entity)
{
  if (!entity)
    return {false, "Invalid Entity", "", t};

  CollisionGeometry geom = setup_collision_geometry_for_type(
      pose, entity->type, entity->size, params);
  CollisionGeometry geom_bounds = setup_collision_geometry(pose, entity->size, 1.0);
  if (check_entity_bounds_collision(entity->type, pose, geom_bounds.corners, params))
  {
    return {false, "Boundary Collision", "Boundary", t};
  }

  // Swept directly off the timetable instead of building a fresh
  // get_poses(t) map (planner_opt Stage 2). NOTE (tie-break): if multiple
  // entities collide with `pose` simultaneously, which one gets reported can
  // change vs. the old fresh-map iteration order -- see
  // TimeTable::for_each_pose.
  CollisionInfo result{true, "Valid", "", t};
  bool collision_found = false;
  timetable.for_each_pose(t, [&](EntityMeta *ent, const Pose &ent_pose)
  {
    if (collision_found || ent == entity || ent == ignored_entity)
      return;

    auto collision = check_entity_collision(geom, pose, ent, ent_pose, params);
    if (collision.has_collision)
    {
      collision_found = true;
      result = {false,
                std::string(ent->type == EntityType::ROBOT ? "Robot Collision"
                                                           : "Object Collision"),
                ent ? ent->name : "",
                t};
    }
  });

  return result;
}

CollisionInfo find_stationary_pose_conflict_until_last_timestamp(EntityMeta *entity,
                                                                 const Pose &pose,
                                                                 double from_t,
                                                                 TimeTable &timetable,
                                                                 const Params &params,
                                                                 double step)
{
  double max_t = timetable.get_max_time();
  for (double t = from_t; t <= max_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time(entity, pose, t, timetable, params);
    if (!info.is_valid)
      return info;
  }
  return {true, "Valid", "", from_t};
}

CollisionInfo find_stationary_pose_conflict_until_last_timestamp_ignoring(
    EntityMeta *entity,
    const Pose &pose,
    double from_t,
    TimeTable &timetable,
    const Params &params,
    double step,
    EntityMeta *ignored_entity)
{
  double max_t = timetable.get_max_time();
  for (double t = from_t; t <= max_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time_ignoring(
            entity, pose, t, timetable, params, ignored_entity);
    if (!info.is_valid)
      return info;
  }
  return {true, "Valid", "", from_t};
}

CollisionInfo find_stationary_pose_conflict_in_interval(
    EntityMeta *entity,
    const Pose &pose,
    double from_t,
    double to_t,
    TimeTable &timetable,
    const Params &params,
    double step)
{
  if (to_t <= from_t + 1e-9)
    return {true, "Valid", "", from_t};

  for (double t = from_t; t <= to_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time(entity, pose, t, timetable, params);
    if (!info.is_valid)
      return info;
  }

  return {true, "Valid", "", from_t};
}

CollisionInfo find_stationary_pose_conflict_in_interval_ignoring(
    EntityMeta *entity,
    const Pose &pose,
    double from_t,
    double to_t,
    TimeTable &timetable,
    const Params &params,
    double step,
    EntityMeta *ignored_entity)
{
  if (to_t <= from_t + 1e-9)
    return {true, "Valid", "", from_t};

  for (double t = from_t; t <= to_t + 1e-9; t += step)
  {
    CollisionInfo info =
        check_stationary_pose_collision_at_time_ignoring(
            entity, pose, t, timetable, params, ignored_entity);
    if (!info.is_valid)
      return info;
  }

  return {true, "Valid", "", from_t};
}

CollisionInfo find_robot_waiting_pose_conflict_over_interval(
    RobotMeta *robot,
    const Pose &wait_pose,
    double from_t,
    double to_t,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    EntityMeta *preferred_ignored_entity)
{
  if (!robot || to_t <= from_t + 1e-9)
    return {true, "Valid", "", from_t};

  EntityMeta *ignored_entity = preferred_ignored_entity;
  if (!ignored_entity)
  {
    std::string contact_name = find_valid_start_contact_entity(
        robot, wait_pose, from_t, timetable, entities);
    if (!contact_name.empty())
    {
      auto it = entities.find(contact_name);
      if (it != entities.end())
        ignored_entity = it->second;
    }
  }

  const double step = shared_collision_check_step(params);
  if (ignored_entity)
  {
    return find_stationary_pose_conflict_in_interval_ignoring(
        robot, wait_pose, from_t, to_t, timetable, params, step,
        ignored_entity);
  }

  return find_stationary_pose_conflict_in_interval(
      robot, wait_pose, from_t, to_t, timetable, params, step);
}

double delayed_segment_start_after_stationary_conflict(
    double current_start,
    double segment_duration,
    double conflict_time,
    double step)
{
  const double shift_from_conflict =
      conflict_time - std::max(0.0, segment_duration) + step;
  return std::max(current_start + step, shift_from_conflict);
}

// ==========================================
// Parking Hint Collision Checks
// ==========================================

bool parking_pose_conflicts_with_blocked_hint(const Pose &candidate_pose,
                                              RobotMeta *blocker,
                                              const Trajectory *blocked_traj_hint)
{
  if (!blocker || !blocked_traj_hint)
    return false;

  Corners cand_corners = get_corners(candidate_pose.x, candidate_pose.y, candidate_pose.yaw,
                                     blocker->size.front_length, blocker->size.rear_length,
                                     blocker->size.width);
  for (size_t i = 0; i < blocked_traj_hint->waypoints.size(); ++i)
  {
    const auto &wp = blocked_traj_hint->waypoints[i];
    Corners wp_corners = get_corners(wp.x, wp.y, wp.yaw,
                                     blocker->size.front_length, blocker->size.rear_length,
                                     blocker->size.width);
    if (rectangles_intersect(cand_corners, wp_corners))
    {
      return true;
    }
  }
  return false;
}

bool parking_candidate_clears_blocked_hint(
    const Trajectory *blocked_traj_hint,
    RobotMeta *blocker,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision,
    double *out_last_check_time,
    double hint_reference_time)
{
  if (!blocked_traj_hint || !blocker || blocked_traj_hint->waypoints.empty())
  {
    if (out_last_collision)
      *out_last_collision = CollisionInfo{true, "No blocked hint", "", 0.0};
    if (out_last_check_time)
      *out_last_check_time = 0.0;
    return true;
  }

  Trajectory hint = *blocked_traj_hint;
  double hint_start_time = hint.start_time;
  if (hint_start_time <= 0.0)
  {
    // hint.start_time is unassigned (segment trajectories are built with
    // start_time = -1 until scheduling succeeds, see ReloPushPath2TrajPtr).
    // Prefer the caller-supplied real scheduling-window reference time; its
    // waypoints are already relative to that window in this case. Only fall
    // back to the legacy waypoint-front-time heuristic (which assumes
    // blocked_traj_hint carries its own, possibly absolute, timestamps) when
    // no usable reference time was passed in.
    if (hint_reference_time > 0.0)
    {
      hint_start_time = hint_reference_time;
    }
    else
    {
      hint_start_time = hint.waypoints.front().time;
      if (hint_start_time > 1e-9)
      {
        make_waypoint_times_relative(hint.waypoints, hint_start_time);
      }
    }
  }

  CollisionInfo local_collision;
  double local_check_time = hint_start_time;
  double safe_start = find_wait_only_start_time_avoiding_entity(
      hint, hint_start_time, blocker, timetable, params,
      &local_collision, &local_check_time);
  if (out_last_collision)
    *out_last_collision = local_collision;
  if (out_last_check_time)
    *out_last_check_time = local_check_time;

  std::cout << "  [ClearHint] blocker=" << blocker->name
            << " hint_start=" << std::fixed << std::setprecision(2) << hint_start_time
            << " -> " << (safe_start >= 0.0 ? "CLEARS at t=" : "BLOCKED, last check t=")
            << std::fixed << std::setprecision(2)
            << (safe_start >= 0.0 ? safe_start : local_check_time)
            << std::endl;

  return safe_start >= 0.0;
}

// ==========================================
// Pre-commit Validation
// ==========================================

bool validate_pre_commit_trajectory(
    TimeTable &trial_timetable,
    const Trajectory &traj,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    const std::vector<TransferContactWindow> *transfer_windows,
    CollisionInfo *out_collision,
    std::string *out_failure_reason)
{
  const double start_time = traj.start_time;
  const double end_time = traj.waypoints.empty()
                              ? start_time
                              : start_time + traj.waypoints.back().time;
  const auto validation_windows =
      transfer_windows_with_candidate(transfer_windows, traj);
  auto verify = verify_timetable_collision_free(
      trial_timetable, entities, params, validation_windows,
      start_time, end_time);

  if (verify.is_valid)
    return true;

  if (out_collision)
  {
    std::string entity_name = verify.entity_a;
    if (traj.entity && entity_name == traj.entity->name)
      entity_name = verify.entity_b;
    if (entity_name == "Boundary" && verify.entity_b != "Boundary")
      entity_name = verify.entity_b;

    *out_collision = CollisionInfo{
        false, verify.reason, entity_name, verify.time};
  }

  std::ostringstream oss;
  oss << "pre-commit validation failed";
  if (!verify.reason.empty())
    oss << " (" << verify.reason;
  if (!verify.entity_a.empty() || !verify.entity_b.empty())
  {
    oss << " between " << verify.entity_a << " and " << verify.entity_b;
  }
  if (!verify.reason.empty())
    oss << ")";
  oss << " at t=" << std::fixed << std::setprecision(2) << verify.time;

  if (out_failure_reason)
    *out_failure_reason = oss.str();

  std::cout << "  [PreCommit] Rejected trajectory: collision at t="
            << std::fixed << std::setprecision(2) << verify.time
            << " between " << verify.entity_a
            << " and " << verify.entity_b << std::endl;

  return false;
}

// ==========================================
// Full Conflict-Resolution Scheduler
// ==========================================

bool is_soft_robot_collision(const CollisionInfo &info,
                             const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto it = entities.find(info.entity_name);
  if (it == entities.end() || !it->second)
    return false;
  return it->second->type == EntityType::ROBOT &&
         info.entity_name != "Boundary" &&
         info.reason.find("Boundary") == std::string::npos;
}

std::string find_valid_start_contact_entity(
    RobotMeta *robot,
    const Pose &start_pose,
    double start_time,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities)
{
  auto poses = timetable.get_poses(start_time);
  for (const auto &[name, entity] : entities)
  {
    if (!entity || entity->type != EntityType::OBJECT)
      continue;

    auto pose_it = poses.find(entity);
    if (pose_it == poses.end())
      continue;

    if (is_valid_transfer_contact(robot, start_pose, entity, pose_it->second))
      return name;
  }
  return "";
}

double find_safe_start_time(Trajectory *traj, double earliest_start,
                            TimeTable &timetable, const Params &params,
                            const std::unordered_map<std::string, EntityMeta *> &entities,
                            const RuntimeOptions &options,
                            double *out_wait_added,
                            CollisionInfo *out_last_collision,
                            double *out_last_check_time,
                            IdleBlockerRelocationPolicy idle_blocker_policy,
                            PlanTimingStats *plan_stats)
{
  ScopedWallTimer sched_timer(plan_stats, &PlanTimingStats::sched_wall_s);
  if (plan_stats)
    ++plan_stats->n_find_safe_start_calls;

  if (out_wait_added)
    *out_wait_added = 0.0;
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  double check_time = earliest_start;
  double step = 0.5;
  constexpr double kExtraDelayBuffer = 0.5;

  std::string last_relocated_robot = "";
  double last_relocation_time = -100.0;
  CollisionInfo last_collision;
  double last_check_time = earliest_start;
  std::unordered_map<std::string, int> relocate_attempt_count;
  bool tried_boundary_projection = false;

  // Terminal-hold memo (planner_opt Stage 2): built lazily on first use
  // below. The candidate loop's terminal pose (traj->waypoints.back()) and
  // `timetable` are fixed for the whole loop *except* at the two explicit
  // hold_cache.invalidate() call sites below (a successful blocker
  // relocation committing to `timetable`, and the one-shot boundary
  // projection mutating `traj->waypoints`) -- both force a lazy rebuild on
  // the next hold evaluation.
  TerminalHoldCache hold_cache;

  auto write_failure_outputs = [&]()
  {
    if (out_last_collision)
      *out_last_collision = last_collision;
    if (out_last_check_time)
      *out_last_check_time = last_check_time;
  };

  while (check_time <=
         timetable_delay_search_horizon(earliest_start, timetable, step) +
             1e-9)
  {
    if (plan_stats)
      ++plan_stats->n_start_candidates_tried;

    CollisionInfo col_info =
        check_trajectory_motion_and_terminal_hold_detailed(
            *traj, check_time, timetable, params, traj->approach_goal_entity,
            plan_stats, &hold_cache);
    last_collision = col_info;
    last_check_time = check_time;

    if (col_info.is_valid)
    {
      double waited = std::max(0.0, check_time - earliest_start);

      if (waited > 1e-9)
      {
        const double buffered_start = check_time + kExtraDelayBuffer;
        CollisionInfo buffered_info =
            check_trajectory_motion_and_terminal_hold_detailed(
                *traj, buffered_start, timetable, params,
                traj->approach_goal_entity, plan_stats, &hold_cache);
        if (buffered_info.is_valid)
        {
          check_time = buffered_start;
          waited = std::max(0.0, check_time - earliest_start);
          col_info = buffered_info;
        }
      }

      if (out_wait_added)
        *out_wait_added = waited;
      if (out_last_collision)
        *out_last_collision = col_info;
      if (out_last_check_time)
        *out_last_check_time = check_time;
      if (waited > 1e-9)
        std::cout << "  [Delay] Delayed " << waited << "s for safety." << std::endl;
      return check_time;
    }

    const double traj_duration =
        traj->waypoints.empty() ? 0.0 : std::max(0.0, traj->waypoints.back().time);
    const bool terminal_hold_conflict =
        !traj->is_transfer &&
        col_info.time >=
            check_time + traj_duration -
                std::max(shared_collision_check_step(params), 1e-3) - 1e-6;
    if (terminal_hold_conflict)
    {
      check_time += step;
      continue;
    }

    // Handle Collision
    EntityMeta *collider = nullptr;
    if (entities.count(col_info.entity_name))
    {
      collider = entities.at(col_info.entity_name);
    }

    RobotMeta *resolvable_robot = find_resolvable_robot_blocker(
        col_info, timetable, entities);

    if (resolvable_robot)
    {
      RobotMeta *blocker = resolvable_robot;
      const bool blocker_is_waiting = timetable.is_waiting(blocker, col_info.time);
      const bool blocker_static_after =
          timetable.is_entity_static_after(col_info.time, blocker);
      if (blocker_is_waiting || blocker_static_after)
      {
        bool should_relocate_idle_blocker =
            idle_blocker_policy == IdleBlockerRelocationPolicy::RelocateAnyIdle;

        if (idle_blocker_policy ==
            IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt)
        {
          const bool blocker_waiting_at_attempt_start =
              timetable.is_waiting(blocker, earliest_start);
          const bool blocker_static_after_attempt_start =
              timetable.is_entity_static_after(earliest_start, blocker);
          const bool blocker_was_scheduled_to_move_at_attempt_start =
              !blocker_waiting_at_attempt_start &&
              !blocker_static_after_attempt_start;

          if (blocker_was_scheduled_to_move_at_attempt_start)
          {
            should_relocate_idle_blocker = true;
            std::cout << "  [Delay] " << blocker->name
                      << " was higher-priority traffic at candidate start t="
                      << std::fixed << std::setprecision(2) << earliest_start
                      << "s and became idle/static on "
                      << (traj && traj->entity ? traj->entity->name : "the trajectory")
                      << "'s corridor at t=" << col_info.time
                      << "s." << std::endl;
          }
        }

        if (!should_relocate_idle_blocker)
        {
          check_time += step;
          continue;
        }

        if (++relocate_attempt_count[blocker->name] > 3)
        {
          check_time += step;
          continue;
        }

        // Blocker is stationary/idle. Move it.
        if (blocker->name != last_relocated_robot || (check_time - last_relocation_time > 5.0))
        {
          if (idle_blocker_policy ==
              IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt)
          {
            std::cout << "  [Delay] Safe-parking became-idle blocker "
                      << blocker->name
                      << " before retrying "
                      << (traj && traj->entity ? traj->entity->name : "the trajectory")
                      << "'s initial-transit schedule." << std::endl;
          }

          if (relocate_blocking_robot(blocker, timetable, params, entities,
                                      options, traj, check_time, "blocker",
                                      plan_stats))
          {
            // relocate_blocking_robot committed a new trajectory into
            // `timetable`, invalidating any cached terminal-hold verdicts.
            hold_cache.invalidate();
            last_relocated_robot = blocker->name;
            last_relocation_time = check_time;
            if (idle_blocker_policy ==
                IdleBlockerRelocationPolicy::RelocateIfBecameIdleDuringAttempt)
            {
              std::cout << "  [Delay] Retrying initial-transit scheduling after parking "
                        << blocker->name << "." << std::endl;
            }
            // Retry this time step (decrement so next loop increment checks same time)
            check_time -= step;
          }
        }
      }
      else
      {
        // Dynamic blocker is still moving. Keep waiting and re-evaluate.
      }
    }
    else if (collider && collider->type == EntityType::OBJECT)
    {
      bool object_will_move = !timetable.is_entity_static_after(col_info.time, collider);
      if (!object_will_move)
      {
        // Static object conflict cannot be solved by waiting.
        std::cerr << "  [Error] Static object " << collider->name
                  << " blocks path at t=" << std::fixed << std::setprecision(2)
                  << col_info.time << ". Waiting cannot resolve." << std::endl;
        write_failure_outputs();
        return -1.0;
      }
    }
    else if (col_info.entity_name == "Boundary" ||
             col_info.reason.find("Boundary") != std::string::npos)
    {
      if (!tried_boundary_projection)
      {
        tried_boundary_projection = true;
        RobotMeta *traj_robot = dynamic_cast<RobotMeta *>(traj->entity);
        if (traj_robot && !traj->waypoints.empty())
        {
          auto projected = traj->waypoints;
          project_waypoints_inside_bounds(projected, traj_robot, params,
                                          traj->transferred_object);

          double max_shift = 0.0;
          for (size_t k = 0; k < projected.size(); ++k)
          {
            double dx = projected[k].x - traj->waypoints[k].x;
            double dy = projected[k].y - traj->waypoints[k].y;
            max_shift = std::max(max_shift, std::hypot(dx, dy));
          }

          if (max_shift > 1e-6)
          {
            traj->waypoints = std::move(projected);
            traj->CalcualteTimeStamps(traj_robot);
            // The candidate trajectory's terminal pose/duration just
            // changed, invalidating any cached terminal-hold verdicts (which
            // were computed against the old goal pose).
            hold_cache.invalidate();
            std::cout << "  [Adjust] Projected segment path inside bounds and retrying." << std::endl;
            continue;
          }
        }
      }

      std::cerr << "  [Error] Trajectory collides with boundary at t="
                << std::fixed << std::setprecision(2) << col_info.time
                << ". Waiting cannot resolve." << std::endl;
      write_failure_outputs();
      return -1.0;
    }

    check_time += step;
  }

  std::cerr << "  [Error] Could not find safe slot through timetable horizon t="
            << std::fixed << std::setprecision(2)
            << timetable_delay_search_horizon(earliest_start, timetable, step)
            << "s (waited "
            << std::max(0.0, last_check_time - earliest_start)
            << "s). Last collision: "
            << last_collision.reason << " with " << last_collision.entity_name
            << " at t=" << last_collision.time << std::endl;
  {
    auto collider_it = entities.find(last_collision.entity_name);
    const char *collider_kind =
        (collider_it != entities.end() && collider_it->second)
            ? (collider_it->second->type == EntityType::ROBOT ? "robot" : "object")
            : "unknown";
    std::cerr << "  [Delay] FAILED to find safe start: earliest_start="
              << std::fixed << std::setprecision(2) << earliest_start
              << "s, horizon=[" << earliest_start << ".."
              << timetable_delay_search_horizon(earliest_start, timetable, step)
              << "], last blocker=" << last_collision.entity_name
              << " (" << collider_kind << ")";
    if (!last_relocated_robot.empty() &&
        last_collision.entity_name == last_relocated_robot)
    {
      std::cerr << " [SAME robot just relocated at check_time="
                << last_relocation_time << "]";
    }
    std::cerr << std::endl;
  }
  write_failure_outputs();
  return -1.0; // Failure signal
}

// ==========================================
// Reserve and Commit Trajectory
// ==========================================

bool reserve_and_commit_trajectory(
    Trajectory *traj,
    RobotMeta *robot,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const RuntimeOptions &options,
    std::vector<TransferContactWindow> *transfer_windows,
    TaskExecutionStats *stats,
    std::string *out_failure_reason,
    CollisionInfo *out_last_collision,
    double *out_last_check_time,
    PlanTimingStats *plan_stats)
{
  if (out_last_collision)
    *out_last_collision = CollisionInfo{true, "Not evaluated", "", earliest_start};
  if (out_last_check_time)
    *out_last_check_time = earliest_start;

  traj->kind = traj->is_transfer ? TrajectoryKind::TRANSFER : traj->kind;

  if (traj->is_transfer && traj->transferred_object)
  {
    constexpr double kTransferReservationStep = 0.5;

    double candidate_earliest_start = earliest_start;
    std::string transfer_failure_reason =
        "failed to reserve parked transfer object against higher-priority traffic";

    while (candidate_earliest_start <=
           timetable_delay_search_horizon(earliest_start, timetable,
                                          kTransferReservationStep) +
               1e-9)
    {
      TimeTable trial_timetable = timetable;

      double wait_added = 0.0;
      CollisionInfo last_schedule_collision;
      double last_schedule_check_time = candidate_earliest_start;
      double safe_start_time =
          find_safe_start_time(traj, candidate_earliest_start,
                               trial_timetable, params, entities, options,
                               &wait_added, &last_schedule_collision,
                               &last_schedule_check_time,
                               IdleBlockerRelocationPolicy::RelocateAnyIdle,
                               plan_stats);
      if (safe_start_time < 0.0)
      {
        if (out_last_collision)
          *out_last_collision = last_schedule_collision;
        if (out_last_check_time)
          *out_last_check_time = last_schedule_check_time;

        if (!last_schedule_collision.reason.empty() &&
            last_schedule_collision.reason != "Not evaluated")
        {
          std::ostringstream oss;
          oss << "failed to find safe start for transfer segment";
          if (!last_schedule_collision.reason.empty())
          {
            oss << " (" << last_schedule_collision.reason;
            if (!last_schedule_collision.entity_name.empty())
              oss << " with " << last_schedule_collision.entity_name;
            if (last_schedule_collision.time > 1e-6)
              oss << " at t=" << std::fixed << std::setprecision(2)
                  << last_schedule_collision.time;
            oss << ")";
          }
          transfer_failure_reason = oss.str();
        }
        break;
      }

      traj->start_time = safe_start_time;
      traj->CalcualteTimeStamps(robot);
      trial_timetable.add_trajectory(*traj);

      const double segment_duration =
          traj->waypoints.empty() ? 0.0 : traj->waypoints.back().time;
      const double arrival_time = safe_start_time + segment_duration;
      const Pose parked_pose =
          trial_timetable.get_pose(traj->transferred_object, arrival_time);

      bool retry_with_delayed_start = false;
      while (true)
      {
        CollisionInfo park_conflict =
            find_stationary_pose_conflict_until_last_timestamp_ignoring(
                traj->transferred_object, parked_pose, arrival_time,
                trial_timetable, params, shared_collision_check_step(params),
                robot);
        if (park_conflict.is_valid)
        {
          CollisionInfo precommit_collision;
          if (!validate_pre_commit_trajectory(
                  trial_timetable, *traj, entities, params, transfer_windows,
                  &precommit_collision, &transfer_failure_reason))
          {
            if (out_last_collision)
              *out_last_collision = precommit_collision;
            if (out_last_check_time)
              *out_last_check_time = precommit_collision.time;
            break;
          }

          accumulate_wait_stats(stats,
                                std::max(0.0, safe_start_time - earliest_start));
          if (out_last_collision)
            *out_last_collision = park_conflict;
          if (out_last_check_time)
            *out_last_check_time = safe_start_time;
          timetable = std::move(trial_timetable);
          append_transfer_window_if_needed(*traj, transfer_windows);
          return true;
        }

        if (out_last_collision)
          *out_last_collision = park_conflict;
        if (out_last_check_time)
          *out_last_check_time = park_conflict.time;

        EntityMeta *collider = nullptr;
        auto collider_it = entities.find(park_conflict.entity_name);
        if (collider_it != entities.end())
          collider = collider_it->second;

        if (auto *blocking_robot = dynamic_cast<RobotMeta *>(collider))
        {
          const bool blocker_waiting =
              trial_timetable.is_waiting(blocking_robot, park_conflict.time);
          const bool blocker_static_after =
              trial_timetable.is_entity_static_after(park_conflict.time,
                                                     blocking_robot);
          if (blocker_waiting || blocker_static_after)
          {
            std::cout << "  [Segment] Parked "
                      << traj->transferred_object->name
                      << " conflicts with idle robot " << blocking_robot->name
                      << ". Relocating blocker first." << std::endl;
            if (relocate_blocking_robot(blocking_robot, trial_timetable, params,
                                        entities, options, nullptr, arrival_time,
                                        "blocker", plan_stats))
            {
              continue;
            }

            std::ostringstream oss;
            oss << "failed to relocate idle robot " << blocking_robot->name
                << " blocking parked " << traj->transferred_object->name;
            transfer_failure_reason = oss.str();
            break;
          }

          const double delayed_start =
              delayed_segment_start_after_stationary_conflict(
                  safe_start_time, segment_duration, park_conflict.time,
                  kTransferReservationStep);
          std::cout << "  [Segment] Delaying transfer start by "
                    << std::fixed << std::setprecision(2)
                    << (delayed_start - safe_start_time)
                    << "s so parked " << traj->transferred_object->name
                    << " clears higher-priority " << blocking_robot->name
                    << " at t=" << park_conflict.time << "s." << std::endl;
          candidate_earliest_start = delayed_start;
          retry_with_delayed_start = true;
          break;
        }

        if (collider && collider->type == EntityType::OBJECT)
        {
          const bool collider_static_after =
              trial_timetable.is_entity_static_after(park_conflict.time,
                                                     collider);
          if (!collider_static_after)
          {
            const double delayed_start =
                delayed_segment_start_after_stationary_conflict(
                    safe_start_time, segment_duration, park_conflict.time,
                    kTransferReservationStep);
            std::cout << "  [Segment] Delaying transfer start by "
                      << std::fixed << std::setprecision(2)
                      << (delayed_start - safe_start_time)
                      << "s so parked " << traj->transferred_object->name
                      << " clears higher-priority object " << collider->name
                      << " at t=" << park_conflict.time << "s." << std::endl;
            candidate_earliest_start = delayed_start;
            retry_with_delayed_start = true;
            break;
          }

          std::ostringstream oss;
          oss << "parked " << traj->transferred_object->name
              << " conflicts with reserved object " << collider->name;
          transfer_failure_reason = oss.str();
          break;
        }

        if (park_conflict.entity_name == "Boundary" ||
            park_conflict.reason.find("Boundary") != std::string::npos)
        {
          transfer_failure_reason =
              "parked transfer object would end up out of bounds";
        }
        else if (!park_conflict.entity_name.empty())
        {
          std::ostringstream oss;
          oss << "parked " << traj->transferred_object->name
              << " conflicts with reserved entity "
              << park_conflict.entity_name;
          transfer_failure_reason = oss.str();
        }
        break;
      }

      if (retry_with_delayed_start)
        continue;

      break;
    }

    if (out_failure_reason)
      *out_failure_reason = transfer_failure_reason;
    std::cerr << "  [Segment] Failed to reserve parked transfer object safely."
              << std::endl;
    return false;
  }

  double wait_added = 0.0;
  TimeTable trial_timetable = timetable;
  double safe_start_time =
      find_safe_start_time(traj, earliest_start, trial_timetable, params, entities,
                           options, &wait_added, out_last_collision,
                           out_last_check_time,
                           IdleBlockerRelocationPolicy::RelocateAnyIdle,
                           plan_stats);
  accumulate_wait_stats(stats, wait_added);

  if (safe_start_time < 0.0)
  {
    if (out_failure_reason)
      *out_failure_reason = "failed to find safe start for scheduled segment";
    std::cerr << "  [Segment] Failed to find safe start time for path segment"
              << " (blocker=" << (out_last_collision ? out_last_collision->entity_name : "?")
              << " at t=" << std::fixed << std::setprecision(2)
              << (out_last_collision ? out_last_collision->time : -1.0)
              << ")." << std::endl;
    return false;
  }

  traj->start_time = safe_start_time;
  trial_timetable.add_trajectory(*traj);

  CollisionInfo precommit_collision;
  std::string precommit_failure_reason;
  if (!validate_pre_commit_trajectory(
          trial_timetable, *traj, entities, params, transfer_windows,
          &precommit_collision, &precommit_failure_reason))
  {
    if (out_last_collision)
      *out_last_collision = precommit_collision;
    if (out_last_check_time)
      *out_last_check_time = precommit_collision.time;
    if (out_failure_reason)
      *out_failure_reason = precommit_failure_reason;
    return false;
  }

  timetable = std::move(trial_timetable);
  append_transfer_window_if_needed(*traj, transfer_windows);
  return true;
}

// ==========================================
// Contact Validation Functions (Extracted)
// ==========================================

bool is_valid_transfer_contact(EntityMeta *e1, const Pose &p1,
                               EntityMeta *e2, const Pose &p2)
{
  RobotMeta *robot = dynamic_cast<RobotMeta *>(e1->type == EntityType::ROBOT ? e1 : e2);
  ObjectMeta *obj = dynamic_cast<ObjectMeta *>(e1->type == EntityType::OBJECT ? e1 : e2);
  if (!robot || !obj)
    return false;

  const Pose &r_pose = (e1 == robot) ? p1 : p2;
  const Pose &o_pose = (e1 == obj) ? p1 : p2;
  double dx = o_pose.x - r_pose.x;
  double dy = o_pose.y - r_pose.y;

  double fx = std::cos(r_pose.yaw);
  double fy = std::sin(r_pose.yaw);
  double forward = dx * fx + dy * fy;
  double lateral = -dx * fy + dy * fx;

  double expected = robot->size.front_length + obj->size.rear_length;
  if (std::abs(forward - expected) > 0.2)
    return false;
  if (std::abs(lateral) > 0.2)
    return false;

  return true;
}

bool is_valid_terminal_approach_contact(RobotMeta *robot,
                                        const Pose &robot_pose,
                                        const Pose &goal_pose,
                                        EntityMeta *collider,
                                        const Pose &collider_pose,
                                        EntityMeta *terminal_approach_entity,
                                        const Params &params)
{
  if (!robot || !terminal_approach_entity || collider != terminal_approach_entity ||
      !collider || collider->type != EntityType::OBJECT)
  {
    return false;
  }

  const double goal_dist =
      std::hypot(robot_pose.x - goal_pose.x, robot_pose.y - goal_pose.y);
  const double goal_yaw =
      std::abs(pi_2_pi(robot_pose.yaw - goal_pose.yaw));
  const double dist_tol = std::max(0.05, params.xy_resolution);
  const double yaw_tol = std::max(0.20, params.yaw_resolution);
  if (goal_dist > dist_tol || goal_yaw > yaw_tol)
    return false;

  return is_valid_transfer_contact(robot, robot_pose, collider, collider_pose);
}

bool has_active_transfer_contact(EntityMeta *e1, EntityMeta *e2, double t,
                                 const std::vector<TransferContactWindow> &windows)
{
  for (const auto &w : windows)
  {
    if (!w.robot || !w.object)
      continue;
    bool same_pair = (e1 == w.robot && e2 == w.object) ||
                     (e2 == w.robot && e1 == w.object);
    if (!same_pair)
      continue;
    if (w.start_time - 1e-6 <= t && t <= w.end_time + 1e-6)
      return true;
  }
  return false;
}

bool has_transfer_pair(EntityMeta *e1, EntityMeta *e2,
                       const std::vector<TransferContactWindow> &windows)
{
  for (const auto &w : windows)
  {
    if (!w.robot || !w.object)
      continue;
    bool same_pair = (e1 == w.robot && e2 == w.object) ||
                     (e2 == w.robot && e1 == w.object);
    if (same_pair)
      return true;
  }
  return false;
}

RobotMeta *find_robot_transferring_object_at_time(
    EntityMeta *object,
    double t,
    TimeTable &timetable)
{
  if (!object)
    return nullptr;

  auto poses = timetable.get_poses(t);
  auto object_it = poses.find(object);
  if (object_it == poses.end())
    return nullptr;

  for (const auto &[ent, ent_pose] : poses)
  {
    if (!ent || ent->type != EntityType::ROBOT)
      continue;

    if (is_valid_transfer_contact(ent, ent_pose, object, object_it->second))
    {
      return dynamic_cast<RobotMeta *>(ent);
    }
  }

  return nullptr;
}
