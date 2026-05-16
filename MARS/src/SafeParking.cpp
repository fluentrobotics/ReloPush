/*****************************************************************
 * Safe Parking Candidate Generation
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <SafeParking.h>
#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <PHAstar/Entities.h>
#include <PHAstar/Utils.h>
#include <PHAstar/Reeds_Shepp.h>
#include <PHAstar/CollisionUtils.h>
#include <PHAstar/TimeTable.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/Visualization.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <unordered_set>
#include <vector>

// Forward declarations for functions still in PHAstar_push_demo.cpp
double find_wait_only_start_time(
    const Trajectory &traj,
    double earliest_start,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

double find_wait_only_start_time_avoiding_entity(
    const Trajectory &traj,
    double earliest_start,
    EntityMeta *blocking_entity,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

bool parking_pose_conflicts_with_blocked_hint(const Pose &candidate_pose,
                                              RobotMeta *blocker,
                                              const Trajectory *blocked_traj_hint);

bool parking_candidate_clears_blocked_hint(
    const Trajectory *blocked_traj_hint,
    RobotMeta *blocker,
    TimeTable &timetable,
    const Params &params,
    CollisionInfo *out_last_collision = nullptr,
    double *out_last_check_time = nullptr);

CollisionInfo find_stationary_pose_conflict_until_last_timestamp(EntityMeta *entity,
                                                                 const Pose &pose,
                                                                 double from_t,
                                                                 TimeTable &timetable,
                                                                 const Params &params,
                                                                 double step = 0.5);

void shift_waypoint_times(std::vector<Waypoint> &waypoints, double delta);
void make_waypoint_times_relative(std::vector<Waypoint> &waypoints, double reference_time);
void visualize_safe_parking_debug(const TimeTable &timetable, RobotMeta *blocker,
                                   double ready_time, const Pose &start_pose,
                                   const std::vector<SafeParkingDebugTrial> &debug_trials,
                                   const Params &params);

// --------------- Thread-local parking state ---------------
namespace {
  thread_local std::mt19937 g_parking_rng;
  thread_local bool g_parking_rng_initialized = false;
  thread_local std::uint32_t g_parking_rng_seed = 0;
}

thread_local ParkingCandidateMode g_parking_candidate_mode = ParkingCandidateMode::EXPAND;

const char *parking_candidate_mode_name(ParkingCandidateMode mode)
{
  switch (mode)
  {
  case ParkingCandidateMode::REVERSE_RECENT:
    return "reverse-recent-path";
  case ParkingCandidateMode::REVERSE_RECENT_SHORTER:
    return "reverse-recent-shorter";
  case ParkingCandidateMode::CONNECTED_VFH:
    return "connected-vfh";
  case ParkingCandidateMode::CONNECTED:
    return "connected-primitives";
  case ParkingCandidateMode::EXPAND:
    return "expand-primitives";
  case ParkingCandidateMode::RANDOM:
  default:
    return "randomized-candidates";
  }
}

const char *parking_candidate_mode_name()
{
  return parking_candidate_mode_name(g_parking_candidate_mode);
}

void initialize_parking_rng(bool has_fixed_seed, std::uint32_t fixed_seed)
{
  if (has_fixed_seed)
  {
    g_parking_rng_seed = fixed_seed;
  }
  else
  {
    std::random_device rd;
    g_parking_rng_seed = rd();
  }

  g_parking_rng.seed(g_parking_rng_seed);
  g_parking_rng_initialized = true;
}

std::mt19937 &parking_rng()
{
  if (!g_parking_rng_initialized)
  {
    initialize_parking_rng(false, 0);
  }
  return g_parking_rng;
}

std::uint32_t parking_rng_seed()
{
  if (!g_parking_rng_initialized)
  {
    initialize_parking_rng(false, 0);
  }
  return g_parking_rng_seed;
}

const double INF = std::numeric_limits<double>::infinity();

std::vector<ParkingCandidate>
generate_parking_candidates_randomized(const Pose &current_pose,
                                       RobotMeta *robot,
                                       const Params &params)
{
  std::vector<ParkingCandidate> candidates;

  double maxc = 1.0 / std::max(robot->transit_turning_radius(), 1e-6);
  double step_size = 0.2; // Can be coarse; only used for discretization (we
                          // ignore the full path)
  double wb = robot->wheel_base;

  auto compute_rs_length = [&](const Pose &goal) -> double
  {
    auto [xs, ys, yaws, ctypes, lengths, steers, directions] =
        ReedShepp::reeds_shepp_path_planning(current_pose.x, current_pose.y,
                                             current_pose.yaw, goal.x, goal.y,
                                             goal.yaw, maxc, step_size, wb);

    if (xs.empty())
    {
      return INF;
    }
    double L = 0.0;
    for (double len : lengths)
    {
      L += std::abs(len);
    }
    return L;
  };

  // 1. Strongly prefer the robot's initial_pose (often designed to be
  // safe/clear)
  double init_L = compute_rs_length(robot->initial_pose);
  candidates.push_back({robot->initial_pose, init_L});

  // 2. Workspace corners with multiple orientations (out-of-the-way locations)
  double margin = 0.6; // Safe margin from exact bounds
  std::vector<double> corner_yaws = {0.0, M_PI / 2, M_PI, -M_PI / 2};
  std::vector<std::pair<double, double>> corner_positions = {
      {params.min_x + margin, params.min_y + margin},
      {params.max_x - margin, params.min_y + margin},
      {params.max_x - margin, params.max_y - margin},
      {params.min_x + margin, params.max_y - margin}};

  for (const auto &pos : corner_positions)
  {
    for (double yaw : corner_yaws)
    {
      Pose p{pos.first, pos.second, yaw};
      double L = compute_rs_length(p);
      candidates.push_back({p, L});
    }
  }

  // 2.5. Nearby ring samples (prefer short clear-away moves)
  std::vector<double> ring_r = {0.6, 0.9, 1.2};
  std::vector<double> ring_a = {0.0, M_PI / 4, M_PI / 2, 3 * M_PI / 4,
                                M_PI, -3 * M_PI / 4, -M_PI / 2, -M_PI / 4};
  for (double r : ring_r)
  {
    for (double a : ring_a)
    {
      Pose p{current_pose.x + r * std::cos(a),
             current_pose.y + r * std::sin(a),
             a};
      if (p.x < params.min_x + margin || p.x > params.max_x - margin ||
          p.y < params.min_y + margin || p.y > params.max_y - margin)
      {
        continue;
      }
      double L = compute_rs_length(p);
      candidates.push_back({p, L});
    }
  }

  // 3. Random samples as fallback (20-30 is plenty)
  auto &gen = parking_rng();
  std::uniform_real_distribution<> x_dist(params.min_x + margin,
                                          params.max_x - margin);
  std::uniform_real_distribution<> y_dist(params.min_y + margin,
                                          params.max_y - margin);
  std::uniform_real_distribution<> yaw_dist(-M_PI, M_PI);

  for (int i = 0; i < 25; ++i)
  {
    Pose p{x_dist(gen), y_dist(gen), yaw_dist(gen)};
    double L = compute_rs_length(p);
    candidates.push_back({p, L});
  }

  // Sort by Reeds-Shepp length (lower is better). INF goes to the end.
  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b)
            {
              constexpr double eps = 1e-6;
              if (std::abs(a.estimated_rs_length - b.estimated_rs_length) > eps)
              {
                return a.estimated_rs_length < b.estimated_rs_length;
              }
              if (std::abs(a.pose.x - b.pose.x) > eps)
                return a.pose.x < b.pose.x;
              if (std::abs(a.pose.y - b.pose.y) > eps)
                return a.pose.y < b.pose.y;
              return a.pose.yaw < b.pose.yaw;
            });
  return candidates;
}

Pose propagate_pose_with_primitive(const Pose &pose,
                                   int direction,
                                   double steer,
                                   double speed,
                                   double wheel_base,
                                   double dt)
{
  Pose out = pose;
  double distance = static_cast<double>(direction) * speed * dt;
  if (std::abs(distance) < 1e-9)
  {
    return out;
  }

  double curvature = std::tan(steer) / std::max(wheel_base, 1e-6);
  if (std::abs(curvature) < 1e-9)
  {
    out.x += distance * std::cos(pose.yaw);
    out.y += distance * std::sin(pose.yaw);
    out.yaw = mod2pi(pose.yaw);
    return out;
  }

  double yaw_delta = distance * curvature;
  double radius = 1.0 / curvature;
  out.x += radius * (std::sin(pose.yaw + yaw_delta) - std::sin(pose.yaw));
  out.y += -radius * (std::cos(pose.yaw + yaw_delta) - std::cos(pose.yaw));
  out.yaw = mod2pi(pose.yaw + yaw_delta);
  return out;
}

std::vector<ParkingCandidate>
generate_parking_candidates_expand_primitives(const Pose &current_pose,
                                              RobotMeta *robot,
                                              const Params &params)
{
  std::vector<ParkingCandidate> candidates;
  if (!robot)
    return candidates;

  constexpr double kDeltaT = 0.5;
  constexpr int kMaxPrimitiveDepth = 12;
  constexpr size_t kMaxCandidates = 300;
  constexpr double kMinParkingDisplacement = 0.25;

  const double max_curvature =
      1.0 / std::max(robot->transit_turning_radius(), 1e-6);
  const double max_steer = std::atan(robot->wheel_base * max_curvature);
  const std::vector<std::pair<int, double>> primitives = {
      {1, 0.0},
      {1, max_steer / 2.0},
      {1, -max_steer / 2.0},
      {1, max_steer},
      {1, -max_steer},
      {-1, 0.0},
      {-1, max_steer / 2.0},
      {-1, -max_steer / 2.0},
      {-1, max_steer},
      {-1, -max_steer}};

  struct ExpansionNode
  {
    Pose pose;
    int depth = 0;
  };

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::vector<ExpansionNode> frontier;
  frontier.push_back({current_pose, 0});

  std::unordered_set<std::string> visited;
  visited.insert(pose_key(current_pose));

  for (int depth = 1; depth <= kMaxPrimitiveDepth && candidates.size() < kMaxCandidates; ++depth)
  {
    std::vector<ExpansionNode> next_frontier;
    for (const auto &node : frontier)
    {
      for (const auto &[direction, steer] : primitives)
      {
        Pose next_pose = propagate_pose_with_primitive(
            node.pose, direction, steer,
            robot->speed_transit,
            robot->wheel_base,
            kDeltaT);

        std::string key = pose_key(next_pose);
        if (visited.count(key))
          continue;
        visited.insert(key);

        CollisionGeometry geom_bounds = setup_collision_geometry(next_pose, robot->size, 1.0);
        if (check_robot_bounds_collision(next_pose, geom_bounds.corners, params))
          continue;

        next_frontier.push_back({next_pose, depth});

        double displacement = std::hypot(next_pose.x - current_pose.x,
                                         next_pose.y - current_pose.y);
        if (displacement < kMinParkingDisplacement)
          continue;

        double yaw_delta = std::atan2(std::sin(next_pose.yaw - current_pose.yaw),
                                      std::cos(next_pose.yaw - current_pose.yaw));
        double travel_cost = depth * std::max(robot->speed_transit, 1e-6) * kDeltaT;
        double estimated_cost = travel_cost + 0.15 * std::abs(yaw_delta);
        candidates.push_back({next_pose, estimated_cost});

        if (candidates.size() >= kMaxCandidates)
          break;
      }
      if (candidates.size() >= kMaxCandidates)
        break;
    }

    if (next_frontier.empty())
      break;
    frontier = std::move(next_frontier);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b)
            {
              constexpr double eps = 1e-6;
              if (std::abs(a.estimated_rs_length - b.estimated_rs_length) > eps)
              {
                return a.estimated_rs_length < b.estimated_rs_length;
              }
              if (std::abs(a.pose.x - b.pose.x) > eps)
                return a.pose.x < b.pose.x;
              if (std::abs(a.pose.y - b.pose.y) > eps)
                return a.pose.y < b.pose.y;
              return a.pose.yaw < b.pose.yaw;
            });

  return candidates;
}

std::vector<ParkingCandidate>
generate_parking_candidates_connected_primitives(
    const Pose &current_pose,
    RobotMeta *robot,
    TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params,
    double start_time)
{
  std::vector<ParkingCandidate> candidates;
  if (!robot)
    return candidates;

  constexpr int kMaxExpandIterations = 3000;
  constexpr int kMaxPrimitiveDepth = 12;
  constexpr size_t kMaxCandidates = 200;
  constexpr double kMinParkingDisplacement = 0.25;

  PHAStar planner(robot, current_pose, &timetable, &entities, params, false,
                  "", start_time, "Safe parking connected primitives");
  planner.set_ignore_other_robots(true);
  planner.set_debug_popup_enabled(false);

  using PQElem = std::tuple<double, std::uint64_t, Node *, int>;
  auto cmp = [](const PQElem &a, const PQElem &b)
  {
    return std::get<0>(a) > std::get<0>(b) ||
           (std::get<0>(a) == std::get<0>(b) &&
            std::get<1>(a) > std::get<1>(b));
  };

  std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
  std::uint64_t node_id = 0;

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::unordered_map<std::string, double> best_cost_by_pose;
  std::unordered_set<std::string> emitted_candidate_keys;
  best_cost_by_pose.emplace(pose_key(current_pose), 0.0);
  open_set.emplace(0.0, node_id++, planner.start.get(), 0);

  int iterations = 0;
  while (!open_set.empty() &&
         iterations < kMaxExpandIterations &&
         candidates.size() < kMaxCandidates)
  {
    auto [priority_cost, _, current, depth] = open_set.top();
    open_set.pop();
    ++iterations;

    if (depth > kMaxPrimitiveDepth)
      continue;

    Pose current_node_pose{current->x, current->y, current->yaw};
    std::string current_key = pose_key(current_node_pose);
    auto best_it = best_cost_by_pose.find(current_key);
    if (best_it != best_cost_by_pose.end() &&
        priority_cost > best_it->second + 1e-9)
    {
      continue;
    }

    if (depth > 0)
    {
      double displacement = std::hypot(current->x - current_pose.x,
                                       current->y - current_pose.y);
      if (displacement >= kMinParkingDisplacement &&
          emitted_candidate_keys.insert(current_key).second)
      {
        ParkingCandidate candidate;
        candidate.pose = current_node_pose;
        candidate.estimated_rs_length = current->cost;
        candidate.connected_waypoints =
            planner.extract_path(
                current, std::vector<std::tuple<double, double, double>>{});
        candidates.push_back(std::move(candidate));
      }
    }

    if (depth >= kMaxPrimitiveDepth)
      continue;

    for (const auto &prim : planner.motion_primitives)
    {
      if (prim.first == 0)
        continue;

      Node *new_node = planner.generate_node(current, prim);
      if (!new_node)
        continue;
      if (!planner.check_collision_along_path(current, new_node, prim))
        continue;

      Pose next_pose{new_node->x, new_node->y, new_node->yaw};
      std::string next_key = pose_key(next_pose);
      auto cost_it = best_cost_by_pose.find(next_key);
      if (cost_it != best_cost_by_pose.end() &&
          new_node->cost + 1e-9 >= cost_it->second)
      {
        continue;
      }

      best_cost_by_pose[next_key] = new_node->cost;
      open_set.emplace(new_node->cost, node_id++, new_node, depth + 1);
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const ParkingCandidate &a, const ParkingCandidate &b)
            {
              constexpr double eps = 1e-6;
              if (std::abs(a.estimated_rs_length - b.estimated_rs_length) > eps)
              {
                return a.estimated_rs_length < b.estimated_rs_length;
              }
              if (std::abs(a.pose.x - b.pose.x) > eps)
                return a.pose.x < b.pose.x;
              if (std::abs(a.pose.y - b.pose.y) > eps)
                return a.pose.y < b.pose.y;
              return a.pose.yaw < b.pose.yaw;
            });

  return candidates;
}

std::vector<ParkingCandidate>
generate_parking_candidates_reverse_recent_path(
    const Pose &current_pose,
    RobotMeta *robot,
    TimeTable &timetable,
    double start_time)
{
  std::vector<ParkingCandidate> candidates;
  if (!robot)
    return candidates;

  constexpr double kMinParkingDisplacement = 0.25;
  constexpr std::size_t kMaxCandidates = 200;
  constexpr std::size_t kMaxReverseStates = 400;

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  auto poses_match = [](const Pose &a, const Pose &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y) <= 1e-4 &&
           std::abs(pi_2_pi(a.yaw - b.yaw)) <= 1e-3;
  };

  const auto &database = timetable.get_database();
  auto db_it = database.find(robot);
  if (db_it == database.end() || db_it->second.size() < 2)
    return candidates;

  const double max_gap = std::max(0.75, timetable.time_increment * 1.5);
  const auto &timeline = db_it->second;
  const auto &spans = timetable.get_trajectory_spans();

  auto find_latest_span_before = [&]() -> std::pair<const TimeTable::TrajectorySpan *, std::size_t>
  {
    const TimeTable::TrajectorySpan *latest_span = nullptr;
    std::size_t latest_span_index = 0;
    for (std::size_t i = 0; i < spans.size(); ++i)
    {
      const auto &span = spans[i];
      if (span.entity != robot)
        continue;
      if (span.end_time > start_time + 1e-6)
        continue;
      if (!latest_span || span.end_time > latest_span->end_time)
      {
        latest_span = &span;
        latest_span_index = i;
      }
    }
    return {latest_span, latest_span_index};
  };

  auto find_previous_non_retraction_span =
      [&](std::size_t before_index) -> const TimeTable::TrajectorySpan *
  {
    for (std::size_t i = before_index; i-- > 0;)
    {
      if (spans[i].entity != robot)
        continue;
      if (spans[i].kind != TrajectoryKind::RETRACTION)
        return &spans[i];
    }
    return nullptr;
  };

  auto collect_span_states =
      [&](double span_start, double span_end,
          bool exclude_terminal) -> std::vector<std::pair<double, Pose>>
  {
    std::vector<std::pair<double, Pose>> states;
    double query_end = span_end + 1e-6;
    if (exclude_terminal)
      query_end = span_end - 1e-6;

    auto latest_after = timeline.upper_bound(query_end);
    if (latest_after == timeline.begin())
      return states;

    auto latest_it = std::prev(latest_after);
    if (latest_it->first < span_start - 1e-6)
      return states;

    states.reserve(std::min<std::size_t>(timeline.size(), kMaxReverseStates));
    states.push_back(*latest_it);

    auto it = latest_it;
    while (it != timeline.begin() && states.size() < kMaxReverseStates)
    {
      auto older_it = std::prev(it);
      if (older_it->first < span_start - 1e-6)
        break;
      if ((it->first - older_it->first) > max_gap)
        break;
      states.push_back(*older_it);
      it = older_it;
    }
    return states;
  };

  auto [latest_span, latest_span_index] = find_latest_span_before();
  if (!latest_span)
    return candidates;

  double source_start_time = latest_span->start_time;
  double source_end_time = latest_span->end_time;
  bool use_connected_waypoints = latest_span->kind == TrajectoryKind::TRANSIT;
  bool exclude_terminal_state = latest_span->kind != TrajectoryKind::TRANSIT;

  if (latest_span->kind == TrajectoryKind::RETRACTION)
  {
    const auto *previous_non_retraction =
        find_previous_non_retraction_span(latest_span_index);
    if (!previous_non_retraction)
      return candidates;
    source_start_time = previous_non_retraction->start_time;
    source_end_time = previous_non_retraction->end_time;
    use_connected_waypoints = false;
    exclude_terminal_state = true;
  }

  if (use_connected_waypoints)
  {
    std::size_t scan_index = latest_span_index;
    while (scan_index > 0)
    {
      const TimeTable::TrajectorySpan *previous_span = nullptr;
      std::size_t previous_index = 0;
      for (std::size_t i = scan_index; i-- > 0;)
      {
        if (spans[i].entity == robot)
        {
          previous_span = &spans[i];
          previous_index = i;
          break;
        }
      }

      if (!previous_span || previous_span->kind != TrajectoryKind::TRANSIT)
        break;
      if ((source_start_time - previous_span->end_time) > max_gap)
        break;

      source_start_time = previous_span->start_time;
      scan_index = previous_index;
    }
  }

  std::vector<std::pair<double, Pose>> source_states =
      collect_span_states(source_start_time, source_end_time,
                          exclude_terminal_state);

  if (source_states.size() < 2)
    return candidates;

  if (!use_connected_waypoints)
  {
    std::unordered_set<std::string> emitted_candidate_keys;
    for (const auto &[source_time, source_pose] : source_states)
    {
      const Pose candidate_pose{source_pose.x, source_pose.y, source_pose.yaw};
      const double displacement = std::hypot(candidate_pose.x - current_pose.x,
                                             candidate_pose.y - current_pose.y);
      if (displacement < kMinParkingDisplacement)
        continue;
      if (!emitted_candidate_keys.insert(pose_key(candidate_pose)).second)
        continue;

      ParkingCandidate candidate;
      candidate.pose = candidate_pose;
      candidate.estimated_rs_length = displacement;
      candidates.push_back(std::move(candidate));
      if (candidates.size() >= kMaxCandidates)
        break;
    }

    return candidates;
  }

  std::vector<Waypoint> reverse_path;
  reverse_path.reserve(source_states.size() + 1);

  Waypoint start_wp;
  start_wp.x = current_pose.x;
  start_wp.y = current_pose.y;
  start_wp.yaw = current_pose.yaw;
  start_wp.time = start_time;
  reverse_path.push_back(start_wp);

  std::unordered_set<std::string> emitted_candidate_keys;
  double candidate_time = start_time;
  double previous_source_time = source_states.front().first;
  Pose last_added_pose = current_pose;

  for (const auto &[source_time, source_pose] : source_states)
  {
    if (poses_match(last_added_pose, source_pose))
    {
      previous_source_time = source_time;
      continue;
    }

    const double travel_dt = std::max(1e-3, previous_source_time - source_time);
    candidate_time += travel_dt;

    Waypoint wp;
    wp.x = source_pose.x;
    wp.y = source_pose.y;
    wp.yaw = source_pose.yaw;
    wp.time = candidate_time;
    reverse_path.push_back(wp);

    previous_source_time = source_time;
    last_added_pose = source_pose;
  }

  if (reverse_path.size() < 2)
    return candidates;

  for (std::size_t i = 1; i < reverse_path.size() && candidates.size() < kMaxCandidates; ++i)
  {
    const Pose candidate_pose{reverse_path[i].x, reverse_path[i].y, reverse_path[i].yaw};
    const double displacement = std::hypot(candidate_pose.x - current_pose.x,
                                           candidate_pose.y - current_pose.y);
    if (displacement < kMinParkingDisplacement)
      continue;

    if (!emitted_candidate_keys.insert(pose_key(candidate_pose)).second)
      continue;

    ParkingCandidate candidate;
    candidate.pose = candidate_pose;
    candidate.estimated_rs_length = reverse_path[i].time - start_time;
    candidate.connected_waypoints.assign(reverse_path.begin(),
                                         reverse_path.begin() + static_cast<std::ptrdiff_t>(i + 1));
    candidates.push_back(std::move(candidate));
  }

  return candidates;
}

std::vector<ParkingCandidate>
generate_parking_candidates(const Pose &current_pose, RobotMeta *robot,
                            const Params &params,
                            TimeTable *timetable,
                            const std::unordered_map<std::string, EntityMeta *> *entities,
                            double start_time)
{
  if (g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT ||
      g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
  {
    if (!timetable)
      return {};
    return generate_parking_candidates_reverse_recent_path(current_pose, robot,
                                                           *timetable, start_time);
  }
  if (g_parking_candidate_mode == ParkingCandidateMode::CONNECTED ||
      g_parking_candidate_mode == ParkingCandidateMode::CONNECTED_VFH)
  {
    if (!timetable || !entities)
      return {};
    return generate_parking_candidates_connected_primitives(current_pose, robot,
                                                            *timetable, *entities,
                                                            params, start_time);
  }
  if (g_parking_candidate_mode == ParkingCandidateMode::EXPAND)
  {
    return generate_parking_candidates_expand_primitives(current_pose, robot,
                                                         params);
  }
  return generate_parking_candidates_randomized(current_pose, robot, params);
}

std::vector<ParkingCandidate>
generate_parking_candidates_for_mode(const Pose &current_pose,
                                     RobotMeta *robot,
                                     const Params &params,
                                     ParkingCandidateMode mode,
                                     TimeTable *timetable,
                                     const std::unordered_map<std::string, EntityMeta *> *entities,
                                     double start_time)
{
  if (mode == ParkingCandidateMode::REVERSE_RECENT ||
      mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
  {
    if (!timetable)
      return {};
    return generate_parking_candidates_reverse_recent_path(current_pose, robot,
                                                           *timetable, start_time);
  }
  if (mode == ParkingCandidateMode::CONNECTED ||
      mode == ParkingCandidateMode::CONNECTED_VFH)
  {
    if (!timetable || !entities)
      return {};
    return generate_parking_candidates_connected_primitives(current_pose, robot,
                                                            *timetable, *entities,
                                                            params, start_time);
  }
  if (mode == ParkingCandidateMode::EXPAND)
  {
    return generate_parking_candidates_expand_primitives(current_pose, robot,
                                                         params);
  }
  return generate_parking_candidates_randomized(current_pose, robot, params);
}

bool is_pose_collision_free_at_time(EntityMeta *entity, const Pose &pose,
                                    double t, TimeTable &timetable,
                                    const Params &params)
{
  if (!entity)
    return false;

  CollisionGeometry geom = setup_collision_geometry_for_type(
      pose,
      entity->type,
      entity->size,
      params);
  CollisionGeometry geom_bounds = setup_collision_geometry(pose, entity->size, 1.0);
  bool out_of_bounds = check_entity_bounds_collision(
      entity->type,
      pose,
      geom_bounds.corners,
      params);

  if (out_of_bounds)
  {
    return false;
  }

  auto poses = timetable.get_poses(t);
  for (const auto &[ent, ent_pose] : poses)
  {
    if (ent == entity)
      continue;
    auto c = check_entity_collision(geom, pose, ent, ent_pose, params);
    if (c.has_collision)
      return false;
  }
  return true;
}

bool is_parking_pose_safe_until_last_timestamp(EntityMeta *entity,
                                               const Pose &pose,
                                               double from_t,
                                               TimeTable &timetable,
                                               const Params &params,
                                               double step)
{
  double max_t = timetable.get_max_time();
  for (double t = from_t; t <= max_t + 1e-9; t += step)
  {
    if (!is_pose_collision_free_at_time(entity, pose, t, timetable, params))
      return false;
  }
  return true;
}

// ==========================================
// Parking Geometry Helpers (Extracted lines 3807-4689)
// ==========================================

double parking_entity_effective_radius(EntityMeta *ent, const Params &params)
{
  if (!ent)
    return 0.0;

  const double inflation = collision_inflation_for_type(ent->type, params);
  const double full_length =
      (ent->size.front_length + ent->size.rear_length) * inflation;
  const double width = ent->size.width * inflation;
  return 0.5 * std::sqrt(full_length * full_length + width * width);
}

double ray_distance_to_boundary(const Pose &origin, double angle, const Params &params)
{
  const double dx = std::cos(angle);
  const double dy = std::sin(angle);
  double best = std::numeric_limits<double>::infinity();

  auto update = [&](double numerator, double denom)
  {
    if (std::abs(denom) < 1e-9)
      return;
    double t = numerator / denom;
    if (t > 0.0)
      best = std::min(best, t);
  };

  update(params.min_x - origin.x, dx);
  update(params.max_x - origin.x, dx);
  update(params.min_y - origin.y, dy);
  update(params.max_y - origin.y, dy);

  return best;
}

ParkingDirectionHistogram build_parking_direction_histogram(
    RobotMeta *blocker,
    const Pose &start_pose,
    double ready_time,
    TimeTable &timetable,
    const Params &params)
{
  ParkingDirectionHistogram hist;
  constexpr int kBins = 72;
  constexpr double kEntityWeight = 1.2;
  constexpr double kBoundaryWeight = 0.8;
  constexpr double kMinClearance = 0.05;

  hist.density.assign(kBins, 0.0);
  const double angle_step = kDubinsTwoPi / static_cast<double>(kBins);
  const double blocker_radius = parking_entity_effective_radius(blocker, params);

  auto poses = timetable.get_poses(ready_time);
  for (const auto &[ent, pose] : poses)
  {
    if (!ent || ent == blocker)
      continue;

    double dx = pose.x - start_pose.x;
    double dy = pose.y - start_pose.y;
    double dist = std::hypot(dx, dy);
    if (dist < 1e-6)
    {
      for (double &v : hist.density)
        v += 10.0;
      continue;
    }

    double other_radius = parking_entity_effective_radius(ent, params);
    double blocked_radius = blocker_radius + other_radius + params.safety_margin;
    double spread = std::asin(std::min(0.999, blocked_radius / std::max(dist, blocked_radius + 1e-6)));
    spread = std::max(spread, angle_step);
    double center = std::atan2(dy, dx);
    double clearance = std::max(kMinClearance, dist - blocked_radius);
    double contribution = kEntityWeight / clearance;

    for (int bin = 0; bin < kBins; ++bin)
    {
      double bin_angle = -M_PI + (static_cast<double>(bin) + 0.5) * angle_step;
      double delta = pi_2_pi(bin_angle - center);
      double abs_delta = std::abs(delta);
      if (abs_delta > spread)
        continue;

      double taper = 1.0 - (abs_delta / spread);
      hist.density[bin] += contribution * (0.25 + 0.75 * taper);
    }
  }

  for (int bin = 0; bin < kBins; ++bin)
  {
    double bin_angle = -M_PI + (static_cast<double>(bin) + 0.5) * angle_step;
    double ray_dist = ray_distance_to_boundary(start_pose, bin_angle, params);
    if (!std::isfinite(ray_dist))
      continue;

    double clearance = std::max(kMinClearance, ray_dist - blocker_radius);
    hist.density[bin] += kBoundaryWeight / clearance;
  }

  if (!hist.density.empty())
  {
    std::vector<double> smoothed(hist.density.size(), 0.0);
    for (size_t i = 0; i < hist.density.size(); ++i)
    {
      size_t prev = (i + hist.density.size() - 1) % hist.density.size();
      size_t next = (i + 1) % hist.density.size();
      smoothed[i] = 0.2 * hist.density[prev] +
                    0.6 * hist.density[i] +
                    0.2 * hist.density[next];
    }
    hist.density = std::move(smoothed);
    hist.max_density = *std::max_element(hist.density.begin(), hist.density.end());
  }

  return hist;
}

ConnectedSafeParkingSearchResult search_safe_parking_connected_search(
    ParkingCandidateMode mode,
    RobotMeta *blocker,
    const Pose &start_pose,
    double ready_time,
    TimeTable &timetable,
    const Params &params,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Trajectory *blocked_traj_hint,
    std::vector<SafeParkingDebugTrial> *debug_trials)
{
  ConnectedSafeParkingSearchResult result;
  if (!blocker)
    return result;

  constexpr int kMaxExpandIterations = 3000;
  constexpr int kMaxPrimitiveDepth = 12;
  constexpr double kMinParkingDisplacement = 0.25;

  auto append_debug_trial = [&](const Pose &candidate_pose,
                                const PlanningResult &trial_result,
                                const TimeTable *context_timetable = nullptr)
  {
    if (!DEBUG_VIS || !debug_trials)
      return;

    SafeParkingDebugTrial trial;
    trial.trial_index = static_cast<int>(debug_trials->size()) + 1;
    trial.candidate_mode = parking_candidate_mode_name(mode);
    trial.candidate_pose = candidate_pose;
    trial.result = trial_result;
    if (blocked_traj_hint)
      trial.blocked_path_hint_waypoints = blocked_traj_hint->waypoints;
    trial.context_timetable = context_timetable ? *context_timetable : timetable;
    debug_trials->push_back(std::move(trial));
  };

  ParkingDirectionHistogram hist;
  const bool use_histogram = (mode == ParkingCandidateMode::CONNECTED_VFH);
  if (use_histogram)
  {
    hist = build_parking_direction_histogram(blocker, start_pose, ready_time,
                                             timetable, params);
  }

  const std::string debug_kind = use_histogram
                                     ? "Safe parking connected VFH search"
                                     : "Safe parking connected search";
  PHAStar planner(blocker, start_pose, &timetable, &entities, params, false,
                  "", ready_time, debug_kind);
  planner.set_ignore_other_robots(true);
  planner.set_debug_popup_enabled(false);

  auto queue_priority = [&](double g_cost, const Pose &pose)
  {
    if (!use_histogram || hist.empty())
      return g_cost;

    constexpr double kHistogramWeight = 3.0;
    double dx = pose.x - start_pose.x;
    double dy = pose.y - start_pose.y;
    double angle = (std::hypot(dx, dy) > 1e-6) ? std::atan2(dy, dx) : pose.yaw;
    return g_cost + kHistogramWeight * hist.sample_normalized(angle);
  };

  using PQElem = std::tuple<double, std::uint64_t, Node *, int>;
  auto cmp = [](const PQElem &a, const PQElem &b)
  {
    return std::get<0>(a) > std::get<0>(b) ||
           (std::get<0>(a) == std::get<0>(b) &&
            std::get<1>(a) > std::get<1>(b));
  };

  std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
  std::uint64_t node_id = 0;

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 20.0));
    int yi = static_cast<int>(std::round(p.y * 20.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::unordered_map<std::string, double> best_cost_by_pose;
  best_cost_by_pose.emplace(pose_key(start_pose), 0.0);
  open_set.emplace(queue_priority(0.0, start_pose), node_id++, planner.start.get(), 0);

  int iterations = 0;
  while (!open_set.empty() && iterations < kMaxExpandIterations)
  {
    auto [priority_cost, _, current, depth] = open_set.top();
    open_set.pop();
    ++iterations;

    if (depth > kMaxPrimitiveDepth)
      continue;

    Pose current_pose{current->x, current->y, current->yaw};
    std::string current_key = pose_key(current_pose);
    auto best_it = best_cost_by_pose.find(current_key);
    if (best_it != best_cost_by_pose.end() &&
        current->cost > best_it->second + 1e-9)
    {
      continue;
    }

    if (depth > 0)
    {
      double displacement = std::hypot(current_pose.x - start_pose.x,
                                       current_pose.y - start_pose.y);
      if (displacement >= kMinParkingDisplacement)
      {
        PlanningResult trial_res;
        trial_res.status = PlanningStatus::SUCCESS;
        trial_res.waypoints =
            planner.extract_path(
                current, std::vector<std::tuple<double, double, double>>{});
        trial_res.failure_detail = use_histogram
                                       ? "Histogram-guided connected parking candidate"
                                       : "Connected primitive parking candidate";

        if (trial_res.waypoints.empty())
        {
          trial_res.status = PlanningStatus::NO_PATH_FOUND;
          trial_res.failure_detail = "Connected search reached pose but extracted no path";
          append_debug_trial(current_pose, trial_res, &timetable);
        }
        else if (parking_pose_conflicts_with_blocked_hint(current_pose, blocker,
                                                          blocked_traj_hint))
        {
          trial_res.status = PlanningStatus::NO_PATH_FOUND;
          trial_res.failure_detail =
              "Candidate pose still intersects the blocked path hint";
          trial_res.colliding_entity = "blocked_path_hint";
          trial_res.failure_time = ready_time;
          append_debug_trial(current_pose, trial_res, &timetable);
        }
        else
        {
          Trajectory trial_traj;
          trial_traj.entity = blocker;
          trial_traj.start_time = ready_time;
          trial_traj.waypoints = trial_res.waypoints;
          for (auto &wp : trial_traj.waypoints)
            wp.time -= ready_time;

          TimeTable trial_timetable = timetable;
          double wait_added = 0.0;
          CollisionInfo last_collision;
          double last_check_time = ready_time;
          double safe_start = find_wait_only_start_time(
              trial_traj, ready_time, trial_timetable, params,
              &last_collision, &last_check_time);
          wait_added = std::max(0.0, safe_start - ready_time);

          if (safe_start < 0.0)
          {
            const double scheduled_start = std::max(ready_time, last_check_time);
            shift_waypoint_times(trial_res.waypoints, scheduled_start - ready_time);
            trial_res.status = PlanningStatus::NO_PATH_FOUND;
            std::ostringstream oss;
            oss << "Connected parking path found, but no wait-only collision-free start slot";
            if (!last_collision.reason.empty())
            {
              oss << " (last: " << last_collision.reason;
              if (!last_collision.entity_name.empty())
                oss << " with " << last_collision.entity_name;
              if (last_collision.time > 1e-6)
                oss << " at t=" << std::fixed << std::setprecision(2)
                    << last_collision.time;
              oss << ")";
            }
            trial_res.failure_detail = oss.str();
            trial_res.colliding_entity = last_collision.entity_name;
            trial_res.failure_time =
                (last_collision.time > 1e-6) ? last_collision.time : last_check_time;
            append_debug_trial(current_pose, trial_res, &trial_timetable);
          }
          else
          {
            shift_waypoint_times(trial_res.waypoints, safe_start - ready_time);
            trial_traj.start_time = safe_start;
            double arrival_time = safe_start +
                                  (trial_traj.waypoints.empty() ? 0.0
                                                                : trial_traj.waypoints.back().time);
            CollisionInfo park_conflict =
                find_stationary_pose_conflict_until_last_timestamp(
                    blocker, current_pose, arrival_time, trial_timetable, params);
            if (!park_conflict.is_valid)
            {
              trial_res.status = PlanningStatus::NO_PATH_FOUND;
              std::ostringstream oss;
              oss << "Parking pose becomes occupied before the timetable ends";
              if (!park_conflict.reason.empty())
              {
                oss << " (" << park_conflict.reason;
                if (!park_conflict.entity_name.empty())
                  oss << " with " << park_conflict.entity_name;
                if (park_conflict.time > 1e-6)
                  oss << " at t=" << std::fixed << std::setprecision(2)
                      << park_conflict.time;
                oss << ")";
              }
              trial_res.failure_detail = oss.str();
              trial_res.colliding_entity = park_conflict.entity_name;
              trial_res.failure_time = park_conflict.time;
              append_debug_trial(current_pose, trial_res, &trial_timetable);
            }
            else
            {
              trial_timetable.add_trajectory(trial_traj);

              CollisionInfo blocked_hint_collision;
              double blocked_hint_check_time = ready_time;
              if (!parking_candidate_clears_blocked_hint(
                      blocked_traj_hint, blocker, trial_timetable, params,
                      &blocked_hint_collision, &blocked_hint_check_time))
              {
                trial_res.status = PlanningStatus::NO_PATH_FOUND;
                std::ostringstream oss;
                oss << "Parking candidate is feasible, but it does not clear the blocked path";
                if (!blocked_hint_collision.reason.empty())
                {
                  oss << " (" << blocked_hint_collision.reason;
                  if (!blocked_hint_collision.entity_name.empty())
                    oss << " with " << blocked_hint_collision.entity_name;
                  if (blocked_hint_collision.time > 1e-6)
                    oss << " at t=" << std::fixed << std::setprecision(2)
                        << blocked_hint_collision.time;
                  oss << ")";
                }
                trial_res.failure_detail = oss.str();
                trial_res.colliding_entity = blocked_hint_collision.entity_name;
                trial_res.failure_time =
                    (blocked_hint_collision.time > 1e-6)
                        ? blocked_hint_collision.time
                        : blocked_hint_check_time;
                append_debug_trial(current_pose, trial_res, &trial_timetable);
                continue;
              }

              trial_res.failure_detail = "Selected safe parking candidate";
              append_debug_trial(current_pose, trial_res, &trial_timetable);

              result.found = true;
              result.parking_pose = current_pose;
              result.committed_timetable = std::move(trial_timetable);
              return result;
            }
          }
        }
      }
    }

    if (depth >= kMaxPrimitiveDepth)
      continue;

    for (const auto &prim : planner.motion_primitives)
    {
      if (prim.first == 0)
        continue;

      Node *new_node = planner.generate_node(current, prim);
      if (!new_node)
        continue;
      if (!planner.check_collision_along_path(current, new_node, prim))
        continue;

      Pose next_pose{new_node->x, new_node->y, new_node->yaw};
      std::string next_key = pose_key(next_pose);
      auto cost_it = best_cost_by_pose.find(next_key);
      if (cost_it != best_cost_by_pose.end() &&
          new_node->cost + 1e-9 >= cost_it->second)
      {
        continue;
      }

      best_cost_by_pose[next_key] = new_node->cost;
      open_set.emplace(queue_priority(new_node->cost, next_pose),
                       node_id++, new_node, depth + 1);
    }
  }

  return result;
}

bool relocate_blocking_robot(RobotMeta *blocker,
                             TimeTable &timetable,
                             const Params &params,
                             const std::unordered_map<std::string, EntityMeta *> &entities,
                             const RuntimeOptions &options,
                             const Trajectory *blocked_traj_hint)
{
  auto &recent_failed_relocations = recent_failed_relocation_cache();

  double ready_time = timetable.get_entity_max_time(blocker);
  // Add small buffer to start time to ensure no conflict with previous finish
  ready_time += 0.1;

  Pose start_pose = timetable.get_pose(blocker, ready_time);
  blocker->initial_pose = start_pose;

  auto make_fail_key = [&](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 10.0));
    int yi = static_cast<int>(std::round(p.y * 10.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 10.0));
    return blocker->name + "@" + std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  std::string fail_key = make_fail_key(start_pose);
  auto fail_it = recent_failed_relocations.find(fail_key);
  if (fail_it != recent_failed_relocations.end() && (ready_time - fail_it->second) < 10.0)
  {
    return false;
  }

  std::cout << "  [Relocate] Attempting to move " << blocker->name
            << " from (" << start_pose.x << ", " << start_pose.y << ")" << std::endl;

  TimeTable debug_timetable = timetable;
  std::vector<SafeParkingDebugTrial> debug_trials;
  auto record_debug_trial = [&](ParkingCandidateMode mode, const Pose &candidate_pose,
                                const PlanningResult &trial_result,
                                const TimeTable *context_timetable = nullptr)
  {
    if (!DEBUG_VIS)
      return;

    SafeParkingDebugTrial trial;
    trial.trial_index = static_cast<int>(debug_trials.size()) + 1;
    trial.candidate_mode = parking_candidate_mode_name(mode);
    trial.candidate_pose = candidate_pose;
    trial.result = trial_result;
    if (blocked_traj_hint)
      trial.blocked_path_hint_waypoints = blocked_traj_hint->waypoints;
    trial.context_timetable = context_timetable ? *context_timetable : timetable;
    debug_trials.push_back(std::move(trial));
  };
  auto show_debug_trials = [&]()
  {
    if (!DEBUG_VIS || debug_trials.empty())
      return;

    visualize_safe_parking_debug(debug_timetable, blocker, ready_time,
                                 start_pose, debug_trials, params);
  };

  std::unordered_set<std::string> tried_pose_keys;
  int relocation_modes_tried = 0;
  int relocation_candidates_planned = 0;
  int relocation_planning_failures = 0;
  int relocation_schedule_failures = 0;
  int relocation_safety_rejections = 0;
  std::vector<ParkingCandidateMode> candidate_modes;
  auto append_mode_once = [&](ParkingCandidateMode mode)
  {
    if (std::find(candidate_modes.begin(), candidate_modes.end(), mode) == candidate_modes.end())
    {
      candidate_modes.push_back(mode);
    }
  };
  append_mode_once(g_parking_candidate_mode);
  if (g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT ||
      g_parking_candidate_mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
  {
    append_mode_once(ParkingCandidateMode::CONNECTED_VFH);
    append_mode_once(ParkingCandidateMode::CONNECTED);
    append_mode_once(ParkingCandidateMode::EXPAND);
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else if (g_parking_candidate_mode == ParkingCandidateMode::CONNECTED_VFH)
  {
    append_mode_once(ParkingCandidateMode::CONNECTED);
    append_mode_once(ParkingCandidateMode::EXPAND);
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else if (g_parking_candidate_mode == ParkingCandidateMode::CONNECTED)
  {
    append_mode_once(ParkingCandidateMode::EXPAND);
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else if (g_parking_candidate_mode == ParkingCandidateMode::EXPAND)
  {
    append_mode_once(ParkingCandidateMode::RANDOM);
  }
  else
  {
    append_mode_once(ParkingCandidateMode::EXPAND);
  }

  auto pose_key = [](const Pose &p)
  {
    int xi = static_cast<int>(std::round(p.x * 10.0));
    int yi = static_cast<int>(std::round(p.y * 10.0));
    int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 10.0));
    return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
  };

  struct ParkingPathRefinement
  {
    bool found = false;
    Pose parking_pose;
    Trajectory trajectory;
    TimeTable committed_timetable;
    PlanningResult result;
  };

  auto try_shorten_reverse_recent_path =
      [&](const PlanningResult &scheduled_res) -> ParkingPathRefinement
  {
    ParkingPathRefinement refinement;
    if (scheduled_res.waypoints.size() < 3)
      return refinement;

    std::unordered_set<std::string> tested_prefix_poses;
    for (std::size_t i = 1; i + 1 < scheduled_res.waypoints.size(); ++i)
    {
      const auto &candidate_wp = scheduled_res.waypoints[i];
      Pose prefix_pose{candidate_wp.x, candidate_wp.y, candidate_wp.yaw};
      if (std::hypot(prefix_pose.x - start_pose.x,
                     prefix_pose.y - start_pose.y) < 0.25)
      {
        continue;
      }

      std::string key = pose_key(prefix_pose);
      if (!tested_prefix_poses.insert(key).second)
        continue;

      if (parking_pose_conflicts_with_blocked_hint(prefix_pose, blocker,
                                                   blocked_traj_hint))
      {
        continue;
      }

      std::vector<Waypoint> prefix_waypoints(
          scheduled_res.waypoints.begin(),
          scheduled_res.waypoints.begin() + static_cast<std::ptrdiff_t>(i + 1));
      const double original_abs_start = prefix_waypoints.front().time;
      make_waypoint_times_relative(prefix_waypoints, original_abs_start);

      Trajectory prefix_traj;
      prefix_traj.entity = blocker;
      prefix_traj.start_time = ready_time;
      prefix_traj.waypoints = prefix_waypoints;
      prefix_traj.is_transfer = false;
      prefix_traj.kind = TrajectoryKind::TRANSIT;
      prefix_traj.transferred_object = nullptr;
      prefix_traj.approach_goal_entity = nullptr;

      CollisionInfo last_collision;
      double last_check_time = ready_time;
      double prefix_safe_start = find_wait_only_start_time(
          prefix_traj, ready_time, timetable, params,
          &last_collision, &last_check_time);
      if (prefix_safe_start < 0.0)
        continue;

      prefix_traj.start_time = prefix_safe_start;
      const double arrival_time =
          prefix_safe_start +
          (prefix_traj.waypoints.empty() ? 0.0
                                         : prefix_traj.waypoints.back().time);

      CollisionInfo park_conflict =
          find_stationary_pose_conflict_until_last_timestamp(
              blocker, prefix_pose, arrival_time, timetable, params);
      if (!park_conflict.is_valid)
        continue;

      TimeTable prefix_timetable = timetable;
      prefix_timetable.add_trajectory(prefix_traj);

      CollisionInfo blocked_hint_collision;
      double blocked_hint_check_time = ready_time;
      if (!parking_candidate_clears_blocked_hint(
              blocked_traj_hint, blocker, prefix_timetable, params,
              &blocked_hint_collision, &blocked_hint_check_time))
      {
        continue;
      }

      PlanningResult prefix_result = scheduled_res;
      prefix_result.waypoints = prefix_waypoints;
      shift_waypoint_times(prefix_result.waypoints, prefix_safe_start);
      prefix_result.failure_detail =
          "Selected shorter reverse-recent parking candidate";
      prefix_result.colliding_entity.clear();
      prefix_result.failure_time = 0.0;

      refinement.found = true;
      refinement.parking_pose = prefix_pose;
      refinement.trajectory = std::move(prefix_traj);
      refinement.committed_timetable = std::move(prefix_timetable);
      refinement.result = std::move(prefix_result);
      return refinement;
    }

    return refinement;
  };

  for (std::size_t mode_index = 0; mode_index < candidate_modes.size(); ++mode_index)
  {
    ParkingCandidateMode candidate_mode = candidate_modes[mode_index];
    relocation_modes_tried++;
    if (mode_index > 0)
    {
      std::cout << "  [Relocate] Retrying with "
                << parking_candidate_mode_name(candidate_mode)
                << " parking candidates." << std::endl;
    }

    if (candidate_mode == ParkingCandidateMode::CONNECTED ||
        candidate_mode == ParkingCandidateMode::CONNECTED_VFH)
    {
      auto connected_result = search_safe_parking_connected_search(
          candidate_mode, blocker, start_pose, ready_time,
          timetable, params, entities, blocked_traj_hint,
          DEBUG_VIS ? &debug_trials : nullptr);
      if (connected_result.found)
      {
        show_debug_trials();
        timetable = std::move(connected_result.committed_timetable);
        recent_failed_relocations.erase(fail_key);
        std::cout << "  [Relocate] SUCCESS: Moved " << blocker->name
                  << " to (" << connected_result.parking_pose.x << ", "
                  << connected_result.parking_pose.y << ")" << std::endl;
        std::cout << "  [Relocate] Summary: modes=" << relocation_modes_tried
                  << ", planned_candidates=" << relocation_candidates_planned
                  << ", planning_failures=" << relocation_planning_failures
                  << ", schedule_failures=" << relocation_schedule_failures
                  << ", safety_rejections=" << relocation_safety_rejections
                  << std::endl;
        return true;
      }
      continue;
    }

    auto candidates =
        generate_parking_candidates_for_mode(start_pose, blocker, params,
                                             candidate_mode, &timetable,
                                             &entities, ready_time);

    for (const auto &cand : candidates)
    {
      if (!std::isfinite(cand.estimated_rs_length))
        continue;

      if (std::hypot(cand.pose.x - start_pose.x, cand.pose.y - start_pose.y) < 0.25)
        continue;

      std::string key = pose_key(cand.pose);
      if (tried_pose_keys.count(key))
        continue;
      tried_pose_keys.insert(key);

      if (parking_pose_conflicts_with_blocked_hint(cand.pose, blocker,
                                                   blocked_traj_hint))
      {
        relocation_safety_rejections++;
        PlanningResult skipped_res;
        skipped_res.status = PlanningStatus::NO_PATH_FOUND;
        skipped_res.failure_detail =
            "Candidate pose still intersects the blocked path hint";
        skipped_res.colliding_entity = "blocked_path_hint";
        skipped_res.failure_time = ready_time;
        record_debug_trial(candidate_mode, cand.pose, skipped_res);
        continue;
      }

      PlanningResult res;
      if (!cand.connected_waypoints.empty())
      {
        res.status = PlanningStatus::SUCCESS;
        res.waypoints = cand.connected_waypoints;
        res.failure_detail = "Connected primitive parking candidate";
      }
      else
      {
        relocation_candidates_planned++;
        PHAStar planner(blocker, cand.pose, &timetable, &entities, params, false,
                        "", ready_time, "Safe parking relocation");
        planner.set_ignore_other_robots(true);
        planner.set_debug_popup_enabled(false);
        planner.max_search_iterations = options.max_search_iterations;
        planner.set_planner_expansion_threads(options.planner_expansion_threads);

        res = planner.Planning_with_res(ready_time);
      }

      if (res.status != PlanningStatus::SUCCESS)
      {
        relocation_planning_failures++;
        record_debug_trial(candidate_mode, cand.pose, res);
        continue;
      }

      Trajectory relo_traj;
      relo_traj.entity = blocker;
      relo_traj.start_time = ready_time;
      relo_traj.waypoints = res.waypoints;
      relo_traj.is_transfer = false;
      relo_traj.kind = TrajectoryKind::TRANSIT;
      relo_traj.transferred_object = nullptr;
      relo_traj.approach_goal_entity = nullptr;
      for (auto &wp : relo_traj.waypoints)
        wp.time -= ready_time;

      double wait_added = 0.0;
      CollisionInfo last_collision;
      double last_check_time = ready_time;
      double safe_start = find_wait_only_start_time(
          relo_traj, ready_time, timetable, params,
          &last_collision, &last_check_time);
      wait_added = std::max(0.0, safe_start - ready_time);
      if (safe_start < 0.0)
      {
        relocation_schedule_failures++;
        PlanningResult trial_res = res;
        shift_waypoint_times(trial_res.waypoints,
                             std::max(ready_time, last_check_time) - ready_time);
        trial_res.status = PlanningStatus::NO_PATH_FOUND;
        std::ostringstream oss;
        oss << "Relocation path found, but no wait-only collision-free start slot";
        if (!last_collision.reason.empty())
        {
          oss << " (last: " << last_collision.reason;
          if (!last_collision.entity_name.empty())
            oss << " with " << last_collision.entity_name;
          if (last_collision.time > 1e-6)
            oss << " at t=" << std::fixed << std::setprecision(2)
                << last_collision.time;
          oss << ")";
        }
        trial_res.failure_detail = oss.str();
        trial_res.colliding_entity = last_collision.entity_name;
        trial_res.failure_time =
            (last_collision.time > 1e-6) ? last_collision.time : last_check_time;
        record_debug_trial(candidate_mode, cand.pose, trial_res);
        continue;
      }

      shift_waypoint_times(res.waypoints, safe_start - ready_time);
      relo_traj.start_time = safe_start;
      double arrival_time = safe_start + (relo_traj.waypoints.empty() ? 0.0 : relo_traj.waypoints.back().time);
      CollisionInfo park_conflict =
          find_stationary_pose_conflict_until_last_timestamp(
              blocker, cand.pose, arrival_time, timetable, params);
      if (!park_conflict.is_valid)
      {
        relocation_safety_rejections++;
        PlanningResult trial_res = res;
        trial_res.status = PlanningStatus::NO_PATH_FOUND;
        std::ostringstream oss;
        oss << "Parking pose becomes occupied before the timetable ends";
        if (!park_conflict.reason.empty())
        {
          oss << " (" << park_conflict.reason;
          if (!park_conflict.entity_name.empty())
            oss << " with " << park_conflict.entity_name;
          if (park_conflict.time > 1e-6)
            oss << " at t=" << std::fixed << std::setprecision(2)
                << park_conflict.time;
          oss << ")";
        }
        trial_res.failure_detail = oss.str();
        trial_res.colliding_entity = park_conflict.entity_name;
        trial_res.failure_time = park_conflict.time;
        record_debug_trial(candidate_mode, cand.pose, trial_res);
        continue;
      }

      PlanningResult success_res = res;
      Pose selected_parking_pose = cand.pose;
      TimeTable committed_timetable = timetable;
      committed_timetable.add_trajectory(relo_traj);

      CollisionInfo blocked_hint_collision;
      double blocked_hint_check_time = ready_time;
      if (!parking_candidate_clears_blocked_hint(
              blocked_traj_hint, blocker, committed_timetable, params,
              &blocked_hint_collision, &blocked_hint_check_time))
      {
        relocation_safety_rejections++;
        PlanningResult trial_res = res;
        trial_res.status = PlanningStatus::NO_PATH_FOUND;
        std::ostringstream oss;
        oss << "Parking candidate is feasible, but it does not clear the blocked path";
        if (!blocked_hint_collision.reason.empty())
        {
          oss << " (" << blocked_hint_collision.reason;
          if (!blocked_hint_collision.entity_name.empty())
            oss << " with " << blocked_hint_collision.entity_name;
          if (blocked_hint_collision.time > 1e-6)
            oss << " at t=" << std::fixed << std::setprecision(2)
                << blocked_hint_collision.time;
          oss << ")";
        }
        trial_res.failure_detail = oss.str();
        trial_res.colliding_entity = blocked_hint_collision.entity_name;
        trial_res.failure_time =
            (blocked_hint_collision.time > 1e-6)
                ? blocked_hint_collision.time
                : blocked_hint_check_time;
        record_debug_trial(candidate_mode, cand.pose, trial_res,
                           &committed_timetable);
        continue;
      }

      if (candidate_mode == ParkingCandidateMode::REVERSE_RECENT_SHORTER)
      {
        ParkingPathRefinement refinement =
            try_shorten_reverse_recent_path(res);
        if (refinement.found)
        {
          selected_parking_pose = refinement.parking_pose;
          success_res = std::move(refinement.result);
          committed_timetable = std::move(refinement.committed_timetable);
          std::cout << "  [Relocate] Shortened reverse-recent parking from ("
                    << std::fixed << std::setprecision(2)
                    << cand.pose.x << ", " << cand.pose.y
                    << ") to (" << selected_parking_pose.x << ", "
                    << selected_parking_pose.y << ")." << std::endl;
        }
      }

      if (success_res.failure_detail.empty())
      {
        success_res.failure_detail = "Selected safe parking candidate";
      }
      record_debug_trial(candidate_mode, selected_parking_pose, success_res,
                         &committed_timetable);
      show_debug_trials();

      timetable = std::move(committed_timetable);
      recent_failed_relocations.erase(fail_key);
      std::cout << "  [Relocate] SUCCESS: Moved " << blocker->name
                << " to (" << selected_parking_pose.x << ", "
                << selected_parking_pose.y << ")" << std::endl;
      std::cout << "  [Relocate] Summary: modes=" << relocation_modes_tried
                << ", planned_candidates=" << relocation_candidates_planned
                << ", planning_failures=" << relocation_planning_failures
                << ", schedule_failures=" << relocation_schedule_failures
                << ", safety_rejections=" << relocation_safety_rejections
                << std::endl;
      return true;
    }
  }
  recent_failed_relocations[fail_key] = ready_time;
  show_debug_trials();
  std::cerr << "  [Relocate] FAILED: Could not find safe parking spot for " << blocker->name << std::endl;
  std::cerr << "  [Relocate] Summary: modes=" << relocation_modes_tried
            << ", planned_candidates=" << relocation_candidates_planned
            << ", planning_failures=" << relocation_planning_failures
            << ", schedule_failures=" << relocation_schedule_failures
            << ", safety_rejections=" << relocation_safety_rejections
            << std::endl;
  return false;
}

