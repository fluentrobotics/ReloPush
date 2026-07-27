#include <PHAstar/PHAstar.h>
#include <AllocationSearch.h>
#include <DqnQModel.h>
#include <DqnFeaturesV2.h>
#include <DqnAllocationSearch.h>
#include <GeometryExport.h>
#include <PgTableExport.h>
#include <EvalPlansCli.h>
#include <PlanningHelpers.h>
#include <CollisionScheduling.h>
#include <TaskExecution.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <array>
#include <cstdio>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ReloPush/config.h>

// Required by PHAstar.h
bool DEBUG_VIS = false;

namespace
{

  constexpr double kPosEps = 1e-2;
  constexpr double kYawEps = 1e-2;
  bool g_enable_visualization = false;
  bool g_include_known_failure_repros = false;
  std::filesystem::path g_test_executable_path;

  void maybe_show_results(
      const std::string &case_name, const TimeTable &timetable,
      const std::unordered_map<std::string, EntityMeta *> &entities,
      const std::vector<Trajectory> &all_trajectories, const Params &params)
  {
    if (!g_enable_visualization)
    {
      return;
    }
    std::cout << "    [VIS] " << case_name
              << " (close the window to continue)\n";

    int argc = 1;
    char app_name[] = "phastar_unit_tests";
    char *argv[] = {app_name, nullptr};
    show_results(argc, argv, timetable, entities, all_trajectories, params);
  }

  struct EntityStore
  {
    std::vector<std::unique_ptr<EntityMeta>> owned;
    std::unordered_map<std::string, EntityMeta *> entities;

    RobotMeta *add_robot(const std::string &name, const Pose &initial_pose)
    {
      auto robot = std::make_unique<RobotMeta>();
      robot->name = name;
      robot->type = EntityType::ROBOT;
      robot->initial_pose = initial_pose;
      robot->size.front_length = 0.32;
      robot->size.rear_length = 0.2;
      robot->size.width = 0.3;
      robot->min_turning_radius = 1.43;
      robot->wheel_base = 0.4;
      robot->speed_transit = 0.2;
      robot->speed_transfer = 0.15;
      RobotMeta *raw = robot.get();
      entities[name] = raw;
      owned.push_back(std::move(robot));
      return raw;
    }

    RobotMeta *add_mars_runtime_robot(const std::string &name,
                                       const Pose &initial_pose)
    {
      RobotMeta *robot = add_robot(name, initial_pose);
      robot->size.front_length = 0.36;
      robot->size.rear_length = 0.12;
      robot->size.width = 0.275;
      robot->min_turning_radius = 1.02;
      robot->min_turning_radius_transit = 1.02;
      robot->min_turning_radius_transfer = 1.43;
      robot->wheel_base = 0.29;
      robot->speed_transit = 0.2;
      robot->speed_transfer = 0.15;
      return robot;
    }

    ObjectMeta *add_object(const std::string &name, const Pose &initial_pose)
    {
      auto obj = std::make_unique<ObjectMeta>();
      obj->name = name;
      obj->type = EntityType::OBJECT;
      obj->initial_pose = initial_pose;
      obj->size.front_length = 0.075;
      obj->size.rear_length = 0.075;
      obj->size.width = 0.15;
      ObjectMeta *raw = obj.get();
      entities[name] = raw;
      owned.push_back(std::move(obj));
      return raw;
    }
  };

  Params make_push_demo_params()
  {
    Params params;
    params.min_x = 0.0;
    params.max_x = 4.0;
    params.min_y = 0.0;
    params.max_y = 5.2;
    params.analytic_threshold = 5.0 * params.max_steer;
    return params;
  }

  Params make_task5_b8_boundary_params()
  {
    Params params = make_push_demo_params();
    params.xy_resolution = 0.1;
    params.yaw_resolution = M_PI / 6.0;
    params.time_step = 1.6;
    params.time_resolution = params.time_step;
    params.rs_step_size = 0.16;
    params.collision_check_time_step = 0.05;
    params.robot_collision_inflation = 1.005;
    return params;
  }

  Params make_relopush_like_task5_b8_params()
  {
    Params params = make_task5_b8_boundary_params();
    params.xy_resolution = 0.08;
    params.yaw_resolution = 0.235;
    params.time_step = 1.2;
    params.time_resolution = params.time_step;
    params.rs_step_size = 0.12;
    return params;
  }

  Params make_contact_boundary_task5_b8_params()
  {
    Params params = make_relopush_like_task5_b8_params();
    params.time_step = 1.0;
    params.time_resolution = params.time_step;
    params.rs_step_size = 0.10;
    params.reverse_penalty = 2.0;
    params.spatial_only_index = true;
    params.disable_wait_primitive = true;
    params.enable_holonomic_heuristic = true;
    params.holonomic_heuristic_resolution = 0.10;
    return params;
  }

  void print_planning_stats(const PlanningResult &res)
  {
    const auto &s = res.debug_stats;
    std::cout << "      status=" << static_cast<int>(res.status)
              << ", detail='" << res.failure_detail << "'"
              << ", iterations=" << s.iterations
              << ", generated=" << s.generated_nodes
              << ", accepted=" << s.accepted_nodes
              << ", reject_collision=" << s.reject_collision
              << ", analytic_collision=" << s.analytic_collision
              << " (boundary=" << s.analytic_boundary_collision
              << ", object=" << s.analytic_object_collision << ")"
              << ", spatial_index=" << (s.spatial_index_collapses ? "true" : "false")
              << ", wait_skipped=" << s.wait_primitives_skipped
              << ", best_dist=" << s.best_dist
              << ", best_yaw_error=" << s.best_yaw_error
              << ", best_pose=(" << s.best_pose.x << ", " << s.best_pose.y
              << ", " << s.best_pose.yaw << ")\n";
  }

  Waypoint make_waypoint(double x, double y, double yaw, double t)
  {
    Waypoint wp;
    wp.x = x;
    wp.y = y;
    wp.yaw = yaw;
    wp.time = t;
    return wp;
  }

  bool near(double a, double b, double eps = kPosEps)
  {
    return std::abs(a - b) <= eps;
  }

  std::string run_command_capture(const std::string &command, int &exit_code)
  {
    std::array<char, 512> buffer{};
    std::string output;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe)
    {
      exit_code = -1;
      return {};
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr)
    {
      output += buffer.data();
    }

    const int rc = pclose(pipe);
    exit_code = rc;
    return output;
  }

  std::string read_text_file(const std::filesystem::path &path)
  {
    std::ifstream ifs(path);
    if (!ifs)
    {
      return {};
    }

    return std::string((std::istreambuf_iterator<char>(ifs)),
                       std::istreambuf_iterator<char>());
  }

  bool has_wait_segment(const std::vector<Waypoint> &wps)
  {
    for (size_t i = 1; i < wps.size(); ++i)
    {
      const double dx = wps[i].x - wps[i - 1].x;
      const double dy = wps[i].y - wps[i - 1].y;
      if (std::hypot(dx, dy) < 1e-3 && (wps[i].time - wps[i - 1].time) > 0.5)
      {
        return true;
      }
    }
    return false;
  }

  double max_y_deviation(const std::vector<Waypoint> &wps, double y_ref)
  {
    double max_dev = 0.0;
    for (const auto &wp : wps)
    {
      max_dev = std::max(max_dev, std::abs(wp.y - y_ref));
    }
    return max_dev;
  }

  bool path_is_collision_free(
      RobotMeta *robot, const std::vector<Waypoint> &waypoints, TimeTable &timetable,
      const std::unordered_map<std::string, EntityMeta *> &entities,
      const Params &params, bool is_transfer = false,
      const std::string &transfer_obj_name = "")
  {
    if (waypoints.empty())
    {
      return false;
    }

    const Pose goal(waypoints.back().x, waypoints.back().y, waypoints.back().yaw);
    PHAStar checker(robot, goal, &timetable, &entities, params, is_transfer,
                    transfer_obj_name, waypoints.front().time);

    const double t_start = waypoints.front().time;
    const double t_end = waypoints.back().time;
    for (double t = t_start; t <= t_end + 1e-6; t += 0.1)
    {
      const Pose pose = TimeTable::interpolate_waypoints(waypoints, t);
      Node probe(pose.x, pose.y, pose.yaw, t, 0.0, 0.0, nullptr, 1);
      const auto info = checker.check_collision_at(&probe);
      if (!info.is_valid)
      {
        return false;
      }
    }
    return true;
  }

  void add_corridor_wall(EntityStore &store, const std::string &prefix, double y)
  {
    int idx = 0;
    for (double x = 0.1; x <= 3.9 + 1e-9; x += 0.12)
    {
      store.add_object(prefix + std::to_string(idx++), Pose(x, y, 0.0));
    }
  }

  struct ParkingSearchResult
  {
    bool found = false;
    Pose chosen_pose;
    PlanningResult result;
  };

  ParkingSearchResult find_safe_parking_candidate(
      RobotMeta *robot, double start_time, const std::vector<Pose> &candidates,
      TimeTable &timetable,
      const std::unordered_map<std::string, EntityMeta *> &entities,
      const Params &params)
  {
    ParkingSearchResult out;
    for (const auto &candidate : candidates)
    {
      robot->initial_pose = timetable.get_pose(robot, start_time);
      PHAStar planner(robot, candidate, &timetable, &entities, params, false, "",
                      start_time);
      planner.max_search_iterations = 10000;
      auto res = planner.Planning_with_res(start_time);
      if (res.status == PlanningStatus::SUCCESS)
      {
        out.found = true;
        out.chosen_pose = candidate;
        out.result = std::move(res);
        return out;
      }
    }
    return out;
  }

  bool parked_pose_is_safe_until_last_timestamp(
      RobotMeta *robot, const Pose &park_pose, double arrival_time,
      double planning_start_time, TimeTable &timetable,
      const std::unordered_map<std::string, EntityMeta *> &entities,
      const Params &params)
  {
    PHAStar checker(robot, park_pose, &timetable, &entities, params, false, "",
                    planning_start_time);
    const double last_t = timetable.get_max_time();
    for (double t = arrival_time + params.time_step; t <= last_t + 1e-6;
         t += params.time_step)
    {
      Node probe(park_pose.x, park_pose.y, park_pose.yaw, t, 0.0, 0.0, nullptr, 0);
      const auto info = checker.check_collision_at(&probe);
      if (!info.is_valid)
      {
        return false;
      }
    }
    return true;
  }

  Pose propagate_pose_with_primitive_for_test(
      const Pose &pose,
      int direction,
      double steer,
      double speed,
      double wheel_base,
      double dt)
  {
    Pose out = pose;
    const double distance = static_cast<double>(direction) * speed * dt;
    if (std::abs(distance) < 1e-9)
    {
      return out;
    }

    const double curvature = std::tan(steer) / std::max(wheel_base, 1e-6);
    if (std::abs(curvature) < 1e-9)
    {
      out.x += distance * std::cos(pose.yaw);
      out.y += distance * std::sin(pose.yaw);
      out.yaw = mod2pi(pose.yaw);
      return out;
    }

    const double yaw_delta = distance * curvature;
    const double radius = 1.0 / curvature;
    out.x += radius * (std::sin(pose.yaw + yaw_delta) - std::sin(pose.yaw));
    out.y += -radius * (std::cos(pose.yaw + yaw_delta) - std::cos(pose.yaw));
    out.yaw = mod2pi(pose.yaw + yaw_delta);
    return out;
  }

  std::vector<Pose> generate_expand_parking_candidates_for_test(
      const Pose &start_pose,
      RobotMeta *robot,
      const Params &params,
      double delta_t = 0.5,
      int max_depth = 12,
      size_t max_candidates = 300)
  {
    std::vector<Pose> candidates;
    if (!robot)
    {
      return candidates;
    }

    const double max_curvature = 1.0 / std::max(robot->min_turning_radius, 1e-6);
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
      const int xi = static_cast<int>(std::round(p.x * 20.0));
      const int yi = static_cast<int>(std::round(p.y * 20.0));
      const int ai = static_cast<int>(std::round(mod2pi(p.yaw) * 20.0));
      return std::to_string(xi) + ":" + std::to_string(yi) + ":" + std::to_string(ai);
    };

    std::vector<ExpansionNode> frontier;
    frontier.push_back({start_pose, 0});

    std::unordered_set<std::string> visited;
    visited.insert(pose_key(start_pose));

    for (int depth = 1; depth <= max_depth && candidates.size() < max_candidates; ++depth)
    {
      std::vector<ExpansionNode> next_frontier;
      for (const auto &node : frontier)
      {
        for (const auto &[direction, steer] : primitives)
        {
          Pose next_pose = propagate_pose_with_primitive_for_test(
              node.pose,
              direction,
              steer,
              robot->speed_transit,
              robot->wheel_base,
              delta_t);

          const std::string key = pose_key(next_pose);
          if (visited.count(key))
          {
            continue;
          }
          visited.insert(key);

          CollisionGeometry geom_bounds =
              setup_collision_geometry(next_pose, robot->size, 1.0);
          if (check_robot_bounds_collision(next_pose, geom_bounds.corners, params))
          {
            continue;
          }

          next_frontier.push_back({next_pose, depth});

          const double displacement =
              std::hypot(next_pose.x - start_pose.x, next_pose.y - start_pose.y);
          if (displacement < 0.25)
          {
            continue;
          }

          candidates.push_back(next_pose);
          if (candidates.size() >= max_candidates)
          {
            break;
          }
        }
        if (candidates.size() >= max_candidates)
        {
          break;
        }
      }

      if (next_frontier.empty())
      {
        break;
      }
      frontier = std::move(next_frontier);
    }

    return candidates;
  }

  bool test_detour_around_static_obstacle()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *robot1 = store.add_robot("robot1", Pose(0.5, 0.45, 0.0));
    store.add_object("obj_block", Pose(2.0, 0.45, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    const Pose goal(3.5, 0.45, 0.0);
    robot1->initial_pose = timetable.get_pose(robot1, 0.0);
    PHAStar planner(robot1, goal, &timetable, &store.entities, params, false, "",
                    0.0);
    planner.max_search_iterations = 10000;
    const auto res = planner.Planning_with_res(0.0);

    if (res.status != PlanningStatus::SUCCESS)
    {
      std::cerr << "    Expected SUCCESS, got status "
                << static_cast<int>(res.status) << "\n";
      return false;
    }
    if (!path_is_collision_free(robot1, res.waypoints, timetable, store.entities,
                                params))
    {
      std::cerr << "    Planned path has a collision.\n";
      return false;
    }
    if (max_y_deviation(res.waypoints, 0.45) < 0.2)
    {
      std::cerr << "    Path did not detour around static obstacle.\n";
      return false;
    }

    Trajectory planned;
    planned.entity = robot1;
    planned.start_time = 0.0;
    planned.is_transfer = false;
    planned.waypoints = res.waypoints;
    maybe_show_results("Detour around static entities", timetable, store.entities,
                       {planned}, params);
    return true;
  }

  bool test_wait_for_dynamic_entity()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *robot1 = store.add_robot("robot1", Pose(0.5, 2.6, 0.0));
    RobotMeta *robot2 = store.add_robot("robot2", Pose(2.2, 1.8, M_PI / 2.0));
    add_corridor_wall(store, "wall_low_", 2.3);
    add_corridor_wall(store, "wall_high_", 2.9);

    const Pose goal(3.5, 2.6, 0.0);

    TimeTable baseline(0.5);
    baseline.add_initial(store.entities);
    robot1->initial_pose = baseline.get_pose(robot1, 0.0);
    PHAStar baseline_planner(robot1, goal, &baseline, &store.entities, params,
                             false, "", 0.0);
    baseline_planner.max_search_iterations = 15000;
    const auto baseline_res = baseline_planner.Planning_with_res(0.0);
    if (baseline_res.status != PlanningStatus::SUCCESS)
    {
      std::cerr << "    Baseline planning failed.\n";
      return false;
    }
    const double baseline_arrival = baseline_res.waypoints.back().time;

    TimeTable blocked(0.5);
    blocked.add_initial(store.entities);
    Trajectory mover;
    mover.entity = robot2;
    mover.start_time = 0.0;
    mover.is_transfer = false;
    mover.waypoints = {make_waypoint(2.2, 1.8, M_PI / 2.0, 0.0),
                       make_waypoint(2.2, 3.4, M_PI / 2.0, 20.0)};
    blocked.add_trajectory(mover);

    robot1->initial_pose = blocked.get_pose(robot1, 0.0);
    PHAStar blocked_planner(robot1, goal, &blocked, &store.entities, params,
                            false, "", 0.0);
    blocked_planner.max_search_iterations = 15000;
    const auto blocked_res = blocked_planner.Planning_with_res(0.0);
    if (blocked_res.status != PlanningStatus::SUCCESS)
    {
      std::cerr << "    Dynamic blocked scenario planning failed.\n";
      return false;
    }

    const double blocked_arrival = blocked_res.waypoints.back().time;
    if (blocked_arrival <= baseline_arrival + 2.0)
    {
      std::cerr << "    Planner did not introduce time delay for dynamic blocker.\n";
      return false;
    }
    if (!has_wait_segment(blocked_res.waypoints))
    {
      std::cerr << "    Planned path has no explicit waiting segment.\n";
      return false;
    }
    if (!path_is_collision_free(robot1, blocked_res.waypoints, blocked,
                                store.entities, params))
    {
      std::cerr << "    Wait path is not collision-free.\n";
      return false;
    }

    Trajectory planned;
    planned.entity = robot1;
    planned.start_time = 0.0;
    planned.is_transfer = false;
    planned.waypoints = blocked_res.waypoints;
    maybe_show_results("Wait for dynamic entities", blocked, store.entities,
                       {mover, planned}, params);
    return true;
  }

  bool test_transfer_updates_object_pose()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *robot = store.add_robot("robot1", Pose(1.0, 1.0, 0.0));

    const double offset = robot->size.front_length + 0.075; // object rear length
    ObjectMeta *obj = store.add_object("obj1", Pose(1.0 + offset, 1.0, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory push;
    push.entity = robot;
    push.transferred_object = obj;
    push.start_time = 0.0;
    push.is_transfer = true;
    push.waypoints = {
        make_waypoint(1.0, 1.0, 0.0, 0.0),
        make_waypoint(1.3, 1.0, 0.0, 2.0),
        make_waypoint(1.3, 1.0, M_PI / 2.0, 4.0),
    };
    timetable.add_trajectory(push);

    const double initial_obj_yaw = obj->initial_pose.yaw;
    const double robot_start_yaw = push.waypoints.front().yaw;
    for (double t : {0.0, 1.0, 2.0, 3.0, 4.0})
    {
      const Pose robot_pose = timetable.get_pose(robot, t);
      Pose expected =
          TimeTable::compute_object_pose(robot_pose, robot->size, obj->size);
      const double delta_yaw = mod2pi(robot_pose.yaw - robot_start_yaw);
      expected.yaw = mod2pi(initial_obj_yaw + delta_yaw);

      const Pose actual = timetable.get_pose(obj, t);
      if (!near(expected.x, actual.x) || !near(expected.y, actual.y) ||
          !near(expected.yaw, actual.yaw, kYawEps))
      {
        std::cerr << "    Object pose mismatch at t=" << t << "\n";
        return false;
      }
    }

    maybe_show_results("Transfer updates pushed object pose", timetable,
                       store.entities, {push}, params);
    return true;
  }

  bool test_other_robot_avoids_temporarily_relocated_object()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *robot1 = store.add_robot("robot1", Pose(1.005, 1.0, 0.0));
    RobotMeta *robot2 = store.add_robot("robot2", Pose(0.5, 1.0, 0.0));
    ObjectMeta *obj = store.add_object("obj_temp", Pose(1.4, 1.0, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory temp_push;
    temp_push.entity = robot1;
    temp_push.transferred_object = obj;
    temp_push.start_time = 0.0;
    temp_push.is_transfer = true;
    temp_push.waypoints = {make_waypoint(1.005, 1.0, 0.0, 0.0),
                           make_waypoint(1.605, 1.0, 0.0, 4.0)};
    timetable.add_trajectory(temp_push);

    Trajectory clear_robot1;
    clear_robot1.entity = robot1;
    clear_robot1.start_time = 4.0;
    clear_robot1.is_transfer = false;
    clear_robot1.waypoints = {make_waypoint(1.605, 1.0, 0.0, 0.0),
                              make_waypoint(1.0, 0.4, -M_PI / 2.0, 4.0)};
    timetable.add_trajectory(clear_robot1);

    const double start_time = 9.0;
    const Pose goal(3.5, 1.0, 0.0);
    robot2->initial_pose = timetable.get_pose(robot2, start_time);
    PHAStar planner(robot2, goal, &timetable, &store.entities, params, false, "",
                    start_time);
    planner.max_search_iterations = 12000;
    const auto res = planner.Planning_with_res(start_time);

    if (res.status != PlanningStatus::SUCCESS)
    {
      std::cerr << "    Planning failed when avoiding temporary object.\n";
      return false;
    }
    if (!path_is_collision_free(robot2, res.waypoints, timetable, store.entities,
                                params))
    {
      std::cerr << "    Planned path collides with entities.\n";
      return false;
    }
    if (max_y_deviation(res.waypoints, 1.0) < 0.15)
    {
      std::cerr << "    Path did not avoid relocated object.\n";
      return false;
    }

    Trajectory planned;
    planned.entity = robot2;
    planned.start_time = start_time;
    planned.is_transfer = false;
    planned.waypoints = res.waypoints;
    maybe_show_results("Avoid temporarily relocated object", timetable,
                       store.entities, {temp_push, clear_robot1, planned},
                       params);
    return true;
  }

  // Regression test for MARS/11path-planning-conflict-review.md Findings 1/2:
  // an idle robot parked directly on another robot's initial-transit goal
  // pose must itself be relocated (mechanism A, via relocate_blocking_robot)
  // rather than the task robot self-parking next to a goal that remains
  // occupied (a near-no-op, pre-fix). Exercises the GOAL_INVALID_COLLISION
  // branch of attempt_validation_blocker_recovery inside plan_initial_transit.
  bool test_idle_blocker_relocated_off_initial_transit_goal()
  {
    Params params = make_push_demo_params();
    RuntimeOptions options; // defaults: initial_transit_methods, safe parking, etc.

    EntityStore store;
    const Pose start_pose(0.5, 1.0, 0.0);
    const Pose goal_pose(3.0, 1.0, 0.0);
    RobotMeta *robot1 = store.add_mars_runtime_robot("robot1", start_pose);
    RobotMeta *robot2 = store.add_mars_runtime_robot("robot2", goal_pose);

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    const Pose blocker_initial_pose = timetable.get_pose(robot2, 0.0);

    double out_abs_start = -1.0;
    double out_abs_end = -1.0;
    const bool succeeded = plan_initial_transit(
        robot1, goal_pose, 0.0, timetable, store.entities, params, options,
        &out_abs_start, &out_abs_end);

    if (!succeeded)
    {
      std::cerr << "    plan_initial_transit failed; expected the idle "
                   "blocker on the goal pose to be relocated instead.\n";
      return false;
    }

    const double robot1_final_time = timetable.get_entity_max_time(robot1);
    const Pose robot1_final_pose = timetable.get_pose(robot1, robot1_final_time);
    if (!near(robot1_final_pose.x, goal_pose.x, 0.15) ||
        !near(robot1_final_pose.y, goal_pose.y, 0.15))
    {
      std::cerr << "    robot1 did not reach the intended goal pose ("
                << robot1_final_pose.x << ", " << robot1_final_pose.y
                << ") vs goal (" << goal_pose.x << ", " << goal_pose.y
                << ").\n";
      return false;
    }

    const double blocker_final_time = timetable.get_entity_max_time(robot2);
    const Pose blocker_final_pose = timetable.get_pose(robot2, blocker_final_time);
    const double blocker_moved_dist =
        std::hypot(blocker_final_pose.x - blocker_initial_pose.x,
                  blocker_final_pose.y - blocker_initial_pose.y);
    if (blocker_moved_dist < 0.2)
    {
      std::cerr << "    Idle blocker robot2 was not relocated off the goal "
                   "pose (moved "
                << blocker_moved_dist << "m); expected a mechanism-A "
                   "relocation, not a self-park no-op.\n";
      return false;
    }

    maybe_show_results("Idle blocker relocated off initial transit goal",
                       timetable, store.entities, {}, params);
    return true;
  }

  // Stage 1 planner timing instrumentation (opt-in per-plan timing; see
  // MARS/include/PHAstarPushDemoTypes.h's PlanTimingStats). Sums every field
  // of a PlanTimingStats instance and asserts add()/operator+=/operator+ all
  // agree and produce doubled values.
  bool test_plan_timing_stats_accumulate()
  {
    PlanTimingStats a;
    a.true_wall_s = 1.0;
    a.search_wall_s_primary = 2.0;
    a.search_wall_s_fine = 3.0;
    a.search_wall_s_contact = 4.0;
    a.search_wall_s_other = 5.0;
    a.n_searches_primary = 1;
    a.n_searches_fine = 2;
    a.n_searches_contact = 3;
    a.n_searches_other = 4;
    a.search_iterations_total = 10;
    a.n_search_cap_hits = 1;
    a.heuristic_time_s = 0.1;
    a.primitive_collision_time_s = 0.2;
    a.analytic_validation_time_s = 0.3;
    a.holonomic_heuristic_time_s = 0.4;
    a.sched_wall_s = 0.5;
    a.n_find_safe_start_calls = 2;
    a.n_start_candidates_tried = 5;
    a.traj_scan_wall_s = 0.6;
    a.terminal_hold_wall_s = 0.7;
    a.safe_parking_wall_s = 0.8;
    a.n_parking_relocations = 1;
    a.n_robot_candidate_attempts = 3;
    a.n_post_validation_retries = 2;
    a.n_obsrelo_segments = 4;

    const PlanTimingStats b = a; // identical copy: summing should double every field

    bool ok = true;
    auto check_double = [&](const char *name, double got, double expected)
    {
      if (std::abs(got - expected) > 1e-9)
      {
        std::cerr << "    " << name << " mismatch: got " << got
                  << ", expected " << expected << "\n";
        ok = false;
      }
    };
    auto check_count = [&](const char *name, long long got, long long expected)
    {
      if (got != expected)
      {
        std::cerr << "    " << name << " mismatch: got " << got
                  << ", expected " << expected << "\n";
        ok = false;
      }
    };

    auto verify_doubled = [&](const PlanTimingStats &sum, const char *op_name)
    {
      check_double(op_name, sum.true_wall_s, 2.0);
      check_double(op_name, sum.search_wall_s_primary, 4.0);
      check_double(op_name, sum.search_wall_s_fine, 6.0);
      check_double(op_name, sum.search_wall_s_contact, 8.0);
      check_double(op_name, sum.search_wall_s_other, 10.0);
      check_count(op_name, sum.n_searches_primary, 2);
      check_count(op_name, sum.n_searches_fine, 4);
      check_count(op_name, sum.n_searches_contact, 6);
      check_count(op_name, sum.n_searches_other, 8);
      check_count(op_name, static_cast<long long>(sum.search_iterations_total), 20);
      check_count(op_name, sum.n_search_cap_hits, 2);
      check_double(op_name, sum.heuristic_time_s, 0.2);
      check_double(op_name, sum.primitive_collision_time_s, 0.4);
      check_double(op_name, sum.analytic_validation_time_s, 0.6);
      check_double(op_name, sum.holonomic_heuristic_time_s, 0.8);
      check_double(op_name, sum.sched_wall_s, 1.0);
      check_count(op_name, sum.n_find_safe_start_calls, 4);
      check_count(op_name, sum.n_start_candidates_tried, 10);
      check_double(op_name, sum.traj_scan_wall_s, 1.2);
      check_double(op_name, sum.terminal_hold_wall_s, 1.4);
      check_double(op_name, sum.safe_parking_wall_s, 1.6);
      check_count(op_name, sum.n_parking_relocations, 2);
      check_count(op_name, sum.n_robot_candidate_attempts, 6);
      check_count(op_name, sum.n_post_validation_retries, 4);
      check_count(op_name, sum.n_obsrelo_segments, 8);
    };

    PlanTimingStats sum_add = a;
    sum_add.add(b);
    verify_doubled(sum_add, "add()");

    PlanTimingStats sum_plus_eq = a;
    sum_plus_eq += b;
    verify_doubled(sum_plus_eq, "operator+=");

    const PlanTimingStats sum_plus = a + b;
    verify_doubled(sum_plus, "operator+");

    // operator+ must not mutate either operand.
    check_double("operator+ lhs unmodified", a.true_wall_s, 1.0);
    check_double("operator+ rhs unmodified", b.true_wall_s, 1.0);

    return ok;
  }

  // Stage 1 planner timing instrumentation: initialize_params must wire
  // RuntimeOptions::default_safe_parking_expand_iterations into
  // Params::safe_parking_expand_max_iterations and
  // RuntimeOptions::default_collision_check_time_step into
  // Params::collision_check_time_step (the latter already existed pre-Stage-1;
  // this locks in that it still does). Also checks the positive_or() fallback:
  // a non-positive option value keeps Params's own compiled-in default rather
  // than propagating a bogus 0 through to the planner.
  bool test_initialize_params_wires_stage1_fields()
  {
    bool ok = true;

    RuntimeOptions options;
    options.default_safe_parking_expand_iterations = 1234;
    options.default_collision_check_time_step = 0.0123;

    const Params params = initialize_params({}, options);

    if (params.safe_parking_expand_max_iterations != 1234)
    {
      std::cerr << "    safe_parking_expand_max_iterations mismatch: got "
                << params.safe_parking_expand_max_iterations << ", expected 1234\n";
      ok = false;
    }
    if (!near(params.collision_check_time_step, 0.0123, 1e-9))
    {
      std::cerr << "    collision_check_time_step mismatch: got "
                << params.collision_check_time_step << ", expected 0.0123\n";
      ok = false;
    }

    RuntimeOptions zero_options;
    zero_options.default_safe_parking_expand_iterations = 0;
    zero_options.default_collision_check_time_step = 0.0;
    const Params defaulted = initialize_params({}, zero_options);
    const Params fresh_defaults; // Params{}'s own compiled-in defaults

    if (defaulted.safe_parking_expand_max_iterations !=
        fresh_defaults.safe_parking_expand_max_iterations)
    {
      std::cerr << "    safe_parking_expand_max_iterations did not fall back to "
                   "the Params default for a non-positive option value (got "
                << defaulted.safe_parking_expand_max_iterations << ", expected "
                << fresh_defaults.safe_parking_expand_max_iterations << ")\n";
      ok = false;
    }
    if (!near(defaulted.collision_check_time_step,
             fresh_defaults.collision_check_time_step, 1e-12))
    {
      std::cerr << "    collision_check_time_step did not fall back to the "
                   "Params default for a non-positive option value (got "
                << defaulted.collision_check_time_step << ", expected "
                << fresh_defaults.collision_check_time_step << ")\n";
      ok = false;
    }

    return ok;
  }

  // Stage 1 planner timing instrumentation smoke test: mirrors
  // test_idle_blocker_relocated_off_initial_transit_goal's tiny scenario
  // (robot2 idle exactly on robot1's initial-transit goal, forcing a
  // mechanism-A relocation) but runs plan_initial_transit twice from
  // identical fresh state -- once with a PlanTimingStats attached, once with
  // nullptr -- and asserts: (1) timing/counters were actually collected
  // (nonzero search wall time from the 3 tier attempts that hit
  // GOAL_INVALID_COLLISION before recovery, plus a successful
  // relocate_blocking_robot invocation -- this exact scenario resolves via
  // attempt_validation_blocker_recovery's direct-relocate branch, whose
  // post-relocation replan succeeds outright with no scheduling/delay
  // needed, so find_safe_start_time is correctly never called here; see
  // n_find_safe_start_calls>=1 covered instead by the CollisionScheduling.cpp
  // call sites this test doesn't exercise), and (2) critically, the PLANNING
  // RESULT (success flag + both out_abs times) is bit-for-bit identical
  // either way, i.e. attaching the stats pointer is purely an observer with
  // zero effect on planner behavior.
  bool test_plan_initial_transit_timing_stats_do_not_affect_result()
  {
    auto run_once = [](PlanTimingStats *stats, bool &out_success,
                       double &out_abs_start, double &out_abs_end)
    {
      Params params = make_push_demo_params();
      RuntimeOptions options; // defaults: initial_transit_methods, safe parking, etc.

      EntityStore store;
      const Pose start_pose(0.5, 1.0, 0.0);
      const Pose goal_pose(3.0, 1.0, 0.0);
      RobotMeta *robot1 = store.add_mars_runtime_robot("robot1", start_pose);
      store.add_mars_runtime_robot("robot2", goal_pose);

      TimeTable timetable(0.5);
      timetable.add_initial(store.entities);

      out_abs_start = -1.0;
      out_abs_end = -1.0;
      out_success = plan_initial_transit(robot1, goal_pose, 0.0, timetable,
                                         store.entities, params, options,
                                         &out_abs_start, &out_abs_end, {},
                                         nullptr, stats);
    };

    PlanTimingStats stats;
    bool success_with_stats = false;
    double abs_start_with_stats = -1.0;
    double abs_end_with_stats = -1.0;
    run_once(&stats, success_with_stats, abs_start_with_stats, abs_end_with_stats);

    bool success_without_stats = false;
    double abs_start_without_stats = -1.0;
    double abs_end_without_stats = -1.0;
    run_once(nullptr, success_without_stats, abs_start_without_stats,
            abs_end_without_stats);

    bool ok = true;

    const double total_search_wall_s = stats.search_wall_s_primary +
                                       stats.search_wall_s_fine +
                                       stats.search_wall_s_contact +
                                       stats.search_wall_s_other;
    if (total_search_wall_s <= 0.0)
    {
      std::cerr << "    expected nonzero search wall time with PlanTimingStats "
                   "attached, got "
                << total_search_wall_s << "\n";
      ok = false;
    }
    // This scenario resolves via a direct relocate_blocking_robot() call
    // (mechanism A) rather than the find_safe_start_time scheduling path
    // (see the doc comment above), so assert the relocation-side counters
    // instead of n_find_safe_start_calls.
    if (stats.safe_parking_wall_s <= 0.0)
    {
      std::cerr << "    expected nonzero safe_parking_wall_s with PlanTimingStats "
                   "attached, got "
                << stats.safe_parking_wall_s << "\n";
      ok = false;
    }
    if (stats.n_parking_relocations < 1)
    {
      std::cerr << "    expected n_parking_relocations >= 1, got "
                << stats.n_parking_relocations << "\n";
      ok = false;
    }

    if (success_with_stats != success_without_stats)
    {
      std::cerr << "    plan_initial_transit success (" << success_with_stats
                << " vs " << success_without_stats
                << ") differs with vs without a PlanTimingStats attached.\n";
      ok = false;
    }
    if (!near(abs_start_with_stats, abs_start_without_stats, 1e-9))
    {
      std::cerr << "    out_abs_start_time (" << abs_start_with_stats << " vs "
                << abs_start_without_stats
                << ") differs with vs without a PlanTimingStats attached.\n";
      ok = false;
    }
    if (!near(abs_end_with_stats, abs_end_without_stats, 1e-9))
    {
      std::cerr << "    out_abs_end_time (" << abs_end_with_stats << " vs "
                << abs_end_without_stats
                << ") differs with vs without a PlanTimingStats attached.\n";
      ok = false;
    }

    return ok;
  }

  // ==========================================
  // Stage 2 planner_opt: hot-path collision-check optimizations
  // ==========================================

  // TimeTable::for_each_pose(t, fn) must visit exactly the (entity, pose) set
  // that get_poses(t) returns -- it is the zero-allocation replacement for
  // get_poses(t) used throughout the collision-check hot path. Exercises an
  // entity with no trajectory (initial pose only), and query times before the
  // first recorded sample, exactly on recorded waypoint times (including two
  // entities sharing one), strictly between samples, and after the last
  // recorded sample.
  bool test_for_each_pose_matches_get_poses()
  {
    EntityStore store;
    RobotMeta *robot1 = store.add_robot("robot1", Pose(0.0, 0.0, 0.0));
    RobotMeta *robot2 = store.add_robot("robot2", Pose(5.0, 5.0, M_PI));
    store.add_object("obj1", Pose(2.0, 2.0, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory traj1;
    traj1.entity = robot1;
    traj1.start_time = 1.0;
    traj1.is_transfer = false;
    traj1.waypoints = {make_waypoint(0.0, 0.0, 0.0, 0.0),
                       make_waypoint(1.0, 0.0, 0.0, 2.0),
                       make_waypoint(2.0, 1.0, M_PI / 4.0, 5.0)};
    timetable.add_trajectory(traj1);

    Trajectory traj2;
    traj2.entity = robot2;
    traj2.start_time = 3.0;
    traj2.is_transfer = false;
    traj2.waypoints = {make_waypoint(5.0, 5.0, M_PI, 0.0),
                       make_waypoint(4.0, 4.0, M_PI, 4.0)};
    timetable.add_trajectory(traj2);
    // obj1 deliberately never gets add_trajectory: it exercises the
    // "initial pose only" per-entity submap path in both APIs.

    const std::vector<double> query_times = {
        -1.0, // before every recorded sample (clamps to the earliest pose)
        1.0,  // exactly on traj1's first absolute waypoint time
        3.0,  // exactly on traj1's 2nd waypoint AND traj2's start, at once
        4.5,  // strictly between recorded samples for both trajectories
        6.0,  // exactly on traj1's last waypoint (end of its trajectory)
        7.0,  // exactly on traj2's last waypoint
        100.0 // after every recorded sample (clamps to the latest pose)
    };

    for (double t : query_times)
    {
      auto expected = timetable.get_poses(t);

      std::unordered_map<EntityMeta *, Pose> visited;
      timetable.for_each_pose(t, [&](EntityMeta *ent, const Pose &pose)
                              { visited[ent] = pose; });

      if (visited.size() != expected.size())
      {
        std::cerr << "    t=" << t << ": for_each_pose visited " << visited.size()
                  << " entities, get_poses returned " << expected.size() << ".\n";
        return false;
      }

      for (const auto &[ent, expected_pose] : expected)
      {
        auto it = visited.find(ent);
        if (it == visited.end())
        {
          std::cerr << "    t=" << t << ": for_each_pose did not visit entity "
                    << ent->name << ".\n";
          return false;
        }
        const Pose &got = it->second;
        if (!near(got.x, expected_pose.x, 1e-9) || !near(got.y, expected_pose.y, 1e-9) ||
            !near(got.yaw, expected_pose.yaw, 1e-9))
        {
          std::cerr << "    t=" << t << ": pose mismatch for " << ent->name
                    << ": for_each_pose=(" << got.x << ", " << got.y << ", " << got.yaw
                    << ") get_poses=(" << expected_pose.x << ", " << expected_pose.y
                    << ", " << expected_pose.yaw << ").\n";
          return false;
        }
      }
    }

    return true;
  }

  // TerminalHoldCache (find_safe_start_time's memoized terminal-hold check)
  // must give IDENTICAL pass/fail and CollisionInfo fields to the pre-Stage-2
  // per-candidate forward scan (check_terminal_hold_detailed with
  // hold_cache=nullptr), for every candidate arrival time in a ladder --
  // including the exact dt-aligned boundary straddling max(C) -- while doing
  // the O(horizon/dt) backward scan only once (on whichever ladder rung
  // happens to need it first) instead of once per rung.
  bool test_terminal_hold_cache_matches_forward_scan()
  {
    Params params = make_push_demo_params();
    const double dt = shared_collision_check_step(params);

    // --- Scenario 1: a robot parked at goal_pose from t=0 through t=10.0,
    // then jumping (over exactly one dt step) to a far-away pose where it
    // stays through the rest of the horizon (t=20.0). The conflict is a
    // single contiguous block [0, 10.0] with an exact, dt-aligned boundary:
    // sample 10.0 collides, sample 10.0+dt does not.
    {
      EntityStore store;
      const Pose goal_pose(2.0, 2.0, 0.0);
      const Pose safe_pose(20.0, 20.0, 0.0);
      store.add_robot("robot_test", Pose(0.0, 0.0, 0.0));
      RobotMeta *robot_blocker = store.add_robot("robot_blocker", goal_pose);
      RobotMeta *robot_test = dynamic_cast<RobotMeta *>(store.entities.at("robot_test"));

      TimeTable timetable(0.5);
      timetable.add_initial(store.entities);

      Trajectory blocker_traj;
      blocker_traj.entity = robot_blocker;
      blocker_traj.start_time = 0.0;
      blocker_traj.is_transfer = false;
      blocker_traj.waypoints = {
          make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, 0.0),
          make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, 10.0),
          make_waypoint(safe_pose.x, safe_pose.y, safe_pose.yaw, 10.0 + dt),
          make_waypoint(safe_pose.x, safe_pose.y, safe_pose.yaw, 20.0)};
      timetable.add_trajectory(blocker_traj);

      Trajectory candidate_traj;
      candidate_traj.entity = robot_test;
      candidate_traj.is_transfer = false;
      candidate_traj.waypoints = {
          make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, 0.0)};

      const std::vector<double> arrivals = {0.0,  5.0,      8.0,  9.0, 9.5,
                                            9.9,  9.95,     10.0, 10.0 + dt,
                                            10.1, 10.5, 12.0, 18.0};

      TerminalHoldCache shared_cache;
      for (double arrival : arrivals)
      {
        CollisionInfo old_result = check_terminal_hold_detailed(
            candidate_traj, arrival, timetable, params, nullptr, nullptr);
        CollisionInfo cached_result = check_terminal_hold_detailed(
            candidate_traj, arrival, timetable, params, nullptr, &shared_cache);

        if (old_result.is_valid != cached_result.is_valid)
        {
          std::cerr << "    arrival=" << arrival << ": is_valid mismatch (old="
                    << old_result.is_valid << ", cached=" << cached_result.is_valid
                    << ").\n";
          return false;
        }
        if (old_result.reason != cached_result.reason)
        {
          std::cerr << "    arrival=" << arrival << ": reason mismatch (old='"
                    << old_result.reason << "', cached='" << cached_result.reason
                    << "').\n";
          return false;
        }
        if (old_result.entity_name != cached_result.entity_name)
        {
          std::cerr << "    arrival=" << arrival << ": entity_name mismatch (old='"
                    << old_result.entity_name << "', cached='"
                    << cached_result.entity_name << "').\n";
          return false;
        }
        if (!near(old_result.time, cached_result.time, 1e-6))
        {
          std::cerr << "    arrival=" << arrival << ": time mismatch (old="
                    << old_result.time << ", cached=" << cached_result.time << ").\n";
          return false;
        }
      }

      // Sanity: the scenario actually exercises both a failing and a passing
      // region (otherwise the equivalence loop above would be vacuous).
      const CollisionInfo before = check_terminal_hold_detailed(
          candidate_traj, 8.0, timetable, params, nullptr, nullptr);
      const CollisionInfo after = check_terminal_hold_detailed(
          candidate_traj, 10.0, timetable, params, nullptr, nullptr);
      if (before.is_valid || !after.is_valid)
      {
        std::cerr << "    Scenario sanity check failed: expected arrival=8.0 to "
                     "fail the hold and arrival=10.0 to pass it (before.is_valid="
                  << before.is_valid << ", after.is_valid=" << after.is_valid
                  << ").\n";
        return false;
      }
    }

    // --- Scenario 2: no conflicting entity anywhere in the horizon (C
    // empty). The cache must still agree with the old path across several
    // arrivals, including ones that hit the "nothing left to check" early
    // return (horizon <= arrival) as well as ones that actually run the
    // (empty) backward scan.
    {
      EntityStore store;
      const Pose goal_pose(2.0, 2.0, 0.0);
      const Pose safe_pose(20.0, 20.0, 0.0);
      store.add_robot("robot_test", Pose(0.0, 0.0, 0.0));
      RobotMeta *robot_other = store.add_robot("robot_other", safe_pose);
      RobotMeta *robot_test = dynamic_cast<RobotMeta *>(store.entities.at("robot_test"));

      TimeTable timetable(0.5);
      timetable.add_initial(store.entities);

      Trajectory other_traj;
      other_traj.entity = robot_other;
      other_traj.start_time = 0.0;
      other_traj.is_transfer = false;
      other_traj.waypoints = {
          make_waypoint(safe_pose.x, safe_pose.y, safe_pose.yaw, 0.0),
          make_waypoint(safe_pose.x, safe_pose.y, safe_pose.yaw, 15.0)};
      timetable.add_trajectory(other_traj);

      Trajectory candidate_traj;
      candidate_traj.entity = robot_test;
      candidate_traj.is_transfer = false;
      candidate_traj.waypoints = {
          make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, 0.0)};

      const std::vector<double> arrivals = {0.0, 3.0, 7.5, 14.9, 15.0, 20.0};
      TerminalHoldCache shared_cache;
      for (double arrival : arrivals)
      {
        CollisionInfo old_result = check_terminal_hold_detailed(
            candidate_traj, arrival, timetable, params, nullptr, nullptr);
        CollisionInfo cached_result = check_terminal_hold_detailed(
            candidate_traj, arrival, timetable, params, nullptr, &shared_cache);

        if (!old_result.is_valid || !cached_result.is_valid)
        {
          std::cerr << "    (C empty) arrival=" << arrival
                    << ": expected both paths to pass (old=" << old_result.is_valid
                    << ", cached=" << cached_result.is_valid << ").\n";
          return false;
        }
        if (!near(old_result.time, cached_result.time, 1e-6))
        {
          std::cerr << "    (C empty) arrival=" << arrival << ": time mismatch (old="
                    << old_result.time << ", cached=" << cached_result.time << ").\n";
          return false;
        }
      }
    }

    return true;
  }

  // End-to-end: find_safe_start_time (which internally builds and reuses one
  // TerminalHoldCache across its whole candidate loop) must return the same
  // start time as a hand-rolled replica of its candidate-stepping logic that
  // calls the pre-Stage-2 uncached primitives directly (hold_cache=nullptr
  // throughout). The scenario places a robot that occupies the candidate's
  // goal pose for a mid-horizon window, so several early candidates fail
  // purely because of that (no idle-robot relocation or boundary projection
  // is ever triggered -- the blocker is never idle, and nothing goes out of
  // bounds -- so replicating just find_safe_start_time's simple
  // wait-and-buffered-recheck logic below is a faithful stand-in for the
  // real function's behavior on the old, uncached path).
  bool test_find_safe_start_time_matches_uncached_path()
  {
    Params params = make_push_demo_params();
    const double dt = shared_collision_check_step(params);

    EntityStore store;
    const Pose start_pose(0.2, 2.0, 0.0);
    const Pose goal_pose(2.0, 2.0, 0.0);
    const Pose far_pose(20.0, 20.0, 0.0);
    RobotMeta *robot_test = store.add_robot("robot_test", start_pose);
    RobotMeta *robot_blocker = store.add_robot("robot_blocker", far_pose);

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    const double T1 = 6.0;
    const double T2 = 10.0;
    Trajectory blocker_traj;
    blocker_traj.entity = robot_blocker;
    blocker_traj.start_time = 0.0;
    blocker_traj.is_transfer = false;
    blocker_traj.waypoints = {
        make_waypoint(far_pose.x, far_pose.y, far_pose.yaw, 0.0),
        make_waypoint(far_pose.x, far_pose.y, far_pose.yaw, T1 - dt),
        make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, T1),
        make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, T2),
        make_waypoint(far_pose.x, far_pose.y, far_pose.yaw, T2 + dt),
        make_waypoint(far_pose.x, far_pose.y, far_pose.yaw, 20.0)};
    timetable.add_trajectory(blocker_traj);

    Trajectory candidate_traj;
    candidate_traj.entity = robot_test;
    candidate_traj.is_transfer = false;
    candidate_traj.waypoints = {
        make_waypoint(start_pose.x, start_pose.y, start_pose.yaw, 0.0),
        make_waypoint(goal_pose.x, goal_pose.y, goal_pose.yaw, 2.0)};

    RuntimeOptions options;
    const double earliest_start = 0.0;

    auto compute_expected_uncached = [&]() -> double
    {
      double check_time = earliest_start;
      const double step = 0.5;
      // Inlined rather than calling timetable_delay_search_horizon(): that
      // function's declaration (CollisionScheduling.h) takes `const
      // TimeTable&` but its out-of-line definition (CollisionScheduling.cpp)
      // takes a non-const `TimeTable&` -- a pre-existing, unrelated
      // declaration/definition mismatch (harmless everywhere it's actually
      // called today, since every existing caller already holds a non-const
      // TimeTable& and so binds straight to the in-TU definition instead of
      // needing the header declaration's symbol) that this test would
      // otherwise be the first cross-TU caller to trip over at link time.
      const double horizon =
          std::max(earliest_start, timetable.get_max_time()) + std::max(step, 1e-3);
      while (check_time <= horizon + 1e-9)
      {
        CollisionInfo col_info = check_trajectory_motion_and_terminal_hold_detailed(
            candidate_traj, check_time, timetable, params, nullptr, nullptr, nullptr);
        if (col_info.is_valid)
        {
          double result_time = check_time;
          if (check_time - earliest_start > 1e-9)
          {
            const double buffered_start = check_time + 0.5;
            CollisionInfo buffered_info = check_trajectory_motion_and_terminal_hold_detailed(
                candidate_traj, buffered_start, timetable, params, nullptr, nullptr, nullptr);
            if (buffered_info.is_valid)
            {
              result_time = buffered_start;
            }
          }
          return result_time;
        }
        check_time += step;
      }
      return -1.0;
    };

    const double expected_start_time = compute_expected_uncached();
    if (expected_start_time < 0.0)
    {
      std::cerr << "    Test scenario sanity check failed: hand-rolled uncached "
                   "search itself found no safe start time.\n";
      return false;
    }

    Trajectory scheduled_traj = candidate_traj;
    double out_wait_added = 0.0;
    CollisionInfo out_last_collision;
    double out_last_check_time = 0.0;
    const double actual_start_time = find_safe_start_time(
        &scheduled_traj, earliest_start, timetable, params, store.entities, options,
        &out_wait_added, &out_last_collision, &out_last_check_time,
        IdleBlockerRelocationPolicy::RelocateAnyIdle, nullptr);

    if (!near(actual_start_time, expected_start_time, 1e-6))
    {
      std::cerr << "    find_safe_start_time returned " << actual_start_time
                << ", hand-rolled uncached path expected " << expected_start_time
                << ".\n";
      return false;
    }

    // Sanity: several early candidates must actually have failed (on the
    // hold, or on motion arriving into an occupied goal -- either way
    // find_safe_start_time treats it as "wait longer"), otherwise this test
    // would not be exercising find_safe_start_time's cache-reuse loop at all.
    if (actual_start_time < earliest_start + 2.0)
    {
      std::cerr << "    Scenario sanity check failed: expected several early "
                   "candidates to fail before succeeding, but find_safe_start_time "
                   "returned "
                << actual_start_time << ".\n";
      return false;
    }

    return true;
  }

  bool test_rearranged_object_is_static_obstacle()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *robot1 = store.add_robot("robot1", Pose(1.005, 2.0, 0.0));
    RobotMeta *robot2 = store.add_robot("robot2", Pose(0.5, 2.0, 0.0));
    ObjectMeta *obj = store.add_object("obj_goal", Pose(1.4, 2.0, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory push_to_goal;
    push_to_goal.entity = robot1;
    push_to_goal.transferred_object = obj;
    push_to_goal.start_time = 0.0;
    push_to_goal.is_transfer = true;
    push_to_goal.waypoints = {make_waypoint(1.005, 2.0, 0.0, 0.0),
                              make_waypoint(1.805, 2.0, 0.0, 4.0)};
    timetable.add_trajectory(push_to_goal);

    Trajectory clear_robot1;
    clear_robot1.entity = robot1;
    clear_robot1.start_time = 4.0;
    clear_robot1.is_transfer = false;
    clear_robot1.waypoints = {make_waypoint(1.805, 2.0, 0.0, 0.0),
                              make_waypoint(1.0, 0.5, -M_PI / 2.0, 4.0)};
    timetable.add_trajectory(clear_robot1);

    const Pose obj_pose_after = timetable.get_pose(obj, 4.0);
    const Pose obj_pose_late = timetable.get_pose(obj, 30.0);
    if (!near(obj_pose_after.x, obj_pose_late.x) ||
        !near(obj_pose_after.y, obj_pose_late.y) ||
        !near(obj_pose_after.yaw, obj_pose_late.yaw, kYawEps))
    {
      std::cerr << "    Rearranged object does not stay static.\n";
      return false;
    }

    const double start_time = 9.0;
    robot2->initial_pose = timetable.get_pose(robot2, start_time);
    PHAStar planner(robot2, obj_pose_after, &timetable, &store.entities, params,
                    false, "", start_time);
    const auto res = planner.Planning_with_res(start_time);

    if (res.status != PlanningStatus::GOAL_INVALID_COLLISION)
    {
      std::cerr << "    Expected GOAL_INVALID_COLLISION, got status "
                << static_cast<int>(res.status) << "\n";
      return false;
    }
    if (res.colliding_entity != "obj_goal")
    {
      std::cerr << "    Expected collision with obj_goal, got "
                << res.colliding_entity << "\n";
      return false;
    }

    maybe_show_results("Rearranged object becomes static obstacle", timetable,
                       store.entities, {push_to_goal, clear_robot1}, params);
    return true;
  }

  bool test_safe_parking_pose_safe_until_last_timestamp()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *blocker = store.add_robot("robot_blocker", Pose(2.0, 2.6, 0.0));
    RobotMeta *mover = store.add_robot("robot_mover", Pose(3.2, 2.6, M_PI));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory mover_traj;
    mover_traj.entity = mover;
    mover_traj.start_time = 0.0;
    mover_traj.is_transfer = false;
    mover_traj.waypoints = {make_waypoint(3.2, 2.6, M_PI, 0.0),
                            make_waypoint(1.2, 2.6, M_PI, 20.0)};
    timetable.add_trajectory(mover_traj);

    const std::vector<Pose> candidates = {
        Pose(2.2, 2.6, 0.0), Pose(0.6, 4.8, M_PI / 2.0), Pose(0.6, 0.4, 0.0)};
    const double start_time = 0.0;

    auto parking = find_safe_parking_candidate(blocker, start_time, candidates,
                                               timetable, store.entities, params);
    if (!parking.found)
    {
      std::cerr << "    Failed to find a safe parking candidate.\n";
      return false;
    }

    const double arrival_time = parking.result.waypoints.back().time;
    if (!parked_pose_is_safe_until_last_timestamp(
            blocker, parking.chosen_pose, arrival_time, start_time, timetable,
            store.entities, params))
    {
      std::cerr << "    Found parking pose is not safe until last timestamp.\n";
      return false;
    }

    Trajectory parking_traj;
    parking_traj.entity = blocker;
    parking_traj.start_time = start_time;
    parking_traj.is_transfer = false;
    parking_traj.waypoints = parking.result.waypoints;

    TimeTable viz_timetable = timetable;
    viz_timetable.add_trajectory(parking_traj);
    maybe_show_results("Safe parking pose remains safe until last timestamp",
                       viz_timetable, store.entities, {mover_traj, parking_traj},
                       params);
    return true;
  }

  bool test_safe_parking_trajectory_collision_free()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *blocker = store.add_robot("robot_blocker", Pose(2.0, 2.6, 0.0));
    RobotMeta *mover = store.add_robot("robot_mover", Pose(3.2, 2.6, M_PI));
    store.add_object("obj_static", Pose(1.8, 4.1, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory mover_traj;
    mover_traj.entity = mover;
    mover_traj.start_time = 0.0;
    mover_traj.is_transfer = false;
    mover_traj.waypoints = {make_waypoint(3.2, 2.6, M_PI, 0.0),
                            make_waypoint(1.2, 2.6, M_PI, 20.0)};
    timetable.add_trajectory(mover_traj);

    const std::vector<Pose> candidates = {
        Pose(2.2, 2.6, 0.0), Pose(0.6, 4.8, M_PI / 2.0), Pose(0.6, 0.4, 0.0)};
    const double start_time = 0.0;

    auto parking = find_safe_parking_candidate(blocker, start_time, candidates,
                                               timetable, store.entities, params);
    if (!parking.found)
    {
      std::cerr << "    Failed to find a safe parking candidate.\n";
      return false;
    }

    if (!path_is_collision_free(blocker, parking.result.waypoints, timetable,
                                store.entities, params))
    {
      std::cerr << "    Safe parking trajectory has collisions.\n";
      return false;
    }

    Trajectory parking_traj;
    parking_traj.entity = blocker;
    parking_traj.start_time = start_time;
    parking_traj.is_transfer = false;
    parking_traj.waypoints = parking.result.waypoints;

    TimeTable viz_timetable = timetable;
    viz_timetable.add_trajectory(parking_traj);
    maybe_show_results("Safe parking trajectory is collision-free",
                       viz_timetable, store.entities,
                       {mover_traj, parking_traj}, params);
    return true;
  }

  bool test_expand_safe_parking_mode_finds_feasible_candidate()
  {
    Params params = make_push_demo_params();
    EntityStore store;
    RobotMeta *blocker = store.add_robot("robot_blocker", Pose(2.0, 2.6, 0.0));
    RobotMeta *mover = store.add_robot("robot_mover", Pose(3.2, 2.6, M_PI));
    store.add_object("obj_static", Pose(1.8, 4.1, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory mover_traj;
    mover_traj.entity = mover;
    mover_traj.start_time = 0.0;
    mover_traj.is_transfer = false;
    mover_traj.waypoints = {make_waypoint(3.2, 2.6, M_PI, 0.0),
                            make_waypoint(1.2, 2.6, M_PI, 20.0)};
    timetable.add_trajectory(mover_traj);

    const double start_time = 0.0;
    const Pose start_pose = timetable.get_pose(blocker, start_time);

    auto expand_candidates = generate_expand_parking_candidates_for_test(
        start_pose, blocker, params, 0.5, 12, 300);
    auto expand_candidates_again = generate_expand_parking_candidates_for_test(
        start_pose, blocker, params, 0.5, 12, 300);

    if (expand_candidates.empty())
    {
      std::cerr << "    Expand mode generated no parking candidates.\n";
      return false;
    }
    if (expand_candidates.size() != expand_candidates_again.size())
    {
      std::cerr << "    Expand mode candidate generation is not deterministic in size.\n";
      return false;
    }
    for (size_t i = 0; i < expand_candidates.size(); ++i)
    {
      if (!near(expand_candidates[i].x, expand_candidates_again[i].x, 1e-6) ||
          !near(expand_candidates[i].y, expand_candidates_again[i].y, 1e-6) ||
          !near(expand_candidates[i].yaw, expand_candidates_again[i].yaw, 1e-6))
      {
        std::cerr << "    Expand mode candidate generation is not deterministic in order.\n";
        return false;
      }
    }

    auto parking = find_safe_parking_candidate(
        blocker, start_time, expand_candidates, timetable, store.entities, params);
    if (!parking.found)
    {
      std::cerr << "    Expand mode failed to find a feasible safe parking candidate.\n";
      return false;
    }

    if (std::hypot(parking.chosen_pose.x - start_pose.x,
                   parking.chosen_pose.y - start_pose.y) < 0.25)
    {
      std::cerr << "    Expand mode chose a parking pose too close to start.\n";
      return false;
    }

    if (!path_is_collision_free(blocker, parking.result.waypoints, timetable,
                                store.entities, params))
    {
      std::cerr << "    Expand mode parking trajectory has collisions.\n";
      return false;
    }

    const double arrival_time = parking.result.waypoints.back().time;
    if (!parked_pose_is_safe_until_last_timestamp(
            blocker, parking.chosen_pose, arrival_time, start_time, timetable,
            store.entities, params))
    {
      std::cerr << "    Expand mode parking pose is not safe until last timestamp.\n";
      return false;
    }

    Trajectory parking_traj;
    parking_traj.entity = blocker;
    parking_traj.start_time = start_time;
    parking_traj.is_transfer = false;
    parking_traj.waypoints = parking.result.waypoints;

    TimeTable viz_timetable = timetable;
    viz_timetable.add_trajectory(parking_traj);
    maybe_show_results("Expand safe parking mode finds feasible candidate",
                       viz_timetable, store.entities,
                       {mover_traj, parking_traj}, params);
    return true;
  }

  bool test_clear_hint_validates_real_scheduling_window()
  {
    Params params = make_push_demo_params();
    EntityStore store;

    RobotMeta *blocker =
        store.add_mars_runtime_robot("blocker", Pose(0.3, 0.3, 0.0));
    RobotMeta *hint_robot =
        store.add_mars_runtime_robot("hint_robot", Pose(1.2, 2.0, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    // Blocker holds its original pose, then relocates and settles onto the
    // candidate parking pose only late in the schedule (mirrors the
    // reported bug: relocations typically land 150-300s into the
    // simulation, not at t=0).
    constexpr double kRealWindow = 200.0;
    const Pose candidate_pose(2.0, 2.0, 0.0);

    Trajectory hold_at_origin;
    hold_at_origin.entity = blocker;
    hold_at_origin.start_time = 0.0;
    hold_at_origin.is_transfer = false;
    hold_at_origin.waypoints = {make_waypoint(0.3, 0.3, 0.0, 0.0),
                                make_waypoint(0.3, 0.3, 0.0, kRealWindow - 1.0)};
    timetable.add_trajectory(hold_at_origin);

    Trajectory arrive_at_candidate;
    arrive_at_candidate.entity = blocker;
    arrive_at_candidate.start_time = kRealWindow - 1.0;
    arrive_at_candidate.is_transfer = false;
    arrive_at_candidate.waypoints = {
        make_waypoint(0.3, 0.3, 0.0, 0.0),
        make_waypoint(candidate_pose.x, candidate_pose.y, candidate_pose.yaw,
                      1.0)};
    timetable.add_trajectory(arrive_at_candidate);

    // blocked_traj_hint: a segment trajectory with an unassigned start_time
    // (-1), exactly as ReloPushPath2TrajPtr leaves it before scheduling
    // succeeds (Task.h). Its (already-relative) waypoints cut straight
    // through candidate_pose.
    Trajectory hint;
    hint.entity = hint_robot;
    hint.start_time = -1.0;
    hint.is_transfer = false;
    hint.waypoints = {make_waypoint(1.2, 2.0, 0.0, 0.0),
                      make_waypoint(candidate_pose.x, candidate_pose.y,
                                    candidate_pose.yaw, 1.0),
                      make_waypoint(2.8, 2.0, 0.0, 2.0)};

    // Sub-case 1 (fixed behavior): validated at the real scheduling window,
    // the blocker is sitting on the hint's path -> must be rejected.
    CollisionInfo collision_fixed;
    double check_time_fixed = 0.0;
    const bool clears_with_fix = parking_candidate_clears_blocked_hint(
        &hint, blocker, timetable, params, &collision_fixed, &check_time_fixed,
        kRealWindow);
    if (clears_with_fix)
    {
      std::cerr << "    Candidate was accepted even though the blocker "
                   "occupies the hint's path during the real scheduling "
                   "window (t="
                << kRealWindow << "s); hint_reference_time was not honored.\n";
      return false;
    }

    // Confirm this is genuinely the reported bug: with no reference time
    // available (the only behavior possible before hint_reference_time
    // existed), the legacy fallback derives its window from the hint's own
    // near-zero waypoint time and wrongly accepts the candidate because the
    // blocker hasn't relocated there yet at t=0.
    CollisionInfo collision_prefix;
    double check_time_prefix = 0.0;
    const bool clears_prefix_logic = parking_candidate_clears_blocked_hint(
        &hint, blocker, timetable, params, &collision_prefix, &check_time_prefix,
        -1.0);
    if (!clears_prefix_logic)
    {
      std::cerr << "    Expected the pre-fix (no reference time) code path "
                   "to still wrongly accept this candidate; scenario no "
                   "longer isolates the original bug.\n";
      return false;
    }

    // Sub-case 2: a candidate that genuinely clears the hint's path, even
    // when validated at the real scheduling window, must still be accepted.
    const Pose clear_candidate_pose(0.6, 4.6, 0.0);
    TimeTable clear_timetable(0.5);
    clear_timetable.add_initial(store.entities);
    clear_timetable.add_trajectory(hold_at_origin);

    Trajectory arrive_at_clear_candidate;
    arrive_at_clear_candidate.entity = blocker;
    arrive_at_clear_candidate.start_time = kRealWindow - 1.0;
    arrive_at_clear_candidate.is_transfer = false;
    arrive_at_clear_candidate.waypoints = {
        make_waypoint(0.3, 0.3, 0.0, 0.0),
        make_waypoint(clear_candidate_pose.x, clear_candidate_pose.y,
                      clear_candidate_pose.yaw, 1.0)};
    clear_timetable.add_trajectory(arrive_at_clear_candidate);

    CollisionInfo collision_clear;
    double check_time_clear = 0.0;
    const bool clears_when_actually_clear = parking_candidate_clears_blocked_hint(
        &hint, blocker, clear_timetable, params, &collision_clear,
        &check_time_clear, kRealWindow);
    if (!clears_when_actually_clear)
    {
      std::cerr << "    A candidate that genuinely clears the hint's path "
                   "during the real scheduling window was incorrectly "
                   "rejected.\n";
      return false;
    }

    return true;
  }

  bool test_replan_transit_segment_after_failed_schedule_avoids_new_object()
  {
    Params params = make_push_demo_params();
    RuntimeOptions options; // defaults: initial_transit_methods, etc.

    EntityStore store;
    RobotMeta *robot =
        store.add_mars_runtime_robot("robot1", Pose(0.4, 2.6, 0.0));
    // Committed directly on the straight-line path the stale geometry below
    // assumes was still clear -- mimics a concurrently-delivered object
    // permanently occupying a connector path planned against an earlier
    // world snapshot.
    store.add_object("obj_new_delivery", Pose(2.0, 2.6, 0.0));

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory traj;
    traj.entity = robot;
    traj.start_time = 0.0;
    traj.is_transfer = false;
    traj.kind = TrajectoryKind::TRANSIT;
    traj.waypoints = {make_waypoint(0.4, 2.6, 0.0, 0.0),
                      make_waypoint(3.6, 2.6, 0.0, 16.0)};

    if (path_is_collision_free(robot, traj.waypoints, timetable, store.entities,
                               params))
    {
      std::cerr << "    Test setup invalid: stale straight-line path does "
                   "not actually collide with obj_new_delivery.\n";
      return false;
    }

    std::string failure_reason;
    const bool replanned = replan_transit_segment_after_failed_schedule(
        &traj, robot, 0.0, timetable, store.entities, params, options,
        &failure_reason);

    if (!replanned)
    {
      std::cerr << "    Transit replanning after failed schedule did not "
                   "find a path around the newly-delivered object ("
                << failure_reason << ").\n";
      return false;
    }

    if (traj.is_transfer || traj.waypoints.size() < 2)
    {
      std::cerr << "    Replanned segment is not a valid transit path.\n";
      return false;
    }

    if (!path_is_collision_free(robot, traj.waypoints, timetable, store.entities,
                                params))
    {
      std::cerr << "    Replanned segment still collides with the "
                   "newly-delivered object.\n";
      return false;
    }

    maybe_show_results(
        "Transit segment replans around newly-delivered object", timetable,
        store.entities, {traj}, params);
    return true;
  }

  bool test_anchor_first_contact_segment_regression()
  {
    const std::filesystem::path exe_dir = g_test_executable_path.parent_path();
    const std::filesystem::path demo_path = exe_dir / "phastar_push_demo";
    if (!std::filesystem::exists(demo_path))
    {
      std::cerr << "    Missing phastar_push_demo executable at "
                << demo_path << "\n";
      return false;
    }

    const std::filesystem::path instance_path =
        std::filesystem::path(CMAKE_SOURCE_DIR) /
        "results/relopush-out/result_seq_ReloPush-BOSS_10_objects.txt_ind77.b64";
    if (!std::filesystem::exists(instance_path))
    {
      std::cerr << "    Missing regression instance file at "
                << instance_path << "\n";
      return false;
    }

    std::string command =
        demo_path.string() +
        " --no-visualization --no-visualize-relopush-plan --no-debug-vis"
        " --assignment-search-iters=0 --local-sequence-search-iters=0"
        " --shuffle-sequence-search-iters=0 --lns-iters=0 --random-seed=1"
        " --input-sequence=" + instance_path.string() + " 2>&1";

    int exit_code = 0;
    const std::string output = run_command_capture(command, exit_code);
    if (exit_code != 0)
    {
      std::cerr << "    Demo regression command failed with exit code "
                << exit_code << "\n";
      return false;
    }

    const std::string anchor_marker =
        "[PHAStar] Contact-start segment transit: trying anchor-only search before Reference E-Graph.";
    if (output.find(anchor_marker) == std::string::npos)
    {
      std::cerr << "    Anchor-first contact-start segment log was not found.\n";
      return false;
    }

    const std::filesystem::path diag_path =
        std::filesystem::path(CMAKE_SOURCE_DIR) /
        "diagnostics/planner_diagnostics_task1_b1_segment2_robot2.txt";
    if (!std::filesystem::exists(diag_path))
    {
      std::cerr << "    Expected task-1 diagnostic file was not generated.\n";
      return false;
    }

    const std::string diag_output = read_text_file(diag_path);
    if (diag_output.empty())
    {
      std::cerr << "    Failed to read task-1 diagnostic file.\n";
      return false;
    }

    const std::string selected_stage =
        "primary Hybrid A* (anchor-first contact): SUCCESS";
    if (diag_output.find(selected_stage) == std::string::npos)
    {
      std::cerr << "    Task-1 segment regression did not use the expected anchor-first success path.\n";
      return false;
    }

    const std::string task1_success = "[Assign] SUCCESS with robot2";
    if (output.find(task1_success) == std::string::npos)
    {
      std::cerr << "    Task 1 did not complete successfully for robot2 in the regression instance.\n";
      return false;
    }

    return true;
  }

  bool test_task5_b8_boundary_contact_repro()
  {
    constexpr double kStartTime = 115.415;
    const Pose start_pose(1.893, 0.562, 3.695);
    const Pose source_clearance_goal(1.025, 0.088, 0.498);
    constexpr double kMarsPrePushDistance = 0.445;
    const Pose b8_pose(1.424, 0.304, 0.498);
    const Pose mars_goal(
        b8_pose.x - kMarsPrePushDistance * std::cos(source_clearance_goal.yaw),
        b8_pose.y - kMarsPrePushDistance * std::sin(source_clearance_goal.yaw),
        source_clearance_goal.yaw);

    // Exact reduced repro from result_seq_ReloPush-BOSS_10_objects.txt_ind88.b64,
    // task 5 / b8 / segment 2 diagnostics.

    struct Trial
    {
      std::string name;
      Pose goal;
      Params params;
      int max_iterations;
    };

    const std::vector<Trial> trials = {
        {"current MARS goal + current fine params",
         mars_goal, make_task5_b8_boundary_params(), 10000},
        {"source-clearance goal + current fine params",
         source_clearance_goal, make_task5_b8_boundary_params(), 10000},
        {"current MARS goal + ReloPush-like primitive params",
         mars_goal, make_relopush_like_task5_b8_params(), 20000},
        {"source-clearance goal + ReloPush-like primitive params",
         source_clearance_goal, make_relopush_like_task5_b8_params(), 20000},
        {"current MARS goal + contact-boundary geometric params",
         mars_goal, make_contact_boundary_task5_b8_params(), 30000},
        {"source-clearance goal + contact-boundary geometric params",
         source_clearance_goal, make_contact_boundary_task5_b8_params(), 30000},
    };

    bool any_success = false;
    for (const auto &trial : trials)
    {
      EntityStore store;
      RobotMeta *robot = store.add_mars_runtime_robot("robot1", start_pose);
      store.add_object("b8", b8_pose);

      TimeTable timetable(0.5);
      timetable.add_initial(store.entities);
      robot->initial_pose = timetable.get_pose(robot, kStartTime);

      PHAStar planner(robot, trial.goal, &timetable, &store.entities,
                      trial.params, false, "", kStartTime,
                      "task5-b8-boundary-contact-repro", "b8");
      planner.max_search_iterations = trial.max_iterations;
      planner.set_ignore_other_robots(true);
      planner.set_planner_expansion_threads(1);

      const auto res = planner.Planning_with_res(kStartTime);
      std::cout << "    [Task5/b8 repro] " << trial.name << "\n";
      print_planning_stats(res);

      if (res.status == PlanningStatus::SUCCESS)
      {
        any_success = true;
        Trajectory planned;
        planned.entity = robot;
        planned.start_time = kStartTime;
        planned.is_transfer = false;
        planned.approach_goal_entity = store.entities["b8"];
        planned.waypoints = res.waypoints;
        maybe_show_results("Task 5 b8 boundary/contact reduced repro",
                           timetable, store.entities, {planned}, trial.params);
      }
    }

    if (!any_success)
    {
      std::cerr << "    None of the task-5/b8 reduced repro trials found a path.\n";
      return false;
    }

    return true;
  }

  double huber_loss_for_test(double pred, double target, double huber_delta)
  {
    const double err = pred - target;
    const double aerr = std::abs(err);
    if (aerr <= huber_delta)
      return 0.5 * err * err;
    return huber_delta * (aerr - 0.5 * huber_delta);
  }

  // Central-difference gradient check for QModel::accumulate_grad against the
  // Huber loss it is meant to implement, for both the linear (hidden==0) and
  // 1-hidden-layer MLP (hidden>0) configurations. huber_delta is set huge so
  // the loss stays a smooth quadratic-in-error function (no clip kink), which
  // keeps the finite-difference comparison clean.
  bool test_qmodel_gradient_check()
  {
    auto check_for_hidden = [](std::size_t dim, std::size_t hidden, unsigned seed) -> bool
    {
      std::mt19937 rng(seed);
      QModel model(dim, hidden, rng);

      // Nudge params off the (possibly all-zero) initialization so the check
      // exercises the model away from a degenerate point.
      std::uniform_real_distribution<double> param_dist(-0.5, 0.5);
      std::vector<double> init_params = model.get_params();
      for (auto &v : init_params)
        v = param_dist(rng);
      model.set_params(init_params);

      std::uniform_real_distribution<double> x_dist(-1.0, 1.0);
      std::vector<double> x(dim);
      for (auto &v : x)
        v = x_dist(rng);
      const double target = x_dist(rng);

      constexpr double huber_delta = 1e9;

      std::vector<double> grad(model.num_params(), 0.0);
      model.accumulate_grad(x, target, huber_delta, grad);

      const std::vector<double> base_params = model.get_params();
      constexpr double eps = 1e-6;
      double max_rel_err = 0.0;
      for (std::size_t p = 0; p < base_params.size(); ++p)
      {
        std::vector<double> perturbed = base_params;

        perturbed[p] = base_params[p] + eps;
        model.set_params(perturbed);
        const double loss_plus =
            huber_loss_for_test(model.predict(x), target, huber_delta);

        perturbed[p] = base_params[p] - eps;
        model.set_params(perturbed);
        const double loss_minus =
            huber_loss_for_test(model.predict(x), target, huber_delta);

        model.set_params(base_params);

        const double numeric_grad = (loss_plus - loss_minus) / (2.0 * eps);
        const double denom = std::max(1.0, std::abs(numeric_grad));
        const double rel_err = std::abs(numeric_grad - grad[p]) / denom;
        max_rel_err = std::max(max_rel_err, rel_err);
      }

      if (max_rel_err >= 1e-4)
      {
        std::cerr << "    QModel gradient check failed for dim=" << dim
                  << " hidden=" << hidden << ": max_rel_err=" << max_rel_err << "\n";
        return false;
      }
      return true;
    };

    if (!check_for_hidden(5, 8, 42))
      return false;
    if (!check_for_hidden(5, 0, 43))
      return false;
    return true;
  }

  // Demonstrates that the 1-hidden-layer MLP can fit a nonlinear (XOR-like)
  // target that a linear model could not, via repeated accumulate_grad +
  // apply_grad minibatch updates.
  bool test_qmodel_mlp_fits_nonlinear_target()
  {
    std::mt19937 rng(7);
    QModel model(2, 8, rng);

    struct Sample
    {
      double x0;
      double x1;
      double y;
    };
    const std::vector<Sample> data = {
        {0.0, 0.0, -1.0},
        {0.0, 1.0, 1.0},
        {1.0, 0.0, 1.0},
        {1.0, 1.0, -1.0},
    };

    auto mse = [&]()
    {
      double sum = 0.0;
      for (const auto &s : data)
      {
        const double pred = model.predict({s.x0, s.x1});
        const double err = pred - s.y;
        sum += err * err;
      }
      return sum / static_cast<double>(data.size());
    };

    const double initial_mse = mse();

    constexpr double lr = 0.05;
    constexpr double huber_delta = 1e9; // effectively plain squared error here
    constexpr int iterations = 2000;
    for (int it = 0; it < iterations; ++it)
    {
      std::vector<double> grad(model.num_params(), 0.0);
      for (const auto &s : data)
        model.accumulate_grad({s.x0, s.x1}, s.y, huber_delta, grad);
      model.apply_grad(grad, data.size(), lr, 0.0);
    }

    const double final_mse = mse();
    std::cout << "    [QModel fit] initial MSE=" << initial_mse
              << " final MSE=" << final_mse << "\n";

    if (!(final_mse < 0.25 * initial_mse))
    {
      std::cerr << "    QModel MLP did not learn nonlinear XOR-like function: "
                << "final MSE " << final_mse << " not < 25% of initial "
                << initial_mse << "\n";
      return false;
    }
    return true;
  }

  // QModel::save/load round-trip, for both the linear (hidden==0) and
  // 1-hidden-layer MLP (hidden>0) configurations. setprecision(17) should
  // round-trip doubles essentially exactly, but compares with a tight epsilon
  // rather than exact `==` to be safe against any formatting/parsing rounding.
  bool test_qmodel_save_load_roundtrip()
  {
    auto check_for_hidden = [](std::size_t dim, std::size_t hidden, unsigned seed) -> bool
    {
      const auto tmp_path = std::filesystem::temp_directory_path() /
                           ("phastar_unit_tests_qmodel_" + std::to_string(hidden) + ".weights");
      std::filesystem::remove(tmp_path);

      std::mt19937 rng(seed);
      QModel model(dim, hidden, rng);
      std::uniform_real_distribution<double> param_dist(-1.0, 1.0);
      std::vector<double> params = model.get_params();
      for (auto &v : params)
        v = param_dist(rng);
      model.set_params(params);

      if (!model.save(tmp_path.string()))
      {
        std::cerr << "    QModel::save failed for dim=" << dim << " hidden=" << hidden << "\n";
        return false;
      }

      std::mt19937 rng2(seed + 1);
      QModel loaded(dim, hidden, rng2);
      if (!loaded.load(tmp_path.string()))
      {
        std::cerr << "    QModel::load failed for dim=" << dim << " hidden=" << hidden << "\n";
        std::filesystem::remove(tmp_path);
        return false;
      }
      std::filesystem::remove(tmp_path);

      const std::vector<double> loaded_params = loaded.get_params();
      if (loaded_params.size() != params.size())
      {
        std::cerr << "    QModel round-trip param count mismatch: expected "
                  << params.size() << " got " << loaded_params.size() << "\n";
        return false;
      }
      for (std::size_t i = 0; i < params.size(); ++i)
      {
        if (std::abs(params[i] - loaded_params[i]) > 1e-12)
        {
          std::cerr << "    QModel round-trip param mismatch at index " << i
                    << ": expected " << params[i] << " got " << loaded_params[i] << "\n";
          return false;
        }
      }
      return true;
    };

    if (!check_for_hidden(5, 0, 101))
      return false;
    if (!check_for_hidden(5, 8, 102))
      return false;
    return true;
  }

  // A load() into a QModel with a different dim/hidden than the saved file
  // must fail gracefully (return false, no crash) and must not modify the
  // target model's existing params.
  bool test_qmodel_load_dim_hidden_mismatch()
  {
    const auto tmp_path = std::filesystem::temp_directory_path() /
                         "phastar_unit_tests_qmodel_mismatch.weights";
    std::filesystem::remove(tmp_path);

    std::mt19937 rng(201);
    QModel source(5, 0, rng);
    std::uniform_real_distribution<double> param_dist(-1.0, 1.0);
    std::vector<double> source_params = source.get_params();
    for (auto &v : source_params)
      v = param_dist(rng);
    source.set_params(source_params);
    if (!source.save(tmp_path.string()))
    {
      std::cerr << "    Failed to save source QModel for mismatch test.\n";
      return false;
    }

    std::mt19937 rng2(202);
    QModel target(5, 8, rng2); // different hidden -> dim/hidden mismatch
    const std::vector<double> target_params_before = target.get_params();

    if (target.load(tmp_path.string()))
    {
      std::cerr << "    QModel::load unexpectedly succeeded on a dim/hidden mismatch.\n";
      std::filesystem::remove(tmp_path);
      return false;
    }
    std::filesystem::remove(tmp_path);

    const std::vector<double> target_params_after = target.get_params();
    if (target_params_after != target_params_before)
    {
      std::cerr << "    QModel params were modified after a failed (mismatched) load.\n";
      return false;
    }
    return true;
  }

  // load() from a path that does not exist must fail gracefully (return
  // false, no crash/exception) and leave the model's params untouched.
  bool test_qmodel_load_missing_file()
  {
    const auto tmp_path = std::filesystem::temp_directory_path() /
                         "phastar_unit_tests_qmodel_missing_does_not_exist.weights";
    std::filesystem::remove(tmp_path);

    std::mt19937 rng(301);
    QModel model(5, 0, rng);
    const std::vector<double> params_before = model.get_params();

    if (model.load(tmp_path.string()))
    {
      std::cerr << "    QModel::load unexpectedly succeeded on a missing file.\n";
      return false;
    }

    const std::vector<double> params_after = model.get_params();
    if (params_after != params_before)
    {
      std::cerr << "    QModel params were modified after a failed (missing file) load.\n";
      return false;
    }
    return true;
  }

// ==========================================
// DQN feature-redesign v2 (DqnFeaturesV2.h) pure-math tests. These exercise
// layer B (geometry/schedule math on plain types) with synthetic data, with
// no FinalAllocation/EdgePath construction needed.
// ==========================================

bool test_dqn_v2_confinement_weight()
{
  const DqnV2::WorkspaceBounds bounds{0.0, 4.0, 0.0, 5.0};

  // Interior point: clearance >= kClearRef -> confinement weight bottoms out at 1.0.
  const ReloPush::State interior(2.0, 2.5, 0.0);
  const double clr_interior = DqnV2::workspace_clearance(interior, bounds);
  const double conf_interior = DqnV2::confinement_weight(clr_interior);
  if (!(clr_interior >= DqnV2::kClearRef) || std::abs(conf_interior - 1.0) > 1e-9)
  {
    std::cerr << "    interior clearance=" << clr_interior << " conf=" << conf_interior << "\n";
    return false;
  }

  // Near-wall point: clearance = 0.1 -> conf = 1 + (1 - 0.1/0.5) = 1.8.
  const ReloPush::State near_wall(0.1, 2.5, 0.0);
  const double clr_wall = DqnV2::workspace_clearance(near_wall, bounds);
  const double conf_wall = DqnV2::confinement_weight(clr_wall);
  if (std::abs(clr_wall - 0.1) > 1e-9 || std::abs(conf_wall - 1.8) > 1e-9)
  {
    std::cerr << "    near-wall clearance=" << clr_wall << " conf=" << conf_wall << "\n";
    return false;
  }
  return true;
}

bool test_dqn_v2_blockage_weight()
{
  const DqnV2::WorkspaceBounds bounds{0.0, 10.0, 0.0, 10.0};
  const std::vector<ReloPush::State> corridor = {
      ReloPush::State(1.0, 5.0, 0.0),
      ReloPush::State(2.0, 5.0, 0.0),
      ReloPush::State(3.0, 5.0, 0.0),
  };
  const double block_dist = 0.5;

  const ReloPush::State near_pose(2.1, 5.1, 0.0);
  const double w_near = DqnV2::blockage_weight(near_pose, corridor, block_dist, bounds);
  if (!(w_near >= 1.0 && w_near <= 2.0))
  {
    std::cerr << "    expected in-range blockage weight, got " << w_near << "\n";
    return false;
  }

  const ReloPush::State far_pose(9.0, 9.0, 0.0);
  const double w_far = DqnV2::blockage_weight(far_pose, corridor, block_dist, bounds);
  if (w_far != 0.0)
  {
    std::cerr << "    expected zero blockage weight for a far pose, got " << w_far << "\n";
    return false;
  }
  return true;
}

bool test_dqn_v2_dag_construction()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  std::vector<TaskGeom> geoms(3);

  // Task 0: push corridor near the origin.
  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(20.0, 20.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0), State(2.0, 0.0, 0.0)};

  // Task 1's goal sits on task 0's push corridor -> hard edge 0 -> 1.
  geoms[1].obj_start = State(20.0, 20.0, 0.0);
  geoms[1].obj_goal = State(1.0, 0.02, 0.0);
  geoms[1].push_pts = {State(20.0, 0.0, 0.0), State(21.0, 0.0, 0.0), State(22.0, 0.0, 0.0)};

  // Task 2's push corridor passes over task 0's start -> hard edge 0 -> 2.
  geoms[2].obj_start = State(30.0, 30.0, 0.0);
  geoms[2].obj_goal = State(31.0, 31.0, 0.0);
  geoms[2].push_pts = {State(0.0, 0.0, 0.0), State(0.0, 1.0, 0.0), State(0.0, 2.0, 0.0)};

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry =
      DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);

  auto contains = [](const std::vector<std::size_t> &v, std::size_t x)
  { return std::find(v.begin(), v.end(), x) != v.end(); };

  if (!geometry.hard_pred[0].empty())
  {
    std::cerr << "    expected hard_pred[0] empty, size=" << geometry.hard_pred[0].size() << "\n";
    return false;
  }
  if (!(geometry.hard_pred[1].size() == 1 && contains(geometry.hard_pred[1], 0)))
  {
    std::cerr << "    expected hard_pred[1] == {0}\n";
    return false;
  }
  if (!(geometry.hard_pred[2].size() == 1 && contains(geometry.hard_pred[2], 0)))
  {
    std::cerr << "    expected hard_pred[2] == {0}\n";
    return false;
  }

  // No reverse edges: by construction every hard predecessor index is < the
  // task it precedes (the loop only ever considers x < y).
  for (std::size_t a = 0; a < geoms.size(); ++a)
    for (std::size_t p : geometry.hard_pred[a])
      if (p >= a)
      {
        std::cerr << "    reverse hard edge detected: " << p << " -> " << a << "\n";
        return false;
      }

  // A topological sort exists (Kahn-style greedy placement must fully drain).
  std::vector<char> placed(geoms.size(), 0);
  for (std::size_t step = 0; step < geoms.size(); ++step)
  {
    bool progressed = false;
    for (std::size_t a = 0; a < geoms.size(); ++a)
    {
      if (placed[a])
        continue;
      bool ready = true;
      for (std::size_t p : geometry.hard_pred[a])
        if (!placed[p])
        {
          ready = false;
          break;
        }
      if (ready)
      {
        placed[a] = 1;
        progressed = true;
        break;
      }
    }
    if (!progressed)
    {
      std::cerr << "    no topological sort exists for the hard-edge DAG\n";
      return false;
    }
  }
  return true;
}

bool test_dqn_v2_forward_sim()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  auto euclid = [](const State &a, const State &b)
  { return DqnV2::dist2d(a, b); };

  const std::vector<State> initial_poses = {State(0.0, 0.0, 0.0), State(10.0, 0.0, 0.0)};
  const double speed_transit = 1.0;
  const std::size_t task_count = 3;
  DqnV2::ForwardSim sim(initial_poses, speed_transit, euclid, task_count);

  std::vector<TaskGeom> geoms(task_count);
  geoms[0].approach = State(1.0, 0.0, 0.0);
  geoms[0].exit = State(2.0, 0.0, 0.0);
  geoms[0].tau_fixed = 5.0;

  geoms[1].approach = State(12.0, 0.0, 0.0);
  geoms[1].exit = State(13.0, 0.0, 0.0);
  geoms[1].tau_fixed = 3.0;

  geoms[2].approach = State(3.0, 0.0, 0.0);
  geoms[2].exit = State(4.0, 0.0, 0.0);
  geoms[2].tau_fixed = 2.0;

  constexpr double kTol = 1e-6;

  // Step 0: robots tie at free_time=0 -> pick lowest index (robot 0).
  auto pv0 = sim.preview(geoms[0]);
  if (pv0.robot != 0 || std::abs(pv0.start - 0.0) > kTol || std::abs(pv0.finish - 6.0) > kTol)
  {
    std::cerr << "    task0 preview mismatch: robot=" << pv0.robot
              << " start=" << pv0.start << " finish=" << pv0.finish << "\n";
    return false;
  }
  sim.commit(0, geoms[0], pv0);
  if (std::abs(sim.free_time[0] - 6.0) > kTol || std::abs(sim.free_time[1] - 0.0) > kTol)
  {
    std::cerr << "    free_time after task0 mismatch: [" << sim.free_time[0] << ", "
              << sim.free_time[1] << "]\n";
    return false;
  }

  // Step 1: robot 1 is earliest-free (0 < 6).
  const double m_t_before_task1 = sim.max_free_time();
  auto pv1 = sim.preview(geoms[1]);
  if (pv1.robot != 1 || std::abs(pv1.start - 0.0) > kTol || std::abs(pv1.finish - 5.0) > kTol)
  {
    std::cerr << "    task1 preview mismatch: robot=" << pv1.robot
              << " start=" << pv1.start << " finish=" << pv1.finish << "\n";
    return false;
  }
  const double seed_makespan = 10.0;
  const double d_makespan = std::max(0.0, pv1.finish - m_t_before_task1) / seed_makespan;
  const double slack = (m_t_before_task1 - pv1.start) / seed_makespan;
  if (std::abs(d_makespan - 0.0) > kTol || std::abs(slack - 0.6) > kTol)
  {
    std::cerr << "    task1 d_makespan/slack mismatch: d_makespan=" << d_makespan
              << " slack=" << slack << "\n";
    return false;
  }
  sim.commit(1, geoms[1], pv1);
  if (std::abs(sim.free_time[0] - 6.0) > kTol || std::abs(sim.free_time[1] - 5.0) > kTol)
  {
    std::cerr << "    free_time after task1 mismatch: [" << sim.free_time[0] << ", "
              << sim.free_time[1] << "]\n";
    return false;
  }

  // Step 2: robot 1 remains earliest-free (5 < 6).
  auto pv2 = sim.preview(geoms[2]);
  if (pv2.robot != 1 || std::abs(pv2.start - 5.0) > kTol || std::abs(pv2.finish - 17.0) > kTol)
  {
    std::cerr << "    task2 preview mismatch: robot=" << pv2.robot
              << " start=" << pv2.start << " finish=" << pv2.finish << "\n";
    return false;
  }
  sim.commit(2, geoms[2], pv2);
  return true;
}

bool test_dqn_v2_feature_vector_shape()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  const std::size_t n = 3;
  std::vector<TaskGeom> geoms(n);
  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(20.0, 20.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0)};
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(1.0, 0.0, 0.0);
  geoms[0].tau_fixed = 4.0;

  geoms[1].obj_start = State(20.0, 20.0, 0.0);
  geoms[1].obj_goal = State(1.0, 0.02, 0.0);
  geoms[1].push_pts = {State(20.0, 0.0, 0.0), State(21.0, 0.0, 0.0)};
  geoms[1].approach = State(20.0, 0.0, 0.0);
  geoms[1].exit = State(21.0, 0.0, 0.0);
  geoms[1].tau_fixed = 3.0;

  geoms[2].obj_start = State(30.0, 30.0, 0.0);
  geoms[2].obj_goal = State(31.0, 31.0, 0.0);
  geoms[2].push_pts = {State(0.0, 0.0, 0.0), State(0.0, 1.0, 0.0)};
  geoms[2].approach = State(0.0, 1.0, 0.0);
  geoms[2].exit = State(0.0, 2.0, 0.0);
  geoms[2].tau_fixed = 2.0;

  const DqnV2::WorkspaceBounds bounds{-100.0, 100.0, -100.0, 100.0};
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, 0.275);

  std::vector<char> is_placed(n, 0);
  is_placed[0] = 1; // task 0 already placed

  const LearnedOrderConstraints constraints; // empty -> learned_risk term contributes 0

  DqnV2::ForwardSim::Preview pv;
  pv.robot = 0;
  pv.start = 6.0;
  pv.finish = 12.0;
  const double m_t_before = 6.0;
  const double seed_makespan = 10.0;

  std::vector<std::pair<double, double>> committed_windows(n, {0.0, 0.0});
  committed_windows[0] = {0.0, 6.0};

  const auto phi = DqnV2::make_feature_vector_v2(
      1, pv, m_t_before, geoms[1], geometry, is_placed, committed_windows, constraints,
      seed_makespan, n);

  if (phi.size() != DqnV2::kFeatDim)
  {
    std::cerr << "    expected dim " << DqnV2::kFeatDim << ", got " << phi.size() << "\n";
    return false;
  }
  if (std::abs(phi[0] - 1.0) > 1e-12)
  {
    std::cerr << "    expected phi[0] == 1 (bias), got " << phi[0] << "\n";
    return false;
  }
  for (std::size_t i = 0; i < phi.size(); ++i)
  {
    if (!std::isfinite(phi[i]))
    {
      std::cerr << "    phi[" << i << "] is not finite: " << phi[i] << "\n";
      return false;
    }
  }
  if (phi[1] < 0.0 || phi[2] < -1e-9 || phi[6] < 0.0 || phi[6] > 1.0 + 1e-9 ||
      phi[7] < 0.0 || phi[8] < 0.0 || phi[8] > 1.0 + 1e-9)
  {
    std::cerr << "    phi values out of expected range: phi[1]=" << phi[1]
              << " phi[2]=" << phi[2] << " phi[6]=" << phi[6] << " phi[7]=" << phi[7]
              << " phi[8]=" << phi[8] << "\n";
    return false;
  }
  return true;
}

// ==========================================
// Transition logger + geometry exporter tests (see
// MARS/09rl-pretrained-study-plan.md sections 3.1/3.2).
// ==========================================

bool test_parse_family_index()
{
  std::string family;
  int index = -1;

  if (!parse_family_index(
          "/some/path/result_seq_ReloPush-BOSS_12_objects.txt_ind5.b64", family, index) ||
      family != "ReloPush-BOSS_12_objects.txt" || index != 5)
  {
    std::cerr << "    normal path parse failed: family='" << family << "' index=" << index << "\n";
    return false;
  }

  if (!parse_family_index("result_seq_Foo_Bar_Baz.txt_ind3.b64", family, index) ||
      family != "Foo_Bar_Baz.txt" || index != 3)
  {
    std::cerr << "    underscore-family parse failed: family='" << family
              << "' index=" << index << "\n";
    return false;
  }

  if (parse_family_index("not_a_matching_path.txt", family, index) ||
      family != "unknown" || index != -1)
  {
    std::cerr << "    non-matching path did not fall back gracefully: family='" << family
              << "' index=" << index << "\n";
    return false;
  }

  return true;
}

bool test_transition_logger_csv_roundtrip()
{
  const auto tmp_path = std::filesystem::temp_directory_path() /
                       "phastar_unit_tests_transition_logger.csv";
  std::filesystem::remove(tmp_path);

  auto make_entry = [](std::size_t step, std::size_t task, bool chosen)
  {
    StepCandidateEntry e;
    e.step = step;
    e.candidate_task = task;
    e.chosen = chosen;
    e.phi.assign(DqnV2::kFeatDim, static_cast<double>(task) + 0.5);
    return e;
  };

  {
    TransitionLogger logger(tmp_path.string());
    std::vector<StepCandidateEntry> step_log = {
        make_entry(0, 0, false), make_entry(0, 1, true), make_entry(1, 2, true)};
    RolloutOutcome outcome;
    outcome.feasible = true;
    outcome.makespan = 12.5;
    outcome.return_target = -1.25;
    outcome.first_fail_rank = step_log.size();
    outcome.first_failed_task = -1;
    logger.log_rollout("ReloPush-BOSS_8_objects.txt", 3, 42u, 1, step_log, outcome);
  }

  const std::string first_contents = read_text_file(tmp_path);
  std::istringstream first_stream(first_contents);
  std::string header_line;
  std::getline(first_stream, header_line);
  const std::string expected_header =
      "family,index,seed,iteration,step,candidate_task,chosen,"
      "phi0,phi1,phi2,phi3,phi4,phi5,phi6,phi7,phi8,"
      "feasible,makespan,return_target,first_fail_rank,first_failed_task";
  if (header_line != expected_header)
  {
    std::cerr << "    unexpected header: '" << header_line << "'\n";
    return false;
  }
  int first_row_count = 0;
  std::string line;
  while (std::getline(first_stream, line))
  {
    if (!line.empty())
      ++first_row_count;
  }
  if (first_row_count != 3)
  {
    std::cerr << "    expected 3 rows after first logger, got " << first_row_count << "\n";
    return false;
  }

  // A second logger on the same (now non-empty) path must append, not
  // duplicate the header.
  {
    TransitionLogger logger2(tmp_path.string());
    std::vector<StepCandidateEntry> step_log2 = {make_entry(0, 0, true)};
    RolloutOutcome outcome2;
    outcome2.feasible = false;
    outcome2.makespan = -1.0;
    outcome2.return_target = -3.0;
    outcome2.first_fail_rank = 0;
    outcome2.first_failed_task = 0;
    logger2.log_rollout("ReloPush-BOSS_8_objects.txt", 3, 42u, 2, step_log2, outcome2);
  }

  const std::string second_contents = read_text_file(tmp_path);
  std::istringstream second_stream(second_contents);
  int header_count = 0;
  int total_rows = 0;
  while (std::getline(second_stream, line))
  {
    if (line.empty())
      continue;
    if (line == expected_header)
      ++header_count;
    else
      ++total_rows;
  }
  std::filesystem::remove(tmp_path);

  if (header_count != 1)
  {
    std::cerr << "    expected exactly 1 header line after second logger, got "
              << header_count << "\n";
    return false;
  }
  if (total_rows != 4)
  {
    std::cerr << "    expected 4 total data rows after append, got " << total_rows << "\n";
    return false;
  }

  return true;
}

RobotMeta make_test_robot_meta(const std::string &name, double x, double y, double yaw)
{
  RobotMeta meta;
  meta.name = name;
  meta.type = EntityType::ROBOT;
  meta.initial_pose = Pose(x, y, yaw);
  meta.size.front_length = 0.36;
  meta.size.rear_length = 0.12;
  meta.size.width = 0.275;
  meta.min_turning_radius = 1.02;
  meta.min_turning_radius_transit = 1.02;
  meta.min_turning_radius_transfer = 1.43;
  meta.wheel_base = 0.29;
  meta.speed_transit = 0.2;
  meta.speed_transfer = 0.15;
  return meta;
}

bool test_dqn_v2_construct_order_step_log_completeness()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  const std::size_t n = 4;
  std::vector<TaskGeom> geoms(n);

  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(20.0, 20.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0)};
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(1.0, 0.0, 0.0);
  geoms[0].tau_fixed = 4.0;

  // Task 1's goal sits on task 0's push corridor -> hard edge 0 -> 1.
  geoms[1].obj_start = State(20.0, 20.0, 0.0);
  geoms[1].obj_goal = State(1.0, 0.02, 0.0);
  geoms[1].push_pts = {State(20.0, 0.0, 0.0), State(21.0, 0.0, 0.0)};
  geoms[1].approach = State(20.0, 0.0, 0.0);
  geoms[1].exit = State(21.0, 0.0, 0.0);
  geoms[1].tau_fixed = 3.0;

  // Task 2's push corridor passes over task 0's start -> hard edge 0 -> 2.
  geoms[2].obj_start = State(30.0, 30.0, 0.0);
  geoms[2].obj_goal = State(31.0, 31.0, 0.0);
  geoms[2].push_pts = {State(0.0, 1.0, 0.0), State(0.0, 2.0, 0.0)};
  geoms[2].approach = State(0.0, 1.0, 0.0);
  geoms[2].exit = State(0.0, 2.0, 0.0);
  geoms[2].tau_fixed = 2.0;

  // Task 3 has no interaction with any other task.
  geoms[3].obj_start = State(40.0, 40.0, 0.0);
  geoms[3].obj_goal = State(41.0, 41.0, 0.0);
  geoms[3].push_pts = {State(40.0, 40.0, 0.0), State(41.0, 40.0, 0.0)};
  geoms[3].approach = State(40.0, 40.0, 0.0);
  geoms[3].exit = State(41.0, 40.0, 0.0);
  geoms[3].tau_fixed = 1.0;

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry =
      DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);

  const LearnedOrderConstraints constraints; // empty -> no learned edges

  std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options; // defaults (dqn_explore_ref_bias=0.5, hard_evidence=3)
  const double seed_makespan = 10.0;
  const double epsilon = 0.5; // exercises both explore and exploit branches

  std::mt19937 rng_logged(123);
  QModel model(DqnV2::kFeatDim, 0, rng_logged);

  std::vector<std::vector<StepCandidateEntry>> step_logs;
  const auto order_logged = construct_order_v2(
      model, n, geoms, geometry, constraints, robot_metas, seed_makespan, epsilon,
      options, rng_logged, &step_logs);

  if (order_logged.size() != n || step_logs.size() != n)
  {
    std::cerr << "    order/step_logs size mismatch: order=" << order_logged.size()
              << " step_logs=" << step_logs.size() << "\n";
    return false;
  }

  // Independently recompute the legal set at each step (same mask
  // is_task_legal_v2 uses) and check step_logs[t] captured every legal
  // candidate exactly once, with exactly one entry marked chosen matching
  // order[t].
  std::vector<char> is_placed(n, 0);
  for (std::size_t t = 0; t < n; ++t)
  {
    std::vector<std::size_t> legal;
    for (std::size_t i = 0; i < n; ++i)
    {
      if (is_placed[i])
        continue;
      if (DqnV2::is_task_legal_v2(i, geometry, is_placed, constraints,
                                  options.dqn_learned_hard_evidence))
        legal.push_back(i);
    }

    if (step_logs[t].size() != legal.size())
    {
      std::cerr << "    step " << t << ": expected " << legal.size()
                << " legal candidates, step_log has " << step_logs[t].size() << "\n";
      return false;
    }

    std::size_t chosen_count = 0;
    std::size_t logged_chosen_task = n; // sentinel (out of range)
    for (const auto &entry : step_logs[t])
    {
      if (entry.step != t)
      {
        std::cerr << "    step " << t << ": entry has wrong step field " << entry.step << "\n";
        return false;
      }
      if (std::find(legal.begin(), legal.end(), entry.candidate_task) == legal.end())
      {
        std::cerr << "    step " << t << ": logged candidate " << entry.candidate_task
                  << " is not legal\n";
        return false;
      }
      if (entry.phi.size() != DqnV2::kFeatDim)
      {
        std::cerr << "    step " << t << ": phi has wrong dimension " << entry.phi.size() << "\n";
        return false;
      }
      if (entry.chosen)
      {
        ++chosen_count;
        logged_chosen_task = entry.candidate_task;
      }
    }
    if (chosen_count != 1)
    {
      std::cerr << "    step " << t << ": expected exactly 1 chosen entry, got "
                << chosen_count << "\n";
      return false;
    }
    if (logged_chosen_task != order_logged[t])
    {
      std::cerr << "    step " << t << ": chosen entry task " << logged_chosen_task
                << " != order[t] " << order_logged[t] << "\n";
      return false;
    }

    is_placed[order_logged[t]] = 1;
  }

  // Critical invariant: enabling logging must not perturb the RNG draw
  // sequence. Run again with an identically-seeded rng/model and
  // step_logs == nullptr; the resulting order must be byte-for-byte
  // identical.
  std::mt19937 rng_unlogged(123);
  QModel model_unlogged(DqnV2::kFeatDim, 0, rng_unlogged);
  const auto order_unlogged = construct_order_v2(
      model_unlogged, n, geoms, geometry, constraints, robot_metas, seed_makespan,
      epsilon, options, rng_unlogged, nullptr);

  if (order_logged != order_unlogged)
  {
    std::cerr << "    logging perturbed the RNG draw sequence: logged/unlogged orders differ\n";
    return false;
  }

  return true;
}

// ==========================================
// DQN feature-redesign v3 (explicit (task, robot) construction) tests. See
// DqnFeaturesV2.h make_feature_vector_v3/ForwardSim::preview_for/commit_for
// and DqnAllocationSearch.cpp construct_order_v3.
// ==========================================

// Shared 4-task synthetic geometry for the v3 construct_order tests below:
// task0 hard-blocks task1 and task2 (0->1, 0->2); task3 is independent.
// Identical to test_dqn_v2_construct_order_step_log_completeness's fixture
// above, so the hard-DAG structure is already validated.
std::vector<DqnV2::TaskGeom> make_v3_test_geoms()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  std::vector<TaskGeom> geoms(4);

  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(20.0, 20.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0)};
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(1.0, 0.0, 0.0);
  geoms[0].tau_fixed = 4.0;

  // Task 1's goal sits on task 0's push corridor -> hard edge 0 -> 1.
  geoms[1].obj_start = State(20.0, 20.0, 0.0);
  geoms[1].obj_goal = State(1.0, 0.02, 0.0);
  geoms[1].push_pts = {State(20.0, 0.0, 0.0), State(21.0, 0.0, 0.0)};
  geoms[1].approach = State(20.0, 0.0, 0.0);
  geoms[1].exit = State(21.0, 0.0, 0.0);
  geoms[1].tau_fixed = 3.0;

  // Task 2's push corridor passes over task 0's start -> hard edge 0 -> 2.
  geoms[2].obj_start = State(30.0, 30.0, 0.0);
  geoms[2].obj_goal = State(31.0, 31.0, 0.0);
  geoms[2].push_pts = {State(0.0, 1.0, 0.0), State(0.0, 2.0, 0.0)};
  geoms[2].approach = State(0.0, 1.0, 0.0);
  geoms[2].exit = State(0.0, 2.0, 0.0);
  geoms[2].tau_fixed = 2.0;

  // Task 3 has no interaction with any other task.
  geoms[3].obj_start = State(40.0, 40.0, 0.0);
  geoms[3].obj_goal = State(41.0, 41.0, 0.0);
  geoms[3].push_pts = {State(40.0, 40.0, 0.0), State(41.0, 40.0, 0.0)};
  geoms[3].approach = State(40.0, 40.0, 0.0);
  geoms[3].exit = State(41.0, 40.0, 0.0);
  geoms[3].tau_fixed = 1.0;

  return geoms;
}

bool test_dqn_v3_feature_vector_properties()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  const std::size_t n = 3;
  std::vector<TaskGeom> geoms(n);
  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(20.0, 20.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0)};
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(1.0, 0.0, 0.0);
  geoms[0].tau_fixed = 4.0;

  geoms[1].obj_start = State(20.0, 20.0, 0.0);
  geoms[1].obj_goal = State(1.0, 0.02, 0.0);
  geoms[1].push_pts = {State(20.0, 0.0, 0.0), State(21.0, 0.0, 0.0)};
  geoms[1].approach = State(5.0, 0.0, 0.0);
  geoms[1].exit = State(21.0, 0.0, 0.0);
  geoms[1].tau_fixed = 3.0;

  geoms[2].obj_start = State(30.0, 30.0, 0.0);
  geoms[2].obj_goal = State(31.0, 31.0, 0.0);
  geoms[2].push_pts = {State(0.0, 0.0, 0.0), State(0.0, 1.0, 0.0)};
  geoms[2].approach = State(0.0, 1.0, 0.0);
  geoms[2].exit = State(0.0, 2.0, 0.0);
  geoms[2].tau_fixed = 2.0;

  const DqnV2::WorkspaceBounds bounds{-100.0, 100.0, -100.0, 100.0};
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, 0.275);

  std::vector<char> is_placed(n, 0);
  is_placed[0] = 1;
  const LearnedOrderConstraints constraints;

  const double maxc = 1.0 / 1.02;
  const double wheel_base = 0.29;
  const double speed_transit = 1.0;
  auto rs_distance = [&](const State &a, const State &b)
  { return DqnV2::reeds_shepp_length(a, b, maxc, 0.5, wheel_base); };

  // Robot 0 gets a head start (free_time 8.0); robot 1 stays at 0.0 -> robot
  // 1 is the earliest-available robot.
  std::vector<State> initial_poses = {State(0.0, 0.0, 0.0), State(-5.0, 0.0, 0.0)};
  DqnV2::ForwardSim sim(initial_poses, speed_transit, rs_distance, n);
  sim.free_time[0] = 8.0;

  const double seed_makespan = 10.0;
  const double m_t_before = sim.max_free_time();
  const std::size_t earliest_robot = sim.pick();
  if (earliest_robot != 1)
  {
    std::cerr << "    test setup error: expected robot 1 to be earliest, got " << earliest_robot << "\n";
    return false;
  }
  const double min_free_time_before = sim.free_time[earliest_robot];

  std::vector<std::pair<double, double>> committed_windows(n, {0.0, 0.0});
  committed_windows[0] = {0.0, 6.0};

  const auto pv_earliest = sim.preview_for(earliest_robot, geoms[1]);
  const auto phi_earliest = DqnV2::make_feature_vector_v3(
      1, pv_earliest, m_t_before, min_free_time_before, geoms[1], geometry, is_placed,
      committed_windows, constraints, seed_makespan, n);

  const auto pv_other = sim.preview_for(0, geoms[1]);
  const auto phi_other = DqnV2::make_feature_vector_v3(
      1, pv_other, m_t_before, min_free_time_before, geoms[1], geometry, is_placed,
      committed_windows, constraints, seed_makespan, n);

  if (phi_earliest.size() != DqnV2::kFeatDimV3 || phi_other.size() != DqnV2::kFeatDimV3)
  {
    std::cerr << "    expected dim " << DqnV2::kFeatDimV3 << ", got " << phi_earliest.size()
              << "/" << phi_other.size() << "\n";
    return false;
  }
  for (double v : phi_earliest)
    if (!std::isfinite(v))
    {
      std::cerr << "    phi_earliest has a non-finite entry\n";
      return false;
    }
  for (double v : phi_other)
    if (!std::isfinite(v))
    {
      std::cerr << "    phi_other has a non-finite entry\n";
      return false;
    }
  if (std::abs(phi_earliest[0] - 1.0) > 1e-12 || std::abs(phi_other[0] - 1.0) > 1e-12)
  {
    std::cerr << "    expected phi[0] == 1 (bias)\n";
    return false;
  }

  // rel_avail (phi[4]): 0 for the earliest robot, > 0 for the other.
  if (std::abs(phi_earliest[4]) > 1e-9)
  {
    std::cerr << "    expected rel_avail == 0 for the earliest robot, got " << phi_earliest[4] << "\n";
    return false;
  }
  if (!(phi_other[4] > 1e-9))
  {
    std::cerr << "    expected rel_avail > 0 for the non-earliest robot, got " << phi_other[4] << "\n";
    return false;
  }

  // Complementarity dmk_plus(phi[1]) * slack(phi[2]) == 0. Both candidates
  // above land in the dmk_plus-dominant branch (pv.finish > m_t_before);
  // construct a third preview -- same robot, a near-zero-duration task --
  // that lands in the slack-dominant branch to exercise the other side too.
  if (std::abs(phi_earliest[1] * phi_earliest[2]) > 1e-12 ||
      std::abs(phi_other[1] * phi_other[2]) > 1e-12)
  {
    std::cerr << "    dmk_plus/slack complementarity violated (finish-dominant branch)\n";
    return false;
  }
  TaskGeom near_geom;
  near_geom.approach = sim.pose[earliest_robot]; // zero transit distance
  near_geom.exit = sim.pose[earliest_robot];
  near_geom.tau_fixed = 1.0; // pv.finish = 0 + 0 + 1.0 = 1.0, well below m_t_before (8.0)
  const auto pv_slack = sim.preview_for(earliest_robot, near_geom);
  const auto phi_slack = DqnV2::make_feature_vector_v3(
      1, pv_slack, m_t_before, min_free_time_before, near_geom, geometry, is_placed,
      committed_windows, constraints, seed_makespan, n);
  if (std::abs(phi_slack[1]) > 1e-12 || !(phi_slack[2] > 1e-9))
  {
    std::cerr << "    expected slack-dominant branch (dmk_plus==0, slack>0), got dmk_plus="
              << phi_slack[1] << " slack=" << phi_slack[2] << "\n";
    return false;
  }
  if (std::abs(phi_slack[1] * phi_slack[2]) > 1e-12)
  {
    std::cerr << "    dmk_plus/slack complementarity violated (slack-dominant branch)\n";
    return false;
  }

  // transit (phi[3]): robot 1 starts at (-5,0,0) facing +x; task1's approach
  // is (5,0,0), also facing +x -- colinear, same heading, so the optimal RS
  // path is a straight forward segment of length 10.0 (hand-computable as
  // the Euclidean distance). Cross-checked against the actual RS function.
  const double hand_len = 10.0;
  const double rs_len =
      DqnV2::reeds_shepp_length(State(-5.0, 0.0, 0.0), geoms[1].approach, maxc, 0.5, wheel_base);
  if (std::abs(rs_len - hand_len) > 1e-6)
  {
    std::cerr << "    sanity check failed: RS length for a straight-ahead same-heading pair "
                 "should be "
              << hand_len << ", got " << rs_len << "\n";
    return false;
  }
  const double expected_transit_over_S = hand_len / speed_transit / seed_makespan;
  if (std::abs(phi_earliest[3] - expected_transit_over_S) > 1e-6)
  {
    std::cerr << "    transit feature mismatch: expected " << expected_transit_over_S << ", got "
              << phi_earliest[3] << "\n";
    return false;
  }

  return true;
}

bool test_dqn_v3_preview_commit_for()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  auto euclid = [](const State &a, const State &b)
  { return DqnV2::dist2d(a, b); };

  // Robot 2 sits at (0,3,0); task approach at (4,0,0) -- a 3-4-5 triangle so
  // the transit distance is hand-computable exactly.
  const std::vector<State> initial_poses = {State(0.0, 0.0, 0.0), State(10.0, 0.0, 0.0),
                                            State(0.0, 3.0, 0.0)};
  const double speed_transit = 2.0;
  const std::size_t task_count = 2;
  DqnV2::ForwardSim sim(initial_poses, speed_transit, euclid, task_count);

  TaskGeom geom;
  geom.approach = State(4.0, 0.0, 0.0);
  geom.exit = State(6.0, 0.0, 0.0);
  geom.tau_fixed = 3.0;

  // Commit task 0 to robot 2 specifically -- NOT the earliest robot (all
  // three tie at free_time 0, so pick() would choose robot 0).
  const auto pv = sim.preview_for(2, geom);
  const double expected_transit = 5.0 / speed_transit; // hypot(4,3) == 5.0
  const double expected_finish = expected_transit + geom.tau_fixed;
  if (pv.robot != 2 || std::abs(pv.start - 0.0) > 1e-9 ||
      std::abs(pv.finish - expected_finish) > 1e-9)
  {
    std::cerr << "    preview_for mismatch: robot=" << pv.robot << " start=" << pv.start
              << " finish=" << pv.finish << " expected_finish=" << expected_finish << "\n";
    return false;
  }

  sim.commit_for(0, geom, pv);

  if (std::abs(sim.free_time[0] - 0.0) > 1e-9 || std::abs(sim.free_time[1] - 0.0) > 1e-9)
  {
    std::cerr << "    commit_for perturbed a non-target robot's free_time: ["
              << sim.free_time[0] << ", " << sim.free_time[1] << ", " << sim.free_time[2] << "]\n";
    return false;
  }
  if (std::abs(sim.free_time[2] - pv.finish) > 1e-9)
  {
    std::cerr << "    commit_for did not set free_time[2] to pv.finish: " << sim.free_time[2]
              << " vs " << pv.finish << "\n";
    return false;
  }
  if (!(sim.pose[0] == initial_poses[0]) || !(sim.pose[1] == initial_poses[1]))
  {
    std::cerr << "    commit_for perturbed a non-target robot's pose\n";
    return false;
  }
  if (!(sim.pose[2] == geom.exit))
  {
    std::cerr << "    commit_for did not move robot 2's pose to geom.exit\n";
    return false;
  }
  if (std::abs(sim.window[0].first - pv.start) > 1e-9 || std::abs(sim.window[0].second - pv.finish) > 1e-9)
  {
    std::cerr << "    commit_for did not record the task window correctly\n";
    return false;
  }

  return true;
}

bool test_dqn_v3_construct_order_validity()
{
  const std::size_t n = 4;
  const auto geoms = make_v3_test_geoms();

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options;
  const double seed_makespan = 10.0;
  const double epsilon = 0.5;

  std::mt19937 rng1(777);
  QModel model1(DqnV2::kFeatDimV3, 0, rng1);
  const auto oa1 = construct_order_v3(model1, n, geoms, geometry, constraints, robot_metas,
                                      seed_makespan, epsilon, options, rng1, nullptr);

  if (oa1.order.size() != n || oa1.assignment.size() != n)
  {
    std::cerr << "    order/assignment size mismatch: order=" << oa1.order.size()
              << " assignment=" << oa1.assignment.size() << "\n";
    return false;
  }

  std::vector<char> seen(n, 0);
  for (std::size_t task : oa1.order)
  {
    if (task >= n || seen[task])
    {
      std::cerr << "    order is not a permutation (task " << task << " out of range or repeated)\n";
      return false;
    }
    seen[task] = 1;
  }
  for (std::size_t r : oa1.assignment)
  {
    if (r >= robot_metas.size())
    {
      std::cerr << "    assignment robot index " << r << " out of range\n";
      return false;
    }
  }

  auto position_of = [&](std::size_t task)
  {
    return static_cast<std::size_t>(
        std::find(oa1.order.begin(), oa1.order.end(), task) - oa1.order.begin());
  };
  if (position_of(0) >= position_of(1) || position_of(0) >= position_of(2))
  {
    std::cerr << "    hard-DAG order violated: pos(0)=" << position_of(0)
              << " pos(1)=" << position_of(1) << " pos(2)=" << position_of(2) << "\n";
    return false;
  }

  // Same seed -> identical (order, assignment).
  std::mt19937 rng2(777);
  QModel model2(DqnV2::kFeatDimV3, 0, rng2);
  const auto oa2 = construct_order_v3(model2, n, geoms, geometry, constraints, robot_metas,
                                      seed_makespan, epsilon, options, rng2, nullptr);

  if (oa1.order != oa2.order || oa1.assignment != oa2.assignment)
  {
    std::cerr << "    same seed produced different (order, assignment)\n";
    return false;
  }

  return true;
}

bool test_dqn_v3_signature_distinguishes_assignment()
{
  AllocationScenarioPlan plan_a;
  plan_a.task_order = {0, 1, 2};
  plan_a.preferred_robot_names_by_original_task = {"robot1", "robot2", "robot1"};

  AllocationScenarioPlan plan_b = plan_a;
  // Same order; only task 0's assigned robot differs.
  plan_b.preferred_robot_names_by_original_task = {"robot2", "robot2", "robot1"};

  const std::string sig_a = task_order_assignment_signature(plan_a);
  const std::string sig_b = task_order_assignment_signature(plan_b);
  if (sig_a == sig_b)
  {
    std::cerr << "    expected different signatures for same order/different assignment, both were '"
              << sig_a << "'\n";
    return false;
  }

  AllocationScenarioPlan plan_a2 = plan_a;
  if (task_order_assignment_signature(plan_a) != task_order_assignment_signature(plan_a2))
  {
    std::cerr << "    expected identical signature for identical (order, assignment) plans\n";
    return false;
  }

  // v1/v2's task_order_signature (untouched) must stay assignment-blind.
  if (task_order_signature(plan_a) != task_order_signature(plan_b))
  {
    std::cerr << "    task_order_signature unexpectedly differs when only assignment changed\n";
    return false;
  }

  return true;
}

bool test_dqn_v3_dispatcher_prior_equivalence()
{
  using ReloPush::State;

  const std::size_t n = 4;
  const auto geoms = make_v3_test_geoms();

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0),
      make_test_robot_meta("robot3", -2.0, -2.0, 0.0)};

  RuntimeOptions options;
  options.dqn_explore_ref_bias = 1.0;
  const double seed_makespan = 10.0;
  const double epsilon = 1.0; // forces the explore branch every step

  std::mt19937 rng(2024);
  QModel model(DqnV2::kFeatDimV3, 0, rng);
  const auto oa = construct_order_v3(model, n, geoms, geometry, constraints, robot_metas,
                                     seed_makespan, epsilon, options, rng, nullptr);

  // epsilon=1.0 + ref_bias=1.0 always follows legal.front(), and legal is
  // index-ascending with this fixture's hard edges (0->1, 0->2) -> the
  // reference order is exactly 0,1,2,3.
  for (std::size_t i = 0; i < n; ++i)
  {
    if (oa.order[i] != i)
    {
      std::cerr << "    expected reference order 0..n-1, got order[" << i << "]=" << oa.order[i] << "\n";
      return false;
    }
  }

  // Independently replay the same reference order through the v1/v2
  // dispatcher-driven ForwardSim::pick/preview/commit (mirrors
  // make_forward_sim_v2 in DqnAllocationSearch.cpp) and check every step's
  // robot choice matches what construct_order_v3 assigned.
  std::vector<State> initial_poses;
  for (const auto &meta : robot_metas)
    initial_poses.emplace_back(meta.initial_pose.x, meta.initial_pose.y, meta.initial_pose.yaw);
  const double maxc = 1.0 / std::max(robot_metas[0].min_turning_radius_transit, 1e-6);
  const double wb = robot_metas[0].wheel_base;
  DqnV2::DistanceFn distance = [maxc, wb](const State &from, const State &to)
  { return DqnV2::reeds_shepp_length(from, to, maxc, 0.5, wb); };
  DqnV2::ForwardSim ref_sim(initial_poses, robot_metas[0].speed_transit, distance, n);

  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = oa.order[t];
    const std::size_t expected_robot = ref_sim.pick();
    if (oa.assignment[task] != expected_robot)
    {
      std::cerr << "    step " << t << ": v3 assignment[" << task << "]=" << oa.assignment[task]
                << " != dispatcher pick()=" << expected_robot << "\n";
      return false;
    }
    const auto pv = ref_sim.preview(geoms[task]);
    ref_sim.commit(task, geoms[task], pv);
  }

  return true;
}

bool test_dqn_v3_construct_order_step_log_completeness()
{
  const std::size_t n = 4;
  const auto geoms = make_v3_test_geoms();

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options; // defaults (dqn_explore_ref_bias=0.5, hard_evidence=3)
  const double seed_makespan = 10.0;
  const double epsilon = 0.5; // exercises both explore and exploit branches

  std::mt19937 rng_logged(123);
  QModel model(DqnV2::kFeatDimV3, 0, rng_logged);

  std::vector<std::vector<StepCandidateEntry>> step_logs;
  const auto oa_logged = construct_order_v3(model, n, geoms, geometry, constraints, robot_metas,
                                            seed_makespan, epsilon, options, rng_logged, &step_logs);

  if (oa_logged.order.size() != n || step_logs.size() != n)
  {
    std::cerr << "    order/step_logs size mismatch: order=" << oa_logged.order.size()
              << " step_logs=" << step_logs.size() << "\n";
    return false;
  }

  std::vector<char> is_placed(n, 0);
  for (std::size_t t = 0; t < n; ++t)
  {
    std::vector<std::size_t> legal;
    for (std::size_t i = 0; i < n; ++i)
    {
      if (is_placed[i])
        continue;
      if (DqnV2::is_task_legal_v2(i, geometry, is_placed, constraints,
                                  options.dqn_learned_hard_evidence))
        legal.push_back(i);
    }
    const std::size_t expected_candidates = legal.size() * robot_metas.size();

    if (step_logs[t].size() != expected_candidates)
    {
      std::cerr << "    step " << t << ": expected " << expected_candidates
                << " candidates, step_log has " << step_logs[t].size() << "\n";
      return false;
    }

    std::size_t chosen_count = 0;
    std::size_t logged_chosen_task = n;
    long logged_chosen_robot = -1;
    for (const auto &entry : step_logs[t])
    {
      if (entry.step != t)
      {
        std::cerr << "    step " << t << ": entry has wrong step field " << entry.step << "\n";
        return false;
      }
      if (std::find(legal.begin(), legal.end(), entry.candidate_task) == legal.end())
      {
        std::cerr << "    step " << t << ": logged candidate " << entry.candidate_task
                  << " is not legal\n";
        return false;
      }
      if (entry.robot < 0 || static_cast<std::size_t>(entry.robot) >= robot_metas.size())
      {
        std::cerr << "    step " << t << ": logged candidate robot " << entry.robot
                  << " out of range\n";
        return false;
      }
      if (entry.phi.size() != DqnV2::kFeatDimV3)
      {
        std::cerr << "    step " << t << ": phi has wrong dimension " << entry.phi.size() << "\n";
        return false;
      }
      if (entry.chosen)
      {
        ++chosen_count;
        logged_chosen_task = entry.candidate_task;
        logged_chosen_robot = entry.robot;
      }
    }
    if (chosen_count != 1)
    {
      std::cerr << "    step " << t << ": expected exactly 1 chosen entry, got " << chosen_count << "\n";
      return false;
    }
    if (logged_chosen_task != oa_logged.order[t] ||
        static_cast<std::size_t>(logged_chosen_robot) != oa_logged.assignment[oa_logged.order[t]])
    {
      std::cerr << "    step " << t << ": chosen entry (" << logged_chosen_task << ","
                << logged_chosen_robot << ") != order/assignment (" << oa_logged.order[t] << ","
                << oa_logged.assignment[oa_logged.order[t]] << ")\n";
      return false;
    }

    is_placed[oa_logged.order[t]] = 1;
  }

  // Critical invariant: enabling logging must not perturb the RNG draw
  // sequence. Run again with an identically-seeded rng/model and
  // step_logs == nullptr; the resulting (order, assignment) must be
  // byte-for-byte identical.
  std::mt19937 rng_unlogged(123);
  QModel model_unlogged(DqnV2::kFeatDimV3, 0, rng_unlogged);
  const auto oa_unlogged = construct_order_v3(model_unlogged, n, geoms, geometry, constraints,
                                              robot_metas, seed_makespan, epsilon, options,
                                              rng_unlogged, nullptr);

  if (oa_logged.order != oa_unlogged.order || oa_logged.assignment != oa_unlogged.assignment)
  {
    std::cerr << "    logging perturbed the RNG draw sequence: logged/unlogged (order, assignment) "
                 "differ\n";
    return false;
  }

  return true;
}

// ==========================================
// DQN v4 (12-dim feature vector: v3's dims + load_imbalance +
// idle_robot_congestion) and Decision 2 (risk-decomposed scoring) tests. See
// DqnFeaturesV2.h make_feature_vector_v4, DqnQModel.h accumulate_grad_logistic,
// and DqnAllocationSearch.cpp construct_order_v3/ingest_rollout_v3's
// decomposed branches.
// ==========================================

bool test_dqn_v4_feature_vector()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  const std::size_t n = 3;
  std::vector<TaskGeom> geoms(n);
  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(20.0, 20.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0)};
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(1.0, 0.0, 0.0);
  geoms[0].tau_fixed = 4.0;

  geoms[1].obj_start = State(20.0, 20.0, 0.0);
  geoms[1].obj_goal = State(1.0, 0.02, 0.0);
  geoms[1].push_pts = {State(20.0, 0.0, 0.0), State(21.0, 0.0, 0.0)};
  geoms[1].approach = State(5.0, 0.0, 0.0);
  geoms[1].exit = State(21.0, 0.0, 0.0);
  geoms[1].tau_fixed = 3.0;

  geoms[2].obj_start = State(30.0, 30.0, 0.0);
  geoms[2].obj_goal = State(31.0, 31.0, 0.0);
  geoms[2].push_pts = {State(0.0, 0.0, 0.0), State(0.0, 1.0, 0.0)};
  geoms[2].approach = State(0.0, 1.0, 0.0);
  geoms[2].exit = State(0.0, 2.0, 0.0);
  geoms[2].tau_fixed = 2.0;

  const DqnV2::WorkspaceBounds bounds{-100.0, 100.0, -100.0, 100.0};
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, 0.275);

  std::vector<char> is_placed(n, 0);
  is_placed[0] = 1;
  const LearnedOrderConstraints constraints;

  // Three robots (need >= 2 for load_imbalance/idle_robot_congestion to be
  // meaningful).
  auto euclid = [](const State &a, const State &b)
  { return DqnV2::dist2d(a, b); };
  std::vector<State> initial_poses = {State(0.0, 0.0, 0.0), State(5.0, 0.0, 0.0), State(-5.0, 0.0, 0.0)};
  DqnV2::ForwardSim sim(initial_poses, 1.0, euclid, n);
  sim.free_time = {8.0, 0.0, 2.0}; // arbitrary pre-step schedule state
  sim.window[0] = {0.0, 6.0};

  const double seed_makespan = 10.0;
  const double m_t_before = sim.max_free_time();
  const std::size_t earliest_robot = sim.pick();
  const double min_free_time_before = sim.free_time[earliest_robot];
  const std::vector<std::pair<double, double>> committed_windows = sim.window;

  const auto pv = sim.preview_for(1, geoms[1]);
  const auto phi_v3 = DqnV2::make_feature_vector_v3(
      1, pv, m_t_before, min_free_time_before, geoms[1], geometry, is_placed,
      committed_windows, constraints, seed_makespan, n);
  const auto phi_v4 = DqnV2::make_feature_vector_v4(
      1, pv, m_t_before, min_free_time_before, geoms[1], geometry, is_placed,
      committed_windows, constraints, seed_makespan, n, sim.free_time, sim.pose);

  if (phi_v3.size() != DqnV2::kFeatDimV3)
  {
    std::cerr << "    v3 dim mismatch: expected " << DqnV2::kFeatDimV3 << ", got "
              << phi_v3.size() << "\n";
    return false;
  }
  if (DqnV2::kFeatDimV4 != 12 || phi_v4.size() != DqnV2::kFeatDimV4)
  {
    std::cerr << "    v4 dim mismatch: expected 12, got " << phi_v4.size() << "\n";
    return false;
  }
  // Regression guard: dims 0..9 must be numerically (here, bit-for-bit,
  // since both funnel through the same detail::fill_v3_core_features helper)
  // identical to make_feature_vector_v3 on the same synthetic state.
  for (std::size_t i = 0; i < DqnV2::kFeatDimV3; ++i)
  {
    if (std::abs(phi_v3[i] - phi_v4[i]) > 1e-12)
    {
      std::cerr << "    dims 0.." << (DqnV2::kFeatDimV3 - 1) << " diverge at index " << i
                << ": v3=" << phi_v3[i] << " v4=" << phi_v4[i] << "\n";
      return false;
    }
  }
  for (double v : phi_v4)
  {
    if (!std::isfinite(v))
    {
      std::cerr << "    phi_v4 has a non-finite entry\n";
      return false;
    }
  }

  // load_imbalance (phi_v4[10]), hand-computed: F' after hypothetically
  // committing this candidate to robot 1 is {8.0, pv.finish, 2.0} (robot 1's
  // free_time becomes pv.finish; robots 0/2 unchanged).
  const double expected_max = std::max({8.0, pv.finish, 2.0});
  const double expected_min = std::min({8.0, pv.finish, 2.0});
  const double expected_imbalance = (expected_max - expected_min) / seed_makespan;
  if (std::abs(phi_v4[10] - expected_imbalance) > 1e-9)
  {
    std::cerr << "    load_imbalance mismatch: expected " << expected_imbalance << ", got "
              << phi_v4[10] << "\n";
    return false;
  }

  // Time-not-count property: assigning a short task to the "short-tasks"
  // robot (lower free_time) yields a SMALLER load_imbalance than assigning
  // the identical task to the "busy" robot -- driven purely by the
  // free_time VALUES, with no notion of how many tasks produced them.
  TaskGeom short_task;
  short_task.approach = sim.pose[1]; // zero transit from robot 1's pose
  short_task.exit = sim.pose[1];
  short_task.tau_fixed = 0.5;

  const std::vector<double> free_time_3 = {10.0, 1.0, 1.0}; // robot 0 busy; 1/2 short-tasks
  const std::vector<State> pose_3 = sim.pose;
  const double m_t_before_3 = *std::max_element(free_time_3.begin(), free_time_3.end());
  const std::size_t earliest_3 =
      std::min_element(free_time_3.begin(), free_time_3.end()) - free_time_3.begin();
  const double min_free_time_before_3 = free_time_3[earliest_3];

  DqnV2::ForwardSim::Preview pv_to_short;
  pv_to_short.robot = 1;
  pv_to_short.start = free_time_3[1];
  pv_to_short.finish = free_time_3[1] + short_task.tau_fixed;

  DqnV2::ForwardSim::Preview pv_to_busy;
  pv_to_busy.robot = 0;
  pv_to_busy.start = free_time_3[0];
  pv_to_busy.finish = free_time_3[0] + short_task.tau_fixed;

  const auto phi_to_short = DqnV2::make_feature_vector_v4(
      1, pv_to_short, m_t_before_3, min_free_time_before_3, short_task, geometry, is_placed,
      committed_windows, constraints, seed_makespan, n, free_time_3, pose_3);
  const auto phi_to_busy = DqnV2::make_feature_vector_v4(
      1, pv_to_busy, m_t_before_3, min_free_time_before_3, short_task, geometry, is_placed,
      committed_windows, constraints, seed_makespan, n, free_time_3, pose_3);

  const double expected_to_short = (10.0 - 1.0) / seed_makespan; // F'={10.0,1.5,1.0}
  const double expected_to_busy = (10.5 - 1.0) / seed_makespan;  // F'={10.5,1.0,1.0}
  if (std::abs(phi_to_short[10] - expected_to_short) > 1e-9 ||
      std::abs(phi_to_busy[10] - expected_to_busy) > 1e-9)
  {
    std::cerr << "    time-not-count hand-computed mismatch: to_short=" << phi_to_short[10]
              << " (expected " << expected_to_short << "), to_busy=" << phi_to_busy[10]
              << " (expected " << expected_to_busy << ")\n";
    return false;
  }
  if (!(phi_to_short[10] < phi_to_busy[10]))
  {
    std::cerr << "    expected assigning the short task to the short-tasks robot to yield a "
                 "SMALLER load_imbalance than assigning it to the busy robot: to_short="
              << phi_to_short[10] << " to_busy=" << phi_to_busy[10] << "\n";
    return false;
  }

  return true;
}

bool test_dqn_v4_idle_robot_congestion()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  const std::size_t n = 2; // task count; not the focus of this test
  std::vector<TaskGeom> geoms(n);
  geoms[0].push_pts = {State(1.0, 5.0, 0.0), State(2.0, 5.0, 0.0), State(3.0, 5.0, 0.0)};
  geoms[0].obj_start = State(1.0, 5.0, 0.0);
  geoms[0].obj_goal = State(3.0, 5.0, 0.0);
  geoms[0].approach = State(1.0, 5.0, 0.0);
  geoms[0].exit = State(3.0, 5.0, 0.0);
  geoms[0].tau_fixed = 1.0;

  geoms[1].push_pts = {State(50.0, 50.0, 0.0)};
  geoms[1].obj_start = State(50.0, 50.0, 0.0);
  geoms[1].obj_goal = State(51.0, 51.0, 0.0);
  geoms[1].approach = State(50.0, 50.0, 0.0);
  geoms[1].exit = State(51.0, 51.0, 0.0);
  geoms[1].tau_fixed = 1.0;

  const DqnV2::WorkspaceBounds bounds{0.0, 100.0, 0.0, 100.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);

  std::vector<char> is_placed(n, 0);
  const LearnedOrderConstraints constraints;
  const std::vector<std::pair<double, double>> committed_windows(n, {0.0, 0.0});
  const double seed_makespan = 10.0;

  // Candidate window: robot 0 executes task 0 over [0, 10).
  DqnV2::ForwardSim::Preview pv;
  pv.robot = 0;
  pv.start = 0.0;
  pv.finish = 10.0;

  auto make_phi = [&](const std::vector<double> &free_time_before,
                      const std::vector<State> &pose_before)
  {
    const double m_t_before = *std::max_element(free_time_before.begin(), free_time_before.end());
    const std::size_t earliest =
        std::min_element(free_time_before.begin(), free_time_before.end()) - free_time_before.begin();
    return DqnV2::make_feature_vector_v4(
        0, pv, m_t_before, free_time_before[earliest], geoms[0], geometry, is_placed,
        committed_windows, constraints, seed_makespan, n, free_time_before, pose_before);
  };

  const double block_dist = 0.5 * robot_width + DqnV2::kObjHalfDiag;

  // Case A: one other robot, parked ON the corridor for the whole window
  // (its free_time is well before the window even starts).
  {
    const std::vector<double> ft = {0.0, -5.0};
    const std::vector<State> pose = {State(0.0, 0.0, 0.0), State(2.0, 5.0, 0.0)};
    const double hand_blockage =
        DqnV2::blockage_weight(pose[1], geoms[0].push_pts, block_dist, bounds);
    if (!(hand_blockage > 0.0))
    {
      std::cerr << "    test setup error: expected a positive hand-computed blockage weight\n";
      return false;
    }
    const auto phi = make_phi(ft, pose);
    const double expected = hand_blockage / 1.0; // (robot_count - 1) == 1
    if (!(phi[11] > 0.0) || std::abs(phi[11] - expected) > 1e-9)
    {
      std::cerr << "    on-corridor whole-window case: expected " << expected << ", got "
                << phi[11] << "\n";
      return false;
    }
  }

  // Case B: parked far away from the corridor -> 0 contribution.
  {
    const std::vector<double> ft = {0.0, -5.0};
    const std::vector<State> pose = {State(0.0, 0.0, 0.0), State(90.0, 90.0, 0.0)};
    const auto phi = make_phi(ft, pose);
    if (std::abs(phi[11] - 0.0) > 1e-9)
    {
      std::cerr << "    far-away case: expected 0, got " << phi[11] << "\n";
      return false;
    }
  }

  // Case C: other robot busy for the entire window (free_time >= finish) ->
  // 0, even though it is parked exactly on the corridor -- this is the
  // dynamic-congestion channel's business (dim 6), not idle_robot_congestion's.
  {
    const std::vector<double> ft = {0.0, 10.0}; // free_time[1] == pv.finish
    const std::vector<State> pose = {State(0.0, 0.0, 0.0), State(2.0, 5.0, 0.0)};
    const auto phi = make_phi(ft, pose);
    if (std::abs(phi[11] - 0.0) > 1e-9)
    {
      std::cerr << "    busy-whole-window case: expected 0, got " << phi[11] << "\n";
      return false;
    }
  }

  // Case D: partial temporal overlap scales linearly with the free fraction
  // of the window.
  {
    const std::vector<State> pose = {State(0.0, 0.0, 0.0), State(2.0, 5.0, 0.0)};
    const double hand_blockage =
        DqnV2::blockage_weight(pose[1], geoms[0].push_pts, block_dist, bounds);

    const double phi_25 = make_phi({0.0, 2.5}, pose)[11];
    const double phi_50 = make_phi({0.0, 5.0}, pose)[11];
    const double phi_75 = make_phi({0.0, 7.5}, pose)[11];

    const double expected_25 = hand_blockage * (10.0 - 2.5) / 10.0;
    const double expected_50 = hand_blockage * (10.0 - 5.0) / 10.0;
    const double expected_75 = hand_blockage * (10.0 - 7.5) / 10.0;

    if (std::abs(phi_25 - expected_25) > 1e-9 || std::abs(phi_50 - expected_50) > 1e-9 ||
        std::abs(phi_75 - expected_75) > 1e-9)
    {
      std::cerr << "    partial-overlap hand-computed mismatch: phi_25=" << phi_25
                << " (expected " << expected_25 << "), phi_50=" << phi_50 << " (expected "
                << expected_50 << "), phi_75=" << phi_75 << " (expected " << expected_75
                << ")\n";
      return false;
    }
    // Equal free_time increments -> equal phi[11] decrements (linear scaling).
    if (std::abs((phi_25 - phi_50) - (phi_50 - phi_75)) > 1e-9)
    {
      std::cerr << "    partial-overlap does not scale linearly: phi_25=" << phi_25
                << " phi_50=" << phi_50 << " phi_75=" << phi_75 << "\n";
      return false;
    }
  }

  return true;
}

// Weighted binary cross-entropy on a logit z: -weight * (y*log(sigmoid(z)) +
// (1-y)*log(1-sigmoid(z))). d/dz == weight*(sigmoid(z)-y), matching
// QModel::accumulate_grad_logistic's `e` exactly -- see the central-difference
// check below.
double weighted_logistic_loss_for_test(double z, double y, double weight)
{
  const double sig = dqn_sigmoid(z);
  constexpr double clip_eps = 1e-12;
  const double clipped = std::min(1.0 - clip_eps, std::max(clip_eps, sig));
  return -weight * (y * std::log(clipped) + (1.0 - y) * std::log(1.0 - clipped));
}

// Central-difference gradient check for QModel::accumulate_grad_logistic
// against the weighted logistic loss it is meant to implement, for both the
// linear (hidden==0) and 1-hidden-layer MLP (hidden>0) configurations, and
// for weight != 1 (mirrors test_qmodel_gradient_check's structure for the
// Huber path).
bool test_qmodel_logistic_gradient_check()
{
  auto check_for_hidden = [](std::size_t dim, std::size_t hidden, unsigned seed, double y,
                             double weight) -> bool
  {
    std::mt19937 rng(seed);
    QModel model(dim, hidden, rng);

    std::uniform_real_distribution<double> param_dist(-0.5, 0.5);
    std::vector<double> init_params = model.get_params();
    for (auto &v : init_params)
      v = param_dist(rng);
    model.set_params(init_params);

    std::uniform_real_distribution<double> x_dist(-1.0, 1.0);
    std::vector<double> x(dim);
    for (auto &v : x)
      v = x_dist(rng);

    std::vector<double> grad(model.num_params(), 0.0);
    model.accumulate_grad_logistic(x, y, weight, grad);

    const std::vector<double> base_params = model.get_params();
    constexpr double eps = 1e-6;
    double max_rel_err = 0.0;
    for (std::size_t p = 0; p < base_params.size(); ++p)
    {
      std::vector<double> perturbed = base_params;

      perturbed[p] = base_params[p] + eps;
      model.set_params(perturbed);
      const double loss_plus = weighted_logistic_loss_for_test(model.predict(x), y, weight);

      perturbed[p] = base_params[p] - eps;
      model.set_params(perturbed);
      const double loss_minus = weighted_logistic_loss_for_test(model.predict(x), y, weight);

      model.set_params(base_params);

      const double numeric_grad = (loss_plus - loss_minus) / (2.0 * eps);
      const double denom = std::max(1.0, std::abs(numeric_grad));
      const double rel_err = std::abs(numeric_grad - grad[p]) / denom;
      max_rel_err = std::max(max_rel_err, rel_err);
    }

    if (max_rel_err >= 1e-4)
    {
      std::cerr << "    QModel logistic gradient check failed for dim=" << dim
                << " hidden=" << hidden << " y=" << y << " weight=" << weight
                << ": max_rel_err=" << max_rel_err << "\n";
      return false;
    }
    return true;
  };

  if (!check_for_hidden(5, 0, 401, 1.0, 1.0))
    return false;
  if (!check_for_hidden(5, 0, 402, 0.0, 4.0))
    return false;
  if (!check_for_hidden(5, 8, 403, 1.0, 3.5))
    return false;
  if (!check_for_hidden(5, 8, 404, 0.0, 1.0))
    return false;

  return true;
}

bool test_dqn_v3_decomposed_ingest()
{
  const std::size_t n = 4;
  const auto geoms = make_v3_test_geoms();

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options;
  options.dqn_feature_version = 3;
  options.dqn_scoring_mode = 1; // decomposed

  const std::vector<std::size_t> order = {0, 1, 2, 3};
  const std::vector<std::size_t> assignment = {0, 1, 0, 1};

  const double seed_makespan = 10.0;
  const double fail_return = 3.0;

  // --- Failed episode, failing at step k=2 (0-indexed). ---
  {
    AllocationRunSummary summary;
    summary.all_tasks_succeeded = false;
    TaskCsvRow r0;
    r0.task_id = 1;
    r0.status = "SUCCESS";
    TaskCsvRow r1;
    r1.task_id = 2;
    r1.status = "SUCCESS";
    TaskCsvRow r2;
    r2.task_id = 3;
    r2.status = "FAILED";
    summary.task_rows = {r0, r1, r2};

    std::vector<Transition> replay;
    std::vector<Transition> f_replay;
    ingest_rollout_v3(order, assignment, summary, seed_makespan, fail_return, geoms, geometry,
                      constraints, robot_metas, options, replay, &f_replay);

    const std::size_t k = 2;
    if (f_replay.size() != k + 1)
    {
      std::cerr << "    failed-episode: expected f_replay size " << (k + 1) << ", got "
                << f_replay.size() << "\n";
      return false;
    }
    std::size_t positive_count = 0;
    std::size_t positive_index = f_replay.size();
    for (std::size_t i = 0; i < f_replay.size(); ++i)
    {
      if (f_replay[i].target >= 0.5)
      {
        ++positive_count;
        positive_index = i;
      }
      else if (std::abs(f_replay[i].target) > 1e-12)
      {
        std::cerr << "    failed-episode: f_replay[" << i << "].target neither 0 nor 1: "
                  << f_replay[i].target << "\n";
        return false;
      }
    }
    if (positive_count != 1 || positive_index != k)
    {
      std::cerr << "    failed-episode: expected exactly one positive label at index " << k
                << ", got count=" << positive_count << " at index=" << positive_index << "\n";
      return false;
    }
    if (!replay.empty())
    {
      std::cerr << "    failed-episode: expected q_replay to gain nothing, got " << replay.size()
                << " entries\n";
      return false;
    }
  }

  // --- Feasible episode. ---
  {
    AllocationRunSummary summary;
    summary.all_tasks_succeeded = true;
    summary.makespan = 7.5;
    for (int i = 0; i < 4; ++i)
    {
      TaskCsvRow row;
      row.task_id = i + 1;
      row.status = "SUCCESS";
      summary.task_rows.push_back(row);
    }

    std::vector<Transition> replay;
    std::vector<Transition> f_replay;
    ingest_rollout_v3(order, assignment, summary, seed_makespan, fail_return, geoms, geometry,
                      constraints, robot_metas, options, replay, &f_replay);

    if (f_replay.size() != n)
    {
      std::cerr << "    feasible-episode: expected f_replay size " << n << ", got "
                << f_replay.size() << "\n";
      return false;
    }
    for (std::size_t i = 0; i < f_replay.size(); ++i)
    {
      if (std::abs(f_replay[i].target) > 1e-12)
      {
        std::cerr << "    feasible-episode: expected all F-labels 0, f_replay[" << i
                  << "].target=" << f_replay[i].target << "\n";
        return false;
      }
    }

    if (replay.size() != n)
    {
      std::cerr << "    feasible-episode: expected q_replay size " << n << ", got "
                << replay.size() << "\n";
      return false;
    }
    const double expected_target = -summary.makespan / seed_makespan;
    for (std::size_t i = 0; i < replay.size(); ++i)
    {
      if (std::abs(replay[i].target - expected_target) > 1e-9)
      {
        std::cerr << "    feasible-episode: q_replay[" << i << "].target=" << replay[i].target
                  << ", expected " << expected_target << "\n";
        return false;
      }
    }
  }

  return true;
}

bool test_dqn_v3_decomposed_selection()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  const std::size_t n = 1; // single task; every candidate differs only by robot
  std::vector<TaskGeom> geoms(n);
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(0.0, 0.0, 0.0);
  geoms[0].tau_fixed = 0.0;
  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(1.0, 0.0, 0.0);

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, 0.275);
  const LearnedOrderConstraints constraints;

  // 5 robots, colinear with task0's approach and same heading (yaw 0) so RS
  // transit length == Euclidean distance exactly (same trick as
  // test_dqn_v3_feature_vector_properties). Distances chosen so phi[1]
  // (dmk_plus) == distance/seed_makespan exactly (tau_fixed=0, fresh sim ->
  // m_t_before=0): r0/r4 -> 0.0 (an intentional tie pair), r1/r3 -> 0.3 (a
  // second intentional tie pair), r2 -> 0.6.
  auto make_robot = [](const std::string &name, double distance)
  {
    RobotMeta meta = make_test_robot_meta(name, -distance, 0.0, 0.0);
    meta.speed_transit = 1.0; // clean unit transit speed
    return meta;
  };
  std::vector<RobotMeta> robot_metas = {make_robot("r0", 0.0), make_robot("r1", 3.0),
                                        make_robot("r2", 6.0), make_robot("r3", 3.0),
                                        make_robot("r4", 0.0)};

  // Sanity-check the colinear-same-heading RS-length-equals-Euclidean-distance
  // assumption directly (mirrors test_dqn_v3_feature_vector_properties)
  // before relying on it for the rest of this test.
  const double maxc = 1.0 / robot_metas[0].min_turning_radius_transit;
  const double wheel_base = robot_metas[0].wheel_base;
  for (double d : {3.0, 6.0})
  {
    const double rs_len =
        DqnV2::reeds_shepp_length(State(-d, 0.0, 0.0), geoms[0].approach, maxc, 0.5, wheel_base);
    if (std::abs(rs_len - d) > 1e-6)
    {
      std::cerr << "    test setup error: expected RS length " << d
                << " for a straight-ahead same-heading pair, got " << rs_len << "\n";
      return false;
    }
  }

  const double seed_makespan = 10.0;
  RuntimeOptions options;
  options.dqn_feature_version = 3;
  options.dqn_scoring_mode = 1;

  // q_head picks out phi[1] alone (q == phi[1]). f_head picks out phi[1]
  // with coefficient 1.5 (p_fail == sigmoid(1.5*phi[1])) -- higher-q
  // candidates are ALSO higher-risk, so a threshold between them can exclude
  // the global best-q candidate and force a genuine
  // filter-then-argmax-among-survivors pick, distinct from both a pure
  // global argmax and a pure global argmin.
  std::mt19937 rng_dummy(1); // feeds model construction only; hidden==0 draws nothing from it.
  QModel q_head(DqnV2::kFeatDimV3, 0, rng_dummy);
  QModel f_head(DqnV2::kFeatDimV3, 0, rng_dummy);
  std::vector<double> qw(DqnV2::kFeatDimV3, 0.0);
  qw[1] = 1.0;
  q_head.set_params(qw);
  std::vector<double> fw(DqnV2::kFeatDimV3, 0.0);
  fw[1] = 1.5;
  f_head.set_params(fw);

  // Hand-computed (phi[1], p_fail) per robot at distance d: phi[1] = d/S;
  // p_fail = sigmoid(1.5 * phi[1]).
  //   r0/r4: phi[1]=0.0 -> p_fail=sigmoid(0.0)  = 0.5
  //   r1/r3: phi[1]=0.3 -> p_fail=sigmoid(0.45) ~= 0.6107
  //   r2:    phi[1]=0.6 -> p_fail=sigmoid(0.9)  ~= 0.7109
  const double p_low = dqn_sigmoid(0.0);
  const double p_mid = dqn_sigmoid(1.5 * 0.3);
  const double p_high = dqn_sigmoid(1.5 * 0.6);
  if (!(p_low < p_mid && p_mid < p_high))
  {
    std::cerr << "    test setup error: expected p_low < p_mid < p_high, got " << p_low << " "
              << p_mid << " " << p_high << "\n";
    return false;
  }

  const double epsilon = 0.0; // coin(rng) < 0.0 is always false -> exploit branch every step

  // Case 1: threshold between p_mid and p_high -> survivors = {r0,r1,r3,r4}
  // (r2 excluded); argmax q among survivors is a tie between r1 and r3
  // (both phi[1]=0.3) -> first-seen (r1, lower candidate-list index) must win.
  {
    options.dqn_fail_threshold = 0.65;
    std::mt19937 rng_run(42);
    const auto oa = construct_order_v3(q_head, n, geoms, geometry, constraints, robot_metas,
                                       seed_makespan, epsilon, options, rng_run, nullptr, &f_head);
    if (oa.assignment[0] != 1)
    {
      std::cerr << "    filter-then-argmax case: expected robot 1, got " << oa.assignment[0]
                << "\n";
      return false;
    }
    if (oa.decomposed_exploit_steps != 1 || oa.decomposed_fallback_steps != 0)
    {
      std::cerr << "    filter-then-argmax case: expected 1 exploit step / 0 fallbacks, got "
                << oa.decomposed_exploit_steps << "/" << oa.decomposed_fallback_steps << "\n";
      return false;
    }
  }

  // Case 2: threshold below p_low -> no candidate survives -> fallback to
  // argmin p_fail over ALL candidates, tied between r0 and r4 (both
  // phi[1]=0.0) -> first-seen (r0) must win.
  {
    options.dqn_fail_threshold = 0.4;
    std::mt19937 rng_run(42);
    const auto oa = construct_order_v3(q_head, n, geoms, geometry, constraints, robot_metas,
                                       seed_makespan, epsilon, options, rng_run, nullptr, &f_head);
    if (oa.assignment[0] != 0)
    {
      std::cerr << "    fallback case: expected robot 0, got " << oa.assignment[0] << "\n";
      return false;
    }
    if (oa.decomposed_exploit_steps != 1 || oa.decomposed_fallback_steps != 1)
    {
      std::cerr << "    fallback case: expected 1 exploit step / 1 fallback, got "
                << oa.decomposed_exploit_steps << "/" << oa.decomposed_fallback_steps << "\n";
      return false;
    }
  }

  return true;
}

bool test_dqn_v3_scoring_off_invariance()
{
  const std::size_t n = 4;
  const auto geoms = make_v3_test_geoms();

  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options; // dqn_scoring_mode defaults to 0 (penalty)
  const double seed_makespan = 10.0;
  std::mt19937 rng_dummy(999); // feeds f_head construction only; hidden==0 draws nothing from it.

  // Run A: pre-Decision-2 call shape (no f_head at all). Run B: identical
  // seed/setup, but with a REAL (non-null) f_head passed in penalty mode
  // (dqn_scoring_mode == 0) -- the hard invariant is that penalty mode
  // ignores f_head entirely, so both the result AND the post-call RNG state
  // must be byte-identical to Run A.
  for (double eps : {0.0, 0.5, 1.0})
  {
    std::mt19937 rng_a(555);
    QModel model_a(DqnV2::kFeatDimV3, 0, rng_a);
    const auto oa_a = construct_order_v3(model_a, n, geoms, geometry, constraints, robot_metas,
                                         seed_makespan, eps, options, rng_a, nullptr);

    std::mt19937 rng_b(555);
    QModel model_b(DqnV2::kFeatDimV3, 0, rng_b);
    QModel f_head_dummy(DqnV2::kFeatDimV3, 0, rng_dummy);
    const auto oa_b = construct_order_v3(model_b, n, geoms, geometry, constraints, robot_metas,
                                         seed_makespan, eps, options, rng_b, nullptr,
                                         &f_head_dummy);

    if (oa_a.order != oa_b.order || oa_a.assignment != oa_b.assignment)
    {
      std::cerr << "    epsilon=" << eps << ": penalty mode's (order, assignment) changed when a "
                   "non-null f_head was passed\n";
      return false;
    }
    if (!(rng_a == rng_b))
    {
      std::cerr << "    epsilon=" << eps << ": penalty mode's post-call RNG state changed when a "
                   "non-null f_head was passed (extra RNG draws leaked in)\n";
      return false;
    }
    if (oa_b.decomposed_exploit_steps != 0 || oa_b.decomposed_fallback_steps != 0 ||
        oa_b.decomposed_survivor_fraction_sum != 0.0)
    {
      std::cerr << "    epsilon=" << eps
                << ": penalty mode unexpectedly populated decomposed-scoring diagnostics\n";
      return false;
    }
  }

  return true;
}

bool test_assignment_divergence_counts()
{
  // Original task indices 0..4; task_order permutes them so row i in
  // task_rows corresponds to task_order[i], not i itself -- this exercises
  // the row -> original-task-index mapping rather than just an identity
  // order.
  AllocationScenarioPlan intended;
  intended.task_order = {2, 0, 3, 1, 4};
  intended.preferred_robot_names_by_original_task = {
      "robot1", // task 0
      "robot2", // task 1
      "robot1", // task 2
      "robot2", // task 3
      "",       // task 4: no preference recorded -- must be skipped entirely
  };

  auto make_row = [](const std::string &status, const std::string &robot)
  {
    TaskCsvRow row;
    row.status = status;
    row.robot_name = robot;
    return row;
  };

  AllocationRunSummary executed;
  // row0 -> task 2 (intended robot1): SUCCESS with robot1 -- match.
  executed.task_rows.push_back(make_row("SUCCESS", "robot1"));
  // row1 -> task 0 (intended robot1): SUCCESS with robot2 -- diverged placement.
  executed.task_rows.push_back(make_row("SUCCESS", "robot2"));
  // row2 -> task 3 (intended robot2): FAILED, last attempted robot2 -- match.
  executed.task_rows.push_back(make_row("FAILED", "robot2"));
  // row3 -> task 1 (intended robot2): FAILED, last attempted robot1 -- diverged failed.
  executed.task_rows.push_back(make_row("FAILED", "robot1"));
  // row4 -> task 4 (no intended preference recorded): SUCCESS -- skipped entirely.
  executed.task_rows.push_back(make_row("SUCCESS", "robot1"));

  const AssignmentDivergence d = count_assignment_divergence(intended, executed);
  if (d.counted_placements != 2 || d.diverged_placements != 1 ||
      d.counted_failed != 2 || d.diverged_failed != 1)
  {
    std::cerr << "    expected {counted_placements=2, diverged_placements=1, "
                 "counted_failed=2, diverged_failed=1}, got {counted_placements="
              << d.counted_placements << ", diverged_placements=" << d.diverged_placements
              << ", counted_failed=" << d.counted_failed
              << ", diverged_failed=" << d.diverged_failed << "}\n";
    return false;
  }

  // Defensive: intended.task_order/executed.task_rows length mismatch yields
  // an all-zero result rather than an out-of-bounds access.
  AllocationScenarioPlan mismatched_intended = intended;
  mismatched_intended.task_order = {2, 0, 3}; // shorter than executed.task_rows
  const AssignmentDivergence d_mismatch =
      count_assignment_divergence(mismatched_intended, executed);
  if (d_mismatch.counted_placements != 0 || d_mismatch.diverged_placements != 0 ||
      d_mismatch.counted_failed != 0 || d_mismatch.diverged_failed != 0)
  {
    std::cerr << "    expected an all-zero result on task_order/task_rows length "
                 "mismatch, got {counted_placements="
              << d_mismatch.counted_placements
              << ", diverged_placements=" << d_mismatch.diverged_placements
              << ", counted_failed=" << d_mismatch.counted_failed
              << ", diverged_failed=" << d_mismatch.diverged_failed << "}\n";
    return false;
  }

  // operator+= accumulates across multiple candidates, as run_dqn_search's
  // v3/v4 diagnostic accumulation relies on.
  AssignmentDivergence total;
  total += d;
  total += d;
  if (total.counted_placements != 4 || total.diverged_placements != 2 ||
      total.counted_failed != 4 || total.diverged_failed != 2)
  {
    std::cerr << "    operator+= did not accumulate as expected, got {counted_placements="
              << total.counted_placements << ", diverged_placements=" << total.diverged_placements
              << ", counted_failed=" << total.counted_failed
              << ", diverged_failed=" << total.diverged_failed << "}\n";
    return false;
  }

  return true;
}

// ==========================================
// D3 offline-pretraining infra: executed-robot relabeled corpus logging
// (options.dqn_relabel_executed), two-head weight loading, and freeze-model.
// ==========================================

// Three spatially-independent tasks (no push-corridor overlaps -> no hard
// edges in build_pairwise_geometry, so every order/step is legal regardless
// of placement order) -- used by the relabeling tests below, which need a
// non-trivial construction order (to exercise the row-position ->
// original-task mapping) without fighting DAG legality.
std::vector<DqnV2::TaskGeom> make_independent_v3_test_geoms()
{
  using DqnV2::TaskGeom;
  using ReloPush::State;

  std::vector<TaskGeom> geoms(3);

  geoms[0].obj_start = State(0.0, 0.0, 0.0);
  geoms[0].obj_goal = State(1.0, 1.0, 0.0);
  geoms[0].push_pts = {State(0.0, 0.0, 0.0), State(1.0, 0.0, 0.0)};
  geoms[0].approach = State(0.0, 0.0, 0.0);
  geoms[0].exit = State(1.0, 0.0, 0.0);
  geoms[0].tau_fixed = 2.0;

  geoms[1].obj_start = State(40.0, 40.0, 0.0);
  geoms[1].obj_goal = State(41.0, 41.0, 0.0);
  geoms[1].push_pts = {State(40.0, 40.0, 0.0), State(41.0, 40.0, 0.0)};
  geoms[1].approach = State(40.0, 40.0, 0.0);
  geoms[1].exit = State(41.0, 40.0, 0.0);
  geoms[1].tau_fixed = 1.5;

  geoms[2].obj_start = State(80.0, 80.0, 0.0);
  geoms[2].obj_goal = State(81.0, 81.0, 0.0);
  geoms[2].push_pts = {State(80.0, 80.0, 0.0), State(81.0, 80.0, 0.0)};
  geoms[2].approach = State(80.0, 80.0, 0.0);
  geoms[2].exit = State(81.0, 80.0, 0.0);
  geoms[2].tau_fixed = 1.0;

  return geoms;
}

// Rebuilds the same Reeds-Shepp DistanceFn/ForwardSim construction that
// make_forward_sim_v2 (file-local to DqnAllocationSearch.cpp, not reachable
// from here) uses internally, so a test can maintain an independent
// reference ForwardSim that behaves identically to the one
// ingest_rollout_v3/ingest_rollout_v3_relabeled build.
DqnV2::ForwardSim make_reference_forward_sim(const std::vector<RobotMeta> &robot_metas,
                                             std::size_t task_count)
{
  std::vector<ReloPush::State> initial_poses;
  initial_poses.reserve(robot_metas.size());
  for (const auto &meta : robot_metas)
    initial_poses.emplace_back(meta.initial_pose.x, meta.initial_pose.y, meta.initial_pose.yaw);

  const double maxc = 1.0 / std::max(robot_metas[0].min_turning_radius_transit, 1e-6);
  const double wb = robot_metas[0].wheel_base;
  DqnV2::DistanceFn distance = [maxc, wb](const ReloPush::State &from, const ReloPush::State &to)
  { return DqnV2::reeds_shepp_length(from, to, maxc, 0.5, wb); };

  return DqnV2::ForwardSim(std::move(initial_poses), robot_metas[0].speed_transit,
                          std::move(distance), task_count);
}

// A 3-task, 2-robot episode where the EXECUTED robot (task_rows[t].robot_name)
// differs from the INTENDED one (assignment[order[t]]) on exactly one step
// (step 1: task 0 intended for "robot1" but actually executed by "robot2").
// Verifies: (a) the chosen row at each step (relabel_step_log) carries phi
// computed for the INTENDED (task, robot) pair against the CORRECTED
// (executed-replayed) state, matched bit-for-bit against an independently
// hand-tracked reference ForwardSim; (b) diverged is 1 exactly on the
// diverging step's chosen row, 0 elsewhere, and every non-chosen row keeps
// the executed_robot=-1/diverged=false defaults; (c) the credited
// replay-buffer transition at each step matches the EXECUTED robot's preview
// against that same corrected state; (d) the NEXT step (step 2) shows the
// corrected state carrying the executed commit forward -- "robot1"
// (intended-but-not-executed at step 1) has free_time/pose UNTOUCHED for the
// rest of the episode, not advanced as pure intended-only replay would leave
// it -- confirmed both directly (reference sim's free_time[0] stays 0) and
// via a cross-check against plain ingest_rollout_v3 on the same input.
bool test_dqn_v3_relabeled_replay_correctness()
{
  const std::size_t n = 3;
  const auto geoms = make_independent_v3_test_geoms();
  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  const std::vector<std::string> robot_names = {"robot1", "robot2"};
  const std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options;
  options.dqn_feature_version = 3;
  options.dqn_relabel_executed = true;

  // Construction order: task 1 first, task 0 second, task 2 third.
  const std::vector<std::size_t> order = {1, 0, 2};
  // Intended assignment[original_task] = robot index: task0->robot1(0),
  // task1->robot2(1), task2->robot1(0).
  const std::vector<std::size_t> assignment = {0, 1, 0};
  // Executed robot per step (index t <-> order[t]): step0(task1)->robot2
  // (matches intended); step1(task0)->robot2 (DIVERGES from intended
  // robot1); step2(task2)->robot1 (matches intended).
  const std::vector<std::size_t> executed_robot_by_step = {1, 1, 0};

  AllocationRunSummary summary;
  summary.all_tasks_succeeded = true;
  summary.makespan = 20.0;
  {
    TaskCsvRow r0;
    r0.task_id = 1; // unused by the relabeled path: row POSITION drives the
                    // original-task mapping, not task_id (see
                    // AssignmentDivergence's doc comment).
    r0.status = "SUCCESS";
    r0.robot_name = "robot2";
    TaskCsvRow r1;
    r1.task_id = 2;
    r1.status = "SUCCESS";
    r1.robot_name = "robot2";
    TaskCsvRow r2;
    r2.task_id = 3;
    r2.status = "SUCCESS";
    r2.robot_name = "robot1";
    summary.task_rows = {r0, r1, r2};
  }

  const double seed_makespan = 20.0;
  const double fail_return = 3.0;

  std::vector<Transition> replay;
  std::vector<std::vector<StepCandidateEntry>> relabel_log;
  ingest_rollout_v3_relabeled(order, assignment, summary, seed_makespan, fail_return, geoms,
                              geometry, constraints, robot_metas, robot_names, options, replay,
                              nullptr, &relabel_log);

  if (replay.size() != n)
  {
    std::cerr << "    expected " << n << " credited replay transitions, got " << replay.size() << "\n";
    return false;
  }
  if (relabel_log.size() != n)
  {
    std::cerr << "    expected relabel_log sized to " << n << " steps, got " << relabel_log.size() << "\n";
    return false;
  }

  DqnV2::ForwardSim ref_sim = make_reference_forward_sim(robot_metas, n);
  std::vector<char> is_placed(n, 0);
  double robot0_free_time_entering_step2 = -1.0;

  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    const std::size_t intended_robot = assignment[task];
    const std::size_t executed_robot = executed_robot_by_step[t];
    const bool expect_diverged = executed_robot != intended_robot;

    // Snapshot robot0 ("robot1")'s free_time on entry to step 2: it was the
    // INTENDED (but not executed) robot at step 1, and is not used again
    // until it legitimately executes task 2 at this very step -- so it must
    // still read 0 here, proving the executed commit at step 1 (to robot1's
    // idx1 "robot2") left robot0 completely untouched rather than advancing
    // it as pure intended-only replay would have.
    if (t == 2)
      robot0_free_time_entering_step2 = ref_sim.free_time[0];

    const double m_t_before = ref_sim.max_free_time();
    const double min_free_time_before = ref_sim.free_time[ref_sim.pick()];

    const auto intended_pv = ref_sim.preview_for(intended_robot, geoms[task]);
    const auto expected_chosen_phi = DqnV2::make_feature_vector_v3(
        task, intended_pv, m_t_before, min_free_time_before, geoms[task], geometry, is_placed,
        ref_sim.window, constraints, seed_makespan, n);

    const auto executed_pv = ref_sim.preview_for(executed_robot, geoms[task]);
    const auto expected_credit_phi = DqnV2::make_feature_vector_v3(
        task, executed_pv, m_t_before, min_free_time_before, geoms[task], geometry, is_placed,
        ref_sim.window, constraints, seed_makespan, n);

    if (replay[t].phi.size() != expected_credit_phi.size())
    {
      std::cerr << "    step " << t << ": credit phi size mismatch\n";
      return false;
    }
    for (std::size_t i = 0; i < expected_credit_phi.size(); ++i)
    {
      if (std::abs(replay[t].phi[i] - expected_credit_phi[i]) > 1e-9)
      {
        std::cerr << "    step " << t << ": credit phi[" << i << "] = " << replay[t].phi[i]
                  << ", expected " << expected_credit_phi[i] << " (executed robot preview)\n";
        return false;
      }
    }

    const auto &step_entries = relabel_log[t];
    std::size_t chosen_count = 0;
    bool found_expected_chosen = false;
    for (const auto &entry : step_entries)
    {
      if (!entry.chosen)
      {
        if (entry.executed_robot != -1 || entry.diverged)
        {
          std::cerr << "    step " << t << ": non-chosen entry (task=" << entry.candidate_task
                    << ", robot=" << entry.robot << ") has non-default executed_robot/diverged\n";
          return false;
        }
        continue;
      }
      ++chosen_count;
      if (entry.candidate_task != task || entry.robot != static_cast<long>(intended_robot))
      {
        std::cerr << "    step " << t << ": chosen row is (task=" << entry.candidate_task
                  << ", robot=" << entry.robot << "), expected (task=" << task
                  << ", robot=" << intended_robot << ")\n";
        return false;
      }
      if (entry.phi.size() != expected_chosen_phi.size())
      {
        std::cerr << "    step " << t << ": chosen phi size mismatch\n";
        return false;
      }
      for (std::size_t i = 0; i < expected_chosen_phi.size(); ++i)
      {
        if (std::abs(entry.phi[i] - expected_chosen_phi[i]) > 1e-9)
        {
          std::cerr << "    step " << t << ": chosen phi[" << i << "] = " << entry.phi[i]
                    << ", expected " << expected_chosen_phi[i]
                    << " (intended robot preview against corrected state)\n";
          return false;
        }
      }
      if (entry.executed_robot != static_cast<long>(executed_robot))
      {
        std::cerr << "    step " << t << ": chosen row executed_robot=" << entry.executed_robot
                  << ", expected " << executed_robot << "\n";
        return false;
      }
      if (entry.diverged != expect_diverged)
      {
        std::cerr << "    step " << t << ": chosen row diverged=" << entry.diverged
                  << ", expected " << expect_diverged << "\n";
        return false;
      }
      found_expected_chosen = true;
    }
    if (chosen_count != 1 || !found_expected_chosen)
    {
      std::cerr << "    step " << t << ": expected exactly 1 chosen row matching the intended "
                << "pair, found " << chosen_count << "\n";
      return false;
    }

    ref_sim.commit_for(task, geoms[task], executed_pv);
    is_placed[task] = 1;
  }

  if (std::abs(robot0_free_time_entering_step2 - 0.0) > 1e-9)
  {
    std::cerr << "    robot1 (index 0) free_time entering step 2 should still be 0 (untouched by "
              << "step 1's executed commit to robot2), got " << robot0_free_time_entering_step2
              << "\n";
    return false;
  }

  // Cross-check against ingest_rollout_v3 (the INTENDED-only path, entirely
  // unchanged by this patch): step 0 (no divergence) must still match; steps
  // 1 and 2 must differ (different credited robot at step 1; step 2's state
  // still carries robot1's stale free_time=0 under relabeling vs. an
  // advanced one under pure-intended replay).
  std::vector<Transition> intended_only_replay;
  ingest_rollout_v3(order, assignment, summary, seed_makespan, fail_return, geoms, geometry,
                    constraints, robot_metas, options, intended_only_replay, nullptr);
  if (intended_only_replay.size() != n)
  {
    std::cerr << "    ingest_rollout_v3 (comparison): expected " << n << " transitions, got "
              << intended_only_replay.size() << "\n";
    return false;
  }
  bool step0_matches = true, step1_differs = false, step2_differs = false;
  for (std::size_t i = 0; i < replay[0].phi.size(); ++i)
    if (std::abs(replay[0].phi[i] - intended_only_replay[0].phi[i]) > 1e-9)
      step0_matches = false;
  for (std::size_t i = 0; i < replay[1].phi.size(); ++i)
    if (std::abs(replay[1].phi[i] - intended_only_replay[1].phi[i]) > 1e-9)
      step1_differs = true;
  for (std::size_t i = 0; i < replay[2].phi.size(); ++i)
    if (std::abs(replay[2].phi[i] - intended_only_replay[2].phi[i]) > 1e-9)
      step2_differs = true;
  if (!step0_matches)
  {
    std::cerr << "    step 0 (no divergence) should match plain ingest_rollout_v3 exactly\n";
    return false;
  }
  if (!step1_differs)
  {
    std::cerr << "    step 1 (diverging step) should differ from plain ingest_rollout_v3\n";
    return false;
  }
  if (!step2_differs)
  {
    std::cerr << "    step 2 should differ from plain ingest_rollout_v3 (relabeled state has "
                 "robot1's free_time still 0; intended-only replay would have advanced it)\n";
    return false;
  }

  return true;
}

// Executed-robot fallback when there is no definite executed robot for a step
// (a FAILED row, or a failing step under early abort leaving later steps'
// task_rows entirely absent -- an "unexecuted tail"). Verifies the chosen row
// at such a step falls back to the intended robot with executed_robot=-1 (no
// definite answer) and diverged=false, exactly as documented for
// options.dqn_relabel_executed.
bool test_dqn_v3_relabeled_replay_unexecuted_tail_fallback()
{
  const std::size_t n = 3;
  const auto geoms = make_independent_v3_test_geoms();
  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  const std::vector<std::string> robot_names = {"robot1", "robot2"};
  const std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options;
  options.dqn_feature_version = 3;
  options.dqn_relabel_executed = true;

  const std::vector<std::size_t> order = {0, 1, 2};
  const std::vector<std::size_t> assignment = {0, 1, 0};

  // Infeasible episode: task_rows only covers step 0 (which itself FAILED),
  // simulating early-abort -- steps 1 and 2 have no row at all (unexecuted
  // tail).
  AllocationRunSummary summary;
  summary.all_tasks_succeeded = false;
  {
    TaskCsvRow r0;
    r0.task_id = 1;
    r0.status = "FAILED";
    r0.robot_name = "robot1";
    summary.task_rows = {r0};
  }

  const double seed_makespan = 20.0;
  const double fail_return = 3.0;

  std::vector<Transition> replay;
  std::vector<std::vector<StepCandidateEntry>> relabel_log;
  ingest_rollout_v3_relabeled(order, assignment, summary, seed_makespan, fail_return, geoms,
                              geometry, constraints, robot_metas, robot_names, options, replay,
                              nullptr, &relabel_log);

  if (relabel_log.size() != n)
  {
    std::cerr << "    expected relabel_log sized to " << n << " steps, got " << relabel_log.size() << "\n";
    return false;
  }

  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    const std::size_t intended_robot = assignment[task];
    bool found = false;
    for (const auto &entry : relabel_log[t])
    {
      if (!entry.chosen)
        continue;
      found = true;
      if (entry.candidate_task != task || entry.robot != static_cast<long>(intended_robot))
      {
        std::cerr << "    step " << t << ": chosen row is (task=" << entry.candidate_task
                  << ", robot=" << entry.robot << "), expected the intended pair (task=" << task
                  << ", robot=" << intended_robot << ")\n";
        return false;
      }
      if (entry.executed_robot != -1)
      {
        std::cerr << "    step " << t << ": expected executed_robot=-1 (no definite executed "
                     "robot), got "
                  << entry.executed_robot << "\n";
        return false;
      }
      if (entry.diverged)
      {
        std::cerr << "    step " << t << ": expected diverged=false when there is no definite "
                     "executed robot to compare against\n";
        return false;
      }
    }
    if (!found)
    {
      std::cerr << "    step " << t << ": no chosen row found\n";
      return false;
    }
  }

  // Credit gating is unchanged/inherited from compute_rollout_outcome
  // (fail_pos=0 -> only step 0 credited in penalty mode); already covered by
  // test_dqn_v3_decomposed_ingest's fail_pos tests, just sanity-checked here.
  if (replay.size() != 1)
  {
    std::cerr << "    expected exactly 1 credited transition (fail_pos=0 gate), got "
              << replay.size() << "\n";
    return false;
  }

  return true;
}

// With options.dqn_relabel_executed at its default (false), run_dqn_search
// dispatches to plain ingest_rollout_v3 -- untouched by this patch (which
// only adds a new sibling, ingest_rollout_v3_relabeled). Confirms that
// function's output on a run summary containing executed-robot divergence is
// EXACTLY the "intended robot replay" it always was: divergence recorded in
// task_rows is simply never consulted for robot identity when relabeling is
// off.
bool test_dqn_v3_relabel_off_invariance()
{
  RuntimeOptions default_options;
  if (default_options.dqn_relabel_executed)
  {
    std::cerr << "    dqn_relabel_executed should default to false\n";
    return false;
  }

  const std::size_t n = 3;
  const auto geoms = make_independent_v3_test_geoms();
  const DqnV2::WorkspaceBounds bounds{-1000.0, 1000.0, -1000.0, 1000.0};
  const double robot_width = 0.275;
  const DqnV2::PairwiseGeometry geometry = DqnV2::build_pairwise_geometry(geoms, bounds, robot_width);
  const LearnedOrderConstraints constraints;

  const std::vector<RobotMeta> robot_metas = {
      make_test_robot_meta("robot1", 0.5, 0.45, 0.0),
      make_test_robot_meta("robot2", 3.5, 4.5, 0.0)};

  RuntimeOptions options;
  options.dqn_feature_version = 3;
  options.dqn_relabel_executed = false; // explicit, though this is the default

  const std::vector<std::size_t> order = {1, 0, 2};
  const std::vector<std::size_t> assignment = {0, 1, 0};

  // Same divergent summary as the relabeled-replay-correctness test above
  // (task 0 actually executed by "robot2", not the intended "robot1"). With
  // relabeling off, this must be irrelevant.
  AllocationRunSummary summary;
  summary.all_tasks_succeeded = true;
  summary.makespan = 20.0;
  {
    TaskCsvRow r0;
    r0.status = "SUCCESS";
    r0.robot_name = "robot2";
    TaskCsvRow r1;
    r1.status = "SUCCESS";
    r1.robot_name = "robot2"; // diverges from intended "robot1" -- must be ignored
    TaskCsvRow r2;
    r2.status = "SUCCESS";
    r2.robot_name = "robot1";
    summary.task_rows = {r0, r1, r2};
  }

  const double seed_makespan = 20.0;
  const double fail_return = 3.0;

  std::vector<Transition> replay;
  ingest_rollout_v3(order, assignment, summary, seed_makespan, fail_return, geoms, geometry,
                    constraints, robot_metas, options, replay, nullptr);

  if (replay.size() != n)
  {
    std::cerr << "    expected " << n << " credited transitions, got " << replay.size() << "\n";
    return false;
  }

  DqnV2::ForwardSim ref_sim = make_reference_forward_sim(robot_metas, n);
  std::vector<char> is_placed(n, 0);
  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    const std::size_t intended_robot = assignment[task];
    const double m_t_before = ref_sim.max_free_time();
    const double min_free_time_before = ref_sim.free_time[ref_sim.pick()];
    const auto pv = ref_sim.preview_for(intended_robot, geoms[task]);
    const auto expected_phi = DqnV2::make_feature_vector_v3(
        task, pv, m_t_before, min_free_time_before, geoms[task], geometry, is_placed,
        ref_sim.window, constraints, seed_makespan, n);

    if (replay[t].phi.size() != expected_phi.size())
    {
      std::cerr << "    step " << t << ": phi size mismatch\n";
      return false;
    }
    for (std::size_t i = 0; i < expected_phi.size(); ++i)
    {
      if (std::abs(replay[t].phi[i] - expected_phi[i]) > 1e-9)
      {
        std::cerr << "    step " << t << ": phi[" << i << "] = " << replay[t].phi[i]
                  << ", expected " << expected_phi[i] << " (pure intended-robot replay, "
                  << "ignoring the executed-robot divergence recorded in task_rows)\n";
        return false;
      }
    }

    ref_sim.commit_for(task, geoms[task], pv);
    is_placed[task] = 1;
  }

  return true;
}

// options.dqn_freeze_model's effect in run_dqn_search is simply "don't call
// train_model/train_model_logistic"; this directly verifies the property
// that skip relies on: calling either function is the only thing that
// changes a QModel's params, so omitting the call (what dqn_freeze_model
// does at each of run_dqn_search's 4 call sites) leaves params bit-identical,
// while actually calling it does not (sanity check that the calls used here
// are not accidentally no-ops).
bool test_dqn_freeze_model_flag()
{
  std::mt19937 rng(42);
  QModel model(5, 0, rng);
  const std::vector<double> params_before = model.get_params();

  std::vector<Transition> replay;
  for (int i = 0; i < 10; ++i)
  {
    Transition tr;
    tr.phi = {1.0, 0.2, 0.3, 0.4, 0.5};
    tr.target = 0.7;
    replay.push_back(tr);
  }

  // Simulates run_dqn_search's `if (!options.dqn_freeze_model) train_model(...)`
  // with dqn_freeze_model == true: the call is skipped entirely.
  const bool dqn_freeze_model = true;
  if (!dqn_freeze_model)
    train_model(model, replay, 0.05, 64, 32, rng);
  if (model.get_params() != params_before)
  {
    std::cerr << "    freeze_model=true: q_head params changed after a skipped train_model call\n";
    return false;
  }

  std::mt19937 rng_f(43);
  QModel f_head(5, 0, rng_f);
  const std::vector<double> f_params_before = f_head.get_params();
  std::vector<Transition> f_replay;
  for (int i = 0; i < 10; ++i)
  {
    Transition tr;
    tr.phi = {1.0, 0.2, 0.3, 0.4, 0.5};
    tr.target = (i % 2 == 0) ? 1.0 : 0.0;
    f_replay.push_back(tr);
  }
  if (!dqn_freeze_model)
    train_model_logistic(f_head, f_replay, 0.05, 64, 32, 0.0, rng_f);
  if (f_head.get_params() != f_params_before)
  {
    std::cerr << "    freeze_model=true: f_head params changed after a skipped "
                 "train_model_logistic call\n";
    return false;
  }

  // Sanity: WITHOUT freezing, the same calls DO change params (proves the
  // skips above were meaningful, not accidental no-ops).
  train_model(model, replay, 0.05, 64, 32, rng);
  if (model.get_params() == params_before)
  {
    std::cerr << "    sanity check failed: train_model did not change q_head params at all\n";
    return false;
  }
  train_model_logistic(f_head, f_replay, 0.05, 64, 32, 0.0, rng_f);
  if (f_head.get_params() == f_params_before)
  {
    std::cerr << "    sanity check failed: train_model_logistic did not change f_head params\n";
    return false;
  }

  return true;
}

// options.dqn_init_fail_weights_path: mirrors run_dqn_search's f_head loading
// block (construct f_head, then f_head->load(path) if the path is
// non-empty). A well-formed matching-dim file loads exactly; a dim-mismatch
// file fails to load and leaves params untouched (QModel::load's existing
// contract, verified here specifically against the v3 (10-dim) f_head shape
// this option targets).
bool test_dqn_init_fail_weights_load()
{
  const auto tmp_path = std::filesystem::temp_directory_path() /
                       "phastar_unit_tests_dqn_fail_weights.weights";
  std::filesystem::remove(tmp_path);

  std::mt19937 rng(501);
  QModel source(DqnV2::kFeatDimV3, 0, rng);
  std::uniform_real_distribution<double> param_dist(-1.0, 1.0);
  std::vector<double> source_params = source.get_params();
  for (auto &v : source_params)
    v = param_dist(rng);
  source.set_params(source_params);
  if (!source.save(tmp_path.string()))
  {
    std::cerr << "    Failed to save source f_head weights.\n";
    return false;
  }

  RuntimeOptions options;
  options.dqn_init_fail_weights_path = tmp_path.string();

  std::mt19937 rng2(502);
  QModel f_head(DqnV2::kFeatDimV3, 0, rng2);
  const bool loaded = f_head.load(options.dqn_init_fail_weights_path);
  std::filesystem::remove(tmp_path);
  if (!loaded)
  {
    std::cerr << "    f_head.load unexpectedly failed for a well-formed matching file.\n";
    return false;
  }
  const std::vector<double> f_head_params = f_head.get_params();
  if (f_head_params.size() != source_params.size())
  {
    std::cerr << "    f_head param count mismatch: expected " << source_params.size() << " got "
              << f_head_params.size() << "\n";
    return false;
  }
  for (std::size_t i = 0; i < source_params.size(); ++i)
  {
    if (std::abs(source_params[i] - f_head_params[i]) > 1e-12)
    {
      std::cerr << "    f_head param mismatch at index " << i << ": expected " << source_params[i]
                << " got " << f_head_params[i] << "\n";
      return false;
    }
  }

  // Dim-mismatch file: load must fail and leave params untouched.
  const auto mismatch_path = std::filesystem::temp_directory_path() /
                            "phastar_unit_tests_dqn_fail_weights_mismatch.weights";
  std::filesystem::remove(mismatch_path);
  std::mt19937 rng3(503);
  QModel mismatched_source(DqnV2::kFeatDimV4, 0, rng3); // 12-dim, not 10
  mismatched_source.save(mismatch_path.string());

  std::mt19937 rng4(504);
  QModel f_head2(DqnV2::kFeatDimV3, 0, rng4); // 10-dim target
  const std::vector<double> f_head2_before = f_head2.get_params();
  RuntimeOptions options2;
  options2.dqn_init_fail_weights_path = mismatch_path.string();
  const bool loaded2 = f_head2.load(options2.dqn_init_fail_weights_path);
  std::filesystem::remove(mismatch_path);
  if (loaded2)
  {
    std::cerr << "    f_head.load unexpectedly succeeded on a dim mismatch.\n";
    return false;
  }
  if (f_head2.get_params() != f_head2_before)
  {
    std::cerr << "    f_head params were modified after a failed (mismatched) load.\n";
    return false;
  }

  return true;
}

bool test_resample_to_k_points()
{
  // Straight line (0,0) -> (4,0) -> (10,0), uneven segment spacing so the
  // bracketing-segment search must advance mid-resample. First segment is
  // non-pushing, second is pushing.
  const std::vector<PathPoint> path = {
      {0.0, 0.0, 0.1, false},
      {4.0, 0.0, 0.2, false},
      {10.0, 0.0, 0.3, true},
  };

  const auto resampled = resample_to_k_points(path, 5);
  if (resampled.size() != 5)
  {
    std::cerr << "    expected 5 resampled points, got " << resampled.size() << "\n";
    return false;
  }

  const std::vector<double> expected_x = {0.0, 2.5, 5.0, 7.5, 10.0};
  const std::vector<double> expected_yaw = {0.1, 0.2, 0.2, 0.3, 0.3};
  const std::vector<bool> expected_pushing = {false, false, false, true, true};
  for (std::size_t i = 0; i < resampled.size(); ++i)
  {
    if (std::abs(resampled[i].x - expected_x[i]) > 1e-9 || std::abs(resampled[i].y) > 1e-9)
    {
      std::cerr << "    point " << i << " position mismatch: (" << resampled[i].x << ", "
                << resampled[i].y << "), expected x=" << expected_x[i] << "\n";
      return false;
    }
    if (std::abs(resampled[i].yaw - expected_yaw[i]) > 1e-9)
    {
      std::cerr << "    point " << i << " yaw mismatch: " << resampled[i].yaw
                << ", expected " << expected_yaw[i] << "\n";
      return false;
    }
    if (resampled[i].is_pushing != expected_pushing[i])
    {
      std::cerr << "    point " << i << " is_pushing mismatch: got "
                << resampled[i].is_pushing << ", expected " << expected_pushing[i] << "\n";
      return false;
    }
  }

  // K=1 -> just the first point.
  const auto single = resample_to_k_points(path, 1);
  if (single.size() != 1 || std::abs(single[0].x - 0.0) > 1e-9)
  {
    std::cerr << "    K=1 did not return just the first point\n";
    return false;
  }

  // Zero-length (all-coincident) input -> K copies of the single point.
  const std::vector<PathPoint> coincident = {{2.0, 3.0, 1.0, true}, {2.0, 3.0, 1.0, true}};
  const auto zero_length = resample_to_k_points(coincident, 4);
  if (zero_length.size() != 4)
  {
    std::cerr << "    zero-length input did not return K points, got "
              << zero_length.size() << "\n";
    return false;
  }
  for (const auto &p : zero_length)
  {
    if (std::abs(p.x - 2.0) > 1e-9 || std::abs(p.y - 3.0) > 1e-9)
    {
      std::cerr << "    zero-length resample point mismatch: (" << p.x << ", " << p.y << ")\n";
      return false;
    }
  }

  // Empty input -> empty output.
  const auto empty_result = resample_to_k_points({}, 5);
  if (!empty_result.empty())
  {
    std::cerr << "    empty input did not return empty output\n";
    return false;
  }

  return true;
}

bool test_json_escape_string()
{
  const std::string input = "he said \"hi\"\\ and\tmore";
  const std::string escaped = json_escape_string(input);

  if (escaped.find("\\\"") == std::string::npos)
  {
    std::cerr << "    escaped output missing \\\" for embedded quote: '" << escaped << "'\n";
    return false;
  }
  if (escaped.find("\\\\") == std::string::npos)
  {
    std::cerr << "    escaped output missing \\\\ for embedded backslash: '" << escaped << "'\n";
    return false;
  }

  // A control character (0x01) must not crash and must not appear literally.
  const std::string control_input = std::string("before") + static_cast<char>(0x01) + "after";
  const std::string control_escaped = json_escape_string(control_input);
  if (control_escaped.find(static_cast<char>(0x01)) != std::string::npos)
  {
    std::cerr << "    control character was not escaped: '" << control_escaped << "'\n";
    return false;
  }
  if (control_escaped.find("\\u0001") == std::string::npos)
  {
    std::cerr << "    expected \\u0001 escape sequence, got '" << control_escaped << "'\n";
    return false;
  }

  return true;
}

// ==========================================
// Policy-gradient PoC P0: batch-oracle CLI (EvalPlansCli.h) unit tests.
// ==========================================

bool test_parse_eval_plan_line_valid()
{
  EvalPlanRequest req;
  std::string error;
  const std::string line = R"({"id": "case-1", "order": [2, 0, 1], "assign": [1, 0, 0]})";
  if (!parse_eval_plan_line(line, req, error))
  {
    std::cerr << "    expected parse success, got error: " << error << "\n";
    return false;
  }
  if (req.id != "case-1")
  {
    std::cerr << "    id mismatch: got '" << req.id << "'\n";
    return false;
  }
  if (req.order != std::vector<std::size_t>({2, 0, 1}))
  {
    std::cerr << "    order mismatch\n";
    return false;
  }
  if (req.assign != std::vector<std::size_t>({1, 0, 0}))
  {
    std::cerr << "    assign mismatch\n";
    return false;
  }

  // Key order and extra whitespace must not matter.
  EvalPlanRequest req2;
  const std::string line2 = "{ \"assign\":[0],\"order\" : [ 0 ] , \"id\":\"x\" }";
  if (!parse_eval_plan_line(line2, req2, error))
  {
    std::cerr << "    expected parse success for reordered/whitespace variant, got error: "
              << error << "\n";
    return false;
  }
  if (req2.id != "x" || req2.order != std::vector<std::size_t>({0}) ||
      req2.assign != std::vector<std::size_t>({0}))
  {
    std::cerr << "    reordered/whitespace variant field mismatch\n";
    return false;
  }

  return true;
}

bool test_parse_eval_plan_line_malformed()
{
  EvalPlanRequest req;
  std::string error;

  if (parse_eval_plan_line(R"({"id": "a", "order": [0, 1]})", req, error))
  {
    std::cerr << "    expected failure for missing \"assign\" key\n";
    return false;
  }

  if (parse_eval_plan_line(R"({"id": "a", "order": [0, 1], "assign": [0]})", req, error))
  {
    std::cerr << "    expected failure for order/assign length mismatch\n";
    return false;
  }

  if (parse_eval_plan_line(R"({"id": "a", "order": [0, x], "assign": [0, 0]})", req, error))
  {
    std::cerr << "    expected failure for non-numeric order element\n";
    return false;
  }

  if (parse_eval_plan_line(R"({"id": "a", "order": [-1, 0], "assign": [0, 0]})", req, error))
  {
    std::cerr << "    expected failure for negative order element\n";
    return false;
  }

  // No opening quote at all around the id value.
  if (parse_eval_plan_line(R"({"id": a, "order": [0], "assign": [0]})", req, error))
  {
    std::cerr << "    expected failure for malformed (unquoted) id\n";
    return false;
  }

  return true;
}

bool test_validate_plan_order()
{
  std::string error;
  if (!validate_plan_order({0, 1, 2}, 3, error))
  {
    std::cerr << "    expected a valid permutation to pass: " << error << "\n";
    return false;
  }
  if (validate_plan_order({0, 1}, 3, error))
  {
    std::cerr << "    expected wrong-length order to fail\n";
    return false;
  }
  if (validate_plan_order({0, 1, 3}, 3, error))
  {
    std::cerr << "    expected out-of-range index to fail\n";
    return false;
  }
  if (validate_plan_order({0, 1, 1}, 3, error))
  {
    std::cerr << "    expected duplicate index to fail\n";
    return false;
  }
  return true;
}

bool test_parse_robot_index_from_name()
{
  const std::vector<std::pair<std::string, long>> cases = {
      {"robot1", 0}, {"robot3", 2}, {"robot12", 11}, {"", -1},
      {"robot0", -1}, {"robotX", -1}, {"bot1", -1}, {"robot", -1},
  };
  for (const auto &[name, expected] : cases)
  {
    const long got = parse_robot_index_from_name(name);
    if (got != expected)
    {
      std::cerr << "    parse_robot_index_from_name('" << name << "') = " << got
                << ", expected " << expected << "\n";
      return false;
    }
  }
  return true;
}

bool test_csv_escape_field()
{
  if (csv_escape_field("plain") != "plain")
  {
    std::cerr << "    plain field should be returned unchanged\n";
    return false;
  }
  if (csv_escape_field("a,b") != "\"a,b\"")
  {
    std::cerr << "    comma field mismatch: '" << csv_escape_field("a,b") << "'\n";
    return false;
  }
  if (csv_escape_field("a\"b") != "\"a\"\"b\"")
  {
    std::cerr << "    quote field mismatch: '" << csv_escape_field("a\"b") << "'\n";
    return false;
  }
  if (csv_escape_field("a\nb") != "\"a\nb\"")
  {
    std::cerr << "    newline field mismatch: '" << csv_escape_field("a\nb") << "'\n";
    return false;
  }
  return true;
}

bool test_eval_plans_cli_end_to_end()
{
  const std::filesystem::path exe_dir = g_test_executable_path.parent_path();
  const std::filesystem::path demo_path = exe_dir / "phastar_push_demo";
  if (!std::filesystem::exists(demo_path))
  {
    std::cerr << "    Missing phastar_push_demo executable at " << demo_path << "\n";
    return false;
  }

  const std::filesystem::path instance_path =
      std::filesystem::path(CMAKE_SOURCE_DIR) /
      "results/relopush-out/result_seq_ReloPush-BOSS_8_objects.txt_ind1.b64";
  if (!std::filesystem::exists(instance_path))
  {
    std::cerr << "    Missing instance file at " << instance_path << "\n";
    return false;
  }

  const std::filesystem::path plans_path =
      std::filesystem::temp_directory_path() / "eval_plans_cli_test_input.jsonl";
  const std::filesystem::path out_path =
      std::filesystem::temp_directory_path() / "eval_plans_cli_test_output.csv";
  std::error_code rm_ec;
  std::filesystem::remove(plans_path, rm_ec);
  std::filesystem::remove(out_path, rm_ec);

  // BOSS_8 has 8 tasks (0..7). Line 1: the identity order, all on robot1
  // (index 0) -- a well-formed, plausible plan. Line 2: deliberately
  // malformed (order/assign length mismatch).
  {
    std::ofstream plans_out(plans_path);
    plans_out << R"({"id": "identity", "order": [0,1,2,3,4,5,6,7], )"
              << R"("assign": [0,0,0,0,0,0,0,0]})" << "\n";
    plans_out << R"({"id": "bad", "order": [0,1], "assign": [0]})" << "\n";
  }

  std::string command =
      demo_path.string() +
      " --no-visualization --no-visualize-relopush-plan --no-debug-vis"
      " --random-seed=1"
      " --eval-plans=" + plans_path.string() +
      " --eval-plans-out=" + out_path.string() +
      " --input-sequence=" + instance_path.string() + " 2>&1";

  int exit_code = 0;
  const std::string output = run_command_capture(command, exit_code);
  if (exit_code != 0)
  {
    std::cerr << "    --eval-plans command failed with exit code " << exit_code
              << "\n    output: " << output << "\n";
    return false;
  }

  if (!std::filesystem::exists(out_path))
  {
    std::cerr << "    Expected output CSV was not created at " << out_path << "\n";
    return false;
  }

  const std::string csv = read_text_file(out_path);
  std::istringstream csv_stream(csv);
  std::string header;
  std::getline(csv_stream, header);
  if (header.find("id,feasible,makespan,first_fail_step,executed_robots,eval_wall_s") ==
      std::string::npos)
  {
    std::cerr << "    Unexpected CSV header: '" << header << "'\n";
    return false;
  }

  std::string identity_row, bad_row;
  std::getline(csv_stream, identity_row);
  std::getline(csv_stream, bad_row);

  if (identity_row.rfind("identity,", 0) != 0)
  {
    std::cerr << "    Expected first data row to start with 'identity,', got: '" << identity_row
              << "'\n";
    return false;
  }
  if (identity_row.find(",-1,-1,-1,,") != std::string::npos)
  {
    std::cerr << "    identity plan row looks like the malformed-line placeholder: '"
              << identity_row << "'\n";
    return false;
  }

  if (bad_row.rfind("bad,-1,-1,-1,,", 0) != 0)
  {
    std::cerr << "    Expected malformed-line row 'bad,-1,-1,-1,,...', got: '" << bad_row
              << "'\n";
    return false;
  }

  std::filesystem::remove(plans_path, rm_ec);
  std::filesystem::remove(out_path, rm_ec);
  return true;
}

// ==========================================
// Policy-gradient PoC P0: per-instance PG table exporter (PgTableExport.h)
// unit test.
// ==========================================

namespace
{

// Test-local, deliberately minimal JSON scalar readers (mirrors the "small
// hand-rolled parser for a known, controlled format" approach the production
// code itself uses -- see EvalPlansCli.cpp/GeometryExport.cpp -- rather than
// pulling in a JSON library just for this regression test).
double read_number_after_key(const std::string &text, std::size_t key_pos)
{
  const auto colon = text.find(':', key_pos);
  if (colon == std::string::npos)
    return std::numeric_limits<double>::quiet_NaN();
  std::size_t p = colon + 1;
  while (p < text.size() && std::isspace(static_cast<unsigned char>(text[p])))
    ++p;
  const auto end = text.find_first_of(",\n}]", p);
  try
  {
    return std::stod(text.substr(p, end == std::string::npos ? std::string::npos : end - p));
  }
  catch (const std::exception &)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

double read_scalar_field(const std::string &text, const std::string &key)
{
  const auto key_pos = text.find("\"" + key + "\"");
  if (key_pos == std::string::npos)
    return std::numeric_limits<double>::quiet_NaN();
  return read_number_after_key(text, key_pos);
}

bool extract_xyz_after(const std::string &text, std::size_t from_pos, double &x, double &y,
                      double &yaw)
{
  const auto x_pos = text.find("\"x\"", from_pos);
  if (x_pos == std::string::npos)
    return false;
  const auto y_pos = text.find("\"y\"", x_pos);
  if (y_pos == std::string::npos)
    return false;
  const auto yaw_pos = text.find("\"yaw\"", y_pos);
  if (yaw_pos == std::string::npos)
    return false;
  x = read_number_after_key(text, x_pos);
  y = read_number_after_key(text, y_pos);
  yaw = read_number_after_key(text, yaw_pos);
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
}

// First entry of the first row of a "key": [[...], [...], ...] matrix.
double extract_first_matrix_entry(const std::string &text, const std::string &key)
{
  const auto key_pos = text.find("\"" + key + "\"");
  if (key_pos == std::string::npos)
    return std::numeric_limits<double>::quiet_NaN();
  const auto outer_bracket = text.find('[', key_pos);
  if (outer_bracket == std::string::npos)
    return std::numeric_limits<double>::quiet_NaN();
  const auto row_bracket = text.find('[', outer_bracket + 1);
  if (row_bracket == std::string::npos)
    return std::numeric_limits<double>::quiet_NaN();
  std::size_t p = row_bracket + 1;
  while (p < text.size() && std::isspace(static_cast<unsigned char>(text[p])))
    ++p;
  const auto end = text.find_first_of(",]", p);
  if (end == std::string::npos)
    return std::numeric_limits<double>::quiet_NaN();
  try
  {
    return std::stod(text.substr(p, end - p));
  }
  catch (const std::exception &)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

} // namespace

bool test_export_pg_tables_end_to_end()
{
  const std::filesystem::path exe_dir = g_test_executable_path.parent_path();
  const std::filesystem::path demo_path = exe_dir / "phastar_push_demo";
  if (!std::filesystem::exists(demo_path))
  {
    std::cerr << "    Missing phastar_push_demo executable at " << demo_path << "\n";
    return false;
  }

  const std::filesystem::path instance_path =
      std::filesystem::path(CMAKE_SOURCE_DIR) /
      "results/relopush-out/result_seq_ReloPush-BOSS_8_objects.txt_ind1.b64";
  if (!std::filesystem::exists(instance_path))
  {
    std::cerr << "    Missing instance file at " << instance_path << "\n";
    return false;
  }

  const std::filesystem::path out_path =
      std::filesystem::temp_directory_path() / "pg_table_export_test.json";
  std::error_code rm_ec;
  std::filesystem::remove(out_path, rm_ec);

  // NOTE: --export-pg-tables= hooks in AFTER the seed-plan selection (see
  // PHAstar_push_demo.cpp), which sits past the --greedy-only short-circuit
  // (assignment/local-sequence/shuffle-sequence/lns-or-dqn iterations all
  // <= 0 -- see run_greedy_only_pipeline). --lns-iters=1 (rather than 0)
  // keeps this test out of that mode; the LNS search itself never actually
  // runs since --export-pg-tables= returns before it would be dispatched.
  std::string command =
      demo_path.string() +
      " --no-visualization --no-visualize-relopush-plan --no-debug-vis"
      " --assignment-search-iters=0 --local-sequence-search-iters=0"
      " --shuffle-sequence-search-iters=0 --lns-iters=1 --random-seed=1"
      " --export-pg-tables=" + out_path.string() +
      " --input-sequence=" + instance_path.string() + " 2>&1";

  int exit_code = 0;
  const std::string output = run_command_capture(command, exit_code);
  if (exit_code != 0)
  {
    std::cerr << "    --export-pg-tables command failed with exit code " << exit_code
              << "\n    output: " << output << "\n";
    return false;
  }

  if (!std::filesystem::exists(out_path))
  {
    std::cerr << "    Expected output JSON file was not created at " << out_path << "\n";
    return false;
  }

  const std::string json = read_text_file(out_path);
  if (json.empty())
  {
    std::cerr << "    Exported JSON file is empty.\n";
    return false;
  }

  const std::vector<std::string> required_keys = {
      "\"family\"",       "\"index\"",           "\"task_count\"",   "\"robot_count\"",
      "\"robot_names\"",  "\"workspace\"",       "\"robot_width\"",  "\"block_dist\"",
      "\"speed_transit\"", "\"robot_wheel_base\"", "\"robot_min_turning_radius_transit\"",
      "\"greedy_makespan\"", "\"seed_makespan\"", "\"tasks\"",       "\"tau_fixed\"",
      "\"approach\"",     "\"exit\"",            "\"obj_start\"",    "\"obj_goal\"",
      "\"boundary_risk\"", "\"hard_pred\"",       "\"soft_pred\"",   "\"corr_overlap\"",
      "\"w_start\"",      "\"w_goal\"",          "\"pose_count\"",   "\"poses\"",
      "\"transit_time\"", "\"parked_blockage\"", "\"ref_order\"",    "\"ref_assign\""};
  for (const auto &key : required_keys)
  {
    if (json.find(key) == std::string::npos)
    {
      std::cerr << "    Exported JSON is missing required key: " << key << "\n";
      return false;
    }
  }

  // Spot-check: transit_time[0][0] (poses[0] == robot1's initial pose, since
  // robot inits are written before task exits -- see PgTableExport.h; ->
  // task 0's approach pose) must equal reeds_shepp_length(...)/speed_transit
  // computed directly here from the SAME exported JSON's own poses[0]/
  // tasks[0].approach/scalar fields (not from hardcoded robot constants).
  double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;
  const auto robot_init_pos = json.find("\"kind\": \"robot_init\"");
  if (robot_init_pos == std::string::npos ||
      !extract_xyz_after(json, robot_init_pos, robot_x, robot_y, robot_yaw))
  {
    std::cerr << "    Failed to locate/parse the first robot_init pose.\n";
    return false;
  }

  double task_x = 0.0, task_y = 0.0, task_yaw = 0.0;
  const auto approach_pos = json.find("\"approach\"");
  if (approach_pos == std::string::npos ||
      !extract_xyz_after(json, approach_pos, task_x, task_y, task_yaw))
  {
    std::cerr << "    Failed to locate/parse task 0's approach pose.\n";
    return false;
  }

  const double speed_transit = read_scalar_field(json, "speed_transit");
  const double wheel_base = read_scalar_field(json, "robot_wheel_base");
  const double turning_radius = read_scalar_field(json, "robot_min_turning_radius_transit");
  if (!(speed_transit > 0.0) || !std::isfinite(wheel_base) || !std::isfinite(turning_radius))
  {
    std::cerr << "    Failed to parse speed_transit/robot_wheel_base/"
                 "robot_min_turning_radius_transit scalars.\n";
    return false;
  }

  const ReloPush::State from(robot_x, robot_y, robot_yaw);
  const ReloPush::State to(task_x, task_y, task_yaw);
  const double maxc = 1.0 / std::max(turning_radius, 1e-6);
  const double expected = DqnV2::reeds_shepp_length(from, to, maxc, 0.5, wheel_base) / speed_transit;

  const double exported_entry = extract_first_matrix_entry(json, "transit_time");
  if (!std::isfinite(exported_entry))
  {
    std::cerr << "    Failed to parse transit_time[0][0] from the exported JSON.\n";
    return false;
  }

  if (std::abs(exported_entry - expected) > 1e-6)
  {
    std::cerr << "    transit_time[0][0] mismatch: exported=" << exported_entry
              << " expected(reeds_shepp_length/speed)=" << expected << "\n";
    return false;
  }

  std::filesystem::remove(out_path, rm_ec);
  return true;
}

// ==========================================
// Policy-gradient PoC P0 (follow-up): --export-decision-time-log= unit test.
// Added after team-lead review of the parity methodology: this log's whole
// purpose is to give script/pg/parity_check.py a phi ground truth computed
// under construct_order_v3's own DECISION-TIME state-advancement rule
// (chosen_robot, not any executed one -- see PgTableExport.h's doc comment
// on export_decision_time_log), as a check independent of the RELABELED log
// --dqn-log-transitions= + --dqn-relabel-executed produces. This test
// verifies the export is deterministic (all-zero-weight model + epsilon=0.0
// -> ties always keep the first-seen candidate, so every step's chosen
// (task, robot) is (lowest legal task index, robot 0)) and that its rows
// carry no relabel-specific data (executed_robot/diverged stay at their
// StepCandidateEntry defaults, since construct_order_v3 never sets them).
// ==========================================

bool test_export_decision_time_log_end_to_end()
{
  const std::filesystem::path exe_dir = g_test_executable_path.parent_path();
  const std::filesystem::path demo_path = exe_dir / "phastar_push_demo";
  if (!std::filesystem::exists(demo_path))
  {
    std::cerr << "    Missing phastar_push_demo executable at " << demo_path << "\n";
    return false;
  }

  const std::filesystem::path instance_path =
      std::filesystem::path(CMAKE_SOURCE_DIR) /
      "results/relopush-out/result_seq_ReloPush-BOSS_8_objects.txt_ind1.b64";
  if (!std::filesystem::exists(instance_path))
  {
    std::cerr << "    Missing instance file at " << instance_path << "\n";
    return false;
  }

  const std::filesystem::path out_path =
      std::filesystem::temp_directory_path() / "decision_time_log_test.csv";
  std::error_code rm_ec;
  std::filesystem::remove(out_path, rm_ec);

  // Same --greedy-only avoidance as test_export_pg_tables_end_to_end (this
  // flag hooks in at the same point in PHAstar_push_demo.cpp).
  std::string command =
      demo_path.string() +
      " --no-visualization --no-visualize-relopush-plan --no-debug-vis"
      " --assignment-search-iters=0 --local-sequence-search-iters=0"
      " --shuffle-sequence-search-iters=0 --lns-iters=1 --random-seed=1"
      " --export-decision-time-log=" + out_path.string() +
      " --input-sequence=" + instance_path.string() + " 2>&1";

  int exit_code = 0;
  const std::string output = run_command_capture(command, exit_code);
  if (exit_code != 0)
  {
    std::cerr << "    --export-decision-time-log command failed with exit code " << exit_code
              << "\n    output: " << output << "\n";
    return false;
  }

  if (!std::filesystem::exists(out_path))
  {
    std::cerr << "    Expected output CSV was not created at " << out_path << "\n";
    return false;
  }

  const std::string csv = read_text_file(out_path);
  std::istringstream csv_stream(csv);
  std::string header;
  std::getline(csv_stream, header);
  if (header.find("candidate_task,candidate_robot,chosen,"
                  "phi0,phi1,phi2,phi3,phi4,phi5,phi6,phi7,phi8,phi9,phi10,phi11,"
                  "feasible,makespan,return_target,first_fail_rank,first_failed_task,"
                  "executed_robot,diverged") == std::string::npos)
  {
    std::cerr << "    Unexpected CSV header (expected the extended v3/v4 schema): '" << header
              << "'\n";
    return false;
  }

  // BOSS_8 has 8 tasks: with an all-zero-weight model and epsilon=0.0, the
  // chosen (task, robot) at every step must be (lowest legal task index, 0)
  // -- see export_decision_time_log's doc comment for why. Collect the
  // chosen row per step and confirm this exact prediction, plus that
  // executed_robot/diverged (relabel-only fields) stay at their unset
  // defaults on every row (this log never runs ingest_rollout_v3_relabeled).
  std::string line;
  std::vector<std::vector<std::string>> rows;
  while (std::getline(csv_stream, line))
  {
    if (line.empty())
      continue;
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ','))
      fields.push_back(field);
    // getline(..., ',') silently drops the final field when the line ends in
    // a trailing delimiter (nothing left to "find" once that comma is
    // consumed, vs. a real empty token between two delimiters, which IS
    // captured correctly) -- exactly the case for a non-chosen row, whose
    // line ends in an empty executed_robot field then an empty diverged
    // field: "...,,". Restore that one missing trailing empty field.
    if (!line.empty() && line.back() == ',')
      fields.push_back("");
    rows.push_back(fields);
  }

  // Column indices per the header above: 0 family,1 index,2 seed,3 iteration,
  // 4 step,5 candidate_task,6 candidate_robot,7 chosen,...,25 executed_robot,
  // 26 diverged.
  constexpr std::size_t kStepCol = 4, kTaskCol = 5, kRobotCol = 6, kChosenCol = 7,
                        kExecutedRobotCol = 25, kDivergedCol = 26;

  std::map<int, std::vector<std::size_t>> row_indices_by_step;
  for (std::size_t i = 0; i < rows.size(); ++i)
  {
    if (rows[i].size() <= kDivergedCol)
    {
      std::cerr << "    row " << i << " has too few columns (" << rows[i].size() << ")\n";
      return false;
    }
    row_indices_by_step[std::stoi(rows[i][kStepCol])].push_back(i);
  }

  if (row_indices_by_step.size() != 8)
  {
    std::cerr << "    expected 8 steps (BOSS_8), got " << row_indices_by_step.size() << "\n";
    return false;
  }

  for (const auto &[step, indices] : row_indices_by_step)
  {
    int chosen_count = 0;
    for (std::size_t i : indices)
    {
      const auto &row = rows[i];
      if (row[kChosenCol] == "1")
      {
        ++chosen_count;
        if (row[kTaskCol] != std::to_string(step))
        {
          std::cerr << "    step " << step << ": expected chosen task==" << step
                    << " (all-zero-weight tie-break keeps the lowest legal task index, "
                       "and BOSS_8 has no precedence edges blocking the identity order), got "
                    << row[kTaskCol] << "\n";
          return false;
        }
        if (row[kRobotCol] != "0")
        {
          std::cerr << "    step " << step << ": expected chosen robot==0, got " << row[kRobotCol]
                    << "\n";
          return false;
        }
        if (row[kExecutedRobotCol] != "-1")
        {
          std::cerr << "    step " << step
                    << ": expected executed_robot to stay at its unset default (-1) on a "
                       "decision-time-only log, got '"
                    << row[kExecutedRobotCol] << "'\n";
          return false;
        }
        if (row[kDivergedCol] != "0")
        {
          std::cerr << "    step " << step
                    << ": expected diverged to stay at its unset default (0) on a "
                       "decision-time-only log, got '"
                    << row[kDivergedCol] << "'\n";
          return false;
        }
      }
    }
    if (chosen_count != 1)
    {
      std::cerr << "    step " << step << " has " << chosen_count
                << " chosen rows, expected exactly 1\n";
      return false;
    }
  }

  std::filesystem::remove(out_path, rm_ec);
  return true;
}

// ==========================================
// Stage 3 planner optimization: failure-triage predicates + holonomic
// feasibility pre-gate (see MARS/src/PlanningHelpers.cpp's Stage 3 design
// note; RuntimeOptions::enable_tier_triage / enable_tier_gate).
// ==========================================

// D1: triage predicate unit tests. Each predicate is a pure function over a
// hand-built PlanningResult/debug_stats fixture, so these exercise the exact
// rule boundaries without needing to drive a real search to failure.
bool test_tier_triage_predicates()
{
  bool ok = true;

  // Rule 1 (BLOCKED-BY-ROBOT SHORT-CIRCUIT).
  {
    PlanningResult blocked;
    blocked.status = PlanningStatus::BLOCKED_BY_ROBOT;
    blocked.waypoints.push_back(make_waypoint(0.0, 0.0, 0.0, 0.0));
    if (!triage_blocked_by_robot_shortcut_applies(blocked))
    {
      std::cerr << "    expected shortcut to apply: BLOCKED_BY_ROBOT with a "
                   "non-empty backup path\n";
      ok = false;
    }

    PlanningResult no_path;
    no_path.status = PlanningStatus::NO_PATH_FOUND;
    no_path.waypoints.push_back(make_waypoint(0.0, 0.0, 0.0, 0.0));
    if (triage_blocked_by_robot_shortcut_applies(no_path))
    {
      std::cerr << "    expected shortcut to NOT apply for NO_PATH_FOUND\n";
      ok = false;
    }

    PlanningResult blocked_no_waypoints;
    blocked_no_waypoints.status = PlanningStatus::BLOCKED_BY_ROBOT;
    if (triage_blocked_by_robot_shortcut_applies(blocked_no_waypoints))
    {
      std::cerr << "    expected shortcut to NOT apply when BLOCKED_BY_ROBOT "
                   "somehow carries no waypoints (defensive check)\n";
      ok = false;
    }
  }

  // Rule 2 (CAP-HIT-WITH-PROGRESS RETRY), gated on best_dist vs. the
  // straight-line distance (kTriageProgressFraction == 0.5).
  {
    PlanningResult capped_close;
    capped_close.debug_stats.search_iteration_limit_hit = 1;
    capped_close.debug_stats.best_dist = 2.0;
    if (!triage_primary_retry_applies(capped_close, 10.0))
    {
      std::cerr << "    expected retry to apply: cap hit + best_dist(2.0) <= "
                   "0.5 * 10.0\n";
      ok = false;
    }

    PlanningResult capped_far;
    capped_far.debug_stats.search_iteration_limit_hit = 1;
    capped_far.debug_stats.best_dist = 8.0;
    if (triage_primary_retry_applies(capped_far, 10.0))
    {
      std::cerr << "    expected retry to NOT apply: best_dist(8.0) > 0.5 * "
                   "10.0\n";
      ok = false;
    }

    PlanningResult at_boundary;
    at_boundary.debug_stats.search_iteration_limit_hit = 1;
    at_boundary.debug_stats.best_dist = 5.0;
    if (!triage_primary_retry_applies(at_boundary, 10.0))
    {
      std::cerr << "    expected retry to apply at the exact "
                   "kTriageProgressFraction boundary (best_dist == 0.5 * "
                   "straight_line_distance)\n";
      ok = false;
    }

    PlanningResult no_cap_hit;
    no_cap_hit.debug_stats.search_iteration_limit_hit = 0;
    no_cap_hit.debug_stats.best_dist = 0.1;
    if (triage_primary_retry_applies(no_cap_hit, 10.0))
    {
      std::cerr << "    expected retry to NOT apply when the iteration cap "
                   "was not hit, regardless of progress\n";
      ok = false;
    }
  }

  // Rule 3 (NEAR-GOAL-COLLISION SKIP-TO-CONTACT), gated on best_dist vs.
  // analytic_threshold AND reject_collision vs. generated_nodes
  // (kTriageCollisionRejectFraction == 0.5).
  {
    PlanningResult contact_shaped;
    contact_shaped.debug_stats.analytic_threshold = 1.0;
    contact_shaped.debug_stats.best_dist = 0.5;
    contact_shaped.debug_stats.generated_nodes = 100;
    contact_shaped.debug_stats.reject_collision = 60;
    if (!triage_skip_to_contact_applies(contact_shaped))
    {
      std::cerr << "    expected skip-to-contact to apply: near goal + "
                   "60/100 collision-reject fraction\n";
      ok = false;
    }

    PlanningResult low_reject_fraction;
    low_reject_fraction.debug_stats.analytic_threshold = 1.0;
    low_reject_fraction.debug_stats.best_dist = 0.5;
    low_reject_fraction.debug_stats.generated_nodes = 100;
    low_reject_fraction.debug_stats.reject_collision = 10;
    if (triage_skip_to_contact_applies(low_reject_fraction))
    {
      std::cerr << "    expected skip-to-contact to NOT apply: only 10/100 "
                   "collision-reject fraction\n";
      ok = false;
    }

    PlanningResult far_from_goal;
    far_from_goal.debug_stats.analytic_threshold = 1.0;
    far_from_goal.debug_stats.best_dist = 5.0;
    far_from_goal.debug_stats.generated_nodes = 100;
    far_from_goal.debug_stats.reject_collision = 90;
    if (triage_skip_to_contact_applies(far_from_goal))
    {
      std::cerr << "    expected skip-to-contact to NOT apply: "
                   "best_dist(5.0) > analytic_threshold(1.0) despite a high "
                   "reject fraction\n";
      ok = false;
    }

    PlanningResult zero_generated;
    zero_generated.debug_stats.analytic_threshold = 1.0;
    zero_generated.debug_stats.best_dist = 0.0;
    zero_generated.debug_stats.generated_nodes = 0;
    zero_generated.debug_stats.reject_collision = 0;
    if (triage_skip_to_contact_applies(zero_generated))
    {
      std::cerr << "    expected skip-to-contact to NOT apply when "
                   "generated_nodes == 0 (denominator guard, not a "
                   "trivial 0 >= 0 misfire)\n";
      ok = false;
    }
  }

  // method_list_has_fine_before_contact: gates rule 3's applicability.
  {
    const std::vector<TransitPlannerStep> default_list = {
        {TransitPlannerMethod::PrimaryHybridAStar},
        {TransitPlannerMethod::FineHybridAStar},
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar},
    };
    if (!method_list_has_fine_before_contact(default_list))
    {
      std::cerr << "    expected true for [Primary, Fine, Contact]\n";
      ok = false;
    }

    const std::vector<TransitPlannerStep> no_contact = {
        {TransitPlannerMethod::PrimaryHybridAStar},
        {TransitPlannerMethod::FineHybridAStar},
    };
    if (method_list_has_fine_before_contact(no_contact))
    {
      std::cerr << "    expected false for [Primary, Fine] (no Contact)\n";
      ok = false;
    }

    const std::vector<TransitPlannerStep> contact_before_fine = {
        {TransitPlannerMethod::ContactBoundaryGeometricHybridAStar},
        {TransitPlannerMethod::FineHybridAStar},
    };
    if (method_list_has_fine_before_contact(contact_before_fine))
    {
      std::cerr << "    expected false when Contact precedes Fine\n";
      ok = false;
    }
  }

  return ok;
}

// D2: gate soundness. All four scenarios share one workspace/robot geometry:
// a 10x4 map, robot inscribed radius 0.15 (front=rear=0.3, width=0.3 ->
// min(0.3,0.3,0.15)=0.15, diameter 0.30), start at (1,2), goal at (9,2), with
// obstacle(s) forming a vertical barrier at x=5 whose only opening (if any)
// is a gap centered on y=2.
bool test_holonomic_gate_soundness()
{
  bool ok = true;

  Params gate_params;
  gate_params.min_x = 0.0;
  gate_params.max_x = 10.0;
  gate_params.min_y = 0.0;
  gate_params.max_y = 4.0;
  gate_params.holonomic_heuristic_resolution = 0.2;
  // robot_boundary_origin_only defaults to true (Params.h), matching the
  // real planner's default boundary mode.

  const Pose start_pose(1.0, 2.0, 0.0);
  const Pose goal_pose(9.0, 2.0, 0.0);

  // gap_half_width < 0 means the two barrier halves overlap (fully sealed).
  auto build_barrier_scenario = [&](double gap_half_width, EntityStore &store,
                                    TimeTable &timetable)
  {
    RobotMeta *robot = store.add_robot("robot1", start_pose);
    robot->size.front_length = 0.3;
    robot->size.rear_length = 0.3;
    robot->size.width = 0.3;

    // Lower half of the barrier: spans y in [0, 2 - gap_half_width].
    const double lower_extent = 2.0 - gap_half_width;
    ObjectMeta *lower = store.add_object(
        "barrier_lower", Pose(5.0, lower_extent / 2.0, 0.0));
    lower->size.front_length = 0.25;
    lower->size.rear_length = 0.25;
    lower->size.width = lower_extent;

    // Upper half of the barrier: spans y in [2 + gap_half_width, 4].
    const double upper_extent = 4.0 - (2.0 + gap_half_width);
    ObjectMeta *upper = store.add_object(
        "barrier_upper",
        Pose(5.0, 2.0 + gap_half_width + upper_extent / 2.0, 0.0));
    upper->size.front_length = 0.25;
    upper->size.rear_length = 0.25;
    upper->size.width = upper_extent;

    timetable.add_initial(store.entities);
    return robot;
  };

  // Scenario 1: fully sealed (0.1 overlap) -> unreachable.
  {
    EntityStore store;
    TimeTable timetable(0.5);
    RobotMeta *robot = build_barrier_scenario(-0.1, store, timetable);
    if (!holonomic_gate_unreachable(start_pose, goal_pose, robot, timetable,
                                    store.entities, gate_params, 0.0))
    {
      std::cerr << "    expected UNREACHABLE: barrier has no gap at all "
                   "(0.1 overlap)\n";
      ok = false;
    }
  }

  // Scenario 2: 0.5-wide gap (> inscribed diameter 0.30) -> reachable.
  {
    EntityStore store;
    TimeTable timetable(0.5);
    RobotMeta *robot = build_barrier_scenario(0.25, store, timetable);
    if (holonomic_gate_unreachable(start_pose, goal_pose, robot, timetable,
                                   store.entities, gate_params, 0.0))
    {
      std::cerr << "    expected REACHABLE: 0.5-wide gap is wider than the "
                   "robot's inscribed diameter (0.30)\n";
      ok = false;
    }
  }

  // Scenario 3: 0.1-wide gap (< inscribed diameter 0.30) -> still
  // unreachable. A corridor narrower than the inscribed diameter blocks even
  // the under-approximated disc, so "unreachable" here is correct and sound
  // (the real, larger robot is at least as wide as the disc).
  {
    EntityStore store;
    TimeTable timetable(0.5);
    RobotMeta *robot = build_barrier_scenario(0.05, store, timetable);
    if (!holonomic_gate_unreachable(start_pose, goal_pose, robot, timetable,
                                    store.entities, gate_params, 0.0))
    {
      std::cerr << "    expected UNREACHABLE: 0.1-wide gap is narrower than "
                   "the robot's inscribed diameter (0.30), so even the "
                   "under-approximated disc cannot pass\n";
      ok = false;
    }
  }

  // Scenario 4: sealed barrier (as in scenario 1), but built entirely from a
  // single object with a scheduled future move after the gate's search start
  // time -- treated as non-static, so it must be excluded from the gate's
  // obstacle set entirely, leaving the corridor open.
  {
    EntityStore store;
    const Pose blocking_pose(5.0, 2.0, 0.0);
    RobotMeta *robot = store.add_robot("robot1", start_pose);
    robot->size.front_length = 0.3;
    robot->size.rear_length = 0.3;
    robot->size.width = 0.3;
    ObjectMeta *mover = store.add_object("mover", blocking_pose);
    mover->size.front_length = 0.25;
    mover->size.rear_length = 0.25;
    mover->size.width = 4.0; // fully seals the y in [0,4] span at x=5.

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    Trajectory move_away;
    move_away.entity = mover;
    move_away.is_transfer = false;
    move_away.start_time = 0.0;
    move_away.waypoints.push_back(
        make_waypoint(blocking_pose.x, blocking_pose.y, blocking_pose.yaw, 0.0));
    move_away.waypoints.push_back(make_waypoint(20.0, 20.0, 0.0, 10.0));
    timetable.add_trajectory(move_away);

    if (!store.entities.count("mover") ||
        timetable.is_entity_static_after(0.0, store.entities.at("mover")))
    {
      std::cerr << "    test setup error: 'mover' should be non-static after "
                   "t=0 (it has a scheduled move to (20,20) at t=10)\n";
      ok = false;
    }

    if (holonomic_gate_unreachable(start_pose, goal_pose, robot, timetable,
                                   store.entities, gate_params, 0.0))
    {
      std::cerr << "    expected REACHABLE: the only blocking object has a "
                   "scheduled future move, so it must be excluded as "
                   "non-static\n";
      ok = false;
    }
  }

  return ok;
}

// D3: flags-off regression. Guards against Stage 3's new fields accidentally
// defaulting to (or being silently flipped to) enabled: with both at their
// compiled-in default (false), plan_initial_transit's method loop must be
// bit-identical to an explicitly-confirmed-off baseline run of the same
// scenario used by the PlanTimingStats smoke test above (idle blocker parked
// exactly on robot1's initial-transit goal, forcing a mechanism-A
// relocation).
bool test_tier_triage_gate_flags_off_regression()
{
  bool ok = true;

  RuntimeOptions defaults;
  if (defaults.enable_tier_triage || defaults.enable_tier_gate)
  {
    std::cerr << "    RuntimeOptions defaults changed: enable_tier_triage "
                 "and enable_tier_gate must both default to false\n";
    ok = false;
  }

  auto run_once = [](const RuntimeOptions &options, bool &out_success,
                     double &out_abs_start, double &out_abs_end)
  {
    Params params = make_push_demo_params();

    EntityStore store;
    const Pose start_pose(0.5, 1.0, 0.0);
    const Pose goal_pose(3.0, 1.0, 0.0);
    RobotMeta *robot1 = store.add_mars_runtime_robot("robot1", start_pose);
    store.add_mars_runtime_robot("robot2", goal_pose);

    TimeTable timetable(0.5);
    timetable.add_initial(store.entities);

    out_abs_start = -1.0;
    out_abs_end = -1.0;
    out_success = plan_initial_transit(robot1, goal_pose, 0.0, timetable,
                                       store.entities, params, options,
                                       &out_abs_start, &out_abs_end, {},
                                       nullptr, nullptr);
  };

  RuntimeOptions baseline; // explicitly-confirmed-off "direct call" baseline
  baseline.enable_tier_triage = false;
  baseline.enable_tier_gate = false;

  bool success_baseline = false;
  double abs_start_baseline = -1.0;
  double abs_end_baseline = -1.0;
  run_once(baseline, success_baseline, abs_start_baseline, abs_end_baseline);

  bool success_defaults = false;
  double abs_start_defaults = -1.0;
  double abs_end_defaults = -1.0;
  run_once(defaults, success_defaults, abs_start_defaults, abs_end_defaults);

  if (!success_baseline)
  {
    std::cerr << "    baseline scenario (idle blocker on initial-transit "
                 "goal) was expected to succeed\n";
    ok = false;
  }
  if (success_baseline != success_defaults)
  {
    std::cerr << "    flags-off regression: success mismatch (baseline="
              << success_baseline << ", defaults=" << success_defaults
              << ")\n";
    ok = false;
  }
  if (!near(abs_start_baseline, abs_start_defaults, 1e-9))
  {
    std::cerr << "    flags-off regression: abs_start mismatch (baseline="
              << abs_start_baseline << ", defaults=" << abs_start_defaults
              << ")\n";
    ok = false;
  }
  if (!near(abs_end_baseline, abs_end_defaults, 1e-9))
  {
    std::cerr << "    flags-off regression: abs_end mismatch (baseline="
              << abs_end_baseline << ", defaults=" << abs_end_defaults
              << ")\n";
    ok = false;
  }

  return ok;
}

} // namespace

int main(int argc, char **argv)
{
  g_test_executable_path = std::filesystem::path(argv[0]);

  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--visualize") == 0)
    {
      g_enable_visualization = true;
    }
    else if (std::strcmp(argv[i], "--include-known-failure-repros") == 0)
    {
      g_include_known_failure_repros = true;
    }
  }

  if (g_enable_visualization)
  {
    std::cout << "[  INFO    ] Visualization enabled via --visualize\n";
  }
  if (g_include_known_failure_repros)
  {
    std::cout << "[  INFO    ] Known-failure repro tests enabled\n";
  }

  using TestFn = std::function<bool()>;
  const std::vector<std::pair<std::string, TestFn>> tests = {
      {"Detour around static entities", test_detour_around_static_obstacle},
      {"Wait for dynamic entities", test_wait_for_dynamic_entity},
      {"Transfer updates pushed object pose", test_transfer_updates_object_pose},
      {"Avoid temporarily relocated object",
       test_other_robot_avoids_temporarily_relocated_object},
      {"Rearranged object becomes static obstacle",
       test_rearranged_object_is_static_obstacle},
      {"Idle blocker relocated off initial transit goal",
       test_idle_blocker_relocated_off_initial_transit_goal},
      {"Safe parking pose remains safe until last timestamp",
       test_safe_parking_pose_safe_until_last_timestamp},
      {"Safe parking trajectory is collision-free",
       test_safe_parking_trajectory_collision_free},
      {"Expand safe parking mode finds feasible candidate",
       test_expand_safe_parking_mode_finds_feasible_candidate},
      {"ClearHint validates the real scheduling window",
       test_clear_hint_validates_real_scheduling_window},
      {"Transit segment replan avoids newly-delivered object",
       test_replan_transit_segment_after_failed_schedule_avoids_new_object},
      {"Anchor-first contact segment regression",
       test_anchor_first_contact_segment_regression},
      {"QModel gradient check (linear + MLP)", test_qmodel_gradient_check},
      {"QModel MLP fits nonlinear target", test_qmodel_mlp_fits_nonlinear_target},
      {"QModel save/load round-trip (linear + MLP)", test_qmodel_save_load_roundtrip},
      {"QModel load dim/hidden mismatch handling", test_qmodel_load_dim_hidden_mismatch},
      {"QModel load missing-file handling", test_qmodel_load_missing_file},
      {"DQN v2 confinement weight", test_dqn_v2_confinement_weight},
      {"DQN v2 blockage weight", test_dqn_v2_blockage_weight},
      {"DQN v2 DAG construction", test_dqn_v2_dag_construction},
      {"DQN v2 forward schedule simulation", test_dqn_v2_forward_sim},
      {"DQN v2 feature vector shape", test_dqn_v2_feature_vector_shape},
      {"Parse family/index from sequence path", test_parse_family_index},
      {"TransitionLogger CSV round-trip", test_transition_logger_csv_roundtrip},
      {"DQN v2 construct_order_v2 step_log completeness + RNG invariance",
       test_dqn_v2_construct_order_step_log_completeness},
      {"DQN v3 feature vector properties (dim/rel_avail/complementarity/transit)",
       test_dqn_v3_feature_vector_properties},
      {"DQN v3 ForwardSim preview_for/commit_for", test_dqn_v3_preview_commit_for},
      {"DQN v3 construct_order_v3 validity + determinism",
       test_dqn_v3_construct_order_validity},
      {"DQN v3 signature distinguishes assignment",
       test_dqn_v3_signature_distinguishes_assignment},
      {"DQN v3 dispatcher-prior equivalence", test_dqn_v3_dispatcher_prior_equivalence},
      {"DQN v3 construct_order_v3 step_log completeness + RNG invariance",
       test_dqn_v3_construct_order_step_log_completeness},
      {"DQN v4 feature vector (dim12/dims0-9 match v3/load_imbalance/time-not-count)",
       test_dqn_v4_feature_vector},
      {"DQN v4 idle_robot_congestion", test_dqn_v4_idle_robot_congestion},
      {"QModel logistic gradient check (linear + MLP, pos_weight != 1)",
       test_qmodel_logistic_gradient_check},
      {"DQN v3 decomposed ingest (f_replay/q_replay)", test_dqn_v3_decomposed_ingest},
      {"DQN v3 decomposed selection (filter-argmax/fallback/tie-break)",
       test_dqn_v3_decomposed_selection},
      {"DQN v3 scoring-off invariance (penalty mode ignores f_head)",
       test_dqn_v3_scoring_off_invariance},
      {"DQN diag: assignment divergence counts", test_assignment_divergence_counts},
      {"DQN v3 relabeled replay correctness (chosen-row features/diverged/state carry-forward)",
       test_dqn_v3_relabeled_replay_correctness},
      {"DQN v3 relabeled replay: unexecuted-tail fallback to intended robot",
       test_dqn_v3_relabeled_replay_unexecuted_tail_fallback},
      {"DQN v3 relabel-off invariance (ignores executed-robot divergence)",
       test_dqn_v3_relabel_off_invariance},
      {"DQN freeze-model flag (skipped train_model/train_model_logistic leaves params untouched)",
       test_dqn_freeze_model_flag},
      {"DQN init-fail-weights load (f_head match/dim-mismatch)", test_dqn_init_fail_weights_load},
      {"Geometry export: resample_to_k_points", test_resample_to_k_points},
      {"Geometry export: JSON string escaping", test_json_escape_string},
      {"EvalPlansCli: parse_eval_plan_line valid inputs", test_parse_eval_plan_line_valid},
      {"EvalPlansCli: parse_eval_plan_line malformed inputs",
       test_parse_eval_plan_line_malformed},
      {"EvalPlansCli: validate_plan_order", test_validate_plan_order},
      {"EvalPlansCli: parse_robot_index_from_name", test_parse_robot_index_from_name},
      {"EvalPlansCli: csv_escape_field", test_csv_escape_field},
      {"EvalPlansCli: --eval-plans end-to-end (CSV shape + malformed-line row)",
       test_eval_plans_cli_end_to_end},
      {"PgTableExport: --export-pg-tables end-to-end (required keys + transit_time spot-check)",
       test_export_pg_tables_end_to_end},
      {"PgTableExport: --export-decision-time-log end-to-end (deterministic construction + "
       "no relabel fields)",
       test_export_decision_time_log_end_to_end},
      {"PlanTimingStats: add()/operator+=/operator+ sum fields correctly",
       test_plan_timing_stats_accumulate},
      {"initialize_params wires Stage 1 fields (safe_parking_expand_max_iterations/"
       "collision_check_time_step) + positive_or fallback",
       test_initialize_params_wires_stage1_fields},
      {"plan_initial_transit: PlanTimingStats attached vs nullptr yields identical result",
       test_plan_initial_transit_timing_stats_do_not_affect_result},
      {"Stage 2: TimeTable::for_each_pose matches get_poses at every sample kind",
       test_for_each_pose_matches_get_poses},
      {"Stage 2: TerminalHoldCache matches the uncached forward scan (incl. C empty)",
       test_terminal_hold_cache_matches_forward_scan},
      {"Stage 2: find_safe_start_time matches the uncached candidate-loop replica",
       test_find_safe_start_time_matches_uncached_path},
      {"Stage 3: tier-triage predicates (blocked-shortcut/primary-retry/skip-to-contact)",
       test_tier_triage_predicates},
      {"Stage 3: holonomic gate soundness (sealed/wide-gap/narrow-gap/moving-object)",
       test_holonomic_gate_soundness},
      {"Stage 3: tier-triage/tier-gate flags-off regression vs. direct-call baseline",
       test_tier_triage_gate_flags_off_regression},
  };

  std::vector<std::pair<std::string, TestFn>> all_tests = tests;
  if (g_include_known_failure_repros)
  {
    all_tests.push_back({"Task 5 b8 boundary/contact reduced repro",
                         test_task5_b8_boundary_contact_repro});
  }

  int failed = 0;
  for (const auto &[name, fn] : all_tests)
  {
    std::cout << "[ RUN      ] " << name << "\n";
    const bool ok = fn();
    if (ok)
    {
      std::cout << "[       OK ] " << name << "\n";
    }
    else
    {
      std::cout << "[  FAILED  ] " << name << "\n";
      ++failed;
    }
  }

  if (failed == 0)
  {
    std::cout << "[  PASSED  ] " << all_tests.size() << " tests.\n";
    return 0;
  }
  std::cout << "[  FAILED  ] " << failed << " tests.\n";
  return 1;
}
