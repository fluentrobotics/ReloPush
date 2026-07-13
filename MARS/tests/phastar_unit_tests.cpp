#include <PHAstar/PHAstar.h>
#include <DqnQModel.h>
#include <DqnFeaturesV2.h>
#include <DqnAllocationSearch.h>
#include <GeometryExport.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iostream>
#include <limits>
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
      {"Safe parking pose remains safe until last timestamp",
       test_safe_parking_pose_safe_until_last_timestamp},
      {"Safe parking trajectory is collision-free",
       test_safe_parking_trajectory_collision_free},
      {"Expand safe parking mode finds feasible candidate",
       test_expand_safe_parking_mode_finds_feasible_candidate},
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
      {"Geometry export: resample_to_k_points", test_resample_to_k_points},
      {"Geometry export: JSON string escaping", test_json_escape_string},
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
