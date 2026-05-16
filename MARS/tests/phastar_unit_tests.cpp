#include <PHAstar/PHAstar.h>

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
