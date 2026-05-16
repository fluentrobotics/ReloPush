/*****************************************************************
 * ReloPush Single-Robot Setup & Visualization Helpers
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <ReloPushSetup.h>
#include <PHAstarPushDemoOptions.h>
#include <PHAstarPushDemoTypes.h>
#include <PHAstar/Entities.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/TimeTable.h>
#include <PHAstar/Visualization.h>
#include <PHAstar/Params.h>
#include <Task.h>
#include <ReloPush/SerializeFinalSequence.h>  // for FinalAllocation, HandoffInstanceInfo

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// Functions still defined in PHAstar_push_demo.cpp (not yet extracted)
Params initialize_params(const std::vector<FinalAllocation> &loadedSequence,
                         const RuntimeOptions &options);
std::string default_sequence_path();


double compute_relopush_single_robot_makespan(
    const std::vector<FinalAllocation> &loaded_sequence)
{
  constexpr float v_p = 0.28f;
  constexpr float v_np = 0.35f;
  constexpr float v_backward = -0.3f;

  auto compute_segment_duration =
      [&](const ReloPush::StatePathPtr &path_ptr, bool is_pushing) -> double
  {
    if (!path_ptr || path_ptr->empty())
    {
      return 0.0;
    }

    ReloPush::State previous_state = path_ptr->front();
    double elapsed = 0.0;
    bool is_prev_forward = true;

    for (std::size_t i = 1; i < path_ptr->size(); ++i)
    {
      const ReloPush::State &next_state = path_ptr->at(i);
      const double dx = next_state.x - previous_state.x;
      const double dy = next_state.y - previous_state.y;
      const double dist = std::sqrt(dx * dx + dy * dy);

      const double heading_x = std::cos(previous_state.yaw);
      const double heading_y = std::sin(previous_state.yaw);
      const double dot = dx * heading_x + dy * heading_y;

      bool add_wait = false;
      double chosen_vel = 0.0;
      if (is_pushing)
      {
        chosen_vel = v_p;
      }
      else if (dot >= 0.0)
      {
        chosen_vel = v_np;
        if (!is_prev_forward)
          add_wait = true;
        is_prev_forward = true;
      }
      else
      {
        chosen_vel = v_backward;
        if (is_prev_forward)
          add_wait = true;
        is_prev_forward = false;
      }

      const double speed = std::fabs(chosen_vel);
      const double dt = (speed > 1e-6) ? (dist / speed) : 0.0;
      elapsed += dt;
      if (add_wait)
      {
        elapsed += 0.8;
      }

      previous_state = next_state;
    }

    return elapsed;
  };

  double total_duration = 0.0;
  for (const auto &allocation : loaded_sequence)
  {
    total_duration += compute_segment_duration(allocation.firstApproachPath, false);

    if (allocation.obsReloPaths)
    {
      for (const auto &obs_path : *allocation.obsReloPaths)
      {
        auto path_ptr = obs_path.toStatePath();
        if (path_ptr && !path_ptr->empty())
        {
          path_ptr->at(path_ptr->size() - 1) =
              path_ptr->back().get_postPush(Constants::obs_relo_offset);
        }
        total_duration += compute_segment_duration(path_ptr, obs_path.is_pushing);
      }
    }

    for (std::size_t i = 0; i < allocation.paths.size(); ++i)
    {
      const auto &edge_group = allocation.paths[i];
      for (std::size_t path_idx = 0; path_idx < edge_group.paths.size(); ++path_idx)
      {
        auto edge_path = edge_group.paths[path_idx]->toStatePath();
        if (edge_path && !edge_path->empty() &&
            edge_group.paths.size() > 1 && path_idx == 0)
        {
          edge_path->push_back(
              edge_path->back().get_postPush(Constants::additional_push_dist));
        }
        total_duration += compute_segment_duration(
            edge_path,
            edge_group.paths[path_idx]->is_pushing);
      }

      if (allocation.edgeTransitPaths.size() > i &&
          !allocation.edgeTransitPaths.empty())
      {
        total_duration += compute_segment_duration(
            allocation.edgeTransitPaths[i],
            false);
      }
    }
  }

  return total_duration;
}

RobotMeta *make_relopush_robot()
{
  RobotMeta *robot = new RobotMeta;
  robot->name = "ReloPush-BOSS single robot";
  robot->type = EntityType::ROBOT;
  robot->size.front_length = 0.36;
  robot->size.rear_length = 0.12;
  robot->size.width = 0.275;
  robot->min_turning_radius = 1.43;
  robot->min_turning_radius_transit = 1.02;
  robot->min_turning_radius_transfer = 1.43;
  robot->wheel_base = 0.29;
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;
  return robot;
}

bool find_first_relopush_robot_pose(const std::vector<FinalAllocation> &loaded_sequence,
                                    Pose &out_pose)
{
  for (const auto &allocation : loaded_sequence)
  {
    if (allocation.firstApproachPath && !allocation.firstApproachPath->empty())
    {
      out_pose = PoseFromReloPushState(allocation.firstApproachPath->front());
      return true;
    }

    if (allocation.obsReloPaths)
    {
      for (const auto &obs_path : *allocation.obsReloPaths)
      {
        auto path = obs_path.toStatePath();
        if (path && !path->empty())
        {
          out_pose = PoseFromReloPushState(path->front());
          return true;
        }
      }
    }

    for (const auto &edge_group : allocation.paths)
    {
      for (const auto &edge_path : edge_group.paths)
      {
        if (!edge_path)
          continue;
        auto path = edge_path->toStatePath();
        if (path && !path->empty())
        {
          out_pose = PoseFromReloPushState(path->front());
          return true;
        }
      }
    }

    for (const auto &path : allocation.edgeTransitPaths)
    {
      if (path && !path->empty())
      {
        out_pose = PoseFromReloPushState(path->front());
        return true;
      }
    }
  }

  return false;
}

Trajectory make_relopush_trajectory(const ReloPush::StatePathPtr &path,
                                    bool is_transfer,
                                    RobotMeta *robot,
                                    EntityMeta *transferred_object,
                                    double start_time)
{
  Trajectory traj;
  traj.entity = robot;
  traj.transferred_object = transferred_object;
  traj.start_time = start_time;
  traj.is_transfer = is_transfer;
  traj.kind = is_transfer ? TrajectoryKind::TRANSFER : TrajectoryKind::TRANSIT;

  if (!path || path->empty())
    return traj;

  traj.waypoints.reserve(path->size());
  for (const auto &state : *path)
  {
    Waypoint wp = WaypointFromReloPushState(state);
    wp.time = 0.0;
    wp.linear_velocity = is_transfer ? robot->speed_transfer
                                     : robot->speed_transit;
    traj.waypoints.push_back(wp);
  }

  traj.CalcualteTimeStamps(robot);
  return traj;
}

void append_relopush_trajectory(TimeTable &timetable,
                                RobotMeta *robot,
                                EntityMeta *transferred_object,
                                const ReloPush::StatePathPtr &path,
                                bool is_transfer,
                                double &current_time)
{
  Trajectory traj = make_relopush_trajectory(
      path, is_transfer, robot, transferred_object, current_time);
  if (traj.waypoints.empty())
    return;

  timetable.add_trajectory(traj);
  current_time += traj.waypoints.back().time;
}

void append_relopush_edge_path(TimeTable &timetable,
                               RobotMeta *robot,
                               EntityMeta *transferred_object,
                               const EdgePath &edge_path,
                               double &current_time)
{
  append_relopush_trajectory(timetable, robot, transferred_object,
                             edge_path.toStatePath(),
                             edge_path.is_pushing,
                             current_time);
}

void visualize_relopush_plan(
    int argc,
    char **argv,
    const std::vector<FinalAllocation> &loaded_sequence,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const RuntimeOptions &options)
{
  if (loaded_sequence.empty())
    return;

  std::vector<std::unique_ptr<EntityMeta>> entity_storage;
  std::unordered_map<std::string, EntityMeta *> entities;

  RobotMeta *robot = make_relopush_robot();
  Pose first_robot_pose{};
  if (find_first_relopush_robot_pose(loaded_sequence, first_robot_pose))
    robot->initial_pose = first_robot_pose;
  entity_storage.emplace_back(robot);
  entities[robot->name] = robot;

  for (const auto &[name, info] : loaded_sequence.front().snapshot.mo_list)
  {
    ObjectMeta *obj = new ObjectMeta;
    obj->name = name;
    obj->type = EntityType::OBJECT;
    obj->initial_pose = {info.x, info.y, NormalizeReloPushYaw(info.nominalOrientation)};
    obj->size.front_length = 0.075;
    obj->size.rear_length = 0.075;
    obj->size.width = 0.15;
    entity_storage.emplace_back(obj);
    entities[name] = obj;
  }

  TimeTable timetable;
  timetable.add_initial(entities);

  double current_time = 0.0;
  for (const auto &allocation : loaded_sequence)
  {
    append_relopush_trajectory(timetable, robot, nullptr,
                               allocation.firstApproachPath,
                               false, current_time);

    if (allocation.obsReloPaths)
    {
      std::size_t obs_path_base_idx = 0;
      for (std::size_t obs_ind = 1;
           obs_ind + 1 < allocation.vertexChain.size();
           ++obs_ind)
      {
        const auto obs_it = entities.find(allocation.vertexChain[obs_ind].name);
        EntityMeta *obs_entity =
            (obs_it != entities.end()) ? obs_it->second : nullptr;

        const std::size_t push_path_idx = obs_path_base_idx;
        const std::size_t post_path_idx = obs_path_base_idx + 1;
        if (allocation.obsReloPaths->size() > push_path_idx)
        {
          append_relopush_edge_path(
              timetable, robot, obs_entity,
              allocation.obsReloPaths->at(push_path_idx), current_time);
        }
        if (allocation.obsReloPaths->size() > post_path_idx)
        {
          append_relopush_edge_path(
              timetable, robot, nullptr,
              allocation.obsReloPaths->at(post_path_idx), current_time);
        }
        obs_path_base_idx += 2;
      }
    }

    for (std::size_t edge_idx = 0; edge_idx < allocation.paths.size(); ++edge_idx)
    {
      const auto &edge_group = allocation.paths[edge_idx];
      EntityMeta *edge_object = nullptr;
      auto edge_object_it = entities.find(allocation.object.name);
      if (edge_object_it != entities.end())
      {
        edge_object = edge_object_it->second;
      }

      for (std::size_t path_idx = 0; path_idx < edge_group.paths.size(); ++path_idx)
      {
        if (!edge_group.paths[path_idx])
          continue;

        auto edge_path = edge_group.paths[path_idx]->toStatePath();
        if (edge_path && !edge_path->empty() &&
            edge_group.paths.size() > 1 && path_idx == 0)
        {
          edge_path->push_back(
              edge_path->back().get_postPush(Constants::additional_push_dist));
        }

        append_relopush_trajectory(
            timetable, robot, edge_object, edge_path,
            edge_group.paths[path_idx]->is_pushing, current_time);
      }

      if (allocation.edgeTransitPaths.size() > edge_idx &&
          allocation.edgeTransitPaths[edge_idx])
      {
        append_relopush_trajectory(
            timetable, robot, nullptr,
            allocation.edgeTransitPaths[edge_idx],
            false, current_time);
      }
    }
  }

  Params params = initialize_params(loaded_sequence, options);
  std::ostringstream context;
  context << "ReloPush-BOSS single-robot plan replay"
          << "\nInstance: " << instance_info.file_name
          << "\nIndex: " << instance_info.instance_index
          << "\nRobot dimensions: front=0.36, rear=0.12, width=0.275"
          << "\nSpeeds: transit=0.20, transfer=0.15";

  std::cout << "[Debug] Visualizing ReloPush-BOSS single-robot plan "
            << "before multi-robot allocation. Replay makespan="
            << std::fixed << std::setprecision(2)
            << timetable.get_max_time() << "s" << std::endl;
  show_results(argc, argv, timetable, entities, params);
}

ReloPush::HandoffInstanceInfo default_instance_info(const RuntimeOptions &options)
{
  ReloPush::HandoffInstanceInfo info;
  std::string input_path = options.input_sequence_path.empty()
                               ? default_sequence_path()
                               : options.input_sequence_path;
  const std::size_t slash_pos = input_path.find_last_of("/\\");
  info.file_name = (slash_pos == std::string::npos)
                       ? input_path
                       : input_path.substr(slash_pos + 1);
  info.instance_index = -1;
  return info;
}

std::uint32_t mix_seed(std::uint32_t seed, std::uint32_t salt)
{
  std::uint32_t x = seed ^ (salt + 0x9e3779b9u + (seed << 6) + (seed >> 2));
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}
