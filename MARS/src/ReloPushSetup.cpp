/*****************************************************************
 * ReloPush Single-Robot Setup & Visualization Helpers
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <ReloPushSetup.h>
#include <CsvLogging.h>
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
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// Functions still defined in PHAstar_push_demo.cpp (not yet extracted)
Params initialize_params(const std::vector<FinalAllocation> &loadedSequence,
                         const RuntimeOptions &options);
std::string default_sequence_path();

namespace
{
constexpr double kPushVelocity = 0.28;
constexpr double kTransitVelocity = 0.35;
constexpr double kReverseVelocity = -0.3;
constexpr double kDirectionSwitchWait = 0.8;
constexpr double kNearZeroDot = 1e-9;

struct RelopushSegmentTiming
{
  int segment_index = 0;
  int task_index = 0;
  std::string object_name;
  std::string label;
  bool is_pushing = false;
  std::size_t waypoint_count = 0;
  std::uint64_t path_hash = 0;
  double total_distance = 0.0;
  double double_duration = 0.0;
  double float_duration = 0.0;
  double no_initial_reverse_wait_duration = 0.0;
  int wait_count = 0;
  int reverse_count = 0;
  int initial_reverse_wait_count = 0;
  int classification_disagreements = 0;
  int near_zero_dot_count = 0;
  double min_abs_dot = std::numeric_limits<double>::infinity();
  double first_dot = 0.0;
  double last_dot = 0.0;
  bool first_forward = true;
  bool last_forward = true;
  bool has_motion = false;
  ReloPush::State start;
  ReloPush::State goal;
};

std::uint64_t fnv1a_append(std::uint64_t hash,
                           const void *data,
                           std::size_t size)
{
  constexpr std::uint64_t kPrime = 1099511628211ull;
  const auto *bytes = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0; i < size; ++i)
  {
    hash ^= static_cast<std::uint64_t>(bytes[i]);
    hash *= kPrime;
  }
  return hash;
}

std::uint64_t fnv1a_append_double(std::uint64_t hash, double value)
{
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return fnv1a_append(hash, &bits, sizeof(bits));
}

std::uint64_t hash_state_path(const ReloPush::StatePathPtr &path)
{
  std::uint64_t hash = 1469598103934665603ull;
  if (!path)
    return hash;

  const std::uint64_t count = static_cast<std::uint64_t>(path->size());
  hash = fnv1a_append(hash, &count, sizeof(count));
  for (const auto &state : *path)
  {
    hash = fnv1a_append_double(hash, state.x);
    hash = fnv1a_append_double(hash, state.y);
    hash = fnv1a_append_double(hash, state.yaw);
    hash = fnv1a_append_double(hash, static_cast<double>(state.time));
  }
  return hash;
}

std::uint64_t hash_file_contents(const std::string &path)
{
  std::ifstream ifs(path, std::ios::binary);
  std::uint64_t hash = 1469598103934665603ull;
  if (!ifs)
    return hash;

  char buffer[4096];
  while (ifs)
  {
    ifs.read(buffer, sizeof(buffer));
    const std::streamsize count = ifs.gcount();
    if (count > 0)
      hash = fnv1a_append(hash, buffer, static_cast<std::size_t>(count));
  }
  return hash;
}

std::string hex_u64(std::uint64_t value)
{
  std::ostringstream oss;
  oss << std::hex << std::setw(16) << std::setfill('0') << value;
  return oss.str();
}

ReloPush::StatePathPtr clone_state_path(const ReloPush::StatePathPtr &path)
{
  if (!path)
    return nullptr;
  return std::make_shared<ReloPush::StatePath>(*path);
}

ReloPush::StatePathPtr adjusted_obs_relocation_path(const EdgePath &obs_path)
{
  auto path = clone_state_path(obs_path.toStatePath());
  if (path && !path->empty())
  {
    path->at(path->size() - 1) =
        path->back().get_postPush(Constants::obs_relo_offset);
  }
  return path;
}

ReloPush::StatePathPtr adjusted_edge_path(const EdgePath &edge_path,
                                          bool add_prerelo_push)
{
  auto path = clone_state_path(edge_path.toStatePath());
  if (path && !path->empty() && add_prerelo_push)
  {
    path->push_back(
        path->back().get_postPush(Constants::additional_push_dist));
  }
  return path;
}

RelopushSegmentTiming analyze_relopush_segment(
    const ReloPush::StatePathPtr &path,
    bool is_pushing,
    int segment_index,
    int task_index,
    const std::string &object_name,
    const std::string &label)
{
  RelopushSegmentTiming stats;
  stats.segment_index = segment_index;
  stats.task_index = task_index;
  stats.object_name = object_name;
  stats.label = label;
  stats.is_pushing = is_pushing;
  stats.waypoint_count = path ? path->size() : 0;
  stats.path_hash = hash_state_path(path);

  if (!path || path->empty())
    return stats;

  stats.start = path->front();
  stats.goal = path->back();
  bool is_prev_forward = true;
  bool is_prev_forward_float = true;
  ReloPush::State previous_state = path->front();

  for (std::size_t i = 1; i < path->size(); ++i)
  {
    const ReloPush::State &next_state = path->at(i);
    const double dx = next_state.x - previous_state.x;
    const double dy = next_state.y - previous_state.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    const double heading_x = std::cos(previous_state.yaw);
    const double heading_y = std::sin(previous_state.yaw);
    const double dot = dx * heading_x + dy * heading_y;

    const float dx_f = static_cast<float>(next_state.x - previous_state.x);
    const float dy_f = static_cast<float>(next_state.y - previous_state.y);
    const float dist_f = std::sqrt(dx_f * dx_f + dy_f * dy_f);
    const float heading_x_f = std::cos(static_cast<float>(previous_state.yaw));
    const float heading_y_f = std::sin(static_cast<float>(previous_state.yaw));
    const float dot_f = dx_f * heading_x_f + dy_f * heading_y_f;

    if (!stats.has_motion)
    {
      stats.first_dot = dot;
      stats.first_forward = dot >= 0.0;
      stats.has_motion = true;
    }
    stats.last_dot = dot;
    stats.last_forward = dot >= 0.0;
    stats.total_distance += dist;
    stats.min_abs_dot = std::min(stats.min_abs_dot, std::abs(dot));
    if (std::abs(dot) <= kNearZeroDot)
      ++stats.near_zero_dot_count;

    bool add_wait = false;
    bool add_wait_float = false;
    double chosen_vel = kPushVelocity;
    float chosen_vel_float = static_cast<float>(kPushVelocity);

    if (!is_pushing)
    {
      const bool forward = dot >= 0.0;
      const bool forward_float = dot_f >= 0.0f;
      if (forward)
      {
        chosen_vel = kTransitVelocity;
        if (!is_prev_forward)
          add_wait = true;
        is_prev_forward = true;
      }
      else
      {
        chosen_vel = kReverseVelocity;
        ++stats.reverse_count;
        if (is_prev_forward)
          add_wait = true;
        if (i == 1 && add_wait)
          ++stats.initial_reverse_wait_count;
        is_prev_forward = false;
      }

      if (forward_float)
      {
        chosen_vel_float = static_cast<float>(kTransitVelocity);
        if (!is_prev_forward_float)
          add_wait_float = true;
        is_prev_forward_float = true;
      }
      else
      {
        chosen_vel_float = static_cast<float>(kReverseVelocity);
        if (is_prev_forward_float)
          add_wait_float = true;
        is_prev_forward_float = false;
      }

      if (forward != forward_float)
        ++stats.classification_disagreements;
    }

    const double speed = std::fabs(chosen_vel);
    const double dt = (speed > 1e-6) ? (dist / speed) : 0.0;
    stats.double_duration += dt;
    stats.no_initial_reverse_wait_duration += dt;
    if (add_wait)
    {
      stats.double_duration += kDirectionSwitchWait;
      ++stats.wait_count;
      if (i != 1)
        stats.no_initial_reverse_wait_duration += kDirectionSwitchWait;
    }

    const float speed_f = std::fabs(chosen_vel_float);
    const double dt_f = (speed_f > 1e-6f) ? (dist_f / speed_f) : 0.0;
    stats.float_duration += dt_f;
    if (add_wait_float)
      stats.float_duration += kDirectionSwitchWait;

    previous_state = next_state;
  }

  return stats;
}

std::vector<RelopushSegmentTiming> collect_relopush_segment_timings(
    const std::vector<FinalAllocation> &loaded_sequence)
{
  std::vector<RelopushSegmentTiming> segments;

  auto append_segment =
      [&](const ReloPush::StatePathPtr &path,
          bool is_pushing,
          int task_index,
          const std::string &object_name,
          const std::string &label)
  {
    segments.push_back(analyze_relopush_segment(
        path, is_pushing, static_cast<int>(segments.size()), task_index,
        object_name, label));
  };

  for (std::size_t task_idx = 0; task_idx < loaded_sequence.size(); ++task_idx)
  {
    const auto &allocation = loaded_sequence[task_idx];
    const int task_number = static_cast<int>(task_idx) + 1;
    const std::string object_name = allocation.object.name;
    const std::string task_prefix =
        "task" + std::to_string(task_number) + ":" + object_name;

    append_segment(clone_state_path(allocation.firstApproachPath), false,
                   task_number, object_name, task_prefix + ":first_approach");

    if (allocation.obsReloPaths)
    {
      for (std::size_t obs_idx = 0; obs_idx < allocation.obsReloPaths->size();
           ++obs_idx)
      {
        const auto &obs_path = allocation.obsReloPaths->at(obs_idx);
        append_segment(adjusted_obs_relocation_path(obs_path),
                       obs_path.is_pushing,
                       task_number,
                       object_name,
                       task_prefix + ":obs" + std::to_string(obs_idx + 1));
      }
    }

    for (std::size_t edge_idx = 0; edge_idx < allocation.paths.size();
         ++edge_idx)
    {
      const auto &edge_group = allocation.paths[edge_idx];
      for (std::size_t path_idx = 0; path_idx < edge_group.paths.size();
           ++path_idx)
      {
        if (!edge_group.paths[path_idx])
          continue;

        const bool add_prerelo_push =
            edge_group.paths.size() > 1 && path_idx == 0;
        append_segment(adjusted_edge_path(*edge_group.paths[path_idx],
                                          add_prerelo_push),
                       edge_group.paths[path_idx]->is_pushing,
                       task_number,
                       object_name,
                       task_prefix + ":edge" + std::to_string(edge_idx + 1) +
                           "." + std::to_string(path_idx + 1));
      }

      if (allocation.edgeTransitPaths.size() > edge_idx &&
          allocation.edgeTransitPaths[edge_idx])
      {
        append_segment(clone_state_path(allocation.edgeTransitPaths[edge_idx]),
                       false,
                       task_number,
                       object_name,
                       task_prefix + ":connector" +
                           std::to_string(edge_idx + 1));
      }
    }
  }

  return segments;
}

double carry_direction_total(
    const std::vector<RelopushSegmentTiming> &segments)
{
  double total = 0.0;
  bool previous_forward = true;
  for (const auto &segment : segments)
  {
    total += segment.double_duration -
             kDirectionSwitchWait * segment.wait_count;

    if (segment.is_pushing)
    {
      previous_forward = true;
      continue;
    }

    if (!segment.has_motion)
      continue;

    bool first_direction = segment.first_forward;
    if (first_direction != previous_forward)
      total += kDirectionSwitchWait;

    int internal_wait_count = segment.wait_count -
                              segment.initial_reverse_wait_count;
    total += kDirectionSwitchWait *
             static_cast<double>(std::max(0, internal_wait_count));
    previous_forward = segment.last_forward;
  }
  return total;
}

void write_segment_timing_csv(
    const std::string &path,
    const std::vector<RelopushSegmentTiming> &segments)
{
  std::filesystem::create_directories(diagnostics_dir());
  std::ofstream ofs(path);
  if (!ofs.is_open())
  {
    std::cerr << "[ReloPushDiag] Failed to open CSV: " << path << std::endl;
    return;
  }

  ofs << std::setprecision(17);
  ofs << "segment_index,task_index,object,label,is_pushing,waypoints,path_hash,"
      << "start_x,start_y,start_yaw,goal_x,goal_y,goal_yaw,total_distance,"
      << "double_duration,float_duration,no_initial_reverse_wait_duration,"
      << "wait_count,reverse_count,initial_reverse_wait_count,"
      << "classification_disagreements,near_zero_dot_count,min_abs_dot,"
      << "first_dot,last_dot,first_forward,last_forward\n";

  for (const auto &segment : segments)
  {
    const double min_abs_dot =
        std::isfinite(segment.min_abs_dot) ? segment.min_abs_dot : 0.0;
    ofs << segment.segment_index << ","
        << segment.task_index << ","
        << csv_escape(segment.object_name) << ","
        << csv_escape(segment.label) << ","
        << (segment.is_pushing ? 1 : 0) << ","
        << segment.waypoint_count << ","
        << hex_u64(segment.path_hash) << ","
        << segment.start.x << ","
        << segment.start.y << ","
        << segment.start.yaw << ","
        << segment.goal.x << ","
        << segment.goal.y << ","
        << segment.goal.yaw << ","
        << segment.total_distance << ","
        << segment.double_duration << ","
        << segment.float_duration << ","
        << segment.no_initial_reverse_wait_duration << ","
        << segment.wait_count << ","
        << segment.reverse_count << ","
        << segment.initial_reverse_wait_count << ","
        << segment.classification_disagreements << ","
        << segment.near_zero_dot_count << ","
        << min_abs_dot << ","
        << segment.first_dot << ","
        << segment.last_dot << ","
        << (segment.first_forward ? 1 : 0) << ","
        << (segment.last_forward ? 1 : 0) << "\n";
  }
}

void write_makespan_summary(
    const std::string &path,
    const std::vector<RelopushSegmentTiming> &segments,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const RuntimeOptions &options,
    const std::string &sequence_path,
    double reported_makespan,
    const std::string &csv_path)
{
  std::filesystem::create_directories(diagnostics_dir());
  std::ofstream ofs(path);
  if (!ofs.is_open())
  {
    std::cerr << "[ReloPushDiag] Failed to open summary: " << path << std::endl;
    return;
  }

  double double_total = 0.0;
  double float_total = 0.0;
  double no_initial_reverse_total = 0.0;
  double total_distance = 0.0;
  int total_waits = 0;
  int total_reverse_steps = 0;
  int total_initial_reverse_waits = 0;
  int total_disagreements = 0;
  int total_near_zero = 0;

  for (const auto &segment : segments)
  {
    double_total += segment.double_duration;
    float_total += segment.float_duration;
    no_initial_reverse_total += segment.no_initial_reverse_wait_duration;
    total_distance += segment.total_distance;
    total_waits += segment.wait_count;
    total_reverse_steps += segment.reverse_count;
    total_initial_reverse_waits += segment.initial_reverse_wait_count;
    total_disagreements += segment.classification_disagreements;
    total_near_zero += segment.near_zero_dot_count;
  }

  std::error_code ec;
  const auto file_size = std::filesystem::file_size(sequence_path, ec);
  const std::uint64_t file_hash = hash_file_contents(sequence_path);

  ofs << std::fixed << std::setprecision(9);
  ofs << "[ReloPushMakespanDiagnostics]\n";
  ofs << "summary_csv=" << csv_path << "\n";
  ofs << "instance_file=" << instance_info.file_name << "\n";
  ofs << "instance_index=" << instance_info.instance_index << "\n";
  ofs << "sequence_path=" << sequence_path << "\n";
  ofs << "sequence_file_size="
      << (ec ? std::string("unavailable") : std::to_string(file_size)) << "\n";
  ofs << "sequence_file_fnv1a64=" << hex_u64(file_hash) << "\n";
  const int sequence_task_count =
      segments.empty() ? 0 : segments.back().task_index;
  ofs << "configured_robot_count=" << options.robot_count << "\n";
  ofs << "sequence_tasks=" << sequence_task_count << "\n";
  ofs << "segments=" << segments.size() << "\n";
  ofs << "reported_makespan=" << reported_makespan << "\n";
  ofs << "recomputed_double_total=" << double_total << "\n";
  ofs << "recomputed_float_total=" << float_total << "\n";
  ofs << "double_minus_float=" << (double_total - float_total) << "\n";
  ofs << "no_initial_reverse_wait_total="
      << no_initial_reverse_total << "\n";
  ofs << "carry_direction_between_segments_total="
      << carry_direction_total(segments) << "\n";
  ofs << "total_distance=" << total_distance << "\n";
  ofs << "direction_switch_wait_seconds=" << kDirectionSwitchWait << "\n";
  ofs << "total_direction_switch_waits=" << total_waits << "\n";
  ofs << "total_initial_reverse_waits=" << total_initial_reverse_waits << "\n";
  ofs << "total_reverse_steps=" << total_reverse_steps << "\n";
  ofs << "classification_disagreements_double_vs_float="
      << total_disagreements << "\n";
  ofs << "near_zero_dot_threshold=" << kNearZeroDot << "\n";
  ofs << "near_zero_dot_steps=" << total_near_zero << "\n";
  ofs << "v_push=" << kPushVelocity << "\n";
  ofs << "v_transit=" << kTransitVelocity << "\n";
  ofs << "v_reverse=" << kReverseVelocity << "\n";
  ofs << "Constants::additional_push_dist="
      << Constants::additional_push_dist << "\n";
  ofs << "Constants::obs_relo_offset=" << Constants::obs_relo_offset << "\n";
  ofs << "sizeof_float=" << sizeof(float) << "\n";
  ofs << "sizeof_double=" << sizeof(double) << "\n";
  ofs << "sizeof_long_double=" << sizeof(long double) << "\n";
  ofs << "FLT_EVAL_METHOD=" << FLT_EVAL_METHOD << "\n";
  ofs << "double_is_iec559="
      << (std::numeric_limits<double>::is_iec559 ? 1 : 0) << "\n";
#if defined(__clang__)
  ofs << "compiler=clang " << __clang_version__ << "\n";
#elif defined(__GNUC__)
  ofs << "compiler=gcc " << __VERSION__ << "\n";
#else
  ofs << "compiler=unknown\n";
#endif
#if defined(__APPLE__)
  ofs << "platform_macro=__APPLE__\n";
#elif defined(__linux__)
  ofs << "platform_macro=__linux__\n";
#else
  ofs << "platform_macro=unknown\n";
#endif
  ofs << "random_seed=" << options.base_random_seed << "\n";
  ofs << "has_fixed_random_seed="
      << (options.has_fixed_random_seed ? 1 : 0) << "\n";
  ofs << "lns_threads=" << options.lns_threads << "\n";
  ofs << "robot_count_option=" << options.robot_count << "\n";
}
} // namespace

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

void write_relopush_makespan_diagnostics(
    const std::vector<FinalAllocation> &loaded_sequence,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const RuntimeOptions &options,
    const std::string &sequence_path,
    double reported_makespan)
{
  const std::string stem =
      "relopush_makespan_" +
      sanitize_filename_component(instance_info.file_name) + "_ind" +
      std::to_string(instance_info.instance_index);
  const std::string summary_path = diagnostics_path(stem + ".txt");
  const std::string csv_path = diagnostics_path(stem + "_segments.csv");

  const auto segments = collect_relopush_segment_timings(loaded_sequence);
  write_segment_timing_csv(csv_path, segments);
  write_makespan_summary(summary_path, segments, instance_info, options,
                         sequence_path, reported_makespan, csv_path);

  std::cout << "[ReloPushDiag] Wrote makespan diagnostics: "
            << summary_path << std::endl;
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
