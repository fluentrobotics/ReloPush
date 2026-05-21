/*****************************************************************
 * CSV Logging & Diagnostics
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <CsvLogging.h>

// Include only lightweight headers to avoid ODR violations from
// PHAstar.h -> Visualization.h (which defines non-inline functions).
// PHAstarPushDemoTypes.h is included for struct definitions; it pulls
// Task.h -> PHAstar.h -> Visualization.h, so we must bypass that chain.
//
// We include PHAstarPushDemoTypes.h from the .cpp because we need the
// full definitions of TaskCsvRow, AllocationRunSummary, etc.
// The Visualization.h ODR issue must be fixed at the source.
#include <PHAstarPushDemoTypes.h>
#include <PHAstar/Entities.h>
#include <PHAstar/Utils.h>
#include <ReloPush/FinalSequenceHandoff.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

// Forward declaration from CollisionScheduling
double shared_collision_check_step(const Params &params);

namespace
{
  void ensure_parent_directory(const std::string &path)
  {
    const std::filesystem::path fs_path(path);
    const std::filesystem::path parent = fs_path.parent_path();
    if (!parent.empty())
      std::filesystem::create_directories(parent);
  }

  std::string format_instance_record_number(double value)
  {
    if (!std::isfinite(value))
      return "INF";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << value;
    return oss.str();
  }

  std::string format_lns_batch_times(
      const std::vector<double> &lns_batch_planning_times_s)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < lns_batch_planning_times_s.size(); ++i)
    {
      if (i > 0)
        oss << ";";
      if (std::isfinite(lns_batch_planning_times_s[i]))
        oss << lns_batch_planning_times_s[i];
      else
        oss << "INF";
    }
    return oss.str();
  }

  std::string format_lns_batch_makespans(
      const std::vector<double> &lns_batch_best_makespans)
  {
    return format_lns_batch_times(lns_batch_best_makespans);
  }

  double sum_lns_batch_times(
      const std::vector<double> &lns_batch_planning_times_s)
  {
    double total = 0.0;
    for (double time_s : lns_batch_planning_times_s)
    {
      if (std::isfinite(time_s))
        total += time_s;
    }
    return total;
  }

  std::string format_lns_batch_counts(
      const std::vector<int> &lns_batch_failed_iterations)
  {
    std::ostringstream oss;
    for (std::size_t i = 0; i < lns_batch_failed_iterations.size(); ++i)
    {
      if (i > 0)
        oss << ";";
      oss << lns_batch_failed_iterations[i];
    }
    return oss.str();
  }
}

std::string csv_escape(const std::string &value)
{
  bool need_quotes = value.find(',') != std::string::npos ||
                     value.find('"') != std::string::npos ||
                     value.find('\n') != std::string::npos;
  if (!need_quotes)
    return value;

  std::string escaped = "\"";
  for (char c : value)
  {
    if (c == '"')
      escaped += "\"\"";
    else
      escaped += c;
  }
  escaped += "\"";
  return escaped;
}

const char *planning_status_name(PlanningStatus status)
{
  switch (status)
  {
  case PlanningStatus::SUCCESS:
    return "SUCCESS";
  case PlanningStatus::START_INVALID_COLLISION:
    return "START_INVALID_COLLISION";
  case PlanningStatus::START_OUT_OF_BOUNDS:
    return "START_OUT_OF_BOUNDS";
  case PlanningStatus::GOAL_INVALID_COLLISION:
    return "GOAL_INVALID_COLLISION";
  case PlanningStatus::GOAL_OUT_OF_BOUNDS:
    return "GOAL_OUT_OF_BOUNDS";
  case PlanningStatus::NO_PATH_FOUND:
    return "NO_PATH_FOUND";
  case PlanningStatus::TIMEOUT_EXCEEDED:
    return "TIMEOUT_EXCEEDED";
  case PlanningStatus::HIGH_COST_UNFEASIBLE:
    return "HIGH_COST_UNFEASIBLE";
  case PlanningStatus::BLOCKED_BY_ROBOT:
    return "BLOCKED_BY_ROBOT";
  case PlanningStatus::INTERNAL_ERROR:
  default:
    return "INTERNAL_ERROR";
  }
}

std::string relopush_output_dir()
{
  return std::string(CMAKE_SOURCE_DIR) + "/results/relopush-out";
}

std::string relopush_sequence_path(const std::string &instance_file_name,
                                   int instance_index)
{
  return relopush_output_dir() + "/result_seq_" + instance_file_name +
         "_ind" + std::to_string(instance_index) + ".b64";
}

std::string index_log_dir()
{
  return std::string(CMAKE_SOURCE_DIR) + "/results/index-logs";
}

std::string index_log_path(const std::string &filename)
{
  return index_log_dir() + "/" + filename;
}

std::string diagnostics_dir()
{
  return std::string(CMAKE_SOURCE_DIR) + "/diagnostics";
}

std::string diagnostics_path(const std::string &filename)
{
  return diagnostics_dir() + "/" + filename;
}

void write_task_csv_log(const std::string &csv_path,
                        const std::vector<TaskCsvRow> &rows)
{
  ensure_parent_directory(csv_path);
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open CSV file: " << csv_path << std::endl;
    return;
  }

  ofs << "task_id,object,status,robot,start_time,end_time,total_waiting,attempts,failure_reason\n";
  ofs << std::fixed << std::setprecision(2);

  for (const auto &row : rows)
  {
    ofs << row.task_id << ","
        << csv_escape(row.object_name) << ","
        << csv_escape(row.status) << ","
        << csv_escape(row.robot_name) << ","
        << row.start_time << ","
        << row.end_time << ","
        << row.total_waiting << ","
        << row.attempts << ","
        << csv_escape(row.failure_reason) << "\n";
  }

  std::cout << "[Log] Wrote task execution CSV: " << csv_path << std::endl;
}

void write_timetable_robot_pose_csv(const std::string &csv_path,
                                    const std::string &scenario_label,
                                    const TimeTable &timetable,
                                    const std::unordered_map<std::string, EntityMeta *> &entities,
                                    double sample_step)
{
  ensure_parent_directory(csv_path);
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open timetable pose CSV: "
              << csv_path << std::endl;
    return;
  }

  ofs << "scenario,robot,time,x,y,yaw,yaw_unwrapped\n";
  ofs << std::fixed << std::setprecision(6);

  const double max_t = timetable.get_max_time();
  for (const auto &[name, entity] : entities)
  {
    if (!entity || entity->type != EntityType::ROBOT)
      continue;

    bool has_prev = false;
    double yaw_unwrapped = 0.0;
    Pose prev_pose{};

    for (double t = 0.0; t <= max_t + 1e-9; t += sample_step)
    {
      Pose pose = timetable.get_pose(entity, t);
      if (!has_prev)
      {
        yaw_unwrapped = pose.yaw;
        has_prev = true;
      }
      else
      {
        yaw_unwrapped += pi_2_pi(pose.yaw - prev_pose.yaw);
      }

      ofs << csv_escape(scenario_label) << ","
          << csv_escape(name) << ","
          << t << ","
          << pose.x << ","
          << pose.y << ","
          << pose.yaw << ","
          << yaw_unwrapped << "\n";

      prev_pose = pose;
    }
  }

  std::cout << "[Log] Wrote timetable robot pose CSV: " << csv_path << std::endl;
}

void write_orientation_jump_csv(const std::string &csv_path,
                                const std::vector<OrientationJumpRecord> &records)
{
  ensure_parent_directory(csv_path);
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open orientation jump CSV: "
              << csv_path << std::endl;
    return;
  }

  ofs << "scenario,source,robot,prev_time,curr_time,prev_x,prev_y,prev_yaw,curr_x,curr_y,curr_yaw,raw_yaw_delta,wrapped_yaw_delta,distance\n";
  ofs << std::fixed << std::setprecision(6);

  for (const auto &record : records)
  {
    ofs << csv_escape(record.scenario_label) << ","
        << csv_escape(record.source) << ","
        << csv_escape(record.robot_name) << ","
        << record.prev_time << ","
        << record.curr_time << ","
        << record.prev_pose.x << ","
        << record.prev_pose.y << ","
        << record.prev_pose.yaw << ","
        << record.curr_pose.x << ","
        << record.curr_pose.y << ","
        << record.curr_pose.yaw << ","
        << record.raw_yaw_delta << ","
        << record.wrapped_yaw_delta << ","
        << record.distance << "\n";
  }

  std::cout << "[Log] Wrote orientation jump CSV: " << csv_path << std::endl;
}

std::vector<OrientationJumpRecord> collect_robot_orientation_jumps(
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    double dense_sample_step,
    double wrapped_jump_threshold)
{
  std::vector<OrientationJumpRecord> out;
  const double max_t = timetable.get_max_time();

  auto maybe_append = [&](const std::string &source,
                          const std::string &robot_name,
                          double prev_time,
                          const Pose &prev_pose,
                          double curr_time,
                          const Pose &curr_pose)
  {
    const double raw_yaw_delta = curr_pose.yaw - prev_pose.yaw;
    const double wrapped_yaw_delta = pi_2_pi(raw_yaw_delta);
    if (std::abs(wrapped_yaw_delta) < wrapped_jump_threshold)
      return;

    OrientationJumpRecord record;
    record.scenario_label = scenario_label;
    record.source = source;
    record.robot_name = robot_name;
    record.prev_time = prev_time;
    record.curr_time = curr_time;
    record.prev_pose = prev_pose;
    record.curr_pose = curr_pose;
    record.raw_yaw_delta = raw_yaw_delta;
    record.wrapped_yaw_delta = wrapped_yaw_delta;
    record.distance = std::hypot(curr_pose.x - prev_pose.x,
                                 curr_pose.y - prev_pose.y);
    out.push_back(record);
  };

  for (const auto &[name, entity] : entities)
  {
    if (!entity || entity->type != EntityType::ROBOT)
      continue;

    bool has_prev_dense = false;
    double prev_dense_time = 0.0;
    Pose prev_dense_pose{};
    for (double t = 0.0; t <= max_t + 1e-9; t += dense_sample_step)
    {
      const Pose pose = timetable.get_pose(entity, t);
      if (has_prev_dense)
      {
        maybe_append("dense-get-pose", name,
                     prev_dense_time, prev_dense_pose,
                     t, pose);
      }
      prev_dense_time = t;
      prev_dense_pose = pose;
      has_prev_dense = true;
    }

    const auto &db = timetable.get_database();
    auto it = db.find(entity);
    if (it == db.end())
      continue;

    bool has_prev_stored = false;
    double prev_stored_time = 0.0;
    Pose prev_stored_pose{};
    for (const auto &[t, pose] : it->second)
    {
      if (has_prev_stored)
      {
        maybe_append("stored-samples", name,
                     prev_stored_time, prev_stored_pose,
                     t, pose);
      }
      prev_stored_time = t;
      prev_stored_pose = pose;
      has_prev_stored = true;
    }
  }

  std::sort(out.begin(), out.end(),
            [](const OrientationJumpRecord &a, const OrientationJumpRecord &b)
            {
              const double a_mag = std::abs(a.wrapped_yaw_delta);
              const double b_mag = std::abs(b.wrapped_yaw_delta);
              if (std::abs(a_mag - b_mag) > 1e-9)
                return a_mag > b_mag;
              if (std::abs(a.curr_time - b.curr_time) > 1e-9)
                return a.curr_time < b.curr_time;
              if (a.robot_name != b.robot_name)
                return a.robot_name < b.robot_name;
              return a.source < b.source;
            });
  return out;
}

void log_timetable_orientation_diagnostics(
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params)
{
  const std::string label_slug = sanitize_filename_component(scenario_label);
  const std::string pose_csv = diagnostics_path(
      "timetable_robot_pose_log_" + label_slug + ".csv");
  const std::string jump_csv = diagnostics_path(
      "timetable_robot_yaw_jumps_" + label_slug + ".csv");

  const double sample_step = std::max(1e-3, std::min(0.05, shared_collision_check_step(params)));
  const double jump_threshold = std::max(0.75, params.yaw_resolution * 2.0);

  write_timetable_robot_pose_csv(pose_csv, scenario_label, timetable, entities,
                                 sample_step);
  auto jump_records =
      collect_robot_orientation_jumps(scenario_label, timetable, entities,
                                      sample_step, jump_threshold);
  write_orientation_jump_csv(jump_csv, jump_records);

  if (jump_records.empty())
  {
    std::cout << "[Diag] No suspicious robot yaw jumps found in scenario "
              << scenario_label << " using threshold "
              << std::fixed << std::setprecision(2)
              << jump_threshold << " rad." << std::endl;
    return;
  }

  std::cout << "[Diag] Found " << jump_records.size()
            << " suspicious robot yaw jumps in scenario "
            << scenario_label << "." << std::endl;
  const std::size_t preview_count = std::min<std::size_t>(5, jump_records.size());
  for (std::size_t i = 0; i < preview_count; ++i)
  {
    const auto &record = jump_records[i];
    std::cout << "        [" << record.source << "] "
              << record.robot_name << " t=" << std::fixed << std::setprecision(2)
              << record.prev_time << " -> " << record.curr_time
              << " yaw=" << record.prev_pose.yaw << " -> " << record.curr_pose.yaw
              << " wrapped_delta=" << record.wrapped_yaw_delta
              << " raw_delta=" << record.raw_yaw_delta
              << " distance=" << record.distance << std::endl;
  }
}

std::string sanitize_filename_component(const std::string &label)
{
  std::string out;
  out.reserve(label.size());

  for (char c : label)
  {
    unsigned char uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc))
      out.push_back(static_cast<char>(std::tolower(uc)));
    else
      out.push_back('_');
  }

  while (!out.empty() && out.front() == '_')
    out.erase(out.begin());
  while (!out.empty() && out.back() == '_')
    out.pop_back();

  if (out.empty())
    out = "scenario";
  return out;
}

std::string default_result_summary_path(const ReloPush::HandoffInstanceInfo &instance_info,
                                        const std::string &scenario_label)
{
  return diagnostics_path(
         "mars_result_summary_" +
         sanitize_filename_component(instance_info.file_name) + "_ind" +
         std::to_string(instance_info.instance_index) + "_" +
         sanitize_filename_component(scenario_label) + ".png");
}

void write_allocation_search_summary_csv(
    const std::string &csv_path,
    const std::vector<AllocationRunSummary> &summaries,
    double greedy_makespan)
{
  ensure_parent_directory(csv_path);
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open allocation summary CSV: " << csv_path << std::endl;
    return;
  }

  ofs << "scenario,feasible,makespan,delta_vs_greedy,successful_tasks,failed_tasks,parking_seed\n";
  ofs << std::fixed << std::setprecision(2);

  for (const auto &summary : summaries)
  {
    double delta = (std::isfinite(summary.makespan) && std::isfinite(greedy_makespan))
                       ? (summary.makespan - greedy_makespan)
                       : std::numeric_limits<double>::infinity();
    ofs << csv_escape(summary.label) << ","
        << (summary.all_tasks_succeeded ? "TRUE" : "FALSE") << ",";
    if (std::isfinite(summary.makespan))
      ofs << summary.makespan;
    else
      ofs << "INF";
    ofs << ",";
    if (std::isfinite(delta))
      ofs << delta;
    else
      ofs << "INF";
    ofs << ","
        << summary.successful_tasks << ","
        << summary.failed_tasks << ","
        << summary.parking_seed << "\n";
  }

  std::cout << "[Log] Wrote allocation comparison CSV: " << csv_path << std::endl;
}

void write_search_trial_records_csv(
    const std::string &csv_path,
    const std::vector<SearchTrialRecord> &records)
{
  ensure_parent_directory(csv_path);
  std::ofstream ofs(csv_path);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open search trial CSV: " << csv_path << std::endl;
    return;
  }

  ofs << "search_label,iteration,feasible,makespan,successful_tasks,failed_tasks,parking_seed,order\n";
  ofs << std::fixed << std::setprecision(2);

  for (const auto &record : records)
  {
    ofs << csv_escape(record.search_label) << ","
        << record.iteration << ","
        << (record.feasible ? "TRUE" : "FALSE") << ",";
    if (std::isfinite(record.makespan))
      ofs << record.makespan;
    else
      ofs << "INF";
    ofs << ","
        << record.successful_tasks << ","
        << record.failed_tasks << ","
        << record.parking_seed << ","
        << csv_escape(record.order_description) << "\n";
  }

  std::cout << "[Log] Wrote search trial CSV: " << csv_path << std::endl;
}

void write_instance_run_record_csv(
    const std::string &csv_path,
    const ReloPush::HandoffInstanceInfo &instance_info,
    double relopush_single_robot_makespan,
    double greedy_makespan,
    double lns_best_makespan,
    int lns_iterations,
    int lns_failed_iterations,
    double greedy_allocation_planning_time_s,
    const std::vector<double> &lns_batch_planning_times_s,
    const std::vector<double> &lns_batch_best_makespans,
    const std::vector<int> &lns_batch_failed_iterations,
    int path_max_search_iterations_default,
    int path_max_search_iterations_fine,
    int safe_parking_max_search_iterations,
    int lns_threads,
    int robot_count,
    const std::string &lns_mode,
    const std::string &order_constraint_learning,
    const std::string &best_overall_label,
    double best_overall_makespan)
{
  const std::string header =
      "file_name,instance_index,relopush_single_robot_makespan,"
      "greedy_allocation_makespan,lns_best_makespan,lns_iterations,"
      "lns_failed_iterations,"
      "greedy_allocation_planning_time_s,lns_batch_planning_times_s,"
      "lns_total_planning_time_s,lns_batch_best_makespans,"
      "lns_batch_failed_iterations,"
      "path_max_search_iterations_default,path_max_search_iterations_fine,"
      "safe_parking_max_search_iterations,lns_threads,robot_count,lns_mode,"
      "order_constraint_learning,best_overall_label,best_overall_makespan";

  bool write_header = false;
  {
    std::ifstream ifs(csv_path);
    if (!ifs.good() || ifs.peek() == std::ifstream::traits_type::eof())
    {
      write_header = true;
    }
    else
    {
      bool found_current_header = false;
      std::string line;
      while (std::getline(ifs, line))
      {
        if (!line.empty() && line.back() == '\r')
          line.pop_back();
        if (line == header)
        {
          found_current_header = true;
          break;
        }
      }
      write_header = !found_current_header;
    }
  }

  ensure_parent_directory(csv_path);
  std::ofstream ofs(csv_path, std::ios::app);
  if (!ofs.is_open())
  {
    std::cerr << "[Log] Failed to open instance record CSV: " << csv_path << std::endl;
    return;
  }

  if (write_header)
  {
    ofs << header << "\n";
  }

  ofs << csv_escape(instance_info.file_name) << ","
      << instance_info.instance_index << ","
      << format_instance_record_number(relopush_single_robot_makespan) << ","
      << format_instance_record_number(greedy_makespan) << ","
      << format_instance_record_number(lns_best_makespan) << ","
      << lns_iterations << ","
      << lns_failed_iterations << ","
      << format_instance_record_number(greedy_allocation_planning_time_s) << ","
      << csv_escape(format_lns_batch_times(lns_batch_planning_times_s)) << ","
      << format_instance_record_number(sum_lns_batch_times(lns_batch_planning_times_s)) << ","
      << csv_escape(format_lns_batch_makespans(lns_batch_best_makespans)) << ","
      << csv_escape(format_lns_batch_counts(lns_batch_failed_iterations)) << ","
      << path_max_search_iterations_default << ","
      << path_max_search_iterations_fine << ","
      << safe_parking_max_search_iterations << ","
      << lns_threads << ","
      << robot_count << ","
      << csv_escape(lns_mode) << ","
      << csv_escape(order_constraint_learning) << ","
      << csv_escape(best_overall_label) << ","
      << format_instance_record_number(best_overall_makespan) << "\n";

  std::cout << "[Log] Appended instance record CSV: " << csv_path << std::endl;
}
