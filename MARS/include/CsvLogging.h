#ifndef CSV_LOGGING_H
#define CSV_LOGGING_H

// Forward declarations only — avoid pulling in the heavyweight include chain
// (PHAstarPushDemoTypes.h -> Task.h -> PHAstar.h -> Visualization.h)
// which has non-inline function definitions that cause ODR violations in
// multi-TU builds.
#include <PHAstar/Point.h>  // for Pose (used by value in OrientationJumpRecord)
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations of types used in the API
struct Params;
struct EntityMeta;
class TimeTable;

// From PHAstarPushDemoTypes.h — we forward-declare what we need
struct TaskCsvRow;
struct AllocationRunSummary;
struct SearchTrialRecord;
enum class PlanningStatus;

namespace ReloPush {
  struct HandoffInstanceInfo;
}

// ==========================================
// CSV Logging & Diagnostics
// ==========================================

std::string csv_escape(const std::string &field);
const char *planning_status_name(PlanningStatus status);
std::string relopush_output_dir();
std::string relopush_sequence_path(const std::string &instance_file_name,
                                   int instance_index);
std::string index_log_dir();
std::string index_log_path(const std::string &filename);
std::string diagnostics_dir();
std::string diagnostics_path(const std::string &filename);

void write_task_csv_log(const std::string &csv_path,
                        const std::vector<TaskCsvRow> &rows);

struct OrientationJumpRecord
{
  std::string scenario_label;
  std::string source;
  std::string robot_name;
  double prev_time = 0.0;
  double curr_time = 0.0;
  Pose prev_pose;
  Pose curr_pose;
  double raw_yaw_delta = 0.0;
  double wrapped_yaw_delta = 0.0;
  double distance = 0.0;
};

void write_timetable_robot_pose_csv(
    const std::string &csv_path,
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    double sample_step);

void write_orientation_jump_csv(
    const std::string &csv_path,
    const std::vector<OrientationJumpRecord> &records);

std::vector<OrientationJumpRecord> collect_robot_orientation_jumps(
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    double dense_sample_step,
    double wrapped_jump_threshold);

void log_timetable_orientation_diagnostics(
    const std::string &scenario_label,
    const TimeTable &timetable,
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const Params &params);

std::string sanitize_filename_component(const std::string &label);

std::string default_result_summary_path(
    const ReloPush::HandoffInstanceInfo &instance_info,
    const std::string &scenario_label);

void write_allocation_search_summary_csv(
    const std::string &csv_path,
    const std::vector<AllocationRunSummary> &summaries,
    double greedy_makespan);

void write_search_trial_records_csv(
    const std::string &csv_path,
    const std::vector<SearchTrialRecord> &records);

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
    // Per-candidate LNS records (see SequenceSearchOutcome's doc comments in
    // PHAstarPushDemoTypes.h), appended as the CSV's three trailing columns
    // so existing position/DictReader-based consumers of the earlier columns
    // are unaffected. Semicolon-separated lists, one entry per LNS
    // iteration, empty for greedy-only (and DQN-mode) runs.
    // lns_candidate_makespans uses -1 (not "INF") as its infeasible
    // sentinel -- see format_lns_candidate_makespans() in CsvLogging.cpp.
    const std::vector<double> &lns_candidate_planning_times_s,
    const std::vector<int> &lns_candidate_feasible,
    const std::vector<double> &lns_candidate_makespans,
    int path_max_search_iterations_default,
    int path_max_search_iterations_fine,
    int safe_parking_max_search_iterations,
    int lns_threads,
    int robot_count,
    const std::string &lns_mode,
    const std::string &lns_fine_segment_retry,
    const std::string &order_constraint_learning,
    const std::string &best_overall_label,
    double best_overall_makespan);

#endif // CSV_LOGGING_H
