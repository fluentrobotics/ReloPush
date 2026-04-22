#pragma once

#include <PHAstar/Params.h>
#include <Task.h>
#include <PHAstar/TimeTable.h>

#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <streambuf>
#include <string>
#include <unordered_map>
#include <vector>

struct TaskExecutionStats
{
  double total_waiting = 0.0;
  int delayed_segments = 0;
  bool has_initial_transit = false;
  double initial_transit_start_time = -1.0;
  double initial_transit_end_time = -1.0;
  bool has_initial_wait_conflict = false;
  Pose initial_wait_pose;
  double initial_wait_conflict_time = -1.0;
  std::string initial_wait_conflict_entity;
};

struct TimeTableVerificationResult
{
  bool is_valid = true;
  double time = 0.0;
  std::string reason;
  std::string entity_a;
  std::string entity_b;
};

// Time interval where a specific robot-object contact is intentionally valid.
// During [start_time, end_time], collision checks may permit this pair overlap
// as an active transfer/pushing contact.
struct TransferContactWindow
{
  // Robot participating in the transfer contact.
  EntityMeta *robot = nullptr;
  // Object being transferred by the robot.
  EntityMeta *object = nullptr;
  // Absolute start timestamp (seconds) of valid contact.
  double start_time = 0.0;
  // Absolute end timestamp (seconds) of valid contact.
  double end_time = 0.0;
};

// One CSV log record per task summarizing assignment and execution outcome.
// Time values are absolute seconds in the global timetable.
struct TaskCsvRow
{
  // 1-based task index in processing order.
  int task_id = 0;
  // Name of the target object for this task.
  std::string object_name;
  // "SUCCESS" or "FAILED".
  std::string status;
  // Robot name that succeeded or last attempted robot on failure.
  std::string robot_name;
  // Candidate robot free time used as task start reference.
  double start_time = -1.0;
  // Robot finish timestamp for successful execution.
  double end_time = -1.0;
  // Accumulated waiting time inserted by scheduler conflict resolution.
  double total_waiting = 0.0;
  // Number of robot-candidate attempts for this task.
  int attempts = 0;
  // Failure diagnostics; empty for successful rows.
  std::string failure_reason;
};

struct AllocationScenarioPlan
{
  std::vector<std::size_t> task_order;
  std::vector<std::string> preferred_robot_names_by_original_task;
};

struct AllocationRunSummary
{
  std::string label;
  AllocationScenarioPlan plan;
  std::uint32_t parking_seed = 0;
  bool all_tasks_succeeded = false;
  int successful_tasks = 0;
  int failed_tasks = 0;
  double makespan = std::numeric_limits<double>::infinity();
  std::vector<TaskCsvRow> task_rows;
};

struct SearchTrialRecord
{
  std::string search_label;
  int iteration = 0;
  std::uint32_t parking_seed = 0;
  bool feasible = false;
  double makespan = std::numeric_limits<double>::infinity();
  int successful_tasks = 0;
  int failed_tasks = 0;
  std::string order_description;
};

struct ScenarioEvaluationRequest
{
  AllocationScenarioPlan plan;
  std::string label;
  std::uint32_t parking_seed = 0;
};

struct ScenarioEvaluationResult
{
  AllocationRunSummary summary;
};

struct AssignmentSearchCandidate
{
  int iteration = 0;
  AllocationScenarioPlan plan;
  std::uint32_t parking_seed = 0;
  int preference_changes = 0;
  std::string preview_changes;
};

struct SequenceSearchCandidate
{
  int iteration = 0;
  AllocationScenarioPlan plan;
  std::uint32_t parking_seed = 0;
  int position_changes = 0;
  std::string preview_order;
};

struct LnsSearchCandidate
{
  int iteration = 0;
  std::string destroy_operator;
  std::vector<std::size_t> destroyed_tasks;
  std::vector<double> destroy_scores;
  std::uint32_t parking_seed = 0;
  std::uint32_t repair_seed = 0;
  bool diversification = false;
  double destroy_fraction = 0.10;
};

struct SequenceSearchOutcome
{
  AllocationRunSummary best_feasible;
  AllocationRunSummary best_partial;
  bool has_feasible = false;
  bool has_partial = false;
};

struct LearnedOrderConstraints
{
  std::vector<std::vector<int>> evidence_counts;
  std::vector<std::vector<bool>> enforced;
  int enforced_count = 0;
};

struct NullBuffer : public std::streambuf
{
  int overflow(int c) override
  {
    return c;
  }
};

class ScopedStreamSilencer
{
public:
  explicit ScopedStreamSilencer(bool enabled)
      : enabled_(enabled)
  {
    if (!enabled_)
      return;

    cout_buf_ = std::cout.rdbuf(&null_buf_);
    cerr_buf_ = std::cerr.rdbuf(&null_buf_);
  }

  ~ScopedStreamSilencer()
  {
    if (!enabled_)
      return;

    std::cout.rdbuf(cout_buf_);
    std::cerr.rdbuf(cerr_buf_);
  }

  ScopedStreamSilencer(const ScopedStreamSilencer &) = delete;
  ScopedStreamSilencer &operator=(const ScopedStreamSilencer &) = delete;

private:
  bool enabled_ = false;
  NullBuffer null_buf_;
  std::streambuf *cout_buf_ = nullptr;
  std::streambuf *cerr_buf_ = nullptr;
};

class ScopedGlobalStreamSilencer
{
public:
  explicit ScopedGlobalStreamSilencer(bool enabled)
      : enabled_(enabled), lock_(stream_mutex_, std::defer_lock)
  {
    if (!enabled_)
      return;

    lock_.lock();
    cout_buf_ = std::cout.rdbuf(&null_buf_);
    cerr_buf_ = std::cerr.rdbuf(&null_buf_);
  }

  ~ScopedGlobalStreamSilencer()
  {
    if (!enabled_)
      return;

    std::cout.rdbuf(cout_buf_);
    std::cerr.rdbuf(cerr_buf_);
  }

private:
  bool enabled_ = false;
  NullBuffer null_buf_;
  std::streambuf *cout_buf_ = nullptr;
  std::streambuf *cerr_buf_ = nullptr;
  std::unique_lock<std::mutex> lock_;
  inline static std::mutex stream_mutex_;
};

inline void cleanup_entities(std::unordered_map<std::string, EntityMeta *> &entities)
{
  for (auto &pair : entities)
    delete pair.second;
  entities.clear();
}

struct ExecutedScenario
{
  AllocationRunSummary summary;
  Params params;
  TimeTable timetable{0.5};
  std::unordered_map<std::string, EntityMeta *> entities;

  ExecutedScenario() = default;
  ~ExecutedScenario()
  {
    cleanup_entities(entities);
  }

  ExecutedScenario(const ExecutedScenario &) = delete;
  ExecutedScenario &operator=(const ExecutedScenario &) = delete;

  ExecutedScenario(ExecutedScenario &&other) noexcept
      : summary(std::move(other.summary)),
        params(other.params),
        timetable(std::move(other.timetable)),
        entities(std::move(other.entities))
  {
    other.entities.clear();
  }

  ExecutedScenario &operator=(ExecutedScenario &&other) noexcept
  {
    if (this == &other)
      return *this;

    cleanup_entities(entities);
    summary = std::move(other.summary);
    params = other.params;
    timetable = std::move(other.timetable);
    entities = std::move(other.entities);
    other.entities.clear();
    return *this;
  }
};

struct ParkingCandidate
{
  Pose pose;
  double estimated_rs_length;
  std::vector<Waypoint> connected_waypoints;
};
