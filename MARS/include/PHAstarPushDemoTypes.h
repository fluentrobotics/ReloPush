#pragma once

#include <PHAstar/Params.h>
#include <Task.h>
#include <PHAstar/TimeTable.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
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
  Pose initial_transit_start_pose;
  Pose initial_transit_target_pose;
  double initial_transit_requested_start_time = -1.0;
  double initial_transit_start_time = -1.0;
  double initial_transit_end_time = -1.0;
  bool initial_transit_delay_scheduled = false;
  bool has_waiting_pose_conflict = false;
  Pose waiting_pose;
  double waiting_conflict_time = -1.0;
  std::string waiting_conflict_entity;
  std::string waiting_conflict_stage;
  bool has_initial_wait_conflict = false;
  Pose initial_wait_pose;
  double initial_wait_conflict_time = -1.0;
  std::string initial_wait_conflict_entity;
};

// ==========================================
// Stage 1 planner timing instrumentation (opt-in, zero behavior change)
// ==========================================
//
// Which planner tier a PHAStar::Planning_with_res call belongs to, for
// PlanTimingStats bucketing. "Other" covers safe-parking searches, the ghost
// / geometry-fallback initial-transit searches, and anything else not part
// of the primary/fine/contact tier cascade.
enum class PlanSearchTier
{
  Primary,
  Fine,
  Contact,
  Other
};

// Aggregate, opt-in, per-plan timing/counter instrumentation. A single
// instance accumulates over one full plan evaluation (one
// execute_allocation_scenario() call, i.e. every task in that plan), mirrors
// TaskExecutionStats's null-safe accumulator-pointer pattern: every function
// that can contribute takes a nullable `PlanTimingStats *plan_stats =
// nullptr` and skips all bookkeeping when it is null, so passing nullptr
// (the default everywhere except evaluate_scenario_batch()) is exactly
// today's behavior and cost. Plain data, cheap to copy/accumulate -- see
// MARS/planner_opt Stage 1 notes.
//
// NOTE on overlap: these counters are independent diagnostic views, not a
// strict non-overlapping partition of true_wall_s. In particular
// safe_parking_wall_s intentionally overlaps with search_wall_s_other (a
// safe-parking-internal PHAStar search or connected-candidate frontier sweep
// adds to both), and sched_wall_s/traj_scan_wall_s/terminal_hold_wall_s
// overlap with whichever search_wall_s_* bucket ran inside the same
// find_safe_start_time call. Treat each field as answering its own question
// ("how much time did X take"), not as a slice of a pie that sums to 100%.
struct PlanTimingStats
{
  // True wall-clock time of the single execute_allocation_scenario() call
  // this plan was evaluated in (see evaluate_scenario_batch); unlike the
  // oracle CSV's batch-mean eval_wall_s, this is a real per-plan measurement.
  double true_wall_s = 0.0;

  // Seconds (steady_clock) from the start of the evaluate_scenario_batch()
  // call this plan was submitted in, to the moment this plan's
  // execute_allocation_scenario() call started/ended (submission order, not
  // wall order -- see evaluate_scenario_batch's `batch_start` capture in
  // AllocationSearch.cpp). Lets a caller derive the true wall time of any
  // K-prefix of submitted plans as max(end_offset_s) over the first K rows,
  // for both the sequential (worker_count<=1) and threaded paths. Not
  // included in add()/operator+= below: these are batch-relative
  // timestamps, not durations, so summing two instances would be
  // meaningless -- read them from the individual per-plan PlanTimingStats.
  double start_offset_s = 0.0;
  double end_offset_s = 0.0;

  // Wall-clock time spent inside PHAStar::Planning_with_res, by tier.
  double search_wall_s_primary = 0.0;
  double search_wall_s_fine = 0.0;
  double search_wall_s_contact = 0.0;
  // Safe-parking searches + connected-candidate frontier sweeps + ghost /
  // geometry-fallback initial-transit searches + anything untagged.
  double search_wall_s_other = 0.0;

  int n_searches_primary = 0;
  int n_searches_fine = 0;
  int n_searches_contact = 0;
  int n_searches_other = 0;

  // Sum of PlanningResult::debug_stats.iterations over every search counted
  // above.
  std::size_t search_iterations_total = 0;
  // Number of searches whose debug_stats.search_iteration_limit_hit was set
  // (i.e. that hit their configured iteration cap).
  int n_search_cap_hits = 0;

  // Sums of the matching PlanningDebugStats phase-time fields over every
  // search counted above.
  double heuristic_time_s = 0.0;
  double primitive_collision_time_s = 0.0;
  double analytic_validation_time_s = 0.0;
  double holonomic_heuristic_time_s = 0.0;

  // Scheduling (find_safe_start_time and its inner collision checks).
  double sched_wall_s = 0.0;
  int n_find_safe_start_calls = 0;
  int n_start_candidates_tried = 0;
  double traj_scan_wall_s = 0.0;
  double terminal_hold_wall_s = 0.0;

  // Safe parking / relocation (relocate_blocking_robot).
  double safe_parking_wall_s = 0.0;
  int n_parking_relocations = 0;

  int n_robot_candidate_attempts = 0;
  int n_post_validation_retries = 0;
  int n_obsrelo_segments = 0;

  // ---- Stage 3 tier-triage (RuntimeOptions::enable_tier_triage) ----
  // Rule 1 (BLOCKED-BY-ROBOT SHORT-CIRCUIT): a tier's result carried a
  // robot-blocked backup path, so the cascade stopped escalating and used it
  // instead of trying later (finer) tiers.
  int n_triage_blocked_shortcuts = 0;
  // Rule 2 (CAP-HIT-WITH-PROGRESS RETRY): the primary tier hit its iteration
  // cap with real progress toward the goal, so it was retried once with a
  // doubled cap.
  int n_triage_primary_retries = 0;
  // Of the n_triage_primary_retries retries above, how many produced a
  // non-empty (usable) result.
  int n_triage_retry_successes = 0;
  // Rule 3 (NEAR-GOAL-COLLISION SKIP-TO-CONTACT): a tier's failure looked
  // contact-geometry-shaped (near goal, dominated by collision rejections),
  // so the cascade skipped the Fine tier and went straight to ContactBoundary.
  int n_triage_skips_to_contact = 0;

  // ---- Stage 3 holonomic feasibility pre-gate (RuntimeOptions::
  // enable_tier_gate) ----
  // Number of gate reachability checks actually run (once per Fine or
  // Contact tier attempt while the gate is enabled).
  int n_gate_checks = 0;
  // Of those, how many proved the goal unreachable at the Fine tier's
  // holonomic resolution, skipping the Fine search.
  int n_gate_skips_fine = 0;
  // Same, for the Contact tier.
  int n_gate_skips_contact = 0;
  // Total wall-clock time spent inside the gate's reachability checks
  // (holonomic_gate_unreachable), across every n_gate_checks call.
  double gate_wall_s = 0.0;

  // Records one PHAStar::Planning_with_res call's outcome: wall time into
  // the tier bucket + debug-stat phase-time sums + iteration/cap-hit counts.
  // Callers extract the raw fields from PlanningResult::debug_stats
  // themselves (rather than this header taking a PlanningResult dependency)
  // to keep this header's include footprint small.
  void record_search(PlanSearchTier tier, double wall_seconds,
                      std::size_t iterations, std::size_t iteration_limit_hit,
                      double heuristic_time_sec,
                      double primitive_collision_time_sec,
                      double analytic_validation_time_sec,
                      double holonomic_heuristic_time_sec)
  {
    switch (tier)
    {
    case PlanSearchTier::Primary:
      search_wall_s_primary += wall_seconds;
      ++n_searches_primary;
      break;
    case PlanSearchTier::Fine:
      search_wall_s_fine += wall_seconds;
      ++n_searches_fine;
      break;
    case PlanSearchTier::Contact:
      search_wall_s_contact += wall_seconds;
      ++n_searches_contact;
      break;
    case PlanSearchTier::Other:
    default:
      search_wall_s_other += wall_seconds;
      ++n_searches_other;
      break;
    }
    search_iterations_total += iterations;
    if (iteration_limit_hit > 0)
      ++n_search_cap_hits;
    heuristic_time_s += heuristic_time_sec;
    primitive_collision_time_s += primitive_collision_time_sec;
    analytic_validation_time_s += analytic_validation_time_sec;
    holonomic_heuristic_time_s += holonomic_heuristic_time_sec;
  }

  // Adds every field of `other` into this instance. Used both by unit tests
  // and available for callers that need to merge two partial accumulations.
  void add(const PlanTimingStats &other)
  {
    true_wall_s += other.true_wall_s;

    search_wall_s_primary += other.search_wall_s_primary;
    search_wall_s_fine += other.search_wall_s_fine;
    search_wall_s_contact += other.search_wall_s_contact;
    search_wall_s_other += other.search_wall_s_other;

    n_searches_primary += other.n_searches_primary;
    n_searches_fine += other.n_searches_fine;
    n_searches_contact += other.n_searches_contact;
    n_searches_other += other.n_searches_other;

    search_iterations_total += other.search_iterations_total;
    n_search_cap_hits += other.n_search_cap_hits;

    heuristic_time_s += other.heuristic_time_s;
    primitive_collision_time_s += other.primitive_collision_time_s;
    analytic_validation_time_s += other.analytic_validation_time_s;
    holonomic_heuristic_time_s += other.holonomic_heuristic_time_s;

    sched_wall_s += other.sched_wall_s;
    n_find_safe_start_calls += other.n_find_safe_start_calls;
    n_start_candidates_tried += other.n_start_candidates_tried;
    traj_scan_wall_s += other.traj_scan_wall_s;
    terminal_hold_wall_s += other.terminal_hold_wall_s;

    safe_parking_wall_s += other.safe_parking_wall_s;
    n_parking_relocations += other.n_parking_relocations;

    n_robot_candidate_attempts += other.n_robot_candidate_attempts;
    n_post_validation_retries += other.n_post_validation_retries;
    n_obsrelo_segments += other.n_obsrelo_segments;

    n_triage_blocked_shortcuts += other.n_triage_blocked_shortcuts;
    n_triage_primary_retries += other.n_triage_primary_retries;
    n_triage_retry_successes += other.n_triage_retry_successes;
    n_triage_skips_to_contact += other.n_triage_skips_to_contact;

    n_gate_checks += other.n_gate_checks;
    n_gate_skips_fine += other.n_gate_skips_fine;
    n_gate_skips_contact += other.n_gate_skips_contact;
    gate_wall_s += other.gate_wall_s;
  }

  PlanTimingStats &operator+=(const PlanTimingStats &other)
  {
    add(other);
    return *this;
  }
};

inline PlanTimingStats operator+(PlanTimingStats lhs, const PlanTimingStats &rhs)
{
  lhs += rhs;
  return lhs;
}

// RAII wall-clock accumulator: adds elapsed seconds to one or two
// PlanTimingStats double fields when destroyed, covering every exit path of
// the enclosing scope (return, break out of a loop, etc.) without needing to
// instrument each one individually. Both `stats` and the field pointers may
// be null; a null `stats` makes construction/destruction a cheap no-op,
// mirroring the null-safety of the raw pointer-based accumulation used
// everywhere else in this instrumentation. Modeled after this header's
// existing ScopedStreamSilencer RAII helper.
class ScopedWallTimer
{
public:
  explicit ScopedWallTimer(PlanTimingStats *stats,
                            double PlanTimingStats::*field_a,
                            double PlanTimingStats::*field_b = nullptr)
      : stats_(stats), field_a_(field_a), field_b_(field_b),
        start_(std::chrono::steady_clock::now())
  {
  }

  ~ScopedWallTimer()
  {
    if (!stats_)
      return;
    const double elapsed = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - start_)
                                .count();
    if (field_a_)
      stats_->*field_a_ += elapsed;
    if (field_b_)
      stats_->*field_b_ += elapsed;
  }

  ScopedWallTimer(const ScopedWallTimer &) = delete;
  ScopedWallTimer &operator=(const ScopedWallTimer &) = delete;

private:
  PlanTimingStats *stats_;
  double PlanTimingStats::*field_a_;
  double PlanTimingStats::*field_b_;
  std::chrono::steady_clock::time_point start_;
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
  // Base64 serialization of the full ExecutedScenario (params + entities +
  // timetable) this plan produced -- see MARS/include/
  // ExecutedScenarioSerialization.h. Populated by evaluate_scenario_batch()
  // only when the caller requested save-and-replay output (either
  // RuntimeOptions::eval_plans_result_out_path or
  // eval_plans_result_out_dir is non-empty); empty string otherwise (no
  // perf regression when the feature is unused). See
  // MARS/16save-replay-implementation.md.
  std::string serialized_result;
  // Opt-in per-plan timing/counter instrumentation (Stage 1 planner
  // optimization work; see PlanTimingStats). Always populated (cheaply) by
  // evaluate_scenario_batch() for every plan; only written out when
  // RuntimeOptions::eval_plans_timing_out_path is set (see EvalPlansCli.cpp).
  PlanTimingStats timing;
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

struct LnsEvaluationResult
{
  AllocationRunSummary summary;
  std::unique_ptr<ExecutedScenario> executed;
  // Wall-clock time (steady_clock) of this single candidate's
  // repair_destroyed_tasks_with_sampled_insertion() call, measured inside
  // evaluate_lns_batch's worker lambda (both the worker_count<=1 sequential
  // path and the threaded path). Carried into
  // SequenceSearchOutcome::lns_candidate_planning_times_s below.
  double wall_seconds = 0.0;
};

struct SequenceSearchOutcome
{
  AllocationRunSummary best_feasible;
  AllocationRunSummary best_partial;
  std::unique_ptr<ExecutedScenario> best_feasible_execution;
  std::vector<double> lns_batch_planning_times_s;
  std::vector<double> lns_batch_best_makespans;
  std::vector<int> lns_batch_failed_iterations;
  // Per-candidate LNS records, one entry per LNS iteration, appended in
  // iteration order across every batch (run_adaptive_lns_search's
  // sequential post-batch loop in SearchOrchestrator.cpp). Empty for
  // greedy-only runs (this outcome is never populated) and for DQN search
  // mode (run_dqn_search returns a default-constructed SequenceSearchOutcome
  // for these three fields; only the LNS path populates them).
  std::vector<double> lns_candidate_planning_times_s;
  // 1 = candidate.all_tasks_succeeded, 0 = infeasible.
  std::vector<int> lns_candidate_feasible;
  // Candidate makespan when feasible; -1.0 (not NaN/inf) as the sentinel for
  // an infeasible candidate, matching the CSV convention documented at
  // write_instance_run_record_csv's header comment.
  std::vector<double> lns_candidate_makespans;
  int lns_failed_iterations = 0;
  bool has_feasible = false;
  bool has_partial = false;
};

struct ParkingCandidate
{
  Pose pose;
  double estimated_rs_length;
  std::vector<Waypoint> connected_waypoints;
};

struct SegmentReplanContext
{
  int task_id = -1;
  int segment_id = -1;
  std::string object_name;
  std::string start_contact_entity;
  bool tight_or_contact_case = false;
  bool has_live_object_pose = false;
  Pose live_object_pose;
  double mars_prepush_distance = 0.0;
  double source_prepush_distance = 0.0;
  Pose source_clearance_goal;
};

enum class IdleBlockerRelocationPolicy
{
  RelocateAnyIdle,
  // Never relocate an idle blocker; only start-time delay may resolve the
  // conflict. Valid policy value (should_relocate_idle_blocker simply stays
  // false), but currently unused by any call site.
  WaitOnly,
  RelocateIfBecameIdleDuringAttempt,
};
