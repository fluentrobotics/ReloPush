/*****************************************************************
 * Allocation Search — LNS, Greedy, Sequence/Assignment Search
 * Extracted from PHAstar_push_demo.cpp
 ******************************************************************/

#include <AllocationSearch.h>
#include <TaskExecution.h>
#include <SafeParking.h>
#include <ExecutedScenarioSerialization.h>

#include <PHAstar/Params.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Functions that remain in PHAstar_push_demo.cpp for now
std::string default_sequence_path();
std::uint32_t mix_seed(std::uint32_t seed, std::uint32_t salt);
extern bool DEBUG_VIS;

namespace
{
class ScopedDebugVisualizationOverride
{
public:
  explicit ScopedDebugVisualizationOverride(bool enabled)
      : enabled_(enabled), previous_(DEBUG_VIS)
  {
    if (enabled_)
      DEBUG_VIS = false;
  }

  ~ScopedDebugVisualizationOverride()
  {
    if (enabled_)
      DEBUG_VIS = previous_;
  }

  ScopedDebugVisualizationOverride(const ScopedDebugVisualizationOverride &) = delete;
  ScopedDebugVisualizationOverride &
  operator=(const ScopedDebugVisualizationOverride &) = delete;

private:
  bool enabled_ = false;
  bool previous_ = false;
};

RuntimeOptions make_parallel_lns_worker_options(const RuntimeOptions &options)
{
  RuntimeOptions worker_options = options;
  worker_options.planner_expansion_threads = 1;
  worker_options.print_planning_status = false;

  // LNS candidates run in worker threads when lns_threads > 1. Qt windows
  // must stay on the main thread, so disable every Qt-facing option there.
  if (options.lns_threads > 1)
  {
    worker_options.debug_vis = false;
    worker_options.enable_visualization = false;
    worker_options.visualize_relopush_plan = false;
    worker_options.enable_result_summary_figure = false;
  }

  return worker_options;
}

std::mutex &lns_worker_status_mutex()
{
  static std::mutex mutex;
  return mutex;
}

void print_lns_worker_status(std::size_t worker_idx,
                             std::size_t worker_count,
                             const LnsSearchCandidate &candidate,
                             const std::string &state,
                             const AllocationRunSummary *summary = nullptr)
{
  std::lock_guard<std::mutex> lock(lns_worker_status_mutex());
  std::clog << "[LNS worker " << (worker_idx + 1) << "/" << worker_count
            << "] " << state
            << " iter=" << candidate.iteration
            << " op=" << candidate.destroy_operator
            << " k=" << candidate.destroyed_tasks.size();

  if (summary)
  {
    std::clog << " feas=" << (summary->all_tasks_succeeded ? 1 : 0);
    if (summary->all_tasks_succeeded)
    {
      std::clog << " mk=" << std::fixed << std::setprecision(2)
                << summary->makespan << "s";
    }
    else
    {
      std::clog << " suc=" << summary->successful_tasks << "/"
                << (summary->successful_tasks + summary->failed_tasks);
    }
  }

  std::clog << std::endl;
}
} // namespace

// ==========================================
// Config Display
// ==========================================

void print_runtime_options(const RuntimeOptions &options)
{
  auto print_method_list =
      [](const std::string &label,
         const std::vector<TransitPlannerStep> &methods)
  {
    std::cout << "[Config] " << label << ":";
    if (methods.empty())
    {
      std::cout << " none" << std::endl;
      return;
    }
    std::cout << std::endl;
    for (std::size_t idx = 0; idx < methods.size(); ++idx)
    {
      const auto &step = methods[idx];
      std::cout << "  " << (idx + 1) << ") "
                << transit_planner_method_name(step.method)
                << " [" << reference_egraph_use_name(step.reference_egraph)
                << ", "
                << transit_planner_applicability_name(step.applicability)
                << "]" << std::endl;
    }
  };

  std::cout << "[Config] Debug vis logs: "
            << (options.debug_vis ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Planning status terminal output: "
            << (options.print_planning_status ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Robot boundary mode: "
            << (options.robot_boundary_origin_only ? "origin-only" : "corner-strict")
            << std::endl;
  std::cout << "[Config] Base RNG seed: " << options.base_random_seed
            << (options.has_fixed_random_seed ? " (fixed)" : " (randomized)")
            << std::endl;
  std::cout << "[Config] LNS threads: "
            << options.lns_threads << std::endl;
  std::cout << "[Config] Requested MARS robots: "
            << options.robot_count << std::endl;
  std::cout << "[Config] Planner expansion threads: "
            << options.planner_expansion_threads << std::endl;
  if (options.lns_threads > 1 && options.planner_expansion_threads > 1)
  {
    std::cout << "[Config] Planner expansion threads are disabled inside "
                 "parallel LNS/search batches to avoid nested oversubscription."
              << std::endl;
  }
  std::cout << "[Config] Max search iterations: "
            << options.max_search_iterations << std::endl;
  std::cout << "[Config] Default search params: "
            << "xy=" << options.default_xy_resolution
            << ", yaw=" << options.default_yaw_resolution
            << ", time_step=" << options.default_time_step
            << ", rs_step=" << options.default_rs_step_size
            << ", collision_steps=" << options.default_collision_steps
            << ", collision_step="
            << options.default_collision_check_time_step
            << ", analytic_threshold_scale="
            << options.default_analytic_threshold_scale << std::endl;
  std::cout << "[Config] Default search costs/collision: "
            << "turn_penalty=" << options.default_turn_penalty
            << ", reverse_penalty=" << options.default_reverse_penalty
            << ", switch_penalty=" << options.default_switch_penalty
            << ", wait_penalty=" << options.default_wait_penalty
            << ", inflation=" << options.default_inflation
            << ", safety_margin=" << options.default_safety_margin
            << ", robot_collision_inflation="
            << options.default_robot_collision_inflation << std::endl;
  std::cout << "[Config] Retraction distance: "
            << options.retraction_distance << "m" << std::endl;
  std::cout << "[Config] Fine segment retry params: "
            << "max_iter=" << options.fine_segment_max_search_iterations
            << ", xy=" << options.fine_segment_xy_resolution
            << ", yaw=" << options.fine_segment_yaw_resolution
            << ", time_step=" << options.fine_segment_time_step
            << ", rs_step=" << options.fine_segment_rs_step_size
            << ", collision_step="
            << options.fine_segment_collision_check_time_step << std::endl;
  std::cout << "[Config] Contact-boundary params: "
            << "max_iter=" << options.contact_boundary_max_search_iterations
            << ", xy=" << options.contact_boundary_xy_resolution
            << ", yaw=" << options.contact_boundary_yaw_resolution
            << ", time_step=" << options.contact_boundary_time_step
            << ", rs_step=" << options.contact_boundary_rs_step_size
            << ", reverse_penalty="
            << options.contact_boundary_reverse_penalty
            << ", holonomic_resolution="
            << options.contact_boundary_holonomic_resolution
            << std::endl;
  std::cout << "[Config] Safe parking max search iterations: "
            << (options.safe_parking_max_search_iterations > 0
                    ? options.safe_parking_max_search_iterations
                    : options.max_search_iterations)
            << (options.safe_parking_max_search_iterations > 0
                    ? ""
                    : " (inherits max search iterations)")
            << std::endl;
  std::cout << "[Config] Parking candidate mode: "
            << parking_candidate_mode_name(options.parking_candidate_mode)
            << std::endl;
  std::cout << "[Config] Failed-candidate idle parking: "
            << (options.enable_failed_candidate_idle_parking ? "enabled" : "disabled")
            << ", initial_transit_threshold="
            << options.failed_candidate_initial_transit_failure_threshold
            << std::endl;
  std::cout << "[Config] Order-constraint learning: "
            << (options.enable_order_constraint_learning ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Assignment search iterations: "
            << options.assignment_search_iterations << std::endl;
  std::cout << "[Config] Local sequence search iterations: "
            << options.local_sequence_search_iterations << std::endl;
  std::cout << "[Config] Shuffle sequence search iterations: "
            << options.shuffle_sequence_search_iterations << std::endl;
  std::cout << "[Config] LNS iterations: "
            << options.lns_iterations << std::endl;
  std::cout << "[Config] LNS fine segment retry: "
            << (options.enable_lns_fine_segment_retry ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] LNS mode: "
            << (options.lns_reassign_only ? "lns-reassign-only"
                                          : "lns-task-reassign")
            << std::endl;
  std::cout << "[Config] Reference E-Graph params: "
            << "epsilon=" << options.reference_egraph_epsilon
            << ", spacing=" << options.reference_egraph_waypoint_spacing
            << ", snap_radius=" << options.reference_egraph_snap_radius
            << ", snap_yaw=" << options.reference_egraph_snap_yaw
            << ", lookahead=" << options.reference_egraph_successor_lookahead
            << ", max_nodes=" << options.reference_egraph_max_nodes
            << std::endl;
  print_method_list("Initial transit methods", options.initial_transit_methods);
  print_method_list("Segment transit methods", options.segment_transit_methods);
  std::cout << "[Config] Visualization: "
            << (options.enable_visualization ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] ReloPush plan replay visualization: "
            << (options.visualize_relopush_plan ? "enabled" : "disabled")
            << std::endl;
  std::cout << "[Config] Result summary figure: "
            << (options.enable_result_summary_figure ? "enabled" : "disabled");
  if (options.enable_result_summary_figure)
  {
    std::cout << ", gap=" << options.result_summary_subplot_gap << "px";
    if (!options.result_summary_output_path.empty())
      std::cout << ", output=" << options.result_summary_output_path;
  }
  std::cout << std::endl;
  std::cout << "[Config] Input mode: "
            << (options.integrated_mode ? "integrated-handoff" : "sequence-file")
            << std::endl;
  if (options.integrated_mode)
  {
    std::cout << "[Config] Handoff endpoint: "
              << options.handoff_endpoint << std::endl;
  }
  else
  {
    std::cout << "[Config] Sequence file: "
              << (options.input_sequence_path.empty()
                      ? default_sequence_path()
                      : options.input_sequence_path)
              << std::endl;
  }
}

// ==========================================
// Environment & Task Initialization
// ==========================================

std::vector<Task> initialize_tasks(
    const std::vector<FinalAllocation> &loaded_sequence,
    std::unordered_map<std::string, EntityMeta *> &entities)
{
  std::vector<Task> tasks;
  tasks.reserve(loaded_sequence.size());
  for (const auto &fa : loaded_sequence)
  {
    tasks.emplace_back(fa, entities);
  }

  std::cout << "[System] Initialized " << tasks.size() << " tasks."
            << std::endl;
  return tasks;
}

AllocationScenarioPlan make_identity_plan(std::size_t task_count)
{
  AllocationScenarioPlan plan;
  plan.task_order.resize(task_count);
  std::iota(plan.task_order.begin(), plan.task_order.end(), 0);
  return plan;
}

// Forward declarations for thread-local state (defined in TaskExecution.cpp)
extern thread_local std::unordered_map<std::string, int> g_initial_transit_failure_counts;
extern thread_local std::unordered_map<std::string, double> g_recent_failed_relocations;

// Reset thread-local planning state before environment initialization
static void reset_thread_local_planning_state()
{
  g_recent_failed_relocations.clear();
  g_initial_transit_failure_counts.clear();
}

void initialize_environment(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    std::uint32_t parking_seed,
    Params &params,
    std::unordered_map<std::string, EntityMeta *> &entities,
    TimeTable &timetable,
    std::vector<RobotMeta *> &all_robots,
    bool verbose)
{
  reset_thread_local_planning_state();
  params = initialize_params(loaded_sequence, options);
  params.robot_boundary_origin_only = options.robot_boundary_origin_only;
  g_parking_candidate_mode = options.parking_candidate_mode;
  initialize_parking_rng(true, parking_seed);

  if (verbose)
  {
    std::cout << "[Config] Parking RNG seed: " << parking_rng_seed() << std::endl;
    std::cout << "[Config] Parking candidate mode: "
              << parking_candidate_mode_name() << std::endl;
    std::cout << "[Config] Safe parking max search iterations: "
              << (options.safe_parking_max_search_iterations > 0
                      ? options.safe_parking_max_search_iterations
                      : options.max_search_iterations)
              << (options.safe_parking_max_search_iterations > 0
                      ? ""
                      : " (inherits max search iterations)")
              << std::endl;
    std::ostringstream collision_tuning;
    collision_tuning << std::fixed << std::setprecision(3)
                     << "[Config] Collision tuning: inflation="
                     << params.inflation
                     << ", safety_margin=" << params.safety_margin
                     << ", robot_collision_inflation="
                     << params.robot_collision_inflation
                     << " (temporary experiment)";
    std::cout << collision_tuning.str() << std::endl;
  }

  entities = initialize_entities(loaded_sequence, options.robot_count);
  timetable.add_initial(entities);

  all_robots.clear();
  for (const auto &[name, ent] : entities)
  {
    if (ent->type == EntityType::ROBOT)
      all_robots.push_back(dynamic_cast<RobotMeta *>(ent));
  }
  std::sort(all_robots.begin(), all_robots.end(),
            [](const RobotMeta *a, const RobotMeta *b)
            {
              return a->name < b->name;
            });
  if (verbose)
  {
    std::cout << "[Config] Active MARS robots: "
              << all_robots.size()
              << " (requested " << options.robot_count << ")"
              << std::endl;
  }
  if (verbose && !all_robots.empty())
  {
    const RobotMeta *sample_robot = all_robots.front();
    std::cout << std::fixed << std::setprecision(3)
              << "[Config] Robot turning radii: transit="
              << sample_robot->transit_turning_radius()
              << "m, transfer="
              << sample_robot->transfer_turning_radius()
              << "m" << std::endl;
  }
}

std::unordered_map<std::string, RobotMeta *>
build_robot_lookup(const std::vector<RobotMeta *> &all_robots)
{
  std::unordered_map<std::string, RobotMeta *> lookup;
  for (auto *robot : all_robots)
  {
    if (robot)
      lookup[robot->name] = robot;
  }
  return lookup;
}

std::vector<std::size_t> normalized_task_order(
    const AllocationScenarioPlan &plan,
    std::size_t task_count)
{
  if (plan.task_order.size() != task_count)
  {
    auto identity_plan = make_identity_plan(task_count);
    return identity_plan.task_order;
  }

  std::vector<bool> seen(task_count, false);
  for (std::size_t idx : plan.task_order)
  {
    if (idx >= task_count || seen[idx])
    {
      auto identity_plan = make_identity_plan(task_count);
      return identity_plan.task_order;
    }
    seen[idx] = true;
  }

  return plan.task_order;
}

std::vector<Task> build_tasks_for_plan(
    const std::vector<FinalAllocation> &loaded_sequence,
    std::unordered_map<std::string, EntityMeta *> &entities,
    const std::vector<RobotMeta *> &all_robots,
    const AllocationScenarioPlan &plan)
{
  auto base_tasks = initialize_tasks(loaded_sequence, entities);
  auto order = normalized_task_order(plan, base_tasks.size());
  auto robot_lookup = build_robot_lookup(all_robots);

  std::vector<Task> ordered_tasks;
  ordered_tasks.reserve(base_tasks.size());

  for (std::size_t original_idx : order)
  {
    ordered_tasks.push_back(base_tasks[original_idx]);
    Task &task = ordered_tasks.back();
    task.assignedRobot = nullptr;

    if (original_idx < plan.preferred_robot_names_by_original_task.size())
    {
      const auto &preferred_name =
          plan.preferred_robot_names_by_original_task[original_idx];
      auto it = robot_lookup.find(preferred_name);
      if (!preferred_name.empty() && it != robot_lookup.end())
      {
        task.assignedRobot = it->second;
      }
    }
  }

  return ordered_tasks;
}

// ==========================================
// Scenario Execution & Summary
// ==========================================

AllocationRunSummary summarize_run(
    const std::string &label,
    const AllocationScenarioPlan &plan,
    std::uint32_t parking_seed,
    const std::vector<TaskCsvRow> &task_rows,
    const TimeTable &timetable)
{
  AllocationRunSummary summary;
  summary.label = label;
  summary.plan = plan;
  summary.parking_seed = parking_seed;
  summary.task_rows = task_rows;
  summary.successful_tasks = static_cast<int>(
      std::count_if(task_rows.begin(), task_rows.end(),
                    [](const TaskCsvRow &row)
                    { return row.status == "SUCCESS"; }));
  summary.failed_tasks = static_cast<int>(task_rows.size()) - summary.successful_tasks;
  summary.all_tasks_succeeded = summary.failed_tasks == 0;
  summary.makespan = summary.all_tasks_succeeded
                         ? timetable.get_max_time()
                         : std::numeric_limits<double>::infinity();
  return summary;
}

ExecutedScenario execute_allocation_scenario(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &plan,
    const std::string &label,
    std::uint32_t parking_seed,
    bool verbose,
    bool abort_on_first_failure,
    PlanTimingStats *plan_stats)
{
  ScopedStreamSilencer silencer(!verbose && !options.print_planning_status);

  ExecutedScenario executed;
  std::vector<RobotMeta *> all_robots;
  initialize_environment(loaded_sequence, options, parking_seed,
                         executed.params, executed.entities,
                         executed.timetable, all_robots, verbose);

  auto tasks = build_tasks_for_plan(loaded_sequence, executed.entities,
                                    all_robots, plan);
  auto task_rows = execute_task_allocation_loop(
      tasks, all_robots, executed.timetable, executed.entities, executed.params,
      options, abort_on_first_failure, plan_stats);

  executed.summary = summarize_run(label, plan, parking_seed, task_rows,
                                   executed.timetable);
  return executed;
}

std::vector<ScenarioEvaluationResult> evaluate_scenario_batch(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const std::vector<ScenarioEvaluationRequest> &requests)
{
  std::vector<ScenarioEvaluationResult> results(requests.size());
  if (requests.empty())
    return results;

  // Save-and-replay support (see MARS/16save-replay-implementation.md):
  // serialize each plan's full ExecutedScenario WHILE it is still alive, iff
  // the caller asked for save-and-replay output via either result-out flag.
  // Neither flag set (the common case) skips serialization entirely, so this
  // feature costs nothing when unused.
  const bool save_executed_scenarios =
      !options.eval_plans_result_out_path.empty() ||
      !options.eval_plans_result_out_dir.empty();

  std::size_t worker_count = std::min<std::size_t>(
      requests.size(),
      static_cast<std::size_t>(std::max(1, options.lns_threads)));

  if (worker_count <= 1)
  {
    for (std::size_t i = 0; i < requests.size(); ++i)
    {
      // Per-plan timing instrumentation (Stage 1 planner optimization work;
      // see PlanTimingStats). `timing` is a plan-local stack object -- no
      // sharing, no locks needed -- always populated (cheap: chrono +
      // counters only) regardless of whether --eval-plans-timing-out= was
      // requested; only the CSV write in EvalPlansCli.cpp is conditional.
      PlanTimingStats timing;
      const auto plan_t0 = std::chrono::steady_clock::now();
      auto executed = execute_allocation_scenario(
          loaded_sequence, options, requests[i].plan, requests[i].label,
          requests[i].parking_seed, false, options.early_abort_eval_on_failure,
          &timing);
      timing.true_wall_s = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - plan_t0)
                               .count();
      if (save_executed_scenarios)
        results[i].serialized_result = serialize_executed_scenario_b64(executed);
      results[i].summary = std::move(executed.summary);
      results[i].timing = std::move(timing);
    }
    return results;
  }

  ScopedGlobalStreamSilencer silencer(!options.print_planning_status);
  RuntimeOptions worker_options = options;
  worker_options.planner_expansion_threads = 1;
  std::atomic<std::size_t> next_index{0};
  std::vector<std::thread> workers;
  workers.reserve(worker_count);

  for (std::size_t worker_idx = 0; worker_idx < worker_count; ++worker_idx)
  {
    workers.emplace_back(
        [&loaded_sequence, &worker_options, &requests, &results, &next_index,
         save_executed_scenarios]()
        {
          while (true)
          {
            std::size_t idx = next_index.fetch_add(1);
            if (idx >= requests.size())
              break;

            // Per-plan timing instrumentation: `timing` is a local stack
            // variable inside this worker's loop iteration, so each worker
            // owns a distinct instance with no cross-thread sharing (results
            // themselves are written to disjoint `idx` slots, same as
            // .serialized_result/.summary below).
            PlanTimingStats timing;
            const auto plan_t0 = std::chrono::steady_clock::now();
            auto executed = execute_allocation_scenario(
                loaded_sequence, worker_options, requests[idx].plan, requests[idx].label,
                requests[idx].parking_seed, true, worker_options.early_abort_eval_on_failure,
                &timing);
            timing.true_wall_s = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - plan_t0)
                                     .count();
            if (save_executed_scenarios)
              results[idx].serialized_result = serialize_executed_scenario_b64(executed);
            results[idx].summary = std::move(executed.summary);
            results[idx].timing = std::move(timing);
          }
        });
  }

  for (auto &worker : workers)
  {
    if (worker.joinable())
      worker.join();
  }

  return results;
}

// Forward declaration for the repair function (defined later in this file)
ExecutedScenario repair_destroyed_tasks_with_sampled_insertion(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<std::size_t> &destroyed_tasks,
    const std::vector<std::string> &robot_names,
    const std::vector<double> &destroy_scores,
    std::uint32_t parking_seed,
    std::uint32_t repair_seed,
    const std::string &label,
    bool disable_local_silencer);

std::vector<LnsEvaluationResult> evaluate_lns_batch(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<LnsSearchCandidate> &candidates,
    const std::vector<std::string> &robot_names,
    const std::string &label)
{
  std::vector<LnsEvaluationResult> results(candidates.size());
  if (candidates.empty())
    return results;

  std::size_t worker_count = std::min<std::size_t>(
      candidates.size(),
      static_cast<std::size_t>(std::max(1, options.lns_threads)));

  if (worker_count <= 1)
  {
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      auto executed = repair_destroyed_tasks_with_sampled_insertion(
          loaded_sequence, options, base_plan, base_summary, constraints,
          candidates[i].destroyed_tasks, robot_names, candidates[i].destroy_scores,
          candidates[i].parking_seed, candidates[i].repair_seed, label, false);
      results[i].summary = executed.summary;
      results[i].executed =
          std::make_unique<ExecutedScenario>(std::move(executed));
    }
    return results;
  }

  ScopedGlobalStreamSilencer silencer(true);
  RuntimeOptions worker_options = make_parallel_lns_worker_options(options);
  ScopedDebugVisualizationOverride debug_vis_override(options.lns_threads > 1);
  std::atomic<std::size_t> next_index{0};
  std::vector<std::thread> workers;
  workers.reserve(worker_count);

  for (std::size_t worker_idx = 0; worker_idx < worker_count; ++worker_idx)
  {
    workers.emplace_back(
        [&loaded_sequence, &worker_options, &base_plan, &base_summary, &constraints,
         &candidates, &results, &robot_names, &label, &next_index,
         worker_idx, worker_count]()
        {
          while (true)
          {
            std::size_t idx = next_index.fetch_add(1);
            if (idx >= candidates.size())
              break;

            print_lns_worker_status(worker_idx, worker_count,
                                    candidates[idx], "running");
            auto executed = repair_destroyed_tasks_with_sampled_insertion(
                loaded_sequence, worker_options, base_plan, base_summary, constraints,
                candidates[idx].destroyed_tasks, robot_names,
                candidates[idx].destroy_scores, candidates[idx].parking_seed,
                candidates[idx].repair_seed, label, true);
            results[idx].summary = executed.summary;
            results[idx].executed =
                std::make_unique<ExecutedScenario>(std::move(executed));
            print_lns_worker_status(worker_idx, worker_count,
                                    candidates[idx], "complete",
                                    &results[idx].summary);
          }
        });
  }

  for (auto &worker : workers)
  {
    if (worker.joinable())
      worker.join();
  }

  return results;
}

// ==========================================
// Plan Mutation
// ==========================================

AllocationScenarioPlan make_assignment_plan_from_greedy(
    const AllocationRunSummary &greedy_summary)
{
  AllocationScenarioPlan plan = greedy_summary.plan;
  plan.preferred_robot_names_by_original_task.assign(
      greedy_summary.task_rows.size(), "");

  for (const auto &row : greedy_summary.task_rows)
  {
    if (row.task_id <= 0)
      continue;
    std::size_t idx = static_cast<std::size_t>(row.task_id - 1);
    if (idx < plan.preferred_robot_names_by_original_task.size() &&
        row.status == "SUCCESS")
    {
      plan.preferred_robot_names_by_original_task[idx] = row.robot_name;
    }
  }

  return plan;
}

AllocationScenarioPlan mutate_assignment_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::string> &robot_names,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  std::size_t task_count = candidate.task_order.size();
  if (task_count == 0 || robot_names.empty())
    return candidate;

  if (candidate.preferred_robot_names_by_original_task.size() < task_count)
  {
    candidate.preferred_robot_names_by_original_task.resize(task_count);
  }

  std::vector<std::size_t> shuffled_indices(task_count);
  std::iota(shuffled_indices.begin(), shuffled_indices.end(), 0);
  std::shuffle(shuffled_indices.begin(), shuffled_indices.end(), rng);

  std::uniform_int_distribution<int> num_changes_dist(
      1, std::max<int>(1, static_cast<int>(task_count / 3)));
  int num_changes = num_changes_dist(rng);

  for (int i = 0; i < num_changes; ++i)
  {
    std::size_t task_idx = shuffled_indices[static_cast<std::size_t>(i)];
    const std::string &current =
        candidate.preferred_robot_names_by_original_task[task_idx];

    std::vector<std::string> alternatives;
    alternatives.reserve(robot_names.size());
    for (const auto &robot_name : robot_names)
    {
      if (robot_name != current)
        alternatives.push_back(robot_name);
    }

    if (alternatives.empty())
      continue;

    std::uniform_int_distribution<std::size_t> pick_dist(0, alternatives.size() - 1);
    candidate.preferred_robot_names_by_original_task[task_idx] =
        alternatives[pick_dist(rng)];
  }

  return candidate;
}

AllocationScenarioPlan mutate_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  std::size_t task_count = candidate.task_order.size();
  if (task_count < 2)
    return candidate;

  std::uniform_int_distribution<int> num_edits_dist(
      1, std::min<int>(3, static_cast<int>(task_count - 1)));
  int num_edits = num_edits_dist(rng);

  for (int edit = 0; edit < num_edits; ++edit)
  {
    std::uniform_int_distribution<std::size_t> index_dist(0, task_count - 1);
    std::size_t i = index_dist(rng);
    std::size_t j = index_dist(rng);
    while (j == i)
      j = index_dist(rng);

    std::bernoulli_distribution swap_or_relocate(0.5);
    if (swap_or_relocate(rng))
    {
      std::swap(candidate.task_order[i], candidate.task_order[j]);
      continue;
    }

    std::size_t moved_task = candidate.task_order[i];
    candidate.task_order.erase(candidate.task_order.begin() + static_cast<std::ptrdiff_t>(i));
    if (j > i)
      --j;
    candidate.task_order.insert(candidate.task_order.begin() + static_cast<std::ptrdiff_t>(j),
                                moved_task);
  }

  candidate.preferred_robot_names_by_original_task.clear();
  return candidate;
}

AllocationScenarioPlan mutate_full_shuffle_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  if (candidate.task_order.size() < 2)
    return candidate;

  std::shuffle(candidate.task_order.begin(), candidate.task_order.end(), rng);
  candidate.preferred_robot_names_by_original_task.clear();

  return candidate;
}

// ==========================================
// Run Comparison Predicates
// ==========================================

bool is_better_run(const AllocationRunSummary &candidate,
                   const AllocationRunSummary &current_best)
{
  if (!candidate.all_tasks_succeeded)
    return false;
  if (!current_best.all_tasks_succeeded)
    return true;
  return candidate.makespan + 1e-6 < current_best.makespan;
}

bool is_preferred_search_result(const AllocationRunSummary &candidate,
                                const AllocationRunSummary &current_best)
{
  if (candidate.all_tasks_succeeded != current_best.all_tasks_succeeded)
    return candidate.all_tasks_succeeded;

  if (candidate.all_tasks_succeeded)
    return candidate.makespan + 1e-6 < current_best.makespan;

  if (candidate.successful_tasks != current_best.successful_tasks)
    return candidate.successful_tasks > current_best.successful_tasks;

  if (candidate.failed_tasks != current_best.failed_tasks)
    return candidate.failed_tasks < current_best.failed_tasks;

  return false;
}

bool plans_have_same_order(const AllocationScenarioPlan &a,
                           const AllocationScenarioPlan &b)
{
  return a.task_order == b.task_order;
}

bool same_execution_signature(const AllocationRunSummary &a,
                              const AllocationRunSummary &b)
{
  if (a.task_rows.size() != b.task_rows.size())
    return false;

  for (std::size_t i = 0; i < a.task_rows.size(); ++i)
  {
    const auto &lhs = a.task_rows[i];
    const auto &rhs = b.task_rows[i];
    if (lhs.object_name != rhs.object_name ||
        lhs.robot_name != rhs.robot_name ||
        lhs.status != rhs.status)
    {
      return false;
    }
  }

  return true;
}

// ==========================================
// Plan Inspection Utilities
// ==========================================

std::string task_order_signature(const AllocationScenarioPlan &plan)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < plan.task_order.size(); ++i)
  {
    if (i > 0)
      oss << "-";
    oss << plan.task_order[i];
  }
  return oss.str();
}

int count_assignment_preference_changes(const AllocationScenarioPlan &a,
                                        const AllocationScenarioPlan &b)
{
  std::size_t limit = std::max(a.preferred_robot_names_by_original_task.size(),
                               b.preferred_robot_names_by_original_task.size());
  int changes = 0;
  for (std::size_t i = 0; i < limit; ++i)
  {
    std::string left =
        (i < a.preferred_robot_names_by_original_task.size())
            ? a.preferred_robot_names_by_original_task[i]
            : "";
    std::string right =
        (i < b.preferred_robot_names_by_original_task.size())
            ? b.preferred_robot_names_by_original_task[i]
            : "";
    if (left != right)
      ++changes;
  }
  return changes;
}

int count_task_position_changes(const AllocationScenarioPlan &a,
                                const AllocationScenarioPlan &b)
{
  std::size_t limit = std::min(a.task_order.size(), b.task_order.size());
  int changes = 0;
  for (std::size_t i = 0; i < limit; ++i)
  {
    if (a.task_order[i] != b.task_order[i])
      ++changes;
  }

  changes += static_cast<int>(
      std::max(a.task_order.size(), b.task_order.size()) - limit);
  return changes;
}

std::string preview_task_order(const AllocationScenarioPlan &plan,
                               const std::vector<FinalAllocation> &loaded_sequence,
                               std::size_t max_items)
{
  std::ostringstream oss;
  std::size_t shown = 0;
  for (std::size_t idx : plan.task_order)
  {
    if (idx >= loaded_sequence.size())
      continue;
    if (shown > 0)
      oss << ",";
    oss << loaded_sequence[idx].object.name;
    ++shown;
    if (shown >= max_items)
      break;
  }

  if (plan.task_order.size() > shown)
    oss << ",...";
  return oss.str();
}

std::string preview_assignment_changes(const AllocationScenarioPlan &base_plan,
                                       const AllocationScenarioPlan &candidate_plan,
                                       const std::vector<FinalAllocation> &loaded_sequence,
                                       std::size_t max_items)
{
  std::ostringstream oss;
  std::size_t shown = 0;
  std::size_t limit = std::min(loaded_sequence.size(),
                               std::max(base_plan.preferred_robot_names_by_original_task.size(),
                                        candidate_plan.preferred_robot_names_by_original_task.size()));

  for (std::size_t i = 0; i < limit; ++i)
  {
    std::string before =
        (i < base_plan.preferred_robot_names_by_original_task.size())
            ? base_plan.preferred_robot_names_by_original_task[i]
            : "";
    std::string after =
        (i < candidate_plan.preferred_robot_names_by_original_task.size())
            ? candidate_plan.preferred_robot_names_by_original_task[i]
            : "";
    if (before == after)
      continue;

    if (shown > 0)
      oss << ", ";
    oss << loaded_sequence[i].object.name << ":" << before << "->" << after;
    ++shown;
    if (shown >= max_items)
      break;
  }

  if (shown == 0)
    return "none";

  if (count_assignment_preference_changes(base_plan, candidate_plan) >
      static_cast<int>(shown))
  {
    oss << ", ...";
  }

  return oss.str();
}

AllocationScenarioPlan make_distinct_assignment_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::string> &robot_names,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  for (int attempt = 0; attempt < 8; ++attempt)
  {
    candidate = mutate_assignment_plan(base_plan, robot_names, rng);
    if (count_assignment_preference_changes(base_plan, candidate) > 0)
      return candidate;
  }
  return candidate;
}

AllocationScenarioPlan make_distinct_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    std::mt19937 &rng)
{
  AllocationScenarioPlan candidate = base_plan;
  for (int attempt = 0; attempt < 8; ++attempt)
  {
    candidate = mutate_sequence_plan(base_plan, rng);
    if (!plans_have_same_order(base_plan, candidate))
      return candidate;
  }
  return candidate;
}

// Forward declaration (defined in Order Constraint Learning section below)
static AllocationScenarioPlan project_plan_to_order_constraints(
    const AllocationScenarioPlan &plan,
    std::size_t task_count,
    const LearnedOrderConstraints &constraints);

bool sample_unique_sequence_plan(
    const AllocationScenarioPlan &base_plan,
    AllocationScenarioPlan (*mutator)(const AllocationScenarioPlan &, std::mt19937 &),
    std::mt19937 &rng,
    std::unordered_set<std::string> &tried_signatures,
    const LearnedOrderConstraints *constraints,
    AllocationScenarioPlan *out_plan,
    int max_attempts)
{
  if (!out_plan || !mutator)
    return false;

  for (int attempt = 0; attempt < max_attempts; ++attempt)
  {
    AllocationScenarioPlan candidate = mutator(base_plan, rng);
    if (constraints)
    {
      candidate = project_plan_to_order_constraints(
          candidate, base_plan.task_order.size(), *constraints);
    }
    if (plans_have_same_order(base_plan, candidate))
      continue;

    std::string signature = task_order_signature(candidate);
    if (!tried_signatures.insert(signature).second)
      continue;

    *out_plan = std::move(candidate);
    return true;
  }

  return false;
}

// ==========================================
// Order Constraint Learning
// ==========================================

LearnedOrderConstraints make_learned_order_constraints(std::size_t task_count)
{
  LearnedOrderConstraints constraints;
  constraints.evidence_counts.assign(
      task_count, std::vector<int>(task_count, 0));
  constraints.enforced.assign(
      task_count, std::vector<bool>(task_count, false));
  return constraints;
}

static bool order_constraint_path_exists(
    const LearnedOrderConstraints &constraints,
    std::size_t start_task,
    std::size_t goal_task)
{
  if (start_task >= constraints.enforced.size() ||
      goal_task >= constraints.enforced.size())
  {
    return false;
  }

  std::vector<bool> visited(constraints.enforced.size(), false);
  std::vector<std::size_t> stack = {start_task};
  visited[start_task] = true;

  while (!stack.empty())
  {
    std::size_t current = stack.back();
    stack.pop_back();
    if (current == goal_task)
      return true;

    for (std::size_t next = 0; next < constraints.enforced[current].size(); ++next)
    {
      if (!constraints.enforced[current][next] || visited[next])
        continue;
      visited[next] = true;
      stack.push_back(next);
    }
  }

  return false;
}

static AllocationScenarioPlan project_plan_to_order_constraints(
    const AllocationScenarioPlan &plan,
    std::size_t task_count,
    const LearnedOrderConstraints &constraints)
{
  AllocationScenarioPlan projected = plan;
  auto base_order = normalized_task_order(plan, task_count);

  if (task_count == 0 || constraints.enforced.size() != task_count)
  {
    projected.task_order = std::move(base_order);
    return projected;
  }

  std::vector<int> preferred_rank(task_count, static_cast<int>(task_count));
  for (std::size_t i = 0; i < base_order.size(); ++i)
  {
    preferred_rank[base_order[i]] = static_cast<int>(i);
  }

  std::vector<int> indegree(task_count, 0);
  for (std::size_t before = 0; before < task_count; ++before)
  {
    for (std::size_t after = 0; after < task_count; ++after)
    {
      if (constraints.enforced[before][after])
        ++indegree[after];
    }
  }

  std::vector<bool> scheduled(task_count, false);
  projected.task_order.clear();
  projected.task_order.reserve(task_count);

  for (std::size_t step = 0; step < task_count; ++step)
  {
    std::size_t chosen = task_count;
    int chosen_rank = std::numeric_limits<int>::max();
    for (std::size_t task_idx = 0; task_idx < task_count; ++task_idx)
    {
      if (scheduled[task_idx] || indegree[task_idx] != 0)
        continue;

      int rank = preferred_rank[task_idx];
      if (rank < chosen_rank ||
          (rank == chosen_rank && task_idx < chosen))
      {
        chosen = task_idx;
        chosen_rank = rank;
      }
    }

    if (chosen >= task_count)
    {
      projected.task_order = std::move(base_order);
      return projected;
    }

    scheduled[chosen] = true;
    projected.task_order.push_back(chosen);
    for (std::size_t after = 0; after < task_count; ++after)
    {
      if (constraints.enforced[chosen][after])
        --indegree[after];
    }
  }

  return projected;
}

static std::vector<int> build_task_position_lookup(
    const AllocationScenarioPlan &plan,
    std::size_t task_count)
{
  auto order = normalized_task_order(plan, task_count);
  std::vector<int> positions(task_count, -1);
  for (std::size_t i = 0; i < order.size(); ++i)
  {
    positions[order[i]] = static_cast<int>(i);
  }
  return positions;
}

static bool failure_reason_mentions_object(
    const std::string &failure_reason,
    const std::string &object_name)
{
  if (failure_reason.empty() || object_name.empty())
    return false;

  std::size_t pos = failure_reason.find(object_name);
  while (pos != std::string::npos)
  {
    bool left_ok = (pos == 0) ||
                   !std::isalnum(static_cast<unsigned char>(failure_reason[pos - 1]));
    std::size_t after_pos = pos + object_name.size();
    bool right_ok = (after_pos >= failure_reason.size()) ||
                    !std::isalnum(static_cast<unsigned char>(failure_reason[after_pos]));
    if (left_ok && right_ok)
      return true;
    pos = failure_reason.find(object_name, pos + 1);
  }

  return false;
}

static bool note_order_constraint_evidence(
    LearnedOrderConstraints &constraints,
    std::size_t before_task,
    std::size_t after_task,
    int required_support,
    const std::vector<FinalAllocation> &loaded_sequence,
    const char *reason_tag)
{
  if (before_task >= constraints.evidence_counts.size() ||
      after_task >= constraints.evidence_counts.size() ||
      before_task == after_task)
  {
    return false;
  }

  int &support = constraints.evidence_counts[before_task][after_task];
  ++support;
  bool enforced_now = false;

  if (!constraints.enforced[before_task][after_task] &&
      support >= required_support &&
      !order_constraint_path_exists(constraints, after_task, before_task))
  {
    constraints.enforced[before_task][after_task] = true;
    ++constraints.enforced_count;
    enforced_now = true;
  }

  if (support == 1 || enforced_now)
  {
    std::cout << "[Learn][Order] "
              << loaded_sequence[before_task].object.name << " -> "
              << loaded_sequence[after_task].object.name
              << " support=" << support << "/" << required_support;
    if (reason_tag && *reason_tag)
      std::cout << " reason=" << reason_tag;
    if (enforced_now)
      std::cout << " enforced";
    std::cout << std::endl;
  }
  return enforced_now;
}

void learn_order_constraints_from_failed_summary(
    const AllocationRunSummary &failed_summary,
    const AllocationScenarioPlan &reference_plan,
    const std::vector<FinalAllocation> &loaded_sequence,
    LearnedOrderConstraints &constraints)
{
  if (failed_summary.all_tasks_succeeded || failed_summary.task_rows.empty() ||
      loaded_sequence.empty())
  {
    return;
  }

  auto candidate_order = normalized_task_order(
      failed_summary.plan, loaded_sequence.size());
  int failed_position = -1;
  const TaskCsvRow *failed_row = nullptr;
  for (std::size_t i = 0; i < failed_summary.task_rows.size(); ++i)
  {
    if (failed_summary.task_rows[i].status != "SUCCESS")
    {
      failed_position = static_cast<int>(i);
      failed_row = &failed_summary.task_rows[i];
      break;
    }
  }

  if (!failed_row || failed_position < 0 ||
      static_cast<std::size_t>(failed_position) >= candidate_order.size())
  {
    return;
  }

  std::size_t failed_task = candidate_order[static_cast<std::size_t>(failed_position)];
  auto candidate_positions = build_task_position_lookup(
      failed_summary.plan, loaded_sequence.size());
  auto reference_positions = build_task_position_lookup(
      reference_plan, loaded_sequence.size());

  std::vector<bool> handled_before(loaded_sequence.size(), false);

  for (std::size_t before_task = 0; before_task < loaded_sequence.size(); ++before_task)
  {
    if (before_task == failed_task ||
        candidate_positions[before_task] < 0 ||
        candidate_positions[failed_task] < 0)
    {
      continue;
    }

    if (!failure_reason_mentions_object(
            failed_row->failure_reason,
            loaded_sequence[before_task].object.name))
    {
      continue;
    }

    if (reference_positions[before_task] < reference_positions[failed_task] &&
        candidate_positions[before_task] > candidate_positions[failed_task])
    {
      note_order_constraint_evidence(
          constraints, before_task, failed_task, 1,
          loaded_sequence, "blocking-object");
      handled_before[before_task] = true;
    }
    else if (reference_positions[before_task] > reference_positions[failed_task] &&
             candidate_positions[before_task] < candidate_positions[failed_task])
    {
      note_order_constraint_evidence(
          constraints, failed_task, before_task, 1,
          loaded_sequence, "blocking-object");
      handled_before[before_task] = true;
    }
  }

  std::vector<std::size_t> missing_predecessors;
  std::vector<std::size_t> advanced_successors;
  for (std::size_t before_task = 0; before_task < loaded_sequence.size(); ++before_task)
  {
    if (before_task == failed_task || handled_before[before_task] ||
        reference_positions[before_task] < 0 ||
        reference_positions[failed_task] < 0 ||
        candidate_positions[before_task] < 0 ||
        candidate_positions[failed_task] < 0)
    {
      continue;
    }

    if (reference_positions[before_task] < reference_positions[failed_task] &&
        candidate_positions[before_task] > candidate_positions[failed_task])
    {
      missing_predecessors.push_back(before_task);
    }
    else if (reference_positions[before_task] > reference_positions[failed_task] &&
             candidate_positions[before_task] < candidate_positions[failed_task])
    {
      advanced_successors.push_back(before_task);
    }
  }

  std::sort(missing_predecessors.begin(), missing_predecessors.end(),
            [&](std::size_t lhs, std::size_t rhs)
            {
              return reference_positions[lhs] > reference_positions[rhs];
            });
  std::sort(advanced_successors.begin(), advanced_successors.end(),
            [&](std::size_t lhs, std::size_t rhs)
            {
              return reference_positions[lhs] < reference_positions[rhs];
            });

  int learned_from_reference = 0;
  for (std::size_t before_task : missing_predecessors)
  {
    int gap = reference_positions[failed_task] - reference_positions[before_task];
    int required_support = (gap <= 1) ? 1 : 2;
    note_order_constraint_evidence(
        constraints, before_task, failed_task, required_support,
        loaded_sequence, "failed-order");
    ++learned_from_reference;
    if (learned_from_reference >= 2)
      break;
  }

  int learned_successor_constraints = 0;
  for (std::size_t after_task : advanced_successors)
  {
    int gap = reference_positions[after_task] - reference_positions[failed_task];
    int required_support = (gap <= 1) ? 1 : 2;
    note_order_constraint_evidence(
        constraints, failed_task, after_task, required_support,
        loaded_sequence, "failed-order");
    ++learned_successor_constraints;
    if (learned_successor_constraints >= 2)
      break;
  }
}

// ==========================================
// LNS Destroy Operators & Helpers
// ==========================================

static std::unordered_map<std::string, std::size_t> build_object_index_lookup(
    const std::vector<FinalAllocation> &loaded_sequence)
{
  std::unordered_map<std::string, std::size_t> lookup;
  lookup.reserve(loaded_sequence.size());
  for (std::size_t i = 0; i < loaded_sequence.size(); ++i)
  {
    lookup[loaded_sequence[i].object.name] = i;
  }
  return lookup;
}

static double summary_effective_completion_time(const AllocationRunSummary &summary)
{
  if (std::isfinite(summary.makespan))
    return summary.makespan;

  double completion = 0.0;
  for (const auto &row : summary.task_rows)
  {
    completion = std::max(completion, std::max(row.end_time, row.start_time));
  }
  return completion;
}

std::vector<double> compute_task_destroy_scores(
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence)
{
  auto object_lookup = build_object_index_lookup(loaded_sequence);
  const double ref_completion =
      std::max(1.0, summary_effective_completion_time(summary));

  std::vector<double> scores(loaded_sequence.size(), 1.0);
  for (const auto &row : summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it == object_lookup.end())
      continue;

    std::size_t task_idx = it->second;
    double end_time = std::max(row.end_time, row.start_time);
    double wait_component = std::sqrt(std::max(0.0, row.total_waiting));
    double end_ratio = end_time > 0.0 ? (end_time / ref_completion) : 0.0;
    double critical_component = 6.0 * std::max(0.0, end_ratio - 0.55);
    double tail_bonus = (end_ratio >= 0.85) ? 6.0 : 0.0;
    double failure_bonus = (row.status == "SUCCESS") ? 0.0 : 20.0;
    scores[task_idx] += 4.0 * wait_component + critical_component +
                        tail_bonus + failure_bonus;
  }

  return scores;
}

static std::vector<std::size_t> sample_weighted_task_indices(
    const std::vector<double> &weights,
    std::size_t desired_count,
    std::mt19937 &rng)
{
  desired_count = std::min(desired_count, weights.size());
  std::vector<double> mutable_weights = weights;
  std::vector<std::size_t> selected;
  selected.reserve(desired_count);

  while (selected.size() < desired_count)
  {
    double weight_sum = 0.0;
    for (double weight : mutable_weights)
      weight_sum += std::max(0.0, weight);

    if (weight_sum <= 1e-9)
      break;

    std::discrete_distribution<std::size_t> pick_dist(
        mutable_weights.begin(), mutable_weights.end());
    std::size_t picked = pick_dist(rng);
    if (picked >= mutable_weights.size() || mutable_weights[picked] <= 0.0)
      continue;

    selected.push_back(picked);
    mutable_weights[picked] = 0.0;
  }

  if (selected.size() < desired_count)
  {
    std::vector<std::size_t> fallback;
    fallback.reserve(weights.size());
    for (std::size_t i = 0; i < weights.size(); ++i)
    {
      if (std::find(selected.begin(), selected.end(), i) == selected.end())
        fallback.push_back(i);
    }
    std::shuffle(fallback.begin(), fallback.end(), rng);
    while (selected.size() < desired_count && !fallback.empty())
    {
      selected.push_back(fallback.back());
      fallback.pop_back();
    }
  }

  return selected;
}

std::vector<std::size_t> destroy_random_tasks(
    std::size_t task_count,
    std::size_t destroy_count,
    std::mt19937 &rng)
{
  std::vector<std::size_t> tasks(task_count);
  std::iota(tasks.begin(), tasks.end(), 0);
  std::shuffle(tasks.begin(), tasks.end(), rng);
  tasks.resize(std::min(destroy_count, tasks.size()));
  return tasks;
}

std::vector<std::size_t> destroy_wait_neighborhood_tasks(
    const AllocationScenarioPlan &plan,
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence,
    std::size_t destroy_count,
    std::mt19937 &rng)
{
  destroy_count = std::min(destroy_count, loaded_sequence.size());
  auto scores = compute_task_destroy_scores(summary, loaded_sequence);
  auto order = normalized_task_order(plan, loaded_sequence.size());
  auto positions = build_task_position_lookup(plan, loaded_sequence.size());
  auto object_lookup = build_object_index_lookup(loaded_sequence);

  std::vector<std::string> robot_by_task(loaded_sequence.size(), "");
  for (const auto &row : summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it != object_lookup.end())
      robot_by_task[it->second] = row.robot_name;
  }

  std::vector<bool> selected(loaded_sequence.size(), false);
  std::vector<std::size_t> destroyed;
  destroyed.reserve(destroy_count);

  auto add_task = [&](std::size_t task_idx)
  {
    if (task_idx >= selected.size() || selected[task_idx])
      return;
    selected[task_idx] = true;
    destroyed.push_back(task_idx);
  };

  std::size_t anchor_count = std::max<std::size_t>(1, (destroy_count + 1) / 2);
  auto anchors = sample_weighted_task_indices(scores, anchor_count, rng);
  for (std::size_t anchor : anchors)
  {
    add_task(anchor);
    if (destroyed.size() >= destroy_count)
      break;

    int position = positions[anchor];
    if (position >= 0)
    {
      if (position > 0)
        add_task(order[static_cast<std::size_t>(position - 1)]);
      if (destroyed.size() >= destroy_count)
        break;
      if (static_cast<std::size_t>(position + 1) < order.size())
        add_task(order[static_cast<std::size_t>(position + 1)]);
      if (destroyed.size() >= destroy_count)
        break;
    }

    const std::string &robot_name = robot_by_task[anchor];
    if (!robot_name.empty())
    {
      std::size_t best_peer = loaded_sequence.size();
      double best_score = -1.0;
      for (std::size_t task_idx = 0; task_idx < robot_by_task.size(); ++task_idx)
      {
        if (selected[task_idx] || robot_by_task[task_idx] != robot_name)
          continue;
        if (scores[task_idx] > best_score)
        {
          best_score = scores[task_idx];
          best_peer = task_idx;
        }
      }
      if (best_peer < loaded_sequence.size())
        add_task(best_peer);
    }

    if (destroyed.size() >= destroy_count)
      break;
  }

  auto extras = sample_weighted_task_indices(scores, destroy_count, rng);
  for (std::size_t task_idx : extras)
  {
    add_task(task_idx);
    if (destroyed.size() >= destroy_count)
      break;
  }

  return destroyed;
}

std::vector<std::size_t> destroy_critical_suffix_tasks(
    const AllocationRunSummary &summary,
    const std::vector<FinalAllocation> &loaded_sequence,
    std::size_t destroy_count,
    std::mt19937 &rng)
{
  destroy_count = std::min(destroy_count, loaded_sequence.size());
  auto object_lookup = build_object_index_lookup(loaded_sequence);
  auto scores = compute_task_destroy_scores(summary, loaded_sequence);

  struct TimedTask
  {
    std::size_t task_idx = 0;
    std::string robot_name;
    double end_time = -1.0;
  };

  std::vector<TimedTask> timed_tasks;
  timed_tasks.reserve(summary.task_rows.size());
  for (const auto &row : summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it == object_lookup.end())
      continue;
    timed_tasks.push_back({it->second, row.robot_name,
                           std::max(row.end_time, row.start_time)});
  }

  std::sort(timed_tasks.begin(), timed_tasks.end(),
            [](const TimedTask &a, const TimedTask &b)
            { return a.end_time > b.end_time; });

  std::vector<bool> selected(loaded_sequence.size(), false);
  std::vector<std::size_t> destroyed;
  destroyed.reserve(destroy_count);

  auto add_task = [&](std::size_t task_idx)
  {
    if (task_idx >= selected.size() || selected[task_idx])
      return;
    selected[task_idx] = true;
    destroyed.push_back(task_idx);
  };

  std::string critical_robot =
      timed_tasks.empty() ? "" : timed_tasks.front().robot_name;
  for (const auto &task : timed_tasks)
  {
    if (!critical_robot.empty() && task.robot_name == critical_robot)
      add_task(task.task_idx);
    if (destroyed.size() >= destroy_count)
      return destroyed;
  }

  for (const auto &task : timed_tasks)
  {
    add_task(task.task_idx);
    if (destroyed.size() >= destroy_count)
      return destroyed;
  }

  auto extras = sample_weighted_task_indices(scores, destroy_count, rng);
  for (std::size_t task_idx : extras)
  {
    add_task(task_idx);
    if (destroyed.size() >= destroy_count)
      break;
  }

  return destroyed;
}

static AllocationScenarioPlan remove_tasks_from_plan(
    const AllocationScenarioPlan &base_plan,
    const std::vector<std::size_t> &removed_tasks,
    std::size_t task_count)
{
  AllocationScenarioPlan partial = base_plan;
  partial.task_order = normalized_task_order(base_plan, task_count);
  partial.preferred_robot_names_by_original_task.resize(task_count);

  std::vector<bool> removed(task_count, false);
  for (std::size_t task_idx : removed_tasks)
  {
    if (task_idx < task_count)
    {
      removed[task_idx] = true;
      partial.preferred_robot_names_by_original_task[task_idx].clear();
    }
  }

  partial.task_order.erase(
      std::remove_if(partial.task_order.begin(), partial.task_order.end(),
                     [&](std::size_t task_idx)
                     { return task_idx < removed.size() && removed[task_idx]; }),
      partial.task_order.end());
  return partial;
}

static AllocationScenarioPlan complete_partial_plan_with_pending(
    const AllocationScenarioPlan &partial_plan,
    const std::vector<std::size_t> &pending_tasks,
    const std::vector<int> &base_positions,
    const AllocationScenarioPlan &base_plan,
    std::size_t task_count)
{
  AllocationScenarioPlan completed = partial_plan;
  completed.preferred_robot_names_by_original_task.resize(task_count);

  std::vector<std::size_t> pending_sorted = pending_tasks;
  std::sort(pending_sorted.begin(), pending_sorted.end(),
            [&](std::size_t a, std::size_t b)
            {
              int pos_a = (a < base_positions.size()) ? base_positions[a] : -1;
              int pos_b = (b < base_positions.size()) ? base_positions[b] : -1;
              return pos_a < pos_b;
            });

  std::size_t base_pref_count =
      base_plan.preferred_robot_names_by_original_task.size();
  for (std::size_t task_idx : pending_sorted)
  {
    std::size_t insert_pos = completed.task_order.size();
    if (task_idx < base_positions.size() && base_positions[task_idx] >= 0)
    {
      insert_pos = std::min<std::size_t>(
          static_cast<std::size_t>(base_positions[task_idx]),
          completed.task_order.size());
    }
    completed.task_order.insert(
        completed.task_order.begin() + static_cast<std::ptrdiff_t>(insert_pos),
        task_idx);

    if (task_idx < base_pref_count &&
        completed.preferred_robot_names_by_original_task[task_idx].empty())
    {
      completed.preferred_robot_names_by_original_task[task_idx] =
          base_plan.preferred_robot_names_by_original_task[task_idx];
    }
  }

  return completed;
}

static std::vector<std::size_t> build_sampled_insertion_positions(
    const AllocationScenarioPlan &partial_plan,
    int original_position,
    std::mt19937 &rng,
    std::size_t max_positions = 4)
{
  std::size_t total_positions = partial_plan.task_order.size() + 1;
  if (total_positions <= max_positions)
  {
    std::vector<std::size_t> positions(total_positions);
    std::iota(positions.begin(), positions.end(), 0);
    return positions;
  }

  std::vector<std::size_t> positions;
  positions.reserve(max_positions);
  auto add_position = [&](std::size_t position)
  {
    position = std::min(position, total_positions - 1);
    if (std::find(positions.begin(), positions.end(), position) == positions.end())
      positions.push_back(position);
  };

  add_position(0);
  add_position(total_positions - 1);

  std::size_t clamped_original = 0;
  if (original_position > 0)
  {
    clamped_original = std::min<std::size_t>(
        static_cast<std::size_t>(original_position), total_positions - 1);
  }
  add_position(clamped_original);
  if (clamped_original > 0)
    add_position(clamped_original - 1);
  if (clamped_original + 1 < total_positions)
    add_position(clamped_original + 1);

  std::uniform_int_distribution<std::size_t> pos_dist(0, total_positions - 1);
  while (positions.size() < max_positions)
  {
    add_position(pos_dist(rng));
  }

  std::sort(positions.begin(), positions.end());
  return positions;
}

ExecutedScenario repair_destroyed_tasks_with_sampled_insertion(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &base_plan,
    const AllocationRunSummary &base_summary,
    const LearnedOrderConstraints &constraints,
    const std::vector<std::size_t> &destroyed_tasks,
    const std::vector<std::string> &robot_names,
    const std::vector<double> &destroy_scores,
    std::uint32_t parking_seed,
    std::uint32_t repair_seed,
    const std::string &label,
    bool disable_local_silencer)
{
  std::mt19937 rng(repair_seed);
  RuntimeOptions lns_options = options;
  if (!lns_options.enable_lns_fine_segment_retry)
  {
    auto keep_primary_only = [](const TransitPlannerStep &step)
    {
      return step.method == TransitPlannerMethod::PrimaryHybridAStar;
    };
    lns_options.initial_transit_methods.erase(
        std::remove_if(lns_options.initial_transit_methods.begin(),
                       lns_options.initial_transit_methods.end(),
                       [&](const TransitPlannerStep &step)
                       { return !keep_primary_only(step); }),
        lns_options.initial_transit_methods.end());
    lns_options.segment_transit_methods.erase(
        std::remove_if(lns_options.segment_transit_methods.begin(),
                       lns_options.segment_transit_methods.end(),
                       [&](const TransitPlannerStep &step)
                       { return !keep_primary_only(step); }),
        lns_options.segment_transit_methods.end());
  }

  std::size_t task_count = loaded_sequence.size();
  AllocationScenarioPlan partial_plan =
      remove_tasks_from_plan(base_plan, destroyed_tasks, task_count);
  auto base_positions = build_task_position_lookup(base_plan, task_count);
  auto object_lookup = build_object_index_lookup(loaded_sequence);

  std::vector<std::string> executed_robot_by_task(task_count, "");
  std::vector<double> wait_by_task(task_count, 0.0);
  for (const auto &row : base_summary.task_rows)
  {
    auto it = object_lookup.find(row.object_name);
    if (it != object_lookup.end())
    {
      executed_robot_by_task[it->second] = row.robot_name;
      wait_by_task[it->second] = row.total_waiting;
    }
  }

  auto make_ranked_repair_order = [&]()
  {
    std::vector<std::size_t> repair_order = destroyed_tasks;
    std::shuffle(repair_order.begin(), repair_order.end(), rng);
    std::stable_sort(repair_order.begin(), repair_order.end(),
                     [&](std::size_t a, std::size_t b)
                     {
                       double score_a = (a < destroy_scores.size()) ? destroy_scores[a] : 0.0;
                       double score_b = (b < destroy_scores.size()) ? destroy_scores[b] : 0.0;
                       return score_a > score_b;
                     });
    return repair_order;
  };

  if (lns_options.lns_reassign_only)
  {
    AllocationScenarioPlan reassigned_plan = base_plan;
    reassigned_plan.task_order = normalized_task_order(base_plan, task_count);
    reassigned_plan.preferred_robot_names_by_original_task.resize(task_count);

    for (std::size_t task_idx = 0; task_idx < task_count; ++task_idx)
    {
      if (task_idx < executed_robot_by_task.size() &&
          !executed_robot_by_task[task_idx].empty())
      {
        reassigned_plan.preferred_robot_names_by_original_task[task_idx] =
            executed_robot_by_task[task_idx];
      }
    }

    auto choose_alternative_robot =
        [&](const AllocationScenarioPlan &plan,
            std::size_t task_idx) -> std::string
    {
      if (task_idx >= task_count)
        return "";

      std::string current =
          (task_idx < plan.preferred_robot_names_by_original_task.size())
              ? plan.preferred_robot_names_by_original_task[task_idx]
              : "";
      if (current.empty() && task_idx < executed_robot_by_task.size())
        current = executed_robot_by_task[task_idx];

      std::vector<std::string> alternatives;
      alternatives.reserve(robot_names.size());
      for (const auto &robot_name : robot_names)
      {
        if (robot_name != current)
          alternatives.push_back(robot_name);
      }

      if (alternatives.empty())
        return current;

      std::shuffle(alternatives.begin(), alternatives.end(), rng);
      return alternatives.front();
    };

    std::vector<std::size_t> valid_destroyed_tasks;
    valid_destroyed_tasks.reserve(destroyed_tasks.size());
    for (std::size_t task_idx : make_ranked_repair_order())
    {
      if (task_idx >= task_count)
        continue;

      valid_destroyed_tasks.push_back(task_idx);
      std::string chosen_preference =
          choose_alternative_robot(reassigned_plan, task_idx);
      if (!chosen_preference.empty())
      {
        reassigned_plan.preferred_robot_names_by_original_task[task_idx] =
            chosen_preference;
      }
    }

    auto repaired = execute_allocation_scenario(
        loaded_sequence, lns_options, reassigned_plan, label,
        parking_seed, disable_local_silencer,
        lns_options.early_abort_eval_on_failure);
    repaired.summary.label = label;

    for (int refine_iter = 0;
         refine_iter < 2 && !valid_destroyed_tasks.empty();
         ++refine_iter)
    {
      AllocationScenarioPlan candidate_plan = repaired.summary.plan;
      candidate_plan.task_order = normalized_task_order(base_plan, task_count);
      candidate_plan.preferred_robot_names_by_original_task.resize(task_count);

      std::uniform_int_distribution<std::size_t> destroyed_pick_dist(
          0, valid_destroyed_tasks.size() - 1);
      std::size_t picked_task =
          valid_destroyed_tasks[destroyed_pick_dist(rng)];

      std::string chosen_preference =
          choose_alternative_robot(candidate_plan, picked_task);
      if (!chosen_preference.empty())
      {
        candidate_plan.preferred_robot_names_by_original_task[picked_task] =
            chosen_preference;
      }

      auto refined = execute_allocation_scenario(
          loaded_sequence, lns_options, candidate_plan, label,
          parking_seed, disable_local_silencer,
          lns_options.early_abort_eval_on_failure);
      if (is_preferred_search_result(refined.summary, repaired.summary))
      {
        repaired = std::move(refined);
        repaired.summary.label = label;
      }
    }

    return repaired;
  }

  std::vector<std::size_t> repair_order = make_ranked_repair_order();

  for (std::size_t task_idx : repair_order)
  {
    auto candidate_positions = build_sampled_insertion_positions(
        partial_plan,
        (task_idx < base_positions.size()) ? base_positions[task_idx] : -1,
        rng);

    std::size_t chosen_position = candidate_positions.empty() ? partial_plan.task_order.size()
                                                              : candidate_positions.front();
    int original_position =
        (task_idx < base_positions.size()) ? base_positions[task_idx] : -1;
    if (wait_by_task[task_idx] > 0.5)
    {
      for (std::size_t pos : candidate_positions)
      {
        if (original_position >= 0 &&
            pos <= static_cast<std::size_t>(original_position))
        {
          chosen_position = pos;
          break;
        }
      }
    }
    else if (!candidate_positions.empty())
    {
      std::uniform_int_distribution<std::size_t> pick_pos_dist(
          0, candidate_positions.size() - 1);
      chosen_position = candidate_positions[pick_pos_dist(rng)];
    }

    std::string chosen_preference = "";
    const std::string base_preference =
        (task_idx < base_plan.preferred_robot_names_by_original_task.size())
            ? base_plan.preferred_robot_names_by_original_task[task_idx]
            : "";
    const std::string executed_robot =
        (task_idx < executed_robot_by_task.size())
            ? executed_robot_by_task[task_idx]
            : "";

    if (wait_by_task[task_idx] > 1.0 && !executed_robot.empty())
    {
      std::vector<std::string> alternative_robots;
      for (const auto &robot_name : robot_names)
      {
        if (robot_name != executed_robot)
          alternative_robots.push_back(robot_name);
      }
      if (!alternative_robots.empty())
      {
        std::shuffle(alternative_robots.begin(), alternative_robots.end(), rng);
        chosen_preference = alternative_robots.front();
      }
    }

    if (chosen_preference.empty())
    {
      std::vector<std::string> fallback_preferences;
      auto add_preference = [&](const std::string &pref)
      {
        if (std::find(fallback_preferences.begin(), fallback_preferences.end(), pref) ==
            fallback_preferences.end())
        {
          fallback_preferences.push_back(pref);
        }
      };

      add_preference(base_preference);
      add_preference(executed_robot);
      add_preference("");

      std::vector<std::string> shuffled_robot_names = robot_names;
      std::shuffle(shuffled_robot_names.begin(), shuffled_robot_names.end(), rng);
      for (const auto &robot_name : shuffled_robot_names)
      {
        add_preference(robot_name);
        if (fallback_preferences.size() >= 3)
          break;
      }

      if (!fallback_preferences.empty())
      {
        std::uniform_int_distribution<std::size_t> pref_pick_dist(
            0, fallback_preferences.size() - 1);
        chosen_preference = fallback_preferences[pref_pick_dist(rng)];
      }
    }

    partial_plan.preferred_robot_names_by_original_task.resize(task_count);
    partial_plan.task_order.insert(
        partial_plan.task_order.begin() + static_cast<std::ptrdiff_t>(chosen_position),
        task_idx);
    partial_plan.preferred_robot_names_by_original_task[task_idx] =
        chosen_preference;
  }

  partial_plan = project_plan_to_order_constraints(
      partial_plan, task_count, constraints);

  auto repaired = execute_allocation_scenario(
      loaded_sequence, lns_options, partial_plan, label,
      parking_seed, disable_local_silencer,
      lns_options.early_abort_eval_on_failure);
  repaired.summary.label = label;

  for (int refine_iter = 0; refine_iter < 2 && !destroyed_tasks.empty(); ++refine_iter)
  {
    AllocationScenarioPlan candidate_plan = repaired.summary.plan;
    candidate_plan.preferred_robot_names_by_original_task.resize(task_count);

    std::uniform_int_distribution<std::size_t> destroyed_pick_dist(
        0, destroyed_tasks.size() - 1);
    std::size_t picked_task = destroyed_tasks[destroyed_pick_dist(rng)];

    std::bernoulli_distribution swap_or_reassign(0.5);
    if (swap_or_reassign(rng) && candidate_plan.task_order.size() > 1)
    {
      auto positions = build_task_position_lookup(candidate_plan, task_count);
      int current_pos = (picked_task < positions.size()) ? positions[picked_task] : -1;
      if (current_pos >= 0)
      {
        std::uniform_int_distribution<std::size_t> pos_dist(
            0, candidate_plan.task_order.size() - 1);
        std::size_t new_pos = pos_dist(rng);
        if (new_pos != static_cast<std::size_t>(current_pos))
        {
          candidate_plan.task_order.erase(
              candidate_plan.task_order.begin() + current_pos);
          if (new_pos > static_cast<std::size_t>(current_pos))
            --new_pos;
          candidate_plan.task_order.insert(
              candidate_plan.task_order.begin() + static_cast<std::ptrdiff_t>(new_pos),
              picked_task);
        }
      }
    }
    else
    {
      std::vector<std::string> preference_options;
      preference_options.push_back("");
      for (const auto &robot_name : robot_names)
      {
        if (std::find(preference_options.begin(), preference_options.end(), robot_name) ==
            preference_options.end())
        {
          preference_options.push_back(robot_name);
        }
      }
      std::uniform_int_distribution<std::size_t> pref_dist(
          0, preference_options.size() - 1);
      candidate_plan.preferred_robot_names_by_original_task[picked_task] =
          preference_options[pref_dist(rng)];
    }

    candidate_plan = project_plan_to_order_constraints(
        candidate_plan, task_count, constraints);

    auto refined = execute_allocation_scenario(
        loaded_sequence, lns_options, candidate_plan, label,
        parking_seed, disable_local_silencer,
        lns_options.early_abort_eval_on_failure);
    if (is_preferred_search_result(refined.summary, repaired.summary))
    {
      repaired = std::move(refined);
      repaired.summary.label = label;
    }
  }

  return repaired;
}

// ==========================================
// Print Utilities
// ==========================================

void print_comparison_line(const AllocationRunSummary &summary,
                           double greedy_makespan)
{
  std::cout << "[Compare] " << summary.label << ": ";
  if (!summary.all_tasks_succeeded)
  {
    std::cout << "infeasible (" << summary.successful_tasks << "/"
              << (summary.successful_tasks + summary.failed_tasks)
              << " tasks succeeded)" << std::endl;
    return;
  }

  std::cout << "makespan=" << std::fixed << std::setprecision(2)
            << summary.makespan << "s";
  if (std::isfinite(greedy_makespan))
  {
    double delta = summary.makespan - greedy_makespan;
    std::cout << " (delta vs greedy="
              << (delta >= 0.0 ? "+" : "") << delta << "s)";
  }
  std::cout << ", seed=" << summary.parking_seed << std::endl;
}

std::vector<std::string> collect_robot_names(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options)
{
  ScopedStreamSilencer silencer(true);
  Params params;
  std::unordered_map<std::string, EntityMeta *> entities;
  TimeTable timetable(0.5);
  std::vector<RobotMeta *> all_robots;
  initialize_environment(loaded_sequence, options,
                         mix_seed(options.base_random_seed, 0x13579bdu),
                         params, entities, timetable, all_robots, false);

  std::vector<std::string> robot_names;
  robot_names.reserve(all_robots.size());
  for (auto *robot : all_robots)
  {
    if (robot)
      robot_names.push_back(robot->name);
  }

  cleanup_entities(entities);
  return robot_names;
}

std::vector<RobotMeta> collect_robot_metas(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options)
{
  ScopedStreamSilencer silencer(true);
  Params params;
  std::unordered_map<std::string, EntityMeta *> entities;
  TimeTable timetable(0.5);
  std::vector<RobotMeta *> all_robots;
  initialize_environment(loaded_sequence, options,
                         mix_seed(options.base_random_seed, 0x13579bdu),
                         params, entities, timetable, all_robots, false);

  std::vector<RobotMeta> metas;
  metas.reserve(all_robots.size());
  for (auto *robot : all_robots)
  {
    if (robot)
      metas.push_back(*robot);
  }

  cleanup_entities(entities);
  return metas;
}

AllocationRunSummary make_placeholder_summary(
    const std::string &label,
    const AllocationScenarioPlan &plan)
{
  AllocationRunSummary summary;
  summary.label = label;
  summary.plan = plan;
  summary.parking_seed = 0;
  summary.all_tasks_succeeded = false;
  summary.successful_tasks = 0;
  summary.failed_tasks = 0;
  summary.makespan = std::numeric_limits<double>::infinity();
  return summary;
}

std::string format_feasible_makespan_list(
    const std::vector<SearchTrialRecord> &records)
{
  std::ostringstream oss;
  bool first = true;
  for (const auto &record : records)
  {
    if (!record.feasible || !std::isfinite(record.makespan))
      continue;

    if (!first)
      oss << ", ";
    oss << std::fixed << std::setprecision(2) << record.makespan;
    first = false;
  }

  return first ? "none" : oss.str();
}

void print_search_method_summary(
    const std::string &label,
    bool has_feasible_candidate,
    const AllocationRunSummary &best_feasible,
    bool has_partial_candidate,
    const AllocationRunSummary &best_partial,
    double greedy_makespan)
{
  if (has_feasible_candidate)
  {
    print_comparison_line(best_feasible, greedy_makespan);
    return;
  }

  std::cout << "[Compare] " << label << ": ";
  if (has_partial_candidate)
  {
    std::cout << "no feasible candidate";
    if (best_partial.successful_tasks + best_partial.failed_tasks > 0)
    {
      std::cout << " (best partial " << best_partial.successful_tasks << "/"
                << (best_partial.successful_tasks + best_partial.failed_tasks)
                << " tasks, seed=" << best_partial.parking_seed << ")";
    }
    std::cout << std::endl;
    return;
  }

  std::cout << "no candidates evaluated" << std::endl;
}

// ==========================================
// Initialization Functions (Extracted)
// ==========================================

namespace
{
  double positive_or(double value, double fallback)
  {
    return value > 0.0 ? value : fallback;
  }

  int positive_or(int value, int fallback)
  {
    return value > 0 ? value : fallback;
  }
}

Params initialize_params(const std::vector<FinalAllocation> &loadedSequence,
                         const RuntimeOptions &options)
{
  Params params;

  params.xy_resolution = positive_or(options.default_xy_resolution, params.xy_resolution);
  params.yaw_resolution = positive_or(options.default_yaw_resolution, params.yaw_resolution);
  params.time_step = positive_or(options.default_time_step, params.time_step);
  params.time_resolution = params.time_step;
  params.rs_step_size = positive_or(options.default_rs_step_size, params.rs_step_size);
  params.collision_steps = positive_or(options.default_collision_steps, params.collision_steps);
  params.collision_check_time_step =
      positive_or(options.default_collision_check_time_step, params.collision_check_time_step);
  params.max_steer = positive_or(options.default_max_steer, params.max_steer);
  params.turn_penalty = positive_or(options.default_turn_penalty, params.turn_penalty);
  params.reverse_penalty = positive_or(options.default_reverse_penalty, params.reverse_penalty);
  params.switch_penalty = positive_or(options.default_switch_penalty, params.switch_penalty);
  params.wait_penalty = positive_or(options.default_wait_penalty, params.wait_penalty);
  params.max_time = positive_or(options.default_max_time, params.max_time);
  params.final_push_distance =
      positive_or(options.default_final_push_distance, params.final_push_distance);
  params.inflation = positive_or(options.default_inflation, params.inflation);
  params.safety_margin = std::max(0.0, options.default_safety_margin);
  params.robot_collision_inflation =
      positive_or(options.default_robot_collision_inflation, params.robot_collision_inflation);
  params.safe_parking_expand_max_iterations =
      positive_or(options.default_safe_parking_expand_iterations,
                  params.safe_parking_expand_max_iterations);

  // PHAStar converts this scale to a mode-specific distance using the
  // robot's turning radius when each planner instance is constructed.
  params.analytic_threshold_scale =
      positive_or(options.default_analytic_threshold_scale,
                  params.analytic_threshold_scale);

  // Set workspace boundaries if available
  if (!loadedSequence.empty())
  {
    const auto &bound = loadedSequence[0].snapshot.parameters.boundary;
    params.min_x = bound.xMin;
    params.min_y = bound.yMin;
    params.max_x = bound.xMax;
    params.max_y = bound.yMax;
    std::cout << "[Init] Workspace set: [" << bound.xMin << ", " << bound.xMax
              << "] x [" << bound.yMin << ", " << bound.yMax << "]"
              << std::endl;
  }
  return params;
}

std::unordered_map<std::string, EntityMeta *>
initialize_entities(const std::vector<FinalAllocation> &loadedSequence,
                    int requested_robot_count)
{
  std::unordered_map<std::string, EntityMeta *> entities;

  double common_front_length = 0.36;
  double common_rear_length = 0.12;
  double common_width = 0.275;
  double common_min_turning_radius_transit = 1.02;
  double common_min_turning_radius_transfer = 1.43;
  double common_wheel_base = 0.29;
  double common_speed_transit = 0.2;
  double common_speed_transfer = 0.15;

  struct PredefinedRobot
  {
    const char *name;
    Pose initial_pose;
  };

  const std::vector<PredefinedRobot> predefined_robots = {
      {"robot1", {0.5, 0.45, 0.0}},
      {"robot2", {0.5, 3.0, 0.0}},
      {"robot3", {0.5, 4.5, 0.0}},
      {"robot4", {4.0, 4.05, M_PI}},
  };

  const std::size_t active_robot_count = std::min<std::size_t>(
      predefined_robots.size(),
      static_cast<std::size_t>(std::max(1, requested_robot_count)));

  for (std::size_t i = 0; i < active_robot_count; ++i)
  {
    RobotMeta *robot = new RobotMeta;
    robot->name = predefined_robots[i].name;
    robot->type = EntityType::ROBOT;
    robot->initial_pose = predefined_robots[i].initial_pose;
    robot->size.front_length = common_front_length;
    robot->size.rear_length = common_rear_length;
    robot->size.width = common_width;
    robot->min_turning_radius = common_min_turning_radius_transfer;
    robot->min_turning_radius_transit = common_min_turning_radius_transit;
    robot->min_turning_radius_transfer = common_min_turning_radius_transfer;
    robot->wheel_base = common_wheel_base;
    robot->speed_transit = common_speed_transit;
    robot->speed_transfer = common_speed_transfer;
    entities[robot->name] = robot;
  }

  // Parse Objects
  if (!loadedSequence.empty())
  {
    for (const auto &[name, info] : loadedSequence[0].snapshot.mo_list)
    {
      ObjectMeta *obj = new ObjectMeta;
      obj->name = name;
      obj->type = EntityType::OBJECT;
      obj->initial_pose = {info.x, info.y, info.nominalOrientation};
      // Assuming ObjectMeta size struct is similar
      obj->size.front_length = 0.075;
      obj->size.rear_length = 0.075;
      obj->size.width = 0.15;
      entities[name] = obj;
    }
  }
  return entities;
}

static Pose calcRobotPoseFromObj(const Pose &obj_pose, const OccuRect &robot_size,
                                 const OccuRect &obj_size)
{
  double offset = robot_size.front_length + obj_size.rear_length + 0.1 + 0.05;
  Pose robot_pose;
  robot_pose.x = obj_pose.x - offset * std::cos(obj_pose.yaw);
  robot_pose.y = obj_pose.y - offset * std::sin(obj_pose.yaw);
  robot_pose.yaw = obj_pose.yaw;
  return robot_pose;
}
