/*****************************************************************
 * Search Orchestrator — Strategy Runners for MARS
 * Extracted from PHAstar_push_demo_main
 ******************************************************************/

#include <SearchOrchestrator.h>
#include <AllocationSearch.h>
#include <ResultVisualization.h>
#include <CsvLogging.h>
#include <ReloPushSetup.h>
#include <config.h>

#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <sstream>

namespace
{
std::string instance_record_csv_path(const ReloPush::HandoffInstanceInfo &instance_info)
{
  const std::string filename =
      "mars_instance_record_" +
      sanitize_filename_component(instance_info.file_name) + ".csv";
  if (instance_info.file_name.rfind("ReloPush-BOSS_", 0) == 0)
    return std::string(CMAKE_SOURCE_DIR) + "/results/" + filename;
  return index_log_path(filename);
}

double elapsed_seconds(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now() - start)
      .count();
}

int effective_fine_path_max_search_iterations(const RuntimeOptions &options)
{
  return std::max(options.max_search_iterations,
                  options.fine_segment_max_search_iterations);
}

int effective_safe_parking_max_search_iterations(const RuntimeOptions &options)
{
  return options.safe_parking_max_search_iterations > 0
             ? options.safe_parking_max_search_iterations
             : options.max_search_iterations;
}

std::string lns_mode_label(const RuntimeOptions &options)
{
  return options.lns_reassign_only ? "lns-reassign-only"
                                   : "lns-task-reassign";
}

std::string order_constraint_learning_label(const RuntimeOptions &options)
{
  return options.enable_order_constraint_learning ? "enabled" : "disabled";
}
} // namespace

int run_greedy_only_pipeline(
    int argc, char **argv,
    const RuntimeOptions &options,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const std::vector<FinalAllocation> &loaded_sequence,
    double relopush_single_robot_makespan,
    ReloPush::FinalSequenceHandoffServer *handoff_server)
{
  std::cout << "[Search] No alternative search enabled. Running greedy scenario directly";
  if (options.enable_visualization)
    std::cout << " with visualization";
  else
    std::cout << " headless";
  std::cout << "." << std::endl;

  const std::string greedy_label = "greedy";
  const std::uint32_t greedy_parking_seed = mix_seed(options.base_random_seed, 0xC0FFEE01u);
  AllocationScenarioPlan greedy_plan = make_identity_plan(loaded_sequence.size());

  const auto greedy_start = std::chrono::steady_clock::now();
  auto greedy_executed = execute_allocation_scenario(
      loaded_sequence, options, greedy_plan, greedy_label,
      greedy_parking_seed, true);
  const double greedy_allocation_planning_time_s =
      elapsed_seconds(greedy_start);
  const auto &greedy_summary = greedy_executed.summary;

  std::cout << "[Compare] Makespan summary" << std::endl;
  print_comparison_line(greedy_summary, greedy_summary.makespan);

  std::string greedy_csv = index_log_path("task_execution_log_greedy.csv");
  write_task_csv_log(greedy_csv, greedy_summary.task_rows);

  std::string csv_path = index_log_path("task_execution_log.csv");
  write_task_csv_log(csv_path, greedy_summary.task_rows);
  log_timetable_orientation_diagnostics(greedy_summary.label,
                                        greedy_executed.timetable,
                                        greedy_executed.entities,
                                        greedy_executed.params);

  std::string comparison_csv =
      index_log_path("allocation_search_summary.csv");
  write_allocation_search_summary_csv(comparison_csv, {greedy_summary},
                                      greedy_summary.makespan);

  std::string instance_record_csv = instance_record_csv_path(instance_info);
  const auto robot_names = collect_robot_names(loaded_sequence, options);
  write_instance_run_record_csv(
      instance_record_csv,
      instance_info,
      relopush_single_robot_makespan,
      greedy_summary.makespan,
      std::numeric_limits<double>::infinity(),
      options.lns_iterations,
      0,
      greedy_allocation_planning_time_s,
      {},
      {},
      {},
      options.max_search_iterations,
      effective_fine_path_max_search_iterations(options),
      effective_safe_parking_max_search_iterations(options),
      options.lns_threads,
      static_cast<int>(robot_names.size()),
      lns_mode_label(options),
      order_constraint_learning_label(options),
      greedy_summary.label,
      greedy_summary.makespan);

  export_result_summary_figure(options, instance_info,
                               greedy_summary.label,
                               greedy_executed.timetable,
                               greedy_executed.entities,
                               greedy_executed.params);

  // Notify ReloPush if needed
  if (handoff_server && handoff_server->hasPendingRequest())
  {
    try
    {
      handoff_server->sendReply(ReloPush::makeMarsReply(
          greedy_summary.all_tasks_succeeded,
          greedy_summary.all_tasks_succeeded ? "greedy-planning-complete"
                                             : "greedy-planning-failed"));
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[Integration] Failed to send completion reply to ReloPush: "
                << ex.what() << std::endl;
    }
  }

  if (options.enable_visualization)
  {
    show_results(argc, argv, greedy_executed.timetable, greedy_executed.entities,
                 greedy_executed.params);
  }

  return greedy_summary.all_tasks_succeeded ? 0 : 1;
}

void run_assignment_reassignment_search(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &greedy_plan,
    const AllocationRunSummary &greedy_summary,
    const std::vector<std::string> &robot_names,
    int lns_batch_size,
    std::mt19937 &search_rng,
    AllocationRunSummary &best_feasible,
    AllocationRunSummary &best_partial,
    bool &has_feasible,
    bool &has_partial)
{
  const std::string assignment_label = "fixed-sequence-random-reassign";

  AllocationScenarioPlan greedy_assignment_plan =
      make_assignment_plan_from_greedy(greedy_summary);
  best_feasible = make_placeholder_summary(assignment_label, greedy_assignment_plan);
  best_partial = make_placeholder_summary(assignment_label, greedy_assignment_plan);
  AllocationRunSummary assignment_guiding_summary = greedy_summary;
  assignment_guiding_summary.label = assignment_label;
  assignment_guiding_summary.plan = greedy_assignment_plan;
  AllocationScenarioPlan assignment_current_plan = greedy_assignment_plan;
  has_partial = false;
  has_feasible = false;

  if (options.assignment_search_iterations <= 0 || robot_names.empty())
  {
    std::cout << "[Search] Skipping fixed-sequence reassignment search." << std::endl;
    return;
  }

  std::cout << "[Search] Randomized robot reassignment search (fixed task sequence)..."
            << std::endl;
  for (int iter = 0; iter < options.assignment_search_iterations;)
  {
    AllocationScenarioPlan batch_base_plan = assignment_current_plan;
    std::vector<AssignmentSearchCandidate> batch_candidates;
    std::vector<ScenarioEvaluationRequest> batch_requests;
    batch_candidates.reserve(static_cast<std::size_t>(lns_batch_size));
    batch_requests.reserve(static_cast<std::size_t>(lns_batch_size));

    int batch_end = std::min(options.assignment_search_iterations,
                             iter + lns_batch_size);
    for (; iter < batch_end; ++iter)
    {
      AssignmentSearchCandidate candidate;
      candidate.iteration = iter + 1;
      candidate.plan = make_distinct_assignment_plan(
          batch_base_plan, robot_names, search_rng);
      candidate.parking_seed = search_rng();
      candidate.preference_changes = count_assignment_preference_changes(
          batch_base_plan, candidate.plan);
      candidate.preview_changes = preview_assignment_changes(
          batch_base_plan, candidate.plan, loaded_sequence);

      std::cout << "[Search][Assign " << candidate.iteration << "/"
                << options.assignment_search_iterations << "] "
                << "pref_changes=" << candidate.preference_changes
                << " {" << candidate.preview_changes
                << "}, seed=" << candidate.parking_seed << std::endl;

      batch_requests.push_back(
          {candidate.plan, assignment_label, candidate.parking_seed});
      batch_candidates.push_back(std::move(candidate));
    }

    auto batch_results = evaluate_scenario_batch(
        loaded_sequence, options, batch_requests);

    for (std::size_t batch_idx = 0; batch_idx < batch_candidates.size(); ++batch_idx)
    {
      const auto &candidate_info = batch_candidates[batch_idx];
      auto candidate_summary = batch_results[batch_idx].summary;

      std::cout << "[Search][Assign " << candidate_info.iteration << "] ";
      if (candidate_summary.all_tasks_succeeded)
      {
        std::cout << "makespan=" << std::fixed << std::setprecision(2)
                  << candidate_summary.makespan << "s";
      }
      else
      {
        std::cout << "infeasible (" << candidate_summary.successful_tasks << "/"
                  << (candidate_summary.successful_tasks + candidate_summary.failed_tasks)
                  << " tasks)";
      }
      std::cout << ", execution="
                << (same_execution_signature(candidate_summary, greedy_summary)
                        ? "same-as-greedy"
                        : "changed")
                << std::endl;

      if (!has_partial ||
          is_preferred_search_result(candidate_summary, best_partial))
      {
        best_partial = candidate_summary;
        best_partial.label = assignment_label;
        has_partial = true;
        if (!candidate_summary.all_tasks_succeeded)
        {
          std::cout << "[Search][Assign " << candidate_info.iteration
                    << "] new best partial candidate recorded." << std::endl;
        }
      }

      if (candidate_summary.all_tasks_succeeded &&
          (!has_feasible ||
           is_better_run(candidate_summary, best_feasible)))
      {
        best_feasible = candidate_summary;
        best_feasible.label = assignment_label;
        has_feasible = true;
        std::cout << "[Search][Assign " << candidate_info.iteration
                  << "] new feasible-best candidate recorded." << std::endl;
      }

      if (is_preferred_search_result(candidate_summary, assignment_guiding_summary))
      {
        assignment_guiding_summary = candidate_summary;
        assignment_guiding_summary.label = assignment_label;
        assignment_current_plan = candidate_summary.plan;
      }
    }
  }
}

SequenceSearchOutcome run_sequence_search_method(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationScenarioPlan &greedy_plan,
    const AllocationRunSummary &greedy_summary,
    const std::string &label,
    const std::string &banner,
    const std::string &progress_tag,
    int iterations,
    AllocationScenarioPlan (*mutator)(const AllocationScenarioPlan &, std::mt19937 &),
    int lns_batch_size,
    std::mt19937 &search_rng,
    std::unordered_set<std::string> &tried_signatures,
    std::vector<SearchTrialRecord> &sequence_trial_records,
    const LearnedOrderConstraints &disabled_constraints,
    int *out_enforced_constraint_count)
{
  SequenceSearchOutcome outcome;
  outcome.best_feasible = make_placeholder_summary(label, greedy_plan);
  outcome.best_partial = make_placeholder_summary(label, greedy_plan);
  LearnedOrderConstraints method_constraints =
      make_learned_order_constraints(loaded_sequence.size());
  AllocationRunSummary method_order_learning_reference_summary = greedy_summary;
  auto maybe_update_method_order_learning_reference =
      [&](const AllocationRunSummary &summary)
  {
    if (summary.all_tasks_succeeded &&
        is_better_run(summary, method_order_learning_reference_summary))
    {
      method_order_learning_reference_summary = summary;
    }
  };

  if (iterations <= 0 || loaded_sequence.size() <= 1)
  {
    if (out_enforced_constraint_count)
      *out_enforced_constraint_count = 0;
    std::cout << "[Search] Skipping " << banner << "." << std::endl;
    return outcome;
  }

  std::cout << "[Search] " << banner << "..." << std::endl;
  for (int iter = 0; iter < iterations;)
  {
    std::vector<SequenceSearchCandidate> batch_candidates;
    std::vector<ScenarioEvaluationRequest> batch_requests;
    batch_candidates.reserve(static_cast<std::size_t>(lns_batch_size));
    batch_requests.reserve(static_cast<std::size_t>(lns_batch_size));

    int batch_end = std::min(iterations, iter + lns_batch_size);
    for (; iter < batch_end; ++iter)
    {
      AllocationScenarioPlan candidate_plan;
      if (!sample_unique_sequence_plan(greedy_plan, mutator, search_rng,
                                       tried_signatures,
                                       options.enable_order_constraint_learning
                                           ? &method_constraints
                                           : nullptr,
                                       &candidate_plan))
      {
        std::cout << "[Search][" << progress_tag
                  << "] Unable to sample a new unseen order after repeated attempts. "
                  << "Stopping early at iteration " << (iter + 1) << "."
                  << std::endl;
        iter = iterations;
        break;
      }

      SequenceSearchCandidate candidate;
      candidate.iteration = iter + 1;
      candidate.plan = candidate_plan;
      candidate.parking_seed = search_rng();
      candidate.position_changes = count_task_position_changes(
          greedy_plan, candidate.plan);
      candidate.preview_order = preview_task_order(candidate.plan, loaded_sequence);

      std::cout << "[Search][" << progress_tag << " " << candidate.iteration << "/"
                << iterations << "] "
                << "position_changes=" << candidate.position_changes
                << ", order={" << candidate.preview_order
                << "}, seed=" << candidate.parking_seed << std::endl;

      batch_requests.push_back(
          {candidate.plan, label, candidate.parking_seed});
      batch_candidates.push_back(std::move(candidate));
    }

    auto batch_results = evaluate_scenario_batch(
        loaded_sequence, options, batch_requests);

    for (std::size_t batch_idx = 0; batch_idx < batch_candidates.size(); ++batch_idx)
    {
      const auto &candidate_info = batch_candidates[batch_idx];
      auto candidate_summary = batch_results[batch_idx].summary;

      std::cout << "[Search][" << progress_tag << " " << candidate_info.iteration
                << "] ";
      if (candidate_summary.all_tasks_succeeded)
      {
        std::cout << "makespan=" << std::fixed << std::setprecision(2)
                  << candidate_summary.makespan << "s";
      }
      else
      {
        std::cout << "infeasible (" << candidate_summary.successful_tasks << "/"
                  << (candidate_summary.successful_tasks + candidate_summary.failed_tasks)
                  << " tasks)";
      }
      std::cout << ", execution="
                << (same_execution_signature(candidate_summary, greedy_summary)
                        ? "same-as-greedy"
                        : "changed")
                << std::endl;

      SearchTrialRecord record;
      record.search_label = label;
      record.iteration = candidate_info.iteration;
      record.parking_seed = candidate_info.parking_seed;
      record.feasible = candidate_summary.all_tasks_succeeded;
      record.makespan = candidate_summary.makespan;
      record.successful_tasks = candidate_summary.successful_tasks;
      record.failed_tasks = candidate_summary.failed_tasks;
      record.order_description =
          preview_task_order(candidate_info.plan, loaded_sequence, loaded_sequence.size());
      sequence_trial_records.push_back(std::move(record));

      bool learned_from_failure = false;
      if (!outcome.has_partial ||
          is_preferred_search_result(candidate_summary, outcome.best_partial))
      {
        outcome.best_partial = candidate_summary;
        outcome.best_partial.label = label;
        outcome.has_partial = true;
        if (options.enable_order_constraint_learning &&
            !candidate_summary.all_tasks_succeeded)
        {
          learn_order_constraints_from_failed_summary(
              candidate_summary, method_order_learning_reference_summary.plan,
              loaded_sequence, method_constraints);
          learned_from_failure = true;
          std::cout << "[Search][" << progress_tag << " " << candidate_info.iteration
                    << "] new best partial candidate recorded." << std::endl;
        }
      }

      if (candidate_summary.all_tasks_succeeded &&
          (!outcome.has_feasible ||
           is_better_run(candidate_summary, outcome.best_feasible)))
      {
        outcome.best_feasible = candidate_summary;
        outcome.best_feasible.label = label;
        outcome.has_feasible = true;
        maybe_update_method_order_learning_reference(candidate_summary);
        std::cout << "[Search][" << progress_tag << " " << candidate_info.iteration
                  << "] new feasible-best candidate recorded." << std::endl;
      }
      else if (options.enable_order_constraint_learning &&
               !candidate_summary.all_tasks_succeeded && !learned_from_failure)
      {
        learn_order_constraints_from_failed_summary(
            candidate_summary, method_order_learning_reference_summary.plan,
            loaded_sequence, method_constraints);
      }
    }
  }

  int feasible_count = static_cast<int>(
      std::count_if(sequence_trial_records.begin(), sequence_trial_records.end(),
                    [&](const SearchTrialRecord &record)
                    { return record.search_label == label && record.feasible; }));
  int total_count = static_cast<int>(
      std::count_if(sequence_trial_records.begin(), sequence_trial_records.end(),
                    [&](const SearchTrialRecord &record)
                    { return record.search_label == label; }));

  std::vector<SearchTrialRecord> method_records;
  method_records.reserve(static_cast<std::size_t>(total_count));
  for (const auto &record : sequence_trial_records)
  {
    if (record.search_label == label)
      method_records.push_back(record);
  }

  std::cout << "[Search][" << progress_tag << "] Feasible shuffled plans: "
            << feasible_count << "/" << total_count
            << ", makespans={" << format_feasible_makespan_list(method_records)
            << "}" << std::endl;
  if (out_enforced_constraint_count)
    *out_enforced_constraint_count = method_constraints.enforced_count;

  return outcome;
}

SequenceSearchOutcome run_adaptive_lns_search(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationRunSummary &lns_seed_summary,
    const AllocationScenarioPlan &greedy_plan,
    const std::vector<std::string> &robot_names,
    int lns_batch_size,
    std::mt19937 &lns_rng,
    const LearnedOrderConstraints &disabled_constraints,
    int *out_enforced_constraint_count)
{
  const std::string lns_label = "lns-adaptive";

  SequenceSearchOutcome lns_outcome;
  lns_outcome.best_feasible = make_placeholder_summary(lns_label, greedy_plan);
  lns_outcome.best_partial = make_placeholder_summary(lns_label, greedy_plan);

  if (options.lns_iterations <= 0 || !lns_seed_summary.all_tasks_succeeded ||
      loaded_sequence.size() <= 1 || robot_names.empty())
  {
    if (out_enforced_constraint_count)
      *out_enforced_constraint_count = 0;
    std::cout << "[Search] Skipping adaptive LNS." << std::endl;
    return lns_outcome;
  }

  std::cout << "[Search] Adaptive LNS from "
            << lns_seed_summary.label;
  if (options.lns_reassign_only)
  {
    std::cout << " (fixed task sequence, robot reassignment only)";
  }
  std::cout << "..." << std::endl;

  AllocationRunSummary lns_current = lns_seed_summary;
  lns_current.label = lns_label;
  lns_outcome.best_feasible = lns_current;
  lns_outcome.best_partial = lns_current;
  lns_outcome.has_feasible = true;
  lns_outcome.has_partial = true;
  LearnedOrderConstraints lns_learned_order_constraints =
      make_learned_order_constraints(loaded_sequence.size());
  AllocationRunSummary lns_order_learning_reference_summary = lns_current;
  auto maybe_update_lns_order_learning_reference =
      [&](const AllocationRunSummary &summary)
  {
    if (summary.all_tasks_succeeded &&
        is_better_run(summary, lns_order_learning_reference_summary))
    {
      lns_order_learning_reference_summary = summary;
    }
  };

  double destroy_fraction = 0.10;
  int no_improve_iterations = 0;

  for (int iter = 0; iter < options.lns_iterations;)
  {
    AllocationRunSummary batch_base_summary = lns_current;
    AllocationScenarioPlan batch_base_plan = lns_current.plan;
    const LearnedOrderConstraints &batch_constraints =
        (options.enable_order_constraint_learning &&
         !options.lns_reassign_only)
            ? lns_learned_order_constraints
            : disabled_constraints;
    auto batch_destroy_scores =
        compute_task_destroy_scores(batch_base_summary, loaded_sequence);

    std::vector<LnsSearchCandidate> batch_candidates;
    batch_candidates.reserve(static_cast<std::size_t>(lns_batch_size));

    int batch_end = std::min(options.lns_iterations,
                             iter + lns_batch_size);
    for (; iter < batch_end; ++iter)
    {
      LnsSearchCandidate candidate_info;
      candidate_info.iteration = iter + 1;
      candidate_info.destroy_fraction = destroy_fraction;
      candidate_info.diversification = ((candidate_info.iteration % 50) == 0);

      std::size_t destroy_count = static_cast<std::size_t>(std::round(
          loaded_sequence.size() *
          (candidate_info.diversification ? 0.60 : destroy_fraction)));
      destroy_count = std::max<std::size_t>(1, destroy_count);
      destroy_count = std::min<std::size_t>(destroy_count, loaded_sequence.size() - 1);

      candidate_info.destroy_scores = batch_destroy_scores;
      if (candidate_info.diversification)
      {
        candidate_info.destroy_operator = "random-large";
        candidate_info.destroyed_tasks = destroy_random_tasks(
            loaded_sequence.size(), destroy_count, lns_rng);
      }
      else
      {
        std::uniform_real_distribution<double> op_pick(0.0, 1.0);
        double picked = op_pick(lns_rng);
        if (picked < 0.55)
        {
          candidate_info.destroy_operator = "wait-bottleneck";
          candidate_info.destroyed_tasks = destroy_wait_neighborhood_tasks(
              batch_base_plan, batch_base_summary, loaded_sequence,
              destroy_count, lns_rng);
        }
        else if (picked < 0.85)
        {
          candidate_info.destroy_operator = "critical-tail";
          candidate_info.destroyed_tasks = destroy_critical_suffix_tasks(
              batch_base_summary, loaded_sequence, destroy_count, lns_rng);
        }
        else
        {
          candidate_info.destroy_operator = "random";
          candidate_info.destroyed_tasks = destroy_random_tasks(
              loaded_sequence.size(), destroy_count, lns_rng);
        }
      }

      candidate_info.parking_seed = lns_rng();
      candidate_info.repair_seed = lns_rng();

      std::cout << "[LNS " << candidate_info.iteration << "/"
                << options.lns_iterations << "] "
                << "op=" << candidate_info.destroy_operator
                << " k=" << candidate_info.destroyed_tasks.size()
                << " seed=" << candidate_info.parking_seed;
      if (options.lns_threads > 1)
      {
        std::cout << " queued";
      }
      std::cout << std::endl;

      batch_candidates.push_back(std::move(candidate_info));
    }

    const auto lns_batch_start = std::chrono::steady_clock::now();
    auto batch_results = evaluate_lns_batch(
        loaded_sequence, options, batch_base_plan, batch_base_summary,
        batch_constraints, batch_candidates, robot_names, lns_label);
    lns_outcome.lns_batch_planning_times_s.push_back(
        elapsed_seconds(lns_batch_start));

    int batch_failed_iterations = 0;
    for (std::size_t batch_idx = 0; batch_idx < batch_candidates.size(); ++batch_idx)
    {
      const auto &candidate_info = batch_candidates[batch_idx];
      AllocationRunSummary candidate = batch_results[batch_idx].summary;
      candidate.label = lns_label;
      if (!candidate.all_tasks_succeeded)
      {
        ++lns_outcome.lns_failed_iterations;
        ++batch_failed_iterations;
      }

      if (options.enable_order_constraint_learning &&
          !options.lns_reassign_only &&
          !candidate.all_tasks_succeeded)
      {
        learn_order_constraints_from_failed_summary(
            candidate, lns_order_learning_reference_summary.plan,
            loaded_sequence, lns_learned_order_constraints);
      }

      if (!lns_outcome.has_partial ||
          is_preferred_search_result(candidate, lns_outcome.best_partial))
      {
        lns_outcome.best_partial = candidate;
        lns_outcome.best_partial.label = lns_label;
        lns_outcome.has_partial = true;
      }

      bool improved_global = false;
      if (candidate.all_tasks_succeeded &&
          (!lns_outcome.has_feasible ||
           is_better_run(candidate, lns_outcome.best_feasible)))
      {
        lns_outcome.best_feasible = candidate;
        lns_outcome.best_feasible.label = lns_label;
        if (batch_results[batch_idx].executed)
        {
          batch_results[batch_idx].executed->summary = lns_outcome.best_feasible;
          lns_outcome.best_feasible_execution =
              std::move(batch_results[batch_idx].executed);
        }
        lns_outcome.has_feasible = true;
        improved_global = true;
        maybe_update_lns_order_learning_reference(candidate);
      }

      bool accepted_current = false;
      std::string accept_reason = "reject";
      if (is_preferred_search_result(candidate, lns_current))
      {
        lns_current = candidate;
        lns_current.label = lns_label;
        accepted_current = true;
        accept_reason = improved_global ? "improved" : "accept";
      }
      else if (candidate.all_tasks_succeeded && lns_current.all_tasks_succeeded)
      {
        std::uniform_real_distribution<double> walk_accept_dist(0.0, 1.0);
        if (walk_accept_dist(lns_rng) < 0.10)
        {
          lns_current = candidate;
          lns_current.label = lns_label;
          accepted_current = true;
          accept_reason = "walk";
        }
      }

      if (improved_global)
      {
        destroy_fraction = 0.10;
        no_improve_iterations = 0;
      }
      else
      {
        ++no_improve_iterations;
        if (no_improve_iterations % 30 == 0)
        {
          destroy_fraction = std::min(0.50, destroy_fraction + 0.05);
        }
      }

      std::cout << "[LNS " << candidate_info.iteration << "/"
                << options.lns_iterations << "] "
                << "op=" << candidate_info.destroy_operator
                << " k=" << candidate_info.destroyed_tasks.size()
                << " feas=" << (candidate.all_tasks_succeeded ? 1 : 0);
      if (candidate.all_tasks_succeeded)
      {
        std::cout << " mk=" << std::fixed << std::setprecision(2)
                  << candidate.makespan << "s";
      }
      else
      {
        std::cout << " suc=" << candidate.successful_tasks << "/"
                  << (candidate.successful_tasks + candidate.failed_tasks);
      }

      if (lns_outcome.has_feasible)
      {
        std::cout << " best=" << std::fixed << std::setprecision(2)
                  << lns_outcome.best_feasible.makespan << "s";
      }
      else
      {
        std::cout << " best=none";
      }

      std::cout << " acc=" << accept_reason;
      if (accepted_current && accept_reason == "walk")
      {
        std::cout << " cur=" << std::fixed << std::setprecision(2)
                  << lns_current.makespan << "s";
      }
      if (options.lns_threads > 1)
      {
        std::cout << " batch=" << batch_candidates.size();
      }
      std::cout << std::endl;
    }
    lns_outcome.lns_batch_failed_iterations.push_back(batch_failed_iterations);
    lns_outcome.lns_batch_best_makespans.push_back(
        lns_outcome.has_feasible
            ? lns_outcome.best_feasible.makespan
            : std::numeric_limits<double>::infinity());
  }

  if (out_enforced_constraint_count)
    *out_enforced_constraint_count = lns_learned_order_constraints.enforced_count;

  return lns_outcome;
}

int finalize_and_replay_best(
    int argc, char **argv,
    const RuntimeOptions &options,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const std::vector<FinalAllocation> &loaded_sequence,
    const std::vector<AllocationRunSummary> &summaries,
    const AllocationRunSummary &best_summary,
    const ExecutedScenario *cached_best_executed,
    double greedy_allocation_planning_time_s,
    const std::vector<double> &lns_batch_planning_times_s,
    const std::vector<double> &lns_batch_best_makespans,
    const std::vector<int> &lns_batch_failed_iterations,
    int lns_failed_iterations,
    int robot_count,
    double relopush_single_robot_makespan,
    double greedy_makespan,
    double lns_best_makespan,
    const std::vector<SearchTrialRecord> &trial_records,
    ReloPush::FinalSequenceHandoffServer *handoff_server)
{
  for (const auto &summary : summaries)
  {
    std::string scenario_csv = index_log_path(
        "task_execution_log_" +
        sanitize_filename_component(summary.label) + ".csv");
    write_task_csv_log(scenario_csv, summary.task_rows);
  }

  std::string comparison_csv =
      index_log_path("allocation_search_summary.csv");
  write_allocation_search_summary_csv(comparison_csv, summaries, greedy_makespan);

  if (!trial_records.empty())
  {
    std::string trial_csv =
        index_log_path("task_shuffle_trial_log.csv");
    write_search_trial_records_csv(trial_csv, trial_records);
  }

  std::string instance_record_csv = instance_record_csv_path(instance_info);
  write_instance_run_record_csv(
      instance_record_csv,
      instance_info,
      relopush_single_robot_makespan,
      greedy_makespan,
      lns_best_makespan,
      options.lns_iterations,
      lns_failed_iterations,
      greedy_allocation_planning_time_s,
      lns_batch_planning_times_s,
      lns_batch_best_makespans,
      lns_batch_failed_iterations,
      options.max_search_iterations,
      effective_fine_path_max_search_iterations(options),
      effective_safe_parking_max_search_iterations(options),
      options.lns_threads,
      robot_count,
      lns_mode_label(options),
      order_constraint_learning_label(options),
      best_summary.label,
      best_summary.makespan);

  std::cout << "[Final] "
            << (cached_best_executed ? "Using cached best scenario: "
                                     : "Replaying best scenario: ")
            << best_summary.label;
  if (options.enable_visualization)
    std::cout << " (with visualization)";
  else
    std::cout << " (headless)";
  std::cout << std::endl;

  std::unique_ptr<ExecutedScenario> replayed_best_executed;
  const ExecutedScenario *best_executed = cached_best_executed;
  if (!best_executed)
  {
    replayed_best_executed = std::make_unique<ExecutedScenario>(
        execute_allocation_scenario(
            loaded_sequence, options, best_summary.plan,
            best_summary.label, best_summary.parking_seed, true));
    best_executed = replayed_best_executed.get();
  }

  std::string csv_path = index_log_path("task_execution_log.csv");
  write_task_csv_log(csv_path, best_executed->summary.task_rows);
  log_timetable_orientation_diagnostics(best_executed->summary.label,
                                        best_executed->timetable,
                                        best_executed->entities,
                                        best_executed->params);

  export_result_summary_figure(options, instance_info,
                               best_executed->summary.label,
                               best_executed->timetable,
                               best_executed->entities,
                               best_executed->params);

  // Notify ReloPush if needed
  if (handoff_server && handoff_server->hasPendingRequest())
  {
    try
    {
      handoff_server->sendReply(ReloPush::makeMarsReply(
          best_executed->summary.all_tasks_succeeded,
          best_executed->summary.all_tasks_succeeded ? "best-scenario-complete"
                                                     : "best-scenario-failed"));
    }
    catch (const std::exception &ex)
    {
      std::cerr << "[Integration] Failed to send completion reply to ReloPush: "
                << ex.what() << std::endl;
    }
  }

  if (options.enable_visualization)
  {
    show_results(argc, argv, best_executed->timetable, best_executed->entities,
                 best_executed->params);
  }

  return best_executed->summary.all_tasks_succeeded ? 0 : 1;
}
