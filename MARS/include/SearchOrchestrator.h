#ifndef SEARCH_ORCHESTRATOR_H
#define SEARCH_ORCHESTRATOR_H

#include <PHAstarPushDemoOptions.h>
#include <PHAstarPushDemoTypes.h>
#include <ReloPush/TaskAllocation.hpp>
#include <ReloPush/FinalSequenceHandoff.h>
#include <vector>
#include <string>
#include <memory>

// Single-strategy fast path (no search enabled)
int run_greedy_only_pipeline(
    int argc, char **argv,
    const RuntimeOptions &options,
    const ReloPush::HandoffInstanceInfo &instance_info,
    const std::vector<FinalAllocation> &loaded_sequence,
    double relopush_single_robot_makespan,
    ReloPush::FinalSequenceHandoffServer *handoff_server);

// Per-strategy search runners
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
    bool &has_partial);

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
    int *out_enforced_constraint_count);

SequenceSearchOutcome run_adaptive_lns_search(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationRunSummary &lns_seed_summary,
    const AllocationScenarioPlan &greedy_plan,
    const std::vector<std::string> &robot_names,
    int lns_batch_size,
    std::mt19937 &lns_rng,
    const LearnedOrderConstraints &disabled_constraints,
    int *out_enforced_constraint_count);

// Final CSVs + summary figure + optional replay + handoff reply
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
    double relopush_single_robot_makespan,
    double greedy_makespan,
    double lns_best_makespan,
    const std::vector<SearchTrialRecord> &trial_records,
    ReloPush::FinalSequenceHandoffServer *handoff_server);

#endif // SEARCH_ORCHESTRATOR_H
