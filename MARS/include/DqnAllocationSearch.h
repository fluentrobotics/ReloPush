#ifndef DQN_ALLOCATION_SEARCH_H
#define DQN_ALLOCATION_SEARCH_H

#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <ReloPush/TaskAllocation.hpp>

#include <random>
#include <string>
#include <vector>

// ==========================================
// Per-instance online Q-learning allocator
// ==========================================
//
// Drop-in alternative to run_adaptive_lns_search. Instead of destroy/repair,
// it *constructs* a task insertion order step-by-step with an epsilon-greedy
// linear Q-policy, evaluates the constructed plans with the same scenario
// evaluator the LNS path uses, and regresses the Q-function toward the
// (Monte-Carlo) return observed from each rollout.
//
// First-cut scope (matches agreed design):
//   - Action space: task order only. Robot assignment is left to the existing
//     greedy 'earliest-available' rule (empty preferred-robot names).
//   - Budget: reuses options.lns_iterations / options.lns_threads.
//   - Model: linear Q(s,a) = w . phi(s,a) (std::vector, no Eigen/torch needed).
//
// Because the planning reward is essentially terminal (feasible -> -makespan,
// infeasible -> large penalty credited up to the first failed task), the
// Bellman target reduces to the Monte-Carlo return, so we regress directly
// onto that return rather than running 1-step TD with a target network. This
// is the most sample-efficient choice for the tiny per-instance budget.
//
// Returns the same SequenceSearchOutcome type as run_adaptive_lns_search so the
// orchestrator consumes it identically.
SequenceSearchOutcome run_dqn_search(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationRunSummary &seed_summary,
    const AllocationScenarioPlan &greedy_plan,
    const std::vector<std::string> &robot_names,
    int batch_size,
    std::mt19937 &rng,
    const LearnedOrderConstraints &disabled_constraints,
    int *out_enforced_constraint_count);

#endif // DQN_ALLOCATION_SEARCH_H
