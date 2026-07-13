#ifndef DQN_ALLOCATION_SEARCH_H
#define DQN_ALLOCATION_SEARCH_H

#include <PHAstarPushDemoTypes.h>
#include <PHAstarPushDemoOptions.h>
#include <ReloPush/TaskAllocation.hpp>
#include <PHAstar/Entities.h>
#include <DqnQModel.h>
#include <DqnFeaturesV2.h>
#include <TransitionLogger.h>

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
// `robot_metas` (one entry per robot, homogeneous fleet) is only consulted by
// the v2 feature path (options.dqn_feature_version >= 2) -- for geometric
// extraction speeds and the forward schedule simulation's initial poses. The
// legacy (v1) path never touches it.
SequenceSearchOutcome run_dqn_search(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationRunSummary &seed_summary,
    const AllocationScenarioPlan &greedy_plan,
    const std::vector<std::string> &robot_names,
    const std::vector<RobotMeta> &robot_metas,
    int batch_size,
    std::mt19937 &rng,
    const LearnedOrderConstraints &disabled_constraints,
    int *out_enforced_constraint_count);

// Constructs one task order with the v2 epsilon-greedy policy (see
// DqnAllocationSearch.cpp for the full design rationale). Exposed here
// (rather than kept file-local) so it is directly unit-testable -- see
// MARS/tests/phastar_unit_tests.cpp. `step_logs`, when non-null, is sized to
// task_count and filled with every legal candidate's (task, phi, chosen)
// entry at each step; passing nullptr (the default) is byte-for-byte
// identical, including the RNG draw sequence, to omitting the parameter.
std::vector<std::size_t> construct_order_v2(
    const QModel &model,
    std::size_t task_count,
    const std::vector<DqnV2::TaskGeom> &geoms,
    const DqnV2::PairwiseGeometry &geometry,
    const LearnedOrderConstraints &constraints,
    const std::vector<RobotMeta> &robot_metas,
    double seed_makespan,
    double epsilon,
    const RuntimeOptions &options,
    std::mt19937 &rng,
    std::vector<std::vector<StepCandidateEntry>> *step_logs = nullptr);

#endif // DQN_ALLOCATION_SEARCH_H
