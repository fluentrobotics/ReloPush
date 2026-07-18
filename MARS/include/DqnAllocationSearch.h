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
// First-cut scope (matches agreed design), still the v1/v2 action space:
//   - Action space: task order only. Robot assignment is left to the existing
//     greedy 'earliest-available' rule (empty preferred-robot names).
//   - Budget: reuses options.lns_iterations / options.lns_threads.
//   - Model: linear Q(s,a) = w . phi(s,a) (std::vector, no Eigen/torch needed).
// v3 (options.dqn_feature_version >= 3) widens the action space to explicit
// (task, robot) pairs -- see construct_order_v3 below and its design comment
// in DqnAllocationSearch.cpp.
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
// the v2/v3 feature paths (options.dqn_feature_version >= 2) -- for geometric
// extraction speeds and the forward schedule simulation's initial poses. The
// legacy (v1) path never touches it.

// One (phi, target) training sample for a QModel, accumulated into a replay
// buffer and consumed in minibatches by train_model/train_model_logistic
// (DqnAllocationSearch.cpp). Reused for two distinct purposes depending on
// which replay buffer it sits in: in q_replay, `target` is the Monte-Carlo
// return regression target (see ingest_rollout/_v2/_v3); in f_replay
// (decomposed scoring only, see ingest_rollout_v3's decomposed branch),
// `target` instead holds a binary failure label in {0.0, 1.0}. Exposed here
// (rather than kept file-local) so both replay-buffer contents are directly
// unit-testable.
struct Transition
{
  std::vector<double> phi;
  double target = 0.0;
};

// Minibatch SGD regression of the Q model toward stored (Monte-Carlo) returns
// (Huber-clipped error + small L2). For model.hidden == 0 (the default linear
// model) this reproduces the exact arithmetic and rng draw order of the
// original hand-rolled linear update. No-op if `replay` is empty. Exposed
// here (rather than kept file-local) so it is directly unit-testable, and so
// options.dqn_freeze_model's call sites in run_dqn_search are simple
// presence/absence of a call rather than needing an internal early-return
// flag threaded through.
void train_model(
    QModel &model,
    const std::vector<Transition> &replay,
    double learning_rate,
    int grad_steps,
    int minibatch_size,
    std::mt19937 &rng);

// Minibatch SGD of a QModel toward binary failure labels (Decision 2's
// f_head), via the weighted logistic gradient path. Mirrors train_model's
// minibatch-SGD structure exactly (same replay-sampling/apply_grad shape),
// but calls accumulate_grad_logistic instead of accumulate_grad. The
// positive-class weight is recomputed once per call from `f_replay`'s own
// label composition when `fixed_pos_weight` <= 0 (auto: clamp(#neg/#pos, 1,
// 10), guarded against #pos == 0 by defaulting to 1.0); a positive
// `fixed_pos_weight` is used as-is for every sample. Only ever called when
// decomposed scoring is active -- penalty mode never calls this function, so
// it never draws the extra RNG this uses. No-op if `f_replay` is empty.
// Exposed here for the same reasons as train_model above.
void train_model_logistic(
    QModel &model,
    const std::vector<Transition> &f_replay, // .target holds a 0/1 label
    double learning_rate,
    int grad_steps,
    int minibatch_size,
    double fixed_pos_weight,
    std::mt19937 &rng);

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

// ==========================================
// v3: explicit (task, robot) construction (options.dqn_feature_version >= 3).
// ==========================================
//
// Result of one construct_order_v3() rollout: both the task construction
// order AND the per-task robot assignment it built alongside it (unlike v1/v2,
// where robot assignment is left entirely to the executor's greedy
// earliest-available dispatch).
struct TaskOrderAssignment
{
  std::vector<std::size_t> order;      // construction order (original task indices)
  std::vector<std::size_t> assignment; // assignment[original_task_index] = robot index

  // Decomposed-scoring diagnostics (Decision 2, see construct_order_v3's
  // exploit-branch decomposed path). All three stay at their zero default in
  // penalty mode (options.dqn_scoring_mode == 0) and are only incremented
  // once per EXPLOIT step (never explore steps) when decomposed. Aggregated
  // across rollouts by run_dqn_search for the final "fallbacks=K avg_pass=..."
  // summary line.
  int decomposed_exploit_steps = 0;   // # exploit steps considered this rollout
  int decomposed_fallback_steps = 0;  // # of those steps where the p_fail-argmin fallback fired
  double decomposed_survivor_fraction_sum = 0.0; // sum of |survivors|/|candidates| over exploit steps
};

// Constructs one (order, assignment) pair with the v3/v4 epsilon-greedy
// policy: legality reuses is_task_legal_v2 unchanged; candidates are every
// (legal task, robot) pair, scored by model.predict(make_feature_vector_v3/
// _v4(...)) (dispatched internally on options.dqn_feature_version == 4). See
// DqnAllocationSearch.cpp for the full design rationale and the RNG draw
// contract. `step_logs`, when non-null, is sized to task_count and filled
// with every legal (task, robot) candidate's entry at each step (the
// StepCandidateEntry.robot field distinguishes candidates that share a task);
// passing nullptr (the default) is byte-for-byte identical, including the RNG
// draw sequence, to omitting the parameter -- mirrors construct_order_v2's
// contract.
// `f_head`, when non-null AND options.dqn_scoring_mode == 1 (decomposed),
// switches the EXPLOIT branch only (never explore) to Decision 2's
// filter-then-argmax selection: candidates with sigmoid(f_head->predict(phi))
// <= options.dqn_fail_threshold survive, and the argmax of `model` (q_head)
// among survivors is chosen (first-seen tie-break); if no candidate survives,
// falls back to the argmin of p_fail over all candidates (first-seen
// tie-break). Passing nullptr (the default), or dqn_scoring_mode == 0,
// reproduces the original single-model argmax exactly, with no extra RNG
// draws and no extra QModel::predict calls beyond what penalty mode always
// did -- this is the byte-identical invariant penalty mode must keep.
TaskOrderAssignment construct_order_v3(
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
    std::vector<std::vector<StepCandidateEntry>> *step_logs = nullptr,
    const QModel *f_head = nullptr);

// v3 only: extends task_order_signature (AllocationSearch.h) with the
// per-task robot assignment (plan.preferred_robot_names_by_original_task),
// so two v3 candidates that share a task order but differ in robot
// assignment get distinct signatures. Exposed here (rather than kept
// file-local) so it is directly unit-testable.
std::string task_order_assignment_signature(const AllocationScenarioPlan &plan);

// v3 only: assignment[original_task_index] = robot index -> robot NAME per
// original task index. Exposed here (rather than kept file-local) so
// PgTableExport.cpp can reuse it (see DqnAllocationSearch.cpp for the full
// doc comment).
std::vector<std::string> assignment_to_robot_names(
    const std::vector<std::size_t> &assignment,
    const std::vector<std::string> &robot_names);

// v3 only: recovers the per-task robot assignment ACTUALLY executed by a
// (feasible) run summary. Exposed here (rather than kept file-local) so
// PgTableExport.cpp can reuse it for the exported ref_assign field (see
// DqnAllocationSearch.cpp for the full doc comment).
std::vector<std::size_t> assignment_from_summary(
    const AllocationRunSummary &summary,
    const std::vector<std::string> &robot_names,
    std::size_t task_count);

// Replays the forward simulation over an executed (order, assignment) pair
// to recompute phi identically to construction time (same preview_for/
// commit_for sequence per step, no RNG use), turning the rollout into
// per-step training transitions -- same Monte-Carlo broadcast logic as
// ingest_rollout_v2, crediting steps 0..first_fail_rank (see
// compute_rollout_outcome in DqnAllocationSearch.cpp). Feature dispatch
// mirrors construct_order_v3: options.dqn_feature_version == 4 selects
// make_feature_vector_v4 (fed sim.free_time/sim.pose taken before each
// step's commit), otherwise make_feature_vector_v3.
//
// Penalty mode (options.dqn_scoring_mode == 0, the default): behavior is
// exactly the original ingest_rollout_v3 -- every credited step pushes one
// (phi, target) transition to `replay`, target == outcome.return_target
// broadcast to every step (-makespan/seed_makespan if feasible, else
// -fail_return). `f_replay` is ignored.
//
// Decomposed mode (options.dqn_scoring_mode == 1, requires `f_replay` !=
// nullptr): every credited step ALSO pushes (phi, label) to `*f_replay`,
// label = 1.0 iff (the episode is infeasible AND this step == fail_pos),
// else 0.0. `replay` (q_replay) only gains entries from FEASIBLE episodes in
// this mode -- infeasible episodes contribute nothing to `replay` (fail_return
// plays no role in the decomposed path).
//
// Exposed here (rather than kept file-local) so both replay buffers are
// directly unit-testable.
void ingest_rollout_v3(
    const std::vector<std::size_t> &order,
    const std::vector<std::size_t> &assignment, // assignment[original_task_index] = robot index
    const AllocationRunSummary &summary,
    double seed_makespan,
    double fail_return,
    const std::vector<DqnV2::TaskGeom> &geoms,
    const DqnV2::PairwiseGeometry &geometry,
    const LearnedOrderConstraints &constraints,
    const std::vector<RobotMeta> &robot_metas,
    const RuntimeOptions &options,
    std::vector<Transition> &replay,
    std::vector<Transition> *f_replay = nullptr);

// options.dqn_relabel_executed variant of ingest_rollout_v3: replays with
// commit_for against the robot that ACTUALLY EXECUTED each task (recovered
// from `summary.task_rows`), instead of the intended `assignment`, falling
// back to the intended robot for any task with no definite executed robot
// (failing step under early abort, unexecuted tail, or an unresolved robot
// name in task_rows). "Definite executed robot" mirrors
// count_assignment_divergence's row-position -> original-task-index
// correspondence: task_rows[t] corresponds to original task order[t] (NOT
// TaskCsvRow::task_id -- see AssignmentDivergence's doc comment below for why
// row position is the verified-correct correspondence), and only a
// status == "SUCCESS" row counts as definite.
//
// Per-step credit into `replay`/`f_replay` (penalty vs. decomposed mode) is
// otherwise identical to ingest_rollout_v3 -- see its doc comment above --
// except every preview/commit uses the executed-or-intended robot for that
// step's `task` rather than `assignment[task]`.
//
// `relabel_step_log`, when non-null, is sized to order.size() and filled with
// an EXTENDED per-step candidate log: every legal (task, robot) pair at that
// step (same legality rule as construct_order_v3), scored against the
// CORRECTED (relabeled) state -- i.e. NOT the construct-time step log, which
// reflects intended-state predictions. `chosen` is set on the (task, robot)
// pair the policy INTENDED (task order[t], robot assignment[order[t]]), and
// that chosen entry's `executed_robot`/`diverged` fields record the executed
// robot recovered above and whether it differs from the intended one (both
// stay at their StepCandidateEntry defaults, -1/false, on every non-chosen
// entry and on a chosen entry with no definite executed robot). Passing
// nullptr (the default) skips this candidate enumeration entirely (no extra
// QModel::predict calls) -- callers that only need training-buffer ingestion
// (no corpus logging) should pass nullptr.
//
// Exposed here (rather than kept file-local) so it is directly unit-testable
// -- see MARS/tests/phastar_unit_tests.cpp.
void ingest_rollout_v3_relabeled(
    const std::vector<std::size_t> &order,
    const std::vector<std::size_t> &assignment, // INTENDED assignment[original_task_index] = robot index
    const AllocationRunSummary &summary,        // EXECUTED run summary
    double seed_makespan,
    double fail_return,
    const std::vector<DqnV2::TaskGeom> &geoms,
    const DqnV2::PairwiseGeometry &geometry,
    const LearnedOrderConstraints &constraints,
    const std::vector<RobotMeta> &robot_metas,
    const std::vector<std::string> &robot_names,
    const RuntimeOptions &options,
    std::vector<Transition> &replay,
    std::vector<Transition> *f_replay = nullptr,
    std::vector<std::vector<StepCandidateEntry>> *relabel_step_log = nullptr);

// ==========================================
// Diagnostics: assignment divergence (measurement only; v3/v4).
// ==========================================
//
// v3/v4 candidates carry an explicit intended (task, robot) assignment in
// AllocationScenarioPlan.preferred_robot_names_by_original_task, but the
// executor (prepare_task_candidates, TaskExecution.cpp) only treats it as a
// PREFERENCE: the preferred robot is rotated to the front of the candidate
// list and the executor silently falls back to another robot if it fails to
// plan. That means the robot ingest_rollout_v3's phi(task, robot) credits
// can differ from the robot that actually executed the task, which
// mislabels the training signal. AssignmentDivergence/count_assignment_
// divergence measure how often that happens; both are read-only over
// already-computed AllocationScenarioPlan/AllocationRunSummary data -- no
// RNG use, no effect on candidate construction or any result.
struct AssignmentDivergence
{
  // "Placements": task_rows entries with a definite EXECUTED robot, i.e.
  // status == "SUCCESS". diverged_placements counts those whose executed
  // robot (TaskCsvRow::robot_name) differs from the intended one.
  int counted_placements = 0;
  int diverged_placements = 0;
  // Failed rows: task_rows entries with status != "SUCCESS" whose
  // robot_name is non-empty. On a failed row, TaskCsvRow::robot_name is set
  // by execute_single_task_with_candidates (TaskExecution.cpp) to the LAST
  // candidate attempted before giving up -- since the intended/preferred
  // robot is always rotated to the FRONT of the candidate list and tried
  // FIRST, this is almost always some other robot whenever more than one
  // robot was attempted, so diverged_failed is a much weaker/noisier signal
  // than diverged_placements and is kept separate rather than pooled with
  // it.
  int counted_failed = 0;
  int diverged_failed = 0;

  AssignmentDivergence &operator+=(const AssignmentDivergence &other)
  {
    counted_placements += other.counted_placements;
    diverged_placements += other.diverged_placements;
    counted_failed += other.counted_failed;
    diverged_failed += other.diverged_failed;
    return *this;
  }
};

// Compares `intended` (the plan submitted to the evaluator) against
// `executed` (the AllocationRunSummary the evaluator produced for that
// plan), row by row. executed.task_rows[i] corresponds to original task
// index intended.task_order[i]: build_tasks_for_plan (AllocationSearch.cpp)
// constructs the executor's task list by walking plan.task_order in order,
// and execute_task_allocation_loop (TaskExecution.cpp) walks that list 1:1
// to produce task_rows, so row i's original task index is task_order[i].
// Rows are skipped (not counted at all) when: intended.task_order and
// executed.task_rows differ in length (defensive -- should not happen for a
// plan/summary pair that came from the same evaluation); the row's original
// task index has no entry in preferred_robot_names_by_original_task; or
// that entry is empty (no preference was ever recorded for the task, so
// there is nothing to diverge from).
inline AssignmentDivergence count_assignment_divergence(
    const AllocationScenarioPlan &intended,
    const AllocationRunSummary &executed)
{
  AssignmentDivergence result;
  const std::size_t n = executed.task_rows.size();
  if (intended.task_order.size() != n)
    return result;

  for (std::size_t i = 0; i < n; ++i)
  {
    const std::size_t original_task = intended.task_order[i];
    if (original_task >= intended.preferred_robot_names_by_original_task.size())
      continue;
    const std::string &intended_robot =
        intended.preferred_robot_names_by_original_task[original_task];
    if (intended_robot.empty())
      continue;

    const TaskCsvRow &row = executed.task_rows[i];
    if (row.status == "SUCCESS")
    {
      ++result.counted_placements;
      if (row.robot_name != intended_robot)
        ++result.diverged_placements;
    }
    else if (!row.robot_name.empty())
    {
      ++result.counted_failed;
      if (row.robot_name != intended_robot)
        ++result.diverged_failed;
    }
  }
  return result;
}

#endif // DQN_ALLOCATION_SEARCH_H
