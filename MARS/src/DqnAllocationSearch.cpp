/*****************************************************************
 * Per-instance online Q-learning task allocator.
 *
 * Constructs task insertion orders with an epsilon-greedy linear Q-policy,
 * evaluates them with evaluate_scenario_batch (the same env step LNS uses),
 * and regresses the linear Q-function toward the Monte-Carlo return of each
 * rollout. See DqnAllocationSearch.h for the design rationale.
 ******************************************************************/

#include <DqnAllocationSearch.h>
#include <DqnQModel.h>
#include <DqnFeaturesV2.h>
#include <TransitionLogger.h>
#include <AllocationSearch.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
constexpr const char *kDqnLabel = "dqn-online";

// Feature layout for phi(state, candidate task). Keep small for the tiny
// per-instance sample budget; the retrospective return is the real teacher.
enum FeatureIndex
{
  kFeatBias = 0,
  kFeatCost,        // task intrinsic cost (single-robot reference)
  kFeatObsRelo,     // # obstacle-relocation sub-paths (difficulty proxy)
  kFeatNumPaths,    // # path segments (difficulty proxy)
  kFeatPosFrac,     // placement position / (N-1)
  kFeatOrigDev,     // |reference rank - placement position| / N
  kFeatMustPrecede, // # remaining tasks this task must precede / N
  kFeatDim
};

// Normalized scales for the static per-task features.
struct StaticTaskFeatures
{
  double cost = 0.0;
  double obs_relo = 0.0;
  double num_paths = 0.0;
};

double safe_div(double a, double b)
{
  return b > 1e-9 ? a / b : 0.0;
}

double elapsed_seconds(const std::chrono::steady_clock::time_point &start)
{
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now() - start)
      .count();
}

std::vector<StaticTaskFeatures> build_static_features(
    const std::vector<FinalAllocation> &loaded_sequence)
{
  const std::size_t n = loaded_sequence.size();
  std::vector<StaticTaskFeatures> feats(n);
  double max_cost = 0.0, max_obs = 0.0, max_paths = 0.0;

  for (std::size_t i = 0; i < n; ++i)
  {
    const FinalAllocation &fa = loaded_sequence[i];
    feats[i].cost = fa.cost;
    feats[i].obs_relo =
        fa.obsReloPaths ? static_cast<double>(fa.obsReloPaths->size()) : 0.0;
    feats[i].num_paths = static_cast<double>(fa.paths.size());

    max_cost = std::max(max_cost, feats[i].cost);
    max_obs = std::max(max_obs, feats[i].obs_relo);
    max_paths = std::max(max_paths, feats[i].num_paths);
  }

  for (auto &f : feats)
  {
    f.cost = safe_div(f.cost, max_cost);
    f.obs_relo = safe_div(f.obs_relo, max_obs);
    f.num_paths = safe_div(f.num_paths, max_paths);
  }
  return feats;
}

// True if an enforced predecessor of `task` is still unplaced (so `task` is not
// yet legal to insert under the learned precedence constraints).
bool has_unplaced_predecessor(
    std::size_t task,
    const std::vector<char> &is_placed,
    const LearnedOrderConstraints &constraints)
{
  const std::size_t n = is_placed.size();
  if (constraints.enforced.size() != n)
    return false;
  for (std::size_t p = 0; p < n; ++p)
  {
    if (p == task || is_placed[p])
      continue;
    if (constraints.enforced[p].size() == n && constraints.enforced[p][task])
      return true;
  }
  return false;
}

std::size_t count_must_precede(
    std::size_t task,
    const std::vector<char> &is_placed,
    const LearnedOrderConstraints &constraints)
{
  const std::size_t n = is_placed.size();
  if (constraints.enforced.size() != n || constraints.enforced[task].size() != n)
    return 0;
  std::size_t count = 0;
  for (std::size_t k = 0; k < n; ++k)
  {
    if (k == task || is_placed[k])
      continue;
    if (constraints.enforced[task][k])
      ++count;
  }
  return count;
}

std::vector<double> make_feature_vector(
    std::size_t task,
    std::size_t placement_position,
    std::size_t task_count,
    const std::vector<char> &is_placed,
    const std::vector<StaticTaskFeatures> &static_feats,
    const LearnedOrderConstraints &constraints)
{
  std::vector<double> phi(kFeatDim, 0.0);
  phi[kFeatBias] = 1.0;
  phi[kFeatCost] = static_feats[task].cost;
  phi[kFeatObsRelo] = static_feats[task].obs_relo;
  phi[kFeatNumPaths] = static_feats[task].num_paths;

  const double denom_pos = task_count > 1 ? static_cast<double>(task_count - 1) : 1.0;
  phi[kFeatPosFrac] = static_cast<double>(placement_position) / denom_pos;
  phi[kFeatOrigDev] =
      safe_div(std::abs(static_cast<double>(task) -
                        static_cast<double>(placement_position)),
               static_cast<double>(task_count));
  phi[kFeatMustPrecede] =
      safe_div(static_cast<double>(count_must_precede(task, is_placed, constraints)),
               static_cast<double>(task_count));
  return phi;
}

// Transition (phi + regression target/label) is declared in
// DqnAllocationSearch.h -- exposed there (rather than kept file-local) so
// both q_replay and f_replay contents are directly unit-testable.

// Construct one task order with an epsilon-greedy linear Q-policy. Candidates
// that would violate enforced precedence are masked out.
std::vector<std::size_t> construct_order(
    const QModel &model,
    std::size_t task_count,
    const std::vector<StaticTaskFeatures> &static_feats,
    const LearnedOrderConstraints &constraints,
    double epsilon,
    std::mt19937 &rng)
{
  std::vector<std::size_t> order;
  order.reserve(task_count);
  std::vector<char> is_placed(task_count, 0);
  std::uniform_real_distribution<double> coin(0.0, 1.0);

  for (std::size_t step = 0; step < task_count; ++step)
  {
    std::vector<std::size_t> legal;
    legal.reserve(task_count - step);
    for (std::size_t i = 0; i < task_count; ++i)
    {
      if (is_placed[i])
        continue;
      if (!has_unplaced_predecessor(i, is_placed, constraints))
        legal.push_back(i);
    }
    if (legal.empty())
    {
      // Degenerate (should not happen with an acyclic constraint graph):
      // fall back to any remaining task.
      for (std::size_t i = 0; i < task_count; ++i)
        if (!is_placed[i])
          legal.push_back(i);
    }

    std::size_t chosen = legal.front();
    if (coin(rng) < epsilon)
    {
      std::uniform_int_distribution<std::size_t> pick(0, legal.size() - 1);
      chosen = legal[pick(rng)];
    }
    else
    {
      double best_q = -std::numeric_limits<double>::infinity();
      for (std::size_t i : legal)
      {
        auto phi = make_feature_vector(i, step, task_count, is_placed,
                                       static_feats, constraints);
        double q = model.predict(phi);
        if (q > best_q)
        {
          best_q = q;
          chosen = i;
        }
      }
    }

    order.push_back(chosen);
    is_placed[chosen] = 1;
  }
  return order;
}

// Turn one evaluated rollout into per-step training transitions using the
// retrospective (real) outcome. Feasible -> -makespan/seed for every executed
// step; infeasible -> a fixed penalty credited up to the first failed task.
void ingest_rollout(
    const std::vector<std::size_t> &order,
    const AllocationRunSummary &summary,
    double seed_makespan,
    double fail_return,
    const std::vector<StaticTaskFeatures> &static_feats,
    const LearnedOrderConstraints &constraints,
    std::vector<Transition> &replay)
{
  const std::size_t n = order.size();
  if (n == 0)
    return;

  const bool feasible = summary.all_tasks_succeeded;
  std::size_t fail_pos = n; // inclusive upper bound of executed steps
  if (!feasible)
  {
    fail_pos = 0;
    for (std::size_t t = 0; t < summary.task_rows.size(); ++t)
    {
      if (summary.task_rows[t].status != "SUCCESS")
      {
        fail_pos = t;
        break;
      }
    }
  }

  const double target =
      feasible ? -safe_div(summary.makespan, seed_makespan) : -fail_return;

  std::vector<char> is_placed(n, 0);
  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    // Only learn from steps that actually executed (all of them when feasible,
    // up to and including the first failure otherwise).
    if (feasible || t <= fail_pos)
    {
      Transition tr;
      tr.phi = make_feature_vector(task, t, n, is_placed, static_feats, constraints);
      tr.target = target;
      replay.push_back(std::move(tr));
    }
    is_placed[task] = 1;
  }
}

// ==========================================
// v2 feature path (options.dqn_feature_version >= 2). See
// MARS/08dqn-feature-redesign.md and DqnFeaturesV2.h. Kept entirely separate
// from the v1 functions above so the legacy path is untouched.
// ==========================================

// Builds a ForwardSim seeded with each robot's initial pose, using a
// Reeds-Shepp transit-length distance function (metas are homogeneous, so
// metas[0] supplies the turning radius/wheel base/speed).
DqnV2::ForwardSim make_forward_sim_v2(
    const std::vector<RobotMeta> &robot_metas,
    std::size_t task_count)
{
  std::vector<ReloPush::State> initial_poses;
  initial_poses.reserve(robot_metas.size());
  for (const auto &meta : robot_metas)
    initial_poses.emplace_back(meta.initial_pose.x, meta.initial_pose.y, meta.initial_pose.yaw);

  const double maxc = 1.0 / std::max(robot_metas[0].min_turning_radius_transit, 1e-6);
  const double wb = robot_metas[0].wheel_base;
  DqnV2::DistanceFn distance = [maxc, wb](const ReloPush::State &from, const ReloPush::State &to)
  {
    return DqnV2::reeds_shepp_length(from, to, maxc, 0.5, wb);
  };

  return DqnV2::ForwardSim(std::move(initial_poses), robot_metas[0].speed_transit,
                          std::move(distance), task_count);
}

} // namespace

// Construct one task order with the v2 epsilon-greedy policy: legality uses
// the hard-edge DAG + learned-hard-evidence mask; the epsilon branch splits
// into a uniform portion and a reference-order-following portion; the greedy
// branch scores legal candidates via the forward-sim-derived v2 features.
// `step_logs`, when non-null, is sized to task_count and filled with every
// legal candidate's (task, phi, chosen) entry at each step. When null (the
// default / current callers), behavior and RNG draw sequence are byte-for-
// byte identical to before this parameter existed -- see the exploit/explore
// branches below for how each avoids extra rng() calls or recomputation.
// Declared in DqnAllocationSearch.h (external linkage) so it is directly
// unit-testable; see MARS/tests/phastar_unit_tests.cpp.
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
    std::vector<std::vector<StepCandidateEntry>> *step_logs)
{
  std::vector<std::size_t> order;
  order.reserve(task_count);
  std::vector<char> is_placed(task_count, 0);
  DqnV2::ForwardSim sim = make_forward_sim_v2(robot_metas, task_count);

  std::uniform_real_distribution<double> coin(0.0, 1.0);
  std::uniform_real_distribution<double> ref_bias_coin(0.0, 1.0);

  if (step_logs != nullptr)
    step_logs->assign(task_count, {});

  for (std::size_t step = 0; step < task_count; ++step)
  {
    std::vector<std::size_t> legal;
    legal.reserve(task_count - step);
    for (std::size_t i = 0; i < task_count; ++i)
    {
      if (is_placed[i])
        continue;
      if (DqnV2::is_task_legal_v2(i, geometry, is_placed, constraints,
                                  options.dqn_learned_hard_evidence))
        legal.push_back(i);
    }
    if (legal.empty())
    {
      // Degenerate (should not happen: the hard-edge graph is acyclic by
      // construction): fall back to any remaining task.
      for (std::size_t i = 0; i < task_count; ++i)
        if (!is_placed[i])
          legal.push_back(i);
    }

    std::size_t chosen = legal.front();
    std::vector<StepCandidateEntry> step_entries; // only populated when logging

    if (coin(rng) < epsilon)
    {
      if (ref_bias_coin(rng) < options.dqn_explore_ref_bias)
      {
        chosen = legal.front(); // legal is index-ascending -> reference order's next pick
      }
      else
      {
        std::uniform_int_distribution<std::size_t> pick(0, legal.size() - 1);
        chosen = legal[pick(rng)];
      }

      // Logging-only: computed AFTER the pick, so it never perturbs the rng
      // draw sequence above. No rng() calls below this point.
      if (step_logs != nullptr)
      {
        step_entries.reserve(legal.size());
        for (std::size_t i : legal)
        {
          const auto pv = sim.preview(geoms[i]);
          const double m_t = sim.max_free_time();
          StepCandidateEntry entry;
          entry.step = step;
          entry.candidate_task = i;
          entry.chosen = (i == chosen);
          entry.phi = DqnV2::make_feature_vector_v2(i, pv, m_t, geoms[i], geometry, is_placed,
                                                    sim.window, constraints, seed_makespan, task_count);
          step_entries.push_back(std::move(entry));
        }
      }
    }
    else
    {
      double best_q = -std::numeric_limits<double>::infinity();
      if (step_logs != nullptr)
        step_entries.reserve(legal.size());
      for (std::size_t i : legal)
      {
        const auto pv = sim.preview(geoms[i]);
        const double m_t = sim.max_free_time();
        auto phi = DqnV2::make_feature_vector_v2(i, pv, m_t, geoms[i], geometry, is_placed,
                                                 sim.window, constraints, seed_makespan, task_count);
        const double q = model.predict(phi);
        if (step_logs != nullptr)
        {
          // Reuse the phi already computed above for the argmax -- no
          // second computation.
          StepCandidateEntry entry;
          entry.step = step;
          entry.candidate_task = i;
          entry.chosen = false;
          entry.phi = phi;
          step_entries.push_back(std::move(entry));
        }
        if (q > best_q)
        {
          best_q = q;
          chosen = i;
        }
      }
      if (step_logs != nullptr)
      {
        for (auto &entry : step_entries)
          entry.chosen = (entry.candidate_task == chosen);
      }
    }

    if (step_logs != nullptr)
      (*step_logs)[step] = std::move(step_entries);

    const auto pv = sim.preview(geoms[chosen]);
    sim.commit(chosen, geoms[chosen], pv);

    order.push_back(chosen);
    is_placed[chosen] = 1;
  }
  return order;
}

// Construct one (order, assignment) pair with the v3/v4 epsilon-greedy
// policy: candidates are every (legal task, robot) pair -- legality reuses
// is_task_legal_v2 unchanged; the epsilon branch splits into a uniform
// portion over the flattened candidate list and a reference-order-plus-
// earliest-robot portion (the greedy dispatcher as exploration prior); the
// greedy branch scores every (task, robot) candidate via make_feature_vector_v3
// or _v4 (options.dqn_feature_version == 4) and commits the argmax through
// preview_for/commit_for -- or, in decomposed scoring mode, the
// filter-then-argmax selection described in the header. `step_logs` follows
// the exact same nullptr-default, RNG-invariant contract as construct_order_v2
// (see its comment above) -- logging-only work always happens strictly after
// the RNG-consuming pick, on both branches; decomposed scoring adds zero RNG
// draws of its own (both f_head->predict and model.predict are pure).
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
    std::vector<std::vector<StepCandidateEntry>> *step_logs,
    const QModel *f_head)
{
  TaskOrderAssignment result;
  result.order.reserve(task_count);
  result.assignment.assign(task_count, 0);
  std::vector<char> is_placed(task_count, 0);
  DqnV2::ForwardSim sim = make_forward_sim_v2(robot_metas, task_count);
  const std::size_t robot_count = robot_metas.size();
  const bool use_v4 = options.dqn_feature_version == 4;

  std::uniform_real_distribution<double> coin(0.0, 1.0);
  std::uniform_real_distribution<double> ref_bias_coin(0.0, 1.0);

  if (step_logs != nullptr)
    step_logs->assign(task_count, {});

  for (std::size_t step = 0; step < task_count; ++step)
  {
    std::vector<std::size_t> legal;
    legal.reserve(task_count - step);
    for (std::size_t i = 0; i < task_count; ++i)
    {
      if (is_placed[i])
        continue;
      if (DqnV2::is_task_legal_v2(i, geometry, is_placed, constraints,
                                  options.dqn_learned_hard_evidence))
        legal.push_back(i);
    }
    if (legal.empty())
    {
      // Degenerate (should not happen: the hard-edge graph is acyclic by
      // construction): fall back to any remaining task.
      for (std::size_t i = 0; i < task_count; ++i)
        if (!is_placed[i])
          legal.push_back(i);
    }

    // Flattened candidate list, tasks-major ascending: task index ascending,
    // robot index ascending within a task.
    std::vector<std::pair<std::size_t, std::size_t>> candidates;
    candidates.reserve(legal.size() * std::max<std::size_t>(robot_count, 1));
    for (std::size_t t : legal)
      for (std::size_t r = 0; r < robot_count; ++r)
        candidates.emplace_back(t, r);

    // State-level quantities shared by every candidate at this step (taken
    // before any candidate is previewed/committed) -- mirrors v2's m_t_before.
    const double m_t_before = sim.max_free_time();
    const std::size_t earliest_robot = sim.pick();
    const double min_free_time_before = sim.free_time[earliest_robot];

    // Feature dispatch shared by both branches below: v4 (options.dqn_feature_
    // version == 4) adds load_imbalance/idle_robot_congestion on top of v3's
    // dims via sim.free_time/sim.pose (the pre-commit state for this step,
    // unchanged for every candidate considered here -- sim itself is only
    // mutated once, after the chosen candidate is decided, below). Captures
    // by reference, so it is redefined fresh each step rather than hoisted
    // out of the loop.
    auto compute_phi = [&](std::size_t t, const DqnV2::ForwardSim::Preview &pv_c) -> std::vector<double>
    {
      return use_v4 ? DqnV2::make_feature_vector_v4(t, pv_c, m_t_before, min_free_time_before,
                                                     geoms[t], geometry, is_placed, sim.window,
                                                     constraints, seed_makespan, task_count,
                                                     sim.free_time, sim.pose)
                    : DqnV2::make_feature_vector_v3(t, pv_c, m_t_before, min_free_time_before,
                                                     geoms[t], geometry, is_placed, sim.window,
                                                     constraints, seed_makespan, task_count);
    };

    std::size_t chosen_task = legal.front();
    std::size_t chosen_robot = earliest_robot;
    std::vector<StepCandidateEntry> step_entries; // only populated when logging

    if (coin(rng) < epsilon)
    {
      if (ref_bias_coin(rng) < options.dqn_explore_ref_bias)
      {
        // legal is index-ascending -> reference order's next pick, paired
        // with the greedy dispatcher's robot choice.
        chosen_task = legal.front();
        chosen_robot = earliest_robot;
      }
      else
      {
        std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
        const auto &c = candidates[pick(rng)];
        chosen_task = c.first;
        chosen_robot = c.second;
      }

      // Logging-only: computed AFTER the pick, so it never perturbs the rng
      // draw sequence above. No rng() calls below this point.
      if (step_logs != nullptr)
      {
        step_entries.reserve(candidates.size());
        for (const auto &c : candidates)
        {
          const auto pv = sim.preview_for(c.second, geoms[c.first]);
          StepCandidateEntry entry;
          entry.step = step;
          entry.candidate_task = c.first;
          entry.robot = static_cast<long>(c.second);
          entry.chosen = (c.first == chosen_task && c.second == chosen_robot);
          entry.phi = compute_phi(c.first, pv);
          step_entries.push_back(std::move(entry));
        }
      }
    }
    else
    {
      // Exploit branch. Decomposed scoring (Decision 2) only ever changes
      // SELECTION here -- the explore branch above and its draw order are
      // completely untouched, which is deliberate: it keeps exploration
      // identical across the penalty/decomposed scoring arms for a clean
      // experimental comparison.
      const bool decomposed = (options.dqn_scoring_mode == 1) && (f_head != nullptr);
      if (step_logs != nullptr)
        step_entries.reserve(candidates.size());

      if (!decomposed)
      {
        // Original single-model argmax -- byte-identical to before v4/
        // decomposed scoring existed (compute_phi reduces to the exact same
        // make_feature_vector_v3 call when options.dqn_feature_version != 4).
        double best_q = -std::numeric_limits<double>::infinity();
        for (const auto &c : candidates)
        {
          const auto pv = sim.preview_for(c.second, geoms[c.first]);
          auto phi = compute_phi(c.first, pv);
          const double q = model.predict(phi);
          if (step_logs != nullptr)
          {
            // Reuse the phi already computed above for the argmax -- no
            // second computation.
            StepCandidateEntry entry;
            entry.step = step;
            entry.candidate_task = c.first;
            entry.robot = static_cast<long>(c.second);
            entry.chosen = false;
            entry.phi = phi;
            step_entries.push_back(std::move(entry));
          }
          if (q > best_q)
          {
            best_q = q;
            chosen_task = c.first;
            chosen_robot = c.second;
          }
        }
      }
      else
      {
        // Decision 2 selection: phi + p_fail = sigmoid(f_head->predict(phi))
        // computed once per candidate; survivors (p_fail <= dqn_fail_threshold)
        // compete on q_head (`model`) argmax (strict >, first-seen tie-break);
        // if none survive, fall back to the candidate with the lowest p_fail
        // (strict <, first-seen tie-break).
        std::vector<std::vector<double>> phis(candidates.size());
        std::vector<double> p_fails(candidates.size());
        std::size_t argmin_pfail_idx = 0;
        double min_pfail = std::numeric_limits<double>::infinity();
        for (std::size_t idx = 0; idx < candidates.size(); ++idx)
        {
          const auto &c = candidates[idx];
          const auto pv = sim.preview_for(c.second, geoms[c.first]);
          phis[idx] = compute_phi(c.first, pv);
          const double p_fail = dqn_sigmoid(f_head->predict(phis[idx]));
          p_fails[idx] = p_fail;
          if (p_fail < min_pfail)
          {
            min_pfail = p_fail;
            argmin_pfail_idx = idx;
          }
          if (step_logs != nullptr)
          {
            StepCandidateEntry entry;
            entry.step = step;
            entry.candidate_task = c.first;
            entry.robot = static_cast<long>(c.second);
            entry.chosen = false;
            entry.phi = phis[idx];
            step_entries.push_back(std::move(entry));
          }
        }

        std::size_t survivor_count = 0;
        bool any_survivor = false;
        double best_q = -std::numeric_limits<double>::infinity();
        std::size_t best_idx = 0;
        for (std::size_t idx = 0; idx < candidates.size(); ++idx)
        {
          if (p_fails[idx] > options.dqn_fail_threshold)
            continue;
          ++survivor_count;
          const double q = model.predict(phis[idx]);
          if (q > best_q)
          {
            best_q = q;
            best_idx = idx;
            any_survivor = true;
          }
        }

        const bool fell_back = !any_survivor;
        const std::size_t chosen_idx = fell_back ? argmin_pfail_idx : best_idx;
        chosen_task = candidates[chosen_idx].first;
        chosen_robot = candidates[chosen_idx].second;

        result.decomposed_exploit_steps += 1;
        if (fell_back)
          result.decomposed_fallback_steps += 1;
        result.decomposed_survivor_fraction_sum +=
            static_cast<double>(survivor_count) / static_cast<double>(candidates.size());
      }

      if (step_logs != nullptr)
      {
        for (auto &entry : step_entries)
          entry.chosen = (entry.candidate_task == chosen_task &&
                          static_cast<std::size_t>(entry.robot) == chosen_robot);
      }
    }

    if (step_logs != nullptr)
      (*step_logs)[step] = std::move(step_entries);

    const auto pv = sim.preview_for(chosen_robot, geoms[chosen_task]);
    sim.commit_for(chosen_task, geoms[chosen_task], pv);

    result.order.push_back(chosen_task);
    result.assignment[chosen_task] = chosen_robot;
    is_placed[chosen_task] = 1;
  }
  return result;
}

// v3 only: extends task_order_signature (AllocationSearch.h/cpp, untouched)
// with the per-task robot assignment, since v1/v2's order-only signature
// cannot distinguish two v3 candidates that share a task order but differ in
// robot assignment. External linkage (declared in DqnAllocationSearch.h,
// like construct_order_v2/v3) so it is directly unit-testable.
std::string task_order_assignment_signature(const AllocationScenarioPlan &plan)
{
  std::string sig = task_order_signature(plan);
  sig += "|";
  for (std::size_t i = 0; i < plan.preferred_robot_names_by_original_task.size(); ++i)
  {
    if (i > 0)
      sig += ",";
    sig += plan.preferred_robot_names_by_original_task[i];
  }
  return sig;
}

namespace
{

// Retrospective feasibility/target/first-fail computation shared by
// ingest_rollout_v2 and the transition logger (so the two never compute
// outcome differently). See RolloutOutcome in TransitionLogger.h.
RolloutOutcome compute_rollout_outcome(
    const std::vector<std::size_t> &order,
    const AllocationRunSummary &summary,
    double seed_makespan, double fail_return)
{
  RolloutOutcome outcome;
  const std::size_t n = order.size();
  outcome.feasible = summary.all_tasks_succeeded;

  std::size_t fail_pos = n;
  if (!outcome.feasible)
  {
    fail_pos = 0;
    for (std::size_t t = 0; t < summary.task_rows.size(); ++t)
    {
      if (summary.task_rows[t].status != "SUCCESS")
      {
        fail_pos = t;
        break;
      }
    }
  }

  outcome.makespan = outcome.feasible ? summary.makespan : -1.0;
  outcome.return_target =
      outcome.feasible ? -safe_div(summary.makespan, seed_makespan) : -fail_return;
  outcome.first_fail_rank = fail_pos;
  outcome.first_failed_task =
      (!outcome.feasible && fail_pos < n) ? static_cast<long>(order[fail_pos]) : -1;
  return outcome;
}

// Replays the forward simulation over an executed order to recompute phi
// identically to construction time (same preview/commit sequence per step,
// no RNG use), turning the rollout into per-step training transitions.
void ingest_rollout_v2(
    const std::vector<std::size_t> &order,
    const AllocationRunSummary &summary,
    double seed_makespan,
    double fail_return,
    const std::vector<DqnV2::TaskGeom> &geoms,
    const DqnV2::PairwiseGeometry &geometry,
    const LearnedOrderConstraints &constraints,
    const std::vector<RobotMeta> &robot_metas,
    std::vector<Transition> &replay)
{
  const std::size_t n = order.size();
  if (n == 0)
    return;

  const RolloutOutcome outcome =
      compute_rollout_outcome(order, summary, seed_makespan, fail_return);
  const bool feasible = outcome.feasible;
  const std::size_t fail_pos = outcome.first_fail_rank; // == n if feasible
  const double target = outcome.return_target;

  DqnV2::ForwardSim sim = make_forward_sim_v2(robot_metas, n);
  std::vector<char> is_placed(n, 0);

  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    const auto pv = sim.preview(geoms[task]);
    const double m_t = sim.max_free_time();

    if (feasible || t <= fail_pos)
    {
      Transition tr;
      tr.phi = DqnV2::make_feature_vector_v2(task, pv, m_t, geoms[task], geometry, is_placed,
                                             sim.window, constraints, seed_makespan, n);
      tr.target = target;
      replay.push_back(std::move(tr));
    }

    sim.commit(task, geoms[task], pv);
    is_placed[task] = 1;
  }
}

} // namespace

// Replays the forward simulation over an executed (order, assignment) pair
// to recompute phi identically to construction time (same preview_for/
// commit_for sequence per step, no RNG use), turning the rollout into
// per-step training transitions. Same Monte-Carlo broadcast logic as
// ingest_rollout_v2 (reuses compute_rollout_outcome; same fail_pos
// crediting), but replaying through preview_for/commit_for so every executed
// step's features are recomputed against the ACTUAL committed robot rather
// than the dispatcher's earliest-available pick. Feature dispatch mirrors
// construct_order_v3's compute_phi (v4 when options.dqn_feature_version ==
// 4, fed sim.free_time/sim.pose taken before each step's commit).
//
// Penalty mode (options.dqn_scoring_mode != 1, or f_replay == nullptr):
// exactly the original ingest_rollout_v3 body -- every credited step pushes
// one (phi, target) transition to `replay`. Decomposed mode additionally
// pushes (phi, label) to `*f_replay` for every credited step (label 1.0 iff
// this episode is infeasible AND this step is the first-failed one, else
// 0.0), and restricts `replay` (q_replay) to feasible episodes only --
// infeasible episodes contribute nothing to q_replay in that mode, so
// `fail_return` plays no role in it. See DqnAllocationSearch.h for the full
// contract; exposed here (rather than kept file-local) so both replay
// buffers are directly unit-testable.
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
    std::vector<Transition> *f_replay)
{
  const std::size_t n = order.size();
  if (n == 0)
    return;

  const RolloutOutcome outcome =
      compute_rollout_outcome(order, summary, seed_makespan, fail_return);
  const bool feasible = outcome.feasible;
  const std::size_t fail_pos = outcome.first_fail_rank; // == n if feasible
  const double target = outcome.return_target;
  const bool use_v4 = options.dqn_feature_version == 4;
  const bool decomposed = options.dqn_scoring_mode == 1 && f_replay != nullptr;

  DqnV2::ForwardSim sim = make_forward_sim_v2(robot_metas, n);
  std::vector<char> is_placed(n, 0);

  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    const std::size_t robot = task < assignment.size() ? assignment[task] : 0;
    const double m_t_before = sim.max_free_time();
    const std::size_t earliest_robot = sim.pick();
    const double min_free_time_before = sim.free_time[earliest_robot];
    const auto pv = sim.preview_for(robot, geoms[task]);

    if (feasible || t <= fail_pos)
    {
      std::vector<double> phi =
          use_v4 ? DqnV2::make_feature_vector_v4(task, pv, m_t_before, min_free_time_before,
                                                 geoms[task], geometry, is_placed, sim.window,
                                                 constraints, seed_makespan, n, sim.free_time,
                                                 sim.pose)
                 : DqnV2::make_feature_vector_v3(task, pv, m_t_before, min_free_time_before,
                                                 geoms[task], geometry, is_placed, sim.window,
                                                 constraints, seed_makespan, n);

      if (decomposed)
      {
        Transition f_tr;
        f_tr.phi = phi;
        f_tr.target = (!feasible && t == fail_pos) ? 1.0 : 0.0;
        f_replay->push_back(std::move(f_tr));

        if (feasible)
        {
          Transition q_tr;
          q_tr.phi = std::move(phi);
          q_tr.target = target;
          replay.push_back(std::move(q_tr));
        }
        // Infeasible episodes contribute nothing to q_replay in decomposed
        // mode -- fail_return plays no role here.
      }
      else
      {
        Transition tr;
        tr.phi = std::move(phi);
        tr.target = target;
        replay.push_back(std::move(tr));
      }
    }

    sim.commit_for(task, geoms[task], pv);
    is_placed[task] = 1;
  }
}

// options.dqn_relabel_executed variant of ingest_rollout_v3 -- see the full
// doc comment in DqnAllocationSearch.h.
void ingest_rollout_v3_relabeled(
    const std::vector<std::size_t> &order,
    const std::vector<std::size_t> &assignment, // INTENDED assignment
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
    std::vector<Transition> *f_replay,
    std::vector<std::vector<StepCandidateEntry>> *relabel_step_log)
{
  const std::size_t n = order.size();
  if (n == 0)
    return;

  const RolloutOutcome outcome =
      compute_rollout_outcome(order, summary, seed_makespan, fail_return);
  const bool feasible = outcome.feasible;
  const std::size_t fail_pos = outcome.first_fail_rank; // == n if feasible
  const double target = outcome.return_target;
  const bool use_v4 = options.dqn_feature_version == 4;
  const bool decomposed = options.dqn_scoring_mode == 1 && f_replay != nullptr;
  const std::size_t robot_count = robot_metas.size();

  std::unordered_map<std::string, std::size_t> name_to_idx;
  name_to_idx.reserve(robot_names.size());
  for (std::size_t r = 0; r < robot_names.size(); ++r)
    name_to_idx.emplace(robot_names[r], r);

  DqnV2::ForwardSim sim = make_forward_sim_v2(robot_metas, n);
  std::vector<char> is_placed(n, 0);

  for (std::size_t t = 0; t < n; ++t)
  {
    const std::size_t task = order[t];
    const std::size_t intended_robot = task < assignment.size() ? assignment[task] : 0;

    // Recover the EXECUTED robot for this step from the run summary, falling
    // back to the intended robot when there is no definite executed robot
    // (failing step under early abort, unexecuted tail, or an unresolved
    // robot name). task_rows[t] corresponds to original task order[t] (row
    // POSITION, not TaskCsvRow::task_id) -- mirrors count_assignment_
    // divergence exactly; see AssignmentDivergence's doc comment below for
    // why row position is the verified-correct correspondence. Only a
    // status == "SUCCESS" row counts as definite.
    std::size_t executed_robot = intended_robot;
    bool has_executed = false;
    bool diverged = false;
    if (t < summary.task_rows.size() && summary.task_rows[t].status == "SUCCESS")
    {
      auto it = name_to_idx.find(summary.task_rows[t].robot_name);
      if (it != name_to_idx.end())
      {
        executed_robot = it->second;
        has_executed = true;
        diverged = executed_robot != intended_robot;
      }
    }

    const double m_t_before = sim.max_free_time();
    const std::size_t earliest_robot = sim.pick();
    const double min_free_time_before = sim.free_time[earliest_robot];

    // Extended corpus logging: enumerate every legal (task, robot) candidate
    // against the CORRECTED (relabeled) state -- same legality rule as
    // construct_order_v3 -- independent of the replay-buffer credit below, so
    // this runs even for steps whose credit is skipped by the
    // feasibility/fail_pos gate below.
    if (relabel_step_log != nullptr)
    {
      std::vector<std::size_t> legal;
      legal.reserve(n - t);
      for (std::size_t i = 0; i < n; ++i)
      {
        if (is_placed[i])
          continue;
        if (DqnV2::is_task_legal_v2(i, geometry, is_placed, constraints,
                                    options.dqn_learned_hard_evidence))
          legal.push_back(i);
      }
      if (legal.empty())
      {
        for (std::size_t i = 0; i < n; ++i)
          if (!is_placed[i])
            legal.push_back(i);
      }

      std::vector<StepCandidateEntry> step_entries;
      step_entries.reserve(legal.size() * std::max<std::size_t>(robot_count, 1));
      for (std::size_t cand_task : legal)
      {
        for (std::size_t r = 0; r < robot_count; ++r)
        {
          const auto cand_pv = sim.preview_for(r, geoms[cand_task]);
          std::vector<double> cand_phi =
              use_v4 ? DqnV2::make_feature_vector_v4(cand_task, cand_pv, m_t_before, min_free_time_before,
                                                      geoms[cand_task], geometry, is_placed, sim.window,
                                                      constraints, seed_makespan, n, sim.free_time,
                                                      sim.pose)
                    : DqnV2::make_feature_vector_v3(cand_task, cand_pv, m_t_before, min_free_time_before,
                                                      geoms[cand_task], geometry, is_placed, sim.window,
                                                      constraints, seed_makespan, n);
          StepCandidateEntry entry;
          entry.step = t;
          entry.candidate_task = cand_task;
          entry.robot = static_cast<long>(r);
          entry.chosen = (cand_task == task && r == intended_robot);
          entry.phi = std::move(cand_phi);
          if (entry.chosen)
          {
            entry.executed_robot = has_executed ? static_cast<long>(executed_robot) : -1;
            entry.diverged = has_executed && diverged;
          }
          step_entries.push_back(std::move(entry));
        }
      }
      relabel_step_log->push_back(std::move(step_entries));
    }

    // Replay-buffer credit: same preview/commit contract as ingest_rollout_v3,
    // but against the EXECUTED-or-intended robot for this step, not the
    // intended one.
    const auto pv = sim.preview_for(executed_robot, geoms[task]);
    if (feasible || t <= fail_pos)
    {
      std::vector<double> phi =
          use_v4 ? DqnV2::make_feature_vector_v4(task, pv, m_t_before, min_free_time_before,
                                                  geoms[task], geometry, is_placed, sim.window,
                                                  constraints, seed_makespan, n, sim.free_time,
                                                  sim.pose)
                : DqnV2::make_feature_vector_v3(task, pv, m_t_before, min_free_time_before,
                                                  geoms[task], geometry, is_placed, sim.window,
                                                  constraints, seed_makespan, n);

      if (decomposed)
      {
        Transition f_tr;
        f_tr.phi = phi;
        f_tr.target = (!feasible && t == fail_pos) ? 1.0 : 0.0;
        f_replay->push_back(std::move(f_tr));

        if (feasible)
        {
          Transition q_tr;
          q_tr.phi = std::move(phi);
          q_tr.target = target;
          replay.push_back(std::move(q_tr));
        }
        // Infeasible episodes contribute nothing to q_replay in decomposed
        // mode -- fail_return plays no role here.
      }
      else
      {
        Transition tr;
        tr.phi = std::move(phi);
        tr.target = target;
        replay.push_back(std::move(tr));
      }
    }

    sim.commit_for(task, geoms[task], pv);
    is_placed[task] = 1;
  }
}

// v3 only: assignment[original_task_index] = robot index -> robot NAME per
// original task index, sized to assignment.size(), for populating
// AllocationScenarioPlan::preferred_robot_names_by_original_task (see
// make_assignment_plan_from_greedy in AllocationSearch.cpp for the existing
// precedent of threading a robot choice through that same field). External
// linkage (declared in DqnAllocationSearch.h, like assignment_from_summary
// below) so PgTableExport.cpp can reuse it for the exported ref_assign
// field's robot-name lookup without duplicating this logic; behavior is
// unchanged for every existing caller (this file only).
std::vector<std::string> assignment_to_robot_names(
    const std::vector<std::size_t> &assignment,
    const std::vector<std::string> &robot_names)
{
  std::vector<std::string> names(assignment.size());
  for (std::size_t i = 0; i < assignment.size(); ++i)
    names[i] = assignment[i] < robot_names.size() ? robot_names[assignment[i]] : std::string();
  return names;
}

// v3 only: recovers the per-task robot assignment ACTUALLY executed by a
// (feasible) run summary, for warm-starting ingest_rollout_v3 from the seed
// episode. Mirrors make_assignment_plan_from_greedy's task_rows walk
// (AllocationSearch.cpp): row.task_id is 1-based, row.robot_name is the
// robot that executed (and, for a feasible summary, succeeded on) that task.
// External linkage (declared in DqnAllocationSearch.h) so PgTableExport.cpp
// can recover the certified reference assignment (ref_assign) for the
// exported seed plan the same way run_dqn_search recovers it for warm-start
// ingestion; behavior is unchanged for every existing caller (this file
// only).
std::vector<std::size_t> assignment_from_summary(
    const AllocationRunSummary &summary,
    const std::vector<std::string> &robot_names,
    std::size_t task_count)
{
  std::unordered_map<std::string, std::size_t> name_to_idx;
  name_to_idx.reserve(robot_names.size());
  for (std::size_t r = 0; r < robot_names.size(); ++r)
    name_to_idx.emplace(robot_names[r], r);

  std::vector<std::size_t> assignment(task_count, 0);
  for (const auto &row : summary.task_rows)
  {
    if (row.task_id <= 0)
      continue;
    const std::size_t idx = static_cast<std::size_t>(row.task_id - 1);
    if (idx >= task_count || row.status != "SUCCESS")
      continue;
    auto it = name_to_idx.find(row.robot_name);
    if (it != name_to_idx.end())
      assignment[idx] = it->second;
  }
  return assignment;
}

// Minibatch SGD regression of the Q model toward stored returns (Huber-clipped
// error + small L2). This is fitted-value regression onto Monte-Carlo returns.
// For model.hidden == 0 (the default linear model) this reproduces the exact
// arithmetic and rng draw order of the original hand-rolled linear update.
// Declared in DqnAllocationSearch.h (external linkage) so it is directly
// unit-testable and so options.dqn_freeze_model can gate its call sites in
// run_dqn_search below.
void train_model(
    QModel &model,
    const std::vector<Transition> &replay,
    double learning_rate,
    int grad_steps,
    int minibatch_size,
    std::mt19937 &rng)
{
  if (replay.empty())
    return;

  constexpr double huber_delta = 1.0;
  constexpr double l2 = 1e-4;
  const std::size_t num_params = model.num_params();
  std::uniform_int_distribution<std::size_t> pick(0, replay.size() - 1);
  const std::size_t batch =
      std::min<std::size_t>(std::max(1, minibatch_size), replay.size());

  for (int step = 0; step < grad_steps; ++step)
  {
    std::vector<double> grad(num_params, 0.0);
    for (std::size_t b = 0; b < batch; ++b)
    {
      const Transition &tr = replay[pick(rng)];
      model.accumulate_grad(tr.phi, tr.target, huber_delta, grad);
    }
    model.apply_grad(grad, batch, learning_rate, l2);
  }
}

// Minibatch SGD of a QModel toward binary failure labels (Decision 2's
// f_head), via the weighted logistic gradient path. Mirrors train_model's
// minibatch-SGD structure exactly (same replay-sampling/apply_grad shape),
// but calls accumulate_grad_logistic instead of accumulate_grad. The
// positive-class weight is recomputed once per call from `f_replay`'s own
// label composition when `fixed_pos_weight` <= 0 (auto: clamp(#neg/#pos, 1,
// 10), guarded against #pos == 0 by defaulting to 1.0); a positive
// `fixed_pos_weight` is used as-is for every sample. Only ever called when
// decomposed scoring is active -- penalty mode never calls this function, so
// it never draws the extra RNG this uses.
void train_model_logistic(
    QModel &model,
    const std::vector<Transition> &f_replay, // .target holds a 0/1 label
    double learning_rate,
    int grad_steps,
    int minibatch_size,
    double fixed_pos_weight,
    std::mt19937 &rng)
{
  if (f_replay.empty())
    return;

  std::size_t pos_count = 0, neg_count = 0;
  for (const auto &tr : f_replay)
  {
    if (tr.target >= 0.5)
      ++pos_count;
    else
      ++neg_count;
  }
  const double pos_weight =
      fixed_pos_weight > 0.0
          ? fixed_pos_weight
          : (pos_count > 0
                 ? std::clamp(static_cast<double>(neg_count) / static_cast<double>(pos_count),
                              1.0, 10.0)
                 : 1.0);

  constexpr double l2 = 1e-4;
  const std::size_t num_params = model.num_params();
  std::uniform_int_distribution<std::size_t> pick(0, f_replay.size() - 1);
  const std::size_t batch =
      std::min<std::size_t>(std::max(1, minibatch_size), f_replay.size());

  for (int step = 0; step < grad_steps; ++step)
  {
    std::vector<double> grad(num_params, 0.0);
    for (std::size_t b = 0; b < batch; ++b)
    {
      const Transition &tr = f_replay[pick(rng)];
      const double weight = tr.target >= 0.5 ? pos_weight : 1.0;
      model.accumulate_grad_logistic(tr.phi, tr.target, weight, grad);
    }
    model.apply_grad(grad, batch, learning_rate, l2);
  }
}

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
    int *out_enforced_constraint_count)
{
  SequenceSearchOutcome outcome;
  outcome.best_feasible = make_placeholder_summary(kDqnLabel, greedy_plan);
  outcome.best_partial = make_placeholder_summary(kDqnLabel, greedy_plan);

  const std::size_t task_count = loaded_sequence.size();
  const int total_iterations = options.dqn_iterations;
  // use_v2 now means "exactly v2" (not ">= 2") so it can keep gating the
  // v2-specific construct/ingest/model-dim/logging choices once v3 exists;
  // use_geometry_features is the ">= 2" condition for the precompute (TaskGeom
  // / PairwiseGeometry / ForwardSim) that v2 and v3 both need. For
  // dqn_feature_version in {1,2} both booleans take on exactly the values the
  // old single `use_v2 = (dqn_feature_version >= 2)` did, so v1/v2 behavior
  // below is unchanged.
  const bool use_v2 = options.dqn_feature_version == 2;
  const bool use_v3 = options.dqn_feature_version >= 3;
  const bool use_geometry_features = use_v2 || use_v3;
  // Decomposed scoring (Decision 2) needs the v3/v4 explicit (task, robot)
  // machinery; parse_runtime_options already downgrades an inconsistent
  // --dqn-scoring=decomposed to penalty when dqn_feature_version < 3, but
  // guard here too in case `options` was constructed directly (e.g. tests)
  // rather than through that validation path.
  const bool decomposed_scoring = options.dqn_scoring_mode == 1 && use_v3;

  if (total_iterations <= 0 || !seed_summary.all_tasks_succeeded ||
      task_count <= 1 || robot_names.empty() ||
      (use_geometry_features && robot_metas.empty()) ||
      !std::isfinite(seed_summary.makespan) || seed_summary.makespan <= 0.0)
  {
    if (out_enforced_constraint_count)
      *out_enforced_constraint_count = 0;
    std::cout << "[Search] Skipping DQN allocation search." << std::endl;
    return outcome;
  }

  std::cout << "[Search] DQN online allocation search from "
            << seed_summary.label
            << (use_v3 ? " (task+robot)..." : " (task-order only)...") << std::endl;

  // Seed the running best with the feasible seed solution.
  AllocationRunSummary current = seed_summary;
  current.label = kDqnLabel;
  outcome.best_feasible = current;
  outcome.best_partial = current;
  outcome.has_feasible = true;
  outcome.has_partial = true;

  const double seed_makespan = seed_summary.makespan;
  // Failure return (normalized): larger magnitude than any plausible feasible
  // return so the policy strongly avoids insertions that lead to infeasibility.
  const double fail_return = 3.0;

  // Transition logging: v2 (original 9-phi schema, byte-identical to before
  // dqn_relabel_executed existed) or v3/v4 WITH options.dqn_relabel_executed
  // (extended 12-phi + candidate_robot/executed_robot/diverged schema -- see
  // TransitionLogger::log_rollout and ingest_rollout_v3_relabeled). v3/v4
  // WITHOUT dqn_relabel_executed still declines to log (unchanged): the
  // construct-time step log ingest_rollout_v3 would otherwise reuse reflects
  // intended-state predictions, not the executed-state corpus the extended
  // schema is for. See MARS/09rl-pretrained-study-plan.md 3.1.
  std::unique_ptr<TransitionLogger> transition_logger;
  std::string log_family = "unknown";
  int log_index = -1;
  const bool relabel_logging = use_v3 && options.dqn_relabel_executed;
  if (!options.dqn_log_transitions_path.empty())
  {
    if (use_v2 || relabel_logging)
    {
      transition_logger = std::make_unique<TransitionLogger>(
          options.dqn_log_transitions_path, relabel_logging);
      parse_family_index(options.input_sequence_path, log_family, log_index);
    }
    else
    {
      std::cerr << "[TransitionLogger] --dqn-log-transitions requires "
                   "--dqn-features=2, ignoring."
                << std::endl;
    }
  }
  const bool logging_active = static_cast<bool>(transition_logger);

  const bool learn_order = options.enable_order_constraint_learning;
  LearnedOrderConstraints dqn_constraints =
      make_learned_order_constraints(task_count);
  const LearnedOrderConstraints &active_constraints =
      learn_order ? dqn_constraints : disabled_constraints;
  AllocationRunSummary order_learning_reference = current;

  // v1: static per-task features (cost/obsRelo/numPaths). v2 & v3: geometric
  // precompute (per-task corridor geometry + reference-order-consistent
  // DAG/congestion/boundary matrices), built once and reused by every
  // rollout. Only one of the two is populated, matching dqn_feature_version.
  const auto static_feats = use_geometry_features ? std::vector<StaticTaskFeatures>()
                                                  : build_static_features(loaded_sequence);
  std::vector<DqnV2::TaskGeom> v2_geoms;
  DqnV2::PairwiseGeometry v2_geometry;
  if (use_geometry_features)
  {
    v2_geoms = DqnV2::extract_task_geoms(loaded_sequence, robot_metas[0]);
    const auto &boundary = loaded_sequence[0].snapshot.parameters.boundary;
    DqnV2::WorkspaceBounds workspace{boundary.xMin, boundary.xMax, boundary.yMin, boundary.yMax};
    v2_geometry = DqnV2::build_pairwise_geometry(v2_geoms, workspace, robot_metas[0].size.width);
  }

  const std::size_t model_dim = options.dqn_feature_version == 4 ? DqnV2::kFeatDimV4
                               : use_v3                          ? DqnV2::kFeatDimV3
                               : use_v2                          ? DqnV2::kFeatDim
                                                                   : kFeatDim;
  QModel model(model_dim, static_cast<std::size_t>(options.dqn_hidden_units), rng);
  // Decomposed scoring's second model (f_head): constructed AFTER q_head from
  // the same rng, and ONLY when decomposed_scoring is active -- QModel's
  // constructor draws from rng when hidden > 0, so constructing it
  // unconditionally would perturb the RNG stream in penalty mode too,
  // breaking the byte-identical invariant that mode must keep.
  std::unique_ptr<QModel> f_head;
  if (decomposed_scoring)
  {
    f_head = std::make_unique<QModel>(model_dim, static_cast<std::size_t>(options.dqn_hidden_units), rng);
    // Zero-shot/deployment plumbing: load pretrained f_head weights if
    // requested, mirroring dqn_init_weights_path's handling for q_head below.
    // On failure (missing file or dim/hidden mismatch), QModel::load already
    // left f_head's freshly-initialized params untouched, so we just warn and
    // keep them.
    if (!options.dqn_init_fail_weights_path.empty())
    {
      if (!f_head->load(options.dqn_init_fail_weights_path))
      {
        std::cerr << "[QModel] Failed to load init fail-weights from "
                  << options.dqn_init_fail_weights_path
                  << ", keeping freshly-initialized f_head weights." << std::endl;
      }
    }
  }
  std::vector<Transition> replay;
  std::vector<Transition> f_replay; // decomposed scoring only; unused otherwise

  // Fine-tune mode: load pretrained weights if requested. On success, use the
  // fine-tune learning-rate/epsilon schedule for the rest of this run instead
  // of the cold-start hyperparameters; on failure (missing file or dim/hidden
  // mismatch), QModel::load already left `model` untouched, so we just fall
  // back to the normal cold start.
  bool finetune_mode = false;
  if (!options.dqn_init_weights_path.empty())
  {
    if (model.load(options.dqn_init_weights_path))
    {
      finetune_mode = true;
    }
    else
    {
      std::cerr << "[QModel] Failed to load init weights from "
                << options.dqn_init_weights_path
                << ", falling back to cold start." << std::endl;
    }
  }
  const double learning_rate = finetune_mode ? options.dqn_finetune_learning_rate
                                             : options.dqn_learning_rate;
  const double epsilon_start = finetune_mode ? options.dqn_finetune_epsilon_start
                                             : options.dqn_epsilon_start;
  const double epsilon_end = finetune_mode ? options.dqn_finetune_epsilon_end
                                           : options.dqn_epsilon_end;

  // Warm-start: treat the feasible seed order as a positive example.
  {
    const auto seed_order =
        normalized_task_order(seed_summary.plan, task_count);
    if (use_v3)
    {
      const auto seed_assignment =
          assignment_from_summary(seed_summary, robot_names, task_count);
      ingest_rollout_v3(seed_order, seed_assignment, seed_summary, seed_makespan,
                        fail_return, v2_geoms, v2_geometry, active_constraints,
                        robot_metas, options, replay, decomposed_scoring ? &f_replay : nullptr);
    }
    else if (use_v2)
      ingest_rollout_v2(seed_order, seed_summary, seed_makespan, fail_return,
                        v2_geoms, v2_geometry, active_constraints, robot_metas, replay);
    else
      ingest_rollout(seed_order, seed_summary, seed_makespan, fail_return,
                     static_feats, active_constraints, replay);
    if (!options.dqn_freeze_model)
    {
      train_model(model, replay, learning_rate,
                  options.dqn_grad_steps_per_iter, options.dqn_minibatch_size, rng);
      if (decomposed_scoring && f_head)
        train_model_logistic(*f_head, f_replay, learning_rate, options.dqn_grad_steps_per_iter,
                             options.dqn_minibatch_size, options.dqn_fail_pos_weight, rng);
    }
  }

  std::unordered_set<std::string> tried_signatures;
  if (use_v3)
  {
    tried_signatures.insert(task_order_assignment_signature(greedy_plan));
    tried_signatures.insert(task_order_assignment_signature(seed_summary.plan));
  }
  else
  {
    tried_signatures.insert(task_order_signature(greedy_plan));
    tried_signatures.insert(task_order_signature(seed_summary.plan));
  }

  const int effective_batch = std::max(1, batch_size);

  // Decomposed-scoring diagnostics (Decision 2), aggregated across every
  // rollout's TaskOrderAssignment for the final summary line. Stay at 0 in
  // penalty mode.
  long long decomposed_total_exploit_steps = 0;
  long long decomposed_total_fallback_steps = 0;
  double decomposed_total_survivor_fraction_sum = 0.0;

  // v3/v4-only assignment-divergence diagnostics (measurement only -- see
  // AssignmentDivergence in DqnAllocationSearch.h). Accumulated across every
  // evaluated candidate from data (requests[b].plan, results[b].summary)
  // that ingest_rollout_v3 already consumes for training, so this adds no
  // RNG draws, no extra candidate construction, and does not alter any
  // result. Stay at 0 for v1/v2 (accumulation below is gated on use_v3).
  AssignmentDivergence divergence_total;
  AssignmentDivergence divergence_feasible;
  AssignmentDivergence divergence_infeasible;
  long long divergence_episodes = 0;

  for (int iter = 0; iter < total_iterations;)
  {
    const double progress =
        total_iterations > 1
            ? static_cast<double>(iter) / static_cast<double>(total_iterations - 1)
            : 1.0;
    const double epsilon =
        epsilon_end + (epsilon_start - epsilon_end) * (1.0 - progress);

    // Build a batch of constructed candidate plans.
    std::vector<ScenarioEvaluationRequest> requests;
    std::vector<std::vector<std::size_t>> batch_orders;
    // v3 only: batch_assignments[b][original_task_index] = robot index,
    // parallel to batch_orders. Left empty (per entry) for v1/v2.
    std::vector<std::vector<std::size_t>> batch_assignments;
    std::vector<int> batch_iters;
    // Per-request step_logs (only populated when logging_active), indexed
    // the same as requests/batch_orders/batch_iters.
    std::vector<std::vector<std::vector<StepCandidateEntry>>> batch_step_logs;
    const int batch_end = std::min(total_iterations, iter + effective_batch);
    for (; iter < batch_end; ++iter)
    {
      std::vector<std::size_t> order;
      std::vector<std::size_t> assignment;
      std::vector<std::vector<StepCandidateEntry>> order_step_log;
      // v3/v4 only: diagnostics from the accepted attempt's construct_order_v3
      // call (decomposed scoring; stays default-zero otherwise).
      TaskOrderAssignment last_oa;
      // Try a few times to produce an order (v1/v2) or order+assignment (v3)
      // not already evaluated.
      for (int attempt = 0; attempt < 8; ++attempt)
      {
        double eps = attempt == 0 ? epsilon : std::max(epsilon, 0.5);
        AllocationScenarioPlan probe;
        if (use_v3)
        {
          TaskOrderAssignment oa = construct_order_v3(
              model, task_count, v2_geoms, v2_geometry, active_constraints,
              robot_metas, seed_makespan, eps, options, rng,
              // construct-time step_logs unused: v3/v4 corpus logging (when
              // active) is relabel-mode-only, and ingest_rollout_v3_relabeled
              // rebuilds its own step log from the executed replay below
              // rather than reusing this intended-state one (this ternary was
              // always nullptr pre-relabel too, since logging_active implied
              // use_v2, never use_v3).
              nullptr, f_head.get());
          order = oa.order;
          assignment = oa.assignment;
          last_oa = oa;
          probe.task_order = order;
          probe.preferred_robot_names_by_original_task =
              assignment_to_robot_names(assignment, robot_names);
          if (tried_signatures.insert(task_order_assignment_signature(probe)).second)
            break;
        }
        else
        {
          order = use_v2
              ? construct_order_v2(model, task_count, v2_geoms, v2_geometry,
                                   active_constraints, robot_metas, seed_makespan,
                                   eps, options, rng,
                                   logging_active ? &order_step_log : nullptr)
              : construct_order(model, task_count, static_feats,
                               active_constraints, eps, rng);
          probe.task_order = order;
          if (tried_signatures.insert(task_order_signature(probe)).second)
            break;
        }
      }

      if (decomposed_scoring)
      {
        decomposed_total_exploit_steps += last_oa.decomposed_exploit_steps;
        decomposed_total_fallback_steps += last_oa.decomposed_fallback_steps;
        decomposed_total_survivor_fraction_sum += last_oa.decomposed_survivor_fraction_sum;
      }

      ScenarioEvaluationRequest req;
      req.plan.task_order = order; // task-order only (v1/v2); v3 adds the assignment below
      if (use_v3)
        req.plan.preferred_robot_names_by_original_task =
            assignment_to_robot_names(assignment, robot_names);
      req.label = kDqnLabel;
      req.parking_seed = rng();
      requests.push_back(std::move(req));
      batch_orders.push_back(std::move(order));
      batch_assignments.push_back(std::move(assignment));
      batch_iters.push_back(iter + 1);
      if (logging_active)
        batch_step_logs.push_back(std::move(order_step_log));
    }

    const auto batch_start = std::chrono::steady_clock::now();
    auto results = evaluate_scenario_batch(loaded_sequence, options, requests);
    outcome.lns_batch_planning_times_s.push_back(elapsed_seconds(batch_start));

    int batch_failed = 0;
    for (std::size_t b = 0; b < results.size(); ++b)
    {
      AllocationRunSummary candidate = results[b].summary;
      candidate.label = kDqnLabel;

      if (!candidate.all_tasks_succeeded)
      {
        ++outcome.lns_failed_iterations;
        ++batch_failed;
        if (learn_order)
        {
          learn_order_constraints_from_failed_summary(
              candidate, order_learning_reference.plan, loaded_sequence,
              dqn_constraints);
        }
      }

      if (use_v3)
      {
        const AssignmentDivergence step_divergence =
            count_assignment_divergence(requests[b].plan, candidate);
        divergence_total += step_divergence;
        if (candidate.all_tasks_succeeded)
          divergence_feasible += step_divergence;
        else
          divergence_infeasible += step_divergence;
        ++divergence_episodes;
      }

      // v3 relabel-logging only: freshly-built (executed-state) step log,
      // populated by ingest_rollout_v3_relabeled below instead of reusing the
      // (intended-state) construct-time batch_step_logs[b].
      std::vector<std::vector<StepCandidateEntry>> relabel_step_log;
      if (use_v3)
      {
        if (options.dqn_relabel_executed)
          ingest_rollout_v3_relabeled(batch_orders[b], batch_assignments[b], candidate,
                                      seed_makespan, fail_return, v2_geoms, v2_geometry,
                                      active_constraints, robot_metas, robot_names, options,
                                      replay, decomposed_scoring ? &f_replay : nullptr,
                                      logging_active ? &relabel_step_log : nullptr);
        else
          ingest_rollout_v3(batch_orders[b], batch_assignments[b], candidate, seed_makespan,
                            fail_return, v2_geoms, v2_geometry, active_constraints,
                            robot_metas, options, replay, decomposed_scoring ? &f_replay : nullptr);
      }
      else if (use_v2)
        ingest_rollout_v2(batch_orders[b], candidate, seed_makespan, fail_return,
                          v2_geoms, v2_geometry, active_constraints, robot_metas, replay);
      else
        ingest_rollout(batch_orders[b], candidate, seed_makespan, fail_return,
                       static_feats, active_constraints, replay);

      if (logging_active)
      {
        const RolloutOutcome rollout_outcome = compute_rollout_outcome(
            batch_orders[b], candidate, seed_makespan, fail_return);
        std::vector<StepCandidateEntry> flattened;
        // v3/v4 relabel-logging reads its own freshly-built (executed-state)
        // step log; every other logging path (v2, today's only other active
        // one) reads the construct-time step log as before.
        auto &source_log = (use_v3 && options.dqn_relabel_executed)
                                ? relabel_step_log
                                : batch_step_logs[b];
        for (auto &step_entries : source_log)
          for (auto &entry : step_entries)
            flattened.push_back(std::move(entry));
        transition_logger->log_rollout(log_family, log_index,
                                       options.base_random_seed, batch_iters[b],
                                       flattened, rollout_outcome);
      }

      if (!outcome.has_partial ||
          is_preferred_search_result(candidate, outcome.best_partial))
      {
        outcome.best_partial = candidate;
        outcome.best_partial.label = kDqnLabel;
        outcome.has_partial = true;
      }

      bool improved = false;
      if (candidate.all_tasks_succeeded &&
          (!outcome.has_feasible ||
           is_better_run(candidate, outcome.best_feasible)))
      {
        outcome.best_feasible = candidate;
        outcome.best_feasible.label = kDqnLabel;
        outcome.has_feasible = true;
        improved = true;
        if (is_better_run(candidate, order_learning_reference))
          order_learning_reference = candidate;
      }

      std::cout << "[DQN " << batch_iters[b] << "/" << total_iterations << "] "
                << "eps=" << std::fixed << std::setprecision(2) << epsilon
                << " feas=" << (candidate.all_tasks_succeeded ? 1 : 0);
      if (candidate.all_tasks_succeeded)
        std::cout << " mk=" << std::fixed << std::setprecision(2)
                  << candidate.makespan << "s";
      else
        std::cout << " suc=" << candidate.successful_tasks << "/"
                  << (candidate.successful_tasks + candidate.failed_tasks);
      if (outcome.has_feasible)
        std::cout << " best=" << std::fixed << std::setprecision(2)
                  << outcome.best_feasible.makespan << "s";
      else
        std::cout << " best=none";
      std::cout << (improved ? " *" : "") << std::endl;
    }

    outcome.lns_batch_best_makespans.push_back(
        outcome.has_feasible ? outcome.best_feasible.makespan
                             : std::numeric_limits<double>::infinity());
    outcome.lns_batch_failed_iterations.push_back(batch_failed);

    // Learn from the batch.
    if (!options.dqn_freeze_model)
    {
      train_model(model, replay, learning_rate,
                  options.dqn_grad_steps_per_iter, options.dqn_minibatch_size, rng);
      if (decomposed_scoring && f_head)
        train_model_logistic(*f_head, f_replay, learning_rate, options.dqn_grad_steps_per_iter,
                             options.dqn_minibatch_size, options.dqn_fail_pos_weight, rng);
    }
  }

  if (out_enforced_constraint_count)
    *out_enforced_constraint_count = learn_order ? dqn_constraints.enforced_count : 0;

  std::cout << "[Search] DQN online allocation search done. best="
            << (outcome.has_feasible
                    ? (std::to_string(outcome.best_feasible.makespan) + "s")
                    : std::string("none"))
            << " failed=" << outcome.lns_failed_iterations << "/"
            << total_iterations;
  if (decomposed_scoring)
  {
    const double avg_pass = decomposed_total_exploit_steps > 0
                                 ? decomposed_total_survivor_fraction_sum /
                                       static_cast<double>(decomposed_total_exploit_steps)
                                 : 0.0;
    std::cout << " fallbacks=" << decomposed_total_fallback_steps
              << " avg_pass=" << std::fixed << std::setprecision(2) << avg_pass;
  }
  std::cout << std::endl;

  // v3/v4-only: assignment-divergence diagnostic summary (measurement only,
  // see AssignmentDivergence in DqnAllocationSearch.h). Output-only -- does
  // not influence outcome or any further computation.
  if (use_v3)
  {
    const int c = divergence_total.counted_placements;
    const int d = divergence_total.diverged_placements;
    const double pct = c > 0 ? 100.0 * static_cast<double>(d) / static_cast<double>(c) : 0.0;
    std::cout << "[DQN diag] assignment divergence: " << d << "/" << c
              << " placements (" << std::fixed << std::setprecision(2) << pct
              << "%) across " << divergence_episodes << " episodes (feasible: "
              << divergence_feasible.diverged_placements << "/"
              << divergence_feasible.counted_placements << ", infeasible: "
              << divergence_infeasible.diverged_placements << "/"
              << divergence_infeasible.counted_placements << ")" << std::endl;

    // Supplementary (not part of the required line above): last-attempted-
    // robot divergence on FAILED rows. Kept separate because it is a much
    // weaker signal -- see AssignmentDivergence's counted_failed/
    // diverged_failed comment in DqnAllocationSearch.h.
    if (divergence_total.counted_failed > 0)
    {
      const double failed_pct = 100.0 * static_cast<double>(divergence_total.diverged_failed) /
                                static_cast<double>(divergence_total.counted_failed);
      std::cout << "[DQN diag] failed-row divergence (last attempted robot): "
                << divergence_total.diverged_failed << "/" << divergence_total.counted_failed
                << " (" << std::fixed << std::setprecision(2) << failed_pct << "%)" << std::endl;
    }

    if (outcome.has_feasible)
    {
      const AssignmentDivergence best_divergence =
          count_assignment_divergence(outcome.best_feasible.plan, outcome.best_feasible);
      std::cout << "[DQN diag] best-plan divergence: " << best_divergence.diverged_placements
                << "/" << best_divergence.counted_placements << std::endl;
    }
    else
    {
      std::cout << "[DQN diag] best-plan divergence: n/a (no feasible solution found)"
                << std::endl;
    }
  }

  return outcome;
}
