/*****************************************************************
 * Per-instance online Q-learning task allocator.
 *
 * Constructs task insertion orders with an epsilon-greedy linear Q-policy,
 * evaluates them with evaluate_scenario_batch (the same env step LNS uses),
 * and regresses the linear Q-function toward the Monte-Carlo return of each
 * rollout. See DqnAllocationSearch.h for the design rationale.
 ******************************************************************/

#include <DqnAllocationSearch.h>
#include <AllocationSearch.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
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

// Linear Q-function: Q(s,a) = w . phi(s,a).
struct LinearQModel
{
  std::vector<double> weights;

  explicit LinearQModel(std::size_t dim) : weights(dim, 0.0) {}

  double predict(const std::vector<double> &phi) const
  {
    double q = 0.0;
    for (std::size_t i = 0; i < weights.size() && i < phi.size(); ++i)
      q += weights[i] * phi[i];
    return q;
  }
};

struct Transition
{
  std::vector<double> phi;
  double target = 0.0; // Monte-Carlo return (normalized)
};

// Construct one task order with an epsilon-greedy linear Q-policy. Candidates
// that would violate enforced precedence are masked out.
std::vector<std::size_t> construct_order(
    const LinearQModel &model,
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

// Minibatch SGD regression of the linear Q toward stored returns (Huber-clipped
// error + small L2). This is fitted-value regression onto Monte-Carlo returns.
void train_model(
    LinearQModel &model,
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
  const std::size_t dim = model.weights.size();
  std::uniform_int_distribution<std::size_t> pick(0, replay.size() - 1);
  const std::size_t batch =
      std::min<std::size_t>(std::max(1, minibatch_size), replay.size());

  for (int step = 0; step < grad_steps; ++step)
  {
    std::vector<double> grad(dim, 0.0);
    for (std::size_t b = 0; b < batch; ++b)
    {
      const Transition &tr = replay[pick(rng)];
      double q = model.predict(tr.phi);
      double err = q - tr.target;
      // Huber gradient: clip the error magnitude.
      double clipped = std::max(-huber_delta, std::min(huber_delta, err));
      for (std::size_t i = 0; i < dim && i < tr.phi.size(); ++i)
        grad[i] += clipped * tr.phi[i];
    }
    for (std::size_t i = 0; i < dim; ++i)
    {
      grad[i] = grad[i] / static_cast<double>(batch) + l2 * model.weights[i];
      model.weights[i] -= learning_rate * grad[i];
    }
  }
}
} // namespace

SequenceSearchOutcome run_dqn_search(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RuntimeOptions &options,
    const AllocationRunSummary &seed_summary,
    const AllocationScenarioPlan &greedy_plan,
    const std::vector<std::string> &robot_names,
    int batch_size,
    std::mt19937 &rng,
    const LearnedOrderConstraints &disabled_constraints,
    int *out_enforced_constraint_count)
{
  SequenceSearchOutcome outcome;
  outcome.best_feasible = make_placeholder_summary(kDqnLabel, greedy_plan);
  outcome.best_partial = make_placeholder_summary(kDqnLabel, greedy_plan);

  const std::size_t task_count = loaded_sequence.size();
  const int total_iterations = options.lns_iterations;

  if (total_iterations <= 0 || !seed_summary.all_tasks_succeeded ||
      task_count <= 1 || robot_names.empty() ||
      !std::isfinite(seed_summary.makespan) || seed_summary.makespan <= 0.0)
  {
    if (out_enforced_constraint_count)
      *out_enforced_constraint_count = 0;
    std::cout << "[Search] Skipping DQN allocation search." << std::endl;
    return outcome;
  }

  std::cout << "[Search] DQN online allocation search from "
            << seed_summary.label << " (task-order only)..." << std::endl;

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

  const bool learn_order = options.enable_order_constraint_learning;
  LearnedOrderConstraints dqn_constraints =
      make_learned_order_constraints(task_count);
  const LearnedOrderConstraints &active_constraints =
      learn_order ? dqn_constraints : disabled_constraints;
  AllocationRunSummary order_learning_reference = current;

  const auto static_feats = build_static_features(loaded_sequence);
  LinearQModel model(kFeatDim);
  std::vector<Transition> replay;

  // Warm-start: treat the feasible seed order as a positive example.
  {
    const auto seed_order =
        normalized_task_order(seed_summary.plan, task_count);
    ingest_rollout(seed_order, seed_summary, seed_makespan, fail_return,
                   static_feats, active_constraints, replay);
    train_model(model, replay, options.dqn_learning_rate,
                options.dqn_grad_steps_per_iter, options.dqn_minibatch_size, rng);
  }

  std::unordered_set<std::string> tried_signatures;
  tried_signatures.insert(task_order_signature(greedy_plan));
  tried_signatures.insert(task_order_signature(seed_summary.plan));

  const int effective_batch = std::max(1, batch_size);

  for (int iter = 0; iter < total_iterations;)
  {
    const double progress =
        total_iterations > 1
            ? static_cast<double>(iter) / static_cast<double>(total_iterations - 1)
            : 1.0;
    const double epsilon =
        options.dqn_epsilon_end +
        (options.dqn_epsilon_start - options.dqn_epsilon_end) * (1.0 - progress);

    // Build a batch of constructed candidate plans.
    std::vector<ScenarioEvaluationRequest> requests;
    std::vector<std::vector<std::size_t>> batch_orders;
    std::vector<int> batch_iters;
    const int batch_end = std::min(total_iterations, iter + effective_batch);
    for (; iter < batch_end; ++iter)
    {
      std::vector<std::size_t> order;
      // Try a few times to produce an order not already evaluated.
      for (int attempt = 0; attempt < 8; ++attempt)
      {
        double eps = attempt == 0 ? epsilon : std::max(epsilon, 0.5);
        order = construct_order(model, task_count, static_feats,
                                active_constraints, eps, rng);
        AllocationScenarioPlan probe;
        probe.task_order = order;
        if (tried_signatures.insert(task_order_signature(probe)).second)
          break;
      }

      ScenarioEvaluationRequest req;
      req.plan.task_order = order; // task-order only; robot pref left to greedy
      req.label = kDqnLabel;
      req.parking_seed = rng();
      requests.push_back(std::move(req));
      batch_orders.push_back(std::move(order));
      batch_iters.push_back(iter + 1);
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

      ingest_rollout(batch_orders[b], candidate, seed_makespan, fail_return,
                     static_feats, active_constraints, replay);

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
    train_model(model, replay, options.dqn_learning_rate,
                options.dqn_grad_steps_per_iter, options.dqn_minibatch_size, rng);
  }

  if (out_enforced_constraint_count)
    *out_enforced_constraint_count = learn_order ? dqn_constraints.enforced_count : 0;

  std::cout << "[Search] DQN online allocation search done. best="
            << (outcome.has_feasible
                    ? (std::to_string(outcome.best_feasible.makespan) + "s")
                    : std::string("none"))
            << " failed=" << outcome.lns_failed_iterations << "/"
            << total_iterations << std::endl;

  return outcome;
}
