#ifndef DQN_FEATURES_V2_H
#define DQN_FEATURES_V2_H

#include <PHAstarPushDemoTypes.h> // LearnedOrderConstraints
#include <PHAstar/Entities.h>     // RobotMeta, Pose, OccuRect
#include <ReloPush/State.h>       // ReloPush::State
#include <PHAstar/Reeds_Shepp.h>  // ReedShepp::reeds_shepp_path_planning

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

// ==========================================
// DQN feature-redesign v2: geometry + forward-schedule machinery.
// ==========================================
//
// See MARS/08dqn-feature-redesign.md for the design rationale. This header
// holds the pure geometry/schedule math (layer B): everything here operates
// on plain types (ReloPush::State, TaskGeom, RobotMeta) so it is directly
// unit-testable with synthetic data, without constructing FinalAllocation or
// EdgePath objects. The extraction of TaskGeom from FinalAllocation (layer A,
// which needs the heavy ReloPush headers) lives in DqnFeaturesV2.cpp.

struct FinalAllocation; // forward declaration; only extract_task_geoms() needs it

namespace DqnV2
{

constexpr std::size_t kFeatDim = 9;

// v3: explicit (task, robot) construction. Trimmed robot-conditional feature
// vector -- see make_feature_vector_v3.
constexpr std::size_t kFeatDimV3 = 10;

// v4: same (task, robot) action space and construct/ingest machinery as v3,
// with two additional schedule-state features appended -- see
// make_feature_vector_v4.
constexpr std::size_t kFeatDimV4 = 12;

// Confinement reference clearance (~robot full length 0.48 m): at or beyond
// this clearance from the workspace boundary, confinement weight bottoms
// out at 1.0.
constexpr double kClearRef = 0.5;
// Boundary-risk normalization reference (~transit turning radius).
constexpr double kBoundaryRef = 1.0;
// Object footprint half-diagonal, added to the blockage distance threshold.
constexpr double kObjHalfDiag = 0.11;
// Corridor waypoint subsampling spacing used both at extraction time and as
// the arclength element ds in corridor-overlap integration.
constexpr double kCorridorSampleSpacing = 0.1;
// Cap on learned-constraint evidence used to normalize the learned_risk
// feature. Deliberately independent of options.dqn_learned_hard_evidence,
// which only gates the mask.
constexpr double kLearnedRiskCap = 3.0;

struct WorkspaceBounds
{
  double xMin = 0.0;
  double xMax = 0.0;
  double yMin = 0.0;
  double yMax = 0.0;
};

// Per-task geometry extracted from the reference (seed) solution.
struct TaskGeom
{
  ReloPush::State approach;                 // first waypoint of the task's own work path
  ReloPush::State exit;                     // last waypoint of the task's last edge sub-path
  double tau_fixed = 0.0;                   // total busy time after arrival (seconds)
  std::vector<ReloPush::State> push_pts;    // waypoints of is_pushing sub-paths, subsampled
  std::vector<ReloPush::State> transit_pts; // waypoints of !is_pushing sub-paths, subsampled
  ReloPush::State obj_start;                // = FinalAllocation.startPose
  ReloPush::State obj_goal;                 // = FinalAllocation.goalPose
};

// Pairwise geometric structure over the reference-order-consistent DAG, built
// once per instance and reused by every rollout.
struct PairwiseGeometry
{
  std::vector<std::vector<std::size_t>> hard_pred; // hard_pred[a]: tasks that must precede a
  std::vector<std::vector<std::size_t>> soft_pred; // soft_pred[a]: transit-corridor predecessors of a
  std::vector<std::vector<double>> corr_overlap;   // corr_overlap[a][j]
  std::vector<std::vector<double>> w_start;        // w_start[j][a]
  std::vector<std::vector<double>> w_goal;         // w_goal[j][a]
  std::vector<double> boundary_risk;               // boundary_risk[a]

  // Workspace bounds and robot width this geometry was built against (the
  // same inputs build_pairwise_geometry already took as arguments, just also
  // retained here). Unused by v1/v2/v3; v4's idle_robot_congestion feature
  // needs them to call blockage_weight() against a candidate task's own
  // corridor rather than a precomputed pairwise matrix entry.
  WorkspaceBounds bounds;
  double robot_width = 0.0;
};

inline double dist2d(const ReloPush::State &a, const ReloPush::State &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

// Free-space clearance of a point against the workspace boundary. Clamped at
// 0 (points outside the boundary are treated as zero clearance, not negative).
inline double workspace_clearance(const ReloPush::State &p, const WorkspaceBounds &bounds)
{
  const double clr = std::min({p.x - bounds.xMin, bounds.xMax - p.x,
                               p.y - bounds.yMin, bounds.yMax - p.y});
  return std::max(0.0, clr);
}

// Confinement weight in [1, 2]: 1.0 in open space (clearance >= kClearRef),
// rising to 2.0 as clearance -> 0.
inline double confinement_weight(double clearance)
{
  return 1.0 + std::max(0.0, 1.0 - clearance / kClearRef);
}

// Minimum workspace clearance over a set of points; +inf if empty (caller
// decides the fallback).
inline double min_clearance(const std::vector<ReloPush::State> &pts, const WorkspaceBounds &bounds)
{
  double m = std::numeric_limits<double>::infinity();
  for (const auto &p : pts)
    m = std::min(m, workspace_clearance(p, bounds));
  return m;
}

// Whether pose q "blocks" a corridor (point set), and the confinement weight
// at the nearest corridor point if so (0.0 otherwise).
inline double blockage_weight(const ReloPush::State &q,
                              const std::vector<ReloPush::State> &corridor,
                              double block_dist, const WorkspaceBounds &bounds)
{
  if (corridor.empty())
    return 0.0;

  double min_dist = std::numeric_limits<double>::infinity();
  std::size_t nearest = 0;
  for (std::size_t i = 0; i < corridor.size(); ++i)
  {
    const double d = dist2d(q, corridor[i]);
    if (d < min_dist)
    {
      min_dist = d;
      nearest = i;
    }
  }
  if (min_dist < block_dist)
    return confinement_weight(workspace_clearance(corridor[nearest], bounds));
  return 0.0;
}

// Confinement-weighted fraction of a's corridor points within `robot_width`
// of b's corridor. 0 if either corridor is empty. ds is approximated as a
// constant sample spacing (matches the ~0.1 m subsampling at extraction time)
// rather than tracked true arclength across the push/transit concatenation.
inline double corridor_overlap_fraction(const std::vector<ReloPush::State> &a_pts,
                                        const std::vector<ReloPush::State> &b_pts,
                                        double robot_width,
                                        const WorkspaceBounds &bounds,
                                        double sample_spacing = kCorridorSampleSpacing)
{
  if (a_pts.empty() || b_pts.empty())
    return 0.0;

  double numerator = 0.0;
  for (const auto &p : a_pts)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const auto &q : b_pts)
      min_dist = std::min(min_dist, dist2d(p, q));
    if (min_dist < robot_width)
      numerator += confinement_weight(workspace_clearance(p, bounds)) * sample_spacing;
  }
  const double total_arclen = sample_spacing * static_cast<double>(a_pts.size());
  return total_arclen > 1e-9 ? numerator / total_arclen : 0.0;
}

// Builds the reference-order-consistent hard/soft DAG plus the congestion and
// boundary-risk matrices. Task index order (0..N-1) is taken as the reference
// (certified-feasible) order, so only edges x->y with x<y are ever created --
// the hard-edge graph is therefore acyclic by construction.
inline PairwiseGeometry build_pairwise_geometry(const std::vector<TaskGeom> &geoms,
                                                const WorkspaceBounds &bounds,
                                                double robot_width)
{
  const std::size_t n = geoms.size();
  PairwiseGeometry g;
  g.hard_pred.assign(n, {});
  g.soft_pred.assign(n, {});
  g.corr_overlap.assign(n, std::vector<double>(n, 0.0));
  g.w_start.assign(n, std::vector<double>(n, 0.0));
  g.w_goal.assign(n, std::vector<double>(n, 0.0));
  g.boundary_risk.assign(n, 0.0);
  g.bounds = bounds;
  g.robot_width = robot_width;

  const double block_dist = 0.5 * robot_width + kObjHalfDiag;

  std::vector<std::vector<ReloPush::State>> combined(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    combined[i].reserve(geoms[i].push_pts.size() + geoms[i].transit_pts.size());
    combined[i].insert(combined[i].end(), geoms[i].push_pts.begin(), geoms[i].push_pts.end());
    combined[i].insert(combined[i].end(), geoms[i].transit_pts.begin(), geoms[i].transit_pts.end());
  }

  for (std::size_t a = 0; a < n; ++a)
  {
    std::vector<ReloPush::State> boundary_pts = geoms[a].push_pts;
    boundary_pts.push_back(geoms[a].approach);
    double clr = min_clearance(boundary_pts, bounds);
    if (!std::isfinite(clr))
      clr = 0.0;
    g.boundary_risk[a] = std::clamp(1.0 - clr / kBoundaryRef, 0.0, 1.0);
  }

  for (std::size_t a = 0; a < n; ++a)
  {
    for (std::size_t j = 0; j < n; ++j)
    {
      if (j == a)
        continue;
      g.w_start[j][a] = blockage_weight(geoms[j].obj_start, combined[a], block_dist, bounds);
      g.w_goal[j][a] = blockage_weight(geoms[j].obj_goal, combined[a], block_dist, bounds);
      g.corr_overlap[a][j] = corridor_overlap_fraction(combined[a], combined[j], robot_width, bounds);
    }
  }

  for (std::size_t x = 0; x < n; ++x)
  {
    for (std::size_t y = x + 1; y < n; ++y)
    {
      const bool hard_xy =
          blockage_weight(geoms[x].obj_start, geoms[y].push_pts, block_dist, bounds) > 0.0 ||
          blockage_weight(geoms[y].obj_goal, geoms[x].push_pts, block_dist, bounds) > 0.0;
      if (hard_xy)
      {
        g.hard_pred[y].push_back(x);
      }
      else if (blockage_weight(geoms[x].obj_start, geoms[y].transit_pts, block_dist, bounds) > 0.0)
      {
        g.soft_pred[y].push_back(x);
      }
    }
  }

  return g;
}

// True iff task `a` is legal to insert now: every hard predecessor is placed,
// and no unplaced learned-precedence predecessor has reached the hard-evidence
// threshold. Learned edges below the threshold are not masked -- they only
// feed the learned_risk feature (see make_feature_vector_v2).
inline bool is_task_legal_v2(std::size_t a, const PairwiseGeometry &geometry,
                             const std::vector<char> &is_placed,
                             const LearnedOrderConstraints &constraints,
                             int hard_evidence_threshold)
{
  for (std::size_t p : geometry.hard_pred[a])
    if (!is_placed[p])
      return false;

  const std::size_t n = is_placed.size();
  if (constraints.enforced.size() == n)
  {
    for (std::size_t p = 0; p < n; ++p)
    {
      if (p == a || is_placed[p])
        continue;
      if (constraints.enforced[p].size() != n || !constraints.enforced[p][a])
        continue;
      const int evidence = (constraints.evidence_counts.size() == n &&
                            constraints.evidence_counts[p].size() == n)
                               ? constraints.evidence_counts[p][a]
                               : 0;
      if (evidence >= hard_evidence_threshold)
        return false;
    }
  }
  return true;
}

// Reeds-Shepp path length; falls back to Euclidean distance if start and goal
// coincide (avoids a degenerate RS call) or if no valid RS path is found.
inline double reeds_shepp_length(const ReloPush::State &from, const ReloPush::State &to,
                                 double max_curvature, double step_size, double wheel_base)
{
  if (dist2d(from, to) < 1e-6)
    return 0.0;

  auto result = ReedShepp::reeds_shepp_path_planning(
      from.x, from.y, from.yaw, to.x, to.y, to.yaw, max_curvature, step_size, wheel_base);
  const auto &lengths = std::get<4>(result);
  if (lengths.empty())
    return dist2d(from, to);

  double total = 0.0;
  for (double l : lengths)
    total += std::abs(l);
  return total;
}

using DistanceFn = std::function<double(const ReloPush::State &, const ReloPush::State &)>;

// Lightweight forward schedule simulation mirroring the executor's greedy
// robot-selection rule (find_earliest_robot: argmin free time, tie -> lowest
// index). preview() is a pure query (does not mutate state); commit() advances
// the chosen robot's free time and pose.
struct ForwardSim
{
  std::vector<double> free_time;
  std::vector<ReloPush::State> pose;
  std::vector<std::pair<double, double>> window; // per task index, valid once committed
  double speed_transit = 1.0;
  DistanceFn distance;

  ForwardSim(std::vector<ReloPush::State> initial_poses, double speed_transit_in,
            DistanceFn distance_fn, std::size_t task_count)
      : free_time(initial_poses.size(), 0.0),
        pose(std::move(initial_poses)),
        window(task_count, {0.0, 0.0}),
        speed_transit(speed_transit_in),
        distance(std::move(distance_fn))
  {
  }

  std::size_t pick() const
  {
    std::size_t best = 0;
    for (std::size_t r = 1; r < free_time.size(); ++r)
      if (free_time[r] < free_time[best])
        best = r;
    return best;
  }

  double max_free_time() const
  {
    double m = 0.0;
    for (double t : free_time)
      m = std::max(m, t);
    return m;
  }

  struct Preview
  {
    std::size_t robot = 0;
    double start = 0.0;
    double finish = 0.0;
  };

  Preview preview(const TaskGeom &geom) const
  {
    Preview pv;
    pv.robot = pick();
    const double tau_transit = speed_transit > 1e-9
                                    ? distance(pose[pv.robot], geom.approach) / speed_transit
                                    : 0.0;
    pv.start = free_time[pv.robot];
    pv.finish = pv.start + tau_transit + geom.tau_fixed;
    return pv;
  }

  void commit(std::size_t task_index, const TaskGeom &geom, const Preview &pv)
  {
    if (task_index < window.size())
      window[task_index] = {pv.start, pv.finish};
    free_time[pv.robot] = pv.finish;
    pose[pv.robot] = geom.exit;
  }

  // v3: preview committing `geom` to a SPECIFIED robot `r`, instead of
  // picking the earliest-available one via pick(). Used by the explicit
  // (task, robot) construction path; pick()/preview()/commit() above are
  // left untouched for the v1/v2 dispatcher-driven path.
  Preview preview_for(std::size_t r, const TaskGeom &geom) const
  {
    Preview pv;
    pv.robot = r;
    const double tau_transit = speed_transit > 1e-9
                                    ? distance(pose[r], geom.approach) / speed_transit
                                    : 0.0;
    pv.start = free_time[r];
    pv.finish = pv.start + tau_transit + geom.tau_fixed;
    return pv;
  }

  // v3: commit a preview_for()-produced preview. Equivalent to commit()
  // (which already only reads pv.robot, never re-derives it); kept as a
  // distinctly named entry point for the explicit-assignment path.
  void commit_for(std::size_t task_index, const TaskGeom &geom, const Preview &pv)
  {
    commit(task_index, geom, pv);
  }
};

// phi(state, candidate task a). `is_placed`/`committed_windows` reflect the
// state at construction/replay time; `pv`/`m_t_before` come from
// sim.preview(geoms[a]) and sim.max_free_time() taken before this candidate is
// committed.
inline std::vector<double> make_feature_vector_v2(
    std::size_t task,
    const ForwardSim::Preview &pv,
    double m_t_before,
    const TaskGeom &geom,
    const PairwiseGeometry &geometry,
    const std::vector<char> &is_placed,
    const std::vector<std::pair<double, double>> &committed_windows,
    const LearnedOrderConstraints &constraints,
    double seed_makespan,
    std::size_t task_count)
{
  std::vector<double> phi(kFeatDim, 0.0);
  const double S = seed_makespan > 1e-9 ? seed_makespan : 1.0;
  const std::size_t n = task_count;
  const double denom_n1 = n > 1 ? static_cast<double>(n - 1) : 1.0;

  phi[0] = 1.0;
  phi[1] = std::max(0.0, pv.finish - m_t_before) / S;
  phi[2] = (m_t_before - pv.start) / S;
  phi[3] = geom.tau_fixed / S;

  double cong_static = 0.0;
  for (std::size_t j = 0; j < n; ++j)
  {
    if (j == task)
      continue;
    cong_static += is_placed[j] ? geometry.w_goal[j][task] : geometry.w_start[j][task];
  }
  phi[4] = cong_static / (2.0 * denom_n1);

  double cong_dynamic = 0.0;
  const double window_span = std::max(pv.finish - pv.start, 1e-9);
  for (std::size_t j = 0; j < n; ++j)
  {
    if (j == task || !is_placed[j])
      continue;
    const auto &wj = committed_windows[j];
    const double overlap = std::min(pv.finish, wj.second) - std::max(pv.start, wj.first);
    if (overlap > 0.0)
      cong_dynamic += geometry.corr_overlap[task][j] * (overlap / window_span);
  }
  phi[5] = cong_dynamic;

  std::size_t dag_violations = 0;
  for (std::size_t p : geometry.soft_pred[task])
    if (!is_placed[p])
      ++dag_violations;
  phi[6] = static_cast<double>(dag_violations) / denom_n1;

  double learned_risk = 0.0;
  if (constraints.enforced.size() == n)
  {
    for (std::size_t p = 0; p < n; ++p)
    {
      if (p == task || is_placed[p])
        continue;
      if (constraints.enforced[p].size() != n || !constraints.enforced[p][task])
        continue;
      const double evidence = (constraints.evidence_counts.size() == n &&
                               constraints.evidence_counts[p].size() == n)
                                  ? static_cast<double>(constraints.evidence_counts[p][task])
                                  : kLearnedRiskCap;
      learned_risk += std::min(evidence, kLearnedRiskCap) / kLearnedRiskCap;
    }
  }
  phi[7] = learned_risk / denom_n1;

  phi[8] = geometry.boundary_risk[task];

  return phi;
}

namespace detail
{
// Shared dims [0..9] computation for make_feature_vector_v3 and
// make_feature_vector_v4 (v4 appends dims 10/11 on top of this same core).
// `phi` must already be sized to at least 10 entries; only indices 0..9 are
// written. Factored out verbatim from the original make_feature_vector_v3
// body so both callers stay numerically (indeed bit-for-bit) identical on
// these dims -- see the callers below for the parameter contract (`pv` from
// sim.preview_for(r, geoms[task]); `m_t_before`/`min_free_time_before` are
// state-level quantities shared by every candidate at this step).
inline void fill_v3_core_features(
    std::vector<double> &phi,
    std::size_t task,
    const ForwardSim::Preview &pv,
    double m_t_before,
    double min_free_time_before,
    const TaskGeom &geom,
    const PairwiseGeometry &geometry,
    const std::vector<char> &is_placed,
    const std::vector<std::pair<double, double>> &committed_windows,
    const LearnedOrderConstraints &constraints,
    double seed_makespan,
    std::size_t task_count)
{
  const double S = seed_makespan > 1e-9 ? seed_makespan : 1.0;
  const std::size_t n = task_count;
  const double denom_n1 = n > 1 ? static_cast<double>(n - 1) : 1.0;

  phi[0] = 1.0;
  phi[1] = std::max(0.0, pv.finish - m_t_before) / S;
  phi[2] = std::max(0.0, m_t_before - pv.finish) / S;
  // tau_transit = pv.finish - pv.start - geom.tau_fixed (same RS length /
  // transit speed quantity preview_for() folds into pv.finish).
  phi[3] = ((pv.finish - pv.start) - geom.tau_fixed) / S;
  phi[4] = (pv.start - min_free_time_before) / S;

  double cong_static = 0.0;
  for (std::size_t j = 0; j < n; ++j)
  {
    if (j == task)
      continue;
    cong_static += is_placed[j] ? geometry.w_goal[j][task] : geometry.w_start[j][task];
  }
  phi[5] = cong_static / (2.0 * denom_n1);

  double cong_dynamic = 0.0;
  const double window_span = std::max(pv.finish - pv.start, 1e-9);
  for (std::size_t j = 0; j < n; ++j)
  {
    if (j == task || !is_placed[j])
      continue;
    const auto &wj = committed_windows[j];
    const double overlap = std::min(pv.finish, wj.second) - std::max(pv.start, wj.first);
    if (overlap > 0.0)
      cong_dynamic += geometry.corr_overlap[task][j] * (overlap / window_span);
  }
  phi[6] = cong_dynamic;

  phi[7] = geometry.boundary_risk[task];

  std::size_t dag_violations = 0;
  for (std::size_t p : geometry.soft_pred[task])
    if (!is_placed[p])
      ++dag_violations;
  phi[8] = static_cast<double>(dag_violations) / denom_n1;

  double learned_risk = 0.0;
  if (constraints.enforced.size() == n)
  {
    for (std::size_t p = 0; p < n; ++p)
    {
      if (p == task || is_placed[p])
        continue;
      if (constraints.enforced[p].size() != n || !constraints.enforced[p][task])
        continue;
      const double evidence = (constraints.evidence_counts.size() == n &&
                               constraints.evidence_counts[p].size() == n)
                                  ? static_cast<double>(constraints.evidence_counts[p][task])
                                  : kLearnedRiskCap;
      learned_risk += std::min(evidence, kLearnedRiskCap) / kLearnedRiskCap;
    }
  }
  phi[9] = learned_risk / denom_n1;
}
} // namespace detail

// phi(state, candidate (task, robot) pair) for the v3 explicit-assignment
// policy. Reuses the same congestion/DAG/boundary/learned-risk math as
// make_feature_vector_v2 above (identical helper calls), but the schedule
// terms (dmk+, slack, transit, rel_avail) are conditional on the CHOSEN
// robot rather than the dispatcher-previewed earliest-available one:
// `pv` must come from sim.preview_for(r, geoms[task]) for the candidate
// robot r (pv.robot == r); `m_t_before`/`min_free_time_before` are
// sim.max_free_time()/sim.free_time[sim.pick()] taken before this
// candidate is committed -- state-level quantities shared by every
// candidate considered at this step, not recomputed per candidate. No
// origin-deviation-style feature, is-earliest flag, or load-imbalance
// feature (v2 has none of these either); see DqnAllocationSearch.cpp
// construct_order_v3 for how candidates are enumerated.
inline std::vector<double> make_feature_vector_v3(
    std::size_t task,
    const ForwardSim::Preview &pv,
    double m_t_before,
    double min_free_time_before,
    const TaskGeom &geom,
    const PairwiseGeometry &geometry,
    const std::vector<char> &is_placed,
    const std::vector<std::pair<double, double>> &committed_windows,
    const LearnedOrderConstraints &constraints,
    double seed_makespan,
    std::size_t task_count)
{
  std::vector<double> phi(kFeatDimV3, 0.0);
  detail::fill_v3_core_features(phi, task, pv, m_t_before, min_free_time_before, geom, geometry,
                                is_placed, committed_windows, constraints, seed_makespan,
                                task_count);
  return phi;
}

// phi(state, candidate (task, robot) pair) for the v4 explicit-assignment
// policy. Dims [0..9] are IDENTICAL to make_feature_vector_v3 (shared via
// detail::fill_v3_core_features above); this adds two schedule-state
// features that make_feature_vector_v3 has none of:
//
//   [10] load_imbalance: time-based, post-commit per-robot free-time spread
//        (max - min)/S after hypothetically committing this candidate. Pure
//        schedule-state exposure -- no baked-in "balanced is better"
//        assumption; the model learns the sign. `free_time_before` is
//        sim.free_time taken before this candidate is committed (state-level,
//        shared by every candidate at this step, like m_t_before); pv.robot's
//        entry is replaced by pv.finish to reflect the hypothetical commit.
//   [11] idle_robot_congestion: parked-robot blockage of the candidate's own
//        combined push+transit corridor during its [pv.start, pv.finish)
//        execution window, time-weighted by temporal overlap and normalized
//        by (robot_count - 1) so it stays a congestion measure rather than a
//        fleet-size proxy. A robot with free_time_before >= pv.finish is
//        busy executing elsewhere for the whole window (the dynamic
//        congestion channel's business, dim 6 -- not this one's) and is
//        skipped. `pose_before` is sim.pose taken before this candidate is
//        committed, parallel to `free_time_before`.
//
// `geometry.bounds`/`geometry.robot_width` (see PairwiseGeometry) supply the
// workspace bounds and block_dist inputs blockage_weight() needs, matching
// exactly what build_pairwise_geometry used to build `geometry` itself.
inline std::vector<double> make_feature_vector_v4(
    std::size_t task,
    const ForwardSim::Preview &pv,
    double m_t_before,
    double min_free_time_before,
    const TaskGeom &geom,
    const PairwiseGeometry &geometry,
    const std::vector<char> &is_placed,
    const std::vector<std::pair<double, double>> &committed_windows,
    const LearnedOrderConstraints &constraints,
    double seed_makespan,
    std::size_t task_count,
    const std::vector<double> &free_time_before,
    const std::vector<ReloPush::State> &pose_before)
{
  std::vector<double> phi(kFeatDimV4, 0.0);
  detail::fill_v3_core_features(phi, task, pv, m_t_before, min_free_time_before, geom, geometry,
                                is_placed, committed_windows, constraints, seed_makespan,
                                task_count);

  const double S = seed_makespan > 1e-9 ? seed_makespan : 1.0;

  // [10] load_imbalance.
  double max_f = -std::numeric_limits<double>::infinity();
  double min_f = std::numeric_limits<double>::infinity();
  for (std::size_t r = 0; r < free_time_before.size(); ++r)
  {
    const double f = (r == pv.robot) ? pv.finish : free_time_before[r];
    max_f = std::max(max_f, f);
    min_f = std::min(min_f, f);
  }
  phi[10] = (std::isfinite(max_f) && std::isfinite(min_f)) ? (max_f - min_f) / S : 0.0;

  // [11] idle_robot_congestion.
  double idle_cong = 0.0;
  const double window_span = pv.finish - pv.start;
  if (window_span > 1e-9)
  {
    std::vector<ReloPush::State> corridor;
    corridor.reserve(geom.push_pts.size() + geom.transit_pts.size());
    corridor.insert(corridor.end(), geom.push_pts.begin(), geom.push_pts.end());
    corridor.insert(corridor.end(), geom.transit_pts.begin(), geom.transit_pts.end());
    const double block_dist = 0.5 * geometry.robot_width + kObjHalfDiag;

    for (std::size_t r = 0; r < free_time_before.size(); ++r)
    {
      if (r == pv.robot || r >= pose_before.size())
        continue;
      if (free_time_before[r] >= pv.finish)
        continue; // busy executing elsewhere for the whole window
      const double overlap_start = std::max(pv.start, free_time_before[r]);
      const double overlap = pv.finish - overlap_start;
      if (overlap <= 0.0)
        continue;
      const double w = blockage_weight(pose_before[r], corridor, block_dist, geometry.bounds);
      idle_cong += w * (overlap / window_span);
    }
  }
  const std::size_t robot_count = free_time_before.size();
  const double denom_r1 = robot_count > 1 ? static_cast<double>(robot_count - 1) : 1.0;
  phi[11] = idle_cong / denom_r1;

  return phi;
}

// Extracts per-task geometry from the reference solution (layer A -- needs
// the full FinalAllocation/EdgePath definitions, implemented in
// DqnFeaturesV2.cpp). Robots are homogeneous, so a single representative
// RobotMeta supplies the speeds used to compute tau_fixed.
std::vector<TaskGeom> extract_task_geoms(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RobotMeta &meta);

} // namespace DqnV2

#endif // DQN_FEATURES_V2_H
