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

// Extracts per-task geometry from the reference solution (layer A -- needs
// the full FinalAllocation/EdgePath definitions, implemented in
// DqnFeaturesV2.cpp). Robots are homogeneous, so a single representative
// RobotMeta supplies the speeds used to compute tau_fixed.
std::vector<TaskGeom> extract_task_geoms(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RobotMeta &meta);

} // namespace DqnV2

#endif // DQN_FEATURES_V2_H
