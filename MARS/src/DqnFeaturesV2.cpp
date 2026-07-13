/*****************************************************************
 * DQN feature-redesign v2: extraction of per-task geometry (layer A) from
 * FinalAllocation/EdgePath. The pure geometry/schedule math (layer B) lives
 * entirely in DqnFeaturesV2.h so it stays unit-testable without these heavy
 * ReloPush types. See DqnFeaturesV2.h and MARS/08dqn-feature-redesign.md.
 ******************************************************************/

#include <DqnFeaturesV2.h>
#include <ReloPush/TaskAllocation.hpp>

namespace DqnV2
{
namespace
{

// Appends a subsample of `path` (~`spacing` m apart, always keeping the first
// and last waypoint) to `out`.
void subsample_into(const ReloPush::StatePath &path, double spacing,
                    std::vector<ReloPush::State> &out)
{
  if (path.empty())
    return;

  out.push_back(path.front());
  double accum = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i)
  {
    accum += dist2d(path[i - 1], path[i]);
    if (accum >= spacing)
    {
      out.push_back(path[i]);
      accum = 0.0;
    }
  }
  if (path.size() > 1)
    out.push_back(path.back());
}

} // namespace

std::vector<TaskGeom> extract_task_geoms(
    const std::vector<FinalAllocation> &loaded_sequence,
    const RobotMeta &meta)
{
  const std::size_t n = loaded_sequence.size();
  std::vector<TaskGeom> geoms(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const FinalAllocation &fa = loaded_sequence[i];
    TaskGeom &g = geoms[i];
    g.obj_start = fa.startPose;
    g.obj_goal = fa.goalPose;

    bool have_first = false;
    ReloPush::State last_wp;
    bool have_last = false;

    // Note: EdgePath::getFirstWaypoint()/getLastWaypoint()/getLength() are not
    // const, but shared_ptr indirection (obsReloPaths, EdgeData::paths) does
    // not propagate const from the enclosing `const FinalAllocation&`, so
    // calling them here is well-formed.
    auto consume_subpath = [&](EdgePath &ep)
    {
      auto state_path = ep.toStatePath(kCorridorSampleSpacing);
      if (!state_path || state_path->empty())
        return;

      if (!have_first)
      {
        g.approach = ep.getFirstWaypoint();
        have_first = true;
      }
      last_wp = ep.getLastWaypoint();
      have_last = true;

      const double speed = ep.is_pushing ? meta.speed_transfer : meta.speed_transit;
      if (speed > 1e-9)
        g.tau_fixed += ep.getLength() / speed;

      subsample_into(*state_path, kCorridorSampleSpacing,
                     ep.is_pushing ? g.push_pts : g.transit_pts);
    };

    if (fa.obsReloPaths)
    {
      for (auto &ep : *fa.obsReloPaths)
        consume_subpath(ep);
    }
    for (const auto &edge : fa.paths)
    {
      for (const auto &ep_ptr : edge.paths)
      {
        if (ep_ptr)
          consume_subpath(*ep_ptr);
      }
    }

    if (have_last)
      g.exit = last_wp;
  }

  return geoms;
}

} // namespace DqnV2
