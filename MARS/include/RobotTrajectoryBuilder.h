#pragma once

#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>
#include <ReloPush/trajectory.hpp>

#include <cmath>
#include <utility>
#include <vector>

// Factored out of MARS/src/SearchOrchestrator.cpp's run_on_robots_pipeline
// (per-robot trajectory construction + is_pushing_at_time, originally inline
// at ~994-1006/1096-1133) so both the physical/simulated-robot MPC pipeline
// (run_on_robots_pipeline) and mars_sim_viz (Phase B) share exactly one
// implementation. Header-only: a pure function of `timetable`, no I/O, no
// side effects, so its behavior is trivially the single source of truth for
// both callers -- see MARS/tests/test_sim_viz_handoff.cpp's trajectory-
// builder parity test.
namespace RobotTrajectoryBuilder
{
// Reference-velocity fallback used when an entity in `robots_sorted_by_name`
// is not (for whatever reason) a RobotMeta. Matches the historical
// hardcoded fallback that lived inline in run_on_robots_pipeline before this
// factoring, so behavior is unchanged.
constexpr double kDefaultSpeedTransit = 0.2;
constexpr double kDefaultSpeedTransfer = 0.15;
} // namespace RobotTrajectoryBuilder

// Builds one ReloPush::trajectory per entry in `robots_sorted_by_name`
// (typically every ROBOT entity in `timetable`, sorted by name for
// determinism -- the caller owns collection/sorting, matching
// run_on_robots_pipeline's existing convention of also needing that same
// sorted list for port assignment/process spawning).
//
// For each robot: one waypoint per (time, pose) sample in that robot's
// timetable row (TimeTable::get_database()), with rel_t = t - <that robot's
// own first sample time> (so every robot's trajectory starts at rel_t=0, as
// the MPC controller wire protocol expects regardless of when that robot's
// segment actually starts in the shared timetable clock), is_pushing = true
// iff t falls inside one of that robot's TRANSFER trajectory spans (see
// TimeTable::TrajectorySpan / TimeTable::get_trajectory_spans(), 1e-4
// endpoint tolerance), and ref_vel = dir_sign * (is_pushing ?
// RobotMeta::speed_transfer : RobotMeta::speed_transit) (or the kDefault*
// fallbacks above if the entity does not dynamic_cast to RobotMeta*).
//
// dir_sign encodes whether this leg of the plan is a forward or a REVERSING
// maneuver: it is the sign of the forward-heading component of the
// displacement to the NEXT waypoint, cos(yaw)*dx + sin(yaw)*dy (yaw taken at
// the CURRENT waypoint), i.e. +1 when the next waypoint lies ahead of the
// current heading and -1 when it lies behind it (a genuine reversal). The
// last waypoint (no "next" to look ahead to) and any waypoint whose
// displacement to the next one is too small to trust (<1e-6) carry forward
// the previous segment's sign instead of flipping spuriously. Downstream
// consumers (mpc::MPCCostFunctor's velocity-tracking residual, mpc::
// resolve_dir_change()) rely on this sign to detect and react to reversals
// -- a trajectory that is always ref_vel>=0 through an actual reversal
// leaves those consumers with no signal that a direction change is
// required.
//
// A robot absent from the timetable's database, or present with an empty
// pose map, yields an empty trajectory (0 waypoints) at that robot's
// position in the result -- never a thrown exception or a skipped entry, so
// the result always has exactly robots_sorted_by_name.size() elements in
// the same order.
inline std::vector<std::pair<EntityMeta *, ReloPush::trajectory>>
build_robot_trajectories(const TimeTable &timetable,
                          const std::vector<EntityMeta *> &robots_sorted_by_name)
{
  std::vector<std::pair<EntityMeta *, ReloPush::trajectory>> result;
  result.reserve(robots_sorted_by_name.size());

  const auto &spans = timetable.get_trajectory_spans();
  const auto &db = timetable.get_database();

  auto is_pushing_at_time = [&spans](EntityMeta *robot, double t)
  {
    for (const auto &span : spans)
    {
      if (span.entity == robot && span.is_transfer)
      {
        if (t >= (span.start_time - 1e-4) && t <= (span.end_time + 1e-4))
        {
          return true;
        }
      }
    }
    return false;
  };

  for (EntityMeta *robot : robots_sorted_by_name)
  {
    ReloPush::trajectory rp_traj;
    rp_traj.time_zero = 0.0f;

    if (robot)
    {
      RobotMeta *robot_meta = dynamic_cast<RobotMeta *>(robot);
      const double speed_transit = robot_meta
                                        ? robot_meta->speed_transit
                                        : RobotTrajectoryBuilder::kDefaultSpeedTransit;
      const double speed_transfer = robot_meta
                                         ? robot_meta->speed_transfer
                                         : RobotTrajectoryBuilder::kDefaultSpeedTransfer;

      const auto db_it = db.find(robot);
      if (db_it != db.end() && !db_it->second.empty())
      {
        const auto &path_map = db_it->second;
        // Indexable copy (path_map is a std::map, so this is already
        // time-sorted) so each waypoint can look ahead to the next one when
        // deciding its direction sign.
        const std::vector<std::pair<double, Pose>> samples(path_map.begin(), path_map.end());
        const double first_t = samples.front().first;

        double dir_sign = 1.0; // carried across waypoints; see header comment.
        for (size_t i = 0; i < samples.size(); ++i)
        {
          const double t = samples[i].first;
          const Pose &pose = samples[i].second;

          if (i + 1 < samples.size())
          {
            const Pose &next_pose = samples[i + 1].second;
            const double dx = next_pose.x - pose.x;
            const double dy = next_pose.y - pose.y;
            if (std::hypot(dx, dy) > 1e-6)
            {
              const double fwd_component = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
              dir_sign = (fwd_component >= 0.0) ? 1.0 : -1.0;
            }
            // else: displacement too small to trust -- keep the previous
            // segment's dir_sign.
          }
          // else (last waypoint): no lookahead available -- keep whatever
          // dir_sign the previous segment carried.

          const float rel_t = static_cast<float>(t - first_t);
          const bool is_push = is_pushing_at_time(robot, t);
          const float ref_vel =
              static_cast<float>(dir_sign * (is_push ? speed_transfer : speed_transit));

          ReloPush::trajectory_elem elem(
              static_cast<float>(pose.x),
              static_cast<float>(pose.y),
              static_cast<float>(pose.yaw),
              ref_vel,
              rel_t,
              is_push);
          rp_traj.append_waypoint(elem);
        }
      }
    }

    result.emplace_back(robot, std::move(rp_traj));
  }

  return result;
}
