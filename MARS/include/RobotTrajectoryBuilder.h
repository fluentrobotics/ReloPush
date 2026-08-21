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

// A segment (between two consecutive waypoint samples) is classified as a
// HOLD when its average speed -- displacement over dt, NOT raw displacement
// alone, since sample spacing across a plan varies widely -- falls below
// this. 0.02 m/s is comfortably below any real transit/transfer cruise
// speed (kDefaultSpeedTransit/kDefaultSpeedTransfer above, and every real
// RobotMeta::speed_transit/speed_transfer) and comfortably above pose-map
// floating-point noise for two samples that are genuinely the same pose.
// See the ref_vel hold-zeroing doc comment below for how this is used.
constexpr double kHoldSpeedThreshold = 0.02;
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
// HOLD-SEGMENT ZEROING: a waypoint is treated as belonging to a HOLD when
// EITHER the segment to its next sample OR the segment from its previous
// sample has average speed (displacement/dt) below
// RobotTrajectoryBuilder::kHoldSpeedThreshold (~0.02 m/s) -- i.e. the robot's
// pose is effectively unchanged across that segment, however long it spans
// in wall-clock time. Such a waypoint gets ref_vel forced to 0.0f, instead
// of dir_sign * (transfer-or-transit speed) like every other waypoint. A
// genuine multi-second wait in a real plan produces exactly ONE such
// segment, bracketed by exactly two samples: the last sample before the
// wait begins and the first sample once motion resumes (both at the same
// pose) -- e.g. a 9.5s same-pose gap between a forward transit leg and a
// reversing transfer leg. Pre-fix, only dir_sign was carried across that
// gap (never a zero), so those two bracketing waypoints kept whatever
// nonzero speed label their OWN neighboring moving leg had (e.g. +0.2
// before, -0.15 after), and get_ref_state_at_time's linear interpolation
// (see MPC's MpcCore.cpp) swept a continuous nonzero ref_vel
// across the ENTIRE hold -- never reading zero except at one transient
// crossing instant -- even though the reference is not moving at all.
// Forcing BOTH bracketing waypoints to 0.0f fixes that: the last moving
// segment before the hold now interpolates as a natural decel ramp down to
// 0 (rather than an instantaneous cliff into a nonzero-labeled hold), the
// first moving segment after the hold interpolates as a natural accel ramp
// up from 0, and the hold segment itself interpolates flat at exactly 0.0
// throughout -- which is what lets the controller's STOP-SNAP/park
// machinery (see MPC's LaunchGovernor) actually engage and stay
// engaged for the hold's full duration instead of fighting a spurious
// "reference wants motion" signal. This is the production path
// (build_robot_trajectories feeds both the real MPC pipeline and
// mars_sim_viz -- see this file's own top-of-file comment), so real robots
// get exactly this semantics, not just the simulator. dir_sign itself is
// unaffected by hold-zeroing (it still carries forward exactly as described
// above); only the final ref_vel magnitude is overridden. A degenerate
// duplicate-timestamp sample pair (dt <= 0) is never classified as a hold
// by this check (would require dividing by a non-positive dt) -- it falls
// through to whatever the displacement-only dir_sign carry-forward above
// already does for it, unchanged from pre-fix behavior. is_pushing is also
// unaffected -- it is still computed purely from the transfer spans,
// independent of ref_vel.
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
        // Whether the segment [i-1, i] (the one ending at the CURRENT
        // waypoint) was a hold -- carried from the previous iteration's
        // next_segment_is_hold, mirroring how dir_sign is carried forward.
        // See the HOLD-SEGMENT ZEROING header comment above.
        bool prev_segment_was_hold = false;
        for (size_t i = 0; i < samples.size(); ++i)
        {
          const double t = samples[i].first;
          const Pose &pose = samples[i].second;

          bool next_segment_is_hold = false;
          if (i + 1 < samples.size())
          {
            const double next_t = samples[i + 1].first;
            const Pose &next_pose = samples[i + 1].second;
            const double dx = next_pose.x - pose.x;
            const double dy = next_pose.y - pose.y;
            const double displacement = std::hypot(dx, dy);
            if (displacement > 1e-6)
            {
              const double fwd_component = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
              dir_sign = (fwd_component >= 0.0) ? 1.0 : -1.0;
            }
            // else: displacement too small to trust -- keep the previous
            // segment's dir_sign.

            const double dt = next_t - t;
            if (dt > 0.0 && (displacement / dt) < RobotTrajectoryBuilder::kHoldSpeedThreshold)
            {
              next_segment_is_hold = true;
            }
            // else (dt <= 0, a degenerate duplicate-timestamp sample): not
            // classified as a hold via this check -- see header comment.
          }
          // else (last waypoint): no lookahead available -- keep whatever
          // dir_sign the previous segment carried; next_segment_is_hold
          // stays false (only prev_segment_was_hold can still zero it).

          const bool is_hold_waypoint = next_segment_is_hold || prev_segment_was_hold;
          prev_segment_was_hold = next_segment_is_hold;

          const float rel_t = static_cast<float>(t - first_t);
          const bool is_push = is_pushing_at_time(robot, t);
          const float ref_vel =
              is_hold_waypoint
                  ? 0.0f
                  : static_cast<float>(dir_sign * (is_push ? speed_transfer : speed_transit));

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
