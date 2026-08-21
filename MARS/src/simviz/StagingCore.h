#pragma once

// STAGING PHASE: pure planning/decision math for driving each robot from
// its ACTUAL current pose to its planner-assumed start pose before a
// planned scenario begins executing (real robots never start exactly where
// the plan assumes; this is a no-op in the sim/localization-perfect case
// today and unchanged once a real mocap bridge is wired up -- see
// SimVizCore.h's LocalizationListener::has_fresh_pose()).
//
// Everything in this header is Qt-free and touches no ZMQ/QProcess state --
// unit-testable in-process (see MARS/tests/test_sim_viz_staging.cpp).
// Orchestration (deciding WHEN to call these, spawning the single-robot
// mini-executions, polling for completion, retrying) lives in
// SimVizManager (SimVizCore.h/.cpp), since that needs ExecutionManager/
// LocalizationListener.
//
// DEVIATIONS FROM THE ORIGINAL INTEGRATION NOTES (discovered while reading
// the actual PHAStar/TimeTable headers under include/PHAstar/):
//
//   1. PHAStar's constructor does NOT take an explicit start pose -- it
//      reads `r->initial_pose` off the RobotMeta* directly (PHAstar.h:1465:
//      "start = std::make_unique<Node>(r->initial_pose.x, ...)"). Likewise
//      TimeTable::add_initial() seeds every entity from ITS OWN
//      initial_pose field, not a pose passed alongside it
//      (TimeTable.h:31-37: "per_entity_table[ent][0.0] = ent->initial_pose").
//      There is no "pass an explicit current pose" overload of either API.
//      plan_staging_leg() therefore temporarily overwrites initial_pose on
//      the staged RobotMeta* and on every `others[i].robot` for the
//      duration of planning, then restores every original value via an
//      RAII guard (even on an exception) -- the caller's real
//      ScenarioModel/entities are never left mutated. This mutate-then-
//      restore is why plan_staging_leg() takes RobotMeta* (needs a
//      write-through pointer into the real entity), not a copy.
//
//   2. "allow-reverse" is NOT a flag to pass: PHAStar's motion-primitive set
//      already includes reverse ({-1, steer}) primitives whenever
//      is_transfer=false (PHAstar.h's ctor, the `else` branch of the
//      is_transfer check) -- exactly the mode staging always plans in
//      (never carrying an object). No extra option exists or is needed.
//
//   3. Params has no additive (metres) margin knob for robot-vs-robot
//      clearance -- only two MULTIPLICATIVE factors, `inflation` and
//      `robot_collision_inflation` (see CollisionUtils.h's
//      collision_inflation_for_type(): "params.inflation *
//      params.robot_collision_inflation" for ROBOT-type entities).
//      plan_staging_leg() converts `margin_m` into an ADDITIVE bump on
//      robot_collision_inflation sized against each other robot's own
//      collision_origin_radius() (its farthest-corner-from-origin radius),
//      so the inflated footprint grows by approximately margin_m metres at
//      its extremal corner: robot_collision_inflation += margin_m / radius.
//      This is an approximation (the grown amount varies slightly by
//      direction since inflation scales the whole rectangle uniformly, not
//      just the margin band) but is the closest match to "extra clearance
//      of margin_m metres" the existing collision-geometry API supports
//      without changing CollisionUtils.h (out of this task's allowed edit
//      scope).

#include <PHAstar/Entities.h>
#include <PHAstar/Params.h>
#include <PHAstar/PlanningResult.h>
#include <PHAstar/TimeTable.h>
#include <ReloPush/trajectory.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace staging
{

// ---------------------------------------------------------------------
// staging_decision
// ---------------------------------------------------------------------

enum class Decision
{
  Skip, // current is already within tolerance of start -- no staging leg needed.
  Stage // current is far enough from start that a staging leg should run.
};

// Boundary is inclusive: a pose exactly AT pos_tol_m/yaw_tol_rad counts as
// Skip (">" not ">=" triggers Stage). `current`/`start` are compared in the
// world frame; yaw error is wrapped to (-pi, pi] before comparing against
// yaw_tol_rad.
Decision staging_decision(const Pose &current, const Pose &start, double pos_tol_m,
                           double yaw_tol_rad);

// ---------------------------------------------------------------------
// TEST-ONLY presence/offset helpers (pure -- see simviz_main.cpp's
// --staging-test-offsets= flag and SimVizConfig::staging_test_offsets for
// how these are actually wired in).
// ---------------------------------------------------------------------

struct TestOffset
{
  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;
};

// Parses "robot1:dx,dy,dyaw;robot2:dx,dy,dyaw" (whitespace-tolerant around
// ';'/':'/',' separators). A malformed entry (wrong field count, unparsable
// number) is skipped -- this function is pure and never logs; a caller that
// wants a diagnostic should compare `parse_staging_test_offsets(spec).size()`
// against the number of ';'-separated tokens itself.
std::unordered_map<std::string, TestOffset> parse_staging_test_offsets(const std::string &spec);

// World-frame offset application: result = (start.x + dx, start.y + dy,
// wrap(start.yaw + dyaw)).
Pose apply_test_offset(const Pose &start, const TestOffset &offset);

struct PresenceResult
{
  std::vector<std::string> present; // order preserved from `all_robots_sorted`.
  std::vector<std::string> missing; // order preserved from `all_robots_sorted`.
};

// TEST-ONLY presence rule: a robot is PRESENT iff it has an entry in
// `offsets`; every other robot in `all_robots_sorted` is MISSING. The
// "hardware mode" presence rule (a robot is present iff it has published a
// fresh localization pose within SimVizConfig::staging_localization_wait_s)
// has no pure equivalent -- it needs a live LocalizationListener -- and is
// implemented directly in SimVizManager (SimVizCore.cpp).
PresenceResult filter_presence_by_test_offsets(
    const std::vector<std::string> &all_robots_sorted,
    const std::unordered_map<std::string, TestOffset> &offsets);

// ---------------------------------------------------------------------
// plan_staging_leg / build_staging_trajectory
// ---------------------------------------------------------------------

// One other (present) robot to treat as a static blocking obstacle while
// planning `robot`'s staging leg, at ITS current best-known pose (measured/
// test-offset for a not-yet-staged robot, its own start pose for an
// already-staged/verified one -- the caller, SimVizManager::tick_staging(),
// decides which).
struct OtherRobot
{
  RobotMeta *robot = nullptr;
  Pose current_pose;
};

struct StagingLegPlan
{
  bool success = false;
  std::vector<Waypoint> waypoints;
  PlanningStatus status = PlanningStatus::INTERNAL_ERROR;
  std::string failure_detail;
};

// Plans a path for `robot` from `current` to `target` (the robot's
// planner-assumed start pose), treating every entry of `others` as a
// static (non-moving) obstacle at its given current_pose -- no objects are
// added (staging assumes the workspace is otherwise as the scenario
// describes it, minus in-flight object state, which the caller is expected
// to not need for a pre-execution staging leg). `base_params` supplies the
// workspace rect (min_x/max_x/min_y/max_y) and every other planner tuning
// knob (xy_resolution, time_step, ...) -- typically the scenario's own
// Params, i.e. ScenarioModel::params(). `margin_m` is added as extra
// robot-vs-robot clearance (see this header's DEVIATION 3 above); pass 0.0
// for none.
//
// robots DO block (ignore_other_robots is left at its default false) --
// the entire point of staging is to route around them. is_transfer is
// always false (never carrying an object), which per DEVIATION 2 above
// also means the planner's native reverse-motion primitives are available.
StagingLegPlan plan_staging_leg(RobotMeta *robot, const Pose &current, const Pose &target,
                                 const std::vector<OtherRobot> &others, const Params &base_params,
                                 double margin_m);

// Builds a ReloPush::trajectory from a successful plan_staging_leg()
// result via the SAME production pipeline MARS/include/RobotTrajectoryBuilder.h
// uses for the main scenario (constant-speed_transit reparameterization,
// signed ref_vel through any reversal, rel_t starting at 0): waypoints ->
// Trajectory(robot, nullptr, 0.0, waypoints, /*is_transfer=*/false) ->
// CalcualteTimeStamps(robot) -> a fresh single-entity TimeTable ->
// build_robot_trajectories(). Returns a default-constructed (empty)
// trajectory if `robot` is null or `waypoints` is empty.
ReloPush::trajectory build_staging_trajectory(RobotMeta *robot,
                                               const std::vector<Waypoint> &waypoints);

} // namespace staging
