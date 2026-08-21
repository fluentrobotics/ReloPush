#include "StagingCore.h"

#include <PHAstar/CollisionUtils.h>
#include <PHAstar/PHAstar.h>
#include <RobotTrajectoryBuilder.h>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace staging
{

namespace
{

double wrap_pi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

// Trims leading/trailing ASCII whitespace -- --staging-test-offsets= is a
// hand-typed CLI value in test scripts, so tolerating "robot1 : 0.4, 0.1,
// 0.5" is cheap insurance.
std::string trim(const std::string &s)
{
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::vector<std::string> split(const std::string &s, char delim)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string tok;
  while (std::getline(ss, tok, delim))
    out.push_back(tok);
  return out;
}

} // namespace

Decision staging_decision(const Pose &current, const Pose &start, double pos_tol_m,
                           double yaw_tol_rad)
{
  const double pos_err = std::hypot(current.x - start.x, current.y - start.y);
  const double yaw_err = std::fabs(wrap_pi(current.yaw - start.yaw));
  if (pos_err <= pos_tol_m && yaw_err <= yaw_tol_rad)
    return Decision::Skip;
  return Decision::Stage;
}

std::unordered_map<std::string, TestOffset> parse_staging_test_offsets(const std::string &spec)
{
  std::unordered_map<std::string, TestOffset> out;
  for (const std::string &entry_raw : split(spec, ';'))
  {
    const std::string entry = trim(entry_raw);
    if (entry.empty())
      continue;
    const size_t colon = entry.find(':');
    if (colon == std::string::npos)
      continue;
    const std::string name = trim(entry.substr(0, colon));
    const std::string rest = entry.substr(colon + 1);
    const std::vector<std::string> fields = split(rest, ',');
    if (name.empty() || fields.size() != 3)
      continue;
    try
    {
      TestOffset off;
      off.dx = std::stod(trim(fields[0]));
      off.dy = std::stod(trim(fields[1]));
      off.dyaw = std::stod(trim(fields[2]));
      out[name] = off;
    }
    catch (const std::exception &)
    {
      continue; // malformed number -- skip this entry, pure function never logs (see header).
    }
  }
  return out;
}

Pose apply_test_offset(const Pose &start, const TestOffset &offset)
{
  return Pose(start.x + offset.dx, start.y + offset.dy, wrap_pi(start.yaw + offset.dyaw));
}

PresenceResult filter_presence_by_test_offsets(
    const std::vector<std::string> &all_robots_sorted,
    const std::unordered_map<std::string, TestOffset> &offsets)
{
  PresenceResult out;
  for (const std::string &name : all_robots_sorted)
  {
    if (offsets.find(name) != offsets.end())
      out.present.push_back(name);
    else
      out.missing.push_back(name);
  }
  return out;
}

StagingLegPlan plan_staging_leg(RobotMeta *robot, const Pose &current, const Pose &target,
                                 const std::vector<OtherRobot> &others, const Params &base_params,
                                 double margin_m)
{
  StagingLegPlan result;
  if (!robot)
  {
    result.failure_detail = "plan_staging_leg: null robot";
    return result;
  }

  // DEVIATION 1 (see StagingCore.h): PHAStar/TimeTable both read start
  // poses off EntityMeta::initial_pose directly -- there is no "pass an
  // explicit current pose" parameter on either API. Temporarily overwrite
  // initial_pose on `robot` and every other[i].robot, restoring every
  // original value via this RAII guard (even if planning throws).
  struct PoseRestore
  {
    EntityMeta *ent;
    Pose original;
  };
  std::vector<PoseRestore> restores;
  auto set_pose = [&](EntityMeta *ent, const Pose &p)
  {
    restores.push_back({ent, ent->initial_pose});
    ent->initial_pose = p;
  };
  struct Restorer
  {
    std::vector<PoseRestore> *r;
    ~Restorer()
    {
      // Reverse order so a robot appearing more than once (shouldn't
      // happen -- `others` is caller-deduplicated -- but stay correct
      // regardless) restores its ORIGINAL pose, not an intermediate one.
      for (auto it = r->rbegin(); it != r->rend(); ++it)
        it->ent->initial_pose = it->original;
    }
  } restorer{&restores};

  set_pose(robot, current);

  std::unordered_map<std::string, EntityMeta *> entities;
  entities[robot->name] = robot;
  for (const OtherRobot &o : others)
  {
    if (!o.robot)
      continue;
    set_pose(o.robot, o.current_pose);
    entities[o.robot->name] = o.robot;
  }

  // Fresh, robot-local TimeTable: add_initial() seeds every entity above
  // from the initial_pose we just set (DEVIATION 1) -- no objects are
  // added (staging assumes an object-free workspace, per this task's
  // spec).
  TimeTable tt(base_params.time_step > 0.0 ? base_params.time_step : 0.5);
  tt.add_initial(entities);

  Params p = base_params;
  // DEVIATION 3 (see StagingCore.h): convert an additive metres margin into
  // an additive bump on the MULTIPLICATIVE robot_collision_inflation
  // factor, sized against the largest other-robot footprint radius so the
  // inflated footprint grows by ~margin_m metres at its farthest corner.
  if (margin_m > 0.0)
  {
    double max_extra_factor = 0.0;
    for (const OtherRobot &o : others)
    {
      if (!o.robot)
        continue;
      const double radius = collision_origin_radius(o.robot->size, 1.0);
      if (radius > 1e-6)
        max_extra_factor = std::max(max_extra_factor, margin_m / radius);
    }
    if (max_extra_factor <= 0.0)
    {
      // No sized-others to calibrate against (e.g. a lone robot staging
      // with nobody else present) -- fall back to bumping the factor by
      // margin_m directly rather than applying no margin at all.
      max_extra_factor = margin_m;
    }
    p.robot_collision_inflation += max_extra_factor;
  }

  PHAStar planner(robot, target, &tt, &entities, p, /*trans=*/false, /*obj_name=*/"",
                   /*start_t=*/0.0);
  // ignore_other_robots defaults false -- robots DO block, the entire
  // point of staging (see StagingCore.h's class doc comment).
  PlanningResult res = planner.Planning_with_res();

  result.status = res.status;
  result.failure_detail = res.failure_detail;
  result.success = (res.status == PlanningStatus::SUCCESS) && !res.waypoints.empty();
  if (result.success)
    result.waypoints = res.waypoints;
  return result;
}

ReloPush::trajectory build_staging_trajectory(RobotMeta *robot,
                                               const std::vector<Waypoint> &waypoints)
{
  if (!robot || waypoints.empty())
    return ReloPush::trajectory{};

  Trajectory traj(robot, nullptr, /*time_start=*/0.0, waypoints, /*is_transfer=*/false);
  // Constant-speed_transit reparameterization -- see
  // RobotTrajectoryBuilder.h's doc comment / this file's own header
  // comment for why the search's own (variable-time) waypoint times are
  // discarded in favor of this, matching the production pipeline exactly.
  traj.CalcualteTimeStamps(robot);

  TimeTable tt{};
  tt.add_trajectory(traj);
  auto pairs = build_robot_trajectories(tt, {robot});
  return pairs.empty() ? ReloPush::trajectory{} : pairs.front().second;
}

} // namespace staging
