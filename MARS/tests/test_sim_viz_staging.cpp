// STAGING PHASE: standalone (non-ctest) test for mars_sim_viz's pre-run
// staging phase -- before executing a planned scenario, each PRESENT robot
// drives from its ACTUAL current pose to its planner-assumed start pose.
// Same standalone-binary style as test_sim_viz_stall.cpp/test_sim_viz_
// execute.cpp (which this file mirrors closely): spawns the REAL
// mars_sim_viz --headless binary + real mpc_controller/mpc_robot_sim
// children for the process parts; the StagingCore parts run fully
// in-process (no spawning at all).
//
// TEST PORT BLOCK: same block test_sim_viz_execute.cpp/test_sim_viz_pause.cpp/
// test_sim_viz_noise.cpp/test_sim_viz_stall.cpp use (this test never runs
// concurrently with them -- each standalone binary is invoked and fully
// reaped before the next): control = 45601, handshake = 45620+idx,
// vesc = 45640+idx, localization = 45660+idx.
//
// Parts:
//   1. staging_decision() boundary cases (in-process, no I/O).
//   2. plan_staging_leg() around a blocking inflated robot in a corridor
//      scenario where the direct line would collide: asserts a path is
//      found AND every waypoint is collision-free against the other
//      robot's inflated footprint (re-using CollisionUtils.h's own
//      check_entity_collision() -- the exact production collision test,
//      not an approximation), plus a margin>0 smoke check.
//   3. build_staging_trajectory() validity: rel_t starts at 0, monotone
//      times, constant |ref_vel| == speed_transit, signed through a
//      deliberate reversal.
//   4. Process test A (happy path): synthetic 2-robot scenario, headless
//      --stage-first --staging-test-offsets with robot1 offset (0.4m,
//      30deg) and robot2 exactly at its start pose (present, no leg
//      needed) -- asserts STATUS shows "STAGING robot1 (1/2)" while
//      robot1's leg runs, robot1's post-staging pose (read from the leg's
//      own CSV, which lives in a directory distinct from the main run's so
//      it is never overwritten) is within tolerance, then the main run
//      completes with BOTH robots DONE (final poses within tolerance of
//      the scenario's expected finals).
//   5. Process test B (missing-robot case): offsets list only robot1 ->
//      robot2 excluded from staging AND the main run -- asserts the main
//      run completes with robot1 only (no robot2.csv written under the
//      main run's directory) and that mars_sim_viz's stdout logs the
//      exclusion.

#include "SimVizCore.h"
#include "StagingCore.h"
#include "test_sim_viz_shared.h"

#include <PHAstar/CollisionUtils.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

bool DEBUG_VIS = false;

using namespace simviz_test;

namespace
{

constexpr int kControlPort = 45601;
constexpr int kHandshakePortStart = 45620;
constexpr int kVescPortStart = 45640;
constexpr int kLocPortStart = 45660;

bool g_overall_pass = true;

void check(bool cond, const std::string &what)
{
  if (!cond)
  {
    std::cerr << "[test] FAILED: " << what << std::endl;
    g_overall_pass = false;
  }
  else
  {
    std::cout << "[test] ok: " << what << std::endl;
  }
}

// A RobotMeta with real steering geometry (build_synthetic_scenario()'s
// fixture robots have wheel_base=0/min_turning_radius=0, i.e. straight-line
// -only -- fine for that fixture's tests, useless here since Part 2 needs a
// robot that can actually steer around an obstacle). Mirrors
// phastar_unit_tests.cpp's EntityStore::add_mars_runtime_robot() values.
std::unique_ptr<RobotMeta> make_steerable_robot(const std::string &name, const Pose &initial_pose)
{
  auto robot = std::make_unique<RobotMeta>();
  robot->name = name;
  robot->type = EntityType::ROBOT;
  robot->initial_pose = initial_pose;
  robot->size.front_length = 0.36;
  robot->size.rear_length = 0.12;
  robot->size.width = 0.275;
  robot->min_turning_radius = 1.02;
  robot->min_turning_radius_transit = 1.02;
  robot->min_turning_radius_transfer = 1.43;
  robot->wheel_base = 0.29;
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;
  return robot;
}

// Mirrors test_sim_viz_shared.h's build_synthetic_scenario() (same 2-robot/
// 1-object layout and makespan) but with STEERABLE robots (build_synthetic_
// scenario()'s fixture robots deliberately have wheel_base=min_turning_
// radius=0 -- fine for that fixture's straight-line-only tests, but that
// makes yaw UNCHANGEABLE: a robot with max_steer permanently 0 can never
// reorient, so it could never plan a staging leg that needs to turn, e.g.
// this test's robot1 offset that includes a 30deg yaw error). Also sets an
// explicit `params` (build_synthetic_scenario() leaves it default-
// constructed, i.e. a 5x5-ish box) sized to comfortably fit both robots'
// positions plus staging maneuvering room -- ONLY staging leg planning
// (StagingCore's PHAStar search) ever reads params; the main run itself
// just replays the hand-built trajectories below via mpc_controller/
// mpc_robot_sim, so this has no effect on the main run's own behavior.
SyntheticScenario build_staging_scenario(const std::string &label)
{
  SyntheticScenario out;
  ExecutedScenario &scn = out.scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 2;
  scn.summary.failed_tasks = 0;

  scn.params.min_x = -1.0;
  scn.params.max_x = 4.0;
  scn.params.min_y = -2.0;
  scn.params.max_y = 4.0;
  scn.params.xy_resolution = 0.1;
  scn.params.yaw_resolution = M_PI / 6.0;
  scn.params.time_step = 1.0;
  scn.params.time_resolution = scn.params.time_step;
  scn.params.max_time = 60.0;

  RobotMeta *robot1 = make_steerable_robot("robot1", Pose(0.0, 0.0, 0.0)).release();
  RobotMeta *robot2 = make_steerable_robot("robot2", Pose(0.0, 2.0, 0.0)).release();

  ObjectMeta *object1 = new ObjectMeta();
  object1->name = "object1";
  object1->type = EntityType::OBJECT;
  object1->initial_pose = Pose(1.36, 2.0, 0.0); // ahead of robot2's segment-B start
  object1->size = OccuRect{0.1, 0.1, 0.2};
  object1->goal_pose = Pose(2.76, 2.0, 0.0);

  scn.entities["robot1"] = robot1;
  scn.entities["robot2"] = robot2;
  scn.entities["object1"] = object1;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  // robot1: single transit segment, 2.2m @ 0.2 m/s = 11s (same as
  // build_synthetic_scenario()).
  {
    Pose p1(0.0, 0.0, 0.0);
    Pose p2(2.2, 0.0, 0.0);
    WaypointPath path = {Waypoint(p1), Waypoint(p2)};
    Trajectory traj(robot1, nullptr, 0.0, path, /*is_transfer=*/false);
    traj.CalcualteTimeStamps(robot1, 0.0);
    scn.timetable.add_trajectory(traj);
    out.robot1_final = p2;
  }

  // robot2: transit then transfer (same shape as build_synthetic_scenario()).
  {
    Pose p1(0.0, 2.0, 0.0);
    Pose p2(1.0, 2.0, 0.0);
    WaypointPath path = {Waypoint(p1), Waypoint(p2)};
    Trajectory traj(robot2, nullptr, 0.0, path, /*is_transfer=*/false);
    traj.CalcualteTimeStamps(robot2, 0.0);
    scn.timetable.add_trajectory(traj);

    Pose p3(1.0, 2.0, 0.0);
    Pose p4(2.4, 2.0, 0.0);
    WaypointPath path2 = {Waypoint(p3), Waypoint(p4)};
    Trajectory traj2(robot2, object1, 5.0, path2, /*is_transfer=*/true);
    traj2.CalcualteTimeStamps(robot2, 0.0);
    scn.timetable.add_trajectory(traj2);
    out.robot2_final = p4;
  }

  return out;
}

Params make_corridor_params()
{
  Params p;
  p.min_x = -1.0;
  p.max_x = 4.0;
  p.min_y = -2.0;
  p.max_y = 2.0;
  p.xy_resolution = 0.1;
  p.yaw_resolution = M_PI / 6.0;
  p.time_step = 1.0;
  p.time_resolution = p.time_step;
  p.max_time = 60.0;
  return p;
}

// ---------------------------------------------------------------------
// Part 1: staging_decision() boundary cases.
// ---------------------------------------------------------------------
void part1_staging_decision()
{
  std::cout << "\n[test] === Part 1: staging_decision() boundary cases ===" << std::endl;

  const double pos_tol = 0.05;
  const double yaw_tol = 0.15;
  const Pose start(1.0, 2.0, 0.5);

  check(staging::staging_decision(start, start, pos_tol, yaw_tol) == staging::Decision::Skip,
        "exact match -> Skip");

  // "Exactly at tolerance" is tested a hair BELOW the boundary (pos_tol -
  // 1e-9) rather than bit-exact equality: staging_decision()'s "<=" check
  // is on hypot()'s floating-point result, which for an exact pos_tol input
  // can round a few ULP high or low depending on the construction path --
  // this still exercises the inclusive boundary without being a test of
  // IEEE-754 rounding behavior.
  check(staging::staging_decision(Pose(1.0 + pos_tol - 1e-9, 2.0, 0.5), start, pos_tol, yaw_tol) ==
            staging::Decision::Skip,
        "position error just at (a hair below) tolerance (inclusive boundary) -> Skip");

  check(staging::staging_decision(Pose(1.0 + pos_tol + 1e-3, 2.0, 0.5), start, pos_tol, yaw_tol) ==
            staging::Decision::Stage,
        "position error just past tolerance -> Stage");

  check(staging::staging_decision(Pose(1.0, 2.0, 0.5 + yaw_tol - 1e-9), start, pos_tol, yaw_tol) ==
            staging::Decision::Skip,
        "yaw error just at (a hair below) tolerance (inclusive boundary) -> Skip");

  check(staging::staging_decision(Pose(1.0, 2.0, 0.5 + yaw_tol + 1e-3), start, pos_tol, yaw_tol) ==
            staging::Decision::Stage,
        "yaw error just past tolerance -> Stage");

  // Wrap-around: start.yaw near +pi, current.yaw near -pi -- the SHORT way
  // around is small even though the raw difference is ~2*pi.
  const Pose wrap_start(0.0, 0.0, 3.13);
  const Pose wrap_current(0.0, 0.0, -3.13);
  check(staging::staging_decision(wrap_current, wrap_start, pos_tol, yaw_tol) ==
            staging::Decision::Skip,
        "yaw wrap-around (+pi vs -pi, true difference small) -> Skip");

  // Combined: within position tolerance but yaw well outside -> Stage.
  check(staging::staging_decision(Pose(1.01, 2.0, 0.5 + 1.0), start, pos_tol, yaw_tol) ==
            staging::Decision::Stage,
        "position within tolerance but yaw far outside -> Stage");
}

// ---------------------------------------------------------------------
// Part 2: plan_staging_leg() around a blocking robot.
// ---------------------------------------------------------------------
void part2_plan_staging_leg()
{
  std::cout << "\n[test] === Part 2: plan_staging_leg() around a blocking robot ===" << std::endl;

  auto robot = make_steerable_robot("stager", Pose(0.0, 0.0, 0.0));
  auto blocker = make_steerable_robot("blocker", Pose(1.5, 0.0, 0.0));

  const Pose current(0.0, 0.0, 0.0);
  const Pose target(3.0, 0.0, 0.0); // straight line through blocker's pose.
  const Params base_params = make_corridor_params();

  std::vector<staging::OtherRobot> others;
  others.push_back(staging::OtherRobot{blocker.get(), blocker->initial_pose});

  // --- margin = 0.0: exact-collision check against base_params. ---
  const staging::StagingLegPlan plan =
      staging::plan_staging_leg(robot.get(), current, target, others, base_params, 0.0);
  check(plan.success, "plan_staging_leg finds a path around the blocking robot (status=" +
                           plan.failure_detail + ")");

  if (plan.success)
  {
    check(!plan.waypoints.empty(), "returned plan has at least one waypoint");

    // Faithful clearance check: re-use CollisionUtils.h's own
    // check_entity_collision() (the SAME function the planner itself uses
    // internally) against every waypoint, rather than an approximate
    // circle/rectangle re-derivation that could disagree with the
    // planner's own notion of "collision-free".
    bool any_collision = false;
    for (const Waypoint &wp : plan.waypoints)
    {
      const CollisionGeometry subject_geom =
          setup_collision_geometry_for_type(wp, EntityType::ROBOT, robot->size, base_params);
      const EntityCollisionResult res =
          check_entity_collision(subject_geom, wp, blocker.get(), blocker->initial_pose, base_params);
      if (res.has_collision)
      {
        any_collision = true;
        std::cerr << "[test]   waypoint (" << wp.x << "," << wp.y << "," << wp.yaw
                  << ") collides with blocker" << std::endl;
      }
    }
    check(!any_collision,
          "every waypoint in the staging leg is collision-free against the blocking robot "
          "(min clearance > 0)");

    // Prove the blocker actually forced a detour (rather than, say, the
    // planner ignoring it): the path must deviate from y=0 near the
    // blocker's x position.
    double max_abs_y_near_blocker = 0.0;
    for (const Waypoint &wp : plan.waypoints)
    {
      if (std::fabs(wp.x - blocker->initial_pose.x) < 0.5)
        max_abs_y_near_blocker = std::max(max_abs_y_near_blocker, std::fabs(wp.y));
    }
    check(max_abs_y_near_blocker > 0.15,
          "path detours away from y=0 near the blocker (got max|y|=" +
              to_str(max_abs_y_near_blocker) + "m)");
  }

  // Restoration check (DEVIATION 1): plan_staging_leg() must not leave
  // either RobotMeta's initial_pose mutated.
  check(robot->initial_pose.x == 0.0 && robot->initial_pose.y == 0.0,
        "plan_staging_leg restores the staged robot's initial_pose afterward");
  check(blocker->initial_pose.x == 1.5 && blocker->initial_pose.y == 0.0,
        "plan_staging_leg restores the other robot's initial_pose afterward");

  // --- margin > 0.0: smoke check -- still finds a (wider) detour. Modest
  // value (0.1m, not e.g. 0.3m): DEVIATION 3's margin->inflation
  // conversion bumps robot_collision_inflation, which inflates BOTH the
  // blocker's AND the staged robot's OWN footprint (collision_inflation_
  // for_type() applies to any ROBOT-type subject, not just obstacles) --
  // too large a value in this compact 4x6m test workspace can make the
  // corridor genuinely infeasible within the search's iteration budget,
  // which is a workspace-sizing artifact of this test, not a
  // plan_staging_leg defect (Part 2's own margin=0.0 case above already
  // proves the core collision/detour behavior).
  const staging::StagingLegPlan plan_margin =
      staging::plan_staging_leg(robot.get(), current, target, others, base_params, 0.1);
  check(plan_margin.success, "plan_staging_leg with margin=0.1 still finds a path");
}

// ---------------------------------------------------------------------
// Part 3: build_staging_trajectory() validity.
// ---------------------------------------------------------------------
void part3_build_staging_trajectory()
{
  std::cout << "\n[test] === Part 3: build_staging_trajectory() validity ===" << std::endl;

  auto robot = make_steerable_robot("stager", Pose(0.0, 0.0, 0.0));

  // Forward leg then a deliberate reversal: (0,0,0) -> (1,0,0) -> (0,0,0).
  std::vector<Waypoint> wps;
  {
    Pose p0(0.0, 0.0, 0.0);
    Pose p1(1.0, 0.0, 0.0);
    Pose p2(0.0, 0.0, 0.0);
    wps.push_back(Waypoint(p0));
    wps.push_back(Waypoint(p1));
    wps.push_back(Waypoint(p2));
  }

  const ReloPush::trajectory traj = staging::build_staging_trajectory(robot.get(), wps);
  check(traj.trajectory_points && !traj.trajectory_points->empty(),
        "build_staging_trajectory returns a non-empty trajectory");
  if (!traj.trajectory_points || traj.trajectory_points->empty())
    return;

  const auto &pts = *traj.trajectory_points;
  // NOTE: the production pipeline (a TimeTable round-trip -- see
  // build_staging_trajectory()'s doc comment) re-samples at TimeTable's
  // default time_increment (0.5s) in ADDITION to the exact input waypoint
  // times, so the output generally has MORE than 3 points -- this
  // deliberately does NOT assert an exact count, only the invariants the
  // spec actually cares about (rel_t/monotonicity/|ref_vel|/sign).
  std::cout << "[test]   build_staging_trajectory produced " << pts.size() << " points (from 3 "
            << "input waypoints -- TimeTable re-sampling, see NOTE above)" << std::endl;

  check(pts.front().time == 0.0f, "rel_t starts at 0 (got " + to_str(pts.front().time) + ")");

  bool monotone = true;
  for (size_t i = 1; i < pts.size(); ++i)
  {
    if (!(pts[i].time > pts[i - 1].time))
    {
      monotone = false;
      break;
    }
  }
  check(monotone, "times are strictly monotone across all " + std::to_string(pts.size()) +
                       " points (through the reversal)");

  const double speed_transit = robot->speed_transit;
  bool constant_speed = true;
  for (const auto &pt : pts)
  {
    if (std::fabs(std::fabs(pt.ref_vel) - speed_transit) >= 1e-4)
    {
      constant_speed = false;
      break;
    }
  }
  check(constant_speed, "|ref_vel| == speed_transit (" + to_str(speed_transit) +
                             ") at every point");

  // Forward leg (0,0,0)->(1,0,0): ref_vel positive at the start. Reversal
  // leg (1,0,0)->(0,0,0): ref_vel negative by the end.
  check(pts.front().ref_vel > 0.0,
        "first point's ref_vel is positive (forward leg, got " + to_str(pts.front().ref_vel) +
            ")");
  check(pts.back().ref_vel < 0.0, "last point's ref_vel is negative (reversal leg, got " +
                                       to_str(pts.back().ref_vel) + ")");
}

// ---------------------------------------------------------------------
// Process-test plumbing (mirrors test_sim_viz_execute.cpp / test_sim_viz_
// stall.cpp). This file used to hold the spawned mars_sim_viz pid as a bare
// pid_t, reaped only by a manual wait_for_pid_exit()/kill_and_reap() pair
// at the tail of each Part function -- any exception thrown between spawn
// and that tail (req(), filesystem I/O, a zmq error, ...) skipped it
// entirely, leaking an orphaned mars_sim_viz (still bound to this file's
// explicit --headless/test-port args, but never reaped). Now uses
// simviz_test::ProcessGuard (test_sim_viz_shared.h), same as every other
// process-spawning test in this suite: reaping happens in its destructor,
// which runs on every ordinary exit path AND on exceptions that unwind
// through the guard's scope -- see main()'s top-level try/catch below,
// which is what actually guarantees that unwinding happens. Verified
// empirically on this toolchain: an exception that escapes main()
// uncaught calls std::terminate() WITHOUT running local destructors, so a
// scope-local guard alone is not sufficient without a reachable catch
// somewhere in the call chain.
// ---------------------------------------------------------------------

// Polls STATUS until a predicate matches or `timeout_s` elapses. Returns
// the last-seen STATUS string either way.
std::string poll_status_until(const std::string &endpoint,
                               const std::function<bool(const std::string &)> &pred,
                               double timeout_s)
{
  std::string last;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    last = req(endpoint, "STATUS");
    if (pred(last))
      return last;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return last;
}

pid_t spawn_wrapped_capture_stdout(const std::string &exe, const std::vector<std::string> &args,
                                    const std::string &stdout_path)
{
  const char *home = std::getenv("HOME");
  const char *user = std::getenv("USER");
  std::vector<std::string> wrapped_args = {
      "-i", std::string("HOME=") + (home ? home : ""),
      std::string("USER=") + (user ? user : ""), "PATH=/usr/local/bin:/usr/bin:/bin",
      "QT_QPA_PLATFORM=offscreen", exe};
  for (const auto &a : args)
    wrapped_args.push_back(a);

  const std::string env_exe = "/usr/bin/env";
  std::vector<char *> argv;
  argv.push_back(const_cast<char *>(env_exe.c_str()));
  for (auto &a : wrapped_args)
    argv.push_back(const_cast<char *>(a.c_str()));
  argv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0)
    throw std::runtime_error("fork() failed");
  if (pid == 0)
  {
    int fd = open(stdout_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (fd >= 0)
    {
      dup2(fd, STDOUT_FILENO);
      close(fd);
    }
    execv(env_exe.c_str(), argv.data());
    std::fprintf(stderr, "[test] execv('%s') failed\n", env_exe.c_str());
    _exit(127);
  }
  return pid;
}

std::string read_file(const std::string &path)
{
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// ---------------------------------------------------------------------
// Part 4: process test A -- happy path (robot1 staged, robot2 skip).
// ---------------------------------------------------------------------
void part4_process_happy_path(const std::string &sim_viz_exe, const std::string &controller_exe,
                               const std::string &robot_sim_exe,
                               const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 4: process happy path (--stage-first) ===" << std::endl;

  SyntheticScenario synth = build_staging_scenario("staging_happy");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "staging_happy.scn.b64");
  const std::filesystem::path run_dir = tmp_dir / "runs_happy";
  const std::string stdout_path = (tmp_dir / "happy_stdout.txt").string();

  // robot1 offset by (0.4m, 0m, ~30deg) from its start pose (0,0,0);
  // robot2 offset by 0 (present, already at its start pose -> Skip, no
  // leg). Both present.
  const std::string offsets = "robot1:0.4,0.0,0.5236;robot2:0.0,0.0,0.0";

  std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + scn_path,
      "--stage-first",
      "--staging-test-offsets=" + offsets,
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
  };
  ProcessGuard viz_guard(spawn_wrapped_capture_stdout(sim_viz_exe, viz_args, stdout_path),
                          "part4_mars_sim_viz");
  const pid_t pid = viz_guard.pid();
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "Part 4: control socket answers PING");

  const std::string staging_status = poll_status_until(
      endpoint, [](const std::string &s) { return s.rfind("STAGING robot1", 0) == 0; }, 15.0);
  check(staging_status.rfind("STAGING robot1", 0) == 0,
        "Part 4: STATUS shows STAGING robot1 (1/2) while its leg runs (last='" + staging_status +
            "')");

  // Wait for staging to finish (STATUS no longer starts with "STAGING") --
  // by then, per ExecutionManager::terminate_children()'s blocking
  // waitForFinished(), the leg's mpc_robot_sim has already fully exited and
  // flushed its --log-csv file.
  const std::string post_staging_status = poll_status_until(
      endpoint, [](const std::string &s) { return s.rfind("STAGING", 0) != 0; }, 30.0);
  check(post_staging_status.rfind("STAGING", 0) != 0,
        "Part 4: STATUS moves past STAGING within 30s (last='" + post_staging_status + "')");

  // Verify robot1's post-staging pose from the staging leg's own CSV
  // (distinct directory: label + "__staging_robot1_a1" + pid -- never
  // overwritten by the main run).
  const std::filesystem::path leg_dir =
      run_dir / (synth.scn.summary.label + "__staging_robot1_a1_" + std::to_string(pid));
  const std::vector<CsvRow> leg_rows = read_csv((leg_dir / "robot1.csv").string());
  check(!leg_rows.empty(), "Part 4: robot1's staging-leg CSV is non-empty (" +
                                (leg_dir / "robot1.csv").string() + ")");
  if (!leg_rows.empty())
  {
    const CsvRow &last = leg_rows.back();
    const Pose expected_start(0.0, 0.0, 0.0); // robot1's planner start pose.
    const double pos_err = std::hypot(last.x - expected_start.x, last.y - expected_start.y);
    const double yaw_err = std::fabs(wrap_pi(last.yaw - expected_start.yaw));
    std::cout << "[test]   robot1 post-staging: (" << last.x << "," << last.y << "," << last.yaw
              << ") pos_err=" << to_str(pos_err) << " yaw_err=" << to_str(yaw_err) << std::endl;
    check(pos_err <= 0.05 + 1e-6,
          "Part 4: robot1 lands within staging_pos_tol (0.05m) of its start pose (got " +
              to_str(pos_err) + "m)");
    check(yaw_err <= 0.15 + 1e-6,
          "Part 4: robot1 lands within staging_yaw_tol (0.15rad) of its start pose (got " +
              to_str(yaw_err) + "rad)");
  }

  // Now wait for the MAIN run to complete -- both robots DONE.
  StatusOutcome outcome = poll_until_done(endpoint, synth.scn.timetable.get_max_time() + 60.0);
  check(outcome.done, "Part 4: main run reaches DONE (last='" + outcome.final_status + "')");

  if (outcome.done)
  {
    const std::filesystem::path main_dir =
        run_dir / (synth.scn.summary.label + "_" + std::to_string(pid));
    for (const auto &[robot_name, expected_final] :
         std::vector<std::pair<std::string, Pose>>{{"robot1", synth.robot1_final},
                                                     {"robot2", synth.robot2_final}})
    {
      const std::vector<CsvRow> rows = read_csv((main_dir / (robot_name + ".csv")).string());
      check(!rows.empty(), "Part 4: main run " + robot_name + " CSV non-empty");
      if (!rows.empty())
      {
        const CsvRow &last = rows.back();
        const double pos_err = std::hypot(last.x - expected_final.x, last.y - expected_final.y);
        const double yaw_err = std::fabs(wrap_pi(last.yaw - expected_final.yaw));
        check(pos_err <= kFinalPosTolM,
              "Part 4: main run " + robot_name + " final position within tolerance (got " +
                  to_str(pos_err) + "m)");
        check(yaw_err <= kFinalYawTolRad,
              "Part 4: main run " + robot_name + " final yaw within tolerance (got " +
                  to_str(yaw_err) + "rad)");
      }
    }
  }

  // No --exit-on-done here (control socket must stay reachable to poll
  // STATUS through the whole staging+main sequence above) -- ABORT gives
  // mars_sim_viz a chance to tear down its own children cleanly, then
  // viz_guard.terminate() (SIGTERM -> grace period -> SIGKILL, always
  // reaped by PID) brings mars_sim_viz itself down explicitly here.
  // viz_guard's destructor is a no-op safety net (terminate() is
  // idempotent) if this function instead returns/throws before reaching
  // this point.
  req(endpoint, "ABORT");
  viz_guard.terminate();

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "Part 4: no stray mpc_robot_sim after teardown");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "Part 4: no stray mpc_controller after teardown");
}

// ---------------------------------------------------------------------
// Part 5: process test B -- missing-robot case.
// ---------------------------------------------------------------------
void part5_process_missing_robot(const std::string &sim_viz_exe, const std::string &controller_exe,
                                  const std::string &robot_sim_exe,
                                  const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 5: process missing-robot case (--stage-first) ===" << std::endl;

  SyntheticScenario synth = build_staging_scenario("staging_missing");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "staging_missing.scn.b64");
  const std::filesystem::path run_dir = tmp_dir / "runs_missing";
  const std::string stdout_path = (tmp_dir / "missing_stdout.txt").string();

  // Only robot1 is listed -> robot1 PRESENT (already at its start pose, no
  // leg needed); robot2 is MISSING -> excluded from staging AND the main
  // run.
  const std::string offsets = "robot1:0.0,0.0,0.0";

  std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + scn_path,
      "--stage-first",
      "--staging-test-offsets=" + offsets,
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
  };
  ProcessGuard viz_guard(spawn_wrapped_capture_stdout(sim_viz_exe, viz_args, stdout_path),
                          "part5_mars_sim_viz");
  const pid_t pid = viz_guard.pid();
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "Part 5: control socket answers PING");

  // robot1 is Skip-classified (no leg) so staging finishes almost
  // immediately -- just wait straight for DONE.
  StatusOutcome outcome = poll_until_done(endpoint, synth.scn.timetable.get_max_time() + 60.0);
  check(outcome.done, "Part 5: main run (robot1 only) reaches DONE (last='" +
                           outcome.final_status + "')");

  const std::filesystem::path main_dir =
      run_dir / (synth.scn.summary.label + "_" + std::to_string(pid));
  const std::vector<CsvRow> robot1_rows = read_csv((main_dir / "robot1.csv").string());
  check(!robot1_rows.empty(), "Part 5: robot1 CSV non-empty (present robot ran)");

  const bool robot2_csv_exists = std::filesystem::exists(main_dir / "robot2.csv");
  check(!robot2_csv_exists,
        "Part 5: robot2 CSV does NOT exist under the main run dir (excluded, never spawned)");

  // See Part 4's identical comment: ABORT, then viz_guard.terminate() to
  // reap mars_sim_viz explicitly (with its destructor as a no-op safety
  // net on early return/exception) BEFORE reading stdout_path below, so
  // the capture file reflects the fully-terminated process.
  req(endpoint, "ABORT");
  viz_guard.terminate();

  const std::string stdout_text = read_file(stdout_path);
  check(stdout_text.find("robot2") != std::string::npos &&
            stdout_text.find("excluded") != std::string::npos,
        "Part 5: mars_sim_viz stdout logs robot2's exclusion");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "Part 5: no stray mpc_robot_sim after teardown");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "Part 5: no stray mpc_controller after teardown");
}

} // namespace

int main()
{
  part1_staging_decision();
  part2_plan_staging_leg();
  part3_build_staging_trajectory();

  std::string dir;
  try
  {
    dir = self_dir();
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[test] fatal: " << ex.what() << std::endl;
    return 1;
  }

  const std::string sim_viz_exe = dir + "/mars_sim_viz";
  const std::string controller_exe = dir + "/../MPC/mpc_controller";
  const std::string robot_sim_exe = dir + "/../MPC/mpc_robot_sim";

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_staging_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  // Both Part functions below spawn a real mars_sim_viz, guarded by a
  // scope-local ProcessGuard (RAII reap on scope exit). That RAII guarantee
  // only holds if the stack actually unwinds through the guard's scope --
  // on this toolchain, an exception that escapes main() uncaught calls
  // std::terminate() WITHOUT unwinding first (verified empirically), which
  // would skip every local destructor, including the guards', and leak the
  // spawned mars_sim_viz as an orphan. Catching here (rather than letting
  // anything from req()/filesystem I/O/zmq propagate past main()) is what
  // makes the guards' destructors actually run.
  try
  {
    part4_process_happy_path(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
    part5_process_missing_robot(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[test] fatal: uncaught exception: " << ex.what() << std::endl;
    g_overall_pass = false;
  }
  catch (...)
  {
    std::cerr << "[test] fatal: uncaught non-std exception" << std::endl;
    g_overall_pass = false;
  }

  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n" << (g_overall_pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << std::endl;
  return g_overall_pass ? 0 : 1;
}
