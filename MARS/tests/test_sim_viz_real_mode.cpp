// PART B (DESIGN doc): standalone (non-ctest) test for mars_sim_viz's
// REAL-ROBOT MODE -- ExecutionManager::start_execution()'s real_mode branch
// (spawns ONLY mpc_controller per robot, never mpc_robot_sim) plus
// SimVizManager::stage_real_robots() / the STAGE_REAL control-socket verb
// ("move real robots to initial poses"). Same standalone-binary style as
// test_sim_viz_staging.cpp (which this file mirrors closely): spawns the
// REAL mars_sim_viz --headless binary; "real hardware" is stood in by
// ordinary mpc_robot_sim processes that the TEST ITSELF spawns and owns
// directly (never through mars_sim_viz) -- proving mars_sim_viz's own
// ExecutionManager never spawns a simulator in real_mode is exactly what
// Part 1 asserts (direct-child-process inspection via pgrep -P).
//
// TEST PORT BLOCK (per this task's port allocation): control = 45601,
// handshake = 45620+idx (mpc_controller's own handshake REP port, spawned
// by mars_sim_viz), stand-in "real hardware" vesc = 45640+idx / loc =
// 45660+idx (bound by the TEST's own mpc_robot_sim processes -- real_mode
// never binds these itself, so they are free for this use, exactly the
// "vesc 45640+, loc 45660+" block this task's rules allocate). Each part
// below is a fully independent mars_sim_viz + stand-in-sim instance,
// spawned and fully reaped (ProcessGuard destructors, synchronous
// SIGTERM-then-SIGKILL) before the next part starts, so reusing the same
// port block across parts is safe (same convention test_sim_viz_staging.cpp's
// Part 4/Part 5 already rely on).
//
// Parts:
//   1. (T2) real_mode EXECUTE: 2-robot synthetic scenario, real_robots.json
//      overrides pointing both robots at stand-in mpc_robot_sim endpoints.
//      Asserts: mars_sim_viz's DIRECT children are exactly 2 mpc_controller
//      processes (never mpc_robot_sim) while running; EXECUTE reaches DONE;
//      each stand-in's own --log-csv shows a final pose within tolerance of
//      the scenario's expected final; mars_sim_viz's children are fully
//      reaped (0 direct children) once DONE.
//   2. (T3) STAGE_REAL end-to-end: 2 steerable robots, stand-in sims started
//      ~0.3m/30deg OFFSET from their planner-assumed initial poses;
//      --stage-real-use-localization-source substitutes a persistent
//      LocalizationListener (wired to the stand-ins' own loc endpoints, via
//      real_robots.json) for MocapManager as STAGE_REAL's current-pose
//      source, so this test needs no live OptiTrack bridge (see
//      SimVizConfig::stage_real_use_localization_source's doc comment in
//      SimVizCore.h). Asserts: STAGE_REAL -> ACK_STAGE_REAL; STATUS shows
//      "STAGING_REAL <robot> (k/N)" progress, robot1 before robot2 (name
//      order); AT MOST ONE mpc_controller child is ever alive at a time
//      during the sequence (proving the robots move sequentially, never
//      two controllers active at once); both stand-ins' own CSVs show a
//      final pose within stage_real_pos_tol/yaw_tol of their planner
//      start pose; mars_sim_viz's children are fully reaped once the
//      sequence finishes.
//   3. (T3) ABORT mid-sequence: same shape as Part 2, but ABORT is sent as
//      soon as STATUS first shows "STAGING_REAL" (i.e. the first leg is
//      in flight) -- asserts ABORT -> ACK_ABORT, STATUS settles back to
//      IDLE, and mars_sim_viz's children are fully reaped shortly after.
//   4. STAGE_REAL leg exceeds its dedicated per-leg bounded timeout
//      (SimVizManager::tick_stage_real()'s "traj duration + margin" deadline
//      -- see kStageRealLegTimeoutMarginS's doc comment in SimVizCore.cpp).
//      Uses --stage-real-leg-margin-s= (TEST-ONLY override of
//      SimVizConfig::stage_real_leg_timeout_margin_s) to shrink the margin,
//      then PAUSEs mid-leg: pausing freezes ExecutionManager::tick()'s OWN
//      generic (shorter/weaker, traj+3s) completion-forcing check (it
//      returns early while paused_ -- FEATURE 2B), so exec_mgr_ can never
//      reach Done/Err on its own -- exactly the "leg against real hardware
//      that never converges" case the DEDICATED deadline exists to catch,
//      reproduced deterministically instead of relying on real hardware
//      actually failing to converge. Asserts: STATUS reaches "ERR ..."
//      (never hangs forever); the reason contains the deadline's own
//      "exceeded its bounded timeout" text; mars_sim_viz's children are
//      fully reaped; and a SUBSEQUENT STAGE_REAL on the SAME mars_sim_viz
//      process (not paused this time) still completes normally (manager
//      recovers).

#include "SimVizCore.h"
#include "StagingCore.h"
#include "test_sim_viz_shared.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
constexpr int kStandinVescPortStart = 45640;
constexpr int kStandinLocPortStart = 45660;

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

// A RobotMeta with real steering geometry -- copied from test_sim_viz_
// staging.cpp's make_steerable_robot() (see that file's comment for the
// rationale: build_synthetic_scenario()'s fixture robots have wheel_base=0/
// min_turning_radius=0, i.e. straight-line-only, useless for Part 2/3 below
// which need a robot that can actually turn to correct a yaw offset).
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

// 2 steerable robots, each with one short single-segment trajectory that
// starts EXACTLY at its own initial_pose (so ScenarioModel::
// robot_trajectories()'s first waypoint == robot->initial_pose == the
// STAGE_REAL target pose -- see SimVizManager::staging_start_pose_for()).
// No object entity -- STAGE_REAL never touches the main scenario's own
// motion, only its t=0 poses.
ExecutedScenario build_stage_real_scenario(const std::string &label)
{
  ExecutedScenario scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 2;
  scn.summary.failed_tasks = 0;

  scn.params.min_x = -1.0;
  scn.params.max_x = 4.0;
  scn.params.min_y = -1.0;
  scn.params.max_y = 4.0;
  scn.params.xy_resolution = 0.1;
  scn.params.yaw_resolution = M_PI / 6.0;
  scn.params.time_step = 1.0;
  scn.params.time_resolution = scn.params.time_step;
  scn.params.max_time = 60.0;

  RobotMeta *robot1 = make_steerable_robot("robot1", Pose(0.0, 0.0, 0.0)).release();
  RobotMeta *robot2 = make_steerable_robot("robot2", Pose(0.0, 2.0, 0.0)).release();
  scn.entities["robot1"] = robot1;
  scn.entities["robot2"] = robot2;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  {
    Pose p1(0.0, 0.0, 0.0);
    Pose p2(1.0, 0.0, 0.0);
    WaypointPath path = {Waypoint(p1), Waypoint(p2)};
    Trajectory traj(robot1, nullptr, 0.0, path, /*is_transfer=*/false);
    traj.CalcualteTimeStamps(robot1, 0.0);
    scn.timetable.add_trajectory(traj);
  }
  {
    Pose p1(0.0, 2.0, 0.0);
    Pose p2(1.0, 2.0, 0.0);
    WaypointPath path = {Waypoint(p1), Waypoint(p2)};
    Trajectory traj(robot2, nullptr, 0.0, path, /*is_transfer=*/false);
    traj.CalcualteTimeStamps(robot2, 0.0);
    scn.timetable.add_trajectory(traj);
  }

  return scn;
}

// ---------------------------------------------------------------------
// Direct-child-process inspection (proves what mars_sim_viz itself spawned,
// as opposed to processes the TEST spawned directly to stand in as "real
// hardware"). mars_sim_viz's QProcess spawns "env -i ... <binary> args..."
// -- env execve()s directly into <binary> (no fork), so a direct child of
// mars_sim_viz's own PID IS that binary's process image; pgrep -P lists
// exactly those.
// ---------------------------------------------------------------------
std::vector<pid_t> direct_child_pids(pid_t ppid)
{
  std::vector<pid_t> out;
  const std::string cmd = "pgrep -P " + std::to_string(ppid) + " 2>/dev/null";
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe)
    return out;
  char buf[64];
  while (fgets(buf, sizeof(buf), pipe) != nullptr)
  {
    pid_t p = static_cast<pid_t>(std::atoi(buf));
    if (p > 0)
      out.push_back(p);
  }
  pclose(pipe);
  return out;
}

std::string proc_cmdline(pid_t pid)
{
  std::ifstream f("/proc/" + std::to_string(pid) + "/cmdline", std::ios::binary);
  if (!f.is_open())
    return "";
  std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  for (auto &c : raw)
    if (c == '\0')
      c = ' ';
  return raw;
}

// Counts mars_sim_viz's direct children whose cmdline contains `needle`.
int count_direct_children_matching(pid_t ppid, const std::string &needle)
{
  int n = 0;
  for (pid_t p : direct_child_pids(ppid))
  {
    if (proc_cmdline(p).find(needle) != std::string::npos)
      ++n;
  }
  return n;
}

// SAVE-FAILS BUG FIX regression (Part 5 below): like spawn_wrapped() but the
// CHILD chdir()s to `work_dir` before exec (repro: `cd build-release/MARS &&
// ./mars_sim_viz`) and has its stdout redirected to `stdout_path` so the
// test can grep mars_sim_viz's own startup log for the
// "[mars_sim_viz] Loaded real-robot endpoints config '<path>'" message --
// the only observable signal that --real-robots-config='s DEFAULT actually
// resolved to a real, loadable file rather than a CWD-relative miss (a
// missing DEFAULT path is silently ignored, so its ABSENCE alone wouldn't
// prove anything).
pid_t spawn_wrapped_in_dir_capture_stdout(const std::string &exe, const std::vector<std::string> &args,
                                           const std::string &work_dir, const std::string &stdout_path)
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
      dup2(fd, STDERR_FILENO);
      close(fd);
    }
    if (chdir(work_dir.c_str()) != 0)
    {
      std::fprintf(stderr, "[test] chdir('%s') failed\n", work_dir.c_str());
      _exit(126);
    }
    execv(env_exe.c_str(), argv.data());
    std::fprintf(stderr, "[test] execv('%s') failed\n", env_exe.c_str());
    _exit(127);
  }
  return pid;
}

std::string write_real_robots_config(
    const std::filesystem::path &path,
    const std::vector<std::pair<std::string, std::pair<int, int>>> &robots)
{
  nlohmann::json j;
  j["_doc"] = "test config -- see MARS/config/real_robots.json";
  nlohmann::json robots_j = nlohmann::json::object();
  for (const auto &[name, ports] : robots)
  {
    nlohmann::json r;
    r["vesc_endpoint"] = "tcp://127.0.0.1:" + std::to_string(ports.first);
    r["localization_endpoint"] = "tcp://127.0.0.1:" + std::to_string(ports.second);
    robots_j[name] = r;
  }
  j["robots"] = robots_j;
  std::ofstream out(path);
  out << j.dump(2);
  return path.string();
}

// ---------------------------------------------------------------------
// Part 1 (T2): real_mode EXECUTE, no simulator spawned by mars_sim_viz.
// ---------------------------------------------------------------------
void part1_real_mode_execute(const std::string &sim_viz_exe, const std::string &controller_exe,
                              const std::string &robot_sim_exe,
                              const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 1 (T2): real_mode EXECUTE, no sim spawned ===" << std::endl;

  SyntheticScenario synth = build_synthetic_scenario("real_mode_execute_test");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "real_mode_execute.scn.b64");
  const double makespan = synth.scn.timetable.get_max_time();

  const std::vector<std::pair<std::string, Pose>> initial_poses = {
      {"robot1", Pose(0.0, 0.0, 0.0)}, {"robot2", Pose(0.0, 2.0, 0.0)}};

  // Stand-in "real hardware": one mpc_robot_sim per robot, spawned and
  // owned DIRECTLY by this test (never by mars_sim_viz), bound at the
  // test's own ports, starting exactly at the scenario's initial poses.
  std::vector<ProcessGuard> standins;
  std::vector<std::string> standin_csvs;
  std::vector<std::pair<std::string, std::pair<int, int>>> endpoint_spec;
  int idx = 0;
  for (const auto &[name, pose] : initial_poses)
  {
    const int vesc_port = kStandinVescPortStart + idx;
    const int loc_port = kStandinLocPortStart + idx;
    const std::string csv_path = (tmp_dir / ("part1_standin_" + name + ".csv")).string();
    std::vector<std::string> args = {
        "--robot-name",   name,
        "--cmd-endpoint", "tcp://*:" + std::to_string(vesc_port),
        "--loc-endpoint", "tcp://*:" + std::to_string(loc_port),
        "--x",            to_str(pose.x),
        "--y",            to_str(pose.y),
        "--yaw",          to_str(pose.yaw),
        "--log-csv",      csv_path,
    };
    standins.emplace_back(spawn_wrapped(robot_sim_exe, args), "part1_standin_" + name);
    standin_csvs.push_back(csv_path);
    endpoint_spec.emplace_back(name, std::make_pair(vesc_port, loc_port));
    ++idx;
  }
  std::cout << "[test] spawned " << standins.size()
            << " stand-in mpc_robot_sim process(es) as \"real hardware\"" << std::endl;
  // Let them finish binding/start publishing before mars_sim_viz's
  // controllers try to connect / this test's freshness checks matter.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const std::string real_robots_path = write_real_robots_config(
      tmp_dir / "part1_real_robots.json", endpoint_spec);

  std::vector<std::string> viz_args = {
      "--headless",
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kStandinVescPortStart), // unused fallback
      "--loc-port-start=" + std::to_string(kStandinLocPortStart),   // unused fallback
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe, // unused in real_mode, harmless
      "--run-dir=" + (tmp_dir / "part1_runs").string(),
      "--real-mode",
      "--real-robots-config=" + real_robots_path,
  };
  ProcessGuard viz_guard(spawn_wrapped(sim_viz_exe, viz_args), "part1_mars_sim_viz");
  std::cout << "[test] spawned mars_sim_viz pid=" << viz_guard.pid() << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "Part1: control socket answers PING");

  std::string reply = req(endpoint, "EXECUTE " + scn_path);
  check(reply == "ACK_EXECUTE", "Part1: EXECUTE -> ACK_EXECUTE (" + reply + ")");

  // Give the handshake a moment to complete, then inspect mars_sim_viz's
  // OWN direct children while the run is (very likely) still active.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  {
    const int n_sim = count_direct_children_matching(viz_guard.pid(), "mpc_robot_sim");
    const int n_ctrl = count_direct_children_matching(viz_guard.pid(), "mpc_controller");
    check(n_sim == 0, "Part1: mars_sim_viz spawned NO mpc_robot_sim child in real_mode (found " +
                           std::to_string(n_sim) + ")");
    check(n_ctrl == static_cast<int>(initial_poses.size()),
          "Part1: mars_sim_viz spawned exactly " + std::to_string(initial_poses.size()) +
              " mpc_controller child(ren) (found " + std::to_string(n_ctrl) + ")");
  }

  StatusOutcome outcome = poll_until_done(endpoint, makespan + 20.0);
  check(outcome.done, "Part1: run reaches DONE (last='" + outcome.final_status + "')");

  if (outcome.done)
  {
    for (size_t i = 0; i < initial_poses.size(); ++i)
    {
      const std::string &name = initial_poses[i].first;
      const Pose &expected_final = (name == "robot1") ? synth.robot1_final : synth.robot2_final;
      const std::vector<CsvRow> rows = read_csv(standin_csvs[i]);
      check(!rows.empty(), "Part1: " + name + " stand-in CSV non-empty (" + standin_csvs[i] + ")");
      if (!rows.empty())
      {
        const CsvRow &last = rows.back();
        const double pos_err = std::hypot(last.x - expected_final.x, last.y - expected_final.y);
        const double yaw_err = std::fabs(wrap_pi(last.yaw - expected_final.yaw));
        std::cout << "[test]   " << name << " final: (" << last.x << "," << last.y << ","
                  << last.yaw << ") expected=(" << expected_final.x << "," << expected_final.y
                  << "," << expected_final.yaw << ") pos_err=" << to_str(pos_err)
                  << " yaw_err=" << to_str(yaw_err) << std::endl;
        check(pos_err <= kFinalPosTolM, "Part1: " + name + " final position within " +
                                             to_str(kFinalPosTolM) + "m (got " + to_str(pos_err) +
                                             "m)");
        check(yaw_err <= kFinalYawTolRad, "Part1: " + name + " final yaw within " +
                                               to_str(kFinalYawTolRad) + "rad (got " +
                                               to_str(yaw_err) + "rad)");
      }
    }
  }

  // finalize_run()'s terminate_children() already reaped both
  // mpc_controller children by the time DONE was observed above -- assert
  // that directly (mars_sim_viz itself, and the stand-in sims, are still
  // alive; only ITS children should be gone).
  {
    const auto remaining = direct_child_pids(viz_guard.pid());
    check(remaining.empty(),
          "Part1: mars_sim_viz has 0 direct children after DONE (found " +
              std::to_string(remaining.size()) + ")");
  }

  req(endpoint, "ABORT"); // defensive no-op if already Idle.
  // viz_guard/standins destruct (SIGTERM, reaped) at function return.
}

// ---------------------------------------------------------------------
// Poll STATUS while a STAGE_REAL sequence is active, recording the
// first-appearance order of "STAGING_REAL <robot>" names and the maximum
// number of mars_sim_viz's OWN mpc_controller children observed alive at
// once (proves sequential, never-two-at-once execution). Returns once
// STATUS no longer starts with "STAGING_REAL" (sequence finished) or
// `timeout_s` elapses.
// ---------------------------------------------------------------------
struct StageRealPollResult
{
  std::vector<std::string> robot_sequence; // first-appearance order.
  int max_concurrent_controllers = 0;
  std::string final_status;
  bool finished = false;
};

StageRealPollResult poll_stage_real_sequence(const std::string &endpoint, pid_t viz_pid,
                                              double timeout_s)
{
  StageRealPollResult out;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    const std::string status = req(endpoint, "STATUS");
    if (status.rfind("STAGING_REAL", 0) == 0)
    {
      std::istringstream iss(status);
      std::string tag, name;
      iss >> tag >> name;
      if (out.robot_sequence.empty() || out.robot_sequence.back() != name)
      {
        out.robot_sequence.push_back(name);
        std::cout << "[test]   STAGE_REAL progress: " << status << std::endl;
      }
      const int n_ctrl = count_direct_children_matching(viz_pid, "mpc_controller");
      out.max_concurrent_controllers = std::max(out.max_concurrent_controllers, n_ctrl);
    }
    else
    {
      out.final_status = status;
      out.finished = true;
      return out;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  out.final_status = "<<TIMEOUT waiting for STAGE_REAL sequence to finish>>";
  return out;
}

// Polls STATUS until the first "STAGING_REAL" reply (or timeout). Returns
// the last-seen STATUS string either way.
std::string wait_for_stage_real_start(const std::string &endpoint, double timeout_s)
{
  std::string last;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    last = req(endpoint, "STATUS");
    if (last.rfind("STAGING_REAL", 0) == 0)
      return last;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return last;
}

// Polls STATUS until it equals `expected` exactly, or `timeout_s` elapses.
std::string poll_status_until_equals(const std::string &endpoint, const std::string &expected,
                                      double timeout_s)
{
  std::string last;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    last = req(endpoint, "STATUS");
    if (last == expected)
      return last;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return last;
}

// Polls STATUS until it starts with `prefix`, or `timeout_s` elapses.
std::string poll_status_until_prefix(const std::string &endpoint, const std::string &prefix,
                                      double timeout_s)
{
  std::string last;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    last = req(endpoint, "STATUS");
    if (last.rfind(prefix, 0) == 0)
      return last;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return last;
}

// Retries PAUSE until it returns ACK_PAUSE or `timeout_s` elapses. PAUSE
// fails with "ERR handshake not complete yet" (see ExecutionManager::
// send_pause_resume()'s t0_valid_ guard) until the leg's handshake thread
// has collected ACK_START -- a race against this test's own PAUSE call
// landing right after start_execution() fires (see setup_stage_real_
// fixture()/tick_stage_real()), so this must retry rather than assume a
// single attempt lands after t0_valid_ flips.
bool pause_with_retries(const std::string &endpoint, double timeout_s)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (req(endpoint, "PAUSE") == "ACK_PAUSE")
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return false;
}

// ---------------------------------------------------------------------
// Sets up one STAGE_REAL scenario instance: writes the scn.b64 + a
// real_robots.json pointing at stand-in mpc_robot_sim processes started
// OFFSET from their planner-assumed initial poses, then spawns mars_sim_viz
// (--real-mode --stage-real-use-localization-source). Returns everything
// the caller needs to drive/inspect the sequence; ProcessGuards are
// returned by value (moved) so the CALLER controls their lifetime (Part 2
// waits for full completion before tearing down; Part 3 aborts mid-flight).
// ---------------------------------------------------------------------
struct StageRealFixture
{
  ProcessGuard viz;
  std::vector<ProcessGuard> standins;
  std::vector<std::string> standin_csvs;
  std::string scn_path;
  std::string endpoint;
};

// `leg_margin_s` < 0 (the default): no --stage-real-leg-margin-s= override,
// mars_sim_viz uses its production default (kStageRealLegTimeoutMarginS,
// 15s -- see SimVizConfig::stage_real_leg_timeout_margin_s's doc comment).
// >= 0: passed through verbatim -- Part 4 (deadline test) uses this to
// shrink the per-leg deadline so it doesn't need a 15s+ real-time wait.
StageRealFixture setup_stage_real_fixture(const std::string &sim_viz_exe,
                                           const std::string &controller_exe,
                                           const std::string &robot_sim_exe,
                                           const std::filesystem::path &tmp_dir,
                                           const std::string &label,
                                           double leg_margin_s = -1.0)
{
  ExecutedScenario scn = build_stage_real_scenario(label);
  StageRealFixture fx;
  fx.scn_path = write_scn_file(scn, tmp_dir / (label + ".scn.b64"));

  // ~0.3m / ~30deg (0.5236 rad) offset from each robot's planner-assumed
  // initial pose -- matches this task's DESIGN doc ("~0.3m/30deg").
  const std::vector<std::pair<std::string, Pose>> offset_starts = {
      {"robot1", Pose(0.3, 0.0, 0.5236)}, {"robot2", Pose(0.3, 2.0, 0.5236)}};

  std::vector<std::pair<std::string, std::pair<int, int>>> endpoint_spec;
  int idx = 0;
  for (const auto &[name, pose] : offset_starts)
  {
    const int vesc_port = kStandinVescPortStart + idx;
    const int loc_port = kStandinLocPortStart + idx;
    const std::string csv_path = (tmp_dir / (label + "_standin_" + name + ".csv")).string();
    std::vector<std::string> args = {
        "--robot-name",   name,
        "--cmd-endpoint", "tcp://*:" + std::to_string(vesc_port),
        "--loc-endpoint", "tcp://*:" + std::to_string(loc_port),
        "--x",            to_str(pose.x),
        "--y",            to_str(pose.y),
        "--yaw",          to_str(pose.yaw),
        "--log-csv",      csv_path,
    };
    fx.standins.emplace_back(spawn_wrapped(robot_sim_exe, args), label + "_standin_" + name);
    fx.standin_csvs.push_back(csv_path);
    endpoint_spec.emplace_back(name, std::make_pair(vesc_port, loc_port));
    ++idx;
  }
  std::cout << "[test] (" << label << ") spawned " << fx.standins.size()
            << " stand-in mpc_robot_sim process(es), offset from their targets" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  const std::string real_robots_path =
      write_real_robots_config(tmp_dir / (label + "_real_robots.json"), endpoint_spec);

  std::vector<std::string> viz_args = {
      "--headless",
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kStandinVescPortStart), // unused fallback
      "--loc-port-start=" + std::to_string(kStandinLocPortStart),   // unused fallback
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe, // unused in real_mode
      "--run-dir=" + (tmp_dir / (label + "_runs")).string(),
      "--real-mode",
      "--real-robots-config=" + real_robots_path,
      "--stage-real-use-localization-source", // TEST-ONLY: no live OptiTrack bridge here.
      "--stage-real-pos-tol=0.05",
      "--stage-real-yaw-tol=0.15",
  };
  if (leg_margin_s >= 0.0)
    viz_args.push_back("--stage-real-leg-margin-s=" + to_str(leg_margin_s));
  fx.viz = ProcessGuard(spawn_wrapped(sim_viz_exe, viz_args), label + "_mars_sim_viz");
  std::cout << "[test] (" << label << ") spawned mars_sim_viz pid=" << fx.viz.pid() << std::endl;

  fx.endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(fx.endpoint, 15.0), "(" + label + "): control socket answers PING");
  // Let the persistent real_pose_listener_ (started at SimVizManager::
  // start(), see SimVizConfig::stage_real_use_localization_source) receive
  // at least one fresh sample from each stand-in before STAGE_REAL is sent.
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  return fx;
}

// ---------------------------------------------------------------------
// Part 2 (T3): STAGE_REAL end-to-end -- sequential execution, both robots
// land within tolerance.
// ---------------------------------------------------------------------
void part2_stage_real_happy_path(const std::string &sim_viz_exe, const std::string &controller_exe,
                                  const std::string &robot_sim_exe,
                                  const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 2 (T3): STAGE_REAL end-to-end ===" << std::endl;

  StageRealFixture fx = setup_stage_real_fixture(sim_viz_exe, controller_exe, robot_sim_exe,
                                                  tmp_dir, "part2_stage_real");

  const std::string reply = req(fx.endpoint, "STAGE_REAL " + fx.scn_path);
  check(reply == "ACK_STAGE_REAL", "Part2: STAGE_REAL -> ACK_STAGE_REAL (" + reply + ")");

  StageRealPollResult poll = poll_stage_real_sequence(fx.endpoint, fx.viz.pid(), 90.0);
  check(poll.finished, "Part2: STAGE_REAL sequence finishes within 90s (last='" +
                            poll.final_status + "')");
  check(!poll.final_status.empty() && poll.final_status.rfind("ERR", 0) != 0,
        "Part2: sequence does not end in ERR (final='" + poll.final_status + "')");

  const std::vector<std::string> expected_order = {"robot1", "robot2"};
  check(poll.robot_sequence == expected_order,
        "Part2: robots staged in name order (robot1 then robot2) -- got [" +
            (poll.robot_sequence.empty() ? std::string("<none>")
                                          : [&] {
                                              std::string s;
                                              for (size_t i = 0; i < poll.robot_sequence.size(); ++i)
                                                s += (i ? "," : "") + poll.robot_sequence[i];
                                              return s;
                                            }()) +
            "]");

  check(poll.max_concurrent_controllers <= 1,
        "Part2: never more than 1 mpc_controller child alive at once during the sequence (max "
        "observed=" +
            std::to_string(poll.max_concurrent_controllers) + ")");

  const std::vector<Pose> targets = {Pose(0.0, 0.0, 0.0), Pose(0.0, 2.0, 0.0)};
  for (size_t i = 0; i < fx.standin_csvs.size(); ++i)
  {
    const std::vector<CsvRow> rows = read_csv(fx.standin_csvs[i]);
    check(!rows.empty(), "Part2: stand-in[" + std::to_string(i) + "] CSV non-empty");
    if (rows.empty())
      continue;
    const CsvRow &last = rows.back();
    const double pos_err = std::hypot(last.x - targets[i].x, last.y - targets[i].y);
    const double yaw_err = std::fabs(wrap_pi(last.yaw - targets[i].yaw));
    std::cout << "[test]   stand-in[" << i << "] final: (" << last.x << "," << last.y << ","
              << last.yaw << ") target=(" << targets[i].x << "," << targets[i].y << ","
              << targets[i].yaw << ") pos_err=" << to_str(pos_err)
              << " yaw_err=" << to_str(yaw_err) << std::endl;
    check(pos_err <= 0.05 + 1e-6, "Part2: stand-in[" + std::to_string(i) +
                                       "] lands within stage_real_pos_tol (0.05m) of target "
                                       "(got " +
                                       to_str(pos_err) + "m)");
    check(yaw_err <= 0.15 + 1e-6, "Part2: stand-in[" + std::to_string(i) +
                                       "] lands within stage_real_yaw_tol (0.15rad) of target "
                                       "(got " +
                                       to_str(yaw_err) + "rad)");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  {
    const auto remaining = direct_child_pids(fx.viz.pid());
    check(remaining.empty(), "Part2: mars_sim_viz has 0 direct children once the sequence "
                              "finishes (found " +
                                  std::to_string(remaining.size()) + ")");
  }

  req(fx.endpoint, "ABORT"); // defensive no-op.
  // fx's ProcessGuards destruct (SIGTERM, reaped) at function return.
}

// ---------------------------------------------------------------------
// Part 3 (T3): ABORT mid-sequence tears down cleanly.
// ---------------------------------------------------------------------
void part3_stage_real_abort_mid_sequence(const std::string &sim_viz_exe,
                                          const std::string &controller_exe,
                                          const std::string &robot_sim_exe,
                                          const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 3 (T3): ABORT mid-STAGE_REAL-sequence ===" << std::endl;

  StageRealFixture fx = setup_stage_real_fixture(sim_viz_exe, controller_exe, robot_sim_exe,
                                                  tmp_dir, "part3_stage_real_abort");

  const std::string reply = req(fx.endpoint, "STAGE_REAL " + fx.scn_path);
  check(reply == "ACK_STAGE_REAL", "Part3: STAGE_REAL -> ACK_STAGE_REAL (" + reply + ")");

  const std::string mid_status = wait_for_stage_real_start(fx.endpoint, 30.0);
  check(mid_status.rfind("STAGING_REAL robot1", 0) == 0,
        "Part3: STATUS shows STAGING_REAL robot1 (leg in flight) before ABORT (last='" +
            mid_status + "')");

  const std::string abort_reply = req(fx.endpoint, "ABORT");
  check(abort_reply == "ACK_ABORT", "Part3: ABORT -> ACK_ABORT (" + abort_reply + ")");

  const std::string post_abort_status =
      poll_status_until_equals(fx.endpoint, "IDLE", 15.0);
  check(post_abort_status == "IDLE",
        "Part3: STATUS settles to IDLE after ABORT (last='" + post_abort_status + "')");

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  {
    const auto remaining = direct_child_pids(fx.viz.pid());
    check(remaining.empty(),
          "Part3: mars_sim_viz has 0 direct children shortly after ABORT (found " +
              std::to_string(remaining.size()) + ")");
  }

  // fx's ProcessGuards destruct (SIGTERM, reaped) at function return.
}

// ---------------------------------------------------------------------
// Part 4: STAGE_REAL leg exceeds its DEDICATED per-leg bounded timeout
// (tick_stage_real()'s "traj duration + margin" deadline, checked BEFORE
// exec_mgr_'s own state -- see SimVizCore.cpp's kStageRealLegTimeoutMarginS
// doc comment). Reproduced deterministically by PAUSEing mid-leg: pausing
// freezes ExecutionManager::tick()'s own generic (shorter/weaker, traj+3s)
// completion-forcing check (FEATURE 2B: "completion cannot fire while
// paused"), so exec_mgr_ can never reach Done/Err by itself -- exactly the
// "real hardware that never converges" case the dedicated deadline exists
// to catch, without depending on the stand-in actually failing to move.
// --stage-real-leg-margin-s= (TEST-ONLY override, see setup_stage_real_
// fixture()'s doc comment) shrinks the wait for the deadline to fire, but
// is kept well above ExecutionManager::tick()'s own kCompletionMarginS
// (3s) + handshake overhead so the SECOND (non-paused, "recovery")
// STAGE_REAL leg below still completes normally via the ordinary path
// instead of also racing this same dedicated deadline.
// ---------------------------------------------------------------------
void part4_stage_real_leg_deadline_exceeded(const std::string &sim_viz_exe,
                                             const std::string &controller_exe,
                                             const std::string &robot_sim_exe,
                                             const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 4: STAGE_REAL leg exceeds its bounded timeout ===" << std::endl;

  constexpr double kTestLegMarginS = 6.0;

  StageRealFixture fx = setup_stage_real_fixture(sim_viz_exe, controller_exe, robot_sim_exe,
                                                  tmp_dir, "part4_stage_real_deadline",
                                                  kTestLegMarginS);

  const std::string reply = req(fx.endpoint, "STAGE_REAL " + fx.scn_path);
  check(reply == "ACK_STAGE_REAL", "Part4: STAGE_REAL -> ACK_STAGE_REAL (" + reply + ")");

  const std::string mid_status = wait_for_stage_real_start(fx.endpoint, 30.0);
  check(mid_status.rfind("STAGING_REAL robot1", 0) == 0,
        "Part4: STATUS shows STAGING_REAL robot1 (leg in flight) before PAUSE (last='" +
            mid_status + "')");

  const bool paused_ok = pause_with_retries(fx.endpoint, 10.0);
  check(paused_ok, "Part4: PAUSE succeeds mid-leg (freezes exec_mgr_'s generic completion check)");

  // Wait past the leg's dedicated deadline (leg duration + kTestLegMarginS,
  // set at leg-start, well before this PAUSE) -- generous timeout since
  // this test doesn't otherwise know the staging leg's planned duration.
  const std::string post_status = poll_status_until_prefix(fx.endpoint, "ERR ", 60.0);
  check(post_status.rfind("ERR ", 0) == 0,
        "Part4: STATUS transitions to ERR (never hangs forever) (last='" + post_status + "')");
  check(post_status.find("exceeded its bounded timeout") != std::string::npos,
        "Part4: ERR reason contains the deadline's own message (got '" + post_status + "')");

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  {
    const auto remaining = direct_child_pids(fx.viz.pid());
    check(remaining.empty(),
          "Part4: mars_sim_viz has 0 direct children after the deadline ERR (found " +
              std::to_string(remaining.size()) + ")");
  }

  // Manager recovers: a SUBSEQUENT STAGE_REAL on the SAME still-alive
  // mars_sim_viz process (and the SAME stand-in "real hardware" processes
  // -- only mars_sim_viz's OWN controller child was torn down by
  // fail_stage_real()'s teardown) completes normally, not paused this time,
  // so it takes the ordinary "leg finished" path well inside
  // kTestLegMarginS (see this function's header comment).
  const std::string reply2 = req(fx.endpoint, "STAGE_REAL " + fx.scn_path);
  check(reply2 == "ACK_STAGE_REAL", "Part4: post-ERR STAGE_REAL -> ACK_STAGE_REAL (" + reply2 + ")");

  StageRealPollResult poll2 = poll_stage_real_sequence(fx.endpoint, fx.viz.pid(), 90.0);
  check(poll2.finished, "Part4: post-ERR STAGE_REAL sequence finishes within 90s (last='" +
                             poll2.final_status + "')");
  check(!poll2.final_status.empty() && poll2.final_status.rfind("ERR", 0) != 0,
        "Part4: post-ERR STAGE_REAL sequence does not end in ERR (final='" + poll2.final_status +
            "')");

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  {
    const auto remaining = direct_child_pids(fx.viz.pid());
    check(remaining.empty(),
          "Part4: mars_sim_viz has 0 direct children once the recovery sequence finishes (found " +
              std::to_string(remaining.size()) + ")");
  }

  req(fx.endpoint, "ABORT"); // defensive no-op.
  // fx's ProcessGuards destruct (SIGTERM, reaped) at function return.
}

std::string read_file(const std::string &path)
{
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// ---------------------------------------------------------------------
// Part 5 (SAVE-FAILS BUG FIX): --real-robots-config='s DEFAULT used to be
// the bare relative string "MARS/config/real_robots.json", resolved against
// mars_sim_viz's CURRENT WORKING DIRECTORY -- correct only when launched
// from the repo root. The user's actual launch convention is `cd
// build-release/MARS && ./mars_sim_viz`, under which the OLD default
// resolved to the nonexistent "build-release/MARS/MARS/config/
// real_robots.json" (a missing DEFAULT path is silently ignored at load
// time -- see simviz_main.cpp's print_usage() -- but SimVizConfig::
// real_robots_config_path was then ALSO wrong for every later "Save
// Mapping" write, which is NOT silent -- that's the user's actual bug
// report). Fixed to resolve relative to the executable's own location
// instead (finalize_real_robots_config() in simviz_main.cpp).
//
// This test reproduces the user's exact launch convention: spawns the REAL
// mars_sim_viz binary with its CWD set to build-release/MARS (this test
// binary's own directory, `dir` in main() below -- mars_sim_viz is built
// into that SAME directory, so this is byte-identical to `cd
// build-release/MARS && ./mars_sim_viz`) and, deliberately, NO
// --real-robots-config= override, so the default-resolution path is what's
// under test. Asserts (via the startup log, the only observable signal --
// see spawn_wrapped_in_dir_capture_stdout()'s doc comment):
//   - mars_sim_viz starts up fine (control socket answers PING) even
//     though CWD is not the repo root;
//   - its own log shows it successfully LOADED a real-robot endpoints
//     config (proving the default resolved to an actual file, not a silent
//     miss) at an ABSOLUTE path ending in "MARS/config/real_robots.json"
//     (the REAL project file already checked into the repo -- this test
//     never writes to it, only mars_sim_viz's normal READ-ONLY startup
//     load touches it);
//   - that path is NOT the old broken "build-release/MARS/MARS/config/..."
//     CWD-relative guess.
// ---------------------------------------------------------------------
void part5_default_real_robots_config_cwd_independence(const std::string &sim_viz_exe,
                                                         const std::string &work_dir,
                                                         const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 5 (SAVE-FAILS bug fix): default --real-robots-config= resolves "
               "relative to the executable, not CWD ==="
            << std::endl;

  const int control_port = kControlPort + 900; // distinct block -- avoid clashing with parts 1-4.
  const std::string log_path = (tmp_dir / "part5_stdout.log").string();

  std::vector<std::string> viz_args = {
      "--headless",
      "--control-port=" + std::to_string(control_port),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart + 900),
      "--vesc-port-start=" + std::to_string(kStandinVescPortStart + 900),
      "--loc-port-start=" + std::to_string(kStandinLocPortStart + 900),
      "--run-dir=" + (tmp_dir / "part5_runs").string(),
      // Deliberately NO --real-robots-config= -- exercising the DEFAULT.
  };
  ProcessGuard viz_guard(
      spawn_wrapped_in_dir_capture_stdout(sim_viz_exe, viz_args, work_dir, log_path),
      "part5_mars_sim_viz");
  std::cout << "[test] spawned mars_sim_viz pid=" << viz_guard.pid() << " with CWD=" << work_dir
            << " (repro: `cd build-release/MARS && ./mars_sim_viz`)" << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(control_port);
  check(wait_for_ping(endpoint, 15.0),
        "Part5: control socket answers PING (started OK with CWD=build-release/MARS and no "
        "--real-robots-config= override)");

  viz_guard.terminate();
  std::this_thread::sleep_for(std::chrono::milliseconds(300)); // let the log flush after SIGTERM.

  const std::string log_text = read_file(log_path);
  const bool found_load_message =
      log_text.find("[mars_sim_viz] Loaded real-robot endpoints config '") != std::string::npos;
  check(found_load_message,
        "Part5: startup log shows the default real-robots-config was actually LOADED (not "
        "silently missed) -- log:\n" +
            log_text);
  check(log_text.find("MARS/config/real_robots.json") != std::string::npos,
        "Part5: loaded path ends in the real project's 'MARS/config/real_robots.json'");
  check(log_text.find("build-release/MARS/MARS/config") == std::string::npos,
        "Part5: no stale CWD-relative 'build-release/MARS/MARS/config' path in the log (the OLD "
        "bug's exact symptom)");
  std::cout << "[test]   Part5 log excerpt: "
            << log_text.substr(0, std::min<size_t>(log_text.size(), 300)) << std::endl;
}

} // namespace

int main()
{
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
      ("mars_simviz_real_mode_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  part1_real_mode_execute(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  part2_stage_real_happy_path(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  part3_stage_real_abort_mid_sequence(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  part4_stage_real_leg_deadline_exceeded(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  part5_default_real_robots_config_cwd_independence(sim_viz_exe, dir, tmp_dir);

  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n" << (g_overall_pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << std::endl;
  return g_overall_pass ? 0 : 1;
}
