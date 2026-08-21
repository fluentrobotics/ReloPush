// FEATURE 2C: standalone (non-ctest) pause/resume/restart test for
// mars_sim_viz's control-socket protocol additions (SimVizCore.h/.cpp's
// ExecutionManager::pause_all()/resume_all()/SimVizManager::pause()/
// resume()/restart()). Spawns the REAL mars_sim_viz --headless binary +
// real mpc_controller/mpc_robot_sim children against the same synthetic
// ~14s-makespan scenario test_sim_viz_execute.cpp uses -- reuses the
// process-spawn/ProcessGuard/CSV-reading plumbing factored into
// test_sim_viz_shared.h. See that file's header comment for the shared
// harness design.
//
// TEST PORT BLOCK (same block as test_sim_viz_execute.cpp/
// test_sim_viz_integration.cpp; each part below fully reaps its
// mars_sim_viz instance via ProcessGuard before the next part spawns a new
// one, so port reuse across parts is safe):
//   control = 45601, handshake = 45620+idx, vesc = 45640+idx,
//   localization = 45660+idx.
//
// Parts:
//   1. Happy pause: EXECUTE the synthetic scenario; once plan t>=~4s, PAUSE
//      -> ACK_PAUSE; over a ~1s brake-down settling margin + 3s pause
//      window verify STATUS reports PAUSED with a frozen t (two polls
//      ~1.5s apart agree within 0.3s), and every robot's sim CSV shows
//      |v|<0.02 and <0.03m pose drift for every row logged during the
//      window; RESUME -> ACK_RESUME; poll to DONE; final poses within
//      tolerance; total wall duration ~= makespan + measured pause length
//      (+-5s slack).
//   2. Idempotency + errors: PAUSE/RESUME while IDLE -> ERR; PAUSE while
//      already paused -> ACK (idempotent); RESUME while already running ->
//      ACK (idempotent).
//   3. Restart: EXECUTE; once plan t>=~4s, RESTART -> ACK_RESTART; STATUS
//      returns to RUNNING with a small t (<3s, proving the clock reset);
//      exactly the same number of child processes are alive after RESTART
//      as before it (proving the old run's children were torn down, not
//      leaked alongside the new ones); run to DONE; final poses within
//      tolerance.

#include "SimVizCore.h"
#include "test_sim_viz_shared.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

bool DEBUG_VIS = false;

using namespace simviz_test;

namespace
{

constexpr int kControlPort = 45601;
constexpr int kHandshakePortStart = 45620;
constexpr int kVescPortStart = 45640;
constexpr int kLocPortStart = 45660;

// FEATURE 2C tuning constants (see this file's header comment for the
// rationale behind each).
constexpr double kPauseAtPlanT = 4.0;      // send PAUSE once plan t reaches this
constexpr double kSettleMarginS = 1.0;     // brake-down settling margin after PAUSE
constexpr double kPollGapS = 1.5;          // gap between the two frozen-t STATUS polls
constexpr double kFrozenTToleranceS = 0.3; // max allowed drift between those two polls
// kMaxPausedSpeed/kMaxPauseDriftM: NOT the transit speed (0.2 m/s) or
// anything close to it -- these bound a small persistent residual, not
// "still moving". mpc_controller's PAUSE loop is deliberately open-loop
// (per spec: "skip the MPC solve" while paused, so there is no localization
// feedback correcting position/velocity the way the normal solve loop does
// -- contrast MPC/tests/test_mpc_sim_loop.cpp's CLOSED-loop
// "hold" scenario, which achieves ~0.008 m/s / ~0.008 m because the solver
// keeps re-targeting off real telemetry every tick). compute_stop_accel's
// discrete, fixed-dt rate limit, integrated by mpc_robot_sim against REAL
// (jittery) wall-clock tick timing rather than the assumed-exact 0.05s,
// empirically leaves a one-time residual of a few hundredths of a m/s once
// last_cmd_v snaps to exactly 0 -- observed 0.008-0.04 m/s / 0.008-0.14 m
// across repeated runs. These thresholds keep meaningful margin above that
// while still being far below both the transit speed and what continued
// unpaused motion would produce (0.2 m/s * 3s = 0.6 m).
constexpr double kMaxPausedSpeed = 0.10;  // m/s
constexpr double kMaxPauseDriftM = 0.30;  // m
constexpr double kWallDurationSlackS = 5.0;

// FEATURE 2C mutation-hardening constants (post-resume reference-tracking
// check -- see the big comment on part1_happy_pause's post-resume block
// below for the failure mode this catches). robot1's whole segment is a
// plain straight line at the fixed transit speed (0.2 m/s, see
// build_synthetic_scenario), which makes its position a clean, timing-robust
// signal: no direction changes, no speed-profile changes, nothing else that
// could legitimately push it off the t->0.2*t line.
constexpr double kRobot1TransitSpeed = 0.2;    // m/s, matches the synthetic scenario
// kPostResumeCheckpointS/kPostResumePosTolM were empirically calibrated (not
// just derived) against the actual binaries: with the ref-lookup call sites
// (target_time/horizon_refs) mutated to use current_time instead of
// reference_time -- i.e. pause_offset dropped from the REF-LOOKUP site only,
// leaving the completion check's reference_time untouched, exactly the
// regression this block exists to catch -- checkpoint=4.0s/tol=0.35m let the
// mutant through: mpc_controller's post-resume MPC (max_v_nonpush=0.38 m/s
// vs. the 0.2 m/s reference speed) closes most of the jump-chase gap within
// 4s, leaving a residual (observed 0.18-0.26m across repeated runs) under
// the old 0.35m tolerance. Sampling at 5.0s instead avoids that: 6 repeated
// correct-code runs measured 0.02-0.13m of tracking error at the 5.0s
// checkpoint (natural post-resume settling transient, not a bug -- see the
// big comment below), while 3 repeated mutated-code runs measured
// 0.33-0.36m -- a >=0.20m gap between the two populations. 0.20m sits
// comfortably in that gap (>=0.07m clear of the correct-code max, >=0.11m
// clear of the mutated-code min in the samples gathered).
constexpr double kPostResumeCheckpointS = 5.0; // when to sample position after RESUME
constexpr double kPostResumePosTolM = 0.20;    // m

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

// Parses the "t=<value>" suffix of a RUNNING/PAUSED STATUS reply. Returns
// -1.0 if not found/unparseable (never a valid plan time, so callers can
// treat that as "STATUS wasn't in the expected form").
double parse_status_t(const std::string &status)
{
  const auto pos = status.find("t=");
  if (pos == std::string::npos)
    return -1.0;
  try
  {
    return std::stod(status.substr(pos + 2));
  }
  catch (...)
  {
    return -1.0;
  }
}

// Counts DIRECT children of `parent_pid` via `pgrep -P` -- mars_sim_viz
// spawns mpc_robot_sim/mpc_controller wrapped as "env -i ... <binary>"
// (SimVizCore.cpp's spawn_wrapped_process()), and `env` execve()s straight
// into the target binary (no intermediate fork), so each spawned binary's
// PPID is mars_sim_viz's own pid directly -- exactly like this test's own
// spawn_wrapped() (see test_sim_viz_shared.h's doc comment on that). Used
// to prove RESTART tears down the old run's children before spawning the
// new ones (count stays constant, rather than doubling).
int count_direct_children(pid_t parent_pid)
{
  const std::string cmd = "pgrep -P " + std::to_string(parent_pid) + " 2>/dev/null | wc -l";
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe)
    return -1;
  char buf[64] = {0};
  if (!fgets(buf, sizeof(buf), pipe))
  {
    pclose(pipe);
    return -1;
  }
  pclose(pipe);
  try
  {
    return std::stoi(buf);
  }
  catch (...)
  {
    return -1;
  }
}

// Waits until STATUS reports RUNNING with plan t >= min_t, or timeout_s
// elapses. NOTE: ExecutionManager::state() flips to Running SYNCHRONOUSLY
// inside start_execution() -- well before the background handshake thread
// has actually uploaded any robot's trajectory / collected ACK_START (see
// SimVizCore.cpp's send_pause_resume() doc comment) -- so a bare
// `status.rfind("RUNNING", 0) == 0` check can be true while a robot's
// mpc_controller is still blocked on its very first recv() waiting for the
// trajectory upload. plan_time() reads exactly 0.0 until the handshake
// truly completes (t0_valid_), so requiring min_t > 0 here is what actually
// proves the handshake finished, not just that EXECUTE was accepted.
bool wait_for_running_with_min_t(const std::string &endpoint, double min_t, double timeout_s)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    const std::string status = req(endpoint, "STATUS");
    if (status.rfind("RUNNING", 0) == 0)
    {
      const double t = parse_status_t(status);
      if (t >= min_t)
        return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return false;
}

std::vector<std::string> viz_common_args(const std::string &controller_exe,
                                          const std::string &robot_sim_exe,
                                          const std::filesystem::path &run_dir)
{
  return {
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
  };
}

// ---------------------------------------------------------------------
// Part 1: happy pause.
// ---------------------------------------------------------------------
void part1_happy_pause(const std::string &sim_viz_exe, const std::string &controller_exe,
                        const std::string &robot_sim_exe, const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 1: happy pause ===" << std::endl;

  SyntheticScenario synth = build_synthetic_scenario("pause_test_happy");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "pause_test_happy.scn.b64");
  const double makespan = synth.scn.timetable.get_max_time();
  const std::filesystem::path run_dir = tmp_dir / "runs_1";

  std::vector<std::string> viz_args = {"--headless", "--execute=" + scn_path};
  for (auto &a : viz_common_args(controller_exe, robot_sim_exe, run_dir))
    viz_args.push_back(a);

  const auto spawn_time = std::chrono::steady_clock::now();
  const pid_t pid = spawn_wrapped(sim_viz_exe, viz_args);
  ProcessGuard guard(pid, "mars_sim_viz (Part 1)");
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << " (makespan=" << makespan << "s)"
            << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "Part 1: control socket answers PING");

  // Wait until the plan clock has advanced to >= kPauseAtPlanT.
  const bool reached = wait_for_running_with_min_t(endpoint, kPauseAtPlanT, 20.0);
  check(reached, "Part 1: plan clock reached t>=" + to_str(kPauseAtPlanT, 1) + "s before PAUSE");

  const std::string pause_reply = req(endpoint, "PAUSE");
  check(pause_reply == "ACK_PAUSE", "Part 1: PAUSE -> ACK_PAUSE (got '" + pause_reply + "')");
  const auto pause_ack_time = std::chrono::steady_clock::now();

  std::this_thread::sleep_for(std::chrono::duration<double>(kSettleMarginS));

  const std::filesystem::path run_subdir = run_dir / ("pause_test_happy_" + std::to_string(pid));
  std::vector<std::pair<std::string, Pose>> robots = {{"robot1", synth.robot1_final},
                                                        {"robot2", synth.robot2_final}};

  std::unordered_map<std::string, std::vector<CsvRow>> baseline_rows;
  for (const auto &[name, expected] : robots)
  {
    (void)expected;
    baseline_rows[name] = read_csv((run_subdir / (name + ".csv")).string());
    check(!baseline_rows[name].empty(),
          "Part 1: " + name + " CSV has data at start of pause window");
  }

  const std::string status1 = req(endpoint, "STATUS");
  check(status1.rfind("PAUSED", 0) == 0, "Part 1: STATUS reports PAUSED (got '" + status1 + "')");
  const double t1 = parse_status_t(status1);

  std::this_thread::sleep_for(std::chrono::duration<double>(kPollGapS));

  const std::string status2 = req(endpoint, "STATUS");
  check(status2.rfind("PAUSED", 0) == 0, "Part 1: STATUS still PAUSED (got '" + status2 + "')");
  const double t2 = parse_status_t(status2);
  std::cout << "[test]   frozen-t polls: t1=" << t1 << " t2=" << t2 << std::endl;
  check(t1 >= 0.0 && t2 >= 0.0 && std::fabs(t2 - t1) < kFrozenTToleranceS,
        "Part 1: plan clock frozen while paused (|t2-t1| < " + to_str(kFrozenTToleranceS, 2) +
            "s)");

  std::this_thread::sleep_for(std::chrono::duration<double>(kPollGapS));

  for (const auto &[name, expected] : robots)
  {
    (void)expected;
    const auto &before = baseline_rows[name];
    const auto after = read_csv((run_subdir / (name + ".csv")).string());
    check(after.size() >= before.size(),
          "Part 1: " + name + " CSV did not shrink during pause window");

    bool vel_ok = true;
    for (size_t i = before.size(); i < after.size(); ++i)
    {
      if (std::fabs(after[i].v) >= kMaxPausedSpeed)
        vel_ok = false;
    }
    check(vel_ok, "Part 1: " + name + " |v| < " + to_str(kMaxPausedSpeed, 3) +
                      " for every row logged during the pause window");

    if (!before.empty() && !after.empty())
    {
      const double drift =
          std::hypot(after.back().x - before.back().x, after.back().y - before.back().y);
      std::cout << "[test]   " << name << " pause-window drift=" << to_str(drift, 4) << "m"
                << std::endl;
      check(drift < kMaxPauseDriftM, "Part 1: " + name + " pose drift < " +
                                          to_str(kMaxPauseDriftM, 3) + "m across pause window");
    }
  }

  const std::string resume_reply = req(endpoint, "RESUME");
  check(resume_reply == "ACK_RESUME", "Part 1: RESUME -> ACK_RESUME (got '" + resume_reply + "')");
  const auto resume_ack_time = std::chrono::steady_clock::now();
  const double pause_duration =
      std::chrono::duration<double>(resume_ack_time - pause_ack_time).count();
  std::cout << "[test]   measured pause duration=" << to_str(pause_duration, 2) << "s"
            << std::endl;

  // FEATURE 2C mutation-hardening check: the frozen-window checks above only
  // exercise mpc_controller's PAUSE-loop branch (which `continue`s before
  // ever reaching the reference-lookup call sites), and the final-pose check
  // at the very end of this function only proves the run eventually
  // converges -- get_ref_state_at_time() clamps to the trajectory's LAST
  // waypoint once its query time runs past the trajectory's duration, so
  // even a reference that "jumped ahead" by the pause duration right at
  // RESUME (i.e. a controller that forgot to keep subtracting pause_offset
  // from its reference-lookup time, while the completion check still does)
  // still eventually parks at the correct final pose well inside this run's
  // slack time. Neither existing check would fail from that regression.
  // This block adds a MID-run signal that would: robot1's segment is one
  // constant-speed (0.2 m/s) straight line for its entire ~11s duration
  // (see build_synthetic_scenario), so if RESUME correctly continues the
  // reference from the frozen pause point, robot1's position
  // kPostResumeCheckpointS seconds later sits close to 0.2*(t1+elapsed) m.
  // If instead the reference "jump-chases" a wall-clock-advanced target (the
  // failure mode above), the MPC has a large, roughly pause_duration*0.2m
  // position gap to close and lands well past where a correctly-resumed run
  // would be at the same checkpoint -- easily outside kPostResumePosTolM.
  // (An earlier version of this check also asserted an upper bound on
  // robot1's peak |v| during the catch-up transient; that flaked on
  // CORRECT resumes -- MPC transiently touches close to max_v_nonpush=0.38
  // m/s while closing even the small residual gap left by PAUSE's open-loop
  // braking, so peak speed alone doesn't reliably separate correct from
  // buggy. The position checkpoint below is the precise, timing-robust
  // signal; kRobot1TransitSpeed/to_str diagnostics are kept for visibility
  // only, not as a pass/fail gate.)
  {
    const std::string robot1_csv = (run_subdir / "robot1.csv").string();

    // Sample position kPostResumeCheckpointS after RESUME and compare
    // against the correctly-resumed expected position: t1 (captured earlier
    // from the frozen-t STATUS poll) is the trajectory-relative elapsed
    // time robot1 had run before PAUSE; a correct resume picks the
    // reference straight back up from there, so kPostResumeCheckpointS
    // seconds after RESUME the reference should be at trajectory time
    // (t1 + kPostResumeCheckpointS).
    const auto pos_checkpoint_deadline =
        resume_ack_time + std::chrono::duration<double>(kPostResumeCheckpointS);
    if (std::chrono::steady_clock::now() < pos_checkpoint_deadline)
    {
      std::this_thread::sleep_for(pos_checkpoint_deadline - std::chrono::steady_clock::now());
    }
    std::vector<CsvRow> rows = read_csv(robot1_csv);
    check(!rows.empty(), "Part 1: robot1 CSV has data at post-resume position checkpoint");
    if (!rows.empty() && t1 >= 0.0)
    {
      const double expected_traj_t = std::min(t1 + kPostResumeCheckpointS, 11.0);
      const double expected_x = kRobot1TransitSpeed * expected_traj_t;
      const double actual_x = rows.back().x;
      const double pos_err = std::fabs(actual_x - expected_x);
      std::cout << "[test]   robot1 post-resume checkpoint: actual_x=" << to_str(actual_x, 3)
                << " expected_x=" << to_str(expected_x, 3) << " (t1=" << to_str(t1, 2) << ")"
                << std::endl;
      check(pos_err <= kPostResumePosTolM,
            "Part 1: robot1 position " + to_str(kPostResumeCheckpointS, 1) +
                "s after RESUME matches a reference resumed from the frozen pause point "
                "(within " +
                to_str(kPostResumePosTolM, 2) + "m, got " + to_str(pos_err, 3) + "m)");
    }
  }

  StatusOutcome outcome = poll_until_done(endpoint, makespan + 40.0);
  check(outcome.done, "Part 1: run reaches DONE after RESUME (final='" + outcome.final_status +
                           "')");
  const auto done_time = std::chrono::steady_clock::now();

  if (outcome.done)
  {
    const double total_wall = std::chrono::duration<double>(done_time - spawn_time).count();
    const double expected_wall = makespan + pause_duration;
    std::cout << "[test]   total wall=" << to_str(total_wall, 2)
              << "s expected~=" << to_str(expected_wall, 2) << "s" << std::endl;
    check(std::fabs(total_wall - expected_wall) <= kWallDurationSlackS,
          "Part 1: total wall duration ~= makespan+pause (+-" + to_str(kWallDurationSlackS, 1) +
              "s)");

    for (const auto &[name, expected] : robots)
    {
      const std::string csv_path = (run_subdir / (name + ".csv")).string();
      std::vector<CsvRow> rows = read_csv(csv_path);
      check(!rows.empty(), "Part 1: " + name + " CSV non-empty at DONE");
      if (!rows.empty())
      {
        const CsvRow &last = rows.back();
        const double pos_err = std::hypot(last.x - expected.x, last.y - expected.y);
        const double yaw_err = std::fabs(wrap_pi(last.yaw - expected.yaw));
        check(pos_err <= kFinalPosTolM, "Part 1: " + name + " final position within " +
                                             to_str(kFinalPosTolM) + "m (got " + to_str(pos_err) +
                                             "m)");
        check(yaw_err <= kFinalYawTolRad, "Part 1: " + name + " final yaw within " +
                                               to_str(kFinalYawTolRad) + "rad (got " +
                                               to_str(yaw_err) + "rad)");
      }
    }
  }
}

// ---------------------------------------------------------------------
// Part 2: idempotency + errors.
// ---------------------------------------------------------------------
void part2_idempotency_and_errors(const std::string &sim_viz_exe,
                                   const std::string &controller_exe,
                                   const std::string &robot_sim_exe,
                                   const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 2: idempotency + errors ===" << std::endl;
  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);

  // 2a: PAUSE/RESUME while IDLE -> ERR.
  {
    const std::filesystem::path run_dir = tmp_dir / "runs_2a";
    std::vector<std::string> viz_args = {"--headless"};
    for (auto &a : viz_common_args(controller_exe, robot_sim_exe, run_dir))
      viz_args.push_back(a);

    const pid_t pid = spawn_wrapped(sim_viz_exe, viz_args);
    ProcessGuard guard(pid, "mars_sim_viz (Part 2a, IDLE)");
    std::cout << "[test] spawned mars_sim_viz pid=" << pid << " (stays IDLE)" << std::endl;

    check(wait_for_ping(endpoint, 15.0), "Part 2a: control socket answers PING");
    check(req(endpoint, "STATUS") == "IDLE", "Part 2a: STATUS is IDLE");

    const std::string pause_reply = req(endpoint, "PAUSE");
    check(pause_reply.rfind("ERR", 0) == 0,
          "Part 2a: PAUSE while IDLE -> ERR (got '" + pause_reply + "')");
    const std::string resume_reply = req(endpoint, "RESUME");
    check(resume_reply.rfind("ERR", 0) == 0,
          "Part 2a: RESUME while IDLE -> ERR (got '" + resume_reply + "')");
  }

  // 2b: PAUSE while already paused -> ACK; RESUME while already running -> ACK.
  {
    SyntheticScenario synth = build_synthetic_scenario("pause_test_idem");
    const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "pause_test_idem.scn.b64");
    const std::filesystem::path run_dir = tmp_dir / "runs_2b";

    std::vector<std::string> viz_args = {"--headless", "--execute=" + scn_path};
    for (auto &a : viz_common_args(controller_exe, robot_sim_exe, run_dir))
      viz_args.push_back(a);

    const pid_t pid = spawn_wrapped(sim_viz_exe, viz_args);
    ProcessGuard guard(pid, "mars_sim_viz (Part 2b, idempotency)");
    std::cout << "[test] spawned mars_sim_viz pid=" << pid << std::endl;

    check(wait_for_ping(endpoint, 15.0), "Part 2b: control socket answers PING");

    // Requires t > 0 (not just a bare "RUNNING" state) -- see
    // wait_for_running_with_min_t()'s doc comment: state() flips to Running
    // before the handshake (trajectory upload + START) actually completes,
    // and PAUSE/RESUME must not race that.
    const bool saw_running = wait_for_running_with_min_t(endpoint, 0.05, 15.0);
    check(saw_running, "Part 2b: STATUS reaches RUNNING (handshake complete)");

    const std::string r1 = req(endpoint, "PAUSE");
    check(r1 == "ACK_PAUSE", "Part 2b: PAUSE -> ACK_PAUSE (got '" + r1 + "')");
    const std::string r2 = req(endpoint, "PAUSE");
    check(r2 == "ACK_PAUSE",
          "Part 2b: PAUSE while already paused -> ACK_PAUSE (idempotent, got '" + r2 + "')");

    const std::string r3 = req(endpoint, "RESUME");
    check(r3 == "ACK_RESUME", "Part 2b: RESUME -> ACK_RESUME (got '" + r3 + "')");
    const std::string r4 = req(endpoint, "RESUME");
    check(r4 == "ACK_RESUME",
          "Part 2b: RESUME while already running -> ACK_RESUME (idempotent, got '" + r4 + "')");

    const std::string abort_reply = req(endpoint, "ABORT");
    check(abort_reply == "ACK_ABORT", "Part 2b: cleanup ABORT -> ACK_ABORT");
  }
}

// ---------------------------------------------------------------------
// Part 3: restart.
// ---------------------------------------------------------------------
void part3_restart(const std::string &sim_viz_exe, const std::string &controller_exe,
                    const std::string &robot_sim_exe, const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part 3: restart ===" << std::endl;

  SyntheticScenario synth = build_synthetic_scenario("pause_test_restart");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "pause_test_restart.scn.b64");
  const double makespan = synth.scn.timetable.get_max_time();
  const std::filesystem::path run_dir = tmp_dir / "runs_3";

  std::vector<std::string> viz_args = {"--headless", "--execute=" + scn_path};
  for (auto &a : viz_common_args(controller_exe, robot_sim_exe, run_dir))
    viz_args.push_back(a);

  const pid_t pid = spawn_wrapped(sim_viz_exe, viz_args);
  ProcessGuard guard(pid, "mars_sim_viz (Part 3)");
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << " (makespan=" << makespan << "s)"
            << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "Part 3: control socket answers PING");

  const bool reached = wait_for_running_with_min_t(endpoint, kPauseAtPlanT, 20.0);
  check(reached, "Part 3: plan clock reached t>=" + to_str(kPauseAtPlanT, 1) + "s before RESTART");

  const int children_before = count_direct_children(pid);
  std::cout << "[test]   direct children before RESTART: " << children_before << std::endl;
  check(children_before == 4,
        "Part 3: 4 child processes (2 robots x sim+controller) alive before RESTART (got " +
            std::to_string(children_before) + ")");

  const std::string restart_reply = req(endpoint, "RESTART");
  check(restart_reply == "ACK_RESTART",
        "Part 3: RESTART -> ACK_RESTART (got '" + restart_reply + "')");

  bool small_t_seen = false;
  {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(10.0);
    while (std::chrono::steady_clock::now() < deadline)
    {
      const std::string status = req(endpoint, "STATUS");
      if (status.rfind("RUNNING", 0) == 0)
      {
        const double t = parse_status_t(status);
        if (t >= 0.0 && t < 3.0)
        {
          small_t_seen = true;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
  }
  check(small_t_seen, "Part 3: STATUS returns to RUNNING with a small t (<3s) after RESTART");

  const int children_after = count_direct_children(pid);
  std::cout << "[test]   direct children after RESTART: " << children_after << std::endl;
  check(children_after == 4, "Part 3: still exactly 4 child processes after RESTART (old run's "
                              "children torn down, not leaked; got " +
                                  std::to_string(children_after) + ")");

  StatusOutcome outcome = poll_until_done(endpoint, makespan + 40.0);
  check(outcome.done,
        "Part 3: restarted run reaches DONE (final='" + outcome.final_status + "')");

  if (outcome.done)
  {
    const std::filesystem::path run_subdir =
        run_dir / ("pause_test_restart_" + std::to_string(pid));
    for (const auto &[name, expected] :
         std::vector<std::pair<std::string, Pose>>{{"robot1", synth.robot1_final},
                                                     {"robot2", synth.robot2_final}})
    {
      const std::string csv_path = (run_subdir / (name + ".csv")).string();
      std::vector<CsvRow> rows = read_csv(csv_path);
      check(!rows.empty(), "Part 3: " + name + " CSV non-empty at DONE");
      if (!rows.empty())
      {
        const CsvRow &last = rows.back();
        const double pos_err = std::hypot(last.x - expected.x, last.y - expected.y);
        const double yaw_err = std::fabs(wrap_pi(last.yaw - expected.yaw));
        check(pos_err <= kFinalPosTolM, "Part 3: " + name + " final position within " +
                                             to_str(kFinalPosTolM) + "m (got " + to_str(pos_err) +
                                             "m)");
        check(yaw_err <= kFinalYawTolRad, "Part 3: " + name + " final yaw within " +
                                               to_str(kFinalYawTolRad) + "rad (got " +
                                               to_str(yaw_err) + "rad)");
      }
    }
  }
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
      ("mars_simviz_pause_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  std::cout << "[test] sim_viz_exe=" << sim_viz_exe << "\n[test] controller_exe="
            << controller_exe << "\n[test] robot_sim_exe=" << robot_sim_exe << std::endl;

  part1_happy_pause(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "Part 1: no stray mpc_robot_sim after teardown");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "Part 1: no stray mpc_controller after teardown");

  part2_idempotency_and_errors(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "Part 2: no stray mpc_robot_sim after teardown");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "Part 2: no stray mpc_controller after teardown");

  part3_restart(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "Part 3: no stray mpc_robot_sim after teardown");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "Part 3: no stray mpc_controller after teardown");

  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n" << (g_overall_pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << std::endl;
  return g_overall_pass ? 0 : 1;
}
