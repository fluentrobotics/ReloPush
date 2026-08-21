// mars_sim_viz --execute / --exit-on-done direct-load test.
//
// Covers the new CLI surface added to MARS/src/simviz/simviz_main.cpp: a
// saved MARS result file (.scn.b64) can be loaded and run directly at
// startup, with no MARS process and no control-socket doorbell, in headless
// mode. Reuses the process-spawn/ProcessGuard/synthetic-scenario-builder
// plumbing factored out of test_sim_viz_integration.cpp into
// test_sim_viz_shared.h; see that file's header comment for the shared
// harness design.
//
// TEST PORT BLOCK (same block as test_sim_viz_integration.cpp -- per this
// task's port allocation; each part below fully reaps its mars_sim_viz
// instance before the next part spawns a new one, so port reuse across
// parts is safe):
//   control = 45601, handshake = 45620+idx, vesc = 45640+idx,
//   localization = 45660+idx.
//
// Parts:
//   A. Direct-load happy path: `--headless --execute=<fixture>
//      --exit-on-done` on the synthetic ~14s-makespan scenario -> process
//      exits 0 within makespan+30s; per-robot sim CSV final pose within
//      0.10m/0.15rad of the timetable's final pose; no stray children after
//      exit.
//   B. Direct-load bad path: `--headless --execute=/nonexistent...
//      --exit-on-done` -> nonzero exit within ~10s, stderr contains the
//      failure reason.
//   C. Control-socket coexistence: during a `--execute`-launched run (no
//      --exit-on-done, so the process stays up), STATUS answers RUNNING;
//      ABORT tears it down (ACK_ABORT, then STATUS -> IDLE).
//   D. Real MARS output: ScenarioModel::load_from_file() (linked directly,
//      not via a spawned process) on the genuine
//      results/sim_handoff/greedy.scn.b64 produced by the live end-to-end
//      smoke -- prints robot/object counts + makespan, does NOT execute it
//      to completion (makespan is hundreds of seconds). Then, to prove the
//      user-facing command works on genuine MARS output, a bounded ~15s
//      `--headless --execute=<greedy file>` run (no --exit-on-done) is
//      spawned and torn down via ABORT before its makespan is reached.

#include "SimVizCore.h"
#include "test_sim_viz_shared.h"

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

bool DEBUG_VIS = false;

#ifndef MARS_TEST_GREEDY_HANDOFF_SCN
#define MARS_TEST_GREEDY_HANDOFF_SCN ""
#endif
#ifndef MARS_TEST_FIXTURE_SCN
#define MARS_TEST_FIXTURE_SCN ""
#endif

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

// Spawns `exe args...` wrapped in "env -i HOME=.. USER=.. PATH=..
// QT_QPA_PLATFORM=offscreen" (same wrapping as simviz_test::spawn_wrapped()),
// additionally redirecting the child's stderr to `stderr_path` (truncated/
// created) so a caller can inspect the failure reason a headless
// --execute-that-fails prints, per this task's spec item 3b. Because `env`
// execve()s straight into the target binary (no intermediate shell), the
// returned pid is mars_sim_viz's own pid, exactly like spawn_wrapped().
pid_t spawn_wrapped_capture_stderr(const std::string &exe, const std::vector<std::string> &args,
                                    const std::string &stderr_path)
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
    int fd = open(stderr_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (fd >= 0)
    {
      dup2(fd, STDERR_FILENO);
      close(fd);
    }
    execv(env_exe.c_str(), argv.data());
    std::fprintf(stderr, "[test] execv('%s') failed\n", env_exe.c_str());
    _exit(127);
  }
  return pid;
}

struct WaitOutcome
{
  bool exited = false;
  int exit_code = -1;
  bool timed_out = false;
};

// Polls waitpid(WNOHANG) for `pid` until it exits or `timeout_s` elapses.
// On timeout, does NOT touch the process -- callers decide whether to
// SIGKILL-and-reap (this test always does, to avoid leaving a stray
// mars_sim_viz around on assertion failure).
WaitOutcome wait_for_pid_exit(pid_t pid, double timeout_s)
{
  WaitOutcome out;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    int status = 0;
    pid_t r = waitpid(pid, &status, WNOHANG);
    if (r == pid)
    {
      out.exited = true;
      out.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      return out;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  out.timed_out = true;
  return out;
}

void kill_and_reap(pid_t pid)
{
  if (pid <= 0)
    return;
  kill(pid, SIGKILL);
  int status = 0;
  waitpid(pid, &status, 0);
}

std::string read_file(const std::string &path)
{
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// ---------------------------------------------------------------------
// Part A: direct-load happy path.
// ---------------------------------------------------------------------
void part_a_happy_path(const std::string &sim_viz_exe, const std::string &controller_exe,
                        const std::string &robot_sim_exe, const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part A: --execute happy path (--exit-on-done) ===" << std::endl;

  SyntheticScenario synth = build_synthetic_scenario("execute_test_happy");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "execute_test_happy.scn.b64");
  const double makespan = synth.scn.timetable.get_max_time();
  const std::filesystem::path run_dir = tmp_dir / "runs_a";

  std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + scn_path,
      "--exit-on-done",
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
  };
  const pid_t pid = spawn_wrapped(sim_viz_exe, viz_args);
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << " (makespan=" << makespan << "s)"
            << std::endl;

  WaitOutcome outcome = wait_for_pid_exit(pid, makespan + 30.0);
  if (outcome.timed_out)
  {
    std::cerr << "[test] mars_sim_viz did not exit within makespan+30s; killing" << std::endl;
    kill_and_reap(pid);
  }
  check(outcome.exited && !outcome.timed_out,
        "process exited on its own within makespan+30s (--exit-on-done)");
  check(outcome.exited && outcome.exit_code == 0,
        "exit code 0 (got " + std::to_string(outcome.exit_code) + ")");

  if (outcome.exited && outcome.exit_code == 0)
  {
    const std::filesystem::path run_subdir =
        run_dir / ("execute_test_happy_" + std::to_string(pid));
    for (const auto &[robot_name, expected_final] :
         std::vector<std::pair<std::string, Pose>>{{"robot1", synth.robot1_final},
                                                     {"robot2", synth.robot2_final}})
    {
      const std::string csv_path = (run_subdir / (robot_name + ".csv")).string();
      std::vector<CsvRow> rows = read_csv(csv_path);
      check(!rows.empty(), "Part A: " + robot_name + " CSV non-empty (" + csv_path + ")");
      if (!rows.empty())
      {
        const CsvRow &last = rows.back();
        const double pos_err = std::hypot(last.x - expected_final.x, last.y - expected_final.y);
        const double yaw_err = std::fabs(wrap_pi(last.yaw - expected_final.yaw));
        std::cout << "[test]   " << robot_name << " final: csv=(" << last.x << "," << last.y
                  << "," << last.yaw << ") expected=(" << expected_final.x << ","
                  << expected_final.y << "," << expected_final.yaw
                  << ") pos_err=" << to_str(pos_err) << " yaw_err=" << to_str(yaw_err)
                  << std::endl;
        check(pos_err <= kFinalPosTolM, "Part A: " + robot_name + " final position within " +
                                             to_str(kFinalPosTolM) + "m (got " + to_str(pos_err) +
                                             "m)");
        check(yaw_err <= kFinalYawTolRad, "Part A: " + robot_name + " final yaw within " +
                                               to_str(kFinalYawTolRad) + "rad (got " +
                                               to_str(yaw_err) + "rad)");
      }
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "Part A: no stray mpc_robot_sim after process exit");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "Part A: no stray mpc_controller after process exit");
}

// ---------------------------------------------------------------------
// Part B: direct-load bad path.
// ---------------------------------------------------------------------
void part_b_bad_path(const std::string &sim_viz_exe, const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part B: --execute bad path (nonexistent file) ===" << std::endl;

  const std::string bogus_path = "/nonexistent/path/does_not_exist.scn.b64";
  const std::string stderr_path = (tmp_dir / "part_b_stderr.txt").string();

  std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + bogus_path,
      "--exit-on-done",
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
  };
  const pid_t pid = spawn_wrapped_capture_stderr(sim_viz_exe, viz_args, stderr_path);
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << " (expect prompt nonzero exit)"
            << std::endl;

  WaitOutcome outcome = wait_for_pid_exit(pid, 10.0);
  if (outcome.timed_out)
  {
    std::cerr << "[test] mars_sim_viz did not exit within 10s on a bad --execute path; killing"
              << std::endl;
    kill_and_reap(pid);
  }
  check(outcome.exited && !outcome.timed_out, "process exited within ~10s on load failure");
  check(outcome.exited && outcome.exit_code != 0,
        "exit code nonzero (got " + std::to_string(outcome.exit_code) + ")");

  const std::string stderr_text = read_file(stderr_path);
  std::cout << "[test]   stderr: " << stderr_text << std::endl;
  check(stderr_text.find(bogus_path) != std::string::npos ||
            stderr_text.find("cannot open") != std::string::npos,
        "stderr contains the load-failure reason");
}

// ---------------------------------------------------------------------
// Part C: control-socket coexistence during a --execute-launched run.
// ---------------------------------------------------------------------
void part_c_control_socket_coexistence(const std::string &sim_viz_exe,
                                        const std::string &controller_exe,
                                        const std::string &robot_sim_exe,
                                        const std::filesystem::path &tmp_dir)
{
  std::cout << "\n[test] === Part C: control-socket coexistence during --execute run ==="
            << std::endl;

  SyntheticScenario synth = build_synthetic_scenario("execute_test_coexist");
  const std::string scn_path =
      write_scn_file(synth.scn, tmp_dir / "execute_test_coexist.scn.b64");
  const std::filesystem::path run_dir = tmp_dir / "runs_c";

  // NOTE: no --exit-on-done here -- the whole point of this part is that
  // the process stays up (and its control socket stays answerable) after a
  // --execute-launched run starts, exactly as if EXECUTE had come in over
  // the control socket.
  std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + scn_path,
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
  };
  ProcessGuard viz_guard(spawn_wrapped(sim_viz_exe, viz_args), "mars_sim_viz (Part C)");
  std::cout << "[test] spawned mars_sim_viz pid=" << viz_guard.pid() << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 10.0), "control socket answers PING alongside a --execute run");

  // Poll STATUS until it reports RUNNING (the handshake/spawn takes a
  // moment) or a short timeout elapses.
  bool saw_running = false;
  std::string last_status;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(10.0);
  while (std::chrono::steady_clock::now() < deadline)
  {
    last_status = req(endpoint, "STATUS");
    if (last_status.rfind("RUNNING", 0) == 0)
    {
      saw_running = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  check(saw_running, "STATUS reports RUNNING for the --execute-launched run (last=" +
                          last_status + ")");

  const std::string abort_reply = req(endpoint, "ABORT");
  check(abort_reply == "ACK_ABORT", "ABORT -> ACK_ABORT (" + abort_reply + ")");

  const std::string status_after_abort = req(endpoint, "STATUS");
  check(status_after_abort == "IDLE",
        "STATUS -> IDLE after ABORT (got '" + status_after_abort + "')");

  // viz_guard's destructor SIGTERMs the process at end of function scope.
}

// ---------------------------------------------------------------------
// Part D: validate against the real MARS-produced greedy.scn.b64.
// ---------------------------------------------------------------------
void part_d_real_mars_output(const std::string &sim_viz_exe)
{
  std::cout << "\n[test] === Part D: real MARS output (results/sim_handoff/greedy.scn.b64) ==="
            << std::endl;

  std::string fixture_path = MARS_TEST_GREEDY_HANDOFF_SCN;
  if (fixture_path.empty() || !std::filesystem::exists(fixture_path))
  {
    std::cerr << "[test]   results/sim_handoff/greedy.scn.b64 missing/not found at '"
              << fixture_path << "'; falling back to " << MARS_TEST_FIXTURE_SCN << std::endl;
    fixture_path = MARS_TEST_FIXTURE_SCN;
  }
  check(!fixture_path.empty() && std::filesystem::exists(fixture_path),
        "a real .scn.b64 fixture is available for Part D (" + fixture_path + ")");
  if (fixture_path.empty() || !std::filesystem::exists(fixture_path))
    return;

  simviz::ScenarioModel model;
  bool loaded = false;
  try
  {
    model = simviz::ScenarioModel::load_from_file(fixture_path);
    loaded = true;
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[test]   ScenarioModel::load_from_file threw: " << ex.what() << std::endl;
  }
  check(loaded, "ScenarioModel::load_from_file() deserializes " + fixture_path);
  if (!loaded)
    return;

  const size_t n_robots = model.robots_sorted_by_name().size();
  const size_t n_objects = model.objects_sorted_by_name().size();
  const double makespan = model.max_time();
  std::cout << "[test]   label='" << model.label() << "' robots=" << n_robots
            << " objects=" << n_objects << " makespan=" << makespan << "s" << std::endl;
  check(n_robots > 0, "Part D fixture has at least one robot entity");

  // Optional bounded live run: --execute the real file for ~15s, then
  // ABORT -- proves the user-facing command works on genuine MARS output
  // without waiting out its (hundreds-of-seconds) full makespan.
  const char *skip_env = std::getenv("MARS_TEST_SKIP_PART_D_LIVE_RUN");
  if (skip_env && std::string(skip_env) == "1")
  {
    std::cout << "[test]   (skipping bounded live --execute run: "
                 "MARS_TEST_SKIP_PART_D_LIVE_RUN=1)"
              << std::endl;
    return;
  }

  std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + fixture_path,
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
  };
  ProcessGuard viz_guard(spawn_wrapped(sim_viz_exe, viz_args), "mars_sim_viz (Part D)");
  std::cout << "[test] spawned mars_sim_viz pid=" << viz_guard.pid()
            << " against the real greedy.scn.b64 (bounded ~15s probe)" << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0),
        "Part D: control socket answers PING (mpc_controller/mpc_robot_sim for a real, "
        "possibly-many-robot scenario may take a little longer to come up)");

  std::this_thread::sleep_for(std::chrono::seconds(15));
  const std::string status = req(endpoint, "STATUS");
  std::cout << "[test]   STATUS after ~15s: " << status << std::endl;
  check(status.rfind("RUNNING", 0) == 0 || status.rfind("DONE", 0) == 0,
        "Part D: run is RUNNING (or already DONE, if the real makespan happened to be short) "
        "15s in (got '" +
            status + "')");

  const std::string abort_reply = req(endpoint, "ABORT");
  check(abort_reply == "ACK_ABORT", "Part D: ABORT -> ACK_ABORT (" + abort_reply + ")");
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
      ("mars_simviz_execute_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  std::cout << "[test] sim_viz_exe=" << sim_viz_exe << "\n[test] controller_exe="
            << controller_exe << "\n[test] robot_sim_exe=" << robot_sim_exe << std::endl;

  part_a_happy_path(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  part_b_bad_path(sim_viz_exe, tmp_dir);
  part_c_control_socket_coexistence(sim_viz_exe, controller_exe, robot_sim_exe, tmp_dir);
  part_d_real_mars_output(sim_viz_exe);

  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n" << (g_overall_pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << std::endl;
  return g_overall_pass ? 0 : 1;
}
