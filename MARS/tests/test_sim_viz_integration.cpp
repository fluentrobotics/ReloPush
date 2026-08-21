// Phase B1 HEADLESS INTEGRATION TEST for mars_sim_viz.
//
// Spawns the REAL mars_sim_viz --headless binary (never in-process) plus,
// transitively (mars_sim_viz spawns them), real mpc_controller /
// mpc_robot_sim children -- exercising the full PING/EXECUTE/STATUS
// protocol end-to-end against a synthetic, short (~14s makespan) scenario
// built programmatically (2 robots, one transfer span moving one object)
// and serialized to a temp .scn.b64 file, exactly the artifact MARS's own
// SimVizHandoff.cpp would hand off in production.
//
// Standalone diagnostic binary (not wired into ctest), mirroring
// MPC/tests/test_mpc_sim_loop.cpp's style -- including its
// ProcessGuard (copied here almost verbatim; see that file's header
// comment for the rationale) and self_dir() sibling-executable resolution.
// The process-spawn/ProcessGuard/scenario-builder plumbing itself now
// lives in test_sim_viz_shared.h, shared with test_sim_viz_execute.cpp
// (the --execute/--exit-on-done direct-load test).
//
// TEST PORT BLOCK (per this task's port allocation -- never the production
// defaults, never test_mpc_sim_loop.cpp's own 45111/45161/45261 block):
//   control = 45601, handshake = 45620+idx, vesc = 45640+idx,
//   localization = 45660+idx.
//
// Assertions:
//   1. PING -> PONG, EXECUTE -> ACK_EXECUTE, STATUS polls IDLE->RUNNING->DONE
//      within makespan+20s.
//   2. Each robot's mpc_robot_sim --log-csv final pose is within 0.10m /
//      0.15rad of that robot's final timetable pose (ground truth: what was
//      actually integrated, exactly like test_mpc_sim_loop.cpp's own
//      assertion style).
//   3. No mpc_controller/mpc_robot_sim process bound to this test's ports
//      remains alive after DONE (clean reap).
//   4. A SECOND EXECUTE on the SAME already-used mars_sim_viz instance
//      succeeds and completes (reusability without restart).
//   5. EXECUTE with a bogus path -> "ERR ..." and the manager remains
//      usable afterward (a subsequent valid EXECUTE still succeeds).

#include "test_sim_viz_shared.h"

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

bool DEBUG_VIS = false;

using namespace simviz_test;

namespace
{

constexpr int kControlPort = 45601;
constexpr int kHandshakePortStart = 45620;
constexpr int kVescPortStart = 45640;
constexpr int kLocPortStart = 45660;

} // namespace

int main()
{
  bool overall_pass = true;
  auto check = [&](bool cond, const std::string &what) {
    if (!cond)
    {
      std::cerr << "[test] FAILED: " << what << std::endl;
      overall_pass = false;
    }
    else
    {
      std::cout << "[test] ok: " << what << std::endl;
    }
  };

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
      ("mars_simviz_integration_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::filesystem::path run_dir = tmp_dir / "runs";

  SyntheticScenario synth = build_synthetic_scenario("integration_test_scenario");
  const std::string scn_path =
      write_scn_file(synth.scn, tmp_dir / "integration_test_scenario.scn.b64");
  const double makespan = synth.scn.timetable.get_max_time();
  std::cout << "[test] synthetic scenario makespan=" << makespan << "s, written to " << scn_path
            << std::endl;

  std::cout << "[test] sim_viz_exe=" << sim_viz_exe << "\n[test] controller_exe="
            << controller_exe << "\n[test] robot_sim_exe=" << robot_sim_exe << std::endl;

  std::vector<std::string> viz_args = {
      "--headless",
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
  };
  ProcessGuard viz_guard(spawn_wrapped(sim_viz_exe, viz_args), "mars_sim_viz");
  std::cout << "[test] spawned mars_sim_viz pid=" << viz_guard.pid() << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);

  if (!wait_for_ping(endpoint, 10.0))
  {
    std::cerr << "[test] FATAL: mars_sim_viz never answered PING within 10s" << std::endl;
    return 1;
  }
  check(true, "PING -> PONG (mars_sim_viz is up)");

  // ---- Run 1: full EXECUTE -> DONE, then verify CSVs + clean reap. ----
  {
    std::string reply = req(endpoint, "EXECUTE " + scn_path);
    check(reply == "ACK_EXECUTE", "run1 EXECUTE -> ACK_EXECUTE (" + reply + ")");

    StatusOutcome outcome = poll_until_done(endpoint, makespan + 20.0);
    check(outcome.done, "run1 STATUS eventually DONE (final=" + outcome.final_status + ")");

    if (outcome.done)
    {
      // run_dir/<sanitized label>_<mars_sim_viz pid>/<robot>.csv -- see
      // ExecutionManager::start_execution().
      const std::filesystem::path run_subdir =
          run_dir / ("integration_test_scenario_" + std::to_string(viz_guard.pid()));

      for (const auto &[robot_name, expected_final] :
           std::vector<std::pair<std::string, Pose>>{{"robot1", synth.robot1_final},
                                                       {"robot2", synth.robot2_final}})
      {
        const std::string csv_path = (run_subdir / (robot_name + ".csv")).string();
        std::vector<CsvRow> rows = read_csv(csv_path);
        check(!rows.empty(), robot_name + " CSV non-empty (" + csv_path + ")");
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
          check(pos_err <= kFinalPosTolM,
                robot_name + " final position within " + to_str(kFinalPosTolM) + "m (got " +
                    to_str(pos_err) + "m)");
          check(yaw_err <= kFinalYawTolRad,
                robot_name + " final yaw within " + to_str(kFinalYawTolRad) + "rad (got " +
                    to_str(yaw_err) + "rad)");
        }
      }
    }

    // Give the OS a moment to finish reaping mars_sim_viz's grandchildren
    // (ExecutionManager already waitForFinished()'d them before reporting
    // DONE, but pgrep may need a beat to reflect that).
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
          "no stray mpc_robot_sim bound to this test's vesc port block after DONE");
    check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
          "no stray mpc_controller bound to this test's handshake port block after DONE");
  }

  // ---- Run 2: reusability -- EXECUTE again on the SAME instance. ----
  {
    std::string reply = req(endpoint, "EXECUTE " + scn_path);
    check(reply == "ACK_EXECUTE", "run2 (reusability) EXECUTE -> ACK_EXECUTE (" + reply + ")");
    StatusOutcome outcome = poll_until_done(endpoint, makespan + 20.0);
    check(outcome.done, "run2 STATUS eventually DONE (final=" + outcome.final_status + ")");
  }

  // ---- Bogus EXECUTE -> ERR, manager remains usable afterward. ----
  {
    std::string reply = req(endpoint, "EXECUTE /nonexistent/path/does_not_exist.scn.b64");
    check(reply.rfind("ERR", 0) == 0, "bogus EXECUTE -> ERR (" + reply + ")");

    // Still usable: a subsequent valid EXECUTE succeeds. We only need
    // ACK_EXECUTE here (run1/run2 already proved full completion); abort
    // immediately after to keep the test's total runtime bounded.
    std::string reply2 = req(endpoint, "EXECUTE " + scn_path);
    check(reply2 == "ACK_EXECUTE",
          "EXECUTE after bogus EXECUTE still works -> ACK_EXECUTE (" + reply2 + ")");
    std::string abort_reply = req(endpoint, "ABORT");
    check(abort_reply == "ACK_ABORT", "final ABORT -> ACK_ABORT (" + abort_reply + ")");
  }

  // viz_guard's destructor (end of main) SIGTERMs mars_sim_viz by its exact
  // PID. mars_sim_viz now installs a SIGTERM/SIGINT handler
  // (simviz_main.cpp) that tears down ALL spawned grandchildren via
  // ExecutionManager::abort() before exiting, so this is safe even if a run
  // were still active; here the ABORT above already reaped the last active
  // run's children, and runs 1-2 already completed (DONE) and were reaped
  // internally, so this SIGTERM has nothing left to clean up in practice.
  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n" << (overall_pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << std::endl;
  return overall_pass ? 0 : 1;
}
