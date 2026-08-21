// MOTOR STALL: standalone (non-ctest) test for mars_sim_viz's min-speed
// deadband UI controls -- the spawn-time plumbing (ExecutionManager::
// start_execution() adding --deadband/--min-moving-speed=/
// --min-sustain-speed= to each spawned mpc_robot_sim, plus writing/passing
// a --robot-spec override to each spawned mpc_controller), the live
// sim_config publish channel (CONTRACT 2: "deadband"/"min_moving_speed"/
// "min_sustain_speed" fields on the same "/<robot>/sim_config" topic FEATURE
// C's noise sliders already use), the headless --deadband/--stall-level=
// CLI flags, and STATUS's additive " stall=off"/" stall=<L>" suffix. Same
// standalone-binary style as test_sim_viz_noise.cpp (which this file
// mirrors closely): spawns the REAL mars_sim_viz --headless binary + real
// mpc_controller/mpc_robot_sim children against the synthetic 2-robot
// scenario from test_sim_viz_shared.h (build_synthetic_scenario()).
//
// TEST PORT BLOCK: same block test_sim_viz_noise.cpp/test_sim_viz_pause.cpp/
// test_sim_viz_execute.cpp use (this test never runs concurrently with
// them -- each standalone binary is invoked and fully reaped before the
// next): control = 45601, handshake = 45620+idx, vesc = 45640+idx,
// localization = 45660+idx.
//
// DESIGN CHOICE (per this task's own guidance: "with the stall enabled,
// robots may behave differently until package B (controller launch
// governor) lands -- for THIS test either use stall-level 0.0 with deadband
// ON for the run-to-DONE part, or don't wait for DONE (verify plumbing then
// ABORT)"): this test does NOT wait for DONE at all -- it verifies spawn-time
// + live-reconfiguration plumbing while RUNNING, then ABORTs, exactly
// mirroring test_sim_viz_noise.cpp's own two-part structure (which also
// never waits for DONE).
//
// CONCURRENT-WORK NOTE: as of when this test was written, mpc_robot_sim's
// LIVE sim_config parsing only recognized "noise_sigma_pct"/
// "steer_noise_sigma_pct" (mpc::parse_sim_config_payload(), MPC/
// src/SimCore.cpp) -- the "deadband"/"min_moving_speed"/"min_sustain_speed"
// live-reconfiguration fields this test's Part 2 publishes were being added
// CONCURRENTLY by another agent and may not have landed yet. Part 2's check
// is deliberately FORMAT-AGNOSTIC (it does not assume any particular log
// wording) -- it records the sim's log size right before the live publish,
// then polls the NEWLY APPENDED tail for the distinctive value "0.16",
// which only a change-aware log line could produce. If mpc_robot_sim's live
// parsing has not landed yet, this check will time out and fail cleanly;
// rebuilding mpc_robot_sim (in case the concurrent change has landed on
// disk since this binary was last built) and re-running once should pick
// it up.
//
// Parts:
//   1. Spawn with --deadband --stall-level=0.12: verify STATUS's diagnostic
//      string includes "stall=0.12"; verify each spawned mpc_robot_sim's
//      own startup log line shows "deadband=on ... min_moving_speed=0.12
//      min_sustain_speed=0.12" (spawn-time passthrough, not just
//      mars_sim_viz's own belief about the value); verify each spawned
//      mpc_controller's own startup log shows it loaded a --robot-spec
//      override from a "stall_robot_spec_override.json" file (proving the
//      controller-side launch-tuning override reaches it, independent of
//      package B's own governor logic).
//   2. Live change: hand-PUB {"deadband": 1, "min_moving_speed": 0.16,
//      "min_sustain_speed": 0.16} directly on topic "/robot1/sim_config"
//      to robot1's vesc port (the exact wire format/endpoint
//      ExecutionManager's own set_stall_enabled()/set_stall_level()
//      combined publish uses; CONTRACT 2 explicitly allows "any subset" of
//      fields, so this deliberately omits the noise fields) and verify
//      robot1's sim log picks up the new value -- see the concurrent-work
//      note above.

#include "SimVizCore.h"
#include "test_sim_viz_shared.h"

#include <zmq.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

bool DEBUG_VIS = false;

using namespace simviz_test;

namespace
{

constexpr int kControlPort = 45601;
constexpr int kHandshakePortStart = 45620;
constexpr int kVescPortStart = 45640;
constexpr int kLocPortStart = 45660;

constexpr double kSpawnStallLevel = 0.12;
constexpr double kLiveStallLevel = 0.16; // distinct from kSpawnStallLevel, avoids substring collision.

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

// Identical to test_sim_viz_noise.cpp's spawn_wrapped_logged() -- see that
// file's header comment for why a log-file redirect (rather than just
// ForwardedChannels, which only forwards up to mars_sim_viz's OWN stdout/
// stderr) is needed to observe a grandchild's (mpc_robot_sim's/
// mpc_controller's) log lines from a separate test process. Duplicated
// per-test-binary rather than hoisted into test_sim_viz_shared.h, matching
// this project's existing convention (test_sim_viz_noise.cpp keeps its own
// copy too).
pid_t spawn_wrapped_logged(const std::string &exe, const std::vector<std::string> &args,
                            const std::string &log_path)
{
  const char *home = std::getenv("HOME");
  const char *user = std::getenv("USER");
  const std::string env_exe = "/usr/bin/env";
  std::vector<std::string> wrapped_args = {
      "-i", std::string("HOME=") + (home ? home : ""), std::string("USER=") + (user ? user : ""),
      "PATH=/usr/local/bin:/usr/bin:/bin", "QT_QPA_PLATFORM=offscreen", exe};
  for (const auto &a : args)
    wrapped_args.push_back(a);

  std::vector<char *> argv;
  argv.push_back(const_cast<char *>(env_exe.c_str()));
  for (auto &a : wrapped_args)
    argv.push_back(const_cast<char *>(a.c_str()));
  argv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0)
    throw std::runtime_error("fork() failed: " + std::string(std::strerror(errno)));
  if (pid == 0)
  {
    int fd = open(log_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd >= 0)
    {
      dup2(fd, STDOUT_FILENO);
      dup2(fd, STDERR_FILENO);
      close(fd);
    }
    execv(env_exe.c_str(), argv.data());
    std::fprintf(stderr, "[test] execv('%s') failed: %s\n", env_exe.c_str(),
                 std::strerror(errno));
    _exit(127);
  }
  return pid;
}

std::string read_log_file(const std::string &path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open())
    return "";
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// Byte length of `path`, or 0 if it doesn't exist yet -- used to mark "the
// log up to here is already-checked spawn-time content" before Part 2's
// live publish, so its format-agnostic value search (see this file's header
// comment) only looks at NEWLY APPENDED content.
std::streamoff log_size(const std::string &path)
{
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in.is_open())
    return 0;
  return in.tellg();
}

std::string read_log_tail(const std::string &path, std::streamoff from_offset)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open())
    return "";
  in.seekg(from_offset);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

bool wait_for_log_line(const std::string &log_path, const std::string &needle, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (read_log_file(log_path).find(needle) != std::string::npos)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return false;
}

bool wait_for_log_tail_containing(const std::string &log_path, std::streamoff from_offset,
                                   const std::string &needle, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (read_log_tail(log_path, from_offset).find(needle) != std::string::npos)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return false;
}

bool wait_for_running(const std::string &endpoint, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (req(endpoint, "STATUS").rfind("RUNNING", 0) == 0)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }
  return false;
}

// Hand-publishes a sim_config payload on topic "/<robot>/sim_config" to
// `port` -- the exact same wire format/endpoint ExecutionManager's own
// set_stall_enabled()/set_stall_level()/publish_sim_config() combined
// publish uses, and exactly what a real GUI checkbox/spinbox change would
// cause -- standing in for the GUI controls themselves, which cannot be
// driven headlessly. Deliberately sends ONLY the deadband fields (CONTRACT
// 2 explicitly allows "any subset" of noise_sigma_pct/steer_noise_sigma_pct/
// deadband/min_moving_speed/min_sustain_speed), mirroring what a real
// live-only stall-level change would look like.
void hand_publish_stall_config(int port, const std::string &robot_name, bool deadband,
                                double min_moving_speed, double min_sustain_speed)
{
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  pub.set(zmq::sockopt::linger, 0);
  pub.connect("tcp://127.0.0.1:" + std::to_string(port));
  const std::string topic = "/" + robot_name + "/sim_config";
  std::ostringstream payload;
  payload << "{\"deadband\":" << (deadband ? 1 : 0) << ",\"min_moving_speed\":" << min_moving_speed
          << ",\"min_sustain_speed\":" << min_sustain_speed << "}";
  const std::string payload_str = payload.str();
  // PUB/SUB has no late-joiner replay -- give the freshly-connected socket a
  // moment before the first (only) publish, same margin
  // test_sim_viz_noise.cpp's hand_publish_noise_config() uses.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  zmq::message_t topic_msg(topic.begin(), topic.end());
  zmq::message_t payload_msg(payload_str.begin(), payload_str.end());
  pub.send(topic_msg, zmq::send_flags::sndmore);
  pub.send(payload_msg, zmq::send_flags::none);
  // Give the message a moment to actually go out before the socket (and its
  // owning context) are destroyed at scope exit.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
      ("mars_simviz_stall_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  SyntheticScenario synth = build_synthetic_scenario("stall_test");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "stall_test.scn.b64");
  const std::filesystem::path run_dir = tmp_dir / "runs";
  const std::string log_path = (tmp_dir / "mars_sim_viz.log").string();

  const std::vector<std::string> viz_args = {
      "--headless",
      "--execute=" + scn_path,
      "--control-port=" + std::to_string(kControlPort),
      "--handshake-port-start=" + std::to_string(kHandshakePortStart),
      "--vesc-port-start=" + std::to_string(kVescPortStart),
      "--loc-port-start=" + std::to_string(kLocPortStart),
      "--mpc-controller-path=" + controller_exe,
      "--robot-sim-path=" + robot_sim_exe,
      "--run-dir=" + run_dir.string(),
      "--deadband",
      "--stall-level=" + to_str(kSpawnStallLevel, 2),
  };

  const pid_t pid = spawn_wrapped_logged(sim_viz_exe, viz_args, log_path);
  ProcessGuard guard(pid, "mars_sim_viz (stall test)");
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << " log=" << log_path << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "control socket answers PING");
  check(wait_for_running(endpoint, 20.0), "run reaches RUNNING");

  // ---------------------------------------------------------------------
  // Part 1: spawn-time flags reach mars_sim_viz's own STATUS, each spawned
  // mpc_robot_sim's own startup log line, and each spawned mpc_controller's
  // own startup log line (--robot-spec override).
  // ---------------------------------------------------------------------
  {
    const std::string status = req(endpoint, "STATUS");
    std::cout << "[test]   STATUS: " << status << std::endl;
    check(status.find("stall=0.12") != std::string::npos,
          "STATUS diagnostic string includes 'stall=0.12' (got '" + status + "')");

    const bool robot1_launch_seen =
        wait_for_log_line(log_path, "[SIM robot1] Launching:", 10.0);
    const bool robot2_launch_seen =
        wait_for_log_line(log_path, "[SIM robot2] Launching:", 10.0);
    check(robot1_launch_seen, "robot1's mpc_robot_sim printed its startup line");
    check(robot2_launch_seen, "robot2's mpc_robot_sim printed its startup line");

    std::string log_so_far = read_log_file(log_path);
    // robot_sim.cpp's startup line reports "deadband=on min_moving_speed=X
    // min_sustain_speed=Y" only when the effective flag is enabled (see
    // that file's startup cout) -- proving spawn-time passthrough of BOTH
    // --deadband and the explicit --min-moving-speed=/--min-sustain-speed=
    // values, not just mars_sim_viz's own belief about them. ANTI-RE-STALL
    // HYSTERESIS: ExecutionManager::start_execution() now derives sustain as
    // 0.7*level (kinetic < static friction -- see SimVizCore.cpp's
    // spawn_stall_sustain doc comment) rather than passing the level
    // unchanged to both thresholds, so kSpawnStallLevel=0.12 implies
    // min_sustain_speed=0.084 (0.7*0.12), NOT 0.12 -- min_moving_speed stays
    // exactly the level itself.
    check(log_so_far.find("deadband=on") != std::string::npos,
          "spawned mpc_robot_sim(s) actually received --deadband");
    check(log_so_far.find("min_moving_speed=0.12 min_sustain_speed=0.084") != std::string::npos,
          "spawned mpc_robot_sim(s) actually received --min-moving-speed=0.12 "
          "--min-sustain-speed=0.084 (0.7*level hysteresis)");

    // mpc_controller's own startup log (MPC/src/main.cpp):
    // "[MPC <robot>] Loaded robot spec from <path>" on successful
    // --robot-spec load; <path> ends in "stall_robot_spec_override.json"
    // (ExecutionManager::start_execution()'s MOTOR STALL override file).
    // Unlike the sim's startup line above, this is logged only AFTER the
    // full MARS handshake completes (trajectory + START received -- see
    // main.cpp's flow), well after "Launching:"/RUNNING, so it needs its
    // own wait rather than being covered by the robot1/robot2_launch_seen
    // waits above. Checked per-robot (not just "does the filename appear
    // anywhere") so this fails loudly if only one of the two robots'
    // controllers got the override.
    const bool robot1_spec_seen =
        wait_for_log_line(log_path, "[MPC robot1] Loaded robot spec from", 15.0);
    const bool robot2_spec_seen =
        wait_for_log_line(log_path, "[MPC robot2] Loaded robot spec from", 15.0);
    check(robot1_spec_seen, "robot1's mpc_controller loaded a --robot-spec override");
    check(robot2_spec_seen, "robot2's mpc_controller loaded a --robot-spec override");

    log_so_far = read_log_file(log_path); // refresh -- the waits above appended new content.
    check(log_so_far.find("stall_robot_spec_override.json") != std::string::npos,
          "the loaded --robot-spec override came from mars_sim_viz's own "
          "stall_robot_spec_override.json (MOTOR STALL's shared override file)");
    check(log_so_far.find("Failed to load --robot-spec") == std::string::npos,
          "no mpc_controller reported a --robot-spec LOAD FAILURE");
  }

  // ---------------------------------------------------------------------
  // Part 2: simulate a live stall-level change via a hand PUB directly on
  // robot1's "/robot1/sim_config" topic (the same wire path
  // ExecutionManager::set_stall_enabled()/set_stall_level() use) and verify
  // the sim's log picks up the new value. See this file's header comment
  // ("CONCURRENT-WORK NOTE") on why this check is format-agnostic.
  // ---------------------------------------------------------------------
  {
    const int robot1_vesc_port = kVescPortStart; // robot1 is index 0 (sorted first)
    const std::streamoff offset_before_live_change = log_size(log_path);
    std::cout << "[test]   hand-publishing deadband=1 min_moving_speed=" << kLiveStallLevel
              << " min_sustain_speed=" << kLiveStallLevel << " to robot1's sim_config topic on "
              << "port " << robot1_vesc_port << " (log offset " << offset_before_live_change
              << ")" << std::endl;
    hand_publish_stall_config(robot1_vesc_port, "robot1", /*deadband=*/true, kLiveStallLevel,
                               kLiveStallLevel);

    const bool live_change_seen =
        wait_for_log_tail_containing(log_path, offset_before_live_change, "0.16", 10.0);
    check(live_change_seen,
          "robot1's sim logged NEW content mentioning the live stall level (0.16) after the "
          "hand-published sim_config change -- if this fails, mpc_robot_sim's live "
          "'deadband'/'min_moving_speed'/'min_sustain_speed' sim_config parsing (being added "
          "concurrently, see this file's header comment) may not be built yet; rebuild "
          "mpc_robot_sim and re-run once");
  }

  const std::string abort_reply = req(endpoint, "ABORT");
  check(abort_reply == "ACK_ABORT", "cleanup ABORT -> ACK_ABORT (got '" + abort_reply + "')");

  check(no_stray_process_matching("cmd-endpoint tcp://\\*:" + std::to_string(kVescPortStart)),
        "no stray mpc_robot_sim after teardown");
  check(no_stray_process_matching("--port " + std::to_string(kHandshakePortStart)),
        "no stray mpc_controller after teardown");

  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n" << (g_overall_pass ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED") << std::endl;
  return g_overall_pass ? 0 : 1;
}
