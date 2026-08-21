// FEATURE C (+ PART (1) STEERING-NOISE SPLIT): standalone (non-ctest) test
// for mars_sim_viz's two independent motor-noise sliders -- ACCEL channel
// (SimVizCore.h/.cpp's ExecutionManager::set_noise_sigma_pct()/
// noise_sigma_pct()) and STEER channel (set_steer_noise_sigma_pct()/
// steer_noise_sigma_pct()) -- the per-robot FEATURE C PUB sockets connected
// at spawn time, the ~1Hz republish in tick(), the headless
// --noise-sigma-pct=/--steer-noise-sigma-pct=/--noise-seed= CLI flags, and
// STATUS's additive "noise=A%/S%" suffix. Same standalone-binary style as
// test_sim_viz_pause.cpp/test_sim_viz_execute.cpp: spawns the REAL
// mars_sim_viz --headless binary + real mpc_controller/mpc_robot_sim
// children against the synthetic 2-robot scenario from test_sim_viz_shared.h
// (build_synthetic_scenario()).
//
// TEST PORT BLOCK: same block test_sim_viz_pause.cpp/test_sim_viz_execute.cpp
// use (this test never runs concurrently with them -- each standalone binary
// is invoked and fully reaped before the next):
//   control = 45601, handshake = 45620+idx, vesc = 45640+idx,
//   localization = 45660+idx.
//
// Log capture: QProcess::ForwardedChannels (SimVizCore.cpp's
// spawn_wrapped_process()) means every spawned mpc_robot_sim's stdout/
// stderr lines are forwarded straight to mars_sim_viz's OWN stdout/stderr.
// spawn_wrapped_logged() below (a small variant of test_sim_viz_shared.h's
// spawn_wrapped(), redirecting fd 1/2 to a file before execv) captures that
// combined stream to a plain file this test can grep -- the only way to
// observe a child's stderr line ("noise sigma set to X") from a fully
// separate process without instrumenting mars_sim_viz/mpc_robot_sim
// themselves.
//
// Parts:
//   1. Spawn with --noise-sigma-pct=0.10 --steer-noise-sigma-pct=0.05 (+ a
//      fixed --noise-seed= for determinism): verify STATUS's diagnostic
//      string includes "noise=10%/5%", and each spawned mpc_robot_sim's own
//      startup log line shows it actually received "--noise-sigma-pct=0.1"
//      AND "--steer-noise-sigma-pct=0.05" (proving spawn-time passthrough
//      for BOTH channels, not just what mars_sim_viz itself believes).
//   2. Live change: hand-PUB {"noise_sigma_pct": 0.18,
//      "steer_noise_sigma_pct": 0.09} directly on topic "/robot1/sim_config"
//      to robot1's vesc port (the exact wire format/endpoint
//      ExecutionManager's own set_noise_sigma_pct()/
//      set_steer_noise_sigma_pct() combined publish uses) and verify
//      robot1's sim logs "noise sigma set to accel=0.18 steer=0.09" --
//      proving the sim-only live-reconfiguration channel is reachable and
//      wired correctly end to end for BOTH channels, independent of the GUI
//      sliders themselves (which cannot be driven headlessly).

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

constexpr double kSpawnNoiseSigmaPct = 0.10;
constexpr std::uint64_t kNoiseSeed = 424242;
constexpr double kLiveNoiseSigmaPct = 0.18;

// PART (1) STEERING-NOISE SPLIT: independent STEER-channel counterparts,
// deliberately DIFFERENT values from the ACCEL-channel ones above so a test
// that accidentally cross-wires the two channels (e.g. reads/writes the
// wrong sigma) fails loudly instead of coincidentally matching.
constexpr double kSpawnSteerNoiseSigmaPct = 0.05;
constexpr double kLiveSteerNoiseSigmaPct = 0.09;

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

// Variant of test_sim_viz_shared.h's spawn_wrapped(): identical "env -i ..."
// wrapping, but redirects the spawned process's stdout+stderr to `log_path`
// first -- see this file's header comment on why that's needed to observe a
// grandchild's (mpc_robot_sim's) log lines from a separate test process.
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

// Polls `log_path` until its content contains `needle`, or timeout_s
// elapses.
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

// Hand-publishes `{"noise_sigma_pct": accel_sigma_pct, "steer_noise_sigma_pct":
// steer_sigma_pct}` on topic "/<robot>/sim_config" to `port` -- the exact
// same wire format/endpoint ExecutionManager's own
// set_noise_sigma_pct()/set_steer_noise_sigma_pct()/publish_noise_config()
// uses (SimVizCore.cpp, PART (1) STEERING-NOISE SPLIT: both channels'
// current values are always sent together), and exactly what a real GUI
// slider drag would cause -- standing in for the GUI sliders themselves,
// which cannot be driven headlessly.
void hand_publish_noise_config(int port, const std::string &robot_name, double accel_sigma_pct,
                                double steer_sigma_pct)
{
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  pub.set(zmq::sockopt::linger, 0);
  pub.connect("tcp://127.0.0.1:" + std::to_string(port));
  const std::string topic = "/" + robot_name + "/sim_config";
  std::ostringstream payload;
  payload << "{\"noise_sigma_pct\":" << accel_sigma_pct
          << ",\"steer_noise_sigma_pct\":" << steer_sigma_pct << "}";
  const std::string payload_str = payload.str();
  // PUB/SUB has no late-joiner replay -- give the freshly-connected socket a
  // moment before the first (only) publish, same margin
  // test_sim_viz_window_render.cpp's publish_fake_localization() uses.
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
      ("mars_simviz_noise_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  SyntheticScenario synth = build_synthetic_scenario("noise_test");
  const std::string scn_path = write_scn_file(synth.scn, tmp_dir / "noise_test.scn.b64");
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
      "--noise-sigma-pct=" + to_str(kSpawnNoiseSigmaPct, 2),
      "--steer-noise-sigma-pct=" + to_str(kSpawnSteerNoiseSigmaPct, 2),
      "--noise-seed=" + std::to_string(kNoiseSeed),
  };

  const pid_t pid = spawn_wrapped_logged(sim_viz_exe, viz_args, log_path);
  ProcessGuard guard(pid, "mars_sim_viz (noise test)");
  std::cout << "[test] spawned mars_sim_viz pid=" << pid << " log=" << log_path << std::endl;

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kControlPort);
  check(wait_for_ping(endpoint, 15.0), "control socket answers PING");
  check(wait_for_running(endpoint, 20.0), "run reaches RUNNING");

  // ---------------------------------------------------------------------
  // Part 1: spawn-time flag reaches both mars_sim_viz's own STATUS and each
  // spawned mpc_robot_sim's own startup log line.
  // ---------------------------------------------------------------------
  {
    const std::string status = req(endpoint, "STATUS");
    std::cout << "[test]   STATUS: " << status << std::endl;
    check(status.find("noise=10%/5%") != std::string::npos,
          "STATUS diagnostic string includes 'noise=10%/5%' (got '" + status + "')");

    // Give both sims a moment to print their startup line + at least one
    // periodic republish before reading the log.
    const bool robot1_launch_seen =
        wait_for_log_line(log_path, "[SIM robot1] Launching:", 10.0);
    const bool robot2_launch_seen =
        wait_for_log_line(log_path, "[SIM robot2] Launching:", 10.0);
    check(robot1_launch_seen, "robot1's mpc_robot_sim printed its startup line");
    check(robot2_launch_seen, "robot2's mpc_robot_sim printed its startup line");

    const std::string log_so_far = read_log_file(log_path);
    // robot_sim.cpp's startup line reports the accel channel as
    // "noise_sigma_pct(accel)=<value>" (PART (1) STEERING-NOISE SPLIT
    // relabeled it from the pre-split "noise_sigma_pct=<value>").
    check(log_so_far.find("noise_sigma_pct(accel)=0.1") != std::string::npos,
          "spawned mpc_robot_sim(s) actually received --noise-sigma-pct=0.1 (spawn-time "
          "passthrough, not just mars_sim_viz's own belief about the value)");
    // PART (1) STEERING-NOISE SPLIT: robot_sim.cpp's own startup log line
    // reports the flag it parsed as "steer_noise_sigma_pct=<value>" (see
    // robot_sim.cpp's startup cout, distinct from the accel line above).
    check(log_so_far.find("steer_noise_sigma_pct=0.05") != std::string::npos,
          "spawned mpc_robot_sim(s) actually received --steer-noise-sigma-pct=0.05 (spawn-time "
          "passthrough for the independent STEER channel)");

    const bool republish_seen =
        wait_for_log_line(log_path, "[SIM robot1] noise sigma set to accel=0.1 steer=0.05", 5.0);
    check(republish_seen,
          "robot1's sim logs the ~1Hz idempotent noise-config republish while Running (both "
          "channels)");
  }

  // ---------------------------------------------------------------------
  // Part 2: simulate a live slider change via a hand PUB directly on
  // robot1's "/robot1/sim_config" topic (the same wire path
  // ExecutionManager::set_noise_sigma_pct() uses) and verify the sim logs
  // the update.
  // ---------------------------------------------------------------------
  {
    const int robot1_vesc_port = kVescPortStart; // robot1 is index 0 (sorted first)
    std::cout << "[test]   hand-publishing noise_sigma_pct=" << kLiveNoiseSigmaPct
              << " steer_noise_sigma_pct=" << kLiveSteerNoiseSigmaPct
              << " to robot1's sim_config topic on port " << robot1_vesc_port << std::endl;
    hand_publish_noise_config(robot1_vesc_port, "robot1", kLiveNoiseSigmaPct,
                               kLiveSteerNoiseSigmaPct);

    const bool live_change_seen = wait_for_log_line(
        log_path, "[SIM robot1] noise sigma set to accel=0.18 steer=0.09", 10.0);
    check(live_change_seen,
          "robot1's sim logged the live noise-config change for BOTH channels ('noise sigma set "
          "to accel=0.18 steer=0.09')");
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
