#pragma once

// Shared harness helpers for the standalone (non-ctest) mars_sim_viz
// integration-style test binaries: test_sim_viz_integration.cpp (the
// original PING/EXECUTE/STATUS end-to-end test) and test_sim_viz_execute.cpp
// (the --execute/--exit-on-done direct-load test). Both spawn the REAL
// mars_sim_viz --headless binary (never in-process) against a synthetic,
// short (~14s makespan) scenario built programmatically and serialized to a
// temp .scn.b64 file -- exactly the artifact MARS's own SimVizHandoff.cpp
// would hand off in production.
//
// Factored out of test_sim_viz_integration.cpp (see that file's own header
// comment for the fuller design-context prose) purely to avoid duplicating
// ~300 lines of process-spawn/ProcessGuard/scenario-builder code across the
// two test binaries; nothing here is test-binary-specific.
//
// ProcessGuard/self_dir()/spawn()/spawn_wrapped() mirror
// MPC/tests/test_mpc_sim_loop.cpp's style, copied near-verbatim
// (see that file's header comment for the rationale).

#include <ExecutedScenarioSerialization.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <zmq.hpp>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace simviz_test
{

constexpr double kFinalPosTolM = 0.10;
constexpr double kFinalYawTolRad = 0.15;

inline std::string to_str(double v, int precision = 6)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

inline double wrap_pi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

// ---------------------------------------------------------------------
// Sibling-executable resolution (mirrors MPC/tests/
// test_mpc_sim_loop.cpp's self_dir()). Every test binary using this header
// lives in build-release/MARS/ alongside mars_sim_viz; MPC's
// binaries live one directory up, under MPC/.
// ---------------------------------------------------------------------
inline std::string self_dir()
{
  char buf[4096];
  ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n <= 0)
    throw std::runtime_error("readlink(/proc/self/exe) failed");
  buf[n] = '\0';
  std::string p(buf);
  auto pos = p.find_last_of('/');
  return pos == std::string::npos ? std::string(".") : p.substr(0, pos);
}

// ---------------------------------------------------------------------
// RAII child-process guard, copied from MPC/tests/
// test_mpc_sim_loop.cpp (see that file for the full rationale comment):
// always reaps by the PID it was constructed with, SIGTERM -> grace
// period -> SIGKILL fallback, safe to invoke even after natural exit.
// ---------------------------------------------------------------------
class ProcessGuard
{
public:
  ProcessGuard() = default;
  ProcessGuard(pid_t pid, std::string label) : pid_(pid), label_(std::move(label)) {}
  ProcessGuard(const ProcessGuard &) = delete;
  ProcessGuard &operator=(const ProcessGuard &) = delete;
  ProcessGuard(ProcessGuard &&other) noexcept { *this = std::move(other); }
  ProcessGuard &operator=(ProcessGuard &&other) noexcept
  {
    if (this != &other)
    {
      terminate();
      pid_ = other.pid_;
      label_ = std::move(other.label_);
      other.pid_ = -1;
    }
    return *this;
  }
  ~ProcessGuard() { terminate(); }

  pid_t pid() const { return pid_; }

  bool wait_for_exit(double timeout_s, double poll_interval_s = 0.2)
  {
    if (pid_ <= 0)
      return true;
    auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
      int status = 0;
      pid_t r = waitpid(pid_, &status, WNOHANG);
      if (r == pid_)
      {
        pid_ = -1;
        return true;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(poll_interval_s));
    }
    return false;
  }

  void terminate()
  {
    if (pid_ <= 0)
      return;
    kill(pid_, SIGTERM);
    for (int i = 0; i < 100; ++i)
    { // ~2s grace period
      int status = 0;
      pid_t r = waitpid(pid_, &status, WNOHANG);
      if (r == pid_)
      {
        pid_ = -1;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cerr << "[test] pid " << pid_ << " (" << label_
              << ") did not exit after SIGTERM; sending SIGKILL" << std::endl;
    kill(pid_, SIGKILL);
    int status = 0;
    waitpid(pid_, &status, 0);
    pid_ = -1;
  }

private:
  pid_t pid_ = -1;
  std::string label_;
};

inline pid_t spawn(const std::string &exe, const std::vector<std::string> &args)
{
  std::vector<char *> argv;
  argv.push_back(const_cast<char *>(exe.c_str()));
  for (const auto &a : args)
    argv.push_back(const_cast<char *>(a.c_str()));
  argv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0)
    throw std::runtime_error("fork() failed: " + std::string(std::strerror(errno)));
  if (pid == 0)
  {
    execv(exe.c_str(), argv.data());
    std::fprintf(stderr, "[test] execv('%s') failed: %s\n", exe.c_str(), std::strerror(errno));
    _exit(127);
  }
  return pid;
}

// Spawns `exe args...` wrapped in "env -i HOME=.. USER=.. PATH=..
// QT_QPA_PLATFORM=offscreen", per this task's "run binaries wrapped" rule.
inline pid_t spawn_wrapped(const std::string &exe, const std::vector<std::string> &args)
{
  const char *home = std::getenv("HOME");
  const char *user = std::getenv("USER");
  std::vector<std::string> wrapped_args = {
      "-i", std::string("HOME=") + (home ? home : ""),
      std::string("USER=") + (user ? user : ""), "PATH=/usr/local/bin:/usr/bin:/bin",
      "QT_QPA_PLATFORM=offscreen", exe};
  for (const auto &a : args)
    wrapped_args.push_back(a);
  return spawn("/usr/bin/env", wrapped_args);
}

// ---------------------------------------------------------------------
// Control-socket REQ helper. Fresh socket per call (mirrors
// SimVizHandoff.cpp's send_and_wait / test_sim_viz_control.cpp's req()).
// ---------------------------------------------------------------------
inline std::string req(const std::string &endpoint, const std::string &msg,
                        int timeout_ms = 5000)
{
  zmq::context_t ctx(1);
  zmq::socket_t sock(ctx, zmq::socket_type::req);
  sock.set(zmq::sockopt::linger, 0);
  sock.set(zmq::sockopt::rcvtimeo, timeout_ms);
  sock.set(zmq::sockopt::sndtimeo, timeout_ms);
  sock.connect(endpoint);
  sock.send(zmq::buffer(msg), zmq::send_flags::none);
  zmq::message_t reply;
  auto res = sock.recv(reply, zmq::recv_flags::none);
  if (!res)
    return "<<TIMEOUT>>";
  return std::string(static_cast<const char *>(reply.data()), reply.size());
}

// Retries PING until it succeeds or `timeout_s` elapses (mars_sim_viz needs
// a brief moment after spawn to bind its control socket).
inline bool wait_for_ping(const std::string &endpoint, double timeout_s)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (req(endpoint, "PING", 500) == "PONG mars_sim_viz v1")
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  return false;
}

// Polls STATUS until it starts with "DONE " (success) or "ERR " (failure),
// or `timeout_s` elapses. Prints progress periodically.
struct StatusOutcome
{
  bool done = false;
  std::string final_status;
};

inline StatusOutcome poll_until_done(const std::string &endpoint, double timeout_s)
{
  StatusOutcome outcome;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  int tick = 0;
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::string status = req(endpoint, "STATUS");
    if (tick++ % 5 == 0)
      std::cout << "[test]   STATUS: " << status << std::endl;
    if (status.rfind("DONE ", 0) == 0 || status.rfind("ERR ", 0) == 0)
    {
      outcome.done = status.rfind("DONE ", 0) == 0;
      outcome.final_status = status;
      return outcome;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  outcome.final_status = "<<TIMEOUT waiting for DONE/ERR>>";
  return outcome;
}

// ---------------------------------------------------------------------
// Synthetic scenario: 2 robots, one transfer span (robot2 carries object1
// for the second half of its path). Total makespan ~14.3s.
// ---------------------------------------------------------------------
struct SyntheticScenario
{
  ExecutedScenario scn;
  Pose robot1_final;
  Pose robot2_final;
};

inline SyntheticScenario build_synthetic_scenario(const std::string &label)
{
  SyntheticScenario out;
  ExecutedScenario &scn = out.scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 2;
  scn.summary.failed_tasks = 0;

  RobotMeta *robot1 = new RobotMeta();
  robot1->name = "robot1";
  robot1->type = EntityType::ROBOT;
  robot1->initial_pose = Pose(0.0, 0.0, 0.0);
  robot1->size = OccuRect{0.36, 0.12, 0.275};
  robot1->speed_transit = 0.2;
  robot1->speed_transfer = 0.15;

  RobotMeta *robot2 = new RobotMeta();
  robot2->name = "robot2";
  robot2->type = EntityType::ROBOT;
  robot2->initial_pose = Pose(0.0, 2.0, 0.0);
  robot2->size = OccuRect{0.36, 0.12, 0.275};
  robot2->speed_transit = 0.2;
  robot2->speed_transfer = 0.15;

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

  // robot1: single transit segment, 2.2m @ 0.2 m/s = 11s.
  {
    Pose p1(0.0, 0.0, 0.0);
    Pose p2(2.2, 0.0, 0.0);
    WaypointPath path = {Waypoint(p1), Waypoint(p2)};
    Trajectory traj(robot1, nullptr, 0.0, path, /*is_transfer=*/false);
    traj.CalcualteTimeStamps(robot1, 0.0);
    scn.timetable.add_trajectory(traj);
    out.robot1_final = p2;
  }

  // robot2: transit segment (0,2,0)->(1,2,0), 1m @ 0.2 m/s = 5s, THEN a
  // transfer segment (1,2,0)->(2.4,2,0) carrying object1, 1.4m @ 0.15 m/s
  // = 9.333s. Total makespan ~14.33s.
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
    // NOTE: CalcualteTimeStamps() always assigns RELATIVE waypoint times
    // starting at 0 (waypoints[0].time is hardcoded to 0 regardless of the
    // start_time_in argument -- see Entities.h); the ABSOLUTE offset comes
    // solely from the Trajectory's own start_time field (the 3rd ctor arg,
    // used by TimeTable::add_trajectory's `offset`). Passing start_time_in
    // = 5.0 here as well would double-count that offset. Always 0.0, same
    // as every other caller (see e.g. test_sim_viz_handoff.cpp's
    // build_test_scenario()).
    Trajectory traj2(robot2, object1, 5.0, path2, /*is_transfer=*/true);
    traj2.CalcualteTimeStamps(robot2, 0.0);
    scn.timetable.add_trajectory(traj2);
    out.robot2_final = p4;
  }

  return out;
}

inline std::string write_scn_file(const ExecutedScenario &scn, const std::filesystem::path &path)
{
  const std::string b64 = serialize_executed_scenario_b64(scn);
  std::ofstream out(path, std::ios::binary);
  out << b64;
  return path.string();
}

// ---------------------------------------------------------------------
// Sim CSV row (see MPC/src/robot_sim.cpp's --log-csv header:
// t,x,y,yaw,v,a_cmd,delta_cmd,watchdog_active). Copied from
// test_mpc_sim_loop.cpp.
// ---------------------------------------------------------------------
struct CsvRow
{
  double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0, v = 0.0;
};

inline std::vector<CsvRow> read_csv(const std::string &path)
{
  std::vector<CsvRow> rows;
  std::ifstream f(path);
  if (!f.is_open())
    return rows;
  std::string line;
  std::getline(f, line); // header
  while (std::getline(f, line))
  {
    if (line.empty())
      continue;
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ','))
    {
      try
      {
        vals.push_back(std::stod(tok));
      }
      catch (...)
      {
        vals.clear();
        break;
      }
    }
    if (vals.size() < 5)
      continue;
    CsvRow r;
    r.t = vals[0];
    r.x = vals[1];
    r.y = vals[2];
    r.yaw = vals[3];
    r.v = vals[4];
    rows.push_back(r);
  }
  return rows;
}

// Returns true (silently) if no process on this machine has `pattern` in
// its command line; otherwise prints the matches and returns false. Uses
// pgrep -af (read-only), never kills anything -- matches this task's "never
// pkill by name" rule (only ProcessGuard, keyed by exact PID, ever signals
// a process in this test).
inline bool no_stray_process_matching(const std::string &pattern)
{
  const std::string cmd = "pgrep -af '" + pattern + "' 2>/dev/null";
  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe)
    return true; // can't check; don't fail the test over a missing pgrep
  std::string output;
  char buf[256];
  while (fgets(buf, sizeof(buf), pipe) != nullptr)
    output += buf;
  pclose(pipe);
  if (!output.empty())
  {
    std::cout << "[test]   WARNING: stray process(es) matching '" << pattern << "':\n"
              << output;
    return false;
  }
  return true;
}

} // namespace simviz_test
