// Regression test for the "wrong-robot mocap-linking" bug report: a 3-robot
// scenario (robot1/robot2/robot3), ONE Motive rigid body aliased to robot2,
// mocap Connected, EXECUTE with NO --real-mode flag at all. EXPECTED (per
// the bug's fix in ExecutionManager::start_execution(), SimVizCore.cpp):
// robot2 is wired REAL (mocap-linked, no mpc_robot_sim spawned for it) and
// robot1/robot3 are simulated normally -- automatically, without the
// operator ever setting config_.real_mode.
//
// Also exercises the PORT COLLISION fix: config.loc_port_start is
// deliberately set to the SAME value as config.mocap.loc_port_start (the
// bridge's single auto-discovery port) -- reproducing the exact default-value
// coincidence (both default to 3260 in production) that made robot1's own
// mpc_robot_sim fail to bind ("Address already in use") and exit immediately
// in the original bug report. With the fix, robot1 (the alphabetically-first,
// index-0 SIMULATED robot, which is where the collision lands) gets shifted
// to a collision-free port instead and its simulator starts normally.
//
// In-process simviz::SimVizManager (never a separately-spawned mars_sim_viz)
// against a REAL fake_motive + MocapManager-owned optitrack_zmq_bridge --
// same conventions as test_sim_viz_mocap.cpp's Part 3.
//
// TEST PORTS ONLY: control 45611, handshake 45720+, vesc 45740+, loc
// 45780+/45780 (deliberately == the bridge's own port below), bridge ZMQ
// auto-discovery loc port 45780, fake_motive/bridge UDP command port 45512 /
// data port 45513.

#include "SimVizCore.h"
#include "MocapCore.h"
#include "test_sim_viz_shared.h"

#include <QCoreApplication>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

bool DEBUG_VIS = false;

namespace
{

using simviz::MocapConfig;
using simviz::MocapManager;
using simviz::MocapState;

int g_checks = 0;
int g_failures = 0;

void check(bool cond, const std::string &msg)
{
  ++g_checks;
  if (cond)
  {
    std::cout << "[test] ok: " << msg << std::endl;
  }
  else
  {
    ++g_failures;
    std::cout << "[test] FAILED: " << msg << std::endl;
  }
}

constexpr int kFakeMotiveCommandPort = 45512;
constexpr int kBridgeLocalDataPort = 45513;
// DELIBERATE COLLISION: same value as the sim ports' loc_port_start below --
// reproduces the production default (both 3260) that triggered the bug.
constexpr int kSharedLocPortStart = 45780;

constexpr int kTestControlPort = 45611;
constexpr int kTestHandshakePortStart = 45720;
constexpr int kTestVescPortStart = 45740;

std::string synthetic_mocap_config_txt()
{
  return "Server:127.0.0.1\nLocal Address:127.0.0.1\nCommand Port:" +
         std::to_string(kFakeMotiveCommandPort) + "\nData Port:" +
         std::to_string(kBridgeLocalDataPort) + "\nMulticast Interface:239.255.42.99\n";
}

// Aliases the single Motive body "mushr2" -> "robot2", matching the bug
// report exactly ("ONE rigid body published by Motive, mapped (alias) to
// robot2").
nlohmann::json synthetic_map_config_json()
{
  nlohmann::json j;
  j["_doc"] = {{"note", "test-only synthetic mocap map-config for the mixed-fleet regression"}};
  j["y_up"] = false;
  j["x0"] = 0.0;
  j["y0"] = 0.0;
  j["theta0"] = 0.0;
  j["yaw_offset"] = nlohmann::json::object();
  j["aliases"] = {{"mushr2", "robot2"}};
  j["mocap_to_world_matrix"] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  return j;
}

// 3 robots (robot1/robot2/robot3), each a short single-segment transit
// trajectory starting at its own initial_pose. No object entities -- keeps
// this focused on the real-vs-sim wiring/port assignment, not the object
// handoff path.
ExecutedScenario build_three_robot_scenario(const std::string &label)
{
  ExecutedScenario scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 3;
  scn.summary.failed_tasks = 0;

  auto make_robot = [](const std::string &name, Pose initial_pose) {
    RobotMeta *r = new RobotMeta();
    r->name = name;
    r->type = EntityType::ROBOT;
    r->initial_pose = initial_pose;
    r->size = OccuRect{0.36, 0.12, 0.275};
    r->speed_transit = 0.2;
    r->speed_transfer = 0.15;
    return r;
  };

  RobotMeta *robot1 = make_robot("robot1", Pose(0.0, 0.0, 0.0));
  RobotMeta *robot2 = make_robot("robot2", Pose(0.0, 2.0, 0.0));
  RobotMeta *robot3 = make_robot("robot3", Pose(0.0, 4.0, 0.0));
  scn.entities["robot1"] = robot1;
  scn.entities["robot2"] = robot2;
  scn.entities["robot3"] = robot3;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  auto add_segment = [&](RobotMeta *r, Pose p1, Pose p2) {
    WaypointPath path = {Waypoint(p1), Waypoint(p2)};
    Trajectory traj(r, nullptr, 0.0, path, /*is_transfer=*/false);
    traj.CalcualteTimeStamps(r, 0.0);
    scn.timetable.add_trajectory(traj);
  };
  add_segment(robot1, Pose(0.0, 0.0, 0.0), Pose(1.0, 0.0, 0.0));
  add_segment(robot2, Pose(0.0, 2.0, 0.0), Pose(1.0, 2.0, 0.0));
  add_segment(robot3, Pose(0.0, 4.0, 0.0), Pose(1.0, 4.0, 0.0));

  return scn;
}

// mars_sim_viz's own PID (this test process, since everything here runs
// in-process) IS the parent of every QProcess it spawns -- pgrep -P lists
// exactly those, same convention as test_sim_viz_real_mode.cpp's
// direct_child_pids()/count_direct_children_matching().
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

} // namespace

int main()
{
  int argc = 1;
  char argv0[] = "test_sim_viz_mixed_fleet";
  char *argv[] = {argv0, nullptr};
  QCoreApplication app(argc, argv);

  std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() / "mars_simviz_mixed_fleet_test";
  std::filesystem::remove_all(tmp_dir);
  std::filesystem::create_directories(tmp_dir);

  const std::filesystem::path map_config_path = tmp_dir / "map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  std::vector<std::string> fake_motive_args = {
      "--target-ip",          "127.0.0.1",
      "--target-port",        std::to_string(kBridgeLocalDataPort),
      "--rate",               "60",
      "--natnet-version",     "3.1",
      "--body-ids",           "1", // ONE rigid body, per the bug report.
      "--body-names",         "mushr2",
      "--serve-command-port", std::to_string(kFakeMotiveCommandPort),
      "--served-app-name",    "TestMotiveMixedFleet",
      "--served-version",     "3.1",
      "--duration-s",         "120",
  };
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe, fake_motive_args), "fake_motive[mixed_fleet]");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  simviz::SimVizConfig config;
  config.control_port = kTestControlPort;
  config.handshake_port_start = kTestHandshakePortStart;
  config.vesc_port_start = kTestVescPortStart;
  // DELIBERATE COLLISION with config.mocap.loc_port_start below (see this
  // file's header doc comment) -- proves the port-collision fix, not just
  // the mixed-fleet real/sim decision.
  config.loc_port_start = kSharedLocPortStart;
  config.run_dir = (tmp_dir / "runs").string();
  // real_mode left at its default (false) -- NO --real-mode equivalent is
  // ever set here, matching the bug report's "GUI never sets real_mode".
  config.mocap.map_config_path = map_config_path.string();
  config.mocap.mocap_config_path = mocap_config_path.string();
  config.mocap.server_ip = "127.0.0.1";
  config.mocap.server_ip_explicit = true;
  config.mocap.mode = "unicast";
  config.mocap.mode_explicit = true;
  config.mocap.command_port = kFakeMotiveCommandPort;
  config.mocap.command_port_explicit = true;
  config.mocap.local_data_port = kBridgeLocalDataPort;
  config.mocap.local_data_port_explicit = true;
  config.mocap.loc_port_start = kSharedLocPortStart;

  simviz::SimVizManager manager(config);
  try
  {
    manager.start();
  }
  catch (const std::exception &ex)
  {
    check(false, std::string("manager.start() threw: ") + ex.what());
    fake_motive_guard.terminate();
    return g_failures == 0 ? 0 : 1;
  }

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kTestControlPort);
  const pid_t self_pid = getpid();

  std::atomic<bool> client_done{false};
  std::string execute_reply;
  std::string status_after_connect_check;
  bool connected_ok = false;

  std::thread client(
      [&]()
      {
        // MOCAP_CONNECT, then poll until Connected with robot2's aliased
        // body live.
        {
          std::string reply = simviz_test::req(endpoint, "MOCAP_CONNECT");
          check(reply == "ACK", "MOCAP_CONNECT -> ACK (" + reply + ")");
        }
        {
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
          while (std::chrono::steady_clock::now() < deadline)
          {
            status_after_connect_check = simviz_test::req(endpoint, "MOCAP_STATUS");
            if (status_after_connect_check.rfind("CONNECTED", 0) == 0 &&
                status_after_connect_check.find("robot2:") != std::string::npos)
            {
              connected_ok = true;
              break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
          }
          check(connected_ok,
                "MOCAP_STATUS reaches CONNECTED with 'robot2:' aliased body live (last: " +
                    status_after_connect_check + ")");
        }

        if (!connected_ok)
        {
          client_done.store(true);
          return;
        }

        // EXECUTE the 3-robot scenario -- NO real_mode ever set; the
        // real-vs-sim decision is evaluated purely from mocap
        // Connected + mapping, at this exact moment.
        ExecutedScenario scn = build_three_robot_scenario("mixed_fleet_test");
        const std::string scn_path =
            simviz_test::write_scn_file(scn, tmp_dir / "mixed_fleet_test.scn.b64");
        execute_reply = simviz_test::req(endpoint, "EXECUTE " + scn_path);
        check(execute_reply == "ACK_EXECUTE", "EXECUTE -> ACK_EXECUTE (" + execute_reply + ")");

        // Give the child processes (mpc_controller/mpc_robot_sim) a moment
        // to actually spawn and bind before the process-tree snapshot below.
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));

        client_done.store(true);
      });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(40);
  while (!client_done.load() && std::chrono::steady_clock::now() < deadline)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  check(client_done.load(), "client thread finished within its deadline");
  client.join();

  // Snapshot the process tree WHILE the run is still active: exactly 2
  // mpc_robot_sim children (robot1 + robot3 -- robot2 is mocap-linked, no
  // simulator) and exactly 3 mpc_controller children (every robot gets a
  // controller, real or simulated).
  const int n_sim = count_direct_children_matching(self_pid, "mpc_robot_sim");
  const int n_ctl = count_direct_children_matching(self_pid, "mpc_controller");
  check(n_sim == 2,
        "exactly 2 mpc_robot_sim children (robot1+robot3 simulated, robot2 mocap-linked) -- "
        "found " +
            std::to_string(n_sim));
  check(n_ctl == 3, "exactly 3 mpc_controller children (one per robot) -- found " +
                         std::to_string(n_ctl));

  // Let the run finish (or at least keep ticking so ABORT below tears down
  // cleanly) before asserting on final STATUS.
  {
    const auto run_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    std::string last_status;
    while (std::chrono::steady_clock::now() < run_deadline)
    {
      last_status = simviz_test::req(endpoint, "STATUS");
      manager.tick();
      if (last_status.rfind("DONE", 0) == 0 || last_status.rfind("ERR", 0) == 0)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    // NOT ERR is the key assertion here: an ERR (e.g. a robot's controller
    // never getting a working localization endpoint because its sim
    // crashed on a port collision, or robot2 never getting mocap data)
    // would surface as a trajectory-upload timeout / handshake failure.
    check(last_status.rfind("ERR", 0) != 0,
          "run does not end in ERR (no port-collision crash, mocap-linked robot2 got its pose) "
          "-- last='" +
              last_status + "'");
  }

  simviz_test::req(endpoint, "ABORT");
  for (int i = 0; i < 20; ++i)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  fake_motive_guard.terminate();

  std::cout << "\n[test] " << g_checks << " checks run, " << g_failures << " failed."
            << std::endl;
  if (g_failures == 0)
  {
    std::cout << "[Test] All test_sim_viz_mixed_fleet tests passed." << std::endl;
    return 0;
  }
  std::cout << "SOME CHECKS FAILED" << std::endl;
  return 1;
}
