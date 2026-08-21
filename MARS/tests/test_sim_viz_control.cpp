// Phase B1 unit tests for the mars_sim_viz control protocol
// (simviz::SimVizManager / simviz::ControlServer / simviz::ExecutionManager
// -- see MARS/src/simviz/SimVizCore.h). Offscreen (QT_QPA_PLATFORM does not
// matter here -- QCoreApplication only, no widgets), drives the manager
// in-process rather than spawning a separate mars_sim_viz binary.
//
// Exercises: PING, STATUS (Idle/Running), EXECUTE with a bogus path (ERR,
// stays Idle), EXECUTE success -> ACK_EXECUTE -> busy, a second EXECUTE
// while busy -> "ERR busy", ABORT -> "ACK_ABORT" -> back to Idle, and
// re-EXECUTE after ABORT to prove the manager is reusable without a
// restart.
//
// Uses fake robot-process binaries (a tiny shell script generated at test
// time that just sleeps, ignoring whatever argv mars_sim_viz passes it) so
// this stays a fast unit test: we only need "some child process is alive"
// to exercise the Running/busy path, not a real MPC handshake (that is
// what test_sim_viz_integration.cpp is for). TEST PORT BLOCK ports only
// (control 45601; handshake/vesc/loc port-starts are configured but never
// actually dialed by the fake scripts).

#include "SimVizCore.h"

#include <ExecutedScenarioSerialization.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <QCoreApplication>

#include <zmq.hpp>

#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

bool DEBUG_VIS = false;

namespace
{

constexpr int kTestControlPort = 45601;
constexpr int kTestHandshakePortStart = 45620;
constexpr int kTestVescPortStart = 45640;
constexpr int kTestLocPortStart = 45660;

std::string write_fake_robot_binary(const std::filesystem::path &dir)
{
  const std::filesystem::path path = dir / "fake_robot_proc.sh";
  {
    std::ofstream out(path);
    out << "#!/bin/sh\nexec sleep 300\n";
  }
  chmod(path.c_str(), 0755);
  return path.string();
}

std::string build_and_write_test_scenario(const std::filesystem::path &dir,
                                           const std::string &label)
{
  ExecutedScenario scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 1;
  scn.summary.makespan = 10.0;

  RobotMeta *robot = new RobotMeta();
  robot->name = "robot1";
  robot->type = EntityType::ROBOT;
  robot->initial_pose = Pose(0.0, 0.0, 0.0);
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;
  scn.entities["robot1"] = robot;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  Pose p1(0.0, 0.0, 0.0);
  Pose p2(1.0, 0.0, 0.0);
  WaypointPath path = {Waypoint(p1), Waypoint(p2)};
  Trajectory traj(robot, nullptr, 0.0, path, /*is_transfer=*/false);
  traj.CalcualteTimeStamps(robot, 0.0);
  scn.timetable.add_trajectory(traj);

  const std::string b64 = serialize_executed_scenario_b64(scn);
  const std::filesystem::path out_path = dir / (label + ".scn.b64");
  std::ofstream out(out_path, std::ios::binary);
  out << b64;
  return out_path.string();
}

// Fresh REQ socket per call (simplest correct pattern for a short exchange;
// mirrors SimVizHandoff.cpp's send_and_wait).
std::string req(const std::string &endpoint, const std::string &msg, int timeout_ms = 6000)
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

} // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);

  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() /
                                         ("mars_simviz_control_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::string fake_bin = write_fake_robot_binary(tmp_dir);
  const std::string scn_path = build_and_write_test_scenario(tmp_dir, "control_test_scenario");

  simviz::SimVizConfig config;
  config.control_port = kTestControlPort;
  config.handshake_port_start = kTestHandshakePortStart;
  config.vesc_port_start = kTestVescPortStart;
  config.loc_port_start = kTestLocPortStart;
  config.mpc_controller_path = fake_bin;
  config.robot_sim_path = fake_bin;
  config.run_dir = (tmp_dir / "runs").string();

  simviz::SimVizManager manager(config);
  try
  {
    manager.start();
  }
  catch (const std::exception &ex)
  {
    std::cerr << "[test] manager.start() failed: " << ex.what() << std::endl;
    return 1;
  }

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kTestControlPort);

  std::atomic<bool> client_done{false};
  std::atomic<bool> overall_pass{true};

  auto check = [&](bool cond, const std::string &what) {
    if (!cond)
    {
      std::cerr << "[test] FAILED: " << what << std::endl;
      overall_pass.store(false);
    }
    else
    {
      std::cout << "[test] ok: " << what << std::endl;
    }
  };

  std::thread client([&]() {
    // (1) PING
    check(req(endpoint, "PING") == "PONG mars_sim_viz v1", "PING -> PONG mars_sim_viz v1");

    // (2) STATUS while idle
    check(req(endpoint, "STATUS") == "IDLE", "STATUS (idle) -> IDLE");

    // (3) EXECUTE with a bogus path -> ERR, and still idle after.
    {
      std::string reply = req(endpoint, "EXECUTE /nonexistent/path/does_not_exist.scn.b64");
      check(reply.rfind("ERR", 0) == 0, "EXECUTE bogus path -> ERR (" + reply + ")");
      check(req(endpoint, "STATUS") == "IDLE", "STATUS after failed EXECUTE -> IDLE");
    }

    // (4) EXECUTE a real (small) scenario -> ACK_EXECUTE, then busy.
    {
      std::string reply = req(endpoint, "EXECUTE " + scn_path);
      check(reply == "ACK_EXECUTE", "EXECUTE valid scenario -> ACK_EXECUTE (" + reply + ")");

      // Give start_execution() a moment to spawn the fake children and flip
      // state to Running before we poll STATUS.
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      std::string status = req(endpoint, "STATUS");
      check(status.rfind("RUNNING", 0) == 0, "STATUS after EXECUTE -> RUNNING ... (" + status + ")");
    }

    // (5) A second EXECUTE while busy -> "ERR busy", first run untouched.
    {
      std::string reply = req(endpoint, "EXECUTE " + scn_path);
      check(reply == "ERR busy", "EXECUTE while busy -> ERR busy (" + reply + ")");
    }

    // (6) ABORT -> ACK_ABORT, then back to Idle.
    {
      std::string reply = req(endpoint, "ABORT");
      check(reply == "ACK_ABORT", "ABORT -> ACK_ABORT (" + reply + ")");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      check(req(endpoint, "STATUS") == "IDLE", "STATUS after ABORT -> IDLE");
    }

    // (7) Reusability: EXECUTE again on the same (already-used) manager.
    {
      std::string reply = req(endpoint, "EXECUTE " + scn_path);
      check(reply == "ACK_EXECUTE",
            "EXECUTE again after ABORT -> ACK_EXECUTE (reusability) (" + reply + ")");
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      std::string status = req(endpoint, "STATUS");
      check(status.rfind("RUNNING", 0) == 0,
            "STATUS after second EXECUTE -> RUNNING ... (" + status + ")");

      // Clean up: abort so the child fake processes don't linger.
      check(req(endpoint, "ABORT") == "ACK_ABORT", "final ABORT -> ACK_ABORT");
    }

    client_done.store(true);
  });

  // Drive the manager's tick loop on the main (Qt) thread -- QProcess and
  // friends were constructed here, so this is also where they must be
  // polled/torn down.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (!client_done.load() && std::chrono::steady_clock::now() < deadline)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!client_done.load())
  {
    std::cerr << "[test] FAILED: client thread did not finish within the 30s deadline"
              << std::endl;
    overall_pass.store(false);
  }
  client.join();

  // Final teardown tick(s): every request (including ABORT) is actually
  // handled inside manager.tick() on this thread (the client thread only
  // sends the ZMQ request and blocks on the reply), so a few more ticks
  // here just make sure any in-flight teardown settles before we exit.
  for (int i = 0; i < 10; ++i)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::filesystem::remove_all(tmp_dir);

  if (overall_pass.load())
  {
    std::cout << "\n[Test] All test_sim_viz_control tests passed." << std::endl;
    return 0;
  }
  std::cerr << "\n[Test] SOME test_sim_viz_control tests FAILED." << std::endl;
  return 1;
}
