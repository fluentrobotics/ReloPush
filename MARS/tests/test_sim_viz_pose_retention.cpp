// ISSUE 1 regression test ("when the simulation is done, the robots jump to
// somewhere else"). Root cause: LocalizationListener::stop() used to clear
// its per-robot pose cache; ExecutionManager::terminate_children() calls
// stop() on every teardown (finalize_run()'s DONE/ERR path AND abort()), so
// SimVizCanvas's live-pose lookup (SimVizWindow.cpp's robot-drawing block)
// would go back to std::nullopt the instant a run ended and fall back to
// drawing the robot at the scenario's initial_pose -- the reported "jump".
//
// Fix: LocalizationListener::stop() no longer clears the cache; only
// LocalizationListener::reset() does, and ExecutionManager::start_execution()
// calls reset() exactly once per genuinely NEW run (EXECUTE / File->Open /
// RESTART all funnel through it), so a fresh scenario's initial_pose is
// still the correct pre-localization fallback.
//
// This test drives simviz::SimVizManager in-process (same pattern as
// test_sim_viz_control.cpp: fake sleeping child binaries -- their real
// behavior doesn't matter, only that "some child process is alive" long
// enough to exercise Running) against the TEST PORT BLOCK
// (control=45601, handshake=45620+, vesc=45640+, loc=45660+), and stands in
// for what a real mpc_robot_sim would publish on --loc-endpoint via a
// hand-rolled ZMQ PUB socket bound directly to the loc port, so a live
// localization sample reaches ExecutionManager's LocalizationListener
// without needing the real MPC binaries.
//
// Exercises, in order:
//   (1) a live pose arrives while Running;
//   (2) the run naturally ERRs out (the fake mpc_controller never answers
//       the handshake, so ExecutionManager::fail_handshake() fires and
//       tick() finalizes as Err) -- finalize_run()'s teardown path is
//       EXACTLY the same terminate_children() call DONE uses, so this
//       exercises the identical mechanism; the pose must still be queryable
//       afterward (retention);
//   (3) a fresh EXECUTE of the SAME scenario resets the cache (queried
//       immediately, before any new sample arrives, must be empty again);
//   (4) a second run's live pose arrives, then ABORT (the other run-ending
//       state named in the ISSUE 1 spec) -- the pose must still be
//       queryable afterward too.

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
#include <cmath>
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

// Fresh REQ socket per call (mirrors test_sim_viz_control.cpp's req()).
std::string req(const std::string &endpoint, const std::string &msg, int timeout_ms = 8000)
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

// Publishes `{"x":..,"y":..,"yaw":..}` on topic "/<robot>/localization"
// repeatedly until `stop` is set -- stands in for what mpc_robot_sim would
// publish on its --loc-endpoint (MPC/src/robot_sim.cpp: PUB
// binds, LocalizationListener's SUB connects), without needing to spawn
// that real binary.
void publish_fake_localization(int port, const std::string &robot_name, double x, double y,
                                double yaw, std::atomic<bool> &stop)
{
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  pub.set(zmq::sockopt::linger, 0);
  pub.bind("tcp://*:" + std::to_string(port));
  const std::string topic = "/" + robot_name + "/localization";
  std::ostringstream payload;
  payload << "{\"x\":" << x << ",\"y\":" << y << ",\"yaw\":" << yaw << "}";
  const std::string payload_str = payload.str();
  // PUB/SUB has no late-joiner replay -- give the SUB side a moment to
  // connect + subscribe before the first publish.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (!stop.load())
  {
    zmq::message_t topic_msg(topic.begin(), topic.end());
    zmq::message_t payload_msg(payload_str.begin(), payload_str.end());
    pub.send(topic_msg, zmq::send_flags::sndmore);
    pub.send(payload_msg, zmq::send_flags::none);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}

} // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_pose_retention_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::string fake_bin = write_fake_robot_binary(tmp_dir);
  const std::string scn_path = build_and_write_test_scenario(tmp_dir, "pose_retention_scenario");

  simviz::SimVizConfig config;
  config.control_port = kTestControlPort;
  config.handshake_port_start = kTestHandshakePortStart;
  config.vesc_port_start = kTestVescPortStart;
  config.loc_port_start = kTestLocPortStart;
  config.mpc_controller_path = fake_bin; // never answers the handshake --
  config.robot_sim_path = fake_bin;      // deliberate: drives a natural ERR.
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
  const int loc_port = kTestLocPortStart; // robot1 is the only/first robot -> index 0

  std::atomic<bool> client_done{false};
  std::atomic<bool> overall_pass{true};

  auto check = [&](bool cond, const std::string &what)
  {
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

  // Direct, in-process access to the manager's ExecutionManager -- this is
  // the "core-level" seam the ISSUE 1 fix targets (SimVizCore.h/.cpp),
  // independent of any GUI widget.
  simviz::ExecutionManager &exec_mgr = manager.execution_manager();

  auto poll_for_pose = [&](double expect_x, double expect_y, double timeout_s) -> bool
  {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
      auto live = exec_mgr.localization().latest_pose("robot1");
      if (live && live->has_pose && std::abs(live->pose.x - expect_x) < 1e-6 &&
          std::abs(live->pose.y - expect_y) < 1e-6)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  };

  std::thread client(
      [&]()
      {
        // ---------------------------------------------------------------
        // Run 1: EXECUTE -> live pose arrives -> the run ERRs out (fake
        // controller never handshakes) -> pose must SURVIVE that teardown
        // (finalize_run()'s Err path uses the exact same
        // terminate_children() the DONE path does).
        // ---------------------------------------------------------------
        {
          std::string reply = req(endpoint, "EXECUTE " + scn_path);
          check(reply == "ACK_EXECUTE", "run1 EXECUTE -> ACK_EXECUTE (" + reply + ")");

          std::atomic<bool> stop_pub{false};
          std::thread pub(publish_fake_localization, loc_port, "robot1", 5.0, 5.0, 1.0,
                           std::ref(stop_pub));

          const bool got_pose = poll_for_pose(5.0, 5.0, 5.0);
          check(got_pose, "run1: live pose (5,5) reached LocalizationListener while Running");

          // Let the run reach ERR (handshake never completes -> ~6.5s
          // timeout -> fail_handshake() -> tick() finalizes as Err).
          auto outcome_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
          std::string final_status;
          bool reached_err = false;
          while (std::chrono::steady_clock::now() < outcome_deadline)
          {
            final_status = req(endpoint, "STATUS");
            if (final_status.rfind("ERR ", 0) == 0)
            {
              reached_err = true;
              break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
          }
          check(reached_err, "run1: STATUS eventually ERR (final='" + final_status + "')");

          stop_pub.store(true);
          pub.join();

          // THE regression check: retention after teardown.
          auto live_after_err = exec_mgr.localization().latest_pose("robot1");
          check(live_after_err.has_value() && live_after_err->has_pose,
                "run1: latest_pose('robot1') still non-empty after ERR teardown (retention)");
          if (live_after_err.has_value())
          {
            check(std::abs(live_after_err->pose.x - 5.0) < 1e-6 &&
                      std::abs(live_after_err->pose.y - 5.0) < 1e-6,
                  "run1: retained pose still (5,5), not reverted to initial_pose (0,0)");
          }
        }

        // ---------------------------------------------------------------
        // Run 2: a FRESH EXECUTE of the same scenario must reset the
        // cache -- queried immediately (before any new sample can
        // possibly arrive), latest_pose() must be empty again.
        // ---------------------------------------------------------------
        {
          std::string reply = req(endpoint, "EXECUTE " + scn_path);
          check(reply == "ACK_EXECUTE", "run2 EXECUTE -> ACK_EXECUTE (" + reply + ")");

          auto live_immediately_after_execute = exec_mgr.localization().latest_pose("robot1");
          check(!live_immediately_after_execute.has_value(),
                "run2: latest_pose('robot1') reset to empty immediately after fresh EXECUTE "
                "(no stale pose carried over from run1)");
        }

        // ---------------------------------------------------------------
        // Still run 2: feed a DIFFERENT live pose, then ABORT it directly
        // -- the other run-ending state named in the ISSUE 1 spec. Pose
        // must survive that teardown too.
        // ---------------------------------------------------------------
        {
          std::atomic<bool> stop_pub{false};
          std::thread pub(publish_fake_localization, loc_port, "robot1", 7.0, 2.0, -0.5,
                           std::ref(stop_pub));

          const bool got_pose = poll_for_pose(7.0, 2.0, 5.0);
          check(got_pose, "run2: live pose (7,2) reached LocalizationListener while Running");

          std::string reply = req(endpoint, "ABORT");
          check(reply == "ACK_ABORT", "run2 ABORT -> ACK_ABORT (" + reply + ")");

          stop_pub.store(true);
          pub.join();

          auto live_after_abort = exec_mgr.localization().latest_pose("robot1");
          check(live_after_abort.has_value() && live_after_abort->has_pose,
                "run2: latest_pose('robot1') still non-empty after ABORT teardown (retention)");
          if (live_after_abort.has_value())
          {
            check(std::abs(live_after_abort->pose.x - 7.0) < 1e-6 &&
                      std::abs(live_after_abort->pose.y - 2.0) < 1e-6,
                  "run2: retained pose still (7,2), not reverted to initial_pose (0,0)");
          }
        }

        client_done.store(true);
      });

  // Drive the manager's tick loop on the main (Qt) thread -- QProcess and
  // friends were constructed here, so this is also where they must be
  // polled/torn down.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
  while (!client_done.load() && std::chrono::steady_clock::now() < deadline)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (!client_done.load())
  {
    std::cerr << "[test] FAILED: client thread did not finish within the 45s deadline"
              << std::endl;
    overall_pass.store(false);
  }
  client.join();

  // Final teardown ticks so any in-flight abort settles before exit.
  for (int i = 0; i < 10; ++i)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::filesystem::remove_all(tmp_dir);

  if (overall_pass.load())
  {
    std::cout << "\n[Test] All test_sim_viz_pose_retention tests passed." << std::endl;
    return 0;
  }
  std::cerr << "\n[Test] SOME test_sim_viz_pose_retention tests FAILED." << std::endl;
  return 1;
}
