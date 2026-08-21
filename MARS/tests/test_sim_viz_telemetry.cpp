// RIGHT-SIDE MONITOR PANEL (core layer): regression/coverage test for
// LocalizationListener's new telemetry tracking (SimVizCore.h/.cpp) --
// modeled directly on test_sim_viz_pose_retention.cpp's ISSUE 1 pose test,
// since telemetry rides the SAME SUB socket/thread and is documented to
// share the SAME retention/reset lifecycle as poses (see
// LocalizationListener::latest_telemetry()'s doc comment in SimVizCore.h).
//
// This test drives simviz::SimVizManager in-process (fake sleeping child
// binaries -- their real behavior doesn't matter, only that "some child
// process is alive" long enough to exercise Running) against the TEST PORT
// BLOCK (control=45601, handshake=45620+, vesc=45640+, loc=45660+), and
// stands in for what a real mpc_robot_sim would publish on its
// --loc-endpoint (BOTH the "/<robot>/localization" AND "/<robot>/telemetry"
// topics, on the SAME bound PUB socket -- see MPC/src/
// robot_sim.cpp) via a hand-rolled ZMQ PUB socket, so live samples reach
// ExecutionManager's LocalizationListener without needing the real MPC
// binaries.
//
// Exercises, in order:
//   (1) a live telemetry sample arrives while Running, and is queryable via
//       latest_telemetry() with every field round-tripped correctly
//       (including the moving/watchdog booleans);
//   (2) the run naturally ERRs out (the fake mpc_controller never answers
//       the handshake) -- finalize_run()'s teardown path is exactly the
//       terminate_children() call DONE uses, so this exercises the
//       identical mechanism; the telemetry sample must still be queryable
//       afterward (retention, mirroring pose retention);
//   (3) a fresh EXECUTE of the SAME scenario resets the telemetry cache
//       (queried immediately, before any new sample arrives, must be
//       empty again);
//   (4) a second run's live telemetry arrives, then ABORT (the other
//       run-ending state) -- telemetry must still be queryable afterward
//       too.
//   (5) LAG-DIAGNOSIS BUGFIX REGRESSION (three-clock lag-diagnosis task): a
//       fresh third run, then a burst of many localization messages
//       published back-to-back with no delay -- latest_pose() must catch up
//       to the LAST one within a short deadline (see that section's own
//       comment for the full story: LocalizationListener::thread_main()
//       used to drain only one message per socket per cycle, so cached
//       poses fell further and further behind over a long run).

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

// Fresh REQ socket per call (mirrors test_sim_viz_pose_retention.cpp's req()).
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

// Publishes BOTH "/<robot>/localization" ({"x":..,"y":..,"yaw":..}) AND
// "/<robot>/telemetry" ({"t":..,"v":..,"v_cmd":..,"steering":..,"accel":..,
// "moving":0|1,"watchdog":0|1}) on the SAME bound PUB socket, repeatedly
// until `stop` is set -- stands in for what the real mpc_robot_sim
// publishes on its --loc-endpoint (both topics, one socket -- see
// MPC/src/robot_sim.cpp / SimCore.h's TelemetrySample doc
// comment), without needing to spawn that real binary.
void publish_fake_localization_and_telemetry(int port, const std::string &robot_name, double x,
                                              double y, double yaw, double t, double v,
                                              double v_cmd, double steering, double accel,
                                              bool moving, bool watchdog, std::atomic<bool> &stop)
{
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  pub.set(zmq::sockopt::linger, 0);
  pub.bind("tcp://*:" + std::to_string(port));

  const std::string loc_topic = "/" + robot_name + "/localization";
  std::ostringstream loc_payload;
  loc_payload << "{\"x\":" << x << ",\"y\":" << y << ",\"yaw\":" << yaw << "}";
  const std::string loc_payload_str = loc_payload.str();

  const std::string telem_topic = "/" + robot_name + "/telemetry";
  std::ostringstream telem_payload;
  telem_payload << "{\"t\":" << t << ",\"v\":" << v << ",\"v_cmd\":" << v_cmd
                << ",\"steering\":" << steering << ",\"accel\":" << accel
                << ",\"moving\":" << (moving ? 1 : 0) << ",\"watchdog\":" << (watchdog ? 1 : 0)
                << "}";
  const std::string telem_payload_str = telem_payload.str();

  // PUB/SUB has no late-joiner replay -- give the SUB side a moment to
  // connect + subscribe (to BOTH topics) before the first publish.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (!stop.load())
  {
    {
      zmq::message_t topic_msg(loc_topic.begin(), loc_topic.end());
      zmq::message_t payload_msg(loc_payload_str.begin(), loc_payload_str.end());
      pub.send(topic_msg, zmq::send_flags::sndmore);
      pub.send(payload_msg, zmq::send_flags::none);
    }
    {
      zmq::message_t topic_msg(telem_topic.begin(), telem_topic.end());
      zmq::message_t payload_msg(telem_payload_str.begin(), telem_payload_str.end());
      pub.send(topic_msg, zmq::send_flags::sndmore);
      pub.send(payload_msg, zmq::send_flags::none);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}

} // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_telemetry_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::string fake_bin = write_fake_robot_binary(tmp_dir);
  const std::string scn_path = build_and_write_test_scenario(tmp_dir, "telemetry_scenario");

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

  simviz::ExecutionManager &exec_mgr = manager.execution_manager();

  auto poll_for_telemetry = [&](double expect_v, double timeout_s) -> bool
  {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
      auto live = exec_mgr.localization().latest_telemetry("robot1");
      if (live && live->has_telemetry && std::abs(live->v - expect_v) < 1e-6)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  };

  std::thread client(
      [&]()
      {
        // ---------------------------------------------------------------
        // Run 1: EXECUTE -> live telemetry arrives -> the run ERRs out
        // (fake controller never handshakes) -> telemetry must SURVIVE
        // that teardown.
        // ---------------------------------------------------------------
        {
          std::string reply = req(endpoint, "EXECUTE " + scn_path);
          check(reply == "ACK_EXECUTE", "run1 EXECUTE -> ACK_EXECUTE (" + reply + ")");

          std::atomic<bool> stop_pub{false};
          std::thread pub(publish_fake_localization_and_telemetry, loc_port, "robot1", 5.0, 5.0,
                           1.0, /*t=*/3.5, /*v=*/1.25, /*v_cmd=*/1.30, /*steering=*/0.11,
                           /*accel=*/0.42, /*moving=*/true, /*watchdog=*/false,
                           std::ref(stop_pub));

          const bool got_telem = poll_for_telemetry(1.25, 5.0);
          check(got_telem, "run1: live telemetry (v=1.25) reached LocalizationListener while "
                            "Running");

          // Round-trip every field, not just v (used only for the poll
          // above).
          {
            auto live = exec_mgr.localization().latest_telemetry("robot1");
            check(live.has_value() && live->has_telemetry, "run1: latest_telemetry() populated");
            if (live.has_value())
            {
              check(std::abs(live->t - 3.5) < 1e-6, "run1: telemetry.t round-tripped (3.5)");
              check(std::abs(live->v_cmd - 1.30) < 1e-6,
                    "run1: telemetry.v_cmd round-tripped (1.30)");
              check(std::abs(live->steering - 0.11) < 1e-6,
                    "run1: telemetry.steering round-tripped (0.11)");
              check(std::abs(live->accel - 0.42) < 1e-6,
                    "run1: telemetry.accel round-tripped (0.42)");
              check(live->moving == true, "run1: telemetry.moving round-tripped (true)");
              check(live->watchdog == false, "run1: telemetry.watchdog round-tripped (false)");
            }
          }

          // The SAME publish also fed a live pose -- LocalizationListener
          // should still be tracking both independently on the one socket.
          {
            auto live_pose = exec_mgr.localization().latest_pose("robot1");
            check(live_pose.has_value() && live_pose->has_pose &&
                      std::abs(live_pose->pose.x - 5.0) < 1e-6,
                  "run1: latest_pose() ALSO populated from the same fake publisher (pose/"
                  "telemetry tracked independently on one socket)");
          }

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
          auto live_after_err = exec_mgr.localization().latest_telemetry("robot1");
          check(live_after_err.has_value() && live_after_err->has_telemetry,
                "run1: latest_telemetry('robot1') still non-empty after ERR teardown "
                "(retention)");
          if (live_after_err.has_value())
          {
            check(std::abs(live_after_err->v - 1.25) < 1e-6,
                  "run1: retained telemetry.v still 1.25");
          }
        }

        // ---------------------------------------------------------------
        // Run 2: a FRESH EXECUTE of the same scenario must reset the
        // telemetry cache -- queried immediately (before any new sample
        // can possibly arrive), latest_telemetry() must be empty again.
        // ---------------------------------------------------------------
        {
          std::string reply = req(endpoint, "EXECUTE " + scn_path);
          check(reply == "ACK_EXECUTE", "run2 EXECUTE -> ACK_EXECUTE (" + reply + ")");

          auto live_immediately_after_execute = exec_mgr.localization().latest_telemetry("robot1");
          check(!live_immediately_after_execute.has_value(),
                "run2: latest_telemetry('robot1') reset to empty immediately after fresh "
                "EXECUTE (no stale sample carried over from run1)");
        }

        // ---------------------------------------------------------------
        // Still run 2: feed a DIFFERENT live telemetry sample, then ABORT
        // it directly -- the other run-ending state. Telemetry must
        // survive that teardown too.
        // ---------------------------------------------------------------
        {
          std::atomic<bool> stop_pub{false};
          std::thread pub(publish_fake_localization_and_telemetry, loc_port, "robot1", 7.0, 2.0,
                           -0.5, /*t=*/9.0, /*v=*/0.0, /*v_cmd=*/0.9, /*steering=*/-0.2,
                           /*accel=*/0.0, /*moving=*/false, /*watchdog=*/true,
                           std::ref(stop_pub));

          bool got_telem = false;
          const auto deadline =
              std::chrono::steady_clock::now() + std::chrono::seconds(5);
          while (std::chrono::steady_clock::now() < deadline)
          {
            auto live = exec_mgr.localization().latest_telemetry("robot1");
            if (live && live->has_telemetry && live->watchdog)
            {
              got_telem = true;
              break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }
          check(got_telem, "run2: live telemetry (watchdog=true, moving=false) reached "
                            "LocalizationListener while Running");

          std::string reply = req(endpoint, "ABORT");
          check(reply == "ACK_ABORT", "run2 ABORT -> ACK_ABORT (" + reply + ")");

          stop_pub.store(true);
          pub.join();

          auto live_after_abort = exec_mgr.localization().latest_telemetry("robot1");
          check(live_after_abort.has_value() && live_after_abort->has_telemetry,
                "run2: latest_telemetry('robot1') still non-empty after ABORT teardown "
                "(retention)");
          if (live_after_abort.has_value())
          {
            check(live_after_abort->moving == false && live_after_abort->watchdog == true,
                  "run2: retained telemetry still moving=false/watchdog=true, not reverted");
          }
        }

        // ---------------------------------------------------------------
        // LAG-DIAGNOSIS BUGFIX REGRESSION (three-clock lag-diagnosis task):
        // LocalizationListener::thread_main() used to recv() AT MOST ONE
        // message per socket per ~20ms outer-loop cycle. Localization AND
        // telemetry are both published at --loc-rate-hz (default 30Hz --
        // see robot_sim.cpp), i.e. ~60 combined messages/s per robot, which
        // a 1-message/~20ms drain rate (~50/s budget) cannot keep up with;
        // ZMQ SUB sockets queue backlog rather than dropping old messages
        // (no ZMQ_CONFLATE), so latest_pose() would silently fall further
        // and further behind over a long run -- confirmed LIVE on a real
        // 3-robot run (see this task's report): one robot's cached pose was
        // over 26 SECONDS stale by ~90s into the run and still falling
        // further behind, even though the robot's true simulated position
        // tracked its reference to centimeter accuracy the whole time. This
        // is a strong candidate for exactly what this task's user bug
        // report saw (the GUI's rendering of a robot's own live position
        // falling further behind over time). Fixed: drain ALL queued
        // messages per socket per cycle, not just one.
        //
        // Regression test: publish a burst of kBurstCount messages with NO
        // inter-message delay (emulating "many messages queued up") on a
        // FRESH run (fresh EXECUTE resets the pose cache), each with a
        // distinct, identifiable y coordinate ending at kBurstCount-1, then
        // assert latest_pose() reaches that EXACT final value within
        // kBurstDeadlineS. Under the pre-fix single-recv-per-cycle code,
        // draining kBurstCount messages would take
        // roughly kBurstCount*20ms (e.g. 200*20ms = 4s) at minimum --
        // kBurstDeadlineS is chosen well under that, so this test would
        // FAIL against the pre-fix code and PASSES against the fix (drains
        // an entire burst in a handful of ~20ms cycles regardless of size).
        // ---------------------------------------------------------------
        {
          std::string reply = req(endpoint, "EXECUTE " + scn_path);
          check(reply == "ACK_EXECUTE", "run3 EXECUTE -> ACK_EXECUTE (" + reply + ")");

          auto live_immediately_after_execute = exec_mgr.localization().latest_pose("robot1");
          check(!live_immediately_after_execute.has_value(),
                "run3: latest_pose('robot1') reset to empty immediately after fresh EXECUTE");

          constexpr int kBurstCount = 200;
          constexpr double kBurstDeadlineS = 1.5;

          zmq::context_t burst_ctx(1);
          zmq::socket_t burst_pub(burst_ctx, zmq::socket_type::pub);
          burst_pub.set(zmq::sockopt::linger, 0);
          burst_pub.bind("tcp://*:" + std::to_string(loc_port));
          const std::string loc_topic = "/robot1/localization";
          // PUB/SUB has no late-joiner replay -- give the SUB side a moment
          // to connect + subscribe before the burst (same rationale as
          // publish_fake_localization_and_telemetry's own 100ms wait).
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          auto send_burst = [&]()
          {
            for (int i = 0; i < kBurstCount; ++i)
            {
              std::ostringstream payload;
              payload << "{\"x\":1.0,\"y\":" << i << ",\"yaw\":0.0}";
              const std::string payload_str = payload.str();
              zmq::message_t topic_msg(loc_topic.begin(), loc_topic.end());
              zmq::message_t payload_msg(payload_str.begin(), payload_str.end());
              burst_pub.send(topic_msg, zmq::send_flags::sndmore);
              burst_pub.send(payload_msg, zmq::send_flags::none);
              // No sleep WITHIN a burst -- fire it as fast as possible,
              // exactly the "arrival outpaces the old drain rate" scenario.
            }
          };
          // Send the whole burst up to kMaxBurstAttempts times, short-
          // circuiting as soon as latest_pose() catches up: a PUB socket has
          // no guaranteed delivery to a SUB that only *just* connected (the
          // well-known ZMQ "slow joiner" symptom -- a subscription can take
          // a few ms to propagate, so a single one-shot burst sent
          // immediately after the 100ms wait above can occasionally arrive
          // before the subscription is fully live and be entirely missed);
          // repeating is the standard, simplest mitigation, and does not
          // weaken what this test is actually checking -- the DRAIN rate
          // once messages ARE arriving, not first-message delivery
          // guarantees; the assertion below is still keyed off the same
          // short kBurstDeadlineS-scale timing.
          constexpr int kMaxBurstAttempts = 5;
          for (int attempt = 0; attempt < kMaxBurstAttempts; ++attempt)
          {
            send_burst();
            auto live = exec_mgr.localization().latest_pose("robot1");
            if (live && live->has_pose && live->pose.y >= kBurstCount - 1 - 1e-6)
              break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
          }

          bool caught_up = false;
          const auto burst_deadline =
              std::chrono::steady_clock::now() + std::chrono::duration<double>(kBurstDeadlineS);
          while (std::chrono::steady_clock::now() < burst_deadline)
          {
            auto live = exec_mgr.localization().latest_pose("robot1");
            if (live && live->has_pose && live->pose.y >= kBurstCount - 1 - 1e-6)
            {
              caught_up = true;
              break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
          if (!caught_up)
          {
            auto live = exec_mgr.localization().latest_pose("robot1");
            std::cerr << "[test] DIAG: latest_pose after deadline: has_pose="
                      << (live.has_value() ? live->has_pose : false)
                      << " y=" << (live.has_value() ? live->pose.y : -1.0) << std::endl;
          }
          check(caught_up, "run3: latest_pose('robot1') caught up to the LAST of " +
                                std::to_string(kBurstCount) + " burst-published messages within " +
                                std::to_string(kBurstDeadlineS) +
                                "s (drains fully instead of falling behind)");

          req(endpoint, "ABORT");
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
    std::cout << "\n[Test] All test_sim_viz_telemetry tests passed." << std::endl;
    return 0;
  }
  std::cerr << "\n[Test] SOME test_sim_viz_telemetry tests FAILED." << std::endl;
  return 1;
}
