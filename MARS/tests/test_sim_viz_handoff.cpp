// Phase A regression tests for the MARS-side sim-viz handoff gate (see
// MARS/include/SimVizHandoff.h and MARS/include/RobotTrajectoryBuilder.h).
//
// (a) viz absent -> VizAbsent within bounded time, no file written, no hang.
// (b) mock viz REP thread (PONG + ACK_EXECUTE) -> HandoffOk, EXECUTE message
//     carries the absolute path of a file that exists and deserializes back
//     to a scenario containing the same robot AND object entities plus a
//     non-empty timetable.
// (c) trajectory-builder parity: hand-computed timetable fixture -> verify
//     build_robot_trajectories' waypoint count/rel-times/ref_vel/is_pushing
//     against hand-computed expectations.
//
// Uses the TEST PORT BLOCK control port (45601) reserved for this task; no
// other ports are needed since Phase A never spawns robot processes.

#include <SimVizHandoff.h>
#include <RobotTrajectoryBuilder.h>
#include <ExecutedScenarioSerialization.h>
#include <AllocationSearch.h> // GOAL-POSE FIX regression test: FinalAllocation, Task, initialize_tasks()
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <zmq.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <unistd.h>

// Required global for configuration used by other linked MARS sources.
bool DEBUG_VIS = false;

// NOTE: plain assert() is compiled out by -DNDEBUG, which is exactly what
// this project's build-release target uses (CXX_FLAGS includes -DNDEBUG).
// TEST_ASSERT below is functionally identical to assert() (abort with a
// diagnostic on failure) but is NEVER compiled out, so this test binary
// actually verifies something when built via the mandated
// `cmake --build build-release` workflow.
#define TEST_ASSERT(cond)                                                            \
  do                                                                                  \
  {                                                                                   \
    if (!(cond))                                                                     \
    {                                                                                 \
      std::cerr << "[test] ASSERTION FAILED: " << #cond << " at " << __FILE__ << ":" \
                << __LINE__ << std::endl;                                            \
      std::abort();                                                                  \
    }                                                                                 \
  } while (0)

namespace
{
constexpr int kTestControlPort = 45601;

bool near_eq(double a, double b, double eps = 1e-4)
{
  return std::fabs(a - b) < eps;
}

// -----------------------------------------------------------------------
// Shared fixture: a tiny ExecutedScenario with one robot and one object,
// connected by a single transfer trajectory span.
// -----------------------------------------------------------------------
ExecutedScenario build_test_scenario(const std::string &label)
{
  ExecutedScenario scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 1;
  scn.summary.failed_tasks = 0;
  scn.summary.makespan = 10.0;

  RobotMeta *robot1 = new RobotMeta();
  robot1->name = "robot1";
  robot1->type = EntityType::ROBOT;
  robot1->initial_pose = Pose(0.0, 0.0, 0.0);
  robot1->speed_transit = 0.2;
  robot1->speed_transfer = 0.15;

  ObjectMeta *object1 = new ObjectMeta();
  object1->name = "object1";
  object1->type = EntityType::OBJECT;
  object1->initial_pose = Pose(1.0, 1.0, 0.0);
  object1->goal_pose = Pose(3.0, 3.0, 0.0);

  scn.entities["robot1"] = robot1;
  scn.entities["object1"] = object1;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  Pose p1(0.0, 0.0, 0.0);
  Pose p2(1.0, 0.0, 0.0);
  Pose p3(2.0, 0.0, 0.0);
  WaypointPath path = {Waypoint(p1), Waypoint(p2), Waypoint(p3)};
  Trajectory traj(robot1, object1, 0.0, path, /*is_transfer=*/true);
  traj.CalcualteTimeStamps(robot1, 0.0);
  scn.timetable.add_trajectory(traj);

  return scn;
}

std::string read_file(const std::filesystem::path &path)
{
  std::ifstream in(path, std::ios::binary);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// -----------------------------------------------------------------------
// (a) Visualizer absent
// -----------------------------------------------------------------------
void test_viz_absent()
{
  std::cout << "[Test] (a) viz absent..." << std::endl;

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_handoff_test_absent_" + std::to_string(getpid()));
  std::filesystem::remove_all(out_dir);

  RuntimeOptions options;
  options.sim_viz_handoff = true;
  options.sim_viz_endpoint = "tcp://127.0.0.1:" + std::to_string(kTestControlPort);
  options.sim_handoff_out_dir = out_dir.string();

  ExecutedScenario scn = build_test_scenario("viz_absent_test");

  const auto t0 = std::chrono::steady_clock::now();
  const SimVizHandoffResult result =
      maybe_handoff_to_sim_viz(options, scn, "viz_absent_test");
  const double elapsed_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  std::cout << "[Test]   result=" << sim_viz_handoff_result_name(result)
            << " elapsed=" << elapsed_s << "s" << std::endl;

  TEST_ASSERT(result == SimVizHandoffResult::VizAbsent);
  TEST_ASSERT(elapsed_s < 4.0);
  TEST_ASSERT(!std::filesystem::exists(out_dir / "viz_absent_test.scn.b64"));

  std::filesystem::remove_all(out_dir);
  std::cout << "[Test] (a) PASSED" << std::endl;
}

// -----------------------------------------------------------------------
// (b) Mock visualizer accepts the handoff
// -----------------------------------------------------------------------
void run_mock_sim_viz(int port, std::atomic<bool> &stop_flag,
                       std::string &captured_execute_path)
{
  zmq::context_t ctx(1);
  zmq::socket_t sock(ctx, zmq::socket_type::rep);
  sock.set(zmq::sockopt::linger, 0);
  sock.set(zmq::sockopt::rcvtimeo, 200);
  sock.bind("tcp://127.0.0.1:" + std::to_string(port));

  while (!stop_flag.load())
  {
    zmq::message_t req;
    const auto recv_result = sock.recv(req, zmq::recv_flags::none);
    if (!recv_result)
      continue;

    const std::string msg(static_cast<const char *>(req.data()), req.size());
    if (msg == "PING")
    {
      const std::string reply = "PONG mars_sim_viz v1";
      sock.send(zmq::buffer(reply), zmq::send_flags::none);
    }
    else if (msg.rfind("EXECUTE ", 0) == 0)
    {
      captured_execute_path = msg.substr(std::string("EXECUTE ").size());
      const std::string reply = "ACK_EXECUTE";
      sock.send(zmq::buffer(reply), zmq::send_flags::none);
      stop_flag.store(true);
    }
    else
    {
      const std::string reply = "ERR unexpected message in test mock";
      sock.send(zmq::buffer(reply), zmq::send_flags::none);
    }
  }
}

void test_handoff_ok()
{
  std::cout << "[Test] (b) mock viz accepts handoff..." << std::endl;

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_handoff_test_ok_" + std::to_string(getpid()));
  std::filesystem::remove_all(out_dir);

  std::atomic<bool> stop{false};
  std::string captured_execute_path;
  std::thread server(run_mock_sim_viz, kTestControlPort, std::ref(stop),
                      std::ref(captured_execute_path));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  RuntimeOptions options;
  options.sim_viz_handoff = true;
  options.sim_viz_endpoint = "tcp://127.0.0.1:" + std::to_string(kTestControlPort);
  options.sim_handoff_out_dir = out_dir.string();

  ExecutedScenario scn = build_test_scenario("handoff_ok_test");
  const SimVizHandoffResult result =
      maybe_handoff_to_sim_viz(options, scn, "handoff_ok_test");

  // Make sure the mock server thread has a chance to stop, then join.
  stop.store(true);
  server.join();

  std::cout << "[Test]   result=" << sim_viz_handoff_result_name(result) << std::endl;
  TEST_ASSERT(result == SimVizHandoffResult::HandoffOk);

  TEST_ASSERT(!captured_execute_path.empty());
  const std::filesystem::path execute_path(captured_execute_path);
  TEST_ASSERT(execute_path.is_absolute());
  TEST_ASSERT(std::filesystem::exists(execute_path));

  const std::string blob = read_file(execute_path);
  TEST_ASSERT(!blob.empty());

  ExecutedScenario loaded = deserialize_executed_scenario_b64(blob);
  TEST_ASSERT(loaded.entities.count("robot1") == 1);
  TEST_ASSERT(loaded.entities.count("object1") == 1);
  TEST_ASSERT(loaded.entities.at("robot1")->type == EntityType::ROBOT);
  TEST_ASSERT(loaded.entities.at("object1")->type == EntityType::OBJECT);
  TEST_ASSERT(!loaded.timetable.get_database().empty());
  TEST_ASSERT(!loaded.timetable.get_trajectory_spans().empty());

  std::filesystem::remove_all(out_dir);
  std::cout << "[Test] (b) PASSED" << std::endl;
}

// -----------------------------------------------------------------------
// (d) GOAL-POSE FIX regression: ObjectMeta::goal_pose is back-filled by
// Task's constructor (see MARS/include/Task.h) from FinalAllocation::
// goalPose -- the REAL write path (initialize_tasks() -> Task(fa, entities)
// -> targetObject->goal_pose = GoalPoseObj), not a hand-populated
// ExecutedScenario fixture like build_test_scenario() above (which bypasses
// the bug entirely by setting object1->goal_pose directly). Before the fix,
// `new ObjectMeta` left goal_pose's inherited x/y indeterminate (only Pose's
// own `yaw` NSDMI zero-inits) and Task's constructor never wrote to it, so
// this test would have observed garbage x/y (and yaw==0.0 regardless of the
// FinalAllocation's actual goal yaw) instead of the asserted values.
// -----------------------------------------------------------------------
void test_goal_pose_write_back()
{
  std::cout << "[Test] (d) goal-pose write-back via initialize_tasks()..." << std::endl;

  // Mirrors AllocationSearch.cpp's initialize_entities() EXACTLY (including
  // the no-parens `new ObjectMeta` -- default-, not value-, initialization,
  // which is what leaves GeometryPoint's inherited x/y indeterminate rather
  // than zero-initialized; see the goal-pose diagnosis) so this test
  // reproduces the real pre-fix bug rather than a milder stand-in.
  ObjectMeta *obj = new ObjectMeta;
  obj->name = "object1";
  obj->type = EntityType::OBJECT;
  obj->initial_pose = Pose(1.0, 1.0, 0.0);

  std::unordered_map<std::string, EntityMeta *> entities;
  entities["object1"] = obj;

  FinalAllocation fa;
  fa.object.name = "object1";
  fa.startPose = ReloPush::State(1.0, 1.0, 0.0);
  fa.goalPose = ReloPush::State(5.25, 6.75, 1.2);

  const std::vector<Task> tasks = initialize_tasks({fa}, entities);
  TEST_ASSERT(tasks.size() == 1);

  ObjectMeta *loaded_obj = dynamic_cast<ObjectMeta *>(entities.at("object1"));
  TEST_ASSERT(loaded_obj != nullptr);

  const double expected_yaw = NormalizeReloPushYaw(fa.goalPose.yaw);
  std::cout << "[Test]   entities[\"object1\"]->goal_pose = (" << loaded_obj->goal_pose.x << ", "
            << loaded_obj->goal_pose.y << ", " << loaded_obj->goal_pose.yaw << ")" << std::endl;
  TEST_ASSERT(near_eq(loaded_obj->goal_pose.x, fa.goalPose.x));
  TEST_ASSERT(near_eq(loaded_obj->goal_pose.y, fa.goalPose.y));
  TEST_ASSERT(near_eq(loaded_obj->goal_pose.yaw, expected_yaw));

  // Also verify the Task object's own GoalPoseObj (the value the fix reads
  // FROM) agrees, so a future regression that decouples the two is caught
  // here rather than only downstream.
  TEST_ASSERT(near_eq(tasks[0].GoalPoseObj.x, fa.goalPose.x));
  TEST_ASSERT(near_eq(tasks[0].GoalPoseObj.y, fa.goalPose.y));
  TEST_ASSERT(near_eq(tasks[0].GoalPoseObj.yaw, expected_yaw));

  std::cout << "[Test] (d) PASSED" << std::endl;
}

// -----------------------------------------------------------------------
// (c) Trajectory-builder parity against a hand-computed fixture. Includes a
// robot3 case reproducing a REAL MARS same-pose hold gap (see
// RobotTrajectoryBuilder.h's HOLD-SEGMENT ZEROING doc comment): a forward
// transit leg, then a 10s gap at the same pose (mirroring the production
// incident's ~9.5s hold between a +0.2 transit leg and a -0.15 reversing
// transfer leg), then a reversing transfer leg -- verifies BOTH waypoints
// bracketing the hold read ref_vel==0.0 (not the pre-fix +0.2/-0.15 the two
// neighboring moving legs would otherwise have left them at) while the
// moving legs on either side keep their normal nonzero, correctly-signed
// values.
// -----------------------------------------------------------------------
void test_trajectory_builder_parity()
{
  std::cout << "[Test] (c) trajectory-builder parity..." << std::endl;

  RobotMeta robot1;
  robot1.name = "robot1";
  robot1.type = EntityType::ROBOT;
  robot1.speed_transit = 0.2;
  robot1.speed_transfer = 0.15;

  RobotMeta robot2;
  robot2.name = "robot2";
  robot2.type = EntityType::ROBOT;
  robot2.speed_transit = 0.3;
  robot2.speed_transfer = 0.1;

  RobotMeta robot3;
  robot3.name = "robot3";
  robot3.type = EntityType::ROBOT;
  robot3.speed_transit = 0.2;
  robot3.speed_transfer = 0.15;

  // robot1: three transit-only samples starting at absolute t=10 (nonzero
  // start, to check rel_t is offset relative to THIS robot's first sample).
  std::unordered_map<EntityMeta *, std::map<double, Pose>> per_entity;
  per_entity[&robot1] = {
      {10.0, Pose(0.0, 0.0, 0.0)},
      {10.5, Pose(1.0, 0.0, 0.0)},
      {11.0, Pose(2.0, 0.0, 0.0)},
  };

  // robot2: three samples at t=0,1,2; a transfer span [0.5, 1.5] covers only
  // the middle sample.
  per_entity[&robot2] = {
      {0.0, Pose(0.0, 0.0, 0.0)},
      {1.0, Pose(1.0, 0.0, 0.0)},
      {2.0, Pose(2.0, 0.0, 0.0)},
  };

  // robot3: forward transit leg (t=0 -> t=1, dir_sign=+1) -> a 10s same-pose
  // hold (t=1 -> t=11, pose frozen at (1,0,0)) -> a reversing transfer leg
  // (t=11.5 -> t=12, pose 1.0 -> 0.0, dir_sign flips to -1) once the
  // transfer span [11.2, 12.0] begins. The hold's own dt (10.0s) vastly
  // exceeds the moving legs' dt (1.0s/0.5s), matching a real plan's wildly
  // uneven sample spacing.
  per_entity[&robot3] = {
      {0.0, Pose(0.0, 0.0, 0.0)},
      {1.0, Pose(1.0, 0.0, 0.0)},
      {11.0, Pose(1.0, 0.0, 0.0)},
      {11.5, Pose(0.5, 0.0, 0.0)},
      {12.0, Pose(0.0, 0.0, 0.0)},
  };

  TimeTable::TrajectorySpan span;
  span.entity = &robot2;
  span.transferred_object = nullptr;
  span.start_time = 0.5;
  span.end_time = 1.5;
  span.is_transfer = true;
  span.kind = TrajectoryKind::TRANSFER;

  TimeTable::TrajectorySpan span3;
  span3.entity = &robot3;
  span3.transferred_object = nullptr;
  span3.start_time = 11.2;
  span3.end_time = 12.0;
  span3.is_transfer = true;
  span3.kind = TrajectoryKind::TRANSFER;

  std::vector<TimeTable::TrajectorySpan> spans = {span, span3};

  TimeTable timetable(0.5);
  timetable.load_serialized_state(0.5, per_entity, spans);

  std::vector<EntityMeta *> robots_sorted_by_name = {&robot1, &robot2, &robot3};
  auto trajectories = build_robot_trajectories(timetable, robots_sorted_by_name);

  TEST_ASSERT(trajectories.size() == 3);

  // --- robot1 ---
  TEST_ASSERT(trajectories[0].first == &robot1);
  const auto &t1 = *trajectories[0].second.trajectory_points;
  TEST_ASSERT(t1.size() == 3);
  const float expected_rel_t1[3] = {0.0f, 0.5f, 1.0f};
  for (size_t i = 0; i < 3; ++i)
  {
    TEST_ASSERT(near_eq(t1[i].time, expected_rel_t1[i]));
    TEST_ASSERT(t1[i].is_pushing == false);
    TEST_ASSERT(near_eq(t1[i].ref_vel, robot1.speed_transit));
  }

  // --- robot2 ---
  TEST_ASSERT(trajectories[1].first == &robot2);
  const auto &t2 = *trajectories[1].second.trajectory_points;
  TEST_ASSERT(t2.size() == 3);
  const float expected_rel_t2[3] = {0.0f, 1.0f, 2.0f};
  const bool expected_pushing2[3] = {false, true, false};
  for (size_t i = 0; i < 3; ++i)
  {
    TEST_ASSERT(near_eq(t2[i].time, expected_rel_t2[i]));
    TEST_ASSERT(t2[i].is_pushing == expected_pushing2[i]);
    const double expected_vel =
        expected_pushing2[i] ? robot2.speed_transfer : robot2.speed_transit;
    TEST_ASSERT(near_eq(t2[i].ref_vel, expected_vel));
  }

  // --- robot3: the hold case -- see this function's own doc comment. ---
  TEST_ASSERT(trajectories[2].first == &robot3);
  const auto &t3 = *trajectories[2].second.trajectory_points;
  TEST_ASSERT(t3.size() == 5);
  const float expected_rel_t3[5] = {0.0f, 1.0f, 11.0f, 11.5f, 12.0f};
  const bool expected_pushing3[5] = {false, false, false, true, true};
  // index 1 (t=1, hold-begin) and index 2 (t=11, hold-end) must both read
  // exactly 0.0 -- pre-fix they would have carried +0.2 (leg1's transit
  // speed, dir_sign carried forward across the near-zero displacement) and
  // -0.15 (leg2's own reversing transfer speed, computed from ITS forward
  // lookahead) respectively, never zero.
  const float expected_vel3[5] = {
      static_cast<float>(robot3.speed_transit),   // t=0: forward transit leg.
      0.0f,                                        // t=1: hold-begin.
      0.0f,                                        // t=11: hold-end.
      static_cast<float>(-robot3.speed_transfer), // t=11.5: reversing transfer leg.
      static_cast<float>(-robot3.speed_transfer), // t=12: reversing transfer leg (last waypoint).
  };
  for (size_t i = 0; i < 5; ++i)
  {
    TEST_ASSERT(near_eq(t3[i].time, expected_rel_t3[i]));
    TEST_ASSERT(t3[i].is_pushing == expected_pushing3[i]);
    TEST_ASSERT(near_eq(t3[i].ref_vel, expected_vel3[i]));
  }

  std::cout << "[Test] (c) PASSED" << std::endl;
}

// -----------------------------------------------------------------------
// (e) Reversal direction sign: a robot driving BACKWARD (position moving
// opposite to its own yaw) must get a NEGATIVE ref_vel, not the always-
// positive value the pre-fix builder produced. See RobotTrajectoryBuilder.h's
// dir_sign doc comment.
// -----------------------------------------------------------------------
void test_trajectory_builder_reversal_sign()
{
  std::cout << "[Test] (e) trajectory-builder reversal sign..." << std::endl;

  RobotMeta robot3;
  robot3.name = "robot3";
  robot3.type = EntityType::ROBOT;
  robot3.speed_transit = 0.2;
  robot3.speed_transfer = 0.15;

  // Facing yaw=0 (i.e. +x) throughout. Samples 0->1->2 move forward (+x);
  // samples 2->3->4 move backward (-x) while yaw stays 0 -- a genuine
  // unsignaled reversal, e.g. a three-point-turn-free backup maneuver.
  // Sample 4 is the last waypoint (no lookahead) and must carry forward the
  // reversing sign rather than snapping back to +1.
  std::unordered_map<EntityMeta *, std::map<double, Pose>> per_entity;
  per_entity[&robot3] = {
      {0.0, Pose(0.0, 0.0, 0.0)},
      {1.0, Pose(1.0, 0.0, 0.0)},
      {2.0, Pose(2.0, 0.0, 0.0)},
      {3.0, Pose(1.0, 0.0, 0.0)},
      {4.0, Pose(0.0, 0.0, 0.0)},
  };

  TimeTable timetable(0.5);
  timetable.load_serialized_state(0.5, per_entity, {});

  std::vector<EntityMeta *> robots_sorted_by_name = {&robot3};
  auto trajectories = build_robot_trajectories(timetable, robots_sorted_by_name);

  TEST_ASSERT(trajectories.size() == 1);
  const auto &t3 = *trajectories[0].second.trajectory_points;
  TEST_ASSERT(t3.size() == 5);

  const double expected_sign[5] = {1.0, 1.0, -1.0, -1.0, -1.0};
  for (size_t i = 0; i < 5; ++i)
  {
    const double expected_vel = expected_sign[i] * robot3.speed_transit;
    TEST_ASSERT(near_eq(t3[i].ref_vel, expected_vel));
  }

  std::cout << "[Test] (e) PASSED" << std::endl;
}
} // namespace

int main()
{
  test_viz_absent();
  test_handoff_ok();
  test_goal_pose_write_back();
  test_trajectory_builder_parity();
  test_trajectory_builder_reversal_sign();

  std::cout << "\n[Test] All test_sim_viz_handoff tests passed." << std::endl;
  return 0;
}
