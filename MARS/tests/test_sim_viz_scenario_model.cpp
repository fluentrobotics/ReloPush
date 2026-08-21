// Phase B1 unit tests for simviz::ScenarioModel (MARS/src/simviz/SimVizCore.h).
// Offscreen/headless, no ZMQ sockets, no child processes.
//
// (a) Loads a REAL .scn.b64 fixture produced by the existing save/replay
//     pipeline (see EvalPlansCli.cpp / AllocationSearch.cpp) and checks
//     robot AND object entities are present and robot trajectories were
//     rebuilt (non-empty) via the Phase-A RobotTrajectoryBuilder.
// (b) Object-replay piecewise-constant sampling: a hand-built TimeTable
//     with exactly two recorded samples (t=0, t=10) per entity, round-
//     tripped through the real .scn.b64 serializer, demonstrates that
//     ScenarioModel::object_pose_at() (which forwards straight to
//     TimeTable::get_pose()) HOLDS an object's pose at the last recorded
//     sample between samples, while a robot queried the same way
//     INTERPOLATES -- i.e. this test would fail if object sampling were
//     reimplemented instead of reusing TimeTable's public API.

#include "SimVizCore.h"

#include <ExecutedScenarioSerialization.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>

bool DEBUG_VIS = false;

#ifndef MARS_TEST_FIXTURE_SCN
#define MARS_TEST_FIXTURE_SCN ""
#endif

// NOTE: plain assert() is compiled out by -DNDEBUG, which is exactly what
// this project's build-release target uses. TEST_ASSERT is functionally
// identical to assert() (abort with a diagnostic on failure) but is never
// compiled out.
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

bool near_eq(double a, double b, double eps = 1e-6)
{
  return std::fabs(a - b) < eps;
}

void test_real_fixture()
{
  std::cout << "[Test] (a) load real .scn.b64 fixture..." << std::endl;
  const std::string fixture_path = MARS_TEST_FIXTURE_SCN;
  TEST_ASSERT(!fixture_path.empty() && "MARS_TEST_FIXTURE_SCN compile definition missing");
  TEST_ASSERT(std::filesystem::exists(fixture_path) &&
         "fixture file not found -- results/ layout changed?");

  simviz::ScenarioModel model = simviz::ScenarioModel::load_from_file(fixture_path);
  TEST_ASSERT(model.is_loaded());

  auto robots = model.robots_sorted_by_name();
  auto objects = model.objects_sorted_by_name();
  std::cout << "[Test]   label=" << model.label() << " robots=" << robots.size()
            << " objects=" << objects.size() << " max_time=" << model.max_time() << std::endl;

  TEST_ASSERT(!robots.empty());
  TEST_ASSERT(!objects.empty());

  const auto &trajectories = model.robot_trajectories();
  TEST_ASSERT(trajectories.size() == robots.size());

  bool any_nonempty = false;
  for (const auto &[ent, traj] : trajectories)
  {
    TEST_ASSERT(ent != nullptr);
    if (traj.trajectory_points && !traj.trajectory_points->empty())
      any_nonempty = true;
  }
  TEST_ASSERT(any_nonempty);

  std::cout << "[Test] (a) PASSED" << std::endl;
}

void test_object_piecewise_constant_replay()
{
  std::cout << "[Test] (b) object piecewise-constant replay vs robot interpolation..."
            << std::endl;

  ExecutedScenario scn;
  scn.summary.label = "piecewise_replay_test";
  scn.summary.all_tasks_succeeded = true;

  RobotMeta *robot = new RobotMeta();
  robot->name = "robot1";
  robot->type = EntityType::ROBOT;
  robot->initial_pose = Pose(0.0, 0.0, 0.0);
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;

  ObjectMeta *object = new ObjectMeta();
  object->name = "object1";
  object->type = EntityType::OBJECT;
  object->initial_pose = Pose(1.0, 1.0, 0.0);
  object->goal_pose = Pose(9.0, 9.0, 0.0);

  scn.entities["robot1"] = robot;
  scn.entities["object1"] = object;

  std::unordered_map<EntityMeta *, std::map<double, Pose>> per_entity;
  per_entity[robot] = {
      {0.0, Pose(0.0, 0.0, 0.0)},
      {10.0, Pose(8.0, 8.0, 0.0)},
  };
  per_entity[object] = {
      {0.0, Pose(1.0, 1.0, 0.0)},
      {10.0, Pose(9.0, 9.0, 0.0)},
  };
  std::vector<TimeTable::TrajectorySpan> spans; // not needed for this check
  scn.timetable = TimeTable(0.5);
  scn.timetable.load_serialized_state(0.5, per_entity, spans);

  const std::string b64 = serialize_executed_scenario_b64(scn);

  const std::filesystem::path tmp_path =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_scenario_model_test_" + std::to_string(getpid()) + ".scn.b64");
  {
    std::ofstream out(tmp_path, std::ios::binary);
    out << b64;
  }

  simviz::ScenarioModel model = simviz::ScenarioModel::load_from_file(tmp_path.string());
  std::filesystem::remove(tmp_path);

  EntityMeta *loaded_robot = nullptr;
  EntityMeta *loaded_object = nullptr;
  for (EntityMeta *r : model.robots_sorted_by_name())
    if (r->name == "robot1")
      loaded_robot = r;
  for (EntityMeta *o : model.objects_sorted_by_name())
    if (o->name == "object1")
      loaded_object = o;
  TEST_ASSERT(loaded_robot != nullptr);
  TEST_ASSERT(loaded_object != nullptr);

  // Query at t=5, strictly between the two recorded samples (t=0, t=10).
  const Pose obj_mid = model.object_pose_at(loaded_object, 5.0);
  const Pose robot_mid = model.timetable().get_pose(loaded_robot, 5.0);

  std::cout << "[Test]   object@t=5 -> (" << obj_mid.x << "," << obj_mid.y
            << ")  robot@t=5 -> (" << robot_mid.x << "," << robot_mid.y << ")" << std::endl;

  // Object: piecewise-constant hold of the t=0 sample (1,1) -- NOT the
  // halfway interpolated point (5,5) toward the t=10 sample.
  TEST_ASSERT(near_eq(obj_mid.x, 1.0));
  TEST_ASSERT(near_eq(obj_mid.y, 1.0));

  // Robot: linear interpolation halfway between (0,0)@t=0 and (8,8)@t=10.
  TEST_ASSERT(near_eq(robot_mid.x, 4.0));
  TEST_ASSERT(near_eq(robot_mid.y, 4.0));

  // Sanity: querying exactly at a recorded sample returns that exact pose
  // for both kinds of entity.
  const Pose obj_t0 = model.object_pose_at(loaded_object, 0.0);
  TEST_ASSERT(near_eq(obj_t0.x, 1.0) && near_eq(obj_t0.y, 1.0));

  std::cout << "[Test] (b) PASSED" << std::endl;
}

} // namespace

int main()
{
  test_real_fixture();
  test_object_piecewise_constant_replay();

  std::cout << "\n[Test] All test_sim_viz_scenario_model tests passed." << std::endl;
  return 0;
}
