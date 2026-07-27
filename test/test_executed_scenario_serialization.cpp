// Unit test for MARS/include/ExecutedScenarioSerialization.h -- the
// save-and-replay serializer for a scored ExecutedScenario (params +
// entities + timetable). Builds a synthetic ExecutedScenario purely in
// memory (no planner run), serializes it to a base64 blob, deserializes it
// back, and asserts a lossless round-trip of everything the viewer
// (show_results()) actually reads plus every Params scalar field.
//
// No test framework dependency (matches test/test_load_finalSequence.cpp's
// style): plain asserts via a small `ok` helper, fprintf on failure, and
// "SERIALIZATION_TEST_OK" + exit 0 on success.

#include <ExecutedScenarioSerialization.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

// Required by PHAstar.h (see MARS/tests/phastar_unit_tests.cpp for the same
// pattern) -- this test target links AllocationSearch.cpp/SafeParking.cpp/
// etc. directly rather than PHAstar_push_demo.cpp (which normally defines
// this global), so it must be defined here instead.
bool DEBUG_VIS = false;

namespace
{

bool g_failed = false;

void ok(bool cond, const char *what)
{
  if (!cond)
  {
    std::fprintf(stderr, "[FAIL] %s\n", what);
    g_failed = true;
  }
}

bool nearly(double a, double b, double eps = 1e-9)
{
  return std::fabs(a - b) < eps;
}

// Builds the synthetic scenario: 2 robots + 1 object, a non-default Params,
// and a TimeTable(0.5) populated via add_trajectory() with a couple of
// waypoints per robot so per_entity_table/trajectory_spans are both
// non-trivial (the transfer trajectory also exercises the pushed-object pose
// propagation, i.e. the object's poses come from add_trajectory(), not a
// second explicit call).
ExecutedScenario build_synthetic_scenario()
{
  ExecutedScenario scn;

  scn.params.xy_resolution = 0.25;
  scn.params.yaw_resolution = 0.3;
  scn.params.time_step = 1.5;
  scn.params.time_resolution = 1.5;
  scn.params.min_x = -2.0;
  scn.params.min_y = -3.0;
  scn.params.max_x = 12.0;
  scn.params.max_y = 9.0;
  scn.params.max_steer = 0.4;
  scn.params.turn_penalty = 2.2;
  scn.params.reverse_penalty = 33.0;
  scn.params.switch_penalty = 1.7;
  scn.params.wait_penalty = 5.0;
  scn.params.max_time = 555.0;
  scn.params.collision_steps = 7;
  scn.params.collision_check_time_step = 0.07;
  scn.params.analytic_threshold_scale = 3.3;
  scn.params.analytic_threshold = 4.4;
  scn.params.rs_step_size = 0.11;
  scn.params.inflation = 1.2;
  scn.params.safety_margin = 0.09;
  scn.params.robot_collision_inflation = 1.05;
  scn.params.movement_length = 2.5;
  scn.params.final_push_distance = 0.05;
  scn.params.robot_boundary_origin_only = false;
  scn.params.spatial_only_index = true;
  scn.params.disable_wait_primitive = true;
  scn.params.enable_holonomic_heuristic = true;
  scn.params.holonomic_heuristic_resolution = 0.22;

  scn.summary.label = "synthetic-test";
  scn.summary.all_tasks_succeeded = true;
  scn.summary.successful_tasks = 1;
  scn.summary.failed_tasks = 0;
  scn.summary.makespan = 12.5;

  RobotMeta *robot1 = new RobotMeta();
  robot1->name = "robot1";
  robot1->type = EntityType::ROBOT;
  robot1->initial_pose = Pose(0.0, 0.0, 0.0);
  robot1->size = OccuRect{0.3, 0.2, 0.25};
  robot1->min_turning_radius = 0.5;
  robot1->min_turning_radius_transit = 0.4;
  robot1->min_turning_radius_transfer = 0.6;
  robot1->wheel_base = 0.35;
  robot1->speed_transit = 0.8;
  robot1->speed_transfer = 0.5;
  scn.entities["robot1"] = robot1;

  RobotMeta *robot2 = new RobotMeta();
  robot2->name = "robot2";
  robot2->type = EntityType::ROBOT;
  robot2->initial_pose = Pose(5.0, 1.0, 1.57);
  robot2->size = OccuRect{0.35, 0.22, 0.28};
  robot2->min_turning_radius = 0.55;
  robot2->min_turning_radius_transit = 0.45;
  robot2->min_turning_radius_transfer = 0.65;
  robot2->wheel_base = 0.4;
  robot2->speed_transit = 0.9;
  robot2->speed_transfer = 0.6;
  scn.entities["robot2"] = robot2;

  ObjectMeta *obj1 = new ObjectMeta();
  obj1->name = "object1";
  obj1->type = EntityType::OBJECT;
  obj1->initial_pose = Pose(2.0, 2.0, 0.0);
  obj1->size = OccuRect{0.15, 0.15, 0.2};
  obj1->goal_pose = Pose(8.0, 3.0, 0.78);
  scn.entities["object1"] = obj1;

  // robot1: plain transit trajectory (no transfer).
  {
    Trajectory traj;
    traj.entity = robot1;
    traj.start_time = 0.0;
    traj.is_transfer = false;
    traj.kind = TrajectoryKind::TRANSIT;

    Pose p0(0.0, 0.0, 0.0);
    Waypoint w0(p0);
    w0.time = 0.0;
    Pose p1(1.0, 0.0, 0.0);
    Waypoint w1(p1);
    w1.time = 1.0;
    Pose p2(2.0, 1.0, 0.5);
    Waypoint w2(p2);
    w2.time = 2.0;
    traj.waypoints = {w0, w1, w2};

    scn.timetable.add_trajectory(traj);
  }

  // robot2: transfer trajectory pushing object1, so object1's poses in the
  // timetable come from the transfer-propagation logic inside
  // add_trajectory(), not a second explicit trajectory.
  {
    Trajectory traj;
    traj.entity = robot2;
    traj.transferred_object = obj1;
    traj.start_time = 3.0;
    traj.is_transfer = true;
    traj.kind = TrajectoryKind::TRANSFER;

    Pose p0(5.0, 1.0, 1.57);
    Waypoint w0(p0);
    w0.time = 0.0;
    Pose p1(5.0, 2.0, 1.57);
    Waypoint w1(p1);
    w1.time = 1.0;
    Pose p2(5.0, 3.0, 1.2);
    Waypoint w2(p2);
    w2.time = 2.0;
    traj.waypoints = {w0, w1, w2};

    scn.timetable.add_trajectory(traj);
  }

  return scn;
}

} // namespace

int main()
{
  ExecutedScenario original = build_synthetic_scenario();

  const std::string blob = serialize_executed_scenario_b64(original);
  ok(!blob.empty(), "serialized blob should be non-empty");

  ExecutedScenario loaded = deserialize_executed_scenario_b64(blob);

  // --- entities: count, names, types, sizes ---
  ok(loaded.entities.size() == original.entities.size(), "entity count mismatch");

  for (const auto &[name, orig_ent] : original.entities)
  {
    auto it = loaded.entities.find(name);
    if (it == loaded.entities.end())
    {
      ok(false, ("missing entity after round-trip: " + name).c_str());
      continue;
    }
    EntityMeta *loaded_ent = it->second;
    ok(loaded_ent->name == orig_ent->name, ("entity name mismatch for " + name).c_str());
    ok(loaded_ent->type == orig_ent->type, ("entity type mismatch for " + name).c_str());
    ok(nearly(loaded_ent->size.front_length, orig_ent->size.front_length),
       ("size.front_length mismatch for " + name).c_str());
    ok(nearly(loaded_ent->size.rear_length, orig_ent->size.rear_length),
       ("size.rear_length mismatch for " + name).c_str());
    ok(nearly(loaded_ent->size.width, orig_ent->size.width),
       ("size.width mismatch for " + name).c_str());
    ok(nearly(loaded_ent->initial_pose.x, orig_ent->initial_pose.x) &&
           nearly(loaded_ent->initial_pose.y, orig_ent->initial_pose.y) &&
           nearly(loaded_ent->initial_pose.yaw, orig_ent->initial_pose.yaw),
       ("initial_pose mismatch for " + name).c_str());

    if (orig_ent->type == EntityType::ROBOT)
    {
      auto *orig_r = static_cast<RobotMeta *>(orig_ent);
      auto *loaded_r = dynamic_cast<RobotMeta *>(loaded_ent);
      ok(loaded_r != nullptr, ("expected RobotMeta subtype for " + name).c_str());
      if (loaded_r)
      {
        ok(nearly(loaded_r->min_turning_radius, orig_r->min_turning_radius),
           ("min_turning_radius mismatch for " + name).c_str());
        ok(nearly(loaded_r->min_turning_radius_transit, orig_r->min_turning_radius_transit),
           ("min_turning_radius_transit mismatch for " + name).c_str());
        ok(nearly(loaded_r->min_turning_radius_transfer, orig_r->min_turning_radius_transfer),
           ("min_turning_radius_transfer mismatch for " + name).c_str());
        ok(nearly(loaded_r->wheel_base, orig_r->wheel_base),
           ("wheel_base mismatch for " + name).c_str());
        ok(nearly(loaded_r->speed_transit, orig_r->speed_transit),
           ("speed_transit mismatch for " + name).c_str());
        ok(nearly(loaded_r->speed_transfer, orig_r->speed_transfer),
           ("speed_transfer mismatch for " + name).c_str());
      }
    }
    else
    {
      auto *orig_o = static_cast<ObjectMeta *>(orig_ent);
      auto *loaded_o = dynamic_cast<ObjectMeta *>(loaded_ent);
      ok(loaded_o != nullptr, ("expected ObjectMeta subtype for " + name).c_str());
      if (loaded_o)
      {
        ok(nearly(loaded_o->goal_pose.x, orig_o->goal_pose.x) &&
               nearly(loaded_o->goal_pose.y, orig_o->goal_pose.y) &&
               nearly(loaded_o->goal_pose.yaw, orig_o->goal_pose.yaw),
           ("goal_pose mismatch for " + name).c_str());
      }
    }
  }

  // --- Params: every scalar field ---
  const Params &op = original.params;
  const Params &lp = loaded.params;
  ok(nearly(lp.xy_resolution, op.xy_resolution), "xy_resolution mismatch");
  ok(nearly(lp.yaw_resolution, op.yaw_resolution), "yaw_resolution mismatch");
  ok(nearly(lp.time_step, op.time_step), "time_step mismatch");
  ok(nearly(lp.time_resolution, op.time_resolution), "time_resolution mismatch");
  ok(nearly(lp.min_x, op.min_x), "min_x mismatch");
  ok(nearly(lp.min_y, op.min_y), "min_y mismatch");
  ok(nearly(lp.max_x, op.max_x), "max_x mismatch");
  ok(nearly(lp.max_y, op.max_y), "max_y mismatch");
  ok(nearly(lp.max_steer, op.max_steer), "max_steer mismatch");
  ok(nearly(lp.turn_penalty, op.turn_penalty), "turn_penalty mismatch");
  ok(nearly(lp.reverse_penalty, op.reverse_penalty), "reverse_penalty mismatch");
  ok(nearly(lp.switch_penalty, op.switch_penalty), "switch_penalty mismatch");
  ok(nearly(lp.wait_penalty, op.wait_penalty), "wait_penalty mismatch");
  ok(nearly(lp.max_time, op.max_time), "max_time mismatch");
  ok(lp.collision_steps == op.collision_steps, "collision_steps mismatch");
  ok(nearly(lp.collision_check_time_step, op.collision_check_time_step),
     "collision_check_time_step mismatch");
  ok(nearly(lp.analytic_threshold_scale, op.analytic_threshold_scale),
     "analytic_threshold_scale mismatch");
  ok(nearly(lp.analytic_threshold, op.analytic_threshold), "analytic_threshold mismatch");
  ok(nearly(lp.rs_step_size, op.rs_step_size), "rs_step_size mismatch");
  ok(nearly(lp.inflation, op.inflation), "inflation mismatch");
  ok(nearly(lp.safety_margin, op.safety_margin), "safety_margin mismatch");
  ok(nearly(lp.robot_collision_inflation, op.robot_collision_inflation),
     "robot_collision_inflation mismatch");
  ok(nearly(lp.movement_length, op.movement_length), "movement_length mismatch");
  ok(nearly(lp.final_push_distance, op.final_push_distance), "final_push_distance mismatch");
  ok(lp.robot_boundary_origin_only == op.robot_boundary_origin_only,
     "robot_boundary_origin_only mismatch");
  ok(lp.spatial_only_index == op.spatial_only_index, "spatial_only_index mismatch");
  ok(lp.disable_wait_primitive == op.disable_wait_primitive, "disable_wait_primitive mismatch");
  ok(lp.enable_holonomic_heuristic == op.enable_holonomic_heuristic,
     "enable_holonomic_heuristic mismatch");
  ok(nearly(lp.holonomic_heuristic_resolution, op.holonomic_heuristic_resolution),
     "holonomic_heuristic_resolution mismatch");

  // --- summary digest ---
  ok(loaded.summary.label == original.summary.label, "summary.label mismatch");
  ok(loaded.summary.all_tasks_succeeded == original.summary.all_tasks_succeeded,
     "summary.all_tasks_succeeded mismatch");
  ok(loaded.summary.successful_tasks == original.summary.successful_tasks,
     "summary.successful_tasks mismatch");
  ok(loaded.summary.failed_tasks == original.summary.failed_tasks,
     "summary.failed_tasks mismatch");
  ok(nearly(loaded.summary.makespan, original.summary.makespan), "summary.makespan mismatch");

  // --- TimeTable: time_increment + get_poses(t) at several timestamps ---
  ok(nearly(loaded.timetable.time_increment, original.timetable.time_increment),
     "time_increment mismatch");

  EntityMeta *orig_robot1 = original.entities.at("robot1");
  EntityMeta *orig_robot2 = original.entities.at("robot2");
  EntityMeta *orig_obj1 = original.entities.at("object1");
  EntityMeta *loaded_robot1 = loaded.entities.at("robot1");
  EntityMeta *loaded_robot2 = loaded.entities.at("robot2");
  EntityMeta *loaded_obj1 = loaded.entities.at("object1");

  const std::vector<double> sample_times = {0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 3.5, 4.0, 5.0};
  for (double t : sample_times)
  {
    Pose orig_r1 = original.timetable.get_pose(orig_robot1, t);
    Pose load_r1 = loaded.timetable.get_pose(loaded_robot1, t);
    ok(nearly(orig_r1.x, load_r1.x) && nearly(orig_r1.y, load_r1.y) &&
           nearly(orig_r1.yaw, load_r1.yaw),
       "robot1 pose mismatch at sample time");

    Pose orig_r2 = original.timetable.get_pose(orig_robot2, t);
    Pose load_r2 = loaded.timetable.get_pose(loaded_robot2, t);
    ok(nearly(orig_r2.x, load_r2.x) && nearly(orig_r2.y, load_r2.y) &&
           nearly(orig_r2.yaw, load_r2.yaw),
       "robot2 pose mismatch at sample time");

    Pose orig_o1 = original.timetable.get_pose(orig_obj1, t);
    Pose load_o1 = loaded.timetable.get_pose(loaded_obj1, t);
    ok(nearly(orig_o1.x, load_o1.x) && nearly(orig_o1.y, load_o1.y) &&
           nearly(orig_o1.yaw, load_o1.yaw),
       "object1 pose mismatch at sample time");
  }

  ok(nearly(loaded.timetable.get_max_time(), original.timetable.get_max_time()),
     "get_max_time() mismatch");

  ok(loaded.timetable.get_trajectory_spans().size() ==
         original.timetable.get_trajectory_spans().size(),
     "trajectory_spans count mismatch");

  if (g_failed)
  {
    std::fprintf(stderr, "SERIALIZATION_TEST_FAILED\n");
    return 1;
  }

  std::printf("SERIALIZATION_TEST_OK\n");
  return 0;
}
