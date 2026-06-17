#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>
#include <SearchOrchestrator.h>
#include <ReloPush/trajectory.hpp>
#include <ReloPush/base64.h>
#include <thread>
#include <vector>
#include <iostream>
#include <cassert>
#include <chrono>
#include <zmq.hpp>

bool DEBUG_VIS = false;

void run_dummy_controller(int port, const std::string &expected_robot_name)
{
  zmq::context_t ctx(1);
  zmq::socket_t socket(ctx, zmq::socket_type::rep);
  socket.bind("tcp://127.0.0.1:" + std::to_string(port));

  std::cout << "[Dummy Controller " << expected_robot_name << "] Listening on port " << port << "..." << std::endl;

  // 1. Await trajectory upload
  zmq::message_t req1;
  socket.recv(req1, zmq::recv_flags::none);
  std::string encoded_traj(static_cast<char*>(req1.data()), req1.size());
  std::cout << "[Dummy Controller " << expected_robot_name << "] Received encoded trajectory payload." << std::endl;

  // Decode and verify
  std::string decoded = base64_decode(encoded_traj);
  ReloPush::trajectory rp_traj(decoded);
  std::cout << "[Dummy Controller " << expected_robot_name << "] Decoded trajectory has "
            << rp_traj.trajectory_points->size() << " waypoints." << std::endl;
  assert(rp_traj.trajectory_points->size() > 0);

  // Send Ack
  std::string ack1 = "ACK_RECEIVE_" + expected_robot_name;
  zmq::message_t rep1(ack1.size());
  memcpy(rep1.data(), ack1.c_str(), ack1.size());
  socket.send(rep1, zmq::send_flags::none);

  // 2. Await synchronized START command
  zmq::message_t req2;
  socket.recv(req2, zmq::recv_flags::none);
  std::string start_cmd(static_cast<char*>(req2.data()), req2.size());
  std::cout << "[Dummy Controller " << expected_robot_name << "] Received command: " << start_cmd << std::endl;
  assert(start_cmd == "START");

  // Send Start Confirmation Ack
  std::string ack2 = "ACK_START_" + expected_robot_name;
  zmq::message_t rep2(ack2.size());
  memcpy(rep2.data(), ack2.c_str(), ack2.size());
  socket.send(rep2, zmq::send_flags::none);

  std::cout << "[Dummy Controller " << expected_robot_name << "] Success!" << std::endl;
}

int main()
{
  std::cout << "[Test] Spawning mock robot controllers..." << std::endl;

  // Ports start at 11110. robot1 -> 11111, robot2 -> 11112.
  std::thread t1(run_dummy_controller, 11111, "robot1");
  std::thread t2(run_dummy_controller, 11112, "robot2");

  // Give threads a short time to bind
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "[Test] Constructing mock timetable..." << std::endl;
  TimeTable timetable(0.5);

  // Create mock robot metas
  RobotMeta robot1;
  robot1.name = "robot1";
  robot1.type = EntityType::ROBOT;
  robot1.speed_transit = 0.2;
  robot1.speed_transfer = 0.15;
  robot1.initial_pose = Pose(0.0, 0.0, 0.0);

  RobotMeta robot2;
  robot2.name = "robot2";
  robot2.type = EntityType::ROBOT;
  robot2.speed_transit = 0.25;
  robot2.speed_transfer = 0.18;
  robot2.initial_pose = Pose(1.0, 1.0, 0.0);

  std::unordered_map<std::string, EntityMeta *> entities = {
      {"robot1", &robot1},
      {"robot2", &robot2}};
  timetable.add_initial(entities);

  // Add trajectories
  Pose p1_1(0.0, 0.0, 0.0);
  Pose p1_2(1.0, 0.0, 0.0);
  Pose p1_3(2.0, 0.0, 0.0);
  WaypointPath path1 = {
      Waypoint(p1_1),
      Waypoint(p1_2),
      Waypoint(p1_3)};
  Trajectory traj1(&robot1, nullptr, 0.0, path1, false);
  traj1.CalcualteTimeStamps(&robot1, 0.0);
  timetable.add_trajectory(traj1);

  Pose p2_1(1.0, 1.0, 0.0);
  Pose p2_2(2.0, 1.0, 0.0);
  Pose p2_3(3.0, 1.0, 0.0);
  WaypointPath path2 = {
      Waypoint(p2_1),
      Waypoint(p2_2),
      Waypoint(p2_3)};
  Trajectory traj2(&robot2, nullptr, 0.0, path2, true); // transfer trajectory!
  traj2.CalcualteTimeStamps(&robot2, 0.0);
  timetable.add_trajectory(traj2);

  RuntimeOptions options;
  options.run_on_robots = true;
  options.robot_controller_port_start = 11110;
  options.spawn_mpc = false;

  std::cout << "[Test] Executing run_on_robots_pipeline..." << std::endl;
  run_on_robots_pipeline(options, timetable);

  std::cout << "[Test] Joining controller threads..." << std::endl;
  t1.join();
  t2.join();

  std::cout << "[Test] Integration test completed successfully." << std::endl;
  return 0;
}
