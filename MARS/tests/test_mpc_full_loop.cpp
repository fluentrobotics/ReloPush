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
#include <atomic>
#include <zmq.hpp>

// Atomic flag to coordinate shutdown of threads
std::atomic<bool> g_test_running(true);

// Required global variable for debug visualization configuration
bool DEBUG_VIS = false;

// Mock localization publisher: binds to the SUB socket endpoint for MPC controller and periodically publishes JSON pose
void run_mock_localization(int port, const std::string &robot_name)
{
  zmq::context_t ctx(1);
  zmq::socket_t pub_socket(ctx, zmq::socket_type::pub);
  pub_socket.set(zmq::sockopt::linger, 0);
  
  // Bind to the port the subscriber will connect to
  std::string addr = "tcp://127.0.0.1:" + std::to_string(port);
  pub_socket.bind(addr);
  
  std::cout << "[Mock Localization " << robot_name << "] Bound to " << addr << " (publishing at 20Hz)..." << std::endl;

  std::string topic = "/" + robot_name + "/localization";
  
  while (g_test_running)
  {
    std::string payload = "{\"x\": 0.5, \"y\": 0.2, \"yaw\": 0.1}";
    
    zmq::message_t topic_msg(topic.begin(), topic.end());
    zmq::message_t payload_msg(payload.begin(), payload.end());
    
    pub_socket.send(topic_msg, zmq::send_flags::sndmore);
    pub_socket.send(payload_msg, zmq::send_flags::none);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  
  std::cout << "[Mock Localization " << robot_name << "] Stopped." << std::endl;
}

// Mock VESC receiver: binds to the PUB socket endpoint for MPC controller and receives published Ackermann drive commands
void run_mock_vesc(int port, const std::string &robot_name)
{
  zmq::context_t ctx(1);
  zmq::socket_t sub_socket(ctx, zmq::socket_type::sub);
  sub_socket.set(zmq::sockopt::linger, 0);
  
  std::string addr = "tcp://127.0.0.1:" + std::to_string(port);
  sub_socket.bind(addr);
  
  std::string topic = "/" + robot_name + "/ackermann";
  sub_socket.set(zmq::sockopt::subscribe, topic);
  
  std::cout << "[Mock VESC " << robot_name << "] Bound to " << addr << " (subscribed to " << topic << ")..." << std::endl;

  int message_count = 0;
  zmq::message_t msg;
  
  while (g_test_running)
  {
    // Receive message with a timeout to avoid hanging indefinitely if there's an issue
    if (sub_socket.recv(msg, zmq::recv_flags::dontwait))
    {
      std::string part(static_cast<char*>(msg.data()), msg.size());
      if (msg.more())
      {
        sub_socket.recv(msg, zmq::recv_flags::none);
        part = std::string(static_cast<char*>(msg.data()), msg.size());
      }
      
      message_count++;
      if (message_count <= 5)
      {
        std::cout << "[Mock VESC " << robot_name << "] Received drive payload " << message_count << ": " << part << std::endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  std::cout << "[Mock VESC " << robot_name << "] Stopped. Total received messages: " << message_count << std::endl;
  assert(message_count > 0 && "VESC should have received commands from MPC controller!");
}

int main()
{
  std::cout << "[Test] Starting C++ MPC closed-loop integration test..." << std::endl;

  // robot1 sequential ports:
  // Handshake: 11111
  // VESC PUB: 3161
  // Loc SUB: 3261
  
  // robot2 sequential ports:
  // Handshake: 11112
  // VESC PUB: 3162
  // Loc SUB: 3262

  std::thread loc1(run_mock_localization, 3261, "robot1");
  std::thread vesc1(run_mock_vesc, 3161, "robot1");
  
  std::thread loc2(run_mock_localization, 3262, "robot2");
  std::thread vesc2(run_mock_vesc, 3162, "robot2");

  // Give mock threads a short time to bind
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "[Test] Constructing TimeTable for robot1 and robot2..." << std::endl;
  TimeTable timetable(0.5);

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

  Pose p1_1(0.0, 0.0, 0.0);
  Pose p1_2(0.1, 0.0, 0.0);
  Pose p1_3(0.2, 0.0, 0.0);
  WaypointPath path1 = {
      Waypoint(p1_1),
      Waypoint(p1_2),
      Waypoint(p1_3)};
  Trajectory traj1(&robot1, nullptr, 0.0, path1, false);
  traj1.CalcualteTimeStamps(&robot1, 0.0);
  timetable.add_trajectory(traj1);

  Pose p2_1(1.0, 1.0, 0.0);
  Pose p2_2(1.1, 1.0, 0.0);
  Pose p2_3(1.2, 1.0, 0.0);
  WaypointPath path2 = {
      Waypoint(p2_1),
      Waypoint(p2_2),
      Waypoint(p2_3)};
  Trajectory traj2(&robot2, nullptr, 0.0, path2, true);
  traj2.CalcualteTimeStamps(&robot2, 0.0);
  timetable.add_trajectory(traj2);

  RuntimeOptions options;
  options.run_on_robots = true;
  options.robot_controller_port_start = 11110;
  options.mpc_vesc_port_start = 3160;
  options.mpc_localization_port_start = 3260;
  options.mpc_vesc_ip = "127.0.0.1";
  options.mpc_localization_ip = "127.0.0.1";

  std::cout << "[Test] Executing robot execution pipeline..." << std::endl;
  run_on_robots_pipeline(options, timetable);

  std::cout << "[Test] Shuting down mock threads..." << std::endl;
  g_test_running = false;

  loc1.join();
  vesc1.join();
  loc2.join();
  vesc2.join();

  std::cout << "[Test] C++ MPC integration test completed successfully." << std::endl;
  return 0;
}
