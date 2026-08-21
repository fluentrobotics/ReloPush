// RIGHT-SIDE MONITOR PANEL (widget layer): offscreen smoke test for
// simviz::SimVizWindow's new per-robot monitor cells + "Motor stall" UI
// controls (MARS/src/simviz/SimVizWindow.h/.cpp). Run under
// QT_QPA_PLATFORM=offscreen, same as test_sim_viz_window_render.cpp, whose
// style/technique this file mirrors closely: a REAL simviz::SimVizWindow
// (never a bare SimVizCanvas) driving a REAL ExecutionManager against fake
// sleeping-shell child binaries (their real behavior doesn't matter, only
// that "some child process is alive" long enough for start_execution() to
// succeed -- same technique test_sim_viz_window_render.cpp's test (d)/(e)
// use), plus a hand-rolled ZMQ PUB socket standing in for what the real
// mpc_robot_sim would publish (BOTH the "/<robot>/localization" and
// "/<robot>/telemetry" topics, on the same bound socket) so live telemetry
// reaches the window's monitor panel without needing the real MPC binaries.
//
// Deliberately drives the window via canvas()->set_scenario() +
// manager().execution_manager().start_execution() directly (mirroring test
// (d)/(e) in test_sim_viz_window_render.cpp) rather than
// open_scenario_file(), so a hypothetical failure path can never pop a
// blocking QMessageBox under offscreen/headless CI.
//
// Parts:
//   (a) MOTOR STALL controls reflect SimVizConfig at construction time
//       (default unchecked/0.10/disabled-spinbox, and a CLI-equivalent
//       enabled/0.18 config) -- mirrors test_window_noise_slider_reflects_
//       initial_config() in test_sim_viz_window_render.cpp.
//   (b) monitor cells: none before a scenario loads; one per robot after
//       (both showing "—" placeholders before any telemetry arrives);
//       values populate from fed telemetry via refresh_display() (the
//       same method the ~33ms repaint timer calls); a robot with no
//       telemetry fed keeps showing "—" independently.
//   (c) state tag priority order: STALLED (moving=0, |v_cmd|>0.02) takes
//       priority over WATCHDOG, which takes priority over the plain
//       MOVING/IDLE reading of the moving flag.

#include "SimVizCore.h"
#include "SimVizWindow.h"

#include <ExecutedScenarioSerialization.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <QApplication>
#include <QChar>
#include <QString>

#include <zmq.hpp>

#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

bool DEBUG_VIS = false;

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

constexpr int kTestControlPort = 45699; // unused (start() never called) -- TEST PORT BLOCK-adjacent
constexpr int kTestHandshakePortStart = 45620;
constexpr int kTestVescPortStart = 45640;
constexpr int kTestLocPortStart = 45660;

// Built from numeric Unicode codepoints (not typed as literal characters in
// THIS file) so the expected-value comparisons below never depend on this
// source file's own encoding matching SimVizWindow.cpp's -- only on
// SimVizWindow.cpp's characters round-tripping correctly through
// QLabel::text().toStdString() (which is UTF-8, per QString::toStdString()'s
// documented behavior), independent of exactly how either .cpp file happens
// to be saved.
std::string em_dash()
{
  return QString(QChar(0x2014)).toStdString();
} // "—"
std::string delta()
{
  return QString(QChar(0x03B4)).toStdString();
} // "δ"
std::string sup_two()
{
  return QString(QChar(0x00B2)).toStdString();
} // "²"

std::string write_fake_child_binary(const std::filesystem::path &dir)
{
  const std::filesystem::path path = dir / "fake_child_proc.sh";
  {
    std::ofstream out(path);
    out << "#!/bin/sh\nexec sleep 300\n";
  }
  chmod(path.c_str(), 0755);
  return path.string();
}

std::string build_and_write_two_robot_scenario(const std::filesystem::path &dir,
                                                 const std::string &label)
{
  ExecutedScenario scn;
  scn.summary.label = label;
  scn.summary.all_tasks_succeeded = true;
  scn.params.min_x = 0.0;
  scn.params.min_y = 0.0;
  scn.params.max_x = 6.0;
  scn.params.max_y = 6.0;

  RobotMeta *robot1 = new RobotMeta();
  robot1->name = "robot1";
  robot1->type = EntityType::ROBOT;
  robot1->initial_pose = Pose(0.0, 0.0, 0.0);
  robot1->size = OccuRect{0.3, 0.2, 0.3};
  robot1->speed_transit = 0.2;
  robot1->speed_transfer = 0.15;

  RobotMeta *robot2 = new RobotMeta();
  robot2->name = "robot2";
  robot2->type = EntityType::ROBOT;
  robot2->initial_pose = Pose(0.0, 2.0, 0.0);
  robot2->size = OccuRect{0.3, 0.2, 0.3};
  robot2->speed_transit = 0.2;
  robot2->speed_transfer = 0.15;

  scn.entities["robot1"] = robot1;
  scn.entities["robot2"] = robot2;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  const std::string b64 = serialize_executed_scenario_b64(scn);
  const std::filesystem::path out_path = dir / (label + ".scn.b64");
  std::ofstream out(out_path, std::ios::binary);
  out << b64;
  return out_path.string();
}

// Publishes BOTH the localization AND telemetry topics on one bound PUB
// socket -- see test_sim_viz_telemetry.cpp's identically-named helper (this
// file's own copy: kept file-local rather than shared, matching this
// project's existing per-test-binary duplication convention for small
// harness helpers, e.g. each test_sim_viz_*.cpp's own req()/spawn_wrapped
// variants).
void publish_fake_localization_and_telemetry(int port, const std::string &robot_name, double t,
                                              double v, double v_cmd, double steering,
                                              double accel, bool moving, bool watchdog,
                                              std::atomic<bool> &stop)
{
  zmq::context_t ctx(1);
  zmq::socket_t pub(ctx, zmq::socket_type::pub);
  pub.set(zmq::sockopt::linger, 0);
  pub.bind("tcp://*:" + std::to_string(port));

  const std::string loc_topic = "/" + robot_name + "/localization";
  const std::string loc_payload_str = "{\"x\":0,\"y\":0,\"yaw\":0}";

  const std::string telem_topic = "/" + robot_name + "/telemetry";
  std::ostringstream telem_payload;
  telem_payload << "{\"t\":" << t << ",\"v\":" << v << ",\"v_cmd\":" << v_cmd
                << ",\"steering\":" << steering << ",\"accel\":" << accel
                << ",\"moving\":" << (moving ? 1 : 0) << ",\"watchdog\":" << (watchdog ? 1 : 0)
                << "}";
  const std::string telem_payload_str = telem_payload.str();

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

void test_stall_controls_default_and_cli_config()
{
  std::cout << "[Test] (a) MOTOR STALL controls reflect initial config..." << std::endl;

  simviz::SimVizConfig default_config;
  default_config.control_port = kTestControlPort; // unused (start() not called)
  simviz::SimVizWindow default_window(default_config);
  // GUI-MODE STARTUP DEFAULTS: a config with stall_enabled left at its own
  // `false` field default now reads back CHECKED once routed through
  // SimVizWindow's apply_gui_startup_defaults() (SimVizWindow.cpp) -- the
  // headless/CLI-flag default itself (SimVizConfig::stall_enabled, i.e.
  // --deadband) is unchanged at false/absent, only what a freshly-opened
  // GUI window's checkbox shows. stall_level's 0.10 default is unchanged
  // (it was already the desired 0.1 m/s value).
  TEST_ASSERT(default_window.stall_enabled_checked() == true);
  TEST_ASSERT(std::abs(default_window.stall_level_value() - 0.10) < 1e-9);
  TEST_ASSERT(default_window.stall_level_enabled() == true);
  std::cout << "[Test]   default config: checked (GUI startup default), 0.10, spinbox enabled "
               "-- OK"
            << std::endl;

  simviz::SimVizConfig stall_config;
  stall_config.control_port = kTestControlPort + 1; // unused
  stall_config.stall_enabled = true;
  stall_config.stall_level = 0.18;
  simviz::SimVizWindow stall_window(stall_config);
  TEST_ASSERT(stall_window.stall_enabled_checked() == true);
  TEST_ASSERT(std::abs(stall_window.stall_level_value() - 0.18) < 1e-9);
  TEST_ASSERT(stall_window.stall_level_enabled() == true);
  std::cout << "[Test]   stall_enabled=true/stall_level=0.18 config: checked, 0.18, spinbox "
               "enabled -- OK"
            << std::endl;

  std::cout << "[Test] (a) PASSED" << std::endl;
}

void test_monitor_panel_cells_and_telemetry_display()
{
  std::cout << "[Test] (b)+(c) monitor cells + telemetry display + state tag priority..."
            << std::endl;

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_monitor_panel_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::string fake_bin = write_fake_child_binary(tmp_dir);
  const std::string scn_path = build_and_write_two_robot_scenario(tmp_dir, "monitor_panel_test");

  auto model = std::make_shared<simviz::ScenarioModel>(
      simviz::ScenarioModel::load_from_file(scn_path));
  TEST_ASSERT(model->is_loaded());

  simviz::SimVizConfig config;
  config.control_port = kTestControlPort; // unused (start() not called)
  config.handshake_port_start = kTestHandshakePortStart;
  config.vesc_port_start = kTestVescPortStart;
  config.loc_port_start = kTestLocPortStart;
  config.mpc_controller_path = fake_bin;
  config.robot_sim_path = fake_bin;
  config.run_dir = (tmp_dir / "runs").string();

  simviz::SimVizWindow window(config);

  // Before any scenario is loaded: no cells at all.
  TEST_ASSERT(!window.monitor_cell_text("robot1").has_value());
  TEST_ASSERT(!window.monitor_cell_text("robot2").has_value());
  std::cout << "[Test]   no monitor cells before a scenario loads -- OK" << std::endl;

  // Drive the window the same way open_scenario_file() would (canvas +
  // ExecutionManager::start_execution()), without going through
  // open_scenario_file() itself so a hypothetical failure path can never
  // pop a blocking QMessageBox here (mirrors test_sim_viz_window_render.
  // cpp's test (d)/(e), which use a standalone ExecutionManager for the
  // same reason).
  window.canvas()->set_scenario(model);
  const bool accepted = window.manager().execution_manager().start_execution(model);
  TEST_ASSERT(accepted);

  // refresh_display() is the same method the ~33ms repaint timer calls --
  // exercising it directly here is exactly what this task's testability
  // doc comment on refresh_display() describes.
  window.refresh_display();

  auto robot1_cell = window.monitor_cell_text("robot1");
  auto robot2_cell = window.monitor_cell_text("robot2");
  TEST_ASSERT(robot1_cell.has_value());
  TEST_ASSERT(robot2_cell.has_value());
  std::cout << "[Test]   one monitor cell per robot exists once the scenario loaded -- OK"
            << std::endl;

  TEST_ASSERT(robot1_cell->velocity == "v: " + em_dash());
  TEST_ASSERT(robot1_cell->steering == delta() + ": " + em_dash());
  TEST_ASSERT(robot1_cell->accel == "a: " + em_dash());
  TEST_ASSERT(robot1_cell->state_tag == em_dash());
  std::cout << "[Test]   \"" << em_dash() << "\" placeholders shown before any telemetry arrives "
                                              "-- OK"
            << std::endl;

  // Feed live telemetry for robot1 ONLY (index 0 -> kTestLocPortStart).
  // moving=true, watchdog=false, v_cmd irrelevant to the tag in this
  // branch -- expect a plain "MOVING" tag and formatted v/steering/accel.
  std::atomic<bool> stop_pub{false};
  std::thread pub(publish_fake_localization_and_telemetry, kTestLocPortStart, "robot1",
                   /*t=*/3.5, /*v=*/1.25, /*v_cmd=*/1.30, /*steering=*/0.11, /*accel=*/0.42,
                   /*moving=*/true, /*watchdog=*/false, std::ref(stop_pub));

  bool got_telem = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline)
  {
    auto live = window.manager().execution_manager().localization().latest_telemetry("robot1");
    if (live && live->has_telemetry)
    {
      got_telem = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  TEST_ASSERT(got_telem && "fed telemetry sample never reached LocalizationListener");

  window.refresh_display();
  robot1_cell = window.monitor_cell_text("robot1");
  robot2_cell = window.monitor_cell_text("robot2");
  TEST_ASSERT(robot1_cell.has_value() && robot2_cell.has_value());

  TEST_ASSERT(robot1_cell->velocity == "v: 1.25 m/s");
  TEST_ASSERT(robot1_cell->steering == delta() + ": 0.11 rad");
  TEST_ASSERT(robot1_cell->accel == "a: 0.42 m/s" + sup_two());
  TEST_ASSERT(robot1_cell->state_tag == "MOVING");
  std::cout << "[Test]   robot1 cell shows fed telemetry values + MOVING tag -- OK" << std::endl;

  // LAG READOUT (three-clock lag-diagnosis task): renders once a live POSE
  // has arrived -- independent of telemetry (a separate PUB topic), which
  // is why this is checked here rather than gated on the telemetry-driven
  // block above. publish_fake_localization_and_telemetry() feeds (0,0,0),
  // exactly robot1's only recorded TimeTable sample (this scenario has no
  // added trajectory, just add_initial()) -- a perfect on-schedule match
  // at any plan time, so lag reads "lag: 0.00s".
  TEST_ASSERT(robot1_cell->lag == "lag: 0.00s");
  std::cout << "[Test]   robot1 cell's LAG READOUT renders once a live pose arrives -- OK"
            << std::endl;

  // robot2 got no telemetry at all -- must still show placeholders,
  // independent of robot1's now-populated cell.
  TEST_ASSERT(robot2_cell->velocity == "v: " + em_dash());
  TEST_ASSERT(robot2_cell->state_tag == em_dash());
  // robot2 got no localization pose either (same fake-PUB call only
  // published robot1's topic) -- lag stays "—", independent of robot1.
  TEST_ASSERT(robot2_cell->lag == "lag: " + em_dash());
  std::cout << "[Test]   robot2 cell (no telemetry/pose fed) still shows \"" << em_dash()
            << "\" placeholders, including lag, independent of robot1 -- OK" << std::endl;

  stop_pub.store(true);
  pub.join();

  // ---------------------------------------------------------------------
  // State tag priority order: STALLED > WATCHDOG > MOVING/IDLE. Re-publish
  // robot1's telemetry with each combination in turn and re-check the tag.
  // ---------------------------------------------------------------------
  auto feed_and_check_tag = [&](double v_cmd, bool moving, bool watchdog,
                                 const std::string &expect_tag, const std::string &what)
  {
    std::atomic<bool> stop{false};
    std::thread p(publish_fake_localization_and_telemetry, kTestLocPortStart, "robot1",
                   /*t=*/1.0, /*v=*/0.0, v_cmd, /*steering=*/0.0, /*accel=*/0.0, moving, watchdog,
                   std::ref(stop));

    // Poll until the specific sample we just fed (identified by its moving/
    // watchdog combination, which is unique to each call below) is visible.
    bool seen = false;
    const auto dl = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < dl)
    {
      auto live = window.manager().execution_manager().localization().latest_telemetry("robot1");
      if (live && live->has_telemetry && live->moving == moving && live->watchdog == watchdog)
      {
        seen = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    TEST_ASSERT(seen && "fed telemetry sample for state-tag check never arrived");

    window.refresh_display();
    auto cell = window.monitor_cell_text("robot1");
    TEST_ASSERT(cell.has_value());
    if (cell->state_tag != expect_tag)
    {
      std::cerr << "[test] FAILED: state tag mismatch for " << what << ": expected '"
                << expect_tag << "', got '" << cell->state_tag << "'" << std::endl;
      std::abort();
    }
    std::cout << "[Test]   " << what << " -> state_tag='" << cell->state_tag << "' -- OK"
              << std::endl;

    stop.store(true);
    p.join();
  };

  // STALLED: moving=0, |v_cmd|=0.5 > 0.02.
  feed_and_check_tag(/*v_cmd=*/0.5, /*moving=*/false, /*watchdog=*/false, "STALLED",
                      "moving=0, v_cmd=0.5, watchdog=0");
  // STALLED still wins even with watchdog also engaged (priority order).
  feed_and_check_tag(/*v_cmd=*/0.5, /*moving=*/false, /*watchdog=*/true, "STALLED",
                      "moving=0, v_cmd=0.5, watchdog=1 (STALLED beats WATCHDOG)");
  // WATCHDOG: moving=1 (so the STALLED condition is false regardless of
  // v_cmd), watchdog=1.
  feed_and_check_tag(/*v_cmd=*/0.5, /*moving=*/true, /*watchdog=*/true, "WATCHDOG",
                      "moving=1, v_cmd=0.5, watchdog=1");
  // MOVING: moving=1, watchdog=0.
  feed_and_check_tag(/*v_cmd=*/0.5, /*moving=*/true, /*watchdog=*/false, "MOVING",
                      "moving=1, v_cmd=0.5, watchdog=0");
  // IDLE: moving=0, |v_cmd|=0.0 <= 0.02, watchdog=0.
  feed_and_check_tag(/*v_cmd=*/0.0, /*moving=*/false, /*watchdog=*/false, "IDLE",
                      "moving=0, v_cmd=0.0, watchdog=0");

  window.manager().execution_manager().abort();
  std::filesystem::remove_all(tmp_dir);

  std::cout << "[Test] (b)+(c) PASSED" << std::endl;
}

} // namespace

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  test_stall_controls_default_and_cli_config();
  test_monitor_panel_cells_and_telemetry_display();

  std::cout << "\n[Test] All test_sim_viz_monitor_panel tests passed." << std::endl;
  return 0;
}
