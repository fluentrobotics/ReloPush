// Phase B2 offscreen render smoke test for simviz::SimVizCanvas
// (MARS/src/simviz/SimVizWindow.h). Run under
// QT_QPA_PLATFORM=offscreen (the test wrapper this task requires for every
// binary run).
//
// Deliberately does NOT go through ExecutionManager::start_execution() (no
// real mpc_controller/mpc_robot_sim spawn, no ZMQ handshake) -- it exercises
// exactly the rendering path (SimVizCanvas::set_scenario() +
// SimVizCanvas::paintEvent(), via QWidget::grab()) with a REAL .scn.b64
// fixture's entities/goals/timetable, which is what "load a scenario, grab
// the widget to a QImage, assert non-trivial content" (this task's design
// doc) asks for. The full spawn-and-render pipeline (real mpc_controller +
// mpc_robot_sim children driving live localization into the SAME canvas) is
// covered by test_sim_viz_integration (headless, no pixels) plus manual use
// of the real `mars_sim_viz` GUI -- actually spawning binaries just to
// rasterize a few rectangles would make this "smoke test" as slow/fragile
// as an integration test for no rendering-coverage benefit.
//
// Also exercises: SimVizWindow construction (menu/toolbar/canvas/status bar
// wiring) without start() (so no control-socket bind, no port needed), and
// SimVizCanvas rendering a hand-built scenario with an object mid-transfer
// (piecewise-constant replay) so the object-fill + goal-dashed-outline
// drawing paths both execute.

#include "SimVizCore.h"
#include "SimVizWindow.h"

#include <ExecutedScenarioSerialization.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <QApplication>
#include <QImage>
#include <QRect>
#include <QSet>

#include <zmq.hpp>

#include <sys/stat.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

bool DEBUG_VIS = false;

#ifndef MARS_TEST_FIXTURE_SCN
#define MARS_TEST_FIXTURE_SCN ""
#endif
#ifndef MARS_TEST_RENDER_OUT_DIR
#define MARS_TEST_RENDER_OUT_DIR "."
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

int count_distinct_colors(const QImage &img)
{
  QSet<QRgb> colors;
  for (int y = 0; y < img.height(); ++y)
  {
    for (int x = 0; x < img.width(); ++x)
    {
      colors.insert(img.pixel(x, y));
      if (colors.size() > 64) // early-out; we only need "more than a couple"
        return colors.size();
    }
  }
  return colors.size();
}

// ISSUE 2: does `img` contain any pixel close to `target` (within a small
// per-channel tolerance, to absorb antialiasing on the dashed outline's
// core pixels)? Used to assert the #E76F51 goal-outline color is absent by
// default and present once SimVizCanvas::set_show_goal_outlines(true) is
// called.
bool contains_color(const QImage &img, const QColor &target, int tol = 12)
{
  const int tr = target.red(), tg = target.green(), tb = target.blue();
  for (int y = 0; y < img.height(); ++y)
  {
    for (int x = 0; x < img.width(); ++x)
    {
      const QRgb px = img.pixel(x, y);
      if (std::abs(qRed(px) - tr) <= tol && std::abs(qGreen(px) - tg) <= tol &&
          std::abs(qBlue(px) - tb) <= tol)
        return true;
    }
  }
  return false;
}

const QColor kGoalOutlineColorForTest("#E76F51"); // mirrors SimVizWindow.cpp's kGoalOutlineColor

QImage grab_canvas(simviz::SimVizCanvas &canvas)
{
  canvas.resize(900, 700);
  canvas.show();
  // Force a synchronous paint (grab() does this internally too, but an
  // explicit repaint() first makes intent clear and matches how a real
  // event-loop-driven repaint would look right after set_scenario()).
  canvas.repaint();
  return canvas.grab().toImage();
}

void test_real_fixture_renders()
{
  std::cout << "[Test] (a) load real .scn.b64 fixture into SimVizCanvas..." << std::endl;
  const std::string fixture_path = MARS_TEST_FIXTURE_SCN;
  TEST_ASSERT(!fixture_path.empty() && "MARS_TEST_FIXTURE_SCN compile definition missing");
  TEST_ASSERT(std::filesystem::exists(fixture_path) &&
         "fixture file not found -- results/ layout changed?");

  auto model =
      std::make_shared<simviz::ScenarioModel>(simviz::ScenarioModel::load_from_file(fixture_path));
  TEST_ASSERT(model->is_loaded());
  TEST_ASSERT(!model->robots_sorted_by_name().empty());
  TEST_ASSERT(!model->objects_sorted_by_name().empty());

  simviz::SimVizCanvas canvas;
  canvas.set_scenario(model);
  TEST_ASSERT(!canvas.show_goal_outlines()); // ISSUE 2: default is hidden

  const QImage img = grab_canvas(canvas);
  TEST_ASSERT(img.width() > 0 && img.height() > 0);

  const int distinct = count_distinct_colors(img);
  std::cout << "[Test]   rendered " << img.width() << "x" << img.height() << ", "
            << distinct << "+ distinct colors" << std::endl;
  TEST_ASSERT(distinct > 2);

  // ISSUE 2: goal outlines hidden by default -- the dashed #E76F51 boxes
  // must not appear in the default render.
  TEST_ASSERT(!contains_color(img, kGoalOutlineColorForTest));

  const std::string out_path =
      std::string(MARS_TEST_RENDER_OUT_DIR) + "/test_sim_viz_window_render_fixture.png";
  const bool saved = img.save(QString::fromStdString(out_path));
  std::cout << "[Test]   saved " << (saved ? "" : "FAILED ") << out_path << std::endl;
  TEST_ASSERT(saved);

  std::cout << "[Test] (a) PASSED" << std::endl;
}

// Hand-built scenario with a robot mid-TRANSFER of an object, so the
// object-fill + dashed-goal-outline paths both draw something visibly
// different from the fixture-only run above.
void test_synthetic_transfer_scenario_renders()
{
  std::cout << "[Test] (b) synthetic transfer scenario renders object + goal..." << std::endl;

  ExecutedScenario scn;
  scn.summary.label = "render_smoke_transfer_test";
  scn.summary.all_tasks_succeeded = true;
  scn.params.min_x = 0.0;
  scn.params.min_y = 0.0;
  scn.params.max_x = 6.0;
  scn.params.max_y = 6.0;

  RobotMeta *robot = new RobotMeta();
  robot->name = "robot1";
  robot->type = EntityType::ROBOT;
  robot->initial_pose = Pose(1.0, 1.0, 0.0);
  robot->size = OccuRect{0.3, 0.2, 0.3};
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;

  ObjectMeta *object = new ObjectMeta();
  object->name = "object1";
  object->type = EntityType::OBJECT;
  object->initial_pose = Pose(2.0, 2.0, 0.0);
  object->goal_pose = Pose(5.0, 5.0, 0.0);
  object->size = OccuRect{0.2, 0.2, 0.2};

  scn.entities["robot1"] = robot;
  scn.entities["object1"] = object;

  std::unordered_map<EntityMeta *, std::map<double, Pose>> per_entity;
  per_entity[robot] = {
      {0.0, Pose(1.0, 1.0, 0.0)},
      {10.0, Pose(3.0, 3.0, 0.5)},
  };
  per_entity[object] = {
      {0.0, Pose(2.0, 2.0, 0.0)},
      {10.0, Pose(3.3, 3.3, 0.5)},
  };
  std::vector<TimeTable::TrajectorySpan> spans;
  TimeTable::TrajectorySpan span;
  span.entity = robot;
  span.transferred_object = object;
  span.start_time = 0.0;
  span.end_time = 10.0;
  span.is_transfer = true;
  spans.push_back(span);

  scn.timetable = TimeTable(0.5);
  scn.timetable.load_serialized_state(0.5, per_entity, spans);

  const std::string b64 = serialize_executed_scenario_b64(scn);
  const std::filesystem::path tmp_path =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_window_render_test_" + std::to_string(getpid()) + ".scn.b64");
  {
    std::ofstream out(tmp_path, std::ios::binary);
    out << b64;
  }

  auto model = std::make_shared<simviz::ScenarioModel>(
      simviz::ScenarioModel::load_from_file(tmp_path.string()));
  std::filesystem::remove(tmp_path);
  TEST_ASSERT(model->is_loaded());

  simviz::SimVizCanvas canvas;
  canvas.set_scenario(model);

  const QImage img = grab_canvas(canvas);
  const int distinct = count_distinct_colors(img);
  std::cout << "[Test]   rendered " << img.width() << "x" << img.height() << ", "
            << distinct << "+ distinct colors" << std::endl;
  TEST_ASSERT(distinct > 2);

  // ISSUE 2: default render (flag untouched, i.e. false) must NOT contain
  // the goal-outline color even though this scenario's object has a
  // goal_pose distinct from its initial_pose (so the outline WOULD be
  // clearly visible if drawn).
  TEST_ASSERT(!canvas.show_goal_outlines());
  TEST_ASSERT(!contains_color(img, kGoalOutlineColorForTest));

  // Programmatically enable the toggle -- the outline must now appear.
  canvas.set_show_goal_outlines(true);
  TEST_ASSERT(canvas.show_goal_outlines());
  const QImage img_with_goals = grab_canvas(canvas);
  TEST_ASSERT(contains_color(img_with_goals, kGoalOutlineColorForTest));
  std::cout << "[Test]   goal outline color present after set_show_goal_outlines(true)"
            << std::endl;

  // And disabling it again removes it -- the flag is a live toggle, not a
  // one-shot.
  canvas.set_show_goal_outlines(false);
  const QImage img_hidden_again = grab_canvas(canvas);
  TEST_ASSERT(!contains_color(img_hidden_again, kGoalOutlineColorForTest));

  std::cout << "[Test] (b) PASSED" << std::endl;
}

// SimVizWindow itself constructs cleanly (menu/toolbar/status bar/canvas
// wiring) and exposes a usable canvas -- without calling start() (so no
// control-socket bind / no port needed here).
void test_window_constructs_and_exposes_canvas()
{
  std::cout << "[Test] (c) SimVizWindow constructs, canvas accessible..." << std::endl;

  simviz::SimVizConfig config;
  config.control_port = 45699; // unused (start() not called) -- TEST PORT BLOCK-adjacent
  simviz::SimVizWindow window(config);
  TEST_ASSERT(window.canvas() != nullptr);

  const std::string fixture_path = MARS_TEST_FIXTURE_SCN;
  auto model =
      std::make_shared<simviz::ScenarioModel>(simviz::ScenarioModel::load_from_file(fixture_path));
  window.canvas()->set_scenario(model);

  const QImage img = grab_canvas(*window.canvas());
  const int distinct = count_distinct_colors(img);
  TEST_ASSERT(distinct > 2);

  std::cout << "[Test] (c) PASSED" << std::endl;
}

// FEATURE C regression: the toolbar noise slider must reflect a CLI-supplied
// initial config.noise_sigma_pct from first construction, not just default
// to 0 until the user manually drags it (see SimVizConfig::noise_sigma_pct's
// doc comment / SimVizWindow.cpp's slider-init code).
void test_window_noise_slider_reflects_initial_config()
{
  std::cout << "[Test] (c2) noise slider reflects CLI-supplied initial config..." << std::endl;

  simviz::SimVizConfig default_config;
  default_config.control_port = 45698; // unused (start() not called)
  simviz::SimVizWindow default_window(default_config);
  // GUI-MODE STARTUP DEFAULTS: a config with noise_sigma_pct left at its
  // own 0.0 field default now reads back 10% once routed through
  // SimVizWindow's apply_gui_startup_defaults() (SimVizWindow.cpp) -- the
  // headless/CLI-flag default itself (SimVizConfig::noise_sigma_pct) is
  // unchanged at 0.0, only what a freshly-opened GUI window's slider shows.
  TEST_ASSERT(default_window.noise_slider_percent() == 10);
  std::cout << "[Test]   default (noise_sigma_pct=0) window slider = 10% (GUI startup default) "
               "-- OK"
            << std::endl;

  simviz::SimVizConfig noisy_config;
  noisy_config.control_port = 45697; // unused (start() not called)
  noisy_config.noise_sigma_pct = 0.15;
  simviz::SimVizWindow noisy_window(noisy_config);
  TEST_ASSERT(noisy_window.noise_slider_percent() == 15);
  std::cout << "[Test]   noise_sigma_pct=0.15 window slider = "
            << noisy_window.noise_slider_percent() << "% -- OK" << std::endl;

  std::cout << "[Test] (c2) PASSED" << std::endl;
}

// ---------------------------------------------------------------------
// ISSUE 1 (widget level): "when the simulation is done, the robots jump to
// somewhere else." Drives a REAL simviz::ExecutionManager (fake sleeping
// child binaries, same pattern as test_sim_viz_control.cpp -- their actual
// behavior does not matter here, only that "some child process is alive"
// long enough for start_execution() to succeed) plus a hand-rolled ZMQ PUB
// socket standing in for what the real mpc_robot_sim binary would publish
// on its --loc-endpoint, so a live localization sample reaches the
// manager's LocalizationListener without needing the real MPC binaries.
// Then abort()s the run (one of the run-ending states named in the ISSUE 1
// spec) and grabs the canvas: if the retention fix is in place, the robot
// is still drawn at the fed live pose; if the old clear-on-stop() bug were
// reintroduced, latest_pose() would go back to nullopt and the canvas
// would fall back to initial_pose, making this render IDENTICAL to a
// render with no ExecutionManager attached at all (which unconditionally
// draws at initial_pose) -- that equality is exactly what this test
// disproves.
// ---------------------------------------------------------------------

constexpr int kIssue1TestHandshakePort = 45620; // TEST PORT BLOCK
constexpr int kIssue1TestVescPort = 45640;
constexpr int kIssue1TestLocPort = 45660;

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

// Publishes `{"x":..,"y":..,"yaw":..}` on topic "/<robot>/localization"
// repeatedly until `stop` is set -- mirrors mpc_robot_sim's own PUB-binds/
// SUB-connects wiring (MPC/src/robot_sim.cpp) closely enough for
// LocalizationListener (which only ever CONNECTs its SUB socket) to receive
// it, without needing to spawn that real binary.
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
  // Give the SUB side a moment to connect + subscribe before the first
  // publish (PUB/SUB has no late-joiner replay -- messages sent before the
  // SUB's subscription is established are simply dropped).
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

void test_robot_drawn_at_live_pose_after_run_end()
{
  std::cout << "[Test] (d) ISSUE 1: robot stays at last-known pose after run end..."
            << std::endl;

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_render_issue1_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::string fake_bin = write_fake_child_binary(tmp_dir);

  ExecutedScenario scn;
  scn.summary.label = "issue1_widget_test";
  scn.summary.all_tasks_succeeded = true;
  scn.params.min_x = 0.0;
  scn.params.min_y = 0.0;
  scn.params.max_x = 6.0;
  scn.params.max_y = 6.0;

  RobotMeta *robot = new RobotMeta();
  robot->name = "robot1";
  robot->type = EntityType::ROBOT;
  robot->initial_pose = Pose(0.0, 0.0, 0.0);
  robot->size = OccuRect{0.3, 0.2, 0.3};
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
  const std::filesystem::path scn_path = tmp_dir / "issue1_widget_test.scn.b64";
  {
    std::ofstream out(scn_path, std::ios::binary);
    out << b64;
  }

  auto model = std::make_shared<simviz::ScenarioModel>(
      simviz::ScenarioModel::load_from_file(scn_path.string()));
  TEST_ASSERT(model->is_loaded());

  simviz::SimVizConfig config;
  config.handshake_port_start = kIssue1TestHandshakePort;
  config.vesc_port_start = kIssue1TestVescPort;
  config.loc_port_start = kIssue1TestLocPort;
  config.mpc_controller_path = fake_bin;
  config.robot_sim_path = fake_bin;
  config.run_dir = (tmp_dir / "runs").string();

  simviz::ExecutionManager exec_mgr(config);

  // The fed "live" pose -- deliberately far from robot1's initial_pose
  // (0,0,0) so a render that fell back to initial_pose is unmistakably
  // different from one using this pose.
  constexpr double kFedX = 4.0, kFedY = 3.0, kFedYaw = 0.7;
  std::atomic<bool> stop_publisher{false};
  std::thread publisher(publish_fake_localization, kIssue1TestLocPort, "robot1", kFedX, kFedY,
                         kFedYaw, std::ref(stop_publisher));

  const bool accepted = exec_mgr.start_execution(model);
  TEST_ASSERT(accepted);

  // Poll for the fed live pose to arrive.
  bool got_live = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline)
  {
    auto live = exec_mgr.localization().latest_pose("robot1");
    if (live && live->has_pose)
    {
      got_live = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  TEST_ASSERT(got_live && "fed localization sample never reached LocalizationListener");

  // ISSUE 1's run-ending teardown: abort() tears down children (same
  // terminate_children() path finalize_run() uses for DONE/ERR).
  exec_mgr.abort();
  TEST_ASSERT(simviz::run_state_name(exec_mgr.state()) == std::string("Idle"));

  // Retention check at the core level (this is the direct, unambiguous
  // assertion -- the widget-level image-diff below is a secondary,
  // "does the wiring actually reach the canvas" check).
  {
    auto live = exec_mgr.localization().latest_pose("robot1");
    TEST_ASSERT(live && live->has_pose && "latest_pose() went back to empty after abort()");
    TEST_ASSERT(std::abs(live->pose.x - kFedX) < 1e-6);
    TEST_ASSERT(std::abs(live->pose.y - kFedY) < 1e-6);
  }

  stop_publisher.store(true);
  publisher.join();

  // Widget-level check: canvas rendered post-abort (retention in effect)
  // vs. the same canvas with NO ExecutionManager attached (which
  // unconditionally draws at initial_pose) must differ -- if the old
  // clear-on-stop() bug were reintroduced, these two renders would be
  // pixel-identical.
  simviz::SimVizCanvas canvas;
  canvas.set_scenario(model);
  canvas.set_execution_manager(&exec_mgr);
  const QImage img_post_abort = grab_canvas(canvas);

  canvas.set_execution_manager(nullptr);
  const QImage img_initial_only = grab_canvas(canvas);

  TEST_ASSERT(img_post_abort != img_initial_only &&
              "post-abort render is identical to the initial_pose-only render -- robot snapped "
              "back to initial_pose instead of staying at its last-known live pose");

  std::filesystem::remove_all(tmp_dir);
  std::cout << "[Test] (d) PASSED" << std::endl;
}

// ---------------------------------------------------------------------
// FEATURE B: reference-pose overlay. Deliberate white-box mirror of
// SimVizCanvas::paintEvent()'s map_coord transform (padding=10, the
// outer-rect-inflated-by-lf fit-and-center math) for a fixed 900x700 canvas
// -- same "couple the test to implementation specifics" technique test (d)
// above already relies on (there, exact pixel-perfect fed-vs-fallback pose
// comparison; here, exact screen-space ROI placement). Any future change to
// that transform must update this mirror too.
// ---------------------------------------------------------------------

QPointF expected_map_coord(double x, double y, double min_x, double min_y, double max_x,
                            double max_y, double lf, int canvas_w, int canvas_h)
{
  const double outer_min_x = min_x - lf;
  const double outer_max_x = max_x + lf;
  const double outer_min_y = min_y - lf;
  const double outer_max_y = max_y + lf;
  const double workspace_width = std::max(outer_max_x - outer_min_x, 1e-3);
  const double workspace_height = std::max(outer_max_y - outer_min_y, 1e-3);

  const int padding = 10;
  const double available_w = std::max(canvas_w - 2 * padding, 1);
  const double available_h = std::max(canvas_h - 2 * padding, 1);
  const double aspect_ratio = workspace_width / workspace_height;
  const double available_ratio = available_w / available_h;

  double draw_w, draw_h, rect_x, rect_y;
  if (available_ratio > aspect_ratio)
  {
    draw_h = available_h;
    draw_w = aspect_ratio * draw_h;
    rect_x = (canvas_w - draw_w) / 2.0;
    rect_y = padding;
  }
  else
  {
    draw_w = available_w;
    draw_h = draw_w / aspect_ratio;
    rect_x = padding;
    rect_y = (canvas_h - draw_h) / 2.0;
  }

  const double scale = std::min(draw_w / workspace_width, draw_h / workspace_height);
  const double drawn_w = workspace_width * scale;
  const double drawn_h = workspace_height * scale;
  const double offset_x = rect_x + (draw_w - drawn_w) / 2.0;
  const double offset_y = rect_y + (draw_h - drawn_h) / 2.0;

  const double wx = offset_x + (x - outer_min_x) * scale;
  const double wy = offset_y + drawn_h - (y - outer_min_y) * scale;
  return QPointF(wx, wy);
}

void test_reference_pose_overlay_renders()
{
  std::cout << "[Test] (e) FEATURE B: reference-pose overlay toggle..." << std::endl;

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_render_refpose_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  const std::string fake_bin = write_fake_child_binary(tmp_dir);

  ExecutedScenario scn;
  scn.summary.label = "refpose_widget_test";
  scn.summary.all_tasks_succeeded = true;
  scn.params.min_x = 0.0;
  scn.params.min_y = 0.0;
  scn.params.max_x = 6.0;
  scn.params.max_y = 6.0;

  RobotMeta *robot = new RobotMeta();
  robot->name = "robot1";
  robot->type = EntityType::ROBOT;
  robot->initial_pose = Pose(0.0, 0.0, 0.0);
  robot->size = OccuRect{0.3, 0.2, 0.3}; // front_length=0.3 -> lf=0.3 (this scenario's only robot)
  robot->speed_transit = 0.2;
  robot->speed_transfer = 0.15;
  scn.entities["robot1"] = robot;

  // No trajectory needed: TimeTable::add_initial() alone records robot1's
  // pose at t=0.0 -- exactly what plan_time()==0.0 below will query.
  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  const std::string b64 = serialize_executed_scenario_b64(scn);
  const std::filesystem::path scn_path = tmp_dir / "refpose_widget_test.scn.b64";
  {
    std::ofstream out(scn_path, std::ios::binary);
    out << b64;
  }

  auto model = std::make_shared<simviz::ScenarioModel>(
      simviz::ScenarioModel::load_from_file(scn_path.string()));
  TEST_ASSERT(model->is_loaded());

  simviz::SimVizConfig config;
  config.handshake_port_start = kIssue1TestHandshakePort;
  config.vesc_port_start = kIssue1TestVescPort;
  config.loc_port_start = kIssue1TestLocPort;
  config.mpc_controller_path = fake_bin;
  config.robot_sim_path = fake_bin;
  config.run_dir = (tmp_dir / "runs").string();

  simviz::ExecutionManager exec_mgr(config);

  // Fed live pose deliberately far from the timetable's t=0 reference pose
  // (robot1's initial_pose (0,0,0)). NOTE: exec_mgr.tick() is never called
  // in this test (same technique as test (d) above), so t0_valid_ never
  // becomes true and plan_time() stays deterministically 0.0 the whole
  // time -- the reference pose is therefore always exactly robot1's
  // initial_pose, with no wall-clock race to account for.
  constexpr double kFedX = 0.0, kFedY = 3.0, kFedYaw = 1.2;
  std::atomic<bool> stop_publisher{false};
  std::thread publisher(publish_fake_localization, kIssue1TestLocPort, "robot1", kFedX, kFedY,
                         kFedYaw, std::ref(stop_publisher));

  const bool accepted = exec_mgr.start_execution(model);
  TEST_ASSERT(accepted);

  bool got_live = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline)
  {
    auto live = exec_mgr.localization().latest_pose("robot1");
    if (live && live->has_pose)
    {
      got_live = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  TEST_ASSERT(got_live && "fed localization sample never reached LocalizationListener");
  TEST_ASSERT(std::abs(exec_mgr.plan_time()) < 1e-9 &&
              "plan_time unexpectedly nonzero -- this test's t=0 reference-pose assumption "
              "requires exec_mgr.tick() to never have been called");

  simviz::SimVizCanvas canvas;
  canvas.set_scenario(model);
  canvas.set_execution_manager(&exec_mgr);
  // FEATURE B: SimVizCanvas's own default is still hidden -- only
  // SimVizWindow's "Show reference poses" View menu action defaults to
  // checked now (GUI-MODE STARTUP DEFAULTS, see SimVizWindow.cpp); a bare
  // SimVizCanvas constructed directly, as here, is unaffected.
  TEST_ASSERT(!canvas.show_reference_poses());

  const QImage img_off = grab_canvas(canvas);

  canvas.set_show_reference_poses(true);
  TEST_ASSERT(canvas.show_reference_poses());
  const QImage img_on = grab_canvas(canvas);

  TEST_ASSERT(img_off != img_on &&
              "enabling the reference-pose overlay produced no visible change");

  // Reference-pose (dashed, alpha~120) footprint+arrow lands at robot1's
  // initial_pose (0,0,0); live/actual (solid, alpha~150) footprint lands at
  // the fed (0,3,1.2) -- the two ROIs below are ~300px apart on the 900x700
  // canvas, comfortably non-overlapping.
  const QPointF ref_screen = expected_map_coord(0.0, 0.0, 0.0, 0.0, 6.0, 6.0, 0.3, 900, 700);
  const QPointF live_screen = expected_map_coord(kFedX, kFedY, 0.0, 0.0, 6.0, 6.0, 0.3, 900, 700);
  std::cout << "[Test]   expected ref screen~=(" << ref_screen.x() << "," << ref_screen.y()
            << ") live screen~=(" << live_screen.x() << "," << live_screen.y() << ")"
            << std::endl;

  constexpr int kRoiHalf = 45; // covers footprint (~31px) + arrow (14px) + antialiasing margin
  auto roi_rect = [&](const QPointF &center)
  {
    return QRect(static_cast<int>(center.x()) - kRoiHalf, static_cast<int>(center.y()) - kRoiHalf,
                 2 * kRoiHalf, 2 * kRoiHalf)
        .intersected(QRect(0, 0, img_off.width(), img_off.height()));
  };

  const QImage ref_roi_off = img_off.copy(roi_rect(ref_screen));
  const QImage ref_roi_on = img_on.copy(roi_rect(ref_screen));
  TEST_ASSERT(ref_roi_off != ref_roi_on &&
              "the reference-pose overlay's footprint/arrow did not render at the expected "
              "timetable-pose screen location");
  std::cout << "[Test]   dashed reference marker appeared at the expected timetable-pose "
               "location once enabled"
            << std::endl;

  // "Both marker styles": the solid actual-pose footprint at the LIVE
  // location must be untouched by the toggle -- proving the overlay adds a
  // second, distinct marker rather than replacing/altering the first.
  const QImage live_roi_off = img_off.copy(roi_rect(live_screen));
  const QImage live_roi_on = img_on.copy(roi_rect(live_screen));
  TEST_ASSERT(live_roi_off == live_roi_on &&
              "toggling the reference-pose overlay unexpectedly changed the solid actual-pose "
              "footprint's rendering");
  std::cout << "[Test]   solid actual-pose marker unaffected by the toggle" << std::endl;

  // And disabling it again removes it -- a live toggle, not one-shot (same
  // check test (b) above does for show_goal_outlines_).
  canvas.set_show_reference_poses(false);
  TEST_ASSERT(!canvas.show_reference_poses());
  const QImage img_off_again = grab_canvas(canvas);
  TEST_ASSERT(img_off_again.copy(roi_rect(ref_screen)) == ref_roi_off &&
              "disabling the reference-pose overlay did not remove the dashed marker");

  stop_publisher.store(true);
  publisher.join();
  std::filesystem::remove_all(tmp_dir);

  std::cout << "[Test] (e) PASSED" << std::endl;
}

} // namespace

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  test_real_fixture_renders();
  test_synthetic_transfer_scenario_renders();
  test_window_constructs_and_exposes_canvas();
  test_window_noise_slider_reflects_initial_config();
  test_robot_drawn_at_live_pose_after_run_end();
  test_reference_pose_overlay_renders();

  std::cout << "\n[Test] All test_sim_viz_window_render tests passed." << std::endl;
  return 0;
}
