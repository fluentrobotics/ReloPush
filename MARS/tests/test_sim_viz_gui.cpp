// T4 (this task's DESIGN doc, part C): offscreen GUI tests for the
// restructured simviz::SimVizWindow (QTabWidget: tab 0 "Simulation" =
// pre-existing layout moved unchanged, tab 1 "OptiTrack" = simviz::MocapTab)
// plus MocapTab itself. Run under QT_QPA_PLATFORM=offscreen.
//
// Part 1: window construction -- tab structure (count/titles), tab 0's
// canvas still accessible/renders via canvas() exactly like before this
// task (the UNMODIFIED test_sim_viz_window_render.cpp is this project's
// real regression coverage for that; this is a light sanity spot-check).
//
// Part 2: MocapTab with a REAL live body list -- spawns fake_motive +
// mars_sim_viz's MocapManager spawns/owns a REAL optitrack_zmq_bridge (same
// process-level pattern as test_sim_viz_mocap.cpp's Part 2, against a
// COPIED map-config with a NON-identity mocap_to_world_matrix -- a 30deg
// rotation + translation -- so the transform visual's planner/Motive frames
// are visibly distinct rather than coincident). A synthetic 2-robot
// scenario is loaded ("robot1" aliased from fake_motive's "mushr2" body,
// "robot2" left unmapped) so the mapping table has real rows. Asserts:
// indicator color/state text track Connecting->Connected, the rigid-body
// table gets real rows, the mapping table builds one row per scenario
// robot with sane freshness/tooltip behavior (real_mode is false for this
// window, so "Move robots to initial poses" must stay disabled with an
// explanatory tooltip), and the transform widget/tab render non-trivial
// pixel content -- grabbed to a PNG for manual inspection.
//
// TEST PORTS ONLY: fake_motive/bridge UDP command 45510 / data 45511
// (mirrors test_sim_viz_mocap.cpp's Part 2 exactly -- never run both
// concurrently, see this task's "run port-using suites sequentially"
// rule), bridge ZMQ auto-discovery loc port 45685.

#include "MocapTab.h"
#include "SimVizCore.h"
#include "SimVizWindow.h"
#include "test_sim_viz_shared.h"

#include <ExecutedScenarioSerialization.h>
#include <PHAstar/Entities.h>
#include <PHAstar/TimeTable.h>

#include <QApplication>
#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QImage>
#include <QLineEdit>
#include <QSet>
#include <QTableWidget>

#include <nlohmann/json.hpp>

#include <sys/stat.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <thread>
#include <unistd.h>

bool DEBUG_VIS = false;

#ifndef MARS_TEST_RENDER_OUT_DIR
#define MARS_TEST_RENDER_OUT_DIR "."
#endif

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

constexpr int kFakeMotiveCommandPort = 45510;
constexpr int kBridgeLocalDataPort = 45511;
constexpr int kBridgeLocPortStart = 45685;

int count_distinct_colors(const QImage &img)
{
  QSet<QRgb> colors;
  for (int y = 0; y < img.height(); ++y)
  {
    for (int x = 0; x < img.width(); ++x)
    {
      colors.insert(img.pixel(x, y));
      if (colors.size() > 64)
        return colors.size();
    }
  }
  return colors.size();
}

// Finds every pixel in `img` whose color is within `tol` (per channel) of
// `target`, returning their bounding box's [min_y, max_y] (inclusive) --
// used by the MocapTransformWidget Motive-axes-glyph render test to locate
// where a specific axis color was drawn without needing to duplicate that
// widget's internal fit-to-viewport math. `*found` (if given) is set to
// whether any matching pixel existed at all.
std::pair<int, int> find_color_y_range(const QImage &img, const QColor &target, int tol,
                                        bool *found = nullptr)
{
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();
  bool any = false;
  for (int y = 0; y < img.height(); ++y)
  {
    for (int x = 0; x < img.width(); ++x)
    {
      const QColor px = img.pixelColor(x, y);
      if (std::abs(px.red() - target.red()) <= tol && std::abs(px.green() - target.green()) <= tol &&
          std::abs(px.blue() - target.blue()) <= tol)
      {
        any = true;
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }
    }
  }
  if (found)
    *found = any;
  return {min_y, max_y};
}

// Fake sleeping-shell child binary -- exactly test_sim_viz_window_render.cpp's
// write_fake_child_binary() (its actual behavior doesn't matter, only that
// "some child process is alive" long enough for start_execution() to
// succeed and populate scenario_model()); duplicated here (rather than
// shared) since it's a two-line helper and test_sim_viz_window_render.cpp
// isn't a shared header.
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

QImage grab_widget(QWidget &w, int width, int height)
{
  w.resize(width, height);
  w.show();
  w.repaint();
  return w.grab().toImage();
}

std::string synthetic_mocap_config_txt()
{
  return "ip_address:127.0.0.1\nType:Unicast\nCommand Port:" +
         std::to_string(kFakeMotiveCommandPort) +
         "\nData Port:" + std::to_string(kBridgeLocalDataPort) +
         "\nMulticast Interface:239.255.42.99\n";
}

// NON-identity transform: 30deg rotation + (1.0, 0.5) translation -- so the
// transform widget's planner/Motive frames are visibly distinct (a pure
// identity matrix would draw them coincident, per this task's design doc's
// own "identity/uncalibrated" note).
nlohmann::json synthetic_map_config_json()
{
  const double theta = 30.0 * M_PI / 180.0;
  nlohmann::json j;
  j["_doc"] = "test-only synthetic mocap map-config for test_sim_viz_gui (T4)";
  j["y_up"] = false;
  j["yaw_offset"] = {{"robot1", 0.1}};
  j["aliases"] = {{"mushr2", "robot1"}};
  j["mocap_to_world_matrix"] = {{std::cos(theta), -std::sin(theta), 1.0},
                                 {std::sin(theta), std::cos(theta), 0.5},
                                 {0, 0, 1}};
  return j;
}

std::shared_ptr<simviz::ScenarioModel> build_two_robot_scenario(const std::filesystem::path &dir)
{
  ExecutedScenario scn;
  scn.summary.label = "test_sim_viz_gui_scn";
  scn.summary.all_tasks_succeeded = true;
  scn.params.min_x = -2.0;
  scn.params.min_y = -2.0;
  scn.params.max_x = 4.0;
  scn.params.max_y = 4.0;

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
  robot2->initial_pose = Pose(1.0, 1.0, 0.0);
  robot2->size = OccuRect{0.3, 0.2, 0.3};
  robot2->speed_transit = 0.2;
  robot2->speed_transfer = 0.15;

  scn.entities["robot1"] = robot1;
  scn.entities["robot2"] = robot2;

  scn.timetable = TimeTable(0.5);
  scn.timetable.add_initial(scn.entities);

  const std::string b64 = serialize_executed_scenario_b64(scn);
  const std::filesystem::path scn_path = dir / "test_sim_viz_gui_scn.scn.b64";
  {
    std::ofstream out(scn_path, std::ios::binary);
    out << b64;
  }
  return std::make_shared<simviz::ScenarioModel>(
      simviz::ScenarioModel::load_from_file(scn_path.string()));
}

void test_window_tab_structure()
{
  std::cout << "[Test] (a) SimVizWindow constructs with tabs..." << std::endl;

  simviz::SimVizConfig config;
  config.control_port = 45601; // unused (start() not called)
  simviz::SimVizWindow window(config);

  TEST_ASSERT(window.tabs() != nullptr);
  TEST_ASSERT(window.tabs()->count() == 2);
  TEST_ASSERT(window.tabs()->tabText(0).toStdString() == "Simulation");
  TEST_ASSERT(window.tabs()->tabText(1).toStdString() == "OptiTrack");
  TEST_ASSERT(window.tabs()->currentIndex() == 0);
  TEST_ASSERT(window.canvas() != nullptr);
  TEST_ASSERT(window.mocap_tab() != nullptr);

  // GUI-MODE STARTUP DEFAULTS: a freshly-constructed SimVizWindow (built
  // from a default-constructed, otherwise-untouched SimVizConfig, exactly
  // as here) starts with motor+steering noise at 10%, the "Show reference
  // poses" View menu action checked (and propagated to the canvas), and
  // motor stall enabled at its existing 0.10 m/s level -- see
  // apply_gui_startup_defaults() in SimVizWindow.cpp. Headless mode /
  // SimVizConfig's own field defaults (0%, unchecked/hidden, stall off) are
  // untouched -- see test_sim_viz_control.cpp / test_sim_viz_integration.cpp
  // for that headless-side coverage.
  TEST_ASSERT(window.noise_slider_percent() == 10);
  TEST_ASSERT(window.steer_noise_slider_percent() == 10);
  TEST_ASSERT(window.canvas()->show_reference_poses());
  TEST_ASSERT(window.stall_enabled_checked() == true);
  TEST_ASSERT(std::abs(window.stall_level_value() - 0.10) < 1e-9);
  TEST_ASSERT(window.stall_level_enabled() == true);
  std::cout << "[Test]   GUI startup defaults: noise=10%/10%, reference poses ON, stall ON @ "
               "0.10 m/s -- OK"
            << std::endl;

  // Tab 0's canvas still renders (light spot-check -- the UNMODIFIED
  // test_sim_viz_window_render.cpp is this project's real regression
  // coverage for the fixture-driven render path).
  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_gui_test_tab0_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);
  auto model = build_two_robot_scenario(tmp_dir);
  window.canvas()->set_scenario(model);
  const QImage img = grab_widget(*window.canvas(), 700, 500);
  TEST_ASSERT(count_distinct_colors(img) > 2);
  std::filesystem::remove_all(tmp_dir);

  std::cout << "[Test] (a) PASSED" << std::endl;
}

void test_mocap_tab_live_render()
{
  std::cout << "[Test] (b) MocapTab with a real live body list..." << std::endl;

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_gui_test_tab1_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  const std::filesystem::path map_config_path = tmp_dir / "map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  {
    std::ifstream probe(fake_motive_exe);
    TEST_ASSERT(probe.good());
  }
  std::vector<std::string> fake_motive_args = {
      "--target-ip",          "127.0.0.1",
      "--target-port",        std::to_string(kBridgeLocalDataPort),
      "--rate",               "60",
      "--natnet-version",     "3.1",
      "--body-ids",           "1,2",
      "--body-names",         "mushr2,block",
      "--serve-command-port", std::to_string(kFakeMotiveCommandPort),
      "--served-app-name",    "TestMotiveGui",
      "--served-version",     "3.1",
      "--duration-s",         "300",
  };
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe, fake_motive_args), "fake_motive[gui]");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  const std::string fake_bin = write_fake_child_binary(tmp_dir);

  simviz::SimVizConfig config;
  config.control_port = 45602; // unused (start() not called)
  config.handshake_port_start = 45720; // TEST PORT BLOCK -- never mars_sim_viz's real defaults
  config.vesc_port_start = 45740;
  config.loc_port_start = 45760;
  config.mpc_controller_path = fake_bin;
  config.robot_sim_path = fake_bin;
  config.run_dir = (tmp_dir / "runs").string();
  config.mocap.map_config_path = map_config_path.string();
  config.mocap.mocap_config_path = mocap_config_path.string();
  config.mocap.server_ip = "127.0.0.1";
  config.mocap.server_ip_explicit = true;
  config.mocap.mode = "unicast";
  config.mocap.mode_explicit = true;
  config.mocap.command_port = kFakeMotiveCommandPort;
  config.mocap.command_port_explicit = true;
  config.mocap.local_data_port = kBridgeLocalDataPort;
  config.mocap.local_data_port_explicit = true;
  config.mocap.loc_port_start = kBridgeLocPortStart;
  // real_mode deliberately left false -- exercises the "Move robots to
  // initial poses" button's disabled/tooltip path below.

  simviz::SimVizWindow window(config);
  auto model = build_two_robot_scenario(tmp_dir);
  window.canvas()->set_scenario(model);
  window.manager().execution_manager().start_execution(model);
  // Immediately abort -- we only wanted scenario_model() populated (so the
  // mapping table has rows) without an actual mpc_controller/mpc_robot_sim
  // spawn racing this test; abort() tears down whatever was spawned and
  // returns to Idle, but scenario_model() is retained (see
  // ExecutionManager's ISSUE 1 retention contract exercised by
  // test_sim_viz_window_render.cpp test (d)).
  window.manager().execution_manager().abort();
  TEST_ASSERT(window.manager().execution_manager().scenario_model() != nullptr);

  simviz::MocapTab *tab = window.mocap_tab();
  TEST_ASSERT(tab != nullptr);

  // Before connecting: indicator reflects Disconnected, Move button
  // disabled (no live bodies + real_mode false).
  window.refresh_display();
  TEST_ASSERT(tab->state_label()->text().toStdString().rfind("DISCONNECTED", 0) == 0);
  TEST_ASSERT(!tab->move_to_initial_button()->isEnabled());
  TEST_ASSERT(tab->move_to_initial_button()->toolTip().contains("real-mode"));
  std::cout << "[test]   pre-connect: indicator=Disconnected, Move button disabled with "
               "real-mode tooltip -- OK"
            << std::endl;

  // Connect (direct manager call -- same production entry point the
  // Connect button's slot uses, see MocapTab::on_connect_toggle()).
  {
    std::string err;
    const bool started = window.manager().mocap_manager().start_bridge(&err);
    TEST_ASSERT(started);
  }

  bool reached_connected = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (std::chrono::steady_clock::now() < deadline)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    if (window.manager().mocap_manager().state() == simviz::MocapState::Connected)
    {
      reached_connected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  TEST_ASSERT(reached_connected);
  std::cout << "[test]   MocapManager reached Connected" << std::endl;

  // A few more refresh_display() calls so the mapping/body tables and
  // transform widget settle against the now-live body list.
  for (int i = 0; i < 5; ++i)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  TEST_ASSERT(tab->state_label()->text().toStdString().rfind("CONNECTED", 0) == 0);
  TEST_ASSERT(!tab->uptime_label()->text().isEmpty());
  std::cout << "[test]   indicator state text == Connected..., uptime label non-empty" << std::endl;

  TEST_ASSERT(tab->body_table()->rowCount() >= 2);
  std::cout << "[test]   rigid-body table has " << tab->body_table()->rowCount() << " rows"
            << std::endl;

  // Staged transform readout (part C of the transform-restructure task):
  // body_table_ now has 12 columns -- id/age, then RAW/MOCAP-ROTATION/
  // CALIBRATION in pipeline order -- see MocapTab::update_body_table()'s
  // doc comment. Cross-check the 'mushr2' row's raw/mocap columns against
  // an INDEPENDENTLY computed simviz::compute_staged_transform_view() call
  // (same TransformInfo + same final pose the table itself displays in
  // columns 9-11), proving the UI reflects that function's math exactly,
  // not just that the columns are non-empty.
  {
    TEST_ASSERT(tab->body_table()->columnCount() == 12);
    const std::vector<std::string> expected_headers = {
        "Motive", "id", "age (s)", "raw x", "raw y/z", "raw yaw",
        "mocap x", "mocap y", "mocap yaw", "x", "y", "yaw"};
    for (int col = 0; col < 12; ++col)
    {
      TEST_ASSERT(tab->body_table()->horizontalHeaderItem(col)->text().toStdString() ==
                  expected_headers[static_cast<size_t>(col)]);
    }

    int mushr2_row = -1;
    for (int row = 0; row < tab->body_table()->rowCount(); ++row)
    {
      if (tab->body_table()->item(row, 0)->text().toStdString() == "mushr2")
      {
        mushr2_row = row;
        break;
      }
    }
    TEST_ASSERT(mushr2_row >= 0);

    auto cell_double = [&](int col) {
      QString text = tab->body_table()->item(mushr2_row, col)->text();
      // "raw y/z" (col 4) is prefixed "y=" or "z=" -- strip it before parsing.
      const int eq = text.indexOf('=');
      if (eq >= 0)
        text = text.mid(eq + 1);
      bool ok = false;
      const double v = text.toDouble(&ok);
      TEST_ASSERT(ok);
      return v;
    };
    // Deliberately read the FINAL pose back from the table's own x/y/yaw
    // columns (9-11), NOT a fresh MocapManager::body_pose() call -- the
    // synthetic body is CONTINUOUSLY moving (fake_motive publishes at
    // 60Hz), so a separately-fetched "live" pose would race against the
    // snapshot update_body_table() already used to compute columns 3-8 in
    // THIS SAME row, on THIS SAME refresh_display() pass (bodies() is
    // copied out under lock ONCE per update_body_table() call, so every
    // column in one row is internally consistent with every other column
    // in that row, even though the table as a whole may be one publish
    // cycle stale by the time this check runs). Tolerance absorbs the
    // table's OWN 3-decimal display rounding, doubly applied (once to
    // final_x/y/yaw before being fed back into compute_staged_transform_view()
    // here, once more to the raw/mocap columns being compared against).
    const double final_x = cell_double(9), final_y = cell_double(10), final_yaw = cell_double(11);

    const simviz::TransformInfo info = window.manager().mocap_manager().transform_info();
    const simviz::StagedTransformView expected =
        simviz::compute_staged_transform_view(info, Pose(final_x, final_y, final_yaw), "robot1");
    TEST_ASSERT(expected.ok);

    const double eps = 3e-3;
    TEST_ASSERT(std::abs(cell_double(6) - expected.mocap_x) < eps);
    TEST_ASSERT(std::abs(cell_double(7) - expected.mocap_y) < eps);
    TEST_ASSERT(std::abs(cell_double(8) - expected.mocap_yaw) < eps);
    // synthetic_map_config_json() sets y_up=false -- raw y/z shows "y=..."
    // (identity mocap_rotation position: raw_y_m == mocap_y exactly).
    TEST_ASSERT(tab->body_table()->item(mushr2_row, 4)->text().toStdString().rfind("y=", 0) == 0);
    TEST_ASSERT(std::abs(cell_double(3) - expected.raw_x_m) < eps);
    TEST_ASSERT(std::abs(cell_double(4) - expected.raw_y_m) < eps);
    TEST_ASSERT(std::abs(cell_double(5) - expected.raw_yaw) < eps);
    std::cout << "[test]   staged readout (row '" << mushr2_row
              << "'): raw=(" << cell_double(3) << ", " << tab->body_table()->item(mushr2_row, 4)->text().toStdString()
              << ", " << cell_double(5) << ") mocap=(" << cell_double(6) << ", " << cell_double(7)
              << ", " << cell_double(8) << ") matches compute_staged_transform_view() -- OK"
              << std::endl;
  }

  TEST_ASSERT(tab->mapping_table()->rowCount() == 2);
  {
    // Row order is name-sorted ("robot1" < "robot2").
    QComboBox *combo0 = qobject_cast<QComboBox *>(tab->mapping_table()->cellWidget(0, 1));
    TEST_ASSERT(combo0 != nullptr);
    TEST_ASSERT(combo0->currentText().toStdString() == "mushr2");
    QLineEdit *vesc0 = qobject_cast<QLineEdit *>(tab->mapping_table()->cellWidget(0, 3));
    TEST_ASSERT(vesc0 != nullptr && !vesc0->text().isEmpty());
    std::cout << "[test]   mapping row 0 (robot1): mapped body='"
              << combo0->currentText().toStdString() << "' vesc='" << vesc0->text().toStdString()
              << "'" << std::endl;
  }

  // Still real_mode==false -- Move button must stay disabled even now that
  // a body is live and fresh (only the real-mode precondition is failing).
  TEST_ASSERT(!tab->move_to_initial_button()->isEnabled());
  TEST_ASSERT(tab->move_to_initial_button()->toolTip().contains("real-mode"));
  std::cout << "[test]   post-connect: Move button still disabled (real_mode==false) -- OK"
            << std::endl;

  // --- Render assertions ---
  const QImage tab_img = grab_widget(*tab, 1100, 650);
  const int distinct = count_distinct_colors(tab_img);
  std::cout << "[Test]   MocapTab rendered " << tab_img.width() << "x" << tab_img.height() << ", "
            << distinct << "+ distinct colors" << std::endl;
  TEST_ASSERT(distinct > 4);

  const QImage transform_img = grab_widget(*tab->transform_widget(), 400, 320);
  const int transform_distinct = count_distinct_colors(transform_img);
  std::cout << "[Test]   transform widget rendered, " << transform_distinct << "+ distinct colors"
            << std::endl;
  TEST_ASSERT(transform_distinct > 2);
  TEST_ASSERT(!tab->transform_numbers_label()->text().isEmpty());

  const std::string out_path = std::string(MARS_TEST_RENDER_OUT_DIR) + "/test_sim_viz_mocap_tab.png";
  const bool saved = tab_img.save(QString::fromStdString(out_path));
  std::cout << "[Test]   saved " << (saved ? "" : "FAILED ") << out_path << std::endl;
  TEST_ASSERT(saved);

  // --- Calibration UI (part B): fields+checkbox+Apply exist and round-trip
  // a value through the save path (on this test's COPIED map_config_path --
  // never the real project's MPC/config/mocap_map_config.json).
  // The bridge is Connected at this point, so this also exercises Apply's
  // "restart the running bridge" branch (MocapManager::stop_bridge()+
  // start_bridge()), not just the save. ---
  {
    TEST_ASSERT(tab->calib_tx_spin() != nullptr);
    TEST_ASSERT(tab->calib_ty_spin() != nullptr);
    TEST_ASSERT(tab->calib_theta_deg_spin() != nullptr);
    TEST_ASSERT(tab->calib_y_up_checkbox() != nullptr);
    TEST_ASSERT(tab->calib_apply_button() != nullptr);

    tab->calib_tx_spin()->setValue(1.5);
    tab->calib_ty_spin()->setValue(-0.5);
    tab->calib_theta_deg_spin()->setValue(45.0);
    tab->calib_y_up_checkbox()->setChecked(true);
    tab->calib_apply_button()->click(); // synchronously runs on_apply_calibration().

    bool reconnected = false;
    const auto reconnect_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < reconnect_deadline)
    {
      window.manager().mocap_manager().tick();
      window.refresh_display();
      if (window.manager().mocap_manager().state() == simviz::MocapState::Connected)
      {
        reconnected = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    TEST_ASSERT(reconnected);
    std::cout << "[test]   calibration Apply: bridge reconnected after restart" << std::endl;

    nlohmann::json saved_cfg;
    {
      std::ifstream f(map_config_path);
      f >> saved_cfg;
    }
    TEST_ASSERT(!saved_cfg.contains("mocap_to_world_matrix"));
    TEST_ASSERT(std::fabs(saved_cfg.value("x0", 0.0) - 1.5) < 1e-9);
    TEST_ASSERT(std::fabs(saved_cfg.value("y0", 0.0) - (-0.5)) < 1e-9);
    TEST_ASSERT(std::fabs(saved_cfg.value("theta0", 0.0) - (45.0 * M_PI / 180.0)) < 1e-9);
    TEST_ASSERT(saved_cfg.value("y_up", false) == true);
    TEST_ASSERT(saved_cfg.contains("aliases") && saved_cfg["aliases"]["mushr2"] == "robot1");
    std::cout << "[test]   calibration Apply: map-config on disk reflects new x0/y0/theta0/y_up, "
                 "mocap_to_world_matrix removed, aliases preserved -- OK"
              << std::endl;
  }

  // --- BUG FIX regression (stale-state clobber): Apply used to write ALL
  // FOUR calibration fields from the GUI's snapshot unconditionally, so a
  // field the user never touched THIS session got silently overwritten
  // with whatever the GUI last loaded -- clobbering an out-of-band/
  // hand-corrected edit to the config file. Reproduces the exact reported
  // scenario: hand-edit the map-config file OUT OF BAND (bypassing
  // MocapManager entirely) with a DIFFERENT y_up + theta0 than what the
  // GUI currently shows (from the Apply above: y_up=true, theta0=45deg),
  // then edit ONLY tx in the GUI and Apply -- the file must KEEP the
  // out-of-band y_up/theta0 (not the stale GUI values) AND get the new
  // tx. ---
  {
    nlohmann::json hand_edited;
    {
      std::ifstream f(map_config_path);
      f >> hand_edited;
    }
    const bool out_of_band_y_up = !hand_edited.value("y_up", false); // true -> false.
    const double out_of_band_theta0 = 10.0 * M_PI / 180.0;           // != the GUI's stale 45deg.
    const double unchanged_y0 = hand_edited.value("y0", 0.0);        // -0.5, untouched by this edit.
    hand_edited["y_up"] = out_of_band_y_up;
    hand_edited["theta0"] = out_of_band_theta0;
    {
      std::ofstream f(map_config_path);
      f << hand_edited.dump(2);
    }
    std::cout << "[test]   stale-state clobber regression: hand-edited map-config out of band "
                 "(y_up -> "
              << (out_of_band_y_up ? "true" : "false") << ", theta0 -> "
              << (out_of_band_theta0 * 180.0 / M_PI)
              << "deg); GUI still shows the OLD (now stale) values" << std::endl;

    // The GUI still shows the PREVIOUS Apply's values (x0=1.5, y0=-0.5,
    // theta0=45deg, y_up=true) -- stale relative to the file just
    // hand-edited above. User edits ONLY tx.
    const double new_tx = 9.25;
    tab->calib_tx_spin()->setValue(new_tx);
    tab->calib_apply_button()->click(); // synchronously runs on_apply_calibration().

    bool reconnected2 = false;
    const auto reconnect_deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < reconnect_deadline2)
    {
      window.manager().mocap_manager().tick();
      window.refresh_display();
      if (window.manager().mocap_manager().state() == simviz::MocapState::Connected)
      {
        reconnected2 = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    TEST_ASSERT(reconnected2);
    std::cout << "[test]   stale-state clobber regression: bridge reconnected after Apply"
              << std::endl;

    nlohmann::json saved_cfg2;
    {
      std::ifstream f(map_config_path);
      f >> saved_cfg2;
    }
    TEST_ASSERT(std::fabs(saved_cfg2.value("x0", 0.0) - new_tx) < 1e-9);
    TEST_ASSERT(std::fabs(saved_cfg2.value("y0", 0.0) - unchanged_y0) < 1e-9);
    TEST_ASSERT(std::fabs(saved_cfg2.value("theta0", 0.0) - out_of_band_theta0) < 1e-9);
    TEST_ASSERT(saved_cfg2.value("y_up", false) == out_of_band_y_up);
    TEST_ASSERT(saved_cfg2.contains("aliases") && saved_cfg2["aliases"]["mushr2"] == "robot1");
    std::cout << "[test]   stale-state clobber regression: file keeps the out-of-band "
                 "y_up/theta0 AND gets the new tx -- OK"
              << std::endl;

    // The GUI itself must also reflect the FINAL on-disk truth after Apply
    // (load_calibration_fields() runs at the end of on_apply_calibration()),
    // not merely whatever the user happened to type into tx.
    TEST_ASSERT(std::fabs(tab->calib_tx_spin()->value() - new_tx) < 1e-6);
    TEST_ASSERT(std::fabs(tab->calib_theta_deg_spin()->value() -
                           out_of_band_theta0 * 180.0 / M_PI) < 1e-3);
    TEST_ASSERT(tab->calib_y_up_checkbox()->isChecked() == out_of_band_y_up);
    std::cout << "[test]   stale-state clobber regression: GUI fields reflect final on-disk "
                 "truth after Apply -- OK"
              << std::endl;
  }

  // --- Disconnect -> Disconnected again. ---
  window.manager().mocap_manager().stop_bridge();
  window.refresh_display();
  TEST_ASSERT(window.manager().mocap_manager().state() == simviz::MocapState::Disconnected);
  TEST_ASSERT(tab->state_label()->text().toStdString().rfind("DISCONNECTED", 0) == 0);
  TEST_ASSERT(tab->uptime_label()->text().isEmpty());
  std::cout << "[test]   disconnect -> Disconnected, uptime cleared -- OK" << std::endl;

  fake_motive_guard.terminate();
  std::filesystem::remove_all(tmp_dir);

  std::cout << "[Test] (b) PASSED" << std::endl;
}

// Part B2 (BUG fixes): MocapTab with NO scenario loaded at all -- the
// user's exact "restart the UI, reconnect, mapping table stays empty"
// report, plus the rigid-body table's "we only need the names come from
// Motive" report. fake_motive serves a body named "my block" (WITH a
// space) alongside the aliased "mushr2" specifically so a passing test
// actually distinguishes the fix from the old behavior: auto-discovery's
// sanitize_topic_name() turns "my block" into the topic-safe "my_block",
// so the OLD body_table_/mapping_table_ code (which reverse-guessed
// identity from the published/sanitized name) would have shown "my_block"
// -- this asserts the REAL Motive name "my block" (with its space intact)
// appears instead.
void test_mocap_tab_no_scenario()
{
  std::cout << "[Test] (b2) MocapTab with NO scenario loaded (BUG report repro)..." << std::endl;

  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() /
      ("mars_simviz_gui_test_noscenario_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  const std::filesystem::path map_config_path = tmp_dir / "map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe,
                          {
                              "--target-ip",          "127.0.0.1",
                              "--target-port",        std::to_string(kBridgeLocalDataPort),
                              "--rate",               "60",
                              "--natnet-version",     "3.1",
                              "--body-ids",            "1,2",
                              "--body-names",           "mushr2,my block",
                              "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
                              "--served-app-name",     "TestMotiveGuiNoScenario",
                              "--served-version",      "3.1",
                              "--duration-s",          "300",
                          }),
      "fake_motive[gui-no-scenario]");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid()
             << " serving 'mushr2' (aliased->robot1) and 'my block' (unaliased, has a space)"
            << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  simviz::SimVizConfig config;
  config.control_port = 45604; // unused (start() not called)
  config.handshake_port_start = 45725; // TEST PORT BLOCK -- never mars_sim_viz's real defaults
  config.vesc_port_start = 45745;
  config.loc_port_start = 45765;
  config.run_dir = (tmp_dir / "runs").string();
  // BUG FIX regression: on_save_mapping() used to write real_robot_endpoints
  // via save_real_robots_config() UNCONDITIONALLY, even in ScenarioLess mode
  // (no VESC column exists here at all) -- an unset/unwritable path then
  // popped a blocking QMessageBox::warning() on every Save Mapping click
  // regardless of whether anything VESC-related was ever edited. Now gated
  // on an actual VESC-endpoint edit (see on_save_mapping()'s any_vesc_edit
  // doc comment), so this path is set here only to prove the OPPOSITE: that
  // the file is never even created below (mapping-only saves in
  // ScenarioLess mode must not touch it at all).
  config.real_robots_config_path = (tmp_dir / "real_robots.json").string();
  config.mocap.map_config_path = map_config_path.string();
  config.mocap.mocap_config_path = mocap_config_path.string();
  config.mocap.server_ip = "127.0.0.1";
  config.mocap.server_ip_explicit = true;
  config.mocap.mode = "unicast";
  config.mocap.mode_explicit = true;
  config.mocap.command_port = kFakeMotiveCommandPort;
  config.mocap.command_port_explicit = true;
  config.mocap.local_data_port = kBridgeLocalDataPort;
  config.mocap.local_data_port_explicit = true;
  config.mocap.loc_port_start = kBridgeLocPortStart;

  simviz::SimVizWindow window(config);
  // Deliberately NO scenario loaded -- no canvas()->set_scenario(), no
  // execution_manager().start_execution() -- this is the exact "just
  // restarted the UI, haven't (re)loaded a scenario yet" state the user's
  // report describes.
  TEST_ASSERT(window.manager().execution_manager().scenario_model() == nullptr);

  simviz::MocapTab *tab = window.mocap_tab();
  TEST_ASSERT(tab != nullptr);

  window.refresh_display();
  TEST_ASSERT(tab->mapping_table()->rowCount() == 0);
  std::cout << "[test]   pre-connect, no scenario: mapping table has 0 rows (nothing detected "
               "yet) -- OK"
            << std::endl;

  // --- Connect. ---
  {
    std::string err;
    TEST_ASSERT(window.manager().mocap_manager().start_bridge(&err));
  }
  bool reached_connected = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (std::chrono::steady_clock::now() < deadline)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    if (window.manager().mocap_manager().state() == simviz::MocapState::Connected)
    {
      reached_connected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  TEST_ASSERT(reached_connected);
  for (int i = 0; i < 5; ++i)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "[test]   MocapManager reached Connected (no scenario loaded throughout)"
            << std::endl;

  // --- BUG 1 fix: mapping table now has rows, keyed off DETECTED bodies. ---
  TEST_ASSERT(tab->mapping_table()->rowCount() == 2);
  std::cout << "[test]   BUG 1 fix: mapping table has " << tab->mapping_table()->rowCount()
            << " rows with NO scenario loaded (used to be permanently 0) -- OK" << std::endl;

  auto find_mapping_row = [&](const std::string &motive_name) -> int
  {
    for (int row = 0; row < tab->mapping_table()->rowCount(); ++row)
    {
      QTableWidgetItem *item = tab->mapping_table()->item(row, 0);
      if (item && item->text().toStdString() == motive_name)
        return row;
    }
    return -1;
  };

  const int mushr2_row = find_mapping_row("mushr2");
  const int my_block_row = find_mapping_row("my block");
  TEST_ASSERT(mushr2_row >= 0);
  TEST_ASSERT(my_block_row >= 0);
  std::cout << "[test]   mapping table row identities are REAL Motive names ('mushr2', 'my "
               "block' -- with its space) -- OK"
            << std::endl;

  // MAPPING DROPDOWN: planner-name column is now a fixed-choice QComboBox
  // (""/robot1/robot2/robot3/robot4), never a free-text QLineEdit.
  QComboBox *mushr2_alias_combo =
      qobject_cast<QComboBox *>(tab->mapping_table()->cellWidget(mushr2_row, 1));
  TEST_ASSERT(mushr2_alias_combo != nullptr);
  TEST_ASSERT(mushr2_alias_combo->currentText().toStdString() == "robot1");
  std::cout << "[test]   'mushr2' row: alias pre-selected from mocap_map_config.json -> 'robot1' "
               "-- OK"
            << std::endl;

  QComboBox *my_block_alias_combo =
      qobject_cast<QComboBox *>(tab->mapping_table()->cellWidget(my_block_row, 1));
  TEST_ASSERT(my_block_alias_combo != nullptr);
  TEST_ASSERT(my_block_alias_combo->currentText().isEmpty()); // index 0 == "" == unmapped.
  std::cout << "[test]   'my block' row: no configured alias -> unmapped (\"\"), fixed-choice "
               "combo -- OK"
            << std::endl;

  // --- BUG 2 fix: rigid-body table shows REAL Motive names, never the
  // published/sanitized guess ("my_block"). ---
  TEST_ASSERT(tab->body_table()->rowCount() == 2);
  bool body_table_has_real_space_name = false;
  bool body_table_has_sanitized_name = false;
  for (int row = 0; row < tab->body_table()->rowCount(); ++row)
  {
    QTableWidgetItem *item = tab->body_table()->item(row, 0);
    TEST_ASSERT(item != nullptr);
    const std::string name = item->text().toStdString();
    if (name == "my block")
      body_table_has_real_space_name = true;
    if (name == "my_block")
      body_table_has_sanitized_name = true;
  }
  TEST_ASSERT(body_table_has_real_space_name);
  TEST_ASSERT(!body_table_has_sanitized_name);
  std::cout << "[test]   BUG 2 fix: rigid-body table shows the REAL Motive name 'my block' "
               "(never the sanitized 'my_block' the OLD code would have guessed) -- OK"
            << std::endl;

  // --- In-progress selection change must survive further refresh() calls
  // (never clobbered by a row that didn't structurally change) -- same
  // "additive rebuild never touches an existing row's widget" contract the
  // old free-text QLineEdit test exercised, now against the fixed-choice
  // combo's SELECTION instead of typed text. ---
  const int kInProgressIndex = my_block_alias_combo->findText("robot2");
  TEST_ASSERT(kInProgressIndex > 0);
  my_block_alias_combo->setCurrentIndex(kInProgressIndex);
  for (int i = 0; i < 5; ++i)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  QComboBox *my_block_alias_combo_after =
      qobject_cast<QComboBox *>(tab->mapping_table()->cellWidget(my_block_row, 1));
  TEST_ASSERT(my_block_alias_combo_after == my_block_alias_combo); // same widget instance -- row
                                                                     // structure untouched.
  TEST_ASSERT(my_block_alias_combo_after->currentText().toStdString() == "robot2");
  std::cout << "[test]   in-progress combo selection ('robot2') on 'my block' survives 5 more "
               "refresh() calls unclobbered -- OK"
            << std::endl;

  // --- Change 1 (per-row yaw-offset column): the column now renders as an
  // editable QDoubleSpinBox (default 0.0 deg -- synthetic_map_config_json()'s
  // "yaw_offset" starts as {}), per-row dirty-tracked the same way the
  // Calibration group's fields are (see on_save_mapping()'s doc comment).
  // Edit ONLY 'mushr2' row's spinbox (its alias is already 'robot1' from the
  // preset), leave 'my block' row's untouched, and pre-seed an out-of-band
  // on-disk entry for a THIRD robot no row touches this session at all --
  // Save Mapping below must write only 'robot1', leave no entry for
  // 'my block''s eventual alias, and preserve the out-of-band entry
  // byte-for-byte (reload-then-merge, per row). ---
  QDoubleSpinBox *mushr2_yaw_spin =
      qobject_cast<QDoubleSpinBox *>(tab->mapping_table()->cellWidget(mushr2_row, 2));
  QDoubleSpinBox *my_block_yaw_spin =
      qobject_cast<QDoubleSpinBox *>(tab->mapping_table()->cellWidget(my_block_row, 2));
  TEST_ASSERT(mushr2_yaw_spin != nullptr);
  TEST_ASSERT(my_block_yaw_spin != nullptr);
  // 'mushr2' is pre-aliased to 'robot1' (synthetic_map_config_json()'s
  // "aliases"), which already has a "yaw_offset" entry (0.1 rad) -- the
  // spinbox's default must reflect that, converted to degrees. 'my block'
  // has no alias at all (unmapped), so its default is 0.0 (no robot name to
  // look a yaw_offset up by yet).
  const double kMushr2SeedYawDeg = 0.1 * 180.0 / M_PI;
  TEST_ASSERT(std::fabs(mushr2_yaw_spin->value() - kMushr2SeedYawDeg) < 1e-3);
  TEST_ASSERT(std::fabs(my_block_yaw_spin->value() - 0.0) < 1e-6);
  std::cout << "[test]   yaw-offset column: renders as an editable QDoubleSpinBox, defaults "
               "reflect each row's looked-up yaw_offset ('mushr2'/robot1="
            << kMushr2SeedYawDeg << "deg, 'my block' (unmapped)=0deg) -- OK" << std::endl;

  {
    nlohmann::json cfg;
    std::ifstream f(map_config_path);
    f >> cfg;
    cfg["yaw_offset"]["robot4"] = 0.77; // out-of-band entry no row touches this session.
    std::ofstream of(map_config_path);
    of << cfg.dump(2);
  }

  mushr2_yaw_spin->setValue(12.5); // user edit -> row dirty.
  // my_block_yaw_spin intentionally left untouched (stays at its default).

  // --- Save Mapping (while Disconnected, so on_save_mapping()'s "restart "
  // the bridge?" QMessageBox::question() -- which would otherwise block "
  // this headless test on a modal dialog -- never fires). motive_assets() "
  // / bodies() are retained after stop_bridge() (this class's own "leave "
  // the last-known data queryable" convention), so the rows survive too. ---
  window.manager().mocap_manager().stop_bridge();
  window.refresh_display();
  TEST_ASSERT(tab->mapping_table()->rowCount() == 2); // rows retained across disconnect.
  QComboBox *my_block_alias_combo_disc =
      qobject_cast<QComboBox *>(tab->mapping_table()->cellWidget(my_block_row, 1));
  TEST_ASSERT(my_block_alias_combo_disc != nullptr);
  const int final_index = my_block_alias_combo_disc->findText("robot3");
  TEST_ASSERT(final_index > 0);
  my_block_alias_combo_disc->setCurrentIndex(final_index); // final choice, overrides 'robot2' above.
  tab->save_mapping_button()->click(); // synchronously runs on_save_mapping().

  {
    nlohmann::json saved_cfg;
    std::ifstream f(map_config_path);
    f >> saved_cfg;
    TEST_ASSERT(saved_cfg.contains("aliases"));
    TEST_ASSERT(saved_cfg["aliases"]["my block"] == "robot3");
    TEST_ASSERT(saved_cfg["aliases"]["mushr2"] == "robot1"); // untouched alias preserved.
  }
  std::cout << "[test]   Save Mapping (scenario-less): 'my block' -> 'robot3' alias written to "
               "map-config on disk, 'mushr2' -> 'robot1' preserved -- OK"
            << std::endl;

  // --- Change 1 assertions: only the touched row's yaw_offset changed on
  // disk; 'my block' -> 'robot3' got no yaw_offset entry at all (that row's
  // spinbox was never touched, so on_save_mapping() never calls
  // set_yaw_offset() for it); the out-of-band 'robot4' entry pre-seeded
  // above survives byte-for-byte (reload-then-merge). ---
  {
    nlohmann::json saved_cfg;
    std::ifstream f(map_config_path);
    f >> saved_cfg;
    TEST_ASSERT(saved_cfg.contains("yaw_offset"));
    TEST_ASSERT(saved_cfg["yaw_offset"].contains("robot1"));
    TEST_ASSERT(std::fabs(saved_cfg["yaw_offset"]["robot1"].get<double>() -
                           (12.5 * M_PI / 180.0)) < 1e-6);
    TEST_ASSERT(!saved_cfg["yaw_offset"].contains("robot3"));
    TEST_ASSERT(saved_cfg["yaw_offset"].contains("robot4"));
    TEST_ASSERT(std::fabs(saved_cfg["yaw_offset"]["robot4"].get<double>() - 0.77) < 1e-9);
  }
  std::cout << "[test]   Save Mapping: only 'robot1' (the touched row) got a new yaw_offset "
               "(12.5deg), 'robot3' got none (untouched row), out-of-band 'robot4' entry "
               "preserved -- OK"
            << std::endl;

  // The spinbox itself must be re-synced to the saved value (in place, no
  // row-recreation) and its dirty flag cleared -- a SECOND click with no
  // further edits must not re-write anything (nothing left dirty).
  TEST_ASSERT(std::fabs(mushr2_yaw_spin->value() - 12.5) < 1e-3);
  std::cout << "[test]   yaw-offset spinbox re-synced to the saved value after Save Mapping -- OK"
            << std::endl;

  // --- BUG FIX regression: ScenarioLess mode has no VESC column at all, so
  // Save Mapping must NEVER write real_robots_config_path -- the file must
  // not even exist (see this test's real_robots_config_path doc comment
  // above for the OLD unconditional-write bug this proves is fixed). ---
  TEST_ASSERT(!std::filesystem::exists(config.real_robots_config_path));
  std::cout << "[test]   BUG FIX: real-robots config was NOT written by a scenario-less mapping "
               "save (no VESC endpoint exists to save) -- OK"
            << std::endl;

  // --- Reconnect against a DIFFERENT fake_motive serving fewer bodies ->
  // the row for the body a fresh modeldef no longer reports is REMOVED. ---
  fake_motive_guard = simviz_test::ProcessGuard(
      simviz_test::spawn(fake_motive_exe,
                          {
                              "--target-ip",          "127.0.0.1",
                              "--target-port",        std::to_string(kBridgeLocalDataPort),
                              "--rate",               "60",
                              "--natnet-version",     "3.1",
                              "--body-ids",            "1",
                              "--body-names",           "mushr2",
                              "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
                              "--served-app-name",     "TestMotiveGuiNoScenario2",
                              "--served-version",      "3.1",
                              "--duration-s",          "300",
                          }),
      "fake_motive[gui-no-scenario-2]");
  std::cout << "[test]   swapped to a second fake_motive serving ONLY 'mushr2' (pid="
             << fake_motive_guard.pid() << ")" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  {
    std::string err;
    TEST_ASSERT(window.manager().mocap_manager().start_bridge(&err));
  }
  reached_connected = false;
  const auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (std::chrono::steady_clock::now() < deadline2)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    if (window.manager().mocap_manager().state() == simviz::MocapState::Connected)
    {
      reached_connected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  TEST_ASSERT(reached_connected);
  for (int i = 0; i < 5; ++i)
  {
    window.manager().mocap_manager().tick();
    window.refresh_display();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  TEST_ASSERT(tab->mapping_table()->rowCount() == 1);
  TEST_ASSERT(tab->body_table()->rowCount() == 1);
  TEST_ASSERT(find_mapping_row("mushr2") >= 0);
  TEST_ASSERT(find_mapping_row("my block") < 0); // row removed -- no longer in the fresh modeldef.
  std::cout << "[test]   reconnect with a smaller inventory: mapping/body tables shrink to 1 row "
               "('my block' row removed) -- OK"
            << std::endl;

  window.manager().mocap_manager().stop_bridge();
  fake_motive_guard.terminate();
  std::filesystem::remove_all(tmp_dir);

  std::cout << "[Test] (b2) PASSED" << std::endl;
}

// Part C: MocapTransformWidget draws each live body as an ORIENTED ARROW
// (not a plain dot) -- proven here without needing a live bridge at all:
// two renders of the SAME widget class at the SAME position but different
// yaw must differ in their pixel content, which would NOT be true of a
// rotation-agnostic dot.
void test_transform_widget_arrow_orientation()
{
  std::cout << "[Test] (c) MocapTransformWidget draws oriented arrows for live bodies..."
            << std::endl;

  simviz::TransformInfo info; // identity matrix (default) -- irrelevant to this check.

  simviz::MocapTransformWidget widget_yaw0;
  widget_yaw0.set_data(info, {{"robot1", Pose(0.5, 0.5, 0.0)}});
  const QImage img_yaw0 = grab_widget(widget_yaw0, 300, 300);

  simviz::MocapTransformWidget widget_yaw90;
  widget_yaw90.set_data(info, {{"robot1", Pose(0.5, 0.5, M_PI / 2.0)}});
  const QImage img_yaw90 = grab_widget(widget_yaw90, 300, 300);

  TEST_ASSERT(img_yaw0.size() == img_yaw90.size());
  int differing_pixels = 0;
  for (int y = 0; y < img_yaw0.height(); ++y)
  {
    for (int x = 0; x < img_yaw0.width(); ++x)
    {
      if (img_yaw0.pixel(x, y) != img_yaw90.pixel(x, y))
        ++differing_pixels;
    }
  }
  std::cout << "[test]   yaw=0 vs yaw=90deg render at the same position: " << differing_pixels
            << " differing pixels" << std::endl;
  // An oriented arrow rotated 90deg sweeps a visibly different set of
  // pixels than the same arrow at yaw=0 (a rotation-agnostic dot would
  // show ZERO differing pixels here).
  TEST_ASSERT(differing_pixels > 10);

  const std::string out_path_yaw0 =
      std::string(MARS_TEST_RENDER_OUT_DIR) + "/test_sim_viz_mocap_arrow_yaw0.png";
  const std::string out_path_yaw90 =
      std::string(MARS_TEST_RENDER_OUT_DIR) + "/test_sim_viz_mocap_arrow_yaw90.png";
  TEST_ASSERT(img_yaw0.save(QString::fromStdString(out_path_yaw0)));
  TEST_ASSERT(img_yaw90.save(QString::fromStdString(out_path_yaw90)));
  std::cout << "[test]   saved " << out_path_yaw0 << " and " << out_path_yaw90 << std::endl;

  std::cout << "[Test] (c) PASSED" << std::endl;
}

// Part D (Change 2): MocapTransformWidget's Motive-axes glyph draws Motive's
// OWN FIXED viewport convention (+X right, +Z down-screen) -- INDEPENDENT of
// TransformInfo::y_up, which only governs live pose-data mapping, not this
// glyph. The +Y(up) out-of-plane circled-dot marker this glyph used to also
// draw was removed (Motive +X/+Z arrows are unaffected) -- see this file's
// removed sub-test (3), formerly asserting that marker's presence. Colors
// below mirror MocapTab.cpp's private kMotiveAxisXColor/kMotiveAxisYColor/
// kPlannerAxisXColor constants exactly (not exported -- this test
// re-declares the same hex values rather than exposing them, same as this
// file's other "reconstruct via observable rendering" tests).
void test_transform_widget_motive_axes_glyph()
{
  std::cout << "[Test] (d) MocapTransformWidget Motive-axes glyph (Change 2)..." << std::endl;

  const QColor kPlannerAxisXColor("#D5896F");
  const QColor kMotiveAxisXColor("#CC79A7");
  const QColor kMotiveAxisYColor("#56B4E9");  // Motive's +Z in-plane arrow.
  constexpr int kTol = 25;

  // --- (1) y_up must have NO effect on this glyph at identity calibration
  // (no bodies -> y_up is otherwise irrelevant to anything else drawn). ---
  {
    simviz::TransformInfo info_zup; // default AffineTransform2D == identity.
    info_zup.y_up = false;
    simviz::MocapTransformWidget widget_zup;
    widget_zup.set_data(info_zup, {});
    const QImage img_zup = grab_widget(widget_zup, 300, 300);

    simviz::TransformInfo info_yup = info_zup;
    info_yup.y_up = true;
    simviz::MocapTransformWidget widget_yup;
    widget_yup.set_data(info_yup, {});
    const QImage img_yup = grab_widget(widget_yup, 300, 300);

    TEST_ASSERT(img_zup.size() == img_yup.size());
    int differing = 0;
    for (int y = 0; y < img_zup.height(); ++y)
      for (int x = 0; x < img_zup.width(); ++x)
        if (img_zup.pixel(x, y) != img_yup.pixel(x, y))
          ++differing;
    std::cout << "[test]   y_up=false vs y_up=true render (identity calibration): " << differing
               << " differing pixels (expected 0 -- axes glyph no longer conditions on y_up)"
              << std::endl;
    TEST_ASSERT(differing == 0);

    // --- (2) at theta=0 (identity), Motive's +Z in-plane arrow must draw
    // BELOW (screen-Y greater than) the origin -- located here via the
    // planner X-axis color (always horizontal through the origin, since the
    // planner frame is never affected by calibration) as the origin's
    // screen-Y reference. ---
    bool found_planner_x = false, found_motive_z = false, found_motive_x = false;
    const auto planner_x_range = find_color_y_range(img_zup, kPlannerAxisXColor, kTol, &found_planner_x);
    const auto motive_z_range = find_color_y_range(img_zup, kMotiveAxisYColor, kTol, &found_motive_z);
    const auto motive_x_range = find_color_y_range(img_zup, kMotiveAxisXColor, kTol, &found_motive_x);
    (void)motive_x_range;
    TEST_ASSERT(found_planner_x);
    TEST_ASSERT(found_motive_z);
    TEST_ASSERT(found_motive_x);
    const int origin_y = planner_x_range.first; // horizontal line -> min==max==origin's screen-Y.
    std::cout << "[test]   origin screen-Y (via planner +X axis)=" << origin_y
              << ", Motive +Z color Y-range=[" << motive_z_range.first << "," << motive_z_range.second
              << "]" << std::endl;
    // The dashed Z line's LOWEST (max) Y must sit clearly below the origin.
    TEST_ASSERT(motive_z_range.second > origin_y + 4);
    std::cout << "[test]   Motive +Z arrow endpoint draws below the origin at theta=0, regardless "
                 "of y_up -- OK"
              << std::endl;

    const std::string out_path =
        std::string(MARS_TEST_RENDER_OUT_DIR) + "/test_sim_viz_mocap_axes_glyph_identity.png";
    TEST_ASSERT(img_zup.save(QString::fromStdString(out_path)));
    std::cout << "[test]   saved " << out_path << std::endl;
  }

  // --- (4) the glyph still rotates with a non-zero calibration theta: a
  // 90deg calibration rotation must produce a visibly different render than
  // identity (only the calibration matrix's effect changed, per Change 2's
  // "rendering-only, calibration math untouched" contract). ---
  {
    simviz::TransformInfo info_identity;
    simviz::MocapTransformWidget widget_identity;
    widget_identity.set_data(info_identity, {});
    const QImage img_identity = grab_widget(widget_identity, 300, 300);

    simviz::TransformInfo info_rot90;
    info_rot90.matrix = simviz::AffineTransform2D{0.0, -1.0, 0.0, 1.0, 0.0, 0.0}; // +90deg rotation.
    info_rot90.rotation_deg = 90.0;
    simviz::MocapTransformWidget widget_rot90;
    widget_rot90.set_data(info_rot90, {});
    const QImage img_rot90 = grab_widget(widget_rot90, 300, 300);

    TEST_ASSERT(img_identity.size() == img_rot90.size());
    int differing = 0;
    for (int y = 0; y < img_identity.height(); ++y)
      for (int x = 0; x < img_identity.width(); ++x)
        if (img_identity.pixel(x, y) != img_rot90.pixel(x, y))
          ++differing;
    std::cout << "[test]   identity vs 90deg-calibration-rotation render: " << differing
               << " differing pixels" << std::endl;
    TEST_ASSERT(differing > 10);
    std::cout << "[test]   Motive-axes glyph still rotates with a non-zero calibration theta -- OK"
              << std::endl;
  }

  std::cout << "[Test] (d) PASSED" << std::endl;
}

} // namespace

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  test_window_tab_structure();
  test_mocap_tab_live_render();
  test_mocap_tab_no_scenario();
  test_transform_widget_arrow_orientation();
  test_transform_widget_motive_axes_glyph();

  std::cout << "\n[Test] All test_sim_viz_gui tests passed." << std::endl;
  return 0;
}
