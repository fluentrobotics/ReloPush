// Offscreen GUI test for simviz::CalibrationTab (teleop-driven velocity/
// steering calibration map capture -- see MARS/src/simviz/CalibrationTab.h's
// own header comment for the full design). Unlike test_sim_viz_gui.cpp's
// Mocap-tab coverage, this test spawns NO real process/socket at all: it
// substitutes RecordingTeleopSink (below) for the real ZMQ TeleopWorker via
// CalibrationTab::set_command_sink_for_test(), and drives mocap/telemetry
// ingestion directly via feed_pose_for_test()/feed_telemetry_for_test() --
// exactly the testability seams the tab was designed around, so this binary
// needs no fake_motive/optitrack_zmq_bridge/vesc_driver child processes.
//
// Run under QT_QPA_PLATFORM=offscreen (standalone binary, no add_test(), not
// installed -- same convention as test_sim_viz_gui.cpp/test_sim_viz_mocap.cpp).
//
// Covers:
//   (a) synthetic pose+telemetry feeding advances coverage and updates the
//       instruction label.
//   (b) W press -> duty command with the expected value/sign, release ->
//       stop; the "invert drive keys" checkbox flips the sign.
//   (c) a current-abort breach sends stop and disables teleop (checkbox +
//       internal state + the "shutdown_blocking" guard sequence).
//   (d) switching Task (Velocity map <-> Steering map) creates a fresh
//       session/core (new CalibrationCore instance, zero samples).

#include "CalibrationTab.h"
#include "SimVizWindow.h"

#include <mpc/CalibrationCore.h>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QKeyEvent>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

// Required by SafeParking.cpp/StagingCore.cpp (linked in via
// PHASTAR_SHARED_RELOPUSH_SOURCES's dependency chain) -- same convention
// every other test_sim_viz_*.cpp binary in this suite follows (see e.g.
// test_sim_viz_gui.cpp's own `bool DEBUG_VIS = false;`).
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

// ===========================================================================
// RecordingTeleopSink: the "recording stub" this tab's design set out for --
// no sockets at all, just records every call so the test can assert on
// exactly what CalibrationTab decided to send.
// ===========================================================================
class RecordingTeleopSink : public simviz::ITeleopCommandSink
{
public:
  struct RawCall
  {
    double value = 0.0;
    int ttl_ms = 0;
  };

  void ping() override { ++ping_count; }
  void set_source(const std::string &source) override { set_source_calls.push_back(source); }
  void raw_duty(double value, int ttl_ms) override { raw_calls.push_back({value, ttl_ms}); }
  void servo(double value) override { servo_calls.push_back(value); }
  void stop() override { ++stop_count; }
  void shutdown_blocking() override
  {
    ++shutdown_blocking_count;
    // Mirrors the real guard sequence's effect (stop then set_source
    // ackermann) so a test asserting on set_source_calls' tail sees exactly
    // what the real ITeleopCommandSink contract promises.
    ++stop_count;
    set_source_calls.push_back("ackermann");
  }

  int ping_count = 0;
  int stop_count = 0;
  int shutdown_blocking_count = 0;
  std::vector<std::string> set_source_calls;
  std::vector<RawCall> raw_calls;
  std::vector<double> servo_calls;
};

// Creates a temp dir with a minimal synthetic mocap map-config (never the
// real project's MPC/config/mocap_map_config.json -- same "tests own their
// config copy" rule every other simviz test in this suite follows) and a
// SimVizConfig pointing at it. control_port is set but never used (start()
// is never called -- no socket is ever bound), mirroring test_sim_viz_gui.cpp's
// test_window_tab_structure()'s own "unused" comment.
std::filesystem::path make_tmp_dir(const std::string &suffix)
{
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("mars_simviz_calib_test_" + suffix + "_" + std::to_string(getpid()));
  std::filesystem::create_directories(dir);
  return dir;
}

simviz::SimVizConfig make_test_config(const std::filesystem::path &tmp_dir, int control_port)
{
  const std::filesystem::path map_config_path = tmp_dir / "map_config.json";
  {
    nlohmann::json j;
    j["_doc"] = "test-only synthetic mocap map-config for test_sim_viz_calibration_tab";
    j["y_up"] = false;
    std::ofstream f(map_config_path);
    f << j.dump(2);
  }

  simviz::SimVizConfig config;
  config.control_port = control_port; // unused -- window.start() is never called in this test.
  config.mocap.map_config_path = map_config_path.string();
  return config;
}

// Simulates ~2s of steady forward driving at a constant commanded duty and a
// constant (matching) mocap velocity, directly through CalibrationTab's
// test-injection points plus CalibrationCore::feed_command() (reachable via
// CalibrationTab::core()) -- enough to clear CalibrationCore's steadiness
// gate (0.5s command-hold + velocity-estimator warmup, see
// MPC/src/CalibrationCore.cpp's feed_pose()) and accumulate several accepted
// samples in the v=0.15 m/s velocity bin.
void feed_steady_forward_drive(simviz::CalibrationTab &tab, double v_mps, double duty_value,
                                double servo_value, double seconds)
{
  mpc::CalibrationCore *core = tab.core();
  TEST_ASSERT(core != nullptr);

  constexpr double kDt = 0.02; // 50 Hz.
  double t = 0.0;
  double x = 0.0;
  const int n = static_cast<int>(seconds / kDt);
  for (int i = 0; i < n; ++i)
  {
    t += kDt;
    x += v_mps * kDt;
    core->feed_command(t, "duty", duty_value, servo_value);
    tab.feed_telemetry_for_test(t, /*erpm=*/3000.0, /*duty=*/duty_value, /*current_motor=*/2.0,
                                 /*current_in=*/1.5, /*v_in=*/12.0);
    tab.feed_pose_for_test(t, x, /*y=*/0.0, /*yaw=*/0.0);
  }
}

// ---------------------------------------------------------------------
// (a) synthetic pose+telemetry feeding advances coverage / instruction.
// ---------------------------------------------------------------------
void test_coverage_advances_with_synthetic_feed()
{
  std::cout << "[Test] (a) synthetic pose+telemetry feeding advances coverage..." << std::endl;

  const std::filesystem::path tmp_dir = make_tmp_dir("coverage");
  simviz::SimVizConfig config = make_test_config(tmp_dir, 45901);
  simviz::SimVizWindow window(config);
  // Offscreen QPA still needs an actual show() for QWidget::isVisible() to
  // reflect reality (isVisible() requires the whole ancestor chain up to the
  // top-level window to have been shown; a merely-constructed-but-never-shown
  // window reports every descendant as not visible regardless of their own
  // setVisible(true) calls -- see abort_banner()'s assertions below).
  window.show();

  simviz::CalibrationTab *tab = window.calibration_tab();
  TEST_ASSERT(tab != nullptr);
  TEST_ASSERT(tab->core() != nullptr);
  TEST_ASSERT(tab->core()->total_samples() == 0);
  TEST_ASSERT(tab->core()->accepted_samples() == 0);

  RecordingTeleopSink sink;
  tab->set_command_sink_for_test(&sink);

  // v=0.15 m/s lands exactly on a velocity bin center (v_min=0.05,
  // v_step=0.05 -> centers ..., 0.10, 0.15, 0.20, ...) -- see
  // MPC/include/mpc/CalibrationCore.h's VelocityCoverageConfig doc comment.
  feed_steady_forward_drive(*tab, /*v_mps=*/0.15, /*duty_value=*/0.30, /*servo_value=*/0.5,
                             /*seconds=*/2.0);

  TEST_ASSERT(tab->core()->total_samples() > 0);
  TEST_ASSERT(tab->core()->accepted_samples() > 0);
  std::cout << "[test]   total=" << tab->core()->total_samples()
            << " accepted=" << tab->core()->accepted_samples() << std::endl;

  window.refresh_display(); // pumps CalibrationTab::refresh() -> connection lights etc.

  TEST_ASSERT(!tab->instruction_label()->text().isEmpty());
  TEST_ASSERT(tab->coverage_counts_label()->text().contains("Accepted"));
  TEST_ASSERT(!tab->coverage_counts_label()->text().contains("Accepted 0 /"));
  std::cout << "[test]   instruction label: '" << tab->instruction_label()->text().toStdString()
            << "'" << std::endl;
  std::cout << "[test]   coverage counts label: '"
            << tab->coverage_counts_label()->text().toStdString() << "'" << std::endl;

  std::filesystem::remove_all(tmp_dir);
  std::cout << "[Test] (a) PASSED" << std::endl;
}

// ---------------------------------------------------------------------
// (b) W press -> duty command (expected value/sign), release -> stop;
//     invert-drive checkbox flips the sign.
// ---------------------------------------------------------------------
void test_keyboard_drive_commands()
{
  std::cout << "[Test] (b) keyboard W/S drive commands + invert..." << std::endl;

  const std::filesystem::path tmp_dir = make_tmp_dir("keys");
  simviz::SimVizConfig config = make_test_config(tmp_dir, 45902);
  simviz::SimVizWindow window(config);
  // Offscreen QPA still needs an actual show() for QWidget::isVisible() to
  // reflect reality (isVisible() requires the whole ancestor chain up to the
  // top-level window to have been shown; a merely-constructed-but-never-shown
  // window reports every descendant as not visible regardless of their own
  // setVisible(true) calls -- see abort_banner()'s assertions below).
  window.show();
  simviz::CalibrationTab *tab = window.calibration_tab();
  TEST_ASSERT(tab != nullptr);

  RecordingTeleopSink sink;
  tab->set_command_sink_for_test(&sink);

  tab->teleop_enable_checkbox()->setChecked(true); // -> on_teleop_toggle(true) -> enable_teleop().
  TEST_ASSERT(tab->teleop_enabled());
  TEST_ASSERT(sink.ping_count >= 1);
  TEST_ASSERT(!sink.set_source_calls.empty());
  TEST_ASSERT(sink.set_source_calls.back() == "calib");
  std::cout << "[test]   Enable teleop -> ping() + set_source('calib') -- OK" << std::endl;

  const double expected_duty = tab->duty_magnitude();
  TEST_ASSERT(expected_duty > 0.0);

  QKeyEvent press_w(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier);
  QApplication::sendEvent(tab, &press_w);
  TEST_ASSERT(!sink.raw_calls.empty());
  TEST_ASSERT(std::fabs(sink.raw_calls.back().value - expected_duty) < 1e-9);
  TEST_ASSERT(sink.raw_calls.back().ttl_ms == 500);
  std::cout << "[test]   W press -> raw duty command value=" << sink.raw_calls.back().value
            << " ttl_ms=" << sink.raw_calls.back().ttl_ms << " -- OK" << std::endl;

  const size_t stop_count_before_release = sink.stop_count;
  QKeyEvent release_w(QEvent::KeyRelease, Qt::Key_W, Qt::NoModifier);
  QApplication::sendEvent(tab, &release_w);
  TEST_ASSERT(sink.stop_count > stop_count_before_release);
  std::cout << "[test]   W release -> stop() sent immediately -- OK" << std::endl;

  // Auto-repeat key-press events must be ignored entirely (no new command).
  const size_t raw_calls_before_autorepeat = sink.raw_calls.size();
  QKeyEvent autorepeat_w(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier, QString(), /*autorep=*/true);
  QApplication::sendEvent(tab, &autorepeat_w);
  TEST_ASSERT(sink.raw_calls.size() == raw_calls_before_autorepeat);
  std::cout << "[test]   auto-repeat W press ignored -- OK" << std::endl;

  // Invert drive keys -> W now sends the OPPOSITE sign.
  tab->invert_drive_checkbox()->setChecked(true);
  QApplication::sendEvent(tab, &press_w);
  TEST_ASSERT(!sink.raw_calls.empty());
  TEST_ASSERT(std::fabs(sink.raw_calls.back().value - (-expected_duty)) < 1e-9);
  std::cout << "[test]   invert-drive checked -> W press now sends value="
            << sink.raw_calls.back().value << " -- OK" << std::endl;
  QApplication::sendEvent(tab, &release_w);

  // S (backward) with invert still on -> sign flips back to +duty.
  QKeyEvent press_s(QEvent::KeyPress, Qt::Key_S, Qt::NoModifier);
  QApplication::sendEvent(tab, &press_s);
  TEST_ASSERT(std::fabs(sink.raw_calls.back().value - expected_duty) < 1e-9);
  std::cout << "[test]   invert-drive checked -> S press sends value=" << sink.raw_calls.back().value
            << " -- OK" << std::endl;
  QKeyEvent release_s(QEvent::KeyRelease, Qt::Key_S, Qt::NoModifier);
  QApplication::sendEvent(tab, &release_s);

  // Clean teardown -- destructor's safety net will also fire, but do it
  // explicitly here so the assertions below observe a settled state.
  tab->teleop_enable_checkbox()->setChecked(false);
  TEST_ASSERT(!tab->teleop_enabled());
  TEST_ASSERT(sink.shutdown_blocking_count >= 1);
  std::cout << "[test]   Disable teleop -> shutdown_blocking() (stop + set_source('ackermann')) -- OK"
            << std::endl;

  std::filesystem::remove_all(tmp_dir);
  std::cout << "[Test] (b) PASSED" << std::endl;
}

// ---------------------------------------------------------------------
// (c) current-abort breach sends stop and disables teleop.
// ---------------------------------------------------------------------
void test_current_abort_disables_teleop()
{
  std::cout << "[Test] (c) current-abort breach disables teleop..." << std::endl;

  const std::filesystem::path tmp_dir = make_tmp_dir("abort");
  simviz::SimVizConfig config = make_test_config(tmp_dir, 45903);
  simviz::SimVizWindow window(config);
  // Offscreen QPA still needs an actual show() for QWidget::isVisible() to
  // reflect reality (isVisible() requires the whole ancestor chain up to the
  // top-level window to have been shown; a merely-constructed-but-never-shown
  // window reports every descendant as not visible regardless of their own
  // setVisible(true) calls -- see abort_banner()'s assertions below).
  window.show();
  simviz::CalibrationTab *tab = window.calibration_tab();
  TEST_ASSERT(tab != nullptr);
  // QTabWidget only actually shows its CURRENT page -- the other tabs stay
  // hidden regardless of the top-level window's own visibility. Switch to
  // the Calibration tab so abort_banner()->isVisible() below reflects reality
  // (this also exercises the real "user is looking at this tab" path, not
  // just an implementation detail needed for the assertion to work).
  window.tabs()->setCurrentWidget(tab);

  RecordingTeleopSink sink;
  tab->set_command_sink_for_test(&sink);

  tab->teleop_enable_checkbox()->setChecked(true);
  TEST_ASSERT(tab->teleop_enabled());
  TEST_ASSERT(!tab->abort_banner()->isVisible());

  const double threshold = tab->current_abort_spin()->value();
  TEST_ASSERT(threshold > 0.0);

  // Hold W so there's an active drive command to be forcibly stopped.
  QKeyEvent press_w(QEvent::KeyPress, Qt::Key_W, Qt::NoModifier);
  QApplication::sendEvent(tab, &press_w);
  TEST_ASSERT(!sink.raw_calls.empty());

  const int stop_count_before = sink.stop_count;
  const int shutdown_before = sink.shutdown_blocking_count;

  // Telemetry frame with motor current AT the threshold -- must trip the abort.
  tab->feed_telemetry_for_test(/*t=*/1.0, /*erpm=*/4000.0, /*duty=*/0.3,
                                /*current_motor=*/threshold + 1.0, /*current_in=*/2.0,
                                /*v_in=*/12.0);

  TEST_ASSERT(!tab->teleop_enabled());
  TEST_ASSERT(!tab->teleop_enable_checkbox()->isChecked());
  TEST_ASSERT(sink.stop_count > stop_count_before);
  TEST_ASSERT(sink.shutdown_blocking_count > shutdown_before);
  TEST_ASSERT(tab->abort_banner()->isVisible());
  TEST_ASSERT(tab->abort_banner()->text().contains("CURRENT ABORT"));
  std::cout << "[test]   current_motor=" << (threshold + 1.0) << "A >= threshold=" << threshold
            << "A -> stop() + shutdown_blocking() + teleop disabled + banner visible -- OK"
            << std::endl;

  // Re-enabling clears the tripped flag and hides the banner.
  tab->teleop_enable_checkbox()->setChecked(true);
  TEST_ASSERT(tab->teleop_enabled());
  TEST_ASSERT(!tab->abort_banner()->isVisible());
  std::cout << "[test]   re-enabling teleop clears the abort banner -- OK" << std::endl;

  tab->teleop_enable_checkbox()->setChecked(false);
  std::filesystem::remove_all(tmp_dir);
  std::cout << "[Test] (c) PASSED" << std::endl;
}

// ---------------------------------------------------------------------
// (d) task switch creates a fresh session/core.
// ---------------------------------------------------------------------
void test_task_switch_creates_fresh_core()
{
  std::cout << "[Test] (d) task switch creates a fresh session/core..." << std::endl;

  const std::filesystem::path tmp_dir = make_tmp_dir("taskswitch");
  simviz::SimVizConfig config = make_test_config(tmp_dir, 45904);
  simviz::SimVizWindow window(config);
  // Offscreen QPA still needs an actual show() for QWidget::isVisible() to
  // reflect reality (isVisible() requires the whole ancestor chain up to the
  // top-level window to have been shown; a merely-constructed-but-never-shown
  // window reports every descendant as not visible regardless of their own
  // setVisible(true) calls -- see abort_banner()'s assertions below).
  window.show();
  simviz::CalibrationTab *tab = window.calibration_tab();
  TEST_ASSERT(tab != nullptr);
  TEST_ASSERT(tab->task_combo()->currentIndex() == 0); // "Velocity map" default.

  mpc::CalibrationCore *core_before = tab->core();
  TEST_ASSERT(core_before != nullptr);

  feed_steady_forward_drive(*tab, /*v_mps=*/0.15, /*duty_value=*/0.30, /*servo_value=*/0.5,
                             /*seconds=*/1.0);
  TEST_ASSERT(tab->core()->total_samples() > 0);
  const int samples_before_switch = tab->core()->total_samples();
  std::cout << "[test]   velocity-task core accumulated " << samples_before_switch
            << " samples before switching tasks" << std::endl;

  tab->task_combo()->setCurrentIndex(1); // "Steering map".
  TEST_ASSERT(tab->core() != core_before);
  TEST_ASSERT(tab->core()->total_samples() == 0);
  TEST_ASSERT(tab->core()->accepted_samples() == 0);
  std::cout << "[test]   switching to 'Steering map' produced a NEW CalibrationCore with 0 "
               "samples (old velocity-task samples not carried over) -- OK"
            << std::endl;

  mpc::CalibrationCore *core_steering = tab->core();
  tab->task_combo()->setCurrentIndex(0); // back to "Velocity map".
  TEST_ASSERT(tab->core() != core_steering);
  TEST_ASSERT(tab->core()->total_samples() == 0);
  std::cout << "[test]   switching back to 'Velocity map' again produced a fresh core -- OK"
            << std::endl;

  std::filesystem::remove_all(tmp_dir);
  std::cout << "[Test] (d) PASSED" << std::endl;
}

} // namespace

int main(int argc, char **argv)
{
  QApplication app(argc, argv);

  test_coverage_advances_with_synthetic_feed();
  test_keyboard_drive_commands();
  test_current_abort_disables_teleop();
  test_task_switch_creates_fresh_core();

  // Best-effort cleanup of the "results/robot_calib/..." session directories
  // this test's CalibrationTab instances created as a side effect of
  // start_new_session() (CWD-relative, same convention as
  // MPC/src/motor_calibration.cpp's own results/ output dir -- see
  // CalibrationTab::session_dir()'s doc comment).
  std::error_code ec;
  std::filesystem::remove_all("results/robot_calib", ec);

  std::cout << "\n[Test] All test_sim_viz_calibration_tab tests passed." << std::endl;
  return 0;
}
