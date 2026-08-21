#pragma once

// CalibrationTab: the "Calibration" tab (added after the OptiTrack/Mocap tab
// in SimVizWindow's QTabWidget -- see SimVizWindow.cpp's tab-construction
// block). Lets the user TELEOPERATE a real robot via keyboard while mocap +
// VESC telemetry samples are ingested into MPC/include/mpc/CalibrationCore.h
// (already built/tested as the `calibration_core` static library -- see
// MPC/CMakeLists.txt's calibration_core target), tracking coverage of two
// driving tasks (velocity map, steering map) and fitting/exporting the two
// calibration maps the robot's real driver loads at boot.
//
// SAFETY (read before touching the teleop/guard code in CalibrationTab.cpp):
// this tab talks to a REAL VESC driver on a REAL robot over the single-frame
// plain-JSON REQ/REP control protocol MPC/src/CalibClient.cpp already
// implements against (this file reimplements that wire format directly
// rather than linking MotorCalibCore.h/CalibClient.h, to keep this Qt-facing
// file's dependency footprint to just calibration_core + zmq). The driver's
// own ttl_ms is the DEADMAN: a raw "duty" command auto-expires after
// ttl_ms (kDutyTtlMs, 500ms) if never refreshed, so a GUI hang or crashed
// worker thread stops the robot on its own within half a second -- see
// TeleopWorker/kDutyResendMs below and the "hold key -> resend every 100ms"
// behavior wired up in CalibrationTab.cpp. The driver must NEVER be left on
// "calib" source when this tab isn't actively driving it -- see
// ITeleopCommandSink::shutdown_blocking()'s doc comment for why disabling
// teleop (toggle off, tab/window hide, or this tab's destructor) uses a
// bounded BLOCKING request instead of the normal fire-and-forget queued one
// every other command uses.

#include "MocapCore.h"
#include "SimVizCore.h"

#include <mpc/CalibrationCore.h>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QProcess>
#include <QPushButton>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QWidget>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class QHideEvent;
class QKeyEvent;

namespace simviz
{

// ===========================================================================
// Command-sink abstraction -- the testability seam this task's spec asks
// for: CalibrationTab only ever talks to an ITeleopCommandSink*, never to
// TeleopWorker/ZMQ directly, so a test can substitute a recording stub (see
// MARS/tests/test_sim_viz_calibration_tab.cpp's RecordingTeleopSink) with no
// sockets involved at all.
// ===========================================================================

// Every method is fire-and-forget from the caller's perspective (queued to
// whatever thread/mechanism the concrete sink actually talks to the driver
// on) EXCEPT shutdown_blocking(), the one deliberately-synchronous exception
// -- see its own doc comment below.
class ITeleopCommandSink
{
public:
  virtual ~ITeleopCommandSink() = default;

  virtual void ping() = 0;
  virtual void set_source(const std::string &source) = 0;
  // `value` is a signed duty fraction (typically small, see
  // CalibrationTab's duty_magnitude_ hard cap of 0.08); `ttl_ms` mirrors the
  // driver's raw-command deadman (see this file's header comment).
  virtual void raw_duty(double value, int ttl_ms) = 0;
  virtual void servo(double value) = 0;
  virtual void stop() = 0;

  // Blocking teardown sequence: stop() THEN set_source("ackermann"), waiting
  // for both to complete (or time out) before returning. This is the ONE
  // command-sink method allowed to block the calling (GUI) thread -- the
  // alternative (a normal fire-and-forget queued request) risks the process
  // exiting mid-flight with the real driver left on "calib" source
  // indefinitely, which is the one outcome this tab must never allow. Called
  // from CalibrationTab::disable_teleop() (toggle-off, current-abort,
  // hideEvent, and the destructor -- see that method's doc comment).
  virtual void shutdown_blocking() = 0;
};

// ===========================================================================
// TeleopWorker: the REAL ZMQ REQ (control plane) + SUB (telemetry) client --
// lives on its own QThread so the GUI thread never blocks on ZMQ. Implements
// ITeleopCommandSink by marshaling each fire-and-forget call onto the worker
// thread via QMetaObject::invokeMethod (Qt::QueuedConnection); the interface
// itself provides no return value, so this is safe to call from the GUI
// thread. shutdown_blocking() instead uses Qt::BlockingQueuedConnection (see
// ITeleopCommandSink::shutdown_blocking()'s doc comment for why that one
// method is allowed to block).
// ===========================================================================
class TeleopWorker : public QObject, public ITeleopCommandSink
{
  Q_OBJECT
public:
  // `epoch` is a steady_clock::time_point shared with the owning
  // CalibrationTab (captured once at that tab's construction) -- both the
  // GUI thread's mocap-feed poll and this worker's telemetry poll
  // self-timestamp against the SAME reference point (steady_clock has no
  // thread affinity: a time_point captured on one thread is valid to diff
  // against steady_clock::now() read on any other thread in the same
  // process), so pose and telemetry samples fed into one CalibrationCore
  // share one consistent monotonic clock basis -- see CalibrationTab.cpp's
  // header comment for why this matters (the mocap bridge's own internal
  // timestamp and the VESC driver's own wire "t" field are on two AS-YET
  // uncoordinated clocks, and CalibrationCore's steadiness/coverage logic
  // needs pose and telemetry timestamps to be mutually comparable).
  TeleopWorker(std::string host, int control_port, int telemetry_port, std::string robot_topic,
               std::chrono::steady_clock::time_point epoch);
  ~TeleopWorker() override;

  // ITeleopCommandSink.
  void ping() override;
  void set_source(const std::string &source) override;
  void raw_duty(double value, int ttl_ms) override;
  void servo(double value) override;
  void stop() override;
  void shutdown_blocking() override;

signals:
  // Emitted (queued, cross-thread) after every control-plane request
  // completes -- `cmd` is the "cmd" field that was sent ("ping",
  // "set_source", "raw", "servo", "stop"), so the GUI can tell which
  // action a given reply belongs to.
  void control_reply(QString cmd, bool ok, QString error);
  // Emitted once per DRAINED telemetry frame (never collapsed/batched --
  // CalibrationCore needs every sample, not just the newest one per poll),
  // in receipt order; the GUI's "live" readouts naturally show the newest
  // since it's simply the last signal processed.
  void telemetry_frame(double t, double erpm, double duty, double current_motor,
                        double current_in, double v_in);

public slots:
  // Wired to QThread::started -- creates the REQ/SUB sockets ON the worker
  // thread (zmq::socket_t must only be touched by the thread that created
  // it) and starts the telemetry poll timer.
  void start();

private slots:
  void do_ping();
  void do_set_source(QString source);
  void do_raw_duty(double value, int ttl_ms);
  void do_servo(double value);
  void do_stop();
  void do_shutdown_blocking();
  void poll_telemetry();

private:
  struct ReqResult
  {
    bool ok = false;
    std::string error;
  };
  // REQ/REP with the CalibClient.cpp retry-once-after-resocket pattern (see
  // MPC/src/CalibClient.cpp's ZmqDriverClient::request()): 500ms rcvtimeo,
  // one retry on timeout/send failure with the socket recreated first (a
  // REQ socket that timed out mid-recv can't legally send again without
  // being recreated).
  ReqResult send_request(const nlohmann::json &req_json);
  void ensure_control_socket();
  double now_s() const;

  std::string host_;
  int control_port_;
  int telemetry_port_;
  std::string robot_topic_;
  std::chrono::steady_clock::time_point epoch_;

  std::unique_ptr<zmq::context_t> ctx_;
  std::unique_ptr<zmq::socket_t> control_sock_;
  std::unique_ptr<zmq::socket_t> telem_sock_;
  QTimer *poll_timer_ = nullptr;
};

// ===========================================================================
// CoverageBinsWidget: custom-painted coverage-bin row (velocity: signed v
// bins spanning the configured range with the stall dead-band gap falling
// out naturally from plotting by each bin's own signed center rather than by
// index; steering: notches across the servo range) -- colored by
// mpc::BinState (empty=gray, partial=yellow with a fill-fraction bar,
// done=green, not_applicable=hatched).
// ===========================================================================
class CoverageBinsWidget : public QWidget
{
  Q_OBJECT
public:
  explicit CoverageBinsWidget(QWidget *parent = nullptr);
  void set_data(const mpc::CoverageReport &report);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  mpc::CoverageReport report_;
};

// ===========================================================================
// FitPlotWidget: small custom-painted scatter-plus-fitted-curve plot, reused
// for both tasks (velocity: v vs erpm; steering: servo vs effective delta).
// ===========================================================================
class FitPlotWidget : public QWidget
{
  Q_OBJECT
public:
  struct Point
  {
    double x = 0.0;
    double y = 0.0;
  };

  explicit FitPlotWidget(QWidget *parent = nullptr);
  void set_data(std::vector<Point> scatter, std::vector<Point> curve, double rms, bool ok,
                std::string error, std::string x_label, std::string y_label);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  std::vector<Point> scatter_;
  std::vector<Point> curve_;
  double rms_ = 0.0;
  bool ok_ = false;
  std::string error_;
  std::string x_label_;
  std::string y_label_;
};

// ===========================================================================
// CalibrationTab
// ===========================================================================
class CalibrationTab : public QWidget
{
  Q_OBJECT
public:
  explicit CalibrationTab(SimVizManager &manager, QWidget *parent = nullptr);
  ~CalibrationTab() override;

  // Refreshes freshness dots (control/telemetry/mocap) from wall-clock age --
  // called once per repaint tick from SimVizWindow::refresh_display(), same
  // "externally driven, directly callable by tests" convention as
  // MocapTab::refresh(). Also polls MocapManager for a fresh pose sample
  // (see this file's header comment on the 30ms mocap-feed poll).
  void refresh();

  // --- Testability seams ---
  // Substitutes a test double for the real ZMQ worker. Must be called BEFORE
  // Connect is pressed (on_connect_clicked() checks this to decide whether
  // to spin up a real TeleopWorker/QThread at all) -- ownership is NOT
  // taken, the caller keeps the stub alive for the tab's lifetime. Passing
  // nullptr restores normal (real ZMQ worker) behavior.
  void set_command_sink_for_test(ITeleopCommandSink *sink);

  // Test-only direct injection points, bypassing the ZMQ worker entirely --
  // feed a synthetic pose/telemetry sample exactly as the real worker's
  // signals would, driving the SAME CalibrationCore ingestion + coverage +
  // instruction-label refresh path a live session uses.
  void feed_pose_for_test(double t, double x, double y, double yaw);
  void feed_telemetry_for_test(double t, double erpm, double duty, double current_motor,
                                double current_in, double v_in);

  QComboBox *robot_combo() const { return robot_combo_; }
  QLineEdit *host_edit() const { return host_edit_; }
  QSpinBox *control_port_spin() const { return control_port_spin_; }
  QSpinBox *telemetry_port_spin() const { return telemetry_port_spin_; }
  QPushButton *connect_button() const { return connect_button_; }
  QLabel *control_status_dot() const { return control_status_dot_; }
  QLabel *telemetry_status_dot() const { return telemetry_status_dot_; }
  QLabel *mocap_status_dot() const { return mocap_status_dot_; }
  QLabel *v_in_label() const { return v_in_label_; }
  QLabel *current_label() const { return current_label_; }
  QDoubleSpinBox *current_abort_spin() const { return current_abort_spin_; }
  QLabel *abort_banner() const { return abort_banner_; }

  QComboBox *task_combo() const { return task_combo_; }
  QPushButton *load_session_button() const { return load_session_button_; }

  CoverageBinsWidget *coverage_widget() const { return coverage_widget_; }
  QLabel *instruction_label() const { return instruction_label_; }
  QLabel *coverage_percent_label() const { return coverage_percent_label_; }
  QLabel *coverage_counts_label() const { return coverage_counts_label_; }
  QLabel *top_rejection_label() const { return top_rejection_label_; }

  QCheckBox *teleop_enable_checkbox() const { return teleop_enable_checkbox_; }
  QCheckBox *invert_drive_checkbox() const { return invert_drive_checkbox_; }
  QCheckBox *invert_steering_checkbox() const { return invert_steering_checkbox_; }
  QLabel *duty_magnitude_label() const { return duty_magnitude_label_; }
  QLabel *notch_label() const { return notch_label_; }
  QLabel *live_erpm_label() const { return live_erpm_label_; }
  QLabel *live_mocap_v_label() const { return live_mocap_v_label_; }

  QDoubleSpinBox *notch_min_spin() const { return notch_min_spin_; }
  QDoubleSpinBox *notch_max_spin() const { return notch_max_spin_; }
  QSpinBox *notch_count_spin() const { return notch_count_spin_; }

  FitPlotWidget *fit_plot_widget() const { return fit_plot_widget_; }
  QPushButton *fit_button() const { return fit_button_; }
  QPushButton *export_button() const { return export_button_; }
  QPushButton *install_button() const { return install_button_; }
  QLabel *fit_status_label() const { return fit_status_label_; }

  // Current core (recreated on task switch / notch-config change) -- exposed
  // so a test can query coverage()/rejection_counts()/etc. directly instead
  // of re-deriving them from label text.
  mpc::CalibrationCore *core() const { return core_.get(); }

  bool teleop_enabled() const { return teleop_enabled_; }
  double duty_magnitude() const { return duty_magnitude_; }
  int notch_index() const { return notch_index_; }
  double current_signed_duty() const { return current_signed_duty_; }

protected:
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void hideEvent(QHideEvent *event) override;

private slots:
  void on_connect_clicked();
  void on_teleop_toggle(bool checked);
  void on_task_changed(int index);
  void on_load_session_clicked();
  void on_notch_config_changed();
  void on_fit_clicked();
  void on_export_clicked();
  void on_install_clicked();
  void on_resend_timer();
  void on_control_reply(QString cmd, bool ok, QString error);
  void on_worker_telemetry(double t, double erpm, double duty, double current_motor,
                            double current_in, double v_in);

private:
  void build_ui();
  // (Re)creates core_ for task_combo_'s current selection, bound to a fresh
  // session CSV path (append mode) -- see this file's header comment /
  // team-task spec: "Switching task builds a fresh CalibrationCore". Also
  // called when the steering-notch ladder configuration changes (notches
  // may only be set BEFORE any sample is ingested -- rebuilding a fresh core
  // is the simplest way to honor that constraint whenever the ladder
  // changes after some samples already exist).
  void start_new_session();
  void update_instruction_and_coverage();
  void update_connection_lights();
  void apply_current_abort_if_needed(double current_motor);
  void enable_teleop();
  // Safety-critical teardown -- see ITeleopCommandSink::shutdown_blocking()'s
  // doc comment. Idempotent (no-op if teleop_enabled_ is already false), so
  // it's safe to call unconditionally from every exit path (toggle-off,
  // current-abort, hideEvent, destructor).
  void disable_teleop();
  void send_current_duty();
  void send_stop_and_clear_drive_state();
  void step_notch(int delta);
  void recompute_steering_notches();
  std::string session_dir() const;
  std::string session_csv_path_for(mpc::Task task) const;
  double now_s() const;
  void teardown_worker();

  SimVizManager &manager_;
  std::chrono::steady_clock::time_point epoch_;

  // Connection strip.
  QComboBox *robot_combo_ = nullptr;
  QLineEdit *host_edit_ = nullptr;
  QSpinBox *control_port_spin_ = nullptr;
  QSpinBox *telemetry_port_spin_ = nullptr;
  QPushButton *connect_button_ = nullptr;
  QLabel *control_status_dot_ = nullptr;
  QLabel *telemetry_status_dot_ = nullptr;
  QLabel *mocap_status_dot_ = nullptr;
  QLabel *v_in_label_ = nullptr;
  QLabel *current_label_ = nullptr;
  QDoubleSpinBox *current_abort_spin_ = nullptr;
  QLabel *abort_banner_ = nullptr;

  // Task panel.
  QComboBox *task_combo_ = nullptr;
  QPushButton *load_session_button_ = nullptr;

  // Coverage panel.
  CoverageBinsWidget *coverage_widget_ = nullptr;
  QLabel *instruction_label_ = nullptr;
  QLabel *coverage_percent_label_ = nullptr;
  QLabel *coverage_counts_label_ = nullptr;
  QLabel *top_rejection_label_ = nullptr;

  // Teleop panel.
  QCheckBox *teleop_enable_checkbox_ = nullptr;
  QCheckBox *invert_drive_checkbox_ = nullptr;
  QCheckBox *invert_steering_checkbox_ = nullptr;
  QLabel *legend_label_ = nullptr;
  QLabel *duty_magnitude_label_ = nullptr;
  QLabel *notch_label_ = nullptr;
  QLabel *live_erpm_label_ = nullptr;
  QLabel *live_mocap_v_label_ = nullptr;

  QDoubleSpinBox *notch_min_spin_ = nullptr;
  QDoubleSpinBox *notch_max_spin_ = nullptr;
  QSpinBox *notch_count_spin_ = nullptr;

  // Fit panel.
  FitPlotWidget *fit_plot_widget_ = nullptr;
  QPushButton *fit_button_ = nullptr;
  QPushButton *export_button_ = nullptr;
  QPushButton *install_button_ = nullptr;
  QLabel *fit_status_label_ = nullptr;

  // --- Core state ---
  mpc::CalibrationConfig core_cfg_;
  std::unique_ptr<mpc::CalibrationCore> core_;
  mpc::Task current_task_ = mpc::Task::kVelocity;
  std::optional<mpc::VelocityFitResult> last_velocity_fit_;
  std::optional<mpc::SteeringFitResult> last_steering_fit_;

  // --- Teleop worker/sink ---
  std::unique_ptr<QThread> worker_thread_;
  std::unique_ptr<TeleopWorker> worker_;
  ITeleopCommandSink *test_sink_ = nullptr;  // non-null iff a test injected one; see set_command_sink_for_test().
  ITeleopCommandSink *sink_ = nullptr;       // points at worker_.get() normally, or test_sink_.
  bool teleop_enabled_ = false;
  bool current_abort_tripped_ = false;

  // Keyboard drive state.
  bool forward_held_ = false;
  bool backward_held_ = false;
  double duty_magnitude_ = 0.04;
  double current_signed_duty_ = 0.0;
  int notch_index_ = 0;  // relative to the center notch -- see step_notch().
  double current_servo_value_ = 0.5;
  std::vector<double> notches_;
  QTimer *resend_timer_ = nullptr;  // ~100ms, always running; no-ops unless a drive key is held.

  // Mocap poll.
  QTimer *mocap_poll_timer_ = nullptr;  // ~30ms.
  double last_mocap_sample_t_ = -1.0;
  bool have_mocap_sample_ = false;
  std::chrono::steady_clock::time_point last_mocap_wall_;
  bool have_mocap_wall_ = false;
  double prev_mocap_x_ = 0.0, prev_mocap_y_ = 0.0, prev_mocap_t_ = 0.0;
  bool have_prev_mocap_ = false;

  std::chrono::steady_clock::time_point last_telemetry_wall_;
  bool have_telemetry_ever_ = false;
  double last_v_in_ = 0.0;
  double last_current_motor_ = 0.0;
  double last_erpm_ = 0.0;

  // "REP ok" status light: reflects the OK/error flag of the most recent
  // control-plane reply (any command), not a freshness window -- a REQ/REP
  // exchange is either working or it isn't; there's no "went stale" concept
  // the way there is for a SUB stream (telemetry/mocap).
  bool last_control_ok_ = false;
  bool have_control_reply_ever_ = false;
};

}  // namespace simviz
