#pragma once

// Phase B2: the Qt GUI on top of the Phase B1 headless core (SimVizCore.h).
// SimVizWindow never reaches into ControlServer or ExecutionManager's
// process bookkeeping directly -- it only polls SimVizManager's public
// accessors (state()/label()/plan_time()/error_reason()/localization()) and
// calls its ExecutionManager's public start_execution()/abort(), exactly as
// a remote REQ client would via the PING/EXECUTE/STATUS/ABORT protocol.
//
// LOAD/START SPLIT: File->Open (open_scenario_file()) and a GUI
// --execute=<file> launch now only LOAD + PREVIEW a scenario (canvas draws
// initial poses/objects/goals immediately, exactly as before) -- they do
// NOT start execution. A separate toolbar START button (on_start_action())
// begins execution, evaluating the mocap mixed-fleet real-vs-sim decision
// at PRESS time (see ExecutionManager::start_execution()'s doc comment) so
// the operator can load a plan, then connect/map mocap, then press Start.
// This split is GUI-only: the control-socket EXECUTE verb (the MARS
// --sim-viz-handoff doorbell flow) and --headless --execute both still go
// straight through SimVizManager::execute_scenario_file() (unchanged) and
// so still auto-start, exactly as before. Abort returns to loaded-not-
// running (Start re-enabled) rather than clearing the loaded scenario.
// SimVizWindow additionally renders the scenario as soon as it loads
// (rather than only once live localization data starts arriving), and
// keeps the LAST loaded/executed scenario around (loaded_model_ before
// Start is ever pressed, then ExecutionManager::scenario_model(), a
// minimal seam added for this phase) so the canvas has something to draw
// for the whole Loaded/Running/Done/Err lifecycle.

#include "CalibrationTab.h"
#include "MocapTab.h"
#include "SimVizCore.h"

#include <QCheckBox>
#include <QColor>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace simviz
{

// One faded trail sample for one robot's on-canvas trail.
struct TrailSample
{
  Pose pose;
  std::chrono::steady_clock::time_point observed_at;
};

// Display-only canvas: aspect-preserving Y-flip transform ported from
// src/ReloPush/Visualization/VisualizationWidget.cpp (paintEvent's
// mapCoord/drawFootprintAtState math, VisualizationWidget.cpp:216-270) plus
// the oriented-arrow/goal-outline drawing patterns from that same file
// (:440-533) -- reusing the MATH, not the (single-robot) classes, per this
// task's ground truth. Knows nothing about ZMQ/QProcess/the control
// protocol; it only ever reads a ScenarioModel (static entity/goal/
// timetable data) plus an optional ExecutionManager pointer (for live
// localization poses + the plan clock + run state), both supplied by
// SimVizWindow. Safe to use standalone (as the offscreen render smoke test
// does, via set_scenario() with no ExecutionManager attached).
class SimVizCanvas : public QWidget
{
  Q_OBJECT
public:
  explicit SimVizCanvas(QWidget *parent = nullptr);

  // Sets/replaces the scenario to render. Resets per-robot trails. Safe to
  // call with a null model (clears the canvas back to "nothing loaded").
  void set_scenario(std::shared_ptr<ScenarioModel> model);

  // Optional: when set, the canvas additionally draws robots at their LIVE
  // localization poses (falling back to the scenario's initial_pose until
  // the first live sample arrives) and objects at their TIMETABLE-REPLAYED
  // pose sampled at exec_mgr->plan_time(); when null (or its state() is
  // Idle with no scenario ever executed), robots/objects are drawn at
  // plan-time-0 (initial_pose / object_pose_at(model, 0)) as a static
  // preview. Not owned.
  void set_execution_manager(ExecutionManager *exec_mgr);

  // Appends a fresh live-pose sample to each robot's trail if the
  // execution manager reports a newer localization sample than what this
  // canvas last recorded, and trims samples older than kTrailWindowS. Call
  // this once per repaint tick (SimVizWindow does, ~30Hz while Running)
  // before update(); paintEvent() itself does not mutate trail state so
  // repeated paints (e.g. from QWidget::grab() in a test) are idempotent.
  void sample_trails();

  // Seconds since the most recent trail sample recorded for `robot_name`
  // (i.e. since sample_trails() last observed a genuinely new live pose for
  // it), or std::nullopt if no sample has ever been recorded. Used by
  // SimVizWindow's status bar for a "localization age" readout that stays
  // in the caller's own steady_clock domain (see sample_trails()'s doc
  // comment on why LocalizationListener's own timestamp domain isn't
  // directly usable for this).
  std::optional<double> last_sample_age_s(const std::string &robot_name,
                                           std::chrono::steady_clock::time_point now) const;

  // ISSUE 2: whether the dashed #E76F51 object-goal-pose outline is drawn.
  // Default false (hidden) -- the user reported these as visual clutter
  // ("the dotted red boxes are still there"). SimVizWindow wires this to a
  // checkable View menu action ("Show goal outlines", default unchecked).
  void set_show_goal_outlines(bool show);
  bool show_goal_outlines() const { return show_goal_outlines_; }

  // FEATURE B: whether the dashed reference-pose overlay is drawn -- for
  // each robot, an UNFILLED, DASHED footprint outline + small heading arrow
  // in that robot's own palette color (reduced alpha ~120), sampled from the
  // SAME scenario timetable (ScenarioModel/TimeTable public API -- robots
  // interpolate, see TimeTable::pose_at()) at the current pause-adjusted
  // plan clock, i.e. exactly the clock+table the object replay above
  // already uses. Visually distinct from the solid actual-pose footprint
  // (no fill, dashed, dimmer). Default false -- opt-in, same pattern as
  // show_goal_outlines_ above. Frozen while PAUSED / static after DONE
  // "for free": both just fall out of plan_time() being frozen/stopped.
  void set_show_reference_poses(bool show);
  bool show_reference_poses() const { return show_reference_poses_; }

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  std::shared_ptr<ScenarioModel> model_;
  ExecutionManager *exec_mgr_ = nullptr; // not owned
  bool show_goal_outlines_ = false;
  bool show_reference_poses_ = false; // FEATURE B

  // robot name -> trail samples, newest last.
  std::unordered_map<std::string, std::deque<TrailSample>> trails_;
  // robot name -> last-seen LocalizationListener steady-clock tag, so
  // sample_trails() only appends once per genuinely new live sample
  // (LocalizationListener's own timestamp domain -- see LiveRobotPose's doc
  // comment in SimVizCore.h -- is relative to when that listener started,
  // which is exactly what we want to de-duplicate against here).
  std::unordered_map<std::string, double> last_seen_tag_;
};

// Top-level window: menu bar (File->Open), toolbar (Abort), the canvas, and
// a status bar (state / label / plan time / per-robot localization age).
// Owns one SimVizManager (the same class simviz_main.cpp's headless mode
// drives via a bare QTimer) -- the window is simply an additional, GUI-mode
// front end for it; the control-socket protocol behaves identically in
// both modes.
class SimVizWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit SimVizWindow(SimVizConfig config, QWidget *parent = nullptr);

  // Binds the control socket (may throw std::runtime_error, matching
  // SimVizManager::start()) and starts the tick/repaint timers. Call once
  // before showing the window.
  void start();

  SimVizManager &manager() { return manager_; }
  SimVizCanvas *canvas() { return canvas_; }

  // DESIGN part C: the central widget is now a QTabWidget -- tab 0
  // "Simulation" (the pre-existing canvas + right panel, moved unchanged)
  // and tab 1 "OptiTrack" (mocap_tab()). Exposed for tests that want to
  // assert tab structure/switch tabs; canvas()/every other pre-existing
  // seam is untouched (tab 0's contents are byte-identical to before this
  // task, just reparented under the tab widget). Tab 2 "Calibration"
  // (calibration_tab()) is a later addition -- see CalibrationTab.h's own
  // header comment.
  QTabWidget *tabs() { return tabs_; }
  MocapTab *mocap_tab() { return mocap_tab_; }
  CalibrationTab *calibration_tab() { return calibration_tab_; }

  // FEATURE C (testability): the toolbar "Motor (accel) noise σ" slider's
  // current integer percent (0..25), or -1 if the slider hasn't been
  // constructed yet. Initialized from
  // manager_.execution_manager().noise_sigma_pct() at construction time
  // (see SimVizWindow.cpp) so a CLI-supplied initial --noise-sigma-pct= is
  // reflected immediately, not only after the user manually drags the
  // slider.
  int noise_slider_percent() const { return noise_slider_ ? noise_slider_->value() : -1; }

  // PART (1) STEERING-NOISE SPLIT (testability): mirrors
  // noise_slider_percent() above for the independent "Steering noise σ"
  // slider / --steer-noise-sigma-pct=.
  int steer_noise_slider_percent() const
  {
    return steer_noise_slider_ ? steer_noise_slider_->value() : -1;
  }

  // MOTOR STALL (testability): the right panel's "Enable stall (min speed)"
  // checkbox / "Stall level" spinbox current state, or false/-1.0 if not
  // constructed yet (mirrors noise_slider_percent()'s -1 convention).
  // Initialized from manager_.execution_manager().stall_enabled()/
  // stall_level() at construction time, same CLI-reflects-immediately
  // rationale as the noise sliders above.
  bool stall_enabled_checked() const
  {
    return stall_checkbox_ ? stall_checkbox_->isChecked() : false;
  }
  double stall_level_value() const { return stall_level_spin_ ? stall_level_spin_->value() : -1.0; }
  // True iff the "Stall level" spinbox is currently enabled (i.e. the
  // checkbox is checked) -- verifies the "enabled only while checked"
  // coupling independent of stall_enabled_checked()/stall_level_value()
  // above.
  bool stall_level_enabled() const { return stall_level_spin_ && stall_level_spin_->isEnabled(); }

  // STAGING PHASE (testability): the right panel's "Stage first" checkbox
  // current state, or false if not constructed yet. Initialized from
  // manager_.staging_enabled() (NOT hardcoded), same CLI-reflects-
  // immediately rationale as stall_enabled_checked() above.
  bool stage_first_checked() const
  {
    return stage_first_checkbox_ ? stage_first_checkbox_->isChecked() : false;
  }

  // RIGHT-SIDE MONITOR PANEL (testability): current display strings for
  // `robot_name`'s cell (v/steering/accel readouts + state tag), or
  // std::nullopt if no cell exists for that robot (e.g. no scenario loaded
  // yet). Reflects whatever the last refresh_display() call computed --
  // tests call refresh_display() directly rather than spinning the repaint
  // QTimer (see that method's doc comment).
  struct MonitorCellText
  {
    std::string velocity;  // "v: <val> m/s" or "v: —" before telemetry arrives.
    std::string steering;  // "δ: <val> rad" or "δ: —".
    std::string accel;     // "a: <val> m/s²" or "a: —".
    std::string state_tag; // "STALLED"/"WATCHDOG"/"MOVING"/"IDLE", or "—".
    // LAG READOUT (three-clock lag-diagnosis task): "lag: <val>s" (2
    // decimals) or "lag: —" before any live pose has arrived for this
    // robot -- see ExecutionManager::lag_seconds()'s doc comment for what
    // the value means (positive = behind schedule).
    std::string lag;
  };
  std::optional<MonitorCellText> monitor_cell_text(const std::string &robot_name) const;

  // Refreshes all live-data-driven display state: canvas trail sampling +
  // repaint, the right-side monitor panel (robot-cell sync against the
  // current scenario + v/steering/accel/state-tag values from telemetry),
  // and the status bar. Wired to the ~33ms repaint timer (see start());
  // also directly callable by tests that want to observe post-telemetry
  // display state without spinning an event loop / QTimer (mirrors
  // open_scenario_file()'s testability rationale).
  void refresh_display();

  // LOAD/START SPLIT: loads `path` and renders it on the canvas as a static
  // preview (initial poses/objects/goals) -- it does NOT start execution
  // (see this header's top-of-file doc comment). Returns true on success;
  // on failure (unreadable/undeserializable file, or a run already busy)
  // shows a QMessageBox (when this window has a QApplication with a GUI) /
  // logs to stderr, and returns false. Used by both the File->Open action
  // and a GUI --execute=<file> launch, and (indirectly, via the underlying
  // manager) available for tests that want the exact production code path
  // rather than SimVizCanvas::set_scenario() alone. Call start_now() (the
  // Start button's slot) afterwards to actually begin execution.
  bool open_scenario_file(const QString &path);

  // LOAD/START SPLIT (testability): true iff a scenario has been loaded
  // (open_scenario_file() succeeded) but Start has not yet been pressed for
  // it this load -- i.e. exactly when the toolbar Start button should read
  // enabled. False before any load, and false again once execution has
  // actually begun (RunState::Running) or while STAGING/STAGE_REAL is
  // active.
  bool start_available() const;

private slots:
  void on_tick();
  void on_open_action();
  void on_abort_action();
  // FEATURE 2C: single toggle button -- pauses if not currently paused,
  // resumes if it is (see update_status_bar()'s toolbar-state block, which
  // flips the action's label to match).
  void on_pause_resume_action();
  void on_restart_action();
  // LOAD/START SPLIT: begins execution of whatever open_scenario_file()
  // most recently loaded, evaluating the mocap mixed-fleet decision at
  // PRESS time (see ExecutionManager::start_execution()'s doc comment).
  // No-op (button is disabled) if nothing is loaded or a run is already
  // active/staging.
  void on_start_action();

private:
  void update_status_bar();

  // RIGHT-SIDE MONITOR PANEL: (re)builds monitor_cells_ to match the
  // current scenario model's robot list -- but only when the model's
  // IDENTITY actually changed since the last call (see
  // monitor_panel_model_'s doc comment), so this is cheap to call on every
  // refresh_display() tick. No-op (clears existing cells) when no scenario
  // has ever been loaded.
  void sync_monitor_panel_robots();

  // RIGHT-SIDE MONITOR PANEL: refreshes every existing cell's v/steering/
  // accel readouts + state tag from manager_.execution_manager().
  // localization().latest_telemetry() -- "—" for all four fields when no
  // telemetry has arrived yet for that robot (see MonitorCellText's doc
  // comment).
  void update_monitor_panel_values();

  SimVizConfig config_;
  SimVizManager manager_;

  // ISSUE 2: View menu's checkable "Show goal outlines" action; toggling it
  // forwards straight to canvas_->set_show_goal_outlines().
  QAction *show_goal_outlines_action_ = nullptr;

  // FEATURE B: View menu's checkable "Show reference poses" action;
  // toggling it forwards straight to canvas_->set_show_reference_poses().
  // GUI-MODE STARTUP DEFAULTS: defaults to CHECKED (see SimVizWindow.cpp's
  // construction of this action) -- headless mode has no canvas/menu at all,
  // so this is a GUI-only default with nothing analogous to override in
  // SimVizConfig.
  QAction *show_reference_poses_action_ = nullptr;

  SimVizCanvas *canvas_ = nullptr;
  QLabel *status_label_ = nullptr;
  QAction *pause_resume_action_ = nullptr;
  QAction *restart_action_ = nullptr;

  // LOAD/START SPLIT: the toolbar Start button, and the model/path
  // open_scenario_file() most recently loaded but that Start has not yet
  // been pressed for (or has -- see loaded_model_'s doc comment). Not the
  // same seam as ExecutionManager::scenario_model(), which stays null until
  // start_execution() actually runs.
  QAction *start_action_ = nullptr;

  // LOAD/START SPLIT: the last scenario open_scenario_file() loaded (kept
  // around, not cleared, across a Start press -- so pressing Start again
  // after Abort re-executes the SAME loaded scenario without requiring
  // another File->Open). update_status_bar()/sync_monitor_panel_robots()
  // fall back to this whenever manager_.execution_manager().scenario_model()
  // is still null (i.e. before Start has ever been pressed for this load),
  // so the canvas/monitor panel/status bar all reflect a freshly-loaded
  // scenario immediately, not only once execution begins.
  std::shared_ptr<ScenarioModel> loaded_model_;
  std::string loaded_path_;

  // FEATURE C: toolbar ACCEL-channel noise-sigma slider (integer percent
  // 0..25) + its live value readout. Every change forwards to
  // manager_.execution_manager().set_noise_sigma_pct() -- see that method's
  // doc comment for what happens to an already-running run vs. future
  // spawns. GUI-MODE STARTUP DEFAULTS: defaults to 10% (was 0) -- see
  // apply_gui_startup_defaults() in SimVizWindow.cpp; headless mode /
  // SimVizConfig::noise_sigma_pct's own field default stay at 0.
  QSlider *noise_slider_ = nullptr;
  QLabel *noise_value_label_ = nullptr;

  // PART (1) STEERING-NOISE SPLIT: independent STEER-channel counterpart of
  // noise_slider_/noise_value_label_ above, forwarding to
  // manager_.execution_manager().set_steer_noise_sigma_pct(). GUI-MODE
  // STARTUP DEFAULTS: defaults to 10% (was 0), same as noise_slider_ above.
  QSlider *steer_noise_slider_ = nullptr;
  QLabel *steer_noise_value_label_ = nullptr;

  // MOTOR STALL: right panel's "Motor stall" group -- level spinbox (m/s,
  // [0.00,0.25], step 0.01, default 0.10) enabled only while the checkbox is
  // checked. Every change forwards to manager_.execution_manager().
  // set_stall_enabled()/set_stall_level(). Initialized from
  // manager_.execution_manager().stall_enabled()/stall_level() (NOT
  // hardcoded), same CLI-reflects-immediately rationale as the noise
  // sliders above. GUI-MODE STARTUP DEFAULTS: the checkbox now defaults to
  // CHECKED (was unchecked) -- see apply_gui_startup_defaults() in
  // SimVizWindow.cpp; headless mode / SimVizConfig::stall_enabled's own
  // field default stay false. The level spinbox's 0.10 default is
  // unchanged (SimVizConfig::stall_level was already 0.10).
  QCheckBox *stall_checkbox_ = nullptr;
  QDoubleSpinBox *stall_level_spin_ = nullptr;

  // STAGING PHASE: right panel's "Stage first" checkbox -- default
  // UNCHECKED, wired directly to manager_.set_staging_enabled()/
  // staging_enabled() (read once per EXECUTE, not live-published like the
  // noise/stall controls above -- see SimVizManager::
  // start_execution_with_staging()). Initialized from
  // manager_.staging_enabled() (NOT hardcoded), same CLI-reflects-
  // immediately rationale as every other control on this panel.
  QCheckBox *stage_first_checkbox_ = nullptr;

  // RIGHT-SIDE ROBOT MONITOR PANEL: one cell per robot once a scenario is
  // loaded (color swatch + name, then live v/steering/accel/state-tag
  // rows). monitor_cells_layout_ is the vertical layout new cells are
  // appended to (nested inside the right panel's own layout, built in the
  // constructor); monitor_cells_ is keyed by robot name so
  // update_monitor_panel_values() can find each cell's labels without
  // rescanning the layout every tick.
  struct MonitorCell
  {
    QWidget *widget = nullptr; // owns everything below; parented into monitor_cells_layout_.
    QLabel *swatch_label = nullptr;
    QLabel *name_label = nullptr;
    QLabel *v_label = nullptr;
    QLabel *steering_label = nullptr;
    QLabel *accel_label = nullptr;
    QLabel *state_label = nullptr;
    // LAG READOUT (three-clock lag-diagnosis task): see MonitorCellText::lag
    // above. Styled red (via stylesheet) when the value exceeds 2s, default
    // color otherwise -- update_monitor_panel_values() sets both text and
    // color together.
    QLabel *lag_label = nullptr;
  };
  QVBoxLayout *monitor_cells_layout_ = nullptr;
  std::unordered_map<std::string, MonitorCell> monitor_cells_;
  // Identity of the ScenarioModel monitor_cells_ was last (re)built from --
  // compared by pointer in sync_monitor_panel_robots() so cells are only
  // rebuilt when a genuinely NEW scenario loads (File->Open/EXECUTE/
  // RESTART funnel through ExecutionManager::start_execution(), which is
  // what scenario_model() reflects), not on every ~33ms repaint tick.
  std::shared_ptr<ScenarioModel> monitor_panel_model_;

  QTimer *tick_timer_ = nullptr;    // ~50ms: manager_.tick() (protocol + exec manager)
  QTimer *repaint_timer_ = nullptr; // ~33ms (~30Hz): trail sampling + canvas/monitor-panel repaint

  // DESIGN part C: QTabWidget central widget -- tab 0 = the pre-existing
  // Simulation layout (unchanged), tab 1 = mocap_tab_, tab 2 = calibration_tab_.
  QTabWidget *tabs_ = nullptr;
  MocapTab *mocap_tab_ = nullptr;
  CalibrationTab *calibration_tab_ = nullptr;
};

} // namespace simviz
