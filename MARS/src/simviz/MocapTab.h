#pragma once

// DESIGN part C: the "OptiTrack" tab (tab 1 of SimVizWindow's QTabWidget,
// tab 0 being the pre-existing Simulation layout, moved unchanged -- see
// SimVizWindow.h's own header comment). Talks ONLY to SimVizManager's
// public seams (mocap_manager(), execution_manager().scenario_model(),
// config(), stage_real_robots(), stage_real_progress_text(), ...), never to
// ControlServer/ExecutionManager's process bookkeeping directly -- same
// convention SimVizWindow itself follows for the Simulation tab.
//
// Composition:
//   - Connection indicator (colored dot) + state text + bridge uptime +
//     Connect/Disconnect toggle.
//   - Live rigid-body table: one row per Motive rigid body (id / age, then
//     the STAGED TRANSFORM READOUT -- raw Motive / after MOCAP ROTATION /
//     after CALIBRATION+yaw_offset -- see update_body_table()'s doc
//     comment for the exact column layout), keyed off
//     MocapManager::motive_assets() (the modeldef inventory) so the
//     identity shown is ALWAYS the real Motive name -- never a published/
//     sanitized/alias-derived guess (that belongs only in the mapping
//     table below). Falls back to one row per live body
//     (MocapManager::bodies()) with a "(name pending)" placeholder name/id
//     during the brief window before the bridge's modeldef inventory has
//     arrived (see update_body_table()'s doc comment -- in practice the
//     auto-discovery bridge withholds publishing any body until its
//     modeldef is known, so this fallback is mostly defensive).
//   - Mapping table has TWO modes, both rebuilt/refreshed by
//     rebuild_mapping_table()/update_mapping_table_live_columns() (see
//     those methods' doc comments):
//       * Scenario loaded: one row per scenario robot (name-sorted,
//         matching every other robots_sorted_by_name() convention in this
//         codebase) -- planner name | mapped Motive body (combo, populated
//         from MocapManager::motive_assets()) | yaw_offset (EDITABLE
//         QDoubleSpinBox, degrees, default from MocapManager::
//         transform_info(); per-row dirty-tracked -- see
//         mapping_yaw_dirty_'s doc comment) | VESC endpoint (editable
//         text) | freshness dot.
//       * No scenario loaded: one row per DETECTED Motive body (keyed off
//         motive_assets(), same as the rigid-body table above) -- Motive
//         name (read-only) | planner/alias name (fixed-choice combo:
//         ""/robot1/robot2/robot3/robot4, pre-selected from
//         mocap_map_config.json's "aliases" where one already maps this
//         body to one of those four names) | yaw_offset (EDITABLE
//         QDoubleSpinBox, degrees, default looked up by the pre-selected
//         alias if any; per-row dirty-tracked) | (no VESC column -- there
//         is no scenario robot to key real_robot_endpoints by) | freshness
//         dot. Rows are added as new bodies are discovered and removed if a
//         reconnect's modeldef no longer reports them; an in-progress combo
//         selection change (or yaw-offset edit) in another row is never
//         touched by this.
//     "Save Mapping" writes aliases (MocapManager::set_alias()+
//     save_mapping()) -- dirty rows' yaw offsets (MocapManager::
//     set_yaw_offset(), reload-then-merge per row, same convention as the
//     Calibration group's Apply -- see on_save_mapping()'s doc comment) --
//     and, in scenario mode, VESC-endpoint overrides
//     (SimVizManager::set_real_robot_endpoint()+save_real_robots_config())
//     -- back to disk, and offers a bridge restart (mapping edits only
//     take effect on the NEXT start_bridge()).
//   - "Move robots to initial poses" button: enabled once every
//     stage_real_robots() precondition holds for at least one MAPPED
//     scenario robot (UNMAPPED ROBOTS ARE SIM-ONLY -- an unmapped robot is
//     assumed not physically present and never blocks/appears as a move;
//     tooltip explains what's missing otherwise); on click shows a
//     confirmation dialog (per-mapped-robot current->target poses +
//     distances, unmapped robots named "not present (simulate only)")
//     before calling SimVizManager::stage_real_robots().
//   - MocapTransformWidget: custom-painted planner-vs-Motive frame visual
//     (see that class below) + a numeric summary label.
//
// refresh() is called from SimVizWindow::refresh_display() (no internal
// QTimer) -- same "externally driven, directly callable by tests without
// spinning an event loop" convention SimVizWindow's own refresh_display()
// documents.

#include "SimVizCore.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QWidget>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace simviz
{

// Custom-painted transform visual, reusing SimVizCanvas's aspect-preserving
// Y-flip fit-and-center transform MATH (SimVizWindow.cpp's paintEvent) but
// over a data-derived logical viewport (there is no "scenario workspace
// rect" to anchor to here -- calibration is meaningful even with no
// scenario loaded at all) rather than ScenarioModel::params(). Draws:
//   - the planner frame's axes at its origin (solid),
//   - Motive's OWN FIXED viewport axes -- +X right, +Z down-screen, +Y the
//     up/out-of-plane axis -- drawn INDEPENDENTLY of the y_up checkbox
//     (that flag governs how live POSE DATA is mapped via mocap_rotation(),
//     not how Motive's own axes are drawn/labeled in its own viewport).
//     Only the calibration matrix (theta/translation from the UI)
//     orients/places these: +X and +Z as dashed in-plane arrows labeled
//     "Motive +X"/"Motive +Z" (no +Y(up) marker -- that out-of-plane axis
//     has no in-plane direction to draw and was removed from this glyph) --
//     see MocapTransformWidget::paintEvent()'s own doc comment for the exact
//     sign convention that makes an un-rotated (theta=0) glyph draw +Z
//     pointing down-screen,
//   - one ORIENTED ARROW (shaft+head, world-scaled to ~0.15m with a
//     minimum on-screen size) + colored + name label per live body
//     (already in PLANNER frame -- see MocapCore.h's header comment: the
//     bridge applies the full transform chain before publishing), so
//     heading is visible, not just position -- reuses the same
//     oriented-arrow polygon shape as SimVizCanvas's own heading arrow
//     (see this .cpp's draw_oriented_arrow(), ported from
//     SimVizWindow.cpp's).
// A near-identity matrix (the shipped placeholder) additionally draws a
// small "identity / uncalibrated" note, per this task's design doc.
class MocapTransformWidget : public QWidget
{
  Q_OBJECT
public:
  explicit MocapTransformWidget(QWidget *parent = nullptr);

  struct BodyPoint
  {
    std::string name;
    Pose pose;
  };

  // Replaces the data this widget paints and repaints. Safe to call with an
  // empty `bodies` (e.g. before the bridge has ever connected) -- the
  // planner/Motive frame axes still draw.
  void set_data(const TransformInfo &info, std::vector<BodyPoint> bodies);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  TransformInfo info_;
  std::vector<BodyPoint> bodies_;
};

class MocapTab : public QWidget
{
  Q_OBJECT
public:
  explicit MocapTab(SimVizManager &manager, QWidget *parent = nullptr);

  // Refreshes every live-data-driven display element: indicator/state/
  // uptime, the rigid-body table, the mapping table (rebuilding rows only
  // when its underlying identity set actually changed -- see
  // rebuild_mapping_table()'s doc comment -- so in-progress edits are never
  // clobbered mid-edit) plus its body-combo options/freshness dots, the
  // "Move to initial poses" button's enabled state/tooltip, and the
  // transform visual.
  void refresh();

  // Testability seams (mirrors SimVizWindow's noise_slider_percent()-style
  // accessors) -- deliberately exposing the actual widgets (not just
  // derived strings) so a test can both read AND drive them the same way a
  // real user would.
  QLabel *indicator() const { return indicator_; }
  QLabel *state_label() const { return state_label_; }
  QLabel *uptime_label() const { return uptime_label_; }
  QPushButton *connect_button() const { return connect_button_; }
  QTableWidget *body_table() const { return body_table_; }
  QTableWidget *mapping_table() const { return mapping_table_; }
  QPushButton *save_mapping_button() const { return save_mapping_button_; }
  QPushButton *move_to_initial_button() const { return move_to_initial_button_; }
  MocapTransformWidget *transform_widget() const { return transform_widget_; }
  QLabel *transform_numbers_label() const { return transform_numbers_label_; }

  // Calibration group box seams (part B: Motive-planar -> planner
  // calibration). Values are METERS (translation)/DEGREES (rotation) in
  // the UI -- see on_apply_calibration()'s doc comment for the
  // degrees->radians conversion and exact write-out semantics.
  QDoubleSpinBox *calib_tx_spin() const { return calib_tx_spin_; }
  QDoubleSpinBox *calib_ty_spin() const { return calib_ty_spin_; }
  QDoubleSpinBox *calib_theta_deg_spin() const { return calib_theta_deg_spin_; }
  QCheckBox *calib_y_up_checkbox() const { return calib_y_up_checkbox_; }
  QPushButton *calib_apply_button() const { return calib_apply_button_; }
  QLabel *calib_status_label() const { return calib_status_label_; }

private slots:
  void on_connect_toggle();
  void on_save_mapping();
  void on_move_to_initial_poses();
  void on_apply_calibration();

private:
  // Which identity set mapping_table_'s current ROWS were built from --
  // Empty right after construction/whenever the table has zero rows for a
  // reason other than "scenario-less, no bodies yet" (kept simple: this
  // class only ever transitions Empty -> {Scenario,ScenarioLess} once, on
  // the first rebuild_mapping_table() call, and freely between Scenario
  // and ScenarioLess afterward as the scenario is loaded/cleared).
  enum class MappingTableMode
  {
    Empty,
    Scenario,      // one row per scenario robot -- see rebuild_mapping_table().
    ScenarioLess,  // one row per DETECTED Motive body -- see rebuild_mapping_table().
  };

  // Rebuilds mapping_table_'s ROWS:
  //   - A scenario IS loaded: one row per scenario robot, but only when
  //     that robot list's IDENTITY actually changed since the last call
  //     (mirrors SimVizWindow::sync_monitor_panel_robots()'s exact
  //     rationale/technique -- a loaded scenario's robot list never changes
  //     out from under it, so a ScenarioModel pointer-identity check is a
  //     cheap, correct proxy for "did the row set change"), so refresh()
  //     can call this every tick without destroying whatever the user is
  //     mid-typing into a VESC-endpoint cell.
  //   - NO scenario is loaded (BUG report: this used to just clear the
  //     table and stop here, leaving it permanently empty even with the
  //     bridge fully Connected and bodies live -- see this file's own
  //     header comment): one row per Motive body in
  //     MocapManager::motive_assets() (the modeldef inventory -- same
  //     source the rigid-body table now keys off, see update_body_table()),
  //     reconciled ADDITIVELY against the table's current rows (add a row
  //     for a newly-discovered body, remove a row for a body a bridge
  //     restart's fresh modeldef no longer reports) so an in-progress combo
  //     selection change in another row's planner-name QComboBox (fixed
  //     ""/robot1..robot4 choices -- see MAPPING DROPDOWN in MocapTab.cpp)
  //     is never touched.
  void rebuild_mapping_table();

  void update_indicator();
  // Builds/refreshes body_table_ -- see this file's own header comment for
  // the "keyed off motive_assets(), identity is ALWAYS the real Motive
  // name" design and its "(name pending)" pre-modeldef fallback.
  void update_body_table();
  // Per-tick, non-destructive refresh of mapping_table_'s freshness-dot
  // column (and, in Scenario mode, the body combo's available items,
  // additively) -- does NOT touch row structure/widgets, nor (in
  // ScenarioLess mode) the planner-name QComboBoxes (see
  // rebuild_mapping_table() above).
  void update_mapping_table_live_columns();
  void update_transform_panel();
  void update_move_button_enabled();
  // Loads calib_tx_spin_/calib_ty_spin_/calib_theta_deg_spin_/
  // calib_y_up_checkbox_ from the CURRENT in-memory mapping's x0/y0/
  // theta0/y_up (0.0/false if absent). Called once at construction and
  // again after a successful Apply -- deliberately NOT from refresh()
  // (called every tick), so the user's in-progress edits are never
  // clobbered mid-typing (same rationale as rebuild_mapping_table()'s
  // change-gated rebuild for the VESC-endpoint QLineEdits). Sets each
  // spinbox/checkbox's value under a QSignalBlocker and then clears its
  // calib_*_dirty_ flag below -- so this reload-from-in-memory-mapping
  // never itself counts as a "user edit" (see those flags' own doc
  // comment).
  void load_calibration_fields();

  // Every scenario robot's current (from MocapManager) -> target (scenario
  // timetable t=0) pose pair, freshness-gated the same way
  // update_move_button_enabled() checks preconditions -- shared by that
  // method and the confirmation dialog in on_move_to_initial_poses().
  struct RobotStagePreview
  {
    std::string name;
    // UNMAPPED ROBOTS ARE SIM-ONLY: true iff this robot has a Robot-mapping
    // alias (robot_is_mapped(), MocapCore.h) -- independent of
    // has_fresh_current below. An unmapped robot is assumed not physically
    // present and never moved; only a mapped robot's has_fresh_current
    // matters for the move.
    bool is_mapped = false;
    bool has_fresh_current = false;
    Pose current;
    Pose target;
  };
  std::vector<RobotStagePreview> build_stage_preview() const;

  SimVizManager &manager_;

  QLabel *indicator_ = nullptr;
  QLabel *state_label_ = nullptr;
  QLabel *uptime_label_ = nullptr;
  QPushButton *connect_button_ = nullptr;

  QTableWidget *body_table_ = nullptr;

  QTableWidget *mapping_table_ = nullptr;
  QPushButton *save_mapping_button_ = nullptr;
  QLabel *mapping_status_label_ = nullptr;

  QPushButton *move_to_initial_button_ = nullptr;
  QLabel *stage_real_status_label_ = nullptr;

  MocapTransformWidget *transform_widget_ = nullptr;
  QLabel *transform_numbers_label_ = nullptr;

  QDoubleSpinBox *calib_tx_spin_ = nullptr;
  QDoubleSpinBox *calib_ty_spin_ = nullptr;
  QDoubleSpinBox *calib_theta_deg_spin_ = nullptr;
  QCheckBox *calib_y_up_checkbox_ = nullptr;
  QPushButton *calib_apply_button_ = nullptr;
  QLabel *calib_status_label_ = nullptr;

  // BUG FIX (stale-state clobber): per-field "the user actually edited this
  // in the GUI this session" flags for the calibration group. Neither
  // QDoubleSpinBox nor QCheckBox has a QLineEdit::isModified()-style built-
  // in (the mapping table's VESC-endpoint QLineEdits use that directly --
  // see on_save_mapping()'s doc comment for the same "was edited" gating
  // rationale), so this tab tracks it itself: set ONLY by a live user-driven
  // valueChanged()/toggled() signal (connected in the constructor), cleared
  // by load_calibration_fields() (which wraps each setValue()/setChecked()
  // in a QSignalBlocker so ITS OWN programmatic reload never sets these).
  // on_apply_calibration() uses these to decide, per field, whether to
  // write the user's GUI value or the value a fresh on-disk reload just
  // picked up -- see that method's own doc comment for why (previously
  // Apply wrote ALL FOUR fields from the GUI's snapshot unconditionally,
  // which clobbered a hand-corrected y_up/theta0 with stale GUI state the
  // user never touched).
  bool calib_tx_dirty_ = false;
  bool calib_ty_dirty_ = false;
  bool calib_theta_dirty_ = false;
  bool calib_y_up_dirty_ = false;

  // Which mode mapping_table_'s current rows reflect -- see
  // rebuild_mapping_table()'s doc comment.
  MappingTableMode mapping_table_mode_ = MappingTableMode::Empty;

  // Mode == Scenario: identity of the ScenarioModel mapping_table_ was last
  // (re)built from -- same convention as SimVizWindow::monitor_panel_model_.
  std::shared_ptr<ScenarioModel> mapping_table_model_;

  // Mode == ScenarioLess: the Motive name (motive_assets()'s
  // MotiveAssetEntry::name) each current row was built for, IN ROW ORDER --
  // row `i`'s identity is mapping_table_body_keys_[i]. Used to reconcile
  // rebuild_mapping_table()'s additive add/remove against the live
  // inventory without touching a row whose body is still present.
  std::vector<std::string> mapping_table_body_keys_;

  // Per-row "the user actually edited this row's yaw-offset spinbox this
  // session" dirty flags for the mapping table's yaw-offset column -- same
  // rationale/contract as calib_*_dirty_ above (see that doc comment),
  // applied PER ROW instead of per-scalar-field. Keyed by the SAME identity
  // each mode already uses for a row (Scenario mode: robot->name; ScenarioLess
  // mode: mapping_table_body_keys_[row], the Motive name) -- NOT by the
  // published/planner name the yaw offset is actually stored under in
  // mocap_map_config.json's "yaw_offset" object (in ScenarioLess mode that's
  // whatever planner name the row's alias QComboBox currently resolves to,
  // looked up at save time -- see on_save_mapping()). Set only by a live
  // user-driven QDoubleSpinBox::valueChanged() signal (connected at
  // row-creation time in rebuild_mapping_table()); a programmatic setValue()
  // (row creation, or the post-save re-sync) is wrapped in a QSignalBlocker
  // so it never sets this. Cleared (all entries set false) after a
  // successful on_save_mapping().
  std::unordered_map<std::string, bool> mapping_yaw_dirty_;

  // Bridge uptime: started on a successful Connect click, stopped/reset on
  // Disconnect or an externally-observed Error -- MocapManager itself
  // exposes no "process start time" accessor (process_pid()/state() only),
  // so this tab tracks it locally.
  QElapsedTimer uptime_timer_;
  bool uptime_running_ = false;
  MocapState last_seen_state_ = MocapState::Disconnected;
};

} // namespace simviz
