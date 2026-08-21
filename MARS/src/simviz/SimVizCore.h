#pragma once

// Phase B1: the headless mars_sim_viz manager. Everything in this header is
// display-free (no QtWidgets) so it can be driven and unit tested offscreen/
// headlessly -- see MARS/tests/test_sim_viz_control.cpp,
// MARS/tests/test_sim_viz_scenario_model.cpp and
// MARS/tests/test_sim_viz_integration.cpp. The Qt GUI (SimVizWindow, Phase
// B2) is a separate, later seam: it will poll ExecutionManager's state /
// connect to its signals, never reach into ControlServer or the process
// bookkeeping directly.
//
// Composition (owned by SimVizManager, instantiated once by
// MARS/src/simviz/simviz_main.cpp):
//   ControlServer        -- REP socket, PING/EXECUTE/STATUS/ABORT protocol.
//   ScenarioModel         -- wraps a deserialized ExecutedScenario + the
//                             phase-A RobotTrajectoryBuilder output.
//   ExecutionManager      -- spawns/upload/starts/reaps mpc_robot_sim +
//                             mpc_controller child processes for one run.
//   LocalizationListener  -- SUB per robot, latest live pose (diagnostics /
//                             future GUI trail rendering).

#include <PHAstarPushDemoTypes.h>
#include <RobotTrajectoryBuilder.h>
#include <ExecutedScenarioSerialization.h>
#include <ReloPush/trajectory.hpp>

#include "MocapCore.h"

#include <QObject>
#include <QProcess>

#include <zmq.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace simviz
{

// ===========================================================================
// ScenarioModel
// ===========================================================================

// Wraps one deserialized ExecutedScenario (see
// MARS/include/ExecutedScenarioSerialization.h) plus the per-robot
// trajectories derived from it via the Phase-A factored
// RobotTrajectoryBuilder -- the single source of truth for waypoint/
// rel-time/ref-vel/is-pushing math, shared with MARS's own
// run_on_robots_pipeline. Immutable once loaded; a new EXECUTE builds a new
// ScenarioModel rather than mutating an existing one.
class ScenarioModel
{
public:
  ScenarioModel() = default;

  // Reads `path`, base64-decodes + deserializes it (see
  // deserialize_executed_scenario_b64), and builds robot trajectories.
  // Throws std::runtime_error (propagated from the deserializer, or raised
  // directly for an unreadable file) on any failure -- callers decide how to
  // report that (ControlServer's EXECUTE handler turns it into "ERR ...").
  static ScenarioModel load_from_file(const std::string &path);

  bool is_loaded() const { return scenario_ != nullptr; }

  const ExecutedScenario &scenario() const { return *scenario_; }
  const std::string &label() const { return scenario_->summary.label; }
  const Params &params() const { return scenario_->params; }
  double max_time() const { return scenario_->timetable.get_max_time(); }
  const TimeTable &timetable() const { return scenario_->timetable; }

  // Every ROBOT / OBJECT entity, sorted by name for deterministic port/
  // index assignment (matches MARS's own run_on_robots_pipeline convention).
  std::vector<EntityMeta *> robots_sorted_by_name() const;
  std::vector<EntityMeta *> objects_sorted_by_name() const;

  // One entry per `robots_sorted_by_name()`, in the same order -- see
  // RobotTrajectoryBuilder.h for the exact construction rules.
  const std::vector<std::pair<EntityMeta *, ReloPush::trajectory>> &
  robot_trajectories() const
  {
    return robot_trajectories_;
  }

  // Object pose at plan time `t`. Delegates straight to
  // TimeTable::get_pose(), which for an OBJECT entity holds the last
  // recorded pose (piecewise-constant) -- see TimeTable.h's pose_at() --
  // rather than reimplementing that rule here.
  Pose object_pose_at(EntityMeta *object, double t) const
  {
    return scenario_->timetable.get_pose(object, t);
  }

private:
  // shared_ptr so ScenarioModel is cheaply copyable/shareable across the
  // ExecutionManager background handshake thread and the Qt main thread,
  // even though ExecutedScenario itself is move-only.
  std::shared_ptr<ExecutedScenario> scenario_;
  std::vector<std::pair<EntityMeta *, ReloPush::trajectory>> robot_trajectories_;
};

// ===========================================================================
// ControlServer
// ===========================================================================

// Thin, non-blocking wrapper around one REP socket. Owns no protocol
// knowledge -- SimVizManager::tick() is the dispatcher; this class only
// guarantees the strict ZMQ REP recv-then-send alternation.
class ControlServer
{
public:
  ControlServer() = default;
  ~ControlServer();

  ControlServer(const ControlServer &) = delete;
  ControlServer &operator=(const ControlServer &) = delete;

  // Binds "tcp://*:<port>". Throws std::runtime_error on bind failure (e.g.
  // port already in use).
  void bind(int port);

  // Non-blocking poll for one incoming request. Returns the request body if
  // one arrived; std::nullopt otherwise. Must not be called again until
  // reply() has been sent for a previously-returned request (that would
  // violate the REP socket's FSM); SimVizManager::tick() enforces this by
  // always replying before its next poll_once() call.
  std::optional<std::string> poll_once();

  // Sends the reply to the request currently pending (must be called
  // exactly once per poll_once() that returned a value).
  void reply(const std::string &message);

private:
  std::unique_ptr<zmq::context_t> ctx_;
  std::unique_ptr<zmq::socket_t> sock_;
  bool bound_ = false;
};

// ===========================================================================
// LocalizationListener
// ===========================================================================

struct LiveRobotPose
{
  Pose pose;
  // steady_clock seconds (relative, not wall time) of the last update --
  // callers compute "age" by subtracting from their own steady_clock read.
  double last_update_steady_s = -1.0;
  bool has_pose = false;
};

// Mirrors mpc::TelemetrySample (MPC/include/mpc/SimCore.h) field
// for field -- a local copy rather than an #include of that header, matching
// LiveRobotPose's own "hand-rolled parse, no cross-module type dependency"
// convention just below (thread_main() hand-parses the plain-JSON wire
// payload the same way it already does for localization's x/y/yaw). See
// SimCore.h's TelemetrySample doc comment for what each field means; t/v/
// v_cmd/steering/accel/moving/watchdog here are wire-identical.
struct LiveRobotTelemetry
{
  double t = 0.0;
  double v = 0.0;        // effective velocity (0 while deadband-stalled).
  double v_cmd = 0.0;    // command-equivalent velocity (keeps winding up while stalled).
  double steering = 0.0; // applied steering, rad.
  double accel = 0.0;    // applied acceleration, m/s^2.
  bool moving = false;   // FEATURE B hysteresis state; always true when --deadband is off.
  bool watchdog = false; // watchdog engaged this tick.
  // steady_clock seconds (relative, not wall time) of the last update --
  // same domain/convention as LiveRobotPose::last_update_steady_s above
  // (both are stamped from the SAME thread_main() loop iteration).
  double last_update_steady_s = -1.0;
  bool has_telemetry = false;
};

// One SUB socket per robot (background thread), tracking each robot's most
// recently published localization pose AND telemetry sample -- both are
// published on the SAME bound PUB socket (mpc_robot_sim's --loc-endpoint),
// under distinct topics ("/<name>/localization" and "/<name>/telemetry"
// respectively -- see MPC/src/robot_sim.cpp), so one SUB socket
// per robot subscribing to both topics is the least-churn way to track both.
// Purely a diagnostics/GUI facility (telemetry: the SimVizWindow right-side
// robot monitor panel) -- ExecutionManager's own completion logic does NOT
// depend on this (it uses plan-clock + controller-process-exit, see
// ExecutionManager::tick()).
class LocalizationListener
{
public:
  LocalizationListener() = default;
  ~LocalizationListener();

  LocalizationListener(const LocalizationListener &) = delete;
  LocalizationListener &operator=(const LocalizationListener &) = delete;

  // Starts a background thread SUBscribing to
  // "tcp://127.0.0.1:<port>" / topics "/<name>/localization" and
  // "/<name>/telemetry" for every (name, port) pair in `robots`. No-op
  // (after stop()) if already running. Does NOT clear previously-recorded
  // poses/telemetry (see stop()'s doc comment) -- callers that need a clean
  // slate for a genuinely new run must call reset() explicitly
  // (ExecutionManager::start_execution() does this).
  void start(const std::vector<std::pair<std::string, int>> &robots);

  // REAL-ROBOT MODE (see SimVizConfig::real_mode): same as start() above,
  // but takes a full ZMQ endpoint string per robot (e.g.
  // "tcp://192.168.1.5:5555") instead of assuming "tcp://127.0.0.1:<port>"
  // -- a real robot's localization source need not be on loopback. Several
  // robots MAY share the same endpoint (e.g. one OptiTrack bridge
  // auto-discovery port publishing every body's pose under its own
  // "/<name>/localization" topic) -- each gets its own SUB socket connected
  // to that same endpoint, exactly generalizing how start() above already
  // lets two robots share a bare port. start() itself is now a thin
  // wrapper that builds "tcp://127.0.0.1:<port>" strings and delegates
  // here, so both overloads share one implementation.
  void start_with_endpoints(
      const std::vector<std::pair<std::string, std::string>> &robot_endpoints);

  // Joins the background thread. Safe to call multiple times / when not
  // running. Deliberately does NOT clear the recorded poses/telemetry --
  // ISSUE 1 fix: a run ending (DONE/ERR/ABORT, all of which tear down
  // children via ExecutionManager::terminate_children(), which calls this)
  // must leave each robot's last-known pose/telemetry queryable via
  // latest_pose()/latest_telemetry() so the canvas/monitor panel keep
  // showing robots frozen at their final position/reading instead of
  // snapping back to a fallback. Use reset() to actually clear the cache
  // for a fresh run.
  void stop();

  // Clears every recorded pose AND telemetry sample. Called exactly once
  // per genuinely NEW run (ExecutionManager::start_execution(), i.e.
  // EXECUTE / File->Open / RESTART) -- see stop()'s doc comment: stop()
  // alone (used for DONE/ERR/ABORT teardown) intentionally leaves the cache
  // intact so the last-known poses/telemetry survive teardown for display
  // purposes. After reset(), latest_pose()/latest_telemetry() return
  // std::nullopt for every robot until fresh live samples arrive for the
  // new run, so canvas/monitor-panel code correctly falls back to the NEW
  // scenario's initial_pose (poses) / "no data" (telemetry) in the
  // meantime.
  void reset();

  // Latest known pose for `robot_name`, if any has ever arrived (this run
  // or, per stop()'s retention behavior, a just-finished previous run --
  // see reset() for when that history is actually cleared).
  std::optional<LiveRobotPose> latest_pose(const std::string &robot_name) const;

  // Latest known telemetry sample for `robot_name`, if any has ever
  // arrived. Same retention/reset lifecycle as latest_pose() above (see
  // stop()/reset()'s doc comments) -- std::nullopt is the correct "no
  // telemetry source" reading for e.g. real hardware that never publishes
  // this topic.
  std::optional<LiveRobotTelemetry> latest_telemetry(const std::string &robot_name) const;

  // STAGING PHASE "hardware mode" presence check: true iff `robot_name` has
  // a recorded pose whose age is <= max_age_s. Ages are computed against
  // start_time_ (stamped in start(), the SAME steady_clock basis
  // thread_main() stamps last_update_steady_s from) rather than the
  // caller's own independent steady_clock::now() read -- last_update_
  // steady_s is relative to when THIS listener's background thread began,
  // not to the steady_clock epoch, so comparing it against an unrelated
  // steady_clock::now() read would silently be wrong. Source-agnostic (only
  // reads poses_/start_time_), so this works unchanged once a real mocap
  // bridge publishes on the same "/<name>/localization" wire format this
  // class already subscribes to. False if `robot_name` has no recorded
  // pose at all, or if start() has never been called.
  bool has_fresh_pose(const std::string &robot_name, double max_age_s) const;

private:
  void thread_main(std::vector<std::pair<std::string, std::string>> robot_endpoints);

  std::thread thread_;
  std::atomic<bool> stop_requested_{false};
  mutable std::mutex poses_mutex_;
  std::unordered_map<std::string, LiveRobotPose> poses_;
  std::unordered_map<std::string, LiveRobotTelemetry> telemetry_;
  // Stamped at the top of start() -- see has_fresh_pose()'s doc comment
  // above for why external callers need this same basis rather than their
  // own steady_clock::now() read.
  std::chrono::steady_clock::time_point start_time_;
  bool start_time_valid_ = false;
};

// ===========================================================================
// LAG READOUT (three-clock lag-diagnosis task)
// ===========================================================================

// Windowed nearest-pose PROGRESS-TIME search: returns the TimeTable time in
// [center-window_half_s, center+window_half_s] (further clamped to `ent`'s
// own valid data range by TimeTable::get_pose()'s own end-clamping -- see
// TimeTable.h's pose_at()) whose pose is closest to (x, y). Built entirely
// on TOP of TimeTable::get_pose() (no reimplementation of the reference
// lookup/interpolation) by scanning a dense grid of candidate times
// `step_s` apart -- NOT just the timetable's own stored keys, so a HOLD
// (many/all candidates sharing ~the same recorded pose over a long span,
// e.g. a robot parked waiting for a handoff) is represented correctly
// rather than via a sparse, possibly-distant lookup. Among candidates
// within `tie_eps_m` of the single best distance found (the hallmark of a
// hold, where every candidate in it is an equally good match), the one
// closest to `center` wins -- i.e. if the caller's own believed reference
// time already falls inside a hold, this reports that same time back
// (lag 0) instead of snapping to one edge of it. This IS "clamping within
// the current segment window" (see callers' doc comments).
//
// Pure function (no ExecutionManager/Qt/ZMQ dependency) -- unit-testable in
// isolation; see MARS/tests/test_sim_viz_lag.cpp.
double progress_time_search(const TimeTable &timetable, EntityMeta *ent, double x, double y,
                             double center, double window_half_s = 8.0, double step_s = 0.2,
                             double tie_eps_m = 0.03);

// ===========================================================================
// ExecutionManager
// ===========================================================================

enum class RunState
{
  Idle,
  Running,
  Done,
  Err,
};

const char *run_state_name(RunState state);

// ===========================================================================
// PART B: REAL-ROBOT MODE.
// ===========================================================================

// Per-robot endpoint override, keyed by scenario/planner robot name --
// mirrors MARS/config/real_robots.json's "robots" object (see
// load_real_robots_config() below). Either field may be left empty, meaning
// "use ExecutionManager::start_execution()'s own fallback" (see that
// method's real_mode branch in SimVizCore.cpp): vesc_endpoint falls back to
// "tcp://127.0.0.1:<vesc_port_start+i>" (the same port formula sim mode
// would have bound itself to), localization_endpoint falls back to the
// single OptiTrack bridge auto-discovery port
// ("tcp://127.0.0.1:<SimVizConfig::mocap.loc_port_start>").
struct RealRobotEndpoints
{
  std::string vesc_endpoint;
  std::string localization_endpoint;
};

// Loads MARS/config/real_robots.json (or any --real-robots-config= override
// -- see simviz_main.cpp)'s "robots" object into a name-keyed map. Format:
// {"robots": {"<planner robot name>": {"vesc_endpoint": "tcp://...",
// "localization_endpoint": "tcp://..."}, ...}, "_doc": "..."}. Both
// per-robot fields are optional (see RealRobotEndpoints's doc comment
// above); unknown top-level keys (e.g. "_doc") are simply ignored (this is
// read-only -- unlike MocapCore.h's mapping load/save round trip, nothing
// ever writes this file back). Throws std::runtime_error (unreadable file)
// or nlohmann::json::parse_error (invalid JSON), same convention as
// ScenarioModel::load_from_file() / MocapCore.h's load_mapping_json() --
// callers decide how to report that (simviz_main.cpp treats a missing/
// invalid file as non-fatal: a warning, empty map, every robot falls back).
std::unordered_map<std::string, RealRobotEndpoints>
load_real_robots_config(const std::string &path);

// DESIGN part C (GUI "Save Mapping" button): the write-back counterpart of
// load_real_robots_config() above -- this file's ONLY writer (unlike that
// function's own doc comment, which predates this button). Writes
// `endpoints` to `path` as {"robots": {...}, "_doc": <preserved from the
// existing file at `path` if it parses and has a "_doc" key, else a canned
// default>}, pretty-printed (2-space indent, matching save_mapping_json()'s
// style in MocapCore.h). Every entry in `endpoints` is written even if both
// fields are empty (round-trips cleanly through load_real_robots_config()
// either way). Returns false (*err set, if given) on an I/O failure; true
// otherwise.
bool save_real_robots_config(const std::unordered_map<std::string, RealRobotEndpoints> &endpoints,
                              const std::string &path, std::string *err = nullptr);

struct SimVizConfig
{
  int control_port = 5601;
  int handshake_port_start = 11110;
  int vesc_port_start = 3160;
  int loc_port_start = 3260;

  // Explicit overrides; empty => resolve relative to
  // QCoreApplication::applicationDirPath() (see
  // ExecutionManager::resolve_default_mpc_controller_path() /
  // resolve_default_robot_sim_path() in SimVizCore.cpp). Never a hardcoded
  // absolute user path.
  std::string mpc_controller_path;
  std::string robot_sim_path;

  // Base directory for per-run child-process CSV logs
  // ("<run_dir>/<sanitized label>_<pid>/<robot>.csv").
  std::string run_dir = "results/sim_handoff_runs";

  // FEATURE C: initial ACCEL-channel actuation-noise sigma (fraction of
  // actuator limit, [0,0.25] -- same units/range as mpc_robot_sim's own
  // --noise-sigma-pct=), applied to every spawn's --noise-sigma-pct=
  // argument. Live-adjustable afterwards via
  // ExecutionManager::set_noise_sigma_pct() (the GUI's "Motor (accel) noise
  // σ" toolbar slider, or a headless caller). Clamped by ExecutionManager's
  // constructor the same way set_noise_sigma_pct() clamps any later value.
  double noise_sigma_pct = 0.0;

  // PART (1) STEERING-NOISE SPLIT: initial STEER-channel actuation-noise
  // sigma, independent of noise_sigma_pct above (same [0,0.25] range),
  // applied to every spawn's --steer-noise-sigma-pct= argument. Live-
  // adjustable afterwards via ExecutionManager::set_steer_noise_sigma_pct()
  // (the GUI's "Steering noise σ" toolbar slider, or a headless caller).
  // Clamped by ExecutionManager's constructor the same way
  // set_steer_noise_sigma_pct() clamps any later value.
  double steer_noise_sigma_pct = 0.0;

  // FEATURE C (testability only): passthrough --noise-seed= for every
  // spawned mpc_robot_sim, for deterministic noise in test fixtures. Absent
  // (nullopt) by default -- production runs get mpc_robot_sim's own
  // independently-random seed, exactly as if --noise-seed were never passed
  // at all.
  std::optional<std::uint64_t> noise_seed;

  // MOTOR STALL (min-speed deadband) UI: mirrors noise_sigma_pct's
  // spawn-time + live-publish pattern, but gates mpc_robot_sim's opt-in
  // --deadband plant model instead of actuation noise. stall_enabled is the
  // GUI's "Enable stall (min speed)" checkbox (default UNCHECKED) / the
  // headless --deadband flag; stall_level is the "Stall level" spinbox
  // value in m/s ([0,0.25], default 0.10) / --stall-level=, used for BOTH
  // min_moving_speed and min_sustain_speed (see ExecutionManager::
  // set_stall_enabled()/set_stall_level() for what a live change does to an
  // active run). When stall_enabled is false, spawned mpc_robot_sim gets NO
  // --deadband/--min-moving-speed=/--min-sustain-speed= flags at all (the
  // compiled RobotSpec defaults apply) and no --robot-spec override is
  // written for mpc_controller either -- bit-identical to this feature not
  // existing.
  bool stall_enabled = false;
  double stall_level = 0.10;

  // ---------------------------------------------------------------------
  // STAGING PHASE: before executing a planned scenario, each robot drives
  // from its ACTUAL current pose to its planner-assumed start pose (real
  // robots never start exactly where the plan assumes). See
  // MARS/src/simviz/StagingCore.h for the pure planning/decision math and
  // SimVizManager::tick_staging() (SimVizCore.cpp) for the orchestration
  // state machine. Read once per EXECUTE (SimVizManager::
  // start_execution_with_staging()) -- unlike the noise/stall knobs above,
  // staging is not live-adjustable mid-run.
  // ---------------------------------------------------------------------
  bool staging_enabled = false;
  double staging_pos_tol = 0.05;             // meters
  double staging_yaw_tol = 0.15;             // radians
  double staging_margin = 0.15;              // meters of extra clearance around OTHER robots
  int staging_max_retries = 2;
  double staging_localization_wait_s = 3.0;  // "hardware mode" presence freshness budget

  // TEST-ONLY: "robot1:dx,dy,dyaw;robot2:dx,dy,dyaw" (see
  // staging::parse_staging_test_offsets()). When non-empty, a robot listed
  // here is PRESENT with "current pose" = start_pose (+) offset; every
  // other scenario robot is treated as MISSING. Empty (default): presence
  // is instead determined from live localization freshness (see
  // staging_localization_wait_s above) -- the future hardware-mode path.
  std::string staging_test_offsets;

  // ---------------------------------------------------------------------
  // OptiTrack/MocapManager (see MARS/src/simviz/MocapCore.h): additive,
  // default-constructed MocapConfig reproduces this feature's own "not
  // connected until MOCAP_CONNECT is sent" no-op behavior, so every
  // pre-existing caller/test is unaffected by this field's mere existence.
  // ---------------------------------------------------------------------
  MocapConfig mocap;

  // ---------------------------------------------------------------------
  // PART B: REAL-ROBOT MODE. When real_mode is true, ExecutionManager::
  // start_execution() spawns ONLY mpc_controller per robot (NEVER
  // mpc_robot_sim) -- see that method's real_mode branch in SimVizCore.cpp.
  // Default false reproduces the pre-existing "spawn both, sim binds the
  // endpoints" behavior byte-identically for every caller/test that never
  // sets this (every one of this task's prior regression suites).
  // ---------------------------------------------------------------------
  bool real_mode = false;

  // Per-robot endpoint overrides for real_mode, keyed by scenario robot
  // name -- loaded from MARS/config/real_robots.json (or --real-robots-
  // config=) by simviz_main.cpp via load_real_robots_config() above. A
  // robot with no entry here (or an entry with an empty field) uses
  // start_execution()'s fallback (see RealRobotEndpoints's doc comment).
  std::unordered_map<std::string, RealRobotEndpoints> real_robot_endpoints;

  // "Move real robots to initial poses" (SimVizManager::stage_real_robots())
  // tolerances -- independently configurable from staging_pos_tol/
  // staging_yaw_tol above (real-robot staging may want different bounds
  // than the sim-only STAGING PHASE), same meaning/units.
  double stage_real_pos_tol = 0.05; // meters
  double stage_real_yaw_tol = 0.15; // radians

  // TEST-ONLY (mirrors staging_test_offsets' convention above): overrides
  // SimVizCore.cpp's kStageRealLegTimeoutMarginS (the "traj duration + 15s"
  // per-leg bounded timeout tick_stage_real() enforces -- see that
  // constant's own doc comment / tick_stage_real()'s deadline check).
  // Default matches kStageRealLegTimeoutMarginS exactly, so every existing
  // caller/test that never sets this reproduces production behavior
  // byte-identically; a test that needs to exercise the deadline-exceeded
  // failure path without an 15s+ real-time wait can shrink it via
  // --stage-real-leg-margin-s=.
  double stage_real_leg_timeout_margin_s = 15.0;

  // TEST-ONLY (mirrors staging_test_offsets' convention above): when true,
  // stage_real_robots()'s pluggable current-pose source (see
  // SimVizManager::stage_real_current_pose_of()'s doc comment) is a
  // dedicated, persistent LocalizationListener wired to real_robot_
  // endpoints' localization endpoints, started once from
  // SimVizManager::start(), instead of MocapManager::body_pose() --
  // lets a test stand in ordinary mpc_robot_sim processes as "real
  // hardware" (no live OptiTrack bridge needed) and still exercise
  // STAGE_REAL end-to-end. False (default): production wiring, reads
  // MocapManager.
  bool stage_real_use_localization_source = false;

  // DESIGN part C (GUI "Save Mapping" button): the path
  // real_robot_endpoints above was loaded from (simviz_main.cpp sets this to
  // the same --real-robots-config= it resolved, right after the
  // load_real_robots_config() call above -- see that file). Empty (the
  // zero-value default) means "no known on-disk location" -- SimVizManager::
  // save_real_robots_config() fails with an explanatory *err in that case
  // rather than silently writing nowhere. Unlike real_robot_endpoints
  // itself, this field is never read by ExecutionManager/start_execution()
  // -- it exists purely so the GUI's Save button knows where to persist
  // edited VESC-endpoint overrides back to.
  std::string real_robots_config_path;
};

// Drives one plan-execution "run" at a time: spawns mpc_robot_sim +
// mpc_controller per robot (QProcess, env -i wrapper -- see
// spawn_wrapped_process() in the .cpp), performs the MARS wire handshake
// (upload trajectory -> ACK_RECEIVE_<robot>, broadcast START ->
// ACK_START_<robot>) on a background thread so ControlServer's EXECUTE
// handler can reply immediately, then anchors the plan clock at t0 and lets
// tick() (called every ~50ms from SimVizManager) watch for completion.
//
// Never touches mpc_controller/mpc_robot_sim's wire protocol -- see
// MPC/src/main.cpp / robot_sim.cpp -- this class only adapts to
// it (spawns the binaries, speaks the fixed handshake, reads --log-csv
// output indirectly via the child's own lifetime).
class ExecutionManager : public QObject
{
  Q_OBJECT
public:
  explicit ExecutionManager(SimVizConfig config, QObject *parent = nullptr);
  ~ExecutionManager() override;

  // False (state left untouched) if a run is already active -- caller
  // should reply "ERR busy". True means a run was started (state ->
  // Running); the caller may reply "ACK_EXECUTE" immediately without
  // waiting for the handshake to finish.
  //
  // STAGING PHASE additions (all additive/optional -- every pre-existing
  // call site, and every one of this task's 12 regression suites, passes
  // none of these and gets EXACTLY the prior behavior):
  //   `robot_subset`: when non-empty, spawn/upload/start only the robots
  //     named here (a subset of scenario->robots_sorted_by_name(), any
  //     order) instead of every robot in the scenario -- SimVizManager's
  //     STAGING PHASE orchestration uses this both for a single-robot
  //     staging leg ({robot_name}) and for the main run once some robots
  //     were found MISSING (present-robot subset).
  //   `trajectory_overrides`: when a robot's name has an entry here, its
  //     trajectory comes from this map instead of
  //     scenario->robot_trajectories() -- used to upload a STAGING leg's
  //     own planned trajectory (see StagingCore.h's build_staging_
  //     trajectory()) rather than the main scenario's. The spawned sim's
  //     initial pose is still taken from that trajectory's first waypoint
  //     (existing rule, unchanged) -- for a staging leg this is exactly
  //     the robot's current (pre-leg) pose, so the sim spawns where the
  //     robot "actually" is with no separate perturb mechanism needed.
  //   `label_suffix`: appended to scenario->label() for THIS run's
  //     sanitized run_dir name (and the label_ surfaced by label()/
  //     STATUS's DONE/ERR text) -- lets a staging leg's per-robot CSV logs
  //     land in their own directory instead of colliding with (and being
  //     overwritten by) the main run's, so they stay inspectable
  //     afterwards. Empty (default) reproduces the exact prior run_dir/
  //     label_ naming.
  bool start_execution(std::shared_ptr<ScenarioModel> scenario,
                        const std::vector<std::string> &robot_subset = {},
                        const std::unordered_map<std::string, ReloPush::trajectory>
                            &trajectory_overrides = {},
                        const std::string &label_suffix = "");

  // Tears down whatever is active (children killed/reaped) and returns the
  // manager to Idle. Idempotent -- safe to call when nothing is running.
  void abort();

  // STAGING PHASE: forces state() -> Err with the given label/reason,
  // tearing down any children first (defensive -- SimVizManager only ever
  // calls this between runs, i.e. while Idle/Done, since a staging leg's
  // own finalize_run() already reaped its children before orchestration
  // observes Done/Err, but this stays correct even if that invariant ever
  // changes). Used when the STAGING PHASE itself fails (no path found for a
  // leg, a leg's mini-execution errors, or a robot never lands within
  // tolerance after staging_max_retries) so the failure is surfaced through
  // the SAME state()==Err / error_reason() / STATUS "ERR ..." / headless
  // --exit-on-done(nonzero) path a normal in-run failure already uses,
  // rather than inventing a second, parallel failure channel.
  void fail_externally(const std::string &label, const std::string &reason);

  // Must be polled periodically (SimVizManager's QTimer) to progress
  // completion detection / reap exited children. No-op unless state() ==
  // Running.
  void tick();

  RunState state() const;
  std::string label() const;
  // Seconds since the plan clock was anchored (all STARTs acked); 0 before
  // that point. Meaningful only while Running or after Done/Err.
  double plan_time() const;
  std::string error_reason() const;

  // LAG READOUT (three-clock lag-diagnosis task): lag_s = plan_time() -
  // progress_time_search(...), computed entirely VIZ-SIDE from data this
  // manager already has (the current scenario's own TimeTable + this
  // robot's live localization pose) -- reuses progress_time_search() above,
  // no separate reference-lookup implementation. A POSITIVE value means the
  // robot's actual position is behind where the plan clock currently is;
  // near 0 means on-schedule; can go negative transiently (e.g. right after
  // a catch-up correction). std::nullopt when there is nothing meaningful
  // to report: not currently Running (covers Idle and, deliberately, Done/
  // Err -- same "only shown while Running" scope as STATUS's own "t="), no
  // scenario loaded, `robot_name` not in the current scenario, or no live
  // localization sample has arrived yet for it this run (mirrors
  // LocalizationListener::latest_pose()'s own "no data yet" reading).
  std::optional<double> lag_seconds(const std::string &robot_name) const;

  bool is_busy() const { return state() == RunState::Running; }

  const LocalizationListener &localization() const { return localization_; }

  // UNMAPPED ROBOTS ARE SIM-ONLY (mixed real/sim fleet): when config_.
  // real_mode is true, start_execution() looks up EACH robot's mapped
  // status (robot_is_mapped(), MocapCore.h) against `mocap` -- mapped
  // robots get the existing real wiring (controller only, real endpoints);
  // unmapped ones get the normal simulator+controller pair exactly as sim
  // mode does, coexisting in the same run. `mocap` must outlive every
  // subsequent start_execution() call, or be re-set/cleared (nullptr) first
  // -- SimVizManager, the only production owner of both an ExecutionManager
  // and a MocapManager, calls this once at construction with &mocap_
  // (itself outliving exec_mgr_ for the whole SimVizManager lifetime). Left
  // unset (nullptr) by default: config_.real_mode alone then governs EVERY
  // robot uniformly, byte-identical to this feature's pre-existing
  // behavior -- every test/call site that constructs an ExecutionManager
  // directly (never calling this) is therefore unaffected.
  void set_mocap_source(const MocapManager *mocap) { mocap_for_real_mode_ = mocap; }

  // ---------------------------------------------------------------------
  // FEATURE 2B (pause/resume/restart).
  // ---------------------------------------------------------------------

  // Sends "PAUSE" to every child mpc_controller's handshake REP socket
  // (fresh REQ socket per robot, ~2s rcvtimeo -- see SimVizHandoff.h's
  // PING pattern) and expects a reply starting with "ACK_PAUSE_". Requires
  // state() == Running (returns false with last_error() set otherwise; the
  // caller -- SimVizManager::pause() -- is expected to have already checked
  // this and turn it into an "ERR not running" reply, so this failure mode
  // is mostly defensive). Idempotent: safe to call while already paused
  // (mpc_controller itself acks PAUSE again without changing behavior). On
  // ANY per-robot failure, leaves the run in its current (retryable) state
  // -- children are never torn down -- and surfaces the failure via
  // last_error(); the plan clock is only actually frozen (paused_ = true)
  // if every robot acked.
  bool pause_all();

  // Mirrors pause_all(): sends "RESUME", expects "ACK_RESUME_" acks.
  // Idempotent (safe while not paused). On success, un-freezes the plan
  // clock (accumulating the just-completed pause's duration into
  // pause_accum_) and clears last_error().
  bool resume_all();

  // True once pause_all() has succeeded and resume_all() has not since.
  // Reset to false by start_execution()/abort()/finalize_run() (a
  // freshly-started or torn-down run is never "paused").
  bool is_paused() const;

  // Reason the most recent pause_all()/resume_all() call failed, if any;
  // empty after a successful call. Distinct from error_reason() (which is
  // only meaningful once state() == Err) -- a failed pause/resume leaves
  // state() == Running, so this is the only way to observe it.
  std::string last_error() const;

  // Minimal seam added for Phase B2 (SimVizWindow): the scenario most
  // recently passed to start_execution(), regardless of whether that run is
  // still active, Done, or Err -- so the GUI can keep rendering entities/
  // goals/replayed object poses after a run finishes. Never null once
  // start_execution() has been called at least once; null beforehand. Only
  // ever written from the main thread (start_execution()'s caller), so no
  // extra locking beyond state_mutex_ is needed for the read here either.
  std::shared_ptr<ScenarioModel> scenario_model() const
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return current_scenario_;
  }

  // ---------------------------------------------------------------------
  // FEATURE C (motor-noise sliders: ACCEL channel + PART (1) STEER channel).
  // ---------------------------------------------------------------------

  // Clamps `sigma_pct` to [0, 0.25] and stores it as the ACCEL-channel value
  // EVERY FUTURE spawn's --noise-sigma-pct= argument will use (so a value
  // set while Idle takes effect from t=0 of the next run). If a run is
  // currently active, ALSO immediately publishes the combined
  // {"noise_sigma_pct": clamped, "steer_noise_sigma_pct": <current steer
  // value>} payload on topic "/<robot>/sim_config" to every one of that
  // run's robots, over the per-robot PUB socket connected at spawn time (see
  // start_execution()) to that robot's own mpc_robot_sim --cmd-endpoint --
  // the same endpoint mpc_controller's own ackermann PUB connects to, i.e.
  // this is a second PUB peer of the sim's one SUB socket, not a new
  // endpoint. tick() also republishes the current values at ~1Hz while
  // Running, as slow-joiner insurance (idempotent on the sim side either
  // way).
  void set_noise_sigma_pct(double sigma_pct);

  // Current ACCEL-channel noise sigma (fraction of actuator limit, already
  // clamped to [0,0.25]) -- what the NEXT spawn will use, and (while a run
  // is active) what was last published/is being republished to it.
  // Surfaced in the control-socket STATUS diagnostic string as the "A" half
  // of "noise=A%/S%".
  double noise_sigma_pct() const;

  // PART (1) STEERING-NOISE SPLIT: mirrors set_noise_sigma_pct()/
  // noise_sigma_pct() above, but for the independent STEER channel (every
  // future spawn's --steer-noise-sigma-pct= argument; the "steer_noise_
  // sigma_pct" key of the same combined live-publish payload). Surfaced in
  // the control-socket STATUS diagnostic string as the "S" half of
  // "noise=A%/S%".
  void set_steer_noise_sigma_pct(double sigma_pct);
  double steer_noise_sigma_pct() const;

  // ---------------------------------------------------------------------
  // MOTOR STALL (min-speed deadband) UI controls.
  // ---------------------------------------------------------------------

  // Mirrors set_noise_sigma_pct()'s doc comment exactly, but for the
  // stall-enable flag: stores it as what EVERY FUTURE spawn will use (so a
  // value set while Idle takes effect from t=0 of the next run -- either
  // adding --deadband --min-moving-speed=/--min-sustain-speed= to that
  // spawn's args and writing a --robot-spec override for its
  // mpc_controller, or omitting both entirely). If a run is currently
  // active, ALSO immediately publishes the combined sim_config payload
  // (noise + steer + "deadband"/"min_moving_speed"/"min_sustain_speed",
  // ALWAYS sent together -- see publish_sim_config()) to every one of
  // that run's robots, over the same FEATURE C per-robot PUB socket
  // set/topic set_noise_sigma_pct() uses. tick()'s ~1Hz republish covers
  // this value going forward too, for the same slow-joiner-insurance
  // reason.
  void set_stall_enabled(bool enabled);
  bool stall_enabled() const;

  // Mirrors set_stall_enabled() above for the stall LEVEL (m/s, clamped to
  // [0,0.25] the same way noise sigma is clamped to [0,0.25]) -- used for
  // BOTH min_moving_speed and min_sustain_speed. Live-publishing/spawn-time
  // behavior are otherwise identical to set_stall_enabled(); changing the
  // level while stall_enabled() is false still updates what the NEXT spawn
  // (once enabled) and the live-publish payload's numeric fields will carry,
  // even though the receiving mpc_robot_sim ignores them until "deadband":1
  // is also set.
  void set_stall_level(double level_mps);
  double stall_level() const;

private:
  struct ChildProc
  {
    std::string robot_name;
    int handshake_port = 0;
    QProcess *sim = nullptr;
    QProcess *controller = nullptr;
  };

  // FEATURE C: one PUB socket per active robot, CONNECTED (at spawn time,
  // in start_execution()) to that robot's own mpc_robot_sim --cmd-endpoint
  // -- the exact same endpoint/topology mpc_controller's own ackermann PUB
  // uses to reach the sim's bound cmd SUB socket. Only ever touched from
  // the thread that calls start_execution()/tick()/set_noise_sigma_pct()
  // (the Qt/main thread in every caller of this class), matching zmq's
  // one-thread-per-socket requirement -- unlike ControlServer/
  // LocalizationListener, this is not used from a background thread.
  struct NoisePubTarget
  {
    std::string robot_name;
    std::unique_ptr<zmq::socket_t> sock;
  };
  // Sends the current noise_sigma_pct_/steer_noise_sigma_pct_/stall_enabled_/
  // stall_level_ to every target, ALL FIELDS TOGETHER on every call (renamed
  // from the FEATURE C-only publish_noise_config() now that MOTOR STALL's
  // "deadband"/"min_moving_speed"/"min_sustain_speed" fields ride the same
  // payload/socket set -- see NoisePubTarget's doc comment above, which
  // still applies verbatim to the socket plumbing itself).
  void publish_sim_config();

  void finalize_run(bool as_error, const std::string &reason);
  void terminate_children();
  // Shared implementation of pause_all()/resume_all(): pause=true sends
  // PAUSE/expects ACK_PAUSE_, pause=false sends RESUME/expects ACK_RESUME_.
  bool send_pause_resume(bool pause);
  void run_handshake_thread(
      std::shared_ptr<ScenarioModel> scenario,
      std::vector<std::string> robot_names,
      std::vector<int> handshake_ports,
      std::vector<ReloPush::trajectory> trajectories);
  // Requests the background handshake thread to stop at its next check
  // point (between retry attempts / between robots) rather than running its
  // full up-to-~6.5s budget. abort()/finalize_run() set this before
  // join()ing so tick()'s ABORT/completion handling stays bounded to
  // roughly one retry attempt's timeout, not the whole handshake budget.
  std::atomic<bool> handshake_stop_requested_{false};
  void fail_handshake(const std::string &reason);
  std::string resolve_mpc_controller_path() const;
  std::string resolve_robot_sim_path() const;

  SimVizConfig config_;

  mutable std::mutex state_mutex_;
  RunState state_ = RunState::Idle;
  std::string label_;
  std::string error_reason_;
  std::shared_ptr<ScenarioModel> current_scenario_;
  bool t0_valid_ = false;
  std::chrono::steady_clock::time_point t0_;
  double max_plan_time_ = 0.0;
  bool handshake_failed_ = false;
  std::string handshake_failure_reason_;

  // FEATURE 2B (pause/resume): paused_ is the current pause state;
  // pause_started_at_ is only meaningful while paused_ is true;
  // pause_accum_ is the running total of every COMPLETED pause's duration
  // for the current run (reset on start_execution()/abort()). plan_time()
  // and tick()'s completion-detection elapsed both subtract this so the
  // plan clock freezes while paused and does not "owe" that frozen time
  // back after resume. last_error_ surfaces the most recent
  // pause_all()/resume_all() failure (see last_error()'s doc comment).
  bool paused_ = false;
  std::chrono::steady_clock::time_point pause_started_at_;
  double pause_accum_ = 0.0;
  std::string last_error_;

  std::thread handshake_thread_;

  std::vector<ChildProc> children_;
  LocalizationListener localization_;

  // PART (1) STEERING-NOISE SPLIT: the independent STEER-channel counterpart
  // of noise_sigma_pct_ below -- same locking/publish rules, just a
  // separate value carried in the same combined sim_config payload.
  double steer_noise_sigma_pct_ = 0.0;

  // FEATURE C: noise_sigma_pct_ (and steer_noise_sigma_pct_ above) are
  // protected by state_mutex_ like every other run-config knob above;
  // noise_pub_ctx_/noise_pub_targets_/noise_last_publish_* are
  // main-thread-only (see NoisePubTarget's doc comment) and need no
  // locking.
  double noise_sigma_pct_ = 0.0;
  std::unique_ptr<zmq::context_t> noise_pub_ctx_;
  std::vector<NoisePubTarget> noise_pub_targets_;
  bool noise_last_publish_valid_ = false;
  std::chrono::steady_clock::time_point noise_last_publish_;

  // MOTOR STALL (min-speed deadband) UI: protected by state_mutex_, same as
  // noise_sigma_pct_/steer_noise_sigma_pct_ above -- read under lock at
  // spawn time (start_execution()) and by publish_sim_config()/
  // stall_enabled()/stall_level().
  bool stall_enabled_ = false;
  double stall_level_ = 0.10;

  // UNMAPPED ROBOTS ARE SIM-ONLY: see set_mocap_source()'s doc comment.
  // Non-owning; nullptr unless a caller (production: SimVizManager) opts in.
  const MocapManager *mocap_for_real_mode_ = nullptr;
};

// ===========================================================================
// SimVizManager
// ===========================================================================

// Top-level composition: owns the ControlServer + ExecutionManager and
// implements the PING/EXECUTE/STATUS/ABORT protocol dispatch described in
// the design doc. simviz_main.cpp instantiates exactly one of these and
// calls tick() from a QTimer (headless or GUI mode alike).
class SimVizManager : public QObject
{
  Q_OBJECT
public:
  explicit SimVizManager(SimVizConfig config, QObject *parent = nullptr);

  // Binds the control socket. Throws std::runtime_error on failure.
  void start();

  // One iteration: poll the control socket (non-blocking) and dispatch any
  // request, then tick the execution manager. Call from a ~50ms QTimer.
  void tick();

  ExecutionManager &execution_manager() { return exec_mgr_; }
  const ExecutionManager &execution_manager() const { return exec_mgr_; }

  // OptiTrack/MocapManager (see MARS/src/simviz/MocapCore.h) -- exposed for
  // a future GUI's OptiTrack tab, mirroring execution_manager() above; the
  // control-socket MOCAP_CONNECT/MOCAP_DISCONNECT/MOCAP_STATUS verbs
  // (handle_command()) are the headless-testable entry point (see
  // MARS/tests/test_sim_viz_mocap.cpp).
  MocapManager &mocap_manager() { return mocap_; }
  const MocapManager &mocap_manager() const { return mocap_; }

  // DESIGN part C (GUI OptiTrack tab): read-only access to the whole config
  // (real_mode / real_robot_endpoints / mocap.freshness_window_s / the
  // stage_real_*_tol pair, etc.) -- mirrors execution_manager()/
  // mocap_manager() above, additive.
  const SimVizConfig &config() const { return config_; }

  // DESIGN part C (GUI mapping table's editable VESC-endpoint column):
  // in-memory-only edit of one robot's real_robot_endpoints entry (mirrors
  // MocapManager::set_alias()'s "mutates an in-memory copy" convention) --
  // call save_real_robots_config() below to persist. Leaves
  // localization_endpoint untouched (empty, i.e. "use the bridge-port
  // fallback") unless `endpoints` already had a value for it.
  void set_real_robot_endpoint(const std::string &robot_name, const RealRobotEndpoints &endpoints)
  {
    config_.real_robot_endpoints[robot_name] = endpoints;
  }

  // DESIGN part C (GUI "Save Mapping" button): writes config_.
  // real_robot_endpoints back to config_.real_robots_config_path (see that
  // field's doc comment) via the free function of the same name above.
  // False (*err set) if real_robots_config_path is empty or the write
  // fails.
  bool save_real_robots_config(std::string *err = nullptr) const;

  // DESIGN part C (GUI status bar / "Move robots to initial poses"
  // progress): "<robot> (k/N)" while a STAGE_REAL sequence is active
  // (exactly the text handle_command()'s STATUS branch returns for its own
  // "STAGING_REAL " prefix), or empty when none is active. Factored out so
  // both the control-socket STATUS reply and the GUI can share one
  // implementation.
  std::string stage_real_progress_text() const;

  // Loads `path` and starts execution, exactly the same code this class's
  // own handle_command() runs for a remote "EXECUTE <path>" control-socket
  // request (that branch just forwards to this method) -- the single
  // reused entry point behind the control socket, SimVizWindow's File->Open
  // (SimVizWindow::open_scenario_file() calls ScenarioModel::load_from_file
  // + ExecutionManager::start_execution() directly, the same two calls made
  // here), and simviz_main.cpp's headless --execute CLI option. Returns
  // "ACK_EXECUTE" on success or "ERR <reason>" on failure (unreadable/
  // undeserializable file, or a run already busy) -- callers that need a
  // plain success/failure check may test `reply.rfind("ERR", 0) == 0`.
  std::string execute_scenario_file(const std::string &path);

  // FEATURE 2C: shared entry points for the ControlServer's PAUSE/RESUME/
  // RESTART commands AND SimVizWindow's toolbar buttons (same reuse
  // rationale as execute_scenario_file()'s doc comment) -- each returns the
  // exact wire reply ("ACK_PAUSE"/"ACK_RESUME"/"ACK_RESTART" or
  // "ERR <reason>").
  std::string pause();
  std::string resume();

  // Tears down the current run (ExecutionManager::abort(), i.e. the same
  // path ABORT takes) then re-executes the most recently loaded scenario
  // from scratch via execute_scenario_file() (fresh ScenarioModel, t=0,
  // robots respawn at initial_pose) -- the "last scenario" is whatever path
  // note_scenario_path() was most recently called with. Returns
  // "ERR no scenario" if none has ever been loaded, or forwards
  // execute_scenario_file()'s "ERR ..." on a reload failure.
  std::string restart();

  // Records `path` as the scenario restart() should reload. execute_scenario_file()
  // calls this itself on a successful load; SimVizWindow::open_scenario_file()
  // (which loads a ScenarioModel directly rather than through
  // execute_scenario_file(), see that method's doc comment) calls this too so
  // GUI-driven File->Open loads are just as restart()-able as control-socket
  // EXECUTE ones.
  void note_scenario_path(const std::string &path) { last_scenario_path_ = path; }

  // ---------------------------------------------------------------------
  // STAGING PHASE.
  // ---------------------------------------------------------------------

  // The entry point execute_scenario_file() (control socket "EXECUTE"/
  // simviz_main.cpp's --execute) uses, and that SimVizWindow::
  // open_scenario_file() (File->Open) ALSO calls instead of
  // ExecutionManager::start_execution() directly -- so a GUI-driven load
  // honors config_.staging_enabled exactly like a headless/control-socket
  // one, rather than silently bypassing staging the way calling
  // ExecutionManager::start_execution() straight from SimVizWindow would.
  //
  // When config_.staging_enabled is false, this is a pure passthrough to
  // exec_mgr_.start_execution(scenario) -- same return value, same
  // (untouched) *err_out -- i.e. byte-identical behavior to before this
  // method existed. When true: determines present/missing robots (test-
  // offsets mode if config_.staging_test_offsets is non-empty, else
  // "hardware mode" via localization freshness), fails synchronously
  // (*err_out set, returns false) if zero robots are present or a
  // staging sequence/run is already active, otherwise begins the STAGING
  // PHASE sequence and returns true immediately (async -- tick_staging(),
  // driven from tick(), does the actual work; STATUS reports "STAGING
  // <robot> (k/N)" meanwhile, see handle_command()).
  bool start_execution_with_staging(std::shared_ptr<ScenarioModel> scenario,
                                     std::string *err_out = nullptr);

  // True while a STAGING PHASE sequence (one or more single-robot legs,
  // prior to the main run) is in progress -- i.e. between
  // start_execution_with_staging() accepting a staging-enabled EXECUTE and
  // the main run actually starting (or the sequence failing). Used by
  // simviz_main.cpp's --exit-on-done to avoid treating an intermediate
  // staging LEG's own Done/Err (exec_mgr_.state() transitions through both
  // for each leg) as if the whole EXECUTE had finished.
  bool is_staging_active() const { return staging_.active; }

  // Live-toggle for the GUI's "Stage first" checkbox (mirrors
  // ExecutionManager::set_stall_enabled()'s "what every FUTURE run reads"
  // semantics -- unlike stall/noise, staging is only ever consulted at
  // start_execution_with_staging() time, so there is nothing to
  // live-publish to an in-progress run).
  void set_staging_enabled(bool enabled) { config_.staging_enabled = enabled; }
  bool staging_enabled() const { return config_.staging_enabled; }

  // ---------------------------------------------------------------------
  // PART B: REAL-ROBOT MODE -- "move real robots to initial poses".
  // ---------------------------------------------------------------------

  // Loads `path` (exactly like execute_scenario_file()) and begins the
  // STAGE_REAL sequence (see stage_real_robots() below) -- the control-
  // socket "STAGE_REAL <path>" entry point. Returns "ACK_STAGE_REAL" or
  // "ERR ...". Unlike EXECUTE, this never chains into running the main
  // scenario afterward -- "move to initial poses" is the entire action.
  std::string stage_real_scenario_file(const std::string &path);

  // Begins (async, like start_execution_with_staging()) sequentially
  // driving every MAPPED scenario robot from its current REAL pose to its
  // planner-assumed t=0 pose, one at a time (name order), reusing
  // StagingCore's leg planner via a single-robot
  // exec_mgr_.start_execution(..., real_mode) call per leg -- see
  // tick_stage_real() in SimVizCore.cpp for the full state machine.
  // Preconditions (checked synchronously; *err_out set and false returned,
  // nothing started, on any failure): config_.real_mode is true; not
  // already busy (exec_mgr_.is_busy() / staging_.active /
  // stage_real_.active); `scenario` is loaded and has >=1 robot;
  // config_.stage_real_use_localization_source is set OR
  // mocap_.state() == MocapState::Connected; at least one scenario robot is
  // mapped in Robot mapping (robot_is_mapped(), MocapCore.h) AND has a live
  // current pose from stage_real_current_pose_of() (production: a
  // MocapManager body fresher than config_.mocap.freshness_window_s).
  //
  // UNMAPPED ROBOTS ARE SIM-ONLY: a scenario robot with no Robot-mapping
  // alias is assumed not physically present and is silently SKIPPED (never
  // contributes to the move, never causes an error on its own) -- see
  // stage_real_robots()'s own doc comment in SimVizCore.cpp. A robot that
  // IS mapped but has no live pose right now is still an error (partial
  // "some MAPPED robots missing" is NOT allowed, unlike the sim STAGING
  // PHASE's present/missing split -- a mapped robot is expected to be
  // trackable).
  bool stage_real_robots(std::shared_ptr<ScenarioModel> scenario, std::string *err_out = nullptr);

  bool is_stage_real_active() const { return stage_real_.active; }

private:
  std::string handle_command(const std::string &request);

  // One entry per PRESENT robot being staged, in the order they are
  // processed (name-sorted, matching every other robots_sorted_by_name()
  // convention in this codebase).
  struct StagingRobotState
  {
    std::string name;
    RobotMeta *robot = nullptr;
    Pose start_pose;    // planner-assumed start == main run's spawn pose.
    Pose measured_pose; // current best-known ACTUAL pose; refined after each attempt.
    int attempts_done = 0;
  };
  struct StagingSequence
  {
    bool active = false;
    bool waiting_for_leg = false; // a single-robot mini-execution is in flight.
    std::shared_ptr<ScenarioModel> scenario;
    std::vector<StagingRobotState> robots; // PRESENT robots only, processing order.
    std::vector<std::string> missing;      // excluded from staging AND the main run.
    size_t current_index = 0;
  };
  StagingSequence staging_;

  // Drives the StagingSequence state machine forward by (at most) one step
  // per call -- called from tick(), before exec_mgr_.tick(). No-op unless
  // staging_.active. See SimVizCore.cpp for the full state machine.
  void tick_staging();
  // Clears staging_ and forces exec_mgr_ into Err (via
  // ExecutionManager::fail_externally()) with `reason` -- the single exit
  // path for every STAGING PHASE hard-failure mode (no path found, a leg's
  // mini-execution itself errors, retries exhausted).
  void fail_staging(const std::string &reason);
  // ExecutedScenario's own main-run spawn-pose rule (trajectory's first
  // waypoint, else initial_pose -- see ExecutionManager::start_execution())
  // reproduced here so a staging leg's TARGET pose is exactly what the main
  // run will actually spawn/plan against, not merely EntityMeta::
  // initial_pose (which for a robot with a nonzero-length trajectory is
  // only a fallback).
  Pose staging_start_pose_for(EntityMeta *robot, const ReloPush::trajectory &traj) const;

  // -----------------------------------------------------------------------
  // PART B: REAL-ROBOT MODE -- "move real robots to initial poses".
  // -----------------------------------------------------------------------

  // One entry per scenario robot, processing order (name-sorted, matching
  // every other robots_sorted_by_name() convention in this codebase).
  struct StageRealRobotState
  {
    std::string name;
    RobotMeta *robot = nullptr;
    Pose target_pose;    // scenario timetable t=0 pose (== main run's spawn pose).
    Pose measured_pose;  // current best-known ACTUAL pose; refined after each leg.
    int attempts_done = 0;
  };
  struct StageRealSequence
  {
    bool active = false;
    bool waiting_for_leg = false; // a single-robot mini-execution is in flight.
    std::shared_ptr<ScenarioModel> scenario;
    std::vector<StageRealRobotState> robots; // Every MAPPED scenario robot
                                              // (unmapped ones are skipped --
                                              // see stage_real_robots()'s
                                              // doc comment); no present/
                                              // missing split WITHIN that
                                              // mapped subset.
    size_t current_index = 0;
    // Per-leg bounded timeout (DESIGN: "traj duration + 15s"), set fresh
    // each time a leg's start_execution() is issued; leg_deadline_valid ==
    // false whenever no leg is in flight (default-constructed / just
    // cleared). See tick_stage_real() / kStageRealLegTimeoutMarginS in
    // SimVizCore.cpp -- this is a DEDICATED STAGE_REAL timeout, distinct
    // from (and stricter than) ExecutionManager::tick()'s own generic
    // completion-forcing margin.
    std::chrono::steady_clock::time_point leg_deadline;
    bool leg_deadline_valid = false;
  };
  StageRealSequence stage_real_;

  // Drives the StageRealSequence state machine forward by (at most) one
  // step per call -- called from tick(), alongside tick_staging(). No-op
  // unless stage_real_.active. Closely mirrors tick_staging() (see
  // SimVizCore.cpp for the full state machine) but simpler: every robot
  // must already be present (stage_real_robots() itself enforces that
  // synchronously, so there is no missing-robot exclusion here), and there
  // is no "then run the main scenario" step at the end -- the sequence
  // simply ends once every robot is within tolerance.
  void tick_stage_real();
  // Clears stage_real_ and forces exec_mgr_ into Err (via
  // ExecutionManager::fail_externally()) with `reason` -- mirrors
  // fail_staging() above, the single exit path for every STAGE_REAL
  // hard-failure mode.
  void fail_stage_real(const std::string &reason);

  // Pluggable current-pose source for stage_real_robots()/tick_stage_real()
  // (DESIGN NOTE: production reads MocapManager; a test can substitute a
  // dedicated localization listener -- see SimVizConfig::
  // stage_real_use_localization_source's doc comment for the full
  // rationale). Production path: scans mocap_.bodies() for `robot_name`,
  // gated by that body's own age_s against config_.mocap.freshness_window_s
  // (MocapManager::body_pose() alone does not freshness-gate). Test path:
  // real_pose_listener_.has_fresh_pose()/latest_pose() against
  // config_.staging_localization_wait_s (the same "hardware mode" presence
  // budget the sim STAGING PHASE uses). std::nullopt if no live/fresh pose
  // is available from whichever source is active.
  std::optional<Pose> stage_real_current_pose_of(const std::string &robot_name) const;

  // TEST-ONLY pose source (see SimVizConfig::stage_real_use_localization_
  // source): started once, at start(), against config_.real_robot_
  // endpoints' localization endpoints (falling back to the OptiTrack bridge
  // port, same fallback rule ExecutionManager::start_execution()'s
  // real_mode branch uses) -- independent of exec_mgr_'s own
  // run-scoped LocalizationListener (which start_execution() resets on
  // every run and is therefore unsuitable as a persistent "current real
  // pose" source between/before runs).
  LocalizationListener real_pose_listener_;

  SimVizConfig config_;
  ControlServer control_;
  ExecutionManager exec_mgr_;
  MocapManager mocap_;
  std::string last_scenario_path_;
};

} // namespace simviz
