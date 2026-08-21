#pragma once

// MocapManager: OptiTrack/NatNet bridge (optitrack_zmq_bridge) process
// lifecycle + live rigid-body tracking for mars_sim_viz's future "OptiTrack"
// tab (see this task's DESIGN doc, part A). NO QtWidgets dependency
// (QObject/QProcess are fine, same convention as SimVizCore.h's
// ExecutionManager) so this stays headless/offscreen testable -- see
// MARS/tests/test_sim_viz_mocap.cpp.
//
// Owns the bridge QProcess (env -i wrapped, same convention as
// SimVizCore.cpp's spawn_wrapped_process) in AUTO-DISCOVERY mode (no
// --robots passed) so every rigid body Motive's model definition reports
// publishes on the single --loc-port-start port -- see
// MPC/src/optitrack_zmq_bridge.cpp's header comment ("AUTO-
// DISCOVERY" section) for the wire-level contract this class relies on.
// A background ZMQ SUB thread (mirrors SimVizCore.h's LocalizationListener)
// subscribes to "" (every topic) on that port and derives both the live
// body list AND the Connecting->Connected freshness gate from the plain-
// JSON {"x":..,"y":..,"yaw":..} payloads published under
// "/<published_name>/localization" topics -- byte-identical wire format to
// mpc_robot_sim's own localization PUB (see LocalizationListener::
// thread_main()'s doc comment), which is also this class's payload parser
// precedent (this class uses nlohmann::json instead of that file's
// hand-rolled extractor purely because MocapCore already depends on
// nlohmann::json for the map-config file below).
//
// Everything in the "Pure helpers" section below has NO process/Qt/ZMQ
// dependency at all and is directly unit-testable in isolation (see
// test_sim_viz_mocap.cpp's Part 1).

#include <PHAstar/Point.h> // Pose

#include <nlohmann/json.hpp>

#include <QObject>
#include <QProcess>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace simviz
{

// ===========================================================================
// Pure helpers: mocap<->world transform parsing (no I/O, no Qt/ZMQ).
// ===========================================================================

// Field-for-field mirror of MPC/include/mpc/OptiTrackCore.h's
// AffineTransform2D -- NOT #included directly (this file stays independent
// of the MPC build target); a 2D affine transform in homogeneous
// form [[a,b,tx],[c,d,ty],[0,0,1]] (bottom row implicit).
struct AffineTransform2D
{
  double a = 1.0, b = 0.0, tx = 0.0;
  double c = 0.0, d = 1.0, ty = 0.0;
};

struct TransformInfo
{
  bool y_up = false;
  AffineTransform2D matrix; // effective matrix: from "mocap_to_world_matrix"
                            // when present (preferred), else equivalent-to
                            // the legacy "x0"/"y0"/"theta0" rigid transform.
  bool matrix_from_legacy = false; // true iff no "mocap_to_world_matrix" key
                                    // was present (legacy x0/y0/theta0 path).
  double legacy_x0 = 0.0, legacy_y0 = 0.0, legacy_theta0 = 0.0;
  double rotation_deg = 0.0; // atan2(matrix.c, matrix.a) in degrees -- exact
                              // for a pure rotation, approximate otherwise.
  bool non_rigid_warning = false; // matrix's rotation block isn't
                                   // orthonormal/det~=+1 (scale/shear).
  std::string warning;             // set iff non_rigid_warning.
  bool parse_error = false; // "mocap_to_world_matrix" present but malformed
                             // (wrong shape, or bottom row != [0,0,1]) --
                             // `matrix` falls back to identity in this case.
  std::string error;        // set iff parse_error.
  std::unordered_map<std::string, double> yaw_offsets; // keyed by published
                                                         // (planner) robot
                                                         // name, from
                                                         // "yaw_offset".
};

// Parses the subset of a mocap_map_config.json-shaped document this class
// cares about ("y_up", "mocap_to_world_matrix" [preferred] or "x0"/"y0"/
// "theta0" [legacy fallback], "yaw_offset"). Pure function over an
// already-parsed nlohmann::json object -- no file I/O. Never throws: a
// malformed "mocap_to_world_matrix" sets parse_error/error and `matrix`
// falls back to identity rather than propagating a json exception; any
// missing/wrong-typed key silently falls back to its identity default
// (mirrors nlohmann::json::value()'s own convention).
TransformInfo parse_transform_info(const nlohmann::json &map_config);

// Applies a 2D affine transform to a planar pose, EXACTLY mirroring
// MPC/include/mpc/OptiTrackCore.h's apply_mocap_to_world():
//   [x_w,y_w]^T = [[a,b],[c,d]] * [x,y]^T + [tx,ty]^T
//   yaw_w       = normalize_angle(atan2(c, a) + yaw)
// where normalize_angle wraps into (-pi, pi], same convention as
// OptiTrackCore.h's own normalize_angle(). Used both by this file's tests
// (to prove the viz side's transform composition matches OptiTrackCore's
// bit-for-bit) and by MocapTransformWidget's own Motive-axes rendering
// (position-only use of the same math -- see that class's `apply` lambda).
// Does NOT apply any yaw_offset -- same convention as apply_mocap_to_world(),
// callers add that separately (see apply_full_transform_chain() below).
Pose apply_transform_matrix(const AffineTransform2D &matrix, double x, double y, double yaw);

// Applies the FULL raw-mocap-frame-planar -> published-planner-frame-pose
// chain a live body's pose already reflects by the time it reaches
// bodies()/body_pose(): `info`'s effective calibration matrix (whichever
// source -- see TransformInfo::matrix's doc comment) via
// apply_transform_matrix() above, THEN the named robot's per-robot
// yaw_offset (added last and normalized again -- same order
// optitrack_zmq_bridge.cpp's per-robot RobotConfig::yaw_offset handling
// uses). `raw_x`/`raw_y`/`raw_yaw` are the ALREADY up-axis-reduced planar
// mocap-frame pose -- i.e. mirroring OptiTrackCore.h's
// extract_planar_pose()/quat_to_planar_yaw() OUTPUT for whichever `y_up`
// convention is active (this file has no quaternion/3D dependency of its
// own; TransformInfo::y_up is informational here, surfaced for the
// calibration UI's checkbox -- the actual up-axis reduction happens in the
// bridge before publishing). `robot_name` looks up info.yaw_offsets (0.0
// if absent, same as an unconfigured robot in optitrack_zmq_bridge.cpp).
Pose apply_full_transform_chain(const TransformInfo &info, double raw_x, double raw_y,
                                 double raw_yaw, const std::string &robot_name);

// Field-for-field mirror of MPC/include/mpc/OptiTrackCore.h's
// mocap_rotation() -- see that header's doc comment for the full derivation
// (the "Motive: x right/z down/y up-from-plane -> Planner: x right/y
// up/z up-from-plane" spec from the live rig-calibration session) and its
// three hand-checkable cases (facing Motive's +X -> yaw 0; facing +Z ->
// yaw -pi/2; a rotation about +Y by +theta -> yaw +theta). NOT on any live
// data path in THIS process today -- the bridge performs this reduction
// itself and publishes only the ALREADY-mocap-rotated + calibrated planar
// pose over ZMQ (see this file's own header comment), so no raw
// position/quaternion ever reaches mars_sim_viz. Exists here so (a) this
// file's tests can assert bit-for-bit parity with OptiTrackCore's
// definition on shared cases -- proving the two sides agree, per this
// project's "the mocap rotation must be explicit, named, and shared"
// design goal -- and (b) compute_staged_transform_view() below has a
// forward reference to round-trip its own inverse math against.
Pose mocap_rotation(double x_m, double y_m, double z_m, double qx, double qy, double qz,
                     double qw);

// One live body's pose broken into the three NAMED pipeline stages the
// user asked to see (OptiTrack tab's staged diagnostic readout, part C of
// this task): RAW Motive-frame planar values, after MOCAP ROTATION (before
// calibration/yaw_offset), and after CALIBRATION (+ yaw_offset -- the
// live published pose mars_sim_viz already has). Everything here is
// reconstructed by ALGEBRAICALLY INVERTING apply_full_transform_chain()
// from the already-published final pose (see compute_staged_transform_view()'s
// own doc comment for why this is possible without any wire-protocol
// change: mocap_rotation() is a FIXED, parameter-free, invertible map, and
// the calibration matrix + yaw_offset are both already known to this
// process via TransformInfo/robot_name).
struct StagedTransformView
{
  bool ok = false;    // false iff the calibration matrix is singular (can't invert) --
                       // every stage below stays at its default (0.0) in that case.
  std::string error;  // set iff !ok.

  // Stage 1: RAW Motive-frame planar values. Exactly ONE of raw_y_m/raw_z_m
  // is a real reconstructed value; the OTHER is the axis Motive's height
  // component occupies, which the wire protocol never transmits (the
  // bridge drops it during extract_planar_pose()/mocap_rotation() before
  // publishing) -- see raw_height_unknown_axis below to tell them apart.
  // raw_yaw is the physical rotation about Motive's up axis; IDENTICAL in
  // value to mocap_yaw below (mocap_rotation()'s heading step is the
  // identity on the numeric yaw -- see that function's doc comment) --
  // kept as its own field for this struct's "three distinct named stages"
  // presentation, matching the UI ask literally.
  double raw_x_m = 0.0;
  double raw_y_m = 0.0;  // meaningful iff !info.y_up (Z-up: height is y_m, dropped when y_up).
  double raw_z_m = 0.0;  // meaningful iff info.y_up (Y-up: height is y_m, so z_m IS recoverable).
  double raw_yaw = 0.0;
  // "y" (y_up=true, the real rig's convention) or "z" (y_up=false) --
  // whichever of raw_y_m/raw_z_m above is NOT reconstructable (dropped by
  // the bridge before publishing, so the UI should show it as "n/a").
  std::string raw_height_unknown_axis;

  // Stage 2: after MOCAP ROTATION (before the UI calibration matrix/yaw_offset).
  double mocap_x = 0.0;
  double mocap_y = 0.0;
  double mocap_yaw = 0.0;

  // Stage 3: after CALIBRATION + yaw_offset -- identical to the live
  // published pose this was reconstructed FROM (echoed here so a caller/
  // test can show all three stages from one struct without re-reading the
  // original Pose).
  double final_x = 0.0;
  double final_y = 0.0;
  double final_yaw = 0.0;
};

// Computes StagedTransformView by inverting apply_full_transform_chain()'s
// exact composition (world = apply_transform_matrix(info.matrix, raw); yaw
// += yaw_offset) in reverse: subtract robot_name's yaw_offset from
// final_pose.yaw (position is untouched by yaw_offset -- see
// apply_full_transform_chain()'s own body), invert info.matrix's 2x2 block
// (ok=false, error set, if det(matrix) is within 1e-9 of 0 -- a
// degenerate/misconfigured calibration), subtract the translation, then
// invert mocap_rotation()'s position mapping conditioned on info.y_up
// (x_m=mocap_x always; y_up -> z_m=-mocap_y, y_m unknown; !y_up ->
// y_m=mocap_y, z_m unknown) -- yaw is untouched by this last step (see
// mocap_rotation()'s heading-invariant doc comment). Pure function, no
// process/Qt/ZMQ dependency, directly unit-testable; round-trips exactly
// against mocap_rotation() + apply_full_transform_chain() composed forward
// (see test_sim_viz_mocap.cpp).
StagedTransformView compute_staged_transform_view(const TransformInfo &info,
                                                    const Pose &final_pose,
                                                    const std::string &robot_name);

// Returns a COPY of `config` with "x0"/"y0"/"theta0"/"y_up" set to the
// given calibration and "mocap_to_world_matrix" REMOVED entirely if
// present -- the matrix key SUPERSEDES x0/y0/theta0 whenever both are
// present (see parse_transform_info()'s preference and
// optitrack_zmq_bridge.cpp's matching precedence warning), so keeping a
// stale matrix around after a fresh x0/y0/theta0 calibration would
// silently make the new calibration a no-op; removing it here is what
// makes Apply's x0/y0/theta0 values actually take effect. Every other key
// -- aliases, yaw_offset, "_doc", any unknown key -- is passed through
// untouched, same convention as set_alias_in_mapping() below.
nlohmann::json set_calibration_in_mapping(nlohmann::json config, double x0, double y0,
                                           double theta0, bool y_up);

// Returns a COPY of `config` with config["yaw_offset"][robot_name] set to
// yaw_offset_rad (creating the "yaw_offset" object if it did not already
// exist). Every other key -- including other robots' entries in
// "yaw_offset" itself -- is passed through untouched, same "passes
// everything else through" contract as set_alias_in_mapping() above.
// `yaw_offset_rad` == 0.0 is a normal, explicitly-settable value (not
// special-cased away/omitted).
nlohmann::json set_yaw_offset_in_mapping(nlohmann::json config, const std::string &robot_name,
                                          double yaw_offset_rad);

// ===========================================================================
// Mapping model (mocap_map_config.json aliases: read / edit / write).
// ===========================================================================

// Loads `path` and parses it as JSON. Throws std::runtime_error (unreadable
// file) or nlohmann::json::parse_error (invalid JSON) -- callers decide how
// to report that, same convention as ScenarioModel::load_from_file().
nlohmann::json load_mapping_json(const std::string &path);

// Returns a COPY of `config` with config["aliases"][motive_name] set to
// planner_name (creating the "aliases" object if it did not already exist).
// Every other key -- including unknown ones like "_doc" -- is passed
// through untouched, so save_mapping_json(set_alias_in_mapping(cfg, ...))
// round-trips the rest of the file byte-for-byte-equivalent (as JSON, not
// necessarily as text -- key order/whitespace may differ).
nlohmann::json set_alias_in_mapping(nlohmann::json config, const std::string &motive_name,
                                     const std::string &planner_name);

// Writes `config` to `path`, pretty-printed (2-space indent, matching this
// project's config/*.json style). Returns false (and sets *err, if given)
// on an I/O failure; true otherwise.
bool save_mapping_json(const nlohmann::json &config, const std::string &path,
                        std::string *err = nullptr);

// ===========================================================================
// Motive asset inventory line parsing.
// ===========================================================================

// One parsed entry from the bridge's one-time "[BRIDGE] Motive assets: ..."
// stdout inventory line (see optitrack_zmq_bridge.cpp's
// modeldef_inventory_logged block) -- the Motive-name<->id inventory
// MocapManager surfaces alongside the live (published-name-keyed) body
// list.
struct MotiveAssetEntry
{
  int id = -1;
  std::string name;
  int parent_id = -1;
};

// Parses zero or more "rigidbody id=<id> name='<name>' (parent=<pid>);"
// entries out of one bridge stdout line -- the exact format emitted by
// optitrack_zmq_bridge.cpp (e.g. "[BRIDGE] Motive assets: rigidbody id=1
// name='mushr2' (parent=0); rigidbody id=2 name='block' (parent=0);").
// Returns an empty vector for any line that is not a "Motive assets:"
// inventory line at all, AND for the "(none)" empty-inventory case -- pure/
// no I/O, directly unit-testable.
std::vector<MotiveAssetEntry> parse_motive_assets_line(const std::string &line);

// Reconstructs the published (ZMQ topic) name auto-discovery mode WOULD
// assign to a Motive rigid body named `motive_name`, mirroring
// optitrack_zmq_bridge.cpp's mpc::resolve_auto_discovery() precedence
// (field-for-field mirror, same rationale as AffineTransform2D above --
// this file stays independent of the MPC build target): the
// "aliases" map's value for `motive_name` if present (exact key match),
// else `motive_name` sanitized the same way mpc::sanitize_topic_name()
// does (any character outside [A-Za-z0-9_-] becomes '_'). Deliberately does
// NOT replicate resolve_auto_discovery()'s collision-disambiguation suffix
// ("_<id>", applied when two DIFFERENT bodies sanitize to the same name) --
// that needs the full modeldef snapshot this pure function doesn't take,
// and the collision case is rare. This is a best-effort correlation used
// only to find a KNOWN Motive body's live pose/freshness among
// MocapManager::bodies() -- never the source of the body's IDENTITY (that
// is always the real Motive name from motive_assets(), see MocapTab's
// rigid-body table doc comment).
std::string expected_published_name(const std::string &motive_name, const nlohmann::json &mapping);

// UNMAPPED ROBOTS ARE SIM-ONLY: true iff `mapping`'s "aliases" object
// (motive_name -> planner_name) contains at least one entry whose VALUE is
// `planner_name` -- i.e. some Motive body is currently aliased to this
// scenario robot, the mapping-time (config-level) definition of "mapped"
// used to decide real-vs-simulated wiring, independent of whether that body
// is CURRENTLY live/fresh (see MocapTab.cpp's reverse_alias_lookup() for the
// GUI's own near-identical lookup, kept separate since it also needs the
// matched Motive name -- this one only needs the yes/no answer). Shared by
// SimVizManager::stage_real_robots()/ExecutionManager::start_execution()'s
// real-mode mixed-fleet split and MocapTab's move-to-initial-poses gating.
bool robot_is_mapped(const nlohmann::json &mapping, const std::string &planner_name);

// ===========================================================================
// MocapManager
// ===========================================================================

enum class MocapState
{
  Disconnected, // bridge process not running.
  Connecting,   // process running, no live localization frame observed yet
                // (or freshness lapsed -- see tick()'s doc comment).
  Connected,    // at least one body has a localization frame newer than
                // MocapConfig::freshness_window_s.
  Error,        // process died unexpectedly, or failed to start.
};

const char *mocap_state_name(MocapState state);

struct MocapConfig
{
  // Bridge binary path; empty => resolve relative to
  // QCoreApplication::applicationDirPath() (mirrors ExecutionManager's own
  // resolve_default_mpc_controller_path()/resolve_default_robot_sim_path()
  // convention -- see MocapManager::resolve_bridge_path() in MocapCore.cpp
  // for the exact rule).
  std::string bridge_path;

  // --mocap-config passthrough for the SPAWNED BRIDGE (only passed on its
  // command line when non-empty -- an empty value here does NOT mean "use
  // the compiled-in default", it means "omit the flag entirely", letting
  // the bridge apply ITS OWN zero-arg default-config-dir resolution, which
  // is a compile-time absolute path -- see optitrack_zmq_bridge.cpp's
  // MOCAP_DEFAULT_CONFIG_DIR -- and so is already CWD-independent). Tests
  // MUST pass a COPY of the real project config file under their own temp
  // dir here (never the real MPC/config/mocap_config.txt
  // directly -- see this task's rules).
  std::string mocap_config_path;

  // Path to mocap_map_config.json -- used BOTH as the spawned bridge's
  // --map-config passthrough (same "omit if empty at spawn time" wording as
  // mocap_config_path above -- but see below, it is never actually left
  // empty by the time a bridge is spawned) AND, unlike mocap_config_path,
  // read/written directly by THIS PROCESS (MocapManager's constructor loads
  // it into mapping_json_; save_mapping()/the GUI's "Save Mapping" button
  // and calibration "Apply" write it back) -- see MocapManager::mapping()/
  // set_alias()/set_calibration()/save_mapping(). Because of that GUI-side
  // read/write, an empty value here is resolved to a concrete default
  // ONCE, at MocapManager construction, by resolve_map_config_path() (see
  // that method's doc comment in MocapCore.cpp) rather than left empty the
  // way mocap_config_path is -- mars_sim_viz never wires a CLI flag for
  // this field (there is none), so leaving it to mean "omit the bridge
  // flag" the way mocap_config_path's comment describes would make
  // MocapManager::save_mapping() unconditionally fail with "no
  // map_config_path configured" in production (this was exactly the OptiTrack
  // tab's "Save Mapping / Apply calibration always fails" bug report).
  // Tests MUST still pass a COPY of the real project config file under
  // their own temp dir here (same rule as mocap_config_path above) --
  // resolve_map_config_path() only ever synthesizes a default for an EMPTY
  // input, it never overrides an explicit one.
  std::string map_config_path;

  // --server-ip / --mode / --command-port / --local-data-port passthrough,
  // for loopback/test setups (production real-Motive runs rely on
  // mocap_config_path above instead and leave these *_explicit flags
  // false, i.e. the corresponding flag is not passed at all and the
  // bridge's own compiled/config-file default applies).
  std::string server_ip;
  bool server_ip_explicit = false;
  std::string mode;
  bool mode_explicit = false;
  int command_port = 0;
  bool command_port_explicit = false;
  int local_data_port = 0;
  bool local_data_port_explicit = false;

  int loc_port_start = 3260; // --loc-port-start; single auto-discovery port.
  double publish_rate_hz = 30.0;

  // Freshness window (seconds) for the Connecting<->Connected transition
  // and each body's own reported age -- mirrors LocalizationListener::
  // has_fresh_pose()'s max_age_s parameter/convention.
  double freshness_window_s = 2.0;
};

// See this file's header comment for the overall design. Every public
// method is intended to be called from the owning (Qt/main) thread only;
// the background SUB thread is entirely internal/private, matching
// LocalizationListener's existing convention in SimVizCore.h.
class MocapManager : public QObject
{
  Q_OBJECT
public:
  explicit MocapManager(MocapConfig config, QObject *parent = nullptr);
  ~MocapManager() override;

  MocapManager(const MocapManager &) = delete;
  MocapManager &operator=(const MocapManager &) = delete;

  // Spawns the bridge (env -i wrapped, auto-discovery mode -- no --robots
  // passed) and starts the background SUB thread; state -> Connecting.
  // False (state left untouched, *err set if given) if already Connecting/
  // Connected, or if the resolved bridge binary does not exist / fails to
  // start within a short (2s) grace period.
  bool start_bridge(std::string *err = nullptr);

  // Kills/reaps the bridge process (if running) and stops the background
  // SUB thread. Idempotent (safe to call from any state, including
  // Disconnected). State -> Disconnected (even from Error) and
  // error_reason() is cleared. Does NOT clear the last-known body list
  // (mirrors LocalizationListener::stop()'s "leave the last-known data
  // queryable after teardown" convention) -- a caller that needs a clean
  // slate should destroy/reconstruct the manager.
  void stop_bridge();

  // Must be polled periodically (e.g. from SimVizManager::tick()). No-op
  // unless the bridge is (or was) running this session (i.e. after a
  // start_bridge() that hasn't been followed by stop_bridge()). Drains the
  // process's merged stdout/stderr (Motive-assets inventory-line parsing),
  // reaps an unexpectedly-exited process into Error, and re-evaluates the
  // Connecting<->Connected transition against the SUB thread's current
  // per-body freshness (both directions: gains a fresh body -> Connected;
  // the previously-fresh body/bodies all go stale -> back to Connecting,
  // matching this class's "Connected iff a live frame arrived within the
  // last freshness_window_s" continuous-condition semantics).
  void tick();

  MocapState state() const;
  std::string error_reason() const;

  // Diagnostics/testability: PID of the currently-running bridge process
  // (0 if none is running). The bridge is spawned via "env -i ... <bridge>
  // args..." (see spawn_wrapped_bridge() in MocapCore.cpp), but `env`
  // execve()s directly in place (no fork), so this IS the bridge's own PID,
  // not a wrapper's -- e.g. a test that wants to simulate an external kill
  // of the bridge (as opposed to going through stop_bridge()) can signal
  // this PID directly.
  qint64 process_pid() const;

  struct Body
  {
    std::string published_name;
    Pose pose;
    double age_s = -1.0; // seconds since last update; -1 if never seen.
    bool has_pose = false;
  };
  // One entry per body ever seen this bridge session (published-name
  // keyed), each with its most recent pose + age.
  std::vector<Body> bodies() const;
  std::optional<Pose> body_pose(const std::string &published_name) const;

  // Latest pose sample for a published body WITH its own steady-clock
  // timestamp (seconds, monotonic, relative to this bridge session's
  // sub_start_time_ -- the SAME basis Body::age_s is derived from). Added
  // for CalibrationTab's mocap-feed poll (MARS/src/simviz/CalibrationTab.h):
  // body_pose()/bodies() only expose AGE (relative to "now"), which can't by
  // itself tell a caller "is this the SAME sample I already saw" across
  // repeated polls -- this exposes the raw per-sample timestamp so a poller
  // can dedupe on it directly (only feed a downstream consumer once per
  // genuinely new sample). std::nullopt if the body has never been seen.
  // NOTE: this timestamp is on MocapManager's own sub_start_time_ clock
  // basis, NOT necessarily comparable to another process's/thread's steady
  // clock reading -- callers that need to align this against a DIFFERENT
  // live stream (e.g. VESC telemetry) should use it only to detect "is this
  // a new sample", and self-timestamp on their OWN shared clock basis for
  // anything that needs cross-stream time alignment (see CalibrationTab.cpp).
  struct TimedPose
  {
    double t = 0.0;
    Pose pose;
  };
  std::optional<TimedPose> latest_timed_pose(const std::string &published_name) const;

  // Motive-name inventory parsed from the bridge's one-time stdout line --
  // empty until the bridge's first NAT_MODELDEF reply (see
  // parse_motive_assets_line()'s doc comment).
  std::vector<MotiveAssetEntry> motive_assets() const;

  // Mapping (mocap_map_config.json) access -- loaded once at construction
  // (best-effort; failure just leaves an empty JSON object, surfaced via
  // mapping_load_error()) plus in-memory edits via set_alias().
  const nlohmann::json &mapping() const { return mapping_json_; }
  std::string mapping_load_error() const { return mapping_load_error_; }
  void set_alias(const std::string &motive_name, const std::string &planner_name);
  // In-memory calibration edit -- sets x0/y0/theta0/y_up and REMOVES
  // "mocap_to_world_matrix" (see set_calibration_in_mapping()'s doc
  // comment for why the removal is required for the new values to take
  // effect). Same "in-memory only, call save_mapping() to persist"
  // convention as set_alias() above.
  void set_calibration(double x0, double y0, double theta0, bool y_up);
  // In-memory per-body yaw-offset edit -- sets mapping()["yaw_offset"]
  // [robot_name] (keyed on the PUBLISHED/planner name, e.g. "robot1", never
  // the raw Motive asset name). Same "in-memory only, call save_mapping()
  // to persist" convention as set_alias()/set_calibration() above.
  void set_yaw_offset(const std::string &robot_name, double yaw_offset_rad);
  // Writes the in-memory mapping back to config_.map_config_path (pretty,
  // preserving unknown fields -- see save_mapping_json()). Does NOT itself
  // restart the bridge: the bridge only reads its config at startup (see
  // this task's DESIGN doc) -- a caller that wants the edit to take effect
  // on a currently-running bridge must stop_bridge()+start_bridge() after a
  // successful save. False (*err set) if config_.map_config_path is empty
  // or the write fails.
  bool save_mapping(std::string *err = nullptr);
  TransformInfo transform_info() const { return parse_transform_info(mapping_json_); }

  // Reloads mapping_json_ from config_.map_config_path, DISCARDING any
  // unsaved in-memory edits (set_alias()/set_calibration() calls not yet
  // followed by a successful save_mapping()) and replacing it with whatever
  // is CURRENTLY on disk -- same load logic as the constructor, callable
  // again later so a caller that is about to compose a partial write (e.g.
  // MocapTab's calibration Apply, which must not clobber a field the user
  // never touched this session with a stale in-memory/GUI snapshot -- see
  // its own doc comment) can pick up an out-of-band edit made to the config
  // file since this process last loaded it. False (*err set, mapping_json_
  // left UNCHANGED) on an unreadable/malformed file -- same error surface
  // as the constructor's load, but reported to the caller instead of only
  // mapping_load_error().
  bool reload_mapping(std::string *err = nullptr);

private:
  void sub_thread_main(int port);
  void drain_stdout();
  void set_error(const std::string &reason);
  std::string resolve_bridge_path() const;
  // Resolves config_.map_config_path: returns it verbatim (normalized to an
  // absolute path via QFileInfo, so a caller-supplied relative override
  // still behaves sanely) if non-empty; otherwise synthesizes a default
  // relative to QCoreApplication::applicationDirPath() -- EXACTLY the same
  // convention resolve_bridge_path() above already uses for the bridge
  // BINARY, except one directory level further up: the built binaries live
  // at "<build_dir>/MARS/mars_sim_viz" / "<build_dir>/MPC/
  // optitrack_zmq_bridge" (siblings under the build dir, hence
  // resolve_bridge_path()'s single "/../"), but mocap_map_config.json is a
  // SOURCE-tree file that is never copied into the build dir at all, so
  // reaching it from "<build_dir>/MARS" needs one more "/.." to climb out
  // of the build dir entirely before descending into "MPC/
  // config/mocap_map_config.json" -- i.e. "<build_dir>/MARS/../../
  // MPC/config/mocap_map_config.json" resolves to "<repo_root>/
  // MPC/config/mocap_map_config.json", the real project file.
  // Called ONCE from the constructor (which then overwrites config_.
  // map_config_path with the result, so every other method -- including a
  // later start_bridge()'s --map-config passthrough -- sees the resolved,
  // never-empty value too).
  std::string resolve_map_config_path() const;

  MocapConfig config_;
  QProcess *proc_ = nullptr;
  bool bridge_should_be_running_ = false; // true after a successful
                                           // start_bridge() until the next
                                           // stop_bridge() (or an
                                           // unexpectedly-exited process
                                           // observed by tick(), which also
                                           // clears this).
  std::string stdout_buffer_; // partial-line carry-over across tick() calls.

  mutable std::mutex state_mutex_;
  MocapState state_ = MocapState::Disconnected;
  std::string error_reason_;

  mutable std::mutex inventory_mutex_;
  std::vector<MotiveAssetEntry> motive_assets_;

  std::thread sub_thread_;
  std::atomic<bool> sub_stop_requested_{false};
  mutable std::mutex bodies_mutex_;
  struct BodySample
  {
    Pose pose;
    double last_update_steady_s = -1.0; // relative to sub_start_time_.
    bool has_pose = false;
  };
  std::unordered_map<std::string, BodySample> bodies_;
  // Set (main thread) immediately before sub_thread_ is launched and only
  // ever READ by sub_thread_main() afterward -- no race, same "write once
  // before thread start" pattern LocalizationListener::start()'s
  // start_time_ uses, except here the SAME variable is also read by
  // bodies()/tick() on the main thread to compute ages against the exact
  // basis the thread itself stamps from (LocalizationListener instead
  // exposes only the derived has_fresh_pose() query, never a raw age).
  std::chrono::steady_clock::time_point sub_start_time_;
  bool sub_start_time_valid_ = false;

  nlohmann::json mapping_json_;
  std::string mapping_load_error_;
};

} // namespace simviz
