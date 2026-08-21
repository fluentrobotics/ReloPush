// DriverCore.h
//
// Part 3 of VescDriver: the driver's core control logic. PURE (no sockets,
// no serial I/O, no wall-clock reads of its own) so it is fully
// unit-testable -- every call that needs "now" takes it as an explicit
// argument (monotonic seconds, caller's choice of epoch, only ever used for
// differencing). vesc_driver_main.cpp is the thin I/O shell around this:
// it owns the ZMQ sockets (ZmqChannels.h) and the real serial link
// (SerialPort.h/VescProtocol.h), decodes/holds the latest ackermann command
// (AckermannCodec.h) and REP-driven raw calib command, and calls
// DriverCore::tick() once per control-loop iteration.
//
// Portability: C++14 only (no structured bindings, no std::optional, no
// std::filesystem) -- see VescDriver/CMakeLists.txt's HARD PORTABILITY
// RULES. File I/O here (loading driver_config.json / a calibration JSON) is
// plain fstream + the vendored nlohmann::json, which is fine under the
// folder's "no sockets/serial" rule for DriverCore -- that rule is about
// runtime hardware channels, not reading a config file at startup.

#ifndef VESC_DRIVER_DRIVER_CORE_H_
#define VESC_DRIVER_DRIVER_CORE_H_

#include "SpeedGovernor.h"
#include "SteeringAngleMap.h"
#include "VelocityMap.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vesc {

// ---------------------------------------------------------------------
// Config -- mirrors VescDriver/config/driver_config.json key-for-key (see
// that file's own "_doc" field). Every field has an in-class default
// matching the shipped config so a DriverConfig default-constructs to
// something reasonable even before load_driver_config() runs.
// ---------------------------------------------------------------------

struct ServoConfig {
    bool enabled = true;
    double center = 0.5;
    double gain_per_rad = -0.65;
    bool invert = false;
    double min_pos = 0.0;  // clamp floor for the final computed servo_pos -- 0.0 preserves
                            // today's behavior exactly.
    double max_pos = 1.0;  // clamp ceiling -- 1.0 preserves today's behavior exactly.
};

struct SafetyConfig {
    double max_erpm = 3000.0;
    double max_duty = 0.15;
    double max_current = 8.0;
    // NOTE: incoming ackermann accel is clamped to +-safety_max_accel, NOT
    // to the nominal actuator max_accel=0.73 -- the LaunchGovernor on the
    // controller side legitimately publishes up to max_breakaway_accel=3.0
    // during launch kicks (see the frozen wire contract). |v_target| is
    // separately clamped to +-safety_max_v.
    double safety_max_accel = 3.5;
    double safety_max_v = 0.6;
};

struct KickConfig {
    bool enabled = true;
    double kick_cmd = 1200.0;      // same units as the active MotorMap's mode() (erpm or duty) -- actuation=="map" only.
    // actuation=="governor" only: the kick bypass emits this RAW DUTY
    // (vehicle frame, clamped to +-safety.max_duty like every other
    // governor output) instead of kick_cmd -- kick_cmd/kick_erpm_threshold
    // are erpm-flavored and don't make sense against a duty-only governor.
    // On kick end the governor's own slew state is seeded to this exact
    // (signed, clamped) value so its subsequent step()s decay smoothly
    // FROM here instead of jumping from 0 -- see DriverCore.cpp's tick().
    double kick_duty = 0.05;
    double kick_ms = 150.0;
    double min_moving_speed_mps = 0.08;
    double kick_erpm_threshold = 200.0;
};

// "accel" (default): wire accel is integrated into v_target_ every tick,
// wire speed is ignored -- EXACTLY DriverCore's pre-v2 behavior, byte for
// byte (see TickInputs's own AckermannHeld::accel comment and tick()'s
// own doc comments). "velocity": wire speed IS the target (per-field
// NaN-held, safety_max_v-clamped), wire accel is repurposed as a SLEW
// BOUND on how fast the internal setpoint may move toward that target
// (per-field NaN-held, clamped to (0, safety_max_accel]) -- see
// TickInputs's own comments for the exact per-field hold/clamp rules.
enum class CommandSemantics { kAccel, kVelocity };
// Unrecognized strings default to kAccel (documented fallback -- mirrors
// DriverConfig::command_semantics's own default).
CommandSemantics command_semantics_from_string(const std::string& s);
std::string to_string(CommandSemantics s);

// "map" (default -- see DriverCore.cpp's own note on why the COMPILED-IN
// default is "map" rather than the literal task spec's "governor": a
// "governor" compiled-in default would silently change every pre-existing
// DriverCore test/caller that never sets this field, and this driver's
// own "accel semantics: EXACTLY current behavior" contract requires the
// unconfigured case to keep behaving exactly as before this task. Fresh
// installs' config/driver_config.json ships this key EXPLICITLY, so real
// deployments are unaffected either way): v_target_ (however
// CommandSemantics produced it) is converted to a motor command via the
// legacy MotorMap (LinearMap/CalibratedMap, built from
// config.calibration_file -- completely unchanged from pre-v2).
// "governor": v_target_ is converted to a TARGET ERPM via a VelocityMap
// (config.velocity_calib_file, linear-fallback if absent/unloadable) and
// tracked by a SpeedGovernor (config.governor), emitting SET_DUTY only.
enum class ActuationMode { kMap, kGovernor };
// Unrecognized strings default to kMap.
ActuationMode actuation_from_string(const std::string& s);
std::string to_string(ActuationMode m);

struct DriverConfig {
    std::string robot_name = "robot2";
    int ackermann_port = 3160;
    int control_port = 3460;
    int telemetry_port = 3560;
    std::string serial_port;  // empty => auto-discovery
    int baud = 115200;
    std::string mode = "erpm";  // "erpm" | "duty" -- LinearMap's own mode when no calibration is loaded.
    double cmd_per_mps = 4614.0;
    double cmd_offset = 0.0;
    double erpm_per_mps = 4614.0;  // also the default used for telemetry's v_est and for v_now
                                     // whenever no calibration file overrides it (see build_motor_map()).
    ServoConfig servo;
    SafetyConfig safety;
    double watchdog_ms = 1500.0;   // Driver v2: raised from the pre-v2 default of 250ms (see
                                     // config/driver_config.json's own "_watchdog_ms_doc").
    double control_rate_hz = 50.0;
    double telemetry_rate_hz = 20.0;
    KickConfig kick;
    std::string calibration_file;  // empty => LinearMap with the placeholder gains above.

    // --- Driver v2 additions (see CommandSemantics/ActuationMode above) ---
    std::string command_semantics = "accel";  // "accel" | "velocity".
    std::string actuation = "map";            // "map" | "governor" -- see ActuationMode's own comment.
    double wheel_base = 0.29;                 // meters -- also cross-checked against a loaded
                                                // SteeringAngleMap's own wheel_base at startup (main.cpp).
    // velocity semantics ONLY: default/held slew bound (m/s^2) applied
    // before the first finite wire-accel value ever arrives -- see
    // TickInputs's own comment. Deliberately the SAME numeric value as
    // DriverCore.cpp's kWatchdogRampAccel (0.73, the nominal actuator
    // max_accel) -- not a coincidence, both represent "how fast this
    // actuator can reasonably change speed" in the absence of a
    // controller-supplied number.
    double default_slew_mps2 = 0.73;
    // Governor sub-config (kp/ki/ff_gain/duty_slew_per_s/erpm_filter_tau_s)
    // -- reuses SpeedGovernorConfig directly (see SpeedGovernor.h) so its
    // defaults are defined in exactly one place. NOTE: this struct's own
    // max_duty field is UNUSED here -- DriverCore always overrides it
    // from safety.max_duty when actually configuring its SpeedGovernor
    // instance (a single authoritative actuation-output ceiling, shared
    // with the "map" actuation's own clamp), never a second, possibly-
    // drifted copy.
    SpeedGovernorConfig governor;
    // Resolved, already-precedence-applied file paths (explicit CLI flag
    // > $HOME/.vesc/<name>.json > <exe_dir>/../config/<name>.example.json)
    // -- see vesc_driver_main.cpp's own resolution logic. Deliberately
    // NOT driver_config.json keys themselves (mirrors SteeringCalib's own
    // CLI/env-resolved-only convention, see SteeringCalib.h) -- empty
    // means "use the built-in fallback" (VelocityMap's linear defaults /
    // the legacy servo path, respectively).
    std::string velocity_calib_file;
    std::string steering_angle_map_file;
};

// Loads VescDriver/config/driver_config.json-shaped JSON from `path`.
//   - File missing/unreadable, or its top level not a JSON object: ok=false,
//     config left at DriverConfig{} defaults, `error` describes why.
//   - File present and parses: `config` starts from DriverConfig{} defaults
//     and each top-level/nested key overrides its own field independently
//     (missing key -> default retained); a nested object ("servo"/"safety"/
//     "kick") that is present but not itself a JSON object is skipped
//     wholesale (that sub-struct's defaults retained, everything else
//     unaffected) with a note appended to `warnings`. ok=true whenever the
//     file itself parsed as JSON, regardless of how many individual
//     sub-fields needed to fall back to a default.
struct ConfigLoadResult {
    bool ok = false;
    DriverConfig config;
    std::vector<std::string> warnings;
    std::string error;
};
ConfigLoadResult load_driver_config(const std::string& path);

// ---------------------------------------------------------------------
// Motor mapping: v_target (+ v_now, + a_desired) -> a raw command value in
// whichever units mode() reports.
// ---------------------------------------------------------------------

enum class MapMode { kErpm, kDuty };

inline std::string to_string(MapMode m) { return m == MapMode::kErpm ? "erpm" : "duty"; }
// Unrecognized strings default to kErpm (documented fallback -- mirrors
// DriverConfig::mode's own default).
MapMode map_mode_from_string(const std::string& s);

class MotorMap {
   public:
    virtual ~MotorMap() = default;
    virtual MapMode mode() const = 0;
    // v_now: current measured speed (m/s, from erpm_meas/erpm_per_mps).
    // v_target: desired speed (m/s). a_desired: desired acceleration
    // (m/s^2) -- consumed only by CalibratedMap's bilinear grid lookup;
    // LinearMap ignores both v_now and a_desired.
    virtual double compute_cmd(double v_now, double v_target, double a_desired) const = 0;
};

class LinearMap : public MotorMap {
   public:
    LinearMap(MapMode mode, double cmd_offset, double cmd_per_mps)
        : mode_(mode), cmd_offset_(cmd_offset), cmd_per_mps_(cmd_per_mps) {}
    MapMode mode() const override { return mode_; }
    double compute_cmd(double v_now, double v_target, double a_desired) const override;

   private:
    MapMode mode_;
    double cmd_offset_;
    double cmd_per_mps_;
};

// One (v_mps x a_mps2) -> cmd grid, exactly as the frozen calibration
// schema's "grid" object: v_mps/a_mps2 ascending, cmd[row per v][col per a].
struct CalibrationGrid {
    std::vector<double> v_mps;
    std::vector<double> a_mps2;
    std::vector<std::vector<double>> cmd;
};

// Bilinear lookup, clamped to the grid's edges in both axes (a query
// outside [v_mps.front(),v_mps.back()] or [a_mps2.front(),a_mps2.back()] is
// clamped to the nearest edge before interpolating -- never extrapolates).
// Degenerate axes (size 1) are handled as a constant along that axis.
// Precondition: grid.v_mps/a_mps2 non-empty and grid.cmd correctly shaped
// (grid.cmd.size()==v_mps.size(), each row's size()==a_mps2.size()) --
// callers only ever reach this via a CalibrationData that
// parse_calibration_json() has already validated to that shape.
double bilinear_lookup(const CalibrationGrid& grid, double v, double a);

// Parsed contents of one FROZEN-schema calibration JSON file (see the task
// brief's schema comment, reproduced in DriverCore.cpp above
// parse_calibration_json()). Only the fields DriverCore's CalibratedMap
// actually consumes are kept (mode/erpm_per_mps/cmd_per_mps/cmd_offset/
// grid) -- "stall"/"v_ss"/"meta" are accepted (not rejected) but not
// otherwise used by the driver.
struct CalibrationData {
    MapMode mode = MapMode::kErpm;
    double erpm_per_mps = 0.0;
    double cmd_per_mps = 0.0;
    double cmd_offset = 0.0;
    bool has_grid = false;
    CalibrationGrid grid;
};

struct CalibrationParseResult {
    bool ok = false;
    CalibrationData data;
    std::string error;
};

// Parses one calibration JSON document from an in-memory string (no file
// I/O -- see load_calibration_file() below for the file-reading wrapper).
// ok=false (data left default) on: invalid JSON, a top level that isn't an
// object, a missing/wrong-typed required key (mode/erpm_per_mps/
// cmd_per_mps/cmd_offset), or a "grid" that is present, non-null, and
// malformed (wrong shape / non-ascending axis / empty axis). "grid":null or
// "grid" entirely absent both yield has_grid=false (the documented
// linear-fallback case) -- not a parse failure.
CalibrationParseResult parse_calibration_json(const std::string& json_text);

// Reads `path` and parses it via parse_calibration_json(). ok=false (data
// left default) if the file cannot be opened for reading OR its contents
// fail to parse; `error` describes which.
CalibrationParseResult load_calibration_file(const std::string& path);

// Builds the MotorMap a DriverConfig implies: if config.calibration_file is
// non-empty and load_calibration_file() succeeds, a CalibratedMap built
// from it (used_calibration=true, effective_erpm_per_mps taken from the
// calibration file's own erpm_per_mps -- it was fit alongside that exact
// conversion). Otherwise (empty path, unreadable file, or a parse failure)
// falls back to a LinearMap built from config.mode/cmd_offset/cmd_per_mps
// (used_calibration=false, effective_erpm_per_mps = config.erpm_per_mps) --
// this is the "absent file -> LinearMap with config placeholder gains"
// fallback the task brief requires; a present-but-broken file falls back
// the same way (never a hard failure), with `note` explaining why.
struct MotorMapBuildResult {
    std::unique_ptr<MotorMap> map;
    double effective_erpm_per_mps = 0.0;
    bool used_calibration = false;
    std::string note;
};
MotorMapBuildResult build_motor_map(const DriverConfig& config);

// ---------------------------------------------------------------------
// Tick I/O
// ---------------------------------------------------------------------

enum class Source { kAckermann, kCalib };

enum class RawCalibMode { kDuty, kErpm, kCurrent };
RawCalibMode raw_calib_mode_from_string(const std::string& s, bool* ok);
std::string to_string(RawCalibMode m);

// The caller (main loop) owns receiving/decoding both command streams and
// tracking how long ago each was last refreshed by something VALID; this
// struct is just that already-resolved snapshot, handed to tick() fresh
// every call. See each field's own comment for exactly what "held" means.
struct AckermannHeld {
    // true iff a valid ackermann payload has EVER been decoded (i.e. there
    // is something meaningful to act on at all) -- false forever until the
    // first valid one arrives, at which point it latches true (there is no
    // "un-receive").
    bool valid = false;
    // last VALID accel (m/s^2), held across malformed/missing payloads.
    // "accel" semantics: integrated into v_target_ every tick (unchanged
    // pre-v2 meaning). "velocity" semantics: repurposed as a SLEW BOUND
    // on the internal setpoint's approach to `speed` below -- see
    // CommandSemantics's own comment.
    double accel = 0.0;
    double steering = 0.0;  // last VALID steering (rad), likewise held.
    // last VALID speed (m/s), likewise held. Consumed ONLY in "velocity"
    // command_semantics (the target v_target_ setpoint slews toward);
    // ignored (never read) in "accel" semantics, exactly like every
    // pre-v2 version of this driver ignored it.
    double speed = 0.0;
    // Seconds since that last VALID payload -- the caller must NOT advance
    // the "last valid" timestamp on a malformed payload (per the frozen
    // wire contract: "Malformed payloads must NOT refresh the watchdog
    // timer"). Meaningless (ignored) while valid==false.
    double age_s = 1.0e18;
};

struct CalibHeld {
    // true iff a raw calib command has ever been set via the REP "raw" verb
    // and not since cleared (by DriverCore::stop() -- see that method).
    bool has_command = false;
    RawCalibMode mode = RawCalibMode::kDuty;
    double value = 0.0;       // ALREADY safety-clamped (see DriverCore::clamp_raw_value()).
    double age_s = 1.0e18;    // seconds since this command was (re)set.
    double ttl_ms = 500.0;    // this command's own TTL, as given (or defaulted) on the "raw" call.
};

struct TickInputs {
    double now_s = 0.0;  // monotonic seconds, caller's own epoch -- only ever differenced internally.
    Source source = Source::kAckermann;
    AckermannHeld ackermann;
    CalibHeld calib;
    // REP "servo" verb state, already resolved by the caller: active means
    // an explicit position is in effect: DriverCore never clears this
    // itself (nor decides when the ackermann stream should reclaim it) --
    // per the frozen contract ("in ackermann source the ackermann steering
    // stream overrides it on next command") the CALLER clears
    // servo_override_active the instant a fresh valid ackermann payload
    // arrives while source==kAckermann, before building the next
    // TickInputs.
    bool servo_override_active = false;
    double servo_override_value = 0.5;  // 0..1, meaningful only when servo_override_active.
    double erpm_meas = 0.0;             // latest VESC-reported electrical RPM (0 if never measured yet).
    // Latest VESC-reported battery voltage (from GET_VALUES), VEHICLE-
    // frame-independent (a scalar, no sign convention) -- only consumed
    // by actuation=="governor"'s SpeedGovernor feedforward term (see
    // SpeedGovernor.h's own v_in()/feed_vin() comments, which floor the
    // EFFECTIVE value at 6.0V regardless of what's passed here). The
    // caller (main.cpp) is responsible for "when stale, keep last" --
    // DriverCore itself applies no staleness logic of its own, exactly
    // like erpm_meas above.
    double v_in = 0.0;
};

struct MotorAction {
    enum class Type { kNone, kRpm, kDuty, kCurrent, kBrake };
    Type type = Type::kNone;
    double value = 0.0;  // units depend on type: erpm (kRpm), fraction (kDuty), amps (kCurrent/kBrake).
};

struct TickResult {
    MotorAction motor;
    double servo_pos = 0.5;  // 0..1, always populated (caller decides whether to send it based on config.servo.enabled).
    double v_target = 0.0;   // exposed for telemetry/tests, not otherwise consumed by the caller.
};

// The core stateful control-tick machine. One instance per running driver
// process; every method that needs "now" takes it explicitly (see the file
// header) so tests can drive it with synthetic time instead of real sleeps.
class DriverCore {
   public:
    explicit DriverCore(DriverConfig config);

    // Installs a MotorMap explicitly (used by tests to inject a map without
    // going through build_motor_map()/a real config.calibration_file path).
    // Also updates the erpm_per_mps DriverCore itself uses for v_now, if
    // `effective_erpm_per_mps` > 0 (<=0 leaves the previous value
    // unchanged -- lets a test swap only the map without also having to
    // know/repeat the right conversion factor).
    void set_motor_map(std::unique_ptr<MotorMap> map, double effective_erpm_per_mps = -1.0);

    // Advances one control tick and returns this tick's output. See
    // TickInputs's own field comments for what the caller must have
    // resolved before calling this.
    TickResult tick(const TickInputs& in);

    // Immediate stop (REP "stop" verb, legal in any source): forces
    // v_target to 0 and cancels an in-progress kick, and returns a brake
    // MotorAction the caller should send to the VESC right away (bypassing
    // the normal tick() cadence). Does NOT touch source or the servo
    // override -- only actuation state. The caller is responsible for also
    // clearing its OWN held raw calib command (CalibHeld::has_command)
    // before the next tick() call, per the contract's "raw command
    // cleared" -- DriverCore has no visibility into that caller-owned
    // state to clear it itself.
    MotorAction stop();

    // "idle" (no command ever held for the CURRENT source), "active"
    // (holding/acting on a command, not braking), or "watchdog_brake"
    // (ackermann watchdog engaged, or the calib raw command's TTL expired)
    // -- exactly the three values the control REP's "ping" verb reports.
    std::string state_string() const;

    double v_target() const { return v_target_; }
    const DriverConfig& config() const { return config_; }
    double effective_erpm_per_mps() const { return erpm_per_mps_; }
    MapMode map_mode() const { return map_ ? map_->mode() : MapMode::kErpm; }
    // Outcome of the constructor's own build_motor_map(config) call --
    // exposed so vesc_driver_main.cpp can print a startup summary without
    // re-parsing the calibration file itself (build_motor_map() is also
    // freely callable standalone, e.g. by tests, since it takes a
    // DriverConfig by value and has no other side effects).
    bool used_calibration() const { return used_calibration_; }
    const std::string& motor_map_note() const { return motor_map_note_; }

    // --- Driver v2 status accessors (startup banner / telemetry) ---
    CommandSemantics command_semantics() const { return semantics_; }
    ActuationMode actuation() const { return actuation_; }
    // True iff config.velocity_calib_file loaded a real TABLE (false
    // means the built-in linear fallback is in effect, whether because
    // no file was configured or because it failed to load/parse).
    bool velocity_map_used_table() const { return velocity_map_used_table_; }
    const std::string& velocity_map_note() const { return velocity_map_note_; }
    // True iff config.steering_angle_map_file loaded successfully (false
    // means compute_servo_pos() is using the legacy center+gain_per_rad+
    // invert affine path).
    bool has_steering_angle_map() const { return has_steering_angle_map_; }
    const std::string& steering_angle_map_note() const { return steering_angle_map_note_; }
    // Meaningful only when has_steering_angle_map() -- the loaded map's
    // OWN wheel_base, for vesc_driver_main.cpp's startup mismatch check
    // against config().wheel_base (see that file's own comment).
    double steering_angle_map_wheel_base() const { return steering_angle_map_.wheel_base(); }

    // Safety-clamps one raw calib value to config().safety's limit for
    // `mode` (max_duty/max_erpm/max_current respectively). Public/static so
    // the REP "raw" verb handler (main.cpp) can compute the SAME
    // applied_value it must report back to the client without duplicating
    // this logic, and so tick()'s own calib-path clamp cannot drift from
    // it.
    static double clamp_raw_value(RawCalibMode mode, double value, const SafetyConfig& safety);

   private:
    enum class InternalState { kIdle, kActive, kWatchdogBrake };

    // No longer const: holds/updates last_finite_steering_ (see the
    // Driver v2 "steering NaN hold" change, .cpp).
    double compute_servo_pos(const TickInputs& in);
    MotorAction action_from_map_cmd(double cmd) const;  // clamps + wraps per map_->mode().
    double watchdog_brake_amps() const;
    // Dispatches v_target->motor-command conversion on actuation_ (map vs
    // governor) -- shared by both the watchdog-ramp branch and normal
    // tracking, see .cpp. a_desired is only consumed by actuation==kMap's
    // CalibratedMap path; erpm_meas/v_in are only consumed by
    // actuation==kGovernor's SpeedGovernor.
    MotorAction actuate(double v_now, double v_target, double a_desired, double dt, double erpm_meas, double v_in);
    // Resets governor_ -- called from every DriverCore-internal "clean
    // slate" point (idle, calib-source entry, watchdog-zero-brake,
    // stop()) -- see .cpp's own call sites for why ("stop/brake/
    // watchdog-zero/abort paths" per the task brief).
    void reset_governor();

    DriverConfig config_;
    std::unique_ptr<MotorMap> map_;
    double erpm_per_mps_;
    bool used_calibration_ = false;
    std::string motor_map_note_;

    CommandSemantics semantics_ = CommandSemantics::kAccel;
    ActuationMode actuation_ = ActuationMode::kMap;

    VelocityMap velocity_map_;
    bool velocity_map_used_table_ = false;
    std::string velocity_map_note_;

    SteeringAngleMap steering_angle_map_;
    bool has_steering_angle_map_ = false;
    std::string steering_angle_map_note_;

    SpeedGovernor governor_;

    bool has_ticked_ = false;
    double last_tick_time_s_ = 0.0;

    double v_target_ = 0.0;

    // "velocity" command_semantics ONLY -- per-field NaN-held wire
    // speed/accel (accel repurposed as a slew bound there) -- see
    // AckermannHeld's own comments. Irrelevant/unused in "accel"
    // semantics. slew_bound_hold_ is seeded from config_.default_slew_mps2
    // in the constructor (init value, per the task brief); velocity_
    // target_hold_ always starts at 0.
    double velocity_target_hold_ = 0.0;
    double slew_bound_hold_ = 0.0;

    // Last finite steering angle (rad) ever seen on the ackermann stream
    // -- Driver v2 "steering NaN hold" change (see compute_servo_pos()):
    // a NaN/Inf steering value now HOLDS this instead of recentering to
    // 0. Init 0, per the task brief.
    double last_finite_steering_ = 0.0;

    // Ackermann-path watchdog (mirrors mpc::Watchdog's engaged/just_engaged
    // semantics -- see MPC/include/mpc/SimCore.h).
    bool watchdog_engaged_ = false;

    // Kick-start lifecycle (ackermann path only -- see TickInputs's kick
    // condition in the .cpp).
    bool kicking_ = false;
    int kick_sign_ = 1;
    double kick_start_time_s_ = 0.0;

    InternalState state_ = InternalState::kIdle;
};

}  // namespace vesc

#endif  // VESC_DRIVER_DRIVER_CORE_H_
