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
    double kick_cmd = 1200.0;      // same units as the active MotorMap's mode() (erpm or duty).
    double kick_ms = 150.0;
    double min_moving_speed_mps = 0.08;
    double kick_erpm_threshold = 200.0;
};

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
    double watchdog_ms = 250.0;
    double control_rate_hz = 50.0;
    double telemetry_rate_hz = 20.0;
    KickConfig kick;
    std::string calibration_file;  // empty => LinearMap with the placeholder gains above.
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
    double accel = 0.0;     // last VALID accel (m/s^2), held across malformed/missing payloads.
    double steering = 0.0;  // last VALID steering (rad), likewise held.
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

    // Safety-clamps one raw calib value to config().safety's limit for
    // `mode` (max_duty/max_erpm/max_current respectively). Public/static so
    // the REP "raw" verb handler (main.cpp) can compute the SAME
    // applied_value it must report back to the client without duplicating
    // this logic, and so tick()'s own calib-path clamp cannot drift from
    // it.
    static double clamp_raw_value(RawCalibMode mode, double value, const SafetyConfig& safety);

   private:
    enum class InternalState { kIdle, kActive, kWatchdogBrake };

    double compute_servo_pos(const TickInputs& in) const;
    MotorAction action_from_map_cmd(double cmd) const;  // clamps + wraps per map_->mode().
    double watchdog_brake_amps() const;

    DriverConfig config_;
    std::unique_ptr<MotorMap> map_;
    double erpm_per_mps_;
    bool used_calibration_ = false;
    std::string motor_map_note_;

    bool has_ticked_ = false;
    double last_tick_time_s_ = 0.0;

    double v_target_ = 0.0;

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
