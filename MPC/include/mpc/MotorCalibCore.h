#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// MOTOR CALIBRATION TOOL (main-PC side testable core). NO sockets live here
// -- every dependency the state machine needs (the driver's control-plane
// REQ/REP, the clock) is dependency-injected, so the whole thing runs and
// is unit-tested without ZMQ (see MPC/tests/motor_calib_tests.cpp). A later
// agent wires this to a real ZMQ IDriverClient + a real OptiTrack pose feed
// inside a `motor_calibration` main() binary; this file only builds the
// library pieces.
//
// PURPOSE: the robot (RC car, BLDC + VESC) drives back and forth along a
// straight line (default 4.5m) under constant raw motor commands while
// OptiTrack mocap supplies planner-frame poses. From the resulting per-trial
// CSV logs we fit the stall threshold, steady-state v(cmd), the linear
// cmd(v) fallback, and the full dynamic cmd(v,a) grid, then export them in
// the FROZEN CALIBRATION OUTPUT SCHEMA the real-hardware driver's
// CalibratedMap loads (see export_calibration()/parse_calibration() below).
//
// UNLIKE most other mpc:: headers (compare SimCore.h/RobotSpec.h, which
// keep nlohmann::json out of their public signatures and confine it to
// their .cpp), this header exposes nlohmann::json directly in
// export_calibration()/parse_calibration(): those two functions ARE the
// frozen-schema boundary, and the round-trip unit test builds/inspects
// nlohmann::json objects in memory (no file I/O needed for that test).
namespace mpc {

// ---------------------------------------------------------------------
// Raw pose / telemetry samples.
// ---------------------------------------------------------------------

// One planner-frame mocap pose sample.
struct CalibSample {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// One VESC telemetry sample (see FROZEN DRIVER CONTROL PROTOCOL's telemetry
// PUB payload -- only the fields this tool actually consumes/logs).
struct TelemetrySample {
    double t = 0.0;
    double erpm = 0.0;
    double duty = 0.0;
    double current_motor = 0.0;
    double v_in = 0.0;
};

// ---------------------------------------------------------------------
// VelocityEstimator: heading-projected finite-difference velocity +
// centered-window smoothing/acceleration.
// ---------------------------------------------------------------------

// One feed() call's output. v_raw/v_raw_valid describe the sample JUST
// FED. v_smooth/a_smooth describe a EARLIER sample -- see class doc
// comment for why a centered window has inherent latency.
struct VelocityEstimate {
    // Heading-projected finite-difference velocity, computed EXACTLY like
    // MPC/src/main.cpp:773-787 ((dx*cos(yaw)+dy*sin(yaw))/dt, current
    // heading, dt>1e-3 guard): v_raw_valid is false only for the very first
    // sample ever fed (nothing to difference against yet) -- once true it
    // stays true for the estimator's lifetime; a too-close-together dt just
    // carries the previous value forward unchanged (matching main.cpp's
    // est_state.v retention), it never flips v_raw_valid back to false.
    double v_raw = 0.0;
    bool v_raw_valid = false;

    // Centered 5-sample moving average of v_raw (t_smooth is the CENTER
    // sample's own timestamp, i.e. ~2 samples BEHIND the one just fed --
    // t_smooth <= the fed t), and a_smooth = d(v_smooth)/dt between this
    // and the PREVIOUS smoothed sample. smooth_valid requires 5 fed
    // samples; a_valid additionally requires a 6th (a prior smoothed value
    // to difference against).
    double t_smooth = 0.0;
    double v_smooth = 0.0;
    bool smooth_valid = false;
    double a_smooth = 0.0;
    bool a_valid = false;
};

// Feed DISTINCT pose samples (caller's responsibility -- e.g. only feed on
// a genuinely new mocap frame, never a re-delivered duplicate) in
// increasing t order. Pure/stateful, no I/O.
class VelocityEstimator {
public:
    VelocityEstimate feed(double t, double x, double y, double yaw);
    void reset();

private:
    static constexpr std::size_t kWindow = 5;

    bool have_prev_pose_ = false;
    double prev_t_ = 0.0, prev_x_ = 0.0, prev_y_ = 0.0;

    double last_v_raw_ = 0.0;
    bool have_v_raw_ = false;

    struct RawPoint {
        double t;
        double v;
    };
    std::deque<RawPoint> window_;

    bool have_prev_smooth_ = false;
    double prev_smooth_t_ = 0.0, prev_smooth_v_ = 0.0;
};

// ---------------------------------------------------------------------
// LineFrame: the 1-D (longitudinal/lateral) frame the calibration line
// itself defines.
// ---------------------------------------------------------------------

// Captures an origin p0=(x0,y0) and a line heading, then projects any
// planner-frame point onto that line (s, longitudinal) and its normal
// (lateral). Captured ONCE per calibration session (TrialRunner captures it
// lazily off the first pose sample it ever sees -- see that class's doc
// comment); a second capture() call is a no-op unless `force` is passed.
class LineFrame {
public:
    // line_yaw = yaw_override.value_or(initial_yaw) -- i.e. an explicit
    // config override (MotorCalibConfig::Line::line_yaw_rad) wins when
    // present, otherwise the robot's own heading at capture time defines
    // the line axis (matching the physical setup: the operator points the
    // car along the line before starting).
    void capture(double x0, double y0, double initial_yaw,
                 std::optional<double> yaw_override = std::nullopt, bool force = false);

    bool captured() const { return captured_; }
    double x0() const { return x0_; }
    double y0() const { return y0_; }
    double line_yaw() const { return line_yaw_; }

    // Longitudinal coordinate: (p-p0) . (cos(line_yaw), sin(line_yaw)).
    double s(double x, double y) const;
    // Lateral coordinate: (p-p0) . (-sin(line_yaw), cos(line_yaw)).
    double lateral(double x, double y) const;

private:
    bool captured_ = false;
    double x0_ = 0.0, y0_ = 0.0, line_yaw_ = 0.0;
};

// ---------------------------------------------------------------------
// Driver control-plane client (dependency-injected -- see file header
// comment). Mirrors the FROZEN DRIVER CONTROL PROTOCOL's REQ/REP verbs
// exactly; a later agent implements a real ZMQ-backed version. Tests here
// use a scripted fake.
// ---------------------------------------------------------------------

// Command mode for raw(), matching the frozen wire protocol's "mode" field.
// kCurrent is a legal DRIVER command mode but NOT a legal calibration
// EXPORT mode -- the frozen calibration schema's own "mode" field is only
// ever "erpm"|"duty" (see export_calibration()).
enum class RawMode { kDuty, kErpm, kCurrent };

std::string to_string(RawMode mode);
// Throws std::invalid_argument if `s` is not exactly "duty"|"erpm"|"current".
RawMode raw_mode_from_string(const std::string& s);

// Minimal reply shape every driver control-plane verb needs: `ok` mirrors
// the wire protocol's {"ok":bool,...}; `error` mirrors the wire's "error"
// string (present only when ok==false); `applied_value` is populated only
// by raw() (the frozen protocol's "applied_value" -- the value actually
// applied after the driver's own safety clamp).
struct DriverReply {
    bool ok = false;
    std::string error;
    double applied_value = 0.0;
};

class IDriverClient {
public:
    virtual ~IDriverClient() = default;
    virtual DriverReply ping() = 0;
    virtual DriverReply set_source(const std::string& source) = 0;  // "ackermann"|"calib"
    virtual DriverReply raw(RawMode mode, double value, double ttl_ms = 500.0) = 0;
    virtual DriverReply servo(double value) = 0;
    virtual DriverReply stop() = 0;
};

// ---------------------------------------------------------------------
// TrialRunner: the per-trial state machine.
// ---------------------------------------------------------------------

enum class TrialState { kIdle, kArm, kRun, kStopping, kDone, kAborted };

enum class TrialAbortReason {
    kNone,
    kMocapStale,             // mocap sample age exceeded mocap_stale_abort_ms
    kOutOfEnvelope,          // s has traveled beyond the room available EITHER in the commanded
                             // direction (room_ahead_) OR in the opposite direction (room_behind_) --
                             // an absolute |s - trial_start_s| envelope, not scoped to only the
                             // commanded-direction excursion (motion the wrong way -- e.g. reversed
                             // motor polarity, or rollback on a slope -- must trip this too)
    kOverSpeed,              // |v| exceeded max_speed_abort_mps
    kTimeout,                // wall time since RUN entry exceeded max_duration_s
    kInsufficientRoomAtArm,  // ARM-time precheck: commanded direction has no usable room left
    kDriverRefused,          // set_source(calib) or a mid-run raw() came back ok=false
};
std::string to_string(TrialAbortReason reason);

// One constant-command trial request. `value`'s SIGN is the commanded
// direction (used by the ARM-time room precheck and the RUN-time envelope
// check) -- callers are expected to alternate the sign across successive
// trials (a full back-and-forth sweep), but TrialRunner does not itself
// enforce alternation.
struct TrialSpec {
    RawMode mode = RawMode::kDuty;
    double value = 0.0;
    double ttl_ms = 500.0;
};

// One logged row. Logged ONLY while the trial is in kRun (a STOPPING
// sample has no active raw command to attribute cmd_mode/cmd_value to, and
// contributes no useful steady-state/dynamic-fit information -- see
// steady_state_v()'s doc comment). has_telemetry=false means no telemetry
// sample had been seen yet at logging time (nearest-timestamp join found
// nothing to join against); erpm/duty/current_motor/v_in are left at 0.0
// in that case, and write_csv() emits empty fields for them.
struct TrialRow {
    double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0, s = 0.0;
    double v_raw = 0.0, v_smooth = 0.0;
    RawMode cmd_mode = RawMode::kDuty;
    double cmd_value = 0.0;
    bool has_telemetry = false;
    double erpm = 0.0, duty = 0.0, current_motor = 0.0, v_in = 0.0;
};

struct TrialRunnerConfig {
    // Line geometry (see MotorCalibConfig::Line -- make_trial_runner_config()
    // copies these two straight across).
    double line_length_m = 4.5;
    double end_margin_m = 0.75;

    // Safety (see MotorCalibConfig::Safety).
    double mocap_stale_abort_ms = 300.0;
    double max_speed_abort_mps = 0.8;
    double max_duration_s = 6.0;  // wall time since RUN entry (spans RUN+STOPPING)

    // Trial timing (see MotorCalibConfig::Trial).
    double settle_s = 1.5;   // normal RUN duration before STOPPING is entered
    double arm_settle_s = 1.0;

    // Knobs with no config-file counterpart (kept at sensible compiled
    // defaults; make_trial_runner_config() does not touch these).
    double arm_stationary_v_mps = 0.02;  // |v| threshold for "stationary" (ARM entry AND STOPPING exit)
    double reissue_interval_s = 0.2;     // re-issue raw() this often to keep the driver's TTL alive
    double center_servo_value = 0.5;     // commanded once at RUN entry -- straight-line run, no steering
};

// Pure, unit-testable, no-sockets state machine (see file header comment).
// Owns its own LineFrame + VelocityEstimator, both of which stay warm
// across trials within one TrialRunner instance (a fresh TrialRunner is one
// calibration SESSION, not one trial) -- only rows()/the per-trial spec_
// reset on start_trial().
//
// Driving a session: repeatedly call step() (even before the first
// start_trial(), so the LineFrame/VelocityEstimator can warm up off
// whatever poses arrive first); once ready, call start_trial() to begin one
// trial; keep calling step() until it returns false (state() is kDone or
// kAborted); read rows()/write_csv(); call start_trial() again for the next
// command in the sweep.
class TrialRunner {
public:
    explicit TrialRunner(IDriverClient& driver, TrialRunnerConfig cfg = {},
                          std::optional<double> line_yaw_override = std::nullopt);

    // Begins a new trial. Legal only when state() is kIdle, kDone, or
    // kAborted (i.e. no trial currently in flight) -- callers should check
    // state() first; this does not itself assert/throw on misuse so a test
    // can exercise "start over top of a live trial" as a caller bug if it
    // wants to, but production callers must not do this.
    void start_trial(TrialSpec spec);

    // Advances the state machine by one tick. `now` is an injected
    // monotonic clock (seconds) -- the caller owns time entirely, so tests
    // script exact timings with no real sleeps. `have_pose`/`pose` is this
    // tick's fresh mocap sample (have_pose=false when none arrived this
    // tick); `have_telemetry`/`telemetry` likewise for VESC telemetry.
    // Returns false once state() has settled at kIdle/kDone/kAborted
    // (nothing left to do this tick); true means a trial is actively
    // ARM/RUN/STOPPING.
    bool step(double now, bool have_pose, const CalibSample& pose, bool have_telemetry,
              const TelemetrySample& telemetry);

    TrialState state() const { return state_; }
    TrialAbortReason abort_reason() const { return abort_reason_; }
    const std::string& abort_message() const { return abort_message_; }
    const std::vector<TrialRow>& rows() const { return rows_; }
    const LineFrame& line() const { return line_; }

    // Writes rows() to `path` as CSV with EXACTLY this header:
    // t,x,y,yaw,s,v_raw,v_smooth,cmd_mode,cmd_value,erpm,duty,current_motor,v_in
    // Returns false if the file could not be opened for writing.
    bool write_csv(const std::string& path) const;

private:
    void log_row();
    bool abort(TrialAbortReason reason, const std::string& message);
    void enter_run(double now);

    IDriverClient& driver_;
    TrialRunnerConfig cfg_;
    std::optional<double> line_yaw_override_;

    LineFrame line_;
    VelocityEstimator vel_;

    TrialState state_ = TrialState::kIdle;
    TrialAbortReason abort_reason_ = TrialAbortReason::kNone;
    std::string abort_message_;

    TrialSpec spec_;
    std::vector<TrialRow> rows_;
    std::vector<TelemetrySample> telemetry_buffer_;

    // Pose/velocity bookkeeping, kept warm across trials.
    bool have_seen_pose_ = false;
    double last_pose_wall_t_ = 0.0;  // `now` at the last fresh pose
    double last_pose_x_ = 0.0, last_pose_y_ = 0.0, last_pose_yaw_ = 0.0, last_pose_t_ = 0.0;
    double last_v_raw_ = 0.0;
    double last_v_smooth_ = 0.0;
    bool have_valid_smooth_ = false;

    // Per-trial bookkeeping.
    double arm_stationary_since_ = -1.0;  // `now` the current stationary run started, <0 == not stationary
    double trial_start_s_ = 0.0;
    double room_ahead_ = 0.0;   // usable travel (m) in the commanded direction, captured at ARM->RUN
    double room_behind_ = 0.0;  // usable travel (m) in the OPPOSITE direction, captured at ARM->RUN --
                                 // guards against wrong-direction excursions (reversed polarity,
                                 // rollback) that dir*(s_now - trial_start_s_) alone never catches.
    double run_entered_time_ = 0.0;
    double last_raw_issue_time_ = 0.0;
};

// ---------------------------------------------------------------------
// Fitters (pure functions over logged rows -- see file header comment).
// ---------------------------------------------------------------------

struct SteadyStatePoint {
    double cmd = 0.0;
    double v_ss = 0.0;
    bool ok = false;
};

// Mean v_smooth over the TRAILING `steady_window_frac` (by time span) of
// `rows` (expected to be one trial's kRun-phase rows, all under the SAME
// constant cmd_value -- see TrialRow's doc comment for why STOPPING rows
// must not be included), restricted to samples within that trailing window
// whose local |dv/dt| is small (below kSmallDvDtThresh, see .cpp) so an
// early, not-yet-converged sample can't skew the estimate; if NONE of the
// trailing-window samples pass that filter, falls back to the full trailing
// window unfiltered (keeps the result defined rather than spuriously
// ok=false). ok=false if `rows` is empty. cmd is taken from the trailing
// window's own (constant) cmd_value.
SteadyStatePoint steady_state_v(const std::vector<TrialRow>& rows, double steady_window_frac = 0.4);

struct StallFit {
    double min_cmd = 0.0;              // min |cmd| among `points` whose |v_ss| > min_moving
    double kick_cmd = 0.0;              // 1.5 * min_cmd, capped at the max swept |cmd|
    double kick_ms = 150.0;
    double min_moving_speed_mps = 0.0;  // echoes the `min_moving` argument, for export convenience
    bool ok = false;                    // false if no point in `points` clears min_moving
};
StallFit fit_stall(const std::vector<SteadyStatePoint>& points, double min_moving = 0.03);

struct LinearFit {
    double cmd_offset = 0.0;
    double cmd_per_mps = 0.0;
    double fit_rms = 0.0;
    bool ok = false;  // false if fewer than 2 points cleared `stall_min_cmd`
};
// Least squares cmd = cmd_offset + cmd_per_mps*v over `points` with
// ok==true and |cmd| > stall_min_cmd (i.e. strictly above the stall
// threshold -- see fit_stall()'s min_cmd).
LinearFit fit_linear(const std::vector<SteadyStatePoint>& points, double stall_min_cmd);

// Default v/a grids: v_mps = {0.00, 0.05, ..., 0.50} (11 points), a_mps2 =
// {-1.0, -0.8, ..., 1.0} (11 points) -- matches the task's "e.g. 0..0.5 step
// 0.05 x -1..1 step 0.2".
const std::vector<double>& default_grid_v_mps();
const std::vector<double>& default_grid_a_mps2();

struct GridFit {
    std::vector<double> v_mps;
    std::vector<double> a_mps2;
    std::vector<std::vector<double>> cmd;        // cmd[i][j]: row per v_mps[i], col per a_mps2[j]
    std::vector<std::vector<bool>> supported;     // true where cmd[i][j] came from a direct per-v-bin
                                                   // linear fit (not neighbor interpolation/extrapolation)
                                                   // -- not part of the exported schema, diagnostic only.
    bool ok = false;  // true if at least one cell was directly supported
};

// Estimates per-sample accel from each trial's own v_smooth derivative
// (consecutive-row finite difference WITHIN a trial -- `trials` is a list
// of per-trial row vectors specifically so accel is never differentiated
// across a trial boundary, where cmd/velocity can jump discontinuously),
// bins samples by SPEED MAGNITUDE |v_mid| into the (v,a) grid (nearest grid
// point, clamped to the nearest edge for out-of-range samples) -- v_grid is
// a non-negative axis (see default_grid_v_mps()), matching the driver's own
// bilinear_lookup, which likewise clamps any negative v_now to the v=0.00
// edge; binning by the raw SIGNED v_mid instead would force every sample of
// a negative-direction (reverse) trial into the v=0.00 bin regardless of
// its actual distance from stall (e.g. a reverse trial's own steady-cruise
// samples, nowhere near stall, would corrupt that bin's near-stall fit).
// cmd/a_est themselves stay signed, so a v-bin's fit still legitimately
// spans both directions (matching a_mps2's own -/+ range at every row). For
// each v-bin with >=2
// samples spanning more than one distinct cmd, fits a LINEAR a = f(cmd)
// relationship (least squares) and inverts it to get cmd(a) -- exactly the
// "per-v-bin linear fit of a vs cmd" scheme -- for every a_mps2 grid column
// WITHIN (a small margin beyond) that bin's own OBSERVED accel range only;
// columns outside it are deliberately left unsupported here rather than
// extrapolated from a single bin's possibly-unrepresentative line (real
// samples land at slightly different true v within one nominal bin, so a
// compromise line fit through them can be wildly wrong far outside the
// range it was actually constrained by). v-bins/columns without enough
// direct support are filled by linearly interpolating between the nearest
// supported v-bins in the same column (or constant-extrapolating from the
// nearest single supported neighbor at the ends); a column with NO
// supported v-bin at all falls back to the nearest supported cell anywhere
// in the grid (rare -- only reachable with very sparse input).
GridFit fit_grid(const std::vector<std::vector<TrialRow>>& trials,
                  const std::vector<double>& v_grid = default_grid_v_mps(),
                  const std::vector<double>& a_grid = default_grid_a_mps2());

// ---------------------------------------------------------------------
// Export / import: the FROZEN CALIBRATION OUTPUT SCHEMA.
// ---------------------------------------------------------------------

// Builds the frozen-schema JSON exactly (key names/nesting must match the
// real driver's CalibratedMap loader byte-for-byte). `mode` must be kDuty
// or kErpm (throws std::invalid_argument for kCurrent -- the schema's
// "mode" field is never "current"). `erpm_per_mps` is a caller-supplied
// constant (the erpm<->mps conversion the driver uses for its own
// VESC-feedback v_now estimate -- a property of the motor/wheel, not
// something this task's fitters derive; see the frozen schema's own
// "Driver-side lookup" note). fit_rms/kick_ms/min_moving_speed_mps are
// taken from `linear`/`stall` directly (no separate parameters). `v_ss`
// entries are filtered to ok==true and sorted ascending by cmd before
// being written (matches the schema comment "ascending cmd"). `grid==null`
// (a passed-in nullptr, or a non-null grid with ok==false) writes JSON
// `"grid": null`.
nlohmann::json export_calibration(const std::string& robot_name, RawMode mode, double erpm_per_mps,
                                   const LinearFit& linear, const StallFit& stall,
                                   const std::vector<SteadyStatePoint>& v_ss_points,
                                   const GridFit* grid, int n_trials, const std::string& notes = "");

struct ParsedCalibration {
    int version = 0;
    std::string mode;
    std::string robot_name;
    double erpm_per_mps = 0.0;
    double cmd_per_mps = 0.0;
    double cmd_offset = 0.0;
    double stall_min_cmd = 0.0;
    double stall_kick_cmd = 0.0;
    double stall_kick_ms = 0.0;
    double stall_min_moving_speed_mps = 0.0;
    std::vector<SteadyStatePoint> v_ss;  // all entries have ok=true (schema carries no ok flag)
    bool has_grid = false;
    GridFit grid;  // valid only when has_grid; `supported` is left empty (not part of the schema)
    int n_trials = 0;
    double fit_rms = 0.0;
    std::string notes;
};

// Inverse of export_calibration(). Throws std::runtime_error if a required
// key is missing, "mode" is neither "erpm" nor "duty", or "stall"/"meta"
// are not objects.
ParsedCalibration parse_calibration(const nlohmann::json& j);

// ---------------------------------------------------------------------
// Config: MPC/config/motor_calib_config.json loader.
// ---------------------------------------------------------------------

struct MotorCalibConfig {
    std::string robot_ip = "192.168.1.PLACEHOLDER";
    int control_port = 3460;
    int telemetry_port = 3560;
    std::string localization_endpoint = "tcp://127.0.0.1:3260";
    std::string robot_topic_name = "robot2";
    std::string motive_body_name = "mushr2";

    struct Line {
        double length_m = 4.5;
        double end_margin_m = 0.75;
        std::optional<double> line_yaw_rad;  // null in JSON => nullopt => use the robot's initial yaw
    } line;

    struct Safety {
        double mocap_stale_abort_ms = 300.0;
        double max_speed_abort_mps = 0.8;
    } safety;

    struct Sweeps {
        std::vector<double> duty = {0.02, 0.03, 0.04, 0.05, 0.06, 0.08};
        std::vector<double> erpm = {500, 750, 1000, 1250, 1500, 2000, 2500};
    } sweeps;

    struct Trial {
        double max_duration_s = 6.0;
        double settle_s = 1.5;
        double arm_settle_s = 1.0;
        double steady_window_frac = 0.4;
    } trial;

    std::string output_dir = "results/motor_calib";

    static MotorCalibConfig defaults();
};

// Loads MotorCalibConfig from `path`, overriding only the keys present
// (assign_if_present pattern, matching RobotSpec.cpp) -- absent keys keep
// their compiled defaults (matching the shipped
// MPC/config/motor_calib_config.json). Unknown keys warn on stderr but do
// not fail loading. Throws std::runtime_error if the file cannot be opened
// or is not valid JSON.
MotorCalibConfig load_motor_calib_config(const std::string& path);

// Convenience adapter: copies the TrialRunnerConfig-relevant fields out of
// a MotorCalibConfig (line/safety/trial sections). Fields with no
// config-file counterpart (arm_stationary_v_mps/reissue_interval_s/
// center_servo_value) keep TrialRunnerConfig's own compiled defaults.
TrialRunnerConfig make_trial_runner_config(const MotorCalibConfig& cfg);

}  // namespace mpc
