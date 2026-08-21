#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// CALIBRATION CORE (UI-independent teleop-calibration data/fitting library).
// NO sockets, NO Qt live here -- every dependency this needs (pose/
// telemetry/command samples) is fed in by the caller via feed_pose()/
// feed_telemetry()/feed_command(), so the whole thing runs and is
// unit-tested headlessly (see MPC/tests/calibration_core_tests.cpp). A
// later Qt tab wires this to a real ZMQ mocap/VESC feed (mirroring how
// CalibClient.h wires MotorCalibCore.h's TrialRunner to real sockets) and
// to the human-teleop command stream; this file only builds the library
// piece.
//
// PURPOSE: the robot is TELEOPERATED (human joystick/keyboard driving,
// not an automated sweep like MotorCalibCore's straight-line TrialRunner)
// while this library ingests live mocap pose + VESC telemetry + the
// commands being sent, extracts STEADY-STATE samples (command held,
// velocity not changing, sensors fresh), tracks COVERAGE of two driving
// tasks ("which speeds/steering notches still need more steady driving"),
// and fits two calibration maps:
//   - velocity_calib.json : commanded speed (v, m/s) -> VESC erpm, a
//     monotone piecewise-linear table (see fit_velocity_map()).
//   - steering_angle_map.json : effective bicycle-model steering angle
//     (delta, rad) <-> servo command, also monotone piecewise-linear (see
//     fit_steering_map()).
// Both exported schemas are FROZEN CONTRACTS (see export_velocity_calib()/
// export_steering_map() below) -- the robot-side driver parses these
// field names directly.
//
// Reference/prior art: MotorCalibCore.h's VelocityEstimator (heading-
// projected finite-difference velocity + smoothing) and TrialRunner
// (steady-window acceptance) solve an adjacent problem (automated
// straight-line sweeps, not free teleop driving) -- CalibrationCore reuses
// those numerical patterns internally (see CalibrationCore.cpp) but is
// intentionally a SEPARATE, decoupled type family (no shared base
// classes/headers) so it can evolve independently and never risks an ODR
// clash if a future translation unit includes both headers.
namespace mpc {

// ---------------------------------------------------------------------
// Raw ingestion samples (the feed_*() argument shapes, named for clarity
// at call sites -- callers are not required to construct these directly,
// see CalibrationCore::feed_pose()/feed_telemetry()/feed_command()).
// ---------------------------------------------------------------------

// One planner-frame mocap pose sample.
struct PoseSample {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// One VESC telemetry sample (same fields MotorCalibCore::TelemetrySample
// logs, renamed to avoid an ODR clash if both headers are ever included
// together -- see file header comment).
struct TelemetryFrame {
    double t = 0.0;
    double erpm = 0.0;
    double duty = 0.0;
    double current_motor = 0.0;
    double v_in = 0.0;
};

// One teleop command sample. `mode` mirrors the driver's raw command mode
// string (currently only "duty" is meaningfully exercised by this tool --
// steadiness/coverage only look at value+servo, mode is carried through
// for logging/diagnostics).
struct CommandFrame {
    double t = 0.0;
    std::string mode = "duty";
    double value = 0.0;
    double servo = 0.0;
};

// ---------------------------------------------------------------------
// Derived samples: one produced per accepted feed_pose() call (after
// burst-debounce), carrying the derived v/omega plus the latest
// time-aligned command + telemetry. Logged to the session CSV whether
// ACCEPTED or not (see CalibrationCore::feed_pose()'s doc comment for the
// steadiness gate) -- `accepted`/`reject_reason` record the outcome.
// ---------------------------------------------------------------------

enum class RejectReason {
    kNone,               // accepted
    kCommandNotHeld,      // current command hasn't been held long enough yet
    kAccelerating,        // |dv/dt| too large -- not at steady state
    kPoseStale,           // latest pose too old (see doc comment on why this
                           // is nearly always 0 for a synchronous feed_pose()
                           // trigger -- kept for defensive/future-proofing
                           // symmetry with kTelemetryStale)
    kTelemetryStale,      // no fresh-enough VESC telemetry to trust erpm/etc.
    kInsufficientHistory,  // velocity/yaw-rate estimator hasn't warmed up yet
};
std::string to_string(RejectReason reason);

// One row of the session CSV, header EXACTLY:
// t,x,y,yaw,v,omega,erpm,duty_cmd,servo_cmd,v_in,current,accepted,reject_reason
struct DerivedSample {
    double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0;
    double v = 0.0;      // signed, heading-projected + smoothed (m/s)
    double omega = 0.0;  // signed yaw rate (rad/s)
    double erpm = 0.0, duty_cmd = 0.0, servo_cmd = 0.0, v_in = 0.0, current = 0.0;
    bool accepted = false;
    RejectReason reject_reason = RejectReason::kNone;
};

// Appends `rows` to `path` as CSV. Writes the header first only when the
// file doesn't already exist (or `path` is being truncated -- see
// `append`); returns false if the file could not be opened for writing.
// Exposed as a free function so both CalibrationCore's own live logging
// and tests/CLI tooling can reuse it without a full CalibrationCore
// instance.
bool write_derived_samples_csv(const std::string& path, const std::vector<DerivedSample>& rows,
                                bool append);

// Loads a session CSV written by write_derived_samples_csv() (or
// CalibrationCore's own live logging -- same format). Returns false (with
// `*error` set) if the file cannot be opened or the header doesn't match.
bool load_derived_samples_csv(const std::string& path, std::vector<DerivedSample>* out,
                               std::string* error);

// ---------------------------------------------------------------------
// Coverage model.
// ---------------------------------------------------------------------

enum class Task { kVelocity, kSteering };
std::string to_string(Task task);

enum class BinState { kEmpty, kPartial, kDone, kNotApplicable };
std::string to_string(BinState state);

// One coverage bin (a velocity bin's signed-v center, or a steering
// notch's servo value).
struct CoverageBin {
    double center = 0.0;
    int count = 0;
    int target = 0;
    BinState state = BinState::kEmpty;
};

// Points at the single emptiest actionable bin, with a human-readable
// driving instruction. target_bin is an index into the corresponding
// CoverageReport::bins (or -1 when every bin is done/not_applicable --
// `text` then reads "<task> coverage complete.").
struct NextInstruction {
    std::string text;
    int target_bin = -1;
};

struct CoverageReport {
    Task task = Task::kVelocity;
    std::vector<CoverageBin> bins;
    double percent = 0.0;  // 0..100, over applicable (target>0) bins only
    NextInstruction instruction;

    nlohmann::json to_json() const;
};

// ---------------------------------------------------------------------
// Config.
// ---------------------------------------------------------------------

// Velocity task bin ladder: signed v centers at +/-{v_min, v_min+v_step,
// ..., v_max}, |v|<v_min excluded entirely (stall band -- see file header
// comment). Defaults (v_min=0.05, v_max=0.40, v_step=0.05) give 8 centers
// per direction = 16 bins total, covering the full driving envelope up to
// ~0.4 m/s.
struct VelocityCoverageConfig {
    double v_min = 0.05;
    double v_max = 0.40;
    double v_step = 0.05;
    int target_per_bin = 30;
};

// Steering task notches: caller-supplied servo values (the future Qt tab
// computes these from the robot's steering range). A sample counts for
// the nearest notch within +/-0.5*spacing (spacing = the notch ladder's
// own average consecutive gap) when additionally |v| > min_speed_mps.
struct SteeringCoverageConfig {
    std::vector<double> notches;
    int target_per_notch = 60;
    double min_speed_mps = 0.1;
};

// Steadiness acceptance thresholds (see CalibrationCore::feed_pose()'s doc
// comment for exactly how each is evaluated).
struct SteadinessConfig {
    double command_hold_s = 0.5;
    double max_dv_dt_mps2 = 0.15;
    double max_pose_age_s = 0.3;
    double max_telemetry_age_s = 0.2;
    double value_eps = 0.01;   // "same command" tolerance on CommandFrame::value
    double servo_eps = 0.005;  // "same command" tolerance on CommandFrame::servo
};

struct CalibrationConfig {
    std::string robot_name;
    double wheel_base = 0.29;
    VelocityCoverageConfig velocity;
    SteeringCoverageConfig steering;
    SteadinessConfig steadiness;
};

// ---------------------------------------------------------------------
// Fitting.
// ---------------------------------------------------------------------

struct VelocityTablePoint {
    double v = 0.0;
    double erpm = 0.0;
};

struct VelocityFitResult {
    std::vector<VelocityTablePoint> table;  // strictly increasing in v AND erpm
    double min_reliable_erpm = 0.0;
    // Plain linear-regression fallback/diagnostic: erpm = erpm_per_mps*v + offset_erpm.
    double erpm_per_mps = 0.0;
    double offset_erpm = 0.0;
    double rms = 0.0;
    int n_samples = 0;    // accepted, velocity-applicable (|v|>=v_min) samples used
    int bins_filled = 0;  // pre-pooling bin count with >=1 accepted sample
    int bins_total = 0;
    bool ok = false;
    std::string error;
};

struct SteeringTablePoint {
    double delta = 0.0;
    double servo = 0.0;
};

struct SteeringFitResult {
    std::vector<SteeringTablePoint> points;  // strictly monotone in delta
    double delta_min = 0.0, delta_max = 0.0;
    double residual_rms = 0.0;
    int n_samples = 0;  // accepted, steering-applicable samples used
    bool ok = false;
    std::string error;
};

// Fits the velocity map from `bin_samples` (one vector of accepted,
// velocity-applicable DerivedSamples per configured bin -- see
// CalibrationCore::velocity_bin_samples() -- index-aligned with `bins`,
// the fixed bin-center ladder derived from `cfg`). Exposed as a free
// function (in addition to CalibrationCore::fit_velocity_map()) so the CLI
// and tests can fit directly off loaded CSV rows without a live
// CalibrationCore feed. See CalibrationCore.cpp for the full
// per-bin-median -> PAVA-pool -> piecewise-linear-table algorithm and the
// min_reliable_erpm heuristic.
VelocityFitResult fit_velocity_map(const std::vector<double>& bin_centers,
                                    const std::vector<std::vector<DerivedSample>>& bin_samples);

// Fits the steering map from `notch_samples` (accepted, steering-
// applicable DerivedSamples per configured notch, index-aligned with
// `notches`). delta_eff = atan(wheel_base*omega/v) per sample (signed v
// makes reverse driving valid -- see file header comment); per-notch
// median -> direction-aware PAVA -> piecewise-linear table.
SteeringFitResult fit_steering_map(const std::vector<double>& notches,
                                    const std::vector<std::vector<DerivedSample>>& notch_samples,
                                    double wheel_base);

// Clusters `servo_values` into notches: sorts, then merges consecutive
// values within `tolerance` of each other into one notch (mean of the
// group). Used by the CLI to infer the notch ladder from a session CSV's
// own servo_cmd column when no notch config is otherwise available.
std::vector<double> infer_notches(const std::vector<double>& servo_values, double tolerance = 0.005);

// ---------------------------------------------------------------------
// Export -- FROZEN OUTPUT SCHEMAS (the robot-side driver parses these
// field names directly; see each function's body for the exact shape).
// Atomic write: written to `path + ".tmp"` then renamed over `path`.
// Refuses (returns false, `*error` set) if the fit has fewer than 2 table
// points.
// ---------------------------------------------------------------------

nlohmann::json velocity_calib_to_json(const VelocityFitResult& fit, const std::string& robot_name,
                                       const std::string& note = "");
bool export_velocity_calib(const VelocityFitResult& fit, const std::string& robot_name,
                            const std::string& path, std::string* error, const std::string& note = "");

nlohmann::json steering_map_to_json(const SteeringFitResult& fit, const std::string& robot_name,
                                     double wheel_base, const std::string& note = "");
bool export_steering_map(const SteeringFitResult& fit, const std::string& robot_name, double wheel_base,
                          const std::string& path, std::string* error, const std::string& note = "");

// ---------------------------------------------------------------------
// CalibrationCore: the live ingestion + coverage + fitting facade.
// ---------------------------------------------------------------------
class CalibrationCore {
public:
    explicit CalibrationCore(CalibrationConfig cfg = {});

    const CalibrationConfig& config() const { return cfg_; }
    // Replaces the steering notch ladder (e.g. once the CLI has inferred
    // it from a session CSV's servo_cmd column). Only legal before any
    // samples have been ingested (rebinning already-ingested samples is
    // not supported) -- returns false and leaves state unchanged
    // otherwise.
    bool set_steering_notches(const std::vector<double>& notches);

    // Enables (or redirects) live per-derived-sample CSV logging: every
    // subsequent feed_pose()-triggered derived sample is appended to
    // `path` (creating it with the header row if it doesn't already exist,
    // or if `append` is false -- a false `append` truncates/overwrites).
    // Pass an empty path to disable live logging.
    void set_session_csv_path(const std::string& path, bool append = false);

    // Ingestion. See this class's .cpp for the full derivation pipeline
    // (burst-debounce, heading-projected velocity + light smoothing,
    // sliding-window unwrapped-yaw least-squares omega, nearest-in-time
    // telemetry alignment, steadiness acceptance).
    void feed_pose(double t, double x, double y, double yaw);
    void feed_telemetry(double t, double erpm, double duty, double current_motor, double v_in);
    void feed_command(double t, const std::string& mode, double value, double servo);

    // Re-ingests a session CSV (own live-logged format, see
    // write_derived_samples_csv()): every row is counted in
    // rejection_counts(), and ACCEPTED rows are additionally binned into
    // coverage + the fitting dataset -- i.e. resumes both coverage and
    // fit-readiness from a prior process's session file. Returns false
    // (with `*error` set, if non-null) on a read/parse failure; state is
    // left unchanged in that case.
    bool load_session_csv(const std::string& path, std::string* error = nullptr);
    // Same re-ingest as load_session_csv(), from already-loaded rows (the
    // CLI uses this to avoid re-parsing after its own notch-inference
    // pre-scan -- see infer_notches()).
    void ingest_derived_samples(const std::vector<DerivedSample>& rows);

    CoverageReport velocity_coverage() const;
    CoverageReport steering_coverage() const;
    NextInstruction next_instruction(Task task) const;

    const std::map<RejectReason, int>& rejection_counts() const { return rejection_counts_; }
    int total_samples() const { return total_samples_; }
    int accepted_samples() const { return accepted_samples_; }

    // Raw per-bin/per-notch accepted-sample storage (see fit_velocity_map()/
    // fit_steering_map() free functions above) -- index-aligned with the
    // bin-center/notch ladder velocity_coverage().bins / steering_coverage().bins
    // report.
    const std::vector<std::vector<DerivedSample>>& velocity_bin_samples() const {
        return velocity_bin_samples_;
    }
    const std::vector<std::vector<DerivedSample>>& steering_notch_samples() const {
        return steering_notch_samples_;
    }

    VelocityFitResult fit_velocity_map() const;
    SteeringFitResult fit_steering_map() const;
    SteeringFitResult fit_steering_map(double wheel_base) const;

private:
    void ingest_one(const DerivedSample& row, bool also_log_csv);
    std::vector<double> velocity_bin_centers() const;

    CalibrationConfig cfg_;
    bool notches_locked_ = false;  // true once any sample has been ingested

    // -- Live derivation state (velocity/yaw-rate estimation, mirrors
    // MotorCalibCore::VelocityEstimator's numerical approach -- see this
    // class's .cpp). --
    bool have_prev_pose_ = false;
    double prev_pose_t_ = 0.0, prev_pose_x_ = 0.0, prev_pose_y_ = 0.0;
    double last_pose_feed_t_ = -1.0e18;  // burst-debounce (see feed_pose())

    double last_v_raw_ = 0.0;
    bool have_v_raw_ = false;
    double v_smooth_ = 0.0;
    bool have_v_smooth_ = false;
    double prev_v_smooth_ = 0.0;
    double prev_v_smooth_t_ = 0.0;
    bool have_prev_v_smooth_ = false;
    int pose_feed_count_ = 0;

    struct YawPoint {
        double t;
        double yaw_unwrapped;
    };
    std::vector<YawPoint> yaw_window_;
    bool have_yaw_ = false;
    double last_yaw_raw_ = 0.0;
    double last_yaw_unwrapped_ = 0.0;

    bool have_telemetry_ = false;
    TelemetryFrame last_telemetry_;

    bool have_command_ = false;
    CommandFrame current_command_;
    double command_held_since_ = 0.0;

    std::string session_csv_path_;
    bool session_csv_has_header_ = false;

    std::map<RejectReason, int> rejection_counts_;
    int total_samples_ = 0;
    int accepted_samples_ = 0;

    std::vector<double> velocity_bin_centers_;  // cached at construction -- cfg_.velocity is immutable post-ctor
    std::vector<std::vector<DerivedSample>> velocity_bin_samples_;  // index-aligned w/ velocity_bin_centers_
    std::vector<std::vector<DerivedSample>> steering_notch_samples_;  // index-aligned w/ cfg_.steering.notches
};

}  // namespace mpc
