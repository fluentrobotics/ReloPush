#include "DriverCore.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

// Vendored copy (see VescDriver's "no includes outside VescDriver/" rule).
#include "../third_party/nlohmann/json.hpp"

namespace vesc {

namespace {

// ---------------------------------------------------------------------
// Watchdog / brake constants (frozen wire contract): the ramp toward zero
// uses the FIXED actuator max_accel=0.73, never config.safety.safety_max_accel
// (that ceiling exists purely for legitimate LaunchGovernor kicks arriving
// on the ackermann stream -- the watchdog's own auto-brake is not one of
// those and must not be able to exceed the nominal actuator limit). Once
// the ramp reaches (snaps to) exactly 0, the driver holds a fixed brake
// current instead of a v_target-derived command -- 2.0A is a conservative
// "hold position, don't fight the user" default, further clamped to
// config.safety.max_current in case that ceiling is configured lower.
constexpr double kWatchdogRampAccel = 0.73;
constexpr double kWatchdogBrakeAmpsDefault = 2.0;

// Moves `from` toward `to` by at most `max_delta` (>= 0), snapping EXACTLY
// to `to` once within reach -- mirrors MPC/src/LaunchGovernor.cpp's own
// rate_limited_toward() byte-for-byte (same algorithm, independently
// reproduced here per VescDriver's "no includes outside VescDriver/" rule).
// This is what actually delivers the frozen contract's "snap to exactly 0"
// behavior: a fixed-formula "always step by +-max_accel*dt" ramp (an
// earlier version of this function) can OVERSHOOT a from/to pair closer
// together than max_delta and oscillate across the target forever instead
// of ever landing on it -- concretely, at this driver's 50Hz control rate
// (dt=0.02s) the per-tick step (0.73*0.02=0.0146 m/s) is LARGER than a
// naive zero-velocity epsilon (0.005 m/s), so a sign-based "brake_accel =
// -sign(v)*0.73" step can walk v_target from just above the epsilon band
// straight past zero to just below it (or vice versa) every single tick,
// never once landing inside the band -- verified as a real, reproducible
// bug via this file's own unit test (VescDriver/tests/vesc_driver_tests.cpp
// h2) before switching to this rate-limited formulation.
double rate_limited_toward(double from, double to, double max_delta) {
    const double delta = to - from;
    if (std::fabs(delta) <= max_delta) {
        return to;
    }
    return from + std::copysign(max_delta, delta);
}

// ---------------------------------------------------------------------
// Bilinear grid lookup helper.
// ---------------------------------------------------------------------

struct Bracket {
    size_t idx = 0;
    double frac = 0.0;
};

// Brackets `x` against ascending `axis` -- see bilinear_lookup()'s own doc
// comment in the header for the exact clamping/degenerate-axis behavior.
Bracket bracket_axis(const std::vector<double>& axis, double x) {
    Bracket b;
    if (axis.size() <= 1) {
        return b;  // idx=0, frac=0.0 -- degenerate "constant" axis.
    }
    if (x <= axis.front()) {
        return b;  // idx=0, frac=0.0 -- clamp to the low edge.
    }
    if (x >= axis.back()) {
        b.idx = axis.size() - 2;
        b.frac = 1.0;
        return b;  // clamp to the high edge.
    }
    for (size_t i = 0; i + 1 < axis.size(); ++i) {
        if (x >= axis[i] && x <= axis[i + 1]) {
            b.idx = i;
            const double span = axis[i + 1] - axis[i];
            b.frac = (span > 1e-12) ? (x - axis[i]) / span : 0.0;
            return b;
        }
    }
    // Unreachable given ascending axis + the range checks above; kept as a
    // safe fallback rather than undefined behavior.
    b.idx = axis.size() - 2;
    b.frac = 1.0;
    return b;
}

}  // namespace

double bilinear_lookup(const CalibrationGrid& grid, double v, double a) {
    if (grid.v_mps.empty() || grid.a_mps2.empty() || grid.cmd.empty()) {
        return 0.0;
    }

    const Bracket vb = bracket_axis(grid.v_mps, v);
    const Bracket ab = bracket_axis(grid.a_mps2, a);
    const size_t vi0 = vb.idx;
    const size_t vi1 = (grid.v_mps.size() > 1) ? vi0 + 1 : vi0;
    const size_t ai0 = ab.idx;
    const size_t ai1 = (grid.a_mps2.size() > 1) ? ai0 + 1 : ai0;

    const double c00 = grid.cmd[vi0][ai0];
    const double c01 = grid.cmd[vi0][ai1];
    const double c10 = grid.cmd[vi1][ai0];
    const double c11 = grid.cmd[vi1][ai1];

    const double c0 = c00 + (c01 - c00) * ab.frac;
    const double c1 = c10 + (c11 - c10) * ab.frac;
    return c0 + (c1 - c0) * vb.frac;
}

// ---------------------------------------------------------------------
// MapMode / RawCalibMode string helpers.
// ---------------------------------------------------------------------

MapMode map_mode_from_string(const std::string& s) { return (s == "duty") ? MapMode::kDuty : MapMode::kErpm; }

RawCalibMode raw_calib_mode_from_string(const std::string& s, bool* ok) {
    if (ok) *ok = true;
    if (s == "duty") return RawCalibMode::kDuty;
    if (s == "erpm") return RawCalibMode::kErpm;
    if (s == "current") return RawCalibMode::kCurrent;
    if (ok) *ok = false;
    return RawCalibMode::kDuty;
}

std::string to_string(RawCalibMode m) {
    switch (m) {
        case RawCalibMode::kDuty: return "duty";
        case RawCalibMode::kErpm: return "erpm";
        case RawCalibMode::kCurrent: return "current";
    }
    return "duty";
}

// ---------------------------------------------------------------------
// LinearMap
// ---------------------------------------------------------------------

double LinearMap::compute_cmd(double /*v_now*/, double v_target, double /*a_desired*/) const {
    return cmd_offset_ + cmd_per_mps_ * v_target;
}

// ---------------------------------------------------------------------
// CalibratedMap (implementation-only -- not exposed in DriverCore.h; built
// exclusively via build_motor_map()/load_calibration_file() below).
// ---------------------------------------------------------------------

namespace {

class CalibratedMap : public MotorMap {
   public:
    explicit CalibratedMap(CalibrationData data) : data_(std::move(data)) {}

    MapMode mode() const override { return data_.mode; }

    double compute_cmd(double v_now, double v_target, double a_desired) const override {
        if (data_.has_grid) {
            return bilinear_lookup(data_.grid, v_now, a_desired);
        }
        return data_.cmd_offset + data_.cmd_per_mps * v_target;
    }

   private:
    CalibrationData data_;
};

}  // namespace

// ---------------------------------------------------------------------
// Calibration JSON parsing (FROZEN schema -- see the task brief / this
// repo's standing driver-control-protocol context for the exact key list:
// version/mode/robot_name/erpm_per_mps/cmd_per_mps/cmd_offset/stall/v_ss/
// grid/meta). Only the keys DriverCore's CalibratedMap actually consumes
// are validated/kept below; "version"/"robot_name"/"stall"/"v_ss"/"meta"
// are accepted without validation (present-or-absent, any shape) since
// this driver never reads them.
// ---------------------------------------------------------------------

CalibrationParseResult parse_calibration_json(const std::string& json_text) {
    CalibrationParseResult out;

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(json_text);
    } catch (...) {
        out.error = "invalid JSON";
        return out;
    }
    if (!j.is_object()) {
        out.error = "top level is not a JSON object";
        return out;
    }

    try {
        CalibrationData data;

        if (!j.contains("mode") || !j.at("mode").is_string()) {
            out.error = "missing or non-string 'mode'";
            return out;
        }
        const std::string mode_str = j.at("mode").get<std::string>();
        if (mode_str == "erpm") {
            data.mode = MapMode::kErpm;
        } else if (mode_str == "duty") {
            data.mode = MapMode::kDuty;
        } else {
            out.error = "unrecognized 'mode' (expected \"erpm\" or \"duty\")";
            return out;
        }

        if (!j.contains("erpm_per_mps") || !j.at("erpm_per_mps").is_number()) {
            out.error = "missing or non-numeric 'erpm_per_mps'";
            return out;
        }
        data.erpm_per_mps = j.at("erpm_per_mps").get<double>();

        if (!j.contains("cmd_per_mps") || !j.at("cmd_per_mps").is_number()) {
            out.error = "missing or non-numeric 'cmd_per_mps'";
            return out;
        }
        data.cmd_per_mps = j.at("cmd_per_mps").get<double>();

        if (!j.contains("cmd_offset") || !j.at("cmd_offset").is_number()) {
            out.error = "missing or non-numeric 'cmd_offset'";
            return out;
        }
        data.cmd_offset = j.at("cmd_offset").get<double>();

        // "grid": absent OR JSON null both mean has_grid=false (linear
        // fallback) -- not a parse failure.
        if (j.contains("grid") && !j.at("grid").is_null()) {
            const nlohmann::json& g = j.at("grid");
            if (!g.is_object() || !g.contains("v_mps") || !g.contains("a_mps2") || !g.contains("cmd") ||
                !g.at("v_mps").is_array() || !g.at("a_mps2").is_array() || !g.at("cmd").is_array()) {
                out.error = "malformed 'grid' object";
                return out;
            }

            CalibrationGrid grid;
            for (const auto& v : g.at("v_mps")) {
                if (!v.is_number()) {
                    out.error = "non-numeric entry in grid.v_mps";
                    return out;
                }
                grid.v_mps.push_back(v.get<double>());
            }
            for (const auto& v : g.at("a_mps2")) {
                if (!v.is_number()) {
                    out.error = "non-numeric entry in grid.a_mps2";
                    return out;
                }
                grid.a_mps2.push_back(v.get<double>());
            }
            if (grid.v_mps.empty() || grid.a_mps2.empty()) {
                out.error = "grid.v_mps/a_mps2 must be non-empty";
                return out;
            }
            for (size_t i = 1; i < grid.v_mps.size(); ++i) {
                if (grid.v_mps[i] <= grid.v_mps[i - 1]) {
                    out.error = "grid.v_mps is not strictly ascending";
                    return out;
                }
            }
            for (size_t i = 1; i < grid.a_mps2.size(); ++i) {
                if (grid.a_mps2[i] <= grid.a_mps2[i - 1]) {
                    out.error = "grid.a_mps2 is not strictly ascending";
                    return out;
                }
            }

            const nlohmann::json& cmd_rows = g.at("cmd");
            if (cmd_rows.size() != grid.v_mps.size()) {
                out.error = "grid.cmd row count does not match grid.v_mps size";
                return out;
            }
            for (const auto& row : cmd_rows) {
                if (!row.is_array() || row.size() != grid.a_mps2.size()) {
                    out.error = "grid.cmd row shape does not match grid.a_mps2 size";
                    return out;
                }
                std::vector<double> row_vals;
                row_vals.reserve(row.size());
                for (const auto& c : row) {
                    if (!c.is_number()) {
                        out.error = "non-numeric entry in grid.cmd";
                        return out;
                    }
                    row_vals.push_back(c.get<double>());
                }
                grid.cmd.push_back(std::move(row_vals));
            }

            data.has_grid = true;
            data.grid = std::move(grid);
        }

        out.data = std::move(data);
        out.ok = true;
    } catch (const std::exception& e) {
        out.ok = false;
        out.error = std::string("exception while parsing calibration JSON: ") + e.what();
    } catch (...) {
        out.ok = false;
        out.error = "unknown exception while parsing calibration JSON";
    }
    return out;
}

CalibrationParseResult load_calibration_file(const std::string& path) {
    CalibrationParseResult out;
    std::ifstream f(path);
    if (!f.is_open()) {
        out.error = "could not open '" + path + "'";
        return out;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parse_calibration_json(ss.str());
}

MotorMapBuildResult build_motor_map(const DriverConfig& config) {
    MotorMapBuildResult out;

    if (!config.calibration_file.empty()) {
        CalibrationParseResult cal = load_calibration_file(config.calibration_file);
        if (cal.ok) {
            out.used_calibration = true;
            out.effective_erpm_per_mps = (cal.data.erpm_per_mps > 1e-9) ? cal.data.erpm_per_mps : config.erpm_per_mps;
            out.note = "loaded calibration '" + config.calibration_file + "' (mode=" + to_string(cal.data.mode) +
                        (cal.data.has_grid ? ", grid" : ", linear-fallback") + ")";
            out.map.reset(new CalibratedMap(std::move(cal.data)));
            return out;
        }
        out.note = "calibration_file '" + config.calibration_file + "' failed to load (" + cal.error +
                    ") -- falling back to LinearMap placeholder gains";
    } else {
        out.note = "no calibration_file configured -- using LinearMap placeholder gains";
    }

    out.used_calibration = false;
    out.effective_erpm_per_mps = config.erpm_per_mps;
    out.map.reset(new LinearMap(map_mode_from_string(config.mode), config.cmd_offset, config.cmd_per_mps));
    return out;
}

// ---------------------------------------------------------------------
// DriverConfig loading (VescDriver/config/driver_config.json shape).
// ---------------------------------------------------------------------

namespace {

// Overwrites *out with j[key] iff j contains `key` AND it converts to T
// cleanly; otherwise *out (already holding a default, or a previously-set
// value) is left untouched. Never throws.
template <typename T>
void assign_if_present(const nlohmann::json& j, const char* key, T* out) {
    if (!j.contains(key)) {
        return;
    }
    try {
        *out = j.at(key).get<T>();
    } catch (...) {
        // Wrong JSON type for this key -- silently retain whatever *out
        // already held (the struct's own default). Mirrors
        // mpc::parse_sim_config_payload's per-field independence.
    }
}

}  // namespace

ConfigLoadResult load_driver_config(const std::string& path) {
    ConfigLoadResult out;

    std::ifstream f(path);
    if (!f.is_open()) {
        out.error = "could not open '" + path + "'";
        return out;
    }
    std::ostringstream ss;
    ss << f.rdbuf();

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(ss.str());
    } catch (const std::exception& e) {
        out.error = std::string("invalid JSON: ") + e.what();
        return out;
    } catch (...) {
        out.error = "invalid JSON (unknown exception)";
        return out;
    }
    if (!j.is_object()) {
        out.error = "top level is not a JSON object";
        return out;
    }

    DriverConfig cfg;  // starts from DriverConfig{} defaults.
    assign_if_present(j, "robot_name", &cfg.robot_name);
    assign_if_present(j, "ackermann_port", &cfg.ackermann_port);
    assign_if_present(j, "control_port", &cfg.control_port);
    assign_if_present(j, "telemetry_port", &cfg.telemetry_port);
    assign_if_present(j, "serial_port", &cfg.serial_port);
    assign_if_present(j, "baud", &cfg.baud);
    assign_if_present(j, "mode", &cfg.mode);
    assign_if_present(j, "cmd_per_mps", &cfg.cmd_per_mps);
    assign_if_present(j, "cmd_offset", &cfg.cmd_offset);
    assign_if_present(j, "erpm_per_mps", &cfg.erpm_per_mps);
    assign_if_present(j, "watchdog_ms", &cfg.watchdog_ms);
    assign_if_present(j, "control_rate_hz", &cfg.control_rate_hz);
    assign_if_present(j, "telemetry_rate_hz", &cfg.telemetry_rate_hz);
    assign_if_present(j, "calibration_file", &cfg.calibration_file);

    if (j.contains("servo")) {
        if (j.at("servo").is_object()) {
            const nlohmann::json& s = j.at("servo");
            assign_if_present(s, "enabled", &cfg.servo.enabled);
            assign_if_present(s, "center", &cfg.servo.center);
            assign_if_present(s, "gain_per_rad", &cfg.servo.gain_per_rad);
            assign_if_present(s, "invert", &cfg.servo.invert);
            assign_if_present(s, "min_pos", &cfg.servo.min_pos);
            assign_if_present(s, "max_pos", &cfg.servo.max_pos);
        } else {
            out.warnings.push_back("'servo' present but not an object -- defaults retained");
        }
    }

    if (j.contains("safety")) {
        if (j.at("safety").is_object()) {
            const nlohmann::json& s = j.at("safety");
            assign_if_present(s, "max_erpm", &cfg.safety.max_erpm);
            assign_if_present(s, "max_duty", &cfg.safety.max_duty);
            assign_if_present(s, "max_current", &cfg.safety.max_current);
            assign_if_present(s, "safety_max_accel", &cfg.safety.safety_max_accel);
            assign_if_present(s, "safety_max_v", &cfg.safety.safety_max_v);
        } else {
            out.warnings.push_back("'safety' present but not an object -- defaults retained");
        }
    }

    if (j.contains("kick")) {
        if (j.at("kick").is_object()) {
            const nlohmann::json& k = j.at("kick");
            assign_if_present(k, "enabled", &cfg.kick.enabled);
            assign_if_present(k, "kick_cmd", &cfg.kick.kick_cmd);
            assign_if_present(k, "kick_ms", &cfg.kick.kick_ms);
            assign_if_present(k, "min_moving_speed_mps", &cfg.kick.min_moving_speed_mps);
            assign_if_present(k, "kick_erpm_threshold", &cfg.kick.kick_erpm_threshold);
        } else {
            out.warnings.push_back("'kick' present but not an object -- defaults retained");
        }
    }

    out.config = cfg;
    out.ok = true;
    return out;
}

// ---------------------------------------------------------------------
// DriverCore
// ---------------------------------------------------------------------

DriverCore::DriverCore(DriverConfig config) : config_(std::move(config)), erpm_per_mps_(config_.erpm_per_mps) {
    MotorMapBuildResult built = build_motor_map(config_);
    map_ = std::move(built.map);
    erpm_per_mps_ = built.effective_erpm_per_mps;
    used_calibration_ = built.used_calibration;
    motor_map_note_ = built.note;
}

void DriverCore::set_motor_map(std::unique_ptr<MotorMap> map, double effective_erpm_per_mps) {
    map_ = std::move(map);
    if (effective_erpm_per_mps > 0.0) {
        erpm_per_mps_ = effective_erpm_per_mps;
    }
}

double DriverCore::clamp_raw_value(RawCalibMode mode, double value, const SafetyConfig& safety) {
    double limit = 0.0;
    switch (mode) {
        case RawCalibMode::kDuty: limit = std::fabs(safety.max_duty); break;
        case RawCalibMode::kErpm: limit = std::fabs(safety.max_erpm); break;
        case RawCalibMode::kCurrent: limit = std::fabs(safety.max_current); break;
    }
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

double DriverCore::watchdog_brake_amps() const {
    return std::min(kWatchdogBrakeAmpsDefault, std::fabs(config_.safety.max_current));
}

double DriverCore::compute_servo_pos(const TickInputs& in) const {
    double pos;
    if (in.servo_override_active) {
        pos = in.servo_override_value;
    } else {
        const double sign = config_.servo.invert ? -1.0 : 1.0;
        // A non-finite (NaN/Inf) steering value on the wire (e.g. a
        // controller-side numerical fault encoded as base64("nan"), which
        // the reference AckermannCodec decodes successfully -- see the
        // frozen wire contract) must not be allowed to produce a non-finite
        // servo_pos: neither of the clamps below can catch it, since every
        // comparison against NaN is false. Treat it as "no steering
        // command" (0 rad, i.e. center-ish) rather than latching a bad
        // value onto the actuator.
        const double steering = std::isfinite(in.ackermann.steering) ? in.ackermann.steering : 0.0;
        pos = config_.servo.center + sign * config_.servo.gain_per_rad * steering;
    }
    if (pos < config_.servo.min_pos) pos = config_.servo.min_pos;
    if (pos > config_.servo.max_pos) pos = config_.servo.max_pos;
    return pos;
}

MotorAction DriverCore::action_from_map_cmd(double cmd) const {
    MotorAction a;
    // Defense in depth: std::max(-lim, std::min(lim, NaN)) == NaN's argument
    // (every comparison against NaN is false, so both std::min and std::max
    // just return their NaN operand unchanged) -- i.e. a non-finite `cmd`
    // would otherwise sail straight through this clamp as +lim (see
    // std::min(lim, NaN): the library-defined tie-break returns the second
    // argument here). The tick() callers upstream now sanitize
    // accel/v_target before they ever reach a MotorMap, so this should be
    // unreachable in practice; kept as a hard backstop so a bad cmd can
    // never leave this function as anything but a safety-clamped, finite
    // value.
    if (!std::isfinite(cmd)) cmd = 0.0;
    const MapMode mode = map_ ? map_->mode() : MapMode::kErpm;
    if (mode == MapMode::kDuty) {
        a.type = MotorAction::Type::kDuty;
        const double lim = std::fabs(config_.safety.max_duty);
        a.value = std::max(-lim, std::min(lim, cmd));
    } else {
        a.type = MotorAction::Type::kRpm;
        const double lim = std::fabs(config_.safety.max_erpm);
        a.value = std::max(-lim, std::min(lim, cmd));
    }
    return a;
}

std::string DriverCore::state_string() const {
    switch (state_) {
        case InternalState::kIdle: return "idle";
        case InternalState::kActive: return "active";
        case InternalState::kWatchdogBrake: return "watchdog_brake";
    }
    return "idle";
}

MotorAction DriverCore::stop() {
    v_target_ = 0.0;
    kicking_ = false;
    watchdog_engaged_ = false;
    state_ = InternalState::kIdle;

    MotorAction a;
    a.type = MotorAction::Type::kBrake;
    a.value = watchdog_brake_amps();
    return a;
}

TickResult DriverCore::tick(const TickInputs& in) {
    double dt = 0.0;
    if (has_ticked_) {
        dt = in.now_s - last_tick_time_s_;
        if (dt < 0.0) dt = 0.0;  // a backward clock jump never integrates negative time.
    }
    last_tick_time_s_ = in.now_s;
    has_ticked_ = true;

    TickResult result;
    result.servo_pos = compute_servo_pos(in);

    const double v_now = (erpm_per_mps_ > 1e-9) ? (in.erpm_meas / erpm_per_mps_) : 0.0;

    if (in.source == Source::kAckermann) {
        if (!in.ackermann.valid) {
            // Nothing has ever been received on the ackermann stream --
            // idle, no output at all (caller sends a keepalive ALIVE for
            // MotorAction::Type::kNone).
            state_ = InternalState::kIdle;
            v_target_ = 0.0;
            kicking_ = false;
            watchdog_engaged_ = false;
            result.motor.type = MotorAction::Type::kNone;
            result.v_target = v_target_;
            return result;
        }

        const double watchdog_timeout_s = config_.watchdog_ms / 1000.0;
        const bool should_engage = in.ackermann.age_s > watchdog_timeout_s;
        if (should_engage) {
            watchdog_engaged_ = true;
        } else {
            // "the next valid command disengages immediately" -- age_s is
            // only <= timeout when the caller's own held-command timestamp
            // was refreshed by a fresh VALID payload (malformed payloads
            // never touch it), so this is exactly that disengage edge.
            watchdog_engaged_ = false;
        }

        if (watchdog_engaged_) {
            state_ = InternalState::kWatchdogBrake;
            kicking_ = false;
            const double v_target_before_ramp = v_target_;
            const double max_delta = std::fabs(kWatchdogRampAccel) * dt;
            v_target_ = rate_limited_toward(v_target_, 0.0, max_delta);
            if (v_target_ == 0.0) {
                result.motor.type = MotorAction::Type::kBrake;
                result.motor.value = watchdog_brake_amps();
            } else {
                // Derive the truthful accel this tick's ramp step actually
                // applied (for the map's a_desired input) rather than
                // re-deriving it from v_target_'s own (possibly
                // already-crossed-zero) sign.
                const double applied_accel = (dt > 1e-9) ? (v_target_ - v_target_before_ramp) / dt : 0.0;
                const double cmd = map_ ? map_->compute_cmd(v_now, v_target_, applied_accel) : 0.0;
                result.motor = action_from_map_cmd(cmd);
            }
            result.v_target = v_target_;
            return result;
        }

        // Normal ackermann tracking: integrate the HELD accel every tick
        // (persists across ticks exactly like mpc_robot_sim's held accel --
        // see the frozen wire contract), clamped only to
        // +-safety_max_accel (NOT to the nominal actuator max_accel: the
        // LaunchGovernor legitimately publishes up to max_breakaway_accel
        // during launch kicks).
        state_ = InternalState::kActive;
        const double accel_limit = std::fabs(config_.safety.safety_max_accel);
        double accel_used = in.ackermann.accel;
        // A non-finite (NaN/Inf) decoded accel defeats every clamp below:
        // every '>'/'<' comparison against NaN is false, so accel_used
        // would pass through unclamped, v_target_ would become NaN (and,
        // since NaN propagates through +=, STAY NaN forever -- not even a
        // later valid finite command can recover it, and the watchdog's own
        // rate_limited_toward(NaN, 0, ...) ramp never reaches exactly 0.0
        // either, so it can't even fall back to braking). base64("nan") is
        // a value the reference AckermannCodec decodes successfully (it is
        // not a "malformed payload" by the frozen wire contract's own
        // definition), so this can arrive as a legitimately-decoded
        // command; treat it as "no commanded acceleration this tick"
        // (v_target_ simply holds at its last value) rather than letting it
        // corrupt persistent state.
        if (!std::isfinite(accel_used)) accel_used = 0.0;
        if (accel_used > accel_limit) accel_used = accel_limit;
        if (accel_used < -accel_limit) accel_used = -accel_limit;

        const double v_target_before = v_target_;
        v_target_ += accel_used * dt;
        if (!std::isfinite(v_target_)) v_target_ = v_target_before;  // defensive: never let v_target_ itself latch non-finite.
        const double v_limit = std::fabs(config_.safety.safety_max_v);
        if (v_target_ > v_limit) v_target_ = v_limit;
        if (v_target_ < -v_limit) v_target_ = -v_limit;

        if (!kicking_ && config_.kick.enabled) {
            const bool was_at_rest = std::fabs(v_target_before) < 1e-9;
            const bool entering_band = was_at_rest && std::fabs(v_target_) > 0.0 &&
                                        std::fabs(v_target_) < config_.kick.min_moving_speed_mps;
            const bool erpm_at_rest = std::fabs(in.erpm_meas) < config_.kick.kick_erpm_threshold;
            if (entering_band && erpm_at_rest) {
                kicking_ = true;
                kick_sign_ = (v_target_ >= 0.0) ? 1 : -1;
                kick_start_time_s_ = in.now_s;
            }
        }

        if (kicking_) {
            const double elapsed_ms = (in.now_s - kick_start_time_s_) * 1000.0;
            if (elapsed_ms < config_.kick.kick_ms) {
                result.motor = action_from_map_cmd(kick_sign_ * config_.kick.kick_cmd);
                result.v_target = v_target_;
                return result;
            }
            kicking_ = false;  // kick window elapsed -- fall through to normal mapping below.
        }

        const double cmd = map_ ? map_->compute_cmd(v_now, v_target_, accel_used) : 0.0;
        result.motor = action_from_map_cmd(cmd);
        result.v_target = v_target_;
        return result;
    }

    // Calib source: no v_target concept -- raw commands apply directly.
    // Force a clean slate so a later switch back to ackermann always
    // resumes integrating from rest, never a stale v_target from before the
    // switch.
    v_target_ = 0.0;
    kicking_ = false;
    watchdog_engaged_ = false;

    if (!in.calib.has_command) {
        state_ = InternalState::kIdle;
        result.motor.type = MotorAction::Type::kNone;
        result.v_target = 0.0;
        return result;
    }

    const bool ttl_expired = (in.calib.age_s * 1000.0) > in.calib.ttl_ms;
    if (ttl_expired) {
        state_ = InternalState::kWatchdogBrake;
        result.motor.type = MotorAction::Type::kBrake;
        result.motor.value = watchdog_brake_amps();
        result.v_target = 0.0;
        return result;
    }

    state_ = InternalState::kActive;
    const double clamped = clamp_raw_value(in.calib.mode, in.calib.value, config_.safety);
    MotorAction a;
    switch (in.calib.mode) {
        case RawCalibMode::kDuty: a.type = MotorAction::Type::kDuty; break;
        case RawCalibMode::kErpm: a.type = MotorAction::Type::kRpm; break;
        case RawCalibMode::kCurrent: a.type = MotorAction::Type::kCurrent; break;
    }
    a.value = clamped;
    result.motor = a;
    result.v_target = 0.0;
    return result;
}

}  // namespace vesc
