// TeleopCore.cpp -- see TeleopCore.h for the full state-machine contract.

#include "TeleopCore.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>

namespace vesc {

// Tolerance for the "fires at exactly the boundary" time comparisons in
// step() below (deadman expiry, brake-hold elapsed). Guards against
// double-precision rounding artifacts in the subtraction (e.g.
// 0.7 - 0.5 == 0.19999999999999996 in IEEE 754 double, just under the
// mathematically-exact 0.2) rather than any real imprecision in the
// caller's clock -- real wall-clock deltas essentially never land on an
// exact tie anyway, this just makes the boundary reliably inclusive.
constexpr double kTimeEpsilonS = 1e-6;

// Ramp-rate-entry clamp bounds (see TeleopCore::clamp_ramp_rate()) --
// deliberately a nonzero floor (unlike clamp_digit_entry()'s magnitude
// floor of 0): a ramp rate of exactly 0 would never reach its target,
// which is a nonsensical "stuck forever" configuration rather than a
// legitimate (if extreme) slow ramp.
constexpr double kDutyRampMin = 0.001;
constexpr double kDutyRampMax = 1.0;
constexpr double kErpmRampMin = 10.0;
constexpr double kErpmRampMax = 20000.0;

// Speed governor mode (see TeleopCore.h's "SPEED GOVERNOR MODE" section).
// EMA smoothing factor for the raw erpm reading -- FW 2.18's own erpm
// estimate is noisy (roughly +-150), so this is deliberately fairly
// aggressive smoothing, not just a nominal filter.
constexpr double kErpmFilterAlpha = 0.3;
// Floor on the v_in used in the feedforward term's denominator -- guards
// against a division blowing up (or even by zero) on a bogus/startup
// v_in reading, not a claim that 6V is a real operating point.
constexpr double kSpeedMinEffectiveVIn = 6.0;

std::string fault_name(uint8_t fault_code) {
    switch (fault_code) {
        case 1: return "OVER_VOLTAGE";
        case 2: return "UNDER_VOLTAGE";
        case 3: return "DRV";
        case 4: return "ABS_OVER_CURRENT";
        case 5: return "OVER_TEMP_FET";
        case 6: return "OVER_TEMP_MOTOR";
        default: return "";
    }
}

TeleopCore::TeleopCore(TeleopConfig config)
    : config_(config),
      duty_mag_(config.duty_mag_default),
      erpm_mag_(config.erpm_mag_default),
      speed_mag_(config.speed_mag_default),
      duty_ramp_(config.duty_ramp),
      erpm_ramp_(config.erpm_ramp),
      ramp_enabled_(config.ramp_enabled),
      steering_position_(clamp_steering(config.steer_center)) {}

double TeleopCore::max_for_mode(TeleopMode m) const {
    // kErpm and kSpeed are both erpm-shaped targets -- share max_erpm.
    if (m == TeleopMode::kDuty) return config_.max_duty;
    return config_.max_erpm;
}

double TeleopCore::step_for_mode(TeleopMode m) const {
    if (m == TeleopMode::kDuty) return config_.duty_step;
    return config_.erpm_step;
}

double TeleopCore::ramp_rate_for_mode(TeleopMode m) const {
    // kSpeed's own governor always slews its duty OUTPUT by duty_ramp_,
    // regardless of mode -- see step_speed_governor() -- so it shares
    // duty's rate here, not erpm's (its magnitude() is a target erpm,
    // but that's not what this rate governs).
    return (m == TeleopMode::kErpm) ? erpm_ramp_ : duty_ramp_;
}

double TeleopCore::clamp_step_result(TeleopMode m, double value) const {
    const double lo = step_for_mode(m);
    const double hi = max_for_mode(m);
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

double TeleopCore::clamp_digit_entry(TeleopMode m, double value) const {
    const double hi = max_for_mode(m);
    if (value < 0.0) return 0.0;
    if (value > hi) return hi;
    return value;
}

double TeleopCore::clamp_ramp_rate(TeleopMode m, double value) const {
    // kSpeed's 'A' entry also targets duty_ramp_ (see ramp_rate_for_mode()),
    // so it shares duty's bounds here too.
    const bool duty_like = (m == TeleopMode::kDuty || m == TeleopMode::kSpeed);
    const double lo = duty_like ? kDutyRampMin : kErpmRampMin;
    const double hi = duty_like ? kDutyRampMax : kErpmRampMax;
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

void TeleopCore::begin_brake(const std::string& reason, double now_s) {
    state_ = State::kBraking;
    direction_ = DriveDirection::kStopped;
    brake_started_s_ = now_s;
    stop_reason_ = reason;
    // Every brake path is a hard stop of the ramp too -- the next drive
    // starts fresh from 0, never resuming a stale slew (see start_driving()
    // and the class header's "RAMP MODE" section). Inert while actually
    // braking (step()'s kBraking/kAborted branches never read this), but
    // keeping the invariant "emitted_value_ is 0 whenever not actively
    // driving" true everywhere avoids surprises for any future reader.
    emitted_value_ = 0.0;
    // Covers the stop key, deadman expiry, AND a mode-switch-while-
    // driving stop (handle_key's 'd'/'e'/'v' cases all route their
    // "stop first" through this same function) -- see "SPEED GOVERNOR
    // MODE"'s reset contract.
    reset_governor();
}

void TeleopCore::reset_governor() {
    speed_integrator_ = 0.0;
    erpm_filter_initialized_ = false;
    erpm_filtered_ = 0.0;  // reseeded fresh from the next feed_telemetry() call.
}

double TeleopCore::clamp_steering(double value) const {
    if (value < config_.steer_min_pos) return config_.steer_min_pos;
    if (value > config_.steer_max_pos) return config_.steer_max_pos;
    return value;
}

// Shared by 'j'/'l'/'k'/'R' -- clamps, ALWAYS latches steering_touched_
// (even a snap-to-center that lands exactly where the servo already was
// still counts as a deliberate touch -- see the class header's "NO
// PREMATURE EMISSION"), and latches steering_changed_since_emit_ only
// if the clamped value actually differs from the current position (so
// e.g. repeatedly pressing 'j' at the clamp limit doesn't keep flagging
// a change that never happened).
void TeleopCore::set_steering_position(double new_value) {
    const double clamped = clamp_steering(new_value);
    steering_touched_ = true;
    if (clamped != steering_position_) {
        steering_position_ = clamped;
        steering_changed_since_emit_ = true;
    }
}

void TeleopCore::set_steer_center(double new_center) {
    config_.steer_center = clamp_steering(new_center);
}

void TeleopCore::start_driving(DriveDirection dir) {
    // Reset the ramp's emitted value to 0 ONLY when driving is starting
    // fresh from a non-driving state -- re-pressing f/b/r (or reversing
    // direction) while ALREADY driving must keep sliding from wherever
    // the emitted value currently is, per the class header's "RAMP MODE"
    // section (this is what makes a direction reversal slew smoothly
    // THROUGH zero instead of snapping back to 0 first).
    if (state_ != State::kDriving) {
        emitted_value_ = 0.0;
    }
    state_ = State::kDriving;
    direction_ = dir;
}

void TeleopCore::commit_digit_buffer() {
    const bool committing_ramp_rate = entry_mode_is_ramp_rate_;
    entry_mode_is_ramp_rate_ = false;  // always disarm, whether or not the commit below actually applies
    if (digit_buffer_.empty()) return;
    char* end = nullptr;
    const double parsed = std::strtod(digit_buffer_.c_str(), &end);
    const bool valid = (end != digit_buffer_.c_str());
    digit_buffer_.clear();
    if (!valid) return;
    if (committing_ramp_rate) {
        const double clamped = clamp_ramp_rate(mode_, parsed);
        // kSpeed targets duty_ramp_ too -- see ramp_rate_for_mode().
        if (mode_ == TeleopMode::kErpm) {
            erpm_ramp_ = clamped;
        } else {
            duty_ramp_ = clamped;
        }
        return;
    }
    const double clamped = clamp_digit_entry(mode_, parsed);
    if (mode_ == TeleopMode::kDuty) {
        duty_mag_ = clamped;
    } else if (mode_ == TeleopMode::kErpm) {
        erpm_mag_ = clamped;
    } else {
        speed_mag_ = clamped;
    }
}

KeyEvent TeleopCore::handle_key(char c, double now_s) {
    KeyEvent ev;

    // ANY keypress refreshes the deadman clock -- see the class header's
    // "auto-stops after N seconds without ANY keypress" contract.
    last_key_time_s_ = now_s;
    has_key_ever_ = true;

    switch (c) {
        case 'D':
            // Switching mode while driving stops first, exactly like the
            // stop key -- see the class header's "RAMP MODE" section for
            // why (never slew the emitted value across incompatible
            // units). No-op if already in duty mode and not driving.
            // reset_governor() covers switching mode WITHOUT currently
            // driving too (begin_brake() below already resets it when
            // driving) -- see "SPEED GOVERNOR MODE"'s reset contract.
            if (state_ == State::kDriving) begin_brake("mode_switch", now_s);
            reset_governor();
            mode_ = TeleopMode::kDuty;
            break;
        case 'E':
            if (state_ == State::kDriving) begin_brake("mode_switch", now_s);
            reset_governor();
            mode_ = TeleopMode::kErpm;
            break;
        case 'V':
            if (state_ == State::kDriving) begin_brake("mode_switch", now_s);
            reset_governor();
            mode_ = TeleopMode::kSpeed;
            break;
        case '+':
        case '=':
            if (mode_ == TeleopMode::kDuty) {
                duty_mag_ = clamp_step_result(mode_, duty_mag_ + config_.duty_step);
            } else if (mode_ == TeleopMode::kErpm) {
                erpm_mag_ = clamp_step_result(mode_, erpm_mag_ + config_.erpm_step);
            } else {
                speed_mag_ = clamp_step_result(mode_, speed_mag_ + config_.erpm_step);
            }
            break;
        case '-':
        case '_':
            if (mode_ == TeleopMode::kDuty) {
                duty_mag_ = clamp_step_result(mode_, duty_mag_ - config_.duty_step);
            } else if (mode_ == TeleopMode::kErpm) {
                erpm_mag_ = clamp_step_result(mode_, erpm_mag_ - config_.erpm_step);
            } else {
                speed_mag_ = clamp_step_result(mode_, speed_mag_ - config_.erpm_step);
            }
            break;
        case 'w':
            // Also ignored while in_trim_mode() -- see the class
            // header's "TRIM MODE SAFETY DECISION".
            if (state_ != State::kAborted && !trim_mode_) start_driving(DriveDirection::kForward);
            break;
        case 's':
            if (state_ != State::kAborted && !trim_mode_) start_driving(DriveDirection::kReverse);
            break;
        case ' ':
        case 'x':
            if (state_ != State::kAborted) {
                begin_brake("stop", now_s);
            }
            break;
        case 'c':
            if (state_ == State::kAborted) {
                state_ = State::kIdle;
                direction_ = DriveDirection::kStopped;
                abort_reason_.clear();
            }
            break;
        case 'z':
            ramp_enabled_ = !ramp_enabled_;
            break;
        case 'Z':
            // Arm ramp-rate entry: discards any in-progress buffer (e.g. a
            // half-typed magnitude) and starts a fresh one that the next
            // ENTER commits to the current mode's ramp rate instead.
            entry_mode_is_ramp_rate_ = true;
            digit_buffer_.clear();
            break;
        case 'q':
            ev.type = KeyEventType::kQuit;
            break;
        // --- RETIRED KEYS (pre-WASD map) -----------------------------
        // Deliberately INERT: they perform no drive/steer action, and
        // only hand the caller a hint naming the replacement. Silent
        // no-ops would be worse here -- an operator whose muscle memory
        // still says 'f' would otherwise get no feedback at all and
        // might escalate to a harder press/hold on live hardware.
        case 'f':
            ev.retired_hint = "key 'f' retired -- use 'w' (forward)";
            break;
        case 'b':
        case 'r':
            ev.retired_hint = "key 'b'/'r' retired -- use 's' (backward)";
            break;
        case 'j':
            ev.retired_hint = "key 'j' retired -- use 'a' (steer left)";
            break;
        case 'l':
            ev.retired_hint = "key 'l' retired -- use 'd' (steer right)";
            break;
        case 'e':
            ev.retired_hint = "key 'e' retired -- use 'E' (erpm mode)";
            break;
        case 'v':
            ev.retired_hint = "key 'v' retired -- use 'V' (speed-governor mode)";
            break;
        case 'A':
            ev.retired_hint = "key 'A' retired -- use 'Z' (arm ramp-rate entry)";
            break;
        case 0x1B:  // ESC
            digit_buffer_.clear();
            entry_mode_is_ramp_rate_ = false;
            break;
        // --- steering sub-state-machine (see TeleopCore.h's "STEERING
        // SUB-STATE-MACHINE" section) -- deliberately INDEPENDENT of the
        // drive state above: these branches never read/write duty_mag_/
        // erpm_mag_/state_/direction_ etc. (except 'T', which may kick
        // off a drive brake, and only via the same begin_brake() the
        // mode-switch keys already use).
        case 'a': {
            const double step = in_trim_mode() ? config_.steer_fine_step : config_.steer_coarse_step;
            set_steering_position(steering_position_ + (config_.steer_invert ? step : -step));
            break;
        }
        case 'd': {
            const double step = in_trim_mode() ? config_.steer_fine_step : config_.steer_coarse_step;
            set_steering_position(steering_position_ + (config_.steer_invert ? -step : step));
            break;
        }
        case 'k':
        case 'R':
            // Full synonyms -- see the class header's "DELIBERATE
            // SIMPLIFICATION" note on why 'R' is NOT trim-mode-gated.
            set_steering_position(config_.steer_center);
            break;
        case 'T':
            trim_mode_ = !trim_mode_;
            // Entering trim mode (now true) stops the drive first,
            // exactly like the 'd'/'e'/'v' mode-switch keys -- see
            // "TRIM MODE SAFETY DECISION". Exiting (now false) takes no
            // drive action; step size reverts to coarse automatically
            // via steering_step()/in_trim_mode(), nothing to undo here.
            if (trim_mode_ && state_ == State::kDriving) {
                begin_brake("trim_mode", now_s);
            }
            break;
        case 'W':
            // Overwrites any prior un-consumed pending value -- still
            // exactly ONE pending request outstanding, never a queue.
            pending_center_save_ = true;
            pending_center_save_value_ = steering_position_;
            break;
        case '\r':
        case '\n':
            commit_digit_buffer();
            break;
        default:
            if ((c >= '0' && c <= '9') || c == '.') {
                digit_buffer_ += c;
            }
            // Any other byte: ignored as a command, but the deadman clock
            // above was still refreshed by it.
            break;
    }
    return ev;
}

TeleopMotorAction TeleopCore::step(double now_s) {
    // Ramp dt bookkeeping -- unconditional, every call, mirroring
    // DriverCore::tick()'s own "first-ever call has dt==0" warm-up
    // convention (see TeleopCore.h's class header). Deliberately computed
    // up front regardless of which state branch below actually uses it,
    // so it stays correct even across ticks where driving wasn't active
    // (e.g. idle for a while, then 'f' -- the very next step() call's dt
    // is still just one normal tick period, not the whole idle interval).
    const double dt = has_stepped_ ? (now_s - last_step_time_s_) : 0.0;
    last_step_time_s_ = now_s;
    has_stepped_ = true;

    TeleopMotorAction action;

    if (state_ == State::kAborted) {
        action.type = TeleopMotorAction::Type::kBrake;
        action.value = config_.brake_amps;
        return action;
    }

    if (state_ == State::kBraking) {
        if (now_s - brake_started_s_ >= config_.brake_hold_ms / 1000.0 - kTimeEpsilonS) {
            state_ = State::kIdle;
            action.type = TeleopMotorAction::Type::kNone;
            return action;
        }
        action.type = TeleopMotorAction::Type::kBrake;
        action.value = config_.brake_amps;
        return action;
    }

    if (state_ == State::kDriving) {
        if (now_s - last_key_time_s_ >= config_.deadman_ms / 1000.0 - kTimeEpsilonS) {
            begin_brake("deadman", now_s);
            action.type = TeleopMotorAction::Type::kBrake;
            action.value = config_.brake_amps;
            return action;
        }

        if (mode_ == TeleopMode::kSpeed) {
            return step_speed_governor(dt);
        }

        const double target = target_value();
        if (ramp_enabled_) {
            const double max_delta = ramp_rate_for_mode(mode_) * dt;
            if (emitted_value_ < target) {
                emitted_value_ = std::min(target, emitted_value_ + max_delta);
            } else if (emitted_value_ > target) {
                emitted_value_ = std::max(target, emitted_value_ - max_delta);
            }
        } else {
            // Ramp OFF: identical to the original (pre-ramp) instant
            // behavior -- emitted_value_ simply IS the target every tick.
            emitted_value_ = target;
        }

        action.type = (mode_ == TeleopMode::kDuty) ? TeleopMotorAction::Type::kDuty : TeleopMotorAction::Type::kRpm;
        // VEHICLE -> MOTOR frame (see TeleopConfig::drive_invert).
        action.value = emitted_value_ * drive_sign();
        return action;
    }

    // kIdle.
    action.type = TeleopMotorAction::Type::kNone;
    return action;
}

TeleopMotorAction TeleopCore::step_speed_governor(double dt) {
    // target_value() here is direction()*magnitude(), and magnitude() in
    // kSpeed is the TARGET ERPM (see TeleopCore.h's class header) -- NOT
    // a duty value, despite this function's only output being duty.
    const double target_erpm = target_value();
    const double error = target_erpm - erpm_filtered_;

    const double effective_v_in = std::max(v_in_last_, kSpeedMinEffectiveVIn);
    const double duty_ff =
        (config_.speed_ff_gain > 1e-9) ? target_erpm / (config_.speed_ff_gain * effective_v_in) : 0.0;

    // Incremental PI with anti-windup: only commit the integrator step
    // if the resulting duty command does NOT need clamping this tick --
    // otherwise freeze it at its pre-tick value (std::min/max just
    // SELECT one of their inputs, introducing no rounding of their own,
    // so an exact `==` reliably detects "no clamping occurred" here).
    const double integ_candidate = speed_integrator_ + config_.speed_ki * error * dt;
    const double duty_cmd_unclamped = duty_ff + config_.speed_kp * error + integ_candidate;
    const double duty_cmd = std::min(config_.max_duty, std::max(-config_.max_duty, duty_cmd_unclamped));
    if (duty_cmd == duty_cmd_unclamped) {
        speed_integrator_ = integ_candidate;
    }

    // The governor's own gentleness guarantee: ALWAYS slew-limit the
    // emitted duty toward duty_cmd by duty_ramp_, regardless of
    // ramp_enabled_ (see the class header's "SPEED GOVERNOR MODE"
    // section) -- reuses the same emitted_value_ slewing machinery as
    // duty/erpm ramp mode above, just always-on here.
    const double max_delta = duty_ramp_ * dt;
    if (emitted_value_ < duty_cmd) {
        emitted_value_ = std::min(duty_cmd, emitted_value_ + max_delta);
    } else if (emitted_value_ > duty_cmd) {
        emitted_value_ = std::max(duty_cmd, emitted_value_ - max_delta);
    }

    TeleopMotorAction action;
    action.type = TeleopMotorAction::Type::kDuty;  // speed mode NEVER sends SET_RPM.
    // VEHICLE -> MOTOR frame; the matching MOTOR -> VEHICLE conversion on
    // the measured erpm lives in feed_telemetry() (see drive_invert).
    action.value = emitted_value_ * drive_sign();
    return action;
}

void TeleopCore::feed_telemetry(const VescValues& values, double now_s) {
    (void)now_s;
    if (state_ == State::kAborted) return;

    const bool over_current = std::fabs(values.current_motor) > config_.current_abort;
    const bool has_fault = values.fault != 0;
    if (over_current || has_fault) {
        state_ = State::kAborted;
        direction_ = DriveDirection::kStopped;
        reset_governor();  // see "SPEED GOVERNOR MODE"'s reset contract.

        std::ostringstream oss;
        if (over_current) {
            oss << "current " << std::fixed << std::setprecision(1) << std::fabs(values.current_motor) << "A > "
                << std::fixed << std::setprecision(1) << config_.current_abort << "A abort";
        } else {
            const std::string name = fault_name(values.fault);
            oss << "fault " << static_cast<int>(values.fault);
            if (!name.empty()) oss << " (" << name << ")";
        }
        abort_reason_ = oss.str();
        return;
    }

    // Kept warm regardless of the current mode/driving state, so it's
    // ready the moment the operator switches into speed mode and starts
    // driving -- see "SPEED GOVERNOR MODE".
    v_in_last_ = values.v_in;
    if (!erpm_filter_initialized_) {
        erpm_filtered_ = values.erpm * drive_sign();
        erpm_filter_initialized_ = true;
    } else {
        erpm_filtered_ =
            kErpmFilterAlpha * (values.erpm * drive_sign()) + (1.0 - kErpmFilterAlpha) * erpm_filtered_;
    }
}

double TeleopCore::deadman_remaining_s(double now_s) const {
    const double window_s = config_.deadman_ms / 1000.0;
    if (!has_key_ever_) return window_s;
    const double remaining = (last_key_time_s_ + window_s) - now_s;
    return remaining > 0.0 ? remaining : 0.0;
}

}  // namespace vesc
