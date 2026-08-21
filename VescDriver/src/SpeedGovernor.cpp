#include "SpeedGovernor.h"

#include <algorithm>
#include <cmath>

namespace vesc {

namespace {
// Floor on the v_in used in the feedforward term's denominator -- guards
// against a division blowing up (or even by zero) on a bogus/startup v_in
// reading, not a claim that 6V is a real operating point. Matches
// TeleopCore's former kSpeedMinEffectiveVIn exactly.
constexpr double kMinEffectiveVIn = 6.0;
// Floor on the configured filter time constant so a misconfigured
// erpm_filter_tau_s<=0 can never divide by zero in feed_erpm() below.
constexpr double kMinFilterTauS = 1e-6;
}  // namespace

void SpeedGovernor::feed_erpm(double erpm_vehicle, double dt_since_last_sample) {
    if (!erpm_filter_initialized_) {
        erpm_filtered_ = erpm_vehicle;
        erpm_filter_initialized_ = true;
        return;
    }
    if (dt_since_last_sample <= 0.0) {
        return;  // repeated/backward-clock sample -- leave the filter unchanged.
    }
    const double tau = std::max(config_.erpm_filter_tau_s, kMinFilterTauS);
    const double alpha = 1.0 - std::exp(-dt_since_last_sample / tau);
    erpm_filtered_ = alpha * erpm_vehicle + (1.0 - alpha) * erpm_filtered_;
}

double SpeedGovernor::step(double dt) {
    const double error = target_erpm_ - erpm_filtered_;
    const double effective_v_in = std::max(v_in_, kMinEffectiveVIn);
    const double duty_ff = (config_.ff_gain > 1e-9) ? target_erpm_ / (config_.ff_gain * effective_v_in) : 0.0;

    // Incremental PI with freeze-on-saturation anti-windup -- see the
    // class header's own doc comment for why an exact `==` reliably
    // detects "no clamping occurred" here.
    const double integ_candidate = integrator_ + config_.ki * error * dt;
    const double duty_cmd_unclamped = duty_ff + config_.kp * error + integ_candidate;
    const double lim = std::fabs(config_.max_duty);
    const double duty_cmd = std::min(lim, std::max(-lim, duty_cmd_unclamped));
    saturated_ = (duty_cmd != duty_cmd_unclamped);
    if (!saturated_) {
        integrator_ = integ_candidate;
    }

    // Mandatory output slew -- ALWAYS applied, regardless of any
    // caller-side ramp toggle (see the class header's own doc comment).
    const double max_delta = config_.duty_slew_per_s * dt;
    if (emitted_duty_ < duty_cmd) {
        emitted_duty_ = std::min(duty_cmd, emitted_duty_ + max_delta);
    } else if (emitted_duty_ > duty_cmd) {
        emitted_duty_ = std::max(duty_cmd, emitted_duty_ - max_delta);
    }
    return emitted_duty_;
}

void SpeedGovernor::reset() {
    integrator_ = 0.0;
    saturated_ = false;
    erpm_filter_initialized_ = false;
    erpm_filtered_ = 0.0;  // reseeded fresh from the next feed_erpm() call.
    emitted_duty_ = 0.0;   // the next drive starts fresh from 0, never resuming a stale slew.
    // v_in_ and target_erpm_ are deliberately left untouched -- see their
    // own accessor/setter doc comments in the header.
}

}  // namespace vesc
