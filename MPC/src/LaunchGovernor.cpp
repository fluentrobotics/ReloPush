#include "mpc/LaunchGovernor.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace mpc {

namespace {

// Moves `from` toward `to` by at most `max_delta` (>= 0), snapping exactly
// to `to` once within reach (avoids float overshoot/oscillation once
// caught up). Used by STOP-SNAP and the MOVING-floor override below (LAUNCH
// itself is a single-tick KICK, not a ramp -- see LaunchGovernor.h's doc
// comment) -- see that same doc comment for why both remaining ramps are
// anchored to the caller's last_published_v rather than any state internal
// to this class.
double rate_limited_toward(double from, double to, double max_delta) {
    const double delta = to - from;
    if (std::fabs(delta) <= max_delta) {
        return to;
    }
    return from + std::copysign(max_delta, delta);
}

// See extract_ref_stop_events's doc comment (LaunchGovernor.h) for why
// segments this short are skipped entirely.
constexpr double kMinSegmentDtS = 1.0e-3;

// "Far in the future" end-of-window sentinel for the trajectory's own
// (unbounded) final stop event -- see extract_ref_stop_events' doc comment.
// Deliberately a separate constant from LaunchGovernor::kNoTimeToRefStopHint
// even though the two share a magnitude: that one is a RESULT sentinel
// (a duration, "nothing known"), this one is an END-TIME sentinel (a point
// in time, "never ends") -- different roles that happen to need the same
// order of magnitude, not the same value by necessity.
constexpr double kUnboundedRefStopEndS = 1.0e9;

}  // namespace

void LaunchGovernor::reset() {
    state_ = LaunchGovernorState::kNormal;
    direction_ = 1.0;
    escalation_step_ = 0;
    capped_reported_ = false;
    last_cap_warn_elapsed_ = 0.0;
    raw_exit_streak_ = 0;
    in_correction_launch_ = false;
    self_elapsed_s_ = 0.0;
    last_correction_exit_elapsed_ = -1.0;
    settle_attempts_this_stop_event_ = 0;
    settle_cap_warned_this_stop_event_ = false;
    prev_time_to_ref_stop_ = kNoTimeToRefStopHint;
}

LaunchGovernorDecision LaunchGovernor::step(double v_meas, double v_des, double ref_vel_now,
                                             double last_published_v, const RobotSpec& spec,
                                             double dt, double wall_elapsed_in_state,
                                             bool v_raw_fresh, double v_raw,
                                             double time_to_ref_stop, double position_error) {
    LaunchGovernorDecision out;

    // Active only when spec.min_moving_speed > 0 -- see G3. A spec with
    // min_moving_speed <= 0 means the plant has no modeled deadband at all,
    // so there is nothing for this class to smooth; stay permanently
    // transparent and pinned to kNormal.
    if (!(spec.min_moving_speed > 0.0)) {
        state_ = LaunchGovernorState::kNormal;
        return out;
    }

    // CORRECTION-LAUNCH free-running clock (see self_elapsed_s_'s own doc
    // comment) -- advanced unconditionally every ACTIVE tick, regardless of
    // state, so the cooldown check below has a self-contained notion of
    // elapsed time.
    self_elapsed_s_ += dt;

    // NEW STOP EVENT DETECTION (see RobotSpec::settle_max_attempts's doc
    // comment): a transition from "still approaching" (>0) to "at/inside"
    // (<=0) resets BOTH the per-stop-event attempt count and the cooldown
    // baseline for the stop event just entered -- a cooldown timestamp (or
    // attempt tally) from a DIFFERENT, earlier stop event must never carry
    // over and constrain a fresh one. Checked/updated before this tick's
    // entry logic below so a correction-launch entry evaluated THIS tick
    // already sees the fresh event's reset budget.
    if (prev_time_to_ref_stop_ > 0.0 && time_to_ref_stop <= 0.0) {
        settle_attempts_this_stop_event_ = 0;
        settle_cap_warned_this_stop_event_ = false;
        last_correction_exit_elapsed_ = -1.0;
    }
    prev_time_to_ref_stop_ = time_to_ref_stop;

    const double max_delta = spec.max_accel * dt;

    if (state_ == LaunchGovernorState::kLaunch) {
        // FAST EXIT debounce (see kRawExitDebounceCount's doc comment): only
        // a genuinely FRESH raw sample advances or resets the streak -- a
        // tick with no fresh sample (v_raw_fresh=false, the default for any
        // caller that has not wired this up) leaves it untouched.
        if (v_raw_fresh) {
            if (std::fabs(v_raw) >= kExitFraction * spec.min_moving_speed) {
                ++raw_exit_streak_;
            } else {
                raw_exit_streak_ = 0;
            }
        }
        const bool raw_exit = raw_exit_streak_ >= kRawExitDebounceCount;
        // FALLBACK: the pre-existing filtered-estimate exit check, unchanged
        // -- always evaluated regardless of raw_exit, so a caller that never
        // passes v_raw_fresh=true exits exactly as before.
        const bool fallback_exit = std::fabs(v_meas) >= kExitFraction * spec.min_moving_speed;

        // LAUNCH -> NORMAL: motion confirmed, by EITHER trigger.
        if (raw_exit || fallback_exit) {
            state_ = LaunchGovernorState::kNormal;
            escalation_step_ = 0;
            capped_reported_ = false;
            last_cap_warn_elapsed_ = 0.0;
            raw_exit_streak_ = 0;
            // CORRECTION-LAUNCH bookkeeping (see RobotSpec::settle_max_attempts's
            // doc comment): a confirmed breakaway still counts as one
            // attempt and restarts the cooldown -- "succeeded" here only
            // means "broke away," not "fully closed position_error" (see
            // LaunchGovernor.h's CORRECTION-LAUNCH paragraph).
            if (in_correction_launch_) {
                ++settle_attempts_this_stop_event_;
                last_correction_exit_elapsed_ = self_elapsed_s_;
                in_correction_launch_ = false;
            }
            out.just_exited_launch = true;
            return out;  // override_active=false: the MPC's own v_des takes over immediately.
        }

        // CORRECTION-LAUNCH GIVE-UP TIMEOUT (see LaunchGovernor.h's
        // CORRECTION-LAUNCH paragraph): checked before the escalation/KICK
        // logic below, and only ever relevant while in_correction_launch_
        // (a plain reference-motion launch has no timeout -- it escalates
        // instead, below). No breakaway confirmed within
        // kCorrectionLaunchTimeoutS of entering LAUNCH -> give up this
        // attempt: exit to NORMAL transparently (never ratchet speed for a
        // positioning nudge), count the attempt, start the cooldown, warn
        // once.
        if (in_correction_launch_ && wall_elapsed_in_state >= kCorrectionLaunchTimeoutS) {
            state_ = LaunchGovernorState::kNormal;
            escalation_step_ = 0;
            capped_reported_ = false;
            last_cap_warn_elapsed_ = 0.0;
            raw_exit_streak_ = 0;
            in_correction_launch_ = false;
            ++settle_attempts_this_stop_event_;
            last_correction_exit_elapsed_ = self_elapsed_s_;
            std::cerr << "[LaunchGovernor] correction launch gave up after " << wall_elapsed_in_state
                      << "s without confirmed breakaway (attempt " << settle_attempts_this_stop_event_
                      << "/" << spec.settle_max_attempts << " for this stop event)" << std::endl;
            out.just_exited_launch = true;
            return out;  // override_active=false: transparent this tick.
        }

        if (in_correction_launch_) {
            // Fixed-target KICK, no escalation (see LaunchGovernor.h's
            // CORRECTION-LAUNCH paragraph and RobotSpec::settle_max_attempts's
            // doc comment: "never ratchet speed for a positioning nudge").
            // Same formula/anchor as the plain entry's own KICK -- just
            // republished flat, every tick, until confirmed or timed out
            // above.
            const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);
            out.override_active = true;
            out.v_published = direction_ * base;
            out.derive_accel_from_rest = true;
            return out;
        }

        // Still stuck below breakaway: compute the (possibly escalated)
        // launch TARGET and KICK v_published straight to it (no ramp -- see
        // LaunchGovernor.h's doc comment). The target magnitude itself is
        // recomputed fresh from wall_elapsed_in_state every call (rather
        // than compounding a stored running value tick-by-tick) so it is an
        // exact, drift-free function of elapsed time alone -- which is also
        // what makes it exercisable with directly-injected elapsed values
        // in a unit test.
        const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);
        const double cap = kEscalationCapFactor * base;
        const double elapsed = std::max(0.0, wall_elapsed_in_state);
        const int current_step = static_cast<int>(std::floor(elapsed / kEscalationIntervalS));

        double target_magnitude =
            base * std::pow(kEscalationFactor, static_cast<double>(current_step));
        const bool now_capped = target_magnitude >= cap;
        if (now_capped) {
            target_magnitude = cap;
        }

        if (current_step > escalation_step_) {
            escalation_step_ = current_step;
            // Per-step warnings stop once the cap has been hit at least once
            // (the TARGET is no longer actually changing step to step) --
            // the cap-cadence warning below takes over from there.
            if (!capped_reported_) {
                std::cerr << "[LaunchGovernor] escalating launch target to " << target_magnitude
                          << " m/s (step " << escalation_step_ << ", " << elapsed
                          << "s in LAUNCH without breakaway)" << std::endl;
            }
        }
        if (now_capped) {
            if (!capped_reported_ || (elapsed - last_cap_warn_elapsed_) >= kCapWarnIntervalS) {
                std::cerr << "[LaunchGovernor] launch target capped at " << cap
                          << " m/s -- still waiting for breakaway after " << elapsed << "s"
                          << std::endl;
                last_cap_warn_elapsed_ = elapsed;
                capped_reported_ = true;
            }
        }

        const double target_signed = direction_ * target_magnitude;
        out.override_active = true;
        out.v_published = target_signed;  // KICK: direct jump, not rate_limited_toward().
        // SUSTAINED kick: derive accel from REST every tick, not from
        // last_published_v -- see LaunchGovernorDecision's doc comment for
        // why (last_published_v already equals target_signed by the SECOND
        // tick at an unchanged target, which would otherwise derive accel=0
        // and strand the plant with no further push).
        out.derive_accel_from_rest = true;
        return out;
    }

    // state_ == kNormal.
    //
    // LAUNCH-entry is checked BEFORE stop-snap (see LaunchGovernor.h's doc
    // comment for why): it is the more specific/urgent transition, and its
    // v_des-based "wants motion" signal is only ever positive when the
    // solve's own reference-tracking already sees a reason to move, unlike
    // the instantaneous ref_vel_now used below.
    //
    // ref_indicates_motion is an ADDITIONAL guard beyond the three the
    // governor spec lists (v_meas/v_des only) -- empirically required (see
    // this task's report) to stop a spurious re-launch during a
    // trajectory's last few millimeters of final approach: once the
    // reference has genuinely reached its terminal hold (ref_vel_now truly
    // ~0, staying there), the MPC can still command a tiny sub-
    // min_moving_speed v_des purely to close a negligible RESIDUAL
    // POSITION error -- v_des alone cannot tell "reference about to want
    // motion" apart from "tiny position-only correction on an otherwise-
    // finished reference", but ref_vel_now can. A reference that is
    // genuinely about to want motion (a fresh ramp start, or a reversal
    // crossing) has |ref_vel_now| already above this SAME threshold well
    // before v_des exceeds kWantsMotionThreshold, so this guard does not
    // delay a real launch -- it only suppresses one when the reference
    // itself has truly settled, correctly deferring to STOP-SNAP instead.
    const bool at_rest = std::fabs(v_meas) < kAtRestFraction * spec.min_moving_speed;
    const bool wants_motion = std::fabs(v_des) > kWantsMotionThreshold;
    const bool cannot_break_away = std::fabs(v_des) < spec.min_moving_speed;
    const bool ref_indicates_motion = std::fabs(ref_vel_now) > kStopSnapRefVelThreshold;

    if (at_rest && wants_motion && cannot_break_away && ref_indicates_motion) {
        state_ = LaunchGovernorState::kLaunch;
        direction_ = (v_des > 0.0) ? 1.0 : -1.0;
        escalation_step_ = 0;
        capped_reported_ = false;
        last_cap_warn_elapsed_ = 0.0;
        raw_exit_streak_ = 0;  // fresh LAUNCH sojourn -- no debounce progress carried over.

        // Entry tick: step 0 (no escalation yet) -- the SAME KICK formula
        // as the "still in LAUNCH" branch above (a direct jump to the
        // target, not a ramp -- see LaunchGovernor.h's doc comment), inlined
        // here since wall_elapsed_in_state on the entry tick itself is
        // whatever stale value the caller had from its PREVIOUS state and
        // must not be used for escalation math yet.
        const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);
        const double target_signed = direction_ * base;
        out.override_active = true;
        out.v_published = target_signed;  // KICK: direct jump, not rate_limited_toward().
        out.derive_accel_from_rest = true;  // SUSTAINED kick -- see the other KICK site's comment.
        return out;
    }

    // CORRECTION-LAUNCH entry (see LaunchGovernor.h's CORRECTION-LAUNCH
    // paragraph for the full contract): a SECOND, independent OR-branch of
    // NORMAL->LAUNCH -- mutually exclusive with the plain entry above (that
    // one requires ref_indicates_motion; this one requires the opposite).
    // Exists to finish closing a residual POSITION error at an otherwise-
    // stationary reference -- exactly the case the |ref_vel_now| guard
    // above deliberately suppresses for the plain entry, which without this
    // branch would leave the plant permanently parked short whenever
    // STOP-SNAP happened to freeze it more than settle_position_tolerance
    // away from the reference.
    const bool ref_stationary = !ref_indicates_motion;  // |ref_vel_now| <= kStopSnapRefVelThreshold.
    const bool position_error_over_tolerance = position_error > spec.settle_position_tolerance;
    if (at_rest && ref_stationary && position_error_over_tolerance && wants_motion) {
        const bool cooldown_elapsed =
            (last_correction_exit_elapsed_ < 0.0) ||
            ((self_elapsed_s_ - last_correction_exit_elapsed_) >= spec.settle_cooldown_s);
        const bool attempts_available = settle_attempts_this_stop_event_ < spec.settle_max_attempts;

        if (cooldown_elapsed && attempts_available) {
            state_ = LaunchGovernorState::kLaunch;
            in_correction_launch_ = true;
            direction_ = (v_des > 0.0) ? 1.0 : -1.0;  // works for REVERSE targets too.
            escalation_step_ = 0;
            capped_reported_ = false;
            last_cap_warn_elapsed_ = 0.0;
            raw_exit_streak_ = 0;

            // Same fixed-target KICK formula as the plain entry above --
            // see LaunchGovernor.h's CORRECTION-LAUNCH paragraph for why
            // this deliberately does NOT scale with v_des. One entry-log
            // line (unconditional, not rate-limited -- correction launches
            // are rare/bounded by settle_max_attempts, unlike escalation's
            // own per-tick cadence) so a caller's log can attribute
            // correction-launch counts even on the common, silent-success
            // path (only the give-up/cap-exhausted cases print anything
            // otherwise).
            std::cerr << "[LaunchGovernor] correction launch entered (attempt "
                       << (settle_attempts_this_stop_event_ + 1) << "/" << spec.settle_max_attempts
                       << ", position_error=" << position_error << "m, direction="
                       << (direction_ > 0.0 ? "+" : "-") << ")" << std::endl;
            const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);
            out.override_active = true;
            out.v_published = direction_ * base;
            out.derive_accel_from_rest = true;
            return out;
        }
        if (!attempts_available && !settle_cap_warned_this_stop_event_) {
            std::cerr << "[LaunchGovernor] settle attempts exhausted (" << settle_attempts_this_stop_event_
                       << "/" << spec.settle_max_attempts << ") for this stop event with residual "
                          "position_error=" << position_error
                       << "m > settle_position_tolerance=" << spec.settle_position_tolerance
                       << "m -- staying parked" << std::endl;
            settle_cap_warned_this_stop_event_ = true;
        }
        // else (cooldown not yet elapsed): fall through silently -- this is
        // the expected/common case between attempts, not a giving-up
        // condition worth a warning; the next tick (or a later one, once
        // the cooldown passes) will retry.
    }

    // STOP-SNAP: rate-limited toward 0.0 (NOT an instant jump) -- see
    // LaunchGovernor.h's doc comment for why. Anchored to the SAME
    // last_published_v, so the caller's derived accel is bounded and
    // non-oscillating on the way there AND, critically, never claims to
    // have reached 0.0 faster than the actuator actually could -- an
    // earlier "publish exactly 0.0 in one tick" version of this branch
    // permanently DESYNCED this class's own notion of "where we last
    // published" from the plant's true (accel-clamped) state whenever the
    // needed correction exceeded one tick's max_accel*dt budget, silently
    // stranding a residual the plant never actually shed. See this task's
    // report for the empirical failure this caused (a subsequent reversal
    // LAUNCH's escalation cap was not enough to also cancel out the
    // stranded residual, on top of reaching the new breakaway target).
    const bool des_below_sustain = std::fabs(v_des) < spec.min_sustain_speed;
    const bool ref_wants_zero = std::fabs(ref_vel_now) < kStopSnapRefVelThreshold;
    if (des_below_sustain && ref_wants_zero) {
        out.override_active = true;
        out.v_published = rate_limited_toward(last_published_v, 0.0, max_delta);
        return out;
    }

    // MOVING-floor (ANTI-RE-STALL -- see LaunchGovernor.h's doc comment):
    // checked AFTER LAUNCH-entry and STOP-SNAP above, so a genuine reversal
    // or stop always takes precedence. Guards against a noise-perturbed
    // v_cmd_equiv dipping the SIM's deadband model back into a stall
    // mid-cruise purely from actuation noise, by never publishing INTO the
    // dead zone while the reference still wants motion.
    const bool plant_believed_moving = std::fabs(v_meas) >= kExitFraction * spec.min_moving_speed;
    const bool ref_wants_motion = std::fabs(ref_vel_now) >= kMovingFloorRefVelThreshold;
    const double floor = spec.min_sustain_speed +
                          0.3 * (spec.min_moving_speed - spec.min_sustain_speed) + 0.02;
    const bool des_below_floor = std::fabs(v_des) < floor;
    // FOURTH condition (see kFloorYieldHorizonS's doc comment for the full
    // "why"): the floor yields this close to a reference STOP EVENT -- an
    // intermediate hold or the trajectory's own end (time_to_ref_stop is
    // hold-aware -- see that parameter's own doc comment in
    // LaunchGovernor.h) -- since a low v_des here is the position cost
    // reacting to the approaching stop, not noise to guard against.
    const bool near_ref_stop = time_to_ref_stop <= kFloorYieldHorizonS;
    if (plant_believed_moving && ref_wants_motion && des_below_floor && !near_ref_stop) {
        // sign(ref_vel_now), not sign(v_des): |ref_vel_now| is guaranteed
        // comfortably nonzero here (>= kMovingFloorRefVelThreshold), while
        // v_des's own magnitude -- and so, potentially, its sign near zero
        // -- is exactly what this branch does not trust (see
        // LaunchGovernor.h's doc comment).
        const double dir = (ref_vel_now > 0.0) ? 1.0 : -1.0;
        out.override_active = true;
        out.v_published = rate_limited_toward(last_published_v, floor * dir, max_delta);
        return out;
    }

    // Otherwise: transparent. The MPC's own v_des (and its own accel) is
    // what gets published, unchanged.
    return out;
}

// See LaunchGovernor.h's own doc comment on this function for the full
// contract (segment semantics, the skip-degenerate-segments rule, run
// merging, the unbounded trailing end event).
std::vector<RefStopEvent> extract_ref_stop_events(const std::vector<double>& times,
                                                    const std::vector<double>& ref_vels,
                                                    double vel_threshold, double min_hold_s) {
    std::vector<RefStopEvent> out;
    if (times.empty()) {
        return out;
    }

    bool run_open = false;
    double run_start = 0.0;
    double run_end = 0.0;
    const size_t n = std::min(times.size(), ref_vels.size());
    for (size_t i = 0; i + 1 < n; ++i) {
        const double seg_dt = times[i + 1] - times[i];
        if (seg_dt <= kMinSegmentDtS) {
            // Degenerate segment: transparent to the run below (neither
            // breaks nor extends it) -- see this function's own doc
            // comment.
            continue;
        }
        if (std::fabs(ref_vels[i]) < vel_threshold) {
            if (!run_open) {
                run_open = true;
                run_start = times[i];
            }
            run_end = times[i + 1];
        } else if (run_open) {
            if (run_end - run_start >= min_hold_s) {
                out.push_back(RefStopEvent{run_start, run_end});
            }
            run_open = false;
        }
    }
    // A run still open at the end of the scan (the trajectory's low-
    // velocity tail runs right up to its last sample) closes out here --
    // not specially merged with the final end event appended below, see
    // this function's own doc comment for why that is harmless.
    if (run_open && (run_end - run_start >= min_hold_s)) {
        out.push_back(RefStopEvent{run_start, run_end});
    }

    out.push_back(RefStopEvent{times.back(), kUnboundedRefStopEndS});
    return out;
}

// See LaunchGovernor.h's own doc comment on this function for the full
// contract.
double time_to_next_ref_stop(const std::vector<RefStopEvent>& events, double ref_time) {
    if (events.empty()) {
        return LaunchGovernor::kNoTimeToRefStopHint;
    }
    // events is end_time-ordered (non-decreasing) by construction -- see
    // extract_ref_stop_events, which emits runs in scan order followed by
    // one final (largest end_time) unbounded event -- so a binary search on
    // end_time finds the earliest event this ref_time has not yet fully
    // passed.
    auto it = std::lower_bound(
        events.begin(), events.end(), ref_time,
        [](const RefStopEvent& ev, double t) { return ev.end_time < t; });
    if (it == events.end()) {
        return LaunchGovernor::kNoTimeToRefStopHint;
    }
    return it->start_time - ref_time;
}

}  // namespace mpc
