#pragma once

#include <vector>

#include "mpc/RobotSpec.h"

// mpc_controller LAUNCH GOVERNOR: smooths starts from rest against a BLDC
// deadband plant (see RobotSpec's min_moving_speed/min_sustain_speed/
// launch_margin doc comment, and SimCore.h's FEATURE B deadband model).
//
// THE PROBLEM: with true velocity feedback (main.cpp's finite-differenced
// est_state.v -- see that file's header comment), a controller starting from
// rest commands v ~= a*dt on its first solve (a small fraction of
// min_moving_speed). The plant does not move (it is below its own breakaway
// speed). The NEXT solve sees v_meas == 0 again (feedback is truthful) and
// commands roughly the same small v again -- a stall the MPC's own cost
// function cannot cleanly escape (see this task's own report: it can
// eventually escape on its own via unrelated position-error dynamics, but
// slowly and with unbounded overshoot -- categorically worse than a
// controlled launch, never a lurch, but also never a clean one).
//
// THE FIX HAS TWO HALVES, ONE ON EACH SIDE OF THE WIRE. An earlier version
// of this class alone fixed the stall with a RATE-LIMITED RAMP (bounded by
// spec.max_accel) to breakaway speed. That ramp turned out to be the
// dominant source of reference-tracking LAG (~0.2s per launch, measured):
// it was compensating for the SIM plant model under-representing a real
// BLDC/ESC in speed-control mode, which delivers launch CURRENT (hence
// accel) far above its steady-state/comfort value while breaking static
// friction -- see RobotSpec::max_breakaway_accel and SimCore.h's
// resolve_accel_clamp_ceiling(), the sim-side half of this fix. With the sim
// now modeling that headroom, this class's own half is a single-tick KICK:
// publish the (possibly escalating) launch target DIRECTLY, no ramp, the
// instant LAUNCH is entered or re-evaluated -- the truthful accel this
// implies (derived by the caller, see LaunchGovernorDecision below) is
// exactly what a speed-mode ESC actually delivers on a launch, not an
// artifact to be smoothed away. Control hands back to the MPC the instant
// real motion is confirmed (which is also when the MPC's own
// overshoot-correction naturally takes over -- no special handling needed),
// now helped along by a FAST, raw-sample-debounced exit and a one-shot
// observer injection (see step()'s doc comment) instead of waiting on the
// velocity observer's own (deliberately low-gain, see main.cpp's
// kVelocityObsGain) filtered estimate alone.
//
// DESIGN: this class is a small, pure (no ZMQ/IO except rate-limited stderr
// warnings), unit-testable state machine. main.cpp calls step() once per
// non-paused control tick (right after MpcSolver::solve(), before
// build_payload()) and, when the returned decision is active, substitutes
// its v_published for the MPC's own opt_v -- steering is NEVER touched here
// (the MPC's solved delta is always used verbatim). The MPC keeps solving
// every tick regardless of this class's state; this class only ever
// overrides what gets PUBLISHED (a KICK while entering/stuck in LAUNCH; a
// rate-limited ramp toward 0.0 for STOP-SNAP; a rate-limited nudge up to a
// floor, in NORMAL, to avoid re-stalling mid-cruise -- see step()'s doc
// comment for all three).
//
// Deliberately NOT wall-clock-aware itself: `wall_elapsed_in_state` (seconds
// since the CURRENT state -- NORMAL or LAUNCH -- was entered) is computed
// and tracked by the CALLER (main.cpp, via the same wall clock it already
// uses for PAUSE/RESUME bookkeeping -- see compute_effective_elapsed's
// caller in main.cpp) and passed in fresh every call. This keeps the class
// itself free of std::chrono, which is what makes escalation timing (see
// step()'s doc comment) exercisable in a unit test with injected elapsed
// values instead of real sleeps. Likewise `last_published_v` (the actual
// previous wire value) is tracked and passed in by the caller, not cached
// here -- see step()'s doc comment for why the accel-derivation anchor needs
// it (the LAUNCH kick and the STOP-SNAP/MOVING-floor ramps alike).
namespace mpc {

enum class LaunchGovernorState {
    kNormal,
    kLaunch,
};

// One tick's decision. override_active=false means "transparent this tick --
// publish the MPC's own (opt_v, accel-derived-from-it) unchanged"; the
// caller must not alter anything in that case. override_active=true means
// "publish v_published instead of opt_v" -- the caller is responsible for
// re-deriving a truthful accel from it.
//
// UNIFIED ANCHOR: for EVERY override, accel is derived from a single
// consistent anchor, so the wire's accel*dt == Delta(published speed)
// identity holds by construction, and (STOP-SNAP/MOVING-floor specifically)
// never claiming to have reached a target faster than the actuator actually
// could. The anchor is last_published_v EXCEPT for LAUNCH (see
// derive_accel_from_rest below) -- earlier, buggier anchor choices (all
// empirically observed -- see this task's report): LAUNCH anchored to the
// solver's own v0 produced a genuine max_accel/0/max_accel/0 alternation;
// STOP-SNAP anchored to v0 produced an outright negative-feedback
// oscillator; STOP-SNAP publishing an INSTANT 0.0 (even while still
// deriving its accel from last_published_v) silently stranded an un-shed
// residual on the plant whenever the needed correction exceeded one tick's
// max_accel*dt, since the caller's own bookkeeping believed it had already
// arrived.
//
// The THREE overrides no longer share one mechanism, though -- each is
// anchored the way its own physics calls for (see step()'s doc comment for
// the full state machine):
//   - LAUNCH: v_published = (possibly escalating) target_signed, EVERY tick
//     spent in LAUNCH (not just entry), UNBOUNDED by max_accel*dt.
//     derive_accel_from_rest=true accompanies this: the caller must derive
//     accel as (v_published - 0)/dt -- i.e. AS IF LAUNCHING FROM REST EVERY
//     SINGLE TICK, NOT (v_published - last_published_v)/dt. This is
//     deliberately NOT a one-shot impulse: while the target is UNCHANGED
//     tick to tick, last_published_v already equals it too, so the
//     last_published_v-anchored formula would derive accel=0 on every tick
//     after the first -- and since the SIM only ever integrates the wire's
//     "accel" field (never "speed" -- see SimCore.h's AckermannCommand doc
//     comment), a single tick's impulse not fully achieving breakaway (e.g.
//     from actuation noise, or simply not lining up with a full control
//     period) would strand the plant with NO further push at all, forever
//     (empirically observed -- see this task's report: escalation kept
//     climbing to its cap and reporting "still waiting for breakaway"
//     indefinitely in a real multi-robot run, even though each isolated
//     regression test's own single-shot impulse happened to already be
//     just barely sufficient). Deriving from REST every tick instead
//     delivers a SUSTAINED, REPEATED full-strength kick for as long as
//     LAUNCH persists, robust to noise/variance in any one tick. The
//     resulting accel is deliberately allowed to exceed spec.max_accel
//     (see RobotSpec::max_breakaway_accel / SimCore.h's
//     resolve_accel_clamp_ceiling() for the sim-side plant model this
//     relies on) -- the ONE documented exception to the accel*dt ==
//     Delta(published speed)-implies-|accel|<=max_accel norm every other
//     override (and ordinary transparent MPC publishing) upholds elsewhere
//     in this codebase; it models a real ESC's launch-current headroom
//     applied continuously while fighting static friction, not a bug.
//   - STOP-SNAP: still a rate-limited ramp toward 0.0, anchored to
//     last_published_v, bounded by spec.max_accel*dt per tick (never a flat
//     jump -- see step()'s doc comment for why: an instant jump can
//     silently strand an un-shed residual on the plant).
//   - MOVING-floor (anti-re-stall, NORMAL only): a rate-limited nudge toward
//     the floor, anchored to last_published_v, bounded the same way as
//     STOP-SNAP -- in practice this is almost always a small correction
//     reached in one tick (the MPC's own box-constrained v_des cannot
//     itself have jumped far since the previous, presumably-transparent
//     tick), but bounding it anyway keeps it inside the SAME norm STOP-SNAP
//     upholds rather than relying on that "almost always" holding exactly.
//
// just_exited_launch (see step()'s doc comment) is a separate, purely
// informational flag: true on exactly the tick state_ transitions
// kLaunch->kNormal (by EITHER exit path). It does not affect
// override_active/v_published at all -- the caller (main.cpp) uses it only
// to decide whether to one-shot-inject its OWN velocity observer state (this
// class has no observer of its own to inject into).
struct LaunchGovernorDecision {
    bool override_active = false;
    double v_published = 0.0;
    bool just_exited_launch = false;
    // true iff override_active is a LAUNCH kick (entry or continuing) --
    // see derive_accel_from_rest's explanation just above. false (the
    // default) for STOP-SNAP/MOVING-floor/transparent, where the caller
    // keeps anchoring to last_published_v exactly as before.
    bool derive_accel_from_rest = false;
};

class LaunchGovernor {
   public:
    // NORMAL<->LAUNCH hysteresis thresholds, expressed as fractions of
    // RobotSpec::min_moving_speed (LAUNCH entry/exit) or as fixed velocity
    // magnitudes (motion-wanted / stop-snap-reference-zero). Exposed
    // publicly (mirroring SimCore.h's NoiseModel::kMinSigmaPct/kMaxSigmaPct
    // precedent) so callers/tests can reference the SAME constants the
    // implementation uses instead of duplicating magic numbers.
    static constexpr double kAtRestFraction = 0.25;    // |v_meas| below this * min_moving_speed => "at rest".
    static constexpr double kExitFraction = 0.7;       // |v_meas| at/above this * min_moving_speed => motion confirmed.
    static constexpr double kWantsMotionThreshold = 0.01;     // |v_des| above this => MPC "wants" motion.
    static constexpr double kStopSnapRefVelThreshold = 0.02;  // |ref_vel_now| below this => reference wants ~zero.

    // FAST EXIT (see step()'s doc comment): a fresh RAW finite-difference
    // localization sample (BEFORE the caller's own velocity-observer
    // predict/correct blending -- see main.cpp's kVelocityObsGain) is
    // compared against the SAME kExitFraction*min_moving_speed bar the
    // filtered-estimate exit check uses; kRawExitDebounceCount consecutive
    // FRESH raw samples (not consecutive ticks -- ticks without a fresh
    // sample neither advance nor reset the streak) meeting that bar exit
    // LAUNCH immediately, well before the filtered estimate's own low-gain
    // correction would catch up. 2 (not 1) rejects a single noisy outlier
    // sample without adding more than one extra ~1/loc_rate_hz of latency.
    static constexpr int kRawExitDebounceCount = 2;

    // ANTI-RE-STALL MOVING-floor (see step()'s doc comment): while NORMAL,
    // |ref_vel_now| at/above this magnitude means "the reference wants
    // motion" for the purposes of the floor -- deliberately a DIFFERENT
    // (larger) bar than kStopSnapRefVelThreshold above, since the two guards
    // are mutually exclusive by construction (STOP-SNAP wants ref_vel_now
    // NEAR zero; the floor wants it clearly NOT near zero) and the gap
    // between them (0.02-0.05) is a deliberate dead zone where neither fires
    // (ordinary transparent publishing applies there instead).
    static constexpr double kMovingFloorRefVelThreshold = 0.05;

    // FLOOR YIELD NEAR A REFERENCE STOP EVENT (see the MOVING-floor
    // paragraph of step()'s doc comment for where this plugs in): the
    // MOVING-floor above assumes a re-stall this deep into a cruise can only
    // be transient noise, since the reference "wants motion" (|ref_vel_now|
    // >= kMovingFloorRefVelThreshold) for as long as it has segments left to
    // run -- true everywhere EXCEPT the final kFloorYieldHorizonS seconds
    // approaching a REFERENCE STOP EVENT. A stop event is either the
    // trajectory's own absolute end OR an INTERMEDIATE HOLD -- the reference
    // pauses mid-trajectory at a fixed pose for a stretch of time (e.g. a
    // MARS multi-robot rendezvous wait for a handoff/collision window)
    // before resuming, possibly in a new direction. See
    // extract_ref_stop_events/time_to_next_ref_stop below for how both kinds
    // are detected from a received trajectory's waypoints and merged into
    // one sorted list; step()'s time_to_ref_stop parameter (below) is a
    // lookup against that list, generalized from an earlier version of this
    // parameter (time_to_ref_end) that only ever knew about the absolute
    // end.
    //
    // Both kinds of stop event share the SAME underlying failure mode this
    // guard exists to prevent, and the reason a generic "reference end"
    // check does not already cover intermediate holds for free: the
    // waypoint LABEL a leg carries (ReloPush::trajectory_elem::ref_vel, a
    // per-leg NOMINAL speed -- see RobotTrajectoryBuilder.h -- not a true
    // derivative of position) stays constant for as long as that leg lasts
    // and NEVER decelerates on its own, all the way to a stop. Across an
    // intermediate hold specifically, this label can jump directly from one
    // leg's nonzero value to the NEXT leg's differently-signed nonzero value
    // without ever passing through zero -- observed directly in this task's
    // own acceptance scenario (results/sim_handoff/greedy.scn.b64, robot1's
    // ~9.5s rendezvous hold at ref-time ~172.6-182.1: the label jumps
    // +0.2 -> -0.15 while the reference's actual (x,y) position does not
    // move at all). So close to ANY stop, the MPC's own v_des dropping below
    // the floor is the position-tracking cost correctly recognizing the
    // stop is close, not noise to be guarded against. Forcing the floor
    // through this window was empirically observed (see this task's report)
    // to carry the plant straight through the stop at floor speed with ZERO
    // deceleration authority -- landing 0.10-0.15m PAST the trajectory's
    // absolute end (the originally-observed failure), or gliding ~0.38m
    // past an intermediate hold pose before STOP-SNAP finally caught up
    // (which, absent this generalization, only happened once the
    // INTERPOLATED ref_vel_now drifted near zero mid-hold -- a coincidence
    // of the two flanking legs' particular label values, not a reliable
    // trigger: a hold flanked by two SAME-signed labels would never trip it
    // at all) -- before main.cpp's trajectory-completion wall-clock cutoff
    // caught up (which then hard-zeros the command outright, or -- if
    // main.cpp's own process had already exited -- leaves it to the SIM's
    // watchdog auto-brake alone; either way, far from a controlled stop AT
    // the goal/hold pose).
    //
    // Freezing NEAR a hold pose (yielding the floor, letting the MPC's own
    // position-tracking cost bring the plant to rest there instead) is
    // correct even though the reference's own label claims to still want
    // motion: MARS's multi-robot collision-avoidance windows are computed
    // assuming each robot WAITS AT its hold pose for the full hold duration
    // (see MARS/Push Allocation System.md) -- a robot that glides through a
    // hold at floor speed instead is not just less accurate, it can be
    // physically occupying space inside another robot's reserved window
    // while that window's own math still assumes this robot is parked.
    //
    // kFloorYieldHorizonS (0.4s) itself, and the comparison it feeds
    // (time_to_ref_stop <= kFloorYieldHorizonS in step(), below), are
    // UNCHANGED by this generalization -- mirrors the MPC's own prediction
    // horizon (mpc::kHorizon(8) * the nominal 0.05s control period -- see
    // MpcCore.h) rather than importing that constant directly (this class
    // stays decoupled from MpcCore.h by design -- see this file's own
    // header doc comment): once the reference is this close to a stop
    // event, the horizon the solve itself looks across already spans
    // straight through to it, so the solver's own v_des is already reacting
    // to the approaching stop, not just the instantaneous ref_vel_now
    // snapshot the floor's other three conditions look at. What changed is
    // only the INPUT this constant is compared against -- see
    // time_to_ref_stop's own doc comment below.
    static constexpr double kFloorYieldHorizonS = 0.4;

    // Minimum sustained low-effective-velocity duration for
    // extract_ref_stop_events (below) to report an INTERMEDIATE HOLD: a run
    // of consecutive near-zero-effective-velocity trajectory segments
    // shorter than this is more likely quantization or a momentary
    // heading-change pause than a genuine hold worth yielding the floor
    // for. 0.3s comfortably exceeds one control period (0.05s) and typical
    // MARS waypoint spacing (~0.03-0.5s observed in real trajectories) many
    // times over, so it cannot trip on a single closely-spaced sample pair,
    // while staying well under the shortest real rendezvous hold this task
    // observed (9.5s -- see kFloorYieldHorizonS's own doc comment above).
    static constexpr double kRefStopMinHoldS = 0.3;

    // Sentinel default for step()'s time_to_ref_stop parameter (see that
    // parameter's own doc comment below): a caller that never wires this up
    // (any pre-existing call site/test) passes nothing, which resolves to
    // this -- always >> kFloorYieldHorizonS, so the yield guard above never
    // trips and MOVING-floor behavior is preserved exactly as it was before
    // this parameter existed. Also reused by time_to_next_ref_stop (below)
    // as ITS "nothing known" sentinel, so every layer of this mechanism
    // shares one "effectively never" constant. Renamed from
    // kNoTimeToRefEndHint alongside the step() parameter it defaults (see
    // that rename's own doc comment) -- purely a naming generalization, the
    // VALUE and its role are unchanged.
    static constexpr double kNoTimeToRefStopHint = 1.0e9;

    // Escalation policy (load robustness) while stuck in LAUNCH without
    // exiting -- see step()'s doc comment below.
    static constexpr double kEscalationIntervalS = 0.5;
    static constexpr double kEscalationFactor = 1.1;
    static constexpr double kEscalationCapFactor = 1.5;  // cap == this * the base launch speed.
    static constexpr double kCapWarnIntervalS = 1.0;

    // CORRECTION-LAUNCH TIMEOUT (see step()'s CORRECTION-LAUNCH paragraph
    // below): a correction launch deliberately SKIPS escalation entirely --
    // a single fixed-target kick, never ratcheted (see
    // RobotSpec::settle_max_attempts's doc comment: "never ratchet speed
    // for a positioning nudge") -- so it needs its own give-up horizon
    // instead of relying on escalation's own unbounded climb to eventually
    // confirm or cap. Interpreted as the "normal exit window" a plain
    // LAUNCH sojourn needs to confirm breakaway (empirically ~0.2s, and
    // comfortably inside kEscalationIntervalS's own 0.5s first-decision
    // point -- see main.cpp's kVelocityObsGain doc comment:
    // t_breakaway_after_ref_start measured ~0.203s across every trace) plus
    // this feature's own brief's +1.0s margin for a correction launch
    // specifically, which starts from a smaller, noisier residual signal
    // than a genuine reference-motion launch.
    static constexpr double kCorrectionLaunchTimeoutS = kEscalationIntervalS + 1.0;

    LaunchGovernor() = default;

    // Advances the governor one control tick and returns this tick's
    // publish decision. Arguments:
    //   v_meas          - controller's own REAL (feedback-derived) velocity
    //                      estimate, i.e. main.cpp's est_state.v -- NOT the
    //                      delay-compensated prediction (pred4.v). LAUNCH
    //                      entry/exit are about whether the plant has
    //                      ACTUALLY moved, which only the direct measurement
    //                      can answer truthfully.
    //   v_des           - the MPC's own solved stage-0 velocity this tick
    //                      (solve_out.v_cmd), BEFORE any override.
    //   ref_vel_now     - reference velocity at the CURRENT (pause-adjusted)
    //                      reference time -- i.e. get_ref_state_at_time(
    //                      reference_time, ...), deliberately NOT the
    //                      dt-ahead lookahead time_target used to drive the
    //                      solve itself. Consumed only by the STOP-SNAP
    //                      check below (and the LAUNCH-entry ref_vel guard).
    //   last_published_v - the ACTUAL v value this controller published on
    //                      the wire last tick (unscaled model domain,
    //                      tracked by the caller unconditionally every tick,
    //                      governed or transparent -- see main.cpp). The
    //                      STOP-SNAP/MOVING-floor ramps below are defined
    //                      relative to this, not to any internal state of
    //                      this class, so a caller that skips calling step()
    //                      (e.g. during PAUSE) and later resumes ramps on
    //                      cleanly from wherever the wire actually last was.
    //                      The LAUNCH KICK needs neither this continuity nor
    //                      this value at all -- it jumps directly to its
    //                      target regardless, and (derive_accel_from_rest,
    //                      see LaunchGovernorDecision's doc comment) tells
    //                      the caller to derive its accel from 0, not from
    //                      last_published_v.
    //   spec            - RobotSpec (min_moving_speed/min_sustain_speed/
    //                      launch_margin/max_accel). Read fresh every call;
    //                      this class does not cache it.
    //   dt              - control period. Used to convert spec.max_accel
    //                      into a per-tick rate limit for the STOP-SNAP/
    //                      MOVING-floor ramps (see below); NOT used to limit
    //                      the LAUNCH KICK, which jumps in one tick.
    //   wall_elapsed_in_state - wall-clock seconds since the CURRENT state
    //                      was entered, tracked by the CALLER.
    //   v_raw_fresh     - OPTIONAL (defaults false): true iff THIS tick
    //                      carries a genuinely fresh, trustworthy RAW
    //                      finite-difference localization sample (main.cpp's
    //                      local `v_meas`/`have_dt_meas`, BEFORE the
    //                      caller's own velocity-observer predict/correct
    //                      blending -- see main.cpp's kVelocityObsGain).
    //                      Left at its default by a caller that never wires
    //                      it up (e.g. every EXISTING call site/test before
    //                      this parameter was added): the raw-sample
    //                      debounce below simply never advances, and LAUNCH
    //                      exit falls back to the filtered-estimate check
    //                      alone -- byte-for-byte the PRE-existing behavior.
    //   v_raw           - the raw sample itself, meaningful only when
    //                      v_raw_fresh is true.
    //   time_to_ref_stop - OPTIONAL (defaults to kNoTimeToRefStopHint, a
    //                      sentinel far larger than kFloorYieldHorizonS so
    //                      the guard it feeds never trips): seconds from
    //                      THIS tick's reference-lookup instant to the
    //                      START of the EARLIEST upcoming REFERENCE STOP
    //                      EVENT -- an intermediate hold or the trajectory's
    //                      own absolute end, whichever comes first (see
    //                      extract_ref_stop_events/time_to_next_ref_stop
    //                      below: main.cpp computes the full sorted event
    //                      list ONCE from the received trajectory's
    //                      waypoints at arm time, then looks this value up
    //                      per tick, converting reference_time to the same
    //                      trajectory-relative base get_ref_state_at_time
    //                      already uses). <= 0 for as long as the reference
    //                      is AT OR INSIDE that event's window: positive and
    //                      counting down while still approaching it, <= 0
    //                      from its start through its end, and permanently
    //                      <= 0 once the trajectory's own end has passed --
    //                      exactly matching how this parameter's
    //                      predecessor (time_to_ref_end, absolute-end-only)
    //                      behaved past the last waypoint, since the
    //                      trajectory's end is always the last event in the
    //                      list and is deliberately left unbounded (see
    //                      extract_ref_stop_events). Consumed by the
    //                      MOVING-floor's yield guard (kFloorYieldHorizonS's
    //                      doc comment above has the full "why", for both
    //                      kinds of stop event) AND by CORRECTION-LAUNCH's
    //                      own per-stop-event attempt/cooldown bookkeeping
    //                      (see that paragraph below) -- STOP-SNAP and the
    //                      plain reference-motion LAUNCH-entry are
    //                      unaffected. A caller that never wires this up
    //                      (any pre-existing call site/test) gets
    //                      byte-for-byte the pre-existing MOVING-floor
    //                      behavior, and CORRECTION-LAUNCH's stop-event
    //                      bookkeeping simply never resets (harmless: it
    //                      only ever makes the attempt cap/cooldown
    //                      MORE conservative, never less) -- this
    //                      parameter's rename from time_to_ref_end is a pure
    //                      rename for any caller already passing it
    //                      positionally (every existing call site), not a
    //                      behavior change: a trajectory with no
    //                      intermediate holds resolves to the identical
    //                      value time_to_ref_end always did (see
    //                      extract_ref_stop_events' doc comment for why).
    //   position_error  - OPTIONAL (defaults to 0.0): |current position
    //                      error|, i.e. the distance (meters, always >= 0)
    //                      between the CURRENT interpolated reference pose
    //                      (at reference_time -- the SAME instant ref_vel_now
    //                      was looked up at) and the controller's own real
    //                      pose estimate (main.cpp: ‖ref_now.xy -
    //                      est_state.xy‖). Consumed ONLY by CORRECTION-LAUNCH
    //                      below -- yaw error is deliberately NOT part of
    //                      this (or gated anywhere in this class): closing a
    //                      heading error at rest itself requires motion,
    //                      which the deadband punishes below breakaway, so
    //                      there is nothing this governor can cleanly do
    //                      about yaw alone; position is the error the user
    //                      actually experiences as "the robot didn't finish
    //                      getting there." A caller that never wires this up
    //                      (any pre-existing call site/test) passes the
    //                      default 0.0, which can never exceed a positive
    //                      settle_position_tolerance -- CORRECTION-LAUNCH
    //                      simply never fires, byte-for-byte the
    //                      pre-existing behavior.
    //
    // STATE MACHINE (spec, exact thresholds above):
    //
    //   Active only when spec.min_moving_speed > 0. When it is not, this
    //   class is permanently transparent (state() stays kNormal,
    //   override_active is always false) regardless of any other input --
    //   see G3. RobotSpec::defaults() now compiles with min_moving_speed=0
    //   (an ideal plant, no deadband); a real hardware unit's deployment
    //   activates this class by passing a --robot-spec override (or
    //   equivalent) with a nonzero min_moving_speed -- see RobotSpec.h.
    //
    //   NORMAL -> LAUNCH: |v_meas| < kAtRestFraction*min_moving_speed (at
    //   rest) AND |v_des| > kWantsMotionThreshold (MPC wants motion) AND
    //   |v_des| < min_moving_speed (cannot break away on its own) AND
    //   |ref_vel_now| > kStopSnapRefVelThreshold. Direction is latched to
    //   sign(v_des) for the duration of the LAUNCH sojourn. Checked BEFORE
    //   the STOP-SNAP check below (same priority order the spec lists them
    //   in): a reference that is about to want motion again must launch,
    //   never spuriously snap to a stop -- this ordering is what makes a
    //   reversal crossing (v_des flips sign while at rest) enter LAUNCH with
    //   the new sign instead of being caught by STOP-SNAP.
    //
    //   The |ref_vel_now| guard is an ADDITION beyond the three conditions
    //   the original spec lists (v_meas/v_des only) -- empirically required
    //   (see this task's report) to stop a spurious re-launch during a
    //   trajectory's last few millimeters of final approach: once the
    //   reference has genuinely reached its terminal hold (ref_vel_now truly
    //   ~0, staying there), the MPC can still command a tiny sub-
    //   min_moving_speed v_des purely to close a negligible RESIDUAL
    //   POSITION error (the position-tracking cost term, unrelated to
    //   velocity tracking) -- v_des alone cannot tell "reference about to
    //   want motion" apart from "tiny position-only correction on an
    //   otherwise-finished reference", but ref_vel_now can. A reference that
    //   is genuinely about to want motion (a fresh ramp start, or a
    //   reversal crossing) has |ref_vel_now| already above this SAME
    //   threshold well before v_des exceeds kWantsMotionThreshold, so this
    //   guard does not delay a real launch -- it only suppresses one when
    //   the reference itself has truly settled, correctly deferring to
    //   STOP-SNAP instead.
    //
    //   CORRECTION-LAUNCH entry (a SECOND, independent OR-branch of
    //   NORMAL->LAUNCH, checked alongside the plain reference-motion entry
    //   above -- the two are mutually exclusive by construction, since one
    //   requires |ref_vel_now| > kStopSnapRefVelThreshold and the other
    //   requires the opposite): at rest (the SAME |v_meas| criterion above)
    //   AND the reference is stationary (|ref_vel_now| <=
    //   kStopSnapRefVelThreshold -- i.e. exactly the case the |ref_vel_now|
    //   guard above suppresses) AND position_error > spec.
    //   settle_position_tolerance AND the MPC still wants motion (|v_des| >
    //   kWantsMotionThreshold) AND the cooldown has elapsed since this
    //   governor's last correction-launch exit (spec.settle_cooldown_s,
    //   tracked via this class's own free-running tick clock -- see
    //   step()'s dt parameter and RobotSpec::settle_cooldown_s's doc
    //   comment) AND fewer than spec.settle_max_attempts correction-launch
    //   attempts have already been made for the CURRENT stop event (see
    //   RobotSpec::settle_max_attempts). Deliberately does NOT also require
    //   cannot_break_away (unlike the plain entry above): a correction
    //   launch always kicks to the SAME fixed, conservative target
    //   regardless of how large v_des's own residual-correction command
    //   happens to be -- see the KICK paragraph just below.
    //
    //   On entry: direction_ = sign(v_des) (so a REVERSE-approached target
    //   relaunches backward, exactly like a genuine reversal crossing),
    //   and this sojourn is flagged internally as a correction launch (see
    //   the KICK/timeout/bookkeeping differences below) -- entry itself
    //   publishes the IDENTICAL single-tick KICK the plain entry above
    //   does (v_published = direction_ * min_moving_speed*(1+launch_margin),
    //   derive_accel_from_rest=true).
    //
    //   While a CORRECTION-LAUNCH sojourn continues in LAUNCH: UNLIKE a
    //   plain launch, escalation is SKIPPED entirely -- every tick
    //   republishes the SAME fixed base target (never ratcheted -- see
    //   RobotSpec::settle_max_attempts's doc comment: this is one bounded
    //   positioning nudge, not a load-robustness climb). If breakaway is
    //   not confirmed (by either LAUNCH->NORMAL exit trigger below) within
    //   kCorrectionLaunchTimeoutS of entering LAUNCH, this sojourn GIVES UP:
    //   it exits to NORMAL that same tick (transparent, override_active=
    //   false, exactly like a confirmed exit), counts one attempt against
    //   settle_max_attempts, starts the cooldown, and prints one stderr
    //   warning -- it never silently keeps trying forever, and never
    //   escalates speed to force the issue.
    //
    //   Whenever a CORRECTION-LAUNCH sojourn exits LAUNCH (confirmed
    //   breakaway OR the give-up timeout above): one attempt is counted
    //   against spec.settle_max_attempts for the CURRENT stop event, and
    //   the cooldown clock restarts from that exit -- both whether or not
    //   the attempt actually succeeded, since "succeeded" here just means
    //   "broke away," not "fully closed position_error" (a single kick can
    //   overshoot, undershoot, or need a second nudge; the NEXT tick's own
    //   position_error naturally decides whether another attempt is
    //   warranted, gated by the same cooldown/cap).
    //
    //   Per-stop-event bookkeeping (attempts count AND the cooldown
    //   baseline) resets whenever time_to_ref_stop transitions from >0
    //   (still approaching) to <=0 (a NEW stop event has begun) -- see that
    //   parameter's own doc comment above. A caller that never wires
    //   time_to_ref_stop up (any pre-existing call site/test) never
    //   triggers this reset, which only ever makes the cap/cooldown MORE
    //   conservative across what this class would otherwise treat as
    //   distinct stop events, never less. reset() (see below) also clears
    //   all of this -- required on a new trajectory arm and on RESUME, same
    //   as every other piece of LAUNCH/escalation state.
    //
    //   If CORRECTION-LAUNCH's other four conditions hold (at rest,
    //   reference stationary, position_error over tolerance, MPC wants
    //   motion) but the attempt cap is what's blocking entry, ONE stderr
    //   warning (reporting the residual position_error) is printed the
    //   first tick this is detected for the current stop event -- not
    //   every tick -- and this class simply does NOT enter LAUNCH: control
    //   falls through to STOP-SNAP/transparent below exactly as it would
    //   for any other NORMAL tick, i.e. "stay parked," never a forced hard
    //   zero from this class itself. A cooldown that has not yet elapsed
    //   blocks entry the same way but silently (no warning; it will retry
    //   once the cooldown passes, which is the expected/common case, not a
    //   giving-up condition worth a warning).
    //
    //   While LAUNCH: publishes a SUSTAINED KICK at the escalated launch
    //   target, no ramp, EVERY tick spent in LAUNCH (not just entry) --
    //   THIS PARAGRAPH DESCRIBES THE PLAIN (reference-motion) LAUNCH ONLY;
    //   see the CORRECTION-LAUNCH paragraph above for how a correction
    //   sojourn instead stays flat at the base target with no escalation:
    //       target = direction * (escalated launch speed)   [see below]
    //       v_published = target
    //       derive_accel_from_rest = true
    //   The escalated launch speed magnitude itself is computed exactly as
    //   before: base speed is min_moving_speed * (1 + launch_margin); every
    //   kEscalationIntervalS (0.5s) of continuous wall_elapsed_in_state
    //   without exiting, it is multiplied by kEscalationFactor (1.1),
    //   compounding, capped at kEscalationCapFactor (1.5) * the base launch
    //   speed. v_published is NOT rate-limited toward that target -- it
    //   jumps there directly every call -- AND, critically,
    //   derive_accel_from_rest=true tells the caller to derive accel as
    //   (v_published - 0)/dt, NOT (v_published - last_published_v)/dt (see
    //   LaunchGovernorDecision's doc comment for the full "why": anchoring
    //   to last_published_v would derive accel=0 on every tick after the
    //   target first becomes reachable, since by then last_published_v
    //   already equals it too -- but the sim only ever integrates the
    //   wire's "accel" field, so a single tick's impulse not being quite
    //   enough would strand the plant with no further push, forever). So
    //   EVERY LAUNCH tick republishes the FULL implied accel (typically
    //   large -- e.g. ~2.2 m/s^2 for a 0.11 m/s base target at dt=0.05s,
    //   deliberately larger than spec.max_accel; see
    //   LaunchGovernorDecision's doc comment), a SUSTAINED, REPEATED push
    //   for as long as LAUNCH persists, not a single one-shot attempt. This
    //   replaces an earlier RATE-LIMITED RAMP version of this same
    //   transition (bounded by spec.max_accel*dt/tick, ~0.2s to reach
    //   breakaway) -- see this class's own file-header doc comment for why:
    //   that ramp was compensating for a sim plant that under-modeled a
    //   real ESC's launch-current headroom, not protecting the actuator
    //   from a genuine limit. steering is never this class's concern (the
    //   caller always keeps using the MPC's own solved delta).
    //
    //   Warning cadence (unchanged): one stderr warning is printed per
    //   escalation step WHILE still climbing; once capped, per-step warnings
    //   stop (the TARGET is no longer changing) and are replaced by a single
    //   warning the instant the cap is first hit, then at most once every
    //   kCapWarnIntervalS (1.0s) thereafter for as long as LAUNCH persists --
    //   this class never silently gives up trying.
    //
    //   LAUNCH -> NORMAL: EITHER of two independent CONFIRMED-BREAKAWAY
    //   triggers (a fresh raw-sample debounce for speed, or the pre-existing
    //   filtered-estimate check as an always-on fallback -- see
    //   kRawExitDebounceCount's doc comment above for why a fast path was
    //   added on top of, not instead of, the fallback) -- PLUS, for a
    //   CORRECTION-LAUNCH sojourn specifically, a THIRD, independent
    //   give-up-timeout trigger (kCorrectionLaunchTimeoutS elapsed with
    //   neither of the two below having fired -- see the CORRECTION-LAUNCH
    //   paragraph above for its own attempt/cooldown bookkeeping, which
    //   applies on this exit exactly as it does on a confirmed one):
    //     (a) FAST: kRawExitDebounceCount (2) consecutive FRESH raw samples
    //         (v_raw_fresh=true calls) each with |v_raw| >=
    //         kExitFraction*min_moving_speed. Only fresh-sample calls
    //         advance or reset this streak; calls with v_raw_fresh=false
    //         leave it untouched (so a control loop running faster than the
    //         localization publish rate does not spuriously reset progress
    //         on the ticks between fresh samples).
    //     (b) FALLBACK: |v_meas| >= kExitFraction*min_moving_speed (the
    //         caller's filtered estimate) -- exactly the pre-existing check,
    //         unchanged, so a caller that never passes v_raw_fresh=true (any
    //         pre-existing call site) still exits exactly as before.
    //   Either firing sets just_exited_launch=true on the decision (see
    //   LaunchGovernorDecision's doc comment) -- a caller with its own
    //   velocity observer (main.cpp) uses this to one-shot inject its
    //   latest raw sample into that observer's state, since stall
    //   transitions are discrete events the observer's own low-gain
    //   correction would otherwise take several samples to believe; this
    //   class has no observer state of its own to touch. Nothing else needs
    //   to happen on this transition -- the MPC already has true velocity
    //   feedback, so its very next solve (seeded from the now-nonzero
    //   v_meas) naturally takes over, including correcting any breakaway
    //   overshoot; this class does not need to hand off any other state.
    //
    //   STOP-SNAP (checked in NORMAL, after the LAUNCH-entry check above,
    //   BEFORE the MOVING-floor check below -- both STOP-SNAP and reversal
    //   [handled by the LAUNCH-entry check above it] take precedence over
    //   the floor):
    //   |v_des| < min_sustain_speed AND |ref_vel_now| < kStopSnapRefVelThreshold
    //   (which a completed or holding trajectory satisfies naturally, since
    //   get_ref_state_at_time clamps to/returns a zero-ref_vel waypoint in
    //   both cases -- no separate "trajectory complete" plumbing is needed
    //   here) -> ramp toward 0.0 via rate_limited_toward() (max_accel*dt per
    //   tick, anchored to last_published_v), NOT an instant jump. This is a
    //   deliberate change from an earlier "publish exactly 0.0 in one tick"
    //   version: an instant jump lets the CALLER's own bookkeeping
    //   (last_published_v) claim to have reached 0.0 faster than the
    //   plant's actuator actually could, permanently stranding an un-shed
    //   residual on the plant's true (accel-clamped) state whenever the
    //   needed correction exceeded one tick's max_accel*dt -- empirically
    //   observed (see this task's report) to silently defeat a LATER
    //   reversal LAUNCH, whose escalation cap assumes it only has to reach a
    //   fresh target from last_published_v, not ALSO cancel out a stranded
    //   residual the plant was never actually brought back to 0 from. The
    //   ramp still reaches EXACTLY 0.0 (snapping once within max_accel*dt of
    //   it), just not necessarily on the very first tick.
    //
    //   MOVING-floor (ANTI-RE-STALL, checked in NORMAL, after BOTH the
    //   LAUNCH-entry and STOP-SNAP checks above -- so a reversal or a
    //   genuine stop always takes precedence): fires when ALL of --
    //     - the plant is believed moving: |v_meas| >=
    //       kExitFraction*min_moving_speed (the SAME bar LAUNCH uses to
    //       confirm breakaway -- deliberately conservative: NOT just "not at
    //       rest", so this never fires in the ambiguous band between
    //       kAtRestFraction and kExitFraction, which is neither confirmed
    //       stalled nor confirmed cruising);
    //     - the reference wants motion: |ref_vel_now| >=
    //       kMovingFloorRefVelThreshold;
    //     - the MPC's own desired speed is dangerously low: |v_des| < floor,
    //       where floor = min_sustain_speed + 0.3*(min_moving_speed -
    //       min_sustain_speed) + 0.02 -- i.e. comfortably inside the
    //       hysteresis band above min_sustain_speed, leaving margin before a
    //       noise-perturbed v_cmd_equiv could dip the SIM's deadband model
    //       back into a stall (see SimCore.h's step_deadband) purely from
    //       actuation noise riding on an otherwise-adequate v_des.
    //   -> publishes floor*sign(ref_vel_now) (ref_vel_now, not v_des: by
    //   construction |ref_vel_now| >= kMovingFloorRefVelThreshold is always
    //   comfortably nonzero here, while the whole point of this branch is
    //   that v_des's own magnitude -- and so, potentially, a noisy sign near
    //   zero -- cannot be trusted) via the SAME rate_limited_toward()
    //   STOP-SNAP uses (anchored to last_published_v) -- never commands INTO
    //   the dead zone while motion is wanted. Purely a NORMAL-state, cruise
    //   time behavior: it never fires while LAUNCH is active or during a
    //   genuine STOP-SNAP/reversal (both checked first, above).
    //
    //   A FOURTH condition, checked last (cheapest to skip when far from any
    //   stop event): the floor YIELDS -- does not fire, falls through to
    //   transparent -- whenever time_to_ref_stop <= kFloorYieldHorizonS. See
    //   that constant's own doc comment above for the full rationale (in
    //   short: this close to a reference STOP EVENT -- an intermediate hold
    //   OR the trajectory's own end, neither of which decelerates on its
    //   own -- a low v_des is the position cost recognizing the stop is
    //   close, not noise to guard against, and forcing the floor through
    //   this window was observed to carry the plant straight through the
    //   stop at floor speed with no deceleration authority at all).
    //
    //   Otherwise (in NORMAL, none of LAUNCH-entry/STOP-SNAP/MOVING-floor
    //   fire): transparent -- override_active=false.
    //
    // INTERLOCKS (enforced by the CALLER, not this class): the PAUSE brake
    // path and the shutdown stop-burst already publish their own stop
    // sequences and must bypass this class entirely (never call step()
    // during either); hw-calibration scaling applies on top of whatever
    // v_published this returns exactly as it already does for ordinary MPC
    // speeds (build_payload's vel_scale/vel_scale_back does not
    // distinguish). Call reset() on a new trajectory arming and on RESUME
    // (mirroring MpcSolver::reset_warm_start()'s own reset sites exactly --
    // a stale escalation level or a stale LAUNCH sojourn latched from before
    // a brake-to-zero pause must never survive across it).
    LaunchGovernorDecision step(double v_meas, double v_des, double ref_vel_now,
                                 double last_published_v, const RobotSpec& spec, double dt,
                                 double wall_elapsed_in_state, bool v_raw_fresh = false,
                                 double v_raw = 0.0,
                                 double time_to_ref_stop = kNoTimeToRefStopHint,
                                 double position_error = 0.0);

    // Resets to kNormal and clears all escalation bookkeeping (including the
    // raw-exit debounce streak) AND all CORRECTION-LAUNCH bookkeeping (the
    // free-running tick clock, the last-exit cooldown baseline, the
    // per-stop-event attempt count, and its warn-once flag). See step()'s
    // INTERLOCKS paragraph for the required call sites.
    void reset();

    LaunchGovernorState state() const { return state_; }

    // Number of CORRECTION-LAUNCH attempts (confirmed-breakaway or
    // given-up-on alike) counted so far against spec.settle_max_attempts for
    // the CURRENT stop event -- see step()'s CORRECTION-LAUNCH paragraph.
    // Exposed so a caller (main.cpp's position-aware trajectory-completion
    // check) can tell "still trying" apart from "gave up" without
    // duplicating this class's own bookkeeping.
    int settle_attempts_this_stop_event() const { return settle_attempts_this_stop_event_; }

   private:
    LaunchGovernorState state_ = LaunchGovernorState::kNormal;
    double direction_ = 1.0;          // +-1.0; meaningful only while state_==kLaunch.
    int escalation_step_ = 0;         // floor(wall_elapsed_in_state / kEscalationIntervalS) last observed.
    bool capped_reported_ = false;    // true once the cap has been hit at least once this LAUNCH sojourn.
    double last_cap_warn_elapsed_ = 0.0;  // wall_elapsed_in_state at the last cap-cadence warning.
    int raw_exit_streak_ = 0;  // consecutive fresh raw samples meeting kExitFraction; see kRawExitDebounceCount.

    // CORRECTION-LAUNCH bookkeeping (see step()'s CORRECTION-LAUNCH
    // paragraph for the full contract).
    bool in_correction_launch_ = false;  // true only while state_==kLaunch AND this sojourn is a correction launch.
    // Free-running clock, advanced by dt on every step() call while the
    // governor is active (spec.min_moving_speed > 0) -- deliberately NOT
    // std::chrono (this class stays wall-clock-agnostic per its own file
    // header doc comment); just an accumulated sum of the same dt the
    // caller already passes every non-paused tick, giving this class a
    // self-contained notion of elapsed time for the cooldown check below
    // without needing a new "wall clock" parameter from the caller.
    double self_elapsed_s_ = 0.0;
    // self_elapsed_s_ at the last CORRECTION-LAUNCH exit (confirmed or
    // given-up); negative means "no correction-launch exit yet" (for the
    // CURRENT stop event -- reset alongside the attempt count below), which
    // trivially satisfies the cooldown check on a governor's/stop event's
    // very first correction-launch attempt.
    double last_correction_exit_elapsed_ = -1.0;
    int settle_attempts_this_stop_event_ = 0;
    bool settle_cap_warned_this_stop_event_ = false;  // rate-limits the cap-exhausted warning to once per stop event.
    // Previous tick's time_to_ref_stop, used to detect the >0 -> <=0
    // transition that marks a NEW stop event beginning (see
    // RobotSpec::settle_max_attempts's doc comment) -- initialized to the
    // sentinel (always > 0) so the very first stop event's own transition
    // is detected like any other, and so a caller that never wires
    // time_to_ref_stop up never spuriously transitions on its own.
    double prev_time_to_ref_stop_ = kNoTimeToRefStopHint;
};

// REFERENCE STOP EVENTS -- free functions (not LaunchGovernor methods), see
// kFloorYieldHorizonS's doc comment above for the concept and step()'s
// time_to_ref_stop parameter for how they plug into the state machine.
//
// LAYERING NOTE: this concept -- scanning a ReloPush::trajectory's waypoints
// -- belongs to main.cpp (the only place that type is otherwise touched in
// this class's neighborhood; LaunchGovernor itself stays decoupled from
// trajectory/MpcCore internals by design, per this file's own header doc
// comment). These two functions live here anyway, purely as a build-graph/
// testability choice: LaunchGovernor.cpp is already compiled into every
// target that needs this (mpc_controller AND mpc_unit_tests -- see
// MPC/CMakeLists.txt's MPC_CORE_SOURCES), so this is the one
// place a pure helper like this can be exercised directly by
// mpc_unit_tests.cpp without a build-file change. Accordingly, BOTH
// functions below are deliberately trajectory-agnostic -- they consume and
// return plain time/velocity doubles, never ReloPush::trajectory_elem --
// main.cpp's own arm-time code owns the (small) job of deriving those plain
// arrays from the real waypoints it received (see that file for why it
// derives an EFFECTIVE per-segment velocity from waypoint (x,y) motion
// rather than passing the raw trajectory_elem::ref_vel label straight
// through).
struct RefStopEvent {
    double start_time = 0.0;
    double end_time = 0.0;
};

// Pure, generic hold-run scanner. `times` (size N, ascending -- as
// trajectory waypoint times always are) and `ref_vels` (size N) are
// parallel arrays where ref_vels[i] is the EFFECTIVE reference velocity of
// the SEGMENT from times[i] to times[i+1]; index N-1's entry is never read
// (there is no outgoing segment from the last sample). A segment whose
// duration (times[i+1]-times[i]) is <= 1ms is skipped entirely -- neither
// breaking nor extending a run -- since a segment this short cannot
// represent real elapsed motion or its absence (e.g. an exact-timestamp
// duplicate waypoint pair some upstream trajectory encoders emit at a
// direction-reversal/pickup instant: one entry carries the incoming leg's
// label, the next the outgoing leg's, both stamped with the same source
// time). A HOLD is a maximal run of consecutive (non-skipped) segments with
// |ref_vels[i]| < vel_threshold whose total time span is >= min_hold_s;
// each qualifying run contributes one RefStopEvent{run_start_time,
// run_end_time}. Two holds separated by nothing but skipped segments merge
// into a single event (skipped segments are transparent to the run, per
// above).
//
// The trajectory's own end always contributes exactly one additional final
// RefStopEvent{times.back(), <a sentinel far in the future>} appended last
// -- even when N < 2 (no segments to scan at all) -- deliberately UNBOUNDED
// so time_to_next_ref_stop() below stays <= 0 for the rest of the process's
// life once the reference is exhausted, exactly mirroring how the
// pre-hold-awareness time_to_ref_end computation stayed negative forever
// past the trajectory's last waypoint. A trailing hold whose own run
// reaches times.back() is NOT specially merged with this final event --
// both individually resolve to a <= 0 time_to_next_ref_stop for any
// ref_time inside either one, so leaving them as two (overlapping-at-one-
// point) entries is harmless; see time_to_next_ref_stop's own doc comment.
//
// Returns a single-element vector (just the unbounded end event, at
// times.back()) when N==1 (no segments possible), and an EMPTY vector only
// when times is itself empty (nothing to report at all -- unreachable in
// main.cpp's own use, which never calls this on an empty trajectory; see
// that file).
std::vector<RefStopEvent> extract_ref_stop_events(
    const std::vector<double>& times, const std::vector<double>& ref_vels,
    double vel_threshold = LaunchGovernor::kStopSnapRefVelThreshold,
    double min_hold_s = LaunchGovernor::kRefStopMinHoldS);

// Per-tick lookup against an `events` list already sorted and end_time-
// ordered exactly as extract_ref_stop_events returns (a caller-supplied
// list built any other way must uphold the same ordering for the binary
// search below to be valid). Returns (the start_time of the EARLIEST event
// whose end_time >= ref_time) - ref_time: positive and counting down while
// still approaching that event, <= 0 for as long as ref_time falls inside
// it (start_time <= ref_time <= end_time) -- see
// LaunchGovernor::step()'s time_to_ref_stop parameter doc for how this
// feeds the MOVING-floor's yield guard. O(log events.size()) (a binary
// search on end_time), safe to call every control tick against a list that
// itself was only ever built once, at trajectory-arm time. Returns
// LaunchGovernor::kNoTimeToRefStopHint for an empty `events` (never
// produced by extract_ref_stop_events itself for a non-empty `times` --
// only reachable if a caller passes its own empty list, or an empty
// trajectory that never reaches this call at all -- see main.cpp).
double time_to_next_ref_stop(const std::vector<RefStopEvent>& events, double ref_time);

}  // namespace mpc
