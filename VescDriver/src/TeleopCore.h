// TeleopCore.h
//
// Pure interactive-teleop state machine for vesc_teleop. NO I/O of any
// kind lives here -- no serial, no terminal, no clock reads -- exactly
// like DriverCore.h's own contract: the caller passes every keypress byte
// and the current monotonic time explicitly into handle_key()/step()/
// feed_telemetry(), and reads back what motor command (if any) to send
// this tick. Fully unit-testable with synthetic time, no real sleeps or
// hardware required.
//
// CONTEXT (see vesc_teleop_main.cpp / README.md's "Teleoperation" section
// for the full operator-facing picture): this tool is the FIRST, most
// cautious contact with a brand-new, unknown-firmware VESC driving a bare
// motor with no load -- the previous unit was destroyed by cumulative
// high-current stall damage. Every path through this state machine is
// built around "when in doubt, brake and require an explicit human
// action to resume," not around maximizing convenience.
//
// ---------------------------------------------------------------------
// STATE MACHINE
// ---------------------------------------------------------------------
//   kIdle     -- no drive commanded; step() returns TeleopMotorAction::kNone
//                (the caller sends an ALIVE keepalive for this).
//   kDriving  -- direction() != kStopped; step() returns a clamped
//                duty/erpm command every tick, UNLESS the deadman has
//                expired (see below), in which case THAT SAME step() call
//                falls through to kBraking instead.
//   kBraking  -- a brake_hold_ms-long window (entered either by an
//                explicit stop key or by a deadman expiry) during which
//                step() returns TeleopMotorAction::kBrake every tick;
//                once now_s reaches brake_started_s_+brake_hold_ms,
//                transitions to kIdle on that tick (returning kNone).
//   kAborted  -- entered ONLY via feed_telemetry() (over-current or a
//                nonzero fault code). step() returns
//                TeleopMotorAction::kBrake every tick, LATCHED, until the
//                operator presses 'c' (handle_key('c', ...)) to clear it.
//                'f'/'b'/'r' are IGNORED while aborted -- driving cannot
//                resume until explicitly cleared.
//
// DEADMAN: ANY call to handle_key() (recognized key or not -- matching
// the operator-facing promise "auto-stops after N seconds without ANY
// keypress") refreshes an internal last-key-time clock. While kDriving,
// step() checks `now_s - last_key_time_s_ >= deadman_ms/1000`; once true
// it transitions to kBraking (stop_reason()=="deadman") on that same
// step() call (state-transition boundaries in this class are always
// "fires at exactly the boundary", i.e. >=, not strict >).
//
// SAFETY WATCH: feed_telemetry() must be called with every fresh
// GET_VALUES reply. It transitions (once, latched) to kAborted if
// |current_motor| exceeds current_abort OR the reported fault code is
// nonzero -- a bare motor under test should draw far less current than
// current_abort, so tripping this is a strong signal something is wrong
// with the new hardware/firmware, not normal load.
//
// RAMP MODE (OFF by default -- see TeleopConfig::ramp_enabled): while
// kDriving, instead of instantly commanding target=direction*magnitude(),
// the emitted command slews toward that target by at most the current
// mode's ramp rate (duty_ramp or erpm_ramp, units/second) per tick, using
// dt derived from consecutive step() calls' own now_s (the SAME "first
// call has dt==0" warm-up convention DriverCore.h uses -- see its own
// header). The slewed value ("emitted value") persists across ticks
// while actively driving -- a magnitude/direction change just moves the
// target the slew is chasing (so a direction reversal slews smoothly
// THROUGH zero rather than jumping) -- but resets to 0 every time driving
// restarts from a non-driving state, so it never carries over a stale
// value from a previous, unrelated drive. Switching mode (d/e) while
// driving stops first (exactly like the stop key, reason "mode_switch")
// rather than slewing the emitted value across incompatible units
// (duty fraction vs. erpm). With ramp_enabled==false, the emitted value
// is simply set to the target every tick -- i.e. today's original
// instant behavior, unchanged bit-for-bit.
//
// SAFETY PATHS BYPASS THE RAMP ENTIRELY: the stop key, deadman expiry,
// and the current/fault abort latch all brake IMMEDIATELY, exactly as
// without ramp mode -- none of those paths ever compute or return a
// slewed value; they short-circuit to a full kBrake action before the
// ramp logic (which only lives inside the kDriving branch) is ever
// reached.
//
// SPEED GOVERNOR MODE (TeleopMode::kSpeed, key 'v'): a THIRD mode, added
// after real-hardware testing found FW 2.18's own native ERPM mode
// dangerous -- its speed PID is dead below roughly 900 erpm setpoint,
// then engages violently once past it (a real 33A current spike/abort).
// Duty mode alone drives smoothly but requires the operator to
// compensate for battery voltage sag by hand. Speed mode is a duty-
// actuated, erpm-feedback governor: the operator sets a TARGET ERPM
// (magnitude() in this mode) but the wire only EVER carries SET_DUTY.
// The actual PI-with-feedforward control loop lives in SpeedGovernor.h
// (extracted so DriverCore's own "actuation" backend -- see
// DriverCore.h -- can reuse the identical logic); this class owns one
// SpeedGovernor instance (governor_) and is just the thin key/mode
// wiring around it:
//   - Feed erpm/v_in into the governor via feed_telemetry() (as always);
//     the raw erpm is fed through SpeedGovernor::feed_erpm(), which
//     lightly low-pass filters it (EMA, time-constant based) before use,
//     since FW 2.18's own erpm estimate is noisy (roughly +-150).
//   - error = target_erpm - filtered_erpm. duty_ff = target_erpm /
//     (speed_ff_gain * max(v_in, 6.0)) (0 disables the feedforward term
//     -- pure PI). Incremental PI: the integrator only accumulates when
//     the resulting duty command is NOT saturated at +-max_duty
//     (anti-windup -- freezes instead of winding up unboundedly while
//     the target is temporarily unreachable). See SpeedGovernor.h for
//     the exact formula.
//   - The governor's duty output is ALWAYS slew-limited by duty_ramp
//     (SpeedGovernor's own duty_slew_per_s, mapped 1:1 from
//     TeleopConfig::duty_ramp), REGARDLESS of whether ramp_enabled() is
//     on -- this is the governor's own built-in gentleness guarantee,
//     not an opt-in. Speed mode's own 'A'-armed ramp-rate entry (see
//     below) also targets duty_ramp, since that is what it actually
//     governs.
//   - Direction ('f'/'b') flips the sign of the target exactly like
//     duty/erpm mode; a reversal slews the output duty smoothly through
//     zero via the same mechanism as ramp mode's reversal.
//   - Governor state (the integrator and the erpm filter) is fully
//     RESET on any stop (key or deadman), any current/fault abort, and
//     any mode switch (d/e/v) -- never silently carried over into an
//     unrelated future drive.
//   - Voltage independence is the whole point: because the loop is
//     closed on measured erpm (not an open-loop duty guess), the
//     commanded duty automatically adjusts as the battery sags under
//     load, keeping the actual speed at the operator's requested target
//     rather than drooping with it.
// Safety paths bypass this exactly like ramp mode: stop/deadman/abort
// all still brake immediately, never a gradual governor wind-down.
//
// STEERING SUB-STATE-MACHINE (keys 'j'/'l'/'k'/'R'/'T'/'W'): an
// INDEPENDENT position sub-state-machine for the steering servo -- a
// physically separate axis from everything above, driven by its own
// keys, active identically whether idle/driving/braking/aborted.
// steering_position() always holds a valid, defined value (initialized
// to config_.steer_center, clamped into [steer_min_pos, steer_max_pos])
// -- what varies is only whether the operator has actually touched it
// yet (see "NO PREMATURE EMISSION" below) and whether TRIM MODE is on.
//   'j' / 'l'  -- step the position left/right by steer_coarse_step (or
//                 steer_fine_step while in_trim_mode()), clamped to
//                 [steer_min_pos, steer_max_pos]. steer_invert flips
//                 which key increases vs. decreases the raw position.
//   'k' / 'R'  -- snap to config_.steer_center (clamped). Full synonyms
//                 of each other, deliberately available in ANY
//                 mode/state -- see "DELIBERATE SIMPLIFICATION" below.
//   'T'        -- toggle TRIM MODE on/off. Entering trim mode stops the
//                 drive first (exactly like 'd'/'e'/'v': begin_brake()
//                 with reason "trim_mode") if currently driving, but
//                 never touches steering_position_ itself. Exiting trim
//                 mode takes no drive action -- 'j'/'l' step size just
//                 reverts to coarse (it's computed live from
//                 in_trim_mode(), never stored separately). See "TRIM
//                 MODE SAFETY DECISION" below.
//   'W'        -- raises a PENDING "save steering_position()'s CURRENT
//                 value as the new center" request for main to act on
//                 (this class does no file I/O of its own) -- see
//                 has_pending_center_save()/pending_center_save_value()/
//                 consume_pending_center_save() below. Pressing 'W'
//                 again before a prior pending request is consumed just
//                 overwrites the pending value -- still exactly ONE
//                 pending request outstanding, never a queue. Always
//                 available (harmless outside trim mode, same
//                 rationale as 'k'/'R' below).
//
// TRIM MODE SAFETY DECISION (a deliberate extension beyond the literal
// feature request -- flagged here like this file's other safety calls):
// while in_trim_mode(), 'f'/'b'/'r' are IGNORED, exactly like the
// existing "ignored while aborted" treatment for those same three keys
// -- driving cannot start or resume until 'T' exits trim mode. Trim
// mode exists so the operator can tune steering with the drivetrain
// guaranteed inert; allowing a drive command mid-trim would undermine
// that guarantee. Every other key (mode/step/digit/ramp/'c') remains
// unaffected, same precedent as the abort-key-filtering above.
//
// DELIBERATE SIMPLIFICATION: 'R' could instead be gated to work only
// while in_trim_mode() (matching the mental model "R is a trim-mode
// action"), but this implementation makes it an always-available alias
// for 'k' -- gating the SAME visible effect ("snap to center") to
// sometimes work and sometimes silently do nothing, depending on a mode
// the operator may not be tracking, would be a worse surprise than
// simply letting it always work.
//
// STEERING SAFETY INDEPENDENCE -- CRITICAL: steering_position_,
// steering_touched_, and steering_changed_since_emit_ are NEVER touched
// by the stop key, deadman expiry, the current/fault abort latch, or
// feed_telemetry()'s abort trigger -- those paths affect ONLY the drive
// state (duty/erpm/direction/braking), exactly as before steering
// existed. Rationale: steering is not itself a runaway hazard the way
// an unbounded motor command is, and silently re-centering the servo
// the instant something ELSE goes wrong could leave the wheels
// somewhere worse than wherever the operator last, deliberately, put
// them. Steering keys DO still refresh the deadman clock -- for free,
// via handle_key()'s existing top-of-function "refresh
// last_key_time_s_ for ANY key" behavior, since every new steering
// branch below lives inside that same function, after that line (never
// a bypass path).
//
// NO PREMATURE EMISSION: this class has no I/O, so it cannot itself
// refuse to send a servo command -- but its accessor set is shaped so
// main's own policy can simply be "never send SET_SERVO_POS until
// steering_ever_touched() is true". steering_ever_touched() latches
// true (forever, never reset) the first time 'j'/'l'/'k'/'R' actually
// moves (or, for k/R, re-affirms) steering_position_ -- 'T' alone
// (entering/exiting trim mode without any position-affecting key) does
// NOT set it, since the operator never actually chose a position.
//
// Portability: C++14 only -- see VescDriver/CMakeLists.txt's HARD
// PORTABILITY RULES. Only depends on VescProtocol.h (for VescValues, fed
// in by feed_telemetry()). Deliberately does NOT include SteeringCalib.h
// -- main.cpp bridges the two by copying a loaded SteeringCalib's fields
// into TeleopConfig's flat steer_* fields below, keeping this class
// fully independent/unit-testable.

#ifndef VESC_DRIVER_TELEOP_CORE_H_
#define VESC_DRIVER_TELEOP_CORE_H_

#include "SpeedGovernor.h"
#include "VescProtocol.h"

#include <cstdint>
#include <string>

namespace vesc {

enum class TeleopMode { kDuty, kErpm, kSpeed };

// -1 (reverse), 0 (stopped/not driving), +1 (forward).
enum class DriveDirection { kReverse = -1, kStopped = 0, kForward = 1 };

struct TeleopConfig {
    double duty_mag_default = 0.04;
    double erpm_mag_default = 1000.0;
    double duty_step = 0.005;
    double erpm_step = 100.0;
    double max_duty = 0.2;
    double max_erpm = 6000.0;
    double deadman_ms = 2000.0;
    double brake_hold_ms = 300.0;
    double current_abort = 8.0;
    // SET_CURRENT_BRAKE magnitude (amps) used for every brake this class
    // commands (stop key, deadman, abort) -- deliberately a single fixed,
    // conservative value rather than derived from current_abort, so a
    // low current_abort (tightened for a fragile new unit) never
    // accidentally weakens the brake itself.
    double brake_amps = 3.0;

    // Ramp mode (see the class header's "RAMP MODE" section): per-mode
    // slew rates (units/second) and whether ramping starts enabled.
    // OFF by default so existing callers/tests see byte-for-byte
    // unchanged instant behavior unless they opt in.
    double duty_ramp = 0.1;
    double erpm_ramp = 500.0;
    bool ramp_enabled = false;

    // Speed governor mode (see the class header's "SPEED GOVERNOR MODE"
    // section): duty-actuated, erpm-feedback closed loop. kp/ki are
    // conservative starting points from a real robot's own duty/v_in/
    // erpm log, not tuned in general; speed_ff_gain (erpm per duty*volt)
    // is from that same log -- 0 disables the feedforward term (pure PI).
    double speed_mag_default = 1000.0;  // target erpm.
    double speed_kp = 2e-6;             // duty per erpm of (proportional) error.
    double speed_ki = 1e-5;             // duty per erpm-second of (integral) error.
    double speed_ff_gain = 4400.0;
    // EMA time constant (seconds) for the governor's erpm input filter --
    // see SpeedGovernor.h's feed_erpm() doc comment (alpha = 1-exp(-dt/tau)).
    // New field (Driver v2's SpeedGovernor extraction): the ORIGINAL
    // governor used a fixed per-SAMPLE alpha=0.3 regardless of elapsed
    // time; this replaces it with a time-constant parameterization, which
    // shifts exact filtered-erpm values at a given dt (see
    // SpeedGovernor.h's own header for why this is deliberate).
    double erpm_filter_tau_s = 0.1;

    // Steering sub-state-machine (see the class header's "STEERING
    // SUB-STATE-MACHINE" section). steer_center matches SteeringCalib's
    // own default (VescDriver/src/SteeringCalib.h, a sibling task) --
    // main.cpp copies a loaded SteeringCalib's fields into these flat
    // fields; TeleopCore itself does NOT include or depend on that
    // header.
    double steer_center = 0.5;
    double steer_min_pos = 0.15;
    double steer_max_pos = 0.85;
    bool steer_invert = false;

    // VEHICLE DRIVE-DIRECTION SIGN (shared with vesc_driver via
    // SteeringCalib's drive_invert). True when a POSITIVE motor command
    // physically drives the vehicle BACKWARD (a property of this
    // vehicle's motor wiring, confirmed on the real robot). Everything
    // inside TeleopCore -- target_value(), emitted_value_, the speed
    // governor's error/integrator -- works in VEHICLE frame ("+ is
    // forward"); the sign is applied ONLY where the frame actually
    // changes: on the emitted MotorAction value (vehicle -> motor) and
    // on the measured erpm fed into the governor (motor -> vehicle).
    // Inverting one without the other would flip the governor's feedback
    // polarity and make the loop diverge, so both live in drive_sign().
    bool drive_invert = false;
    double steer_coarse_step = 0.01;
    double steer_fine_step = 0.002;
};

// Deliberately named TeleopMotorAction, NOT MotorAction: DriverCore.h
// already defines `vesc::MotorAction` with a different Type enum
// (kNone/kRpm/kDuty/kCurrent/kBrake vs. this one's kNone/kDuty/kRpm/
// kBrake) -- vesc_driver_tests.cpp includes both headers in the same
// translation unit, so reusing the name would be a hard redefinition
// error.
struct TeleopMotorAction {
    enum class Type { kNone, kDuty, kRpm, kBrake };
    Type type = Type::kNone;
    double value = 0.0;  // units depend on type: duty fraction, erpm, or brake amps.
};

enum class KeyEventType { kNone, kQuit };

struct KeyEvent {
    KeyEventType type = KeyEventType::kNone;
    // Non-empty when a RETIRED key was pressed (e.g. the pre-WASD 'f'):
    // a one-line hint for the caller to print, naming the replacement.
    // The key itself is inert -- it performs no drive/steer action.
    std::string retired_hint;
};

// Maps a VESC fault code (VescValues::fault) to its short name (e.g.
// "ABS_OVER_CURRENT"), or "" for any code this table doesn't recognize --
// callers print the raw numeric code either way and append "(name)" only
// when non-empty.
std::string fault_name(uint8_t fault_code);

class TeleopCore {
public:
    explicit TeleopCore(TeleopConfig config = TeleopConfig());

    // Feeds one keypress byte. now_s is THIS call's monotonic time
    // (caller-supplied, exactly like DriverCore::tick()'s own contract).
    // Recognized keys:
    //   'd'/'e'/'v'  -- select duty/erpm/speed(governor) mode. While
    //                   actively driving, this ALSO stops first (exactly
    //                   like the stop key, reason "mode_switch") -- see
    //                   the class header's "RAMP MODE" section for why.
    //                   Any mode switch ALSO resets the speed governor's
    //                   own state (integrator, erpm filter) -- see
    //                   "SPEED GOVERNOR MODE".
    //   '+'/'='      -- increase the CURRENT mode's magnitude by one step
    //                   (clamped to that mode's max).
    //   '-'/'_'      -- decrease by one step (floored at one step -- never
    //                   goes to/through zero this way).
    //   'f'          -- drive forward (refreshes the deadman even if
    //                   already driving forward -- a no-op state change,
    //                   but still "any keypress").
    //   'b'/'r'      -- drive reverse.
    //   ' '/'s'      -- immediate stop (brake for brake_hold_ms, then idle).
    //   'c'          -- clear a latched abort (no effect if not aborted).
    //   'a'          -- toggle ramp mode on/off.
    //   'A'          -- arm RAMP-RATE entry: the next digit/'.'/ENTER
    //                   sequence commits to the current mode's ramp rate
    //                   (clamped to [0.001,1.0] duty/s or [10,20000]
    //                   erpm/s) instead of its magnitude. In speed mode
    //                   this sets duty_ramp (the same rate the governor
    //                   itself always slew-limits its output by).
    //                   Discards any in-progress buffer. A plain digit
    //                   typed WITHOUT a preceding 'A' still targets
    //                   magnitude, as before.
    //   'q'          -- returns a KeyEvent{kQuit} (caller decides what to
    //                   do with it -- this class has no notion of process
    //                   exit).
    //   digits/'.'   -- accumulate into a pending number buffer (magnitude
    //                   by default, or ramp rate if 'A' armed it first).
    //   ENTER ('\r' or '\n') -- commits the buffer (see 'A' above); clears
    //                   the buffer AND the ramp-rate-entry arming either
    //                   way (even on a parse failure).
    //   ESC (0x1B)   -- clears the buffer and cancels ramp-rate-entry
    //                   arming, without committing.
    //   'j'/'l'      -- steer left/right (see class header's "STEERING
    //                   SUB-STATE-MACHINE" section) -- independent of
    //                   drive mode/state, coarse step normally, fine
    //                   step while in_trim_mode().
    //   'k'/'R'      -- snap steering to config_.steer_center. Available
    //                   in ANY mode/state, not gated to trim mode.
    //   'T'          -- toggle TRIM MODE. Stops the drive first (reason
    //                   "trim_mode") if currently driving; while active,
    //                   'f'/'b'/'r' are ignored (see class header's
    //                   "TRIM MODE SAFETY DECISION").
    //   'W'          -- raise a pending "save current steering position
    //                   as the new center" request for main to consume
    //                   (no file I/O here) -- see
    //                   has_pending_center_save() et al. below.
    // f/b/r and 'c' are ignored while aborted (see class header); f/b/r
    // are ALSO ignored while in_trim_mode() (see "TRIM MODE SAFETY
    // DECISION"). Every other key above still applies even while
    // aborted or trimming (mode/step/digit/ramp edits are harmless
    // bookkeeping, not motor commands) -- and steering keys ('j'/'l'/
    // 'k'/'R'/'T'/'W') work identically in EVERY drive state, per the
    // "STEERING SUB-STATE-MACHINE" section.
    KeyEvent handle_key(char c, double now_s);

    // Advances the state machine to now_s and returns the motor command
    // to send THIS tick. Must be called once per control-loop tick even
    // if no key arrived this tick -- it is what notices deadman expiry
    // and brake-hold elapsing.
    TeleopMotorAction step(double now_s);

    // Safety watch: feed the latest successfully-parsed GET_VALUES reply.
    // Transitions to kAborted (latched) the first time
    // |values.current_motor| > current_abort or values.fault != 0 is
    // seen; a no-op on every subsequent call while already aborted (the
    // first trigger's reason string is kept, not overwritten).
    void feed_telemetry(const VescValues& values, double now_s);

    // --- status accessors (for the CLI's status line) ---
    TeleopMode mode() const { return mode_; }
    double duty_magnitude() const { return duty_mag_; }
    double erpm_magnitude() const { return erpm_mag_; }
    double speed_magnitude() const { return speed_mag_; }  // target erpm, speed mode.
    double magnitude() const {
        if (mode_ == TeleopMode::kDuty) return duty_mag_;
        if (mode_ == TeleopMode::kErpm) return erpm_mag_;
        return speed_mag_;
    }
    DriveDirection direction() const { return direction_; }
    bool is_driving() const { return state_ == State::kDriving; }
    bool is_braking() const { return state_ == State::kBraking; }
    bool is_aborted() const { return state_ == State::kAborted; }
    const std::string& abort_reason() const { return abort_reason_; }
    const std::string& stop_reason() const { return stop_reason_; }
    const std::string& digit_buffer() const { return digit_buffer_; }
    // True if a pending digit_buffer() entry will commit to the current
    // mode's ramp rate (armed via 'A') rather than its magnitude.
    bool digit_entry_is_ramp_rate() const { return entry_mode_is_ramp_rate_; }

    // Seconds remaining before the deadman fires, clamped to >= 0. Only
    // meaningful while is_driving() (that is the only state the deadman
    // actually watches); returns deadman_ms/1000 (the full window) if no
    // key has ever arrived yet, never a stale/negative-looking value.
    double deadman_remaining_s(double now_s) const;

    // --- ramp status accessors (for the CLI's status line) ---
    bool ramp_enabled() const { return ramp_enabled_; }
    double duty_ramp_rate() const { return duty_ramp_; }
    double erpm_ramp_rate() const { return erpm_ramp_; }
    // The rate relevant to the CURRENT mode -- duty_ramp_ for both kDuty
    // and kSpeed (speed mode's governor always slews its duty output by
    // duty_ramp_, see "SPEED GOVERNOR MODE"), erpm_ramp_ for kErpm.
    double ramp_rate() const { return mode_ == TeleopMode::kErpm ? erpm_ramp_ : duty_ramp_; }
    // The value actually being commanded this tick while driving (equals
    // target_value() instantly when ramp_enabled()==false, in kDuty/
    // kErpm; in kSpeed it's the governor's own slewed duty output). 0
    // when not driving.
    double emitted_value() const { return emitted_value_; }
    // The (signed) value the ramp is chasing: direction()*magnitude().
    // In kDuty/kErpm this is the commanded duty/erpm directly; in kSpeed
    // it is the TARGET ERPM the governor is chasing (NOT a duty value --
    // see emitted_value(), which in kSpeed is the governor's actual duty
    // output). Meaningful only while driving; 0 otherwise
    // (direction()==kStopped).
    // +1.0 normally, -1.0 when config_.drive_invert -- see drive_invert's
    // comment for why this is applied at exactly two places.
    double drive_sign() const { return config_.drive_invert ? -1.0 : 1.0; }

    double target_value() const { return static_cast<double>(static_cast<int>(direction_)) * magnitude(); }

    // --- speed governor status accessors (for the CLI's status line;
    // meaningful only while mode()==kSpeed) ---
    double filtered_erpm() const { return governor_.erpm_filtered(); }

    // --- steering status accessors (for the CLI's status line; see the
    // class header's "STEERING SUB-STATE-MACHINE" section) ---
    double steering_position() const { return steering_position_; }
    // The step size CURRENTLY in effect for 'j'/'l' -- steer_fine_step
    // while in_trim_mode(), steer_coarse_step otherwise.
    double steering_step() const { return trim_mode_ ? config_.steer_fine_step : config_.steer_coarse_step; }
    bool in_trim_mode() const { return trim_mode_; }
    // True once 'j'/'l'/'k'/'R' has actually moved (or, for k/R,
    // re-affirmed) steering_position_ at least once. Latched forever --
    // never reset, not even by abort/stop/deadman. See "NO PREMATURE
    // EMISSION" in the class header: main must not send any
    // SET_SERVO_POS until this is true.
    bool steering_ever_touched() const { return steering_touched_; }
    // True if steering_position() has changed since the last
    // mark_steering_emitted() call (or since construction, if never
    // called) -- main's cue that a fresh SET_SERVO_POS is due.
    bool steering_changed_since_emit() const { return steering_changed_since_emit_; }
    // Clears steering_changed_since_emit() -- main calls this right
    // after actually sending a SET_SERVO_POS.
    void mark_steering_emitted() { steering_changed_since_emit_ = false; }

    // 'W': does main have a pending "save steering_position() as the
    // new center" request to act on? This class does no file I/O
    // itself -- see the class header's "STEERING SUB-STATE-MACHINE"
    // section.
    bool has_pending_center_save() const { return pending_center_save_; }
    // Valid only if has_pending_center_save() is true.
    double pending_center_save_value() const { return pending_center_save_value_; }
    // Clears the pending flag -- main calls this after acting on the
    // request, whether the save succeeded or failed.
    void consume_pending_center_save() { pending_center_save_ = false; }

    // Updates the center 'k'/'R' snap to, in-memory, without
    // reconstructing this TeleopCore -- main calls this after
    // successfully persisting a new center from a consumed pending
    // request, so a subsequent 'k'/'R' snaps to the NEW value
    // immediately.
    void set_steer_center(double new_center);

    const TeleopConfig& config() const { return config_; }

private:
    enum class State { kIdle, kDriving, kBraking, kAborted };

    double max_for_mode(TeleopMode m) const;
    double step_for_mode(TeleopMode m) const;
    double ramp_rate_for_mode(TeleopMode m) const;
    double clamp_step_result(TeleopMode m, double value) const;  // floor = one step, ceiling = max_for_mode
    double clamp_digit_entry(TeleopMode m, double value) const;  // floor = 0, ceiling = max_for_mode
    double clamp_ramp_rate(TeleopMode m, double value) const;    // floor/ceiling per mode, see .cpp
    void begin_brake(const std::string& reason, double now_s);
    void commit_digit_buffer();
    void start_driving(DriveDirection dir);  // shared by 'f'/'b'/'r' -- see .cpp for the emitted_value_ reset rule
    void reset_governor();                   // integrator + erpm filter -- see "SPEED GOVERNOR MODE"
    TeleopMotorAction step_speed_governor(double dt);  // kSpeed's own step() branch, see .cpp
    double clamp_steering(double value) const;  // clamps to [config_.steer_min_pos, config_.steer_max_pos]
    void set_steering_position(double new_value);  // shared by j/l/k/R -- clamps + updates touched/changed latches

    TeleopConfig config_;

    TeleopMode mode_ = TeleopMode::kDuty;
    double duty_mag_;
    double erpm_mag_;
    double speed_mag_;
    double duty_ramp_;
    double erpm_ramp_;
    bool ramp_enabled_;

    // Speed governor (see "SPEED GOVERNOR MODE") -- the actual PI+FF+
    // filter+slew state now lives inside SpeedGovernor (see
    // SpeedGovernor.h); reset_governor() delegates to governor_.reset().
    // last_telemetry_time_s_ is TeleopCore's own bookkeeping (NOT
    // governor state) purely to compute the dt SpeedGovernor::feed_erpm()
    // needs between consecutive feed_telemetry() calls.
    SpeedGovernor governor_;
    double last_telemetry_time_s_ = 0.0;

    State state_ = State::kIdle;
    DriveDirection direction_ = DriveDirection::kStopped;

    double last_key_time_s_ = 0.0;
    bool has_key_ever_ = false;

    double brake_started_s_ = 0.0;
    std::string stop_reason_;

    std::string abort_reason_;

    std::string digit_buffer_;
    bool entry_mode_is_ramp_rate_ = false;

    // The ramp's own dt bookkeeping -- deliberately separate from
    // last_key_time_s_ (deadman) and brake_started_s_ (brake-hold): this
    // tracks consecutive step() call times, not keypresses.
    double last_step_time_s_ = 0.0;
    bool has_stepped_ = false;
    double emitted_value_ = 0.0;

    // Steering sub-state-machine (see the class header's "STEERING
    // SUB-STATE-MACHINE" section) -- deliberately has NO interaction
    // with any of the drive-state members above; see "STEERING SAFETY
    // INDEPENDENCE".
    double steering_position_;
    bool trim_mode_ = false;
    bool steering_touched_ = false;
    bool steering_changed_since_emit_ = false;
    bool pending_center_save_ = false;
    double pending_center_save_value_ = 0.0;
};

}  // namespace vesc

#endif  // VESC_DRIVER_TELEOP_CORE_H_
