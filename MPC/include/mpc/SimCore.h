#pragma once

#include <cstdint>
#include <random>
#include <string>

#include "mpc/Kinematics.h"

// Pure logic for mpc_robot_sim: ackermann command decode, localization
// payload encode, actuator clamping, and the watchdog state machine. No ZMQ
// here -- see src/robot_sim.cpp for the thin I/O shell, mirroring how
// MpcCore.h/.cpp keep main.cpp a thin I/O shell around mpc:: math.
//
// WIRE PROTOCOL: this file adapts the simulator to mpc_controller's
// EXISTING, byte-exact wire format (read from MPC/src/main.cpp):
//   - ackermann (mpc_controller -> sim): 2-frame [topic, JSON] where the
//     JSON's "speed"/"steering"/"accel" values are base64(ASCII-decimal(v))
//     -- see main.cpp's encodeAscii()/publish_ackermann().
//   - localization (sim -> mpc_controller): a single JSON payload frame
//     (preceded by a topic frame; main.cpp only inspects the LAST frame
//     received per socket.recv() burst) of the literal form
//     {"x":<number>,"y":<number>,"yaw":<number>} -- PLAIN JSON numbers, NOT
//     base64 -- exactly what main.cpp's control loop parses
//     (`last_payload.front() == '{'` then `nlohmann::json::parse` +
//     `.at("x").get<double>()` etc).
namespace mpc {

// Result of decoding one ackermann command payload. ok=false means the
// payload was empty/malformed/missing a required key/non-base64 -- the
// caller must keep whatever command it had before (last-valid-command
// retention) and must NOT crash.
struct AckermannCommand {
    double speed = 0.0;     // controller's own predicted speed; NOT integrated by the sim,
                             // only used for the drift diagnostic (see check_speed_drift below).
    double steering = 0.0;
    double accel = 0.0;
    bool ok = false;
};

// Decodes one ackermann JSON payload (the second ZMQ frame, without the
// topic frame). Uses the same base64_decode()/ASCII-decimal parsing the
// controller's encodeAscii() produces. Never throws -- any parse failure
// (empty payload, non-'{' payload, invalid JSON, missing "speed"/
// "steering"/"accel" key, non-base64 value, non-numeric decoded string)
// yields AckermannCommand{ok=false} with all other fields left at their
// default (0.0), so callers must check `ok` and retain their previous
// command rather than trusting the zeros.
AckermannCommand decode_ackermann_payload(const std::string& payload);

// Encodes a localization pose exactly as mpc_controller's main.cpp control
// loop parses it: plain JSON, decimal numbers, no base64. Round-trips any
// finite x/y/yaw, including negative values and yaw outside [-pi, pi].
std::string encode_localization_payload(double x, double y, double yaw);

// Result of clamp_command(): the clamped (accel, steering) plus flags for
// whether each individual channel actually needed clamping (so the I/O
// layer can rate-limit a warning per the "one-time (rate-limited) stderr
// warning when a received command violates them" spec requirement).
struct ClampResult {
    double accel = 0.0;
    double steering = 0.0;
    bool accel_clamped = false;
    bool steering_clamped = false;
};

// Clamps accel to +-max_accel and steering to +-max_steer independently.
// Does NOT clamp v -- v follows from bounded accel; push/non-push velocity
// caps are controller policy, not plant physics (see design spec).
ClampResult clamp_command(double accel, double steering, double max_accel, double max_steer);

// Auto-brake acceleration for the watchdog's engaged state: decelerates
// toward zero at the actuator's max_accel magnitude, direction-aware
// (opposes the current sign of v) so it never accelerates further in the
// same direction, and snaps to exactly 0 once |v| is within zero_v_eps of
// zero (so the integrated v cannot chatter across zero forever). Pure/
// stateless so it is independently unit-testable (see S3).
double watchdog_brake_accel(double v, double max_accel, double zero_v_eps = 0.005);

// One tick's resolved plant input: what actually gets integrated this step,
// after accounting for watchdog engagement. When NOT engaged, this is just
// the (already-clamped) last received command passed through unchanged.
// When engaged, accel is replaced by watchdog_brake_accel() and steering is
// HELD at whatever the last commanded steering was (per the design spec:
// "HOLD last steering, apply a = -sign(v)*max_accel...").
//
// snap_v_to_zero is set true iff the watchdog is engaged AND |current_v| is
// already within zero_v_eps of zero -- i.e. watchdog_brake_accel() returned
// 0.0 because no further braking is needed. accel==0 on its own only holds
// the CURRENT velocity, it does not erase a sub-eps residual, so without an
// explicit snap the caller's plant integrates that residual v every tick
// forever (position drifts at a constant rate for as long as the watchdog
// stays engaged with no new command -- observed to persist for tens of
// seconds when a robot idles after finishing while other robots are still
// executing). The caller (robot_sim.cpp) must force its integrated v to
// exactly 0.0 when this flag is set, which is what actually delivers the
// "snaps to exactly 0" behavior documented on watchdog_brake_accel() below.
struct ResolvedCommand {
    double accel = 0.0;
    double steering = 0.0;
    bool snap_v_to_zero = false;
};

ResolvedCommand resolve_command(bool watchdog_engaged, double current_v, double last_cmd_accel,
                                 double last_cmd_steering, double max_accel,
                                 double zero_v_eps = 0.005);

// Watchdog state machine (safety-net precedent for a future real VESC
// bridge). Engages auto-brake when no valid command has been received
// within watchdog_timeout_s of the last one; disengages the instant a new
// valid command arrives. Exposes just_engaged()/just_recovered() edge
// flags, valid for exactly the update() call that caused the transition,
// so the I/O layer can log "engage"/"recovery" exactly once per event.
class Watchdog {
   public:
    // Advances the state machine given how long it has been since the last
    // valid command (last_cmd_age_s) and the configured timeout. Returns
    // the (possibly just-updated) engaged() state.
    bool update(double last_cmd_age_s, double watchdog_timeout_s);

    // Call the instant a new valid command is received. Disengages
    // immediately regardless of `update()`'s timing (per spec: "next
    // received command disengages immediately").
    void on_command_received();

    bool engaged() const { return engaged_; }
    bool just_engaged() const { return just_engaged_; }
    bool just_recovered() const { return just_recovered_; }

   private:
    bool engaged_ = false;
    bool just_engaged_ = false;
    bool just_recovered_ = false;
};

// ---------------------------------------------------------------------
// FEATURE B: mpc_robot_sim deadband plant model ("a BLDC that cannot move
// below its own breakaway speed" -- deliberately NOT a velocity offset).
// OPT-IN: robot_sim.cpp only calls step_deadband()/integrate_pose_with_v()
// below when --deadband (or a --min-moving-speed=/--min-sustain-speed=
// override, which implies it) is passed; with the feature off, the sim's
// existing rollout_step()-based pipeline is untouched, so mpc_robot_sim
// stays bit-identical to before this feature existed.
//
// The command-equivalent velocity (see robot_sim.cpp's v_cmd_equiv) keeps
// integrating applied acceleration every tick exactly like today's
// `state.v` always has -- this deliberately models a motor controller's
// internal ERPM reference winding up while the wheels themselves stay
// stalled. What is NEW is a hysteresis "moving" flag layered on top: while
// NOT moving, the EFFECTIVE velocity (the one that actually drives pose
// integration and everything reported to the outside world -- CSV 'v',
// localization, telemetry 'v') is pinned at exactly 0 regardless of how
// large the command-equivalent velocity has wound up to; the instant
// |v_cmd_equiv| >= min_moving_speed, moving flips true and the effective
// velocity JUMPS directly to v_cmd_equiv (not eased in -- the jump IS the
// phenomenon being modeled, i.e. breakaway). Once moving, effective
// velocity tracks v_cmd_equiv exactly until |v_cmd_equiv| drops below
// min_sustain_speed, at which point moving flips back false and effective
// velocity snaps back to 0. min_sustain_speed <= min_moving_speed (enforced
// by RobotSpec::validate()) is what makes this true hysteresis rather than
// a single threshold: a robot can keep "moving" (driven by the full
// v_cmd_equiv) at speeds below its own breakaway threshold, as long as it
// never dips below the (lower-or-equal) sustain threshold.
// ---------------------------------------------------------------------
struct DeadbandStepResult {
    double effective_v = 0.0;
    bool moving = false;
};

// Pure hysteresis step. v_cmd_equiv is THIS tick's already-integrated
// command-equivalent velocity (see robot_sim.cpp); was_moving is the moving
// flag carried over from the PREVIOUS tick (or the resolved initial state at
// t=0 -- see robot_sim.cpp). Returns the new moving flag plus the effective
// velocity to use for this tick's pose integration/reporting.
DeadbandStepResult step_deadband(double v_cmd_equiv, bool was_moving, double min_moving_speed,
                                  double min_sustain_speed);

// Advances (x, y, yaw) by one dt using an EXPLICIT velocity-for-this-step,
// mirroring mpc::rollout_step's own x/y/yaw formulas (see Kinematics.h)
// EXACTLY -- deliberately duplicated here (Kinematics.h is shared with
// MpcCore and is not touched by this feature) rather than reused, because
// rollout_step always integrates v from accel AND steps pose from that SAME
// newly-integrated v in one call, whereas FEATURE B needs pose to advance
// using the EFFECTIVE velocity while a separate v_cmd_equiv (see above)
// keeps integrating from accel regardless of motion state. The returned
// State4's .v field is set to v_for_pose (robot_sim.cpp relies on this to
// make state.v hold the effective velocity for CSV/localization/telemetry
// reporting, exactly matching what rollout_step's own next.v does today).
State4 integrate_pose_with_v(const State4& state, double v_for_pose, double delta,
                              double wheel_base, double dt);

// BREAKAWAY REALISM (see RobotSpec::max_breakaway_accel's doc comment for
// the physical rationale -- a real BLDC/ESC in speed-control mode delivers
// launch current far above its steady-state/comfort accel while trying to
// break static friction). Resolves the accel clamp CEILING robot_sim.cpp's
// two accel-clamping call sites (the initial received-command clamp_command()
// and FEATURE A's post-noise apply_command_noise() re-clamp) should use for
// THIS tick's command: max_breakaway_accel while the deadband feature is
// enabled AND the plant is currently stalled (was_moving=false, i.e. the
// PREVIOUS tick's resolved deadband state -- see step_deadband's own
// was_moving parameter, which this mirrors), otherwise max_accel unchanged.
// A pure no-op (returns max_accel exactly, bitwise) whenever
// deadband_enabled is false, so mpc_robot_sim stays bit-identical to before
// this feature existed when --deadband is not passed -- same discipline as
// step_deadband/integrate_pose_with_v above.
double resolve_accel_clamp_ceiling(bool deadband_enabled, bool currently_moving, double max_accel,
                                    double max_breakaway_accel);

// ---------------------------------------------------------------------
// Telemetry topic (additive -- see robot_sim.cpp): published on the EXISTING
// bound localization PUB socket under topic "/<robot>/telemetry" at the
// localization rate, plain JSON (not base64), alongside the UNCHANGED
// localization payload -- published unconditionally (not gated by
// --deadband), since it only ever adds a new topic and never changes any
// EXISTING message's bytes. Exposes what the deadband model (and the
// existing watchdog/noise machinery) is doing internally, for tests and
// visualization -- encode/parse are pure functions so both sides (this sim
// and any consumer) can be unit-tested without ZMQ.
// ---------------------------------------------------------------------
struct TelemetrySample {
    double t = 0.0;         // sim time in seconds (matches the CSV 't' column).
    double v = 0.0;         // effective velocity (== CSV 'v'; 0 while deadband-stalled).
    double v_cmd = 0.0;     // command-equivalent velocity (keeps integrating even while stalled).
    double steering = 0.0;  // applied steering this tick (post watchdog-hold).
    double accel = 0.0;     // applied acceleration this tick (post watchdog-brake/noise resolution).
    bool moving = false;    // FEATURE B hysteresis state; always true when --deadband is off.
    bool watchdog = false;  // watchdog engaged this tick.
};

// Plain-JSON encode: {"t":...,"v":...,"v_cmd":...,"steering":...,"accel":...,
// "moving":0|1,"watchdog":0|1}. moving/watchdog encode as JSON integers 0/1
// (not JSON booleans) per the wire spec.
std::string encode_telemetry_payload(const TelemetrySample& sample);

// Parse helper for tests/viz. ok=false on any parse failure (empty payload,
// non-'{' payload, invalid JSON, missing/wrong-typed key) -- never throws;
// sample is left default-constructed in that case, mirroring
// decode_ackermann_payload()'s contract.
struct TelemetryParseResult {
    TelemetrySample sample;
    bool ok = false;
};
TelemetryParseResult parse_telemetry_payload(const std::string& payload);

// ---------------------------------------------------------------------
// FEATURE A: actuation noise ("motor controller with errors").
//
// Models a motor controller that executes each RECEIVED ackermann command
// slightly wrong: the instant a new command is decoded, one perturbation is
// sampled and then HELD (added to every tick's plant input) until the next
// command replaces it -- this is deliberately NOT per-tick white noise. The
// watchdog's internal auto-brake command is never routed through this class
// (see robot_sim.cpp): it is a safety response, not a received command.
//
// TWO INDEPENDENT CHANNELS: accel_sigma_pct scales +-max_accel
// (d_accel ~ N(0, accel_sigma_pct * max_accel)); steer_sigma_pct
// independently scales +-max_steer (d_steer ~ N(0, steer_sigma_pct *
// max_steer)). Each channel is evaluated separately in sample_command(): a
// channel whose sigma is exactly 0 returns an exact-zero perturbation
// WITHOUT drawing from the RNG at all (that channel's output is bitwise
// untouched, independent of what the OTHER channel is doing), so with BOTH
// sigmas at 0 (the default) the RNG is never touched and mpc_robot_sim's
// behavior is bit-identical to the pre-noise implementation.
class NoiseModel {
   public:
    static constexpr double kMinSigmaPct = 0.0;
    static constexpr double kMaxSigmaPct = 0.25;

    // Clamps to [kMinSigmaPct, kMaxSigmaPct]. Non-finite input (NaN/inf) is
    // treated as 0 (the safe "no noise" default) rather than propagating.
    static double clamp_sigma_pct(double sigma_pct);

    NoiseModel() = default;
    // accel_sigma_pct/steer_sigma_pct are each clamped independently via
    // clamp_sigma_pct(); rng is default-constructed (unseeded engine state)
    // -- fine since sigma==0 on a channel never draws from it.
    NoiseModel(double accel_sigma_pct, double steer_sigma_pct) {
        set_accel_sigma_pct(accel_sigma_pct);
        set_steer_sigma_pct(steer_sigma_pct);
    }
    // Seedable constructor -- callers needing determinism (tests, or
    // --noise-seed/--steer-noise-seed on the CLI) should always use this
    // form.
    NoiseModel(double accel_sigma_pct, double steer_sigma_pct, std::uint64_t seed) : rng_(seed) {
        set_accel_sigma_pct(accel_sigma_pct);
        set_steer_sigma_pct(steer_sigma_pct);
    }

    void seed(std::uint64_t seed_value) { rng_.seed(seed_value); }

    void set_accel_sigma_pct(double sigma_pct) { accel_sigma_pct_ = clamp_sigma_pct(sigma_pct); }
    void set_steer_sigma_pct(double sigma_pct) { steer_sigma_pct_ = clamp_sigma_pct(sigma_pct); }
    double accel_sigma_pct() const { return accel_sigma_pct_; }
    double steer_sigma_pct() const { return steer_sigma_pct_; }

    struct Perturbation {
        double d_accel = 0.0;
        double d_steer = 0.0;
    };

    // Samples ONE new perturbation for a newly-received command. Callers
    // must call this exactly once per new command and hold the result
    // (added to the command, then re-clamped via apply_command_noise() below)
    // until the next command arrives. Each channel is sampled independently
    // (accel first, then steer, matching the pre-split draw order when both
    // were driven by one sigma) and only if ITS OWN sigma is > 0.
    Perturbation sample_command(double max_accel, double max_steer);

   private:
    double accel_sigma_pct_ = 0.0;
    double steer_sigma_pct_ = 0.0;
    std::mt19937_64 rng_;
};

// Applies a held FEATURE A perturbation to an already-clamped received
// command and re-clamps the result: applied = clamp_command(last_cmd + pert,
// limits) -- noise FIRST, clamp AFTER, per the design spec. This is a pure
// function (no I/O) so it can be unit-tested directly; robot_sim.cpp's
// ackermann-command handler calls this EXACT function rather than
// re-deriving the add-then-clamp sequence inline, so a test exercising this
// function also exercises robot_sim.cpp's real code path bit-for-bit (see
// mpc_unit_tests.cpp's N4).
ClampResult apply_command_noise(double last_cmd_accel, double last_cmd_steering,
                                 const NoiseModel::Perturbation& pert, double max_accel,
                                 double max_steer);

// Result of parse_sim_config_payload(): the accel/steer sigma_pct (FEATURE
// A/C) AND the deadband enable flag / thresholds (FEATURE B2) to use going
// forward, EACH of the 5 fields resolved independently (already
// clamped/coerced when that field was present and valid; equal to the
// caller-supplied current_* value, UNCHANGED, when that field was absent or
// invalid) plus per-field flags describing what happened. A payload may
// update any subset of the 5 fields -- all, some, or none.
struct SimConfigParseResult {
    // FEATURE A/C: actuation noise sigma (unchanged fields/semantics).
    double accel_sigma_pct = 0.0;
    double steer_sigma_pct = 0.0;
    bool accel_ok = false;          // true iff a valid "noise_sigma_pct" numeric field was found.
    bool steer_ok = false;          // true iff a valid "steer_noise_sigma_pct" numeric field was found.
    bool accel_was_clamped = false; // true iff the parsed accel value was outside [0, kMaxSigmaPct].
    bool steer_was_clamped = false; // true iff the parsed steer value was outside [0, kMaxSigmaPct].
    // true iff "noise_sigma_pct"/"steer_noise_sigma_pct" was PRESENT in the
    // payload but failed to parse (wrong JSON type) -- lets the caller warn
    // on exactly the fields that were actually malformed, independent of
    // whether OTHER fields in the same payload succeeded (see robot_sim.cpp).
    bool accel_bad = false;
    bool steer_bad = false;

    // FEATURE B2 (live deadband reconfiguration via "/<robot>/sim_config"):
    // resolved deadband enable flag + thresholds. Same per-field
    // independence/retention discipline as the noise channels above.
    bool deadband_enabled = false;         // resolved "deadband" (nonzero numeric == on).
    bool deadband_ok = false;              // true iff a valid "deadband" numeric field was found.
    bool deadband_bad = false;             // true iff "deadband" was present but wrong-typed.
    double min_moving_speed = 0.0;         // resolved, clamped to [0, 0.5] when present/valid.
    bool min_moving_speed_ok = false;
    bool min_moving_speed_was_clamped = false;
    bool min_moving_speed_bad = false;
    double min_sustain_speed = 0.0;        // resolved, clamped to [0, APPLIED min_moving_speed]
                                            // (this SAME payload's min_moving_speed if it was also
                                            // present and valid, else the retained current value).
    bool min_sustain_speed_ok = false;
    bool min_sustain_speed_was_clamped = false;
    bool min_sustain_speed_bad = false;

    // OPTIONAL live override of RobotSpec::max_breakaway_accel (see that
    // field's doc comment) -- clamped to [APPLIED max_accel_for_clamp, 10.0]
    // (the caller-supplied floor, typically robot_spec.max_accel: a
    // breakaway ceiling below the nominal actuator ceiling is not headroom
    // at all -- mirrors RobotSpec::validate()'s own >= max_accel
    // requirement). Same per-field independence/retention discipline as
    // every other field above.
    double max_breakaway_accel = 3.0;
    bool max_breakaway_accel_ok = false;
    bool max_breakaway_accel_was_clamped = false;
    bool max_breakaway_accel_bad = false;

    // Convenience: true iff AT LEAST ONE of the 6 fields was successfully
    // parsed. A structurally malformed payload (empty, non-'{', invalid
    // JSON) yields ok=false with ALL fields retained.
    bool ok = false;
};

// Parses the sim-only live-reconfiguration payload published on topic
// "/<robot>/sim_config": plain JSON (numbers, NOT base64 -- unlike the
// ackermann wire format) that may contain any subset of:
//   {"noise_sigma_pct": <number>, "steer_noise_sigma_pct": <number>,
//    "deadband": <0 or 1>, "min_moving_speed": <number>,
//    "min_sustain_speed": <number>}
// Never throws. Unknown extra JSON fields are ignored. A structurally
// malformed payload (empty/non-'{'/invalid-JSON) yields ALL 5 fields
// retained at their current_* values with every *_ok/ok flag false. Within a
// structurally valid payload, each field is parsed independently: a missing
// field leaves just that field retained (its own *_ok stays false, *_bad
// stays false too -- "absent" is not "malformed"); a PRESENT but
// wrong-typed field leaves it retained with *_bad=true (so the caller can
// warn specifically about it) without affecting any other field in the same
// payload.
//
// "deadband": accepts any JSON NUMBER (per the wire contract: 0 or 1);
// resolved via != 0.0 (so any nonzero value reads as "on") -- a JSON
// boolean or non-numeric value is wrong-typed (*_bad=true, retained).
//
// "min_moving_speed": clamped to [0, 0.5] (non-finite treated as 0, mirroring
// NoiseModel::clamp_sigma_pct's philosophy).
//
// "min_sustain_speed": clamped to [0, APPLIED min_moving_speed] -- "applied"
// means out.min_moving_speed AS ALREADY RESOLVED earlier in this SAME call
// (the payload's own "min_moving_speed" if present and valid, else the
// caller-supplied current_min_moving_speed) -- this function always
// resolves min_moving_speed before min_sustain_speed so that ordering holds
// regardless of the two keys' order in the JSON object.
//
// "max_breakaway_accel" (OPTIONAL field -- see RobotSpec::max_breakaway_accel):
// clamped to [current_max_accel_for_clamp, 10.0]; current_max_accel_for_clamp
// is NOT itself a resolved-from-this-payload value (this feature has no
// live "max_accel" reconfiguration channel) -- it is simply the caller's own
// robot_spec.max_accel, passed straight through every call as the clamp
// floor.
SimConfigParseResult parse_sim_config_payload(const std::string& payload,
                                               double current_accel_sigma_pct,
                                               double current_steer_sigma_pct,
                                               bool current_deadband_enabled = false,
                                               double current_min_moving_speed = 0.1,
                                               double current_min_sustain_speed = 0.1,
                                               double current_max_breakaway_accel = 3.0,
                                               double current_max_accel_for_clamp = 0.73);

} // namespace mpc
