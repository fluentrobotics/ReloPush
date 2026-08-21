#pragma once

#include <string>

// Robot geometry/hardware limits and MPC tuning parameters, factored out of
// MPC/src/main.cpp so they can be shared, unit tested, and
// overridden from a JSON file without touching code.
//
// NOTE: the MPC prediction horizon (mpc::kHorizon, see MpcCore.h) is
// deliberately NOT part of either struct below -- it stays a compile-time
// constant. See config/robot_spec.json's "_doc" field for the rationale.
namespace mpc {

// Robot geometry and hardware limits. Defaults mirror the MARS production
// values (MARS/src/ReloPushSetup.cpp make_relopush_robot,
// MARS/src/AllocationSearch.cpp initialize_entities) plus controller-side
// conventions (max_steer/max_accel/max_v_*) that MARS's RobotMeta does not
// carry.
struct RobotSpec {
    double wheel_base = 0.29;
    double front_length = 0.36;
    double rear_length = 0.12;
    double width = 0.275;

    double min_turning_radius_transit = 1.02;
    double min_turning_radius_transfer = 1.43;

    double speed_transit = 0.2;
    double speed_transfer = 0.15;

    // Physical actuator limit; deliberately larger than the planner-implied
    // atan(wheel_base / min_turning_radius_transit) to leave correction
    // authority for the controller.
    double max_steer = 0.33;
    double max_accel = 0.73;
    double max_v_push = 0.295;
    double max_v_nonpush = 0.38;

    // Deadband plant model (mpc_robot_sim FEATURE B, opt-in via --deadband --
    // see SimCore.h) AND the mpc_controller LAUNCH GOVERNOR (see
    // LaunchGovernor.h, active only when min_moving_speed > 0): models a
    // BLDC that cannot physically move below its own breakaway speed.
    // min_moving_speed is the |command-equivalent velocity| threshold that
    // must be reached from rest before the robot is considered "moving" at
    // all (breakaway); min_sustain_speed is the (lower or equal) threshold
    // below which an already-moving robot stalls back out. Both are in m/s.
    // launch_margin is a dimensionless fraction (0.1 == 10%) the LAUNCH
    // GOVERNOR uses to command some headroom above min_moving_speed when
    // deliberately launching from rest; it is not itself consumed by the
    // deadband state machine.
    //
    // THESE ARE HARDWARE PROPERTIES, not simulation-only knobs -- an ideal
    // (no-deadband) plant is the compiled DEFAULT (0.0/0.0: no threshold, no
    // governor activation), matching what every caller gets unless it
    // explicitly opts in. Real hardware deployments must supply the real
    // measured values via a --robot-spec=<path> JSON override (see
    // config/robot_spec.json, which keeps 0.1/0.1 as the hardware truth for
    // this project's actual robots) -- both mpc_controller (to activate the
    // launch governor with the matching threshold) and, for simulated
    // testing, mpc_robot_sim (via --deadband --min-moving-speed=/
    // --min-sustain-speed=, which read RobotSpec too but are typically
    // passed explicitly on its own command line rather than via
    // --robot-spec -- see SimCore.h). launch_margin's default (0.1) is
    // unaffected by this -- it is inert whenever min_moving_speed is 0.
    double min_moving_speed = 0.0;
    double min_sustain_speed = 0.0;
    double launch_margin = 0.1;

    // SIM-SIDE breakaway realism (mpc_robot_sim FEATURE B, opt-in via
    // --deadband -- see SimCore.h's resolve_accel_clamp_ceiling()): while the
    // deadband is enabled AND the plant is currently stalled (not moving),
    // the sim clamps the RECEIVED command's accel to +-max_breakaway_accel
    // instead of the nominal +-max_accel before integrating it. This models
    // a real BLDC/ESC in speed-control mode, which delivers launch current
    // far above its steady-state/comfort accel while trying to break static
    // friction -- a physical effect entirely separate from
    // min_moving_speed/min_sustain_speed (which model the breakaway/sustain
    // SPEED thresholds, not the ACCEL headroom available to reach them). Once
    // the plant is moving, the nominal max_accel ceiling applies as before --
    // this field has no effect at all while moving, and none while the
    // deadband feature is off. Defaults to 3.0 (comfortably above max_accel's
    // own 0.73 default) so a caller that never sets it still gets a
    // physically sensible launch-current headroom rather than an inert
    // (==max_accel) clamp; see validate() for the >= max_accel requirement
    // this relies on.
    double max_breakaway_accel = 3.0;

    // FINISH-AT-REST CORRECTION LAUNCH (see LaunchGovernor.h's
    // CORRECTION-LAUNCH entry in step()'s doc comment, and main.cpp's
    // position-aware completion): the plain LAUNCH-entry guard's own
    // |ref_vel_now| > kStopSnapRefVelThreshold requirement deliberately
    // suppresses a spurious relaunch once the reference has genuinely
    // settled -- correct on its own terms, but taken alone it also means NO
    // residual position error at a stationary reference can ever be
    // corrected: STOP-SNAP parks the robot the instant v_des drops below
    // min_sustain_speed, wherever the deadband happened to freeze it, with
    // no way back (the user-visible symptom: robots parking short of a
    // reference pose and never finishing). settle_position_tolerance draws
    // the line between "close enough, leave it" and "worth one more
    // nudge": approximately the minimum displacement a single breakaway
    // kick can cleanly achieve -- one launch tick's travel at the kick's
    // target speed, plus the raw-exit debounce's own latency, plus the
    // brake distance STOP-SNAP needs to re-park at that speed (roughly
    // 2-3cm total for this project's real robots, per LaunchGovernor's own
    // measured ~0.2s breakaway/exit timing) -- residual errors below this
    // are not reliably correctable without a real risk of overshooting
    // past them and needing an immediate reversal correction, chasing its
    // own tail. Only consulted by the LAUNCH GOVERNOR's CORRECTION-LAUNCH
    // entry; inert whenever min_moving_speed<=0 (governor permanently
    // transparent) since main.cpp's position_error input can only ever
    // matter once a real deadband has stalled the plant short in the first
    // place. See validate() for this field's range check.
    double settle_position_tolerance = 0.03;

    // FINISH-AT-REST CORRECTION LAUNCH: minimum wall-clock seconds between
    // the END of one correction-launch sojourn and the START of the next,
    // within the SAME stop event (see LaunchGovernor.h's
    // kCorrectionLaunchTimeoutS and settle_max_attempts below) -- gives the
    // plant/observer a moment to settle after a give-up or an
    // overshoot-and-restall before trusting the next position_error sample
    // enough to kick again, rather than immediately re-launching off a
    // single noisy reading.
    double settle_cooldown_s = 1.0;

    // FINISH-AT-REST CORRECTION LAUNCH: maximum number of correction-launch
    // attempts (successful breakaway or given-up-on alike) per STOP EVENT
    // (an intermediate hold or the trajectory's own end -- see
    // LaunchGovernor.h's mpc::RefStopEvent/extract_ref_stop_events). Caps
    // how long this project keeps nudging at a residual error it cannot
    // seem to close -- once exhausted, the governor prints one clear
    // stderr warning (with the residual error) and stays parked rather
    // than looping forever.
    int settle_max_attempts = 3;

    static RobotSpec defaults();

    // Returns "" if the spec is internally consistent. Otherwise returns a
    // human-readable description of the first problem found (checked in
    // order): (1) the planner-implied steering angle for the transit turning
    // radius must not exceed the hardware's max_steer; (2) min_sustain_speed
    // must satisfy 0 <= min_sustain_speed <= min_moving_speed (required for
    // the deadband model's hysteresis to make sense -- see min_sustain_speed
    // above); (3) max_breakaway_accel must satisfy max_breakaway_accel >=
    // max_accel (a breakaway headroom that is LOWER than the nominal
    // actuator ceiling is not headroom at all -- see max_breakaway_accel
    // above); (4) WHEN min_moving_speed > 0 (the LAUNCH GOVERNOR is active),
    // settle_position_tolerance must satisfy 0 < settle_position_tolerance
    // <= 0.2 (m) -- see that field's own doc comment for the ~2-3cm
    // correctable-displacement rationale behind this upper bound; skipped
    // entirely for an ideal (min_moving_speed<=0) plant, where this field is
    // inert anyway. Independently of all of those, ALWAYS (even if one of
    // the above already failed) prints a non-fatal stderr WARNING if
    // min_moving_speed >= speed_transfer: a robot that cannot break away
    // below its own pushing-transfer speed may never be able to start a
    // push at all (pushing-margin hazard) -- this does not fail validate()
    // (the returned string is unaffected), since e.g. transit-only
    // deployments never command speed_transfer at all.
    std::string validate() const;
};

// MPC solver weights and loop timing.
struct MpcParams {
    double dt = 0.05;
    int control_delay_steps = 1;
    int loop_hz = 20;

    double w_dist = 5.0;
    double w_dist_dirchange = 50.0;
    double w_yaw = 1.0;
    double w_vel = 0.2;
    double w_lat = 40.0;
    double w_control = 0.001;
    double w_delta_rate = 50.0;

    // Published-"speed"-field scaling (see MpcCore.h build_payload). Always
    // applied -- there is no hardware-calibration on/off switch; every
    // invocation of mpc_controller scales the wire "speed" field this way.
    double vel_scale = 1.01;
    double vel_scale_back = 0.9;
    double dir_change_scale = 0.77;

    static MpcParams defaults();
};

// Loads the "robot" and "mpc" top-level objects from a single JSON file,
// overriding only the keys that are present -- callers should seed
// robot_out/mpc_out with defaults() first so absent keys keep their
// compiled defaults. Unknown keys produce a warning on stderr but do not
// fail loading. Throws std::runtime_error if the file cannot be opened or
// is not valid JSON.
void load_spec_from_json(const std::string& path, RobotSpec& robot_out, MpcParams& mpc_out);

} // namespace mpc
