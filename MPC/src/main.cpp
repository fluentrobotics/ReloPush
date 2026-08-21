#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include <ReloPush/base64.h>
#include <ReloPush/trajectory.hpp>

#include "mpc/LaunchGovernor.h"
#include "mpc/MpcCore.h"
#include "mpc/RobotSpec.h"

// main.cpp is a thin I/O shell: ZMQ sockets, the MARS handshake, the 20Hz
// control loop, and payload (de)serialization. All MPC math lives in mpc::
// (RobotSpec.h/MpcCore.h/Kinematics.h) so it can be unit tested without
// ZMQ. Acceleration-formulation MPC (see MpcCore.h): this loop maintains its
// own real velocity ESTIMATE (mpc::State4, finite-differenced from
// consecutive localization samples -- see estimate_velocity() below), fed
// as SolveInputs::v0 into every solve, instead of trusting its own last
// commanded value. The wire protocol (handshake, ackermann/localization
// topics/JSON schema) is UNCHANGED -- this is a control-loop-internal
// change only.
//
// LAUNCH GOVERNOR (see mpc/LaunchGovernor.h): between the solve and the
// publish below, a small state machine may override the PUBLISHED velocity
// (never steering, never the MPC's own solve) to step cleanly to breakaway
// speed when the plant has a BLDC deadband (RobotSpec::min_moving_speed)
// that the MPC's own small from-rest commands can never cross on their own
// -- true velocity feedback means such a controller would otherwise stall
// forever instead of lurching. Active only when min_moving_speed > 0; see
// LaunchGovernor.h for the full state machine.

namespace {

// MODEL-PRIOR VELOCITY OBSERVER (complementary filter/predict-correct) for
// the finite-differenced velocity estimate (see the "got_new_pose_this_tick"
// handling in main() below). Replaces a pure EMA low-pass that shipped
// earlier and was found (this task's own report) to make the closed loop
// only MARGINALLY stable rather than genuinely fix anything -- see below.
//
// WHY THE EMA WAS REPLACED: est_state.v feeds BOTH the MPC solver's v0
// (every solve) AND the LAUNCH GOVERNOR's exit check, so any phase lag
// added here is lag the WHOLE closed loop has to fight, not just measurement
// cleanup. That the EMA was, in fact, in the loop's own dynamics (not inert)
// was visible in its own history: retuning ONLY its time constant (tau =
// 0.10/0.15/0.20s, otherwise identical builds) swung test_mpc_deadband Part
// B's settle time between 7.28s (the metric's own "never settled" ceiling)
// and 0.875-1.61s -- a properly-damped measurement filter should not change
// a control loop's qualitative stability just by retuning itself. This task
// confirmed directly (six back-to-back runs of the SHIPPED tau=0.15s build,
// on an otherwise-idle machine) that settle_t=7.28s reproduces on the
// MAJORITY of runs (5/6), not tau=0.10s alone as previously believed; a
// PASSING run's own CSV ground-truth v (not just this file's own belief)
// showed the EXACT SAME non-decaying, roughly 1-1.3s-period oscillation
// around the 0.2 m/s cruise reference for the ENTIRE ~7.5s cruise as a
// FAILING run's -- the two runs' published v only measurably diverged in
// PHASE from sub-millisecond scheduling jitter (first visible around
// t~=1.87s into the cruise in this task's traces). I.e. tau=0.15s was never
// actually settling anything: both outcomes are the SAME undamped limit
// cycle, and whether the test's "last time |v-ref|>=0.05" metric happens to
// sample that cycle's small- or large-amplitude phase by cruise-end is
// essentially a coin flip driven by chaotic sensitivity, not a real settle.
//
// THE FIX: predict from the known model, correct only a little from the
// noisy measurement. Every control tick, PREDICT this tick's estimate from
// the PREVIOUS estimate plus the REAL last-commanded acceleration over one
// control period:
//     v_pred = v_est + last_cmd_a * dt
// -- exactly mpc::rollout_step's own v'=v+a*dt (Kinematics.h), i.e. the SAME
// physical model this file's own dead-reckoning fallback already trusts.
// last_cmd_a is a KNOWN, already-applied control input (not a noisy
// sample), so this prediction carries the loop's PHASE with effectively
// zero lag -- unlike an EMA, which only ever chases a noisy measurement
// from behind, one tau at a time. Only on a genuinely FRESH localization
// sample do we then CORRECT v_pred a small fraction of the way toward the
// raw finite-differenced measurement:
//     v_est = v_pred + kVelocityObsGain * (v_meas_raw - v_pred)
// -- a FLAT (not dt_meas-scaled) low gain: the model half already carries
// timing, so the measurement's only remaining job is catching what the
// model alone cannot know about (actuation error, deadband stalls/snaps),
// not carrying phase. kVelocityObsGain=0.15 (mid-range of this brief's own
// 0.1-0.2 suggested band) attenuates the raw per-tick finite-difference
// noise this file's history describes (still present -- traced in this
// task's report to the localization-publish/control-consume rate
// relationship, NOT to dt_meas variability: dt_meas measured essentially
// CONSTANT, ~0.050-0.051s stdev 0.0002s, across every trace this task
// captured) by roughly that same factor per correction, without
// reintroducing an EMA's own lag into the loop. Verified (this task's
// report) not to slow the LAUNCH GOVERNOR's own breakaway detection: this
// gain, applied to the (correctly zero, pre-breakaway) raw measurement,
// converges the pre-breakaway estimate to a steady-state well below the
// governor's own kExitFraction*min_moving_speed crossing point on the SAME
// timescale the plant's own deadband model actually breaks away on --
// t_breakaway_after_ref_start measured ~0.203s across every run, unchanged
// from before this fix, and well inside the governor's own <=0.3s bound.
constexpr double kVelocityObsGain = 0.15;

// SCHEDULE CATCH-UP (see mpc::boosted_vr's doc comment in MpcCore.h for the
// full mechanism and sign-convention derivation). kCatchupGain (K) converts
// a signed along-track SCHEDULE lag in METERS into a velocity correction in
// m/s -- units 1/s -- layered on top of the MPC's own per-stage reference
// velocity target. K=1.0 means "0.1m behind -> +0.1 m/s", the brief's own
// example and the middle of its suggested [0.5, 2.0] 1/s band. Empirical
// basis (this task's own baseline measurement, results/sim_handoff/
// greedy.scn.b64, --deadband --stall-level=0.10 --noise-sigma-pct=0.05
// --steer-noise-sigma-pct=0.05 --noise-seed=1, seed 1): the RAW (zero CSV-
// clock-offset) along-track lag is dominated by a ~0.54s CONSTANT anchor
// (traced to mars_sim_viz's kHandshakeReadinessDelayMs, an offline-logging
// artifact -- see this task's report -- NOT a control-loop issue: the live
// GUI overlay and every mpc_controller's own traj_start_time share the SAME
// anchor, so this gap never appears on the wire). Once that constant is fit
// away (the existing per-window offset-fit acceptance metric), the GENUINE
// along-track tracking error at baseline is already small -- mean
// 0.004-0.008m, max 0.07-0.20m across all 3 robots -- i.e. the gaps this
// boost needs to close are a few cm to ~0.2m, not meters. K=1.0 closes a
// realistic ~0.1m gap in about one second without the extra overshoot risk
// a much higher gain (2.0: 0.1m -> +0.2 m/s) would carry for comparatively
// little benefit at this error scale.
constexpr double kCatchupGain = 1.0;

// SCHEDULE CATCH-UP: e_lag is held at 0.0 (see the control loop below) for
// this many seconds after a REFERENCE STOP EVENT ends, in addition to the
// whole event itself -- a hold's own waypoints do not move, so along-track
// lag computed against them mid-hold is meaningless, and briefly after
// resuming the reading is dominated by the hold/relaunch transient itself
// (e.g. LAUNCH GOVERNOR breakaway dynamics), not genuine schedule slippage
// worth boosting for. Fixed by this task's brief, not tuned.
constexpr double kCatchupHoldGraceS = 1.0;

// POSITION-AWARE COMPLETION (see RobotSpec::settle_position_tolerance's doc
// comment and LaunchGovernor.h's CORRECTION-LAUNCH paragraph for the full
// "why" -- USER-VISIBLE SYMPTOM this exists to fix: robots parking short of
// a reference pose and never finishing). Once the trajectory's own nominal
// completion window (points.back().time + 0.3s + finish_waiting's 1.0s) has
// passed, the control loop keeps running -- instead of immediately
// hard-stopping and exiting -- for up to this many ADDITIONAL seconds while
// position_error stays above spec.settle_position_tolerance and the LAUNCH
// GOVERNOR still has correction-launch attempts left for this stop event,
// giving a correction launch (or several, up to settle_max_attempts) a real
// chance to close the residual. Bounded (this cap, ANDed with the governor's
// own settle_max_attempts) so completion always still terminates -- never an
// unbounded wait. Whenever the robot is already within tolerance the instant
// the nominal window ends (the common, already-converged case), this adds
// ZERO latency: the very first check below is satisfied immediately, exactly
// matching the pre-existing unconditional-stop behavior.
constexpr double kCompletionExtraGraceCapS = 5.0;

// SCHEDULE CATCH-UP: true when rel_time falls inside any REFERENCE STOP
// EVENT in `events` (mpc::RefStopEvent -- see LaunchGovernor.h) or within
// `grace_s` seconds after one ends. Consumes the SAME ref_stop_events list
// the control loop already builds once at trajectory-arm time for the
// LAUNCH GOVERNOR's floor-yield guard (see that variable's own doc comment
// below) -- this is a second, independent consumer of it, not a fresh scan
// of the trajectory.
bool ref_in_or_near_stop_event(const std::vector<mpc::RefStopEvent>& events, double rel_time,
                                double grace_s) {
    for (const auto& ev : events) {
        if (rel_time >= ev.start_time && rel_time <= ev.end_time + grace_s) {
            return true;
        }
    }
    return false;
}

// Base64 ASCII encoder matching vesc_zmq_sender / dummy_straight_mover.
std::string encodeAscii(double value) {
    std::ostringstream oss;
    oss << std::setprecision(16) << value;
    return base64_encode(oss.str());
}

void publish_ackermann(zmq::socket_t& vesc_pub, const std::string& topic, double speed,
                        double steering, double accel) {
    nlohmann::json payload;
    payload["speed"] = encodeAscii(speed);
    payload["steering"] = encodeAscii(steering);
    payload["accel"] = encodeAscii(accel);
    std::string payload_str = payload.dump();

    zmq::message_t topic_msg(topic.begin(), topic.end());
    zmq::message_t payload_msg(payload_str.begin(), payload_str.end());

    vesc_pub.send(topic_msg, zmq::send_flags::sndmore);
    vesc_pub.send(payload_msg, zmq::send_flags::none);
}

struct PoseState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// FEATURE 2A: SIGINT/SIGTERM must still perform the stop-burst shutdown
// (brake to v=0 over the loop's normal rate before exiting), whether or not
// a pause is active -- so the loop checks a flag instead of exiting from
// inside the handler.
volatile std::sig_atomic_t g_shutdown_requested = 0;
void handle_shutdown_signal(int) { g_shutdown_requested = 1; }

// Interruptible/EINTR-safe blocking recv for the pre-START handshake
// (trajectory + START messages). Plain blocking recv() on a signal-delivery
// EINTR throws an uncaught zmq::error_t (cppzmq only swallows EAGAIN, see
// zmq.hpp's recv() -- EINTR rethrows), which aborts the process via
// std::terminate() instead of exiting cleanly. `rep_socket` has RCVTIMEO set
// (see main()) so a plain timeout also surfaces the same way EAGAIN would
// (empty result, no exception) -- this loop just keeps waiting on that,
// while treating a genuine EINTR (or any other transient zmq::error_t)
// identically: log and keep waiting, checking g_shutdown_requested each
// pass so a real SIGINT/SIGTERM unblocks promptly instead of hanging.
// Returns false only when shutdown was requested (caller should exit
// without publishing anything -- the VESC socket doesn't exist yet at this
// point in main()).
bool recv_handshake_or_shutdown(zmq::socket_t& sock, zmq::message_t& out) {
    while (!g_shutdown_requested) {
        try {
            auto result = sock.recv(out, zmq::recv_flags::none);
            if (result) {
                return true;
            }
            // RCVTIMEO elapsed (EAGAIN) -- loop back and re-check the
            // shutdown flag.
        } catch (const zmq::error_t& ex) {
            if (g_shutdown_requested) {
                return false;
            }
            // EINTR from an unrelated/spurious signal, or a transient
            // socket error -- log and keep waiting rather than crashing.
            std::cerr << "[MPC handshake] recv error (retrying): " << ex.what() << std::endl;
        }
    }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    std::string robot_name = "robot1";
    int handshake_port = 11111;
    std::string vesc_endpoint = "tcp://127.0.0.1:3161";
    std::string localization_endpoint = "tcp://127.0.0.1:3261";
    // --robot-spec=<path>: base geometry/hardware-limit/MPC-weight overrides
    // (mpc::load_spec_from_json's "robot"/"mpc" sections), shared across a
    // robot model/fleet. --hw-calibration=<path>: applied AFTER
    // --robot-spec, on top of it, for per-physical-unit tweaks (e.g. one
    // robot's actual max_accel or vel_scale differs slightly from the fleet
    // default) -- same JSON schema, loaded through the same
    // load_spec_from_json() so only the keys actually present in each file
    // override anything. Either or both may be omitted; omitting both keeps
    // the compiled RobotSpec::defaults()/MpcParams::defaults() byte-for-byte
    // (see RobotSpec.h).
    std::string robot_spec_path;
    std::string hw_calibration_path;

    // Parse options
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--robot" && i + 1 < argc) {
            robot_name = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            handshake_port = std::stoi(argv[++i]);
        } else if (arg == "--vesc-endpoint" && i + 1 < argc) {
            vesc_endpoint = argv[++i];
        } else if (arg == "--localization-endpoint" && i + 1 < argc) {
            localization_endpoint = argv[++i];
        } else if (arg == "--robot-spec" && i + 1 < argc) {
            robot_spec_path = argv[++i];
        } else if (arg.rfind("--robot-spec=", 0) == 0) {
            robot_spec_path = arg.substr(std::string("--robot-spec=").size());
        } else if (arg == "--hw-calibration" && i + 1 < argc) {
            hw_calibration_path = argv[++i];
        } else if (arg.rfind("--hw-calibration=", 0) == 0) {
            hw_calibration_path = arg.substr(std::string("--hw-calibration=").size());
        }
    }

    std::cout << "[MPC " << robot_name << "] Launching with HandshakePort=" << handshake_port
              << ", VESC=" << vesc_endpoint << ", Localization=" << localization_endpoint
              << std::endl;

    std::signal(SIGINT, handle_shutdown_signal);
    std::signal(SIGTERM, handle_shutdown_signal);

    // ZeroMQ context
    zmq::context_t context(1);

    // 1. MARS Handshake Layer (REP socket)
    zmq::socket_t rep_socket(context, zmq::socket_type::rep);
    rep_socket.set(zmq::sockopt::linger, 0);
    // RCVTIMEO makes the two blocking handshake recvs below (trajectory,
    // START) wake up periodically instead of blocking forever, so
    // recv_handshake_or_shutdown() can re-check g_shutdown_requested. Chosen
    // short enough to exit promptly on SIGINT/SIGTERM, long enough to not
    // busy-loop.
    rep_socket.set(zmq::sockopt::rcvtimeo, 200);
    std::string rep_addr = "tcp://*:" + std::to_string(handshake_port);
    try {
        rep_socket.bind(rep_addr);
    } catch (const std::exception& ex) {
        std::cerr << "[MPC " << robot_name << "] Failed to bind to REP port: " << ex.what()
                  << std::endl;
        return 1;
    }

    // Phase 1 Handshake: Receive Trajectory
    zmq::message_t traj_msg;
    std::cout << "[MPC " << robot_name << "] Waiting for trajectory..." << std::endl;
    if (!recv_handshake_or_shutdown(rep_socket, traj_msg)) {
        std::cout << "[MPC " << robot_name
                  << "] Shutdown signal received while waiting for trajectory. Exiting."
                  << std::endl;
        return 0;
    }
    std::string traj_encoded(static_cast<char*>(traj_msg.data()), traj_msg.size());

    // Decode and deserialize trajectory
    std::string traj_decoded = base64_decode(traj_encoded);
    ReloPush::trajectory ref_traj;
    ref_traj.deserialize(traj_decoded);
    std::cout << "[MPC " << robot_name << "] Received trajectory with "
              << ref_traj.trajectory_points->size() << " waypoints. Sending ACK..." << std::endl;

    std::string ack_receive = "ACK_RECEIVE_" + robot_name;
    zmq::message_t ack_recv_msg(ack_receive.begin(), ack_receive.end());
    rep_socket.send(ack_recv_msg, zmq::send_flags::none);

    // Phase 2 Handshake: Receive START
    zmq::message_t start_msg;
    std::cout << "[MPC " << robot_name << "] Waiting for START signal..." << std::endl;
    if (!recv_handshake_or_shutdown(rep_socket, start_msg)) {
        std::cout << "[MPC " << robot_name
                  << "] Shutdown signal received while waiting for START. Exiting." << std::endl;
        return 0;
    }
    std::string start_cmd(static_cast<char*>(start_msg.data()), start_msg.size());
    std::cout << "[MPC " << robot_name << "] Command received: " << start_cmd
              << ". Starting motion..." << std::endl;

    std::string ack_start = "ACK_START_" + robot_name;
    zmq::message_t ack_start_msg(ack_start.begin(), ack_start.end());
    rep_socket.send(ack_start_msg, zmq::send_flags::none);

    // 2. VESC Publisher (PUB socket)
    zmq::socket_t vesc_pub(context, zmq::socket_type::pub);
    vesc_pub.set(zmq::sockopt::linger, 0);
    try {
        vesc_pub.connect(vesc_endpoint);
        std::cout << "[MPC " << robot_name << "] VESC publisher connected to " << vesc_endpoint
                  << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[MPC " << robot_name << "] Failed to connect VESC publisher: " << ex.what()
                  << std::endl;
        return 1;
    }

    // 3. Localization Subscriber (SUB socket)
    zmq::socket_t loc_sub(context, zmq::socket_type::sub);
    loc_sub.set(zmq::sockopt::linger, 0);
    try {
        loc_sub.connect(localization_endpoint);
        std::string loc_topic = "/" + robot_name + "/localization";
        loc_sub.set(zmq::sockopt::subscribe, loc_topic);
        std::cout << "[MPC " << robot_name << "] Subscribed to localization topic: " << loc_topic
                  << " at " << localization_endpoint << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "[MPC " << robot_name
                  << "] Failed to connect localization subscriber: " << ex.what() << std::endl;
        return 1;
    }

    // Robot geometry/limits and MPC weights -- compiled defaults are
    // byte-identical to the literals that used to be hardcoded in this file.
    // --robot-spec=<path> overrides the fleet-shared baseline;
    // --hw-calibration=<path> is layered on top for a specific physical
    // unit. Both are optional; omitting either (or both) leaves the
    // corresponding fields at their compiled defaults -- see
    // load_spec_from_json()'s doc comment (only keys actually present in
    // each file are overridden).
    mpc::RobotSpec robot_spec = mpc::RobotSpec::defaults();
    mpc::MpcParams mpc_params = mpc::MpcParams::defaults();
    if (!robot_spec_path.empty()) {
        try {
            mpc::load_spec_from_json(robot_spec_path, robot_spec, mpc_params);
            std::cout << "[MPC " << robot_name << "] Loaded robot spec from " << robot_spec_path
                      << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "[MPC " << robot_name << "] Failed to load --robot-spec '"
                       << robot_spec_path << "': " << ex.what() << ". Using compiled defaults."
                       << std::endl;
        }
    }
    if (!hw_calibration_path.empty()) {
        try {
            mpc::load_spec_from_json(hw_calibration_path, robot_spec, mpc_params);
            std::cout << "[MPC " << robot_name << "] Applied hw calibration from "
                      << hw_calibration_path << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "[MPC " << robot_name << "] Failed to load --hw-calibration '"
                       << hw_calibration_path << "': " << ex.what() << ". Ignoring." << std::endl;
        }
    }
    {
        const std::string spec_err = robot_spec.validate();
        if (!spec_err.empty()) {
            std::cerr << "[MPC " << robot_name << "] WARNING: " << spec_err << std::endl;
        }
    }

    // State trackers. est_state is the controller's own real estimate of
    // (x, y, yaw, v): pose/velocity are both continuously updated from
    // feedback (fresh localization -> finite-differenced velocity; stale
    // localization -> dead-reckoned forward using the REAL last-commanded
    // acceleration), never from an open-loop assumption about what the
    // vehicle "should" be doing. See estimate below / MpcCore.h.
    mpc::State4 est_state{0.0, 0.0, 0.0, 0.0};
    PoseState current_pose;
    double last_cmd_a = 0.0;
    double last_cmd_delta = 0.0;

    // Previous localization SAMPLE (not tick) used to finite-difference a
    // real velocity estimate -- see the "got_new_pose_this_tick" handling
    // below. Only updated when a genuinely NEW pose arrives, so consecutive
    // control-loop ticks that see the same still-fresh pose (loop runs
    // faster than the localization publish rate) never spuriously
    // recompute v as zero.
    bool have_prev_meas_pose = false;
    PoseState prev_meas_pose;
    std::chrono::steady_clock::time_point prev_meas_time;

    // Model-prior observer's own velocity estimate (see kVelocityObsGain's
    // doc comment above for the rationale) -- est_state.v is ALWAYS set to
    // this value, never to a raw finite-difference sample directly, whichever
    // branch below updates it (fresh sample, held-over-fresh, or dead-
    // reckoning), so this is the ONE place "the current velocity estimate"
    // is defined; nothing outside this loop needs to know the observer
    // exists.
    bool have_v_est = false;
    double v_est = 0.0;

    // FAST EXIT / OBSERVER INJECTION (see LaunchGovernor.h's step() doc
    // comment): the latest RAW finite-difference sample -- i.e. THIS loop's
    // `v_meas` local below, held over across ticks -- BEFORE the model-prior
    // observer's predict/correct blending above touches it. Plumbed into
    // launch_governor.step() every tick (fresh or not -- see
    // v_raw_fresh_this_tick below) so its 2-consecutive-sample debounce can
    // exit LAUNCH fast, and read back out on just_exited_launch to one-shot
    // inject v_est/est_state.v -- stall transitions are discrete events with
    // strong measurement evidence, unlike ordinary cruise (which keeps the
    // low-gain kVelocityObsGain correction instead).
    bool have_raw_v = false;
    double last_raw_v = 0.0;

    auto last_loc_time = std::chrono::steady_clock::now();
    bool has_localization = false;

    double traj_start_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

    // REFERENCE STOP EVENTS (see mpc::extract_ref_stop_events/
    // mpc::time_to_next_ref_stop in LaunchGovernor.h, and
    // LaunchGovernor::kFloorYieldHorizonS's doc comment for the full "why"):
    // precomputed ONCE here, at trajectory-arm time, from the waypoints this
    // process just received. The per-tick LAUNCH GOVERNOR call below only
    // ever does an O(log events) lookup against the result
    // (ref_stop_events), never a fresh O(waypoints) scan.
    //
    // extract_ref_stop_events wants an EFFECTIVE per-segment velocity, NOT
    // the raw trajectory_elem::ref_vel label used everywhere else in this
    // file (e.g. just below, and in the control loop's own
    // get_ref_state_at_time calls): that label is a per-LEG NOMINAL speed
    // (RobotTrajectoryBuilder.h: dir_sign * speed_transfer/speed_transit),
    // constant for as long as a leg lasts, and it NEVER decelerates on its
    // own -- across an INTERMEDIATE HOLD specifically, it jumps directly
    // from one leg's nonzero label to the next leg's (possibly
    // differently-signed) nonzero label without ever passing through zero.
    // Observed directly in this task's own acceptance scenario
    // (results/sim_handoff/greedy.scn.b64, robot1's ~9.5s rendezvous hold at
    // ref-time ~172.6-182.1): the label jumps +0.2 -> -0.15 while the
    // waypoints' actual (x,y) position does not move at all across the
    // hold. The label is faithful for the velocity-TRACKING cost (its only
    // other use in this file) but useless for DETECTING a hold. Position is
    // always faithful, by contrast: derive each segment's effective
    // velocity from consecutive waypoints' real (x,y) displacement over
    // their time gap instead, and let extract_ref_stop_events (a generic,
    // trajectory-agnostic function -- see its own doc comment for why it
    // lives in LaunchGovernor.h/.cpp rather than here) do the actual
    // hold-run scan against THAT derived array.
    std::vector<double> ref_stop_times;
    std::vector<double> ref_stop_vels;
    {
        const auto& arm_points = *(ref_traj.trajectory_points);
        ref_stop_times.reserve(arm_points.size());
        ref_stop_vels.reserve(arm_points.size());
        for (size_t i = 0; i < arm_points.size(); ++i) {
            ref_stop_times.push_back(static_cast<double>(arm_points[i].time));
            double seg_v = 0.0;
            if (i + 1 < arm_points.size()) {
                const double seg_dt = static_cast<double>(arm_points[i + 1].time) -
                                       static_cast<double>(arm_points[i].time);
                if (seg_dt > 1e-9) {
                    const double dx = static_cast<double>(arm_points[i + 1].x) -
                                       static_cast<double>(arm_points[i].x);
                    const double dy = static_cast<double>(arm_points[i + 1].y) -
                                       static_cast<double>(arm_points[i].y);
                    seg_v = std::hypot(dx, dy) / seg_dt;
                }
                // else: degenerate (near-zero-duration) segment -- left at
                // 0.0, harmless either way since extract_ref_stop_events
                // independently skips any segment this short by duration
                // alone (see its own doc comment) and never reads this
                // value for it.
            }
            // else (last waypoint): no outgoing segment -- this entry is
            // never read (extract_ref_stop_events only consumes indices
            // [0, size-2] as segment sources); left at 0.0 for a
            // well-defined vector regardless.
            ref_stop_vels.push_back(seg_v);
        }
    }
    const std::vector<mpc::RefStopEvent> ref_stop_events =
        mpc::extract_ref_stop_events(ref_stop_times, ref_stop_vels);

    const std::string ackermann_topic = "/" + robot_name + "/ackermann";
    mpc::MpcSolver solver;
    // FEATURE 2A (warm start): a brand-new trajectory run must never seed
    // its first solve from a stale warm-start buffer. `solver` is freshly
    // constructed right above (so this is a no-op today, since this process
    // only ever runs one trajectory per lifetime), but call it explicitly --
    // matching the RESUME call site below -- so the invariant holds even if
    // a future reconnect/re-arm loop reuses this MpcSolver across runs.
    solver.reset_warm_start();

    // LAUNCH GOVERNOR (see mpc/LaunchGovernor.h). Constructed fresh here (so
    // reset() below is a no-op today), called explicitly for the same
    // future-proofing reason as solver.reset_warm_start() just above.
    mpc::LaunchGovernor launch_governor;
    launch_governor.reset();
    // Wall-clock seconds since the governor's CURRENT state (NORMAL or
    // LAUNCH) was entered. The governor itself never touches the clock (see
    // LaunchGovernor.h's doc comment) -- this loop recomputes the elapsed
    // value every tick from `current_time` (the SAME wall clock already
    // used for the PAUSE/RESUME bookkeeping below) and resets the epoch
    // whenever launch_governor.state() actually changes (see the bottom of
    // the control loop).
    mpc::LaunchGovernorState gov_prev_state = launch_governor.state();
    double gov_state_entered_time = traj_start_time;
    // Actual previous published v (unscaled model domain). Fed INTO
    // launch_governor.step() every tick (the LAUNCH ramp advances from this,
    // not from any state internal to the governor -- see LaunchGovernor.h's
    // doc comment) and used as the UNIFIED accel anchor for every governor
    // override below (LAUNCH ramp step or STOP-SNAP alike). Updated
    // unconditionally at the end of every tick, governed or transparent.
    double last_published_v = 0.0;

    // POSITION-AWARE COMPLETION (see kCompletionExtraGraceCapS's doc
    // comment): wall-clock time the loop first observed
    // past_nominal_completion_window==true, latched on that first tick and
    // never reset (once the nominal window has passed, reference_time only
    // ever advances further past it) -- negative means "not yet observed."
    double completion_grace_start_time = -1.0;

    // FEATURE 2A: PAUSE/RESUME state. `paused`/`pause_start` track the
    // CURRENT pause (pause_start only meaningful while paused); `pause_offset`
    // accumulates the total wall-clock duration of all COMPLETED pauses so
    // far. Both start at their identity values, so mpc::compute_effective_elapsed
    // is a no-op for a controller that is never paused (see the
    // pause_offset != 0.0 guard below, which additionally guarantees the
    // reference-time math is untouched bit-for-bit in that case).
    bool paused = false;
    double pause_start = 0.0;
    double pause_offset = 0.0;

    std::cout << "[MPC " << robot_name << "] Entering control loop..." << std::endl;

    // Main Control Loop at 20Hz (dt = 0.05s)
    while (true) {
        auto loop_start = std::chrono::steady_clock::now();
        double current_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                                   std::chrono::system_clock::now().time_since_epoch())
                                   .count();

        // FEATURE 2A: SIGINT/SIGTERM stop-burst shutdown. Applies whether or
        // not a pause is active -- decelerate smoothly (respecting
        // max_accel, keeping the accel==d(speed)/dt identity via
        // build_payload) rather than an abrupt slam-to-zero, then publish a
        // final hard-zero (matching the existing trajectory-completion stop
        // publish below) and exit.
        if (g_shutdown_requested) {
            while (std::fabs(est_state.v) > 1e-4) {
                mpc::StopAccelResult stop =
                    mpc::compute_stop_accel(est_state.v, robot_spec.max_accel, mpc_params.dt);
                mpc::PayloadValues stop_payload = mpc::build_payload(
                    stop.v_cmd, 0.0, stop.accel, mpc_params.vel_scale,
                    mpc_params.vel_scale_back, mpc_params.dir_change_scale, false);
                publish_ackermann(vesc_pub, ackermann_topic, stop_payload.speed,
                                   stop_payload.steering, stop_payload.accel);
                est_state.v = stop.v_cmd;
                last_cmd_a = stop.accel;
                last_cmd_delta = 0.0;
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(mpc_params.dt));
            }
            publish_ackermann(vesc_pub, ackermann_topic, 0.0, 0.0, 0.0);
            std::cout << "[MPC " << robot_name
                      << "] Shutdown signal received. Stopped and exiting." << std::endl;
            break;
        }

        // FEATURE 2A: poll the handshake REP socket for PAUSE/RESUME,
        // non-blocking, every tick. Replies must never block (dontwait +
        // try/catch, mirroring the existing zmq guards elsewhere in this
        // file).
        try {
            zmq::message_t cmd_msg;
            if (rep_socket.recv(cmd_msg, zmq::recv_flags::dontwait)) {
                std::string cmd(static_cast<char*>(cmd_msg.data()), cmd_msg.size());
                std::string reply;
                if (cmd == "PAUSE") {
                    if (!paused) {
                        paused = true;
                        pause_start = current_time;
                    }
                    // Idempotent: ack again if already paused.
                    reply = "ACK_PAUSE_" + robot_name;
                } else if (cmd == "RESUME") {
                    if (paused) {
                        paused = false;
                        pause_offset += (current_time - pause_start);
                        // Previous solution is stale after a stop -- the
                        // reference continues from where it was paused, and
                        // the MPC catches up within its accel bounds.
                        solver.reset_warm_start();
                        // LAUNCH GOVERNOR: the pause brake path bypasses the
                        // governor entirely (see LaunchGovernor.h's
                        // INTERLOCKS paragraph) while braking est_state.v to
                        // 0 -- a LAUNCH sojourn or escalation level latched
                        // from before the pause is now stale relative to
                        // that fresh zero, so reset it exactly like the warm
                        // start buffer just above.
                        launch_governor.reset();
                        gov_prev_state = launch_governor.state();
                        gov_state_entered_time = current_time;
                    }
                    // Idempotent: ack again if already running.
                    reply = "ACK_RESUME_" + robot_name;
                } else {
                    reply = "ERR unknown";
                }
                try {
                    zmq::message_t reply_msg(reply.begin(), reply.end());
                    rep_socket.send(reply_msg, zmq::send_flags::dontwait);
                } catch (const zmq::error_t& ex) {
                    std::cerr << "[MPC " << robot_name
                              << "] Failed to send handshake reply: " << ex.what() << std::endl;
                }
            }
        } catch (const zmq::error_t& ex) {
            std::cerr << "[MPC " << robot_name << "] Handshake poll error: " << ex.what()
                      << std::endl;
        }

        // Pause-adjusted (frozen-during-pause) wall time used for BOTH the
        // reference lookup and the trajectory-completion check below. Only
        // recomputed once a pause has actually happened (pause_offset != 0.0
        // or paused is true) -- otherwise `reference_time` is `current_time`
        // verbatim, so a controller that is never paused is byte-for-byte
        // unchanged from before this feature existed.
        double reference_time = current_time;
        if (paused || pause_offset != 0.0) {
            double effective_elapsed = mpc::compute_effective_elapsed(
                current_time, traj_start_time, pause_offset, paused, pause_start);
            reference_time = traj_start_time + effective_elapsed;
        }

        // Check if trajectory is complete. Guard points.back() -- an empty
        // trajectory (no waypoints at all) has nothing to track or finish;
        // treat it as immediately complete instead of dereferencing an
        // empty vector's back().
        const auto& points = *(ref_traj.trajectory_points);
        if (points.empty()) {
            publish_ackermann(vesc_pub, ackermann_topic, 0.0, 0.0, 0.0);
            std::cerr << "[MPC " << robot_name
                      << "] Trajectory has no waypoints. Stopping and exiting." << std::endl;
            break;
        }
        double traj_end_time = traj_start_time + points.back().time + 0.3;
        double finish_waiting = 1.0;

        // POSITION-AWARE COMPLETION (see kCompletionExtraGraceCapS's doc
        // comment): past the nominal window, completion no longer fires
        // unconditionally here -- the actual decision (deferred while a
        // correction launch is still worth trying) happens below, once
        // this tick's fresh position_error is known from real localization
        // feedback (est_state.x/y, updated just below).
        const bool past_nominal_completion_window = reference_time > traj_end_time + finish_waiting;

        // Receive localization updates (non-blocking). Wrapped in try/catch
        // like the other recv sites in this file: a multipart message's
        // continuation frame(s) are read with a plain blocking recv() (the
        // first frame is already known to have "more" pending, so it should
        // arrive essentially immediately), which -- like the pre-START
        // handshake recvs -- would throw an uncaught zmq::error_t on EINTR
        // instead of just returning nothing. Treat that (or any other
        // transient recv error here) as "no new localization this tick"
        // rather than crashing; the dead-reckoning fallback below already
        // handles missing localization.
        zmq::message_t msg;
        bool got_msg = false;
        std::string last_payload;

        try {
            while (loc_sub.recv(msg, zmq::recv_flags::dontwait)) {
                got_msg = true;
                std::string part(static_cast<char*>(msg.data()), msg.size());
                while (msg.more()) {
                    auto more_result = loc_sub.recv(msg, zmq::recv_flags::none);
                    if (!more_result) {
                        break;
                    }
                    part = std::string(static_cast<char*>(msg.data()), msg.size());
                }
                last_payload = part;
            }
        } catch (const zmq::error_t& ex) {
            std::cerr << "[MPC " << robot_name << "] Localization recv error (skipping tick): "
                      << ex.what() << std::endl;
        }

        bool got_new_pose_this_tick = false;
        // FAST EXIT (see have_raw_v's doc comment above): true only when
        // THIS tick actually computes a fresh, trustworthy raw finite
        // difference (got_new_pose_this_tick AND have_dt_meas below) --
        // freshly false every loop iteration, exactly like
        // got_new_pose_this_tick itself.
        bool v_raw_fresh_this_tick = false;
        if (got_msg) {
            try {
                if (!last_payload.empty() && last_payload.front() == '{') {
                    auto json_data = nlohmann::json::parse(last_payload);
                    current_pose.x = json_data.at("x").get<double>();
                    current_pose.y = json_data.at("y").get<double>();
                    current_pose.yaw = json_data.at("yaw").get<double>();
                    last_loc_time = std::chrono::steady_clock::now();
                    has_localization = true;
                    got_new_pose_this_tick = true;
                }
            } catch (...) {
                // Ignore parsing errors for non-json or routing headers
            }
        }

        // Check if localization is available
        auto now = std::chrono::steady_clock::now();
        double elapsed_sec = std::chrono::duration<double>(now - last_loc_time).count();

        if (got_new_pose_this_tick) {
            // TRUE velocity feedback: finite-difference this new sample
            // against the previous one, projected onto the CURRENT heading
            // so a genuine reversal yields a negative estimate (matching
            // mpc::RefState::ref_vel's sign convention). The first-ever
            // sample has nothing to difference against, so it keeps
            // whatever velocity estimate est_state already had (0.0 at
            // startup).
            double v_meas = est_state.v;
            double dt_meas = 0.0;
            bool have_dt_meas = false;
            if (have_prev_meas_pose) {
                dt_meas = std::chrono::duration<double>(now - prev_meas_time).count();
                if (dt_meas > 1e-3) {
                    double dx = current_pose.x - prev_meas_pose.x;
                    double dy = current_pose.y - prev_meas_pose.y;
                    v_meas = (dx * std::cos(current_pose.yaw) + dy * std::sin(current_pose.yaw)) /
                             dt_meas;
                    have_dt_meas = true;
                }
                // else: samples too close together in time to trust a
                // finite difference -- keep the previous estimate.
            }
            prev_meas_pose = current_pose;
            prev_meas_time = now;
            have_prev_meas_pose = true;

            // FAST EXIT / OBSERVER INJECTION: hold over this tick's raw
            // sample (see have_raw_v's doc comment above) ONLY when
            // have_dt_meas is true -- the same trustworthiness bar the
            // model-prior observer's own CORRECT step uses below, so the
            // governor's debounce never advances on an untrustworthy
            // (too-close-together-in-time) v_meas either.
            if (have_dt_meas) {
                last_raw_v = v_meas;
                have_raw_v = true;
                v_raw_fresh_this_tick = true;
            }

            est_state.x = current_pose.x;
            est_state.y = current_pose.y;
            est_state.yaw = current_pose.yaw;

            // MODEL-PRIOR PREDICT/CORRECT (see kVelocityObsGain's doc
            // comment for the full rationale/data). The very first-ever
            // sample has no prior estimate to predict from (and no
            // trustworthy dt_meas either -- have_dt_meas is false on it), so
            // it seeds v_est directly instead, exactly as the EMA's own
            // first-sample seed did.
            if (!have_v_est) {
                v_est = v_meas;
                have_v_est = true;
            } else {
                // PREDICT: advance the PREVIOUS estimate using the REAL
                // last-commanded acceleration over one control period --
                // the same v'=v+a*dt model mpc::rollout_step uses elsewhere
                // in this file, carrying phase with effectively zero lag.
                const double v_pred = v_est + last_cmd_a * mpc_params.dt;
                if (have_dt_meas) {
                    // CORRECT: a small, flat (not dt_meas-scaled) nudge
                    // toward the fresh raw measurement -- the model half
                    // above already carries timing, so this only needs to
                    // catch what the model can't know about (actuation
                    // error, deadband stalls/snaps), not carry phase.
                    v_est = v_pred + kVelocityObsGain * (v_meas - v_pred);
                } else {
                    // dt_meas untrustworthy this sample -- nothing new to
                    // correct with, so keep the model's own prediction.
                    v_est = v_pred;
                }
            }
            est_state.v = v_est;
        } else if (has_localization && elapsed_sec <= 0.1) {
            // Localization is still fresh but this tick didn't carry a NEW
            // sample (control loop runs faster than the localization
            // publish rate) -- PREDICT ONLY (no fresh measurement to
            // correct with this tick): advance the model-prior estimate by
            // one control period on the same known last-commanded
            // acceleration the "meas" branch above uses, so the observer's
            // own phase never stalls between fresh samples the way the old
            // EMA's plain hold did.
            if (have_v_est) {
                v_est = v_est + last_cmd_a * mpc_params.dt;
                est_state.v = v_est;
            }
        } else {
            // Dead-reckoning fallback -- roll forward using the REAL last
            // commanded acceleration (a is this formulation's direct
            // control input; unlike the old velocity-formulation's forced
            // a=0, this actually keeps integrating the believed velocity).
            est_state = mpc::rollout_step(est_state, last_cmd_a, last_cmd_delta,
                                           robot_spec.wheel_base, mpc_params.dt);
            // Keep the observer's own estimate in sync with this
            // dead-reckoned belief (this IS the same v'=v+a*dt predict model
            // the observer itself uses, just also advancing x/y/yaw) so a
            // LATER fresh localization sample resumes predict/correct from a
            // consistent baseline instead of blending against a stale
            // pre-dropout value.
            v_est = est_state.v;
            have_v_est = true;
        }

        // POSITION-AWARE COMPLETION (see kCompletionExtraGraceCapS's doc
        // comment): ref_now/position_error are computed here -- right after
        // est_state.x/y is fresh for this tick, and deliberately BEFORE the
        // `paused` branch below -- so this decision (and its use of
        // est_state, a pure function of real feedback/dead-reckoning, never
        // of the solve) is evaluated identically whether or not this tick
        // happens to be paused. ref_now is reused (not recomputed) by the
        // LAUNCH GOVERNOR call further below.
        //
        // position_error is held at 0.0 until has_localization is true (the
        // first real pose sample has been processed at least once): before
        // that, est_state.x/y is still its startup default (0,0,0,0), which
        // is almost never the robot's true spawn pose -- comparing it
        // against ref_now (the ACTUAL first waypoint's pose) produces a
        // spurious, arbitrarily large "error" that has nothing to do with
        // any real residual (empirically observed: this fired an unwanted
        // full-speed CORRECTION-LAUNCH kick on the very first control-loop
        // tick, before any localization had arrived, using ref_now's real
        // pose against est_state's still-default (0,0)). A 0.0 value can
        // never exceed a positive settle_position_tolerance, so this simply
        // suppresses both CORRECTION-LAUNCH entry and the position-aware
        // completion check until there is a trustworthy estimate to judge
        // either against -- exactly one control tick's delay in the
        // ordinary case (localization typically arrives within the first
        // tick or two), never a behavior change once has_localization is
        // (as it almost always is, past startup) true.
        mpc::RefState ref_now =
            mpc::get_ref_state_at_time(reference_time, traj_start_time, ref_traj);
        const double position_error =
            has_localization ? std::hypot(ref_now.x - est_state.x, ref_now.y - est_state.y) : 0.0;
        if (past_nominal_completion_window) {
            if (completion_grace_start_time < 0.0) {
                completion_grace_start_time = current_time;
            }
            const double extra_grace_elapsed = current_time - completion_grace_start_time;
            const bool within_tolerance = position_error <= robot_spec.settle_position_tolerance;
            const bool grace_cap_hit = extra_grace_elapsed >= kCompletionExtraGraceCapS;
            const bool attempts_exhausted =
                launch_governor.settle_attempts_this_stop_event() >= robot_spec.settle_max_attempts;
            if (within_tolerance || grace_cap_hit || attempts_exhausted) {
                if (!within_tolerance) {
                    std::cerr << "[MPC " << robot_name << "] WARNING: trajectory completion "
                              << (grace_cap_hit ? "extra-grace cap (" +
                                                       std::to_string(kCompletionExtraGraceCapS) + "s)"
                                                 : "correction-launch attempt cap")
                              << " reached with residual position_error=" << position_error
                              << "m (> settle_position_tolerance="
                              << robot_spec.settle_position_tolerance << "m)" << std::endl;
                }
                publish_ackermann(vesc_pub, ackermann_topic, 0.0, 0.0, 0.0);
                std::cout << "[MPC " << robot_name
                          << "] Trajectory completed. Stopping vehicle and exiting." << std::endl;
                break;
            }
            // Else: still worth trying -- fall through and run this tick
            // normally (solve + governor, possibly a correction launch).
        }

        // FEATURE 2A: while paused, skip the MPC solve entirely and publish
        // rate-limited stop payloads (brake to v=0, steering 0) at loop
        // rate -- keeps the sim watchdog fed and the accel==d(speed)/dt wire
        // identity intact. Localization/dead-reckoning above still ran this
        // tick, so pose tracking stays live for an accurate resume.
        if (paused) {
            mpc::StopAccelResult stop =
                mpc::compute_stop_accel(est_state.v, robot_spec.max_accel, mpc_params.dt);
            mpc::PayloadValues pause_payload = mpc::build_payload(
                stop.v_cmd, 0.0, stop.accel, mpc_params.vel_scale,
                mpc_params.vel_scale_back, mpc_params.dir_change_scale, false);
            est_state.v = stop.v_cmd;
            // Keep the observer's baseline in sync (same rationale as the
            // dead-reckoning branch above) so a fresh sample after RESUME
            // predicts/corrects from this forced-braking belief, not a stale
            // pre-pause value.
            v_est = est_state.v;
            have_v_est = true;
            last_cmd_a = stop.accel;
            last_cmd_delta = 0.0;
            publish_ackermann(vesc_pub, ackermann_topic, pause_payload.speed,
                               pause_payload.steering, pause_payload.accel);

            auto loop_end = std::chrono::steady_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
            int sleep_ms = 50 - static_cast<int>(duration.count());
            if (sleep_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            }
            continue;
        }

        // Delay compensation (predict control_delay_steps steps ahead;
        // defaults to 1, matching the original single-step prediction), now
        // rolled forward from the REAL velocity estimate using the REAL
        // last-commanded acceleration -- both genuine feedback/control
        // quantities in the acceleration formulation, not the old
        // always-a=0 placeholder.
        mpc::State4 pred4 = mpc::predict_delay_compensated(
            est_state, last_cmd_a, last_cmd_delta, robot_spec.wheel_base, mpc_params.dt,
            mpc_params.control_delay_steps);

        // Determine weights and speeds. Uses reference_time (pause-adjusted;
        // == current_time verbatim whenever the controller has never been
        // paused) so a live pause freezes reference lookups too.
        double target_time = reference_time + mpc_params.dt;
        mpc::RefState ref = mpc::get_ref_state_at_time(target_time, traj_start_time, ref_traj);

        // Uses the REAL (delay-compensated) velocity estimate, not an
        // open-loop last-commanded value, so this reacts to the vehicle's
        // true state through a reversal.
        mpc::DirChangeResolution dir_change = mpc::resolve_dir_change(
            pred4.v, ref.ref_vel, mpc_params.w_dist, mpc_params.w_dist_dirchange);

        // SCHEDULE CATCH-UP (see mpc::boosted_vr's doc comment in MpcCore.h,
        // and kCatchupGain's doc comment above, for the full mechanism):
        // e_lag is the CURRENT signed along-track lag -- est_state (the
        // controller's own real, continuously-updated pose belief, NOT
        // pred4's delay-compensated prediction) vs. the reference's CURRENT
        // (reference_time, deliberately "right now" like ref_now below, not
        // target_time) interpolated pose/heading -- held at 0.0 (no boost/
        // ease-off) whenever the reference is inside a REFERENCE STOP EVENT
        // or within kCatchupHoldGraceS after one (ref_in_or_near_stop_event
        // above).
        const double catchup_rel_time = reference_time - traj_start_time;
        double e_lag = 0.0;
        if (!ref_in_or_near_stop_event(ref_stop_events, catchup_rel_time, kCatchupHoldGraceS)) {
            mpc::RefState catchup_ref =
                mpc::get_ref_state_at_time(reference_time, traj_start_time, ref_traj);
            e_lag = (catchup_ref.x - est_state.x) * std::cos(catchup_ref.yaw) +
                    (catchup_ref.y - est_state.y) * std::sin(catchup_ref.yaw);
        }

        std::vector<mpc::RefState> horizon_refs;
        horizon_refs.reserve(mpc::kHorizon);
        for (int t = 0; t < mpc::kHorizon; ++t) {
            double h_time = reference_time + (t + 1) * mpc_params.dt;
            mpc::RefState stage_ref = mpc::get_ref_state_at_time(h_time, traj_start_time, ref_traj);
            // Boost/ease-off this stage's velocity TARGET (never the
            // solve's own decision -- see MpcCore.h) toward closing e_lag,
            // capped at this stage's own push/non-push speed limit -- the
            // SAME cap MPCCostFunctor's clamp_sym already enforces inside
            // the solve, so this can only ever ask for what the solve was
            // already willing to give under ideal tracking, never more.
            const double stage_vmax =
                stage_ref.is_pushing ? robot_spec.max_v_push : robot_spec.max_v_nonpush;
            stage_ref.ref_vel = mpc::boosted_vr(stage_ref.ref_vel, e_lag, kCatchupGain, stage_vmax);
            horizon_refs.push_back(stage_ref);
        }

        mpc::SolveInputs solve_in;
        solve_in.x0 = pred4.x;
        solve_in.y0 = pred4.y;
        solve_in.yaw0 = pred4.yaw;
        solve_in.v0 = pred4.v;
        solve_in.last_cmd_a = last_cmd_a;
        solve_in.last_cmd_delta = last_cmd_delta;
        solve_in.horizon_refs = horizon_refs;
        solve_in.wheel_base = robot_spec.wheel_base;
        solve_in.dt = mpc_params.dt;
        solve_in.w_dist = dir_change.w_dist;
        solve_in.w_yaw = mpc_params.w_yaw;
        solve_in.w_vel = mpc_params.w_vel;
        solve_in.w_lat = mpc_params.w_lat;
        solve_in.w_control = mpc_params.w_control;
        solve_in.w_delta_rate = mpc_params.w_delta_rate;
        solve_in.max_v_push = robot_spec.max_v_push;
        solve_in.max_v_nonpush = robot_spec.max_v_nonpush;
        solve_in.max_steer = robot_spec.max_steer;
        solve_in.max_accel = robot_spec.max_accel;
        // FEATURE 2A (warm start): the live control loop opts in to
        // shift-seeded warm starting (see MpcCore.h's MpcSolver doc) for
        // solve speed and temporal consistency between consecutive 20Hz
        // ticks. The buffer this seeds from is invalidated on RESUME and on
        // a newly armed trajectory (see the reset_warm_start() call sites),
        // so a stale/pre-pause solution is never carried forward.
        solve_in.use_warm_start = true;

        mpc::MpcOutput solve_out = solver.solve(solve_in);
        double opt_v = solve_out.v_cmd;
        double opt_delta = solve_out.steer;
        double opt_accel = solve_out.accel;

        // LAUNCH GOVERNOR (see mpc/LaunchGovernor.h): may override opt_v
        // (never opt_delta -- steering always stays exactly what the MPC
        // solved). ref_vel_now is looked up at reference_time itself, NOT
        // target_time (one dt further ahead, already used to drive the
        // solve above) -- the governor's STOP-SNAP/MOVING-floor checks
        // specifically want "what does the reference want RIGHT NOW",
        // deliberately different from the horizon the solve tracks.
        // last_published_v (the actual previous wire value, tracked below)
        // is the accel-derivation anchor for every override -- see
        // LaunchGovernor.h's doc comment. v_raw_fresh_this_tick/last_raw_v
        // (see have_raw_v's doc comment above) feed the FAST EXIT debounce;
        // left at their loop-scoped values (false/whatever was last held)
        // on ticks with no fresh raw sample, exactly per step()'s own
        // doc comment on that parameter. ref_now/position_error were
        // already computed earlier this tick (right after localization
        // processing, ahead of the `paused` branch -- see
        // kCompletionExtraGraceCapS's doc comment); reused verbatim here,
        // not recomputed, so the completion check and the governor's own
        // CORRECTION-LAUNCH entry see the identical position_error value.
        double gov_wall_elapsed_in_state = current_time - gov_state_entered_time;
        // FLOOR YIELD NEAR A REFERENCE STOP EVENT (see LaunchGovernor.h's
        // kFloorYieldHorizonS doc comment, and mpc::time_to_next_ref_stop's
        // own doc comment): seconds from THIS tick's reference-lookup
        // instant (reference_time, pause-adjusted, the same clock ref_now
        // was just looked up at) to the start of the EARLIEST upcoming
        // REFERENCE STOP EVENT -- an intermediate hold or the trajectory's
        // own end, whichever comes first -- via an O(log events) lookup
        // against ref_stop_events (precomputed once at arm time above, NOT
        // rescanned here). rel_reference_time converts reference_time into
        // the SAME trajectory-relative time base ref_stop_events/
        // ref_stop_times were built in (traj_start_time-subtracted),
        // mirroring get_ref_state_at_time's own `abs_time - traj_start_time`
        // internal conversion.
        const double rel_reference_time = reference_time - traj_start_time;
        const double time_to_ref_stop =
            mpc::time_to_next_ref_stop(ref_stop_events, rel_reference_time);
        mpc::LaunchGovernorDecision gov_decision = launch_governor.step(
            est_state.v, opt_v, ref_now.ref_vel, last_published_v, robot_spec, mpc_params.dt,
            gov_wall_elapsed_in_state, v_raw_fresh_this_tick, last_raw_v, time_to_ref_stop,
            position_error);
        if (gov_decision.override_active) {
            opt_v = gov_decision.v_published;
            // Anchor for every override (STOP-SNAP ramp, MOVING-floor ramp,
            // or a LAUNCH kick alike -- see LaunchGovernorDecision's doc
            // comment): a truthful accel so the wire's accel*dt ==
            // Delta(published speed) identity holds by construction. For
            // STOP-SNAP/MOVING-floor the anchor is the ACTUAL previous
            // published v, so this tick's accel is exactly +-max_accel
            // while still ramping (or exactly 0 once caught up) -- never
            // clamped or faked, never oscillating. For a LAUNCH kick tick
            // specifically (derive_accel_from_rest), the anchor is 0.0
            // instead -- i.e. EVERY tick spent in LAUNCH republishes the
            // FULL implied accel needed to reach the (possibly escalated)
            // target from rest, not just the first -- see
            // LaunchGovernorDecision's doc comment for why anchoring to
            // last_published_v here would silently drop to accel=0 (and so,
            // no further push at all) the instant v_published stops
            // changing tick to tick, well before breakaway is actually
            // confirmed. This accel may legitimately exceed spec.max_accel
            // by design (see RobotSpec::max_breakaway_accel) -- the one
            // documented exception to the norm every other override upholds.
            const double accel_anchor = gov_decision.derive_accel_from_rest ? 0.0 : last_published_v;
            opt_accel = (mpc_params.dt > 0.0) ? (opt_v - accel_anchor) / mpc_params.dt : 0.0;
        }
        // OBSERVER INJECTION (see have_raw_v's doc comment above): one-shot,
        // exactly on the tick LAUNCH exits (by either trigger) -- overwrite
        // the model-prior observer's own state with the latest raw sample
        // instead of waiting several ticks for kVelocityObsGain's own low
        // gain to believe it. Guarded by have_raw_v so a (pathological)
        // exit before any trustworthy raw sample was ever seen leaves the
        // observer untouched rather than injecting a meaningless 0.0.
        if (gov_decision.just_exited_launch && have_raw_v) {
            v_est = last_raw_v;
            est_state.v = v_est;
        }
        // Track the governor's wall-clock state epoch for next tick's
        // wall_elapsed_in_state (see that variable's doc comment above).
        if (launch_governor.state() != gov_prev_state) {
            gov_prev_state = launch_governor.state();
            gov_state_entered_time = current_time;
        }
        // Track the actual published v (unscaled model domain, matching
        // solve_in.v0's own domain) for next tick's governor call above --
        // updated unconditionally (governed or transparent) so it always
        // reflects the true previous publish.
        last_published_v = opt_v;

        mpc::PayloadValues payload = mpc::build_payload(
            opt_v, opt_delta, opt_accel, mpc_params.vel_scale,
            mpc_params.vel_scale_back, mpc_params.dir_change_scale, dir_change.change_dir);

        // Update tracking states. last_cmd_a is cached here for the NEXT
        // tick's dead-reckoning fallback, predict-only branch, model-prior
        // PREDICT step, and predict_delay_compensated() call (4 call sites
        // above, all under "TRUE velocity feedback"/"Dead-reckoning
        // fallback"/"Delay compensation") -- every one of those treats
        // last_cmd_a as a physically-REAL, SUSTAINED acceleration and
        // extrapolates state from it, so it must stay bounded by
        // spec.max_accel exactly like it always implicitly was before the
        // LAUNCH GOVERNOR's KICK existed (an earlier rate-limited-ramp
        // launch, and every other override/ordinary publish, never
        // commanded outside +-max_accel either). A LAUNCH KICK tick's
        // opt_accel is the one deliberate exception to that bound (see
        // LaunchGovernorDecision's doc comment) -- but that exception is
        // about what gets PUBLISHED ON THE WIRE (payload.accel, just built
        // above from the unclamped opt_accel), not what this controller
        // privately believes "really" happened kinematically. Feeding the
        // unclamped value into last_cmd_a was empirically observed (this
        // task's report) to desync the observer from the actual plant: the
        // very next tick's predict-only/dead-reckoning step would advance
        // v_est by the KICK's artificially large implied accel (e.g.
        // ~0.11 m/s in one 50ms tick from a 2.2 m/s^2 "accel") even though
        // the plant had not yet ACTUALLY broken away, which could (a) cross
        // the LAUNCH GOVERNOR's own kExitFraction fallback-exit bar on
        // v_meas=est_state.v before real feedback confirmed anything, and/or
        // (b) inflate the delay-compensated v0 the very next solve sees --
        // both provoking a hard, spurious brake command that re-stalled the
        // (still actually motionless) plant one tick later, a chained
        // stall-then-relaunch this test's own Part B/D moving-transition-
        // count assertions catch directly.
        last_cmd_a = std::clamp(opt_accel, -robot_spec.max_accel, robot_spec.max_accel);
        last_cmd_delta = opt_delta;

        // Package and send to VESC publisher
        publish_ackermann(vesc_pub, ackermann_topic, payload.speed, payload.steering,
                           payload.accel);

        // Synchronize loop speed to 20Hz (50ms interval)
        auto loop_end = std::chrono::steady_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
        int sleep_ms = 50 - static_cast<int>(duration.count());
        if (sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
    }

    return 0;
}
