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

namespace {

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

    auto last_loc_time = std::chrono::steady_clock::now();
    bool has_localization = false;

    double traj_start_time = std::chrono::duration_cast<std::chrono::duration<double>>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();

    const std::string ackermann_topic = "/" + robot_name + "/ackermann";
    mpc::MpcSolver solver;
    // FEATURE 2A (warm start): a brand-new trajectory run must never seed
    // its first solve from a stale warm-start buffer. `solver` is freshly
    // constructed right above (so this is a no-op today, since this process
    // only ever runs one trajectory per lifetime), but call it explicitly --
    // matching the RESUME call site below -- so the invariant holds even if
    // a future reconnect/re-arm loop reuses this MpcSolver across runs.
    solver.reset_warm_start();

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

        if (reference_time > traj_end_time + finish_waiting) {
            publish_ackermann(vesc_pub, ackermann_topic, 0.0, 0.0, 0.0);
            std::cout << "[MPC " << robot_name
                      << "] Trajectory completed. Stopping vehicle and exiting." << std::endl;
            break;
        }

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
            if (have_prev_meas_pose) {
                double dt_meas = std::chrono::duration<double>(now - prev_meas_time).count();
                if (dt_meas > 1e-3) {
                    double dx = current_pose.x - prev_meas_pose.x;
                    double dy = current_pose.y - prev_meas_pose.y;
                    v_meas = (dx * std::cos(current_pose.yaw) + dy * std::sin(current_pose.yaw)) /
                             dt_meas;
                }
                // else: samples too close together in time to trust a
                // finite difference -- keep the previous estimate.
            }
            prev_meas_pose = current_pose;
            prev_meas_time = now;
            have_prev_meas_pose = true;

            est_state.x = current_pose.x;
            est_state.y = current_pose.y;
            est_state.yaw = current_pose.yaw;
            est_state.v = v_meas;
        } else if (has_localization && elapsed_sec <= 0.1) {
            // Localization is still fresh but this tick didn't carry a NEW
            // sample (control loop runs faster than the localization
            // publish rate) -- hold the last estimate; nothing to recompute
            // until the next fresh sample arrives.
        } else {
            // Dead-reckoning fallback -- roll forward using the REAL last
            // commanded acceleration (a is this formulation's direct
            // control input; unlike the old velocity-formulation's forced
            // a=0, this actually keeps integrating the believed velocity).
            est_state = mpc::rollout_step(est_state, last_cmd_a, last_cmd_delta,
                                           robot_spec.wheel_base, mpc_params.dt);
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

        std::vector<mpc::RefState> horizon_refs;
        horizon_refs.reserve(mpc::kHorizon);
        for (int t = 0; t < mpc::kHorizon; ++t) {
            double h_time = reference_time + (t + 1) * mpc_params.dt;
            horizon_refs.push_back(mpc::get_ref_state_at_time(h_time, traj_start_time, ref_traj));
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

        mpc::PayloadValues payload = mpc::build_payload(
            opt_v, opt_delta, opt_accel, mpc_params.vel_scale,
            mpc_params.vel_scale_back, mpc_params.dir_change_scale, dir_change.change_dir);

        // Update tracking states for the NEXT tick's dead-reckoning
        // fallback and predict_delay_compensated() call.
        last_cmd_a = opt_accel;
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
