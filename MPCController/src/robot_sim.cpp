#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <thread>

#include <zmq.hpp>

#include "mpc/Kinematics.h"
#include "mpc/RobotSpec.h"
#include "mpc/SimCore.h"

// mpc_robot_sim: ONE process kinematically simulating ONE Ackermann robot
// (NO dynamics). Presents exactly the two robot-side ZMQ endpoints
// mpc_controller already talks to (see MPCController/src/main.cpp):
//   - it BINDS a SUB socket on --cmd-endpoint, subscribed to
//     "/<robot>/ackermann", where main.cpp's PUB *connects* and publishes.
//   - it BINDS a PUB socket on --loc-endpoint, publishing
//     "/<robot>/localization", where main.cpp's SUB *connects* and
//     subscribes.
// This is a pure endpoint-configuration swap for real VESC hardware +
// mocap -- the controller's wire protocol (see SimCore.h's header comment)
// is treated as fixed; this file adapts to it, never the reverse.
//
// All decode/encode/clamp/watchdog logic lives in mpc:: (SimCore.h/.cpp) so
// it is unit-testable without ZMQ, mirroring main.cpp/MpcCore's split.

namespace {

volatile std::sig_atomic_t g_shutdown_requested = 0;
void handle_shutdown_signal(int) { g_shutdown_requested = 1; }

enum class RecvOutcome { kGotMessage, kTimedOut, kFatalError };

RecvOutcome try_recv(zmq::socket_t& sock, zmq::message_t& msg, zmq::recv_flags flags) {
    try {
        auto res = sock.recv(msg, flags);
        return res.has_value() ? RecvOutcome::kGotMessage : RecvOutcome::kTimedOut;
    } catch (const zmq::error_t& ex) {
        if (ex.num() == EINTR) {
            return RecvOutcome::kTimedOut;
        }
        std::cerr << "[SIM] zmq recv error: " << ex.what() << std::endl;
        return RecvOutcome::kFatalError;
    }
}

// Simple "at most once per min_interval_s" gate for stderr warnings, so a
// persistently-violating stream of commands (or a dead command channel)
// produces one line, not a flood.
class RateLimiter {
   public:
    bool allow(std::chrono::steady_clock::time_point now, double min_interval_s) {
        if (!fired_ ||
            std::chrono::duration<double>(now - last_fire_).count() >= min_interval_s) {
            last_fire_ = now;
            fired_ = true;
            return true;
        }
        return false;
    }

   private:
    bool fired_ = false;
    std::chrono::steady_clock::time_point last_fire_{};
};

struct Options {
    std::string robot_name = "robot1";
    std::string cmd_endpoint = "tcp://*:3161";
    std::string loc_endpoint = "tcp://*:3261";
    double x0 = 0.0;
    double y0 = 0.0;
    double yaw0 = 0.0;
    double v0 = 0.0;
    std::string robot_spec_path;
    double watchdog_ms = 250.0;
    double sim_rate_hz = 200.0;
    double loc_rate_hz = 30.0;
    std::string log_csv_path;
    double duration_s = 0.0;
    double noise_sigma_pct = 0.0;        // FEATURE A, ACCEL channel; clamped to [0,0.25] after parsing.
    double steer_noise_sigma_pct = 0.0;  // FEATURE A, STEER channel; clamped to [0,0.25] after parsing.
    bool has_noise_seed = false;
    std::uint64_t noise_seed = 0;
};

Options parse_args(int argc, char** argv) {
    Options o;
    const std::string kRobotSpecPrefix = "--robot-spec=";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--robot-name" && i + 1 < argc) {
            o.robot_name = argv[++i];
        } else if (arg == "--cmd-endpoint" && i + 1 < argc) {
            o.cmd_endpoint = argv[++i];
        } else if (arg == "--loc-endpoint" && i + 1 < argc) {
            o.loc_endpoint = argv[++i];
        } else if (arg == "--x" && i + 1 < argc) {
            o.x0 = std::stod(argv[++i]);
        } else if (arg == "--y" && i + 1 < argc) {
            o.y0 = std::stod(argv[++i]);
        } else if (arg == "--yaw" && i + 1 < argc) {
            o.yaw0 = std::stod(argv[++i]);
        } else if (arg == "--v0" && i + 1 < argc) {
            o.v0 = std::stod(argv[++i]);
        } else if (arg.rfind(kRobotSpecPrefix, 0) == 0) {
            o.robot_spec_path = arg.substr(kRobotSpecPrefix.size());
        } else if (arg == "--watchdog-ms" && i + 1 < argc) {
            o.watchdog_ms = std::stod(argv[++i]);
        } else if (arg == "--sim-rate-hz" && i + 1 < argc) {
            o.sim_rate_hz = std::stod(argv[++i]);
        } else if (arg == "--loc-rate-hz" && i + 1 < argc) {
            o.loc_rate_hz = std::stod(argv[++i]);
        } else if (arg == "--log-csv" && i + 1 < argc) {
            o.log_csv_path = argv[++i];
        } else if (arg.rfind("--log-csv=", 0) == 0) {
            o.log_csv_path = arg.substr(std::string("--log-csv=").size());
        } else if (arg == "--duration-s" && i + 1 < argc) {
            o.duration_s = std::stod(argv[++i]);
        } else if (arg.rfind("--noise-sigma-pct=", 0) == 0) {
            const std::string val_str = arg.substr(std::string("--noise-sigma-pct=").size());
            try {
                size_t consumed = 0;
                o.noise_sigma_pct = std::stod(val_str, &consumed);
                if (consumed != val_str.size()) {
                    throw std::invalid_argument("trailing characters");
                }
            } catch (const std::exception&) {
                std::cerr << "[SIM] Warning: invalid --noise-sigma-pct value '" << val_str
                           << "', defaulting to 0" << std::endl;
                o.noise_sigma_pct = 0.0;
            }
        } else if (arg.rfind("--steer-noise-sigma-pct=", 0) == 0) {
            const std::string val_str = arg.substr(std::string("--steer-noise-sigma-pct=").size());
            try {
                size_t consumed = 0;
                o.steer_noise_sigma_pct = std::stod(val_str, &consumed);
                if (consumed != val_str.size()) {
                    throw std::invalid_argument("trailing characters");
                }
            } catch (const std::exception&) {
                std::cerr << "[SIM] Warning: invalid --steer-noise-sigma-pct value '" << val_str
                           << "', defaulting to 0" << std::endl;
                o.steer_noise_sigma_pct = 0.0;
            }
        } else if (arg.rfind("--noise-seed=", 0) == 0) {
            const std::string val_str = arg.substr(std::string("--noise-seed=").size());
            try {
                size_t consumed = 0;
                unsigned long long parsed = std::stoull(val_str, &consumed);
                if (consumed != val_str.size()) {
                    throw std::invalid_argument("trailing characters");
                }
                o.noise_seed = static_cast<std::uint64_t>(parsed);
                o.has_noise_seed = true;
            } catch (const std::exception&) {
                std::cerr << "[SIM] Warning: invalid --noise-seed value '" << val_str
                           << "', using a nondeterministic seed" << std::endl;
                o.has_noise_seed = false;
            }
        } else {
            std::cerr << "[SIM] Warning: unrecognized argument '" << arg << "'" << std::endl;
        }
    }
    return o;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt = parse_args(argc, argv);

    mpc::RobotSpec robot_spec = mpc::RobotSpec::defaults();
    mpc::MpcParams mpc_params_unused = mpc::MpcParams::defaults();
    if (!opt.robot_spec_path.empty()) {
        try {
            mpc::load_spec_from_json(opt.robot_spec_path, robot_spec, mpc_params_unused);
            std::cout << "[SIM " << opt.robot_name << "] Loaded robot spec from "
                       << opt.robot_spec_path << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "[SIM " << opt.robot_name << "] Failed to load robot spec: " << ex.what()
                       << std::endl;
            return 1;
        }
    }

    if (opt.sim_rate_hz <= 0.0) {
        std::cerr << "[SIM " << opt.robot_name << "] --sim-rate-hz must be > 0" << std::endl;
        return 1;
    }
    if (opt.loc_rate_hz <= 0.0) {
        std::cerr << "[SIM " << opt.robot_name << "] --loc-rate-hz must be > 0" << std::endl;
        return 1;
    }

    // FEATURE A: clamp --noise-sigma-pct/--steer-noise-sigma-pct to
    // [0, kMaxSigmaPct] independently, warning if either needed clamping
    // (mirrors the parse-time try/catch's own warning for outright-
    // unparseable values above).
    {
        const double clamped_accel = mpc::NoiseModel::clamp_sigma_pct(opt.noise_sigma_pct);
        if (clamped_accel != opt.noise_sigma_pct) {
            std::cerr << "[SIM " << opt.robot_name << "] Warning: --noise-sigma-pct="
                       << opt.noise_sigma_pct << " out of [" << mpc::NoiseModel::kMinSigmaPct << ","
                       << mpc::NoiseModel::kMaxSigmaPct << "], clamping to " << clamped_accel
                       << std::endl;
        }
        opt.noise_sigma_pct = clamped_accel;

        const double clamped_steer = mpc::NoiseModel::clamp_sigma_pct(opt.steer_noise_sigma_pct);
        if (clamped_steer != opt.steer_noise_sigma_pct) {
            std::cerr << "[SIM " << opt.robot_name << "] Warning: --steer-noise-sigma-pct="
                       << opt.steer_noise_sigma_pct << " out of [" << mpc::NoiseModel::kMinSigmaPct
                       << "," << mpc::NoiseModel::kMaxSigmaPct << "], clamping to " << clamped_steer
                       << std::endl;
        }
        opt.steer_noise_sigma_pct = clamped_steer;
    }
    const std::uint64_t noise_seed =
        opt.has_noise_seed ? opt.noise_seed : std::random_device{}();
    mpc::NoiseModel noise_model(opt.noise_sigma_pct, opt.steer_noise_sigma_pct, noise_seed);

    std::signal(SIGINT, handle_shutdown_signal);
    std::signal(SIGTERM, handle_shutdown_signal);

    std::cout << "[SIM " << opt.robot_name << "] Launching: cmd(SUB bind)=" << opt.cmd_endpoint
               << " loc(PUB bind)=" << opt.loc_endpoint << " sim_rate_hz=" << opt.sim_rate_hz
               << " loc_rate_hz=" << opt.loc_rate_hz << " watchdog_ms=" << opt.watchdog_ms
               << " noise_sigma_pct(accel)=" << noise_model.accel_sigma_pct()
               << " steer_noise_sigma_pct=" << noise_model.steer_sigma_pct() << std::endl;

    zmq::context_t context(1);

    // SUB: binds where mpc_controller's ackermann PUB connects. Also
    // subscribes to the sim-only "/<robot>/sim_config" topic (FEATURE C's
    // live noise-reconfiguration channel) on the SAME socket -- a real VESC
    // bridge never receives this topic, so it is purely additive.
    zmq::socket_t cmd_sub(context, zmq::socket_type::sub);
    cmd_sub.set(zmq::sockopt::linger, 0);
    const std::string ackermann_topic = "/" + opt.robot_name + "/ackermann";
    const std::string sim_config_topic = "/" + opt.robot_name + "/sim_config";
    try {
        cmd_sub.bind(opt.cmd_endpoint);
        cmd_sub.set(zmq::sockopt::subscribe, ackermann_topic);
        cmd_sub.set(zmq::sockopt::subscribe, sim_config_topic);
    } catch (const std::exception& ex) {
        std::cerr << "[SIM " << opt.robot_name << "] Failed to bind cmd SUB socket: " << ex.what()
                   << std::endl;
        return 1;
    }

    // PUB: binds where mpc_controller's localization SUB connects.
    zmq::socket_t loc_pub(context, zmq::socket_type::pub);
    loc_pub.set(zmq::sockopt::linger, 0);
    const std::string localization_topic = "/" + opt.robot_name + "/localization";
    try {
        loc_pub.bind(opt.loc_endpoint);
    } catch (const std::exception& ex) {
        std::cerr << "[SIM " << opt.robot_name << "] Failed to bind loc PUB socket: " << ex.what()
                   << std::endl;
        return 1;
    }

    std::ofstream csv;
    if (!opt.log_csv_path.empty()) {
        csv.open(opt.log_csv_path, std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            std::cerr << "[SIM " << opt.robot_name << "] Failed to open --log-csv path '"
                       << opt.log_csv_path << "'" << std::endl;
            return 1;
        }
        csv << "t,x,y,yaw,v,a_cmd,delta_cmd,watchdog_active\n";
    }

    mpc::State4 state{opt.x0, opt.y0, opt.yaw0, opt.v0};

    const double dt_sim = 1.0 / opt.sim_rate_hz;
    const double watchdog_timeout_s = opt.watchdog_ms / 1000.0;
    const int loc_interval_ticks =
        std::max(1, static_cast<int>(std::lround(opt.sim_rate_hz / opt.loc_rate_hz)));

    double last_cmd_accel = 0.0;      // last RECEIVED command, clamped to actuator limits (no
    double last_cmd_steering = 0.0;   // noise) -- the "commanded" value CSV a_cmd/delta_cmd log.
    double last_applied_accel = 0.0;  // last_cmd_* perturbed by FEATURE A noise, re-clamped --
    double last_applied_steering = 0.0;  // the value that actually feeds the plant when not
                                          // watchdog-engaged.
    double last_cmd_reported_speed = 0.0;
    bool have_reported_speed = false;
    int drift_consecutive_count = 0;
    static constexpr double kSpeedDriftThreshold = 0.05;
    static constexpr int kSpeedDriftWarnAfter = 20;  // ~1s at the controller's 20Hz publish rate.

    mpc::Watchdog watchdog;
    auto sim_start = std::chrono::steady_clock::now();
    auto last_cmd_time = sim_start;  // no command yet => watchdog can engage from t=0.

    RateLimiter clamp_warn_limiter;
    RateLimiter malformed_warn_limiter;
    RateLimiter drift_warn_limiter;
    static constexpr double kWarnMinIntervalS = 2.0;

    // Log the initial pose at t=0 before any integration.
    if (csv.is_open()) {
        csv << 0.0 << ',' << state.x << ',' << state.y << ',' << state.yaw << ',' << state.v
            << ',' << 0.0 << ',' << 0.0 << ',' << 0 << '\n';
    }

    long tick_index = 0;
    auto next_tick_time = sim_start;
    const auto tick_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(dt_sim));

    std::cout << "[SIM " << opt.robot_name << "] Entering sim loop..." << std::endl;

    while (!g_shutdown_requested) {
        auto now = std::chrono::steady_clock::now();

        // 1. Drain the command queue (non-blocking), routing each 2-frame
        //    [topic, payload] message by its topic frame; keep only the
        //    newest payload received THIS TICK per topic (ackermann vs the
        //    FEATURE A/C sim-only "/<robot>/sim_config" live-reconfig
        //    topic), mirroring the pre-noise "keep only the newest" policy
        //    independently for each.
        std::string last_ackermann_payload;
        bool got_ackermann = false;
        std::string last_sim_config_payload;
        bool got_sim_config = false;
        while (true) {
            zmq::message_t msg;
            RecvOutcome outcome = try_recv(cmd_sub, msg, zmq::recv_flags::dontwait);
            if (outcome != RecvOutcome::kGotMessage) {
                break;
            }
            const std::string topic(static_cast<char*>(msg.data()), msg.size());
            std::string payload;
            while (msg.more()) {
                RecvOutcome more_outcome = try_recv(cmd_sub, msg, zmq::recv_flags::none);
                if (more_outcome != RecvOutcome::kGotMessage) {
                    break;
                }
                payload = std::string(static_cast<char*>(msg.data()), msg.size());
            }
            if (topic == ackermann_topic) {
                last_ackermann_payload = payload;
                got_ackermann = true;
            } else if (topic == sim_config_topic) {
                last_sim_config_payload = payload;
                got_sim_config = true;
            }
            // Unknown topic: cmd_sub is only subscribed to the two above, so
            // this should be unreachable; ignored defensively either way.
        }

        if (got_ackermann) {
            mpc::AckermannCommand cmd = mpc::decode_ackermann_payload(last_ackermann_payload);
            if (cmd.ok) {
                mpc::ClampResult clamped = mpc::clamp_command(
                    cmd.accel, cmd.steering, robot_spec.max_accel, robot_spec.max_steer);
                if ((clamped.accel_clamped || clamped.steering_clamped) &&
                    clamp_warn_limiter.allow(now, kWarnMinIntervalS)) {
                    std::cerr << "[SIM " << opt.robot_name
                               << "] Warning: received command violates actuator limits "
                                  "(accel="
                               << cmd.accel << " steering=" << cmd.steering
                               << "), clamping to accel=" << clamped.accel
                               << " steering=" << clamped.steering << std::endl;
                }
                last_cmd_accel = clamped.accel;
                last_cmd_steering = clamped.steering;
                last_cmd_reported_speed = cmd.speed;
                have_reported_speed = true;
                last_cmd_time = now;

                // FEATURE A: sample ONE new perturbation for this newly
                // received command and hold it (via last_applied_*) until
                // the next command replaces it. apply_command_noise() is the
                // SAME pure function mpc_unit_tests.cpp's N4 exercises
                // directly, so this call site cannot silently drift from
                // what N4 verifies (noise FIRST, clamp AFTER -- see
                // SimCore.h/.cpp). At sigma_pct==0 sample_command() returns
                // an exact zero without touching the RNG, so last_applied_*
                // == last_cmd_* bit-for-bit and the plant input below is
                // unchanged from the pre-noise implementation.
                mpc::NoiseModel::Perturbation pert =
                    noise_model.sample_command(robot_spec.max_accel, robot_spec.max_steer);
                mpc::ClampResult applied = mpc::apply_command_noise(
                    last_cmd_accel, last_cmd_steering, pert, robot_spec.max_accel,
                    robot_spec.max_steer);
                last_applied_accel = applied.accel;
                last_applied_steering = applied.steering;

                watchdog.on_command_received();
                if (watchdog.just_recovered()) {
                    std::cout << "[SIM " << opt.robot_name
                               << "] Watchdog recovered: new command received." << std::endl;
                }

                // Drift diagnostic: compare our integrated v against the
                // controller's own predicted "speed" field (ignored for
                // integration by design -- see SimCore.h).
                if (have_reported_speed) {
                    if (std::fabs(state.v - last_cmd_reported_speed) > kSpeedDriftThreshold) {
                        ++drift_consecutive_count;
                    } else {
                        drift_consecutive_count = 0;
                    }
                    if (drift_consecutive_count >= kSpeedDriftWarnAfter &&
                        drift_warn_limiter.allow(now, kWarnMinIntervalS)) {
                        std::cerr << "[SIM " << opt.robot_name
                                   << "] Warning: sim v (" << state.v
                                   << ") has diverged from controller-reported speed ("
                                   << last_cmd_reported_speed << ") by >"
                                   << kSpeedDriftThreshold << " for "
                                   << drift_consecutive_count << " consecutive commands."
                                   << std::endl;
                        drift_consecutive_count = 0;
                    }
                }
            } else if (malformed_warn_limiter.allow(now, kWarnMinIntervalS)) {
                std::cerr << "[SIM " << opt.robot_name
                           << "] Warning: malformed/empty ackermann payload, retaining last "
                              "command (accel="
                           << last_cmd_accel << " steering=" << last_cmd_steering << ")"
                           << std::endl;
            }
        }

        // FEATURE C: live noise-sigma reconfiguration via
        // "/<robot>/sim_config" -- sim-only, a real VESC bridge never
        // receives this topic.
        if (got_sim_config) {
            mpc::SimConfigParseResult cfg = mpc::parse_sim_config_payload(
                last_sim_config_payload, noise_model.accel_sigma_pct(),
                noise_model.steer_sigma_pct());

            // Per-field malformed warnings: fired for EVERY field that was
            // PRESENT in the payload but failed to parse, independent of
            // whether the OTHER field in the SAME payload succeeded (same
            // per-field-independence discipline -- see SimConfigParseResult's
            // *_bad flags).
            if (cfg.accel_bad) {
                std::cerr << "[SIM " << opt.robot_name
                           << "] Warning: invalid 'noise_sigma_pct' in sim_config, retaining accel "
                              "sigma=" << noise_model.accel_sigma_pct() << std::endl;
            }
            if (cfg.steer_bad) {
                std::cerr << "[SIM " << opt.robot_name
                           << "] Warning: invalid 'steer_noise_sigma_pct' in sim_config, retaining "
                              "steer sigma=" << noise_model.steer_sigma_pct() << std::endl;
            }

            if (!cfg.ok) {
                std::cerr << "[SIM " << opt.robot_name
                           << "] Warning: malformed sim_config payload, retaining noise sigma "
                              "accel="
                           << noise_model.accel_sigma_pct()
                           << " steer=" << noise_model.steer_sigma_pct() << std::endl;
            } else {
                noise_model.set_accel_sigma_pct(cfg.accel_sigma_pct);
                noise_model.set_steer_sigma_pct(cfg.steer_sigma_pct);
                std::cerr << "[SIM " << opt.robot_name << "] noise sigma set to accel="
                           << noise_model.accel_sigma_pct()
                           << " steer=" << noise_model.steer_sigma_pct() << std::endl;
            }
        }

        // 2. Watchdog update + command resolution.
        const double last_cmd_age_s = std::chrono::duration<double>(now - last_cmd_time).count();
        const bool wd_engaged = watchdog.update(last_cmd_age_s, watchdog_timeout_s);
        if (watchdog.just_engaged()) {
            std::cout << "[SIM " << opt.robot_name
                       << "] Watchdog engaged: no command for " << last_cmd_age_s
                       << "s (timeout " << watchdog_timeout_s << "s). Auto-braking." << std::endl;
        }

        // Watchdog braking is an INTERNAL safety response, not a received
        // command, so it is never perturbed: while engaged, steering holds
        // the pre-noise commanded value (last_cmd_steering), exactly as
        // before FEATURE A. Otherwise, both accel and steering are the
        // noisy last_applied_* values that actually reach the plant.
        const double resolve_steering_arg = wd_engaged ? last_cmd_steering : last_applied_steering;
        mpc::ResolvedCommand resolved = mpc::resolve_command(
            wd_engaged, state.v, last_applied_accel, resolve_steering_arg,
            robot_spec.max_accel);

        // 3. Integrate one fixed step. Watchdog-engaged + already-sub-eps
        // residual v: force it to exactly 0 before integrating. Without
        // this, accel==0 alone only HOLDS the residual (rollout_step's
        // next.v = state.v + a*dt with a==0 is a no-op on v), so a robot
        // that idles here (e.g. finished its own trajectory early while
        // other robots are still executing) drifts position at that
        // residual's constant rate for as long as it stays idle -- tens of
        // seconds in practice, easily enough to blow the final-position-error
        // acceptance bar despite never diverging.
        if (resolved.snap_v_to_zero) {
            state.v = 0.0;
        }
        // Integrate one fixed step using the SAME model as the
        // controller (mpc::rollout_step, factored into Kinematics.h).
        state = mpc::rollout_step(state, resolved.accel, resolved.steering,
                                   robot_spec.wheel_base, dt_sim);
        ++tick_index;
        const double t = tick_index * dt_sim;

        // CSV a_cmd/delta_cmd stay the COMMANDED pre-noise values (schema
        // UNCHANGED): when watchdog-engaged, that's the brake accel (already
        // noise-free) with the held pre-noise steering; when not engaged,
        // it's last_cmd_* directly -- deliberately NOT resolved.accel/
        // steering, which now carries FEATURE A's noise into the actually-
        // integrated state above. At sigma_pct==0, last_applied_* ==
        // last_cmd_* bit-for-bit, so resolved.* == last_cmd_* too and this
        // is identical to the pre-noise CSV output.
        const double csv_a_cmd = wd_engaged ? resolved.accel : last_cmd_accel;
        const double csv_delta_cmd = last_cmd_steering;
        if (csv.is_open()) {
            csv << t << ',' << state.x << ',' << state.y << ',' << state.yaw << ',' << state.v
                << ',' << csv_a_cmd << ',' << csv_delta_cmd << ','
                << (wd_engaged ? 1 : 0) << '\n';
        }

        // 4. Localization publish at --loc-rate-hz.
        if (tick_index % loc_interval_ticks == 0) {
            const std::string payload =
                mpc::encode_localization_payload(state.x, state.y, state.yaw);
            zmq::message_t topic_msg(localization_topic.begin(), localization_topic.end());
            zmq::message_t payload_msg(payload.begin(), payload.end());
            try {
                if (!loc_pub.send(topic_msg, zmq::send_flags::sndmore)) {
                    std::cerr << "[SIM " << opt.robot_name
                               << "] Warning: failed to send localization topic frame"
                               << std::endl;
                } else if (!loc_pub.send(payload_msg, zmq::send_flags::none)) {
                    std::cerr << "[SIM " << opt.robot_name
                               << "] Warning: failed to send localization payload frame"
                               << std::endl;
                }
            } catch (const zmq::error_t& ex) {
                std::cerr << "[SIM " << opt.robot_name << "] Warning: zmq send error ("
                           << ex.what() << ")" << std::endl;
            }
        }

        // 5. Duration check.
        if (opt.duration_s > 0.0 && t >= opt.duration_s) {
            std::cout << "[SIM " << opt.robot_name << "] Reached --duration-s=" << opt.duration_s
                       << "; stopping." << std::endl;
            break;
        }

        // 6. Real-time pacing.
        next_tick_time += tick_period;
        auto now_after = std::chrono::steady_clock::now();
        if (next_tick_time > now_after) {
            std::this_thread::sleep_until(next_tick_time);
        } else {
            // Fell behind real time -- resync instead of trying to catch up
            // with a burst of zero-sleep ticks.
            next_tick_time = now_after;
        }
    }

    std::cout << "[SIM " << opt.robot_name << "] Shutting down." << std::endl;
    if (csv.is_open()) {
        csv.flush();
        csv.close();
    }
    cmd_sub.close();
    loc_pub.close();

    return 0;
}
