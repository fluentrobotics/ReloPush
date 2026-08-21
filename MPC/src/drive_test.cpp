// drive_test.cpp -> binary `drive_test`
//
// Short hardware drive-test tool: drives a real robot a configurable
// distance (default 2m) straight and back through the REAL VESC driver's
// control-plane (FROZEN DRIVER CONTROL PROTOCOL), reusing the same
// ZmqDriverClient/DriverRestoreGuard/LiveChannels machinery motor_calibration
// uses (see CalibClient.h). Unlike motor_calibration (which runs a full
// sweep/fit campaign via mpc::TrialRunner), this tool is a minimal two-leg
// smoke test: arm calib source, drive +distance, settle, drive back to the
// start, restore ackermann source. No CSV/campaign export.
//
// Two operating modes:
//   - mocap (default): captures an mpc::LineFrame off the first live pose
//     sample and tracks longitudinal position s(x,y) along it.
//   - --use-telemetry: no mocap feed needed at all -- position is
//     approximated by integrating erpm/--erpm-per-mps against wall-clock
//     dt. Approximate/uncalibrated; a clear warning is printed.
//
// Safety checks run every tick during each leg (see run_leg()): envelope
// (s beyond distance+margin, or below -margin on the return leg),
// wrong_direction (s crosses the OPPOSITE margin from the one the commanded
// direction would exceed -- e.g. forward leg but s < -margin -- catching
// reversed motor polarity/rollback that the plain envelope check above
// can't see since it only bounds excess motion in the commanded direction),
// over-speed (|v| > 0.8 m/s), mocap staleness (>300ms since last pose, mocap
// mode only), and a per-leg wall-clock timeout. Any trip stops the driver
// immediately and aborts the whole program (nonzero exit) via the
// DriverRestoreGuard-protected exit path, restoring "ackermann" source.
//
// Ctrl-C/SIGTERM: same g_stop sig_atomic_t pattern as motor_calibration.cpp.

#include "mpc/MotorCalibCore.h"
#include "CalibClient.h"

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_stop = 0;
void handle_stop_signal(int) { g_stop = 1; }

constexpr double kMaxSpeedAbortMps = 0.8;
constexpr double kMocapStaleAbortS = 0.3;
constexpr int kTickPollIntervalMs = 2;
constexpr int kRawReissueIntervalMs = 200;
constexpr double kSettleQuietVMps = 0.02;

// ---------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------
struct CliOptions {
    bool has_robot_ip = false;
    std::string robot_ip;
    int control_port = 3460;
    int telemetry_port = 3560;
    std::string loc_endpoint = "tcp://127.0.0.1:3260";
    std::string topic = "robot2";
    double distance = 2.0;
    std::string mode_str = "erpm";
    double value = 1200.0;
    bool use_telemetry = false;
    double erpm_per_mps = 4614.0;
    double settle_s = 1.5;
    double leg_timeout_s = 20.0;
    double margin = 0.5;
    bool help = false;
};

bool next_value(int argc, char** argv, int* i, std::string* out) {
    const std::string arg = argv[*i];
    const size_t eq = arg.find('=');
    if (eq != std::string::npos) {
        *out = arg.substr(eq + 1);
        return true;
    }
    if (*i + 1 < argc) {
        *out = argv[++(*i)];
        return true;
    }
    return false;
}
bool flag_matches(const std::string& arg, const char* flag) {
    const std::string f = flag;
    return arg == f || arg.rfind(f + "=", 0) == 0;
}

void print_usage() {
    std::cout
        << "Usage: drive_test --robot-ip <ip> [options]\n"
           "  --robot-ip <ip>          Driver's host IP (required)\n"
           "  --control-port <port>    Driver control REQ/REP port (default 3460)\n"
           "  --telemetry-port <port>  Driver telemetry PUB port (default 3560)\n"
           "  --loc-endpoint <ep>      Localization SUB endpoint (default tcp://127.0.0.1:3260)\n"
           "  --topic <name>           Robot topic name (default robot2)\n"
           "  --distance <m>           Distance to drive out and back (default 2.0)\n"
           "  --mode <erpm|duty>       Raw command mode (default erpm)\n"
           "  --value <v>              Raw command magnitude, unsigned (default 1200)\n"
           "  --use-telemetry          No-mocap fallback: integrate erpm/erpm-per-mps\n"
           "  --erpm-per-mps <v>       Only meaningful with --use-telemetry (default 4614)\n"
           "  --settle-s <s>           Settle time between legs (default 1.5)\n"
           "  --leg-timeout-s <s>      Per-leg wall-clock timeout (default 20)\n"
           "  --margin <m>             Envelope safety margin beyond distance (default 0.5)\n"
           "  --help, -h               Show this help\n";
}

bool parse_cli(int argc, char** argv, CliOptions* opt, std::string* error) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string val;
        if (arg == "--help" || arg == "-h") {
            opt->help = true;
        } else if (flag_matches(arg, "--robot-ip")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--robot-ip needs a value"; return false; }
            opt->robot_ip = val;
            opt->has_robot_ip = true;
        } else if (flag_matches(arg, "--control-port")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--control-port needs a value"; return false; }
            opt->control_port = std::stoi(val);
        } else if (flag_matches(arg, "--telemetry-port")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--telemetry-port needs a value"; return false; }
            opt->telemetry_port = std::stoi(val);
        } else if (flag_matches(arg, "--loc-endpoint")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--loc-endpoint needs a value"; return false; }
            opt->loc_endpoint = val;
        } else if (flag_matches(arg, "--topic")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--topic needs a value"; return false; }
            opt->topic = val;
        } else if (flag_matches(arg, "--distance")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--distance needs a value"; return false; }
            opt->distance = std::stod(val);
        } else if (flag_matches(arg, "--mode")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--mode needs a value"; return false; }
            opt->mode_str = val;
        } else if (flag_matches(arg, "--value")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--value needs a value"; return false; }
            opt->value = std::stod(val);
        } else if (arg == "--use-telemetry") {
            opt->use_telemetry = true;
        } else if (flag_matches(arg, "--erpm-per-mps")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--erpm-per-mps needs a value"; return false; }
            opt->erpm_per_mps = std::stod(val);
        } else if (flag_matches(arg, "--settle-s")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--settle-s needs a value"; return false; }
            opt->settle_s = std::stod(val);
        } else if (flag_matches(arg, "--leg-timeout-s")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--leg-timeout-s needs a value"; return false; }
            opt->leg_timeout_s = std::stod(val);
        } else if (flag_matches(arg, "--margin")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--margin needs a value"; return false; }
            opt->margin = std::stod(val);
        } else {
            *error = "unrecognized argument '" + arg + "'";
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------
// Per-leg result.
// ---------------------------------------------------------------------
struct LegResult {
    bool ok = false;
    std::string abort_reason;  // empty if ok
    double distance_covered = 0.0;
    double duration_s = 0.0;
    double peak_abs_v = 0.0;
    double final_s = 0.0;
};

// Runs one leg: commands `signed_value` via raw(mode, signed_value, ttl_ms),
// re-issuing every kRawReissueIntervalMs to keep the driver's TTL alive,
// polling pose/telemetry each tick, tracking s and peak |v|, and checking
// safety envelopes every tick. `is_forward` selects which envelope check
// applies (s > distance+margin for forward, s < -margin for return), which
// OPPOSITE-side bound trips wrong_direction instead (s < -margin for
// forward, s > distance+margin for return -- see run_leg's body), and
// which normal-completion threshold applies (s >= distance for forward,
// s <= 0.05 for return).
// `s_current` is the caller-owned running longitudinal position, shared
// across BOTH legs (and both modes) so a leg always starts from wherever
// the previous leg (or initial capture) actually left off, rather than
// resetting to 0 -- critical for leg 2's out-of-envelope/normal-completion
// checks, which would otherwise see a stale/zero s before the first pose or
// telemetry sample of that leg arrives.
LegResult run_leg(mpc::LiveChannels& live, mpc::RawMode mode, double signed_value, double distance,
                   double margin, double leg_timeout_s, bool is_forward, bool use_telemetry,
                   double erpm_per_mps, mpc::LineFrame* line, mpc::VelocityEstimator* vel_est,
                   double* s_current) {
    LegResult result;
    const auto t_start = std::chrono::steady_clock::now();
    auto last_reissue = std::chrono::steady_clock::time_point::min();
    double last_pose_seen_t = live.now_s();
    double last_telem_erpm = 0.0;
    bool have_telem = false;
    auto last_tick = std::chrono::steady_clock::now();

    while (true) {
        if (g_stop) {
            live.driver().stop();
            result.ok = false;
            result.abort_reason = "interrupted";
            return result;
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed_s = std::chrono::duration<double>(now - t_start).count();
        if (elapsed_s > leg_timeout_s) {
            live.driver().stop();
            result.ok = false;
            result.abort_reason = "leg_timeout";
            return result;
        }

        // Re-issue the raw command periodically to keep the driver's TTL alive.
        if (last_reissue == std::chrono::steady_clock::time_point::min() ||
            std::chrono::duration<double, std::milli>(now - last_reissue).count() >= kRawReissueIntervalMs) {
            live.driver().raw(mode, signed_value, /*ttl_ms=*/500.0);
            last_reissue = now;
        }

        double v_abs = 0.0;

        if (use_telemetry) {
            const auto telem = live.drain_telemetry();
            for (const auto& t : telem) {
                last_telem_erpm = t.erpm;
                have_telem = true;
            }
            const auto tick_now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(tick_now - last_tick).count();
            last_tick = tick_now;
            const double v_est = have_telem ? (last_telem_erpm / erpm_per_mps) : 0.0;
            *s_current += v_est * dt;
            v_abs = std::fabs(v_est);
        } else {
            const auto poses = live.drain_poses();
            for (const auto& p : poses) {
                last_pose_seen_t = live.now_s();
                const auto est = vel_est->feed(p.t, p.x, p.y, p.yaw);
                v_abs = std::fabs(est.smooth_valid ? est.v_smooth : est.v_raw);
                *s_current = line->s(p.x, p.y);
            }
            const double pose_age_s = live.now_s() - last_pose_seen_t;
            if (pose_age_s > kMocapStaleAbortS) {
                live.driver().stop();
                result.ok = false;
                result.abort_reason = "mocap_stale";
                return result;
            }
        }
        const double s = *s_current;

        result.peak_abs_v = std::max(result.peak_abs_v, v_abs);

        if (v_abs > kMaxSpeedAbortMps) {
            live.driver().stop();
            result.ok = false;
            result.abort_reason = "over_speed";
            return result;
        }

        if (is_forward) {
            // Reversed-polarity / rollback guard: commanded +value should move s
            // TOWARD +distance. If it instead runs away in the opposite direction
            // past -margin, the out_of_envelope check below (which only bounds
            // excess motion in the COMMANDED direction) would never trip -- catch
            // it here explicitly.
            if (s < -margin) {
                live.driver().stop();
                result.ok = false;
                result.abort_reason = "wrong_direction";
                return result;
            }
            if (s > distance + margin) {
                live.driver().stop();
                result.ok = false;
                result.abort_reason = "out_of_envelope";
                return result;
            }
            if (s >= distance) {
                live.driver().stop();
                result.ok = true;
                result.distance_covered = s;
                result.duration_s = elapsed_s;
                result.final_s = s;
                return result;
            }
        } else {
            // Same reversed-polarity guard, mirrored for the return leg: commanded
            // -value should move s back TOWARD 0. If it instead runs away past
            // distance+margin, catch it explicitly (out_of_envelope below only
            // bounds excess motion past -margin, the commanded direction).
            if (s > distance + margin) {
                live.driver().stop();
                result.ok = false;
                result.abort_reason = "wrong_direction";
                return result;
            }
            if (s < -margin) {
                live.driver().stop();
                result.ok = false;
                result.abort_reason = "out_of_envelope";
                return result;
            }
            if (s <= 0.05) {
                live.driver().stop();
                result.ok = true;
                result.distance_covered = distance - s;
                result.duration_s = elapsed_s;
                result.final_s = s;
                return result;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kTickPollIntervalMs));
    }
}

}  // namespace

int main(int argc, char** argv) {
    CliOptions opt;
    std::string parse_error;
    if (!parse_cli(argc, argv, &opt, &parse_error)) {
        std::cerr << "[drive_test] error: " << parse_error << "\n";
        print_usage();
        return 1;
    }
    if (opt.help) {
        print_usage();
        return 0;
    }
    if (!opt.has_robot_ip) {
        std::cerr << "[drive_test] error: --robot-ip is required\n";
        print_usage();
        return 1;
    }

    mpc::RawMode mode;
    try {
        mode = mpc::raw_mode_from_string(opt.mode_str);
    } catch (const std::exception& ex) {
        std::cerr << "[drive_test] error: invalid --mode '" << opt.mode_str << "': " << ex.what() << "\n";
        return 1;
    }

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    try {
        mpc::MotorCalibConfig cfg;
        cfg.robot_ip = opt.robot_ip;
        cfg.control_port = opt.control_port;
        cfg.telemetry_port = opt.telemetry_port;
        cfg.localization_endpoint = opt.loc_endpoint;
        cfg.robot_topic_name = opt.topic;

        mpc::LiveChannels live(cfg);

        std::cout << "[drive_test] pinging driver at " << opt.robot_ip << ":" << opt.control_port << "...\n";
        const mpc::DriverReply ping_reply = live.driver().ping();
        if (!ping_reply.ok) {
            std::cerr << "[drive_test] driver unreachable: " << ping_reply.error << "\n";
            return 1;
        }
        std::cout << "[drive_test] driver reachable (ping ok).\n";

        const mpc::DriverReply set_src_reply = live.driver().set_source("calib");
        if (!set_src_reply.ok) {
            std::cerr << "[drive_test] set_source(\"calib\") failed: " << set_src_reply.error << "\n";
            return 1;
        }

        mpc::DriverRestoreGuard guard(live.driver());

        live.driver().servo(0.5);  // center

        if (opt.use_telemetry) {
            std::cout << "[drive_test] WARNING: --use-telemetry mode -- distance is approximate/"
                         "uncalibrated (integrated from erpm/--erpm-per-mps, no mocap ground truth).\n";
        }

        mpc::LineFrame line;
        mpc::VelocityEstimator vel_est_fwd;
        mpc::VelocityEstimator vel_est_ret;
        double s_current = 0.0;  // shared running longitudinal position, both legs/modes (see run_leg's doc comment).

        if (!opt.use_telemetry) {
            std::cout << "[drive_test] waiting for an initial mocap pose to capture the line frame...\n";
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            bool captured = false;
            while (std::chrono::steady_clock::now() < deadline) {
                if (g_stop) {
                    std::cerr << "[drive_test] interrupted while waiting for initial pose\n";
                    live.driver().stop();
                    return 1;
                }
                const auto poses = live.drain_poses();
                if (!poses.empty()) {
                    const auto& p = poses.back();
                    line.capture(p.x, p.y, p.yaw);
                    captured = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(kTickPollIntervalMs));
            }
            if (!captured) {
                std::cerr << "[drive_test] error: no pose arrived within 10s -- aborting\n";
                live.driver().stop();
                return 1;
            }
            std::cout << "[drive_test] line frame captured: x0=" << line.x0() << " y0=" << line.y0()
                       << " yaw=" << line.line_yaw() << "\n";
        }

        // --- Leg 1: forward ---------------------------------------------
        std::cout << "[drive_test] leg 1 (forward, +" << opt.distance << "m)...\n";
        const LegResult leg1 = run_leg(live, mode, +opt.value, opt.distance, opt.margin, opt.leg_timeout_s,
                                        /*is_forward=*/true, opt.use_telemetry, opt.erpm_per_mps, &line,
                                        &vel_est_fwd, &s_current);
        if (!leg1.ok) {
            std::cerr << "[drive_test] leg 1 ABORTED: " << leg1.abort_reason << "\n";
            return 1;
        }
        std::cout << "[drive_test] leg 1 complete: distance=" << leg1.distance_covered
                   << "m duration=" << leg1.duration_s << "s peak|v|=" << leg1.peak_abs_v << "m/s\n";

        // --- Settle --------------------------------------------------------
        std::cout << "[drive_test] settling for " << opt.settle_s << "s...\n";
        if (opt.use_telemetry) {
            std::this_thread::sleep_for(std::chrono::duration<double>(opt.settle_s));
        } else {
            const auto settle_deadline_hard =
                std::chrono::steady_clock::now() + std::chrono::duration<double>(opt.settle_s * 4.0);
            double quiet_since = -1.0;
            while (true) {
                if (g_stop) {
                    std::cerr << "[drive_test] interrupted during settle\n";
                    return 1;
                }
                if (std::chrono::steady_clock::now() > settle_deadline_hard) {
                    std::cout << "[drive_test] settle: hard deadline reached, proceeding anyway.\n";
                    break;
                }
                const auto poses = live.drain_poses();
                double v_abs = 0.0;
                bool got = false;
                for (const auto& p : poses) {
                    const auto est = vel_est_fwd.feed(p.t, p.x, p.y, p.yaw);
                    v_abs = std::fabs(est.smooth_valid ? est.v_smooth : est.v_raw);
                    got = true;
                }
                const double now_s = live.now_s();
                if (got && v_abs > kSettleQuietVMps) {
                    quiet_since = -1.0;
                } else if (quiet_since < 0.0) {
                    quiet_since = now_s;
                }
                if (quiet_since >= 0.0 && (now_s - quiet_since) >= opt.settle_s) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(kTickPollIntervalMs));
            }
        }

        // --- Leg 2: return ---------------------------------------------
        std::cout << "[drive_test] leg 2 (return)...\n";
        const LegResult leg2 = run_leg(live, mode, -opt.value, opt.distance, opt.margin, opt.leg_timeout_s,
                                        /*is_forward=*/false, opt.use_telemetry, opt.erpm_per_mps, &line,
                                        &vel_est_ret, &s_current);
        if (!leg2.ok) {
            std::cerr << "[drive_test] leg 2 ABORTED: " << leg2.abort_reason << "\n";
            return 1;
        }
        std::cout << "[drive_test] leg 2 complete: distance=" << leg2.distance_covered
                   << "m duration=" << leg2.duration_s << "s peak|v|=" << leg2.peak_abs_v << "m/s\n";

        const double final_offset = leg2.final_s;
        std::cout << "[drive_test] DONE: leg1_distance=" << leg1.distance_covered
                   << " leg1_duration=" << leg1.duration_s << " leg1_peak_v=" << leg1.peak_abs_v
                   << " leg2_distance=" << leg2.distance_covered << " leg2_duration=" << leg2.duration_s
                   << " leg2_peak_v=" << leg2.peak_abs_v << " final_offset=" << final_offset << "\n";

        const bool restore_ok = guard.restore();
        if (!restore_ok) {
            std::cerr << "[drive_test] warning: restoring ackermann source failed: " << guard.error() << "\n";
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "[drive_test] uncaught exception: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[drive_test] uncaught non-std exception\n";
        return 1;
    }
}
