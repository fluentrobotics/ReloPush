// motor_calibration.cpp -> binary `motor_calibration`
//
// Main-PC campaign runner for the motor calibration tool. Wires the
// pure/testable mpc::TrialRunner (MotorCalibCore.h) to:
//   - a REAL ZMQ IDriverClient (REQ to the VescDriver control REP, per the
//     FROZEN DRIVER CONTROL PROTOCOL),
//   - a REAL OptiTrack pose feed (SUB on cfg.localization_endpoint, topic
//     "/<robot_topic_name>/localization", plain JSON {"x","y","yaw"} --
//     byte-identical to mpc::encode_localization_payload()'s wire format,
//     see SimCore.h/optitrack_zmq_bridge.cpp -- no "t" field, so this file
//     self-timestamps every arrival with its own monotonic clock), and
//   - the driver's own telemetry PUB (tcp://<robot_ip>:<telemetry_port>,
//     topic "/<robot_topic_name>/vesc_telemetry" -- robot_topic_name is the
//     SAME published name used everywhere in this stack: ackermann/
//     telemetry/localization all key on it, matching VescDriver's own
//     DriverConfig::robot_name).
//
// This file is deliberately thin: almost all of the state-machine/fitting
// logic already lives in mpc:: (MotorCalibCore.h/.cpp) and is unit-tested
// there without sockets (see MPC/tests/motor_calib_tests.cpp). What's left
// here is CLI/config plumbing, the three ZMQ endpoints, the sweep-ordering/
// output-file policy, and JSON export of campaign_summary.json (a schema
// OWNED by this file, unlike calibration_<mode>.json which follows the
// frozen schema via mpc::export_calibration()).
//
// campaign_summary.json shape (this file's own schema, not frozen):
// {
//   "robot_ip", "robot_topic_name", "modes": [...],
//   "trials": [ {"index","mode","value","state":"done"|"aborted",
//                "abort_reason","n_rows","steady_v","steady_cmd",
//                "classification":"moving"|"non_moving"|"unknown","csv_path"} ... ],
//   "fits": { "<mode>": {"cmd_per_mps","cmd_offset","fit_rms","stall":{...},
//                          "n_trials","grid_ok","calibration_file"} , ... },
//   "driver_restored_to_ackermann": bool
// }

#include "mpc/MotorCalibCore.h"
#include "CalibClient.h"

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

volatile std::sig_atomic_t g_stop = 0;
void handle_stop_signal(int) { g_stop = 1; }

// Threshold shared between per-trial "classification" (campaign_summary.json)
// and mpc::fit_stall()'s own min_moving argument, so the two never disagree
// about what counts as "the vehicle actually moved".
constexpr double kDefaultMinMoving = 0.03;  // m/s

// How often the main loop drains the ZMQ SUB sockets (LiveChannels::pump()).
// Deliberately much finer than any realistic localization publish rate
// (typically ~30-120Hz, i.e. 8-33ms): since the wire payload carries no "t"
// field and every sample is self-timestamped at RECEIPT (see
// LiveChannels::pump()'s own doc comment), a poll period comparable to the
// publisher's own period aliases against it -- a message can sit queued for
// anywhere up to one whole poll period before being drained, and that
// queuing-delay VARIANCE (not just its mean) leaks directly into the
// finite-difference velocity estimate. Polling this tightly bounds that
// quantization error to a couple of ms, well below one publish period even
// at 100+ Hz publish rates, without meaningfully increasing CPU load (each
// idle poll is just a couple of non-blocking zmq_poll calls).
constexpr int kPumpPollIntervalMs = 2;

// ---------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------
struct CliOptions {
    std::string config_path;
    bool has_robot_ip = false;
    std::string robot_ip;
    bool has_modes = false;
    std::string modes_str;
    bool has_out = false;
    std::string out_dir;
    int max_trials = 0;  // 0 == all
    bool dry_run = false;
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
        << "Usage: motor_calibration [options]\n\n"
           "  --config PATH        motor_calib_config.json (default: compiled-in\n"
           "                       MPC/config/motor_calib_config.json if present)\n"
           "  --robot-ip IP        overrides config robot_ip\n"
           "  --modes duty,erpm    which raw-command modes to sweep (default: every\n"
           "                       mode whose config sweep list is non-empty)\n"
           "  --out DIR            overrides config output_dir\n"
           "  --max-trials N       cap total trials across all modes (0 = all, default)\n"
           "  --dry-run            print planned trials + resolved endpoints, no sockets\n"
           "  --help, -h           print this message and exit\n";
}

CliOptions parse_cli(int argc, char** argv) {
    CliOptions o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (flag_matches(arg, "--config") && next_value(argc, argv, &i, &value)) {
            o.config_path = value;
        } else if (flag_matches(arg, "--robot-ip") && next_value(argc, argv, &i, &value)) {
            o.has_robot_ip = true;
            o.robot_ip = value;
        } else if (flag_matches(arg, "--modes") && next_value(argc, argv, &i, &value)) {
            o.has_modes = true;
            o.modes_str = value;
        } else if (flag_matches(arg, "--out") && next_value(argc, argv, &i, &value)) {
            o.has_out = true;
            o.out_dir = value;
        } else if (flag_matches(arg, "--max-trials") && next_value(argc, argv, &i, &value)) {
            o.max_trials = std::atoi(value.c_str());
        } else if (arg == "--dry-run") {
            o.dry_run = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else {
            std::cerr << "motor_calibration: ignoring unrecognized argument '" << arg << "'\n";
        }
    }
    return o;
}

// ---------------------------------------------------------------------
// Default --config resolution: mirrors vesc_driver_main.cpp's
// resolve_default_config_path()/own_exe_dir() pattern and
// optitrack_zmq_bridge.cpp's default_mocap_config_path() precedent --
// reimplemented locally per this codebase's established
// self-contained-per-binary convention (see e.g. those files' own
// independently-duplicated own_exe_dir()/path_exists() helpers).
// ---------------------------------------------------------------------
#ifndef MOTOR_CALIB_DEFAULT_CONFIG_PATH
#define MOTOR_CALIB_DEFAULT_CONFIG_PATH ""
#endif

bool path_exists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

struct ResolvedConfig {
    mpc::MotorCalibConfig cfg;
    std::string source_path;  // empty iff built-in MotorCalibConfig::defaults() was used verbatim.
};

// Explicit --config always wins verbatim (a missing/corrupt explicitly-given
// file is a hard error, via mpc::load_motor_calib_config()'s own throw).
// Otherwise the compiled-in default (MOTOR_CALIB_DEFAULT_CONFIG_PATH, set by
// CMakeLists.txt to MPC/config/motor_calib_config.json) is used IF it
// exists; if neither is available, falls back to
// mpc::MotorCalibConfig::defaults() with a warning (never a hard failure --
// this mirrors optitrack_zmq_bridge.cpp's own kNone fallback philosophy).
ResolvedConfig resolve_config(const std::string& cli_config_path) {
    ResolvedConfig out;
    std::string path = cli_config_path;
    if (path.empty()) {
        const std::string def = MOTOR_CALIB_DEFAULT_CONFIG_PATH;
        if (!def.empty() && path_exists(def)) {
            path = def;
        }
    }
    if (path.empty()) {
        std::cout << "[motor_calibration] no --config given and no compiled-in default config "
                     "found; using built-in MotorCalibConfig defaults\n";
        out.cfg = mpc::MotorCalibConfig::defaults();
        return out;
    }
    out.cfg = mpc::load_motor_calib_config(path);  // throws on open/parse failure.
    out.source_path = path;
    return out;
}

// ---------------------------------------------------------------------
// Trial planning (shared by --dry-run and the real run, so the printed
// dry-run plan is EXACTLY what a real run would execute).
// ---------------------------------------------------------------------
struct PlannedTrial {
    std::string mode;
    double value = 0.0;  // signed -- sign is the commanded direction.
};

std::vector<std::string> resolve_modes(const CliOptions& cli, const mpc::MotorCalibConfig& cfg) {
    std::vector<std::string> modes;
    if (cli.has_modes) {
        std::stringstream ss(cli.modes_str);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) modes.push_back(tok);
        }
    } else {
        if (!cfg.sweeps.duty.empty()) modes.push_back("duty");
        if (!cfg.sweeps.erpm.empty()) modes.push_back("erpm");
    }
    return modes;
}

std::vector<double> sweep_values_for(const mpc::MotorCalibConfig& cfg, const std::string& mode) {
    if (mode == "duty") return cfg.sweeps.duty;
    if (mode == "erpm") return cfg.sweeps.erpm;
    return {};
}

// Alternates commanded direction (+,-,+,-,...) within each mode's own sweep
// list, per this task's "alternating direction, per TrialRunner" brief.
// max_trials (0 == unlimited) caps the TOTAL plan length across every mode
// combined, in mode order.
std::vector<PlannedTrial> build_trial_plan(const mpc::MotorCalibConfig& cfg,
                                            const std::vector<std::string>& modes, int max_trials) {
    std::vector<PlannedTrial> plan;
    for (const std::string& mode : modes) {
        int sign = 1;
        for (double v : sweep_values_for(cfg, mode)) {
            if (max_trials > 0 && static_cast<int>(plan.size()) >= max_trials) return plan;
            plan.push_back({mode, sign * v});
            sign = -sign;
        }
    }
    return plan;
}

std::string format_trial_value(double v) {
    std::ostringstream oss;
    if (std::fabs(v - std::llround(v)) < 1e-9) {
        oss << static_cast<long long>(std::llround(v));
    } else {
        oss << std::fixed << std::setprecision(3) << v;
    }
    return oss.str();
}

std::string trial_csv_path(const std::string& out_dir, int index, const std::string& mode, double value) {
    std::ostringstream oss;
    oss << out_dir << "/trial_" << std::setw(3) << std::setfill('0') << index << "_" << mode << "_"
        << format_trial_value(value) << ".csv";
    return oss.str();
}

// ZmqDriverClient, DriverRestoreGuard, parse_pose/parse_telemetry, and
// LiveChannels moved to CalibClient.h/.cpp (namespace mpc::) so drive_test
// can reuse them; pulled in via `using` below.
using mpc::ZmqDriverClient;
using mpc::DriverRestoreGuard;
using mpc::LiveChannels;

// Mirrors mpc::steady_state_v()'s trailing-window approach (see
// MotorCalibCore.cpp) but averages TrialRow::erpm (telemetry-reported motor
// speed) instead of TrialRow::cmd_value. Used to fit erpm_per_mps -- a
// property of the motor/wheel, independent of which raw-command MODE
// produced a given trial -- directly from mocap-vs-telemetry data, since
// MotorCalibConfig carries no such field of its own (unlike cmd_per_mps,
// which is mode-specific and already produced by mpc::fit_linear()).
// Reimplemented locally rather than added to MotorCalibCore.h/.cpp, which
// this task treats as already-frozen/tested (see this file's own header
// comment on scope).
struct ErpmVPoint {
    double v_ss = 0.0;
    double erpm_ss = 0.0;
    bool ok = false;
};
ErpmVPoint steady_state_erpm(const std::vector<mpc::TrialRow>& rows, double steady_window_frac) {
    ErpmVPoint out;
    std::vector<const mpc::TrialRow*> telem_rows;
    for (const auto& r : rows) {
        if (r.has_telemetry) telem_rows.push_back(&r);
    }
    if (telem_rows.empty()) return out;
    double t_min = telem_rows.front()->t, t_max = telem_rows.front()->t;
    for (const auto* r : telem_rows) {
        t_min = std::min(t_min, r->t);
        t_max = std::max(t_max, r->t);
    }
    const double window_start = t_max - steady_window_frac * (t_max - t_min);
    std::vector<const mpc::TrialRow*> window;
    for (const auto* r : telem_rows) {
        if (r->t >= window_start) window.push_back(r);
    }
    if (window.empty()) window.push_back(telem_rows.back());
    double sum_v = 0.0, sum_e = 0.0;
    for (const auto* r : window) {
        sum_v += r->v_smooth;
        sum_e += r->erpm;
    }
    out.v_ss = sum_v / static_cast<double>(window.size());
    out.erpm_ss = sum_e / static_cast<double>(window.size());
    out.ok = true;
    return out;
}

std::string classify(const mpc::SteadyStatePoint& p) {
    if (!p.ok) return "unknown";
    return (std::fabs(p.v_ss) > kDefaultMinMoving) ? "moving" : "non_moving";
}

// Per-mode accumulator, filled trial-by-trial as the sweep runs (mode
// ordering-agnostic -- see build_trial_plan()'s doc comment).
struct ModeAccumulator {
    std::vector<mpc::SteadyStatePoint> points;
    std::vector<std::vector<mpc::TrialRow>> trial_rows;
    int n_trials = 0;
};

void ensure_out_dir(const std::string& dir) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        std::cerr << "[motor_calibration] warning: could not create output dir '" << dir
                   << "': " << ec.message() << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    const CliOptions cli = parse_cli(argc, argv);

    ResolvedConfig resolved;
    try {
        resolved = resolve_config(cli.config_path);
    } catch (const std::exception& ex) {
        std::cerr << "motor_calibration: failed to load config: " << ex.what() << "\n";
        return 1;
    }
    mpc::MotorCalibConfig cfg = resolved.cfg;
    if (cli.has_robot_ip) cfg.robot_ip = cli.robot_ip;
    if (cli.has_out) cfg.output_dir = cli.out_dir;

    const std::vector<std::string> modes = resolve_modes(cli, cfg);
    if (modes.empty()) {
        std::cerr << "motor_calibration: no modes to run (neither --modes nor any config sweep list "
                      "is non-empty)\n";
        return 1;
    }
    const std::vector<PlannedTrial> plan = build_trial_plan(cfg, modes, cli.max_trials);

    std::cout << "[motor_calibration] config: " << (resolved.source_path.empty()
                                                          ? std::string("<built-in defaults>")
                                                          : resolved.source_path)
               << "\n";
    std::cout << "[motor_calibration] robot_ip=" << cfg.robot_ip << " control_port=" << cfg.control_port
               << " telemetry_port=" << cfg.telemetry_port
               << " localization_endpoint=" << cfg.localization_endpoint
               << " robot_topic_name=" << cfg.robot_topic_name << "\n";
    std::cout << "[motor_calibration] line: length_m=" << cfg.line.length_m
               << " end_margin_m=" << cfg.line.end_margin_m << " line_yaw_rad="
               << (cfg.line.line_yaw_rad.has_value() ? std::to_string(*cfg.line.line_yaw_rad)
                                                       : std::string("<robot's initial heading>"))
               << "\n";
    std::cout << "[motor_calibration] output_dir=" << cfg.output_dir << "\n";
    std::cout << "[motor_calibration] planned trials (" << plan.size() << "):\n";
    for (std::size_t i = 0; i < plan.size(); ++i) {
        std::cout << "  " << std::setw(3) << std::setfill('0') << (i + 1) << ") mode=" << plan[i].mode
                   << " value=" << plan[i].value << "\n";
    }

    if (cli.dry_run) {
        std::cout << "[motor_calibration] --dry-run: no sockets opened, nothing executed.\n";
        return 0;
    }

    if (plan.empty()) {
        std::cerr << "motor_calibration: trial plan is empty (empty sweep list(s) or --max-trials 0 "
                      "combined with... check config)\n";
        return 1;
    }

    ensure_out_dir(cfg.output_dir);

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    bool campaign_started = false;  // true once set_source(calib) has actually been issued.
    bool fatal_error = false;
    std::string fatal_message;
    bool restored_ackermann = false;
    std::unordered_map<std::string, ModeAccumulator> accum;
    nlohmann::json trials_json = nlohmann::json::array();
    nlohmann::json fits_json = nlohmann::json::object();

    // Everything below touches the driver over ZMQ and does file I/O for
    // CSVs/calibration files -- any exception thrown after set_source(calib)
    // below (zmq::error_t, std::bad_alloc, file I/O, ...) must not
    // terminate the process with the driver left stuck in calib source.
    // DriverRestoreGuard (constructed right after that set_source(calib)
    // succeeds) restores it on the way out of this try block regardless of
    // how it's left (normal completion, abort, exception, or Ctrl-C); this
    // try/catch makes sure such an exception is logged and exits nonzero
    // instead of an unhandled-exception terminate().
    try {

    LiveChannels live(cfg);

    std::cout << "[motor_calibration] pinging driver at " << cfg.robot_ip << ":" << cfg.control_port
               << " ...\n";
    const mpc::DriverReply ping_reply = live.driver().ping();
    if (!ping_reply.ok) {
        std::cerr << "motor_calibration: driver unreachable (" << ping_reply.error
                   << "). Aborting -- check --robot-ip/control_port and that vesc_driver is running.\n";
        return 1;
    }
    std::cout << "[motor_calibration] driver reachable.\n";

    mpc::TrialRunnerConfig runner_cfg = mpc::make_trial_runner_config(cfg);
    mpc::TrialRunner runner(live.driver(), runner_cfg, cfg.line.line_yaw_rad);

    std::unique_ptr<DriverRestoreGuard> restore_guard;

    // Campaign-start sequence: set_source(calib), servo(center), then pump
    // until the LineFrame captures off the first real pose (or timeout).
    {
        const mpc::DriverReply src = live.driver().set_source("calib");
        if (!src.ok) {
            std::cerr << "motor_calibration: set_source(calib) refused: " << src.error << "\n";
            return 1;
        }
        campaign_started = true;
        // Constructed right after the successful set_source(calib): from
        // here on, every exit path (an early return below, breaking out of
        // the trial loop, an exception, or falling through to the explicit
        // restore() call near the bottom of this try block) restores the
        // driver exactly once.
        restore_guard = std::make_unique<DriverRestoreGuard>(live.driver());
        live.driver().servo(runner_cfg.center_servo_value);

        std::cout << "[motor_calibration] waiting for the first mocap pose to capture the line frame...\n";
        const double deadline = live.now_s() + 10.0;
        while (!runner.line().captured() && live.now_s() < deadline && !g_stop) {
            live.pump(runner);
            std::this_thread::sleep_for(std::chrono::milliseconds(kPumpPollIntervalMs));
        }
        if (!runner.line().captured()) {
            fatal_error = true;
            fatal_message = "no mocap pose arrived within 10s -- check localization_endpoint/"
                              "robot_topic_name";
        } else {
            std::cout << "[motor_calibration] line frame captured: x0=" << runner.line().x0()
                       << " y0=" << runner.line().y0() << " line_yaw=" << runner.line().line_yaw() << "\n";
        }
    }

    if (!fatal_error) {
        for (std::size_t i = 0; i < plan.size() && !g_stop; ++i) {
            const PlannedTrial& pt = plan[i];
            const int index = static_cast<int>(i) + 1;
            const mpc::RawMode mode = mpc::raw_mode_from_string(pt.mode);

            std::cout << "[motor_calibration] trial " << index << "/" << plan.size() << ": mode="
                       << pt.mode << " value=" << pt.value << " ...\n";

            mpc::TrialSpec spec;
            spec.mode = mode;
            spec.value = pt.value;
            spec.ttl_ms = 500.0;
            runner.start_trial(spec);

            auto pump_until_settled = [&]() {
                while (true) {
                    const mpc::TrialState st = live.pump(runner);
                    if (st == mpc::TrialState::kDone || st == mpc::TrialState::kAborted) break;
                    if (g_stop) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(kPumpPollIntervalMs));
                }
            };
            pump_until_settled();

            // A trial that aborts AT ARM purely for lack of travel room in
            // the COMMANDED direction is recoverable: retry ONCE with the
            // direction flipped (same magnitude). LineFrame::capture()'s
            // s=0 origin is wherever the vehicle happened to be at capture
            // time -- not necessarily mid-line -- so e.g. the very first
            // negative-direction trial of an alternating sweep can
            // trivially have zero room "behind" it even though the vehicle
            // is perfectly capable of driving that direction once it has
            // covered some ground forward first. Deliberately does NOT
            // retry a MID-RUN kOutOfEnvelope (a trial that had SOME room at
            // ARM but ran out of it while moving) -- that is a genuine,
            // reportable safety/geometry finding about the configured line
            // vs. commanded speed, not a spurious artifact of where the
            // session happened to start.
            bool direction_flipped = false;
            if (runner.state() == mpc::TrialState::kAborted &&
                runner.abort_reason() == mpc::TrialAbortReason::kInsufficientRoomAtArm) {
                std::cout << "    (insufficient room in the planned direction -- retrying with "
                              "direction flipped)\n";
                spec.value = -spec.value;
                direction_flipped = true;
                runner.start_trial(spec);
                pump_until_settled();
            }

            const double executed_value = spec.value;
            const bool done = runner.state() == mpc::TrialState::kDone;
            const bool aborted = runner.state() == mpc::TrialState::kAborted;
            const std::string csv_path = trial_csv_path(cfg.output_dir, index, pt.mode, executed_value);
            runner.write_csv(csv_path);

            const mpc::SteadyStatePoint ss = mpc::steady_state_v(runner.rows(), cfg.trial.steady_window_frac);
            // Only a cleanly-completed (kDone) trial's steady-state point and
            // raw rows feed the fitters below -- an aborted trial (out-of-
            // envelope, over-speed, mocap-stale, ...) stopped mid-run for a
            // reason unrelated to steady-state behavior, so its samples are
            // transient/non-representative and would bias fit_stall/
            // fit_linear/fit_grid if included. Its CSV is still written
            // above and it's still logged below (excluded_from_fit=true) --
            // only fit input is affected.
            const bool excluded_from_fit = !done;
            ModeAccumulator& acc = accum[pt.mode];
            if (done) {
                acc.points.push_back(ss);
                acc.trial_rows.push_back(runner.rows());
                acc.n_trials += 1;
            }

            nlohmann::json tj;
            tj["index"] = index;
            tj["mode"] = pt.mode;
            tj["value"] = executed_value;
            tj["planned_value"] = pt.value;
            tj["direction_flipped"] = direction_flipped;
            tj["state"] = done ? "done" : (aborted ? "aborted" : "unknown");
            tj["abort_reason"] = mpc::to_string(runner.abort_reason());
            tj["abort_message"] = runner.abort_message();
            tj["n_rows"] = runner.rows().size();
            tj["steady_v"] = ss.ok ? ss.v_ss : 0.0;
            tj["steady_cmd"] = ss.ok ? ss.cmd : 0.0;
            tj["classification"] = classify(ss);
            tj["csv_path"] = csv_path;
            tj["excluded_from_fit"] = excluded_from_fit;
            trials_json.push_back(tj);

            std::cout << "    -> " << (done ? "done" : "ABORTED") << (aborted ? " (" + mpc::to_string(runner.abort_reason()) + ")" : "")
                       << " rows=" << runner.rows().size()
                       << (ss.ok ? " steady_v=" + std::to_string(ss.v_ss) : "") << "\n";

            if (aborted && runner.abort_reason() == mpc::TrialAbortReason::kDriverRefused) {
                fatal_error = true;
                fatal_message = "driver refused a command mid-campaign (" + runner.abort_message() + ") -- "
                                  "stopping the campaign early";
                break;
            }
        }
    }

    // Global erpm_per_mps regression across every trial's telemetry
    // (mode-agnostic -- see steady_state_erpm()'s doc comment).
    double global_erpm_per_mps = 0.0;
    {
        std::vector<mpc::SteadyStatePoint> erpm_points;
        for (const auto& kv : accum) {
            for (const auto& rows : kv.second.trial_rows) {
                const ErpmVPoint p = steady_state_erpm(rows, cfg.trial.steady_window_frac);
                if (p.ok && std::fabs(p.v_ss) > kDefaultMinMoving) {
                    mpc::SteadyStatePoint sp;
                    sp.cmd = p.erpm_ss;
                    sp.v_ss = p.v_ss;
                    sp.ok = true;
                    erpm_points.push_back(sp);
                }
            }
        }
        if (erpm_points.size() >= 2) {
            const mpc::LinearFit erpm_fit = mpc::fit_linear(erpm_points, -1.0);
            if (erpm_fit.ok) global_erpm_per_mps = erpm_fit.cmd_per_mps;
        }
    }

    for (const std::string& mode_str : modes) {
        auto it = accum.find(mode_str);
        // `it->second.points` can legitimately be empty here (every trial
        // for this mode aborted -- see the done-only accumulation above):
        // don't skip the mode outright. Let fit_stall/fit_linear/fit_grid
        // see the (possibly empty) input so they report ok=false and the
        // no-usable-fit path below still produces a reportable
        // campaign_summary.json entry instead of silently omitting the
        // mode entirely.
        if (it == accum.end()) continue;
        const ModeAccumulator& acc = it->second;
        const mpc::RawMode mode = mpc::raw_mode_from_string(mode_str);

        const mpc::StallFit stall = mpc::fit_stall(acc.points, kDefaultMinMoving);
        const mpc::LinearFit linear = mpc::fit_linear(acc.points, stall.ok ? stall.min_cmd : 0.0);
        const mpc::GridFit grid = mpc::fit_grid(acc.trial_rows);

        // erpm_per_mps: prefer the global cross-mode telemetry regression;
        // fall back to this mode's own cmd_per_mps (exact in erpm mode,
        // since raw erpm commands ARE the target erpm -- see this file's
        // header comment), else 1.0 as an inert last resort.
        double erpm_per_mps = global_erpm_per_mps;
        if (erpm_per_mps <= 0.0 && mode == mpc::RawMode::kErpm && linear.ok) {
            erpm_per_mps = linear.cmd_per_mps;
        }
        if (erpm_per_mps <= 0.0) erpm_per_mps = 1.0;

        const std::string calib_path = cfg.output_dir + "/calibration_" + mode_str + ".json";

        nlohmann::json fj;
        fj["cmd_per_mps"] = linear.cmd_per_mps;
        fj["cmd_offset"] = linear.cmd_offset;
        fj["fit_rms"] = linear.fit_rms;
        fj["linear_ok"] = linear.ok;
        fj["erpm_per_mps"] = erpm_per_mps;
        fj["stall"] = {{"min_cmd", stall.min_cmd},
                        {"kick_cmd", stall.kick_cmd},
                        {"kick_ms", stall.kick_ms},
                        {"min_moving_speed_mps", stall.min_moving_speed_mps},
                        {"ok", stall.ok}};
        fj["n_trials"] = acc.n_trials;
        fj["grid_ok"] = grid.ok;

        // Neither representation the driver could actually consume came out
        // usable (no grid cell was directly supported AND the linear fit
        // itself failed, e.g. because the campaign aborted after too few
        // trials/points). Exporting anyway would write a schema-valid file
        // the driver happily loads as a real calibration -- typically
        // cmd_per_mps=0/cmd_offset=0 with grid:null, which VescDriver's
        // CalibratedMap turns into a permanently-inert cmd=0 mapping (or
        // worse) with no signal in the frozen schema that it is garbage.
        // Skip the write; still record why in the summary so an operator
        // sees the mode needs re-running rather than silently getting a
        // bad file on disk.
        if (!linear.ok && !grid.ok) {
            fj["calibration_written"] = false;
            fj["skip_reason"] = "no usable fit: linear.ok=false and grid.ok=false (n_trials=" +
                                  std::to_string(acc.n_trials) + ")";
            fits_json[mode_str] = fj;

            std::cout << "\n=== " << mode_str << " fit summary ===\n";
            std::cout << "  stall.min_cmd=" << stall.min_cmd << " kick_cmd=" << stall.kick_cmd << "\n";
            std::cerr << "motor_calibration: WARNING mode '" << mode_str
                       << "' produced no usable fit (linear.ok=false, grid.ok=false, n_trials=" << acc.n_trials
                       << ") -- NOT writing " << calib_path
                       << " (would be a schema-valid but non-functional calibration file)\n";
            continue;
        }

        const nlohmann::json calib_json = mpc::export_calibration(
            cfg.robot_topic_name, mode, erpm_per_mps, linear, stall, acc.points,
            grid.ok ? &grid : nullptr, acc.n_trials,
            "generated by motor_calibration; erpm_per_mps from cross-mode telemetry regression");

        std::ofstream f(calib_path);
        f << calib_json.dump(2);
        f.close();

        fj["calibration_written"] = true;
        fj["calibration_file"] = calib_path;
        fits_json[mode_str] = fj;

        std::cout << "\n=== " << mode_str << " fit summary ===\n";
        std::cout << "  stall.min_cmd=" << stall.min_cmd << " kick_cmd=" << stall.kick_cmd << "\n";
        std::cout << "  cmd_per_mps=" << linear.cmd_per_mps << " cmd_offset=" << linear.cmd_offset
                   << " fit_rms=" << linear.fit_rms << "\n";
        std::cout << "  erpm_per_mps=" << erpm_per_mps << " grid_ok=" << (grid.ok ? "yes" : "no") << "\n";
        std::cout << "  wrote " << calib_path << "\n";
    }

    // Restore + close, regardless of success/abort/fatal_error/Ctrl-C.
    if (campaign_started) {
        restored_ackermann = restore_guard->restore();
        if (!restored_ackermann) {
            std::cerr << "motor_calibration: WARNING failed to restore driver source to 'ackermann': "
                       << restore_guard->error() << "\n";
        }
    }

    } catch (const std::exception& ex) {
        std::cerr << "motor_calibration: FATAL: unhandled exception during campaign: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "motor_calibration: FATAL: unknown exception during campaign\n";
        return 1;
    }

    nlohmann::json summary;
    summary["robot_ip"] = cfg.robot_ip;
    summary["robot_topic_name"] = cfg.robot_topic_name;
    summary["modes"] = modes;
    summary["trials"] = trials_json;
    summary["fits"] = fits_json;
    summary["driver_restored_to_ackermann"] = restored_ackermann;
    summary["fatal_error"] = fatal_error;
    summary["fatal_message"] = fatal_message;
    summary["interrupted"] = (g_stop != 0);

    const std::string summary_path = cfg.output_dir + "/campaign_summary.json";
    {
        std::ofstream f(summary_path);
        f << summary.dump(2);
    }
    std::cout << "\n[motor_calibration] wrote " << summary_path << "\n";

    if (fatal_error) {
        std::cerr << "motor_calibration: FATAL: " << fatal_message << "\n";
        return 1;
    }
    return 0;
}
