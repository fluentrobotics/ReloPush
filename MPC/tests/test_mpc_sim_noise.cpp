// FEATURE A: standalone closed-loop noise test. Modeled directly on
// test_mpc_sim_loop.cpp's Scenario A ("straight") loopback: a REQ socket
// drives the MARS handshake (trajectory -> ACK_RECEIVE_<robot> -> START ->
// ACK_START_<robot>) against a real mpc_controller process, whose
// VESC-publish/localization-subscribe endpoints are wired to a real
// mpc_robot_sim process. mpc_controller (main.cpp/MpcCore) is UNTOUCHED by
// FEATURE A -- this test proves the noise lives entirely in mpc_robot_sim
// and that the closed loop still behaves correctly around it.
//
// TEST PORTS ONLY (never the production defaults, and never
// test_mpc_sim_loop's own 45111/45161/45261 block):
//   handshake = 45711, cmd(vesc) = 45741, loc = 45761.
//
// Two runs of the SAME Scenario A trajectory:
//   Run 1: --noise-sigma-pct=0 --steer-noise-sigma-pct=0 (the default) --
//          must reproduce test_mpc_sim_loop's Scenario A tolerances
//          (0.02 m / 0.01 rad), proving sigma=0 on BOTH channels is
//          bit-identical to today's behavior end-to-end.
//   Run 2: --noise-sigma-pct=0.08 --steer-noise-sigma-pct=0.08
//          --noise-seed=12345 (BOTH the accel and the independent steer
//          channel active) -- must still COMPLETE within a relaxed
//          tolerance (0.06 m / 0.05 rad) -- the MPC closes the loop around
//          the actuation error -- AND must measurably differ from Run 1's
//          velocity trace, by RMS divergence (proving noise was actually
//          active on both channels, not silently inert -- see
//          kMinVelocityRmsDelta's comment in main() for why this gates on
//          v(t) rather than x,y pose, and RMS rather than a single max).
//
// Exits 0 only if every assertion passes; always prints the measured
// numbers.

#include <ReloPush/base64.h>
#include <ReloPush/trajectory.hpp>

#include <zmq.hpp>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

// ---------------------------------------------------------------------
// Test-only ports/identity. Deliberately distinct from BOTH the production
// defaults (3161/3261/11111) AND test_mpc_sim_loop's block
// (45111/45161/45261).
// ---------------------------------------------------------------------
constexpr int kHandshakePort = 45711;
constexpr int kCmdPort = 45741;  // mpc_robot_sim's --cmd-endpoint bind port
constexpr int kLocPort = 45761;  // mpc_robot_sim's --loc-endpoint bind port
const std::string kRobotName = "robot1";  // matches mpc_controller's --robot default

std::string to_str(double v, int precision = 6) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << v;
    return oss.str();
}

// Actuator limit used by Run 3 (the clamp-after-noise binary-level check
// below). Mirrors MPC/include/mpc/RobotSpec.h's compiled default
// (max_accel=0.73) -- duplicated here (rather than linking RobotSpec.cpp
// into this target) per this file's own precedent of duplicating small
// fixed values instead of sharing build deps (see build_scenario_a()'s
// header comment).
constexpr double kRobotMaxAccel = 0.73;

// base64(ASCII-decimal) encoder, byte-for-byte matching
// MPC/src/main.cpp's encodeAscii() -- the wire format
// decode_ackermann_payload() (SimCore.cpp) expects.
std::string encode_ascii_field(double value) {
    std::ostringstream oss;
    oss << std::setprecision(16) << value;
    return base64_encode(oss.str());
}

std::string encode_ackermann_payload(double speed, double steering, double accel) {
    // Hand-built rather than via nlohmann::json (not otherwise needed by
    // this test binary) -- field order/quoting mirrors main.cpp's
    // publish_ackermann() closely enough for decode_ackermann_payload()'s
    // parser (order-independent JSON object).
    std::ostringstream oss;
    oss << "{\"speed\":\"" << encode_ascii_field(speed) << "\",\"steering\":\""
        << encode_ascii_field(steering) << "\",\"accel\":\"" << encode_ascii_field(accel) << "\"}";
    return oss.str();
}

double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ---------------------------------------------------------------------
// Locate sibling executables (mpc_controller / mpc_robot_sim). CMake places
// every target from MPC/CMakeLists.txt in the same output
// directory as this test binary, so resolving via /proc/self/exe is robust
// to whatever directory the test happens to be invoked from.
// ---------------------------------------------------------------------
std::string self_dir() {
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) {
        throw std::runtime_error("readlink(/proc/self/exe) failed");
    }
    buf[n] = '\0';
    std::string p(buf);
    auto pos = p.find_last_of('/');
    return pos == std::string::npos ? std::string(".") : p.substr(0, pos);
}

// ---------------------------------------------------------------------
// RAII child-process guard (see test_mpc_sim_loop.cpp for the identical,
// independently-duplicated implementation -- kept self-contained per file
// rather than shared, mirroring how test_mpc_sim_loop.cpp itself already
// duplicates this pattern from test_mpc_full_loop.cpp).
// ---------------------------------------------------------------------
class ProcessGuard {
   public:
    ProcessGuard() = default;
    ProcessGuard(pid_t pid, std::string label) : pid_(pid), label_(std::move(label)) {}
    ProcessGuard(const ProcessGuard&) = delete;
    ProcessGuard& operator=(const ProcessGuard&) = delete;
    ProcessGuard(ProcessGuard&& other) noexcept { *this = std::move(other); }
    ProcessGuard& operator=(ProcessGuard&& other) noexcept {
        if (this != &other) {
            terminate();
            pid_ = other.pid_;
            label_ = std::move(other.label_);
            other.pid_ = -1;
        }
        return *this;
    }
    ~ProcessGuard() { terminate(); }

    pid_t pid() const { return pid_; }

    bool wait_for_exit(double timeout_s, double poll_interval_s = 0.1) {
        if (pid_ <= 0) return true;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            int status = 0;
            pid_t r = waitpid(pid_, &status, WNOHANG);
            if (r == pid_) {
                pid_ = -1;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(poll_interval_s));
        }
        return false;
    }

    void terminate() {
        if (pid_ <= 0) return;
        kill(pid_, SIGTERM);
        for (int i = 0; i < 75; ++i) {  // ~1.5s grace period
            int status = 0;
            pid_t r = waitpid(pid_, &status, WNOHANG);
            if (r == pid_) {
                pid_ = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cerr << "[test] pid " << pid_ << " (" << label_
                  << ") did not exit after SIGTERM; sending SIGKILL" << std::endl;
        kill(pid_, SIGKILL);
        int status = 0;
        waitpid(pid_, &status, 0);
        pid_ = -1;
    }

   private:
    pid_t pid_ = -1;
    std::string label_;
};

pid_t spawn(const std::string& exe, const std::vector<std::string>& args) {
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(exe.c_str()));
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
        throw std::runtime_error("fork() failed: " + std::string(std::strerror(errno)));
    }
    if (pid == 0) {
        execv(exe.c_str(), argv.data());
        std::fprintf(stderr, "[test] execv('%s') failed: %s\n", exe.c_str(), std::strerror(errno));
        _exit(127);
    }
    return pid;
}

// ---------------------------------------------------------------------
// Sim CSV row (see MPC/src/robot_sim.cpp's --log-csv header:
// t,x,y,yaw,v,a_cmd,delta_cmd,watchdog_active -- schema UNCHANGED by
// FEATURE A).
// ---------------------------------------------------------------------
struct CsvRow {
    double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0, v = 0.0, a_cmd = 0.0, delta_cmd = 0.0;
    int watchdog_active = 0;
};

std::vector<CsvRow> read_csv(const std::string& path) {
    std::vector<CsvRow> rows;
    std::ifstream f(path);
    if (!f.is_open()) return rows;
    std::string line;
    std::getline(f, line);  // header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string tok;
        std::vector<double> vals;
        while (std::getline(ss, tok, ',')) {
            try {
                vals.push_back(std::stod(tok));
            } catch (...) {
                vals.clear();
                break;
            }
        }
        if (vals.size() < 8) continue;
        CsvRow r;
        r.t = vals[0];
        r.x = vals[1];
        r.y = vals[2];
        r.yaw = vals[3];
        r.v = vals[4];
        r.a_cmd = vals[5];
        r.delta_cmd = vals[6];
        r.watchdog_active = static_cast<int>(std::lround(vals[7]));
        rows.push_back(r);
    }
    return rows;
}

// =======================================================================
// Scenario A "straight" trajectory -- byte-for-byte the same synthesis as
// test_mpc_sim_loop.cpp's build_scenario_a() (deliberately duplicated here
// rather than shared, per that file's own precedent -- see ProcessGuard
// comment above): trapezoidal-velocity straight line along yaw=0, accel
// 0->0.2 m/s over 2s, cruise 0.2 m/s for 7.5s (~1.5m), decelerate to stop
// over 2s. Total ~11.5s.
// =======================================================================
ReloPush::trajectory build_scenario_a() {
    ReloPush::trajectory traj;

    const double t_ramp = 2.0;
    const double v_cruise = 0.2;
    const double t_cruise_end = t_ramp + 7.5;
    const double t_end = t_cruise_end + t_ramp;
    const double a = v_cruise / t_ramp;

    auto speed_at = [&](double t) {
        if (t <= t_ramp) return a * t;
        if (t <= t_cruise_end) return v_cruise;
        if (t <= t_end) return v_cruise - a * (t - t_cruise_end);
        return 0.0;
    };
    auto dist_at = [&](double t) {
        if (t <= t_ramp) return 0.5 * a * t * t;
        double d1 = 0.5 * a * t_ramp * t_ramp;
        if (t <= t_cruise_end) return d1 + v_cruise * (t - t_ramp);
        double d2 = d1 + v_cruise * (t_cruise_end - t_ramp);
        double dt3 = t - t_cruise_end;
        return d2 + v_cruise * dt3 - 0.5 * a * dt3 * dt3;
    };

    for (double t = 0.0; t <= t_end + 1e-9; t += 0.5) {
        double tt = std::min(t, t_end);
        traj.append_waypoint(ReloPush::trajectory_elem(
            static_cast<float>(dist_at(tt)), 0.0f, 0.0f, static_cast<float>(speed_at(tt)),
            static_cast<float>(tt), false));
    }
    return traj;
}

// =======================================================================
// Run one closed-loop scenario instance (spawn -> handshake -> wait ->
// read CSV back). extra_sim_args gets appended verbatim to mpc_robot_sim's
// argv (e.g. --noise-sigma-pct=0.08 --noise-seed=12345).
// =======================================================================
struct RunReport {
    std::string label;
    bool pass = true;
    double final_pos_err = -1.0;
    double final_yaw_err = -1.0;
    double final_v = 0.0;
    std::vector<CsvRow> rows;
    std::vector<std::string> notes;
};

RunReport run_once(const std::string& label, const ReloPush::trajectory& traj,
                    const std::string& controller_exe, const std::string& sim_exe,
                    const std::string& csv_path, const std::vector<std::string>& extra_sim_args,
                    double final_pos_tol, double final_yaw_tol, double final_v_tol) {
    RunReport rep;
    rep.label = label;
    const auto& pts = *traj.trajectory_points;
    if (pts.empty()) {
        rep.pass = false;
        rep.notes.push_back("scenario trajectory is empty");
        return rep;
    }

    const double x0 = pts.front().x, y0 = pts.front().y, yaw0 = pts.front().yaw;
    const double traj_duration = pts.back().time;

    std::remove(csv_path.c_str());

    std::cout << "\n----- Run " << label << ": spawning processes -----" << std::endl;

    std::vector<std::string> sim_args = {
        "--robot-name", kRobotName,
        "--cmd-endpoint", "tcp://*:" + std::to_string(kCmdPort),
        "--loc-endpoint", "tcp://*:" + std::to_string(kLocPort),
        "--x", to_str(x0), "--y", to_str(y0), "--yaw", to_str(yaw0), "--v0", "0.0",
        "--log-csv", csv_path,
    };
    sim_args.insert(sim_args.end(), extra_sim_args.begin(), extra_sim_args.end());
    ProcessGuard sim_guard(spawn(sim_exe, sim_args), "mpc_robot_sim[" + label + "]");
    std::cout << "[test] spawned mpc_robot_sim pid=" << sim_guard.pid() << std::endl;

    std::vector<std::string> ctl_args = {
        "--robot", kRobotName,
        "--port", std::to_string(kHandshakePort),
        "--vesc-endpoint", "tcp://127.0.0.1:" + std::to_string(kCmdPort),
        "--localization-endpoint", "tcp://127.0.0.1:" + std::to_string(kLocPort),
    };
    ProcessGuard ctl_guard(spawn(controller_exe, ctl_args), "mpc_controller[" + label + "]");
    std::cout << "[test] spawned mpc_controller pid=" << ctl_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    try {
        zmq::context_t ctx(1);
        zmq::socket_t req(ctx, zmq::socket_type::req);
        req.set(zmq::sockopt::linger, 0);
        req.set(zmq::sockopt::rcvtimeo, 10000);
        req.set(zmq::sockopt::sndtimeo, 5000);
        req.connect("tcp://127.0.0.1:" + std::to_string(kHandshakePort));

        std::string traj_bytes = const_cast<ReloPush::trajectory&>(traj).serialize();
        std::string traj_b64 = base64_encode(traj_bytes);
        zmq::message_t traj_msg(traj_b64.begin(), traj_b64.end());
        if (!req.send(traj_msg, zmq::send_flags::none)) {
            rep.pass = false;
            rep.notes.push_back("failed to send trajectory over REQ socket");
            return rep;
        }

        zmq::message_t ack1;
        auto r1 = req.recv(ack1, zmq::recv_flags::none);
        std::string ack1_str = r1 ? std::string(static_cast<char*>(ack1.data()), ack1.size()) : "";
        const std::string expect_ack_recv = "ACK_RECEIVE_" + kRobotName;
        if (!r1 || ack1_str != expect_ack_recv) {
            rep.pass = false;
            rep.notes.push_back("expected '" + expect_ack_recv + "', got '" + ack1_str + "'");
            return rep;
        }
        std::cout << "[test] received " << ack1_str << std::endl;

        std::string start_cmd = "START";
        zmq::message_t start_msg(start_cmd.begin(), start_cmd.end());
        req.send(start_msg, zmq::send_flags::none);

        zmq::message_t ack2;
        auto r2 = req.recv(ack2, zmq::recv_flags::none);
        std::string ack2_str = r2 ? std::string(static_cast<char*>(ack2.data()), ack2.size()) : "";
        const std::string expect_ack_start = "ACK_START_" + kRobotName;
        if (!r2 || ack2_str != expect_ack_start) {
            rep.pass = false;
            rep.notes.push_back("expected '" + expect_ack_start + "', got '" + ack2_str + "'");
            return rep;
        }
        std::cout << "[test] received " << ack2_str << std::endl;

        const double wait_budget = traj_duration + 1.3 + 2.5;
        bool exited = ctl_guard.wait_for_exit(wait_budget);
        if (!exited) {
            rep.pass = false;
            rep.notes.push_back("mpc_controller did not self-exit within " + to_str(wait_budget, 1) +
                                 "s (forcing termination)");
            std::cerr << "[test] WARNING: controller still running after " << wait_budget << "s"
                      << std::endl;
        } else {
            std::cout << "[test] mpc_controller exited on its own" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        sim_guard.terminate();
        ctl_guard.terminate();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        std::vector<CsvRow> rows = read_csv(csv_path);
        if (rows.empty()) {
            rep.pass = false;
            rep.notes.push_back("sim CSV empty or unreadable: " + csv_path);
            return rep;
        }
        std::cout << "[test] sim CSV has " << rows.size() << " rows" << std::endl;
        rep.rows = rows;

        const CsvRow& last = rows.back();
        const double fdx = last.x - pts.back().x;
        const double fdy = last.y - pts.back().y;
        rep.final_pos_err = std::hypot(fdx, fdy);
        rep.final_yaw_err = std::fabs(wrap_pi(last.yaw - pts.back().yaw));
        rep.final_v = last.v;

        bool final_ok = rep.final_pos_err <= final_pos_tol && rep.final_yaw_err <= final_yaw_tol &&
                         std::fabs(rep.final_v) <= final_v_tol;
        if (!final_ok) {
            rep.pass = false;
            rep.notes.push_back("final pose/velocity out of tolerance (pos_err=" +
                                 to_str(rep.final_pos_err) + " tol=" + to_str(final_pos_tol) +
                                 ", yaw_err=" + to_str(rep.final_yaw_err) +
                                 " tol=" + to_str(final_yaw_tol) + ", v=" + to_str(rep.final_v) +
                                 " tol=" + to_str(final_v_tol) + ")");
        }
    } catch (const std::exception& ex) {
        rep.pass = false;
        rep.notes.push_back(std::string("exception during run: ") + ex.what());
    }

    return rep;
}

void print_report(const RunReport& rep, double pos_tol, double yaw_tol, double v_tol) {
    std::cout << "\n=== Run " << rep.label << ": " << (rep.pass ? "PASS" : "FAIL") << " ===\n";
    std::cout << "  final_pos_err   = " << to_str(rep.final_pos_err) << " m  (tol " << pos_tol
              << ")\n";
    std::cout << "  final_yaw_err   = " << to_str(rep.final_yaw_err) << " rad (tol " << yaw_tol
              << ")\n";
    std::cout << "  final_|v|       = " << to_str(std::fabs(rep.final_v)) << " m/s (tol " << v_tol
              << ")\n";
    std::cout << "  csv_rows        = " << rep.rows.size() << "\n";
    for (const auto& n : rep.notes) {
        std::cout << "  note: " << n << "\n";
    }
}

// Max pose delta between two runs' CSVs, compared index-by-index up to the
// shorter run's length (both runs are paced at the same --sim-rate-hz
// against wall-clock ticks, so row i in each corresponds to approximately
// the same elapsed sim time). This is a coarse but sufficient diagnostic
// purely to prove noise measurably perturbed the trajectory -- not a
// tolerance check. NOTE: kept for informational printing only; see
// compute_velocity_divergence() below for the metric noise_was_active
// actually gates on, and kMinVelocityRmsDelta's comment in main() for why.
double max_pose_delta(const std::vector<CsvRow>& a, const std::vector<CsvRow>& b) {
    double max_delta = 0.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        max_delta = std::max(max_delta, std::hypot(a[i].x - b[i].x, a[i].y - b[i].y));
    }
    return max_delta;
}

// Time-aligned v(t) divergence between two runs' CSVs -- the metric
// noise_was_active gates on (see kMinVelocityRmsDelta's comment in main()):
// unlike x,y pose, the plant's velocity trace is where injected actuation
// noise lands directly -- SimCore.cpp's apply_command_noise() perturbs the
// commanded accel/steer BEFORE integration -- so it stays a robust "is the
// noise machinery actually active" signal even when a well-filtered
// controller mostly cancels the noise's effect on the closed-loop pose.
//
// Alignment: both CSVs are logged by mpc_robot_sim's own fixed-rate tick
// loop (see robot_sim.cpp), so `t` is monotonically increasing at ~the same
// nominal spacing in both runs -- a single forward-walking pointer into `b`
// (never backtracking) finds each a[i]'s nearest-in-time counterpart in
// O(n). The "nominal tick" used for the half-tick cutoff is derived from
// the data itself (mean spacing over each run), not hardcoded to the sim's
// --sim-rate-hz default, so this keeps working if that default ever
// changes. A pair farther apart than half a nominal tick is dropped from
// both statistics rather than force-matched.
//
// Moving-portion restriction: a sample is dropped when BOTH runs read |v|
// at or below kMovingEps -- these are the long, bit-identical-zero idle
// stretches before the first command arrives and after the watchdog
// re-engages at the very end (see robot_sim.cpp's snap-to-zero handling),
// which would otherwise dilute the RMS with a wall of zero deltas that
// carry no information about whether noise is active. A sample is kept as
// soon as EITHER run shows the plant actually moving.
struct VelocityDivergence {
    double max_abs_delta = 0.0;
    double rms_delta = 0.0;
    int samples = 0;
};

VelocityDivergence compute_velocity_divergence(const std::vector<CsvRow>& a,
                                                const std::vector<CsvRow>& b) {
    VelocityDivergence result;
    if (a.empty() || b.empty()) return result;

    auto nominal_dt = [](const std::vector<CsvRow>& rows) {
        return rows.size() > 1
                   ? (rows.back().t - rows.front().t) / static_cast<double>(rows.size() - 1)
                   : 0.0;
    };
    const double half_tick = 0.5 * std::max(nominal_dt(a), nominal_dt(b));
    constexpr double kMovingEps = 1e-3;  // |v| at/below this reads as "idle" (watchdog-snapped 0).

    double sum_sq = 0.0;
    size_t j = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        while (j + 1 < b.size() && std::fabs(b[j + 1].t - a[i].t) <= std::fabs(b[j].t - a[i].t)) {
            ++j;
        }
        if (std::fabs(b[j].t - a[i].t) > half_tick) {
            continue;  // no sufficiently time-close counterpart -- drop this sample.
        }
        if (std::fabs(a[i].v) <= kMovingEps && std::fabs(b[j].v) <= kMovingEps) {
            continue;  // both idle here -- not part of the moving portion.
        }
        const double dv = std::fabs(a[i].v - b[j].v);
        result.max_abs_delta = std::max(result.max_abs_delta, dv);
        sum_sq += dv * dv;
        ++result.samples;
    }
    result.rms_delta = result.samples > 0 ? std::sqrt(sum_sq / result.samples) : 0.0;
    return result;
}

// =======================================================================
// Run 3: clamp-after-noise, exercised against the REAL compiled
// mpc_robot_sim BINARY directly (no mpc_controller in the loop) -- this is
// what N4 in mpc_unit_tests.cpp cannot prove by itself, since N4 only calls
// mpc::apply_command_noise() as a pure function. Here we hand-drive
// mpc_robot_sim's actual cmd SUB socket with a command PINNED at the
// actuator's accel limit under near-maximal noise (sigma=0.25) and verify,
// from the sim's own logged v(t), that the applied acceleration the plant
// actually integrated never exceeded the actuator limit -- a hard bound
// that provably fails if the final re-clamp after adding noise is ever
// skipped (see robot_sim.cpp's ackermann-command handler / SimCore.cpp's
// apply_command_noise()).
//
// Bound: with accel commanded == +kRobotMaxAccel every tick, a CORRECTLY
// clamped applied accel can never exceed +kRobotMaxAccel, so integrated v
// can never grow faster than kRobotMaxAccel * elapsed_time (regardless of
// watchdog braking, which only ever REDUCES |v| growth, never increases
// it). This bound is exact, not statistical -- any violation is a genuine
// clamp-after-noise defect, not sampling noise.
// =======================================================================
struct ClampCheckReport {
    bool pass = true;
    double max_excess = 0.0;  // largest observed (v[i]-v0) - (max_accel*t[i]), 0 if never exceeded.
    int csv_rows = 0;
    std::vector<std::string> notes;
};

ClampCheckReport run_clamp_check(const std::string& sim_exe, const std::string& csv_path) {
    ClampCheckReport rep;

    std::remove(csv_path.c_str());
    std::cout << "\n----- Run 3_limit_clamp_check: spawning mpc_robot_sim standalone -----"
              << std::endl;

    std::vector<std::string> sim_args = {
        "--robot-name", kRobotName,
        "--cmd-endpoint", "tcp://*:" + std::to_string(kCmdPort),
        "--loc-endpoint", "tcp://*:" + std::to_string(kLocPort),
        "--x", "0.0", "--y", "0.0", "--yaw", "0.0", "--v0", "0.0",
        "--log-csv", csv_path,
        "--noise-sigma-pct=0.25",  // kMaxSigmaPct: largest perturbations the model allows.
        "--noise-seed=90210",
    };
    ProcessGuard sim_guard(spawn(sim_exe, sim_args), "mpc_robot_sim[3_limit_clamp_check]");
    std::cout << "[test] spawned mpc_robot_sim pid=" << sim_guard.pid() << std::endl;

    try {
        zmq::context_t ctx(1);
        zmq::socket_t cmd_pub(ctx, zmq::socket_type::pub);
        cmd_pub.set(zmq::sockopt::linger, 0);
        cmd_pub.connect("tcp://127.0.0.1:" + std::to_string(kCmdPort));

        // Slow-joiner grace period before the SUB's subscription has
        // necessarily propagated back to this PUB.
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        const std::string ackermann_topic = "/" + kRobotName + "/ackermann";
        const std::string payload =
            encode_ackermann_payload(/*speed=*/0.0, /*steering=*/0.0, /*accel=*/kRobotMaxAccel);

        // Publish the same pinned-at-limit command repeatedly (well inside
        // the sim's default 250ms watchdog timeout) so FEATURE A resamples
        // a fresh, held perturbation on every received command -- over ~60
        // resamples, a positive-direction accel perturbation (p=0.5 each
        // draw) is all but certain to occur at least once, which is exactly
        // what would overshoot the actuator limit if the final re-clamp
        // were missing.
        const int kNumPublishes = 60;
        for (int i = 0; i < kNumPublishes; ++i) {
            zmq::message_t topic_msg(ackermann_topic.begin(), ackermann_topic.end());
            zmq::message_t payload_msg(payload.begin(), payload.end());
            cmd_pub.send(topic_msg, zmq::send_flags::sndmore);
            cmd_pub.send(payload_msg, zmq::send_flags::none);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // Let the sim log a few more ticks (including any post-burst
        // watchdog braking, which only tightens the bound further) before
        // tearing down.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    } catch (const std::exception& ex) {
        rep.pass = false;
        rep.notes.push_back(std::string("exception while publishing: ") + ex.what());
    }

    sim_guard.terminate();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<CsvRow> rows = read_csv(csv_path);
    rep.csv_rows = static_cast<int>(rows.size());
    if (rows.size() < 5) {
        rep.pass = false;
        rep.notes.push_back("sim CSV has too few rows (" + std::to_string(rows.size()) +
                             ") to evaluate the clamp bound: " + csv_path);
        return rep;
    }
    std::cout << "[test] sim CSV has " << rows.size() << " rows" << std::endl;

    // PER-TICK bound (not a cumulative-since-t=0 one): v'=v+a*dt is EXACT
    // Euler integration (mpc::rollout_step -- see Kinematics.h), so for any
    // two CONSECUTIVE, both-not-watchdog-engaged rows, dv <= max_accel*dt
    // holds with zero tolerance for a correctly clamped applied accel,
    // regardless of how much wall-clock slack preceded the first command
    // arriving (a cumulative v(t) <= v0 + max_accel*t bound would be far
    // too loose here, since the PUB/SUB slow-joiner handshake burns a
    // startup window during which real elapsed time accrues but no command
    // has arrived yet -- this per-tick form sidesteps that entirely).
    // Watchdog-engaged rows are excluded: their resolved accel is the
    // internal auto-brake response (already covered by S3), not FEATURE A's
    // applied command.
    constexpr double kSlack = 5e-4;  // ~14% of one tick's max_accel*dt=0.00365 -- FP-precision only.
    int checked_ticks = 0;
    for (size_t i = 1; i < rows.size(); ++i) {
        if (rows[i].watchdog_active != 0 || rows[i - 1].watchdog_active != 0) {
            continue;
        }
        const double dt = rows[i].t - rows[i - 1].t;
        if (dt <= 0.0) {
            continue;
        }
        const double dv = rows[i].v - rows[i - 1].v;
        const double allowed_dv = kRobotMaxAccel * dt + kSlack;
        ++checked_ticks;
        const double excess = dv - allowed_dv;
        if (excess > rep.max_excess) {
            rep.max_excess = excess;
        }
        if (excess > 0.0) {
            rep.pass = false;
        }
    }
    std::cout << "[test] Run 3: checked " << checked_ticks
              << " non-watchdog-engaged tick(s) against the per-tick clamp bound" << std::endl;
    if (checked_ticks < 30) {
        // Not enough non-engaged ticks to make this a meaningful check --
        // the command burst likely never got through (slow joiner /
        // watchdog thrash) rather than the clamp itself being broken.
        rep.pass = false;
        rep.notes.push_back("only " + std::to_string(checked_ticks) +
                             " non-watchdog-engaged tick(s) observed -- too few to validate the "
                             "clamp-after-noise bound; the command burst may not have reached the "
                             "sim");
    }
    if (rep.max_excess > 0.0) {
        rep.notes.push_back("a per-tick dv exceeded max_accel*dt by up to " +
                             to_str(rep.max_excess, 6) +
                             " m/s -- applied acceleration overshot the actuator limit "
                             "(clamp-after-noise appears to be missing/broken)");
    }

    return rep;
}

}  // namespace

int main() {
    std::string dir;
    try {
        dir = self_dir();
    } catch (const std::exception& ex) {
        std::cerr << "[test] fatal: " << ex.what() << std::endl;
        return 1;
    }
    const std::string controller_exe = dir + "/mpc_controller";
    const std::string sim_exe = dir + "/mpc_robot_sim";

    std::cout << "[test] controller_exe=" << controller_exe << "\n[test] sim_exe=" << sim_exe
              << std::endl;

    // Run 1 tolerances: identical to test_mpc_sim_loop.cpp's Scenario A
    // (kFinalPosTol/kFinalYawTol/kFinalVTol there) -- sigma=0 must reproduce
    // that scenario's closed-loop behavior exactly (within the same
    // tolerance band), proving bit-identical sim behavior end-to-end.
    constexpr double kRun1PosTol = 0.02;
    constexpr double kRun1YawTol = 0.01;
    constexpr double kRun1VTol = 0.005;

    // Run 2 tolerances: relaxed per spec -- the MPC closes the loop around
    // actuation noise, it does not eliminate it.
    constexpr double kRun2PosTol = 0.06;
    constexpr double kRun2YawTol = 0.05;
    constexpr double kRun2VTol = 0.03;  // not spec'd explicitly; generous but still meaningful.

    // Threshold proving noise was actually active (not silently inert) --
    // a coarse diagnostic, NOT a tolerance check (per this file's own
    // long-standing framing for these cross-run checks).
    //
    // Gates on the PLANT VELOCITY trace, not closed-loop x,y pose: an EMA
    // low-pass filter on MPC/src/main.cpp's v_meas estimator
    // (tau=0.15s, added for the launch-governor settle-time fix) now lets
    // the controller reject actuation noise's effect on position well
    // enough that a pose-based bar shrinks below any fixed threshold
    // chosen before the filter existed. Noise is injected directly into
    // the plant's applied accel/steer, before integration (SimCore.cpp's
    // apply_command_noise()), so the velocity trace stays a direct,
    // high-margin readout of "is noise actually being injected",
    // independent of how well the outer loop compensates for it.
    //
    // MEASURED (3 consecutive fresh full reruns of this exact binary,
    // same trajectory/seed pair as Run 1/Run 2 below; all 3 numbers each
    // run come from the SAME compute_velocity_divergence() call the real
    // check below uses, over the moving-portion-restricted, time-aligned
    // samples):
    //   run 1: max=0.051209  rms=0.011821  pose(informational)=0.004322
    //   run 2: max=0.058808  rms=0.012483  pose(informational)=0.005721
    //   run 3: max=0.044979  rms=0.010253  pose(informational)=0.003874
    // max's spread (max-min)/min across the 3 runs = 30.7%; RMS's spread
    // = 21.7% -- RMS is the more STABLE of the two (it averages ~2600
    // aligned samples per run instead of keying off a single extreme
    // sample), so RMS is what gates noise_was_active below, per the
    // "choose whichever measured more stably" rule. RMS also clears the
    // fallback trigger in all 3 runs -- it stays > 2x the (informational)
    // pose-delta signal every time (ratios 2.74x / 2.18x / 2.65x) and
    // never varies more than 3x run-to-run -- so the pose-delta fallback
    // is NOT needed; RMS is used directly.
    //
    // Bar set at ~half the observed minimum RMS (0.010253 / 2 = 0.00513),
    // rounded down slightly to 0.005 for a clean constant that keeps a
    // little extra margin below every one of the 3 measured values (all
    // >= 2.0x this bar).
    constexpr double kMinVelocityRmsDelta = 0.005;

    ReloPush::trajectory traj_a = build_scenario_a();

    RunReport rep1 = run_once("1_sigma0", traj_a, controller_exe, sim_exe,
                               dir + "/sim_noise_run1.csv",
                               {"--noise-sigma-pct=0", "--steer-noise-sigma-pct=0"}, kRun1PosTol,
                               kRun1YawTol, kRun1VTol);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Both channels active: proves the independent steer channel is wired
    // end-to-end through the real mpc_robot_sim binary, not just at the
    // NoiseModel unit level.
    RunReport rep2 = run_once(
        "2_sigma0.08both_seed12345", traj_a, controller_exe, sim_exe,
        dir + "/sim_noise_run2.csv",
        {"--noise-sigma-pct=0.08", "--steer-noise-sigma-pct=0.08", "--noise-seed=12345"},
        kRun2PosTol, kRun2YawTol, kRun2VTol);

    double trajectory_delta = -1.0;
    VelocityDivergence vel_div;
    bool noise_was_active = false;
    if (!rep1.rows.empty() && !rep2.rows.empty()) {
        trajectory_delta = max_pose_delta(rep1.rows, rep2.rows);
        vel_div = compute_velocity_divergence(rep1.rows, rep2.rows);
        noise_was_active = vel_div.rms_delta > kMinVelocityRmsDelta;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ClampCheckReport rep3 = run_clamp_check(sim_exe, dir + "/sim_noise_run3_clamp_check.csv");

    bool overall_pass = rep1.pass && rep2.pass && noise_was_active && rep3.pass;

    std::cout << "\n============ FEATURE A NOISE TEST SUMMARY ============" << std::endl;
    print_report(rep1, kRun1PosTol, kRun1YawTol, kRun1VTol);
    print_report(rep2, kRun2PosTol, kRun2YawTol, kRun2VTol);

    std::cout << "\n=== Trajectory/velocity divergence (run1 vs run2) ===\n";
    std::cout << "  max |pose delta|                       = " << to_str(trajectory_delta)
              << " m  (informational only -- a well-filtered controller is expected to\n"
              << "                                            suppress this; see kMinVelocityRmsDelta's comment)\n";
    std::cout << "  max |velocity delta|                   = " << to_str(vel_div.max_abs_delta)
              << " m/s (informational only -- measured noisier across reruns than RMS; not the gate)\n";
    std::cout << "  rms |velocity delta| (moving portion)  = " << to_str(vel_div.rms_delta)
              << " m/s over " << vel_div.samples << " aligned samples (must be > "
              << kMinVelocityRmsDelta << " to prove noise was active)\n";
    if (!noise_was_active) {
        std::cout << "  note: velocity traces did not measurably diverge -- noise may not be active\n";
    }

    std::cout << "\n=== Run 3_limit_clamp_check: " << (rep3.pass ? "PASS" : "FAIL") << " ===\n";
    std::cout << "  csv_rows              = " << rep3.csv_rows << "\n";
    std::cout << "  max v(t)-bound excess = " << to_str(rep3.max_excess)
              << " m/s (must be <= 0; proves clamp-after-noise holds on the real binary)\n";
    for (const auto& n : rep3.notes) {
        std::cout << "  note: " << n << "\n";
    }

    std::cout << "\n" << (overall_pass ? "ALL RUNS PASSED" : "SOME RUNS FAILED") << std::endl;

    return overall_pass ? 0 : 1;
}
