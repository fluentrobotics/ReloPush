// test_motor_calib_e2e.cpp -> binary `test_motor_calib_e2e`
//
// Standalone, hardware-free, end-to-end loop for the motor calibration
// tool. Spawns REAL child processes -- VescDriver/tests/fake_vesc (a pty
// VESC stand-in) and VescDriver/src/vesc_driver (the real driver binary) --
// and this TEST PROCESS ITSELF plays OptiTrack: it SUBs to the driver's own
// telemetry PUB for reported erpm, integrates ground-truth position at a
// KNOWN erpm_per_mps (4600, deliberately different from anything
// motor_calibration is told in advance), and PUBs plain-JSON
// {"x","y","yaw"} localization -- exactly the wire format
// optitrack_zmq_bridge/mpc_robot_sim produce. It then spawns the real
// `motor_calibration` binary (wrapped, output captured) against this
// closed loop and asserts on its real output files: per-trial CSVs,
// calibration_erpm.json (frozen schema, parsed via
// mpc::parse_calibration()), and campaign_summary.json.
//
// TEST PORTS ONLY (45930-45949 block):
//   45930  vesc_driver --ackermann-port (unused here -- motor_calibration
//          never publishes ackermann; only bound so vesc_driver can start)
//   45931  vesc_driver --control-port   (motor_calibration's REQ target)
//   45932  vesc_driver --telemetry-port (motor_calibration's + this test's
//          own OptiTrack-sim thread's telemetry SUB source)
//   45933  this test's own OptiTrack-sim PUB bind (motor_calibration's
//          localization_endpoint SUB target)
//
// Ground truth: v = erpm/4600, matching fake_vesc's --stall-erpm 800 (so
// commanded raw erpm below 800 never produces real motion, and
// motor_calibration's own fit_stall() should find min_cmd in (500,1000]
// given the sweep [500,1000,1500,2000,2500]).
//
// RAII process guards (ProcessGuard) + a background-thread guard
// (OptitrackSimThread) + a top-level try/catch in main() together
// guarantee no orphaned child processes/threads on any failure path (see
// ProcessGuard's own doc comment below for why the top-level catch is
// still required even with RAII locals).

#include "mpc/MotorCalibCore.h"

#include <zmq.hpp>

#include <nlohmann/json.hpp>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

// ---------------------------------------------------------------------
// Test-only ports/identity (see file header).
// ---------------------------------------------------------------------
constexpr int kAckermannPort = 45930;
constexpr int kControlPort = 45931;
constexpr int kTelemetryPort = 45932;
constexpr int kLocPort = 45933;
const std::string kRobotTopic = "caltest";
constexpr double kErpmPerMpsTrue = 4600.0;

// ---------------------------------------------------------------------
// RAII child-process guard -- reimplemented locally (mirrors
// VescDriver/tests/vesc_driver_tests.cpp's pty_test::ProcessGuard /
// MPC/tests/test_optitrack_bridge.cpp's identical, independently-
// duplicated implementation per this codebase's own established
// per-file-self-contained convention). SIGTERM, short grace period,
// SIGKILL fallback, idempotent, reaps in its destructor. Relies on
// main()'s top-level try/catch to guarantee this destructor still runs
// even if a check throws between spawn and an explicit terminate() call:
// an exception escaping uncaught calls std::terminate() WITHOUT unwinding
// locals, so a scope-local guard alone is not sufficient without a
// reachable catch somewhere in the call chain.
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

    void terminate() {
        if (pid_ <= 0) return;
        kill(pid_, SIGTERM);
        for (int i = 0; i < 100; ++i) {  // ~2s grace period
            int status = 0;
            const pid_t r = waitpid(pid_, &status, WNOHANG);
            if (r == pid_) {
                pid_ = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::cerr << "[test_motor_calib_e2e] pid " << pid_ << " (" << label_
                   << ") did not exit after SIGTERM; sending SIGKILL\n";
        kill(pid_, SIGKILL);
        waitpid(pid_, nullptr, 0);
        pid_ = -1;
    }

private:
    pid_t pid_ = -1;
    std::string label_;
};

std::string own_exe_dir() {
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "";
    buf[n] = '\0';
    const std::string path(buf);
    const size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

// Resolves a sibling binary's path: tries <own_exe_dir>/<name> first
// (co-located, the common case for targets defined in the SAME
// CMakeLists.txt), then <own_exe_dir>/../VescDriver/<name> (the actual
// top-level-superbuild layout: build/MPC/ and build/VescDriver/ are
// siblings under one build/ tree -- see VescDriver/CMakeLists.txt's own
// "builds standalone AND via add_subdirectory(VescDriver) from the repo
// root" comment), then falls back to bare `name` (PATH lookup) so a
// standalone VescDriver/build/ + MPC build/ arrangement still works if the
// caller's PATH is set up for it.
std::string resolve_sibling_exe(const std::string& name) {
    const std::string dir = own_exe_dir();
    if (!dir.empty()) {
        const std::string co_located = dir + "/" + name;
        if (access(co_located.c_str(), X_OK) == 0) return co_located;
        const std::string vesc_sibling = dir + "/../VescDriver/" + name;
        if (access(vesc_sibling.c_str(), X_OK) == 0) return vesc_sibling;
    }
    return name;
}

// Spawns `exe` (wrapped per this repo's standing env rule) with `args`,
// redirecting its stdout+stderr to `stdout_path`. Mirrors
// vesc_driver_tests.cpp's pty_test::spawn_wrapped_capture_stdout exactly
// (reimplemented locally -- see this file's header comment).
pid_t spawn_wrapped_capture_stdout(const std::string& exe, const std::vector<std::string>& args,
                                     const std::string& stdout_path) {
    const char* home = std::getenv("HOME");
    const char* user = std::getenv("USER");
    std::vector<std::string> wrapped_args = {
        "-i", std::string("HOME=") + (home ? home : ""),
        std::string("USER=") + (user ? user : ""), "PATH=/usr/local/bin:/usr/bin:/bin",
        "QT_QPA_PLATFORM=offscreen", exe};
    for (const auto& a : args) wrapped_args.push_back(a);

    const std::string env_exe = "/usr/bin/env";
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(env_exe.c_str()));
    for (auto& a : wrapped_args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);

    const pid_t pid = fork();
    if (pid < 0) throw std::runtime_error("fork() failed: " + std::string(std::strerror(errno)));
    if (pid == 0) {
        const int fd = open(stdout_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
        }
        execv(env_exe.c_str(), argv.data());
        std::fprintf(stderr, "[test_motor_calib_e2e] execv('%s') failed\n", env_exe.c_str());
        _exit(127);
    }
    return pid;
}

std::string read_file(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string wait_for_pty_line(const std::string& stdout_path, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        const std::string contents = read_file(stdout_path);
        const size_t pos = contents.find("PTY ");
        if (pos != std::string::npos) {
            size_t end = contents.find('\n', pos);
            const std::string line =
                (end == std::string::npos) ? contents.substr(pos) : contents.substr(pos, end - pos);
            return line.substr(4);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return "";
}

// Spawns `exe` wrapped, waits up to `timeout_s` for it to exit on its own
// (polling, never a blocking waitpid -- consistent with every other
// process-wait loop in this file/this codebase). If it hasn't exited by
// the deadline, SIGTERMs it (still reaping) and reports timed_out=true.
struct RunResult {
    bool exited = false;
    int exit_code = -1;
    bool timed_out = false;
};
RunResult run_wrapped_and_wait(const std::string& exe, const std::vector<std::string>& args,
                                 const std::string& stdout_path, double timeout_s) {
    const pid_t pid = spawn_wrapped_capture_stdout(exe, args, stdout_path);
    RunResult r;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    int status = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const pid_t w = waitpid(pid, &status, WNOHANG);
        if (w == pid) {
            r.exited = true;
            r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            return r;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    r.timed_out = true;
    kill(pid, SIGTERM);
    waitpid(pid, &status, 0);
    return r;
}

// ---------------------------------------------------------------------
// Bare ZMQ REQ/REP round trip, for this test's own direct pings against
// the driver's control port (mirrors vesc_driver_tests.cpp's zmq_test::
// req_roundtrip, reimplemented locally with cppzmq instead of raw zmq.h
// since MPC/ (unlike VescDriver/) freely uses cppzmq elsewhere).
// ---------------------------------------------------------------------
bool req_roundtrip(zmq::socket_t& req, const std::string& request_json, std::string* reply_json) {
    zmq::message_t msg(request_json.data(), request_json.size());
    auto sent = req.send(msg, zmq::send_flags::none);
    if (!sent.has_value()) return false;
    zmq::message_t reply;
    auto recvd = req.recv(reply, zmq::recv_flags::none);
    if (!recvd.has_value()) return false;
    reply_json->assign(static_cast<char*>(reply.data()), reply.size());
    return true;
}

// ---------------------------------------------------------------------
// This test process's own OptiTrack stand-in: a background thread that
// SUBs to the driver's telemetry PUB for reported erpm, integrates
// ground-truth x at kErpmPerMpsTrue, and PUBs plain-JSON localization --
// see this file's header comment for the full contract. RAII: the
// destructor signals stop and joins, so it tears down cleanly on every
// exit path (including an exception unwinding through main(), given the
// top-level try/catch -- see ProcessGuard's doc comment above for why
// that catch matters).
// ---------------------------------------------------------------------
class OptitrackSimThread {
public:
    OptitrackSimThread() : stop_(false), last_x_(0.9) {
        thread_ = std::thread([this]() { run(); });
    }
    ~OptitrackSimThread() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }
    double last_x() const { return last_x_.load(); }

private:
    void run() {
        zmq::context_t ctx(1);
        zmq::socket_t pub(ctx, zmq::socket_type::pub);
        pub.set(zmq::sockopt::linger, 0);
        pub.bind("tcp://*:" + std::to_string(kLocPort));

        zmq::socket_t telem_sub(ctx, zmq::socket_type::sub);
        telem_sub.set(zmq::sockopt::linger, 0);
        telem_sub.connect("tcp://127.0.0.1:" + std::to_string(kTelemetryPort));
        telem_sub.set(zmq::sockopt::subscribe, "/" + kRobotTopic + "/vesc_telemetry");

        const std::string loc_topic = "/" + kRobotTopic + "/localization";
        double x = 0.9;   // line start, per this file's header comment.
        double erpm = 0.0;
        auto last_tick = std::chrono::steady_clock::now();

        while (!stop_.load()) {
            while (true) {
                zmq::message_t topic_msg;
                auto r1 = telem_sub.recv(topic_msg, zmq::recv_flags::dontwait);
                if (!r1.has_value()) break;
                if (!topic_msg.more()) break;
                zmq::message_t payload_msg;
                auto r2 = telem_sub.recv(payload_msg, zmq::recv_flags::none);
                if (!r2.has_value()) break;
                const std::string payload(static_cast<char*>(payload_msg.data()), payload_msg.size());
                const nlohmann::json j = nlohmann::json::parse(payload, nullptr, false);
                if (!j.is_discarded() && j.is_object() && j.contains("erpm")) {
                    erpm = j.at("erpm").get<double>();
                }
            }

            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last_tick).count();
            last_tick = now;
            x += (erpm / kErpmPerMpsTrue) * dt;
            last_x_.store(x);

            nlohmann::json pose;
            pose["x"] = x;
            pose["y"] = 0.0;
            pose["yaw"] = 0.0;
            const std::string payload = pose.dump();
            zmq::message_t topic_msg(loc_topic.data(), loc_topic.size());
            zmq::message_t payload_msg(payload.data(), payload.size());
            pub.send(topic_msg, zmq::send_flags::sndmore);
            pub.send(payload_msg, zmq::send_flags::none);

            std::this_thread::sleep_for(std::chrono::milliseconds(33));  // ~30Hz
        }
    }

    std::thread thread_;
    std::atomic<bool> stop_;
    std::atomic<double> last_x_;
};

// ---------------------------------------------------------------------
// Small check-accumulator, mirroring every other test binary in this repo
// (mpc_unit_tests.cpp/optitrack_unit_tests.cpp/vesc_driver_tests.cpp): no
// gtest dependency, just an ok&= accumulator with printed [PASS]/[FAIL]
// lines.
// ---------------------------------------------------------------------
bool check_true(bool cond, const std::string& what) {
    std::cout << (cond ? "  [PASS] " : "  [FAIL] ") << what << "\n";
    return cond;
}

// ---------------------------------------------------------------------
// CSV sanity check: header matches TrialRunner::write_csv()'s documented
// header exactly, row count > min_rows, and a middle data row's s/v_raw
// columns parse to finite, physically-sane numbers.
// ---------------------------------------------------------------------
bool check_trial_csv(const std::string& path, int min_rows, std::string* note) {
    std::ifstream f(path);
    if (!f.is_open()) {
        *note = "could not open '" + path + "'";
        return false;
    }
    std::string header;
    std::getline(f, header);
    const std::string expected_header =
        "t,x,y,yaw,s,v_raw,v_smooth,cmd_mode,cmd_value,erpm,duty,current_motor,v_in";
    if (header != expected_header) {
        *note = "'" + path + "' header mismatch: got '" + header + "'";
        return false;
    }
    std::vector<std::string> rows;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty()) rows.push_back(line);
    }
    if (static_cast<int>(rows.size()) <= min_rows) {
        *note = "'" + path + "' has only " + std::to_string(rows.size()) + " rows (need > " +
                std::to_string(min_rows) + ")";
        return false;
    }
    // Sanity-check a middle row's s (col 4) / v_raw (col 5) columns.
    const std::string& mid = rows[rows.size() / 2];
    std::vector<std::string> cols;
    std::stringstream ss(mid);
    std::string tok;
    while (std::getline(ss, tok, ',')) cols.push_back(tok);
    if (cols.size() < 9) {
        *note = "'" + path + "' middle row has too few columns";
        return false;
    }
    try {
        const double s = std::stod(cols[4]);
        const double v_raw = std::stod(cols[5]);
        if (!std::isfinite(s) || !std::isfinite(v_raw) || std::fabs(v_raw) > 5.0) {
            *note = "'" + path + "' middle row s/v_raw not sane (s=" + cols[4] + " v_raw=" + cols[5] + ")";
            return false;
        }
    } catch (const std::exception& ex) {
        *note = "'" + path + "' middle row s/v_raw failed to parse: " + ex.what();
        return false;
    }
    *note = "'" + path + "' ok (" + std::to_string(rows.size()) + " rows)";
    return true;
}

nlohmann::json load_json_file(const std::string& path, bool* ok) {
    std::ifstream f(path);
    if (!f.is_open()) {
        *ok = false;
        return nlohmann::json();
    }
    nlohmann::json j;
    try {
        f >> j;
    } catch (const std::exception&) {
        *ok = false;
        return nlohmann::json();
    }
    *ok = true;
    return j;
}

}  // namespace

int main() {
    bool ok = true;

    try {
        const std::string fake_vesc_exe = resolve_sibling_exe("fake_vesc");
        const std::string vesc_driver_exe = resolve_sibling_exe("vesc_driver");
        const std::string motor_calibration_exe = resolve_sibling_exe("motor_calibration");
        if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
            return check_true(false, "fake_vesc executable found (tried '" + fake_vesc_exe +
                                          "') -- build VescDriver first") ? 1 : 1;
        }
        if (access(vesc_driver_exe.c_str(), X_OK) != 0) {
            return check_true(false, "vesc_driver executable found (tried '" + vesc_driver_exe +
                                          "') -- build VescDriver first") ? 1 : 1;
        }
        if (access(motor_calibration_exe.c_str(), X_OK) != 0) {
            return check_true(false, "motor_calibration executable found (tried '" +
                                          motor_calibration_exe + "') -- build MPC first") ? 1 : 1;
        }
        std::cout << "[test] fake_vesc=" << fake_vesc_exe << "\n";
        std::cout << "[test] vesc_driver=" << vesc_driver_exe << "\n";
        std::cout << "[test] motor_calibration=" << motor_calibration_exe << "\n";

        const std::string tmp_dir =
            "/tmp/test_motor_calib_e2e_" + std::to_string(static_cast<long>(getpid()));
        std::filesystem::create_directories(tmp_dir);
        const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
        const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
        const std::string driver_stdout_path = tmp_dir + "/vesc_driver_stdout.txt";
        const std::string driver_config_path = tmp_dir + "/driver_config.json";

        {
            std::ofstream f(driver_config_path);
            f << "{"
                 "\"mode\":\"erpm\","
                 "\"cmd_per_mps\":4600.0,"
                 "\"cmd_offset\":0.0,"
                 "\"erpm_per_mps\":4600.0,"
                 "\"servo\":{\"enabled\":false},"
                 "\"safety\":{\"max_erpm\":6000.0,\"max_duty\":1.0,\"max_current\":100.0,"
                 "\"safety_max_accel\":3.5,\"safety_max_v\":0.6},"
                 "\"watchdog_ms\":250,"
                 "\"control_rate_hz\":50,"
                 "\"telemetry_rate_hz\":20,"
                 "\"kick\":{\"enabled\":false}"
                 "}";
        }

        // --- spawn fake_vesc (pty) --------------------------------------
        const std::vector<std::string> fake_args = {"--fw",         "5.2", "--tau-s", "0.15",
                                                       "--stall-erpm", "800", "--log",   fake_log_path};
        ProcessGuard fake_guard(spawn_wrapped_capture_stdout(fake_vesc_exe, fake_args, fake_stdout_path),
                                  "fake_vesc");
        const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
        ok &= check_true(!slave_path.empty(), "fake_vesc printed its PTY slave path within 5s");
        if (slave_path.empty()) return ok ? 0 : 1;
        std::cout << "[test] fake_vesc pty slave = " << slave_path << "\n";

        // --- spawn vesc_driver -------------------------------------------
        const std::vector<std::string> driver_args = {
            "--config",         driver_config_path,
            "--robot",          kRobotTopic,
            "--serial-port",    slave_path,
            "--ackermann-port", std::to_string(kAckermannPort),
            "--control-port",   std::to_string(kControlPort),
            "--telemetry-port", std::to_string(kTelemetryPort),
        };
        ProcessGuard driver_guard(
            spawn_wrapped_capture_stdout(vesc_driver_exe, driver_args, driver_stdout_path), "vesc_driver");
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        // --- this test plays OptiTrack ------------------------------------
        OptitrackSimThread optitrack_sim;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let PUB/SUB subscriptions settle.

        // =================================================================
        // Run 1: the main erpm sweep.
        // =================================================================
        const std::string out_dir1 = tmp_dir + "/out1";
        std::filesystem::create_directories(out_dir1);
        const std::string config1_path = tmp_dir + "/motor_calib_config1.json";
        {
            std::ofstream f(config1_path);
            f << "{"
                 "\"robot_ip\":\"127.0.0.1\","
                 "\"control_port\":" << kControlPort << ","
                 "\"telemetry_port\":" << kTelemetryPort << ","
                 "\"localization_endpoint\":\"tcp://127.0.0.1:" << kLocPort << "\","
                 "\"robot_topic_name\":\"" << kRobotTopic << "\","
                 "\"motive_body_name\":\"" << kRobotTopic << "\","
                 "\"line\":{\"length_m\":4.5,\"end_margin_m\":0.75,\"line_yaw_rad\":0.0},"
                 // max_speed_abort_mps is loosened from the shipped default (0.8) to 2.0: true
                 // commanded speeds here top out around 2500/4600=0.54 m/s, so this stays a
                 // meaningful safety ceiling while tolerating the occasional velocity-estimate
                 // spike that OS scheduling jitter can still produce even after
                 // motor_calibration's own burst-debounce (see LiveChannels::pump()'s doc
                 // comment) -- several wrapped subprocesses share this host in this test.
                 "\"safety\":{\"mocap_stale_abort_ms\":300,\"max_speed_abort_mps\":2.0},"
                 "\"sweeps\":{\"duty\":[],\"erpm\":[500,1000,1500,2000,2500]},"
                 // settle_s/steady_window_frac widened beyond the shipped defaults (1.5/0.4) to
                 // ~3x the trailing sample count feeding steady_state_v(): true convergence is
                 // essentially complete within a few multiples of fake_vesc's tau=0.15s, so the
                 // wider window costs nothing in bias, only reduces variance from residual
                 // receive-side timing jitter in the velocity estimate (see
                 // LiveChannels::pump()'s BURST-DEBOUNCE comment in motor_calibration.cpp) --
                 // still comfortably under max_duration_s once the STOPPING phase is added.
                 "\"trial\":{\"max_duration_s\":5.5,\"settle_s\":2.5,\"arm_settle_s\":1.0,"
                 "\"steady_window_frac\":0.6},"
                 "\"output_dir\":\"" << out_dir1 << "\""
                 "}";
        }

        const std::string mc1_stdout_path = tmp_dir + "/motor_calibration_run1_stdout.txt";
        std::cout << "[test] running motor_calibration (main erpm sweep)...\n";
        const RunResult r1 = run_wrapped_and_wait(
            motor_calibration_exe, {"--config", config1_path, "--modes", "erpm", "--max-trials", "0"},
            mc1_stdout_path, 120.0);
        ok &= check_true(!r1.timed_out, "motor_calibration (run 1) did not time out");
        ok &= check_true(r1.exited && r1.exit_code == 0,
                          "motor_calibration (run 1) exited 0 (got exit_code=" +
                              std::to_string(r1.exit_code) + ")");
        if (!r1.exited || r1.exit_code != 0) {
            std::cout << "----- motor_calibration run1 stdout -----\n"
                       << read_file(mc1_stdout_path) << "\n------------------------------------------\n";
        }

        // --- campaign_summary.json: load first so the CSV check below can
        // use its own "csv_path"/"value" fields -- robust against
        // motor_calibration's insufficient-room direction-flip retry (see
        // that file's own doc comment), which means a given sweep
        // magnitude's ACTUAL commanded sign is not always predictable from
        // the config's sweep order alone.
        nlohmann::json summary1;
        bool summary1_loaded = false;
        {
            summary1 = load_json_file(out_dir1 + "/campaign_summary.json", &summary1_loaded);
            ok &= check_true(summary1_loaded, "campaign_summary.json (run 1) exists and parses as JSON");
        }

        // --- 5 trial CSVs, sane rows -------------------------------------
        if (summary1_loaded && summary1.contains("trials") && summary1.at("trials").is_array()) {
            const auto& trials = summary1.at("trials");
            ok &= check_true(trials.size() == 5, "campaign_summary.json (run 1) has 5 trial entries "
                                                    "(got " + std::to_string(trials.size()) + ")");
            for (const auto& t : trials) {
                const std::string csv_path = t.value("csv_path", std::string());
                if (csv_path.empty()) {
                    ok &= check_true(false, "trial entry missing csv_path");
                    continue;
                }
                std::string note;
                const bool csv_ok = check_trial_csv(csv_path, 30, &note);
                ok &= check_true(csv_ok, note);
            }
        } else {
            ok &= check_true(false, "campaign_summary.json (run 1) has a 'trials' array to derive CSV "
                                      "paths from");
        }

        // --- calibration_erpm.json, frozen schema -------------------------
        {
            bool loaded = false;
            const nlohmann::json calib_json =
                load_json_file(out_dir1 + "/calibration_erpm.json", &loaded);
            ok &= check_true(loaded, "calibration_erpm.json exists and parses as JSON");
            if (loaded) {
                try {
                    const mpc::ParsedCalibration parsed = mpc::parse_calibration(calib_json);
                    ok &= check_true(parsed.version == 1, "calibration_erpm.json version==1");
                    ok &= check_true(parsed.mode == "erpm", "calibration_erpm.json mode==\"erpm\"");
                    ok &= check_true(parsed.robot_name == kRobotTopic,
                                      "calibration_erpm.json robot_name==\"" + kRobotTopic + "\"");
                    const double rel_err = std::fabs(parsed.cmd_per_mps - kErpmPerMpsTrue) / kErpmPerMpsTrue;
                    ok &= check_true(rel_err <= 0.12,
                                      "cmd_per_mps=" + std::to_string(parsed.cmd_per_mps) +
                                          " within 12% of " + std::to_string(kErpmPerMpsTrue) +
                                          " (rel_err=" + std::to_string(rel_err) + ")");
                    ok &= check_true(parsed.stall_min_cmd > 500.0 && parsed.stall_min_cmd <= 1000.0,
                                      "stall.min_cmd=" + std::to_string(parsed.stall_min_cmd) +
                                          " in (500,1000]");
                } catch (const std::exception& ex) {
                    ok &= check_true(false,
                                      std::string("mpc::parse_calibration() threw: ") + ex.what());
                }
            }
        }

        // --- campaign_summary.json: 500-erpm trial classified non_moving --
        // (planned_value==500 is reliable regardless of the insufficient-room
        // retry mechanism: index 0's alternation sign is always '+', and the
        // stalled 500 command produces ~0 displacement, so it never itself
        // needs a direction flip.)
        {
            if (summary1_loaded && summary1.contains("trials") && summary1.at("trials").is_array()) {
                bool found = false;
                std::string classification;
                for (const auto& t : summary1.at("trials")) {
                    if (t.value("planned_value", 0.0) == 500.0) {
                        found = true;
                        classification = t.value("classification", std::string());
                    }
                }
                ok &= check_true(found, "campaign_summary.json has a trial with planned_value==500");
                ok &= check_true(classification == "non_moving",
                                  "the 500-erpm trial (below stall_erpm=800) classified non_moving (got '" +
                                      classification + "')");
                ok &= check_true(summary1.value("driver_restored_to_ackermann", false),
                                  "campaign_summary.json reports driver_restored_to_ackermann==true");
            } else {
                ok &= check_true(false, "campaign_summary.json has a 'trials' array");
            }
        }

        // --- driver actually restored to source=ackermann -----------------
        {
            zmq::context_t ctx(1);
            zmq::socket_t req(ctx, zmq::socket_type::req);
            req.set(zmq::sockopt::linger, 0);
            req.set(zmq::sockopt::rcvtimeo, 2000);
            req.connect("tcp://127.0.0.1:" + std::to_string(kControlPort));
            std::string reply_text;
            ok &= check_true(req_roundtrip(req, "{\"cmd\":\"ping\"}", &reply_text),
                              "direct ping to driver control port round-trips after run 1");
            if (!reply_text.empty()) {
                const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
                ok &= check_true(!reply.is_discarded() && reply.value("source", std::string()) == "ackermann",
                                  "driver reports source==\"ackermann\" after motor_calibration finished");
            }
        }

        // =================================================================
        // Run 2: safety path -- shrunk line -> the single trial must hit
        // the end-of-line envelope.
        // =================================================================
        const std::string out_dir2 = tmp_dir + "/out2";
        std::filesystem::create_directories(out_dir2);
        const std::string config2_path = tmp_dir + "/motor_calib_config2.json";
        {
            std::ofstream f(config2_path);
            f << "{"
                 "\"robot_ip\":\"127.0.0.1\","
                 "\"control_port\":" << kControlPort << ","
                 "\"telemetry_port\":" << kTelemetryPort << ","
                 "\"localization_endpoint\":\"tcp://127.0.0.1:" << kLocPort << "\","
                 "\"robot_topic_name\":\"" << kRobotTopic << "\","
                 "\"motive_body_name\":\"" << kRobotTopic << "\","
                 "\"line\":{\"length_m\":0.4,\"end_margin_m\":0.05,\"line_yaw_rad\":0.0},"
                 // Loosened for the same jitter-tolerance reason as run 1's config above -- this
                 // run specifically wants the ENVELOPE check to be what trips, not a spurious
                 // velocity-estimate spike.
                 "\"safety\":{\"mocap_stale_abort_ms\":300,\"max_speed_abort_mps\":2.0},"
                 "\"sweeps\":{\"duty\":[],\"erpm\":[2500]},"
                 "\"trial\":{\"max_duration_s\":4.0,\"settle_s\":1.5,\"arm_settle_s\":1.0,"
                 "\"steady_window_frac\":0.4},"
                 "\"output_dir\":\"" << out_dir2 << "\""
                 "}";
        }

        const std::size_t fake_log_size_before_run2 = read_file(fake_log_path).size();
        const std::string mc2_stdout_path = tmp_dir + "/motor_calibration_run2_stdout.txt";
        std::cout << "[test] running motor_calibration (safety-path: shrunk line)...\n";
        const RunResult r2 = run_wrapped_and_wait(
            motor_calibration_exe,
            {"--config", config2_path, "--modes", "erpm", "--max-trials", "1"}, mc2_stdout_path, 30.0);
        ok &= check_true(!r2.timed_out, "motor_calibration (run 2, safety path) did not time out");
        ok &= check_true(r2.exited && r2.exit_code == 0,
                          "motor_calibration (run 2) exited 0 (a single aborted trial is not a fatal "
                          "campaign error; got exit_code=" +
                              std::to_string(r2.exit_code) + ")");
        if (!r2.exited || r2.exit_code != 0) {
            std::cout << "----- motor_calibration run2 stdout -----\n"
                       << read_file(mc2_stdout_path) << "\n------------------------------------------\n";
        }

        {
            bool loaded = false;
            const nlohmann::json summary = load_json_file(out_dir2 + "/campaign_summary.json", &loaded);
            ok &= check_true(loaded, "campaign_summary.json (run 2) exists and parses as JSON");
            if (loaded && summary.contains("trials") && summary.at("trials").is_array() &&
                !summary.at("trials").empty()) {
                const nlohmann::json& t0 = summary.at("trials").front();
                ok &= check_true(t0.value("state", std::string()) == "aborted",
                                  "run 2's single trial is state==\"aborted\" (got '" +
                                      t0.value("state", std::string()) + "')");
                ok &= check_true(t0.value("abort_reason", std::string()) == "out_of_envelope",
                                  "run 2's abort_reason==\"out_of_envelope\" (got '" +
                                      t0.value("abort_reason", std::string()) + "')");
            } else {
                ok &= check_true(false, "campaign_summary.json (run 2) has a non-empty 'trials' array");
            }

            // Regression (round-0 finding, MPC/src/motor_calibration.cpp):
            // a single aborted trial gives fit_linear() < 2 usable points
            // (ok=false) and fit_grid() no directly-supported cell
            // (ok=false) -- with neither representation usable,
            // motor_calibration must NOT write calibration_erpm.json at
            // all, rather than exporting a schema-valid garbage file
            // (cmd_per_mps=0/cmd_offset=0/grid:null) that VescDriver's
            // CalibratedMap would happily load as a real (permanently
            // inert) calibration.
            ok &= check_true(!std::filesystem::exists(out_dir2 + "/calibration_erpm.json"),
                              "run 2 (single aborted trial, no usable fit) must NOT write calibration_erpm.json");
            if (loaded && summary.contains("fits") && summary.at("fits").contains("erpm")) {
                const nlohmann::json& fit2 = summary.at("fits").at("erpm");
                ok &= check_true(fit2.value("linear_ok", true) == false,
                                  "run 2's erpm fit summary reports linear_ok==false");
                ok &= check_true(fit2.value("grid_ok", true) == false,
                                  "run 2's erpm fit summary reports grid_ok==false");
                ok &= check_true(fit2.value("calibration_written", true) == false,
                                  "run 2's erpm fit summary reports calibration_written==false");
            } else {
                ok &= check_true(false, "campaign_summary.json (run 2) has a 'fits.erpm' entry");
            }
        }

        // --- driver stop appears in the fake_vesc log shortly after -------
        {
            bool saw_brake = false;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (std::chrono::steady_clock::now() < deadline) {
                const std::string log_now = read_file(fake_log_path);
                if (log_now.size() > fake_log_size_before_run2 &&
                    log_now.find("\"cmd\":\"set_current_brake\"", fake_log_size_before_run2) !=
                        std::string::npos) {
                    saw_brake = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            ok &= check_true(saw_brake,
                              "fake_vesc log shows a new set_current_brake entry within 3s of run 2's "
                              "envelope abort");
        }

        std::cout << "[test] driver stdout tail:\n" << read_file(driver_stdout_path) << "\n";
        std::cout << (ok ? "\nALL CHECKS PASSED\n" : "\nSOME CHECKS FAILED\n");
        return ok ? 0 : 1;

    } catch (const std::exception& ex) {
        std::cerr << "[test_motor_calib_e2e] uncaught exception: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[test_motor_calib_e2e] uncaught non-std exception\n";
        return 1;
    }
}
