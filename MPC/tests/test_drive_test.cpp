// test_drive_test.cpp -> binary `test_drive_test`
//
// Standalone, hardware-free, end-to-end loop for the drive_test tool.
// Mirrors MPC/tests/test_motor_calib_e2e.cpp's harness (ProcessGuard,
// resolve_sibling_exe, spawn_wrapped_capture_stdout, run_wrapped_and_wait,
// wait_for_pty_line, check_true, req_roundtrip, a background OptiTrack-sim
// thread), reimplemented locally per this codebase's established
// per-file-self-contained convention. Spawns real fake_vesc + vesc_driver
// child processes, plays OptiTrack itself (integrating the driver's own
// telemetry-reported erpm at a known ground-truth erpm_per_mps), then runs
// the real drive_test binary against them wrapped, asserting on its exit
// code, stdout, and (mocap-mode run) a direct control-port ping confirming
// the driver was restored to source=="ackermann".
//
// TEST PORTS ONLY (45950-45959 block):
//   45950  vesc_driver --ackermann-port (unused by drive_test -- only bound
//          so vesc_driver can start)
//   45951  vesc_driver --control-port   (drive_test's REQ target)
//   45952  vesc_driver --telemetry-port (drive_test's + this test's own
//          OptiTrack-sim thread's telemetry SUB source)
//   45953  this test's own OptiTrack-sim PUB bind (drive_test's
//          --loc-endpoint SUB target)
//
// Ground truth: v = erpm/4600 (ERPM_PER_MPS_TRUE), matching fake_vesc's
// --stall-erpm 800.

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
constexpr int kAckermannPort = 45950;
constexpr int kControlPort = 45951;
constexpr int kTelemetryPort = 45952;
constexpr int kLocPort = 45953;
// Separate PUB bind for the reversed-polarity scenario (c) below -- a second,
// independent OptitrackSimThread instance reports PHYSICAL motion opposite to
// the (correctly-signed) telemetry erpm, simulating a real motor-polarity
// wiring reversal that only ground-truth mocap feedback -- not telemetry,
// which just echoes the driver's own erpm sign -- can catch.
constexpr int kLocPortReversed = 45954;
const std::string kRobotTopic = "drivetest";
constexpr double kErpmPerMpsTrue = 4600.0;

// ---------------------------------------------------------------------
// RAII child-process guard -- reimplemented locally (see
// test_motor_calib_e2e.cpp's identical, independently-duplicated
// implementation per this codebase's own per-file-self-contained
// convention).
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
        std::cerr << "[test_drive_test] pid " << pid_ << " (" << label_
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
        std::fprintf(stderr, "[test_drive_test] execv('%s') failed\n", env_exe.c_str());
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
// This test process's own OptiTrack stand-in -- see test_motor_calib_e2e.cpp's
// OptitrackSimThread for the identical scheme, adapted to drive_test's
// simpler needs (no line_yaw config -- drive_test captures its own heading).
// ---------------------------------------------------------------------
class OptitrackSimThread {
public:
    // `loc_port`: PUB bind port for this instance's localization feed.
    // `reverse_polarity`: when true, physical x is integrated with the SIGN
    // FLIPPED relative to telemetry-reported erpm -- i.e. the mocap ground
    // truth disagrees with what the driver thinks it commanded, exactly
    // modeling a reversed motor-polarity wiring fault (see scenario (c)).
    explicit OptitrackSimThread(int loc_port = kLocPort, bool reverse_polarity = false)
        : stop_(false), last_x_(0.5), loc_port_(loc_port), reverse_polarity_(reverse_polarity) {
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
        pub.bind("tcp://*:" + std::to_string(loc_port_));

        zmq::socket_t telem_sub(ctx, zmq::socket_type::sub);
        telem_sub.set(zmq::sockopt::linger, 0);
        telem_sub.connect("tcp://127.0.0.1:" + std::to_string(kTelemetryPort));
        telem_sub.set(zmq::sockopt::subscribe, "/" + kRobotTopic + "/vesc_telemetry");

        const std::string loc_topic = "/" + kRobotTopic + "/localization";
        double x = 0.5;  // arbitrary start -- drive_test captures its own line frame off the first pose.
        double erpm = 0.0;
        auto last_tick = std::chrono::steady_clock::now();
        const double polarity_sign = reverse_polarity_ ? -1.0 : 1.0;

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
            x += polarity_sign * (erpm / kErpmPerMpsTrue) * dt;
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
    int loc_port_;
    bool reverse_polarity_;
};

bool check_true(bool cond, const std::string& what) {
    std::cout << (cond ? "  [PASS] " : "  [FAIL] ") << what << "\n";
    return cond;
}

// Parses "final_offset=<num>" out of drive_test's summary print line.
bool parse_final_offset(const std::string& stdout_text, double* out) {
    const std::string key = "final_offset=";
    const size_t pos = stdout_text.rfind(key);
    if (pos == std::string::npos) return false;
    size_t start = pos + key.size();
    size_t end = start;
    while (end < stdout_text.size() && (std::isdigit(static_cast<unsigned char>(stdout_text[end])) ||
                                          stdout_text[end] == '-' || stdout_text[end] == '.' ||
                                          stdout_text[end] == 'e' || stdout_text[end] == '+')) {
        ++end;
    }
    if (end == start) return false;
    try {
        *out = std::stod(stdout_text.substr(start, end - start));
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

}  // namespace

int main() {
    bool ok = true;

    try {
        const std::string fake_vesc_exe = resolve_sibling_exe("fake_vesc");
        const std::string vesc_driver_exe = resolve_sibling_exe("vesc_driver");
        const std::string drive_test_exe = resolve_sibling_exe("drive_test");
        if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
            return check_true(false, "fake_vesc executable found (tried '" + fake_vesc_exe +
                                          "') -- build VescDriver first") ? 1 : 1;
        }
        if (access(vesc_driver_exe.c_str(), X_OK) != 0) {
            return check_true(false, "vesc_driver executable found (tried '" + vesc_driver_exe +
                                          "') -- build VescDriver first") ? 1 : 1;
        }
        if (access(drive_test_exe.c_str(), X_OK) != 0) {
            return check_true(false, "drive_test executable found (tried '" + drive_test_exe +
                                          "') -- build MPC first") ? 1 : 1;
        }
        std::cout << "[test] fake_vesc=" << fake_vesc_exe << "\n";
        std::cout << "[test] vesc_driver=" << vesc_driver_exe << "\n";
        std::cout << "[test] drive_test=" << drive_test_exe << "\n";

        const std::string tmp_dir = "/tmp/test_drive_test_" + std::to_string(static_cast<long>(getpid()));
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
        // (a) mocap-mode run.
        // =================================================================
        const std::string mocap_stdout_path = tmp_dir + "/drive_test_mocap_stdout.txt";
        std::cout << "[test] running drive_test (mocap mode)...\n";
        const RunResult r_mocap = run_wrapped_and_wait(
            drive_test_exe,
            {"--robot-ip", "127.0.0.1", "--control-port", std::to_string(kControlPort), "--telemetry-port",
             std::to_string(kTelemetryPort), "--loc-endpoint", "tcp://127.0.0.1:" + std::to_string(kLocPort),
             "--topic", kRobotTopic, "--distance", "1.0", "--value", "1500", "--mode", "erpm"},
            mocap_stdout_path, 60.0);
        const std::string mocap_stdout = read_file(mocap_stdout_path);
        ok &= check_true(!r_mocap.timed_out, "drive_test (mocap mode) did not time out");
        ok &= check_true(r_mocap.exited && r_mocap.exit_code == 0,
                          "drive_test (mocap mode) exited 0 (got exit_code=" +
                              std::to_string(r_mocap.exit_code) + ")");
        if (!r_mocap.exited || r_mocap.exit_code != 0) {
            std::cout << "----- drive_test (mocap) stdout -----\n"
                       << mocap_stdout << "\n--------------------------------------\n";
        }
        ok &= check_true(mocap_stdout.find("leg 1 complete") != std::string::npos,
                          "drive_test (mocap) stdout reports leg 1 complete");
        ok &= check_true(mocap_stdout.find("leg 2 complete") != std::string::npos,
                          "drive_test (mocap) stdout reports leg 2 complete");
        {
            double final_offset = 0.0;
            const bool parsed = parse_final_offset(mocap_stdout, &final_offset);
            ok &= check_true(parsed, "drive_test (mocap) stdout has a parseable final_offset");
            if (parsed) {
                ok &= check_true(std::fabs(final_offset) <= 0.15,
                                  "final |offset| <= 0.15m (got " + std::to_string(final_offset) + ")");
            }
        }

        // --- driver actually restored to source=ackermann after mocap run --
        {
            zmq::context_t ctx(1);
            zmq::socket_t req(ctx, zmq::socket_type::req);
            req.set(zmq::sockopt::linger, 0);
            req.set(zmq::sockopt::rcvtimeo, 2000);
            req.connect("tcp://127.0.0.1:" + std::to_string(kControlPort));
            std::string reply_text;
            ok &= check_true(req_roundtrip(req, "{\"cmd\":\"ping\"}", &reply_text),
                              "direct ping to driver control port round-trips after mocap run");
            if (!reply_text.empty()) {
                const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
                ok &= check_true(!reply.is_discarded() && reply.value("source", std::string()) == "ackermann",
                                  "driver reports source==\"ackermann\" after drive_test (mocap) finished");
            }
        }

        // =================================================================
        // (b) telemetry-mode run.
        // =================================================================
        const std::string telem_stdout_path = tmp_dir + "/drive_test_telemetry_stdout.txt";
        std::cout << "[test] running drive_test (--use-telemetry mode)...\n";
        const RunResult r_telem = run_wrapped_and_wait(
            drive_test_exe,
            {"--robot-ip", "127.0.0.1", "--control-port", std::to_string(kControlPort), "--telemetry-port",
             std::to_string(kTelemetryPort), "--loc-endpoint", "tcp://127.0.0.1:" + std::to_string(kLocPort),
             "--topic", kRobotTopic, "--distance", "1.0", "--value", "1500", "--mode", "erpm",
             "--use-telemetry", "--erpm-per-mps", "4600"},
            telem_stdout_path, 60.0);
        const std::string telem_stdout = read_file(telem_stdout_path);
        ok &= check_true(!r_telem.timed_out, "drive_test (--use-telemetry) did not time out");
        ok &= check_true(r_telem.exited && r_telem.exit_code == 0,
                          "drive_test (--use-telemetry) exited 0 (got exit_code=" +
                              std::to_string(r_telem.exit_code) + ")");
        if (!r_telem.exited || r_telem.exit_code != 0) {
            std::cout << "----- drive_test (--use-telemetry) stdout -----\n"
                       << telem_stdout << "\n------------------------------------------------\n";
        }
        ok &= check_true(telem_stdout.find("leg 1 complete") != std::string::npos,
                          "drive_test (--use-telemetry) stdout reports leg 1 complete");
        ok &= check_true(telem_stdout.find("leg 2 complete") != std::string::npos,
                          "drive_test (--use-telemetry) stdout reports leg 2 complete");

        // =================================================================
        // (c) reversed-polarity mocap-mode run -- exercises run_leg()'s
        // wrong_direction abort. A second, independent OptitrackSimThread
        // reports PHYSICAL x moving OPPOSITE to the (correctly-signed)
        // telemetry erpm, modeling a real motor-polarity wiring fault: the
        // driver/telemetry chain thinks it's driving +value forward, but the
        // mocap ground truth shows s running away in the negative direction.
        // Only mocap mode can catch this (telemetry-mode integrates the
        // erpm sign itself, so it can never disagree with it) -- leg 1
        // should abort with "wrong_direction" well before leg_timeout or
        // over_speed would otherwise trip, and drive_test should still exit
        // via the DriverRestoreGuard-protected path (nonzero exit, but
        // source restored to "ackermann").
        // =================================================================
        {
            OptitrackSimThread reversed_optitrack_sim(kLocPortReversed, /*reverse_polarity=*/true);
            std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let PUB/SUB subscriptions settle.

            const std::size_t fake_log_size_before_reversed = read_file(fake_log_path).size();

            const std::string reversed_stdout_path = tmp_dir + "/drive_test_reversed_polarity_stdout.txt";
            std::cout << "[test] running drive_test (reversed-polarity mocap mode)...\n";
            const RunResult r_reversed = run_wrapped_and_wait(
                drive_test_exe,
                {"--robot-ip", "127.0.0.1", "--control-port", std::to_string(kControlPort), "--telemetry-port",
                 std::to_string(kTelemetryPort), "--loc-endpoint",
                 "tcp://127.0.0.1:" + std::to_string(kLocPortReversed), "--topic", kRobotTopic, "--distance",
                 "1.0", "--value", "1500", "--mode", "erpm"},
                reversed_stdout_path, 60.0);
            const std::string reversed_stdout = read_file(reversed_stdout_path);
            ok &= check_true(!r_reversed.timed_out, "drive_test (reversed-polarity) did not time out");
            ok &= check_true(r_reversed.exited && r_reversed.exit_code != 0,
                              "drive_test (reversed-polarity) exited nonzero (got exit_code=" +
                                  std::to_string(r_reversed.exit_code) + ")");
            ok &= check_true(reversed_stdout.find("leg 1 ABORTED: wrong_direction") != std::string::npos,
                              "drive_test (reversed-polarity) stdout reports leg 1 ABORTED: wrong_direction");
            if (!r_reversed.exited || r_reversed.exit_code == 0 ||
                reversed_stdout.find("wrong_direction") == std::string::npos) {
                std::cout << "----- drive_test (reversed-polarity) stdout -----\n"
                           << reversed_stdout << "\n--------------------------------------------------\n";
            }

            // --- driver stop (set_current_brake) appears in the fake_vesc log shortly after --
            {
                bool saw_brake = false;
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
                while (std::chrono::steady_clock::now() < deadline) {
                    const std::string log_now = read_file(fake_log_path);
                    if (log_now.size() > fake_log_size_before_reversed &&
                        log_now.find("\"cmd\":\"set_current_brake\"", fake_log_size_before_reversed) !=
                            std::string::npos) {
                        saw_brake = true;
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                ok &= check_true(saw_brake,
                                  "fake_vesc log shows a new set_current_brake entry within 3s of the "
                                  "reversed-polarity wrong_direction abort");
            }

            // --- driver still restored to source=ackermann after the aborted run --
            zmq::context_t ctx(1);
            zmq::socket_t req(ctx, zmq::socket_type::req);
            req.set(zmq::sockopt::linger, 0);
            req.set(zmq::sockopt::rcvtimeo, 2000);
            req.connect("tcp://127.0.0.1:" + std::to_string(kControlPort));
            std::string reply_text;
            ok &= check_true(req_roundtrip(req, "{\"cmd\":\"ping\"}", &reply_text),
                              "direct ping to driver control port round-trips after reversed-polarity run");
            if (!reply_text.empty()) {
                const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
                ok &= check_true(!reply.is_discarded() && reply.value("source", std::string()) == "ackermann",
                                  "driver reports source==\"ackermann\" after drive_test (reversed-polarity) "
                                  "aborted");
            }
        }

        std::cout << "[test] driver stdout tail:\n" << read_file(driver_stdout_path) << "\n";
        std::cout << (ok ? "\nALL CHECKS PASSED\n" : "\nSOME CHECKS FAILED\n");
        return ok ? 0 : 1;

    } catch (const std::exception& ex) {
        std::cerr << "[test_drive_test] uncaught exception: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "[test_drive_test] uncaught non-std exception\n";
        return 1;
    }
}
