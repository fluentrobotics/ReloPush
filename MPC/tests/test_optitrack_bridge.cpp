// Process-level integration test for optitrack_zmq_bridge, modeled on
// test_mpc_deadband.cpp's spawn/ProcessGuard/self_dir structure. Spawns REAL
// fake_motive + optitrack_zmq_bridge child processes (by path, resolved at
// runtime via /proc/self/exe) over UDP loopback, then SUBs to both robots'
// real localization PUB sockets to exercise the actual wire protocol
// end-to-end -- not a mock.
//
// This test does not link OptiTrackCore/SimCore/base64 at all: it only
// drives already-built binaries via subprocess + ZMQ + JSON + file I/O,
// mirroring how test_mpc_sim_loop.cpp drives mpc_controller/mpc_robot_sim
// without needing their internals linked in.
//
// TEST PORTS ONLY: UDP loopback 45510 (fake_motive's served command port)
// and 45511 (bridge's --local-data-port / fake_motive's --target-port); ZMQ
// 45861 (robot1 localization PUB) and 45862 (robot2 localization PUB).

#include <zmq.hpp>

#include <nlohmann/json.hpp>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

// ---------------------------------------------------------------------
// Test-only ports/identity.
// ---------------------------------------------------------------------
constexpr int kFakeMotiveCommandPort = 45510;  // fake_motive --serve-command-port
constexpr int kBridgeLocalDataPort = 45511;    // bridge --local-data-port / fake_motive --target-port
constexpr int kLocPortStart = 45861;           // bridge --loc-port-start (robot1=45861, robot2=45862)
const std::string kRobot1 = "robot1";
const std::string kRobot2 = "robot2";

// ---------------------------------------------------------------------
// Locate sibling executables. CMake places every target from
// MPC/CMakeLists.txt in the same output directory as this test
// binary (see test_mpc_deadband.cpp's identical, independently-duplicated
// helper -- kept self-contained per file per this codebase's own precedent).
// ---------------------------------------------------------------------
// Round 8 (macOS dev box support -- see this project's cross-OS-build notes:
// /proc/self/exe is Linux-only, so this test could not even START on
// macOS): __APPLE__ resolves the running binary's own path via
// _NSGetExecutablePath() instead (the macOS-native equivalent); every other
// platform keeps the original /proc/self/exe readlink() unchanged.
std::string self_dir() {
    char buf[4096];
    std::string p;
#ifdef __APPLE__
    std::uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) != 0) {
        throw std::runtime_error("_NSGetExecutablePath() failed (path longer than buffer)");
    }
    p.assign(buf);
#else
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) {
        throw std::runtime_error("readlink(/proc/self/exe) failed");
    }
    buf[n] = '\0';
    p.assign(buf);
#endif
    auto pos = p.find_last_of('/');
    return pos == std::string::npos ? std::string(".") : p.substr(0, pos);
}

// ---------------------------------------------------------------------
// RAII child-process guard (see test_mpc_deadband.cpp/test_mpc_sim_loop.cpp
// for the identical, independently-duplicated implementation).
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

    bool wait_for_exit(double timeout_s, double poll_interval_s = 0.1,
                        const std::function<void()>& on_poll = nullptr) {
        if (pid_ <= 0) return true;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            int status = 0;
            pid_t r = waitpid(pid_, &status, WNOHANG);
            if (r == pid_) {
                pid_ = -1;
                return true;
            }
            if (on_poll) on_poll();
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

// Spawns exe with args; if stdout_path is non-empty, the child's stdout AND
// stderr are both redirected there (so the test can grep the bridge's own
// log lines, e.g. the version-discovery message) rather than interleaving
// with this test's own output.
pid_t spawn(const std::string& exe, const std::vector<std::string>& args,
            const std::string& stdout_path = "") {
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(exe.c_str()));
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);

    pid_t pid = fork();
    if (pid < 0) {
        throw std::runtime_error("fork() failed: " + std::string(std::strerror(errno)));
    }
    if (pid == 0) {
        if (!stdout_path.empty()) {
            int fd = open(stdout_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fd >= 0) {
                dup2(fd, STDOUT_FILENO);
                dup2(fd, STDERR_FILENO);
                close(fd);
            }
        }
        execv(exe.c_str(), argv.data());
        std::fprintf(stderr, "[test] execv('%s') failed: %s\n", exe.c_str(), std::strerror(errno));
        _exit(127);
    }
    return pid;
}

// ---------------------------------------------------------------------
// ZMQ message capture.
// ---------------------------------------------------------------------
struct RecvMsg {
    std::string topic;
    std::string payload;
    std::chrono::steady_clock::time_point t_recv;
};

void drain_messages(zmq::socket_t& sub, std::vector<RecvMsg>& out) {
    while (true) {
        zmq::message_t topic_msg;
        auto r1 = sub.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!r1.has_value()) break;
        if (!topic_msg.more()) break;  // malformed (no payload frame); ignore.
        zmq::message_t payload_msg;
        auto r2 = sub.recv(payload_msg, zmq::recv_flags::none);
        if (!r2.has_value()) break;
        RecvMsg m;
        m.topic = std::string(static_cast<char*>(topic_msg.data()), topic_msg.size());
        m.payload = std::string(static_cast<char*>(payload_msg.data()), payload_msg.size());
        m.t_recv = std::chrono::steady_clock::now();
        out.push_back(std::move(m));
    }
}

double angle_diff(double a, double b) {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d <= -M_PI) d += 2.0 * M_PI;
    return d;
}

std::vector<RecvMsg> filter_topic(const std::vector<RecvMsg>& all, const std::string& topic) {
    std::vector<RecvMsg> out;
    for (const RecvMsg& m : all) {
        if (m.topic == topic) out.push_back(m);
    }
    return out;
}

// ---------------------------------------------------------------------
// Test report accumulator (mirrors test_mpc_deadband.cpp's PartResult).
// ---------------------------------------------------------------------
struct Report {
    bool pass = true;
    std::vector<std::string> notes;
    void fail(const std::string& note) {
        pass = false;
        notes.push_back("[FAIL] " + note);
    }
    void info(const std::string& note) { notes.push_back("[info] " + note); }
};

}  // namespace

// ---------------------------------------------------------------------
// Part 1 (round 1 scenario, unchanged): unicast mode, 2 robots, one of them
// periodically tracking-dropped. Exercises the core receive/parse/publish
// path, ping-based version discovery, CSV logging.
// ---------------------------------------------------------------------
Report run_part_unicast(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path csv_path = fs::temp_directory_path() / "test_optitrack_bridge_log.csv";
    const fs::path bridge_log_path = fs::temp_directory_path() / "test_optitrack_bridge_stdout.log";
    const fs::path map_config_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part1_map_config.json";
    std::error_code ec;
    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    // BUG FIX: Check 6 below asserts an IDENTITY transform (z_up convention:
    // planar_x==raw_x, planar_y==raw_y, planar_yaw==heading) against this
    // Part's Z-up synthetic fake_motive data. That used to hold with no
    // --map-config passed at all, because the bridge's old default (no
    // config file found) WAS the identity. Now that optitrack_zmq_bridge's
    // default-config-path fallback (round 7) correctly resolves and loads
    // the REAL project file at MPC/config/mocap_map_config.json
    // when --map-config is omitted, this Part would silently pick up
    // whatever calibration is live there (y_up:true + a per-robot
    // yaw_offset as of this writing) instead of identity, breaking Check 6
    // for reasons that have nothing to do with what this Part actually
    // tests (the core receive/parse/publish path). Pass an explicit
    // synthetic identity --map-config instead -- same pattern Parts 4/5
    // already use for their own non-default calibrations -- so this test
    // never depends on the real project config file's current content.
    {
        std::ofstream cfg(map_config_path);
        cfg << "{\"y_up\": false, \"x0\": 0.0, \"y0\": 0.0, \"theta0\": 0.0, \"yaw_offset\": {}}";
    }

    // ---- Spawn fake_motive first (runs until we kill it -- --duration-s 0). ----
    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "127.0.0.1",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--rate",                "120",
        "--natnet-version",      "3.1",
        "--body-ids",            "1,2",
        "--drop-tracking-every", "3",  // robot1 (id=1) drops tracking for 1s every 3s.
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotive",
        "--served-version",      "3.1",
        "--duration-s",          "0",
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive");
    std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // ---- Spawn the bridge (self-terminates after --duration-s). ----
    const double kBridgeDurationS = 8.5;
    std::vector<std::string> bridge_args = {
        "--server-ip",        "127.0.0.1",
        "--mode",             "unicast",
        "--command-port",     std::to_string(kFakeMotiveCommandPort),
        "--local-data-port",  std::to_string(kBridgeLocalDataPort),
        "--natnet-version",   "3.1",
        "--robots",           "robot1,robot2",
        "--rigid-body-ids",   "1,2",
        "--map-config",       map_config_path.string(),
        "--loc-port-start",   std::to_string(kLocPortStart),
        "--publish-rate",     "30",
        "--log-csv",          csv_path.string(),
        "--duration-s",       std::to_string(kBridgeDurationS),
    };
    const auto t_bridge_spawn = std::chrono::steady_clock::now();
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()), "optitrack_zmq_bridge");
    std::cout << "[test] spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart + 1));
        sub.set(zmq::sockopt::subscribe, "/" + kRobot1 + "/localization");
        sub.set(zmq::sockopt::subscribe, "/" + kRobot2 + "/localization");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        // Drain throughout the bridge's run, then a bit longer after it
        // self-exits to catch any final in-flight messages.
        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving the test: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();
    std::cout << "[test] captured " << all_msgs.size() << " total ZMQ messages" << std::endl;

    // =======================================================================
    // Analysis.
    // =======================================================================
    std::vector<RecvMsg> robot1_msgs = filter_topic(all_msgs, "/" + kRobot1 + "/localization");
    std::vector<RecvMsg> robot2_msgs = filter_topic(all_msgs, "/" + kRobot2 + "/localization");
    std::cout << "[test] robot1 messages=" << robot1_msgs.size()
               << " robot2 messages=" << robot2_msgs.size() << std::endl;

    if (robot2_msgs.size() < 30) {
        rep.fail("robot2 (never tracking-dropped) received suspiciously few messages: " +
                 std::to_string(robot2_msgs.size()));
    }
    if (robot1_msgs.empty()) {
        rep.fail("robot1 received zero messages");
    }

    // ---- Check 1: payload is EXACTLY {"x","y","yaw"}, all numeric. ----
    {
        int checked = 0, bad = 0;
        for (const RecvMsg& m : all_msgs) {
            try {
                nlohmann::json j = nlohmann::json::parse(m.payload);
                if (!j.is_object() || j.size() != 3 || !j.contains("x") || !j.contains("y") ||
                    !j.contains("yaw") || !j.at("x").is_number() || !j.at("y").is_number() ||
                    !j.at("yaw").is_number()) {
                    ++bad;
                }
            } catch (const std::exception&) {
                ++bad;
            }
            ++checked;
        }
        rep.info("payload format checked on " + std::to_string(checked) + " messages, " +
                  std::to_string(bad) + " malformed");
        if (checked == 0) {
            rep.fail("no messages to check payload format on");
        } else if (bad > 0) {
            rep.fail(std::to_string(bad) + " message(s) did not have the exact {x,y,yaw} payload shape");
        }
    }

    // ---- Check 2: robot2 (uninterrupted) publish rate is ~30Hz (downsampled
    // from fake_motive's 120Hz), measured over the steady-state window. ----
    if (robot2_msgs.size() >= 30) {
        const double span_s =
            std::chrono::duration<double>(robot2_msgs.back().t_recv - robot2_msgs.front().t_recv)
                .count();
        const double measured_hz = span_s > 0.0 ? (robot2_msgs.size() - 1) / span_s : 0.0;
        rep.info("robot2 measured publish rate = " + std::to_string(measured_hz) +
                  "Hz over " + std::to_string(span_s) + "s (target 30Hz, downsampled from 120Hz)");
        if (measured_hz < 15.0 || measured_hz > 45.0) {
            rep.fail("robot2 publish rate " + std::to_string(measured_hz) +
                      "Hz is far from the expected ~30Hz");
        }
    }

    // ---- Check 3: position continuity + yaw consistency with motion
    // direction, using robot2's uninterrupted stream. ----
    {
        int compared = 0;
        double abs_err_sum = 0.0;
        double max_step = 0.0;
        for (size_t i = 0; i + 1 < robot2_msgs.size(); ++i) {
            nlohmann::json j0, j1;
            try {
                j0 = nlohmann::json::parse(robot2_msgs[i].payload);
                j1 = nlohmann::json::parse(robot2_msgs[i + 1].payload);
            } catch (const std::exception&) {
                continue;
            }
            const double x0 = j0.at("x").get<double>(), y0 = j0.at("y").get<double>();
            const double x1 = j1.at("x").get<double>(), y1 = j1.at("y").get<double>();
            const double yaw0 = j0.at("yaw").get<double>();
            const double dx = x1 - x0, dy = y1 - y0;
            const double step = std::hypot(dx, dy);
            max_step = std::max(max_step, step);
            if (step < 0.001) continue;  // too small to get a meaningful direction from.
            const double secant_dir = std::atan2(dy, dx);
            const double err = std::fabs(angle_diff(secant_dir, yaw0));
            abs_err_sum += err;
            ++compared;
        }
        if (compared < 5) {
            rep.fail("fewer than 5 usable consecutive-sample pairs for yaw-consistency check (got " +
                      std::to_string(compared) + ")");
        } else {
            const double mean_err = abs_err_sum / compared;
            rep.info("yaw-vs-motion-direction: " + std::to_string(compared) +
                      " pairs compared, mean |angle error|=" + std::to_string(mean_err) +
                      " rad, max per-step displacement=" + std::to_string(max_step) + "m");
            if (mean_err > 0.2) {
                rep.fail("yaw does not track motion direction (mean angle error " +
                          std::to_string(mean_err) + " rad > 0.2 rad bound)");
            }
        }

        // Overall displacement across the whole window rules out a "frozen"
        // publisher (radius=1m, ~20s period -- over ~7s of steady tracking
        // this should trace a good fraction of the circle).
        if (robot2_msgs.size() >= 2) {
            try {
                nlohmann::json jf = nlohmann::json::parse(robot2_msgs.front().payload);
                nlohmann::json jl = nlohmann::json::parse(robot2_msgs.back().payload);
                const double total_disp =
                    std::hypot(jl.at("x").get<double>() - jf.at("x").get<double>(),
                               jl.at("y").get<double>() - jf.at("y").get<double>());
                rep.info("robot2 total displacement over observation window = " +
                          std::to_string(total_disp) + "m");
                if (total_disp < 0.2) {
                    rep.fail("robot2 position barely moved (" + std::to_string(total_disp) +
                              "m) -- looks frozen, not tracking a moving circle");
                }
            } catch (const std::exception&) {
                rep.fail("could not parse robot2's first/last payload for displacement check");
            }
        }
    }

    // ---- Check 4: robot1's tracking-drop PAUSES publishing (and resumes),
    // while robot2 (never dropped) keeps flowing throughout -- via 0.5s
    // buckets of "did this robot publish at least once in this bucket",
    // relative to when the bridge was spawned. Robust to wall-clock
    // correlation slop between the test and fake_motive/bridge's own
    // internal clocks. ----
    {
        constexpr double kBucketS = 0.5;
        constexpr double kWarmupS = 1.0;  // skip startup transient.

        auto bucket_of = [&](std::chrono::steady_clock::time_point t) {
            return static_cast<int>(
                std::floor(std::chrono::duration<double>(t - t_bridge_spawn).count() / kBucketS));
        };

        std::set<int> active1, active2;
        for (const RecvMsg& m : robot1_msgs) active1.insert(bucket_of(m.t_recv));
        for (const RecvMsg& m : robot2_msgs) active2.insert(bucket_of(m.t_recv));

        const int warmup_bucket = static_cast<int>(std::ceil(kWarmupS / kBucketS));
        int max_bucket = -1;
        for (int b : active2) max_bucket = std::max(max_bucket, b);
        for (int b : active1) max_bucket = std::max(max_bucket, b);

        std::vector<int> robot1_silent_while_robot2_active;
        std::vector<int> robot2_silent_while_robot1_active;
        for (int b = warmup_bucket; b < max_bucket; ++b) {
            const bool r1_active = active1.count(b) > 0;
            const bool r2_active = active2.count(b) > 0;
            if (r2_active && !r1_active) robot1_silent_while_robot2_active.push_back(b);
            if (r1_active && !r2_active) robot2_silent_while_robot1_active.push_back(b);
        }

        rep.info("bucket analysis: warmup_bucket=" + std::to_string(warmup_bucket) +
                  " max_bucket=" + std::to_string(max_bucket) +
                  " robot1-silent-buckets=" + std::to_string(robot1_silent_while_robot2_active.size()) +
                  " robot2-silent-buckets=" + std::to_string(robot2_silent_while_robot1_active.size()));

        if (robot1_silent_while_robot2_active.empty()) {
            rep.fail("robot1 never went silent while robot2 kept publishing -- expected tracking-drop "
                      "pauses were not observed");
        } else {
            const int last_silent = robot1_silent_while_robot2_active.back();
            const bool resumed_after = active1.count(last_silent + 1) > 0 || active1.count(last_silent + 2) > 0;
            if (!resumed_after) {
                rep.fail("robot1 did not resume publishing shortly after its last observed silent bucket "
                          "(bucket " + std::to_string(last_silent) + ")");
            }
            if (robot1_silent_while_robot2_active.size() < 2) {
                rep.info("only " + std::to_string(robot1_silent_while_robot2_active.size()) +
                          " robot1-silent-while-robot2-active bucket(s) observed -- weak but not failing "
                          "evidence of periodic drop/resume");
            }
        }
        if (!robot2_silent_while_robot1_active.empty()) {
            rep.fail("robot2 went silent " + std::to_string(robot2_silent_while_robot1_active.size()) +
                      " time(s) -- it should never be tracking-dropped in this scenario");
        }
    }

    // ---- Check 5: version-discovery log line appears in the bridge's log. ----
    {
        std::ifstream log_f(bridge_log_path);
        std::string log_contents((std::istreambuf_iterator<char>(log_f)),
                                   std::istreambuf_iterator<char>());
        const bool has_discovery = log_contents.find("Discovered server") != std::string::npos;
        const bool has_name = log_contents.find("TestMotive") != std::string::npos;
        const bool has_version = log_contents.find("3.1") != std::string::npos;
        rep.info(std::string("version-discovery log: found='Discovered server'=") +
                  (has_discovery ? "yes" : "no") + " name=" + (has_name ? "yes" : "no") +
                  " version=" + (has_version ? "yes" : "no"));
        if (!has_discovery || !has_name || !has_version) {
            rep.fail("bridge log does not contain the expected version-discovery line "
                      "('Discovered server' + served app name + version)");
        }
    }

    // ---- Check 6: CSV log written and well-formed, full 6-DoF columns
    // (round 4), with raw/planar heading matching under an identity
    // transform (this Part passes the explicit synthetic identity
    // --map-config written above -- see that block's BUG FIX comment for
    // why an explicit one is now required -- so map_cfg is identity:
    // planar_x==raw_x, planar_y==raw_y, planar_yaw==heading). ----
    {
        std::ifstream csv_f(csv_path);
        if (!csv_f.is_open()) {
            rep.fail("--log-csv file was not created at " + csv_path.string());
        } else {
            std::string header;
            std::getline(csv_f, header);
            const std::string expected_header =
                "t_arrival,robot,raw_x,raw_y,raw_z,qx,qy,qz,qw,roll,pitch,heading,planar_x,planar_y,"
                "planar_yaw,tracking_valid";
            if (header != expected_header) {
                rep.fail("CSV header mismatch: got '" + header + "', expected '" + expected_header + "'");
            }
            int rows = 0, malformed = 0, transform_mismatch = 0;
            std::string line;
            while (std::getline(csv_f, line)) {
                if (line.empty()) continue;
                std::stringstream ss(line);
                std::vector<std::string> fields;
                std::string tok;
                while (std::getline(ss, tok, ',')) fields.push_back(tok);
                if (fields.size() != 16) {
                    ++malformed;
                    continue;
                }
                try {
                    double t_arrival = std::stod(fields[0]);
                    (void)t_arrival;
                    const std::string& robot = fields[1];
                    if (robot != kRobot1 && robot != kRobot2) {
                        ++malformed;
                        continue;
                    }
                    if (fields[15] != "0" && fields[15] != "1") {
                        ++malformed;
                        continue;
                    }
                    const double raw_x = std::stod(fields[2]);
                    const double raw_y = std::stod(fields[3]);
                    const double heading = std::stod(fields[11]);
                    const double planar_x = std::stod(fields[12]);
                    const double planar_y = std::stod(fields[13]);
                    const double planar_yaw = std::stod(fields[14]);
                    // qx,qy,qz,qw,roll,pitch,raw_z all must at least parse as finite doubles.
                    for (std::size_t k : {4, 5, 6, 7, 8, 9, 10}) {
                        if (!std::isfinite(std::stod(fields[k]))) throw std::runtime_error("non-finite");
                    }
                    if (std::fabs(planar_x - raw_x) > 1e-9 || std::fabs(planar_y - raw_y) > 1e-9 ||
                        std::fabs(angle_diff(planar_yaw, heading)) > 1e-9) {
                        ++transform_mismatch;
                    }
                    ++rows;
                } catch (const std::exception&) {
                    ++malformed;
                }
            }
            rep.info("CSV: " + std::to_string(rows) + " well-formed rows, " +
                      std::to_string(malformed) + " malformed, " + std::to_string(transform_mismatch) +
                      " with raw/planar heading mismatch (identity transform expected here)");
            if (rows < 10) {
                rep.fail("CSV has suspiciously few well-formed rows: " + std::to_string(rows));
            }
            if (malformed > 0) {
                rep.fail("CSV has " + std::to_string(malformed) + " malformed row(s)");
            }
            if (transform_mismatch > 0) {
                rep.fail("CSV has " + std::to_string(transform_mismatch) +
                          " row(s) where planar_x/y/yaw differ from raw_x/y/heading under an identity "
                          "transform");
            }
        }
    }

    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    return rep;
}

// ---------------------------------------------------------------------
// Part 2 (round 2): multicast mode + --mocap-config driving the ENTIRE
// connection (server_ip/mode/command_port/data_port/multicast_group) --
// the bridge CLI deliberately does NOT pass any of those flags, so the only
// way this part can receive anything at all is if mocap-config parsing +
// precedence resolution worked end-to-end (an unreachable placeholder IP,
// wrong mode, or wrong ports would each independently make this a silent
// no-frames-ever-arrive failure). Also exercises multicast-mode version
// discovery (round 2 requirement 2) and confirms the PLACEHOLDER warning is
// suppressed when the IP comes from the config file.
//
// fake_motive gains --multicast-target for this part (see fake_motive.cpp).
// The bridge does NOT pass --multicast-interface at all (round 3): it now
// auto-resolves the join interface (mpc::choose_multicast_interface() via
// getifaddrs()) by matching --server-ip's (here, 127.0.0.1, from
// --mocap-config) subnet against this machine's real interfaces, which
// correctly picks "lo" -- exercising round 3's FIX 1 end-to-end, not just
// falling back to the earlier loopback-test-only manual override.
// ---------------------------------------------------------------------
Report run_part_multicast_and_mocap_config(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path csv_path = fs::temp_directory_path() / "test_optitrack_bridge_part2_log.csv";
    const fs::path bridge_log_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part2_stdout.log";
    const fs::path mocap_config_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part2_mocap_config.txt";
    std::error_code ec;
    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(mocap_config_path, ec);

    // Ports overridden to the test block (45510/45511); multicast group
    // left at Motive's real default (239.255.42.99) -- a plain address, not
    // a "production port" conflict, so testing the real default value here
    // is deliberate and meaningful. Field names/casing mirror the user's own
    // real exported file EXACTLY (see optitrack_unit_tests.cpp's fixture).
    {
        std::ofstream cfg(mocap_config_path);
        cfg << "ip_address:127.0.0.1\n"
               "Type:Multicast\n"
               "Command Port:" << kFakeMotiveCommandPort << "\n"
               "Data Port:" << kBridgeLocalDataPort << "\n"
               "Multicast Interface:239.255.42.99\n";
    }

    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "239.255.42.99",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--multicast-target",
        "--rate",                "120",
        "--natnet-version",      "3.1",
        "--body-ids",            "1,2",
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotiveMulticast",
        "--served-version",      "3.1",
        "--duration-s",          "0",
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive[part2]");
    std::cout << "[test] Part 2: spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Deliberately NO --server-ip/--mode/--command-port/--data-port/
    // --multicast-group -- ALL must come from --mocap-config for this part
    // to work at all (see this function's doc comment).
    const double kBridgeDurationS = 5.0;
    std::vector<std::string> bridge_args = {
        "--mocap-config",       mocap_config_path.string(),
        // No --multicast-interface -- relies on round 3's auto-detection
        // (see this function's doc comment).
        "--robots",              "robot1,robot2",
        "--rigid-body-ids",      "1,2",
        "--loc-port-start",      std::to_string(kLocPortStart),
        "--publish-rate",        "30",
        "--log-csv",             csv_path.string(),
        "--duration-s",          std::to_string(kBridgeDurationS),
    };
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()),
                               "optitrack_zmq_bridge[part2]");
    std::cout << "[test] Part 2: spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart + 1));
        sub.set(zmq::sockopt::subscribe, "/" + kRobot1 + "/localization");
        sub.set(zmq::sockopt::subscribe, "/" + kRobot2 + "/localization");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] Part 2: optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving Part 2: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();
    std::cout << "[test] Part 2: captured " << all_msgs.size() << " total ZMQ messages" << std::endl;

    std::vector<RecvMsg> robot1_msgs = filter_topic(all_msgs, "/" + kRobot1 + "/localization");
    std::vector<RecvMsg> robot2_msgs = filter_topic(all_msgs, "/" + kRobot2 + "/localization");
    std::cout << "[test] Part 2: robot1 messages=" << robot1_msgs.size()
               << " robot2 messages=" << robot2_msgs.size() << std::endl;

    // ---- Check 1: frames actually flowed for BOTH robots. As explained in
    // this function's doc comment, this alone is strong evidence the
    // mocap-config-driven server_ip/mode/ports were all resolved correctly
    // -- any one of them being wrong (e.g. still the CLI-default placeholder
    // IP, or still unicast mode, or still the real production ports) would
    // make this a hard zero, not a degraded number. ----
    if (robot1_msgs.size() < 20) {
        rep.fail("robot1 received suspiciously few messages via multicast+mocap-config: " +
                 std::to_string(robot1_msgs.size()));
    }
    if (robot2_msgs.size() < 20) {
        rep.fail("robot2 received suspiciously few messages via multicast+mocap-config: " +
                 std::to_string(robot2_msgs.size()));
    }

    // ---- Check 2: payload shape, same as Part 1. ----
    {
        int checked = 0, bad = 0;
        for (const RecvMsg& m : all_msgs) {
            try {
                nlohmann::json j = nlohmann::json::parse(m.payload);
                if (!j.is_object() || j.size() != 3 || !j.contains("x") || !j.contains("y") ||
                    !j.contains("yaw") || !j.at("x").is_number() || !j.at("y").is_number() ||
                    !j.at("yaw").is_number()) {
                    ++bad;
                }
            } catch (const std::exception&) {
                ++bad;
            }
            ++checked;
        }
        rep.info("payload format checked on " + std::to_string(checked) + " messages, " +
                  std::to_string(bad) + " malformed");
        if (checked == 0) {
            rep.fail("no messages to check payload format on");
        } else if (bad > 0) {
            rep.fail(std::to_string(bad) + " message(s) did not have the exact {x,y,yaw} payload shape");
        }
    }

    // ---- Read the bridge's log once for the remaining checks. ----
    std::string log_contents;
    {
        std::ifstream log_f(bridge_log_path);
        log_contents.assign(std::istreambuf_iterator<char>(log_f), std::istreambuf_iterator<char>());
    }

    // ---- Check 3: mocap-config actually drove the effective settings
    // (server_ip/mode/command_port/data_port/multicast_group all sourced
    // from "config", since none were passed on the CLI). ----
    {
        const bool ok = log_contents.find("server_ip=127.0.0.1 (config)") != std::string::npos &&
                         log_contents.find("mode=multicast (config)") != std::string::npos &&
                         log_contents.find("command_port=" + std::to_string(kFakeMotiveCommandPort) +
                                            " (config)") != std::string::npos &&
                         log_contents.find("data_port=" + std::to_string(kBridgeLocalDataPort) +
                                            " (config)") != std::string::npos &&
                         log_contents.find("multicast_group=239.255.42.99 (config)") != std::string::npos;
        rep.info(std::string("mocap-config drove effective settings: ") + (ok ? "yes" : "no"));
        if (!ok) {
            rep.fail(
                "bridge log's 'Effective settings' line does not show all 5 fields sourced from "
                "(config) as expected");
        }
    }

    // ---- Check 3b (round 3): auto-detected multicast interface -- no
    // --multicast-interface was passed, so this line must show "auto" and
    // must have correctly picked loopback (127.0.0.1 / "lo"), proving FIX 1
    // works end-to-end via the mocap-config-supplied server_ip. ----
    {
        const bool has_line = log_contents.find("multicast join via 127.0.0.1") != std::string::npos &&
                               log_contents.find("auto: subnet matches server") != std::string::npos;
        rep.info(std::string("auto-detected multicast interface (FIX 1): ") + (has_line ? "yes" : "no"));
        if (!has_line) {
            rep.fail("bridge log does not show auto-detected multicast interface via 127.0.0.1");
        }
    }

    // ---- Check 4: PLACEHOLDER warning suppressed (server IP came from
    // config, not the literal default). ----
    {
        const bool has_placeholder_warning =
            log_contents.find("still the PLACEHOLDER") != std::string::npos;
        rep.info(std::string("PLACEHOLDER warning suppressed: ") +
                  (has_placeholder_warning ? "NO (unexpectedly present)" : "yes"));
        if (has_placeholder_warning) {
            rep.fail("PLACEHOLDER warning fired even though --mocap-config supplied a real server IP");
        }
    }

    // ---- Check 5: version discovery works in MULTICAST mode too (round 2
    // requirement 2). ----
    {
        const bool has_discovery = log_contents.find("Discovered server") != std::string::npos;
        const bool has_name = log_contents.find("TestMotiveMulticast") != std::string::npos;
        rep.info(std::string("multicast version-discovery log: found='Discovered server'=") +
                  (has_discovery ? "yes" : "no") + " name=" + (has_name ? "yes" : "no"));
        if (!has_discovery || !has_name) {
            rep.fail("bridge log does not show version discovery working in multicast mode");
        }
    }

    // ---- Check 6: CSV log written with a reasonable number of rows. ----
    {
        std::ifstream csv_f(csv_path);
        int rows = 0;
        if (!csv_f.is_open()) {
            rep.fail("--log-csv file was not created at " + csv_path.string());
        } else {
            std::string header;
            std::getline(csv_f, header);
            std::string line;
            while (std::getline(csv_f, line)) {
                if (!line.empty()) ++rows;
            }
        }
        rep.info("Part 2 CSV rows=" + std::to_string(rows));
        if (rows < 10) {
            rep.fail("Part 2 CSV has suspiciously few rows: " + std::to_string(rows));
        }
    }

    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(mocap_config_path, ec);

    return rep;
}

// ---------------------------------------------------------------------
// Part 3 (round 3): version-confirmation gating. Reproduces the exact bug
// found on a live run against real Motive (NatNet 2.10) with the bridge's
// CLI --natnet-version fallback deliberately set WRONG (3.1): before
// gating, data frames were parsed immediately with the wrong fallback, and
// a wrong-version parse does not always fail cleanly -- frame 1 came out as
// garbage (id=-812564734, x=-6.4e+35) TWICE, reproduced live, because the
// bridge started parsing before the 1Hz ping had a chance to correct the
// version. Asserts: zero garbage/non-finite samples across the ENTIRE run
// (not just "eventually clean"), the "version confirmed 2.10 via ping" log
// line appears, frames_deferred > 0 (proof the gate actually deferred
// something rather than being a no-op), and every published payload parses
// to sane finite coordinates.
// ---------------------------------------------------------------------
Report run_part_version_gating(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path csv_path = fs::temp_directory_path() / "test_optitrack_bridge_part3_log.csv";
    const fs::path bridge_log_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part3_stdout.log";
    std::error_code ec;
    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);

    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "127.0.0.1",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--rate",                "120",
        "--natnet-version",      "2.10",  // the REAL version, per the live-run bug report.
        "--body-ids",            "1,2",
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotive210",
        "--served-version",      "2.10",
        "--duration-s",          "0",
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive[part3]");
    std::cout << "[test] Part 3: spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const double kBridgeDurationS = 5.0;
    std::vector<std::string> bridge_args = {
        "--server-ip",       "127.0.0.1",
        "--mode",             "unicast",
        "--command-port",     std::to_string(kFakeMotiveCommandPort),
        "--local-data-port",  std::to_string(kBridgeLocalDataPort),
        "--natnet-version",   "3.1",  // deliberately WRONG fallback -- see doc comment.
        "--robots",           "robot1,robot2",
        "--rigid-body-ids",   "1,2",
        "--loc-port-start",   std::to_string(kLocPortStart),
        "--publish-rate",     "30",
        "--log-csv",          csv_path.string(),
        "--duration-s",       std::to_string(kBridgeDurationS),
    };
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()),
                               "optitrack_zmq_bridge[part3]");
    std::cout << "[test] Part 3: spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart + 1));
        sub.set(zmq::sockopt::subscribe, "/" + kRobot1 + "/localization");
        sub.set(zmq::sockopt::subscribe, "/" + kRobot2 + "/localization");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] Part 3: optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving Part 3: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();
    std::cout << "[test] Part 3: captured " << all_msgs.size() << " total ZMQ messages" << std::endl;

    if (all_msgs.size() < 40) {
        rep.fail("suspiciously few messages captured: " + std::to_string(all_msgs.size()));
    }

    // ---- Check 1+4: EVERY published payload, across the ENTIRE run, is a
    // well-formed {x,y,yaw[,t]} object with finite, sane-magnitude values --
    // this is the direct regression check for the "frame 1 was garbage"
    // bug: id=-812564734/x=-6.4e+35-style values would trivially blow the
    // 100.0 bound (our fake circle never exceeds a few meters from origin),
    // and NaN/inf are rejected via std::isfinite. This Part streams REAL
    // NatNet 2.10 (see fake_motive_args above), the one version whose
    // trailer this bridge actually walks for a timestamp (round 8) -- once
    // confirmed, EVERY message here is expected to additionally carry "t"
    // (fake_motive emits one by default), so a well-formed payload is
    // size==4 with "t", not size==3 -- see Part 1's/Part 2's identical
    // checks (NatNet 3.1 streams, no trailer walked, size stays 3) for the
    // contrast. ----
    {
        int checked = 0, garbage = 0;
        for (const RecvMsg& m : all_msgs) {
            ++checked;
            try {
                nlohmann::json j = nlohmann::json::parse(m.payload);
                if (!j.is_object() || j.size() != 4 || !j.contains("x") || !j.contains("y") ||
                    !j.contains("yaw") || !j.contains("t")) {
                    ++garbage;
                    continue;
                }
                const double x = j.at("x").get<double>();
                const double y = j.at("y").get<double>();
                const double yaw = j.at("yaw").get<double>();
                const double t = j.at("t").get<double>();
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
                    !std::isfinite(t) || std::fabs(x) > 100.0 || std::fabs(y) > 100.0) {
                    ++garbage;
                }
            } catch (const std::exception&) {
                ++garbage;
            }
        }
        rep.info("payload sanity checked on " + std::to_string(checked) + " messages, " +
                  std::to_string(garbage) + " garbage/non-finite");
        if (garbage > 0) {
            rep.fail(std::to_string(garbage) +
                      " published payload(s) were garbage/non-finite/malformed -- version-gate "
                      "regression");
        }
    }

    std::string log_contents;
    {
        std::ifstream log_f(bridge_log_path);
        log_contents.assign(std::istreambuf_iterator<char>(log_f), std::istreambuf_iterator<char>());
    }

    // ---- Check 2: "version confirmed 2.10 via ping" log line appears. ----
    {
        const bool has_confirm_line = log_contents.find("version confirmed 2.10") != std::string::npos &&
                                        log_contents.find("via ping") != std::string::npos;
        rep.info(std::string("'version confirmed 2.10 via ping' log line: ") +
                  (has_confirm_line ? "yes" : "no"));
        if (!has_confirm_line) {
            rep.fail("bridge log does not contain the expected 'version confirmed 2.10 via ping' line");
        }
    }

    // ---- Check 3: frames_deferred > 0 (the gate actually deferred
    // something -- not a silently-inert no-op). Parsed from the shutdown
    // stats summary's "frames_deferred=N" line. ----
    {
        long frames_deferred = -1;
        const std::string key = "frames_deferred=";
        std::size_t pos = log_contents.find(key);
        if (pos != std::string::npos) {
            try {
                frames_deferred = std::stol(log_contents.substr(pos + key.size()));
            } catch (const std::exception&) {
                frames_deferred = -1;
            }
        }
        rep.info("frames_deferred=" + std::to_string(frames_deferred));
        if (frames_deferred <= 0) {
            rep.fail("frames_deferred was not found or was <= 0 -- expected at least one deferred "
                      "frame before ping confirmation");
        }
    }

    // ---- Check 5: parse_errors should be 0 -- once the version is
    // confirmed (correctly, via ping), every subsequent frame should parse
    // cleanly with no failures at all. ----
    {
        long parse_errors = -1;
        const std::string key = "parse_errors=";
        std::size_t pos = log_contents.find(key);
        if (pos != std::string::npos) {
            try {
                parse_errors = std::stol(log_contents.substr(pos + key.size()));
            } catch (const std::exception&) {
                parse_errors = -1;
            }
        }
        rep.info("parse_errors=" + std::to_string(parse_errors));
        if (parse_errors != 0) {
            rep.fail("expected parse_errors=0 once the version is correctly confirmed via ping, got " +
                      std::to_string(parse_errors));
        }
    }

    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);

    return rep;
}

// ---------------------------------------------------------------------
// Part 4 (round 4): Y-up full-6-DoF diagnostics + rigid-body NAME
// resolution, combined (mirrors the user's real setup, which needs both
// together). fake_motive runs --y-up with a Motive asset name that is
// deliberately NOT "robot1"; the bridge is given --rigid-body-names (not
// ids) plus a y_up:true --map-config, so publishing genuinely waits on
// modeldef-driven name resolution AND the y_up geometry conversion is
// exercised end-to-end at the same time.
// ---------------------------------------------------------------------
Report run_part_yup_and_name_resolution(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path csv_path = fs::temp_directory_path() / "test_optitrack_bridge_part4_log.csv";
    const fs::path bridge_log_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part4_stdout.log";
    const fs::path map_config_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part4_map_config.json";
    std::error_code ec;
    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    // Synthetic test-only map-config (deliberately NOT the real project
    // file at MPC/config/mocap_map_config.json -- this test must
    // not depend on that file's content) -- same shape, y_up:true, identity
    // transform.
    {
        std::ofstream cfg(map_config_path);
        cfg << "{\"y_up\": true, \"x0\": 0.0, \"y0\": 0.0, \"theta0\": 0.0, \"yaw_offset\": {}}";
    }

    const std::string kMotiveAssetName = "PushRobotAlpha";  // deliberately NOT "robot1".

    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "127.0.0.1",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--rate",                "120",
        "--natnet-version",      "3.1",
        "--body-ids",            "1",
        "--body-names",          kMotiveAssetName,
        "--y-up",
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotiveYUp",
        "--served-version",      "3.1",
        "--duration-s",          "0",
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive[part4]");
    std::cout << "[test] Part 4: spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const double kBridgeDurationS = 5.0;
    std::vector<std::string> bridge_args = {
        "--server-ip",        "127.0.0.1",
        "--mode",              "unicast",
        "--command-port",      std::to_string(kFakeMotiveCommandPort),
        "--local-data-port",   std::to_string(kBridgeLocalDataPort),
        "--natnet-version",    "3.1",
        "--robots",             "robot1",
        "--rigid-body-names",   kMotiveAssetName,
        "--map-config",         map_config_path.string(),
        "--loc-port-start",     std::to_string(kLocPortStart),
        "--publish-rate",       "30",
        "--log-csv",            csv_path.string(),
        "--duration-s",         std::to_string(kBridgeDurationS),
    };
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()),
                               "optitrack_zmq_bridge[part4]");
    std::cout << "[test] Part 4: spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.set(zmq::sockopt::subscribe, "/" + kRobot1 + "/localization");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] Part 4: optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving Part 4: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();
    std::cout << "[test] Part 4: captured " << all_msgs.size()
               << " total ZMQ messages on robot1's topic" << std::endl;

    // ---- Check 1: frames actually published -- proves name resolution
    // completed AND the y_up geometry conversion didn't silently break
    // anything. ----
    if (all_msgs.size() < 30) {
        rep.fail("too few messages published to robot1's topic: " + std::to_string(all_msgs.size()));
    }

    // ---- Check 2: payload shape + yaw-vs-motion-direction consistency
    // (same style check as Part 1), on the y_up-converted planar stream. ----
    {
        int compared = 0;
        double abs_err_sum = 0.0;
        double max_step = 0.0;
        for (std::size_t i = 0; i + 1 < all_msgs.size(); ++i) {
            nlohmann::json j0, j1;
            try {
                j0 = nlohmann::json::parse(all_msgs[i].payload);
                j1 = nlohmann::json::parse(all_msgs[i + 1].payload);
            } catch (const std::exception&) {
                continue;
            }
            if (!j0.is_object() || j0.size() != 3 || !j0.contains("x") || !j0.contains("y") ||
                !j0.contains("yaw")) {
                rep.fail("a published payload did not have the exact {x,y,yaw} shape");
                continue;
            }
            const double x0 = j0.at("x").get<double>(), y0 = j0.at("y").get<double>();
            const double x1 = j1.at("x").get<double>(), y1 = j1.at("y").get<double>();
            const double yaw0 = j0.at("yaw").get<double>();
            const double dx = x1 - x0, dy = y1 - y0;
            const double step = std::hypot(dx, dy);
            max_step = std::max(max_step, step);
            if (step < 0.001) continue;
            const double secant_dir = std::atan2(dy, dx);
            const double err = std::fabs(angle_diff(secant_dir, yaw0));
            abs_err_sum += err;
            ++compared;
        }
        if (compared < 5) {
            rep.fail("fewer than 5 usable consecutive-sample pairs for yaw-consistency check (got " +
                      std::to_string(compared) + ")");
        } else {
            const double mean_err = abs_err_sum / compared;
            rep.info("y_up yaw-vs-motion-direction: " + std::to_string(compared) +
                      " pairs compared, mean |angle error|=" + std::to_string(mean_err) +
                      " rad, max per-step displacement=" + std::to_string(max_step) + "m");
            if (mean_err > 0.2) {
                rep.fail("y_up yaw does not track motion direction (mean angle error " +
                          std::to_string(mean_err) + " rad > 0.2 rad bound)");
            }
        }
    }

    std::string log_contents;
    {
        std::ifstream log_f(bridge_log_path);
        log_contents.assign(std::istreambuf_iterator<char>(log_f), std::istreambuf_iterator<char>());
    }

    // ---- Check 3: inventory log line (the user's name-discovery tool). ----
    {
        const bool has_inventory =
            log_contents.find("Motive assets:") != std::string::npos &&
            log_contents.find("name='" + kMotiveAssetName + "'") != std::string::npos;
        rep.info(std::string("inventory log line: ") + (has_inventory ? "yes" : "no"));
        if (!has_inventory) {
            rep.fail("bridge log does not contain the expected 'Motive assets: ... name=''" +
                      kMotiveAssetName + "''' inventory line");
        }
    }

    // ---- Check 4: resolution log line. ----
    {
        const bool has_resolution =
            log_contents.find(kRobot1 + " resolved to rigid_body_id=1 (name-match)") != std::string::npos;
        rep.info(std::string("resolution log line: ") + (has_resolution ? "yes" : "no"));
        if (!has_resolution) {
            rep.fail(
                "bridge log does not contain the expected 'robot1 resolved to rigid_body_id=1 "
                "(name-match)' line");
        }
    }

    // ---- Check 5: CSV has the full 6-DoF columns, all finite; raw_y ~=0
    // (the fake circle's flat height in y_up) and roll/pitch ~=0. ----
    {
        std::ifstream csv_f(csv_path);
        if (!csv_f.is_open()) {
            rep.fail("--log-csv file was not created at " + csv_path.string());
        } else {
            std::string header;
            std::getline(csv_f, header);
            const std::string expected_header =
                "t_arrival,robot,raw_x,raw_y,raw_z,qx,qy,qz,qw,roll,pitch,heading,planar_x,planar_y,"
                "planar_yaw,tracking_valid";
            if (header != expected_header) {
                rep.fail("CSV header mismatch: got '" + header + "', expected '" + expected_header + "'");
            }
            int rows = 0, malformed = 0, bad_height = 0, bad_roll_pitch = 0;
            std::string line;
            while (std::getline(csv_f, line)) {
                if (line.empty()) continue;
                std::stringstream ss(line);
                std::vector<std::string> fields;
                std::string tok;
                while (std::getline(ss, tok, ',')) fields.push_back(tok);
                if (fields.size() != 16) {
                    ++malformed;
                    continue;
                }
                try {
                    bool all_finite = true;
                    for (std::size_t k = 0; k < fields.size(); ++k) {
                        if (k == 1) continue;  // robot name column, not numeric.
                        if (!std::isfinite(std::stod(fields[k]))) all_finite = false;
                    }
                    if (!all_finite) {
                        ++malformed;
                        continue;
                    }
                    const double raw_y = std::stod(fields[3]);
                    if (std::fabs(raw_y) > 0.01) ++bad_height;
                    const double roll = std::stod(fields[9]);
                    const double pitch = std::stod(fields[10]);
                    if (std::fabs(roll) > 0.01 || std::fabs(pitch) > 0.01) ++bad_roll_pitch;
                    ++rows;
                } catch (const std::exception&) {
                    ++malformed;
                }
            }
            rep.info("Part 4 CSV: " + std::to_string(rows) + " rows, " + std::to_string(malformed) +
                      " non-finite/malformed, " + std::to_string(bad_height) + " with |raw_y|>0.01, " +
                      std::to_string(bad_roll_pitch) + " with |roll or pitch|>0.01");
            if (rows < 10) {
                rep.fail("Part 4 CSV has suspiciously few rows: " + std::to_string(rows));
            }
            if (malformed > 0) {
                rep.fail("Part 4 CSV has " + std::to_string(malformed) + " non-finite/malformed row(s)");
            }
            if (bad_height > 0) {
                rep.fail("Part 4 CSV has " + std::to_string(bad_height) +
                          " row(s) with |raw_y| > 0.01 -- expected ~0 (flat circle height in y_up)");
            }
            if (bad_roll_pitch > 0) {
                rep.fail("Part 4 CSV has " + std::to_string(bad_roll_pitch) +
                          " row(s) with |roll| or |pitch| > 0.01 -- expected ~0 for a flat circle");
            }
        }
    }

    // ---- Check 6: stats clean (parse_errors=0 in the shutdown summary). ----
    {
        long parse_errors = -1;
        const std::string key = "parse_errors=";
        std::size_t pos = log_contents.find(key);
        if (pos != std::string::npos) {
            try {
                parse_errors = std::stol(log_contents.substr(pos + key.size()));
            } catch (const std::exception&) {
                parse_errors = -1;
            }
        }
        rep.info("Part 4 parse_errors=" + std::to_string(parse_errors));
        if (parse_errors != 0) {
            rep.fail("Part 4 expected parse_errors=0, got " + std::to_string(parse_errors));
        }
    }

    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    return rep;
}

// ---------------------------------------------------------------------
// Part 5 (round 5): zero-arg AUTO-DISCOVERY publish mode. The bridge is
// given NO --robots/--rigid-body-ids/--rigid-body-names at all -- every
// rigid body Motive's modeldef reports gets published, one topic per body,
// on the SINGLE --loc-port-start port. fake_motive serves two bodies
// ("mushr2", "block"); --map-config aliases "mushr2" -> "robot1" (matching
// the user's real setup) while "block" passes through sanitized (already
// topic-safe, so unchanged). fake_motive also uses --modeldef-delay-s to
// deterministically produce a nonzero "frames processed before modeldef
// arrived" count (see fake_motive.cpp's Options doc comment -- on a fast
// loopback round trip that window is otherwise sub-frame-period and
// empirically 0 almost every run, which would make this assertion flaky
// without the deliberate delay).
// ---------------------------------------------------------------------
Report run_part_auto_discovery(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path csv_path = fs::temp_directory_path() / "test_optitrack_bridge_part5_log.csv";
    const fs::path bridge_log_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part5_stdout.log";
    const fs::path map_config_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part5_map_config.json";
    std::error_code ec;
    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    // Mirrors MPC/config/mocap_map_config.json's real shape
    // (y_up:true, identity transform, the "mushr2"->"robot1" alias) but is
    // its OWN synthetic file -- this test must not depend on the real
    // project file's content.
    {
        std::ofstream cfg(map_config_path);
        cfg << "{\"y_up\": true, \"x0\": 0.0, \"y0\": 0.0, \"theta0\": 0.0, \"yaw_offset\": {}, "
               "\"aliases\": {\"mushr2\": \"robot1\"}}";
    }

    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "127.0.0.1",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--rate",                "120",
        "--natnet-version",      "3.1",
        "--body-ids",            "1,2",
        "--body-names",          "mushr2,block",
        "--y-up",
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotiveAutoDiscovery",
        "--served-version",      "3.1",
        "--modeldef-delay-s",    "0.3",  // see this function's doc comment.
        "--duration-s",          "0",
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive[part5]");
    std::cout << "[test] Part 5: spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Deliberately NO --robots/--rigid-body-ids/--rigid-body-names at all.
    const double kBridgeDurationS = 5.0;
    std::vector<std::string> bridge_args = {
        "--server-ip",        "127.0.0.1",
        "--mode",              "unicast",
        "--command-port",      std::to_string(kFakeMotiveCommandPort),
        "--local-data-port",   std::to_string(kBridgeLocalDataPort),
        "--natnet-version",    "3.1",
        "--map-config",         map_config_path.string(),
        "--loc-port-start",     std::to_string(kLocPortStart),
        "--publish-rate",       "30",
        "--log-csv",            csv_path.string(),
        "--duration-s",         std::to_string(kBridgeDurationS),
    };
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()),
                               "optitrack_zmq_bridge[part5]");
    std::cout << "[test] Part 5: spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        // SUB once, on the SINGLE auto-discovery port -- filter twice (both
        // published topics arrive on this one connection).
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.set(zmq::sockopt::subscribe, "/robot1/localization");   // aliased from "mushr2".
        sub.set(zmq::sockopt::subscribe, "/block/localization");    // sanitized passthrough.

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] Part 5: optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving Part 5: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();

    std::vector<RecvMsg> robot1_msgs = filter_topic(all_msgs, "/robot1/localization");
    std::vector<RecvMsg> block_msgs = filter_topic(all_msgs, "/block/localization");
    std::cout << "[test] Part 5: captured " << all_msgs.size() << " total messages (robot1="
               << robot1_msgs.size() << ", block=" << block_msgs.size() << ") on the single port"
               << std::endl;

    // ---- Check 1: BOTH topics actually published on the single port. ----
    if (robot1_msgs.size() < 30) {
        rep.fail("too few /robot1/localization (aliased from 'mushr2') messages: " +
                  std::to_string(robot1_msgs.size()));
    }
    if (block_msgs.size() < 30) {
        rep.fail("too few /block/localization (sanitized passthrough) messages: " +
                  std::to_string(block_msgs.size()));
    }

    // ---- Check 2: payloads sane (exact {x,y,yaw} shape, finite, bounded --
    // y_up mode, same sanity bound style as Part 3's regression check). ----
    {
        int checked = 0, bad = 0;
        for (const RecvMsg& m : all_msgs) {
            ++checked;
            try {
                nlohmann::json j = nlohmann::json::parse(m.payload);
                if (!j.is_object() || j.size() != 3 || !j.contains("x") || !j.contains("y") ||
                    !j.contains("yaw")) {
                    ++bad;
                    continue;
                }
                const double x = j.at("x").get<double>();
                const double y = j.at("y").get<double>();
                const double yaw = j.at("yaw").get<double>();
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) ||
                    std::fabs(x) > 100.0 || std::fabs(y) > 100.0) {
                    ++bad;
                }
            } catch (const std::exception&) {
                ++bad;
            }
        }
        rep.info("payload sanity checked on " + std::to_string(checked) + " messages, " +
                  std::to_string(bad) + " bad");
        if (bad > 0) {
            rep.fail(std::to_string(bad) + " message(s) were malformed/non-finite/out-of-range");
        }
    }

    std::string log_contents;
    {
        std::ifstream log_f(bridge_log_path);
        log_contents.assign(std::istreambuf_iterator<char>(log_f), std::istreambuf_iterator<char>());
    }

    // ---- Check 3: alias + sanitized-passthrough resolution logged. ----
    {
        const bool has_alias_line =
            log_contents.find("motive_name='mushr2' -> /robot1/localization") != std::string::npos;
        const bool has_passthrough_line =
            log_contents.find("motive_name='block' -> /block/localization") != std::string::npos;
        rep.info(std::string("alias resolution logged: ") + (has_alias_line ? "yes" : "no") +
                  ", passthrough resolution logged: " + (has_passthrough_line ? "yes" : "no"));
        if (!has_alias_line) {
            rep.fail("bridge log does not show 'mushr2' resolving to /robot1/localization via alias");
        }
        if (!has_passthrough_line) {
            rep.fail("bridge log does not show 'block' resolving to /block/localization (passthrough)");
        }
    }

    // ---- Check 4: deferred-before-modeldef counter > 0 (deterministic
    // thanks to --modeldef-delay-s -- see this function's doc comment). ----
    {
        long deferred = -1;
        const std::string key = "frames_deferred_before_modeldef=";
        std::size_t pos = log_contents.rfind(key);  // shutdown summary's copy is the final value.
        if (pos != std::string::npos) {
            try {
                deferred = std::stol(log_contents.substr(pos + key.size()));
            } catch (const std::exception&) {
                deferred = -1;
            }
        }
        rep.info("frames_deferred_before_modeldef=" + std::to_string(deferred));
        if (deferred <= 0) {
            rep.fail("expected frames_deferred_before_modeldef > 0 (with --modeldef-delay-s=0.3s at "
                      "120Hz), got " + std::to_string(deferred));
        }
    }

    // ---- Check 5: CSV keys on PUBLISHED names ("robot1"/"block"), not
    // Motive's raw names ("mushr2"). ----
    {
        std::ifstream csv_f(csv_path);
        if (!csv_f.is_open()) {
            rep.fail("--log-csv file was not created at " + csv_path.string());
        } else {
            std::string header;
            std::getline(csv_f, header);
            int rows = 0, robot1_rows = 0, block_rows = 0, other_rows = 0;
            std::string line;
            while (std::getline(csv_f, line)) {
                if (line.empty()) continue;
                std::stringstream ss(line);
                std::vector<std::string> fields;
                std::string tok;
                while (std::getline(ss, tok, ',')) fields.push_back(tok);
                if (fields.size() != 16) continue;
                ++rows;
                if (fields[1] == "robot1") {
                    ++robot1_rows;
                } else if (fields[1] == "block") {
                    ++block_rows;
                } else {
                    ++other_rows;
                }
            }
            rep.info("Part 5 CSV: " + std::to_string(rows) + " rows (robot1=" +
                      std::to_string(robot1_rows) + ", block=" + std::to_string(block_rows) +
                      ", other=" + std::to_string(other_rows) + ")");
            if (robot1_rows < 10 || block_rows < 10) {
                rep.fail("Part 5 CSV should have rows keyed on published names 'robot1' and 'block' "
                          "(not Motive's raw 'mushr2')");
            }
            if (other_rows > 0) {
                rep.fail("Part 5 CSV has " + std::to_string(other_rows) +
                          " row(s) with an unexpected robot-column value (not 'robot1'/'block')");
            }
        }
    }

    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    return rep;
}

// ---------------------------------------------------------------------
// Part 6 (round 8): mocap clock sync. fake_motive streams REAL NatNet 2.10
// (the one version whose trailer this bridge actually walks -- see
// OptiTrackCore.h's has_timestamp doc comment) with its DEFAULT (nominal-
// schedule, frame_number/rate_hz -- see fake_motive.cpp's
// Options::emit_timestamp doc comment) trailer timestamps; --no-timestamp
// is deliberately NOT passed. The published "t" field must be present,
// monotonic, and its deltas must quantize to whole multiples of the FAKE
// FRAME CLOCK's own period -- NOT this test's own receipt-time jitter
// (network/scheduling delivery of the ~30Hz downsampled publish ticks is
// never perfectly periodic in wall-clock terms, but "t" derives from
// frame_number, not from when this test happened to observe the message).
// ---------------------------------------------------------------------
Report run_part_mocap_clock_sync(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path bridge_log_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part6_stdout.log";
    std::error_code ec;
    fs::remove(bridge_log_path, ec);

    constexpr double kFakeRateHz = 120.0;
    constexpr double kPublishRateHz = 30.0;
    const double kFramePeriodS = 1.0 / kFakeRateHz;

    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "127.0.0.1",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--rate",                std::to_string(kFakeRateHz),
        "--natnet-version",      "2.10",
        "--body-ids",            "1",
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotiveClockSync",
        "--served-version",      "2.10",
        "--duration-s",          "0",
        // --no-timestamp deliberately NOT passed -- default is emit_timestamp=true.
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive[part6]");
    std::cout << "[test] Part 6: spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const double kBridgeDurationS = 5.0;
    std::vector<std::string> bridge_args = {
        "--server-ip",        "127.0.0.1",
        "--mode",              "unicast",
        "--command-port",      std::to_string(kFakeMotiveCommandPort),
        "--local-data-port",   std::to_string(kBridgeLocalDataPort),
        "--natnet-version",    "2.10",
        "--robots",            "robot1",
        "--rigid-body-ids",    "1",
        "--loc-port-start",    std::to_string(kLocPortStart),
        "--publish-rate",      std::to_string(kPublishRateHz),
        "--duration-s",        std::to_string(kBridgeDurationS),
    };
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()),
                               "optitrack_zmq_bridge[part6]");
    std::cout << "[test] Part 6: spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.set(zmq::sockopt::subscribe, "/" + kRobot1 + "/localization");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] Part 6: optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving Part 6: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();
    std::cout << "[test] Part 6: captured " << all_msgs.size() << " messages" << std::endl;

    if (all_msgs.size() < 30) {
        rep.fail("too few messages captured: " + std::to_string(all_msgs.size()));
        fs::remove(bridge_log_path, ec);
        return rep;
    }

    // ---- Check 1: the one-time clock-sync offset log line. ----
    {
        std::ifstream log_f(bridge_log_path);
        std::string log_contents((std::istreambuf_iterator<char>(log_f)),
                                   std::istreambuf_iterator<char>());
        const bool has_sync_line = log_contents.find("mocap clock sync: offset=") != std::string::npos;
        rep.info(std::string("'mocap clock sync: offset=' log line: ") + (has_sync_line ? "yes" : "no"));
        if (!has_sync_line) {
            rep.fail("bridge log does not contain the expected mocap clock sync offset line");
        }
    }

    // ---- Check 2: EVERY message carries a finite "t" alongside x/y/yaw. ----
    std::vector<double> ts;
    ts.reserve(all_msgs.size());
    {
        int checked = 0, missing_t = 0;
        for (const RecvMsg& m : all_msgs) {
            ++checked;
            try {
                nlohmann::json j = nlohmann::json::parse(m.payload);
                if (!j.is_object() || !j.contains("x") || !j.contains("y") || !j.contains("yaw") ||
                    !j.contains("t") || !j.at("t").is_number()) {
                    ++missing_t;
                    continue;
                }
                const double t = j.at("t").get<double>();
                if (!std::isfinite(t)) {
                    ++missing_t;
                    continue;
                }
                ts.push_back(t);
            } catch (const std::exception&) {
                ++missing_t;
            }
        }
        rep.info("checked " + std::to_string(checked) + " messages, " + std::to_string(missing_t) +
                  " missing/non-finite 't'");
        if (missing_t > 0) {
            rep.fail(std::to_string(missing_t) +
                      " message(s) missing a finite 't' field (NatNet 2.10 stream -- 't' should be "
                      "present on every message once the version is confirmed)");
        }
    }

    if (ts.size() < 10) {
        rep.fail("too few timestamped messages to check monotonicity/quantization: " +
                  std::to_string(ts.size()));
        fs::remove(bridge_log_path, ec);
        return rep;
    }

    // ---- Check 3: "t" strictly increasing across consecutive messages. ----
    {
        int violations = 0;
        for (std::size_t i = 1; i < ts.size(); ++i) {
            if (ts[i] <= ts[i - 1]) ++violations;
        }
        rep.info(std::to_string(violations) + " non-increasing 't' step(s) out of " +
                  std::to_string(ts.size() - 1));
        if (violations > 0) {
            rep.fail(std::to_string(violations) +
                      " published 't' value(s) were not strictly increasing across consecutive "
                      "messages");
        }
    }

    // ---- Check 4: "t" deltas quantize to whole multiples of the FAKE FRAME
    // CLOCK's own period -- the direct proof that "t" tracks the mocap
    // frame clock (frame_number/rate_hz) rather than this test's own
    // receipt-time jitter (a receipt-time-derived delta would NOT cleanly
    // land on multiples of a fixed 1/120s period). ----
    {
        constexpr double kQuantizationTolS = 0.0005;  // 0.5ms -- generous vs. float64 precision.
        int violations = 0;
        double max_deviation_s = 0.0;
        for (std::size_t i = 1; i < ts.size(); ++i) {
            const double delta = ts[i] - ts[i - 1];
            const double periods = delta / kFramePeriodS;
            const double nearest_whole = std::round(periods);
            const double deviation_s = std::fabs(periods - nearest_whole) * kFramePeriodS;
            max_deviation_s = std::max(max_deviation_s, deviation_s);
            if (nearest_whole < 1.0 || deviation_s > kQuantizationTolS) {
                ++violations;
            }
        }
        rep.info("t-delta frame-period quantization: " + std::to_string(violations) +
                  " violation(s) out of " + std::to_string(ts.size() - 1) + ", max deviation=" +
                  std::to_string(max_deviation_s) + "s (frame period=" + std::to_string(kFramePeriodS) +
                  "s)");
        if (violations > 0) {
            rep.fail(std::to_string(violations) +
                      " 't' delta(s) did not quantize to a whole multiple of the fake frame period -- "
                      "'t' appears to track receipt jitter rather than the mocap frame clock");
        }
    }

    fs::remove(bridge_log_path, ec);
    return rep;
}

// ---------------------------------------------------------------------
// Part 7: mocap pose-jump filter (mpc::PoseFilter) integration -- see doc/
// MOCAP_POSE_FILTER_PLAN.md. --map-config's "pose_filter" block turns the
// filter ON (it ships disabled by default; Parts 1-6 above never set this
// block, so they exercise the byte-for-byte unaffected default path).
// fake_motive's synthetic track is smooth (no injected jumps -- fake_motive
// isn't a mocap pose_filter has a CLI to modify), so this checks the
// PASS-THROUGH-ON-CLEAN-DATA side: virtually every sample should be
// accepted, and the filtered pose (fx/fy/fyaw, and the ZMQ payload's
// x/y/yaw) should track the raw pose (planar_x/y/yaw) closely -- i.e. the
// filter is close to invisible on clean data, exactly as designed. The
// synthetic single/few-frame-spike/reinit BEHAVIOR itself is covered by
// MPC/tests/pose_filter_tests.cpp's deterministic unit tests and
// pose_filter_replay's real-recording validation (see the plan doc) --
// this test's job is only to prove the BRIDGE WIRING (config parsing, the
// additive ZMQ/CSV fields, enabled-by-config) actually works end-to-end.
// ---------------------------------------------------------------------
Report run_part_pose_filter_integration(const std::string& dir) {
    Report rep;
    namespace fs = std::filesystem;

    const std::string fake_motive_exe = dir + "/fake_motive";
    const std::string bridge_exe = dir + "/optitrack_zmq_bridge";

    const fs::path csv_path = fs::temp_directory_path() / "test_optitrack_bridge_part7_log.csv";
    const fs::path bridge_log_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part7_stdout.log";
    const fs::path map_config_path =
        fs::temp_directory_path() / "test_optitrack_bridge_part7_map_config.json";
    std::error_code ec;
    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);

    // Synthetic test-only map-config (identity transform) with pose_filter
    // explicitly enabled, default gate/noise parameters otherwise (mirrors
    // MPC/config/mocap_map_config.json's real "pose_filter" block shape).
    {
        std::ofstream cfg(map_config_path);
        cfg << "{\"y_up\": false, \"x0\": 0.0, \"y0\": 0.0, \"theta0\": 0.0, \"yaw_offset\": {}, "
               "\"pose_filter\": {\"enabled\": true}}";
    }

    std::vector<std::string> fake_motive_args = {
        "--target-ip",           "127.0.0.1",
        "--target-port",         std::to_string(kBridgeLocalDataPort),
        "--rate",                "120",
        "--natnet-version",      "3.1",
        "--body-ids",            "1,2",
        "--body-names",          "Body1,Body2",
        "--serve-command-port",  std::to_string(kFakeMotiveCommandPort),
        "--served-app-name",     "TestMotivePoseFilter",
        "--served-version",      "3.1",
        "--duration-s",          "0",
    };
    ProcessGuard fake_motive_guard(spawn(fake_motive_exe, fake_motive_args), "fake_motive[part7]");
    std::cout << "[test] Part 7: spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const double kBridgeDurationS = 5.0;
    std::vector<std::string> bridge_args = {
        "--server-ip",        "127.0.0.1",
        "--mode",              "unicast",
        "--command-port",      std::to_string(kFakeMotiveCommandPort),
        "--local-data-port",   std::to_string(kBridgeLocalDataPort),
        "--natnet-version",    "3.1",
        "--robots",             kRobot1 + "," + kRobot2,
        "--rigid-body-ids",     "1,2",
        "--map-config",         map_config_path.string(),
        "--loc-port-start",     std::to_string(kLocPortStart),
        "--publish-rate",       "30",
        "--log-csv",            csv_path.string(),
        "--duration-s",         std::to_string(kBridgeDurationS),
    };
    ProcessGuard bridge_guard(spawn(bridge_exe, bridge_args, bridge_log_path.string()),
                               "optitrack_zmq_bridge[part7]");
    std::cout << "[test] Part 7: spawned optitrack_zmq_bridge pid=" << bridge_guard.pid() << std::endl;

    std::vector<RecvMsg> all_msgs;
    try {
        zmq::context_t ctx(1);
        zmq::socket_t sub(ctx, zmq::socket_type::sub);
        sub.set(zmq::sockopt::linger, 0);
        sub.connect("tcp://127.0.0.1:" + std::to_string(kLocPortStart));
        sub.set(zmq::sockopt::subscribe, "/" + kRobot1 + "/localization");
        sub.set(zmq::sockopt::subscribe, "/" + kRobot2 + "/localization");

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // slow-joiner grace.

        bool exited = bridge_guard.wait_for_exit(
            kBridgeDurationS + 4.0, 0.05, [&]() { drain_messages(sub, all_msgs); });
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        drain_messages(sub, all_msgs);

        if (!exited) {
            rep.fail("optitrack_zmq_bridge did not self-exit within budget");
        } else {
            std::cout << "[test] Part 7: optitrack_zmq_bridge exited on its own" << std::endl;
        }
    } catch (const std::exception& ex) {
        rep.fail(std::string("exception while driving Part 7: ") + ex.what());
    }

    bridge_guard.terminate();
    fake_motive_guard.terminate();
    std::cout << "[test] Part 7: captured " << all_msgs.size() << " messages" << std::endl;

    if (all_msgs.size() < 30) {
        rep.fail("too few messages captured: " + std::to_string(all_msgs.size()));
        fs::remove(csv_path, ec);
        fs::remove(bridge_log_path, ec);
        fs::remove(map_config_path, ec);
        return rep;
    }

    // ---- Check 1: the bridge logged pose_filter enabled=yes. ----
    {
        std::ifstream log_f(bridge_log_path);
        std::string log_contents((std::istreambuf_iterator<char>(log_f)),
                                   std::istreambuf_iterator<char>());
        const bool has_enabled_line = log_contents.find("pose_filter: enabled=yes") != std::string::npos;
        rep.info(std::string("'pose_filter: enabled=yes' log line: ") + (has_enabled_line ? "yes" : "no"));
        if (!has_enabled_line) {
            rep.fail("bridge log does not show pose_filter enabled (map-config's \"pose_filter\" "
                      "block was not picked up)");
        }
    }

    // ---- Check 2: every ZMQ payload gains filt/rej/vx/vy/w on top of
    // x/y/yaw (no "t" here -- natnet_version 3.1 carries no trailer
    // timestamp in this synthetic setup, matching Parts 4/5's own
    // assumption). ----
    {
        int checked = 0, malformed = 0;
        for (const RecvMsg& m : all_msgs) {
            ++checked;
            try {
                nlohmann::json j = nlohmann::json::parse(m.payload);
                if (!j.is_object() || !j.contains("x") || !j.contains("y") || !j.contains("yaw") ||
                    !j.contains("filt") || !j.contains("rej") || !j.contains("vx") ||
                    !j.contains("vy") || !j.contains("w") || j.at("filt").get<int>() != 1) {
                    ++malformed;
                }
            } catch (const std::exception&) {
                ++malformed;
            }
        }
        rep.info("payload filt/rej/vx/vy/w fields checked on " + std::to_string(checked) +
                  " messages, " + std::to_string(malformed) + " malformed/missing");
        if (malformed > 0) {
            rep.fail(std::to_string(malformed) +
                      " message(s) missing the additive pose_filter JSON fields (filt/rej/vx/vy/w)");
        }
    }

    // ---- Check 3: --log-csv header ends with the 6 pose_filter columns,
    // and on this clean synthetic track virtually every row is accepted
    // with fx/fy/fyaw close to planar_x/y/planar_yaw. ----
    {
        std::ifstream csv_f(csv_path);
        std::string header;
        std::getline(csv_f, header);
        const std::string expected_suffix = ",accepted,d2_pos,d2_yaw,fx,fy,fyaw";
        const bool header_ok =
            header.size() >= expected_suffix.size() &&
            header.compare(header.size() - expected_suffix.size(), expected_suffix.size(),
                            expected_suffix) == 0;
        rep.info(std::string("CSV header ends with pose_filter columns: ") + (header_ok ? "yes" : "no") +
                  " (header='" + header + "')");
        if (!header_ok) {
            rep.fail("--log-csv header does not end with ',accepted,d2_pos,d2_yaw,fx,fy,fyaw'");
        }

        int rows = 0, rejected = 0, fx_mismatch = 0;
        std::string line;
        while (std::getline(csv_f, line)) {
            if (line.empty()) continue;
            std::vector<std::string> fields;
            std::stringstream ss(line);
            std::string tok;
            while (std::getline(ss, tok, ',')) fields.push_back(tok);
            if (fields.size() != 22) continue;  // 16 base + 6 pose_filter columns.
            ++rows;
            try {
                const int accepted = std::stoi(fields[16]);
                const double planar_x = std::stod(fields[12]);
                const double planar_y = std::stod(fields[13]);
                const double fx = std::stod(fields[19]);
                const double fy = std::stod(fields[20]);
                if (accepted == 0) {
                    ++rejected;
                } else if (std::hypot(fx - planar_x, fy - planar_y) > 0.05) {
                    ++fx_mismatch;
                }
            } catch (const std::exception&) {
                continue;
            }
        }
        rep.info("Part 7 CSV: " + std::to_string(rows) + " rows, " + std::to_string(rejected) +
                  " rejected, " + std::to_string(fx_mismatch) +
                  " accepted-but-filtered-pose-off-by->5cm");
        if (rows < 30) {
            rep.fail("Part 7 CSV has suspiciously few well-formed pose_filter rows: " +
                      std::to_string(rows));
        }
        // Clean synthetic data -- the filter should be near-invisible (a
        // small handful of rejects on a fresh multi-body track's very first
        // few frames, before both filters have converged, is tolerated).
        if (rejected > rows / 10) {
            rep.fail("too many rejected rows on a clean synthetic track: " + std::to_string(rejected) +
                      "/" + std::to_string(rows));
        }
        if (fx_mismatch > 0) {
            rep.fail(std::to_string(fx_mismatch) +
                      " accepted row(s) had a filtered pose >5cm from the raw pose on clean data");
        }
    }

    fs::remove(csv_path, ec);
    fs::remove(bridge_log_path, ec);
    fs::remove(map_config_path, ec);
    return rep;
}

int main() {
    std::string dir;
    try {
        dir = self_dir();
    } catch (const std::exception& ex) {
        std::cerr << "[test] " << ex.what() << std::endl;
        return 1;
    }

    Report part1 = run_part_unicast(dir);
    std::cout << "\n=== Part 1 (unicast, tracking-drop, ping version discovery): "
               << (part1.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part1.notes) {
        std::cout << "  " << n << "\n";
    }

    // Sequential, not concurrent -- reuses the SAME UDP/ZMQ test ports as
    // Part 1, which has already fully torn its processes down by this point
    // (mirrors test_mpc_deadband.cpp's own multi-part port-reuse discipline).
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    Report part2 = run_part_multicast_and_mocap_config(dir);
    std::cout << "\n=== Part 2 (multicast + mocap-config, multicast version discovery): "
               << (part2.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part2.notes) {
        std::cout << "  " << n << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    Report part3 = run_part_version_gating(dir);
    std::cout << "\n=== Part 3 (version-confirmation gating, wrong-fallback regression): "
               << (part3.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part3.notes) {
        std::cout << "  " << n << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    Report part4 = run_part_yup_and_name_resolution(dir);
    std::cout << "\n=== Part 4 (y_up 6-DoF diagnostics + rigid-body NAME resolution): "
               << (part4.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part4.notes) {
        std::cout << "  " << n << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    Report part5 = run_part_auto_discovery(dir);
    std::cout << "\n=== Part 5 (zero-arg auto-discovery publish mode, alias + sanitized passthrough): "
               << (part5.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part5.notes) {
        std::cout << "  " << n << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    Report part6 = run_part_mocap_clock_sync(dir);
    std::cout << "\n=== Part 6 (mocap clock sync: 't' field, monotonic, frame-clock-quantized deltas): "
               << (part6.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part6.notes) {
        std::cout << "  " << n << "\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    Report part7 = run_part_pose_filter_integration(dir);
    std::cout << "\n=== Part 7 (mocap pose-jump filter bridge integration): "
               << (part7.pass ? "PASS" : "FAIL") << " ===\n";
    for (const std::string& n : part7.notes) {
        std::cout << "  " << n << "\n";
    }

    const bool overall_pass = part1.pass && part2.pass && part3.pass && part4.pass && part5.pass &&
                                part6.pass && part7.pass;
    std::cout << "\n=== test_optitrack_bridge OVERALL: " << (overall_pass ? "PASS" : "FAIL") << " ===\n";
    return overall_pass ? 0 : 1;
}
