// SLOW, process-spawning integration tests for VescDriver: every test
// here forks/execs a real child process (fake_vesc, and/or the real
// vesc_driver/vesc_mcconf/vesc_teleop binaries), often over a pty and/or
// real ZMQ sockets. Same plain bool test_xxx() harness as
// vesc_unit_tests.cpp (see test_support.h), no gtest dependency.
// Standalone manually-run binary -- NOT add_test()'d (see
// VescDriver/CMakeLists.txt's own comment). Split out of the former
// monolithic vesc_driver_tests.cpp: run this one when a change touches
// process spawning / wire behavior / CLI startup, and before deploying
// to the robot -- see vesc_unit_tests.cpp for the fast, pure in-process
// codec/logic tests that should be the everyday default.

#include "../src/AckermannCodec.h"
#include "../src/DriverCore.h"
#include "../src/McconfPatcher.h"
#include "../src/PortDiscovery.h"
#include "../src/SerialPort.h"
#include "../src/SteeringCalib.h"
#include "../src/TeleopCore.h"
#include "../src/VescProtocol.h"
#include "FakeVescModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <zmq.h>

// Vendored base64 (used only to build a couple of test fixtures at
// runtime for the AckermannCodec malformation-class tests below; the
// primary valid-decode test uses a hand-computed literal instead -- see
// test_ackermann_valid_decode()) -- and, further down, to build wire-exact
// ackermann payloads for part 3's pty+ZMQ integration test.
#include "../third_party/base64.h"
// Vendored nlohmann::json -- used by part 3's DriverConfig/calibration
// loader tests and the pty+ZMQ integration test (building/parsing ZMQ
// JSON payloads and scanning fake_vesc's --log JSONL output).
#include "../third_party/nlohmann/json.hpp"

#include "test_support.h"

namespace {

// ---------------------------------------------------------------------
// (g) pty integration: fork/exec the fake_vesc binary and talk to it over
// a real vesc::SerialPort on the pty slave it prints, exactly as this
// driver would talk to real hardware.
// ---------------------------------------------------------------------

namespace pty_test {

// RAII process guard: SIGTERM, short grace period, SIGKILL fallback,
// idempotent, reaps in its destructor. Mirrors the ProcessGuard pattern
// used by every process-spawning test elsewhere in this repo (see
// MARS/tests/test_sim_viz_shared.h) -- reimplemented locally here since
// VescDriver may not #include anything outside VescDriver/. Relies on
// main()'s top-level try/catch (below) to guarantee this destructor still
// runs even if a check throws between spawn and an explicit terminate()
// call: GCC's std::terminate() (an exception escaping uncaught) does NOT
// unwind locals, so a scope-local guard alone is not sufficient without a
// reachable catch somewhere in the call chain.
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

    // Disarms the guard WITHOUT sending any signal or calling waitpid()
    // again -- for a caller that has already itself waitpid()'d the
    // child to a graceful exit (e.g. testing a clean 'q'-triggered
    // shutdown) and doesn't want terminate()'s SIGTERM-then-2s-grace
    // path to run redundantly against a pid that is no longer a valid
    // child (which would just stall on repeated failing waitpid() calls,
    // not do anything harmful, but wastes ~2s for no reason).
    void mark_reaped() { pid_ = -1; }

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
        std::cerr << "[vesc_driver_tests] pid " << pid_ << " (" << label_
                  << ") did not exit after SIGTERM; sending SIGKILL\n";
        kill(pid_, SIGKILL);
        waitpid(pid_, nullptr, 0);
        pid_ = -1;
    }

private:
    pid_t pid_ = -1;
    std::string label_;
};

// Directory this test binary itself was loaded from -- fake_vesc is
// expected to sit right next to it (both are add_executable() targets
// defined in the same CMakeLists.txt with no custom output directory, so
// both land in the same build directory whether built standalone or via
// the repo superbuild).
std::string own_exe_dir() {
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "";
    buf[n] = '\0';
    const std::string path(buf);
    const size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

// Spawns `exe` (wrapped per this repo's standing env rule) with `args`,
// redirecting its stdout to `stdout_path` so the caller can poll that file
// for output (fake_vesc's "PTY <path>" first line in particular) without
// needing a live pipe-read loop.
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
    if (pid < 0) throw std::runtime_error("fork() failed");
    if (pid == 0) {
        const int fd = open(stdout_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            close(fd);
        }
        execv(env_exe.c_str(), argv.data());
        std::fprintf(stderr, "[vesc_driver_tests] execv('%s') failed\n", env_exe.c_str());
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

// Like spawn_wrapped_capture_stdout(), but blocks until the child exits
// (or `timeout_s` elapses, in which case it is SIGKILLed) and returns its
// exit code, or -1 on timeout / -2 if it died from a signal.
int run_wrapped_capture_stdout(const std::string& exe, const std::vector<std::string>& args,
                                const std::string& stdout_path, double timeout_s) {
    const pid_t pid = spawn_wrapped_capture_stdout(exe, args, stdout_path);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    int status = 0;
    for (;;) {
        const pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) break;
        if (std::chrono::steady_clock::now() > deadline) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
            return -1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -2;
}

// Returns the full path of the first file directly under `dir` whose
// name starts with `prefix`, or "" if none/dir doesn't exist. Used to
// locate vesc_mcconf's timestamped backup file without needing
// std::filesystem (forbidden in this folder -- POSIX dirent only).
std::string find_first_file_with_prefix(const std::string& dir, const std::string& prefix) {
    DIR* d = opendir(dir.c_str());
    if (!d) return "";
    std::string found;
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        const std::string name = ent->d_name;
        if (name.rfind(prefix, 0) == 0) {
            found = dir + "/" + name;
            break;
        }
    }
    closedir(d);
    return found;
}

// Polls `stdout_path` until its contents contain a "PTY <path>" line, up
// to `timeout_s`. Returns the slave path (no trailing newline), or "" on
// timeout.
std::string wait_for_pty_line(const std::string& stdout_path, double timeout_s) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        const std::string contents = read_file(stdout_path);
        const size_t pos = contents.find("PTY ");
        if (pos != std::string::npos) {
            size_t end = contents.find('\n', pos);
            const std::string line = (end == std::string::npos) ? contents.substr(pos) : contents.substr(pos, end - pos);
            return line.substr(4);  // strip "PTY " prefix
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return "";
}

// Like spawn_wrapped_capture_stdout(), but ALSO gives the child a pipe on
// its stdin (read end) instead of inheriting this test binary's own
// stdin -- lets a test drive an interactive CLI (vesc_teleop) by writing
// individual key bytes into *stdin_write_fd over time. The caller owns
// that fd and must close() it itself (ProcessGuard only manages the
// child process, not this fd).
//
// `home_override`, if non-empty, REPLACES the child's HOME (instead of
// inheriting this test binary's own real $HOME) in the SAME "env -i
// HOME=... USER=... PATH=... QT_QPA_PLATFORM=offscreen <exe>" environment
// this function already builds -- this is how a test can point a spawned
// vesc_teleop at a fake, throwaway $HOME/.vesc/steering_calib.json
// without ever touching the real one. Empty (the default) preserves the
// prior byte-for-byte behavior for every existing caller.
pid_t spawn_wrapped_stdin_pipe_capture_stdout(const std::string& exe, const std::vector<std::string>& args,
                                               const std::string& stdout_path, int* stdin_write_fd,
                                               const std::string& home_override = "") {
    int pipe_fds[2];
    if (pipe(pipe_fds) != 0) throw std::runtime_error("pipe() failed");

    const char* home = std::getenv("HOME");
    const char* user = std::getenv("USER");
    const std::string effective_home = home_override.empty() ? (home ? home : "") : home_override;
    std::vector<std::string> wrapped_args = {
        "-i", std::string("HOME=") + effective_home,
        std::string("USER=") + (user ? user : ""), "PATH=/usr/local/bin:/usr/bin:/bin",
        "QT_QPA_PLATFORM=offscreen", exe};
    for (const auto& a : args) wrapped_args.push_back(a);

    const std::string env_exe = "/usr/bin/env";
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(env_exe.c_str()));
    for (auto& a : wrapped_args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);

    const pid_t pid = fork();
    if (pid < 0) throw std::runtime_error("fork() failed");
    if (pid == 0) {
        dup2(pipe_fds[0], STDIN_FILENO);
        close(pipe_fds[0]);
        close(pipe_fds[1]);
        const int fd = open(stdout_path.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            close(fd);
        }
        execv(env_exe.c_str(), argv.data());
        std::fprintf(stderr, "[vesc_driver_tests] execv('%s') failed\n", env_exe.c_str());
        _exit(127);
    }
    close(pipe_fds[0]);
    *stdin_write_fd = pipe_fds[1];
    return pid;
}

// Writes all of `s` to `fd` (retrying on EINTR), used to send scripted
// keypress bytes to a child's piped stdin. Returns false (rather than
// letting a dead child's closed pipe raise SIGPIPE and kill THIS whole
// test binary) on any write error -- relies on main() having set
// SIGPIPE to SIG_IGN, so a broken-pipe write() here surfaces as a normal
// EPIPE return rather than a signal.
bool write_to_fd(int fd, const std::string& s) {
    size_t off = 0;
    while (off < s.size()) {
        const ssize_t n = ::write(fd, s.data() + off, s.size() - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        return false;
    }
    return true;
}

}  // namespace pty_test

bool test_pty_fake_vesc_fw_version_and_rpm_tracking() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (expected at '" +
                                      fake_vesc_exe + "' -- build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string log_path = tmp_dir + "/fake_vesc_log.jsonl";

    const std::vector<std::string> args = {
        "--fw", "5.2", "--tau-s", "0.15", "--stall-erpm", "800", "--log", log_path,
    };
    ProcessGuard guard(spawn_wrapped_capture_stdout(fake_vesc_exe, args, stdout_path), "fake_vesc");

    const std::string slave_path = wait_for_pty_line(stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::cout << "    [test] fake_vesc pty slave = " << slave_path << "\n";

    // Give fake_vesc's poll loop a brief moment to settle; harmless either
    // way since the pty itself buffers whatever we write below.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    vesc::SerialPort port;
    ok &= check_true(port.open(slave_path), "SerialPort::open() succeeds on the fake_vesc pty slave (" +
                                                 port.last_error() + ")");
    if (!port.is_open()) return ok;

    vesc::FrameDecoder decoder;

    // FW_VERSION.
    ok &= check_true(port.write_all(vesc::encode_frame(vesc::build_fw_version())), "write FW_VERSION request");
    vesc::FwVersionReply fw_reply;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline && !fw_reply.ok) {
            std::vector<uint8_t> buf;
            if (port.read_available(&buf) > 0) decoder.feed(buf);
            std::vector<uint8_t> payload;
            while (decoder.pop_payload(&payload)) {
                const vesc::FwVersionReply r = vesc::parse_fw_version(payload);
                if (r.ok) fw_reply = r;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    ok &= check_true(fw_reply.ok, "FW_VERSION reply parses ok within 2s");
    ok &= check_true(fw_reply.major == 5 && fw_reply.minor == 2,
                      "FW_VERSION reply reports 5.2 as configured via --fw (got " +
                          std::to_string(fw_reply.major) + "." + std::to_string(fw_reply.minor) + ")");

    // Sends a fresh GET_VALUES request and waits up to `timeout_s` for the
    // reply, returning the last successfully-parsed VescValues seen (all
    // requests within the window are drained; only the newest is kept).
    auto poll_get_values = [&](double timeout_s) -> vesc::VescValues {
        vesc::VescValues last;
        port.write_all(vesc::encode_frame(vesc::build_get_values()));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            std::vector<uint8_t> buf;
            if (port.read_available(&buf) > 0) decoder.feed(buf);
            std::vector<uint8_t> payload;
            while (decoder.pop_payload(&payload)) {
                const vesc::VescValues v = vesc::parse_get_values(payload);
                if (v.ok) last = v;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return last;
    };

    // SET_RPM(1500) -- tau=0.15s, so ~0.6s (4 tau) should have it well
    // converged; assert within 10% of 1500.
    ok &= check_true(port.write_all(vesc::encode_frame(vesc::build_set_rpm(1500))), "write SET_RPM(1500)");
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    const vesc::VescValues after_rise = poll_get_values(0.5);
    ok &= check_true(after_rise.ok, "GET_VALUES reply parses ok after SET_RPM(1500) + settling");
    std::cout << "    [test] erpm after SET_RPM(1500) + ~0.6s settle = " << after_rise.erpm << "\n";
    ok &= check_true(std::fabs(after_rise.erpm - 1500) <= 0.10 * 1500,
                      "erpm within 10% of 1500 after settling (got " + std::to_string(after_rise.erpm) + ")");

    // SET_RPM(500) -- below the 800 stall threshold -> target snaps to 0;
    // erpm should decay back toward 0.
    ok &= check_true(port.write_all(vesc::encode_frame(vesc::build_set_rpm(500))),
                      "write SET_RPM(500) (below stall_erpm=800)");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));  // ~4-5 tau of decay from 1500 toward 0
    const vesc::VescValues after_decay = poll_get_values(0.5);
    ok &= check_true(after_decay.ok, "GET_VALUES reply parses ok after SET_RPM(500) + decay");
    std::cout << "    [test] erpm after SET_RPM(500 < stall) + decay = " << after_decay.erpm << "\n";
    ok &= check_true(std::fabs(after_decay.erpm) <= 0.10 * 1500,
                      "erpm decays to near 0 after a below-stall SET_RPM (got " +
                          std::to_string(after_decay.erpm) + ")");

    guard.terminate();

    // --log JSONL recorded the commands this test sent.
    const std::string log_contents = read_file(log_path);
    ok &= check_true(log_contents.find("\"cmd\":\"set_rpm\"") != std::string::npos,
                      "--log JSONL recorded at least one set_rpm command");
    ok &= check_true(log_contents.find("\"value\":1500") != std::string::npos,
                      "--log JSONL recorded the SET_RPM(1500) value");
    ok &= check_true(log_contents.find("\"value\":500") != std::string::npos,
                      "--log JSONL recorded the SET_RPM(500) value");

    return ok;
}

// ---------------------------------------------------------------------
// (g2) pty + subprocess integration: spawn fake_vesc, then actually
// invoke the real built `vesc_mcconf` binary against it (as a real
// subprocess, wrapped per this repo's standing env rule) and assert on
// its captured stdout/exit code -- scan -> dump -> patch -> scan ->
// restore -> scan.
// ---------------------------------------------------------------------

bool test_pty_vesc_mcconf_cli_e2e() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_mcconf_exe = exe_dir.empty() ? "vesc_mcconf" : exe_dir + "/vesc_mcconf";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (expected at '" +
                                      fake_vesc_exe + "' -- build it first)");
    }
    if (access(vesc_mcconf_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_mcconf executable exists next to vesc_driver_tests (expected at '" +
                                      vesc_mcconf_exe + "' -- build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_mcconf_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string backup_dir = tmp_dir + "/backups";
    mkdir(backup_dir.c_str(), 0755);
    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";

    ProcessGuard fake_guard(spawn_wrapped_capture_stdout(fake_vesc_exe, {"--log", tmp_dir + "/fake_vesc_log.jsonl"},
                                                          fake_stdout_path),
                             "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto run_mcconf = [&](const std::vector<std::string>& extra_args, const std::string& tag) -> std::string {
        // extra_args[0] must be the subcommand (argv[1]) -- vesc_mcconf's
        // parse_cli() expects it first; flags like --serial-port come
        // after, same as the CLI's own documented usage.
        std::vector<std::string> args = extra_args;
        args.push_back("--serial-port");
        args.push_back(slave_path);
        const std::string out_path = tmp_dir + "/mcconf_" + tag + "_stdout.txt";
        const int rc = run_wrapped_capture_stdout(vesc_mcconf_exe, args, out_path, 10.0);
        const std::string out = read_file(out_path);
        std::cout << "    [test] vesc_mcconf " << tag << " rc=" << rc << "\n";
        ok &= check_true(rc == 0, "vesc_mcconf " + tag + " exits 0 (got " + std::to_string(rc) + "); stdout:\n" + out);
        return out;
    };

    // 1) scan -- initial (unpatched) state.
    {
        const std::string out = run_mcconf({"scan"}, "scan1");
        ok &= check_true(out.find("UNAMBIGUOUS") != std::string::npos, "scan1: reports an UNAMBIGUOUS cluster");
        ok &= check_true(out.find("kp=0.008") != std::string::npos, "scan1: shows kp=0.008");
        ok &= check_true(out.find("min_erpm=900") != std::string::npos, "scan1: shows min_erpm=900");
        ok &= check_true(out.find("current_max=25") != std::string::npos, "scan1: shows current_max=25");
        ok &= check_true(out.find("current_min=-25") != std::string::npos, "scan1: shows current_min=-25");
        ok &= check_true(out.find("in_current_max=50") != std::string::npos, "scan1: shows in_current_max=50");
        ok &= check_true(out.find("in_current_min=-20") != std::string::npos, "scan1: shows in_current_min=-20");
        ok &= check_true(out.find("abs_current_max=30") != std::string::npos, "scan1: shows abs_current_max=30");
    }

    // 2) dump -- file written, sane size (the shared synthetic blob is a
    // fixed, deterministic 107 bytes: 4 signature + 16 filler + 24
    // current-limits (4 float32_auto fields + 2 u16 fields + 1
    // float32_auto field) + 22 filler + 23 speed-pid (3 float32_auto
    // fields + 1 u16 kd_filter field + 1 float32_auto field + 1 raw byte
    // + 1 float32_auto field) + 18 filler).
    {
        const std::string dump_path = tmp_dir + "/dump.bin";
        run_mcconf({"dump", dump_path}, "dump");
        std::vector<uint8_t> dumped;
        std::string err;
        std::ifstream f(dump_path, std::ios::binary | std::ios::ate);
        ok &= check_true(f.is_open(), "dump: output file exists");
        if (f.is_open()) {
            const std::streamsize size = f.tellg();
            ok &= check_true(size == 107, "dump: file size is the expected 107 bytes (got " +
                                               std::to_string(static_cast<long>(size)) + ")");
        }
    }

    // 3) patch --ki 0.016 --abs-current-max 60 --kd-filter 0.4. kd_filter
    // is included specifically to exercise patch_kd_filter()'s 2-byte
    // width through the FULL real stack (CLI -> SET_MCCONF -> fake
    // hardware -> GET_MCCONF verify -> re-scan), not just the pure-logic
    // unit test above.
    {
        const std::string out = run_mcconf({"patch", "--ki", "0.016", "--abs-current-max", "60", "--kd-filter",
                                             "0.4", "--backup-dir", backup_dir},
                                            "patch");
        ok &= check_true(out.find("ki: 0.008") != std::string::npos && out.find("0.016") != std::string::npos,
                          "patch: stdout shows ki changing 0.008 -> 0.016");
        ok &= check_true(out.find("kd_filter: 0.2") != std::string::npos && out.find("0.4") != std::string::npos,
                          "patch: stdout shows kd_filter changing 0.2 -> 0.4");
        ok &= check_true(out.find("verified") != std::string::npos, "patch: reports successful verification");
        const std::string backup_file = find_first_file_with_prefix(backup_dir, "mcconf_backup_");
        ok &= check_true(!backup_file.empty(), "patch: a backup file appears under --backup-dir");

        // 4) scan again -- new values visible.
        const std::string scan_out = run_mcconf({"scan"}, "scan2");
        ok &= check_true(scan_out.find("UNAMBIGUOUS") != std::string::npos, "scan2: still unambiguous after patch");
        ok &= check_true(scan_out.find("ki=0.016") != std::string::npos, "scan2: shows the patched ki=0.016");
        ok &= check_true(scan_out.find("abs_current_max=60") != std::string::npos,
                          "scan2: shows the patched abs_current_max=60");
        ok &= check_true(scan_out.find("kd_filter=0.4") != std::string::npos,
                          "scan2: shows the patched kd_filter=0.4 (2-byte field round trip through real "
                          "SET_MCCONF/GET_MCCONF)");
        ok &= check_true(scan_out.find("kp=0.008") != std::string::npos,
                          "scan2: kp is unaffected by the ki/abs_current_max/kd_filter patch");

        // 5) restore from the backup -> values return to the originals.
        if (!backup_file.empty()) {
            run_mcconf({"restore", backup_file}, "restore");
            const std::string scan_out2 = run_mcconf({"scan"}, "scan3");
            ok &= check_true(scan_out2.find("ki=0.008") != std::string::npos,
                              "scan3: ki restored to 0.008 after `restore`");
            ok &= check_true(scan_out2.find("abs_current_max=30") != std::string::npos,
                              "scan3: abs_current_max restored to 30 after `restore`");
            ok &= check_true(scan_out2.find("kd_filter=0.2") != std::string::npos,
                              "scan3: kd_filter restored to 0.2 after `restore`");
        }
    }

    fake_guard.terminate();
    return ok;
}

// ---------------------------------------------------------------------
// (i) pty + ZMQ integration: spawn fake_vesc (pty) AND the real
// vesc_driver binary (--serial-port <that pty>), then drive it exactly
// like mpc_controller / the calibration tool would over its real ZMQ
// surface (raw libzmq, matching this whole folder's C-API-only rule).
// ---------------------------------------------------------------------

namespace zmq_test {

// ASCII-decimal + base64 encoder EXACTLY mirroring mpc_controller's own
// encodeAscii()/publish_ackermann() (MPC/src/main.cpp) -- reproduced here
// (not included from outside VescDriver/) to build wire-exact ackermann
// payloads.
std::string encode_ascii(double value) {
    std::ostringstream oss;
    oss.precision(16);
    oss << value;
    return base64_encode(oss.str());
}

std::string build_ackermann_payload(double speed, double steering, double accel) {
    nlohmann::json j;
    j["speed"] = encode_ascii(speed);
    j["steering"] = encode_ascii(steering);
    j["accel"] = encode_ascii(accel);
    return j.dump();
}

bool send_frame(void* sock, const std::string& data, int flags) {
    zmq_msg_t msg;
    zmq_msg_init_size(&msg, data.size());
    if (!data.empty()) {
        std::memcpy(zmq_msg_data(&msg), data.data(), data.size());
    }
    const int rc = zmq_msg_send(&msg, sock, flags);
    if (rc < 0) {
        zmq_msg_close(&msg);
        return false;
    }
    return true;
}

bool recv_frame(void* sock, std::string* out, int flags) {
    zmq_msg_t msg;
    zmq_msg_init(&msg);
    const int rc = zmq_msg_recv(&msg, sock, flags);
    if (rc < 0) {
        zmq_msg_close(&msg);
        return false;
    }
    out->assign(static_cast<const char*>(zmq_msg_data(&msg)), zmq_msg_size(&msg));
    zmq_msg_close(&msg);
    return true;
}

// Blocking REQ round trip (relies on the caller having set ZMQ_RCVTIMEO on
// `req_sock` so this can't hang forever on a dead peer).
bool req_roundtrip(void* req_sock, const std::string& request_json, std::string* reply_json) {
    if (!send_frame(req_sock, request_json, 0)) return false;
    return recv_frame(req_sock, reply_json, 0);
}

}  // namespace zmq_test

bool test_pty_zmq_vesc_driver_ackermann_watchdog_and_calib() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_driver_exe = exe_dir.empty() ? "vesc_driver" : exe_dir + "/vesc_driver";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_driver_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_driver executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_itg_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
    const std::string driver_config_path = tmp_dir + "/driver_config.json";

    // A config with kick DISABLED (so the accel-ramp phase below produces a
    // clean, uninterrupted rising SET_RPM signal) and generous safety
    // ceilings (the clamps themselves are already covered by the unit
    // tests above, not the point of this integration test).
    {
        std::ofstream f(driver_config_path);
        f << "{"
             "\"mode\":\"erpm\","
             "\"cmd_per_mps\":5000.0,"
             "\"cmd_offset\":0.0,"
             "\"erpm_per_mps\":5000.0,"
             "\"servo\":{\"enabled\":false},"
             "\"safety\":{\"max_erpm\":50000.0,\"max_duty\":1.0,\"max_current\":100.0,"
             "\"safety_max_accel\":3.5,\"safety_max_v\":0.6},"
             "\"watchdog_ms\":250,"
             "\"control_rate_hz\":50,"
             "\"telemetry_rate_hz\":10,"
             "\"kick\":{\"enabled\":false}"
             "}";
    }

    // Test-only ports, distinct from any real deployment range.
    const int ackermann_port = 45911;
    const int control_port = 45912;
    const int telemetry_port = 45913;
    const std::string robot_name = "vesctestbot";

    const std::vector<std::string> fake_args = {"--fw", "5.2", "--log", fake_log_path};
    ProcessGuard fake_guard(spawn_wrapped_capture_stdout(fake_vesc_exe, fake_args, fake_stdout_path), "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed its PTY slave path within 5s");
    if (slave_path.empty()) return ok;

    const std::vector<std::string> driver_args = {
        "--config",          driver_config_path,
        "--robot",           robot_name,
        "--serial-port",     slave_path,
        "--ackermann-port",  std::to_string(ackermann_port),
        "--control-port",    std::to_string(control_port),
        "--telemetry-port",  std::to_string(telemetry_port),
    };
    const std::string driver_stdout_path = tmp_dir + "/vesc_driver_stdout.txt";
    ProcessGuard driver_guard(spawn_wrapped_capture_stdout(vesc_driver_exe, driver_args, driver_stdout_path),
                               "vesc_driver");

    // Give both processes a moment to bind/handshake before we start
    // talking to them.
    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    void* ctx = zmq_ctx_new();
    ok &= check_true(ctx != nullptr, "zmq_ctx_new() for the test's own sockets");
    void* pub = zmq_socket(ctx, ZMQ_PUB);
    void* req = zmq_socket(ctx, ZMQ_REQ);
    const int linger0 = 0;
    zmq_setsockopt(pub, ZMQ_LINGER, &linger0, sizeof(linger0));
    zmq_setsockopt(req, ZMQ_LINGER, &linger0, sizeof(linger0));
    const int rcvtimeo_ms = 2000;
    zmq_setsockopt(req, ZMQ_RCVTIMEO, &rcvtimeo_ms, sizeof(rcvtimeo_ms));

    const std::string pub_endpoint = "tcp://127.0.0.1:" + std::to_string(ackermann_port);
    const std::string req_endpoint = "tcp://127.0.0.1:" + std::to_string(control_port);
    ok &= check_true(zmq_connect(pub, pub_endpoint.c_str()) == 0,
                      "test PUB connects to the driver's ackermann SUB bind");
    ok &= check_true(zmq_connect(req, req_endpoint.c_str()) == 0,
                      "test REQ connects to the driver's control REP bind");

    // PUB/SUB needs a beat for the subscription to propagate before the
    // driver will see anything published.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const std::string ackermann_topic = "/" + robot_name + "/ackermann";

    // ping -- sanity check before any command has ever been sent.
    {
        std::string reply_text;
        ok &= check_true(zmq_test::req_roundtrip(req, "{\"cmd\":\"ping\"}", &reply_text), "ping round-trips");
        if (!reply_text.empty()) {
            const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
            ok &= check_true(!reply.is_discarded() && reply.value("ok", false), "ping reply ok==true");
            ok &= check_true(reply.value("state", std::string()) == "idle", "ping reports idle before any command");
            ok &= check_true(reply.value("robot", std::string()) == robot_name,
                              "ping reports the configured robot name");
        }
    }

    // Publish a constant accel=0.4 ackermann stream at 20Hz for ~1s.
    const auto accel_phase_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - accel_phase_start).count() < 1.0) {
        const std::string payload = zmq_test::build_ackermann_payload(/*speed*/ 0.0, /*steering*/ 0.0, /*accel*/ 0.4);
        zmq_test::send_frame(pub, ackermann_topic, ZMQ_SNDMORE);
        zmq_test::send_frame(pub, payload, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz.
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(150));  // let the last few commands land + log flush.

    const std::string log_after_accel = read_file(fake_log_path);
    std::vector<double> set_rpm_values;
    {
        std::istringstream iss(log_after_accel);
        std::string line;
        while (std::getline(iss, line)) {
            if (line.find("\"cmd\":\"set_rpm\"") == std::string::npos) continue;
            const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
            if (!j.is_discarded() && j.contains("value")) {
                set_rpm_values.push_back(j.at("value").get<double>());
            }
        }
    }
    ok &= check_true(set_rpm_values.size() >= 5,
                      "fake_vesc log recorded several set_rpm commands during the accel phase (got " +
                          std::to_string(set_rpm_values.size()) + ")");
    if (set_rpm_values.size() >= 5) {
        const double first = set_rpm_values.front();
        const double last = set_rpm_values.back();
        std::cout << "    [test] set_rpm first=" << first << " last=" << last << " (n=" << set_rpm_values.size()
                  << ")\n";
        // Expected final v_target ~= 0.4 m/s^2 * ~1.0s ~= 0.4 m/s -> cmd ~=
        // cmd_per_mps(5000)*0.4 = 2000. Generous tolerances throughout, per
        // this test's own brief -- real scheduling jitter is expected.
        ok &= check_true(last > first + 200.0, "set_rpm values clearly rise over the accel phase");
        ok &= check_true(last > 1000.0 && last < 3500.0,
                          "final set_rpm magnitude is in the right ballpark of cmd_per_mps*v_target (got " +
                              std::to_string(last) + ")");
    }

    // Stop publishing -> watchdog should engage and eventually brake.
    // Generous window: watchdog_ms=250 engage delay + a v_target~0.4m/s
    // ramp-down at a FIXED 0.73 m/s^2 takes roughly another 0.5s on top of
    // that (see DriverCore.cpp's kWatchdogRampAccel).
    const auto stop_time = std::chrono::steady_clock::now();
    bool saw_brake_after_stop = false;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - stop_time).count() < 3.0) {
        if (read_file(fake_log_path).find("\"cmd\":\"set_current_brake\"") != std::string::npos) {
            saw_brake_after_stop = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    ok &= check_true(saw_brake_after_stop,
                      "watchdog brake (set_current_brake) appears in the log within 3s of the ackermann "
                      "stream stopping");

    {
        std::string reply_text;
        ok &= check_true(zmq_test::req_roundtrip(req, "{\"cmd\":\"ping\"}", &reply_text),
                          "ping round-trips after the watchdog engages");
        if (!reply_text.empty()) {
            const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
            ok &= check_true(!reply.is_discarded() && reply.value("state", std::string()) == "watchdog_brake",
                              "ping reports watchdog_brake once the ackermann stream has gone stale");
        }
    }

    // set_source -> calib, then a raw duty command -> expect set_duty in
    // the log.
    {
        std::string reply_text;
        ok &= check_true(zmq_test::req_roundtrip(req, "{\"cmd\":\"set_source\",\"source\":\"calib\"}", &reply_text),
                          "set_source round-trips");
        if (!reply_text.empty()) {
            const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
            ok &= check_true(!reply.is_discarded() && reply.value("ok", false), "set_source calib -> ok==true");
        }
    }
    const size_t log_len_before_duty = read_file(fake_log_path).size();
    {
        std::string reply_text;
        ok &= check_true(
            zmq_test::req_roundtrip(req, "{\"cmd\":\"raw\",\"mode\":\"duty\",\"value\":0.05}", &reply_text),
            "raw duty command round-trips");
        if (!reply_text.empty()) {
            const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
            ok &= check_true(!reply.is_discarded() && reply.value("ok", false), "raw duty command -> ok==true");
            ok &= check_true(std::fabs(reply.value("applied_value", -1.0) - 0.05) < 1e-6,
                              "raw duty applied_value echoes the (unclamped, in-range) requested value");
        }
    }
    bool saw_set_duty = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            const std::string log_now = read_file(fake_log_path);
            if (log_now.size() > log_len_before_duty && log_now.find("\"cmd\":\"set_duty\"") != std::string::npos) {
                saw_set_duty = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    ok &= check_true(saw_set_duty,
                      "set_duty appears in the fake_vesc log after switching to calib source + a raw duty command");

    // stop -> a fresh brake burst (log keeps growing with more
    // set_current_brake activity past this point).
    const size_t log_len_before_stop = read_file(fake_log_path).size();
    {
        std::string reply_text;
        ok &= check_true(zmq_test::req_roundtrip(req, "{\"cmd\":\"stop\"}", &reply_text), "stop round-trips");
        if (!reply_text.empty()) {
            const nlohmann::json reply = nlohmann::json::parse(reply_text, nullptr, false);
            ok &= check_true(!reply.is_discarded() && reply.value("ok", false), "stop -> ok==true");
        }
    }
    bool saw_brake_after_explicit_stop = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            const std::string log_now = read_file(fake_log_path);
            if (log_now.size() > log_len_before_stop &&
                log_now.find("\"cmd\":\"set_current_brake\"") != std::string::npos) {
                saw_brake_after_explicit_stop = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    ok &= check_true(saw_brake_after_explicit_stop, "explicit REP \"stop\" produces further brake activity in the log");

    zmq_close(pub);
    zmq_close(req);
    zmq_ctx_destroy(ctx);

    driver_guard.terminate();
    fake_guard.terminate();

    return ok;
}

// ---------------------------------------------------------------------
// (i2) pty + ZMQ integration: vesc_driver's own drive_invert wiring
// (vesc_driver_main.cpp, not DriverCore itself -- DriverCore stays
// wiring-agnostic). Two independent sessions (own fake_vesc + own ZMQ
// ports each, never sharing a serial port): one WITHOUT --drive-invert,
// one WITH it, both switched to calib source and sent the SAME raw
// duty=0.05 command -- the wire's set_duty value must be +0.05 in the
// first session and -0.05 in the second, and each session's own startup
// stdout must print the resolved drive_invert=false/true it actually
// used.
// ---------------------------------------------------------------------

bool test_pty_zmq_vesc_driver_drive_invert() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_driver_exe = exe_dir.empty() ? "vesc_driver" : exe_dir + "/vesc_driver";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_driver_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_driver executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_drive_invert_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string driver_config_path = tmp_dir + "/driver_config.json";
    {
        std::ofstream f(driver_config_path);
        f << "{"
             "\"servo\":{\"enabled\":false},"
             "\"safety\":{\"max_erpm\":50000.0,\"max_duty\":1.0,\"max_current\":100.0,"
             "\"safety_max_accel\":3.5,\"safety_max_v\":0.6},"
             "\"watchdog_ms\":250,"
             "\"control_rate_hz\":50,"
             "\"telemetry_rate_hz\":10,"
             "\"kick\":{\"enabled\":false}"
             "}";
    }

    // Spawns a FRESH fake_vesc + vesc_driver pair, sets source=calib,
    // sends raw duty=0.05, and returns the first logged set_duty value +
    // the driver's own startup stdout.
    auto run_session = [&](bool drive_invert, int port_base, double* out_first_duty, std::string* out_stdout) {
        bool session_ok = true;
        const std::string suffix = drive_invert ? "_inv" : "_noinv";
        const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout" + suffix + ".txt";
        const std::string fake_log_path = tmp_dir + "/fake_vesc_log" + suffix + ".jsonl";
        const std::string driver_stdout_path = tmp_dir + "/vesc_driver_stdout" + suffix + ".txt";

        ProcessGuard fake_guard(
            spawn_wrapped_capture_stdout(fake_vesc_exe, {"--fw", "5.2", "--log", fake_log_path}, fake_stdout_path),
            "fake_vesc" + suffix);
        const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
        session_ok &= check_true(!slave_path.empty(), "fake_vesc (" + suffix + ") printed its PTY slave path within 5s");
        if (slave_path.empty()) return session_ok;

        const int ackermann_port = port_base;
        const int control_port = port_base + 1;
        const int telemetry_port = port_base + 2;
        const std::string robot_name = "vesctestbot" + suffix;

        std::vector<std::string> driver_args = {
            "--config",         driver_config_path,
            "--robot",          robot_name,
            "--serial-port",    slave_path,
            "--ackermann-port", std::to_string(ackermann_port),
            "--control-port",   std::to_string(control_port),
            "--telemetry-port", std::to_string(telemetry_port),
        };
        if (drive_invert) driver_args.push_back("--drive-invert");

        ProcessGuard driver_guard(spawn_wrapped_capture_stdout(vesc_driver_exe, driver_args, driver_stdout_path),
                                   "vesc_driver" + suffix);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));

        void* ctx = zmq_ctx_new();
        void* req = zmq_socket(ctx, ZMQ_REQ);
        const int linger0 = 0;
        zmq_setsockopt(req, ZMQ_LINGER, &linger0, sizeof(linger0));
        const int rcvtimeo_ms = 2000;
        zmq_setsockopt(req, ZMQ_RCVTIMEO, &rcvtimeo_ms, sizeof(rcvtimeo_ms));
        const std::string req_endpoint = "tcp://127.0.0.1:" + std::to_string(control_port);
        session_ok &= check_true(zmq_connect(req, req_endpoint.c_str()) == 0,
                                  "(" + suffix + ") test REQ connects to the driver's control REP bind");

        {
            std::string reply_text;
            session_ok &= check_true(
                zmq_test::req_roundtrip(req, "{\"cmd\":\"set_source\",\"source\":\"calib\"}", &reply_text),
                "(" + suffix + ") set_source round-trips");
        }
        {
            std::string reply_text;
            session_ok &= check_true(
                zmq_test::req_roundtrip(req, "{\"cmd\":\"raw\",\"mode\":\"duty\",\"value\":0.05}", &reply_text),
                "(" + suffix + ") raw duty command round-trips");
        }

        bool saw_set_duty = false;
        double first_value = 0.0;
        {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (std::chrono::steady_clock::now() < deadline && !saw_set_duty) {
                std::istringstream iss(read_file(fake_log_path));
                std::string line;
                while (std::getline(iss, line)) {
                    if (line.find("\"cmd\":\"set_duty\"") == std::string::npos) continue;
                    const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
                    if (j.is_discarded() || !j.contains("value")) continue;
                    first_value = j.at("value").get<double>();
                    saw_set_duty = true;
                    break;
                }
                if (!saw_set_duty) std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
        session_ok &= check_true(saw_set_duty, "(" + suffix + ") set_duty appears in the fake_vesc log");
        *out_first_duty = first_value;
        *out_stdout = read_file(driver_stdout_path);

        zmq_close(req);
        zmq_ctx_destroy(ctx);
        driver_guard.terminate();
        fake_guard.terminate();
        return session_ok;
    };

    double duty_noinv = 0.0;
    double duty_inv = 0.0;
    std::string stdout_noinv;
    std::string stdout_inv;
    ok &= run_session(/*drive_invert=*/false, 45921, &duty_noinv, &stdout_noinv);
    ok &= run_session(/*drive_invert=*/true, 45931, &duty_inv, &stdout_inv);

    ok &= check_true(near_eq(duty_noinv, 0.05, 1e-6),
                      "without --drive-invert, raw duty=0.05 reaches the wire UN-negated (got " +
                          std::to_string(duty_noinv) + ")");
    ok &= check_true(near_eq(duty_inv, -0.05, 1e-6),
                      "with --drive-invert, raw duty=0.05 reaches the wire NEGATED (got " + std::to_string(duty_inv) +
                          ")");

    ok &= check_true(stdout_noinv.find("drive_invert=false") != std::string::npos,
                      "startup output prints the resolved drive_invert=false without the flag");
    ok &= check_true(stdout_inv.find("drive_invert=true") != std::string::npos,
                      "startup output prints the resolved drive_invert=true with --drive-invert");

    return ok;
}

// ---------------------------------------------------------------------
// (k) pty + subprocess integration: spawn fake_vesc (pty), then the real
// `vesc_teleop` binary against it, driving it via a piped stdin exactly
// as an automated harness would (no real terminal) -- scripted key
// sequence -> assert on the resulting SET_DUTY/SET_RPM/SET_CURRENT_BRAKE/
// ALIVE commands in fake_vesc's --log, the startup FW-version banner, and
// a clean 'q'-triggered exit.
// ---------------------------------------------------------------------

bool test_pty_vesc_teleop_cli_e2e() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_teleop_exe = exe_dir.empty() ? "vesc_teleop" : exe_dir + "/vesc_teleop";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_teleop_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_teleop executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_teleop_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
    const std::string teleop_stdout_path = tmp_dir + "/vesc_teleop_stdout.txt";

    ProcessGuard fake_guard(
        spawn_wrapped_capture_stdout(fake_vesc_exe, {"--fw", "6.2", "--log", fake_log_path}, fake_stdout_path),
        "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int stdin_fd = -1;
    const std::vector<std::string> teleop_args = {
        "--serial-port", slave_path, "--rate-hz", "50", "--deadman-ms", "600",
    };
    const pid_t teleop_pid =
        spawn_wrapped_stdin_pipe_capture_stdout(vesc_teleop_exe, teleop_args, teleop_stdout_path, &stdin_fd);
    ProcessGuard teleop_guard(teleop_pid, "vesc_teleop");

    // Give startup (discovery handshake + GET_VALUES snapshot + help
    // text) a moment before the scripted key sequence below.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    auto press = [&](char c) {
        ok &= check_true(write_to_fd(stdin_fd, std::string(1, c)),
                          std::string("write key '") + c + "' to vesc_teleop's stdin");
    };

    // 'w' -- default duty mode -> expect SET_DUTY(+0.04).
    press('w');
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // '+' then 'w' -> value bumps to 0.045.
    press('+');
    press('w');
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // 'E' then 'w' -> switches to erpm mode, forward -> SET_RPM(1000).
    press('E');
    press('w');
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // 's' -> SET_RPM(-1000).
    press('s');
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // No further input for > --deadman-ms(600ms) -- deadman must fire:
    // SET_CURRENT_BRAKE entries, then ALIVE once the brake hold elapses.
    std::this_thread::sleep_for(std::chrono::milliseconds(900));

    press('q');

    // Clean exit within a few seconds (the brake burst itself is ~100ms).
    int status = 0;
    bool exited = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t r = waitpid(teleop_guard.pid(), &status, WNOHANG);
            if (r == teleop_guard.pid()) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    ok &= check_true(exited, "vesc_teleop exits within 5s of 'q'");
    if (exited) {
        ok &= check_true(WIFEXITED(status) && WEXITSTATUS(status) == 0,
                          "vesc_teleop exits with code 0 (clean 'q' shutdown)");
        teleop_guard.mark_reaped();
    }

    close(stdin_fd);

    const std::string teleop_stdout = read_file(teleop_stdout_path);
    ok &= check_true(
        teleop_stdout.find("Firmware:") != std::string::npos && teleop_stdout.find("6.2") != std::string::npos,
        "startup output contains the FW version line (6.2, as configured via fake_vesc --fw)");

    const std::string log = read_file(fake_log_path);
    const size_t pos_duty1 = log.find("\"cmd\":\"set_duty\",\"value\":0.04}");
    ok &= check_true(pos_duty1 != std::string::npos, "log shows SET_DUTY(0.04) after the initial 'w' (default duty mode)");

    const size_t pos_duty2 = log.find("\"cmd\":\"set_duty\",\"value\":0.045}");
    ok &= check_true(pos_duty2 != std::string::npos && (pos_duty1 == std::string::npos || pos_duty2 > pos_duty1),
                      "log shows SET_DUTY(0.045) after '+' then 'w'");

    const size_t pos_rpm_fwd = log.find("\"cmd\":\"set_rpm\",\"value\":1000}");
    ok &= check_true(pos_rpm_fwd != std::string::npos && (pos_duty2 == std::string::npos || pos_rpm_fwd > pos_duty2),
                      "log shows SET_RPM(1000) after 'E' then 'w'");

    const size_t pos_rpm_rev = log.find("\"cmd\":\"set_rpm\",\"value\":-1000}");
    ok &= check_true(pos_rpm_rev != std::string::npos && (pos_rpm_fwd == std::string::npos || pos_rpm_rev > pos_rpm_fwd),
                      "log shows SET_RPM(-1000) after 's'");

    const size_t pos_brake = log.find("\"cmd\":\"set_current_brake\"", pos_rpm_rev == std::string::npos ? 0 : pos_rpm_rev);
    ok &= check_true(pos_brake != std::string::npos, "log shows SET_CURRENT_BRAKE after the deadman window elapses");

    const size_t pos_alive = log.find("\"cmd\":\"alive\"", pos_brake == std::string::npos ? 0 : pos_brake);
    ok &= check_true(pos_alive != std::string::npos,
                      "log shows ALIVE once the deadman's brake-hold elapses (back to idle)");

    fake_guard.terminate();
    return ok;
}

bool test_pty_vesc_teleop_fw2_legacy_e2e() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_teleop_exe = exe_dir.empty() ? "vesc_teleop" : exe_dir + "/vesc_teleop";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_teleop_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_teleop executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_teleop_fw2_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
    const std::string teleop_stdout_path = tmp_dir + "/vesc_teleop_stdout.txt";

    // --fw 2.18 -- fake_vesc's FW_VERSION reply reports 2.18 AND (per
    // fake_vesc.cpp's own dispatch on args.fw_major==2) its GET_VALUES
    // replies switch to the legacy wire layout.
    ProcessGuard fake_guard(
        spawn_wrapped_capture_stdout(fake_vesc_exe, {"--fw", "2.18", "--log", fake_log_path}, fake_stdout_path),
        "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int stdin_fd = -1;
    const std::vector<std::string> teleop_args = {
        "--serial-port", slave_path, "--rate-hz", "50", "--deadman-ms", "600",
    };
    const pid_t teleop_pid =
        spawn_wrapped_stdin_pipe_capture_stdout(vesc_teleop_exe, teleop_args, teleop_stdout_path, &stdin_fd);
    ProcessGuard teleop_guard(teleop_pid, "vesc_teleop");

    // Startup (discovery handshake + GET_VALUES snapshot + help text).
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    auto press = [&](char c) {
        ok &= check_true(write_to_fd(stdin_fd, std::string(1, c)),
                          std::string("write key '") + c + "' to vesc_teleop's stdin");
    };

    // Drive forward briefly (default duty mode) -> expect SET_DUTY(+0.04).
    press('w');
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    // No further input for > --deadman-ms(600ms) -- deadman must still
    // fire on FW 2.x exactly like on modern firmware.
    std::this_thread::sleep_for(std::chrono::milliseconds(900));

    press('q');

    int status = 0;
    bool exited = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t r = waitpid(teleop_guard.pid(), &status, WNOHANG);
            if (r == teleop_guard.pid()) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    ok &= check_true(exited, "vesc_teleop (FW 2.18) exits within 5s of 'q'");
    if (exited) {
        ok &= check_true(WIFEXITED(status) && WEXITSTATUS(status) == 0,
                          "vesc_teleop (FW 2.18) exits with code 0 (clean 'q' shutdown)");
        teleop_guard.mark_reaped();
    }

    close(stdin_fd);

    const std::string teleop_stdout = read_file(teleop_stdout_path);
    ok &= check_true(teleop_stdout.find("Firmware:") != std::string::npos &&
                          teleop_stdout.find("2.18") != std::string::npos,
                      "startup output contains the FW version line (2.18, as configured via fake_vesc --fw)");
    ok &= check_true(teleop_stdout.find("legacy telemetry layout active") != std::string::npos,
                      "startup output notes the legacy (FW 2.x) telemetry layout, not the generic 5.x/6.x warning");
    ok &= check_true(teleop_stdout.find("v_in=12.00V") != std::string::npos,
                      "startup GET_VALUES snapshot shows the fake unit's plausible v_in (12.0V, correctly parsed "
                      "via the legacy layout)");

    const std::string log = read_file(fake_log_path);
    const size_t pos_duty = log.find("\"cmd\":\"set_duty\",\"value\":0.04}");
    ok &= check_true(pos_duty != std::string::npos, "log shows SET_DUTY(0.04) after 'w' (default duty mode)");

    const size_t pos_brake = log.find("\"cmd\":\"set_current_brake\"", pos_duty == std::string::npos ? 0 : pos_duty);
    ok &= check_true(pos_brake != std::string::npos,
                      "log shows SET_CURRENT_BRAKE after the deadman window elapses (deadman still works on FW 2.x)");

    ok &= check_true(log.find("\"cmd\":\"alive\"") == std::string::npos,
                      "NO alive (COMM_ALIVE) entries anywhere in the log -- FW 2.x has no such command, and an "
                      "idle motor needs no keepalive");

    fake_guard.terminate();
    return ok;
}

// ---------------------------------------------------------------------
// (t) pty + subprocess integration: real vesc_teleop's steering path
// end-to-end against fake_vesc --fw 2.18 -- proves (1) SET_SERVO_POS is
// always sent under raw id 11 (the FW2.x-shifted id), NEVER 12 (which
// would alias into COMM_SET_MCCONF on this firmware -- see
// VescProtocol.h's resolve_servo_cmd_id() CAVEAT), (2) the exact a/d/k
// step arithmetic reaches the wire unmodified, and (3) 'T'+'W' persists a
// new calibrated center to a FAKE $HOME (never the real one, never
// VescDriver/config/) that a FRESH vesc_teleop process picks back up on
// its own next startup -- true end-to-end persistence, not just
// SteeringCalib::save()/load() in isolation (see r1/r2/r3 above for
// that).
// ---------------------------------------------------------------------

bool test_pty_vesc_teleop_steering_e2e() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_teleop_exe = exe_dir.empty() ? "vesc_teleop" : exe_dir + "/vesc_teleop";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_teleop_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_teleop executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir =
        "/tmp/vesc_driver_tests_steering_home_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    // A FAKE $HOME, completely separate from the real one -- vesc_teleop's
    // default steering-calib precedence (SteeringCalib.h's
    // resolve_default_load_path()/resolve_default_save_path()) resolves
    // to "$HOME/.vesc/steering_calib.json", so overriding just the
    // child's HOME env var (via spawn_wrapped_stdin_pipe_capture_stdout's
    // home_override, added above) is sufficient to keep this test fully
    // isolated from the real $HOME/.vesc/ and from VescDriver/config/
    // (never touched at all -- no --steering-calib flag is passed below).
    const std::string fake_home = tmp_dir + "/fake_home";
    mkdir(fake_home.c_str(), 0755);

    // RAII cleanup of the whole temp tree (including the fake $HOME) on
    // every return path -- this dir holds nothing but throwaway test
    // fixtures, but per this test's own premise it must never be
    // confused with (or accidentally leak into) anything real.
    struct TmpTreeCleanup {
        std::string dir;
        ~TmpTreeCleanup() {
            if (!dir.empty()) {
                const std::string cmd = "rm -rf -- '" + dir + "'";
                const int rc = std::system(cmd.c_str());
                (void)rc;
            }
        }
    } tmp_cleanup{tmp_dir};

    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
    const std::string teleop1_stdout_path = tmp_dir + "/vesc_teleop1_stdout.txt";
    const std::string teleop2_stdout_path = tmp_dir + "/vesc_teleop2_stdout.txt";

    // --fw 2.18 -- exercises the FW2.x-shifted servo id (11) path, the
    // same hazard-fix case as l3) above, now for steering specifically.
    ProcessGuard fake_guard(
        spawn_wrapped_capture_stdout(fake_vesc_exe, {"--fw", "2.18", "--log", fake_log_path}, fake_stdout_path),
        "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto press = [&](int fd, char c) {
        ok &= check_true(write_to_fd(fd, std::string(1, c)),
                          std::string("write key '") + c + "' to vesc_teleop's stdin");
    };

    // Polls `path`'s contents for `needle` up to `timeout_s` -- same
    // "poll a captured-stdout file for an expected substring" technique
    // wait_for_pty_line() above uses for fake_vesc's own "PTY " line.
    auto wait_for_stdout_substring = [](const std::string& path, const std::string& needle,
                                         double timeout_s) -> bool {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            if (read_file(path).find(needle) != std::string::npos) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    };

    // Waits (polling, timeout) until at least `min_count` "set_servo_pos"
    // JSONL entries are visible in fake_vesc's --log file, then returns
    // ALL of them (value + raw "id") in log order -- reused for both this
    // test's own scripted-key assertions.
    auto poll_servo_log_entries = [&](size_t min_count, double timeout_s) {
        struct Entry {
            double value = 0.0;
            int id = -1;
        };
        std::vector<Entry> entries;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
        while (std::chrono::steady_clock::now() < deadline) {
            entries.clear();
            std::istringstream iss(read_file(fake_log_path));
            std::string line;
            while (std::getline(iss, line)) {
                if (line.empty()) continue;
                const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
                if (j.is_discarded() || !j.is_object()) continue;
                if (j.value("cmd", std::string()) == "set_servo_pos") {
                    Entry e;
                    e.value = j.value("value", 0.0);
                    e.id = j.value("id", -1);
                    entries.push_back(e);
                }
            }
            if (entries.size() >= min_count) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        return entries;
    };

    // ------------------------------------------------------------------
    // Session 1: k, d, d, a -> steering positions 0.5, 0.51, 0.52, 0.51
    // (default center=0.5, --servo-step defaults to 0.01, no
    // --steering-calib/--servo-min/--servo-max overrides). Then T, W to
    // save the last position (0.51) as the new center.
    // ------------------------------------------------------------------
    int stdin_fd = -1;
    const std::vector<std::string> teleop_args = {
        "--serial-port", slave_path, "--rate-hz", "50", "--deadman-ms", "5000",
    };
    const pid_t teleop_pid1 = spawn_wrapped_stdin_pipe_capture_stdout(vesc_teleop_exe, teleop_args,
                                                                       teleop1_stdout_path, &stdin_fd, fake_home);
    ProcessGuard teleop_guard1(teleop_pid1, "vesc_teleop(session1)");

    ok &= check_true(wait_for_stdout_substring(teleop1_stdout_path, "vesc_teleop: steering:", 5.0),
                      "session 1 startup output prints the 'vesc_teleop: steering: load=.../save=...' banner "
                      "within 5s");

    press(stdin_fd, 'k');
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    press(stdin_fd, 'd');
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    press(stdin_fd, 'd');
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    press(stdin_fd, 'a');

    const auto servo_entries = poll_servo_log_entries(4, 5.0);
    ok &= check_true(servo_entries.size() >= 4,
                      "at least 4 set_servo_pos log entries seen (k,d,d,a) within 5s (got " +
                          std::to_string(servo_entries.size()) + ")");
    if (servo_entries.size() >= 4) {
        const double expected_values[4] = {0.5, 0.51, 0.52, 0.51};
        for (size_t i = 0; i < 4; ++i) {
            ok &= check_true(near_eq(servo_entries[i].value, expected_values[i], 0.002),
                              "set_servo_pos[" + std::to_string(i) + "].value ~= " +
                                  std::to_string(expected_values[i]) + " (got " +
                                  std::to_string(servo_entries[i].value) + ")");
        }
        for (size_t i = 0; i < servo_entries.size(); ++i) {
            ok &= check_true(servo_entries[i].id == 11,
                              "set_servo_pos[" + std::to_string(i) +
                                  "] used raw command id 11 (FW2.x-shifted), never 12 (got " +
                                  std::to_string(servo_entries[i].id) + ")");
        }
    }

    // 'T' (enter trim mode) then 'W' (raise pending center-save request
    // for the CURRENT steering position, 0.51).
    press(stdin_fd, 'T');
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    press(stdin_fd, 'W');

    const std::string calib_path = fake_home + "/.vesc/steering_calib.json";
    bool calib_file_appeared = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            struct stat st;
            if (stat(calib_path.c_str(), &st) == 0) {
                calib_file_appeared = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }
    ok &= check_true(calib_file_appeared,
                      "steering_calib.json appears under the FAKE $HOME/.vesc/ within 5s of 'W' (never under the "
                      "real $HOME or VescDriver/config/)");

    vesc::SteeringCalib saved_calib;
    if (calib_file_appeared) {
        std::string load_err;
        const bool loaded = vesc::load(calib_path, &saved_calib, &load_err);
        ok &= check_true(loaded, "the saved calibration file loads back successfully via vesc::load() (well-"
                                  "formed, passes validate_and_clamp()): " + load_err);
        ok &= check_true(near_eq(saved_calib.center, 0.51, 0.002),
                          "saved center matches the last steering position asserted above (0.51), got " +
                              std::to_string(saved_calib.center));
    }

    {
        const std::string teleop1_stdout = read_file(teleop1_stdout_path);
        ok &= check_true(teleop1_stdout.find("STEERING CENTER SAVED") != std::string::npos,
                          "session 1 stdout reports a successful steering-center save ('STEERING CENTER SAVED')");
    }

    // Cleanly end session 1 (same 'q'-then-wait technique as every other
    // pty+subprocess vesc_teleop test in this file).
    press(stdin_fd, 'q');
    {
        int status = 0;
        bool exited = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t r = waitpid(teleop_guard1.pid(), &status, WNOHANG);
            if (r == teleop_guard1.pid()) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        ok &= check_true(exited, "session 1 vesc_teleop exits within 5s of 'q'");
        if (exited) teleop_guard1.mark_reaped();
    }
    close(stdin_fd);

    // ------------------------------------------------------------------
    // Session 2: a FRESH vesc_teleop process, same fake_vesc + SAME fake
    // $HOME -- proves the saved center survives a full process restart,
    // not just SteeringCalib::save()/load() called back-to-back in the
    // same process.
    // ------------------------------------------------------------------
    int stdin_fd2 = -1;
    const pid_t teleop_pid2 = spawn_wrapped_stdin_pipe_capture_stdout(vesc_teleop_exe, teleop_args,
                                                                       teleop2_stdout_path, &stdin_fd2, fake_home);
    ProcessGuard teleop_guard2(teleop_pid2, "vesc_teleop(session2)");

    ok &= check_true(wait_for_stdout_substring(teleop2_stdout_path, "vesc_teleop: steering:", 5.0),
                      "session 2 startup output prints the 'vesc_teleop: steering:' banner within 5s");

    if (calib_file_appeared) {
        char center_field[32];
        std::snprintf(center_field, sizeof(center_field), "center=%.3f", saved_calib.center);
        const std::string teleop2_stdout = read_file(teleop2_stdout_path);
        ok &= check_true(teleop2_stdout.find(center_field) != std::string::npos,
                          std::string("session 2 startup output shows the persisted center ('") + center_field +
                              "') loaded from the FAKE $HOME -- proves persistence across a process restart");
    }

    press(stdin_fd2, 'q');
    {
        int status = 0;
        bool exited = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t r = waitpid(teleop_guard2.pid(), &status, WNOHANG);
            if (r == teleop_guard2.pid()) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        ok &= check_true(exited, "session 2 vesc_teleop exits within 5s of 'q'");
        if (exited) teleop_guard2.mark_reaped();
    }
    close(stdin_fd2);

    fake_guard.terminate();

    // Final safety assertions over the WHOLE log (both sessions): the
    // FW2.x id-11 servo path must never have aliased into COMM_SET_MCCONF
    // (id 12 on FW2.x -- see VescProtocol.h's resolve_servo_cmd_id()
    // CAVEAT), and this test's own premise must not be vacuous.
    const std::string full_log = read_file(fake_log_path);
    ok &= check_true(full_log.find("\"cmd\":\"set_mcconf\"") == std::string::npos,
                      "NO set_mcconf entries anywhere in the log across both sessions -- the FW2.x id-11 servo "
                      "path never aliased into COMM_SET_MCCONF");
    ok &= check_true(full_log.find("\"cmd\":\"set_servo_pos\"") != std::string::npos,
                      "sanity: at least one set_servo_pos entry DOES exist in the log (not vacuously passing)");

    return ok;
}

// ---------------------------------------------------------------------
// (n) pty + subprocess integration: RAMP mode end-to-end against the
// real vesc_teleop binary + fake_vesc -- 'E' then 'w' with --ramp
// --erpm-ramp 2000 should show the fake's SET_RPM log climbing
// gradually to 1000 and holding, then (after 's') descending gradually
// through zero to -1000, then (after enough silence) a deadman brake.
// ---------------------------------------------------------------------

// Extracts, in log order, the "value" field of every JSONL entry in
// `log_contents` whose "cmd" equals `cmd_name` (fake_vesc's --log
// format: one JSON object per line, see tests/fake_vesc.cpp). Malformed/
// non-JSON lines are silently skipped (matches this file's other
// no-throw nlohmann::json::parse() usages elsewhere, e.g. the pty+ZMQ
// integration test's reply parsing).
std::vector<double> extract_log_values(const std::string& log_contents, const std::string& cmd_name) {
    std::vector<double> values;
    std::istringstream iss(log_contents);
    std::string line;
    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        const nlohmann::json j = nlohmann::json::parse(line, nullptr, false);
        if (j.is_discarded() || !j.is_object()) continue;
        if (j.value("cmd", std::string()) == cmd_name) {
            values.push_back(j.value("value", 0.0));
        }
    }
    return values;
}

bool test_pty_vesc_teleop_ramp_e2e() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_teleop_exe = exe_dir.empty() ? "vesc_teleop" : exe_dir + "/vesc_teleop";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_teleop_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_teleop executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_teleop_ramp_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
    const std::string teleop_stdout_path = tmp_dir + "/vesc_teleop_stdout.txt";

    ProcessGuard fake_guard(spawn_wrapped_capture_stdout(fake_vesc_exe, {"--log", fake_log_path}, fake_stdout_path),
                             "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int stdin_fd = -1;
    // deadman-ms=1500 -- comfortably longer than the ~1.0s reversal
    // (2000 erpm/s over a 2000 erpm span) below, so the full ramp-down
    // is observed in the log before the deadman ever gets a chance to
    // cut it short.
    const std::vector<std::string> teleop_args = {
        "--serial-port", slave_path, "--rate-hz", "50", "--deadman-ms", "1500", "--ramp", "--erpm-ramp", "2000",
    };
    const pid_t teleop_pid =
        spawn_wrapped_stdin_pipe_capture_stdout(vesc_teleop_exe, teleop_args, teleop_stdout_path, &stdin_fd);
    ProcessGuard teleop_guard(teleop_pid, "vesc_teleop");

    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // startup.

    auto press = [&](char c) {
        ok &= check_true(write_to_fd(stdin_fd, std::string(1, c)),
                          std::string("write key '") + c + "' to vesc_teleop's stdin");
    };

    // 'E' then 'w' -- erpm mode, drive forward. Ramp target is 1000 (the
    // default erpm magnitude); at 2000erpm/s it takes ~0.5s to reach it.
    // Wait long enough to reach it AND hold there for a bit.
    press('E');
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    press('w');
    std::this_thread::sleep_for(std::chrono::milliseconds(700));

    // 's' -- reverse. From +1000 to -1000 at 2000erpm/s takes ~1.0s. Wait
    // long enough for the full reversal to complete and hold at -1000
    // for a bit, but still comfortably under the 1500ms deadman.
    press('s');
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    // Now go silent past the 1500ms deadman window (~1.2s already
    // elapsed since 's' above) -- wait long enough for the deadman to
    // fire and its brake-hold to elapse.
    std::this_thread::sleep_for(std::chrono::milliseconds(700));

    press('q');

    int status = 0;
    bool exited = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t r = waitpid(teleop_guard.pid(), &status, WNOHANG);
            if (r == teleop_guard.pid()) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    ok &= check_true(exited, "vesc_teleop (ramp) exits within 5s of 'q'");
    if (exited) {
        ok &= check_true(WIFEXITED(status) && WEXITSTATUS(status) == 0, "vesc_teleop (ramp) exits with code 0");
        teleop_guard.mark_reaped();
    }
    close(stdin_fd);

    const std::string log = read_file(fake_log_path);
    const std::vector<double> rpm_values = extract_log_values(log, "set_rpm");
    ok &= check_true(!rpm_values.empty(), "at least one SET_RPM entry logged");

    // --- Ramp-up phase: gradual climb to exactly 1000, then a hold. ---
    size_t up_peak_idx = rpm_values.size();
    for (size_t i = 0; i < rpm_values.size(); ++i) {
        if (rpm_values[i] >= 1000.0 - 1e-6) {
            up_peak_idx = i;
            break;
        }
    }
    ok &= check_true(up_peak_idx < rpm_values.size(), "SET_RPM ramp-up reaches 1000 at some point in the log");
    if (up_peak_idx < rpm_values.size()) {
        bool up_monotonic = true;
        bool saw_intermediate_up = false;
        for (size_t i = 0; i < up_peak_idx; ++i) {
            if (i + 1 <= up_peak_idx && rpm_values[i] > rpm_values[i + 1] + 1e-6) up_monotonic = false;
            if (rpm_values[i] > 1.0 && rpm_values[i] < 999.0) saw_intermediate_up = true;
        }
        ok &= check_true(up_monotonic, "SET_RPM values are non-decreasing during the ramp-up phase");
        ok &= check_true(saw_intermediate_up,
                          "at least one intermediate SET_RPM value strictly between 0 and 1000 during ramp-up "
                          "(proves a gradual ramp, not an instant jump)");
        // Loose sanity bound: target 1000 at 2000erpm/s and --rate-hz 50
        // (20ms/tick) is ~25 ticks in theory; generous slack for
        // scheduler jitter in a test environment while still confirming
        // it's neither near-instant nor absurdly slow.
        ok &= check_true(up_peak_idx >= 5 && up_peak_idx <= 100,
                          "SET_RPM ramp-up takes a plausible number of ticks to reach 1000 (got " +
                              std::to_string(up_peak_idx) + ", expected roughly ~25)");
    }

    // --- Ramp-down phase (after 's'): gradual descent through zero to -1000. ---
    size_t down_start_idx = rpm_values.size();
    for (size_t i = up_peak_idx; i < rpm_values.size(); ++i) {
        if (rpm_values[i] < 1000.0 - 1e-6) {
            down_start_idx = i;
            break;
        }
    }
    ok &= check_true(down_start_idx < rpm_values.size(), "SET_RPM values start descending after 's' (reversal begins)");
    ok &= check_true(down_start_idx > up_peak_idx + 1,
                      "SET_RPM holds at 1000 for at least a couple of ticks before 's' starts the reversal (got a "
                      "hold of " + std::to_string(down_start_idx - up_peak_idx) + " entries)");

    if (down_start_idx < rpm_values.size()) {
        size_t down_trough_idx = rpm_values.size();
        for (size_t i = down_start_idx; i < rpm_values.size(); ++i) {
            if (rpm_values[i] <= -1000.0 + 1e-6) {
                down_trough_idx = i;
                break;
            }
        }
        ok &= check_true(down_trough_idx < rpm_values.size(), "SET_RPM ramp-down reaches -1000 at some point in the log");
        if (down_trough_idx < rpm_values.size()) {
            bool down_monotonic = true;
            bool saw_intermediate_down = false;
            bool saw_zero_crossing = false;
            for (size_t i = down_start_idx; i <= down_trough_idx; ++i) {
                if (i + 1 <= down_trough_idx && rpm_values[i] < rpm_values[i + 1] - 1e-6) down_monotonic = false;
                if (rpm_values[i] > -999.0 && rpm_values[i] < 999.0) saw_intermediate_down = true;
                if (rpm_values[i] > -150.0 && rpm_values[i] < 150.0) saw_zero_crossing = true;
            }
            ok &= check_true(down_monotonic, "SET_RPM values are non-increasing during the ramp-down phase");
            ok &= check_true(saw_intermediate_down,
                              "at least one intermediate SET_RPM value strictly between -1000 and 1000 during ramp-down");
            ok &= check_true(saw_zero_crossing,
                              "SET_RPM values pass near (through) zero during the reversal -- confirms slewing "
                              "THROUGH zero, not resetting to 0 and ramping back up");
        }
    }

    // --- Silence past the deadman -> brake entries (not just the final quit burst). ---
    const size_t pos_after_rpm = log.rfind("\"cmd\":\"set_rpm\"");
    const size_t pos_brake =
        log.find("\"cmd\":\"set_current_brake\"", pos_after_rpm == std::string::npos ? 0 : pos_after_rpm);
    ok &= check_true(pos_brake != std::string::npos,
                      "log shows SET_CURRENT_BRAKE after the last SET_RPM entry -- the deadman still fires "
                      "normally with ramp mode active");

    const std::vector<double> brake_entries = extract_log_values(log, "set_current_brake");
    ok &= check_true(brake_entries.size() >= 10,
                      "at least ~10 SET_CURRENT_BRAKE entries logged (brake_hold_ms's worth from the deadman "
                      "itself, not just the final 5x quit burst) -- got " + std::to_string(brake_entries.size()));

    fake_guard.terminate();
    return ok;
}

// ---------------------------------------------------------------------
// (p) pty + subprocess integration: SPEED GOVERNOR mode end-to-end
// against the real vesc_teleop binary + fake_vesc -- 'V', "2000" digit
// entry, ENTER, 'w' should show the fake's log carrying ONLY SET_DUTY
// (never SET_RPM), with smoothly (slew-respecting) evolving values,
// converging near a reachable target; then silence -> deadman brake.
// ---------------------------------------------------------------------

// Extracts the numeric value following the LAST occurrence of `key`
// (e.g. "meas=") in `text`, up to the next whitespace. Returns false if
// `key` never appears or the following text isn't a valid number.
bool extract_last_labeled_double(const std::string& text, const std::string& key, double* out) {
    const size_t pos = text.rfind(key);
    if (pos == std::string::npos) return false;
    const size_t start = pos + key.size();
    size_t end = start;
    while (end < text.size() && text[end] != ' ' && text[end] != '\n' && text[end] != '\r' && text[end] != '\t') {
        ++end;
    }
    if (end == start) return false;
    char* parse_end = nullptr;
    const double value = std::strtod(text.c_str() + start, &parse_end);
    if (parse_end == text.c_str() + start) return false;
    *out = value;
    return true;
}

bool test_pty_vesc_teleop_speed_governor_e2e() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string fake_vesc_exe = exe_dir.empty() ? "fake_vesc" : exe_dir + "/fake_vesc";
    const std::string vesc_teleop_exe = exe_dir.empty() ? "vesc_teleop" : exe_dir + "/vesc_teleop";
    if (access(fake_vesc_exe.c_str(), X_OK) != 0) {
        return check_true(false, "fake_vesc executable exists next to vesc_driver_tests (build it first)");
    }
    if (access(vesc_teleop_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_teleop executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_teleop_speed_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string fake_stdout_path = tmp_dir + "/fake_vesc_stdout.txt";
    const std::string fake_log_path = tmp_dir + "/fake_vesc_log.jsonl";
    const std::string teleop_stdout_path = tmp_dir + "/vesc_teleop_stdout.txt";

    // --erpm-per-duty=52800 matches the CLI's own default ff_gain(4400) *
    // a typical v_in(12.0, fake_vesc's own default) almost exactly, so
    // the feedforward term alone gets close to the real target and
    // convergence is fast/reliable in this synthetic test (the real PI
    // gains are tuned for real hardware's much lower duty-to-erpm gain,
    // so a badly-mismatched fake plant would make this test slow/
    // marginal for no interesting reason). Target 3000 (not the example
    // 2000 in the task) needs duty ~= 3000/52800 ~= 0.057, comfortably
    // clear of fake_vesc's own default stall_duty=0.03 and max_duty=0.2.
    ProcessGuard fake_guard(
        spawn_wrapped_capture_stdout(fake_vesc_exe, {"--erpm-per-duty", "52800", "--log", fake_log_path},
                                      fake_stdout_path),
        "fake_vesc");
    const std::string slave_path = wait_for_pty_line(fake_stdout_path, 5.0);
    ok &= check_true(!slave_path.empty(), "fake_vesc printed a 'PTY <path>' first line within 5s");
    if (slave_path.empty()) return ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int stdin_fd = -1;
    const std::vector<std::string> teleop_args = {
        "--serial-port", slave_path, "--rate-hz", "50", "--deadman-ms", "3000", "--duty-ramp", "0.5",
    };
    const pid_t teleop_pid =
        spawn_wrapped_stdin_pipe_capture_stdout(vesc_teleop_exe, teleop_args, teleop_stdout_path, &stdin_fd);
    ProcessGuard teleop_guard(teleop_pid, "vesc_teleop");

    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // startup.

    auto press = [&](char c) {
        ok &= check_true(write_to_fd(stdin_fd, std::string(1, c)),
                          std::string("write key '") + c + "' to vesc_teleop's stdin");
    };
    auto press_str = [&](const std::string& s) {
        for (char c : s) press(c);
    };

    press('V');  // speed mode.
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    press_str("3000");
    press('\r');
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    press('w');

    // Let it converge -- slew to duty~0.057 at 0.5/s (~0.11s) + plant
    // settling (fake's default tau_s=0.15s) + PI trim, generously
    // bounded.
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    // Snapshot vesc_teleop's OWN status-line output NOW (mid-run, while
    // still driving) -- its "meas=" field is the filtered erpm the
    // governor itself is using, round-tripped through the real
    // GET_VALUES wire parse, so checking it here validates the whole
    // real pipeline rather than needing a second, racy serial connection
    // onto the same pty. Must be read BEFORE the deadman fires below --
    // that resets the governor's erpm filter, which would otherwise
    // dilute exactly the value being checked.
    const std::string mid_run_stdout = read_file(teleop_stdout_path);
    double last_meas = 0.0;
    const bool got_meas = extract_last_labeled_double(mid_run_stdout, "meas=", &last_meas);
    ok &= check_true(got_meas, "vesc_teleop's status line reports a 'meas=' (filtered erpm) value in speed mode");
    if (got_meas) {
        ok &= check_true(std::fabs(last_meas - 3000.0) <= 500.0,
                          "the measured erpm converges near the 3000 target before the deadman (got " +
                              std::to_string(last_meas) + ")");
    }

    // Now go silent past the 3000ms deadman.
    std::this_thread::sleep_for(std::chrono::milliseconds(3200));

    press('q');

    int status = 0;
    bool exited = false;
    {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const pid_t r = waitpid(teleop_guard.pid(), &status, WNOHANG);
            if (r == teleop_guard.pid()) {
                exited = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
    ok &= check_true(exited, "vesc_teleop (speed governor) exits within 5s of 'q'");
    if (exited) {
        ok &= check_true(WIFEXITED(status) && WEXITSTATUS(status) == 0, "vesc_teleop (speed governor) exits with code 0");
        teleop_guard.mark_reaped();
    }
    close(stdin_fd);

    const std::string log = read_file(fake_log_path);

    ok &= check_true(log.find("\"cmd\":\"set_rpm\"") == std::string::npos,
                      "NO SET_RPM commands anywhere in the log -- speed mode NEVER sends SET_RPM, only SET_DUTY");

    const std::vector<double> duty_values = extract_log_values(log, "set_duty");
    ok &= check_true(!duty_values.empty(), "at least one SET_DUTY entry logged");

    // Slew respected: no single tick's duty jumps by more than
    // --duty-ramp(0.5/s) allows for a --rate-hz(50) tick (20ms ->
    // 0.01/tick nominal), with generous slack for scheduler jitter in a
    // real subprocess.
    bool slew_respected = true;
    for (size_t i = 0; i + 1 < duty_values.size(); ++i) {
        if (std::fabs(duty_values[i + 1] - duty_values[i]) > 0.03) {
            slew_respected = false;
            break;
        }
    }
    ok &= check_true(slew_respected, "SET_DUTY values evolve smoothly, respecting the configured slew limit");

    // Deadman still fires (brake entries beyond just the final quit burst).
    const std::vector<double> brake_entries = extract_log_values(log, "set_current_brake");
    ok &= check_true(brake_entries.size() >= 10,
                      "at least ~10 SET_CURRENT_BRAKE entries logged (the deadman's own brake-hold, not just the "
                      "final 5x quit burst) -- got " + std::to_string(brake_entries.size()));

    fake_guard.terminate();
    return ok;
}

// ---------------------------------------------------------------------
// (q) vesc_teleop --help/-h: must work with ZERO hardware attached --
// no fake_vesc, no --serial-port, no port-discovery attempt at all --
// since it's the only way to review the flag/key reference before ever
// touching a real (or fake) VESC. Deliberately no pty/fake_vesc setup in
// this test at all.
// ---------------------------------------------------------------------

bool test_vesc_teleop_help_no_hardware() {
    bool ok = true;
    using namespace pty_test;

    const std::string exe_dir = own_exe_dir();
    const std::string vesc_teleop_exe = exe_dir.empty() ? "vesc_teleop" : exe_dir + "/vesc_teleop";
    if (access(vesc_teleop_exe.c_str(), X_OK) != 0) {
        return check_true(false, "vesc_teleop executable exists next to vesc_driver_tests (build it first)");
    }

    const std::string tmp_dir = "/tmp/vesc_driver_tests_teleop_help_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);

    // --help (long form): no --serial-port, no fake_vesc running anywhere
    // -- if this fell through to real port discovery it would either
    // hang waiting on candidates or exit 1 with a "could not find a
    // VESC" error, not exit 0 quickly.
    const std::string stdout_path = tmp_dir + "/vesc_teleop_help_stdout.txt";
    const int rc = run_wrapped_capture_stdout(vesc_teleop_exe, {"--help"}, stdout_path, 5.0);
    ok &= check_true(rc == 0,
                      "vesc_teleop --help exits 0 with no serial port available and no fake_vesc running (got rc=" +
                          std::to_string(rc) + ")");

    const std::string out = read_file(stdout_path);
    ok &= check_true(out.find("--speed-kp") != std::string::npos, "--help output mentions --speed-kp");
    ok &= check_true(out.find("--speed-ki") != std::string::npos, "--help output mentions --speed-ki");
    ok &= check_true(out.find("--speed-ff") != std::string::npos, "--help output mentions --speed-ff");
    ok &= check_true(out.find("select speed mode") != std::string::npos,
                      "--help output includes the 'V' key's own key-map entry (speed governor mode)");

    // -h (short form) works the same way.
    const std::string stdout_path_h = tmp_dir + "/vesc_teleop_h_stdout.txt";
    const int rc_h = run_wrapped_capture_stdout(vesc_teleop_exe, {"-h"}, stdout_path_h, 5.0);
    ok &= check_true(rc_h == 0, "vesc_teleop -h (short form) also exits 0 with no hardware");
    const std::string out_h = read_file(stdout_path_h);
    ok &= check_true(out_h.find("--speed-kp") != std::string::npos, "-h output is the same full help (mentions --speed-kp too)");

    // Contrast case: a GENUINE connect attempt (no --help) against a
    // guaranteed-nonexistent port must exit NONZERO with a clear error --
    // --help's exit-0/no-hardware-needed behavior must not have
    // accidentally papered over (or been confused with) real connect
    // failures elsewhere in the flag-parsing/startup path. An explicit
    // --serial-port override (rather than relying on auto-discovery
    // timing) keeps this fast and deterministic -- PortDiscovery.h's own
    // contract has an override-path failure report back within its
    // probe_timeout_ms (400ms default), not hang.
    const std::string stdout_path_fail = tmp_dir + "/vesc_teleop_fail_stdout.txt";
    const int rc_fail = run_wrapped_capture_stdout(
        vesc_teleop_exe, {"--serial-port", "/dev/vesc_driver_tests_nonexistent_port_xyz"}, stdout_path_fail, 5.0);
    // Specifically == 1 (the real process exit code), not just "!= 0" --
    // run_wrapped_capture_stdout() also returns -1 on a 5s timeout and -2
    // on death-by-signal, neither of which should be mistaken for
    // "correctly reported failure" (a hang or crash here would be a real
    // bug, not equivalent to a clean nonzero exit).
    // NOTE: the "could not find/handshake a VESC" message itself goes to
    // stderr (see vesc_teleop_main.cpp), which
    // spawn_wrapped_capture_stdout()/run_wrapped_capture_stdout() don't
    // capture (by design -- shared by many other tests that rely on
    // stdout-only capture) -- so this only checks the exit code, not the
    // message text; that exit code IS the actual thing being verified
    // here (see the check's own message above).
    ok &= check_true(rc_fail == 1,
                      "a genuine no-hardware connect attempt (no --help) exits with code 1, promptly (not a hang "
                      "or crash) -- got rc=" + std::to_string(rc_fail));

    return ok;
}

}  // namespace

int main() {
    // vesc_teleop's pty integration test writes scripted keypress bytes
    // to a child process's stdin pipe; if that child ever exits/crashes
    // early, a subsequent write() would otherwise raise SIGPIPE and kill
    // THIS ENTIRE test binary (losing every other test's results) rather
    // than just failing that one write() call -- ignore it globally so
    // it surfaces as a normal EPIPE return instead.
    signal(SIGPIPE, SIG_IGN);

    const std::vector<std::pair<std::string, bool (*)()>> tests = {
        {"g) pty integration: fork/exec fake_vesc, FW_VERSION + SET_RPM erpm tracking (rise past "
         "stall, decay below stall) + --log JSONL verification",
         test_pty_fake_vesc_fw_version_and_rpm_tracking},
        {"g2) pty + subprocess integration: real vesc_mcconf binary vs. fake_vesc -- scan/dump/patch/"
         "scan/restore/scan",
         test_pty_vesc_mcconf_cli_e2e},
        {"i) pty+ZMQ integration: spawn fake_vesc + the real vesc_driver binary; ackermann accel "
         "tracking -> watchdog brake on stream loss -> REP set_source/raw/stop",
         test_pty_zmq_vesc_driver_ackermann_watchdog_and_calib},
        {"i2) pty+ZMQ integration: vesc_driver's own --drive-invert wiring -- raw duty=0.05 reaches "
         "the wire as +0.05 without the flag and -0.05 with it, and each session's startup stdout "
         "prints the resolved drive_invert value",
         test_pty_zmq_vesc_driver_drive_invert},
        {"k) pty + subprocess integration: real vesc_teleop binary vs. fake_vesc, driven via a piped "
         "stdin -- mode/magnitude/direction commands, deadman brake-then-alive, clean 'q' exit",
         test_pty_vesc_teleop_cli_e2e},
        {"l3) pty + subprocess integration: real vesc_teleop vs. fake_vesc --fw 2.18 -- legacy-layout "
         "startup notice, correct v_in via the legacy parser, SET_DUTY drive + deadman brake still "
         "work, NO alive/COMM_ALIVE entries while idle on FW 2.x",
         test_pty_vesc_teleop_fw2_legacy_e2e},
        {"t) pty + subprocess integration: real vesc_teleop steering vs. fake_vesc --fw 2.18 -- "
         "k/d/d/a reaches the wire as SET_SERVO_POS under raw id 11 (never 12/COMM_SET_MCCONF) with the "
         "exact expected position sequence, and T+W persists a new center to a FAKE $HOME that a "
         "freshly-spawned second vesc_teleop process loads back on its own next startup",
         test_pty_vesc_teleop_steering_e2e},
        {"n) pty + subprocess integration: real vesc_teleop --ramp --erpm-ramp 2000 vs. fake_vesc -- "
         "SET_RPM climbs gradually to 1000 and holds, 's' reverses gradually through zero to -1000, "
         "then silence -> deadman brake",
         test_pty_vesc_teleop_ramp_e2e},
        {"p) pty + subprocess integration: real vesc_teleop speed mode ('V'+\"3000\"+ENTER+'w') vs. "
         "fake_vesc -- ONLY SET_DUTY ever sent (never SET_RPM), values evolve smoothly (slew "
         "respected), measured erpm converges near target, then silence -> deadman brake",
         test_pty_vesc_teleop_speed_governor_e2e},
        {"q) vesc_teleop --help/-h: exits 0 with ZERO hardware attached (no fake_vesc, no "
         "--serial-port) and its output covers the speed-governor 'V' key + --speed-kp/--speed-ki/"
         "--speed-ff flags",
         test_vesc_teleop_help_no_hardware},
    };

    return run_registered_tests(tests);
}
