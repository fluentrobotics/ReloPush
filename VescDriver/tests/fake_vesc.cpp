// fake_vesc.cpp -> binary `fake_vesc`
//
// Hardware-free VESC stand-in for testing: opens a pty master, prints the
// slave's device path on the FIRST stdout line as "PTY <path>" (flushed
// immediately so a parent process can read it before fake_vesc does
// anything else), then speaks the real VESC UART protocol on the pty
// master -- a test can open a vesc::SerialPort on the printed slave path
// and talk to this process exactly as it would talk to a real VESC.
//
// Command-line flags (all optional):
//   --fw MAJOR.MINOR      FW_VERSION reply (default 5.2). MAJOR==2 also
//                          switches GET_VALUES replies to the legacy FW
//                          2.x wire layout (see FakeVescModel::
//                          build_get_values_reply() / VescProtocol.h's
//                          parse_get_values_legacy()) -- lets a test
//                          exercise vesc_teleop's FW-2.x path without
//                          real hardware. GET_VALUES layout AND
//                          SET_SERVO_POS's accepted command id both key
//                          off --fw's major version in a REAL VESC's
//                          eyes, but this fake accepts SET_SERVO_POS
//                          under EITHER raw id 11 or 12 regardless of
//                          --fw (see FakeVescModel::apply_command()) so
//                          tests can drive it flexibly; the JSONL --log
//                          output always records which id was actually
//                          used (see "id" field below), which is how a
//                          caller's own fw-awareness gets verified.
//   --tau-s S             motor model time constant (default 0.15)
//   --erpm-per-duty V     SET_DUTY -> erpm_target scale (default 30000)
//   --stall-duty V        |duty| below this -> erpm_target 0 (default 0.03)
//   --erpm-per-amp V      SET_CURRENT -> erpm_target scale (default 300)
//   --stall-amp V         |amps| below this -> erpm_target 0 (default 1.0)
//   --stall-erpm V        |erpm| below this -> erpm_target 0 (default 800)
//   --log PATH             append one JSON line per received command
//   --duration-s N          self-exit after N seconds (default: run until
//                            SIGTERM/SIGINT)
//
// Portability: C++14 only, POSIX pty (posix_openpt/grantpt/unlockpt/
// ptsname) + poll -- see VescDriver/CMakeLists.txt's HARD PORTABILITY
// RULES. _GNU_SOURCE (rather than the more restrictive _XOPEN_SOURCE) is
// defined below purely to expose those POSIX pty declarations in
// <stdlib.h> on glibc; it does not restrict anything else the standard
// library headers provide.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "FakeVescModel.h"
#include "../src/McconfPatcher.h"
#include "../src/VescProtocol.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;

void handle_stop_signal(int) { g_stop = 1; }

struct Args {
    uint8_t fw_major = 5;
    uint8_t fw_minor = 2;
    vesc_test::FakeVescModelParams model;
    std::string log_path;
    double duration_s = -1.0;  // < 0 means unset (run until signaled)
};

// Accepts both "--flag value" and "--flag=value" forms.
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

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (flag_matches(arg, "--fw") && next_value(argc, argv, &i, &value)) {
            const size_t dot = value.find('.');
            if (dot != std::string::npos) {
                args.fw_major = static_cast<uint8_t>(std::atoi(value.substr(0, dot).c_str()));
                args.fw_minor = static_cast<uint8_t>(std::atoi(value.substr(dot + 1).c_str()));
            }
        } else if (flag_matches(arg, "--tau-s") && next_value(argc, argv, &i, &value)) {
            args.model.tau_s = std::atof(value.c_str());
        } else if (flag_matches(arg, "--erpm-per-duty") && next_value(argc, argv, &i, &value)) {
            args.model.erpm_per_duty = std::atof(value.c_str());
        } else if (flag_matches(arg, "--stall-duty") && next_value(argc, argv, &i, &value)) {
            args.model.stall_duty = std::atof(value.c_str());
        } else if (flag_matches(arg, "--erpm-per-amp") && next_value(argc, argv, &i, &value)) {
            args.model.erpm_per_amp = std::atof(value.c_str());
        } else if (flag_matches(arg, "--stall-amp") && next_value(argc, argv, &i, &value)) {
            args.model.stall_amp = std::atof(value.c_str());
        } else if (flag_matches(arg, "--stall-erpm") && next_value(argc, argv, &i, &value)) {
            args.model.stall_erpm = std::atoi(value.c_str());
        } else if (flag_matches(arg, "--log") && next_value(argc, argv, &i, &value)) {
            args.log_path = value;
        } else if (flag_matches(arg, "--duration-s") && next_value(argc, argv, &i, &value)) {
            args.duration_s = std::atof(value.c_str());
        } else {
            std::fprintf(stderr, "fake_vesc: ignoring unrecognized argument '%s'\n", arg.c_str());
        }
    }
    return args;
}

bool write_all_master(int fd, const std::vector<uint8_t>& data) {
    size_t off = 0;
    while (off < data.size()) {
        const ssize_t n = ::write(fd, data.data() + off, data.size() - off);
        if (n > 0) {
            off += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd;
            pfd.fd = fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            poll(&pfd, 1, 50);
            continue;
        }
        return false;  // hard error (e.g. slave went away) -- drop the reply
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args = parse_args(argc, argv);

    std::signal(SIGTERM, handle_stop_signal);
    std::signal(SIGINT, handle_stop_signal);

    const int master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (master_fd < 0) {
        std::fprintf(stderr, "fake_vesc: posix_openpt() failed: %s\n", std::strerror(errno));
        return 1;
    }
    if (grantpt(master_fd) != 0) {
        std::fprintf(stderr, "fake_vesc: grantpt() failed: %s\n", std::strerror(errno));
        return 1;
    }
    if (unlockpt(master_fd) != 0) {
        std::fprintf(stderr, "fake_vesc: unlockpt() failed: %s\n", std::strerror(errno));
        return 1;
    }
    const char* slave_path = ptsname(master_fd);
    if (!slave_path) {
        std::fprintf(stderr, "fake_vesc: ptsname() failed: %s\n", std::strerror(errno));
        return 1;
    }
    // Copy immediately -- ptsname()'s buffer is not guaranteed to survive
    // any further libc calls.
    const std::string slave_path_copy = slave_path;

    // REQUIRED first line, flushed before anything else happens: a parent
    // process spawning fake_vesc reads this to learn which device to open.
    std::cout << "PTY " << slave_path_copy << std::endl;
    std::cout.flush();

    std::ofstream log_file;
    if (!args.log_path.empty()) {
        log_file.open(args.log_path, std::ios::app);
        if (!log_file.is_open()) {
            std::fprintf(stderr, "fake_vesc: could not open --log path '%s' for append\n",
                         args.log_path.c_str());
        }
    }

    vesc_test::FakeVescModel model(args.model);
    vesc::FrameDecoder decoder;

    // mcconf state: GET_MCCONF/SET_MCCONF handling. Seeded with the SAME
    // shared synthetic blob builder used by McconfPatcher's own unit
    // tests (vesc::build_synthetic_mcconf_blob()) so both exercise
    // identical known offsets/values without duplicating the
    // construction logic -- see McconfPatcher.h. SET_MCCONF simply
    // replaces this blob wholesale; subsequent GETs return whatever was
    // last written, exactly like a real VESC.
    std::vector<uint8_t> mcconf_blob = vesc::build_synthetic_mcconf_blob();

    const auto start_time = std::chrono::steady_clock::now();
    auto last_step = start_time;

    while (!g_stop) {
        if (args.duration_s >= 0.0) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            if (elapsed >= args.duration_s) break;
        }

        struct pollfd pfd;
        pfd.fd = master_fd;
        pfd.events = POLLIN;
        pfd.revents = 0;
        const int pr = poll(&pfd, 1, 10);  // 10ms cadence -- also our model step tick
        if (pr > 0 && (pfd.revents & POLLIN)) {
            uint8_t buf[512];
            const ssize_t n = ::read(master_fd, buf, sizeof(buf));
            if (n > 0) {
                decoder.feed(buf, static_cast<size_t>(n));
            }
            // n<=0 (EAGAIN, EIO while no slave is open yet, transient
            // error, ...) is not fatal -- just keep looping.
        }

        std::vector<uint8_t> payload;
        while (decoder.pop_payload(&payload)) {
            std::string cmd_name;
            double value = 0.0;
            bool has_reply = false;
            std::vector<uint8_t> reply_frame;

            if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::FW_VERSION)) {
                cmd_name = "fw_version";
                reply_frame = vesc::encode_frame(std::vector<uint8_t>{0, args.fw_major, args.fw_minor});
                has_reply = true;
            } else if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::GET_VALUES)) {
                cmd_name = "get_values";
                // --fw 2.x -> emit the legacy GET_VALUES layout (see
                // FakeVescModel::build_get_values_reply()'s own doc
                // comment / VescProtocol.h's parse_get_values_legacy())
                // so the whole vesc_teleop loop is testable against a
                // fake FW-2.x unit without real hardware.
                reply_frame = vesc::encode_frame(model.build_get_values_reply(args.fw_major == 2));
                has_reply = true;
            } else if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::GET_MCCONF)) {
                cmd_name = "get_mcconf";
                value = static_cast<double>(mcconf_blob.size());
                // Framed as command 14 + the full blob -- this routinely
                // exceeds 255 bytes, which is exactly why VescProtocol's
                // kMaxPayloadLen needed raising to fit it through the
                // long-packet (0x03) framing form (encode_frame() picks
                // that form automatically based on payload size).
                std::vector<uint8_t> reply_payload;
                reply_payload.push_back(static_cast<uint8_t>(vesc::CommandId::GET_MCCONF));
                reply_payload.insert(reply_payload.end(), mcconf_blob.begin(), mcconf_blob.end());
                reply_frame = vesc::encode_frame(reply_payload);
                has_reply = true;
            } else if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::SET_MCCONF)) {
                cmd_name = "set_mcconf";
                mcconf_blob.assign(payload.begin() + 1, payload.end());
                value = static_cast<double>(mcconf_blob.size());
                // Firmware replies with a short ack whose payload[0] ==
                // COMM_SET_MCCONF (13) -- vesc_mcconf never trusts this
                // alone (it always re-GETs to verify), but fake_vesc
                // still sends a plausible-looking ack for realism.
                reply_frame = vesc::encode_frame(std::vector<uint8_t>{static_cast<uint8_t>(vesc::CommandId::SET_MCCONF)});
                has_reply = true;
            } else {
                cmd_name = model.apply_command(payload, &value);
            }

            if (!cmd_name.empty() && log_file.is_open()) {
                const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
                log_file << "{\"t\":" << t << ",\"cmd\":\"" << cmd_name << "\"";
                if (cmd_name == "set_servo_pos") {
                    // Record which raw command id was actually received (11
                    // = FW2.x's shifted id, 12 = modern) -- lets a later
                    // test prove a FW2.18-aware caller sent id 11, not 12,
                    // for every servo command in a session. See this file's
                    // --fw doc comment above.
                    log_file << ",\"id\":" << static_cast<int>(payload[0]);
                }
                log_file << ",\"value\":" << value << "}\n";
                log_file.flush();
            }
            if (has_reply) {
                write_all_master(master_fd, reply_frame);
            }
        }

        const auto now = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now - last_step).count();
        last_step = now;
        model.step(dt);
    }

    if (log_file.is_open()) log_file.close();
    ::close(master_fd);
    return 0;
}
