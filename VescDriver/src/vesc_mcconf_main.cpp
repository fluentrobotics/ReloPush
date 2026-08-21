// vesc_mcconf_main.cpp -> binary `vesc_mcconf`
//
// Standalone motor-config (mcconf) tuning CLI: lets speed-PID gains and
// current limits be dumped/inspected/patched over USB from a Jetson,
// without VESC Tool. Talks to the VESC directly (its own SerialPort +
// PortDiscovery + VescProtocol, exactly like vesc_driver_main.cpp) --
// this means the serial port is exclusive: vesc_driver MUST be stopped
// first, or this tool and the driver will fight over the same tty.
//
// Subcommands:
//   vesc_mcconf dump <file>
//   vesc_mcconf scan
//   vesc_mcconf patch [--kp X] [--ki X] [--kd X] [--kd-filter X] [--min-erpm X]
//                      [--ramp X] [--current-max X] [--abs-current-max X]
//   vesc_mcconf restore <file>
//
// Common flags (any subcommand): --serial-port <path> (default:
// auto-discovery, same as vesc_driver), --backup-dir <dir> (default
// $HOME/vesc_mcconf_backups).
//
// Safety model: `patch` ALWAYS writes a timestamped backup of the
// current (pre-mutation) blob before touching anything, and refuses to
// patch any field whose scanner cluster is not unambiguous (exactly one
// match) -- see McconfPatcher.h's file header for why this scans rather
// than trusting a fixed offset. After SET_MCCONF, both `patch` and
// `restore` always re-GET_MCCONF and verify the device's blob is
// byte-identical to what was intended -- the firmware's SET_MCCONF ack
// is never trusted on its own (see VescProtocol.h's is_set_mcconf_ack()
// doc comment).
//
// Portability: C++14 only, POSIX only -- see VescDriver/CMakeLists.txt's
// HARD PORTABILITY RULES.

#include "McconfPatcher.h"
#include "PortDiscovery.h"
#include "SerialPort.h"
#include "VescProtocol.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

// ---------------------------------------------------------------------
// CLI parsing (same "--flag value" / "--flag=value" convention as
// vesc_driver_main.cpp / fake_vesc.cpp).
// ---------------------------------------------------------------------

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

struct OptionalDouble {
    bool has = false;
    double value = 0.0;
};

struct CliOptions {
    std::string subcommand;
    std::string positional;  // dump/restore's <file> argument.
    bool has_serial_port = false;
    std::string serial_port;
    bool has_backup_dir = false;
    std::string backup_dir;

    OptionalDouble kp;
    OptionalDouble ki;
    OptionalDouble kd;
    OptionalDouble kd_filter;
    OptionalDouble min_erpm;
    OptionalDouble ramp;
    OptionalDouble current_max;
    OptionalDouble abs_current_max;
};

void print_usage() {
    std::fprintf(stderr,
                  "Usage:\n"
                  "  vesc_mcconf dump <file> [--serial-port <path>] [--backup-dir <dir>]\n"
                  "  vesc_mcconf scan [--serial-port <path>]\n"
                  "  vesc_mcconf patch [--kp X] [--ki X] [--kd X] [--kd-filter X] [--min-erpm X]\n"
                  "                    [--ramp X] [--current-max X] [--abs-current-max X]\n"
                  "                    [--serial-port <path>] [--backup-dir <dir>]\n"
                  "  vesc_mcconf restore <file> [--serial-port <path>] [--backup-dir <dir>]\n"
                  "\n"
                  "IMPORTANT: stop vesc_driver first -- the serial port is exclusive.\n");
}

bool parse_cli(int argc, char** argv, CliOptions* out) {
    if (argc < 2) return false;
    out->subcommand = argv[1];

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (flag_matches(arg, "--serial-port") && next_value(argc, argv, &i, &value)) {
            out->has_serial_port = true;
            out->serial_port = value;
        } else if (flag_matches(arg, "--backup-dir") && next_value(argc, argv, &i, &value)) {
            out->has_backup_dir = true;
            out->backup_dir = value;
        } else if (flag_matches(arg, "--kp") && next_value(argc, argv, &i, &value)) {
            out->kp.has = true;
            out->kp.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--ki") && next_value(argc, argv, &i, &value)) {
            out->ki.has = true;
            out->ki.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--kd") && next_value(argc, argv, &i, &value)) {
            out->kd.has = true;
            out->kd.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--kd-filter") && next_value(argc, argv, &i, &value)) {
            out->kd_filter.has = true;
            out->kd_filter.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--min-erpm") && next_value(argc, argv, &i, &value)) {
            out->min_erpm.has = true;
            out->min_erpm.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--ramp") && next_value(argc, argv, &i, &value)) {
            out->ramp.has = true;
            out->ramp.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--current-max") && next_value(argc, argv, &i, &value)) {
            out->current_max.has = true;
            out->current_max.value = std::atof(value.c_str());
        } else if (flag_matches(arg, "--abs-current-max") && next_value(argc, argv, &i, &value)) {
            out->abs_current_max.has = true;
            out->abs_current_max.value = std::atof(value.c_str());
        } else if (!arg.empty() && arg[0] != '-') {
            out->positional = arg;
        } else {
            std::fprintf(stderr, "vesc_mcconf: ignoring unrecognized argument '%s'\n", arg.c_str());
        }
    }
    return true;
}

// ---------------------------------------------------------------------
// Backup directory / filesystem helpers (POSIX only -- no
// std::filesystem per this folder's portability rules).
// ---------------------------------------------------------------------

bool path_is_dir(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

// Ensures `dir` exists (mkdir; tolerates already-exists), returns false
// with a message on any other failure.
bool ensure_dir(const std::string& dir, std::string* err) {
    if (path_is_dir(dir)) return true;
    if (::mkdir(dir.c_str(), 0755) == 0) return true;
    if (errno == EEXIST && path_is_dir(dir)) return true;
    *err = "mkdir('" + dir + "') failed: " + std::strerror(errno);
    return false;
}

std::string resolve_backup_dir(const CliOptions& cli) {
    if (cli.has_backup_dir) return cli.backup_dir;
    const char* home = std::getenv("HOME");
    return std::string(home ? home : ".") + "/vesc_mcconf_backups";
}

std::string timestamp_now() {
    const std::time_t t = std::time(nullptr);
    struct tm tm_buf;
    localtime_r(&t, &tm_buf);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
    return std::string(buf);
}

bool write_file_bytes(const std::string& path, const std::vector<uint8_t>& data, std::string* err) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f.is_open()) {
        *err = "could not open '" + path + "' for write";
        return false;
    }
    if (!data.empty()) {
        f.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    }
    if (!f.good()) {
        *err = "write to '" + path + "' failed";
        return false;
    }
    return true;
}

bool read_file_bytes(const std::string& path, std::vector<uint8_t>* out, std::string* err) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        *err = "could not open '" + path + "' for read";
        return false;
    }
    const std::streamsize size = f.tellg();
    f.seekg(0, std::ios::beg);
    out->resize(static_cast<size_t>(size > 0 ? size : 0));
    if (size > 0 && !f.read(reinterpret_cast<char*>(out->data()), size)) {
        *err = "read from '" + path + "' failed";
        return false;
    }
    return true;
}

std::string hex_prefix(const std::vector<uint8_t>& data, size_t n) {
    std::ostringstream oss;
    for (size_t i = 0; i < n && i < data.size(); ++i) {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        oss << buf << (i + 1 < n && i + 1 < data.size() ? " " : "");
    }
    return oss.str();
}

// ---------------------------------------------------------------------
// Serial link helpers.
// ---------------------------------------------------------------------

bool open_vesc(const std::string& serial_port_override, vesc::SerialPort* port, std::string* path_out,
               std::string* err) {
    const vesc::DiscoveryResult disc = vesc::find_vesc_port(serial_port_override, 400, 115200);
    if (!disc.ok) {
        std::ostringstream oss;
        oss << "could not find/handshake a VESC.";
        for (const auto& e : disc.log) {
            oss << "\n  path=" << e.path << " source=" << e.source << " responded=" << (e.responded ? "yes" : "no")
                << " note=" << e.note;
        }
        *err = oss.str();
        return false;
    }
    if (!port->open(disc.path, 115200)) {
        *err = "found a VESC at '" + disc.path + "' during discovery but could not open it for normal use: " +
               port->last_error();
        return false;
    }
    *path_out = disc.path;
    return true;
}

bool wait_for_payload(vesc::SerialPort* port, vesc::FrameDecoder* decoder,
                       const std::function<bool(const std::vector<uint8_t>&)>& pred,
                       std::vector<uint8_t>* out_payload, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<uint8_t> buf;
        if (port->read_available(&buf) > 0) {
            decoder->feed(buf);
        }
        std::vector<uint8_t> payload;
        while (decoder->pop_payload(&payload)) {
            if (pred(payload)) {
                *out_payload = payload;
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

bool get_mcconf(vesc::SerialPort* port, vesc::FrameDecoder* decoder, std::vector<uint8_t>* out_blob,
                 std::string* err) {
    if (!port->write_all(vesc::encode_frame(vesc::build_get_mcconf()))) {
        *err = "write(GET_MCCONF) failed: " + port->last_error();
        return false;
    }
    std::vector<uint8_t> payload;
    const bool got = wait_for_payload(
        port, decoder,
        [](const std::vector<uint8_t>& p) { return !p.empty() && p[0] == static_cast<uint8_t>(vesc::CommandId::GET_MCCONF); },
        &payload, 3000);
    if (!got) {
        *err = "no GET_MCCONF reply within 3s (is the VESC connected? is vesc_driver stopped?)";
        return false;
    }
    const vesc::McconfReply reply = vesc::parse_mcconf_reply(payload, vesc::CommandId::GET_MCCONF);
    if (!reply.ok) {
        *err = "malformed GET_MCCONF reply";
        return false;
    }
    *out_blob = reply.blob;
    return true;
}

// Sends SET_MCCONF. Best-effort waits for the firmware's ack (purely
// informational -- printed, never trusted); the caller MUST always
// re-GET_MCCONF and verify regardless of whether an ack was seen.
void set_mcconf(vesc::SerialPort* port, vesc::FrameDecoder* decoder, const std::vector<uint8_t>& blob) {
    port->write_all(vesc::encode_frame(vesc::build_set_mcconf(blob)));
    std::vector<uint8_t> payload;
    const bool saw_ack =
        wait_for_payload(port, decoder, [](const std::vector<uint8_t>& p) { return vesc::is_set_mcconf_ack(p); },
                          &payload, 1500);
    std::printf("  SET_MCCONF ack seen: %s (not trusted on its own -- re-verifying via GET_MCCONF)\n",
                saw_ack ? "yes" : "no");
}

// ---------------------------------------------------------------------
// scan report
// ---------------------------------------------------------------------

void print_speed_pid_matches(const std::vector<vesc::SpeedPidMatch>& matches) {
    std::printf("  speed-PID cluster: %s (%zu match%s)\n",
                vesc::is_unambiguous(matches) ? "UNAMBIGUOUS" : "AMBIGUOUS", matches.size(),
                matches.size() == 1 ? "" : "es");
    for (const auto& m : matches) {
        std::printf("    offset=%zu kp=%.6g ki=%.6g kd=%.6g kd_filter=%.6g min_erpm=%.6g allow_braking=%u "
                    "ramp_erpms_s=%.6g\n",
                    m.offset, m.kp, m.ki, m.kd, m.kd_filter, m.min_erpm, static_cast<unsigned>(m.allow_braking),
                    m.ramp_erpms_s);
    }
}

void print_current_limits_matches(const std::vector<vesc::CurrentLimitsMatch>& matches) {
    std::printf("  current-limits cluster: %s (%zu match%s)\n",
                vesc::is_unambiguous(matches) ? "UNAMBIGUOUS" : "AMBIGUOUS", matches.size(),
                matches.size() == 1 ? "" : "es");
    for (const auto& m : matches) {
        std::printf("    offset=%zu current_max=%.6g current_min=%.6g in_current_max=%.6g in_current_min=%.6g "
                    "abs_current_max=%.6g\n",
                    m.offset, m.current_max, m.current_min, m.in_current_max, m.in_current_min, m.abs_current_max);
    }
}

// ---------------------------------------------------------------------
// Subcommands
// ---------------------------------------------------------------------

int cmd_dump(vesc::SerialPort* port, vesc::FrameDecoder* decoder, const std::string& file) {
    std::vector<uint8_t> blob;
    std::string err;
    if (!get_mcconf(port, decoder, &blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: dump failed: %s\n", err.c_str());
        return 1;
    }
    if (!write_file_bytes(file, blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: dump failed: %s\n", err.c_str());
        return 1;
    }
    std::printf("dumped %zu bytes to '%s'\n", blob.size(), file.c_str());
    std::printf("first 32 bytes: %s\n", hex_prefix(blob, 32).c_str());
    return 0;
}

int cmd_scan(vesc::SerialPort* port, vesc::FrameDecoder* decoder) {
    std::vector<uint8_t> blob;
    std::string err;
    if (!get_mcconf(port, decoder, &blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: scan failed: %s\n", err.c_str());
        return 1;
    }
    std::printf("mcconf blob: %zu bytes\n", blob.size());
    const std::vector<vesc::SpeedPidMatch> pid_matches = vesc::scan_speed_pid(blob);
    const std::vector<vesc::CurrentLimitsMatch> current_matches = vesc::scan_current_limits(blob);
    print_speed_pid_matches(pid_matches);
    print_current_limits_matches(current_matches);
    return 0;
}

int cmd_patch(vesc::SerialPort* port, vesc::FrameDecoder* decoder, const CliOptions& cli,
              const std::string& backup_dir) {
    const bool wants_pid =
        cli.kp.has || cli.ki.has || cli.kd.has || cli.kd_filter.has || cli.min_erpm.has || cli.ramp.has;
    const bool wants_current = cli.current_max.has || cli.abs_current_max.has;
    if (!wants_pid && !wants_current) {
        std::fprintf(stderr, "vesc_mcconf: patch requires at least one --kp/--ki/--kd/--kd-filter/--min-erpm/"
                              "--ramp/--current-max/--abs-current-max flag\n");
        return 1;
    }

    std::vector<uint8_t> blob;
    std::string err;
    if (!get_mcconf(port, decoder, &blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: patch failed: %s\n", err.c_str());
        return 1;
    }

    // Backup ALWAYS written first, before any mutation -- even if the
    // scan below turns out ambiguous and the patch is aborted.
    if (!ensure_dir(backup_dir, &err)) {
        std::fprintf(stderr, "vesc_mcconf: patch failed: %s\n", err.c_str());
        return 1;
    }
    const std::string backup_path = backup_dir + "/mcconf_backup_" + timestamp_now() + ".bin";
    if (!write_file_bytes(backup_path, blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: patch failed: %s\n", err.c_str());
        return 1;
    }
    std::printf("backup written: %s (%zu bytes)\n", backup_path.c_str(), blob.size());

    const std::vector<vesc::SpeedPidMatch> pid_matches = vesc::scan_speed_pid(blob);
    const std::vector<vesc::CurrentLimitsMatch> current_matches = vesc::scan_current_limits(blob);

    const bool pid_ok = !wants_pid || vesc::is_unambiguous(pid_matches);
    const bool current_ok = !wants_current || vesc::is_unambiguous(current_matches);
    if (!pid_ok || !current_ok) {
        std::fprintf(stderr, "vesc_mcconf: patch ABORTED -- scan for a requested field's cluster is not "
                              "unambiguous. Nothing was written to the device. Full scan report:\n");
        print_speed_pid_matches(pid_matches);
        print_current_limits_matches(current_matches);
        return 1;
    }

    std::vector<uint8_t> patched = blob;
    std::printf("patching:\n");
    if (wants_pid) {
        const vesc::SpeedPidMatch& m = pid_matches.front();
        if (cli.kp.has) {
            std::printf("  kp: %.6g -> %.6g\n", m.kp, cli.kp.value);
            patched = vesc::patch(patched, m.offset + 0, static_cast<float>(cli.kp.value));
        }
        if (cli.ki.has) {
            std::printf("  ki: %.6g -> %.6g\n", m.ki, cli.ki.value);
            patched = vesc::patch(patched, m.offset + 4, static_cast<float>(cli.ki.value));
        }
        if (cli.kd.has) {
            std::printf("  kd: %.6g -> %.6g\n", m.kd, cli.kd.value);
            patched = vesc::patch(patched, m.offset + 8, static_cast<float>(cli.kd.value));
        }
        if (cli.kd_filter.has) {
            // kd_filter is the ONE field in this cluster that is NOT
            // float32_auto -- it's a 2-byte fixed-point field (see
            // McconfPatcher.h) -- so it uses patch_kd_filter(), not
            // patch().
            std::printf("  kd_filter: %.6g -> %.6g\n", m.kd_filter, cli.kd_filter.value);
            patched = vesc::patch_kd_filter(patched, m.offset + 12, cli.kd_filter.value);
        }
        if (cli.min_erpm.has) {
            std::printf("  min_erpm: %.6g -> %.6g\n", m.min_erpm, cli.min_erpm.value);
            patched = vesc::patch(patched, m.offset + 14, static_cast<float>(cli.min_erpm.value));
        }
        if (cli.ramp.has) {
            std::printf("  ramp_erpms_s: %.6g -> %.6g\n", m.ramp_erpms_s, cli.ramp.value);
            patched = vesc::patch(patched, m.offset + 19, static_cast<float>(cli.ramp.value));
        }
    }
    if (wants_current) {
        const vesc::CurrentLimitsMatch& m = current_matches.front();
        if (cli.current_max.has) {
            std::printf("  current_max: %.6g -> %.6g\n", m.current_max, cli.current_max.value);
            patched = vesc::patch(patched, m.offset + 0, static_cast<float>(cli.current_max.value));
        }
        if (cli.abs_current_max.has) {
            std::printf("  abs_current_max: %.6g -> %.6g\n", m.abs_current_max, cli.abs_current_max.value);
            patched = vesc::patch(patched, m.offset + 20, static_cast<float>(cli.abs_current_max.value));
        }
    }

    set_mcconf(port, decoder, patched);

    std::vector<uint8_t> verify_blob;
    if (!get_mcconf(port, decoder, &verify_blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: !!! WARNING !!! could not re-read mcconf after SET_MCCONF to verify: "
                              "%s -- device state UNKNOWN, backup is at '%s'\n",
                      err.c_str(), backup_path.c_str());
        return 1;
    }
    if (verify_blob != patched) {
        std::fprintf(stderr, "vesc_mcconf: !!! WARNING !!! re-read mcconf does NOT match the intended patched "
                              "blob byte-for-byte -- the write may have failed or been partial. Backup is at "
                              "'%s'. Consider running `vesc_mcconf restore %s`.\n",
                      backup_path.c_str(), backup_path.c_str());
        return 1;
    }
    std::printf("verified: re-read mcconf blob matches the intended patch byte-for-byte.\n");
    return 0;
}

int cmd_restore(vesc::SerialPort* port, vesc::FrameDecoder* decoder, const std::string& file) {
    std::vector<uint8_t> blob;
    std::string err;
    if (!read_file_bytes(file, &blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: restore failed: %s\n", err.c_str());
        return 1;
    }
    std::printf("restoring %zu bytes from '%s'\n", blob.size(), file.c_str());

    set_mcconf(port, decoder, blob);

    std::vector<uint8_t> verify_blob;
    if (!get_mcconf(port, decoder, &verify_blob, &err)) {
        std::fprintf(stderr, "vesc_mcconf: !!! WARNING !!! could not re-read mcconf after SET_MCCONF to verify: "
                              "%s -- device state UNKNOWN\n",
                      err.c_str());
        return 1;
    }
    if (verify_blob != blob) {
        std::fprintf(stderr, "vesc_mcconf: !!! WARNING !!! re-read mcconf does NOT match the restored file "
                              "byte-for-byte -- the write may have failed or been partial.\n");
        return 1;
    }
    std::printf("verified: re-read mcconf blob matches the restored file byte-for-byte.\n");
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    CliOptions cli;
    if (!parse_cli(argc, argv, &cli)) {
        print_usage();
        return 1;
    }

    if (cli.subcommand != "dump" && cli.subcommand != "scan" && cli.subcommand != "patch" &&
        cli.subcommand != "restore") {
        std::fprintf(stderr, "vesc_mcconf: unknown subcommand '%s'\n", cli.subcommand.c_str());
        print_usage();
        return 1;
    }
    if ((cli.subcommand == "dump" || cli.subcommand == "restore") && cli.positional.empty()) {
        std::fprintf(stderr, "vesc_mcconf: '%s' requires a <file> argument\n", cli.subcommand.c_str());
        print_usage();
        return 1;
    }

    std::fprintf(stderr, "vesc_mcconf: NOTE -- vesc_driver must be stopped first; this tool needs exclusive "
                          "access to the serial port.\n");

    vesc::SerialPort port;
    std::string serial_path;
    std::string err;
    if (!open_vesc(cli.has_serial_port ? cli.serial_port : std::string(), &port, &serial_path, &err)) {
        std::fprintf(stderr, "vesc_mcconf: could not open the VESC: %s\n", err.c_str());
        return 1;
    }
    std::printf("connected: %s\n", serial_path.c_str());

    vesc::FrameDecoder decoder;
    const std::string backup_dir = resolve_backup_dir(cli);

    int rc = 1;
    if (cli.subcommand == "dump") {
        rc = cmd_dump(&port, &decoder, cli.positional);
    } else if (cli.subcommand == "scan") {
        rc = cmd_scan(&port, &decoder);
    } else if (cli.subcommand == "patch") {
        rc = cmd_patch(&port, &decoder, cli, backup_dir);
    } else if (cli.subcommand == "restore") {
        rc = cmd_restore(&port, &decoder, cli.positional);
    }
    return rc;
}
