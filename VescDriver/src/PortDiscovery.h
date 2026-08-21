// PortDiscovery.h
//
// Finds which serial device the VESC is attached to. Two layers,
// deliberately separated:
//   - rank_candidates(): pure ranking logic (name list in, ordered
//     candidate list out) -- no filesystem or hardware access, fully
//     unit-testable without a device.
//   - find_vesc_port(): the impure driver that enumerates /dev/serial/by-id
//     and /dev/ttyACM0..9, ranks them via rank_candidates(), and probes
//     each in order (open + FW_VERSION request + wait for a valid reply)
//     until one responds or the candidate list is exhausted.
//
// Portability: C++14 only, POSIX dirent/unistd/cstdlib (realpath) only --
// see VescDriver/CMakeLists.txt's HARD PORTABILITY RULES.

#ifndef VESC_DRIVER_PORT_DISCOVERY_H_
#define VESC_DRIVER_PORT_DISCOVERY_H_

#include <cstdint>
#include <string>
#include <vector>

namespace vesc {

// One entry as found under /dev/serial/by-id: its own symlink filename
// (what gets scored -- by-id names embed the USB device's manufacturer/
// product strings, e.g. "usb-ChibiOS_ChibiOS_RT_VESC_...") and the real
// device path it resolves to (e.g. "/dev/ttyACM0").
struct ByIdEntry {
    std::string name;
    std::string resolved_path;
};

// A ranked candidate device to try, highest score first.
struct PortCandidate {
    std::string path;
    std::string source;  // e.g. "by-id:usb-ChibiOS_..." or "ttyACM-fallback"
    int score = 0;
};

// Pure ranking logic (no filesystem/hardware access): scores each by-id
// entry by whether its name contains a VESC-ish keyword ("ChibiOS", "VESC",
// "STMicroelectronics", case-insensitive -- checked in that priority
// order), ranks any by-id entry (recognized keyword or not) above the
// plain /dev/ttyACMn fallback list, and preserves each input list's
// relative order as a tie-breaker within its own tier. If a resolved by-id
// path and a fallback path coincide, the path is returned once, at its
// (higher) by-id-derived rank.
std::vector<PortCandidate> rank_candidates(const std::vector<ByIdEntry>& by_id_entries,
                                            const std::vector<std::string>& ttyacm_fallbacks);

struct DiscoveryLogEntry {
    std::string path;
    std::string source;
    bool responded = false;
    std::string note;
};

struct DiscoveryResult {
    bool ok = false;
    std::string path;
    uint8_t fw_major = 0;
    uint8_t fw_minor = 0;
    std::vector<DiscoveryLogEntry> log;  // every candidate actually probed, in probe order
};

// Finds the VESC's serial port.
//  - If `override_path` is non-empty, it is the only candidate tried (no
//    enumeration) -- but it still must answer FW_VERSION within
//    `probe_timeout_ms` for the result to come back ok=true; a failing
//    override is reported as a failure (with a one-entry log) rather than
//    trusted blindly, so callers can tell a bad --port override from a
//    working one.
//  - Else enumerates /dev/serial/by-id/* and /dev/ttyACM0..9, ranks them
//    via rank_candidates(), and probes each in that order (open, send
//    FW_VERSION, wait up to `probe_timeout_ms` for a valid reply via the
//    streaming frame decoder) until the first responder.
// `baud` is the serial rate every probe opens each candidate at (default
// 115200, the VESC UART default) -- callers with a configured non-default
// baud rate (DriverConfig::baud) must pass it here too, or discovery would
// probe at 115200 while the driver later reopens the discovered port at
// the configured rate, which can silently fail to handshake on a VESC set
// to a different baud.
DiscoveryResult find_vesc_port(const std::string& override_path = std::string(),
                                int probe_timeout_ms = 400, int baud = 115200);

}  // namespace vesc

#endif  // VESC_DRIVER_PORT_DISCOVERY_H_
