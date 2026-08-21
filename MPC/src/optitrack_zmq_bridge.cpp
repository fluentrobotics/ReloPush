// optitrack_zmq_bridge: OptiTrack/NatNet receiver that feeds real robot
// localization to mpc_controller, replacing mpc_robot_sim's simulated
// localization PUB for real hardware. Presents exactly the localization side
// of the ZMQ endpoints mpc_controller already talks to (see
// MPC/src/robot_sim.cpp's header comment): BINDS one PUB per
// robot at --loc-port-start+i publishing "/<robot>/localization" with
// payload {"x":...,"y":...,"yaw":...} (plain JSON, no base64, no "t" field
// -- byte-identical to mpc::encode_localization_payload(), which this file
// links directly from SimCore.cpp rather than re-deriving the format).
//
// v1 scope (RECEIVE-TEST capability): parse a live NatNet stream (real
// Motive, or the fake_motive test tool -- see src/fake_motive.cpp), print/
// log it, and publish. All NatNet wire parsing/building and up-axis-aware
// geometry live in mpc:: (OptiTrackCore.h/.cpp) so they are unit-testable
// without sockets, mirroring how SimCore.h/.cpp keep robot_sim.cpp thin.
//
// JUDGMENT CALL (documented, see this task's report): real NatNet clients
// sometimes use a second, separately-bound "command" socket for robustness.
// This implementation uses exactly ONE UDP socket per run: in --mode
// unicast it binds --local-data-port; in --mode multicast it binds
// --data-port and joins --multicast-group. That SAME socket is used both to
// send the periodic keepalive NAT_PING (to --server-ip:--command-port,
// NOW SENT IN BOTH MODES -- see round 2's version-discovery-in-multicast
// requirement) and to receive NAT_PINGRESPONSE/NAT_FRAMEOFDATA (a ping
// reply naturally arrives back at the socket it was sent from; in multicast
// mode, frames ALSO keep arriving via the joined group on this same
// socket). --data-port is therefore only actually BOUND in multicast mode;
// --command-port/--local-data-port only meaningfully used in unicast mode
// (though --command-port is always the ping's destination port now).
//
// Round 2 additions: --mocap-config <path> loads server_ip/mode/
// command_port/data_port/multicast_group from a Motive-exported
// "key:value" streaming-settings text file (mpc::parse_mocap_config_text());
// explicit CLI flags always override it. A NatNet parser-version
// auto-fallback (mpc::VersionAutoDetector) tries a fixed candidate list
// against a frame that fails to parse whenever no ping response has arrived
// yet, adopting whichever candidate parses cleanly 5 times in a row.
//
// Round 3 additions (both fixing real bugs found on a live run against
// real Motive -- see this task's report for the full live-run evidence):
//   - Automatic multicast interface selection (mpc::choose_multicast_interface):
//     on a dual-homed machine, --multicast-interface's old INADDR_ANY
//     default let the kernel join via the DEFAULT-ROUTE interface, which
//     may not be the one Motive is reachable on at all (confirmed live:
//     zero frames via the default route, works via the Motive-subnet
//     interface). The bridge now gathers real interfaces via getifaddrs()
//     and auto-picks the one whose subnet contains --server-ip;
//     --multicast-interface remains available as an explicit override.
//   - Version confirmation gating (mpc::VersionGate): a wrong-version parse
//     does not always fail cleanly -- parsing real NatNet 2.10 bytes with
//     the 3.1 layout was observed live to spuriously "succeed" with garbage
//     values (frame 1, twice reproduced) before the 1Hz ping ever got a
//     chance to correct it. Data frames are now NOT parsed at all until the
//     version is confirmed (ping response, or a --version-defer-timeout-s
//     timeout falling back to the configured version) -- see VersionGate's
//     own doc comment for the full mechanism.

// Round 7: default config paths. optitrack_zmq_bridge is normally run with
// --mocap-config/--map-config pointing at MPC/config/
// mocap_config.txt and mocap_map_config.json; MOCAP_DEFAULT_CONFIG_DIR
// (a compile definition set by CMakeLists.txt on this target, from
// CMAKE_CURRENT_SOURCE_DIR/config -- i.e. MPC/config) lets a
// zero-arg run derive and load those same two files automatically, so
// `optitrack_zmq_bridge` with NO arguments at all works fully against the
// real project config out of the box. Falls back to an empty string (never
// resolves to an existing file, so behaves exactly like today's no-flag
// case) if the macro is somehow undefined, e.g. a manual compile outside
// this project's CMake build.
#ifndef MOCAP_DEFAULT_CONFIG_DIR
#define MOCAP_DEFAULT_CONFIG_DIR ""
#endif

#include "mpc/OptiTrackCore.h"
#include "mpc/SimCore.h"

#include <nlohmann/json.hpp>

#include <zmq.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csignal>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

volatile std::sig_atomic_t g_shutdown_requested = 0;
void handle_shutdown_signal(int) { g_shutdown_requested = 1; }

// "at most once per min_interval_s" gate for stderr warnings (see
// robot_sim.cpp's identical, independently-duplicated RateLimiter -- kept
// self-contained per file per this codebase's own established precedent).
class RateLimiter {
   public:
    bool allow(std::chrono::steady_clock::time_point now, double min_interval_s) {
        if (!fired_ || std::chrono::duration<double>(now - last_fire_).count() >= min_interval_s) {
            last_fire_ = now;
            fired_ = true;
            return true;
        }
        return false;
    }

   private:
    bool fired_ = false;
    std::chrono::steady_clock::time_point last_fire_{};
};

std::vector<std::string> split_comma(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(tok);
    }
    return out;
}

mpc::NatNetVersion parse_natnet_version(const std::string& s, mpc::NatNetVersion fallback) {
    auto pos = s.find('.');
    if (pos == std::string::npos) return fallback;
    try {
        size_t consumed_maj = 0, consumed_min = 0;
        int maj = std::stoi(s.substr(0, pos), &consumed_maj);
        int min = std::stoi(s.substr(pos + 1), &consumed_min);
        return mpc::NatNetVersion{maj, min};
    } catch (const std::exception&) {
        return fallback;
    }
}

struct Options {
    std::string server_ip = "192.168.1.100";  // PLACEHOLDER -- see main()'s startup warning.
    bool server_ip_explicit = false;          // true iff --server-ip was actually passed on the CLI.
    std::string mode = "unicast";
    bool mode_explicit = false;
    std::string multicast_group = "239.255.42.99";
    bool multicast_group_explicit = false;
    int command_port = 1510;
    bool command_port_explicit = false;
    int data_port = 1511;        // multicast-mode bind/join port only -- see header comment.
    bool data_port_explicit = false;
    int local_data_port = 1511;  // unicast-mode bind port only -- see header comment; not
                                  // config-file-driven (Motive's own export has no such field).
    // Local interface for the multicast IP_ADD_MEMBERSHIP join. Empty by
    // default -- main() auto-resolves it (see resolve_multicast_interface())
    // by matching --server-ip's subnet against this machine's real
    // interfaces (mpc::choose_multicast_interface()), falling back to
    // "0.0.0.0" (INADDR_ANY) if none match. Setting this explicitly on the
    // CLI always overrides auto-detection.
    std::string multicast_interface;
    bool multicast_interface_explicit = false;
    // How long to DEFER parsing data frames while the NatNet version is not
    // yet confirmed (see mpc::VersionGate's doc comment for why deferring,
    // not just detecting, is necessary). Real NAT_PINGRESPONSE typically
    // arrives in well under 1s; 3.0s is a generous margin before falling
    // back to --natnet-version.
    double version_defer_timeout_s = 3.0;
    std::string natnet_version_str = "3.1";
    std::string robots_str = "robot1";
    // Round 5: whether --robots was ACTUALLY passed on the CLI (its
    // compiled default above is "robot1", indistinguishable from an
    // explicit `--robots robot1` without this flag). --robots absent
    // entirely selects auto-discovery mode -- see print_usage().
    bool robots_explicit = false;
    std::string rigid_body_ids_str;    // empty => default 1..N (or name-resolved, see below).
    std::string rigid_body_names_str;  // round 4: mutually exclusive with rigid_body_ids_str.
    int loc_port_start = 3260;
    double publish_rate_hz = 30.0;
    std::string map_config_path;
    std::string mocap_config_path;
    bool print_frames = false;
    std::string log_csv_path;
    double duration_s = 0.0;
    bool zmq_disable = false;
};

// Which of {compiled default, --mocap-config file, explicit CLI flag} an
// Options field's effective value actually came from -- logged at startup
// per field (round 2 requirement) so a user can tell at a glance whether
// their config file was actually picked up.
enum class SettingSource { kDefault, kConfig, kCli };
std::string source_str(SettingSource s) {
    switch (s) {
        case SettingSource::kCli: return "cli";
        case SettingSource::kConfig: return "config";
        default: return "default";
    }
}

Options parse_args(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (arg == "--server-ip") {
            o.server_ip = next();
            o.server_ip_explicit = true;
        } else if (arg == "--mode") {
            o.mode = next();
            o.mode_explicit = true;
        } else if (arg == "--multicast-group") {
            o.multicast_group = next();
            o.multicast_group_explicit = true;
        } else if (arg == "--command-port") {
            o.command_port = std::stoi(next());
            o.command_port_explicit = true;
        } else if (arg == "--data-port") {
            o.data_port = std::stoi(next());
            o.data_port_explicit = true;
        } else if (arg == "--local-data-port") {
            o.local_data_port = std::stoi(next());
        } else if (arg == "--multicast-interface") {
            o.multicast_interface = next();
            o.multicast_interface_explicit = true;
        } else if (arg == "--version-defer-timeout-s") {
            o.version_defer_timeout_s = std::stod(next());
        } else if (arg == "--mocap-config") {
            o.mocap_config_path = next();
        } else if (arg == "--natnet-version") {
            o.natnet_version_str = next();
        } else if (arg == "--robots") {
            o.robots_str = next();
            o.robots_explicit = true;
        } else if (arg == "--rigid-body-ids") {
            o.rigid_body_ids_str = next();
        } else if (arg == "--rigid-body-names") {
            o.rigid_body_names_str = next();
        } else if (arg == "--loc-port-start") {
            o.loc_port_start = std::stoi(next());
        } else if (arg == "--publish-rate") {
            o.publish_rate_hz = std::stod(next());
        } else if (arg == "--map-config") {
            o.map_config_path = next();
        } else if (arg == "--print-frames") {
            o.print_frames = true;
        } else if (arg == "--log-csv") {
            o.log_csv_path = next();
        } else if (arg == "--duration-s") {
            o.duration_s = std::stod(next());
        } else if (arg == "--zmq-disable") {
            o.zmq_disable = true;
        } else {
            std::cerr << "[BRIDGE] Warning: unrecognized argument '" << arg << "'" << std::endl;
        }
    }
    return o;
}

// Round 7: this run's compiled-in default config paths -- the two real
// project config files (see MOCAP_DEFAULT_CONFIG_DIR's doc comment above).
std::string default_mocap_config_path() {
    return std::string(MOCAP_DEFAULT_CONFIG_DIR) + "/mocap_config.txt";
}
std::string default_map_config_path() {
    return std::string(MOCAP_DEFAULT_CONFIG_DIR) + "/mocap_map_config.json";
}

// Plain file-existence probe -- the only I/O mpc::resolve_config_path()
// needs (it takes the result as a bool input and stays pure/testable
// itself, mirroring gather_ipv4_interfaces()/choose_multicast_interface()'s
// I/O-vs-pure-decision split above).
bool path_exists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [options]\n\n"
           "OptiTrack/NatNet -> ZMQ localization bridge.\n\n"
           "Connection:\n"
           "  --server-ip <ip>            Motive server IP (default 192.168.1.100, PLACEHOLDER)\n"
           "  --mode unicast|multicast    (default unicast)\n"
           "  --multicast-group <ip>      multicast group to join (default 239.255.42.99)\n"
           "  --command-port <port>       Motive command port (default 1510)\n"
           "  --data-port <port>          multicast join port (default 1511)\n"
           "  --local-data-port <port>    unicast local bind port (default 1511)\n"
           "  --multicast-interface <ip>  local interface for the multicast group join (default:\n"
           "                              auto-detected by matching --server-ip's subnet against\n"
           "                              this machine's network interfaces via getifaddrs();\n"
           "                              falls back to 0.0.0.0/INADDR_ANY if none match. Only\n"
           "                              needed as an explicit override if auto-detection picks\n"
           "                              the wrong interface, e.g. on a LOOPBACK test setup)\n"
           "  --mocap-config <path>       load server_ip/mode/command_port/data_port/\n"
           "                              multicast_group from a Motive-exported\n"
           "                              streaming-settings text file (key:value lines).\n"
           "                              Any of the flags above, if ALSO given, override the\n"
           "                              corresponding value from this file. If OMITTED, defaults\n"
           "                              to (compiled-in) "
        << default_mocap_config_path()
        << " if that\n"
           "                              file exists; if it does not, falls back to today's\n"
           "                              no-flag behavior (built-in placeholder defaults below).\n"
           "  --natnet-version <maj.min>  fallback parser version (default 3.1) -- used only\n"
           "                              after --version-defer-timeout-s expires with no ping\n"
           "                              response yet; the existing 5-consecutive-frame\n"
           "                              structural auto-detector remains available afterward\n"
           "  --version-defer-timeout-s <s>  how long to defer parsing DATA frames while the\n"
           "                              NatNet version is unconfirmed (default 3.0) -- protects\n"
           "                              against a wrong-version parse spuriously \"succeeding\"\n"
           "                              with garbage values instead of failing cleanly\n"
           "Robots:\n"
           "  --robots <a,b,...>          robot names, in port order. If OMITTED ENTIRELY (the\n"
           "                              default, zero-arg way to run this), the bridge enters\n"
           "                              AUTO-DISCOVERY mode instead: it publishes EVERY rigid body\n"
           "                              Motive's model definition reports, one ZMQ topic per body,\n"
           "                              all on the single --loc-port-start port -- no robot list,\n"
           "                              ids, or names needed at all. Published name = the alias\n"
           "                              from --map-config's \"aliases\" map if one matches that\n"
           "                              body's Motive name, else the Motive name sanitized to be\n"
           "                              topic-safe (spaces/specials -> '_'). Nothing publishes\n"
           "                              until the first NAT_MODELDEF reply arrives; bodies added\n"
           "                              to Motive later join automatically. --rigid-body-ids/\n"
           "                              --rigid-body-names both REQUIRE --robots (they have\n"
           "                              nothing to pair with otherwise) -- passing either without\n"
           "                              --robots is an error, not auto-discovery-with-a-filter.\n"
           "  --rigid-body-ids <ids>      streaming ids, parallel to --robots (mutually exclusive\n"
           "                              with --rigid-body-names)\n"
           "  --rigid-body-names <names>  Motive asset names, parallel to --robots, resolved to ids\n"
           "                              via NAT_MODELDEF (mutually exclusive with\n"
           "                              --rigid-body-ids); a robot stays unpublished until its\n"
           "                              name is found in Motive's inventory\n"
           "                              If --robots is given but NEITHER id flag is: \"auto-index\"\n"
           "                              mode -- publishing starts immediately using ids 1..N, then\n"
           "                              upgrades any robot to a name-match if Motive turns out to\n"
           "                              have an asset literally named the same as that robot\n"
           "  --loc-port-start <port>     first robot's localization PUB port (default 3260) --\n"
           "                              also THE single port used by auto-discovery mode\n"
           "  --map-config <path>         JSON {x0,y0,theta0,y_up,yaw_offset,aliases,\n"
           "                              mocap_to_world_matrix} (default identity/empty). If\n"
           "                              OMITTED, defaults to (compiled-in) "
        << default_map_config_path()
        << "\n"
           "                              if that file exists; if it does not, falls back to\n"
           "                              today's no-flag behavior (identity transform/empty).\n"
           "                              \"aliases\": {\"<motive_name>\": \"<published_name>\", ...}\n"
           "                              only applies in auto-discovery mode; yaw_offset keys on\n"
           "                              the PUBLISHED name in both modes\n"
           "                              \"mocap_to_world_matrix\": [[a,b,tx],[c,d,ty],[0,0,1]], a\n"
           "                              full 2D homogeneous affine transform -- SUPERSEDES\n"
           "                              x0/y0/theta0 entirely if both are present (warns loudly).\n"
           "                              Position: [x_w,y_w]=[[a,b],[c,d]]*[x_m,y_m]+[tx,ty].\n"
           "                              Heading: yaw_w=wrap(atan2(c,a)+yaw_m) (+yaw_offset\n"
           "                              separately, same as the legacy path). Bottom row must be\n"
           "                              [0,0,1] (hard error otherwise); the rotation block should\n"
           "                              be orthonormal with det=+1 -- a scale/shear matrix still\n"
           "                              works (full affine applied to position) but WARNS, since\n"
           "                              the yaw formula is then only an approximation\n"
           "Output:\n"
           "  --publish-rate <hz>         downsampled publish rate (default 30)\n"
           "  --print-frames              dump every frame's rigid bodies (id/x/y/z/yaw/tracking)\n"
           "                              plus 1/s receive-rate stats -- doubles as rigid-body-ID\n"
           "                              discovery for --rigid-body-ids/--rigid-body-names on a\n"
           "                              first live run (also see the one-time \"Motive assets:\"\n"
           "                              inventory line, printed in every mode once a model\n"
           "                              definition reply is received)\n"
           "  --log-csv <path>            CSV log, full 6-DoF: t_arrival,robot,raw_x,raw_y,raw_z,\n"
           "                              qx,qy,qz,qw,roll,pitch,heading,planar_x,planar_y,\n"
           "                              planar_yaw,tracking_valid (heading==published yaw;\n"
           "                              roll/pitch are the other two rotations -- see\n"
           "                              mpc::quat_to_roll_pitch_heading()'s doc comment for the\n"
           "                              exact sequence). The ZMQ payload itself stays planar-only\n"
           "                              {\"x\",\"y\",\"yaw\"} regardless -- this is diagnostics only\n"
           "  --zmq-disable               receive/print/log only, no ZMQ publish (RECEIVE-TEST)\n"
           "  --duration-s <secs>         stop after N seconds (default 0 = forever)\n"
           "  --help, -h                  print this message and exit\n";
}

struct MapConfig {
    double x0 = 0.0, y0 = 0.0, theta0 = 0.0;
    bool y_up = false;
    std::unordered_map<std::string, double> yaw_offset;  // keyed on the PUBLISHED name -- see
                                                            // print_usage() and the round-5 header note.
    // Round 5: Motive asset name -> published name, used ONLY in
    // auto-discovery mode (see mpc::resolve_auto_discovery()).
    std::unordered_map<std::string, std::string> aliases;
    // Round 6: optional "mocap_to_world_matrix" -- when present, SUPERSEDES
    // x0/y0/theta0 entirely (see the precedence warning in load_map_config()
    // below) -- see apply_world_transform()'s doc comment for how the two
    // paths are selected at use.
    bool has_mocap_to_world_matrix = false;
    mpc::AffineTransform2D mocap_to_world_matrix;
};

MapConfig load_map_config(const std::string& path) {
    MapConfig cfg;
    if (path.empty()) return cfg;
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[BRIDGE] Warning: could not open --map-config '" << path
                   << "', using identity transform / defaults" << std::endl;
        return cfg;
    }
    try {
        nlohmann::json j;
        f >> j;
        if (j.contains("x0")) cfg.x0 = j.at("x0").get<double>();
        if (j.contains("y0")) cfg.y0 = j.at("y0").get<double>();
        if (j.contains("theta0")) cfg.theta0 = j.at("theta0").get<double>();
        if (j.contains("y_up")) cfg.y_up = j.at("y_up").get<bool>();
        if (j.contains("yaw_offset") && j.at("yaw_offset").is_object()) {
            for (auto it = j.at("yaw_offset").begin(); it != j.at("yaw_offset").end(); ++it) {
                cfg.yaw_offset[it.key()] = it.value().get<double>();
            }
        }
        if (j.contains("aliases") && j.at("aliases").is_object()) {
            for (auto it = j.at("aliases").begin(); it != j.at("aliases").end(); ++it) {
                cfg.aliases[it.key()] = it.value().get<std::string>();
            }
        }
        if (j.contains("mocap_to_world_matrix")) {
            // Reduce the JSON array-of-arrays to a plain nested double
            // vector -- OptiTrackCore's parser has no JSON dependency, so
            // this reduction (and any JSON-shape exception) stays here.
            std::vector<std::vector<double>> rows;
            for (const auto& row_j : j.at("mocap_to_world_matrix")) {
                std::vector<double> row;
                for (const auto& val_j : row_j) {
                    row.push_back(val_j.get<double>());
                }
                rows.push_back(std::move(row));
            }
            mpc::MocapToWorldParseResult parsed = mpc::parse_mocap_to_world_matrix(rows);
            if (parsed.ok) {
                cfg.mocap_to_world_matrix = parsed.transform;
                cfg.has_mocap_to_world_matrix = true;
                if (parsed.non_rigid_warning) {
                    std::cerr << "[BRIDGE] Warning: " << parsed.warning << std::endl;
                }
            } else {
                std::cerr << "[BRIDGE] Warning: invalid mocap_to_world_matrix ('" << parsed.error
                           << "'); falling back to x0/y0/theta0" << std::endl;
            }
        }
    } catch (const std::exception& ex) {
        std::cerr << "[BRIDGE] Warning: failed to parse --map-config ('" << ex.what()
                   << "'); unparsed fields keep their identity defaults" << std::endl;
    }

    // Round 6 precedence rule: mocap_to_world_matrix, if present and valid,
    // ALWAYS wins over x0/y0/theta0 -- warn loudly if both were given (with
    // x0/y0/theta0 not all zero, i.e. actually meaningful) so a user who
    // forgot to remove the old fields isn't silently surprised.
    if (cfg.has_mocap_to_world_matrix &&
        (cfg.x0 != 0.0 || cfg.y0 != 0.0 || cfg.theta0 != 0.0)) {
        std::cerr << "[BRIDGE] Warning: both mocap_to_world_matrix and nonzero x0/y0/theta0 are "
                      "present in --map-config; x0/y0/theta0 are IGNORED, using the matrix"
                   << std::endl;
    }
    return cfg;
}

// Round 6: single call site for "raw mocap planar pose -> world planar
// pose", so both explicit mode and auto-discovery mode's per-frame loops
// (see the receive loop below) apply the SAME precedence rule instead of
// each needing to know about mocap_to_world_matrix separately: the matrix
// wins whenever present (see load_map_config()'s precedence warning),
// otherwise the legacy x0/y0/theta0 rigid transform applies exactly as
// before round 6 (byte-for-byte unchanged code path for any --map-config
// that never sets mocap_to_world_matrix).
mpc::PlanarPose apply_world_transform(const mpc::PlanarPose& raw, const MapConfig& map_cfg) {
    if (map_cfg.has_mocap_to_world_matrix) {
        return mpc::apply_mocap_to_world(raw, map_cfg.mocap_to_world_matrix);
    }
    return mpc::apply_planar_transform(raw, map_cfg.x0, map_cfg.y0, map_cfg.theta0);
}

struct RobotConfig {
    std::string name;
    std::int32_t rigid_body_id = 0;
    double yaw_offset = 0.0;
};

bool make_sockaddr(const std::string& ip, std::uint16_t port, sockaddr_in& out) {
    std::memset(&out, 0, sizeof(out));
    out.sin_family = AF_INET;
    out.sin_port = htons(port);
    return inet_pton(AF_INET, ip.c_str(), &out.sin_addr) == 1;
}

// Gathers this machine's real IPv4 interfaces (name/addr/netmask) via
// getifaddrs() -- the only I/O in the round-3 multicast-interface-selection
// feature; the actual subnet-matching logic (mpc::choose_multicast_interface)
// is a pure function over this data, unit-tested with synthetic lists.
std::vector<mpc::IfaceInfo> gather_ipv4_interfaces() {
    std::vector<mpc::IfaceInfo> out;
    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) != 0) {
        std::cerr << "[BRIDGE] Warning: getifaddrs() failed: " << std::strerror(errno) << std::endl;
        return out;
    }
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET) continue;
        if (ifa->ifa_netmask == nullptr) continue;
        char addr_buf[INET_ADDRSTRLEN] = {0};
        char mask_buf[INET_ADDRSTRLEN] = {0};
        const auto* sin_addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        const auto* sin_mask = reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask);
        if (inet_ntop(AF_INET, &sin_addr->sin_addr, addr_buf, sizeof(addr_buf)) == nullptr) continue;
        if (inet_ntop(AF_INET, &sin_mask->sin_addr, mask_buf, sizeof(mask_buf)) == nullptr) continue;
        mpc::IfaceInfo info;
        info.name = (ifa->ifa_name != nullptr) ? ifa->ifa_name : "";
        info.addr = addr_buf;
        info.netmask = mask_buf;
        out.push_back(std::move(info));
    }
    freeifaddrs(ifaddr);
    return out;
}

struct IfaceChoice {
    std::string address = "0.0.0.0";
    std::string description = "default: INADDR_ANY (let kernel choose)";
};

// Precedence: --multicast-interface (cli) > auto-match > INADDR_ANY. Only
// meaningful/called in --mode multicast.
IfaceChoice resolve_multicast_interface(const Options& opt) {
    if (opt.multicast_interface_explicit) {
        return {opt.multicast_interface, "cli override"};
    }
    std::vector<mpc::IfaceInfo> ifaces = gather_ipv4_interfaces();
    std::optional<mpc::IfaceInfo> chosen = mpc::choose_multicast_interface(opt.server_ip, ifaces);
    if (chosen.has_value()) {
        return {chosen->addr, chosen->name + ", auto: subnet matches server"};
    }
    return {"0.0.0.0", "default: INADDR_ANY, no interface's subnet matched --server-ip " + opt.server_ip};
}

// Opens the one UDP socket this run needs (see header comment for why only
// one). Returns -1 on failure (already logged).
//
// LOOPBACK MULTICAST NOTE (empirically confirmed on this dev machine,
// independent of this codebase, via a minimal Python send/recv probe before
// writing this): IP_ADD_MEMBERSHIP with imr_interface=INADDR_ANY silently
// joins on the wrong interface when the loopback device itself is not
// flagged MULTICAST-capable (`ip link show lo` shows no MULTICAST flag on
// this machine) -- multicast sent with IP_MULTICAST_IF=127.0.0.1 (see
// fake_motive.cpp's --multicast-target) then never reaches a receiver
// joined via INADDR_ANY, even with IP_MULTICAST_LOOP=1 on the sender.
// Explicitly joining via imr_interface=127.0.0.1 (i.e. --multicast-interface
// 127.0.0.1) fixes it. INADDR_ANY remains the correct FALLBACK default for
// real hardware with no subnet match, so it is never hardcoded.
//
// DUAL-HOMED-MACHINE NOTE (round 3, also confirmed live against real
// Motive): even on real (non-loopback) hardware, plain INADDR_ANY is not
// safe on a multi-homed machine -- the kernel resolves it via the DEFAULT
// ROUTE, which may be a totally different network than the one Motive is
// on (observed live: default-route interface -> zero frames; the
// Motive-subnet interface -> works). main() now calls
// resolve_multicast_interface() BEFORE this function runs, auto-picking the
// interface whose subnet contains --server-ip (mpc::choose_multicast_interface())
// and writing the result into opt.multicast_interface -- this function
// itself is unchanged, it just joins via whatever ends up in that field.
int open_client_socket(const Options& opt) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[BRIDGE] Failed to create UDP socket: " << std::strerror(errno) << std::endl;
        return -1;
    }

    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    const bool unicast = (opt.mode == "unicast");
    const std::uint16_t bind_port =
        static_cast<std::uint16_t>(unicast ? opt.local_data_port : opt.data_port);

    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = htons(bind_port);
    if (bind(fd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
        std::cerr << "[BRIDGE] Failed to bind UDP socket to port " << bind_port << ": "
                   << std::strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    if (!unicast) {
        ip_mreq mreq{};
        if (inet_pton(AF_INET, opt.multicast_group.c_str(), &mreq.imr_multiaddr) != 1) {
            std::cerr << "[BRIDGE] Invalid --multicast-group '" << opt.multicast_group << "'"
                       << std::endl;
            close(fd);
            return -1;
        }
        if (inet_pton(AF_INET, opt.multicast_interface.c_str(), &mreq.imr_interface) != 1) {
            std::cerr << "[BRIDGE] Invalid --multicast-interface '" << opt.multicast_interface << "'"
                       << std::endl;
            close(fd);
            return -1;
        }
        if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) != 0) {
            std::cerr << "[BRIDGE] Failed to join multicast group " << opt.multicast_group << ": "
                       << std::strerror(errno) << std::endl;
            close(fd);
            return -1;
        }
    }

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return fd;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }

    Options opt = parse_args(argc, argv);

    // Round 7: per-config default-path resolution -- BEFORE round 2's
    // --mocap-config loading below, since a defaulted path needs to flow
    // through that exact same loading code (a corrupt default file must
    // fail the same way a corrupt explicitly-given file does -- see
    // mpc::resolve_config_path()'s doc comment). Independent per config
    // flag: --mocap-config and --map-config each get their own
    // resolve_config_path() call and never influence each other.
    const std::string default_mocap_path = default_mocap_config_path();
    const std::string default_map_path = default_map_config_path();

    mpc::ConfigPathResolution mocap_path_resolution =
        mpc::resolve_config_path(opt.mocap_config_path, default_mocap_path, path_exists(default_mocap_path));
    if (mocap_path_resolution.source == mpc::ConfigPathSource::kDefault) {
        std::cout << "[BRIDGE] using default mocap-config: " << mocap_path_resolution.path << std::endl;
        opt.mocap_config_path = mocap_path_resolution.path;
    } else if (mocap_path_resolution.source == mpc::ConfigPathSource::kNone) {
        std::cout << "[BRIDGE] no --mocap-config given and default '" << default_mocap_path
                   << "' not found; using built-in defaults" << std::endl;
    }
    // kCli: opt.mocap_config_path already holds the explicit value -- unchanged.

    mpc::ConfigPathResolution map_path_resolution =
        mpc::resolve_config_path(opt.map_config_path, default_map_path, path_exists(default_map_path));
    if (map_path_resolution.source == mpc::ConfigPathSource::kDefault) {
        std::cout << "[BRIDGE] using default map-config: " << map_path_resolution.path << std::endl;
        opt.map_config_path = map_path_resolution.path;
    } else if (map_path_resolution.source == mpc::ConfigPathSource::kNone) {
        std::cout << "[BRIDGE] no --map-config given and default '" << default_map_path
                   << "' not found; using identity transform / defaults" << std::endl;
    }
    // kCli: opt.map_config_path already holds the explicit value -- unchanged.

    // Round 2: resolve --mocap-config (if given) against whatever the CLI
    // already set, per field, with explicit CLI flags always winning -- see
    // SettingSource's doc comment. Must happen before the PLACEHOLDER check
    // below, since a config-supplied IP should suppress that warning.
    SettingSource server_ip_source =
        opt.server_ip_explicit ? SettingSource::kCli : SettingSource::kDefault;
    SettingSource mode_source = opt.mode_explicit ? SettingSource::kCli : SettingSource::kDefault;
    SettingSource command_port_source =
        opt.command_port_explicit ? SettingSource::kCli : SettingSource::kDefault;
    SettingSource data_port_source =
        opt.data_port_explicit ? SettingSource::kCli : SettingSource::kDefault;
    SettingSource multicast_group_source =
        opt.multicast_group_explicit ? SettingSource::kCli : SettingSource::kDefault;

    if (!opt.mocap_config_path.empty()) {
        std::ifstream mocap_file(opt.mocap_config_path);
        if (!mocap_file.is_open()) {
            std::cerr << "[BRIDGE] Warning: could not open --mocap-config '" << opt.mocap_config_path
                       << "', ignoring" << std::endl;
        } else {
            std::stringstream buf;
            buf << mocap_file.rdbuf();
            mpc::MocapConfigResult mocap = mpc::parse_mocap_config_text(buf.str());
            for (const mpc::MocapConfigDiagnostic& d : mocap.diagnostics) {
                if (d.is_warning) {
                    std::cerr << "[BRIDGE] Warning: --mocap-config line " << d.line_number << ": "
                               << d.message << std::endl;
                }
            }
            if (!opt.server_ip_explicit && mocap.has_server_ip) {
                opt.server_ip = mocap.server_ip;
                server_ip_source = SettingSource::kConfig;
            }
            if (!opt.mode_explicit && mocap.has_mode) {
                opt.mode = mocap.mode;
                mode_source = SettingSource::kConfig;
            }
            if (!opt.command_port_explicit && mocap.has_command_port) {
                opt.command_port = mocap.command_port;
                command_port_source = SettingSource::kConfig;
            }
            if (!opt.data_port_explicit && mocap.has_data_port) {
                opt.data_port = mocap.data_port;
                data_port_source = SettingSource::kConfig;
            }
            if (!opt.multicast_group_explicit && mocap.has_multicast_group) {
                opt.multicast_group = mocap.multicast_group;
                multicast_group_source = SettingSource::kConfig;
            }
            std::cout << "[BRIDGE] Loaded --mocap-config '" << opt.mocap_config_path << "'"
                       << std::endl;
        }
    }

    std::cout << "[BRIDGE] Effective settings: server_ip=" << opt.server_ip << " ("
               << source_str(server_ip_source) << ") mode=" << opt.mode << " ("
               << source_str(mode_source) << ") command_port=" << opt.command_port << " ("
               << source_str(command_port_source) << ") data_port=" << opt.data_port << " ("
               << source_str(data_port_source) << ") multicast_group=" << opt.multicast_group << " ("
               << source_str(multicast_group_source) << ")" << std::endl;

    if (opt.server_ip == "192.168.1.100" && server_ip_source != SettingSource::kConfig) {
        std::cerr << "[BRIDGE] ################################################################\n"
                     "[BRIDGE] # WARNING: --server-ip is still the PLACEHOLDER 192.168.1.100     #\n"
                     "[BRIDGE] # Point it at your real Motive machine's IP before live testing.   #\n"
                     "[BRIDGE] ################################################################"
                  << std::endl;
    }
    if (opt.mode != "unicast" && opt.mode != "multicast") {
        std::cerr << "[BRIDGE] --mode must be 'unicast' or 'multicast', got '" << opt.mode << "'"
                   << std::endl;
        return 1;
    }

    // Round 5: top-level operating mode, decided BEFORE any --robots
    // parsing. --robots itself ABSENT (never passed on the CLI, regardless
    // of its compiled default "robot1") selects NEW auto-discovery mode:
    // every rigid body Motive's modeldef reports gets published, one topic
    // per body, with no robot list to configure at all. --robots PRESENT
    // (with or without --rigid-body-ids/--rigid-body-names) is the
    // EXISTING explicit-mode machinery from rounds 1-4, entirely
    // unchanged -- see print_usage()'s "Robots:" section.
    enum class OperatingMode { kExplicit, kAutoDiscovery };
    const OperatingMode operating_mode =
        opt.robots_explicit ? OperatingMode::kExplicit : OperatingMode::kAutoDiscovery;

    if (operating_mode == OperatingMode::kAutoDiscovery &&
        (!opt.rigid_body_ids_str.empty() || !opt.rigid_body_names_str.empty())) {
        std::cerr << "[BRIDGE] --rigid-body-ids/--rigid-body-names require --robots (nothing to pair "
                      "them with in auto-discovery mode)"
                   << std::endl;
        return 1;
    }

    MapConfig map_cfg = load_map_config(opt.map_config_path);

    std::vector<std::string> robot_names;
    std::vector<RobotConfig> robots;
    // Round 4: three id-resolution modes -- see print_usage()'s "Robots:"
    // section and mpc::resolve_robot_ids()'s doc comment. Meaningless (left
    // at defaults) in auto-discovery mode.
    enum class IdMode { kExplicitIds, kExplicitNames, kAuto };
    IdMode id_mode = IdMode::kAuto;
    std::vector<std::string> explicit_names;  // kExplicitNames only; empty => kAuto's own branch in
                                                // mpc::resolve_robot_ids().
    std::unordered_map<std::int32_t, size_t> id_to_index;
    // Round 4: name/auto-mode resolution state, only actually used (and
    // only ever changes id_to_index) when id_mode != kExplicitIds -- see
    // apply_resolution()'s doc comment below. Shared with auto-discovery
    // mode below purely as the "latest modeldef snapshot" storage --
    // apply_resolution() itself is never called in auto-discovery mode.
    std::vector<mpc::RigidBodyDef> known_modeldef;  // populated on the first successful modeldef parse.
    mpc::ResolutionResult current_resolution;
    bool all_names_resolved_logged = false;

    // Re-runs mpc::resolve_robot_ids() against the CURRENT known_modeldef
    // snapshot, rebuilds robots[]/id_to_index from the result, and logs
    // exactly the DELTAS (a robot going from unresolved->resolved, or --
    // auto mode only -- upgrading from an index-fallback id to a
    // name-match id) rather than re-logging the same mapping every call.
    // Safe/cheap to call repeatedly (e.g. once per modeldef refresh); a
    // no-op with respect to logging if nothing actually changed. Explicit
    // mode ONLY -- see apply_auto_discovery() below for the round-5
    // zero-arg counterpart.
    auto apply_resolution = [&]() {
        mpc::ResolutionResult new_resolution =
            mpc::resolve_robot_ids(robot_names, explicit_names, known_modeldef);
        for (size_t i = 0; i < new_resolution.robots.size(); ++i) {
            const bool was_resolved =
                (i < current_resolution.robots.size()) && current_resolution.robots[i].resolved;
            const bool now_resolved = new_resolution.robots[i].resolved;
            const bool id_changed = was_resolved && now_resolved &&
                                      current_resolution.robots[i].rigid_body_id !=
                                          new_resolution.robots[i].rigid_body_id;
            if ((!was_resolved && now_resolved) || id_changed) {
                std::cout << "[BRIDGE] " << robot_names[i]
                           << " resolved to rigid_body_id=" << new_resolution.robots[i].rigid_body_id
                           << " (" << mpc::to_string(new_resolution.robots[i].source) << ")"
                           << std::endl;
            }
        }
        current_resolution = new_resolution;

        for (size_t i = 0; i < robots.size(); ++i) {
            robots[i].rigid_body_id = current_resolution.robots[i].rigid_body_id;  // 0 if unresolved.
        }
        id_to_index.clear();
        for (size_t i = 0; i < robots.size(); ++i) {
            if (current_resolution.robots[i].resolved) {
                id_to_index[robots[i].rigid_body_id] = i;
            }
        }

        if (current_resolution.all_resolved && !all_names_resolved_logged) {
            all_names_resolved_logged = true;
            std::cout << "[BRIDGE] names_resolved: all " << robots.size() << " robot(s) resolved"
                       << std::endl;
        }
    };

    if (operating_mode == OperatingMode::kExplicit) {
        robot_names = split_comma(opt.robots_str);
        if (robot_names.empty()) {
            std::cerr << "[BRIDGE] --robots produced an empty list" << std::endl;
            return 1;
        }
        if (!opt.rigid_body_ids_str.empty() && !opt.rigid_body_names_str.empty()) {
            std::cerr << "[BRIDGE] --rigid-body-ids and --rigid-body-names are mutually exclusive"
                       << std::endl;
            return 1;
        }
        robots.reserve(robot_names.size());
        for (size_t i = 0; i < robot_names.size(); ++i) {
            RobotConfig rc;
            rc.name = robot_names[i];
            robots.push_back(rc);
        }

        if (!opt.rigid_body_ids_str.empty()) {
            id_mode = IdMode::kExplicitIds;
            std::vector<std::string> id_strs = split_comma(opt.rigid_body_ids_str);
            if (id_strs.size() != robots.size()) {
                std::cerr << "[BRIDGE] --rigid-body-ids has " << id_strs.size()
                           << " entries but --robots has " << robots.size() << "; they must match"
                           << std::endl;
                return 1;
            }
            for (size_t i = 0; i < robots.size(); ++i) {
                try {
                    robots[i].rigid_body_id = std::stoi(id_strs[i]);
                } catch (const std::exception&) {
                    std::cerr << "[BRIDGE] Invalid --rigid-body-ids entry '" << id_strs[i] << "'"
                               << std::endl;
                    return 1;
                }
            }
        } else if (!opt.rigid_body_names_str.empty()) {
            id_mode = IdMode::kExplicitNames;
            explicit_names = split_comma(opt.rigid_body_names_str);
            if (explicit_names.size() != robots.size()) {
                std::cerr << "[BRIDGE] --rigid-body-names has " << explicit_names.size()
                           << " entries but --robots has " << robots.size() << "; they must match"
                           << std::endl;
                return 1;
            }
        }
        // id_mode stays kAuto (explicit_names stays empty) if neither flag was given.

        for (RobotConfig& rc : robots) {
            auto it = map_cfg.yaw_offset.find(rc.name);
            rc.yaw_offset = (it != map_cfg.yaw_offset.end()) ? it->second : 0.0;
        }

        if (id_mode == IdMode::kExplicitIds) {
            // Immediate/trivial -- no modeldef needed at all; id_to_index
            // never changes again (modeldef requests still happen in the
            // background purely for the inventory log -- see the receive loop).
            for (size_t i = 0; i < robots.size(); ++i) {
                id_to_index[robots[i].rigid_body_id] = i;
            }
        } else {
            // kAuto resolves EVERY robot immediately via the unconditional
            // index fallback (mpc::resolve_robot_ids() with an empty modeldef)
            // -- publishing never waits on Motive in auto mode, matching
            // pre-round-4 behavior exactly. kExplicitNames intentionally stays
            // fully UNRESOLVED here (empty modeldef => no name can match yet);
            // apply_resolution() is called again from the receive loop as real
            // modeldef data arrives.
            apply_resolution();
        }
    }

    // ---------------------------------------------------------------------
    // Round 5: auto-discovery mode state -- only meaningfully populated/used
    // when operating_mode == kAutoDiscovery. One PUB socket (bound at
    // --loc-port-start, see the ZMQ setup below), topic-per-body, with
    // published names resolved dynamically as NAT_MODELDEF data arrives (no
    // "index fallback" here, unlike explicit kAuto -- there is no robot
    // list to fall back to; a body simply does not publish until modeldef
    // reports it, mirroring kExplicitNames' deferred-publishing discipline).
    // ---------------------------------------------------------------------
    struct AutoBodyState {
        std::string published_name;
        bool last_tracking_valid = true;
        long invalid_frame_count = 0;
        long published_count = 0;
    };
    std::unordered_map<std::int32_t, AutoBodyState> auto_body_state;  // keyed by rigid_body_id.
    std::set<std::string> unmatched_aliases_logged;
    long auto_frames_deferred_before_modeldef = 0;  // mirrors names-mode's deferred-count bookkeeping.

    // Re-runs mpc::resolve_auto_discovery() against the CURRENT
    // known_modeldef snapshot and aliases, adds any newly-discovered body to
    // auto_body_state (logging it individually), updates any body whose
    // published_name changed (e.g. an alias started matching), and -- only
    // when the map actually changed -- logs the full current body->topic
    // map. Also logs (once each) any alias whose Motive name has no match
    // yet. Safe/cheap to call repeatedly.
    auto apply_auto_discovery = [&]() {
        std::vector<mpc::AutoDiscoveredBody> discovered =
            mpc::resolve_auto_discovery(known_modeldef, map_cfg.aliases);
        bool map_changed = false;
        for (const mpc::AutoDiscoveredBody& b : discovered) {
            auto it = auto_body_state.find(b.rigid_body_id);
            if (it == auto_body_state.end()) {
                map_changed = true;
                AutoBodyState st;
                st.published_name = b.published_name;
                auto_body_state[b.rigid_body_id] = st;
                std::string reason = b.from_alias ? "alias"
                                       : b.used_empty_fallback ? "empty-name fallback"
                                       : b.name_sanitized ? "sanitized"
                                                            : "passthrough";
                std::cout << "[BRIDGE] auto-discovery: new body id=" << b.rigid_body_id
                           << " motive_name='" << b.motive_name << "' -> /" << b.published_name
                           << "/localization (" << reason << ")"
                           << (b.disambiguated ? " [disambiguated]" : "") << std::endl;
            } else if (it->second.published_name != b.published_name) {
                map_changed = true;
                std::cout << "[BRIDGE] auto-discovery: body id=" << b.rigid_body_id
                           << " topic changed to /" << b.published_name << "/localization" << std::endl;
                it->second.published_name = b.published_name;
            }
        }
        if (map_changed) {
            std::ostringstream m;
            m << "[BRIDGE] auto-discovery: publishing on port " << opt.loc_port_start << ", body map:";
            for (const auto& kv : auto_body_state) {
                m << " id=" << kv.first << "->/" << kv.second.published_name << "/localization;";
            }
            std::cout << m.str() << std::endl;
        }
        for (const std::string& mn : mpc::find_unmatched_aliases(map_cfg.aliases, known_modeldef)) {
            if (unmatched_aliases_logged.insert(mn).second) {
                std::cout << "[BRIDGE] auto-discovery: alias for '" << mn
                           << "' has no matching Motive asset yet (may appear later)" << std::endl;
            }
        }
    };

    mpc::NatNetVersion active_version = parse_natnet_version(opt.natnet_version_str, {3, 1});
    // Round 3: gates DATA-frame parsing entirely until the version is
    // confirmed (ping response, or --version-defer-timeout-s expiry) -- see
    // VersionGate's own doc comment for the live bug this fixes.
    mpc::VersionGate version_gate(opt.version_defer_timeout_s);
    // Frame-structure auto-fallback (round 2): only engaged while
    // !version_gate.confirmed_via_ping() -- i.e. only while relying on a
    // TIMEOUT-fallback GUESS, never once a real ping response has
    // authoritatively confirmed the version.
    mpc::VersionAutoDetector version_auto_detector({{4, 0}, {3, 1}, {2, 10}}, 5);

    std::signal(SIGINT, handle_shutdown_signal);
    std::signal(SIGTERM, handle_shutdown_signal);

    std::cout << "[BRIDGE] Launching: mode=" << opt.mode << " server=" << opt.server_ip << ":"
               << opt.command_port << " robots="
               << (operating_mode == OperatingMode::kAutoDiscovery ? "<auto-discovery>" : opt.robots_str)
               << " natnet_fallback=" << active_version.major << "." << active_version.minor
               << " publish_rate=" << opt.publish_rate_hz << "Hz zmq_disable="
               << (opt.zmq_disable ? "yes" : "no") << std::endl;

    // Round 3: auto-select the multicast join interface (see
    // resolve_multicast_interface()/open_client_socket()'s doc comments).
    // Only meaningful in multicast mode; writes the resolved address into
    // opt.multicast_interface for open_client_socket() to consume unchanged.
    if (opt.mode == "multicast") {
        IfaceChoice iface_choice = resolve_multicast_interface(opt);
        opt.multicast_interface = iface_choice.address;
        std::cout << "[BRIDGE] multicast join via " << iface_choice.address << " ("
                   << iface_choice.description << ")" << std::endl;
    }

    int sock = open_client_socket(opt);
    if (sock < 0) {
        return 1;
    }
    // Ping destination -- used in BOTH modes now (round 2: multicast mode
    // also pings for version discovery; see this file's header comment).
    sockaddr_in command_addr{};
    if (!make_sockaddr(opt.server_ip, static_cast<std::uint16_t>(opt.command_port), command_addr)) {
        std::cerr << "[BRIDGE] Invalid --server-ip '" << opt.server_ip << "'" << std::endl;
        close(sock);
        return 1;
    }

    zmq::context_t context(1);
    std::vector<zmq::socket_t> pub_sockets;   // explicit mode: one PUB per robot.
    std::vector<std::string> loc_topics;      // explicit mode, parallel to pub_sockets.
    // Round 5: auto-discovery mode's ONE PUB socket, topic-per-body (see
    // print_usage()'s "AUTO-DISCOVERY" section). Always default-constructed
    // (cheap, unbound) even in explicit mode so both branches compile
    // uniformly; auto_pub_bound tracks whether it actually got bound.
    zmq::socket_t auto_pub_socket(context, zmq::socket_type::pub);
    bool auto_pub_bound = false;

    if (operating_mode == OperatingMode::kExplicit) {
        if (!opt.zmq_disable) {
            for (size_t i = 0; i < robots.size(); ++i) {
                zmq::socket_t pub(context, zmq::socket_type::pub);
                pub.set(zmq::sockopt::linger, 0);
                const std::string endpoint =
                    "tcp://*:" + std::to_string(opt.loc_port_start + static_cast<int>(i));
                try {
                    pub.bind(endpoint);
                } catch (const std::exception& ex) {
                    std::cerr << "[BRIDGE] Failed to bind PUB socket for '" << robots[i].name << "' at "
                               << endpoint << ": " << ex.what() << std::endl;
                    close(sock);
                    return 1;
                }
                std::cout << "[BRIDGE] " << robots[i].name << ": rigid_body_id="
                           << robots[i].rigid_body_id << " -> localization PUB bound at " << endpoint
                           << std::endl;
                pub_sockets.push_back(std::move(pub));
                loc_topics.push_back("/" + robots[i].name + "/localization");
            }
        }
    } else if (!opt.zmq_disable) {
        // Round 5: auto-discovery mode -- ONE bound port, topic-per-body
        // (any number of controllers/viz can connect to this one port and
        // filter by topic).
        auto_pub_socket.set(zmq::sockopt::linger, 0);
        const std::string endpoint = "tcp://*:" + std::to_string(opt.loc_port_start);
        try {
            auto_pub_socket.bind(endpoint);
            auto_pub_bound = true;
            std::cout << "[BRIDGE] auto-discovery: localization PUB bound at " << endpoint
                       << " (topic-per-body; waiting for NAT_MODELDEF before anything publishes)"
                       << std::endl;
        } catch (const std::exception& ex) {
            std::cerr << "[BRIDGE] Failed to bind auto-discovery PUB socket at " << endpoint << ": "
                       << ex.what() << std::endl;
            close(sock);
            return 1;
        }
    }

    std::ofstream csv;
    if (!opt.log_csv_path.empty()) {
        csv.open(opt.log_csv_path, std::ios::out | std::ios::trunc);
        if (!csv.is_open()) {
            std::cerr << "[BRIDGE] Failed to open --log-csv path '" << opt.log_csv_path << "'"
                       << std::endl;
            close(sock);
            return 1;
        }
        // Round 4: full 6-DoF retention (the ZMQ payload stays planar-only
        // {"x","y","yaw"} regardless -- see mpc::quat_to_roll_pitch_heading()'s
        // doc comment for the exact roll/pitch/heading sequence definition;
        // heading == planar_yaw's pre-map-transform value, bit-for-bit).
        csv << "t_arrival,robot,raw_x,raw_y,raw_z,qx,qy,qz,qw,roll,pitch,heading,planar_x,planar_y,"
               "planar_yaw,tracking_valid\n";
    }

    struct RobotPoseSample {
        size_t robot_index = 0;
        mpc::PlanarPose world;
    };
    mpc::Downsampler<std::vector<RobotPoseSample>> publish_gate(opt.publish_rate_hz);

    // Round 5: auto-discovery mode's per-frame publish batch + downsampler
    // (same "latest-sample-wins, one shared gate per frame" pattern as
    // explicit mode's publish_gate above, just carrying rigid_body_id +
    // published_name instead of a fixed robot_index).
    struct AutoBodyPoseSample {
        std::int32_t rigid_body_id = 0;
        std::string published_name;
        mpc::PlanarPose world;
    };
    mpc::Downsampler<std::vector<AutoBodyPoseSample>> auto_publish_gate(opt.publish_rate_hz);

    std::vector<bool> last_tracking_valid(robots.size(), true);
    std::vector<long> invalid_frame_count(robots.size(), 0);
    std::vector<long> published_count(robots.size(), 0);
    // Round 4: per-robot count of frames processed while that robot's name
    // was still unresolved (mirrors VersionGate's frames_deferred
    // bookkeeping) -- only ever increments for kExplicitNames (kAuto never
    // stays unresolved; kExplicitIds never defers at all).
    std::vector<long> names_deferred_count(robots.size(), 0);
    std::set<std::int32_t> unknown_ids_warned;

    long frames_received = 0;
    long parse_errors = 0;
    long frames_deferred = 0;  // round 3: VersionGate deferred these -- see version_gate above.
    long frames_at_last_hz_print = 0;

    // Round 4: NAT_MODELDEF request/retry state. want_modeldef starts true
    // (always try at least once); goes false after each successful parse;
    // re-armed by any unknown rigid-body id appearing in a frame (see the
    // unknown-id handling below) so the inventory/resolution can pick up
    // Motive assets enabled after this bridge started. modeldef_received
    // (once true, forever true) selects the retry cadence: ~1Hz before the
    // first success, a low background rate afterward.
    bool want_modeldef = true;
    bool modeldef_received = false;
    bool modeldef_inventory_logged = false;  // the ONE-TIME "Motive assets: ..." line.

    RateLimiter parse_error_warn_limiter;
    RateLimiter unknown_message_warn_limiter;
    RateLimiter name_error_warn_limiter;  // ~1/10s re-log of unresolved --rigid-body-names entries.

    const auto t_start = std::chrono::steady_clock::now();
    auto last_ping = t_start - std::chrono::seconds(10);  // force an immediate first ping.
    auto last_modeldef_request = t_start - std::chrono::seconds(10);  // force an immediate first request.
    auto last_hz_print = t_start;

    std::vector<std::uint8_t> recv_buf(65536);

    std::cout << "[BRIDGE] Entering receive loop..." << std::endl;

    while (!g_shutdown_requested) {
        pollfd pfd{};
        pfd.fd = sock;
        pfd.events = POLLIN;
        int poll_rc = poll(&pfd, 1, 100);

        if (poll_rc > 0 && (pfd.revents & POLLIN)) {
            sockaddr_in src_addr{};
            socklen_t src_len = sizeof(src_addr);
            ssize_t n = recvfrom(sock, recv_buf.data(), recv_buf.size(), 0,
                                   reinterpret_cast<sockaddr*>(&src_addr), &src_len);
            const auto now = std::chrono::steady_clock::now();
            const double t_arrival = std::chrono::duration<double>(now - t_start).count();

            if (n > 0) {
                const std::uint8_t* data = recv_buf.data();
                const std::size_t size = static_cast<std::size_t>(n);
                mpc::PacketHeader hdr = mpc::peek_packet_header(data, size);
                if (!hdr.ok) {
                    if (parse_error_warn_limiter.allow(now, 2.0)) {
                        std::cerr << "[BRIDGE] Warning: received a malformed/truncated packet ("
                                   << n << " bytes), ignoring" << std::endl;
                    }
                } else if (hdr.message_id == mpc::kNatFrameOfData) {
                    ++frames_received;
                    // Round 3: gate parsing entirely until the version is
                    // confirmed -- see VersionGate's doc comment for the
                    // live bug (spurious wrong-version "success") this
                    // prevents. should_parse_now() itself performs the
                    // timeout-expiry fallback confirmation the instant
                    // elapsed_s crosses --version-defer-timeout-s.
                    if (!version_gate.should_parse_now(t_arrival, active_version)) {
                        ++frames_deferred;  // still deferring -- do not parse this frame at all.
                    } else {
                    if (version_gate.just_confirmed() && !version_gate.confirmed_via_ping()) {
                        std::cout << "[BRIDGE] deferral timeout (" << opt.version_defer_timeout_s
                                   << "s), proceeding with fallback " << active_version.major << "."
                                   << active_version.minor << ", " << version_gate.deferred_count()
                                   << " frames deferred" << std::endl;
                    }
                    mpc::FrameOfData frame = mpc::parse_frame_of_data(data, size, active_version);
                    if (!frame.parse_ok) {
                        ++parse_errors;
                        // Round 2: frame-structure auto-fallback, only while
                        // relying on a timeout-fallback GUESS (never once a
                        // real ping response has authoritatively confirmed
                        // the version -- see VersionGate/VersionAutoDetector
                        // doc comments).
                        if (!version_gate.confirmed_via_ping()) {
                            mpc::VersionAutoDetector::Outcome outcome =
                                version_auto_detector.on_parse_failure(data, size);
                            if (outcome.adopted) {
                                active_version = outcome.adopted_version;
                                std::cout << "[BRIDGE] #### auto-detected NatNet "
                                           << active_version.major << "." << active_version.minor
                                           << " from frame structure (5 consecutive clean parses, no "
                                              "ping response yet) ####"
                                           << std::endl;
                            }
                        }
                        if (parse_error_warn_limiter.allow(now, 1.0)) {
                            std::cerr << "[BRIDGE] Warning: frame parse error: " << frame.error
                                       << " (hint: try --natnet-version if this persists)" << std::endl;
                        }
                    } else {
                        if (!version_gate.confirmed_via_ping()) {
                            version_auto_detector.on_parse_success();
                        }
                        if (opt.print_frames) {
                            std::cout << "[BRIDGE] frame=" << frame.frame_number;
                        }

                        if (operating_mode == OperatingMode::kExplicit) {
                        std::vector<RobotPoseSample> samples;
                        // Round 4: count, per robot, every frame processed
                        // while its name is still unresolved (kExplicitNames
                        // only -- kAuto/kExplicitIds never stay unresolved).
                        if (id_mode == IdMode::kExplicitNames) {
                            for (size_t i = 0; i < robots.size(); ++i) {
                                if (i >= current_resolution.robots.size() ||
                                    !current_resolution.robots[i].resolved) {
                                    ++names_deferred_count[i];
                                }
                            }
                        }
                        for (const mpc::RigidBodySample& rb : frame.rigid_bodies) {
                            auto it = id_to_index.find(rb.id);
                            if (it == id_to_index.end()) {
                                // Printed for EVERY unmatched id on EVERY frame (not just once) --
                                // --print-frames must reliably double as rigid-body-ID discovery on
                                // a first live run against real Motive, where --rigid-body-ids is
                                // necessarily still a placeholder (the real ids are exactly what
                                // this is discovering). Raw (pre-transform) x/y/z shown since the
                                // up-axis/map-config may not be known/correct yet at discovery time.
                                if (opt.print_frames) {
                                    std::cout << " id=" << rb.id << "(unmatched,x=" << rb.x
                                               << ",y=" << rb.y << ",z=" << rb.z
                                               << ",valid=" << (rb.tracking_valid ? 1 : 0) << ")";
                                }
                                if (unknown_ids_warned.insert(rb.id).second) {
                                    std::cerr << "[BRIDGE] Warning: unknown rigid body id=" << rb.id
                                               << ", ignoring (not in --rigid-body-ids/"
                                                  "--rigid-body-names)"
                                               << std::endl;
                                }
                                // Round 4: an unknown id re-arms background
                                // modeldef requesting (Motive's asset list
                                // may have changed since we last asked).
                                want_modeldef = true;
                                continue;
                            }
                            const size_t idx = it->second;
                            const std::string& robot_name = robots[idx].name;

                            mpc::PlanarPose raw = mpc::extract_planar_pose(rb, map_cfg.y_up);
                            mpc::PlanarPose world = apply_world_transform(raw, map_cfg);
                            world.yaw = mpc::normalize_angle(world.yaw + robots[idx].yaw_offset);
                            // Round 4: full 6-DoF, diagnostics-only (--log-csv) -- the ZMQ payload
                            // published below stays planar-only. heading == raw.yaw bit-for-bit
                            // (both come from the SAME quat_to_planar_yaw() call internally).
                            const mpc::EulerRollPitchHeading rph =
                                mpc::quat_to_roll_pitch_heading(rb.qx, rb.qy, rb.qz, rb.qw, map_cfg.y_up);

                            if (rb.tracking_valid != last_tracking_valid[idx]) {
                                std::cout << "[BRIDGE] " << robot_name << ": tracking "
                                           << (rb.tracking_valid ? "RECOVERED" : "LOST") << std::endl;
                                last_tracking_valid[idx] = rb.tracking_valid;
                            }
                            if (!rb.tracking_valid) {
                                ++invalid_frame_count[idx];
                            }

                            if (csv.is_open()) {
                                csv << t_arrival << ',' << robot_name << ',' << rb.x << ',' << rb.y
                                     << ',' << rb.z << ',' << rb.qx << ',' << rb.qy << ',' << rb.qz
                                     << ',' << rb.qw << ',' << rph.roll << ',' << rph.pitch << ','
                                     << rph.heading << ',' << world.x << ',' << world.y << ','
                                     << world.yaw << ',' << (rb.tracking_valid ? 1 : 0) << '\n';
                            }

                            if (opt.print_frames) {
                                std::cout << " " << robot_name << "=(x=" << world.x
                                           << ",y=" << world.y << ",yaw=" << world.yaw
                                           << ",raw_z=" << rb.z
                                           << ",valid=" << (rb.tracking_valid ? 1 : 0) << ")";
                            }

                            if (rb.tracking_valid) {
                                samples.push_back(RobotPoseSample{idx, world});
                            }
                        }

                        if (!opt.zmq_disable) {
                            std::vector<RobotPoseSample> to_publish;
                            if (publish_gate.feed(t_arrival, samples, to_publish)) {
                                for (const RobotPoseSample& s : to_publish) {
                                    const std::string payload = mpc::encode_localization_payload(
                                        s.world.x, s.world.y, s.world.yaw);
                                    zmq::message_t topic_msg(loc_topics[s.robot_index].begin(),
                                                               loc_topics[s.robot_index].end());
                                    zmq::message_t payload_msg(payload.begin(), payload.end());
                                    try {
                                        pub_sockets[s.robot_index].send(topic_msg, zmq::send_flags::sndmore);
                                        pub_sockets[s.robot_index].send(payload_msg, zmq::send_flags::none);
                                        ++published_count[s.robot_index];
                                    } catch (const zmq::error_t& ex) {
                                        std::cerr << "[BRIDGE] Warning: zmq send error for "
                                                   << robots[s.robot_index].name << ": " << ex.what()
                                                   << std::endl;
                                    }
                                }
                            }
                        }
                        } else {
                        // Round 5: auto-discovery mode -- every rigid body
                        // Motive's modeldef reports gets published, one
                        // topic per body, on the single auto_pub_socket.
                        if (!modeldef_received) {
                            ++auto_frames_deferred_before_modeldef;
                        }
                        std::vector<AutoBodyPoseSample> auto_samples;
                        for (const mpc::RigidBodySample& rb : frame.rigid_bodies) {
                            auto it = auto_body_state.find(rb.id);
                            if (it == auto_body_state.end()) {
                                if (opt.print_frames) {
                                    std::cout << " id=" << rb.id << "(undiscovered,x=" << rb.x
                                               << ",y=" << rb.y << ",z=" << rb.z
                                               << ",valid=" << (rb.tracking_valid ? 1 : 0) << ")";
                                }
                                if (unknown_ids_warned.insert(rb.id).second) {
                                    std::cerr << "[BRIDGE] Warning: rigid body id=" << rb.id
                                               << " not yet in Motive's modeldef inventory, ignoring"
                                               << std::endl;
                                }
                                want_modeldef = true;
                                continue;
                            }
                            AutoBodyState& state = it->second;
                            const std::string& published_name = state.published_name;

                            mpc::PlanarPose raw = mpc::extract_planar_pose(rb, map_cfg.y_up);
                            mpc::PlanarPose world = apply_world_transform(raw, map_cfg);
                            // Round 5: yaw_offset keys on the PUBLISHED name
                            // (documented in --map-config's "_doc").
                            auto yo_it = map_cfg.yaw_offset.find(published_name);
                            const double yaw_offset =
                                (yo_it != map_cfg.yaw_offset.end()) ? yo_it->second : 0.0;
                            world.yaw = mpc::normalize_angle(world.yaw + yaw_offset);
                            const mpc::EulerRollPitchHeading rph =
                                mpc::quat_to_roll_pitch_heading(rb.qx, rb.qy, rb.qz, rb.qw, map_cfg.y_up);

                            if (rb.tracking_valid != state.last_tracking_valid) {
                                std::cout << "[BRIDGE] " << published_name << ": tracking "
                                           << (rb.tracking_valid ? "RECOVERED" : "LOST") << std::endl;
                                state.last_tracking_valid = rb.tracking_valid;
                            }
                            if (!rb.tracking_valid) {
                                ++state.invalid_frame_count;
                            }

                            if (csv.is_open()) {
                                csv << t_arrival << ',' << published_name << ',' << rb.x << ',' << rb.y
                                     << ',' << rb.z << ',' << rb.qx << ',' << rb.qy << ',' << rb.qz
                                     << ',' << rb.qw << ',' << rph.roll << ',' << rph.pitch << ','
                                     << rph.heading << ',' << world.x << ',' << world.y << ','
                                     << world.yaw << ',' << (rb.tracking_valid ? 1 : 0) << '\n';
                            }

                            if (opt.print_frames) {
                                std::cout << " " << published_name << "=(x=" << world.x
                                           << ",y=" << world.y << ",yaw=" << world.yaw
                                           << ",raw_z=" << rb.z
                                           << ",valid=" << (rb.tracking_valid ? 1 : 0) << ")";
                            }

                            if (rb.tracking_valid) {
                                auto_samples.push_back(AutoBodyPoseSample{rb.id, published_name, world});
                            }
                        }

                        if (!opt.zmq_disable && auto_pub_bound) {
                            std::vector<AutoBodyPoseSample> to_publish;
                            if (auto_publish_gate.feed(t_arrival, auto_samples, to_publish)) {
                                for (const AutoBodyPoseSample& s : to_publish) {
                                    const std::string payload = mpc::encode_localization_payload(
                                        s.world.x, s.world.y, s.world.yaw);
                                    const std::string topic = "/" + s.published_name + "/localization";
                                    zmq::message_t topic_msg(topic.begin(), topic.end());
                                    zmq::message_t payload_msg(payload.begin(), payload.end());
                                    try {
                                        auto_pub_socket.send(topic_msg, zmq::send_flags::sndmore);
                                        auto_pub_socket.send(payload_msg, zmq::send_flags::none);
                                        auto state_it = auto_body_state.find(s.rigid_body_id);
                                        if (state_it != auto_body_state.end()) {
                                            ++state_it->second.published_count;
                                        }
                                    } catch (const zmq::error_t& ex) {
                                        std::cerr << "[BRIDGE] Warning: zmq send error for "
                                                   << s.published_name << ": " << ex.what() << std::endl;
                                    }
                                }
                            }
                        }
                        }

                        if (opt.print_frames) {
                            std::cout << std::endl;
                        }
                    }
                    }  // end of the version_gate.should_parse_now() else-branch.
                } else if (hdr.message_id == mpc::kNatPingResponse) {
                    mpc::PingResponse resp = mpc::parse_command_response(data, size);
                    if (resp.ok) {
                        mpc::NatNetVersion discovered{resp.natnet_version[0], resp.natnet_version[1]};
                        version_gate.confirm_via_ping(discovered, t_arrival);
                        active_version = discovered;
                        if (version_gate.just_confirmed()) {
                            std::cout << "[BRIDGE] version confirmed " << discovered.major << "."
                                       << discovered.minor << " via ping after "
                                       << version_gate.confirmed_at_s() << "s, "
                                       << version_gate.deferred_count() << " frames deferred"
                                       << std::endl;
                        }
                        std::cout << "[BRIDGE] Discovered server: app=\"" << resp.app_name
                                   << "\" NatNet=" << static_cast<int>(resp.natnet_version[0]) << "."
                                   << static_cast<int>(resp.natnet_version[1])
                                   << " (app_version=" << static_cast<int>(resp.app_version[0]) << "."
                                   << static_cast<int>(resp.app_version[1]) << "."
                                   << static_cast<int>(resp.app_version[2]) << "."
                                   << static_cast<int>(resp.app_version[3]) << ")" << std::endl;
                    }
                } else if (hdr.message_id == mpc::kNatModelDef) {
                    // Round 4: same defensive principle as data frames --
                    // don't attempt to parse a version-dependent payload
                    // before the version is confirmed.
                    if (version_gate.confirmed()) {
                        mpc::ModelDef md = mpc::parse_modeldef(data, size, active_version);
                        if (md.parse_ok) {
                            known_modeldef = md.rigid_bodies;
                            modeldef_received = true;
                            want_modeldef = false;
                            // ONE-TIME inventory line -- the user's name-discovery
                            // tool -- logged in EVERY mode (ids mode too).
                            if (!modeldef_inventory_logged) {
                                modeldef_inventory_logged = true;
                                std::ostringstream inv;
                                inv << "[BRIDGE] Motive assets:";
                                if (known_modeldef.empty()) {
                                    inv << " (none)";
                                }
                                for (const mpc::RigidBodyDef& rbd : known_modeldef) {
                                    inv << " rigidbody id=" << rbd.id << " name='" << rbd.name
                                        << "' (parent=" << rbd.parent_id << ");";
                                }
                                std::cout << inv.str() << std::endl;
                            }
                            if (operating_mode == OperatingMode::kExplicit) {
                                if (id_mode != IdMode::kExplicitIds) {
                                    apply_resolution();
                                }
                            } else {
                                apply_auto_discovery();
                            }
                        } else if (parse_error_warn_limiter.allow(now, 2.0)) {
                            std::cerr << "[BRIDGE] Warning: NAT_MODELDEF parse error: " << md.error
                                       << std::endl;
                        }
                    }
                } else if (unknown_message_warn_limiter.allow(now, 5.0)) {
                    std::cerr << "[BRIDGE] Warning: received unhandled messageID="
                               << hdr.message_id << std::endl;
                }
            }
        }

        const auto now = std::chrono::steady_clock::now();

        // Keepalive ping -- sent in BOTH modes now (round 2: multicast also
        // pings, purely for version discovery; frames themselves keep
        // arriving via the joined multicast group regardless of pings).
        if (std::chrono::duration<double>(now - last_ping).count() >= 1.0) {
            std::vector<std::uint8_t> ping = mpc::build_ping_packet("optitrack_zmq_bridge");
            sendto(sock, ping.data(), ping.size(), 0, reinterpret_cast<sockaddr*>(&command_addr),
                    sizeof(command_addr));
            last_ping = now;
        }

        // Round 4: NAT_REQUEST_MODELDEF -- ~1Hz retry until the first
        // successful reply, then a low (5s) background rate, re-armed by
        // want_modeldef whenever an unknown rigid-body id shows up in a
        // frame. Only sent once the version is confirmed (parsing the
        // reply needs a known version too -- see the receive branch above).
        if (version_gate.confirmed() && want_modeldef) {
            const double modeldef_interval_s = modeldef_received ? 5.0 : 1.0;
            if (std::chrono::duration<double>(now - last_modeldef_request).count() >=
                modeldef_interval_s) {
                std::vector<std::uint8_t> req = mpc::build_modeldef_request();
                sendto(sock, req.data(), req.size(), 0, reinterpret_cast<sockaddr*>(&command_addr),
                        sizeof(command_addr));
                last_modeldef_request = now;
            }
        }

        // Round 4: ~1/10s re-log of any --rigid-body-names entry still
        // unresolved, listing every name currently known in Motive's
        // inventory -- "keep trying" per the design spec, using whatever
        // known_modeldef snapshot is current (may be stale between modeldef
        // refreshes; still accurate/useful).
        if (id_mode == IdMode::kExplicitNames && !current_resolution.all_resolved &&
            name_error_warn_limiter.allow(now, 0.1)) {
            for (const mpc::RobotResolutionEntry& r : current_resolution.robots) {
                if (!r.resolved) {
                    std::cerr << "[BRIDGE] Warning: " << r.error << std::endl;
                }
            }
        }

        if (opt.print_frames && std::chrono::duration<double>(now - last_hz_print).count() >= 1.0) {
            const double elapsed = std::chrono::duration<double>(now - last_hz_print).count();
            const double hz = (frames_received - frames_at_last_hz_print) / elapsed;
            const char* version_source =
                version_gate.confirmed_via_ping()
                    ? "discovered-via-ping"
                    : (version_auto_detector.has_adopted() ? "auto-detected" : "fallback");
            std::cout << "[BRIDGE] stats: recv_hz=" << hz << " frames_total=" << frames_received
                       << " frames_deferred=" << frames_deferred << " parse_errors=" << parse_errors
                       << " version=" << active_version.major << "." << active_version.minor << " ("
                       << version_source << ")";
            if (operating_mode == OperatingMode::kExplicit && id_mode == IdMode::kExplicitNames) {
                std::cout << " names_resolved=" << (current_resolution.all_resolved ? "yes" : "no");
            }
            if (operating_mode == OperatingMode::kAutoDiscovery) {
                std::cout << " auto_discovery_bodies=" << auto_body_state.size()
                           << " frames_deferred_before_modeldef=" << auto_frames_deferred_before_modeldef;
            }
            std::cout << std::endl;
            frames_at_last_hz_print = frames_received;
            last_hz_print = now;
        }

        if (opt.duration_s > 0.0 &&
            std::chrono::duration<double>(now - t_start).count() >= opt.duration_s) {
            std::cout << "[BRIDGE] Reached --duration-s=" << opt.duration_s << "; stopping."
                       << std::endl;
            break;
        }
    }

    const double total_elapsed = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() - t_start)
                                       .count();
    std::cout << "\n[BRIDGE] Shutting down. Stats summary:\n"
               << "  frames_received=" << frames_received << "\n"
               << "  frames_deferred=" << frames_deferred << "\n"
               << "  parse_errors=" << parse_errors << "\n"
               << "  elapsed_s=" << total_elapsed << "\n"
               << "  avg_receive_hz=" << (total_elapsed > 0.0 ? frames_received / total_elapsed : 0.0)
               << std::endl;
    if (operating_mode == OperatingMode::kExplicit) {
        for (size_t i = 0; i < robots.size(); ++i) {
            std::cout << "  " << robots[i].name << ": published=" << published_count[i]
                       << " frames_tracking_lost=" << invalid_frame_count[i];
            if (id_mode == IdMode::kExplicitNames) {
                std::cout << " names_deferred=" << names_deferred_count[i]
                           << " resolved="
                           << ((i < current_resolution.robots.size() &&
                                current_resolution.robots[i].resolved)
                                   ? "yes"
                                   : "no");
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "  frames_deferred_before_modeldef=" << auto_frames_deferred_before_modeldef << "\n";
        for (const auto& kv : auto_body_state) {
            std::cout << "  " << kv.second.published_name << " (id=" << kv.first
                       << "): published=" << kv.second.published_count
                       << " frames_tracking_lost=" << kv.second.invalid_frame_count << std::endl;
        }
    }

    if (csv.is_open()) {
        csv.flush();
        csv.close();
    }
    for (zmq::socket_t& pub : pub_sockets) {
        pub.close();
    }
    if (auto_pub_bound) {
        auto_pub_socket.close();
    }
    close(sock);

    return 0;
}
