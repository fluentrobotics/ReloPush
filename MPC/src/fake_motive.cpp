// fake_motive: synthetic NatNet server for testing optitrack_zmq_bridge
// without real OptiTrack/Motive hardware. Sends NAT_FRAMEOFDATA packets (via
// mpc::build_frame_of_data(), the SAME builder the unit tests round-trip
// against mpc::parse_frame_of_data()) describing N rigid bodies moving on a
// slow circle, and optionally answers NAT_PING with a configurable
// NAT_PINGRESPONSE so the bridge's version-discovery path is testable
// end-to-end too.
//
// All NatNet wire building/geometry conventions live in mpc::
// (OptiTrackCore.h/.cpp) -- this file is a thin I/O shell around it, mirroring
// robot_sim.cpp's own role relative to SimCore.

#include "mpc/OptiTrackCore.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t g_shutdown_requested = 0;
void handle_shutdown_signal(int) { g_shutdown_requested = 1; }

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
        int maj = std::stoi(s.substr(0, pos));
        int min = std::stoi(s.substr(pos + 1));
        return mpc::NatNetVersion{maj, min};
    } catch (const std::exception&) {
        return fallback;
    }
}

struct Options {
    std::string target_ip = "127.0.0.1";
    int target_port = 0;  // 0 = unset; validated >0 before running.
    double rate_hz = 120.0;
    std::string natnet_version_str = "3.1";
    std::string body_ids_str = "1";
    bool y_up = false;
    double drop_tracking_every_s = 0.0;  // 0 = never drop.
    double duration_s = 0.0;             // 0 = forever.
    int serve_command_port = 0;          // 0 = disabled.
    std::string served_app_name = "Fake Motive";
    std::string served_version_str;  // empty => same as natnet_version_str.
    // Round 2: when set, --target-ip/--target-port are expected to be a
    // multicast group/port (e.g. 239.255.42.99:1511) rather than a unicast
    // client address; configures the send socket accordingly (see main()).
    bool multicast_target = false;
    // Round 4: parallel to --body-ids; the Motive asset name served for
    // each id via NAT_MODELDEF (see main()'s command-port handling). Empty
    // (default) => each body is named "Body<id>".
    std::string body_names_str;
    // Round 5 test-support flag: delays the NAT_MODELDEF reply by this many
    // seconds after the request arrives (default 0 = immediate, same as
    // before). NAT_PING is answered immediately regardless -- only exists
    // to make a bridge's "frames processed before modeldef arrived" window
    // deterministically observable in tests (on a fast loopback round trip,
    // that window is otherwise sub-frame-period and empirically often 0).
    double modeldef_delay_s = 0.0;
};

Options parse_args(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (arg == "--target-ip") {
            o.target_ip = next();
        } else if (arg == "--target-port") {
            o.target_port = std::stoi(next());
        } else if (arg == "--rate") {
            o.rate_hz = std::stod(next());
        } else if (arg == "--natnet-version") {
            o.natnet_version_str = next();
        } else if (arg == "--body-ids") {
            o.body_ids_str = next();
        } else if (arg == "--y-up") {
            o.y_up = true;
        } else if (arg == "--drop-tracking-every") {
            o.drop_tracking_every_s = std::stod(next());
        } else if (arg == "--duration-s") {
            o.duration_s = std::stod(next());
        } else if (arg == "--serve-command-port") {
            o.serve_command_port = std::stoi(next());
        } else if (arg == "--served-app-name") {
            o.served_app_name = next();
        } else if (arg == "--served-version") {
            o.served_version_str = next();
        } else if (arg == "--multicast-target") {
            o.multicast_target = true;
        } else if (arg == "--body-names") {
            o.body_names_str = next();
        } else if (arg == "--modeldef-delay-s") {
            o.modeldef_delay_s = std::stod(next());
        } else {
            std::cerr << "[FAKE_MOTIVE] Warning: unrecognized argument '" << arg << "'" << std::endl;
        }
    }
    return o;
}

bool make_sockaddr(const std::string& ip, std::uint16_t port, sockaddr_in& out) {
    std::memset(&out, 0, sizeof(out));
    out.sin_family = AF_INET;
    out.sin_port = htons(port);
    return inet_pton(AF_INET, ip.c_str(), &out.sin_addr) == 1;
}

// Slow circle, radius 1m, one revolution per 20s. Body `body_index` (0-based
// among --body-ids) is centered at (2.5*body_index, 0) so multiple bodies
// stay visibly/numerically separate. See OptiTrackCore.h's
// quat_to_planar_yaw doc comment for the y_up axis-mapping this mirrors:
// y_up=false emits Z-up (x,y) + a qz/qw yaw-only quaternion; y_up=true emits
// Motive's native Y-up (x, z=-y) + a qy/qw yaw-only quaternion -- both encode
// the SAME logical circle, just in different wire conventions, so a bridge
// run with either --map-config y_up setting should recover matching poses.
mpc::RigidBodySample compute_body_pose(double t, std::int32_t id, int body_index, bool y_up,
                                        bool tracking_valid) {
    constexpr double kRadius = 1.0;
    constexpr double kAngularRateRadPerS = 2.0 * M_PI / 20.0;  // slow: 20s per revolution.
    const double center_x = 2.5 * body_index;

    const double angle = kAngularRateRadPerS * t;
    const double logical_x = center_x + kRadius * std::cos(angle);
    const double logical_y = kRadius * std::sin(angle);
    // Tangent direction for CCW motion (velocity angle = position angle + pi/2).
    const double logical_yaw = mpc::normalize_angle(angle + M_PI / 2.0);
    const double half_yaw = logical_yaw / 2.0;

    mpc::RigidBodySample rb;
    rb.id = id;
    rb.mean_error = 0.001;
    rb.tracking_valid = tracking_valid;
    if (!y_up) {
        rb.x = logical_x;
        rb.y = logical_y;
        rb.z = 0.0;
        rb.qx = 0.0;
        rb.qy = 0.0;
        rb.qz = std::sin(half_yaw);
        rb.qw = std::cos(half_yaw);
    } else {
        rb.x = logical_x;
        rb.y = 0.0;  // height.
        rb.z = -logical_y;
        rb.qx = 0.0;
        rb.qy = std::sin(half_yaw);
        rb.qz = 0.0;
        rb.qw = std::cos(half_yaw);
    }
    return rb;
}

// Body 0 (only) goes tracking_invalid for a 1.0s window every
// drop_tracking_every_s seconds (<=0 disables dropping entirely). E.g.
// drop_tracking_every_s=3 drops tracking during t in [0,1), [3,4), [6,7)...
bool tracking_valid_for(int body_index, double t, double drop_tracking_every_s) {
    if (body_index != 0 || drop_tracking_every_s <= 0.0) return true;
    double phase = std::fmod(t, drop_tracking_every_s);
    if (phase < 0.0) phase += drop_tracking_every_s;
    constexpr double kDropWindowS = 1.0;
    return !(phase < kDropWindowS);
}

}  // namespace

int main(int argc, char** argv) {
    Options opt = parse_args(argc, argv);

    if (opt.target_port <= 0) {
        std::cerr << "[FAKE_MOTIVE] --target-port is required and must be > 0" << std::endl;
        return 1;
    }
    if (opt.rate_hz <= 0.0) {
        std::cerr << "[FAKE_MOTIVE] --rate must be > 0" << std::endl;
        return 1;
    }

    std::vector<std::int32_t> body_ids;
    for (const std::string& s : split_comma(opt.body_ids_str)) {
        try {
            body_ids.push_back(std::stoi(s));
        } catch (const std::exception&) {
            std::cerr << "[FAKE_MOTIVE] Invalid --body-ids entry '" << s << "'" << std::endl;
            return 1;
        }
    }
    if (body_ids.empty()) {
        std::cerr << "[FAKE_MOTIVE] --body-ids produced an empty list" << std::endl;
        return 1;
    }

    // Round 4: --body-names, parallel to --body-ids, for serving
    // NAT_MODELDEF (name resolution testing). Defaults to "Body<id>" per
    // entry when not given.
    std::vector<std::string> body_names;
    if (!opt.body_names_str.empty()) {
        body_names = split_comma(opt.body_names_str);
        if (body_names.size() != body_ids.size()) {
            std::cerr << "[FAKE_MOTIVE] --body-names has " << body_names.size()
                       << " entries but --body-ids has " << body_ids.size() << "; they must match"
                       << std::endl;
            return 1;
        }
    } else {
        for (std::int32_t id : body_ids) {
            body_names.push_back("Body" + std::to_string(id));
        }
    }

    const mpc::NatNetVersion stream_version = parse_natnet_version(opt.natnet_version_str, {3, 1});
    const mpc::NatNetVersion served_version =
        opt.served_version_str.empty() ? stream_version
                                        : parse_natnet_version(opt.served_version_str, stream_version);

    std::signal(SIGINT, handle_shutdown_signal);
    std::signal(SIGTERM, handle_shutdown_signal);

    {
        std::string names_joined;
        for (std::size_t i = 0; i < body_names.size(); ++i) {
            if (i != 0) names_joined += ",";
            names_joined += body_names[i];
        }
        std::cout << "[FAKE_MOTIVE] Launching: target=" << opt.target_ip << ":" << opt.target_port
                   << (opt.multicast_target ? " (multicast)" : " (unicast)") << " rate=" << opt.rate_hz
                   << "Hz natnet_version=" << stream_version.major << "." << stream_version.minor
                   << " bodies=" << opt.body_ids_str << " (" << names_joined << ")"
                   << " y_up=" << (opt.y_up ? "yes" : "no")
                   << " drop_tracking_every=" << opt.drop_tracking_every_s << "s"
                   << " serve_command_port=" << opt.serve_command_port << std::endl;
    }

    int send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (send_sock < 0) {
        std::cerr << "[FAKE_MOTIVE] Failed to create send socket: " << std::strerror(errno)
                   << std::endl;
        return 1;
    }
    sockaddr_in target_addr{};
    if (!make_sockaddr(opt.target_ip, static_cast<std::uint16_t>(opt.target_port), target_addr)) {
        std::cerr << "[FAKE_MOTIVE] Invalid --target-ip '" << opt.target_ip << "'" << std::endl;
        close(send_sock);
        return 1;
    }

    // Round 2: --multicast-target configures the send socket so that
    // sendto()-ing a multicast group address (e.g. --target-ip
    // 239.255.42.99) actually reaches a loopback-bound subscriber:
    //   IP_MULTICAST_IF pins the OUTGOING interface to loopback (127.0.0.1)
    //   -- on a multi-interface machine the kernel might otherwise pick a
    //   different default route for the multicast send.
    //   IP_MULTICAST_LOOP=1 (the Linux default already, but set explicitly
    //   for portability/clarity) is what makes a multicast datagram sent
    //   from THIS host visible to a receiver ALSO bound on this same host
    //   -- without it, a loopback multicast test would silently see nothing.
    //   IP_MULTICAST_TTL=1 keeps it off the wider network, appropriate for
    //   a loopback-only test tool.
    if (opt.multicast_target) {
        in_addr iface{};
        if (inet_pton(AF_INET, "127.0.0.1", &iface) != 1) {
            std::cerr << "[FAKE_MOTIVE] Failed to construct loopback IP_MULTICAST_IF address"
                       << std::endl;
            close(send_sock);
            return 1;
        }
        if (setsockopt(send_sock, IPPROTO_IP, IP_MULTICAST_IF, &iface, sizeof(iface)) != 0) {
            std::cerr << "[FAKE_MOTIVE] Warning: setsockopt(IP_MULTICAST_IF) failed: "
                       << std::strerror(errno) << std::endl;
        }
        unsigned char loop = 1;
        if (setsockopt(send_sock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) != 0) {
            std::cerr << "[FAKE_MOTIVE] Warning: setsockopt(IP_MULTICAST_LOOP) failed: "
                       << std::strerror(errno) << std::endl;
        }
        unsigned char ttl = 1;
        if (setsockopt(send_sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) != 0) {
            std::cerr << "[FAKE_MOTIVE] Warning: setsockopt(IP_MULTICAST_TTL) failed: "
                       << std::strerror(errno) << std::endl;
        }
        std::cout << "[FAKE_MOTIVE] Configured send socket for multicast (IP_MULTICAST_IF=127.0.0.1, "
                     "IP_MULTICAST_LOOP=1, IP_MULTICAST_TTL=1)"
                  << std::endl;
    }

    int command_sock = -1;
    if (opt.serve_command_port > 0) {
        command_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (command_sock < 0) {
            std::cerr << "[FAKE_MOTIVE] Failed to create command socket: " << std::strerror(errno)
                       << std::endl;
            close(send_sock);
            return 1;
        }
        int reuse = 1;
        setsockopt(command_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_addr.s_addr = INADDR_ANY;
        bind_addr.sin_port = htons(static_cast<std::uint16_t>(opt.serve_command_port));
        if (bind(command_sock, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) != 0) {
            std::cerr << "[FAKE_MOTIVE] Failed to bind command port " << opt.serve_command_port
                       << ": " << std::strerror(errno) << std::endl;
            close(send_sock);
            close(command_sock);
            return 1;
        }
        std::cout << "[FAKE_MOTIVE] Serving NAT_PING on command port " << opt.serve_command_port
                   << " as app=\"" << opt.served_app_name << "\" NatNet=" << served_version.major
                   << "." << served_version.minor << std::endl;
    }

    const auto t_start = std::chrono::steady_clock::now();
    const auto tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / opt.rate_hz));
    auto next_tick_time = t_start;

    // Round 4: pre-built NAT_MODELDEF reply -- one rigid body per --body-ids
    // entry, named per --body-names, no markersets/skeletons (v1's fake
    // data has no use for them; the PARSER's own handling of those dataset
    // types is covered directly by the unit tests' round-trip fixtures).
    // Built using served_version (the version-discovery-reply version),
    // matching how a real Motive server's modeldef layout would follow
    // whatever NatNet version it actually is.
    std::vector<mpc::RigidBodyBuildDef> modeldef_rigid_bodies;
    for (std::size_t i = 0; i < body_ids.size(); ++i) {
        mpc::RigidBodyBuildDef rb;
        rb.def.id = body_ids[i];
        rb.def.name = body_names[i];
        rb.def.parent_id = 0;
        modeldef_rigid_bodies.push_back(rb);
    }
    const std::vector<std::uint8_t> modeldef_reply =
        mpc::build_modeldef({}, modeldef_rigid_bodies, {}, served_version);

    std::int32_t frame_number = 0;
    long frames_sent = 0;
    long pings_answered = 0;
    long modeldef_requests_answered = 0;
    std::vector<std::uint8_t> recv_buf(4096);

    // Round 5: --modeldef-delay-s pending-response state (see Options'
    // doc comment). Only one request in flight is tracked -- a fresh
    // request simply overwrites/reschedules it, which is fine for this
    // tool's test-support purpose.
    bool pending_modeldef = false;
    std::chrono::steady_clock::time_point pending_modeldef_deadline{};
    sockaddr_in pending_modeldef_addr{};
    socklen_t pending_modeldef_addr_len = 0;

    while (!g_shutdown_requested) {
        auto now = std::chrono::steady_clock::now();

        if (now >= next_tick_time) {
            const double t = std::chrono::duration<double>(now - t_start).count();
            std::vector<mpc::RigidBodySample> bodies;
            bodies.reserve(body_ids.size());
            for (int i = 0; i < static_cast<int>(body_ids.size()); ++i) {
                const bool valid = tracking_valid_for(i, t, opt.drop_tracking_every_s);
                bodies.push_back(compute_body_pose(t, body_ids[static_cast<size_t>(i)], i, opt.y_up,
                                                     valid));
            }
            std::vector<std::uint8_t> packet =
                mpc::build_frame_of_data(frame_number, bodies, stream_version);
            sendto(send_sock, packet.data(), packet.size(), 0,
                    reinterpret_cast<sockaddr*>(&target_addr), sizeof(target_addr));
            ++frame_number;
            ++frames_sent;

            next_tick_time += tick_period;
            if (next_tick_time < now) {
                next_tick_time = now;  // fell behind -- resync, mirrors robot_sim.cpp's pacing.
            }
            now = std::chrono::steady_clock::now();
        }

        if (opt.duration_s > 0.0 &&
            std::chrono::duration<double>(now - t_start).count() >= opt.duration_s) {
            std::cout << "[FAKE_MOTIVE] Reached --duration-s=" << opt.duration_s << "; stopping."
                       << std::endl;
            break;
        }

        // Wait until the next scheduled send (capped at 20ms slices), using
        // poll() on the command socket (when enabled) so an incoming NAT_PING
        // is serviced with low latency instead of a separate busy loop.
        auto remaining = next_tick_time - std::chrono::steady_clock::now();
        long wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count();
        if (wait_ms < 0) wait_ms = 0;
        if (wait_ms > 20) wait_ms = 20;

        if (command_sock >= 0) {
            pollfd pfd{};
            pfd.fd = command_sock;
            pfd.events = POLLIN;
            int rc = poll(&pfd, 1, static_cast<int>(wait_ms));
            if (rc > 0 && (pfd.revents & POLLIN)) {
                sockaddr_in src_addr{};
                socklen_t src_len = sizeof(src_addr);
                ssize_t n = recvfrom(command_sock, recv_buf.data(), recv_buf.size(), 0,
                                       reinterpret_cast<sockaddr*>(&src_addr), &src_len);
                if (n > 0) {
                    mpc::PacketHeader hdr = mpc::peek_packet_header(recv_buf.data(),
                                                                       static_cast<std::size_t>(n));
                    if (hdr.ok && hdr.message_id == mpc::kNatPing) {
                        std::array<std::uint8_t, 4> app_version{{1, 0, 0, 0}};
                        std::array<std::uint8_t, 4> natnet_version{
                            {static_cast<std::uint8_t>(served_version.major),
                             static_cast<std::uint8_t>(served_version.minor), 0, 0}};
                        std::vector<std::uint8_t> resp =
                            mpc::build_ping_response(opt.served_app_name, app_version, natnet_version);
                        sendto(command_sock, resp.data(), resp.size(), 0,
                                reinterpret_cast<sockaddr*>(&src_addr), src_len);
                        ++pings_answered;
                    } else if (hdr.ok && hdr.message_id == mpc::kNatRequestModelDef) {
                        if (opt.modeldef_delay_s <= 0.0) {
                            sendto(command_sock, modeldef_reply.data(), modeldef_reply.size(), 0,
                                    reinterpret_cast<sockaddr*>(&src_addr), src_len);
                            ++modeldef_requests_answered;
                        } else {
                            pending_modeldef = true;
                            pending_modeldef_deadline = std::chrono::steady_clock::now() +
                                                          std::chrono::duration_cast<
                                                              std::chrono::steady_clock::duration>(
                                                              std::chrono::duration<double>(
                                                                  opt.modeldef_delay_s));
                            pending_modeldef_addr = src_addr;
                            pending_modeldef_addr_len = src_len;
                        }
                    }
                }
            }
        } else if (wait_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        }

        // Round 5: fire a delayed NAT_MODELDEF reply once its deadline has
        // passed (checked every loop iteration, which wakes at least every
        // 20ms via the wait_ms cap above even with no incoming traffic).
        if (pending_modeldef && std::chrono::steady_clock::now() >= pending_modeldef_deadline) {
            sendto(command_sock, modeldef_reply.data(), modeldef_reply.size(), 0,
                    reinterpret_cast<sockaddr*>(&pending_modeldef_addr), pending_modeldef_addr_len);
            ++modeldef_requests_answered;
            pending_modeldef = false;
        }
    }

    std::cout << "[FAKE_MOTIVE] Shutting down. frames_sent=" << frames_sent
               << " pings_answered=" << pings_answered
               << " modeldef_requests_answered=" << modeldef_requests_answered << std::endl;

    close(send_sock);
    if (command_sock >= 0) close(command_sock);
    return 0;
}
