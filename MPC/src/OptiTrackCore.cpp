#include "mpc/OptiTrackCore.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <sstream>

namespace mpc {

namespace {

// ---------------------------------------------------------------------
// Little-endian byte <-> integer/float helpers. Implemented via explicit
// byte shifts/memcpy rather than reinterpret_cast of a raw struct, so
// parsing/building is correct regardless of host byte order (NatNet is
// little-endian throughout; see OptiTrackCore.h's header doc comment).
// ---------------------------------------------------------------------

std::uint16_t read_u16_le(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8);
}

std::uint32_t read_u32_le(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::int32_t read_i32_le(const std::uint8_t* p) {
    return static_cast<std::int32_t>(read_u32_le(p));
}

float read_f32_le(const std::uint8_t* p) {
    std::uint32_t bits = read_u32_le(p);
    float f = 0.0f;
    static_assert(sizeof(f) == sizeof(bits), "float must be 32 bits");
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

void write_u16_le(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xFF));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
}

void write_u32_le(std::vector<std::uint8_t>& out, std::uint32_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xFF));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xFF));
}

void write_i32_le(std::vector<std::uint8_t>& out, std::int32_t v) {
    write_u32_le(out, static_cast<std::uint32_t>(v));
}

void write_f32_le(std::vector<std::uint8_t>& out, float v) {
    std::uint32_t bits = 0;
    static_assert(sizeof(v) == sizeof(bits), "float must be 32 bits");
    std::memcpy(&bits, &v, sizeof(bits));
    write_u32_le(out, bits);
}

// ---------------------------------------------------------------------
// Bounds-checked cursor over a byte buffer. Every read checks available
// space FIRST; on failure it sets a sticky `ok_ = false` and returns a
// harmless default (0 / empty string) WITHOUT advancing past the buffer, so
// a caller that keeps reading after the first failure never reads out of
// bounds -- it just keeps getting zeros until it checks ok().
// ---------------------------------------------------------------------
class Reader {
   public:
    Reader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size), limit_(size) {}

    bool ok() const { return ok_; }
    std::size_t offset() const { return offset_; }
    // Explicitly marks this reader as failed -- for callers that detect an
    // implausible declared count (e.g. a marker count that would overrun
    // the buffer) BEFORE any single read naturally trips can_read().
    void fail() { ok_ = false; }

    // Caps all subsequent reads at `limit` bytes from the START of the
    // buffer (clamped to the buffer's own size) -- used to confine parsing
    // to a packet's declared payloadSize even if the underlying buffer is
    // longer (e.g. a caller's fixed-size recv buffer).
    void set_limit(std::size_t limit) { limit_ = std::min(limit, size_); }

    bool can_read(std::size_t n) const {
        if (!ok_) return false;
        // offset_ <= limit_ always holds here; guard the addition against
        // wraparound for a pathological huge `n` (e.g. derived from a
        // corrupted count field cast to size_t).
        return n <= limit_ - offset_;
    }

    std::uint16_t u16() {
        if (!can_read(2)) {
            ok_ = false;
            return 0;
        }
        std::uint16_t v = read_u16_le(data_ + offset_);
        offset_ += 2;
        return v;
    }

    std::int32_t i32() {
        if (!can_read(4)) {
            ok_ = false;
            return 0;
        }
        std::int32_t v = read_i32_le(data_ + offset_);
        offset_ += 4;
        return v;
    }

    float f32() {
        if (!can_read(4)) {
            ok_ = false;
            return 0.0f;
        }
        float v = read_f32_le(data_ + offset_);
        offset_ += 4;
        return v;
    }

    // Reads a NUL-terminated ASCII string. ok_ becomes false (and the
    // returned string is empty) if no NUL terminator is found within the
    // current limit.
    std::string cstr() {
        if (!ok_) return {};
        std::size_t i = offset_;
        while (i < limit_ && data_[i] != 0) ++i;
        if (i >= limit_) {
            ok_ = false;
            return {};
        }
        std::string s(reinterpret_cast<const char*>(data_ + offset_), i - offset_);
        offset_ = i + 1;  // skip the NUL terminator.
        return s;
    }

    // Reads exactly `n` raw bytes as a std::string (used for the fixed-width
    // 256-byte app-name field, which may itself contain an embedded NUL
    // followed by padding -- the caller trims at the first NUL afterward).
    std::string raw(std::size_t n) {
        if (!can_read(n)) {
            ok_ = false;
            return {};
        }
        std::string s(reinterpret_cast<const char*>(data_ + offset_), n);
        offset_ += n;
        return s;
    }

    void skip(std::size_t n) {
        if (!can_read(n)) {
            ok_ = false;
            return;
        }
        offset_ += n;
    }

   private:
    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t limit_;
    std::size_t offset_ = 0;
    bool ok_ = true;
};

}  // namespace

PacketHeader peek_packet_header(const std::uint8_t* data, std::size_t size) {
    PacketHeader h;
    if (data == nullptr || size < 4) {
        return h;
    }
    h.message_id = read_u16_le(data);
    h.payload_size = read_u16_le(data + 2);
    if (4 + static_cast<std::size_t>(h.payload_size) > size) {
        return h;  // declared payload overruns the actual buffer.
    }
    h.ok = true;
    return h;
}

// ---------------------------------------------------------------------
// NAT_PING / NAT_PINGRESPONSE.
// ---------------------------------------------------------------------

std::vector<std::uint8_t> build_ping_packet(const std::string& app_name) {
    std::vector<std::uint8_t> out;
    write_u16_le(out, kNatPing);
    write_u16_le(out, static_cast<std::uint16_t>(app_name.size()));
    out.insert(out.end(), app_name.begin(), app_name.end());
    return out;
}

std::vector<std::uint8_t> build_ping_response(const std::string& app_name,
                                               const std::array<std::uint8_t, 4>& app_version,
                                               const std::array<std::uint8_t, 4>& natnet_version) {
    constexpr std::size_t kNameFieldSize = 256;
    std::vector<std::uint8_t> out;
    write_u16_le(out, kNatPingResponse);
    write_u16_le(out, static_cast<std::uint16_t>(kNameFieldSize + 4 + 4));

    const std::size_t copy_len = std::min(app_name.size(), kNameFieldSize);
    for (std::size_t i = 0; i < copy_len; ++i) {
        out.push_back(static_cast<std::uint8_t>(app_name[i]));
    }
    for (std::size_t i = copy_len; i < kNameFieldSize; ++i) {
        out.push_back(0);
    }
    for (std::uint8_t b : app_version) out.push_back(b);
    for (std::uint8_t b : natnet_version) out.push_back(b);
    return out;
}

PingResponse parse_command_response(const std::uint8_t* data, std::size_t size) {
    PingResponse out;
    PacketHeader hdr = peek_packet_header(data, size);
    if (!hdr.ok) {
        out.error = "truncated or invalid packet header";
        return out;
    }
    if (hdr.message_id != kNatPingResponse) {
        out.error = "not a NAT_PINGRESPONSE packet (messageID=" + std::to_string(hdr.message_id) +
                    ")";
        return out;
    }

    Reader r(data, size);
    r.skip(4);  // header already validated above.
    r.set_limit(4 + static_cast<std::size_t>(hdr.payload_size));

    constexpr std::size_t kNameFieldSize = 256;
    std::string raw_name = r.raw(kNameFieldSize);
    if (!r.ok()) {
        out.error = "truncated app name field";
        return out;
    }
    // Trim at the first embedded NUL (the field is NUL-padded, not
    // necessarily fully used).
    auto nul_pos = raw_name.find('\0');
    out.app_name = (nul_pos == std::string::npos) ? raw_name : raw_name.substr(0, nul_pos);

    const std::string app_version_bytes = r.raw(4);
    const std::string natnet_version_bytes = r.raw(4);
    if (!r.ok()) {
        out.error = "truncated version fields";
        return PingResponse{};
    }
    for (std::size_t i = 0; i < 4; ++i) {
        out.app_version[i] = static_cast<std::uint8_t>(app_version_bytes[i]);
        out.natnet_version[i] = static_cast<std::uint8_t>(natnet_version_bytes[i]);
    }
    out.ok = true;
    return out;
}

// ---------------------------------------------------------------------
// NAT_FRAMEOFDATA.
// ---------------------------------------------------------------------

FrameOfData parse_frame_of_data(const std::uint8_t* data, std::size_t size,
                                 const NatNetVersion& version) {
    FrameOfData out;

    PacketHeader hdr = peek_packet_header(data, size);
    if (!hdr.ok) {
        out.error = "truncated or invalid packet header";
        return out;
    }
    if (hdr.message_id != kNatFrameOfData) {
        out.error = "not a NAT_FRAMEOFDATA packet (messageID=" + std::to_string(hdr.message_id) +
                    ")";
        return out;
    }

    Reader r(data, size);
    r.skip(4);
    r.set_limit(4 + static_cast<std::size_t>(hdr.payload_size));

    out.frame_number = r.i32();

    std::int32_t n_marker_sets = r.i32();
    if (!r.ok() || n_marker_sets < 0) {
        out.error = "bad nMarkerSets";
        return out;
    }
    for (std::int32_t i = 0; i < n_marker_sets; ++i) {
        r.cstr();  // marker set name -- discarded (v1 does not use marker sets).
        if (!r.ok()) {
            out.error = "truncated marker set name";
            return out;
        }
        std::int32_t n_markers = r.i32();
        if (!r.ok() || n_markers < 0 ||
            !r.can_read(static_cast<std::size_t>(n_markers) * 12)) {
            out.error = "bad marker set marker count";
            return out;
        }
        r.skip(static_cast<std::size_t>(n_markers) * 12);  // (x,y,z) floats, discarded.
    }
    if (!r.ok()) {
        out.error = "truncated in marker sets";
        return out;
    }

    std::int32_t n_unlabeled = r.i32();
    if (!r.ok() || n_unlabeled < 0 ||
        !r.can_read(static_cast<std::size_t>(n_unlabeled) * 12)) {
        out.error = "bad nUnlabeledMarkers";
        return out;
    }
    r.skip(static_cast<std::size_t>(n_unlabeled) * 12);

    std::int32_t n_rigid_bodies = r.i32();
    if (!r.ok() || n_rigid_bodies < 0) {
        out.error = "bad nRigidBodies";
        return out;
    }
    // Cheap sanity floor before looping: id(4) + pos(12) + quat(16) = 32
    // bytes is the MINIMUM any rigid body occupies (major>=3 also needs
    // +6 for meanError+params; major<3 needs at least +6 more too). This
    // does not need to be exact -- it just rejects an absurd/corrupted
    // count before the loop below does any per-body work.
    if (!r.can_read(static_cast<std::size_t>(n_rigid_bodies) * 32)) {
        out.error = "nRigidBodies implausible for remaining payload";
        return out;
    }

    out.rigid_bodies.reserve(static_cast<std::size_t>(n_rigid_bodies));
    for (std::int32_t i = 0; i < n_rigid_bodies; ++i) {
        RigidBodySample rb;
        rb.id = r.i32();
        rb.x = r.f32();
        rb.y = r.f32();
        rb.z = r.f32();
        rb.qx = r.f32();
        rb.qy = r.f32();
        rb.qz = r.f32();
        rb.qw = r.f32();

        std::uint16_t params = 0;
        if (version.major < 3) {
            std::int32_t n_rigid_markers = r.i32();
            if (!r.ok() || n_rigid_markers < 0 ||
                !r.can_read(static_cast<std::size_t>(n_rigid_markers) * (12 + 4 + 4))) {
                out.error = "bad nRigidMarkers";
                return out;
            }
            r.skip(static_cast<std::size_t>(n_rigid_markers) * 12);  // marker positions.
            r.skip(static_cast<std::size_t>(n_rigid_markers) * 4);   // marker IDs.
            r.skip(static_cast<std::size_t>(n_rigid_markers) * 4);   // marker sizes.
            rb.mean_error = r.f32();
            params = r.u16();  // v1 only supports major==2 at >=2.6, which always has this field.
        } else {
            rb.mean_error = r.f32();
            params = r.u16();
        }
        if (!r.ok()) {
            out.error = "truncated rigid body";
            return out;
        }
        rb.tracking_valid = (params & 0x01) != 0;
        out.rigid_bodies.push_back(rb);
    }

    // v1 stops here -- skeletons/labeled markers/force plates/devices/timing
    // info are deliberately NOT parsed (see header doc comment).
    out.parse_ok = true;
    return out;
}

std::vector<std::uint8_t> build_frame_of_data(std::int32_t frame_number,
                                                const std::vector<RigidBodySample>& rigid_bodies,
                                                const NatNetVersion& version) {
    std::vector<std::uint8_t> payload;
    write_i32_le(payload, frame_number);
    write_i32_le(payload, 0);  // nMarkerSets = 0.
    write_i32_le(payload, 0);  // nUnlabeledMarkers = 0.
    write_i32_le(payload, static_cast<std::int32_t>(rigid_bodies.size()));
    for (const RigidBodySample& rb : rigid_bodies) {
        write_i32_le(payload, rb.id);
        write_f32_le(payload, static_cast<float>(rb.x));
        write_f32_le(payload, static_cast<float>(rb.y));
        write_f32_le(payload, static_cast<float>(rb.z));
        write_f32_le(payload, static_cast<float>(rb.qx));
        write_f32_le(payload, static_cast<float>(rb.qy));
        write_f32_le(payload, static_cast<float>(rb.qz));
        write_f32_le(payload, static_cast<float>(rb.qw));

        const std::uint16_t params = rb.tracking_valid ? 0x01 : 0x00;
        if (version.major < 3) {
            write_i32_le(payload, 0);  // nRigidMarkers = 0 -- see header doc comment.
            write_f32_le(payload, static_cast<float>(rb.mean_error));
            write_u16_le(payload, params);
        } else {
            write_f32_le(payload, static_cast<float>(rb.mean_error));
            write_u16_le(payload, params);
        }
    }

    std::vector<std::uint8_t> out;
    write_u16_le(out, kNatFrameOfData);
    write_u16_le(out, static_cast<std::uint16_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

// ---------------------------------------------------------------------
// NAT_REQUEST_MODELDEF / NAT_MODELDEF.
// ---------------------------------------------------------------------

std::vector<std::uint8_t> build_modeldef_request() {
    std::vector<std::uint8_t> out;
    write_u16_le(out, kNatRequestModelDef);
    write_u16_le(out, 0);  // empty payload.
    return out;
}

namespace {

void write_cstr(std::vector<std::uint8_t>& out, const std::string& s) {
    out.insert(out.end(), s.begin(), s.end());
    out.push_back(0);  // NUL terminator.
}

// Parses ONE rigid-body dataset body (name/id/parentID/offset + the
// version-dependent per-marker section) -- shared by top-level (type=1)
// and nested skeleton (type=2) rigid bodies, which use the identical
// layout. Check r.ok() after calling this to know whether it succeeded.
RigidBodyDef parse_one_rigid_body(Reader& r, const NatNetVersion& version) {
    RigidBodyDef out;
    out.name = r.cstr();
    out.id = r.i32();
    out.parent_id = r.i32();
    r.f32();
    r.f32();
    r.f32();  // offsetX/Y/Z -- discarded, not part of RigidBodyDef.
    if (!r.ok()) return out;
    if (version.major >= 3) {
        const std::int32_t n_markers = r.i32();
        if (!r.ok() || n_markers < 0 ||
            !r.can_read(static_cast<std::size_t>(n_markers) * (12 + 4))) {
            r.fail();
            return out;
        }
        for (std::int32_t i = 0; i < n_markers; ++i) {
            r.f32();
            r.f32();
            r.f32();
        }  // marker offsets.
        for (std::int32_t i = 0; i < n_markers; ++i) {
            r.i32();
        }  // active labels.
        if (version.major >= 4) {
            for (std::int32_t i = 0; i < n_markers; ++i) {
                r.cstr();  // marker name (v4.x only).
                if (!r.ok()) return out;
            }
        }
    }
    return out;
}

void write_rigid_body_build(std::vector<std::uint8_t>& payload, const RigidBodyBuildDef& rb,
                              const NatNetVersion& version) {
    write_cstr(payload, rb.def.name);
    write_i32_le(payload, rb.def.id);
    write_i32_le(payload, rb.def.parent_id);
    write_f32_le(payload, 0.0f);  // offsetX -- not part of RigidBodyDef, always written as 0.
    write_f32_le(payload, 0.0f);  // offsetY.
    write_f32_le(payload, 0.0f);  // offsetZ.
    if (version.major >= 3) {
        write_i32_le(payload, static_cast<std::int32_t>(rb.marker_offsets.size()));
        for (const std::array<float, 3>& off : rb.marker_offsets) {
            write_f32_le(payload, off[0]);
            write_f32_le(payload, off[1]);
            write_f32_le(payload, off[2]);
        }
        for (std::int32_t label : rb.marker_active_labels) {
            write_i32_le(payload, label);
        }
        if (version.major >= 4) {
            for (const std::string& name : rb.marker_names) {
                write_cstr(payload, name);
            }
        }
    }
}

}  // namespace

ModelDef parse_modeldef(const std::uint8_t* data, std::size_t size, const NatNetVersion& version) {
    ModelDef out;

    PacketHeader hdr = peek_packet_header(data, size);
    if (!hdr.ok) {
        out.error = "truncated or invalid packet header";
        return out;
    }
    if (hdr.message_id != kNatModelDef) {
        out.error = "not a NAT_MODELDEF packet (messageID=" + std::to_string(hdr.message_id) + ")";
        return out;
    }

    Reader r(data, size);
    r.skip(4);
    r.set_limit(4 + static_cast<std::size_t>(hdr.payload_size));

    const std::int32_t n_datasets = r.i32();
    if (!r.ok() || n_datasets < 0) {
        out.error = "bad nDatasets";
        return out;
    }

    for (std::int32_t i = 0; i < n_datasets; ++i) {
        const std::int32_t type = r.i32();
        if (!r.ok()) {
            out.error = "truncated dataset type";
            return out;
        }

        if (type == 0) {
            // Markerset -- parse-and-skip (never surfaced).
            r.cstr();  // name.
            const std::int32_t n_markers = r.i32();
            if (!r.ok() || n_markers < 0) {
                out.error = "bad markerset nMarkers";
                return out;
            }
            for (std::int32_t m = 0; m < n_markers; ++m) {
                r.cstr();  // marker name.
                if (!r.ok()) {
                    out.error = "truncated markerset marker name";
                    return out;
                }
            }
        } else if (type == 1) {
            RigidBodyDef rb = parse_one_rigid_body(r, version);
            if (!r.ok()) {
                out.error = "truncated rigid body dataset";
                return out;
            }
            out.rigid_bodies.push_back(rb);
        } else if (type == 2) {
            r.cstr();  // skeleton name.
            r.i32();   // skeleton id.
            const std::int32_t n_rb = r.i32();
            if (!r.ok() || n_rb < 0) {
                out.error = "bad skeleton nRigidBodies";
                return out;
            }
            for (std::int32_t k = 0; k < n_rb; ++k) {
                parse_one_rigid_body(r, version);  // nested -- parse-and-skip.
                if (!r.ok()) {
                    out.error = "truncated nested skeleton rigid body";
                    return out;
                }
            }
        } else {
            // Unrecognized dataset type -- stop GRACEFULLY, keep what was
            // already parsed (see ModelDef::stopped_early doc comment).
            out.stopped_early = true;
            out.parse_ok = true;
            out.error = "stopped at unrecognized dataset type " + std::to_string(type) +
                        " (dataset index " + std::to_string(i) + "/" + std::to_string(n_datasets) + ")";
            return out;
        }
    }

    out.parse_ok = true;
    return out;
}

std::vector<std::uint8_t> build_modeldef(const std::vector<MarkerSetDef>& marker_sets,
                                           const std::vector<RigidBodyBuildDef>& rigid_bodies,
                                           const std::vector<SkeletonBuildDef>& skeletons,
                                           const NatNetVersion& version) {
    std::vector<std::uint8_t> payload;
    const std::int32_t n_datasets =
        static_cast<std::int32_t>(marker_sets.size() + rigid_bodies.size() + skeletons.size());
    write_i32_le(payload, n_datasets);

    for (const MarkerSetDef& ms : marker_sets) {
        write_i32_le(payload, 0);  // type = markerset.
        write_cstr(payload, ms.name);
        write_i32_le(payload, static_cast<std::int32_t>(ms.marker_names.size()));
        for (const std::string& name : ms.marker_names) {
            write_cstr(payload, name);
        }
    }
    for (const RigidBodyBuildDef& rb : rigid_bodies) {
        write_i32_le(payload, 1);  // type = rigid body.
        write_rigid_body_build(payload, rb, version);
    }
    for (const SkeletonBuildDef& sk : skeletons) {
        write_i32_le(payload, 2);  // type = skeleton.
        write_cstr(payload, sk.name);
        write_i32_le(payload, sk.id);
        write_i32_le(payload, static_cast<std::int32_t>(sk.nested_rigid_bodies.size()));
        for (const RigidBodyBuildDef& nested : sk.nested_rigid_bodies) {
            write_rigid_body_build(payload, nested, version);
        }
    }

    std::vector<std::uint8_t> out;
    write_u16_le(out, kNatModelDef);
    write_u16_le(out, static_cast<std::uint16_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

// ---------------------------------------------------------------------
// Up-axis-aware orientation/position conversion.
// ---------------------------------------------------------------------

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double quat_to_planar_yaw(double qx, double qy, double qz, double qw, bool y_up) {
    // v = R(q) * (1,0,0)^T -- see header doc comment for the derivation.
    const double vx = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double vy = 2.0 * (qx * qy + qz * qw);
    const double vz = 2.0 * (qx * qz - qy * qw);

    if (!y_up) {
        return normalize_angle(std::atan2(vy, vx));
    }
    // y_up: Motive's ground plane is XZ; map (x_m, z_m) -> (x, -z_m) (see
    // header doc comment) applied to the ground components (vx, vz).
    return normalize_angle(std::atan2(-vz, vx));
}

PlanarPose mocap_rotation(double x_m, double /*y_m*/, double z_m, double qx, double qy, double qz,
                           double qw) {
    // See header doc comment: x_p = x_m, y_p = -z_m (y_m, the height, is
    // dropped -- accepted only for API completeness), yaw = the existing
    // y_up=true quat_to_planar_yaw() branch, unchanged/bit-for-bit.
    PlanarPose pose;
    pose.x = x_m;
    pose.y = -z_m;
    pose.yaw = quat_to_planar_yaw(qx, qy, qz, qw, /*y_up=*/true);
    return pose;
}

PlanarPose extract_planar_pose(const RigidBodySample& sample, bool y_up) {
    if (!y_up) {
        PlanarPose pose;
        pose.x = sample.x;
        pose.y = sample.y;
        pose.yaw = quat_to_planar_yaw(sample.qx, sample.qy, sample.qz, sample.qw, false);
        return pose;
    }
    return mocap_rotation(sample.x, sample.y, sample.z, sample.qx, sample.qy, sample.qz,
                           sample.qw);
}

PlanarPose apply_planar_transform(const PlanarPose& pose, double x0, double y0, double theta0) {
    const double c = std::cos(theta0);
    const double s = std::sin(theta0);
    PlanarPose out;
    out.x = x0 + pose.x * c - pose.y * s;
    out.y = y0 + pose.x * s + pose.y * c;
    out.yaw = normalize_angle(pose.yaw + theta0);
    return out;
}

// ---------------------------------------------------------------------
// Mocap-to-world affine transform.
// ---------------------------------------------------------------------

MocapToWorldParseResult parse_mocap_to_world_matrix(const std::vector<std::vector<double>>& rows) {
    MocapToWorldParseResult out;

    if (rows.size() != 3) {
        out.error =
            "mocap_to_world_matrix must have exactly 3 rows, got " + std::to_string(rows.size());
        return out;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        if (rows[i].size() != 3) {
            out.error = "mocap_to_world_matrix row " + std::to_string(i) +
                        " must have exactly 3 entries, got " + std::to_string(rows[i].size());
            return out;
        }
    }

    constexpr double kBottomRowTol = 1e-6;
    if (std::fabs(rows[2][0]) > kBottomRowTol || std::fabs(rows[2][1]) > kBottomRowTol ||
        std::fabs(rows[2][2] - 1.0) > kBottomRowTol) {
        out.error = "mocap_to_world_matrix bottom row must be [0,0,1] (within 1e-6), got [" +
                    std::to_string(rows[2][0]) + "," + std::to_string(rows[2][1]) + "," +
                    std::to_string(rows[2][2]) + "]";
        return out;
    }

    AffineTransform2D t;
    t.a = rows[0][0];
    t.b = rows[0][1];
    t.tx = rows[0][2];
    t.c = rows[1][0];
    t.d = rows[1][1];
    t.ty = rows[1][2];
    out.transform = t;
    out.ok = true;

    // Orthonormality + det(R)~=+1 check -- see header doc comment. A
    // failure here is a WARNING only; `out.ok` stays true and the affine
    // transform is still returned/usable as-is.
    constexpr double kRigidTol = 1e-3;
    const double col1_norm_sq = t.a * t.a + t.c * t.c;
    const double col2_norm_sq = t.b * t.b + t.d * t.d;
    const double col_dot = t.a * t.b + t.c * t.d;
    const double det = t.a * t.d - t.b * t.c;
    if (std::fabs(col1_norm_sq - 1.0) > kRigidTol || std::fabs(col2_norm_sq - 1.0) > kRigidTol ||
        std::fabs(col_dot) > kRigidTol || std::fabs(det - 1.0) > kRigidTol) {
        out.non_rigid_warning = true;
        out.warning =
            "mocap_to_world_matrix's rotation block [[a,b],[c,d]] is not a pure rotation "
            "(non-orthonormal and/or det!=+1) -- scale/shear present; positions get the full "
            "affine transform, yaw still uses atan2(c,a) as an approximation";
    }
    return out;
}

PlanarPose apply_mocap_to_world(const PlanarPose& pose, const AffineTransform2D& transform) {
    PlanarPose out;
    out.x = transform.a * pose.x + transform.b * pose.y + transform.tx;
    out.y = transform.c * pose.x + transform.d * pose.y + transform.ty;
    out.yaw = normalize_angle(std::atan2(transform.c, transform.a) + pose.yaw);
    return out;
}

AffineTransform2D affine_from_xytheta(double x0, double y0, double theta0) {
    AffineTransform2D t;
    const double cs = std::cos(theta0);
    const double sn = std::sin(theta0);
    t.a = cs;
    t.b = -sn;
    t.tx = x0;
    t.c = sn;
    t.d = cs;
    t.ty = y0;
    return t;
}

// ---------------------------------------------------------------------
// Full 6-DoF Euler decomposition.
// ---------------------------------------------------------------------

EulerRollPitchHeading quat_to_roll_pitch_heading(double qx, double qy, double qz, double qw,
                                                    bool y_up) {
    EulerRollPitchHeading out;
    auto clamp_unit = [](double v) { return std::max(-1.0, std::min(1.0, v)); };

    if (!y_up) {
        // Standard ZYX (aerospace) Tait-Bryan extraction -- see header doc
        // comment. heading matches quat_to_planar_yaw's z_up branch exactly.
        out.heading = normalize_angle(std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
        out.pitch = std::asin(clamp_unit(2.0 * (qw * qy - qz * qx)));
        out.roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    } else {
        // y_up: apply the SAME ZYX formulas to the quaternion substitution
        // (qw,qx,qy,qz) -> (qw,qx,-qz,qy) -- see header doc comment for the
        // conjugation derivation. heading matches quat_to_planar_yaw's
        // y_up branch exactly.
        out.heading = normalize_angle(std::atan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz)));
        out.pitch = std::asin(clamp_unit(-2.0 * (qw * qz + qx * qy)));
        out.roll = std::atan2(2.0 * (qw * qx - qy * qz), 1.0 - 2.0 * (qx * qx + qz * qz));
    }
    return out;
}

// ---------------------------------------------------------------------
// Mocap streaming-settings config file.
// ---------------------------------------------------------------------

namespace {

std::string trim(const std::string& s) {
    const std::size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    const std::size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

std::string to_lower_copy(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

}  // namespace

MocapConfigResult parse_mocap_config_text(const std::string& text) {
    MocapConfigResult out;
    std::istringstream stream(text);
    std::string raw_line;
    int line_number = 0;

    while (std::getline(stream, raw_line)) {
        ++line_number;
        // getline() already splits on '\n'; strip a trailing '\r' too, so
        // CRLF-terminated files (Motive is Windows software) work the same
        // as LF-terminated ones.
        if (!raw_line.empty() && raw_line.back() == '\r') {
            raw_line.pop_back();
        }
        const std::string line = trim(raw_line);
        if (line.empty()) {
            continue;  // blank line -- tolerated silently.
        }

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) {
            out.diagnostics.push_back(
                {line_number, "malformed line (missing ':'): '" + line + "'", true});
            continue;
        }
        const std::string key_raw = trim(line.substr(0, colon));  // original case, for diagnostics.
        const std::string key = to_lower_copy(key_raw);            // normalized, for matching.
        const std::string value = trim(line.substr(colon + 1));

        if (key == "ip_address") {
            out.server_ip = value;
            out.has_server_ip = true;
            out.diagnostics.push_back({line_number, "server IP = '" + value + "'", false});
        } else if (key == "type") {
            const std::string lowered = to_lower_copy(value);
            if (lowered == "multicast") {
                out.mode = "multicast";
                out.has_mode = true;
                out.diagnostics.push_back({line_number, "mode = multicast", false});
            } else if (lowered == "unicast") {
                out.mode = "unicast";
                out.has_mode = true;
                out.diagnostics.push_back({line_number, "mode = unicast", false});
            } else {
                out.diagnostics.push_back(
                    {line_number,
                     "unknown 'Type' value '" + value + "' (expected Unicast/Multicast), ignoring",
                     true});
            }
        } else if (key == "command port") {
            try {
                std::size_t consumed = 0;
                const int port = std::stoi(value, &consumed);
                if (consumed != value.size()) throw std::invalid_argument("trailing characters");
                out.command_port = port;
                out.has_command_port = true;
                out.diagnostics.push_back({line_number, "command port = " + std::to_string(port), false});
            } catch (const std::exception&) {
                out.diagnostics.push_back(
                    {line_number, "invalid 'Command Port' value '" + value + "', ignoring", true});
            }
        } else if (key == "data port") {
            try {
                std::size_t consumed = 0;
                const int port = std::stoi(value, &consumed);
                if (consumed != value.size()) throw std::invalid_argument("trailing characters");
                out.data_port = port;
                out.has_data_port = true;
                out.diagnostics.push_back({line_number, "data port = " + std::to_string(port), false});
            } catch (const std::exception&) {
                out.diagnostics.push_back(
                    {line_number, "invalid 'Data Port' value '" + value + "', ignoring", true});
            }
        } else if (key == "multicast interface") {
            // See this function's doc comment in OptiTrackCore.h: Motive's
            // UI mislabels this field -- it is the multicast GROUP address,
            // not a network interface selector.
            out.multicast_group = value;
            out.has_multicast_group = true;
            out.diagnostics.push_back(
                {line_number,
                 "multicast group = '" + value + "' (from 'Multicast Interface', which Motive mislabels)",
                 false});
        } else {
            out.diagnostics.push_back(
                {line_number, "unknown config key '" + key_raw + "', ignoring", true});
        }
    }

    return out;
}

// ---------------------------------------------------------------------
// Config-file path resolution.
// ---------------------------------------------------------------------

ConfigPathResolution resolve_config_path(const std::string& cli_value, const std::string& default_path,
                                          bool default_exists) {
    ConfigPathResolution result;
    if (!cli_value.empty()) {
        result.path = cli_value;
        result.source = ConfigPathSource::kCli;
        return result;
    }
    if (!default_path.empty() && default_exists) {
        result.path = default_path;
        result.source = ConfigPathSource::kDefault;
        return result;
    }
    result.source = ConfigPathSource::kNone;
    return result;
}

std::string to_string(ConfigPathSource source) {
    switch (source) {
        case ConfigPathSource::kCli: return "cli";
        case ConfigPathSource::kDefault: return "default";
        default: return "none";
    }
}

// ---------------------------------------------------------------------
// NatNet parser-version auto-fallback.
// ---------------------------------------------------------------------

VersionAutoDetector::VersionAutoDetector(std::vector<NatNetVersion> candidates,
                                          int required_consecutive)
    : candidates_(std::move(candidates)), required_consecutive_(required_consecutive) {}

VersionAutoDetector::Outcome VersionAutoDetector::on_parse_failure(const std::uint8_t* data,
                                                                     std::size_t size) {
    Outcome out;
    if (adopted_) {
        return out;  // one-shot: never re-fires adopted=true.
    }

    int winning_index = -1;
    for (std::size_t i = 0; i < candidates_.size(); ++i) {
        FrameOfData f = parse_frame_of_data(data, size, candidates_[i]);
        if (f.parse_ok) {
            winning_index = static_cast<int>(i);
            break;
        }
    }

    if (winning_index < 0) {
        streak_ = 0;
        streak_candidate_index_ = -1;
        return out;
    }

    if (winning_index == streak_candidate_index_) {
        ++streak_;
    } else {
        streak_candidate_index_ = winning_index;
        streak_ = 1;
    }

    if (streak_ >= required_consecutive_) {
        adopted_ = true;
        adopted_version_ = candidates_[static_cast<std::size_t>(winning_index)];
        out.adopted = true;
        out.adopted_version = adopted_version_;
    }
    return out;
}

void VersionAutoDetector::on_parse_success() {
    streak_ = 0;
    streak_candidate_index_ = -1;
}

// ---------------------------------------------------------------------
// Multicast interface auto-selection.
// ---------------------------------------------------------------------

namespace {

bool parse_ipv4(const std::string& s, std::uint32_t& out) {
    std::istringstream iss(s);
    std::string token;
    std::vector<unsigned long> parts;
    while (std::getline(iss, token, '.')) {
        if (token.empty() || token.size() > 3) return false;
        for (char c : token) {
            if (!std::isdigit(static_cast<unsigned char>(c))) return false;
        }
        try {
            std::size_t consumed = 0;
            const unsigned long v = std::stoul(token, &consumed);
            if (consumed != token.size() || v > 255) return false;
            parts.push_back(v);
        } catch (const std::exception&) {
            return false;
        }
    }
    if (parts.size() != 4) return false;
    out = (static_cast<std::uint32_t>(parts[0]) << 24) | (static_cast<std::uint32_t>(parts[1]) << 16) |
          (static_cast<std::uint32_t>(parts[2]) << 8) | static_cast<std::uint32_t>(parts[3]);
    return true;
}

// Counts leading 1-bits of a (assumed well-formed, contiguous) netmask.
int netmask_prefix_length(std::uint32_t netmask) {
    int count = 0;
    for (int bit = 31; bit >= 0; --bit) {
        if ((netmask >> bit) & 0x1u) {
            ++count;
        } else {
            break;
        }
    }
    return count;
}

}  // namespace

std::optional<IfaceInfo> choose_multicast_interface(const std::string& server_ip,
                                                       const std::vector<IfaceInfo>& interfaces) {
    std::uint32_t server_num = 0;
    if (!parse_ipv4(server_ip, server_num)) {
        return std::nullopt;
    }

    const IfaceInfo* best = nullptr;
    int best_prefix = -1;
    for (const IfaceInfo& iface : interfaces) {
        std::uint32_t addr_num = 0, mask_num = 0;
        if (!parse_ipv4(iface.addr, addr_num) || !parse_ipv4(iface.netmask, mask_num)) {
            continue;  // malformed entry -- skip defensively, never throw.
        }
        if ((server_num & mask_num) == (addr_num & mask_num)) {
            const int prefix = netmask_prefix_length(mask_num);
            if (prefix > best_prefix) {
                best_prefix = prefix;
                best = &iface;
            }
        }
    }

    if (best == nullptr) {
        return std::nullopt;
    }
    return *best;
}

// ---------------------------------------------------------------------
// Version confirmation gate.
// ---------------------------------------------------------------------

void VersionGate::confirm_via_ping(const NatNetVersion& version, double elapsed_s) {
    const bool first_time = !confirmed_;
    confirmed_ = true;
    confirmed_via_ping_ = true;  // ping is always authoritative, even on a later "upgrade" call.
    confirmed_version_ = version;
    if (first_time) {
        confirmed_at_s_ = elapsed_s;
        just_confirmed_ = true;
    } else {
        just_confirmed_ = false;  // an upgrade, not a new confirmation event.
    }
}

bool VersionGate::should_parse_now(double elapsed_s, const NatNetVersion& fallback_version) {
    if (confirmed_) {
        just_confirmed_ = false;
        return true;
    }
    if (elapsed_s >= defer_timeout_s_) {
        confirmed_ = true;
        confirmed_via_ping_ = false;
        confirmed_version_ = fallback_version;
        confirmed_at_s_ = elapsed_s;
        just_confirmed_ = true;
        return true;
    }
    just_confirmed_ = false;
    ++deferred_count_;
    return false;
}

// ---------------------------------------------------------------------
// Robot-name -> rigid-body-id resolution.
// ---------------------------------------------------------------------

std::string to_string(RobotIdSource source) {
    switch (source) {
        case RobotIdSource::kCliId: return "cli-id";
        case RobotIdSource::kNameMatch: return "name-match";
        case RobotIdSource::kIndexFallback: return "index-fallback";
        default: return "unresolved";
    }
}

ResolutionResult resolve_robot_ids(const std::vector<std::string>& robot_names,
                                     const std::vector<std::string>& explicit_names,
                                     const std::vector<RigidBodyDef>& modeldef) {
    ResolutionResult out;

    auto find_by_name = [&](const std::string& name) -> const RigidBodyDef* {
        for (const RigidBodyDef& rb : modeldef) {
            if (rb.name == name) return &rb;  // first match wins -- see doc comment.
        }
        return nullptr;
    };
    auto available_names_str = [&]() -> std::string {
        if (modeldef.empty()) return "(none discovered yet)";
        std::string s;
        for (std::size_t i = 0; i < modeldef.size(); ++i) {
            if (i != 0) s += ", ";
            s += "'" + modeldef[i].name + "'";
        }
        return s;
    };

    const bool explicit_mode = !explicit_names.empty();
    for (std::size_t i = 0; i < robot_names.size(); ++i) {
        RobotResolutionEntry entry;
        entry.robot_name = robot_names[i];

        if (explicit_mode) {
            const std::string want = (i < explicit_names.size()) ? explicit_names[i] : std::string();
            const RigidBodyDef* found = find_by_name(want);
            if (found != nullptr) {
                entry.resolved = true;
                entry.rigid_body_id = found->id;
                entry.source = RobotIdSource::kNameMatch;
            } else {
                entry.error = "requested rigid-body name '" + want +
                                "' not found in modeldef; available: " + available_names_str();
            }
        } else {
            // Auto mode: try the robot's own name as a Motive name first.
            const RigidBodyDef* found = find_by_name(robot_names[i]);
            if (found != nullptr) {
                entry.resolved = true;
                entry.rigid_body_id = found->id;
                entry.source = RobotIdSource::kNameMatch;
            } else {
                // Unconditional index fallback -- never blocks on Motive.
                entry.resolved = true;
                entry.rigid_body_id = static_cast<std::int32_t>(i + 1);
                entry.source = RobotIdSource::kIndexFallback;
            }
        }
        out.robots.push_back(entry);
    }

    out.all_resolved = true;
    for (const RobotResolutionEntry& r : out.robots) {
        if (!r.resolved) {
            out.all_resolved = false;
            break;
        }
    }
    return out;
}

// ---------------------------------------------------------------------
// Auto-discovery publish mode.
// ---------------------------------------------------------------------

SanitizeResult sanitize_topic_name(const std::string& name, const std::string& fallback) {
    SanitizeResult out;
    if (name.empty()) {
        out.sanitized = fallback;
        out.used_fallback = true;
        return out;
    }
    std::string result = name;
    bool changed = false;
    for (char& c : result) {
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                        c == '_' || c == '-';
        if (!ok) {
            c = '_';
            changed = true;
        }
    }
    out.sanitized = result;
    out.changed = changed;
    return out;
}

std::vector<AutoDiscoveredBody> resolve_auto_discovery(
    const std::vector<RigidBodyDef>& modeldef,
    const std::unordered_map<std::string, std::string>& aliases) {
    std::vector<AutoDiscoveredBody> out;
    std::unordered_map<std::string, bool> used_names;  // published_name -> (dummy) presence set.

    for (const RigidBodyDef& rb : modeldef) {
        AutoDiscoveredBody body;
        body.rigid_body_id = rb.id;
        body.motive_name = rb.name;

        auto alias_it = aliases.find(rb.name);
        if (alias_it != aliases.end()) {
            body.published_name = alias_it->second;
            body.from_alias = true;
        } else {
            SanitizeResult sr = sanitize_topic_name(rb.name, "body" + std::to_string(rb.id));
            body.published_name = sr.sanitized;
            body.name_sanitized = sr.changed;
            body.used_empty_fallback = sr.used_fallback;
        }

        if (used_names.find(body.published_name) != used_names.end()) {
            // Collides with an earlier body (this call) -- disambiguate
            // deterministically; the FIRST body to claim a name keeps it.
            body.published_name += "_" + std::to_string(rb.id);
            body.disambiguated = true;
        }
        used_names[body.published_name] = true;

        out.push_back(body);
    }
    return out;
}

std::vector<std::string> find_unmatched_aliases(const std::unordered_map<std::string, std::string>& aliases,
                                                   const std::vector<RigidBodyDef>& modeldef) {
    std::vector<std::string> out;
    for (const auto& kv : aliases) {
        bool found = false;
        for (const RigidBodyDef& rb : modeldef) {
            if (rb.name == kv.first) {
                found = true;
                break;
            }
        }
        if (!found) {
            out.push_back(kv.first);
        }
    }
    return out;
}

} // namespace mpc
