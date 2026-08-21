// Lightweight unit tests for mpc:: OptiTrackCore (NatNet parse/build,
// up-axis-aware geometry, Downsampler), mirroring the harness style of
// MPC/tests/mpc_unit_tests.cpp: plain bool test_xxx() functions
// registered in main(), no gtest dependency.

#include "mpc/OptiTrackCore.h"

#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

bool near_eq(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

// float32 round-trips through the wire lose precision beyond ~7 significant
// digits (see OptiTrackCore.h's build_frame_of_data doc comment) -- this
// epsilon is for comparing an original double against a value that has been
// through one float32 encode/decode cycle.
constexpr double kFloatRoundTripEps = 1e-4;

mpc::RigidBodySample make_sample(std::int32_t id, double x, double y, double z, double qx,
                                  double qy, double qz, double qw, double mean_error,
                                  bool tracking_valid) {
    mpc::RigidBodySample s;
    s.id = id;
    s.x = x;
    s.y = y;
    s.z = z;
    s.qx = qx;
    s.qy = qy;
    s.qz = qz;
    s.qw = qw;
    s.mean_error = mean_error;
    s.tracking_valid = tracking_valid;
    return s;
}

bool samples_match(const mpc::RigidBodySample& a, const mpc::RigidBodySample& b,
                    std::ostream& log) {
    bool ok = true;
    if (a.id != b.id) {
        log << "    id mismatch: " << a.id << " vs " << b.id << "\n";
        ok = false;
    }
    if (!near_eq(a.x, b.x, kFloatRoundTripEps) || !near_eq(a.y, b.y, kFloatRoundTripEps) ||
        !near_eq(a.z, b.z, kFloatRoundTripEps)) {
        log << "    position mismatch: (" << a.x << "," << a.y << "," << a.z << ") vs (" << b.x
            << "," << b.y << "," << b.z << ")\n";
        ok = false;
    }
    if (!near_eq(a.qx, b.qx, kFloatRoundTripEps) || !near_eq(a.qy, b.qy, kFloatRoundTripEps) ||
        !near_eq(a.qz, b.qz, kFloatRoundTripEps) || !near_eq(a.qw, b.qw, kFloatRoundTripEps)) {
        log << "    quat mismatch: (" << a.qx << "," << a.qy << "," << a.qz << "," << a.qw
            << ") vs (" << b.qx << "," << b.qy << "," << b.qz << "," << b.qw << ")\n";
        ok = false;
    }
    if (!near_eq(a.mean_error, b.mean_error, kFloatRoundTripEps)) {
        log << "    mean_error mismatch: " << a.mean_error << " vs " << b.mean_error << "\n";
        ok = false;
    }
    if (a.tracking_valid != b.tracking_valid) {
        log << "    tracking_valid mismatch: " << a.tracking_valid << " vs " << b.tracking_valid
            << "\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test: build_frame_of_data -> parse_frame_of_data round trip, for each
// NatNet major version v1 supports, with multiple bodies and a mix of
// tracking valid/invalid flags.
// ---------------------------------------------------------------------
bool test_frame_roundtrip_for_version(const mpc::NatNetVersion& version) {
    std::vector<mpc::RigidBodySample> bodies = {
        make_sample(1, 1.5, -2.25, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0023, true),
        make_sample(2, -0.75, 3.125, 0.0, 0.0, 0.0, 0.70710678, 0.70710678, 0.0091, false),
        make_sample(42, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3, 0.9128709292, 0.0005, true),
    };
    std::vector<std::uint8_t> packet = mpc::build_frame_of_data(777, bodies, version);

    mpc::FrameOfData parsed = mpc::parse_frame_of_data(packet.data(), packet.size(), version);
    bool ok = true;
    if (!parsed.parse_ok) {
        std::cerr << "    parse_ok=false, error='" << parsed.error << "'\n";
        return false;
    }
    if (parsed.frame_number != 777) {
        std::cerr << "    frame_number mismatch: got " << parsed.frame_number << "\n";
        ok = false;
    }
    if (parsed.rigid_bodies.size() != bodies.size()) {
        std::cerr << "    rigid body count mismatch: got " << parsed.rigid_bodies.size()
                   << ", expected " << bodies.size() << "\n";
        return false;
    }
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (!samples_match(bodies[i], parsed.rigid_bodies[i], std::cerr)) {
            std::cerr << "    (body index " << i << ", version " << version.major << "."
                       << version.minor << ")\n";
            ok = false;
        }
    }
    return ok;
}

bool test_frame_roundtrip_v2_10() { return test_frame_roundtrip_for_version({2, 10}); }
bool test_frame_roundtrip_v3_1() { return test_frame_roundtrip_for_version({3, 1}); }
bool test_frame_roundtrip_v4_0() { return test_frame_roundtrip_for_version({4, 0}); }

// ---------------------------------------------------------------------
// Test: every truncation prefix of a valid frame packet must return a
// parse error -- never crash, never spuriously succeed.
// ---------------------------------------------------------------------
bool test_frame_fuzz_truncation() {
    bool ok = true;
    for (const mpc::NatNetVersion& version : {mpc::NatNetVersion{2, 10}, mpc::NatNetVersion{3, 1},
                                                mpc::NatNetVersion{4, 0}}) {
        std::vector<mpc::RigidBodySample> bodies = {
            make_sample(1, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.001, true),
            make_sample(5, -1.0, -2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.002, false),
        };
        std::vector<std::uint8_t> packet = mpc::build_frame_of_data(1, bodies, version);

        // Full-length packet must parse OK (sanity check on the fixture).
        mpc::FrameOfData full = mpc::parse_frame_of_data(packet.data(), packet.size(), version);
        if (!full.parse_ok) {
            std::cerr << "    (version " << version.major << "." << version.minor
                       << ") full-length fixture unexpectedly failed to parse: " << full.error
                       << "\n";
            ok = false;
            continue;
        }

        for (size_t len = 0; len < packet.size(); ++len) {
            mpc::FrameOfData truncated = mpc::parse_frame_of_data(packet.data(), len, version);
            if (truncated.parse_ok) {
                std::cerr << "    (version " << version.major << "." << version.minor
                           << ") truncation at length " << len << "/" << packet.size()
                           << " unexpectedly parsed OK\n";
                ok = false;
            }
        }
    }
    return ok;
}

// A byte-flip fuzz on top of truncation: corrupt count fields with large
// values and confirm the parser still bails out cleanly rather than looping
// forever or reading out of bounds.
bool test_frame_fuzz_corrupted_counts() {
    std::vector<mpc::RigidBodySample> bodies = {
        make_sample(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, true),
    };
    const mpc::NatNetVersion version{3, 1};
    std::vector<std::uint8_t> packet = mpc::build_frame_of_data(1, bodies, version);

    // Byte offset 8 is nMarkerSets (4: header, 4: frameNumber, then
    // nMarkerSets) -- corrupt it to a huge value.
    bool ok = true;
    if (packet.size() < 12) {
        std::cerr << "    fixture packet unexpectedly small\n";
        return false;
    }
    std::vector<std::uint8_t> corrupted = packet;
    corrupted[8] = 0xFF;
    corrupted[9] = 0xFF;
    corrupted[10] = 0xFF;
    corrupted[11] = 0x7F;  // huge positive nMarkerSets.
    mpc::FrameOfData result = mpc::parse_frame_of_data(corrupted.data(), corrupted.size(), version);
    if (result.parse_ok) {
        std::cerr << "    corrupted nMarkerSets unexpectedly parsed OK\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test: NAT_PINGRESPONSE build -> parse round trip.
// ---------------------------------------------------------------------
bool test_ping_response_roundtrip() {
    std::array<std::uint8_t, 4> app_version{{1, 2, 3, 4}};
    std::array<std::uint8_t, 4> natnet_version{{3, 1, 0, 0}};
    std::vector<std::uint8_t> packet = mpc::build_ping_response("Motive", app_version, natnet_version);

    mpc::PingResponse parsed = mpc::parse_command_response(packet.data(), packet.size());
    bool ok = true;
    if (!parsed.ok) {
        std::cerr << "    parse failed: " << parsed.error << "\n";
        return false;
    }
    if (parsed.app_name != "Motive") {
        std::cerr << "    app_name mismatch: '" << parsed.app_name << "'\n";
        ok = false;
    }
    if (parsed.app_version != app_version) {
        std::cerr << "    app_version mismatch\n";
        ok = false;
    }
    if (parsed.natnet_version != natnet_version) {
        std::cerr << "    natnet_version mismatch\n";
        ok = false;
    }

    // A NAT_FRAMEOFDATA packet must NOT parse as a ping response.
    std::vector<mpc::RigidBodySample> bodies = {make_sample(1, 0, 0, 0, 0, 0, 0, 1, 0, true)};
    std::vector<std::uint8_t> frame_packet = mpc::build_frame_of_data(1, bodies, {3, 1});
    mpc::PingResponse wrong = mpc::parse_command_response(frame_packet.data(), frame_packet.size());
    if (wrong.ok) {
        std::cerr << "    a NAT_FRAMEOFDATA packet was accepted as a ping response\n";
        ok = false;
    }

    // Truncation fuzz on the ping response too.
    for (size_t len = 0; len < packet.size(); ++len) {
        mpc::PingResponse trunc = mpc::parse_command_response(packet.data(), len);
        if (trunc.ok) {
            std::cerr << "    truncation at length " << len << "/" << packet.size()
                       << " unexpectedly parsed OK\n";
            ok = false;
        }
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test: quat_to_planar_yaw known-angle cases, both up-axis conventions.
// ---------------------------------------------------------------------
bool test_quat_to_planar_yaw_known_angles() {
    bool ok = true;
    const std::vector<double> angles_deg = {0.0, 90.0, 180.0, -90.0};

    for (double deg : angles_deg) {
        const double theta = deg * M_PI / 180.0;
        const double half = theta / 2.0;

        // y_up = false (Z-up): rotation about +Z.
        {
            const double qx = 0.0, qy = 0.0, qz = std::sin(half), qw = std::cos(half);
            const double yaw = mpc::quat_to_planar_yaw(qx, qy, qz, qw, false);
            const double expected = mpc::normalize_angle(theta);
            if (!near_eq(yaw, expected, 1e-6)) {
                std::cerr << "    z-up angle=" << deg << "deg: got yaw=" << yaw << ", expected "
                           << expected << "\n";
                ok = false;
            }
        }

        // y_up = true: rotation about +Y (Motive's native up axis) -- see
        // OptiTrackCore.h's derivation for why this quaternion recovers the
        // SAME logical yaw under the y_up convention.
        {
            const double qx = 0.0, qy = std::sin(half), qz = 0.0, qw = std::cos(half);
            const double yaw = mpc::quat_to_planar_yaw(qx, qy, qz, qw, true);
            const double expected = mpc::normalize_angle(theta);
            if (!near_eq(yaw, expected, 1e-6)) {
                std::cerr << "    y-up angle=" << deg << "deg: got yaw=" << yaw << ", expected "
                           << expected << "\n";
                ok = false;
            }
        }
    }

    // Tilted quat: compose a body-frame roll (about local +X, which a
    // rotation about X always leaves fixed) with a yaw, and confirm the
    // extracted yaw ignores the tilt entirely -- see the Hamilton product
    // reasoning in this test's own derivation (rotating e_x about its own
    // axis is a no-op, so R_yaw*(R_roll*e_x) == R_yaw*e_x regardless of the
    // roll angle).
    auto hamilton_product = [](double w1, double x1, double y1, double z1, double w2, double x2,
                                 double y2, double z2, double& w, double& x, double& y, double& z) {
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    };

    {
        const double theta = 0.3;   // arbitrary non-special yaw.
        const double phi = 0.7;     // arbitrary tilt (roll about local +X).
        // Z-up: yaw about Z, roll about X.
        const double qyaw_w = std::cos(theta / 2.0), qyaw_z = std::sin(theta / 2.0);
        const double qroll_w = std::cos(phi / 2.0), qroll_x = std::sin(phi / 2.0);
        double w, x, y, z;
        hamilton_product(qyaw_w, 0.0, 0.0, qyaw_z, qroll_w, qroll_x, 0.0, 0.0, w, x, y, z);
        const double yaw = mpc::quat_to_planar_yaw(x, y, z, w, false);
        if (!near_eq(yaw, mpc::normalize_angle(theta), 1e-6)) {
            std::cerr << "    z-up tilted quat: got yaw=" << yaw << ", expected " << theta << "\n";
            ok = false;
        }
    }
    {
        const double theta = -1.1;  // arbitrary non-special yaw.
        const double phi = 0.5;     // arbitrary tilt (roll about local +X).
        // y_up: yaw about Y, roll about X.
        const double qyaw_w = std::cos(theta / 2.0), qyaw_y = std::sin(theta / 2.0);
        const double qroll_w = std::cos(phi / 2.0), qroll_x = std::sin(phi / 2.0);
        double w, x, y, z;
        hamilton_product(qyaw_w, 0.0, qyaw_y, 0.0, qroll_w, qroll_x, 0.0, 0.0, w, x, y, z);
        const double yaw = mpc::quat_to_planar_yaw(x, y, z, w, true);
        if (!near_eq(yaw, mpc::normalize_angle(theta), 1e-6)) {
            std::cerr << "    y-up tilted quat: got yaw=" << yaw << ", expected " << theta << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test: extract_planar_pose position mapping for both up-axis conventions.
// ---------------------------------------------------------------------
bool test_extract_planar_pose_position_mapping() {
    bool ok = true;
    mpc::RigidBodySample s = make_sample(1, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0, true);

    mpc::PlanarPose z_up = mpc::extract_planar_pose(s, false);
    if (!near_eq(z_up.x, 1.0) || !near_eq(z_up.y, 2.0)) {
        std::cerr << "    z-up position mapping: got (" << z_up.x << "," << z_up.y << ")\n";
        ok = false;
    }

    mpc::PlanarPose y_up = mpc::extract_planar_pose(s, true);
    if (!near_eq(y_up.x, 1.0) || !near_eq(y_up.y, -3.0)) {
        std::cerr << "    y-up position mapping: got (" << y_up.x << "," << y_up.y
                   << "), expected (1, -3)\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test: mocap_rotation() -- the NAMED first-stage function (see
// OptiTrackCore.h's doc comment) -- against the user's own hand-derived
// spec from the live rig-calibration session: facing Motive's +X -> yaw 0;
// facing Motive's +Z -> yaw -pi/2; a rotation about Motive's +Y by +theta
// -> yaw +theta. Also confirms bit-for-bit agreement with
// extract_planar_pose(sample, y_up=true), since mocap_rotation() IS that
// path's implementation now (see extract_planar_pose()'s doc comment).
// ---------------------------------------------------------------------
bool test_mocap_rotation_named_function() {
    bool ok = true;

    // Case 1: body facing Motive's +X (identity quaternion) -> planner yaw 0.
    // Position (x_m=2, y_m=5 [height, dropped], z_m=3) -> (x_p, y_p) = (2, -3).
    {
        mpc::PlanarPose p = mpc::mocap_rotation(2.0, 5.0, 3.0, 0.0, 0.0, 0.0, 1.0);
        if (!near_eq(p.x, 2.0) || !near_eq(p.y, -3.0) || !near_eq(p.yaw, 0.0)) {
            std::cerr << "    facing +X: got (" << p.x << "," << p.y << "," << p.yaw
                       << "), expected (2, -3, 0)\n";
            ok = false;
        }
    }

    // Case 2: body facing Motive's +Z. Per the standard active-quaternion
    // rotation matrix's first column (vx=cos(theta), vz=-sin(theta) for a
    // pure rotation-by-theta about +Y), the local +X forward axis lands on
    // world +Z (vx=0, vz=+1) at theta=-90deg, NOT +90deg -- i.e. this is a
    // NEGATIVE quarter-turn about +Y -- which is exactly what makes this
    // case's expected yaw come out -pi/2 (see mocap_rotation()'s header doc
    // comment: rotation about Motive's +Y by +theta -> planner yaw +theta).
    {
        const double half = (-M_PI / 2.0) / 2.0;
        mpc::PlanarPose p = mpc::mocap_rotation(0.0, 0.0, 0.0, 0.0, std::sin(half), 0.0,
                                                  std::cos(half));
        const double expected_yaw = -M_PI / 2.0;
        if (!near_eq(p.yaw, expected_yaw, 1e-6)) {
            std::cerr << "    facing +Z: got yaw=" << p.yaw << ", expected " << expected_yaw
                       << "\n";
            ok = false;
        }
    }

    // Case 3: rotation about Motive's +Y by +30deg -> planner yaw +30deg.
    {
        const double theta = 30.0 * M_PI / 180.0;
        const double half = theta / 2.0;
        mpc::PlanarPose p = mpc::mocap_rotation(0.0, 0.0, 0.0, 0.0, std::sin(half), 0.0,
                                                  std::cos(half));
        if (!near_eq(p.yaw, theta, 1e-6)) {
            std::cerr << "    +30deg about Y: got yaw=" << p.yaw << ", expected " << theta
                       << "\n";
            ok = false;
        }
    }

    // Case 4: bit-for-bit agreement with extract_planar_pose(sample, true)
    // across several arbitrary samples (position + a tilted, non-axis-aligned
    // quaternion, so this is not just re-testing the special-angle cases
    // above).
    {
        const std::vector<std::array<double, 7>> samples = {
            {1.5, -0.2, 2.7, 0.0, 0.0, 0.0, 1.0},
            {-3.0, 4.4, 0.1, 0.0, 0.13052619, 0.0, 0.99144486},   // ~15deg about Y.
            {0.0, 0.0, 0.0, 0.09229596, 0.09229596, 0.09229596, 0.98776739},  // tilted, arbitrary.
        };
        for (const auto& s : samples) {
            mpc::PlanarPose direct = mpc::mocap_rotation(s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
            mpc::RigidBodySample rb =
                make_sample(1, s[0], s[1], s[2], s[3], s[4], s[5], s[6], 0.0, true);
            mpc::PlanarPose via_extract = mpc::extract_planar_pose(rb, /*y_up=*/true);
            if (!near_eq(direct.x, via_extract.x, 1e-9) || !near_eq(direct.y, via_extract.y, 1e-9) ||
                !near_eq(direct.yaw, via_extract.yaw, 1e-9)) {
                std::cerr << "    mismatch vs extract_planar_pose: mocap_rotation=(" << direct.x
                           << "," << direct.y << "," << direct.yaw << ") extract_planar_pose=("
                           << via_extract.x << "," << via_extract.y << "," << via_extract.yaw
                           << ")\n";
                ok = false;
            }
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test: apply_planar_transform composition. Applying transform (x1,y1,t1)
// then (x2,y2,t2) must equal applying the single combined transform that
// chains the two.
// ---------------------------------------------------------------------
bool test_planar_transform_composition() {
    bool ok = true;
    mpc::PlanarPose p{1.0, 2.0, 0.4};

    const double x1 = 0.5, y1 = -1.5, t1 = 0.3;
    const double x2 = 2.0, y2 = 1.0, t2 = -0.6;

    mpc::PlanarPose step1 = mpc::apply_planar_transform(p, x1, y1, t1);
    mpc::PlanarPose step2 = mpc::apply_planar_transform(step1, x2, y2, t2);

    // Combined transform: applying (x1,y1,t1) then (x2,y2,t2) to point p is
    // equivalent to the single rigid transform whose rotation is t1+t2 and
    // whose translation is (x2,y2) + R(t2)*(x1,y1).
    const double c2 = std::cos(t2), s2 = std::sin(t2);
    const double combined_x0 = x2 + x1 * c2 - y1 * s2;
    const double combined_y0 = y2 + x1 * s2 + y1 * c2;
    const double combined_theta0 = t1 + t2;
    mpc::PlanarPose combined = mpc::apply_planar_transform(p, combined_x0, combined_y0, combined_theta0);

    if (!near_eq(step2.x, combined.x, 1e-9) || !near_eq(step2.y, combined.y, 1e-9) ||
        !near_eq(mpc::normalize_angle(step2.yaw), mpc::normalize_angle(combined.yaw), 1e-9)) {
        std::cerr << "    composed transform mismatch: step2=(" << step2.x << "," << step2.y << ","
                   << step2.yaw << ") vs combined=(" << combined.x << "," << combined.y << ","
                   << combined.yaw << ")\n";
        ok = false;
    }

    // Identity transform leaves pose unchanged.
    mpc::PlanarPose identity = mpc::apply_planar_transform(p, 0.0, 0.0, 0.0);
    if (!near_eq(identity.x, p.x, 1e-9) || !near_eq(identity.y, p.y, 1e-9) ||
        !near_eq(identity.yaw, p.yaw, 1e-9)) {
        std::cerr << "    identity transform changed the pose\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test: Downsampler timing -- rate, driftlessness, latest-sample-wins,
// resync-after-stall.
// ---------------------------------------------------------------------
bool test_downsampler_timing() {
    bool ok = true;
    mpc::Downsampler<int> ds(30.0);  // target 30Hz.

    std::vector<std::pair<double, int>> emissions;  // (t, value) pairs.
    constexpr double kFeedHz = 120.0;
    constexpr double kFeedDt = 1.0 / kFeedHz;
    constexpr int kNumFeeds = 240;  // 2 seconds at 120Hz.

    for (int i = 0; i < kNumFeeds; ++i) {
        const double t = i * kFeedDt;
        int out = -1;
        if (ds.feed(t, i, out)) {
            emissions.emplace_back(t, out);
        }
    }

    // First feed (t=0) must emit immediately.
    if (emissions.empty() || !near_eq(emissions.front().first, 0.0, 1e-9) ||
        emissions.front().second != 0) {
        std::cerr << "    first feed did not emit immediately\n";
        ok = false;
    }

    // ~60 emissions expected over 2s at 30Hz (4:1 downsample from 120Hz).
    const int expected_count = static_cast<int>(2.0 * 30.0);
    if (std::abs(static_cast<int>(emissions.size()) - expected_count) > 2) {
        std::cerr << "    emission count=" << emissions.size() << ", expected ~" << expected_count
                   << "\n";
        ok = false;
    }

    // Latest-sample-wins + driftlessness: each emission's value must be
    // whichever feed index triggered it (i.e. out == round(t*120)), and
    // successive emission times must stay close to an exact multiple of
    // 1/30s from the start (no cumulative drift versus wall time).
    for (size_t k = 0; k < emissions.size(); ++k) {
        const double expected_t = static_cast<double>(k) / 30.0;
        if (!near_eq(emissions[k].first, expected_t, kFeedDt + 1e-9)) {
            std::cerr << "    emission " << k << " at t=" << emissions[k].first
                       << ", expected close to " << expected_t << " (drift check)\n";
            ok = false;
        }
        const int expected_value = static_cast<int>(std::lround(emissions[k].first * kFeedHz));
        if (emissions[k].second != expected_value) {
            std::cerr << "    emission " << k << " value=" << emissions[k].second
                       << ", expected " << expected_value << " (latest-sample-wins check)\n";
            ok = false;
        }
    }

    // Resync after a stall: feed one sample far in the future, confirm it
    // emits immediately (does not wait for the old schedule), and that the
    // NEXT normal-rate feed after that does not also fire an immediate
    // catch-up burst.
    mpc::Downsampler<int> ds2(30.0);
    int out = -1;
    bool first = ds2.feed(0.0, 100, out);
    if (!first || out != 100) {
        std::cerr << "    ds2 first feed should emit\n";
        ok = false;
    }
    bool second = ds2.feed(0.01, 101, out);  // well within the 1/30s period -- must NOT emit.
    if (second) {
        std::cerr << "    ds2 second feed (within period) unexpectedly emitted\n";
        ok = false;
    }
    bool after_stall = ds2.feed(10.0, 200, out);  // huge gap.
    if (!after_stall || out != 200) {
        std::cerr << "    ds2 feed after a 10s stall should emit immediately with the latest value\n";
        ok = false;
    }
    bool immediately_after_stall = ds2.feed(10.001, 201, out);  // within 1/30s of the resync point.
    if (immediately_after_stall) {
        std::cerr << "    ds2 feed immediately after resync unexpectedly emitted again (burst)\n";
        ok = false;
    }

    // Degenerate target_hz<=0: every feed emits.
    mpc::Downsampler<int> ds_no_downsample(0.0);
    int emitted_count = 0;
    for (int i = 0; i < 10; ++i) {
        int v = -1;
        if (ds_no_downsample.feed(i * 0.001, i, v)) ++emitted_count;
    }
    if (emitted_count != 10) {
        std::cerr << "    target_hz<=0 should emit every sample, got " << emitted_count << "/10\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Tests: parse_mocap_config_text.
// ---------------------------------------------------------------------

bool has_diagnostic_containing(const std::vector<mpc::MocapConfigDiagnostic>& diags,
                                const std::string& substr, bool require_warning) {
    for (const mpc::MocapConfigDiagnostic& d : diags) {
        if ((!require_warning || d.is_warning) && d.message.find(substr) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool any_warning(const std::vector<mpc::MocapConfigDiagnostic>& diags) {
    for (const mpc::MocapConfigDiagnostic& d : diags) {
        if (d.is_warning) return true;
    }
    return false;
}

// The user's own real exported Motive streaming settings, reproduced here
// VERBATIM as a fixture string (per this task's brief: do NOT read the
// user's real file from disk in a test -- this is a hardcoded copy of its
// exact content instead).
bool test_mocap_config_exact_fixture() {
    const std::string text =
        "ip_address:192.168.1.166\n"
        "Type:Multicast\n"
        "Command Port:1510\n"
        "Data Port:1511\n"
        "Multicast Interface:239.255.42.99\n";
    mpc::MocapConfigResult r = mpc::parse_mocap_config_text(text);

    bool ok = true;
    if (!r.has_server_ip || r.server_ip != "192.168.1.166") {
        std::cerr << "    server_ip: has=" << r.has_server_ip << " value='" << r.server_ip << "'\n";
        ok = false;
    }
    if (!r.has_mode || r.mode != "multicast") {
        std::cerr << "    mode: has=" << r.has_mode << " value='" << r.mode << "'\n";
        ok = false;
    }
    if (!r.has_command_port || r.command_port != 1510) {
        std::cerr << "    command_port: has=" << r.has_command_port << " value=" << r.command_port
                   << "\n";
        ok = false;
    }
    if (!r.has_data_port || r.data_port != 1511) {
        std::cerr << "    data_port: has=" << r.has_data_port << " value=" << r.data_port << "\n";
        ok = false;
    }
    if (!r.has_multicast_group || r.multicast_group != "239.255.42.99") {
        std::cerr << "    multicast_group: has=" << r.has_multicast_group << " value='"
                   << r.multicast_group << "'\n";
        ok = false;
    }
    if (any_warning(r.diagnostics)) {
        std::cerr << "    unexpected warning(s) on a fully well-formed fixture\n";
        for (const auto& d : r.diagnostics) {
            if (d.is_warning) std::cerr << "      line " << d.line_number << ": " << d.message << "\n";
        }
        ok = false;
    }
    if (r.diagnostics.size() != 5) {
        std::cerr << "    expected exactly 5 diagnostics (one per line), got " << r.diagnostics.size()
                   << "\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_config_unicast_type() {
    const std::string text = "ip_address:10.0.0.5\nType:unicast\n";  // lowercase "unicast".
    mpc::MocapConfigResult r = mpc::parse_mocap_config_text(text);
    bool ok = true;
    if (!r.has_mode || r.mode != "unicast") {
        std::cerr << "    expected mode='unicast', got has=" << r.has_mode << " value='" << r.mode
                   << "'\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_config_missing_keys_fallback() {
    // Only ip_address present -- everything else must be left unset (has_*
    // false) so the caller falls back to its own CLI/compiled defaults.
    const std::string text = "ip_address:10.0.0.9\n";
    mpc::MocapConfigResult r = mpc::parse_mocap_config_text(text);
    bool ok = true;
    if (!r.has_server_ip || r.server_ip != "10.0.0.9") {
        std::cerr << "    server_ip not parsed correctly\n";
        ok = false;
    }
    if (r.has_mode || r.has_command_port || r.has_data_port || r.has_multicast_group) {
        std::cerr << "    fields absent from the text unexpectedly report has_*=true\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_config_unknown_key_warns() {
    const std::string text =
        "ip_address:10.0.0.1\n"
        "Foo:bar\n"
        "Command Port:1510\n";
    mpc::MocapConfigResult r = mpc::parse_mocap_config_text(text);
    bool ok = true;
    if (!has_diagnostic_containing(r.diagnostics, "Foo", /*require_warning=*/true)) {
        std::cerr << "    expected a warning diagnostic mentioning the unknown key 'Foo'\n";
        ok = false;
    }
    // The unknown line must not derail parsing of the surrounding valid lines.
    if (!r.has_server_ip || !r.has_command_port) {
        std::cerr << "    valid lines around the unknown key failed to parse\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_config_whitespace_and_crlf() {
    // Leading/trailing spaces around both key and value, blank lines, and
    // CRLF line endings, all in one fixture.
    const std::string text =
        "\r\n"
        "  ip_address : 10.0.0.42  \r\n"
        "\r\n"
        "  Type :  Multicast  \r\n";
    mpc::MocapConfigResult r = mpc::parse_mocap_config_text(text);
    bool ok = true;
    if (!r.has_server_ip || r.server_ip != "10.0.0.42") {
        std::cerr << "    server_ip not trimmed correctly, got '" << r.server_ip << "'\n";
        ok = false;
    }
    if (!r.has_mode || r.mode != "multicast") {
        std::cerr << "    mode not trimmed/normalized correctly, got '" << r.mode << "'\n";
        ok = false;
    }
    if (any_warning(r.diagnostics)) {
        std::cerr << "    unexpected warnings on a whitespace/CRLF/blank-line-only fixture\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_config_malformed_line() {
    const std::string text =
        "ip_address:10.0.0.1\n"
        "this line has no colon at all\n"
        "Data Port:1511\n";
    mpc::MocapConfigResult r = mpc::parse_mocap_config_text(text);
    bool ok = true;
    if (!has_diagnostic_containing(r.diagnostics, "missing ':'", /*require_warning=*/true)) {
        std::cerr << "    expected a malformed-line warning diagnostic\n";
        ok = false;
    }
    if (!r.has_server_ip || !r.has_data_port) {
        std::cerr << "    valid lines around the malformed one failed to parse\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test: VersionAutoDetector.
// ---------------------------------------------------------------------
bool test_version_auto_detector_adopts_after_five_consecutive() {
    bool ok = true;
    const std::vector<mpc::NatNetVersion> candidates = {{4, 0}, {3, 1}, {2, 10}};

    std::vector<mpc::RigidBodySample> bodies = {
        make_sample(1, 0.5, 0.25, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0021, true),
        make_sample(2, -0.5, 0.75, 0.0, 0.0, 0.0, 0.1, 0.9949874, 0.0032, true),
    };
    std::vector<std::uint8_t> v4_packet = mpc::build_frame_of_data(1, bodies, {4, 0});

    // Precondition sanity check: bytes built as v4.0 must NOT parse cleanly
    // as v2.10 (major<3 uses a structurally different rigid-body layout) --
    // otherwise this test's premise (the "configured" version keeps
    // failing) would not hold. Fails loudly here rather than producing a
    // confusing failure below if this assumption were ever wrong.
    mpc::FrameOfData wrong_parse = mpc::parse_frame_of_data(v4_packet.data(), v4_packet.size(), {2, 10});
    if (wrong_parse.parse_ok) {
        std::cerr << "    test precondition violated: v4.0-built bytes parsed OK as v2.10\n";
        return false;
    }

    mpc::VersionAutoDetector detector(candidates, 5);
    mpc::VersionAutoDetector::Outcome outcome;
    for (int i = 0; i < 4; ++i) {
        outcome = detector.on_parse_failure(v4_packet.data(), v4_packet.size());
        if (outcome.adopted) {
            std::cerr << "    adopted too early, on call " << (i + 1) << "/5\n";
            ok = false;
        }
    }
    outcome = detector.on_parse_failure(v4_packet.data(), v4_packet.size());  // 5th consecutive.
    if (!outcome.adopted) {
        std::cerr << "    did not adopt after 5 consecutive clean parses by the same candidate\n";
        ok = false;
    } else if (outcome.adopted_version.major != 4 || outcome.adopted_version.minor != 0) {
        std::cerr << "    adopted wrong version: " << outcome.adopted_version.major << "."
                   << outcome.adopted_version.minor << "\n";
        ok = false;
    }
    if (!detector.has_adopted()) {
        std::cerr << "    has_adopted() should be true after adoption\n";
        ok = false;
    }

    // Adoption is one-shot: a 6th call must not report adopted=true again.
    mpc::VersionAutoDetector::Outcome sixth = detector.on_parse_failure(v4_packet.data(), v4_packet.size());
    if (sixth.adopted) {
        std::cerr << "    adopted fired a second time -- must be one-shot\n";
        ok = false;
    }

    // Streak reset on an interrupting call where NO candidate parses
    // (deliberately corrupt bytes, so this does not depend on any
    // cross-version accidental-parse behavior).
    {
        mpc::VersionAutoDetector detector2(candidates, 5);
        const std::vector<std::uint8_t> garbage = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        for (int i = 0; i < 4; ++i) detector2.on_parse_failure(v4_packet.data(), v4_packet.size());
        detector2.on_parse_failure(garbage.data(), garbage.size());  // resets the streak.
        mpc::VersionAutoDetector::Outcome after_reset =
            detector2.on_parse_failure(v4_packet.data(), v4_packet.size());
        if (after_reset.adopted) {
            std::cerr << "    streak should have reset after an all-candidates-fail interrupt\n";
            ok = false;
        }
    }

    // Streak reset via on_parse_success() (a real, structurally-VALID
    // parse of the currently-configured version arriving mid-streak).
    {
        mpc::VersionAutoDetector detector3(candidates, 5);
        for (int i = 0; i < 4; ++i) detector3.on_parse_failure(v4_packet.data(), v4_packet.size());
        detector3.on_parse_success();
        mpc::VersionAutoDetector::Outcome after_success_reset =
            detector3.on_parse_failure(v4_packet.data(), v4_packet.size());
        if (after_success_reset.adopted) {
            std::cerr << "    streak should have reset after on_parse_success()\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Tests: choose_multicast_interface.
// ---------------------------------------------------------------------

bool test_choose_multicast_interface_contains() {
    const std::vector<mpc::IfaceInfo> ifaces = {
        {"enx00e04c4e1788", "192.168.1.113", "255.255.255.0"},
    };
    std::optional<mpc::IfaceInfo> chosen = mpc::choose_multicast_interface("192.168.1.166", ifaces);
    bool ok = true;
    if (!chosen.has_value()) {
        std::cerr << "    expected a match, got nullopt\n";
        return false;
    }
    if (chosen->addr != "192.168.1.113" || chosen->name != "enx00e04c4e1788") {
        std::cerr << "    wrong interface chosen: name='" << chosen->name << "' addr='" << chosen->addr
                   << "'\n";
        ok = false;
    }
    return ok;
}

bool test_choose_multicast_interface_no_match() {
    const std::vector<mpc::IfaceInfo> ifaces = {
        {"enp0s31f6", "141.212.78.162", "255.255.255.128"},
    };
    std::optional<mpc::IfaceInfo> chosen = mpc::choose_multicast_interface("192.168.1.166", ifaces);
    if (chosen.has_value()) {
        std::cerr << "    expected nullopt, got interface '" << chosen->name << "'\n";
        return false;
    }
    return true;
}

bool test_choose_multicast_interface_longest_prefix_wins() {
    // Both interfaces' subnets contain the server IP; the more specific
    // (/24) one must win over the broader (/16) one, regardless of list
    // order.
    const std::vector<mpc::IfaceInfo> ifaces = {
        {"broad_16", "192.168.0.1", "255.255.0.0"},
        {"specific_24", "192.168.1.113", "255.255.255.0"},
    };
    std::optional<mpc::IfaceInfo> chosen = mpc::choose_multicast_interface("192.168.1.166", ifaces);
    bool ok = true;
    if (!chosen.has_value()) {
        std::cerr << "    expected a match, got nullopt\n";
        return false;
    }
    if (chosen->name != "specific_24") {
        std::cerr << "    expected the longer-prefix interface to win, got '" << chosen->name << "'\n";
        ok = false;
    }

    // Order-independence: same two interfaces, reversed list order, same
    // expected winner.
    const std::vector<mpc::IfaceInfo> ifaces_reversed = {ifaces[1], ifaces[0]};
    std::optional<mpc::IfaceInfo> chosen2 =
        mpc::choose_multicast_interface("192.168.1.166", ifaces_reversed);
    if (!chosen2.has_value() || chosen2->name != "specific_24") {
        std::cerr << "    longest-prefix result should not depend on list order\n";
        ok = false;
    }
    return ok;
}

bool test_choose_multicast_interface_exact_server_address() {
    const std::vector<mpc::IfaceInfo> ifaces = {
        {"lo", "127.0.0.1", "255.0.0.0"},
    };
    std::optional<mpc::IfaceInfo> chosen = mpc::choose_multicast_interface("127.0.0.1", ifaces);
    bool ok = true;
    if (!chosen.has_value()) {
        std::cerr << "    expected a match when server_ip equals the interface's own address\n";
        return false;
    }
    if (chosen->addr != "127.0.0.1" || chosen->name != "lo") {
        std::cerr << "    wrong interface chosen\n";
        ok = false;
    }
    return ok;
}

bool test_choose_multicast_interface_malformed_entries_skipped() {
    const std::vector<mpc::IfaceInfo> ifaces = {
        {"bad", "not.an.ip.address", "255.255.255.0"},
        {"good", "192.168.1.113", "255.255.255.0"},
    };
    std::optional<mpc::IfaceInfo> chosen = mpc::choose_multicast_interface("192.168.1.166", ifaces);
    if (!chosen.has_value() || chosen->name != "good") {
        std::cerr << "    a malformed interface entry should be skipped, not derail matching\n";
        return false;
    }
    // A malformed server_ip must yield nullopt, never throw/crash.
    std::optional<mpc::IfaceInfo> from_bad_server =
        mpc::choose_multicast_interface("not-an-ip", ifaces);
    if (from_bad_server.has_value()) {
        std::cerr << "    malformed server_ip should yield nullopt\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------
// Tests: VersionGate.
// ---------------------------------------------------------------------

bool test_version_gate_ping_confirms_before_timeout() {
    bool ok = true;
    mpc::VersionGate gate(3.0);
    const mpc::NatNetVersion fallback{3, 1};

    if (gate.should_parse_now(0.5, fallback)) {
        std::cerr << "    should defer before any confirmation\n";
        ok = false;
    }
    if (gate.deferred_count() != 1) {
        std::cerr << "    deferred_count should be 1, got " << gate.deferred_count() << "\n";
        ok = false;
    }

    const mpc::NatNetVersion discovered{2, 10};
    gate.confirm_via_ping(discovered, 0.6);
    if (!gate.confirmed() || !gate.just_confirmed() || !gate.confirmed_via_ping()) {
        std::cerr << "    expected confirmed/just_confirmed/confirmed_via_ping all true after ping\n";
        ok = false;
    }
    if (gate.confirmed_version().major != 2 || gate.confirmed_version().minor != 10) {
        std::cerr << "    confirmed_version should be 2.10\n";
        ok = false;
    }
    if (!near_eq(gate.confirmed_at_s(), 0.6, 1e-9)) {
        std::cerr << "    confirmed_at_s should be 0.6, got " << gate.confirmed_at_s() << "\n";
        ok = false;
    }

    if (!gate.should_parse_now(0.7, fallback)) {
        std::cerr << "    should parse immediately once confirmed\n";
        ok = false;
    }
    if (gate.just_confirmed()) {
        std::cerr << "    just_confirmed() should be false on a call AFTER the confirming one\n";
        ok = false;
    }
    if (gate.deferred_count() != 1) {
        std::cerr << "    deferred_count should stay at 1 after confirmation, got "
                   << gate.deferred_count() << "\n";
        ok = false;
    }
    return ok;
}

bool test_version_gate_timeout_expiry_fallback() {
    bool ok = true;
    mpc::VersionGate gate(3.0);
    const mpc::NatNetVersion fallback{3, 1};

    int expected_deferred = 0;
    for (double t : {0.5, 1.0, 1.5, 2.0, 2.5, 2.9}) {
        if (gate.should_parse_now(t, fallback)) {
            std::cerr << "    unexpected early confirmation at t=" << t << "\n";
            ok = false;
        }
        ++expected_deferred;
    }
    if (gate.deferred_count() != expected_deferred) {
        std::cerr << "    deferred_count=" << gate.deferred_count() << ", expected " << expected_deferred
                   << "\n";
        ok = false;
    }

    if (!gate.should_parse_now(3.0, fallback)) {
        std::cerr << "    should confirm (timeout) and return true at t=3.0\n";
        ok = false;
    }
    if (!gate.just_confirmed() || gate.confirmed_via_ping()) {
        std::cerr << "    expected just_confirmed=true, confirmed_via_ping=false (timeout path)\n";
        ok = false;
    }
    if (gate.confirmed_version().major != 3 || gate.confirmed_version().minor != 1) {
        std::cerr << "    confirmed_version should be the fallback 3.1\n";
        ok = false;
    }
    if (!near_eq(gate.confirmed_at_s(), 3.0, 1e-9)) {
        std::cerr << "    confirmed_at_s should be 3.0\n";
        ok = false;
    }
    if (gate.deferred_count() != expected_deferred) {
        std::cerr << "    the confirming call itself should not count as deferred\n";
        ok = false;
    }
    return ok;
}

bool test_version_gate_no_re_deferral() {
    bool ok = true;
    mpc::VersionGate gate(1.0);
    const mpc::NatNetVersion fallback{3, 1};

    // Confirm via timeout immediately (elapsed already >= defer_timeout_s).
    if (!gate.should_parse_now(1.0, fallback)) {
        ok = false;
    }
    const long deferred_after_confirm = gate.deferred_count();

    // Calling with a SMALLER elapsed_s than the timeout (as if time went
    // backwards, or just a stale/late call) must still return true and must
    // NOT increment deferred_count() -- confirmation is permanent.
    if (!gate.should_parse_now(0.1, fallback)) {
        std::cerr << "    should_parse_now must return true unconditionally once confirmed\n";
        ok = false;
    }
    if (gate.deferred_count() != deferred_after_confirm) {
        std::cerr << "    deferred_count must not change after confirmation\n";
        ok = false;
    }

    // A ping arriving AFTER timeout-fallback confirmation upgrades the
    // version/trust but must not re-fire just_confirmed() or deferred_count.
    gate.confirm_via_ping({2, 10}, 5.0);
    if (gate.just_confirmed()) {
        std::cerr << "    a late ping after timeout-confirmation must not re-fire just_confirmed()\n";
        ok = false;
    }
    if (!gate.confirmed_via_ping() || gate.confirmed_version().major != 2 ||
        gate.confirmed_version().minor != 10) {
        std::cerr << "    a late ping should still upgrade confirmed_version()/confirmed_via_ping()\n";
        ok = false;
    }
    if (gate.deferred_count() != deferred_after_confirm) {
        std::cerr << "    deferred_count must not change on a late ping either\n";
        ok = false;
    }
    return ok;
}

bool test_version_gate_deferred_frame_counting() {
    bool ok = true;
    mpc::VersionGate gate(2.0);
    const mpc::NatNetVersion fallback{3, 1};

    const std::vector<double> ts = {0.1, 0.2, 0.3, 0.4, 0.5};  // all < 2.0 -- all deferred.
    for (double t : ts) {
        gate.should_parse_now(t, fallback);
    }
    if (gate.deferred_count() != static_cast<long>(ts.size())) {
        std::cerr << "    deferred_count=" << gate.deferred_count() << ", expected " << ts.size()
                   << "\n";
        ok = false;
    }

    // Ping confirms mid-stream -- subsequent calls must not add to
    // deferred_count regardless of how many more are made.
    gate.confirm_via_ping({4, 0}, 0.6);
    for (double t : {0.7, 0.8, 0.9}) {
        gate.should_parse_now(t, fallback);
    }
    if (gate.deferred_count() != static_cast<long>(ts.size())) {
        std::cerr << "    deferred_count changed after confirmation: " << gate.deferred_count() << "\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Tests: quat_to_roll_pitch_heading.
// ---------------------------------------------------------------------

bool test_roll_pitch_heading_known_angles() {
    bool ok = true;
    const double kAngle = 30.0 * M_PI / 180.0;  // stay well clear of +-90deg gimbal territory.
    const double half = kAngle / 2.0;
    const double s = std::sin(half), c = std::cos(half);

    // ---- z_up: pure heading (about +Z). ----
    {
        auto e = mpc::quat_to_roll_pitch_heading(0.0, 0.0, s, c, false);
        if (!near_eq(e.heading, kAngle, 1e-6) || !near_eq(e.pitch, 0.0, 1e-6) ||
            !near_eq(e.roll, 0.0, 1e-6)) {
            std::cerr << "    z-up pure heading: got (roll=" << e.roll << ",pitch=" << e.pitch
                       << ",heading=" << e.heading << ")\n";
            ok = false;
        }
    }
    // ---- z_up: pure pitch (about +Y). ----
    {
        auto e = mpc::quat_to_roll_pitch_heading(0.0, s, 0.0, c, false);
        if (!near_eq(e.heading, 0.0, 1e-6) || !near_eq(e.pitch, kAngle, 1e-6) ||
            !near_eq(e.roll, 0.0, 1e-6)) {
            std::cerr << "    z-up pure pitch: got (roll=" << e.roll << ",pitch=" << e.pitch
                       << ",heading=" << e.heading << ")\n";
            ok = false;
        }
    }
    // ---- z_up: pure roll (about +X). ----
    {
        auto e = mpc::quat_to_roll_pitch_heading(s, 0.0, 0.0, c, false);
        if (!near_eq(e.heading, 0.0, 1e-6) || !near_eq(e.pitch, 0.0, 1e-6) ||
            !near_eq(e.roll, kAngle, 1e-6)) {
            std::cerr << "    z-up pure roll: got (roll=" << e.roll << ",pitch=" << e.pitch
                       << ",heading=" << e.heading << ")\n";
            ok = false;
        }
    }
    // ---- y_up: pure heading (about native +Y). ----
    {
        auto e = mpc::quat_to_roll_pitch_heading(0.0, s, 0.0, c, true);
        if (!near_eq(e.heading, kAngle, 1e-6) || !near_eq(e.pitch, 0.0, 1e-6) ||
            !near_eq(e.roll, 0.0, 1e-6)) {
            std::cerr << "    y-up pure heading: got (roll=" << e.roll << ",pitch=" << e.pitch
                       << ",heading=" << e.heading << ")\n";
            ok = false;
        }
    }
    // ---- y_up: pure roll (about native +X, same axis role as z_up's roll). ----
    {
        auto e = mpc::quat_to_roll_pitch_heading(s, 0.0, 0.0, c, true);
        if (!near_eq(e.heading, 0.0, 1e-6) || !near_eq(e.pitch, 0.0, 1e-6) ||
            !near_eq(e.roll, kAngle, 1e-6)) {
            std::cerr << "    y-up pure roll: got (roll=" << e.roll << ",pitch=" << e.pitch
                       << ",heading=" << e.heading << ")\n";
            ok = false;
        }
    }
    // ---- y_up: pure pitch -- per the header doc comment, pitch is defined
    // as rotation about native -Z, so a rotation about native +Z by -kAngle
    // (qz = sin(-half) = -s) should decompose to pitch=+kAngle. ----
    {
        auto e = mpc::quat_to_roll_pitch_heading(0.0, 0.0, -s, c, true);
        if (!near_eq(e.heading, 0.0, 1e-6) || !near_eq(e.pitch, kAngle, 1e-6) ||
            !near_eq(e.roll, 0.0, 1e-6)) {
            std::cerr << "    y-up pure pitch: got (roll=" << e.roll << ",pitch=" << e.pitch
                       << ",heading=" << e.heading << ")\n";
            ok = false;
        }
    }

    // ---- Cross-check: heading must match quat_to_planar_yaw() bit-for-bit
    // (well within float noise) on an ARBITRARY (non-axis-aligned)
    // orientation, both conventions. ----
    {
        const double qx = 0.18, qy = -0.42, qz = 0.31, qw = 0.83;  // not unit-normalized on purpose --
        // both functions apply the SAME formulas to the SAME raw inputs, so
        // normalization (or lack thereof) cancels out of this comparison.
        for (bool y_up : {false, true}) {
            const double expected = mpc::quat_to_planar_yaw(qx, qy, qz, qw, y_up);
            auto e = mpc::quat_to_roll_pitch_heading(qx, qy, qz, qw, y_up);
            if (!near_eq(e.heading, expected, 1e-9)) {
                std::cerr << "    heading/quat_to_planar_yaw mismatch (y_up=" << y_up
                           << "): " << e.heading << " vs " << expected << "\n";
                ok = false;
            }
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Tests: NAT_MODELDEF round trip / fuzz / graceful-stop-on-unknown-type.
// ---------------------------------------------------------------------

bool test_modeldef_roundtrip_for_version(const mpc::NatNetVersion& version) {
    const std::vector<mpc::MarkerSetDef> marker_sets = {
        {"My Marker Set", {"Marker A", "Marker B", "Marker C"}},
    };

    std::vector<mpc::RigidBodyBuildDef> rigid_bodies;
    {
        mpc::RigidBodyBuildDef rb1;
        rb1.def.id = 1;
        rb1.def.name = "Push Robot 1";  // name with a space, per the test spec.
        rb1.def.parent_id = 0;
        rb1.marker_offsets = {{0.01f, 0.02f, 0.03f}, {-0.01f, 0.0f, 0.05f}};
        rb1.marker_active_labels = {101, 102};
        rb1.marker_names = {"RB1 Marker A", "RB1 Marker B"};  // only emitted for major>=4.
        rigid_bodies.push_back(rb1);

        mpc::RigidBodyBuildDef rb2;
        rb2.def.id = 2;
        rb2.def.name = "Second Body";
        rb2.def.parent_id = 0;
        // Deliberately empty marker arrays -- exercises the nMarkers=0 case.
        rigid_bodies.push_back(rb2);
    }

    std::vector<mpc::SkeletonBuildDef> skeletons;
    {
        mpc::SkeletonBuildDef sk;
        sk.name = "Test Skeleton";
        sk.id = 500;
        mpc::RigidBodyBuildDef nested;
        nested.def.id = 501;
        nested.def.name = "Skeleton Bone 1";
        nested.def.parent_id = 500;
        sk.nested_rigid_bodies.push_back(nested);
        skeletons.push_back(sk);
    }

    std::vector<std::uint8_t> packet = mpc::build_modeldef(marker_sets, rigid_bodies, skeletons, version);
    mpc::ModelDef parsed = mpc::parse_modeldef(packet.data(), packet.size(), version);

    bool ok = true;
    if (!parsed.parse_ok) {
        std::cerr << "    parse_ok=false, error='" << parsed.error << "'\n";
        return false;
    }
    if (parsed.stopped_early) {
        std::cerr << "    unexpectedly stopped_early=true\n";
        ok = false;
    }
    if (parsed.rigid_bodies.size() != 2) {
        std::cerr << "    expected 2 top-level rigid bodies (skeleton-nested ones are NOT surfaced), "
                      "got "
                   << parsed.rigid_bodies.size() << "\n";
        return false;
    }
    if (parsed.rigid_bodies[0].id != 1 || parsed.rigid_bodies[0].name != "Push Robot 1" ||
        parsed.rigid_bodies[0].parent_id != 0) {
        std::cerr << "    rigid body 0 mismatch: id=" << parsed.rigid_bodies[0].id << " name='"
                   << parsed.rigid_bodies[0].name << "' parent=" << parsed.rigid_bodies[0].parent_id
                   << "\n";
        ok = false;
    }
    if (parsed.rigid_bodies[1].id != 2 || parsed.rigid_bodies[1].name != "Second Body" ||
        parsed.rigid_bodies[1].parent_id != 0) {
        std::cerr << "    rigid body 1 mismatch: id=" << parsed.rigid_bodies[1].id << " name='"
                   << parsed.rigid_bodies[1].name << "'\n";
        ok = false;
    }
    return ok;
}

bool test_modeldef_roundtrip_v2_10() { return test_modeldef_roundtrip_for_version({2, 10}); }
bool test_modeldef_roundtrip_v4_0() { return test_modeldef_roundtrip_for_version({4, 0}); }

bool test_modeldef_fuzz_truncation() {
    bool ok = true;
    for (const mpc::NatNetVersion& version :
         {mpc::NatNetVersion{2, 10}, mpc::NatNetVersion{3, 1}, mpc::NatNetVersion{4, 0}}) {
        const std::vector<mpc::MarkerSetDef> marker_sets = {{"MS", {"M1", "M2"}}};
        std::vector<mpc::RigidBodyBuildDef> rigid_bodies;
        mpc::RigidBodyBuildDef rb;
        rb.def.id = 1;
        rb.def.name = "RB";
        rb.def.parent_id = 0;
        rb.marker_offsets = {{1.0f, 2.0f, 3.0f}};
        rb.marker_active_labels = {1};
        rb.marker_names = {"M"};
        rigid_bodies.push_back(rb);
        std::vector<mpc::SkeletonBuildDef> skeletons;
        mpc::SkeletonBuildDef sk;
        sk.name = "SK";
        sk.id = 9;
        mpc::RigidBodyBuildDef nested;
        nested.def.id = 10;
        nested.def.name = "N";
        nested.def.parent_id = 9;
        sk.nested_rigid_bodies.push_back(nested);
        skeletons.push_back(sk);

        std::vector<std::uint8_t> packet = mpc::build_modeldef(marker_sets, rigid_bodies, skeletons, version);

        mpc::ModelDef full = mpc::parse_modeldef(packet.data(), packet.size(), version);
        if (!full.parse_ok || full.stopped_early) {
            std::cerr << "    (version " << version.major << "." << version.minor
                       << ") full-length fixture unexpectedly failed/stopped: " << full.error << "\n";
            ok = false;
            continue;
        }

        for (std::size_t len = 0; len < packet.size(); ++len) {
            mpc::ModelDef truncated = mpc::parse_modeldef(packet.data(), len, version);
            // No unknown-type dataset exists in this fixture, so a
            // truncated prefix must NEVER report parse_ok=true (whether via
            // the normal path or a spurious stopped_early).
            if (truncated.parse_ok) {
                std::cerr << "    (version " << version.major << "." << version.minor
                           << ") truncation at length " << len << "/" << packet.size()
                           << " unexpectedly parsed OK\n";
                ok = false;
            }
        }
    }
    return ok;
}

bool test_modeldef_unknown_type_stops_gracefully() {
    // Hand-built payload (build_modeldef() only ever emits known types 0/1/2
    // by design, so an unknown-type fixture must be constructed directly):
    // nDatasets=2; dataset0=rigidbody(major>=3 layout, nMarkers=0);
    // dataset1=type 99 (unrecognized, nothing further to read for it).
    auto push_i32 = [](std::vector<std::uint8_t>& v, std::int32_t x) {
        const auto u = static_cast<std::uint32_t>(x);
        v.push_back(static_cast<std::uint8_t>(u & 0xFF));
        v.push_back(static_cast<std::uint8_t>((u >> 8) & 0xFF));
        v.push_back(static_cast<std::uint8_t>((u >> 16) & 0xFF));
        v.push_back(static_cast<std::uint8_t>((u >> 24) & 0xFF));
    };
    auto push_f32 = [](std::vector<std::uint8_t>& v, float f) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        v.push_back(static_cast<std::uint8_t>(bits & 0xFF));
        v.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xFF));
        v.push_back(static_cast<std::uint8_t>((bits >> 16) & 0xFF));
        v.push_back(static_cast<std::uint8_t>((bits >> 24) & 0xFF));
    };
    auto push_cstr = [](std::vector<std::uint8_t>& v, const std::string& s) {
        v.insert(v.end(), s.begin(), s.end());
        v.push_back(0);
    };

    std::vector<std::uint8_t> payload;
    push_i32(payload, 2);  // nDatasets = 2.
    push_i32(payload, 1);  // dataset 0: type = rigidbody.
    push_cstr(payload, "RB1");
    push_i32(payload, 1);  // id.
    push_i32(payload, 0);  // parentID.
    push_f32(payload, 0.0f);
    push_f32(payload, 0.0f);
    push_f32(payload, 0.0f);  // offset.
    push_i32(payload, 0);     // nMarkers = 0 (major>=3 layout).
    push_i32(payload, 99);    // dataset 1: type = 99 (unrecognized).

    std::vector<std::uint8_t> packet;
    packet.push_back(static_cast<std::uint8_t>(mpc::kNatModelDef & 0xFF));
    packet.push_back(static_cast<std::uint8_t>((mpc::kNatModelDef >> 8) & 0xFF));
    const auto psize = static_cast<std::uint16_t>(payload.size());
    packet.push_back(static_cast<std::uint8_t>(psize & 0xFF));
    packet.push_back(static_cast<std::uint8_t>((psize >> 8) & 0xFF));
    packet.insert(packet.end(), payload.begin(), payload.end());

    mpc::ModelDef parsed = mpc::parse_modeldef(packet.data(), packet.size(), {3, 1});
    bool ok = true;
    if (!parsed.parse_ok) {
        std::cerr << "    expected parse_ok=true (graceful stop), got false: '" << parsed.error << "'\n";
        ok = false;
    }
    if (!parsed.stopped_early) {
        std::cerr << "    expected stopped_early=true\n";
        ok = false;
    }
    if (parsed.rigid_bodies.size() != 1 || parsed.rigid_bodies[0].id != 1) {
        std::cerr << "    expected exactly the 1 rigid body parsed before the unknown type, got "
                   << parsed.rigid_bodies.size() << "\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Tests: resolve_robot_ids.
// ---------------------------------------------------------------------

bool test_resolve_robot_ids_all_present() {
    const std::vector<mpc::RigidBodyDef> modeldef = {{10, "Alpha", 0}, {20, "Beta", 0}, {30, "Gamma", 0}};
    const std::vector<std::string> robots = {"robot1", "robot2"};
    const std::vector<std::string> names = {"Alpha", "Gamma"};
    mpc::ResolutionResult res = mpc::resolve_robot_ids(robots, names, modeldef);

    bool ok = true;
    if (!res.all_resolved) {
        std::cerr << "    expected all_resolved=true\n";
        ok = false;
    }
    if (res.robots.size() != 2) return false;
    if (!res.robots[0].resolved || res.robots[0].rigid_body_id != 10 ||
        res.robots[0].source != mpc::RobotIdSource::kNameMatch) {
        std::cerr << "    robot1 mismatch\n";
        ok = false;
    }
    if (!res.robots[1].resolved || res.robots[1].rigid_body_id != 30 ||
        res.robots[1].source != mpc::RobotIdSource::kNameMatch) {
        std::cerr << "    robot2 mismatch\n";
        ok = false;
    }
    return ok;
}

bool test_resolve_robot_ids_one_missing() {
    const std::vector<mpc::RigidBodyDef> modeldef = {{10, "Alpha", 0}};
    const std::vector<std::string> robots = {"robot1", "robot2"};
    const std::vector<std::string> names = {"Alpha", "NotThere"};
    mpc::ResolutionResult res = mpc::resolve_robot_ids(robots, names, modeldef);

    bool ok = true;
    if (res.all_resolved) {
        std::cerr << "    expected all_resolved=false (one name missing)\n";
        ok = false;
    }
    if (!res.robots[0].resolved || res.robots[0].rigid_body_id != 10) {
        std::cerr << "    robot1 should have resolved via 'Alpha'\n";
        ok = false;
    }
    if (res.robots[1].resolved) {
        std::cerr << "    robot2 should be unresolved ('NotThere' is not in modeldef)\n";
        ok = false;
    }
    if (res.robots[1].error.find("NotThere") == std::string::npos ||
        res.robots[1].error.find("Alpha") == std::string::npos) {
        std::cerr << "    robot2's error should mention the missing name and list available names, "
                      "got '"
                   << res.robots[1].error << "'\n";
        ok = false;
    }
    return ok;
}

bool test_resolve_robot_ids_duplicate_name() {
    const std::vector<mpc::RigidBodyDef> modeldef = {{10, "Dup", 0}, {20, "Dup", 0}};
    const std::vector<std::string> robots = {"robot1"};
    const std::vector<std::string> names = {"Dup"};
    mpc::ResolutionResult res = mpc::resolve_robot_ids(robots, names, modeldef);

    bool ok = true;
    if (!res.robots[0].resolved || res.robots[0].rigid_body_id != 10) {
        std::cerr << "    expected the FIRST duplicate (id=10) to win, got resolved="
                   << res.robots[0].resolved << " id=" << res.robots[0].rigid_body_id << "\n";
        ok = false;
    }
    return ok;
}

bool test_resolve_robot_ids_auto_mode() {
    // Only "robot2" happens to also be a real Motive asset name.
    const std::vector<mpc::RigidBodyDef> modeldef = {{5, "robot2", 0}};
    const std::vector<std::string> robots = {"robot1", "robot2"};
    mpc::ResolutionResult res = mpc::resolve_robot_ids(robots, {}, modeldef);  // empty -> auto mode.

    bool ok = true;
    if (!res.all_resolved) {
        std::cerr << "    auto mode should always fully resolve\n";
        ok = false;
    }
    if (res.robots[0].rigid_body_id != 1 || res.robots[0].source != mpc::RobotIdSource::kIndexFallback) {
        std::cerr << "    robot1 (no matching Motive name) should fall back to index id=1, got id="
                   << res.robots[0].rigid_body_id << " source=" << mpc::to_string(res.robots[0].source)
                   << "\n";
        ok = false;
    }
    if (res.robots[1].rigid_body_id != 5 || res.robots[1].source != mpc::RobotIdSource::kNameMatch) {
        std::cerr << "    robot2 (Motive has an asset literally named 'robot2') should name-match to "
                      "id=5, got id="
                   << res.robots[1].rigid_body_id << " source=" << mpc::to_string(res.robots[1].source)
                   << "\n";
        ok = false;
    }
    return ok;
}

bool test_resolve_robot_ids_auto_mode_empty_modeldef() {
    // Auto mode must fully resolve via index fallback even with NO modeldef
    // available at all -- never blocks waiting on Motive.
    const std::vector<mpc::RigidBodyDef> modeldef;
    const std::vector<std::string> robots = {"robot1", "robot2"};
    mpc::ResolutionResult res = mpc::resolve_robot_ids(robots, {}, modeldef);

    bool ok = true;
    if (!res.all_resolved) {
        std::cerr << "    auto mode with an empty modeldef must still fully resolve\n";
        ok = false;
    }
    if (res.robots[0].rigid_body_id != 1 || res.robots[1].rigid_body_id != 2) {
        std::cerr << "    expected index fallback ids 1,2\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Tests: sanitize_topic_name.
// ---------------------------------------------------------------------

bool test_sanitize_topic_name_basic() {
    bool ok = true;

    {
        mpc::SanitizeResult r = mpc::sanitize_topic_name("robot 1", "fallback");
        if (r.sanitized != "robot_1" || !r.changed || r.used_fallback) {
            std::cerr << "    'robot 1' -> got '" << r.sanitized << "' changed=" << r.changed
                       << " used_fallback=" << r.used_fallback << "\n";
            ok = false;
        }
    }
    {
        // Already topic-safe: unchanged.
        mpc::SanitizeResult r = mpc::sanitize_topic_name("robot-1_ok", "fallback");
        if (r.sanitized != "robot-1_ok" || r.changed || r.used_fallback) {
            std::cerr << "    'robot-1_ok' should be unchanged, got '" << r.sanitized
                       << "' changed=" << r.changed << "\n";
            ok = false;
        }
    }
    {
        // Every character invalid -- replacement (not removal) keeps length.
        mpc::SanitizeResult r = mpc::sanitize_topic_name("!!!weird@@@", "fallback");
        if (r.sanitized != "___weird___" || !r.changed) {
            std::cerr << "    '!!!weird@@@' -> got '" << r.sanitized << "'\n";
            ok = false;
        }
    }
    {
        // Empty-after-sanitize edge: an EMPTY input uses the fallback
        // (replacement never shrinks a non-empty input to empty).
        mpc::SanitizeResult r = mpc::sanitize_topic_name("", "body5");
        if (r.sanitized != "body5" || !r.used_fallback) {
            std::cerr << "    empty input -> got '" << r.sanitized << "' used_fallback=" << r.used_fallback
                       << "\n";
            ok = false;
        }
    }
    return ok;
}

// ---------------------------------------------------------------------
// Tests: resolve_auto_discovery / find_unmatched_aliases.
// ---------------------------------------------------------------------

bool test_auto_discovery_alias_and_sanitize() {
    const std::vector<mpc::RigidBodyDef> modeldef = {
        {1, "mushr2", 0},
        {2, "block A", 0},
    };
    const std::unordered_map<std::string, std::string> aliases = {{"mushr2", "robot1"}};

    std::vector<mpc::AutoDiscoveredBody> bodies = mpc::resolve_auto_discovery(modeldef, aliases);
    bool ok = true;
    if (bodies.size() != 2) return false;
    if (bodies[0].published_name != "robot1" || !bodies[0].from_alias) {
        std::cerr << "    body 0 (aliased) mismatch: published_name='" << bodies[0].published_name
                   << "' from_alias=" << bodies[0].from_alias << "\n";
        ok = false;
    }
    if (bodies[1].published_name != "block_A" || bodies[1].from_alias || !bodies[1].name_sanitized) {
        std::cerr << "    body 1 (sanitized) mismatch: published_name='" << bodies[1].published_name
                   << "' from_alias=" << bodies[1].from_alias
                   << " name_sanitized=" << bodies[1].name_sanitized << "\n";
        ok = false;
    }
    return ok;
}

bool test_auto_discovery_late_appearing_body() {
    const std::unordered_map<std::string, std::string> aliases;
    const std::vector<mpc::RigidBodyDef> modeldef_v1 = {{1, "A", 0}};
    std::vector<mpc::AutoDiscoveredBody> r1 = mpc::resolve_auto_discovery(modeldef_v1, aliases);

    const std::vector<mpc::RigidBodyDef> modeldef_v2 = {{1, "A", 0}, {2, "B", 0}};
    std::vector<mpc::AutoDiscoveredBody> r2 = mpc::resolve_auto_discovery(modeldef_v2, aliases);

    bool ok = true;
    if (r1.size() != 1) {
        std::cerr << "    first snapshot should have 1 body\n";
        ok = false;
    }
    if (r2.size() != 2 || r2[1].rigid_body_id != 2 || r2[1].published_name != "B") {
        std::cerr << "    second (later) snapshot should include the newly-appeared body B\n";
        ok = false;
    }
    // The pre-existing body's resolution is stable across snapshots.
    if (!r1.empty() && !r2.empty() && r1[0].published_name != r2[0].published_name) {
        std::cerr << "    pre-existing body's published_name changed across snapshots unexpectedly\n";
        ok = false;
    }
    return ok;
}

bool test_auto_discovery_empty_name_fallback() {
    const std::vector<mpc::RigidBodyDef> modeldef = {{5, "", 0}};
    const std::unordered_map<std::string, std::string> aliases;
    std::vector<mpc::AutoDiscoveredBody> bodies = mpc::resolve_auto_discovery(modeldef, aliases);
    bool ok = true;
    if (bodies.size() != 1 || bodies[0].published_name != "body5" || !bodies[0].used_empty_fallback) {
        std::cerr << "    expected fallback 'body5', got '"
                   << (bodies.empty() ? "<none>" : bodies[0].published_name) << "'\n";
        ok = false;
    }
    return ok;
}

bool test_auto_discovery_disambiguation() {
    // Two bodies with the SAME raw name (no alias) -- both sanitize to "A";
    // the first (lower id, per modeldef order) keeps "A", the second is
    // disambiguated to "A_2".
    const std::vector<mpc::RigidBodyDef> modeldef = {{1, "A", 0}, {2, "A", 0}};
    const std::unordered_map<std::string, std::string> aliases;
    std::vector<mpc::AutoDiscoveredBody> bodies = mpc::resolve_auto_discovery(modeldef, aliases);
    bool ok = true;
    if (bodies.size() != 2) return false;
    if (bodies[0].published_name != "A" || bodies[0].disambiguated) {
        std::cerr << "    first 'A' should keep its name undisambiguated, got '"
                   << bodies[0].published_name << "' disambiguated=" << bodies[0].disambiguated << "\n";
        ok = false;
    }
    if (bodies[1].published_name != "A_2" || !bodies[1].disambiguated) {
        std::cerr << "    second 'A' should be disambiguated to 'A_2', got '" << bodies[1].published_name
                   << "' disambiguated=" << bodies[1].disambiguated << "\n";
        ok = false;
    }
    return ok;
}

bool test_find_unmatched_aliases() {
    const std::unordered_map<std::string, std::string> aliases = {{"mushr2", "robot1"}, {"ghost", "robot2"}};
    const std::vector<mpc::RigidBodyDef> modeldef = {{1, "mushr2", 0}};
    std::vector<std::string> unmatched = mpc::find_unmatched_aliases(aliases, modeldef);
    bool ok = true;
    if (unmatched.size() != 1 || unmatched[0] != "ghost") {
        std::cerr << "    expected exactly ['ghost'] unmatched, got " << unmatched.size() << " entries\n";
        ok = false;
    }
    // No unmatched aliases when every key has a match.
    std::vector<std::string> none_unmatched =
        mpc::find_unmatched_aliases({{"mushr2", "robot1"}}, modeldef);
    if (!none_unmatched.empty()) {
        std::cerr << "    expected zero unmatched when every alias key matches\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Tests: parse_mocap_to_world_matrix / apply_mocap_to_world / affine_from_xytheta.
// ---------------------------------------------------------------------

bool test_mocap_to_world_identity() {
    const std::vector<std::vector<double>> rows = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    mpc::MocapToWorldParseResult r = mpc::parse_mocap_to_world_matrix(rows);
    bool ok = true;
    if (!r.ok || r.non_rigid_warning) {
        std::cerr << "    identity: ok=" << r.ok << " non_rigid_warning=" << r.non_rigid_warning
                   << " error='" << r.error << "'\n";
        ok = false;
    }
    mpc::PlanarPose in{1.5, -2.5, 0.3};
    mpc::PlanarPose out = mpc::apply_mocap_to_world(in, r.transform);
    if (!near_eq(out.x, in.x, 1e-9) || !near_eq(out.y, in.y, 1e-9) || !near_eq(out.yaw, in.yaw, 1e-9)) {
        std::cerr << "    identity transform should leave the pose unchanged, got (" << out.x << ","
                   << out.y << "," << out.yaw << ")\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_to_world_rotation_translation() {
    // 90-degree rotation + translate(1,2): a=cos90=0, b=-sin90=-1, c=sin90=1, d=cos90=0.
    const std::vector<std::vector<double>> rows = {{0, -1, 1}, {1, 0, 2}, {0, 0, 1}};
    mpc::MocapToWorldParseResult r = mpc::parse_mocap_to_world_matrix(rows);
    bool ok = true;
    if (!r.ok || r.non_rigid_warning) {
        std::cerr << "    rotation+translation: ok=" << r.ok
                   << " non_rigid_warning=" << r.non_rigid_warning << " error='" << r.error << "'\n";
        ok = false;
    }
    mpc::PlanarPose in{1.0, 0.0, 0.0};
    mpc::PlanarPose out = mpc::apply_mocap_to_world(in, r.transform);
    // x_w = 0*1 + -1*0 + 1 = 1; y_w = 1*1 + 0*0 + 2 = 3; yaw_w = atan2(1,0)+0 = pi/2.
    if (!near_eq(out.x, 1.0, 1e-9) || !near_eq(out.y, 3.0, 1e-9) || !near_eq(out.yaw, M_PI / 2.0, 1e-9)) {
        std::cerr << "    got (x=" << out.x << ",y=" << out.y << ",yaw=" << out.yaw
                   << "), expected (1,3,pi/2)\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_to_world_equivalence_with_legacy() {
    bool ok = true;
    const std::vector<std::tuple<double, double, double>> cases = {
        {0.0, 0.0, 0.0}, {1.5, -2.5, 0.7}, {-3.0, 4.0, 2.9}, {0.2, 0.0, -1.8},
    };
    const mpc::PlanarPose sample{2.3, -0.7, 1.1};
    for (const auto& [x0, y0, theta0] : cases) {
        mpc::AffineTransform2D m = mpc::affine_from_xytheta(x0, y0, theta0);
        mpc::PlanarPose via_matrix = mpc::apply_mocap_to_world(sample, m);
        mpc::PlanarPose via_legacy = mpc::apply_planar_transform(sample, x0, y0, theta0);
        // Floating-point trig round-trip tolerance, not bit-identical -- see
        // affine_from_xytheta()'s doc comment.
        if (!near_eq(via_matrix.x, via_legacy.x, 1e-9) || !near_eq(via_matrix.y, via_legacy.y, 1e-9) ||
            !near_eq(via_matrix.yaw, via_legacy.yaw, 1e-9)) {
            std::cerr << "    (x0=" << x0 << ",y0=" << y0 << ",theta0=" << theta0
                       << ") matrix=(x=" << via_matrix.x << ",y=" << via_matrix.y
                       << ",yaw=" << via_matrix.yaw << ") vs legacy=(x=" << via_legacy.x
                       << ",y=" << via_legacy.y << ",yaw=" << via_legacy.yaw << ")\n";
            ok = false;
        }
    }
    return ok;
}

bool test_mocap_to_world_non_rigid_warns() {
    // Pure scale (x2 in x only) -- not a rotation: det=2, columns not unit length.
    const std::vector<std::vector<double>> rows = {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    mpc::MocapToWorldParseResult r = mpc::parse_mocap_to_world_matrix(rows);
    bool ok = true;
    if (!r.ok) {
        std::cerr << "    non-rigid transform should still be ok=true (warning, not error)\n";
        ok = false;
    }
    if (!r.non_rigid_warning || r.warning.empty()) {
        std::cerr << "    expected non_rigid_warning=true with a non-empty warning message\n";
        ok = false;
    }
    // Position still gets the FULL affine transform (x doubled).
    mpc::PlanarPose out = mpc::apply_mocap_to_world(mpc::PlanarPose{3.0, 1.0, 0.0}, r.transform);
    if (!near_eq(out.x, 6.0, 1e-9) || !near_eq(out.y, 1.0, 1e-9)) {
        std::cerr << "    expected position to still get the full affine scale, got (" << out.x << ","
                   << out.y << ")\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_to_world_bad_bottom_row() {
    const std::vector<std::vector<double>> rows = {{1, 0, 0}, {0, 1, 0}, {0.1, 0, 1}};
    mpc::MocapToWorldParseResult r = mpc::parse_mocap_to_world_matrix(rows);
    bool ok = true;
    if (r.ok) {
        std::cerr << "    expected a hard error for a bad bottom row, got ok=true\n";
        ok = false;
    }
    if (r.error.find("bottom row") == std::string::npos) {
        std::cerr << "    error message should mention 'bottom row', got '" << r.error << "'\n";
        ok = false;
    }
    return ok;
}

bool test_mocap_to_world_malformed_shapes() {
    bool ok = true;
    // Too few rows.
    {
        mpc::MocapToWorldParseResult r =
            mpc::parse_mocap_to_world_matrix({{1, 0, 0}, {0, 1, 0}});
        if (r.ok) {
            std::cerr << "    2-row matrix should be a hard error\n";
            ok = false;
        }
    }
    // Too many rows.
    {
        mpc::MocapToWorldParseResult r = mpc::parse_mocap_to_world_matrix(
            {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}});
        if (r.ok) {
            std::cerr << "    4-row matrix should be a hard error\n";
            ok = false;
        }
    }
    // A row with the wrong number of entries.
    {
        mpc::MocapToWorldParseResult r =
            mpc::parse_mocap_to_world_matrix({{1, 0}, {0, 1, 0}, {0, 0, 1}});
        if (r.ok) {
            std::cerr << "    a 2-entry row should be a hard error\n";
            ok = false;
        }
    }
    // Completely empty.
    {
        mpc::MocapToWorldParseResult r = mpc::parse_mocap_to_world_matrix({});
        if (r.ok) {
            std::cerr << "    an empty matrix should be a hard error\n";
            ok = false;
        }
    }
    return ok;
}

bool test_mocap_to_world_yaw_composition_and_wrapping() {
    bool ok = true;
    // 170deg rotation composed with a pose already at 170deg heading -> 340deg,
    // which must wrap into (-pi, pi].
    const double theta0 = 170.0 * M_PI / 180.0;
    mpc::AffineTransform2D m = mpc::affine_from_xytheta(0.0, 0.0, theta0);
    mpc::PlanarPose in{0.0, 0.0, 170.0 * M_PI / 180.0};
    mpc::PlanarPose out = mpc::apply_mocap_to_world(in, m);
    const double expected = mpc::normalize_angle(theta0 + in.yaw);  // == -20deg wrapped.
    if (!near_eq(out.yaw, expected, 1e-6) || out.yaw > M_PI || out.yaw <= -M_PI) {
        std::cerr << "    got yaw=" << out.yaw << " (deg=" << (out.yaw * 180.0 / M_PI)
                   << "), expected " << expected << " within (-pi,pi]\n";
        ok = false;
    }
    // A negative rotation combined with a negative pose yaw should also wrap correctly.
    mpc::AffineTransform2D m2 = mpc::affine_from_xytheta(0.0, 0.0, -170.0 * M_PI / 180.0);
    mpc::PlanarPose out2 = mpc::apply_mocap_to_world(mpc::PlanarPose{0.0, 0.0, -170.0 * M_PI / 180.0}, m2);
    if (out2.yaw > M_PI || out2.yaw <= -M_PI) {
        std::cerr << "    out2.yaw=" << out2.yaw << " is outside (-pi,pi]\n";
        ok = false;
    }
    return ok;
}

bool test_resolve_config_path_explicit_wins() {
    bool ok = true;
    // Explicit CLI value wins regardless of whether the default exists.
    {
        mpc::ConfigPathResolution r =
            mpc::resolve_config_path("/custom/path.txt", "/default/path.txt", true);
        if (r.source != mpc::ConfigPathSource::kCli || r.path != "/custom/path.txt") {
            std::cerr << "    (default_exists=true) expected kCli '/custom/path.txt', got "
                       << mpc::to_string(r.source) << " '" << r.path << "'\n";
            ok = false;
        }
    }
    {
        mpc::ConfigPathResolution r =
            mpc::resolve_config_path("/custom/path.txt", "/default/path.txt", false);
        if (r.source != mpc::ConfigPathSource::kCli || r.path != "/custom/path.txt") {
            std::cerr << "    (default_exists=false) expected kCli '/custom/path.txt', got "
                       << mpc::to_string(r.source) << " '" << r.path << "'\n";
            ok = false;
        }
    }
    return ok;
}

bool test_resolve_config_path_default_used_when_exists() {
    mpc::ConfigPathResolution r = mpc::resolve_config_path("", "/default/path.txt", true);
    if (r.source != mpc::ConfigPathSource::kDefault || r.path != "/default/path.txt") {
        std::cerr << "    expected kDefault '/default/path.txt', got " << mpc::to_string(r.source)
                   << " '" << r.path << "'\n";
        return false;
    }
    return true;
}

bool test_resolve_config_path_default_missing_falls_to_none() {
    bool ok = true;
    {
        mpc::ConfigPathResolution r = mpc::resolve_config_path("", "/default/path.txt", false);
        if (r.source != mpc::ConfigPathSource::kNone || !r.path.empty()) {
            std::cerr << "    (nonexistent default) expected kNone/'', got " << mpc::to_string(r.source)
                       << " '" << r.path << "'\n";
            ok = false;
        }
    }
    // An empty default_path (no compiled-in default at all) is also kNone,
    // even if default_exists were somehow true.
    {
        mpc::ConfigPathResolution r = mpc::resolve_config_path("", "", true);
        if (r.source != mpc::ConfigPathSource::kNone || !r.path.empty()) {
            std::cerr << "    (empty default_path) expected kNone/'', got " << mpc::to_string(r.source)
                       << " '" << r.path << "'\n";
            ok = false;
        }
    }
    return ok;
}

bool test_resolve_config_path_per_config_independence() {
    bool ok = true;
    // Mirrors optitrack_zmq_bridge.cpp's main(): one resolve_config_path()
    // call per config flag (--mocap-config, --map-config), independently.
    // An explicit --mocap-config must not make --map-config resolve as if
    // it too were explicit (and vice versa) -- each call only ever sees its
    // own arguments.
    mpc::ConfigPathResolution mocap =
        mpc::resolve_config_path("/explicit/mocap.txt", "/default/mocap.txt", true);
    mpc::ConfigPathResolution map = mpc::resolve_config_path("", "/default/map.json", false);
    if (mocap.source != mpc::ConfigPathSource::kCli || mocap.path != "/explicit/mocap.txt") {
        std::cerr << "    mocap: expected kCli '/explicit/mocap.txt', got " << mpc::to_string(mocap.source)
                   << " '" << mocap.path << "'\n";
        ok = false;
    }
    if (map.source != mpc::ConfigPathSource::kNone || !map.path.empty()) {
        std::cerr << "    map: expected kNone/'', got " << mpc::to_string(map.source) << " '" << map.path
                   << "'\n";
        ok = false;
    }
    // Swap which one is explicit -- the other, independently, still resolves
    // to its own default (this time present).
    mpc::ConfigPathResolution mocap2 = mpc::resolve_config_path("", "/default/mocap.txt", true);
    mpc::ConfigPathResolution map2 =
        mpc::resolve_config_path("/explicit/map.json", "/default/map.json", false);
    if (mocap2.source != mpc::ConfigPathSource::kDefault || mocap2.path != "/default/mocap.txt") {
        std::cerr << "    mocap2: expected kDefault '/default/mocap.txt', got "
                   << mpc::to_string(mocap2.source) << " '" << mocap2.path << "'\n";
        ok = false;
    }
    if (map2.source != mpc::ConfigPathSource::kCli || map2.path != "/explicit/map.json") {
        std::cerr << "    map2: expected kCli '/explicit/map.json', got " << mpc::to_string(map2.source)
                   << " '" << map2.path << "'\n";
        ok = false;
    }
    return ok;
}

}  // namespace

int main() {
    using TestFn = std::function<bool()>;
    const std::vector<std::pair<std::string, TestFn>> tests = {
        {"Frame round trip (build->parse), NatNet 2.10", test_frame_roundtrip_v2_10},
        {"Frame round trip (build->parse), NatNet 3.1", test_frame_roundtrip_v3_1},
        {"Frame round trip (build->parse), NatNet 4.0", test_frame_roundtrip_v4_0},
        {"Frame fuzz: every truncation prefix returns parse-error, never crashes",
         test_frame_fuzz_truncation},
        {"Frame fuzz: corrupted huge count field returns parse-error", test_frame_fuzz_corrupted_counts},
        {"Ping response round trip (build->parse) + wrong-message-type/truncation rejection",
         test_ping_response_roundtrip},
        {"quat_to_planar_yaw: known angles (0/90/180/-90deg), both up-axis conventions, tilted quat",
         test_quat_to_planar_yaw_known_angles},
        {"extract_planar_pose: position mapping for both up-axis conventions",
         test_extract_planar_pose_position_mapping},
        {"mocap_rotation: named function matches the user's hand-derived spec + "
         "extract_planar_pose(y_up=true) bit-for-bit",
         test_mocap_rotation_named_function},
        {"apply_planar_transform: composition + identity", test_planar_transform_composition},
        {"Downsampler: rate/driftlessness/latest-sample-wins/resync-after-stall",
         test_downsampler_timing},
        {"mocap-config: exact real-user fixture (all 5 fields, no warnings)",
         test_mocap_config_exact_fixture},
        {"mocap-config: Type=unicast (lowercase) variant", test_mocap_config_unicast_type},
        {"mocap-config: missing keys fall back to has_*=false", test_mocap_config_missing_keys_fallback},
        {"mocap-config: unknown key warns but does not derail parsing",
         test_mocap_config_unknown_key_warns},
        {"mocap-config: whitespace/CRLF/blank-line tolerance", test_mocap_config_whitespace_and_crlf},
        {"mocap-config: malformed line (no ':') warns but does not derail parsing",
         test_mocap_config_malformed_line},
        {"VersionAutoDetector: adopts after 5 consecutive same-candidate parses, one-shot, streak resets",
         test_version_auto_detector_adopts_after_five_consecutive},
        {"choose_multicast_interface: subnet contains server_ip", test_choose_multicast_interface_contains},
        {"choose_multicast_interface: no interface matches -> nullopt",
         test_choose_multicast_interface_no_match},
        {"choose_multicast_interface: multiple matches -> longest prefix wins (order-independent)",
         test_choose_multicast_interface_longest_prefix_wins},
        {"choose_multicast_interface: server_ip exactly equals an interface's own address",
         test_choose_multicast_interface_exact_server_address},
        {"choose_multicast_interface: malformed entries/server_ip skipped, never crash",
         test_choose_multicast_interface_malformed_entries_skipped},
        {"VersionGate: ping confirms before the deferral timeout", test_version_gate_ping_confirms_before_timeout},
        {"VersionGate: deferral timeout expiry falls back to the configured version",
         test_version_gate_timeout_expiry_fallback},
        {"VersionGate: no re-deferral after confirmation (incl. late-ping version upgrade)",
         test_version_gate_no_re_deferral},
        {"VersionGate: deferred-frame counting", test_version_gate_deferred_frame_counting},
        {"quat_to_roll_pitch_heading: known angles (pure roll/pitch/heading, both up conventions) + "
         "quat_to_planar_yaw cross-check",
         test_roll_pitch_heading_known_angles},
        {"NAT_MODELDEF round trip (build->parse), NatNet 2.10", test_modeldef_roundtrip_v2_10},
        {"NAT_MODELDEF round trip (build->parse), NatNet 4.0", test_modeldef_roundtrip_v4_0},
        {"NAT_MODELDEF fuzz: every truncation prefix returns parse-error, never crashes",
         test_modeldef_fuzz_truncation},
        {"NAT_MODELDEF: unrecognized dataset type stops gracefully (parse_ok=true, stopped_early=true)",
         test_modeldef_unknown_type_stops_gracefully},
        {"resolve_robot_ids: all requested names present", test_resolve_robot_ids_all_present},
        {"resolve_robot_ids: one requested name missing -> unresolved with available-names error",
         test_resolve_robot_ids_one_missing},
        {"resolve_robot_ids: duplicate name in modeldef -> first match wins",
         test_resolve_robot_ids_duplicate_name},
        {"resolve_robot_ids: auto mode -- name-match + index fallback", test_resolve_robot_ids_auto_mode},
        {"resolve_robot_ids: auto mode with empty modeldef still fully resolves",
         test_resolve_robot_ids_auto_mode_empty_modeldef},
        {"sanitize_topic_name: spaces/specials replaced, unchanged-when-safe, empty-input fallback",
         test_sanitize_topic_name_basic},
        {"resolve_auto_discovery: alias application + sanitized passthrough",
         test_auto_discovery_alias_and_sanitize},
        {"resolve_auto_discovery: late-appearing body across snapshots", test_auto_discovery_late_appearing_body},
        {"resolve_auto_discovery: empty Motive name uses the fallback", test_auto_discovery_empty_name_fallback},
        {"resolve_auto_discovery: duplicate published name disambiguated with _<id>",
         test_auto_discovery_disambiguation},
        {"find_unmatched_aliases: alias-to-missing-name reporting", test_find_unmatched_aliases},
        {"mocap_to_world_matrix: identity", test_mocap_to_world_identity},
        {"mocap_to_world_matrix: 90deg rotation + translation", test_mocap_to_world_rotation_translation},
        {"mocap_to_world_matrix: equivalence with the legacy x0/y0/theta0 path",
         test_mocap_to_world_equivalence_with_legacy},
        {"mocap_to_world_matrix: non-rigid (scale) warns but still applies the full affine transform",
         test_mocap_to_world_non_rigid_warns},
        {"mocap_to_world_matrix: bad bottom row is a hard error", test_mocap_to_world_bad_bottom_row},
        {"mocap_to_world_matrix: malformed shapes are hard errors", test_mocap_to_world_malformed_shapes},
        {"mocap_to_world_matrix: yaw composition + wrapping", test_mocap_to_world_yaw_composition_and_wrapping},
        {"resolve_config_path: explicit CLI value always wins", test_resolve_config_path_explicit_wins},
        {"resolve_config_path: default path used when it exists",
         test_resolve_config_path_default_used_when_exists},
        {"resolve_config_path: missing/empty default falls to kNone",
         test_resolve_config_path_default_missing_falls_to_none},
        {"resolve_config_path: per-config independence (mocap vs map)",
         test_resolve_config_path_per_config_independence},
    };

    int failed = 0;
    for (const auto& [name, fn] : tests) {
        std::cout << "[ RUN      ] " << name << "\n";
        const bool ok = fn();
        if (ok) {
            std::cout << "[       OK ] " << name << "\n";
        } else {
            std::cout << "[  FAILED  ] " << name << "\n";
            ++failed;
        }
    }

    if (failed == 0) {
        std::cout << "[  PASSED  ] " << tests.size() << " tests.\n";
        return 0;
    }
    std::cout << "[  FAILED  ] " << failed << " tests.\n";
    return 1;
}
