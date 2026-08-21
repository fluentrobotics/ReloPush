#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Pure logic for optitrack_zmq_bridge / fake_motive: NatNet wire-protocol
// parsing + building, up-axis-aware quaternion-to-planar-yaw conversion,
// planar rigid transforms, and a generic downsampler. No sockets here -- see
// src/optitrack_zmq_bridge.cpp (the real client) and src/fake_motive.cpp (the
// synthetic server used for hardware-free testing) for the thin I/O shells,
// mirroring how SimCore.h/.cpp keep robot_sim.cpp a thin shell around mpc::
// math.
//
// NATNET WIRE PROTOCOL (little-endian throughout; see NatNet SDK docs).
// Every multi-byte field below is read/written explicitly byte-by-byte in
// little-endian order in OptiTrackCore.cpp -- never via memcpy of a raw
// struct or reliance on host endianness -- so parsing is correct regardless
// of host byte order.
//
// Shared packet framing (BOTH command-port and data-port packets):
//   uint16 messageID
//   uint16 payloadSize
//   <payloadSize bytes of payload>
// Message IDs this v1 cares about: NAT_PING=0, NAT_PINGRESPONSE=1,
// NAT_FRAMEOFDATA=7 (see kNatPing/kNatPingResponse/kNatFrameOfData below).
//
// NAT_PINGRESPONSE payload: char[256] app name (NUL-padded/truncated),
// uchar[4] app version, uchar[4] NatNet version (major, minor, build, rev).
//
// NAT_FRAMEOFDATA payload:
//   int32 frameNumber
//   int32 nMarkerSets; per set: NUL-terminated ASCII name, int32 nMarkers,
//     nMarkers * (float x,y,z)
//   int32 nUnlabeledMarkers; that many * (float x,y,z)
//   int32 nRigidBodies; per body:
//     int32 id; float x,y,z; float qx,qy,qz,qw
//     if NatNet major < 3 (v1 only supports major==2 at >=2.6, where this
//       layout always applies): int32 nRigidMarkers;
//       nRigidMarkers*(float x,y,z); nRigidMarkers*int32 markerIDs;
//       nRigidMarkers*float markerSizes; float meanError; uint16 params
//     if NatNet major >= 3: float meanError; uint16 params
//     params bit 0x01 == tracking valid this frame
//   (skeletons, labeled markers, force plates, devices, timing info follow
//   -- v1 deliberately does NOT parse any of this and stops after rigid
//   bodies; arrival time is our own timestamp and the wire schema has no `t`
//   field anyway, so nothing downstream needs it.)
//
// DEFENSIVE PARSING: every read is bounds-checked against payloadSize (and
// against the actual buffer length) before it happens. A malformed or
// truncated packet -- including a payload that is truncated mid-field, or a
// declared count (nMarkerSets/nMarkers/nUnlabeledMarkers/nRigidBodies/
// nRigidMarkers) that is negative or too large for the bytes actually
// remaining -- returns a parse-error result. Never throws, never reads out
// of bounds, regardless of input.
namespace mpc {

constexpr std::uint16_t kNatPing = 0;
constexpr std::uint16_t kNatPingResponse = 1;
constexpr std::uint16_t kNatRequestModelDef = 4;
constexpr std::uint16_t kNatModelDef = 5;
constexpr std::uint16_t kNatFrameOfData = 7;

// The NatNet major/minor version governing a frame's wire layout (see the
// major<3 vs major>=3 branch documented above). v1 supports major 2 (assumed
// >=2.6 -- see header doc comment above; NatNet 2.0-2.5's older marker-less
// rigid body layout is NOT supported), 3, and 4 (3 and 4 share the same
// rigid-body layout).
struct NatNetVersion {
    int major = 3;
    int minor = 1;
};

// Peeks the shared [messageID, payloadSize] packet header without
// interpreting the payload -- lets a caller route an incoming datagram to
// the right parser (fake_motive checking for an incoming NAT_PING;
// optitrack_zmq_bridge checking NAT_PINGRESPONSE vs NAT_FRAMEOFDATA vs stray
// traffic) before calling parse_command_response()/parse_frame_of_data().
// ok=false if fewer than 4 bytes are available OR the declared payloadSize
// overruns the buffer -- callers must not trust message_id/payload_size
// otherwise.
struct PacketHeader {
    std::uint16_t message_id = 0;
    std::uint16_t payload_size = 0;
    bool ok = false;
};
PacketHeader peek_packet_header(const std::uint8_t* data, std::size_t size);

// ---------------------------------------------------------------------
// NAT_PING / NAT_PINGRESPONSE.
// ---------------------------------------------------------------------

// Builds a NAT_PING packet. app_name is optional identifying payload (real
// Motive tolerates either an empty payload or a client app name here -- see
// header doc comment); empty (the default) sends a zero-length payload.
std::vector<std::uint8_t> build_ping_packet(const std::string& app_name = "");

// Builds a NAT_PINGRESPONSE packet: app_name is NUL-padded/truncated to
// exactly 256 bytes (the fixed wire width), followed by the 4-byte
// app_version and 4-byte natnet_version arrays verbatim.
std::vector<std::uint8_t> build_ping_response(const std::string& app_name,
                                               const std::array<std::uint8_t, 4>& app_version,
                                               const std::array<std::uint8_t, 4>& natnet_version);

struct PingResponse {
    std::string app_name;
    std::array<std::uint8_t, 4> app_version{{0, 0, 0, 0}};
    std::array<std::uint8_t, 4> natnet_version{{0, 0, 0, 0}};
    bool ok = false;
    std::string error;  // set iff !ok.
};

// Parses a NAT_PINGRESPONSE packet (message_id must be kNatPingResponse).
// Bounds-checked against the 256+4+4=264-byte fixed payload; never throws.
PingResponse parse_command_response(const std::uint8_t* data, std::size_t size);

// ---------------------------------------------------------------------
// NAT_FRAMEOFDATA.
// ---------------------------------------------------------------------

struct RigidBodySample {
    std::int32_t id = 0;
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    double mean_error = 0.0;
    bool tracking_valid = true;
};

struct FrameOfData {
    std::int32_t frame_number = 0;
    std::vector<RigidBodySample> rigid_bodies;
    bool parse_ok = false;
    std::string error;  // set iff !parse_ok.
};

// Parses one NAT_FRAMEOFDATA packet's rigid bodies (marker sets/unlabeled
// markers are skipped over -- their byte layout is still walked so the
// rigid-body section is found correctly, but their contents are discarded;
// skeletons/labeled markers/force plates/devices/timing info are NOT walked
// at all -- v1 stops immediately after rigid bodies, see header doc
// comment). `version` selects the major<3 vs major>=3 rigid-body layout.
FrameOfData parse_frame_of_data(const std::uint8_t* data, std::size_t size,
                                 const NatNetVersion& version);

// Builds a NAT_FRAMEOFDATA packet carrying exactly the given rigid bodies
// (zero marker sets, zero unlabeled markers -- v1's fake-data needs go no
// further than rigid bodies) in the wire layout selected by `version`, for
// major<3 always writing nRigidMarkers=0 (no per-marker detail). Positions/
// quaternions/mean_error are written as IEEE-754 float32 (the wire's native
// width), so a build->parse round trip may lose double precision beyond
// float32 -- this is a real property of the wire format, not a bug.
std::vector<std::uint8_t> build_frame_of_data(std::int32_t frame_number,
                                                const std::vector<RigidBodySample>& rigid_bodies,
                                                const NatNetVersion& version);

// ---------------------------------------------------------------------
// NAT_REQUEST_MODELDEF / NAT_MODELDEF (round 4: rigid-body NAME resolution
// -- Motive assigns names to rigid bodies in its own UI; NAT_FRAMEOFDATA
// only ever carries numeric ids, never names, so a client that wants to map
// a human-chosen robot name to the right streaming id must separately
// request and parse the model definitions).
//
// NAT_MODELDEF payload: int32 nDatasets; per dataset: int32 type, then:
//   type 0 (markerset): NUL-terminated name, int32 nMarkers, nMarkers *
//     NUL-terminated marker names. v1 parses-and-skips this (never used).
//   type 1 (rigid body): NUL-terminated name, int32 ID, int32 parentID,
//     float offsetX/Y/Z (v1 discards the offset -- not part of
//     RigidBodyDef). If NatNet major >= 3, ADDITIONALLY: int32 nMarkers,
//     nMarkers * (float x,y,z) marker offsets, nMarkers * int32
//     activeLabels; if major >= 4, ADDITIONALLY nMarkers * NUL-terminated
//     marker names. major < 3 has none of this extra per-marker section at
//     all (same major<3/>=3/>=4 split as NAT_FRAMEOFDATA's rigid bodies).
//   type 2 (skeleton): NUL-terminated name, int32 id, int32 nRigidBodies,
//     then nRigidBodies nested rigid-body descriptions using the EXACT SAME
//     version-dependent layout as type 1 above. v1 parses-and-skips this
//     (the nested rigid bodies are walked correctly so the cursor lands in
//     the right place for whatever dataset follows, but never surfaced --
//     v1 has no use for skeleton-nested rigid bodies).
//   any other type: UNRECOGNIZED -- parsing stops here and returns
//     GRACEFULLY with whatever rigid bodies were already parsed (see
//     ModelDef::stopped_early below); this is NOT a parse failure, since a
//     future NatNet version could add dataset types 3+ (force plates,
//     devices, etc. per the frame-of-data trailer this codebase already
//     documents skipping) that this v1 has no way to know the layout of.
// Same defensive bounds-checking discipline as NAT_FRAMEOFDATA: any
// genuinely truncated/corrupt packet (a bounds violation mid-structure)
// yields parse_ok=false; never throws, never reads out of bounds.
// ---------------------------------------------------------------------

// Builds a NAT_REQUEST_MODELDEF packet (empty payload).
std::vector<std::uint8_t> build_modeldef_request();

struct RigidBodyDef {
    std::int32_t id = 0;
    std::string name;
    std::int32_t parent_id = 0;
};

struct ModelDef {
    std::vector<RigidBodyDef> rigid_bodies;
    // true iff parsing completed without a bounds/structural failure --
    // this is ALSO true when parsing stopped early on an unrecognized
    // dataset type (see stopped_early below); only a genuine truncation/
    // corruption sets this false.
    bool parse_ok = false;
    // true iff an unrecognized dataset type was encountered and parsing
    // stopped there (gracefully) rather than processing all nDatasets --
    // `rigid_bodies` still holds everything parsed before that point.
    bool stopped_early = false;
    std::string error;  // set iff !parse_ok, or informational when stopped_early.
};

// Parses a NAT_MODELDEF packet (message_id must be kNatModelDef) with the
// major<3/>=3/>=4 rigid-body layout selected by `version` (see this
// section's header doc comment).
ModelDef parse_modeldef(const std::uint8_t* data, std::size_t size, const NatNetVersion& version);

// ---- Builders (test/fake_motive use, so parser tests can round-trip). ----

// A minimal markerset descriptor for building a type=0 dataset -- content
// doesn't need semantic richness since v1 only ever walks PAST markersets.
struct MarkerSetDef {
    std::string name;
    std::vector<std::string> marker_names;
};

// A rigid-body descriptor for building a type=1 (or nested skeleton) rigid
// body, richer than the parser's own RigidBodyDef output so round-trip
// tests can exercise the version-dependent per-marker section with REAL
// (non-empty) data, not just the always-empty case. marker_offsets/
// marker_active_labels are only ever written for major>=3; marker_names
// only for major>=4 -- see build_modeldef()'s version parameter.
struct RigidBodyBuildDef {
    RigidBodyDef def;
    std::vector<std::array<float, 3>> marker_offsets;
    std::vector<std::int32_t> marker_active_labels;
    std::vector<std::string> marker_names;
};

// A minimal skeleton descriptor for building a type=2 dataset; nested
// rigid bodies use the SAME per-version layout as top-level ones.
struct SkeletonBuildDef {
    std::string name;
    std::int32_t id = 0;
    std::vector<RigidBodyBuildDef> nested_rigid_bodies;
};

// Builds a NAT_MODELDEF payload containing, in order: `marker_sets` (type=0
// each), `rigid_bodies` (type=1 each, top-level), `skeletons` (type=2
// each). Any of the three may be empty. `version` selects the
// major<3/>=3/>=4 rigid-body layout (applied to BOTH top-level and nested
// skeleton rigid bodies).
std::vector<std::uint8_t> build_modeldef(const std::vector<MarkerSetDef>& marker_sets,
                                           const std::vector<RigidBodyBuildDef>& rigid_bodies,
                                           const std::vector<SkeletonBuildDef>& skeletons,
                                           const NatNetVersion& version);

// ---------------------------------------------------------------------
// Up-axis-aware orientation/position conversion.
// ---------------------------------------------------------------------

// Wraps `angle` into (-pi, pi].
double normalize_angle(double angle);

// Extracts a planar yaw from a NatNet rigid-body quaternion (Hamilton
// convention, active rotation, right-handed: qx,qy,qz,qw), by rotating the
// body's local +X axis into the world frame and projecting onto the ground
// plane -- NOT a generic Euler-angle extraction, so it is meaningful for any
// pitch/roll the body may also have.
//
// Let v = R(q) * (1,0,0)^T = (vx, vy, vz), i.e. the first column of the
// standard quaternion rotation matrix:
//   vx = 1 - 2*(qy^2 + qz^2)
//   vy = 2*(qx*qy + qz*qw)
//   vz = 2*(qx*qz - qy*qw)
//
// y_up = false (Z-up streams -- ground plane is XY, up is +Z): yaw is the
// rotation about +Z, measured the standard right-handed way from the body's
// ground-projected +X axis:
//   yaw = atan2(vy, vx)
// (this is the ordinary "yaw from quaternion" formula for a Z-up,
// right-handed frame).
//
// y_up = true (Motive's native Y-up convention -- ground plane is XZ, up is
// +Y; vy above is the discarded "height" component of the rotated axis):
// Motive's own +Z axis points toward the viewer in its Y-up frame. To land
// in the SAME right-handed planar (x,y) convention used for y_up=false
// (i.e. so a caller never has to special-case which convention produced a
// given yaw), this function maps Motive's ground-plane (x_m, z_m) onto our
// planar (x, y) via (x, y) = (x_m, -z_m) -- see apply_planar_transform's
// sibling position mapping in extract_planar_pose() below, which uses this
// SAME (x, -z) convention for consistency between position and orientation.
// Applying that mapping to the rotated axis's ground components (vx, vz)
// gives the planar direction vector (vx, -vz), so:
//   yaw = atan2(-vz, vx)
//
// Both branches return a value in (-pi, pi] (via normalize_angle).
double quat_to_planar_yaw(double qx, double qy, double qz, double qw, bool y_up);

struct PlanarPose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// ---------------------------------------------------------------------
// THE MOCAP ROTATION (named per a live rig-calibration session with the
// project's user, verbatim): the FIXED frame change from Motive's raw
// Y-up convention to this project's planar convention -- the first stage
// of the pipeline pose_planner = ui_calibration(mocap_rotation(pose_motive_raw))
// [+ per-robot yaw_offset last] that optitrack_zmq_bridge.cpp's per-frame
// loop and MARS/src/simviz/MocapCore.h's compute_staged_transform_view()
// both implement, in that exact order. User's own words, without
// translation:
//   "Motive: x to right, z to down, y to upward from plane. Planner: x to
//    right, y to up, z to upward from plane."
// i.e. Motive's ground plane is (x_m, z_m) with z_m drawn "down" in a
// top-down view (y_m is the discarded height/up axis); the planner's
// ground plane is (x_p, y_p) with y_p drawn "up" -- so:
//   x_p = x_m
//   y_p = -z_m
// and correspondingly for HEADING (verified bit-for-bit against
// quat_to_planar_yaw's existing y_up=true branch below -- this function is
// a NAMED WRAPPER around that branch, not a new formula):
//   a body facing Motive's +X -> planner yaw 0
//   a body facing Motive's +Z -> planner yaw -pi/2
//   a rotation about Motive's +Y by +theta -> planner yaw +theta
// INVARIANT (exploited by MocapCore.h's compute_staged_transform_view() to
// reconstruct a "raw Motive" stage from an already-published planner pose,
// since the wire protocol never transmits raw position/quaternion): this
// mapping's heading formula is the IDENTITY on the numeric yaw value --
// quat_to_planar_yaw(...,y_up=true)'s output IS the physical rotation angle
// about Motive's own +Y axis (see quat_to_planar_yaw's doc comment/this
// function's .cpp derivation), so "raw yaw about Motive's Y axis" and
// "planner yaw immediately after mocap_rotation" are the SAME NUMBER --
// only ground-plane POSITION axes are relabeled/flipped by this step.
// `y_m` (Motive's height/up component) is accepted for API completeness
// (mirrors RigidBodySample's x,y,z layout so a caller can pass a sample's
// fields directly) but is NOT part of the output -- ground height has no
// planar-pose representation, exactly like extract_planar_pose()'s
// existing y_up=true position mapping.
PlanarPose mocap_rotation(double x_m, double y_m, double z_m, double qx, double qy, double qz,
                           double qw);

// Converts one RigidBodySample's raw NatNet-frame position+orientation into
// a planar (x, y, yaw) pose, using the SAME up-axis convention for position
// as quat_to_planar_yaw() uses for orientation (see its doc comment):
//   y_up = false (Z-up): (x, y) = (sample.x, sample.y)   [z is height, dropped]
//   y_up = true  (Y-up): delegates to mocap_rotation() above (bit-for-bit --
//     this IS the mocap rotation stage; see that function's doc comment).
PlanarPose extract_planar_pose(const RigidBodySample& sample, bool y_up);

// Applies a rigid 2D transform: treats (x0, y0, theta0) as the pose of
// `pose`'s own frame (e.g. the OptiTrack/Motive coordinate origin) expressed
// in the destination (world/map) frame, and maps `pose` (expressed in that
// source frame) into the destination frame:
//   x'   = x0 + pose.x*cos(theta0) - pose.y*sin(theta0)
//   y'   = y0 + pose.x*sin(theta0) + pose.y*cos(theta0)
//   yaw' = normalize_angle(pose.yaw + theta0)
// The identity transform (x0=y0=theta0=0) returns `pose` unchanged (up to
// yaw normalization). Composing apply_planar_transform(apply_planar_transform(p,
// x1,y1,t1), x2,y2,t2) equals applying the single combined transform that
// chains (x1,y1,t1) then (x2,y2,t2) -- exercised directly by the unit tests.
PlanarPose apply_planar_transform(const PlanarPose& pose, double x0, double y0, double theta0);

// ---------------------------------------------------------------------
// Mocap-to-world affine transform (round 6: --map-config's optional
// "mocap_to_world_matrix" -- supersedes the legacy x0/y0/theta0 rigid
// transform above with a full 2D homogeneous AFFINE transform, for a
// measured map-frame calibration that is not necessarily a pure
// rotation+translation, e.g. residual scale/shear from an imperfect
// alignment procedure).
// ---------------------------------------------------------------------

// A 2D affine transform in homogeneous form [[a,b,tx],[c,d,ty],[0,0,1]]
// (bottom row implicit -- validated, never stored, by
// parse_mocap_to_world_matrix() below). Defaults to identity.
struct AffineTransform2D {
    double a = 1.0, b = 0.0, tx = 0.0;
    double c = 0.0, d = 1.0, ty = 0.0;
};

struct MocapToWorldParseResult {
    AffineTransform2D transform;
    bool ok = false;                 // false iff a HARD error (malformed shape / bad bottom row).
    std::string error;                // set iff !ok.
    bool non_rigid_warning = false;   // true iff the rotation block isn't orthonormal/det~=+1.
    std::string warning;               // set iff non_rigid_warning (never fatal -- transform still applies).
};

// Parses/validates a "mocap_to_world_matrix" config value, already reduced
// by the caller to a plain nested array of doubles (OptiTrackCore has no
// JSON dependency -- the bridge does that reduction). `rows` must be
// EXACTLY 3 rows of EXACTLY 3 numbers each ([[a,b,tx],[c,d,ty],[0,0,1]]);
// any other shape is a HARD error (ok=false, e.g. a JSON array that isn't
// 3x3). The bottom row must equal [0,0,1] within 1e-6, else a HARD error
// -- a non-[0,0,1] bottom row cannot represent a valid 2D homogeneous
// transform at all, so this is never just a warning.
//
// The upper-left 2x2 rotation block R=[[a,b],[c,d]] is additionally
// checked for orthonormality (both columns unit length AND perpendicular)
// and det(R) ~= +1 (rules out reflections), tolerance 1e-3 for all of
// these checks together -- a FAILURE here is only a WARNING
// (non_rigid_warning=true, ok stays true): the transform still applies in
// full to positions (a genuine affine map, scale/shear included), but the
// yaw formula (atan2(c,a), see apply_mocap_to_world() below) is only exact
// for a pure rotation, so it becomes an approximation when this warns.
MocapToWorldParseResult parse_mocap_to_world_matrix(const std::vector<std::vector<double>>& rows);

// Applies the affine transform to a planar pose:
//   [x_w, y_w]^T = [[a,b],[c,d]] * [x_m, y_m]^T + [tx, ty]^T
//   yaw_w        = normalize_angle(atan2(c, a) + pose.yaw)
// atan2(c,a) recovers the transform's own rotation angle -- EXACT for a
// pure rotation; an approximation for a non-rigid transform (see
// parse_mocap_to_world_matrix()'s non_rigid_warning doc comment). Does NOT
// apply any yaw_offset -- same convention as apply_planar_transform()
// above, callers add that as a separate step afterward.
PlanarPose apply_mocap_to_world(const PlanarPose& pose, const AffineTransform2D& transform);

// Builds the AffineTransform2D that is EXACTLY equivalent to
// apply_planar_transform(pose, x0, y0, theta0) -- a pure rotation-by-theta0
// then translation-by-(x0,y0) -- exposed so callers (and this file's own
// tests) can verify/construct exact equivalence between the legacy
// x0/y0/theta0 path and the new matrix path. "Exact" here means within
// ordinary floating-point trig round-trip tolerance (sin/cos/atan2), not
// bit-identical -- see the unit test for the tolerance actually used.
AffineTransform2D affine_from_xytheta(double x0, double y0, double theta0);

// ---------------------------------------------------------------------
// Full 6-DoF Euler decomposition (round 4: diagnostics retention). The
// ZMQ localization payload stays planar-only ({"x","y","yaw"}) -- this is
// purely for --log-csv's full 6-DoF columns, so the two OTHER rotational
// DoF (besides heading, which is exactly quat_to_planar_yaw()'s output) are
// recorded for later analysis rather than discarded.
// ---------------------------------------------------------------------

struct EulerRollPitchHeading {
    double roll = 0.0;
    double pitch = 0.0;
    double heading = 0.0;  // == quat_to_planar_yaw(qx,qy,qz,qw,y_up), bit-for-bit.
};

// Decomposes a body quaternion into (roll, pitch, heading) using the
// standard intrinsic Tait-Bryan sequence R = R_heading * R_pitch * R_roll
// about the (up, second, forward) axes of whichever convention is active:
//
// y_up = false (Z-up): the ordinary aerospace/vehicle ZYX convention --
// heading about +Z (up), pitch about the resulting +Y, roll about the
// resulting +X. Closed form:
//   heading = atan2(2*(qw*qz+qx*qy), 1-2*(qy^2+qz^2))   [== quat_to_planar_yaw's z_up branch]
//   pitch   = asin(clamp(2*(qw*qy-qz*qx), -1, 1))
//   roll    = atan2(2*(qw*qx+qy*qz), 1-2*(qx^2+qy^2))
//
// y_up = true (Motive native): derived by conjugating the quaternion
// through the FIXED +90-degree rotation about native +X that maps Motive's
// axes onto the Z-up convention above (X_std=X, Y_std=-Z, Z_std=Y -- the
// SAME axis mapping extract_planar_pose()/quat_to_planar_yaw() already use
// for position/yaw), which works out to the substitution (qw,qx,qy,qz) ->
// (qw,qx,-qz,qy), then applying the SAME ZYX formulas above to the
// substituted quaternion. Closed form (after substitution, verified to
// reproduce quat_to_planar_yaw's y_up branch bit-for-bit):
//   heading = atan2(2*(qw*qy-qx*qz), 1-2*(qy^2+qz^2))    [== quat_to_planar_yaw's y_up branch]
//   pitch   = asin(clamp(-2*(qw*qz+qx*qy), -1, 1))
//   roll    = atan2(2*(qw*qx-qy*qz), 1-2*(qx^2+qz^2))
// In NATIVE Motive-frame terms, this sequence is: heading = rotation about
// native +Y (up); roll = rotation about native +X (the ground-plane axis
// that stays "x" in our planar convention); pitch = rotation about native
// -Z (negated because our planar "y" is -z_native -- see the position
// mapping's own doc comment).
//
// Gimbal lock (pitch near +-90 degrees) is handled only via asin's [-1,1]
// clamp (standard Tait-Bryan caveat: roll/heading become ill-defined/
// degenerate exactly at the pole) -- not specially resolved, since this
// codebase's targets are ground robots that stay close to flat.
EulerRollPitchHeading quat_to_roll_pitch_heading(double qx, double qy, double qz, double qw, bool y_up);

// ---------------------------------------------------------------------
// Mocap streaming-settings config file (--mocap-config). Motive can export
// its own "Streaming" panel settings as a plain key:value text file, e.g.
// (a real user-captured example, field order/casing exactly as Motive
// writes it):
//   ip_address:192.168.1.166
//   Type:Multicast
//   Command Port:1510
//   Data Port:1511
//   Multicast Interface:239.255.42.99
// Parsed as a PURE function of the file's text (no file I/O here -- the
// caller reads the file and passes its contents) so it is unit-testable
// without touching disk.
// ---------------------------------------------------------------------

// One line's outcome. `is_warning` is true for anything the caller should
// surface as a warning (unknown key, malformed line, unparseable value);
// false for a successfully-recognized key, so a caller can log every
// diagnostic at INFO and additionally flag just the warnings, or simply
// filter on is_warning for a warnings-only view.
struct MocapConfigDiagnostic {
    int line_number = 0;  // 1-based.
    std::string message;
    bool is_warning = false;
};

struct MocapConfigResult {
    bool has_server_ip = false;
    std::string server_ip;
    bool has_mode = false;
    std::string mode;  // "unicast" or "multicast" (normalized lowercase).
    bool has_command_port = false;
    int command_port = 0;
    bool has_data_port = false;
    int data_port = 0;
    // From the "Multicast Interface" key -- see parse_mocap_config_text()'s
    // doc comment below for why this is the multicast GROUP address, not a
    // network interface.
    bool has_multicast_group = false;
    std::string multicast_group;

    std::vector<MocapConfigDiagnostic> diagnostics;
};

// Parses Motive's exported streaming-settings text format: one "key:value"
// pair per line. Keys are matched case-insensitively after trimming
// surrounding whitespace from both the key and the value; blank lines and
// CRLF line endings are tolerated silently. A line with no ':' is reported
// as a malformed-line diagnostic (is_warning=true) and skipped -- never
// throws, never stops parsing the rest of the file. An unrecognized key is
// reported as an unknown-key diagnostic (is_warning=true) and otherwise
// ignored (not fatal). A recognized key whose value fails to parse (e.g. a
// non-numeric port) is reported as a diagnostic (is_warning=true) and that
// field is left unset (has_* stays false) -- same per-field independence
// discipline as SimCore.h's parse_sim_config_payload().
//
// Key mapping:
//   "ip_address"           -> server_ip (verbatim, trimmed).
//   "type"                 -> mode: "Multicast"/"Unicast" (any case) ->
//                              "multicast"/"unicast"; any other value is a
//                              diagnostic, mode left unset.
//   "command port"         -> command_port (integer).
//   "data port"             -> data_port (integer).
//   "multicast interface"  -> multicast_group. NOTE: despite its name, this
//                              is NOT a network interface selector -- it is
//                              the multicast GROUP address the client must
//                              join (confirmed against a real Motive-exported
//                              settings file, which places the standard
//                              NatNet default group 239.255.42.99 here).
//                              Motive's own UI simply mislabels this field.
MocapConfigResult parse_mocap_config_text(const std::string& text);

// ---------------------------------------------------------------------
// Config-file path resolution (--mocap-config / --map-config default-path
// selection). optitrack_zmq_bridge.cpp compiles in a default config
// directory (see its MOCAP_DEFAULT_CONFIG_DIR compile definition, set from
// CMakeLists.txt) so the two real project config files
// (MPC/config/mocap_config.txt and mocap_map_config.json) are
// picked up automatically on a zero-arg run, while an explicit
// --mocap-config/--map-config flag keeps behaving exactly as before. The
// decision itself is kept here as a pure function (no file I/O -- the
// caller stats the default path and passes the result in as
// `default_exists`) so it is unit-testable without touching disk, mirroring
// choose_multicast_interface()'s split of "gather real data via syscalls"
// (bridge) vs. "decide from that data" (here).
// ---------------------------------------------------------------------

// Which of {explicit CLI flag, compiled-in default file, nothing} a
// resolved config path actually came from -- kNone means neither was
// usable (no flag given AND the default file does not exist), and the
// caller should fall back to its own pre-existing no-flag behavior
// (built-in placeholder defaults for --mocap-config, identity transform for
// --map-config) exactly as it did before this feature existed.
enum class ConfigPathSource { kCli, kDefault, kNone };

struct ConfigPathResolution {
    std::string path;  // empty iff source == kNone.
    ConfigPathSource source = ConfigPathSource::kNone;
};

// Resolves a single config flag's effective path, independently of any
// other config flag (callers -- see optitrack_zmq_bridge.cpp's main() --
// call this once per config file, e.g. once for --mocap-config and
// separately for --map-config; the two calls never interact):
//   - cli_value non-empty (the flag was actually passed on the CLI) ->
//     kCli, path = cli_value verbatim, REGARDLESS of default_path/
//     default_exists. This is the "explicit always wins, unchanged
//     behavior" guarantee -- a missing/corrupt explicitly-given file must
//     keep failing exactly as it always has; this function does not
//     second-guess that by checking existence for the CLI case at all.
//   - cli_value empty AND default_exists -> kDefault, path = default_path.
//     Note: default_path is used EVEN IF a caller can't guarantee it is
//     well-formed -- a corrupt default file is a real, existing file that
//     was actively chosen, so it must fail to load the SAME way a corrupt
//     explicitly-given file does (an open/parse error), never silently
//     fall further back to kNone behavior.
//   - cli_value empty AND (default_path empty OR !default_exists) ->
//     kNone, path empty. The caller proceeds with its own pre-existing
//     no-flag behavior.
ConfigPathResolution resolve_config_path(const std::string& cli_value, const std::string& default_path,
                                          bool default_exists);

// Human-readable label for ConfigPathSource, e.g. for log lines.
std::string to_string(ConfigPathSource source);

// ---------------------------------------------------------------------
// NatNet parser-version auto-fallback (field robustness for when neither a
// ping response nor the user's --natnet-version guess is reliable).
// ---------------------------------------------------------------------

// Tries a fixed, ordered list of candidate NatNet versions against a frame
// packet that failed to parse with the caller's currently-configured
// version, and adopts the first candidate that parses cleanly for
// `required_consecutive` CONSECUTIVE calls to on_parse_failure() (tracking
// which candidate index won each time -- a different winning candidate, or
// a call where NO candidate parses, resets the streak back to 0; so does an
// interleaved on_parse_success(), since auto-fallback should only ever
// engage while the configured version is genuinely, persistently wrong).
// Adoption (Outcome::adopted) fires at most ONCE per detector instance --
// exactly the "loud one-time log line" the bridge needs -- after which
// has_adopted()/adopted_version() report the result and further
// on_parse_failure() calls simply return adopted=false forever.
//
// LIMITATION (see this task's report for the full explanation): this can
// only reliably distinguish the major<3 vs major>=3 wire-layout families --
// candidates that share a layout (e.g. 3.1 vs 4.0, which parse identically)
// are indistinguishable from packet bytes alone, so whichever such
// candidate is listed FIRST wins arbitrarily when the true stream is really
// the OTHER one; this is harmless for actually decoding the bytes (the
// layout is byte-identical either way), just cosmetically approximate in
// the reported minor version.
class VersionAutoDetector {
   public:
    VersionAutoDetector(std::vector<NatNetVersion> candidates, int required_consecutive = 5);

    struct Outcome {
        bool adopted = false;  // true exactly once, on the call that crosses the threshold.
        NatNetVersion adopted_version{};
    };

    // Call when parsing the CONFIGURED/current version failed for this
    // packet. Tries each candidate (in the order given at construction) via
    // parse_frame_of_data on the SAME bytes; the first one that parses OK
    // is this call's "winner" for streak-tracking purposes.
    Outcome on_parse_failure(const std::uint8_t* data, std::size_t size);

    // Call when the configured/current version successfully parsed a frame
    // on its own -- resets any in-progress streak (auto-fallback should
    // never fire based on a streak that had a "things are fine" call in the
    // middle of it).
    void on_parse_success();

    bool has_adopted() const { return adopted_; }
    NatNetVersion adopted_version() const { return adopted_version_; }

   private:
    std::vector<NatNetVersion> candidates_;
    int required_consecutive_;
    int streak_ = 0;
    int streak_candidate_index_ = -1;
    bool adopted_ = false;
    NatNetVersion adopted_version_{};
};

// ---------------------------------------------------------------------
// Multicast interface auto-selection (round 3: dual-homed-machine fix).
// ---------------------------------------------------------------------

// One IPv4 network interface, as gathered by the bridge via getifaddrs()
// (kept here as plain data so choose_multicast_interface() below stays a
// pure function, unit-testable with synthetic lists -- no socket/syscall
// code lives in OptiTrackCore itself).
struct IfaceInfo {
    std::string name;
    std::string addr;     // dotted-decimal IPv4.
    std::string netmask;  // dotted-decimal IPv4.
};

// Picks which local interface's IPv4 subnet contains `server_ip` -- i.e.
// (server_ip & netmask) == (iface.addr & netmask) -- for the multicast
// IP_ADD_MEMBERSHIP join's imr_interface. On a dual-homed machine,
// INADDR_ANY lets the kernel pick based on the DEFAULT ROUTE, which may be
// on a completely different network than the one Motive is actually
// reachable on (confirmed on a live run: default-route interface -> zero
// frames; the Motive-subnet interface -> works) -- this function exists so
// the bridge can pick the RIGHT one automatically instead of requiring a
// manual --multicast-interface override every time.
//
// Multiple interfaces whose subnet contains server_ip: the LONGEST prefix
// (most specific/smallest subnet) wins; ties keep whichever appears FIRST
// in `interfaces`. No match: returns std::nullopt (caller falls back to
// INADDR_ANY, today's behavior). Malformed addr/netmask entries in the
// input are skipped defensively -- never throws.
std::optional<IfaceInfo> choose_multicast_interface(const std::string& server_ip,
                                                       const std::vector<IfaceInfo>& interfaces);

// ---------------------------------------------------------------------
// Version confirmation gate (round 3: fixes a real live-run bug).
// ---------------------------------------------------------------------

// BUG THIS FIXES (observed live against real Motive, reproduced twice):
// before the NatNet version was confirmed, data frames were parsed
// immediately using the CLI fallback version. A wrong-version parse does
// NOT always fail cleanly -- parsing a genuine NatNet 2.10 frame's bytes
// with the 3.1 (major>=3) layout can spuriously return parse_ok=true with
// garbage values (a nonsense rigid-body id, positions like 1e+35), because
// the two layouts' leading bytes overlap enough to stay in-bounds without
// being semantically meaningful. This means VersionAutoDetector's
// on_parse_failure() -- which only ever runs when parsing genuinely FAILS
// -- never gets a chance to fire: the wrong-version parse doesn't fail, it
// lies. Gating parsing entirely until the version is confirmed removes the
// wrong-version window instead of trying to detect a way out of it after
// the fact.
//
// Confirmation happens exactly once per gate, via WHICHEVER comes first:
//   - confirm_via_ping(): a real NAT_PINGRESPONSE was processed (the fast,
//     authoritative path -- typically well under 1s on a real network).
//   - should_parse_now() observing elapsed_s >= the deferral timeout with
//     no ping yet: falls back to whatever version the caller supplies (and
//     VersionAutoDetector remains available as a backup after this, since
//     a timeout-fallback is a GUESS, not authoritative -- see the bridge's
//     own !version_discovered-style gating around it).
// Once confirmed, deferral NEVER happens again for the gate's lifetime.
class VersionGate {
   public:
    explicit VersionGate(double defer_timeout_s = 3.0) : defer_timeout_s_(defer_timeout_s) {}

    bool confirmed() const { return confirmed_; }
    bool confirmed_via_ping() const { return confirmed_via_ping_; }
    NatNetVersion confirmed_version() const { return confirmed_version_; }
    double confirmed_at_s() const { return confirmed_at_s_; }
    long deferred_count() const { return deferred_count_; }

    // True only for the single call (to this method OR should_parse_now())
    // that actually performed the confirmation -- callers must check this
    // IMMEDIATELY after that call to log the one-time "version confirmed"
    // line; any later, unrelated call to either method resets it to false.
    bool just_confirmed() const { return just_confirmed_; }

    // Call the instant a real NAT_PINGRESPONSE is processed. The FIRST call
    // performs the confirmation (just_confirmed()=true, deferral ends
    // permanently). A call after already confirmed does NOT re-fire
    // just_confirmed() (confirmation/deferral-ending is a one-time event),
    // but ALWAYS still updates confirmed_version()/confirmed_via_ping() to
    // reflect the ping's authoritative answer -- even overriding an earlier
    // timeout-fallback guess, since a real ping response is always more
    // trustworthy than a guess.
    void confirm_via_ping(const NatNetVersion& version, double elapsed_s);

    // Call once per incoming data frame, BEFORE attempting to parse it,
    // with elapsed_s measured from the same monotonic zero-point used by
    // confirm_via_ping()'s own elapsed_s. Returns true iff the caller
    // should proceed to parse this frame now (already confirmed, OR this
    // call itself just performed the timeout-expiry fallback confirmation
    // using `fallback_version`); false (incrementing deferred_count()) iff
    // still waiting.
    bool should_parse_now(double elapsed_s, const NatNetVersion& fallback_version);

   private:
    double defer_timeout_s_;
    bool confirmed_ = false;
    bool just_confirmed_ = false;
    bool confirmed_via_ping_ = false;
    NatNetVersion confirmed_version_{};
    double confirmed_at_s_ = 0.0;
    long deferred_count_ = 0;
};

// ---------------------------------------------------------------------
// Robot-name -> rigid-body-id resolution (round 4: NAME resolution via
// NAT_MODELDEF -- Motive assigns names in its own UI; a robot's CLI name
// (e.g. "robot1") is generally NOT the same string as its Motive asset
// name, so matching them requires the modeldef inventory, not just a
// string-equality guess against frame data (which carries no names at
// all).
// ---------------------------------------------------------------------

enum class RobotIdSource { kCliId, kNameMatch, kIndexFallback, kUnresolved };

struct RobotResolutionEntry {
    std::string robot_name;
    bool resolved = false;
    std::int32_t rigid_body_id = 0;  // valid iff resolved.
    RobotIdSource source = RobotIdSource::kUnresolved;
    // Non-empty iff an EXPLICITLY requested name (--rigid-body-names) was
    // not found in `modeldef` -- lists every currently-known name so the
    // caller's periodic retry log is actionable ("did you mean ...").
    std::string error;
};

struct ResolutionResult {
    std::vector<RobotResolutionEntry> robots;
    bool all_resolved = false;
};

// Human-readable label for RobotIdSource, e.g. for log lines.
std::string to_string(RobotIdSource source);

// Resolves robot_names[i] to a rigid-body id via `modeldef`, in one of two
// modes (this function does NOT handle the --rigid-body-ids case at all --
// that mapping is immediate/trivial and needs no modeldef, so the caller
// should just build robot configs directly for it, exactly as before round
// 4; this function exists only for the NEW name-based paths):
//   - explicit_names non-empty (parallel to robot_names, same size, one
//     entry per robot -- an empty string at index i means "no name was
//     requested for this robot"): each such robot MUST match a modeldef
//     entry by EXACT (case-sensitive) name; a miss leaves that entry
//     unresolved with `error` listing every name currently in `modeldef`
//     (empty modeldef -> "(none discovered yet)") -- callers should retry
//     this function as fresh modeldef snapshots arrive.
//   - explicit_names EMPTY ("auto" mode): first tries robot_names[i]
//     ITSELF as a Motive name match; any robot still unresolved after that
//     pass falls back to the 1-based index position (id = i+1)
//     UNCONDITIONALLY -- even with an empty/unavailable `modeldef` -- so
//     auto mode can never block forever waiting on Motive (mirrors the
//     pre-round-4 "default ids 1..N" convention exactly).
// Duplicate names in `modeldef`: the FIRST match (lowest index in
// `modeldef`) wins -- deterministic, same tie-break precedent as
// choose_multicast_interface().
ResolutionResult resolve_robot_ids(const std::vector<std::string>& robot_names,
                                     const std::vector<std::string>& explicit_names,
                                     const std::vector<RigidBodyDef>& modeldef);

// ---------------------------------------------------------------------
// Auto-discovery publish mode (round 5: zero-arg operation -- when the
// user gives NO --robots/--rigid-body-ids/--rigid-body-names at all, the
// bridge publishes EVERY rigid body Motive's modeldef reports, one topic
// per body, on a single bound PUB port).
// ---------------------------------------------------------------------

// Sanitizes a Motive asset name into a topic-safe published name: any
// character outside [A-Za-z0-9_-] (including spaces) becomes '_' --
// replacement, not removal, so a NON-EMPTY input always yields a
// same-length, non-empty output. An EMPTY input (the only way to get a
// degenerate empty result) uses `fallback` instead (used_fallback=true) --
// callers should pass something like "body<id>" so a nameless Motive asset
// still gets a valid, unique topic.
struct SanitizeResult {
    std::string sanitized;
    bool changed = false;        // true iff any character was replaced.
    bool used_fallback = false;  // true iff `name` was empty and `fallback` was used instead.
};
SanitizeResult sanitize_topic_name(const std::string& name, const std::string& fallback);

struct AutoDiscoveredBody {
    std::int32_t rigid_body_id = 0;
    std::string motive_name;     // raw, exactly as given in modeldef.
    std::string published_name;  // final topic-safe name: alias, sanitized name, or a
                                   // disambiguated/fallback variant -- see the flags below.
    bool from_alias = false;               // published_name came from the `aliases` map.
    bool name_sanitized = false;           // (from_alias == false) sanitize_topic_name() changed it.
    bool used_empty_fallback = false;      // (from_alias == false) motive_name was empty.
    bool disambiguated = false;            // published_name collided with an earlier body in this
                                             // SAME resolve_auto_discovery() call; suffixed with
                                             // "_<id>" to stay unique. Deterministic: bodies are
                                             // processed in `modeldef` order, so whichever appears
                                             // FIRST keeps the un-suffixed name (same tie-break
                                             // philosophy as resolve_robot_ids()'s duplicate-name
                                             // handling).
};

// Resolves the auto-discovery publish list from a modeldef snapshot and an
// alias map (Motive name -> published name, from --map-config's
// "aliases" key). One entry per modeldef rigid body, in modeldef order.
// Pure/stateless -- callers wanting "what's NEW since last time" (to log
// only deltas) should diff two calls' results themselves (see
// optitrack_zmq_bridge.cpp's own body-state tracking).
std::vector<AutoDiscoveredBody> resolve_auto_discovery(
    const std::vector<RigidBodyDef>& modeldef,
    const std::unordered_map<std::string, std::string>& aliases);

// Alias map entries whose key (a Motive asset name) does NOT match any
// body currently in `modeldef` -- informational only (the asset may simply
// not have appeared yet), never an error. Order matches iteration order of
// `aliases` (unspecified, since it's an unordered_map).
std::vector<std::string> find_unmatched_aliases(const std::unordered_map<std::string, std::string>& aliases,
                                                   const std::vector<RigidBodyDef>& modeldef);

// ---------------------------------------------------------------------
// Downsampler: emits at most target_hz, always the LATEST fed sample.
// ---------------------------------------------------------------------

// Generic latest-sample-wins rate gate. Templated (header-only) so it can
// gate any per-call payload (a single pose, a whole per-robot pose map,
// etc.) without OptiTrackCore.cpp needing to know the payload type.
//
// Call feed() once per incoming sample, unconditionally, with its arrival
// time (monotonic seconds, any epoch -- only differences matter). Every
// call's `value` OVERWRITES whatever was pending -- nothing is ever queued
// or averaged -- so whichever call happens to cross the next scheduled
// emission boundary is the one whose value gets copied to `out` and
// returned true; all other calls (the vast majority, when target_hz is well
// below the feed rate) return false and their values are simply discarded.
// This is what "latest-sample-wins" means: the output is always the most
// recently fed value AT THE MOMENT of emission, never the first one seen
// since the last emission and never a blend of several.
//
// The emission schedule advances by exactly one period per emission (so the
// long-run output rate does not drift versus wall time even if feed() is
// called at a jittery/bursty rate) UNLESS the caller falls behind by more
// than one full period (e.g. after a stall), in which case the schedule
// resyncs to the current call's time -- mirrors robot_sim.cpp's own
// real-time-pacing fallback -- rather than bursting a train of catch-up
// emissions on the next several calls.
template <typename T>
class Downsampler {
   public:
    // target_hz <= 0 means "no downsampling": every feed() emits.
    explicit Downsampler(double target_hz)
        : period_s_(target_hz > 0.0 ? 1.0 / target_hz : 0.0) {}

    bool feed(double t_now, const T& value, T& out) {
        if (period_s_ <= 0.0) {
            out = value;
            return true;
        }
        if (first_) {
            first_ = false;
            next_due_ = t_now;  // emit immediately on the very first sample.
        }
        if (t_now < next_due_) {
            return false;
        }
        out = value;
        next_due_ += period_s_;
        if (next_due_ < t_now) {
            // Fell behind by more than one period -- resync instead of
            // bursting a catch-up train of emissions on subsequent calls.
            next_due_ = t_now + period_s_;
        }
        return true;
    }

    // Re-arms the "emit immediately on the next feed()" behavior (e.g. after
    // a deliberate long gap the caller knows about, such as a tracking-loss
    // recovery) without constructing a new Downsampler.
    void reset() { first_ = true; }

    double target_hz() const { return period_s_ > 0.0 ? 1.0 / period_s_ : 0.0; }
    double period_s() const { return period_s_; }

   private:
    double period_s_ = 0.0;
    bool first_ = true;
    double next_due_ = 0.0;
};

} // namespace mpc
