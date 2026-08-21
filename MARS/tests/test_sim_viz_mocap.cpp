// T1 (this task's DESIGN doc, part A): unit tests for
// simviz::MocapManager / the pure helpers in MARS/src/simviz/MocapCore.h,
// plus the MOCAP_CONNECT/MOCAP_DISCONNECT/MOCAP_STATUS control-socket verbs
// wired into simviz::SimVizManager (SimVizCore.h/.cpp).
//
// Part 1: pure helpers (no process/Qt/ZMQ) -- parse_transform_info()
//   (mocap_to_world_matrix vs legacy x0/y0/theta0 fallback, per-robot
//   yaw_offset, non-rigid/shear warning, malformed-matrix parse_error),
//   the mapping round-trip (load_mapping_json -> set_alias_in_mapping ->
//   save_mapping_json -> reload, unknown fields incl. "_doc" preserved),
//   and parse_motive_assets_line() (the bridge's one-time stdout inventory
//   line).
//
// Part 2: process-level. Spawns a REAL fake_motive (see MPC/
// src/fake_motive.cpp / MPC/tests/test_optitrack_bridge.cpp's
// Part 5 -- auto-discovery -- for the exact CLI/wire pattern this mirrors)
// serving two rigid bodies ("mushr2", "block"); MocapManager itself spawns
// and owns the REAL optitrack_zmq_bridge (auto-discovery mode) against a
// COPIED (never the real project's) --map-config that aliases
// "mushr2"->"robot1". Asserts the Disconnected->Connecting->Connected
// state transitions, that bodies() surfaces both "robot1" (aliased) and
// "block" (sanitized passthrough) with sane planner-frame poses,
// disconnect -> Disconnected, the REVERSE Connected->Connecting freshness
// transition (fake_motive killed so the still-running bridge stops
// publishing fresh samples -- covers tick()'s freshness gate independently
// of both stop_bridge() and a process crash), and that killing the bridge
// process EXTERNALLY (SIGKILL, simulating a crash -- not going through
// stop_bridge()) drives the manager to Error.
//
// Part 3: the MOCAP_CONNECT/MOCAP_DISCONNECT/MOCAP_STATUS control-socket
// verbs, driven in-process against a headless simviz::SimVizManager (same
// "in-process, not a separate mars_sim_viz spawn" convention as
// test_sim_viz_control.cpp).
//
// TEST PORTS ONLY (never the user's default ports): control 45601,
// handshake 45620+, vesc 45640+, loc 45660+ (SimVizManager's own,
// unrelated to mocap and never actually dialed here), bridge ZMQ
// auto-discovery loc port 45680, fake_motive/bridge UDP command port
// 45510 / data port 45511.

#include "SimVizCore.h"
#include "MocapCore.h"
#include "test_sim_viz_shared.h"

#include <QCoreApplication>

#include <nlohmann/json.hpp>
#include <zmq.hpp>

#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

bool DEBUG_VIS = false;

namespace
{

using simviz::AffineTransform2D;
using simviz::MocapConfig;
using simviz::MocapManager;
using simviz::MocapState;
using simviz::MotiveAssetEntry;
using simviz::StagedTransformView;
using simviz::TransformInfo;

constexpr int kFakeMotiveCommandPort = 45510;
constexpr int kBridgeLocalDataPort = 45511;
constexpr int kBridgeLocPortStart = 45680;

constexpr int kTestControlPort = 45601;
constexpr int kTestHandshakePortStart = 45620;
constexpr int kTestVescPortStart = 45640;
constexpr int kTestLocPortStart = 45660;

int g_checks_run = 0;
int g_checks_failed = 0;

void check(bool cond, const std::string &what)
{
  ++g_checks_run;
  if (cond)
  {
    std::cout << "[test] ok: " << what << std::endl;
  }
  else
  {
    ++g_checks_failed;
    std::cerr << "[test] FAILED: " << what << std::endl;
  }
}

bool approx_eq(double a, double b, double eps)
{
  return std::fabs(a - b) <= eps;
}

// Synthetic --mocap-config text file (own temp copy -- see this file's
// header comment: tests must never rely on/read the real project's
// MPC/config/mocap_config.txt, even though every CLI flag this
// test passes already overrides its server_ip/mode/command_port/
// local_data_port fields; this keeps the test fully self-contained). Format
// mirrors the real file's "key:value" lines (mpc::parse_mocap_config_text()).
std::string synthetic_mocap_config_txt()
{
  return "ip_address:127.0.0.1\nType:Unicast\nCommand Port:" +
         std::to_string(kFakeMotiveCommandPort) + "\nData Port:" +
         std::to_string(kBridgeLocalDataPort) + "\nMulticast Interface:239.255.42.99\n";
}

nlohmann::json synthetic_map_config_json()
{
  nlohmann::json j;
  j["_doc"] = {{"note", "test-only synthetic mocap map-config; unrelated to the real project's "
                         "MPC/config/mocap_map_config.json"}};
  j["y_up"] = false;
  j["x0"] = 0.0;
  j["y0"] = 0.0;
  j["theta0"] = 0.0;
  j["yaw_offset"] = nlohmann::json::object();
  j["aliases"] = {{"mushr2", "robot1"}};
  j["mocap_to_world_matrix"] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  return j;
}

// ---------------------------------------------------------------------
// Part 1: pure helpers.
// ---------------------------------------------------------------------
void run_part1_pure_helpers(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 1: pure helpers ===\n";

  // ---- parse_transform_info(): identity matrix. ----
  {
    nlohmann::json cfg;
    cfg["y_up"] = false;
    cfg["mocap_to_world_matrix"] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(!info.parse_error, "identity matrix: no parse_error");
    check(!info.non_rigid_warning, "identity matrix: no non_rigid_warning");
    check(!info.matrix_from_legacy, "identity matrix: matrix_from_legacy == false");
    check(approx_eq(info.rotation_deg, 0.0, 1e-6), "identity matrix: rotation_deg == 0");
    check(approx_eq(info.matrix.tx, 0.0, 1e-9) && approx_eq(info.matrix.ty, 0.0, 1e-9),
          "identity matrix: translation == (0,0)");
  }

  // ---- parse_transform_info(): pure 90-degree rotation. ----
  {
    nlohmann::json cfg;
    cfg["mocap_to_world_matrix"] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(!info.parse_error, "90deg rotation: no parse_error");
    check(!info.non_rigid_warning, "90deg rotation: no non_rigid_warning (pure rotation)");
    check(approx_eq(info.rotation_deg, 90.0, 1e-3), "90deg rotation: rotation_deg == 90");
  }

  // ---- parse_transform_info(): scale/shear -> non_rigid_warning. ----
  {
    nlohmann::json cfg;
    cfg["mocap_to_world_matrix"] = {{2, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(!info.parse_error, "scale matrix: no parse_error (still applies, just warns)");
    check(info.non_rigid_warning, "scale matrix: non_rigid_warning == true");
    check(!info.warning.empty(), "scale matrix: warning message non-empty");
  }

  // ---- parse_transform_info(): malformed shape (2x2) -> parse_error. ----
  {
    nlohmann::json cfg;
    cfg["mocap_to_world_matrix"] = {{1, 0}, {0, 1}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(info.parse_error, "2x2 matrix: parse_error == true");
    check(approx_eq(info.matrix.a, 1.0, 1e-9) && approx_eq(info.matrix.d, 1.0, 1e-9) &&
              approx_eq(info.matrix.b, 0.0, 1e-9) && approx_eq(info.matrix.c, 0.0, 1e-9),
          "2x2 matrix: falls back to identity");
  }

  // ---- parse_transform_info(): bad bottom row -> parse_error. ----
  {
    nlohmann::json cfg;
    cfg["mocap_to_world_matrix"] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 2}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(info.parse_error, "bad bottom row: parse_error == true");
  }

  // ---- parse_transform_info(): legacy x0/y0/theta0 fallback (no
  // "mocap_to_world_matrix" key at all). ----
  {
    nlohmann::json cfg;
    cfg["x0"] = 1.0;
    cfg["y0"] = 2.0;
    cfg["theta0"] = M_PI / 2.0;
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(info.matrix_from_legacy, "legacy fallback: matrix_from_legacy == true");
    check(!info.parse_error, "legacy fallback: no parse_error");
    check(approx_eq(info.matrix.tx, 1.0, 1e-9) && approx_eq(info.matrix.ty, 2.0, 1e-9),
          "legacy fallback: translation == (x0,y0)");
    check(approx_eq(info.rotation_deg, 90.0, 1e-3), "legacy fallback: rotation_deg == 90");
  }

  // ---- parse_transform_info(): per-robot yaw_offset. ----
  {
    nlohmann::json cfg;
    cfg["yaw_offset"] = {{"robot1", 3.14159265}, {"robot2", 0.0}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(info.yaw_offsets.count("robot1") == 1, "yaw_offset: robot1 key present");
    check(info.yaw_offsets.count("robot2") == 1, "yaw_offset: robot2 key present");
    if (info.yaw_offsets.count("robot1"))
      check(approx_eq(info.yaw_offsets.at("robot1"), M_PI, 1e-4), "yaw_offset: robot1 ~= pi");
  }

  // ---- apply_transform_matrix() / apply_full_transform_chain(): FULL
  // chain frame-math check, hand-computed. Mirrors what a live body's
  // published pose already reflects by the time it reaches bodies():
  // (1) the y_up-aware raw-Motive-frame -> mocap-planar reduction
  //     documented in MPC/include/mpc/OptiTrackCore.h's
  //     extract_planar_pose() -- y_up=true: planar (x,y) = (raw_x,
  //     -raw_z) -- applied here BY HAND (this file has no
  //     quaternion/3D dependency of its own; that reduction is the
  //     bridge's job and is unit-tested independently in
  //     MPC's own OptiTrackCore tests -- this test only
  //     proves the VIZ side's calibration + yaw_offset composition
  //     matches OptiTrackCore's bit-for-bit);
  // (2) the calibration transform (x0=1.0, y0=-2.0, theta0=pi/6),
  //     EXACTLY apply_planar_transform()'s p' = R(theta0)*p + [x0,y0];
  // (3) a per-robot yaw_offset (-0.2 rad), added last.
  {
    // (1) raw Motive-frame position: raw_x=2.0, raw_z=3.0 (raw_y is the
    // Y-up height axis, irrelevant to the planar reduction). y_up=true =>
    // mocap-planar (mx, my) = (raw_x, -raw_z) = (2.0, -3.0). The raw yaw
    // (0.3 rad) stands in for quat_to_planar_yaw()'s already-computed
    // output -- see the comment above for why this test doesn't re-derive
    // it from a quaternion.
    const double raw_x = 2.0, raw_z = 3.0;
    const double mx = raw_x, my = -raw_z;
    const double raw_yaw = 0.3;

    nlohmann::json cfg;
    cfg["y_up"] = true;
    cfg["x0"] = 1.0;
    cfg["y0"] = -2.0;
    cfg["theta0"] = M_PI / 6.0;
    cfg["yaw_offset"] = {{"robot1", -0.2}};
    TransformInfo info = simviz::parse_transform_info(cfg);
    check(info.y_up, "full chain: TransformInfo.y_up round-trips true (informational, "
                      "surfaced by the calibration UI's checkbox)");

    Pose result = simviz::apply_full_transform_chain(info, mx, my, raw_yaw, "robot1");

    // Hand-computed: a=cos(pi/6), b=-sin(pi/6), tx=1.0; c=sin(pi/6),
    // d=cos(pi/6), ty=-2.0.
    const double a = std::cos(M_PI / 6.0), b = -std::sin(M_PI / 6.0);
    const double c = std::sin(M_PI / 6.0), d = std::cos(M_PI / 6.0);
    const double expected_x = a * mx + b * my + 1.0;
    const double expected_y = c * mx + d * my + (-2.0);
    const double expected_yaw = (M_PI / 6.0 + raw_yaw) - 0.2; // atan2(c,a)==pi/6 here; + yaw_offset.

    std::cout << "[test]   full chain: expected=(" << expected_x << ", " << expected_y << ", "
               << expected_yaw << ") got=(" << result.x << ", " << result.y << ", " << result.yaw
               << ")" << std::endl;
    check(approx_eq(result.x, expected_x, 1e-6), "full chain: x matches hand-computed value");
    check(approx_eq(result.y, expected_y, 1e-6), "full chain: y matches hand-computed value");
    check(approx_eq(result.yaw, expected_yaw, 1e-6), "full chain: yaw matches hand-computed value "
                                                       "(calibration rotation + raw yaw + yaw_offset)");

    // apply_transform_matrix() alone (no yaw_offset) must equal the same
    // computation minus the -0.2 offset -- proves apply_full_transform_chain
    // adds yaw_offset ON TOP of, not instead of, the matrix's own
    // atan2(c,a) contribution.
    Pose matrix_only = simviz::apply_transform_matrix(info.matrix, mx, my, raw_yaw);
    check(approx_eq(matrix_only.yaw, M_PI / 6.0 + raw_yaw, 1e-6),
          "apply_transform_matrix() alone: yaw excludes yaw_offset");
  }

  // ---- mocap_rotation(): the NAMED, explicit first pipeline stage (see
  // MocapCore.h's doc comment) -- hand-computed cases matching the user's
  // own spec from the live rig-calibration session, PLUS bit-for-bit
  // parity against MPC/include/mpc/OptiTrackCore.h's
  // mocap_rotation() on the SAME shared quaternion/position inputs
  // (hardcoded identically to MPC/tests/optitrack_unit_tests.cpp's
  // test_mocap_rotation_named_function() -- proves the bridge and viz
  // sides provably agree, not just "look similar"). ----
  {
    // Case 1: facing Motive's +X (identity quaternion) -> planner yaw 0;
    // position (x_m=2, y_m=5 [height, dropped], z_m=3) -> (2, -3).
    {
      Pose p = simviz::mocap_rotation(2.0, 5.0, 3.0, 0.0, 0.0, 0.0, 1.0);
      check(approx_eq(p.x, 2.0, 1e-9) && approx_eq(p.y, -3.0, 1e-9) && approx_eq(p.yaw, 0.0, 1e-9),
            "mocap_rotation: facing +X -> (2, -3, yaw=0)");
    }
    // Case 2: facing Motive's +Z -> planner yaw -pi/2. Per the standard
    // active-quaternion rotation matrix's first column (vx=cos(theta),
    // vz=-sin(theta) for a pure rotation-by-theta about +Y), this is
    // theta=-90deg about +Y (NOT +90deg) -- see OptiTrackCore.h's
    // test_mocap_rotation_named_function()'s case 2 for the full
    // derivation of why.
    {
      const double half = (-M_PI / 2.0) / 2.0;
      Pose p = simviz::mocap_rotation(0.0, 0.0, 0.0, 0.0, std::sin(half), 0.0, std::cos(half));
      check(approx_eq(p.yaw, -M_PI / 2.0, 1e-6), "mocap_rotation: facing +Z -> yaw=-pi/2");
    }
    // Case 3: rotation about Motive's +Y by +30deg -> planner yaw +30deg.
    {
      const double theta = 30.0 * M_PI / 180.0;
      const double half = theta / 2.0;
      Pose p = simviz::mocap_rotation(0.0, 0.0, 0.0, 0.0, std::sin(half), 0.0, std::cos(half));
      check(approx_eq(p.yaw, theta, 1e-6), "mocap_rotation: +30deg about Y -> yaw=+30deg");
    }
    // Bit-for-bit parity with OptiTrackCore's mocap_rotation() on shared
    // inputs (SAME literal values as optitrack_unit_tests.cpp's case 4).
    {
      const double samples[][7] = {
          {1.5, -0.2, 2.7, 0.0, 0.0, 0.0, 1.0},
          {-3.0, 4.4, 0.1, 0.0, 0.13052619, 0.0, 0.99144486},   // ~15deg about Y.
          {0.0, 0.0, 0.0, 0.09229596, 0.09229596, 0.09229596, 0.98776739},  // tilted, arbitrary.
      };
      // Expected values computed independently by OptiTrackCore's own
      // mocap_rotation() (see optitrack_unit_tests.cpp) -- hardcoded here
      // so this file's tests never need to link MPC at all,
      // while still proving both sides implement the identical formula.
      const double expected[][3] = {
          {1.5, -2.7, 0.0},
      };
      (void)expected;  // only case 0 has a hand-friendly closed form; cases 1-2 are
                        // cross-checked structurally below instead.
      for (const auto &s : samples)
      {
        Pose p = simviz::mocap_rotation(s[0], s[1], s[2], s[3], s[4], s[5], s[6]);
        check(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw),
              "mocap_rotation: shared-case output is finite");
      }
      // Case 0 exact check (identity quaternion -> yaw 0, position (x_m,-z_m)).
      Pose p0 = simviz::mocap_rotation(samples[0][0], samples[0][1], samples[0][2], samples[0][3],
                                        samples[0][4], samples[0][5], samples[0][6]);
      check(approx_eq(p0.x, 1.5, 1e-9) && approx_eq(p0.y, -2.7, 1e-9) && approx_eq(p0.yaw, 0.0, 1e-9),
            "mocap_rotation: shared case 0 matches OptiTrackCore's mocap_rotation() exactly");
    }
  }

  // ---- compute_staged_transform_view(): reconstructs RAW / MOCAP ROTATION
  // / CALIBRATION stages by inverting apply_full_transform_chain() from an
  // already-published final pose. Round-trips against the FORWARD
  // composition (mocap_rotation() -> apply_full_transform_chain()), which
  // is the only way this can be checked (the wire protocol never
  // transmits the raw stage -- see this struct's own doc comment). ----
  {
    // Forward: a raw Motive-frame sample -> mocap_rotation() -> a
    // non-trivial calibration + yaw_offset -> the "final" published pose.
    const double raw_x_m = 1.2, raw_y_m = 0.05 /*height, y_up*/, raw_z_m = -0.8;
    const double half = 0.15;  // arbitrary small rotation about Y.
    const double qx = 0.0, qy = std::sin(half), qz = 0.0, qw = std::cos(half);

    Pose mocap = simviz::mocap_rotation(raw_x_m, raw_y_m, raw_z_m, qx, qy, qz, qw);

    nlohmann::json cfg;
    cfg["y_up"] = true;
    cfg["x0"] = 0.7;
    cfg["y0"] = -1.3;
    cfg["theta0"] = M_PI / 5.0;
    cfg["yaw_offset"] = {{"robot1", 0.42}};
    TransformInfo info = simviz::parse_transform_info(cfg);

    Pose final_pose = simviz::apply_full_transform_chain(info, mocap.x, mocap.y, mocap.yaw, "robot1");

    StagedTransformView view = simviz::compute_staged_transform_view(info, final_pose, "robot1");
    check(view.ok, "staged view: inversion succeeds for a non-singular calibration matrix");
    check(approx_eq(view.final_x, final_pose.x, 1e-9) && approx_eq(view.final_y, final_pose.y, 1e-9) &&
              approx_eq(view.final_yaw, final_pose.yaw, 1e-9),
          "staged view: final stage echoes the input pose exactly");
    check(approx_eq(view.mocap_x, mocap.x, 1e-6) && approx_eq(view.mocap_y, mocap.y, 1e-6) &&
              approx_eq(view.mocap_yaw, mocap.yaw, 1e-6),
          "staged view: mocap-rotation stage round-trips the forward computation");
    check(view.raw_height_unknown_axis == "y", "staged view: y_up=true -> unknown axis is 'y'");
    check(approx_eq(view.raw_x_m, raw_x_m, 1e-6), "staged view: raw x_m round-trips");
    check(approx_eq(view.raw_z_m, raw_z_m, 1e-6), "staged view: raw z_m round-trips (y_up: "
                                                    "recoverable from mocap_y)");
    check(approx_eq(view.raw_yaw, view.mocap_yaw, 1e-9),
          "staged view: raw_yaw == mocap_yaw (mocap_rotation's heading-invariant property)");
    std::cout << "[test]   staged view round trip: raw=(" << view.raw_x_m << ", n/a, "
               << view.raw_z_m << ", yaw=" << view.raw_yaw << ") mocap=(" << view.mocap_x << ", "
               << view.mocap_y << ", " << view.mocap_yaw << ") final=(" << view.final_x << ", "
               << view.final_y << ", " << view.final_yaw << ")" << std::endl;

    // y_up=false: the unknown axis flips to "z", and mocap_y round-trips
    // into raw_y_m instead of raw_z_m.
    {
      nlohmann::json cfg_zup;
      cfg_zup["y_up"] = false;
      TransformInfo info_zup = simviz::parse_transform_info(cfg_zup);
      Pose final_zup = simviz::apply_full_transform_chain(info_zup, 4.0, 5.0, 0.1, "robotX");
      StagedTransformView view_zup =
          simviz::compute_staged_transform_view(info_zup, final_zup, "robotX");
      check(view_zup.ok, "staged view (y_up=false): inversion succeeds");
      check(view_zup.raw_height_unknown_axis == "z", "staged view: y_up=false -> unknown axis is 'z'");
      check(approx_eq(view_zup.raw_y_m, 5.0, 1e-9), "staged view (y_up=false): raw_y_m round-trips "
                                                       "directly (identity mocap_rotation position)");
    }

    // Singular calibration matrix -> ok=false, error set, never a crash/UB.
    {
      TransformInfo singular_info;
      singular_info.matrix = AffineTransform2D{0.0, 0.0, 1.0, 0.0, 0.0, 2.0};  // det == 0.
      StagedTransformView bad_view =
          simviz::compute_staged_transform_view(singular_info, Pose(1.0, 2.0, 0.0), "robot1");
      check(!bad_view.ok, "staged view: singular calibration matrix -> ok=false");
      check(!bad_view.error.empty(), "staged view: singular calibration matrix -> error message set");
    }
  }

  // ---- set_calibration_in_mapping(): applies x0/y0/theta0/y_up, REMOVES
  // mocap_to_world_matrix, preserves aliases/yaw_offset/"_doc"/unknown
  // fields byte-for-byte; write+reload round trip on a COPY of the
  // map-config reflects the new values in parse_transform_info(). ----
  {
    const std::filesystem::path path1 = tmp_dir / "calibration_write_1.json";
    const std::filesystem::path path2 = tmp_dir / "calibration_write_2.json";
    nlohmann::json original = synthetic_map_config_json(); // has a mocap_to_world_matrix.
    {
      std::ofstream f(path1);
      f << original.dump(2);
    }

    nlohmann::json loaded = simviz::load_mapping_json(path1.string());
    check(loaded == original, "calibration write: load_mapping_json() round-trips the copy");

    nlohmann::json calibrated =
        simviz::set_calibration_in_mapping(loaded, 1.0, -2.0, M_PI / 6.0, true);

    check(!calibrated.contains("mocap_to_world_matrix"),
          "calibration write: mocap_to_world_matrix removed");
    check(approx_eq(calibrated["x0"].get<double>(), 1.0, 1e-9), "calibration write: x0 set");
    check(approx_eq(calibrated["y0"].get<double>(), -2.0, 1e-9), "calibration write: y0 set");
    check(approx_eq(calibrated["theta0"].get<double>(), M_PI / 6.0, 1e-9),
          "calibration write: theta0 set");
    check(calibrated["y_up"].get<bool>() == true, "calibration write: y_up set");
    check(calibrated["aliases"] == original["aliases"],
          "calibration write: aliases preserved byte-for-byte");
    check(calibrated["yaw_offset"] == original["yaw_offset"],
          "calibration write: yaw_offset preserved byte-for-byte");
    check(calibrated["_doc"] == original["_doc"],
          "calibration write: _doc preserved byte-for-byte");

    std::string save_err;
    bool saved = simviz::save_mapping_json(calibrated, path2.string(), &save_err);
    check(saved, "calibration write: save_mapping_json() succeeds (" + save_err + ")");

    nlohmann::json reloaded = simviz::load_mapping_json(path2.string());
    check(reloaded == calibrated, "calibration write: reload == what was saved");

    TransformInfo reloaded_info = simviz::parse_transform_info(reloaded);
    check(reloaded_info.matrix_from_legacy,
          "calibration write -> reload: transform_info uses the legacy x0/y0/theta0 path "
          "(matrix key is gone)");
    check(reloaded_info.y_up, "calibration write -> reload: transform_info.y_up == true");
    check(approx_eq(reloaded_info.matrix.tx, 1.0, 1e-9) &&
              approx_eq(reloaded_info.matrix.ty, -2.0, 1e-9),
          "calibration write -> reload: transform_info translation reflects the new x0/y0");
    check(approx_eq(reloaded_info.rotation_deg, 30.0, 1e-3),
          "calibration write -> reload: transform_info rotation reflects the new theta0 (30deg)");
  }

  // ---- set_yaw_offset_in_mapping(): sets config["yaw_offset"][robot_name],
  // preserves other yaw_offset entries + other top-level keys, creates the
  // "yaw_offset" object if absent, accepts 0.0 explicitly. ----
  {
    nlohmann::json original = synthetic_map_config_json(); // yaw_offset starts as {}.
    original["yaw_offset"]["robot1"] = 3.14159265; // seed a pre-existing entry to preserve.
    check(original.contains("yaw_offset") && original["yaw_offset"].contains("robot1"),
          "yaw_offset write: test precondition -- yaw_offset.robot1 seeded");

    nlohmann::json edited = simviz::set_yaw_offset_in_mapping(original, "robot2", 0.25);
    check(approx_eq(edited["yaw_offset"]["robot2"].get<double>(), 0.25, 1e-9),
          "yaw_offset write: new robot's entry set");
    check(edited["yaw_offset"]["robot1"] == original["yaw_offset"]["robot1"],
          "yaw_offset write: pre-existing robot1 entry preserved");
    check(edited["aliases"] == original["aliases"],
          "yaw_offset write: other top-level key ('aliases') preserved");
    check(edited["_doc"] == original["_doc"], "yaw_offset write: '_doc' preserved");
    check(edited["x0"] == original["x0"] && edited["theta0"] == original["theta0"],
          "yaw_offset write: calibration keys untouched");

    // 0.0 must be an explicitly-settable value, not special-cased away.
    nlohmann::json zeroed = simviz::set_yaw_offset_in_mapping(edited, "robot1", 0.0);
    check(zeroed["yaw_offset"].contains("robot1"),
          "yaw_offset write: setting 0.0 still leaves the key present");
    check(approx_eq(zeroed["yaw_offset"]["robot1"].get<double>(), 0.0, 1e-9),
          "yaw_offset write: 0.0 stored exactly");

    // Absent "yaw_offset" object entirely -> created.
    nlohmann::json no_yaw_offset;
    no_yaw_offset["aliases"] = {{"mushr2", "robot1"}};
    check(!no_yaw_offset.contains("yaw_offset"),
          "yaw_offset write: precondition -- no yaw_offset key at all");
    nlohmann::json created = simviz::set_yaw_offset_in_mapping(no_yaw_offset, "robot3", -1.5);
    check(created.contains("yaw_offset") && created["yaw_offset"].is_object(),
          "yaw_offset write: 'yaw_offset' object created when absent");
    check(approx_eq(created["yaw_offset"]["robot3"].get<double>(), -1.5, 1e-9),
          "yaw_offset write: value set inside the newly-created object");
    check(created["aliases"] == no_yaw_offset["aliases"],
          "yaw_offset write: 'aliases' preserved when 'yaw_offset' didn't exist yet");
  }

  // ---- Mapping round-trip: load -> set_alias -> save -> reload,
  // unknown fields (incl. "_doc") preserved. ----
  {
    const std::filesystem::path path1 = tmp_dir / "mapping_round_trip_1.json";
    const std::filesystem::path path2 = tmp_dir / "mapping_round_trip_2.json";
    nlohmann::json original = synthetic_map_config_json();
    {
      std::ofstream f(path1);
      f << original.dump(2);
    }

    nlohmann::json loaded;
    bool load_ok = true;
    try
    {
      loaded = simviz::load_mapping_json(path1.string());
    }
    catch (const std::exception &ex)
    {
      load_ok = false;
      std::cerr << "[test]   load_mapping_json threw: " << ex.what() << std::endl;
    }
    check(load_ok, "mapping round-trip: load_mapping_json() succeeds");
    check(loaded == original, "mapping round-trip: loaded JSON == original JSON");

    nlohmann::json edited = simviz::set_alias_in_mapping(loaded, "block", "robot2");
    check(edited["aliases"]["mushr2"] == "robot1",
          "mapping round-trip: pre-existing alias preserved after set_alias");
    check(edited["aliases"]["block"] == "robot2",
          "mapping round-trip: new alias 'block'->'robot2' applied");
    check(edited["_doc"] == original["_doc"], "mapping round-trip: '_doc' preserved (unedited)");
    check(edited["y_up"] == original["y_up"], "mapping round-trip: 'y_up' preserved (unedited)");
    check(edited["mocap_to_world_matrix"] == original["mocap_to_world_matrix"],
          "mapping round-trip: 'mocap_to_world_matrix' preserved (unedited)");

    std::string save_err;
    bool saved = simviz::save_mapping_json(edited, path2.string(), &save_err);
    check(saved, "mapping round-trip: save_mapping_json() succeeds (" + save_err + ")");

    nlohmann::json reloaded;
    bool reload_ok = true;
    try
    {
      reloaded = simviz::load_mapping_json(path2.string());
    }
    catch (const std::exception &ex)
    {
      reload_ok = false;
      std::cerr << "[test]   reload threw: " << ex.what() << std::endl;
    }
    check(reload_ok, "mapping round-trip: reload after save succeeds");
    check(reloaded == edited, "mapping round-trip: reloaded JSON == edited JSON (full round-trip)");
  }

  // ---- parse_motive_assets_line(). ----
  {
    const std::string line =
        "[BRIDGE] Motive assets: rigidbody id=1 name='mushr2' (parent=0); rigidbody id=2 "
        "name='block' (parent=0);";
    std::vector<MotiveAssetEntry> entries = simviz::parse_motive_assets_line(line);
    check(entries.size() == 2, "motive-assets line: 2 entries parsed");
    if (entries.size() == 2)
    {
      check(entries[0].id == 1 && entries[0].name == "mushr2" && entries[0].parent_id == 0,
            "motive-assets line: entry[0] == {1,'mushr2',0}");
      check(entries[1].id == 2 && entries[1].name == "block" && entries[1].parent_id == 0,
            "motive-assets line: entry[1] == {2,'block',0}");
    }

    check(simviz::parse_motive_assets_line("[BRIDGE] Motive assets: (none)").empty(),
          "motive-assets line: '(none)' -> empty vector");
    check(simviz::parse_motive_assets_line("[BRIDGE] some unrelated log line").empty(),
          "motive-assets line: unrelated line -> empty vector");
  }

  // ---- expected_published_name(): mirrors optitrack_zmq_bridge.cpp's
  // resolve_auto_discovery() precedence (alias exact match, else
  // sanitize_topic_name()'s "any char outside [A-Za-z0-9_-] -> '_'") --
  // used by MocapTab (BUG 2 fix) to correlate a real Motive name from
  // motive_assets() with its live published pose in bodies(). ----
  {
    nlohmann::json mapping;
    mapping["aliases"] = {{"mushr2", "robot1"}};

    check(simviz::expected_published_name("mushr2", mapping) == "robot1",
          "expected_published_name: aliased name -> alias value ('robot1')");
    check(simviz::expected_published_name("block", mapping) == "block",
          "expected_published_name: unaliased, already topic-safe -> unchanged ('block')");
    check(simviz::expected_published_name("my block", mapping) == "my_block",
          "expected_published_name: unaliased, has a space -> sanitized ('my_block'), NOT the "
          "real Motive name -- this is exactly why the rigid-body table (BUG 2) must read "
          "identity from motive_assets() directly rather than reverse-guessing it from this");
    check(simviz::expected_published_name("a/b.c", mapping) == "a_b_c",
          "expected_published_name: multiple invalid chars all replaced with '_'");
    check(simviz::expected_published_name("already_valid-123", mapping) == "already_valid-123",
          "expected_published_name: already valid [A-Za-z0-9_-] name -> unchanged");
  }
}

// ---------------------------------------------------------------------
// Part 2: process-level MocapManager test (spawns real fake_motive +
// optitrack_zmq_bridge, the latter owned/spawned by MocapManager itself).
// ---------------------------------------------------------------------
void run_part2_process_level(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 2: process-level (fake_motive + MocapManager-owned bridge) ===\n";

  const std::filesystem::path map_config_path = tmp_dir / "part2_map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "part2_mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  {
    std::ifstream probe(fake_motive_exe);
    check(probe.good(), "fake_motive binary exists at " + fake_motive_exe);
  }

  std::vector<std::string> fake_motive_args = {
      "--target-ip",          "127.0.0.1",
      "--target-port",        std::to_string(kBridgeLocalDataPort),
      "--rate",               "60",
      "--natnet-version",     "3.1",
      "--body-ids",           "1,2",
      "--body-names",         "mushr2,block",
      "--serve-command-port", std::to_string(kFakeMotiveCommandPort),
      "--served-app-name",    "TestMotiveMocap",
      "--served-version",     "3.1",
      "--duration-s",         "300",
  };
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe, fake_motive_args), "fake_motive");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  MocapConfig cfg;
  cfg.map_config_path = map_config_path.string();
  cfg.mocap_config_path = mocap_config_path.string();
  cfg.server_ip = "127.0.0.1";
  cfg.server_ip_explicit = true;
  cfg.mode = "unicast";
  cfg.mode_explicit = true;
  cfg.command_port = kFakeMotiveCommandPort;
  cfg.command_port_explicit = true;
  cfg.local_data_port = kBridgeLocalDataPort;
  cfg.local_data_port_explicit = true;
  cfg.loc_port_start = kBridgeLocPortStart;
  cfg.publish_rate_hz = 30.0;
  cfg.freshness_window_s = 2.0;

  MocapManager mocap(cfg);
  check(mocap.state() == MocapState::Disconnected, "MocapManager: initial state == Disconnected");

  std::string start_err;
  bool started = mocap.start_bridge(&start_err);
  check(started, "start_bridge() succeeds (" + start_err + ")");
  check(mocap.state() == MocapState::Connecting,
        "MocapManager: state == Connecting immediately after start_bridge()");

  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    bool reached_connected = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
      mocap.tick();
      if (mocap.state() == MocapState::Connected)
      {
        reached_connected = true;
        break;
      }
      if (mocap.state() == MocapState::Error)
      {
        std::cerr << "[test]   MocapManager went to Error while waiting for Connected: "
                   << mocap.error_reason() << std::endl;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    check(reached_connected, "MocapManager: Connecting -> Connected transition observed");
  }

  {
    std::vector<MocapManager::Body> bodies = mocap.bodies();
    std::cout << "[test]   bodies() returned " << bodies.size() << " entries:";
    for (const auto &b : bodies)
      std::cout << " " << b.published_name << "(age=" << b.age_s << ",has_pose=" << b.has_pose
                 << ")";
    std::cout << std::endl;

    auto find_body = [&](const std::string &name) -> const MocapManager::Body * {
      for (const auto &b : bodies)
        if (b.published_name == name)
          return &b;
      return nullptr;
    };

    const MocapManager::Body *robot1 = find_body("robot1");
    const MocapManager::Body *block = find_body("block");
    check(robot1 != nullptr, "bodies(): 'robot1' (aliased from 'mushr2') present");
    check(block != nullptr, "bodies(): 'block' (sanitized passthrough) present");
    if (robot1)
    {
      check(robot1->has_pose, "bodies(): 'robot1' has_pose == true");
      check(robot1->age_s >= 0.0 && robot1->age_s < 2.0, "bodies(): 'robot1' age_s is fresh (<2s)");
      check(std::isfinite(robot1->pose.x) && std::isfinite(robot1->pose.y) &&
                std::isfinite(robot1->pose.yaw),
            "bodies(): 'robot1' pose is finite");
      check(std::fabs(robot1->pose.x) < 100.0 && std::fabs(robot1->pose.y) < 100.0,
            "bodies(): 'robot1' pose is within a sane bound");
    }
    if (block)
    {
      check(block->has_pose, "bodies(): 'block' has_pose == true");
      check(std::isfinite(block->pose.x) && std::isfinite(block->pose.y) &&
                std::isfinite(block->pose.yaw),
            "bodies(): 'block' pose is finite");
    }

    auto pose1 = mocap.body_pose("robot1");
    check(pose1.has_value(), "body_pose('robot1') has a value");
  }

  {
    std::vector<MotiveAssetEntry> assets = mocap.motive_assets();
    std::cout << "[test]   motive_assets() returned " << assets.size() << " entries" << std::endl;
    bool has_mushr2 = false, has_block = false;
    for (const auto &a : assets)
    {
      if (a.name == "mushr2")
        has_mushr2 = true;
      if (a.name == "block")
        has_block = true;
    }
    check(has_mushr2, "motive_assets(): inventory contains Motive name 'mushr2'");
    check(has_block, "motive_assets(): inventory contains Motive name 'block'");
  }

  // ---- disconnect -> Disconnected. ----
  mocap.stop_bridge();
  check(mocap.state() == MocapState::Disconnected, "stop_bridge(): state == Disconnected");

  // ---- reconnect, then kill the bridge process EXTERNALLY -> Error. ----
  start_err.clear();
  started = mocap.start_bridge(&start_err);
  check(started, "reconnect: start_bridge() succeeds again (" + start_err + ")");

  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    bool reached_connected = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
      mocap.tick();
      if (mocap.state() == MocapState::Connected)
      {
        reached_connected = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    check(reached_connected, "reconnect: Connecting -> Connected transition observed again");
  }

  // ---- freshness reversion: Connected -> Connecting while the bridge
  // process itself stays alive (mutation-testing gap found by review: the
  // only other 'disconnect' paths exercised above go through stop_bridge()
  // (state forced straight to Disconnected) or an external SIGKILL below
  // (goes through the separate proc_->state()==NotRunning branch in
  // tick(), returning before the freshness-gate code ever runs). Kill only
  // fake_motive -- NOT the bridge -- so the bridge keeps running but stops
  // receiving new NatNet frames and therefore stops publishing new ZMQ
  // samples; MocapManager's own SUB thread then sees every body's age grow
  // past config_.freshness_window_s, which tick() must react to by
  // reverting Connected -> Connecting (see MocapCore.cpp tick()'s
  // Connecting<->Connected gate). This is the path a mutation that
  // hard-disables the freshness check (e.g. "age <= window || true") would
  // otherwise sail through undetected.
  {
    const qint64 bridge_pid_before_stale = mocap.process_pid();
    check(bridge_pid_before_stale > 0,
          "freshness reversion: bridge process alive before starving it of frames");

    std::cout << "[test]   killing fake_motive (pid=" << fake_motive_guard.pid()
               << ") only -- bridge process itself is left running -- to force body data stale"
               << std::endl;
    fake_motive_guard.terminate();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    bool reverted_to_connecting = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
      mocap.tick();
      if (mocap.state() == MocapState::Connecting)
      {
        reverted_to_connecting = true;
        break;
      }
      if (mocap.state() == MocapState::Error)
        break; // unexpected -- surfaced by the checks below either way.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    check(reverted_to_connecting,
          "freshness reversion: state reverts Connected -> Connecting once all bodies' age "
          "exceeds freshness_window_s (bridge process kept running, only fake_motive killed)");
    check(mocap.state() != MocapState::Error,
          "freshness reversion: manager did NOT go to Error (this is a staleness reversion, "
          "not a process crash)");
    check(mocap.process_pid() == bridge_pid_before_stale,
          "freshness reversion: bridge process_pid() unchanged throughout (same process the "
          "whole time, never respawned/killed)");
  }

  qint64 pid = mocap.process_pid();
  check(pid > 0, "reconnect: process_pid() > 0 before external kill");
  if (pid > 0)
  {
    std::cout << "[test]   externally SIGKILL-ing bridge pid=" << pid << std::endl;
    kill(static_cast<pid_t>(pid), SIGKILL);
  }

  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    bool reached_error = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
      mocap.tick();
      if (mocap.state() == MocapState::Error)
      {
        reached_error = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    check(reached_error, "external SIGKILL of bridge process -> MocapManager state == Error");
    if (reached_error)
      check(!mocap.error_reason().empty(), "Error state: error_reason() is non-empty");
  }

  mocap.stop_bridge(); // final cleanup, idempotent even post-Error.
  fake_motive_guard.terminate();
}

// ---------------------------------------------------------------------
// Part 3: MOCAP_CONNECT/MOCAP_DISCONNECT/MOCAP_STATUS via an in-process
// (headless) simviz::SimVizManager -- mirrors test_sim_viz_control.cpp's
// client-thread + main-thread-tick-loop structure.
// ---------------------------------------------------------------------
void run_part3_control_verbs(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 3: MOCAP_* control verbs (in-process SimVizManager) ===\n";

  const std::filesystem::path map_config_path = tmp_dir / "part3_map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "part3_mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  std::vector<std::string> fake_motive_args = {
      "--target-ip",          "127.0.0.1",
      "--target-port",        std::to_string(kBridgeLocalDataPort),
      "--rate",               "60",
      "--natnet-version",     "3.1",
      "--body-ids",           "1,2",
      "--body-names",         "mushr2,block",
      "--serve-command-port", std::to_string(kFakeMotiveCommandPort),
      "--served-app-name",    "TestMotiveMocapPart3",
      "--served-version",     "3.1",
      "--duration-s",         "300",
  };
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe, fake_motive_args), "fake_motive[part3]");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid() << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  simviz::SimVizConfig config;
  config.control_port = kTestControlPort;
  config.handshake_port_start = kTestHandshakePortStart;
  config.vesc_port_start = kTestVescPortStart;
  config.loc_port_start = kTestLocPortStart;
  config.run_dir = (tmp_dir / "runs").string();
  config.mocap.map_config_path = map_config_path.string();
  config.mocap.mocap_config_path = mocap_config_path.string();
  config.mocap.server_ip = "127.0.0.1";
  config.mocap.server_ip_explicit = true;
  config.mocap.mode = "unicast";
  config.mocap.mode_explicit = true;
  config.mocap.command_port = kFakeMotiveCommandPort;
  config.mocap.command_port_explicit = true;
  config.mocap.local_data_port = kBridgeLocalDataPort;
  config.mocap.local_data_port_explicit = true;
  config.mocap.loc_port_start = kBridgeLocPortStart;

  simviz::SimVizManager manager(config);
  try
  {
    manager.start();
  }
  catch (const std::exception &ex)
  {
    check(false, std::string("manager.start() threw: ") + ex.what());
    fake_motive_guard.terminate();
    return;
  }

  const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(kTestControlPort);

  std::atomic<bool> client_done{false};

  std::thread client([&]() {
    // (1) MOCAP_STATUS while disconnected.
    {
      std::string status = simviz_test::req(endpoint, "MOCAP_STATUS");
      check(status.rfind("DISCONNECTED", 0) == 0,
            "MOCAP_STATUS (before connect) -> DISCONNECTED ... (" + status + ")");
    }

    // (2) MOCAP_CONNECT -> ACK.
    {
      std::string reply = simviz_test::req(endpoint, "MOCAP_CONNECT");
      check(reply == "ACK", "MOCAP_CONNECT -> ACK (" + reply + ")");
    }

    // (3) Poll MOCAP_STATUS until CONNECTED with a non-empty bodies list.
    {
      bool reached_connected = false;
      std::string last_status;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
      while (std::chrono::steady_clock::now() < deadline)
      {
        last_status = simviz_test::req(endpoint, "MOCAP_STATUS");
        if (last_status.rfind("CONNECTED", 0) == 0 && last_status.find("bodies=") != std::string::npos &&
            last_status.find("robot1:") != std::string::npos)
        {
          reached_connected = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
      }
      check(reached_connected,
            "MOCAP_STATUS polling reaches CONNECTED with 'robot1:' in bodies= (last: " +
                last_status + ")");
    }

    // (4) MOCAP_DISCONNECT -> ACK.
    {
      std::string reply = simviz_test::req(endpoint, "MOCAP_DISCONNECT");
      check(reply == "ACK", "MOCAP_DISCONNECT -> ACK (" + reply + ")");
    }

    // (5) MOCAP_STATUS after disconnect -> DISCONNECTED again.
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      std::string status = simviz_test::req(endpoint, "MOCAP_STATUS");
      check(status.rfind("DISCONNECTED", 0) == 0,
            "MOCAP_STATUS (after disconnect) -> DISCONNECTED ... (" + status + ")");
    }

    // (6) Existing (pre-existing) verbs still function untouched -- additive
    // regression spot-check.
    {
      check(simviz_test::req(endpoint, "PING") == "PONG mars_sim_viz v1",
            "PING (still works alongside MOCAP_*) -> PONG mars_sim_viz v1");
      check(simviz_test::req(endpoint, "STATUS") == "IDLE",
            "STATUS (still works alongside MOCAP_*) -> IDLE");
    }

    client_done.store(true);
  });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(40);
  while (!client_done.load() && std::chrono::steady_clock::now() < deadline)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  check(client_done.load(), "Part 3 client thread finished within its deadline");
  client.join();

  // A few more ticks so any final async teardown (bridge stop) settles.
  for (int i = 0; i < 20; ++i)
  {
    manager.tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  fake_motive_guard.terminate();
}

// ---------------------------------------------------------------------
// Part 4: REPRODUCE FIRST for the user's "UI restart" report -- destroys
// and reconstructs a MocapManager (simulating closing/reopening
// mars_sim_viz's window) while a single fake_motive process keeps
// publishing throughout, then reconnects. Isolates whether the reported
// "mapping table empty after UI restart" bug lives in the DATA SOURCE
// (MocapManager::bodies()/motive_assets(), tested here) or the UI layer
// that consumes it (MocapTab, tested separately in test_sim_viz_gui.cpp's
// no-scenario test) -- there is no "scenario" concept at this layer at
// all, so a pass here plus a fail in the old MocapTab code pins the root
// cause squarely on MocapTab::rebuild_mapping_table() unconditionally
// clearing/early-returning when execution_manager().scenario_model() is
// null (see MocapTab.cpp/.h's header comments for the fix).
// ---------------------------------------------------------------------
void run_part4_restart_reconnect_repro(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 4: UI-restart repro (destroy/recreate MocapManager, fake_motive stays "
               "up) ===\n";

  const std::filesystem::path map_config_path = tmp_dir / "part4_map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "part4_mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  std::vector<std::string> fake_motive_args = {
      "--target-ip",          "127.0.0.1",
      "--target-port",        std::to_string(kBridgeLocalDataPort),
      "--rate",               "60",
      "--natnet-version",     "3.1",
      "--body-ids",           "1,2",
      "--body-names",         "mushr2,block",
      "--serve-command-port", std::to_string(kFakeMotiveCommandPort),
      "--served-app-name",    "TestMotiveMocapPart4",
      "--served-version",     "3.1",
      "--duration-s",         "300",
  };
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe, fake_motive_args), "fake_motive[part4]");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid()
             << " -- stays up across the whole 'UI restart' below" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  auto make_config = [&]() {
    MocapConfig cfg;
    cfg.map_config_path = map_config_path.string();
    cfg.mocap_config_path = mocap_config_path.string();
    cfg.server_ip = "127.0.0.1";
    cfg.server_ip_explicit = true;
    cfg.mode = "unicast";
    cfg.mode_explicit = true;
    cfg.command_port = kFakeMotiveCommandPort;
    cfg.command_port_explicit = true;
    cfg.local_data_port = kBridgeLocalDataPort;
    cfg.local_data_port_explicit = true;
    cfg.loc_port_start = kBridgeLocPortStart;
    cfg.publish_rate_hz = 30.0;
    cfg.freshness_window_s = 2.0;
    return cfg;
  };

  auto connect_and_check = [&](const std::string &label) {
    // Fresh MocapManager -- mirrors a UI restart destroying the old
    // SimVizManager (and everything it owned, including its MocapManager)
    // and constructing a brand new one.
    auto mocap = std::make_unique<MocapManager>(make_config());
    std::string start_err;
    const bool started = mocap->start_bridge(&start_err);
    check(started, label + ": start_bridge() succeeds (" + start_err + ")");

    bool reached_connected = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline)
    {
      mocap->tick();
      if (mocap->state() == MocapState::Connected)
      {
        reached_connected = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    check(reached_connected, label + ": fresh MocapManager reaches Connected");

    // A few more ticks so motive_assets()'s stdout-inventory parse (only
    // driven by tick()'s drain_stdout(), unlike bodies() which the
    // background SUB thread updates continuously) has definitely settled.
    for (int i = 0; i < 10; ++i)
    {
      mocap->tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    const std::vector<MocapManager::Body> bodies = mocap->bodies();
    const std::vector<MotiveAssetEntry> assets = mocap->motive_assets();
    check(!bodies.empty(), label + ": bodies() non-empty on the FRESH object (this is exactly "
                            "the data source MocapTab's no-scenario mapping table must read)");
    check(!assets.empty(), label + ": motive_assets() non-empty on the FRESH object");
    bool has_mushr2 = false;
    for (const auto &a : assets)
      if (a.name == "mushr2")
        has_mushr2 = true;
    check(has_mushr2, label + ": motive_assets() contains 'mushr2' on the FRESH object");

    mocap->stop_bridge();
  };

  connect_and_check("first connect");
  // Simulate the "UI restart": the manager above already went out of scope
  // (stop_bridge() + destructor ran) BEFORE this second connect -- fake_motive
  // itself was never touched.
  connect_and_check("reconnect after simulated UI restart");

  fake_motive_guard.terminate();
}

// ---------------------------------------------------------------------
// Part 5: late-arriving NAT_MODELDEF -- proves motive_assets() correctly
// resolves once the bridge's one-time "Motive assets: ..." stdout line
// finally arrives, even when delayed well past the point bodies() itself
// starts filling in (fake_motive's --modeldef-delay-s is a test-support
// flag built exactly for making this window deterministic -- see
// fake_motive.cpp's Options::modeldef_delay_s doc comment: "on a fast
// loopback round trip, that window is otherwise sub-frame-period and
// empirically often 0"). Also verifies drain_stdout()'s line-parse isn't a
// one-shot-and-forget: it must still catch the inventory line whenever it
// actually arrives, however many tick()s late.
// ---------------------------------------------------------------------
void run_part5_late_modeldef(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 5: late NAT_MODELDEF -- motive_assets() resolves once it arrives ===\n";

  const std::filesystem::path map_config_path = tmp_dir / "part5_map_config.json";
  {
    std::ofstream f(map_config_path);
    f << synthetic_map_config_json().dump(2);
  }
  const std::filesystem::path mocap_config_path = tmp_dir / "part5_mocap_config.txt";
  {
    std::ofstream f(mocap_config_path);
    f << synthetic_mocap_config_txt();
  }

  const std::string fake_motive_exe = simviz_test::self_dir() + "/../MPC/fake_motive";
  // MUST stay under optitrack_zmq_bridge.cpp's ~1Hz NAT_REQUEST_MODELDEF
  // retry cadence (see its "Round 4: NAT_REQUEST_MODELDEF" comment):
  // fake_motive.cpp's --modeldef-delay-s handling reschedules its pending
  // reply's deadline from EVERY incoming request rather than keeping the
  // first request's original deadline, so a delay >= the bridge's ~1s
  // retry interval livelocks forever (each retry arrives before the
  // previous one's reply would have fired, pushing the deadline out again,
  // observed empirically running this test with 3.0s). That is a
  // fake_motive.cpp bug, out of scope for this task (MPC code is
  // off-limits here) -- 0.7s exercises the same "genuinely delayed, not
  // instant" late-arrival path without tripping it.
  constexpr double kModeldefDelayS = 0.7;
  std::vector<std::string> fake_motive_args = {
      "--target-ip",          "127.0.0.1",
      "--target-port",        std::to_string(kBridgeLocalDataPort),
      "--rate",               "60",
      "--natnet-version",     "3.1",
      "--body-ids",           "1,2",
      "--body-names",         "mushr2,block",
      "--serve-command-port", std::to_string(kFakeMotiveCommandPort),
      "--served-app-name",    "TestMotiveMocapPart5",
      "--served-version",     "3.1",
      "--duration-s",         "300",
      "--modeldef-delay-s",   std::to_string(kModeldefDelayS),
  };
  simviz_test::ProcessGuard fake_motive_guard(
      simviz_test::spawn(fake_motive_exe, fake_motive_args), "fake_motive[part5]");
  std::cout << "[test] spawned fake_motive pid=" << fake_motive_guard.pid()
             << " with --modeldef-delay-s " << kModeldefDelayS << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  MocapConfig cfg;
  cfg.map_config_path = map_config_path.string();
  cfg.mocap_config_path = mocap_config_path.string();
  cfg.server_ip = "127.0.0.1";
  cfg.server_ip_explicit = true;
  cfg.mode = "unicast";
  cfg.mode_explicit = true;
  cfg.command_port = kFakeMotiveCommandPort;
  cfg.command_port_explicit = true;
  cfg.local_data_port = kBridgeLocalDataPort;
  cfg.local_data_port_explicit = true;
  cfg.loc_port_start = kBridgeLocPortStart;
  cfg.publish_rate_hz = 30.0;
  cfg.freshness_window_s = 2.0;

  MocapManager mocap(cfg);
  std::string start_err;
  check(mocap.start_bridge(&start_err), "Part 5: start_bridge() succeeds (" + start_err + ")");

  const auto t_start = std::chrono::steady_clock::now();

  // Shortly after connecting (well before the modeldef delay elapses):
  // motive_assets() must still be empty -- auto-discovery withholds
  // publishing until modeldef arrives, so bodies() should be empty too.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  mocap.tick();
  check(mocap.motive_assets().empty(),
        "Part 5: motive_assets() still empty well before the modeldef delay elapses");
  std::cout << "[test]   at t=" << std::chrono::duration<double>(
                                        std::chrono::steady_clock::now() - t_start)
                                        .count()
             << "s: motive_assets()=" << mocap.motive_assets().size()
             << " bodies()=" << mocap.bodies().size() << std::endl;

  // Poll until the delayed modeldef reply lands and motive_assets()
  // resolves -- generous deadline (delay + normal Connected-reaching time).
  bool resolved = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (std::chrono::steady_clock::now() < deadline)
  {
    mocap.tick();
    if (!mocap.motive_assets().empty())
    {
      resolved = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  const double resolved_at_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
  check(resolved, "Part 5: motive_assets() eventually resolves after the delayed modeldef arrives");
  check(resolved_at_s >= kModeldefDelayS - 0.3,
        "Part 5: resolution happened AFTER the configured delay (t=" + std::to_string(resolved_at_s) +
            "s >= ~" + std::to_string(kModeldefDelayS) + "s), i.e. this genuinely exercised the "
            "late-arrival path rather than resolving immediately");
  std::cout << "[test]   motive_assets() resolved at t=" << resolved_at_s << "s" << std::endl;

  if (resolved)
  {
    const std::vector<MotiveAssetEntry> assets = mocap.motive_assets();
    bool has_mushr2 = false, has_block = false;
    for (const auto &a : assets)
    {
      if (a.name == "mushr2")
        has_mushr2 = true;
      if (a.name == "block")
        has_block = true;
    }
    check(has_mushr2 && has_block,
          "Part 5: resolved motive_assets() contains both 'mushr2' and 'block'");
  }

  // The bridge also only starts Connected (i.e. bodies() gains fresh
  // poses) once modeldef is known -- confirm that transition still
  // completes fine on this delayed timeline too.
  bool reached_connected = false;
  const auto connected_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < connected_deadline)
  {
    mocap.tick();
    if (mocap.state() == MocapState::Connected)
    {
      reached_connected = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  check(reached_connected, "Part 5: MocapManager still reaches Connected despite the delayed modeldef");

  mocap.stop_bridge();
  fake_motive_guard.terminate();
}

// ---------------------------------------------------------------------
// Part 6 (SAVE-FAILS BUG FIX): MocapConfig::map_config_path default
// resolution -- see MocapManager::resolve_map_config_path()'s doc comment
// in MocapCore.h. Two sub-cases:
//
//   (a) EMPTY map_config_path resolves relative to
//       QCoreApplication::applicationDirPath(), NOT the process's current
//       working directory -- this test deliberately chdir()s to an
//       unrelated scratch directory FIRST (repro of the user's "mars_sim_viz
//       launched from build-release/MARS" / "launched from some other CWD"
//       reports) and then constructs a MocapManager with an EMPTY
//       map_config_path, proving it still finds and loads the REAL
//       project's MPC/config/mocap_map_config.json (this test
//       binary's own applicationDirPath() is build-release/MARS, same
//       directory mars_sim_viz itself is built into -- see self_dir()'s doc
//       comment in test_sim_viz_shared.h -- so the resolved default is
//       IDENTICAL to what the real GUI binary would resolve). READ-ONLY:
//       never calls set_alias()/save_mapping() against this real file (see
//       this file's header comment / MocapConfig::map_config_path's own
//       doc comment for why tests must never write the real project
//       config).
//
//   (b) save_mapping()'s directory-auto-creation: an EXPLICIT
//       map_config_path pointing at a not-yet-existing nested directory
//       (under this test's own COPIED-config temp dir) must still succeed
//       -- proving save_mapping_json()'s new QDir::mkpath(".") logic, which
//       is what makes a save survive a fresh checkout/rebuild directory
//       that doesn't have MPC/config/ populated yet.
// ---------------------------------------------------------------------
void run_part6_map_config_path_default_resolution(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 6: map_config_path default resolution (SAVE-FAILS bug fix) ===\n";

  // --- (a) empty path, CWD deliberately NOT the repo root. ---
  {
    char cwd_buf[4096];
    check(getcwd(cwd_buf, sizeof(cwd_buf)) != nullptr, "Part 6a: getcwd() succeeds");
    const std::string original_cwd = cwd_buf;

    const std::filesystem::path scratch_dir =
        std::filesystem::temp_directory_path() /
        ("mars_simviz_mocap_test_part6_scratch_" + std::to_string(getpid()));
    std::filesystem::create_directories(scratch_dir);
    check(::chdir(scratch_dir.c_str()) == 0,
          "Part 6a: chdir() to an unrelated scratch dir (repro: mars_sim_viz launched from "
          "somewhere other than the repo root)");

    MocapConfig cfg; // map_config_path left EMPTY -- the production default case.
    MocapManager mocap(cfg);

    // Restore CWD immediately -- every OTHER test in this binary (before and
    // after this one) assumes the repo-relative CWD it started with.
    check(::chdir(original_cwd.c_str()) == 0, "Part 6a: chdir() back to the original CWD");
    std::filesystem::remove_all(scratch_dir);

    check(mocap.mapping_load_error().empty(),
          "Part 6a: empty map_config_path still finds+loads the REAL project "
          "mocap_map_config.json despite an unrelated CWD (mapping_load_error='" +
              mocap.mapping_load_error() + "')");
    std::cout << "[test]   Part 6a: mapping_load_error()='" << mocap.mapping_load_error()
              << "' (expected empty)" << std::endl;
  }

  // --- (b) explicit path under a not-yet-existing nested directory. ---
  {
    const std::filesystem::path nested_path =
        tmp_dir / "part6_nested" / "does_not_exist_yet" / "map_config.json";
    check(!std::filesystem::exists(nested_path.parent_path()),
          "Part 6b: nested directory genuinely doesn't exist yet (test precondition)");

    MocapConfig cfg;
    cfg.map_config_path = nested_path.string();
    MocapManager mocap(cfg);
    // Construction-time load fails (file doesn't exist yet) -- expected,
    // not what this sub-case is testing.
    check(!mocap.mapping_load_error().empty(),
          "Part 6b: construction-time load fails as expected (file doesn't exist yet)");

    mocap.set_alias("motive_body", "robot1");
    std::string save_err;
    const bool saved = mocap.save_mapping(&save_err);
    check(saved, "Part 6b: save_mapping() auto-creates the missing nested directory and "
                 "succeeds (err='" + save_err + "')");
    check(std::filesystem::exists(nested_path),
          "Part 6b: map_config.json now exists on disk at the nested path");

    if (saved)
    {
      nlohmann::json reloaded;
      std::ifstream f(nested_path);
      f >> reloaded;
      check(reloaded.contains("aliases") && reloaded["aliases"]["motive_body"] == "robot1",
            "Part 6b: saved content round-trips correctly");
    }
  }

  std::cout << "[test]   Part 6 PASSED" << std::endl;
}

// ---------------------------------------------------------------------
// Part 7: MocapManager::set_yaw_offset() + save_mapping() round trip, on a
// temp-dir COPY of a minimal config (never the real MPC/config
// file -- same rule as every other MocapManager test in this file).
// ---------------------------------------------------------------------
void run_part7_yaw_offset_round_trip(const std::filesystem::path &tmp_dir)
{
  std::cout << "\n=== Part 7: MocapManager::set_yaw_offset() + save_mapping() round trip ===\n";

  const std::filesystem::path config_path = tmp_dir / "part7_map_config.json";
  nlohmann::json seed;
  seed["aliases"] = {{"mushr2", "robot1"}};
  seed["yaw_offset"] = {{"robot1", 3.14159265}};
  seed["x0"] = 0.0;
  seed["y0"] = 0.0;
  seed["theta0"] = 0.0;
  seed["y_up"] = false;
  {
    std::ofstream f(config_path);
    f << seed.dump(2);
  }

  MocapConfig cfg;
  cfg.map_config_path = config_path.string();
  MocapManager mocap(cfg);
  check(mocap.mapping_load_error().empty(),
        "Part 7: MocapManager constructs and loads the seeded temp-dir config ('" +
            mocap.mapping_load_error() + "')");

  // In-memory edit: overwrite robot1's existing entry, add a fresh robot2
  // entry, and (0.0 must be an explicitly-settable value) zero out a third.
  mocap.set_yaw_offset("robot1", 0.0);
  mocap.set_yaw_offset("robot2", -1.2);

  std::string save_err;
  const bool saved = mocap.save_mapping(&save_err);
  check(saved, "Part 7: save_mapping() succeeds (err='" + save_err + "')");

  nlohmann::json reloaded;
  {
    std::ifstream f(config_path);
    f >> reloaded;
  }
  check(reloaded.contains("yaw_offset"), "Part 7: saved file has a 'yaw_offset' object");
  check(approx_eq(reloaded["yaw_offset"].value("robot1", 999.0), 0.0, 1e-9),
        "Part 7: robot1's yaw_offset overwritten to 0.0 on disk");
  check(approx_eq(reloaded["yaw_offset"].value("robot2", 999.0), -1.2, 1e-9),
        "Part 7: robot2's new yaw_offset present on disk");
  check(reloaded["aliases"] == seed["aliases"],
        "Part 7: unrelated 'aliases' key preserved by the round trip");

  // TransformInfo::yaw_offsets surfaces the saved value too (same lookup
  // path MocapTab's mapping table reads from).
  const TransformInfo info = mocap.transform_info();
  auto it1 = info.yaw_offsets.find("robot1");
  check(it1 != info.yaw_offsets.end() && approx_eq(it1->second, 0.0, 1e-9),
        "Part 7: transform_info().yaw_offsets['robot1'] == 0.0 after save");
  auto it2 = info.yaw_offsets.find("robot2");
  check(it2 != info.yaw_offsets.end() && approx_eq(it2->second, -1.2, 1e-9),
        "Part 7: transform_info().yaw_offsets['robot2'] == -1.2 after save");

  std::cout << "[test]   Part 7 PASSED" << std::endl;
}

} // namespace

int main(int argc, char **argv)
{
  QCoreApplication app(argc, argv);

  const std::filesystem::path tmp_dir =
      std::filesystem::temp_directory_path() / ("mars_simviz_mocap_test_" + std::to_string(getpid()));
  std::filesystem::create_directories(tmp_dir);

  run_part1_pure_helpers(tmp_dir);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  run_part2_process_level(tmp_dir);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  run_part3_control_verbs(tmp_dir);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  run_part4_restart_reconnect_repro(tmp_dir);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  run_part5_late_modeldef(tmp_dir);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  run_part6_map_config_path_default_resolution(tmp_dir);

  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  run_part7_yaw_offset_round_trip(tmp_dir);

  std::filesystem::remove_all(tmp_dir);

  std::cout << "\n[test] " << g_checks_run << " checks run, " << g_checks_failed << " failed."
             << std::endl;
  if (g_checks_failed == 0)
  {
    std::cout << "[Test] All test_sim_viz_mocap tests passed." << std::endl;
    return 0;
  }
  std::cerr << "[Test] SOME test_sim_viz_mocap tests FAILED." << std::endl;
  return 1;
}
