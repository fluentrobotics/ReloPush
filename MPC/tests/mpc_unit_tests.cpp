// Lightweight unit tests for the mpc:: core (RobotSpec/MpcCore/Kinematics/
// SimCore), mirroring the harness style of MARS/tests/phastar_unit_tests.cpp:
// plain bool test_xxx() functions registered in main(), no gtest dependency.
//
// Acceleration-formulation MPC: MpcCore's decision variables are [a_t,
// delta_t] (see MpcCore.h); velocity is a state, integrated stage-by-stage
// from SolveInputs::v0 (a real, feedback-derived estimate -- main.cpp
// finite-differences it from consecutive localization samples) via the SAME
// rollout equations as mpc::rollout_step (mpc/Kinematics.h), which is also
// what the standalone mpc_robot_sim simulator (tests S1-S6 below) uses as
// its plant model.

#include "mpc/LaunchGovernor.h"
#include "mpc/MpcCore.h"
#include "mpc/RobotSpec.h"
#include "mpc/SimCore.h"

#include <nlohmann/json.hpp>

#include <ReloPush/base64.h>
#include <ReloPush/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <string>
#include <tuple>
#include <vector>

namespace {

bool near_eq(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

ReloPush::trajectory make_traj(
    const std::vector<std::tuple<float, float, float, float, float, bool>>& pts) {
    ReloPush::trajectory traj;
    for (const auto& p : pts) {
        traj.append_waypoint(ReloPush::trajectory_elem(std::get<0>(p), std::get<1>(p),
                                                         std::get<2>(p), std::get<3>(p),
                                                         std::get<4>(p), std::get<5>(p)));
    }
    return traj;
}

// ---------------------------------------------------------------------
// Helpers: hand-built reference horizons (no ReloPush::trajectory
// serialization involved) for exercising MpcSolver::solve() directly.
// ---------------------------------------------------------------------

// A straight-line, constant-velocity reference: ref.x at stage t (0-based)
// is base_x + v*dt*(t+1) -- i.e. exactly where a vehicle starting at base_x
// and holding velocity v would be after (t+1) rollout steps, so an
// on-track solve (pred_state.x == base_x, pred_state.v == v) should need
// almost no correction.
std::vector<mpc::RefState> make_straight_refs(double base_x, double v, int horizon, double dt,
                                               bool is_pushing = false) {
    std::vector<mpc::RefState> refs;
    refs.reserve(horizon);
    for (int t = 0; t < horizon; ++t) {
        mpc::RefState r;
        r.x = base_x + v * dt * (t + 1);
        r.y = 0.0;
        r.yaw = 0.0;
        r.ref_vel = v;
        r.is_pushing = is_pushing;
        r.ok = true;
        refs.push_back(r);
    }
    return refs;
}

// SolveInputs populated from mpc::RobotSpec::defaults()/mpc::MpcParams::defaults(),
// with the start pose/last-command/horizon_refs/max_v filled from the caller.
mpc::SolveInputs make_default_solve_inputs(const mpc::State4& start_state,
                                            const std::vector<mpc::RefState>& horizon_refs,
                                            bool is_pushing = false, double max_v_push = 0.295,
                                            double max_v_nonpush = 0.38) {
    const mpc::MpcParams p = mpc::MpcParams::defaults();
    const mpc::RobotSpec r = mpc::RobotSpec::defaults();

    mpc::SolveInputs in;
    in.x0 = start_state.x;
    in.y0 = start_state.y;
    in.yaw0 = start_state.yaw;
    in.v0 = start_state.v;
    // Flat-seed with zero acceleration (hold current speed) by default --
    // tests that specifically want a non-zero warm-start seed set
    // last_cmd_a explicitly on the returned SolveInputs.
    in.last_cmd_a = 0.0;
    in.last_cmd_delta = 0.0;
    in.horizon_refs = horizon_refs;
    in.wheel_base = r.wheel_base;
    in.dt = p.dt;
    in.w_dist = p.w_dist;
    in.w_yaw = p.w_yaw;
    in.w_vel = p.w_vel;
    in.w_lat = p.w_lat;
    in.w_control = p.w_control;
    in.w_delta_rate = p.w_delta_rate;
    in.max_v_push = max_v_push;
    in.max_v_nonpush = max_v_nonpush;
    (void)is_pushing;  // caps are now resolved per-stage from each RefState::is_pushing.
    in.max_accel = r.max_accel;
    in.max_steer = r.max_steer;
    return in;
}

// ---------------------------------------------------------------------
// Test 1: RobotSpec/MpcParams defaults match the MARS production values.
// ---------------------------------------------------------------------
bool test_defaults_match_mars_values() {
    mpc::RobotSpec r = mpc::RobotSpec::defaults();
    mpc::MpcParams p = mpc::MpcParams::defaults();

    struct Check {
        const char* name;
        double got;
        double want;
    };
    const std::vector<Check> checks = {
        {"wheel_base", r.wheel_base, 0.29},
        {"front_length", r.front_length, 0.36},
        {"rear_length", r.rear_length, 0.12},
        {"width", r.width, 0.275},
        {"min_turning_radius_transit", r.min_turning_radius_transit, 1.02},
        {"min_turning_radius_transfer", r.min_turning_radius_transfer, 1.43},
        {"speed_transit", r.speed_transit, 0.2},
        {"speed_transfer", r.speed_transfer, 0.15},
        {"max_steer", r.max_steer, 0.33},
        {"max_accel", r.max_accel, 0.73},
        {"max_v_push", r.max_v_push, 0.295},
        {"max_v_nonpush", r.max_v_nonpush, 0.38},
        {"dt", p.dt, 0.05},
        {"control_delay_steps", static_cast<double>(p.control_delay_steps), 1.0},
        {"loop_hz", static_cast<double>(p.loop_hz), 20.0},
        {"w_dist", p.w_dist, 5.0},
        {"w_dist_dirchange", p.w_dist_dirchange, 50.0},
        {"w_yaw", p.w_yaw, 1.0},
        {"w_vel", p.w_vel, 0.2},
        {"w_lat", p.w_lat, 40.0},
        {"w_control", p.w_control, 0.001},
        {"w_delta_rate", p.w_delta_rate, 50.0},
        {"vel_scale", p.vel_scale, 1.01},
        {"vel_scale_back", p.vel_scale_back, 0.9},
        {"dir_change_scale", p.dir_change_scale, 0.77},
    };

    bool ok = true;
    for (const auto& c : checks) {
        if (!near_eq(c.got, c.want, 1e-9)) {
            std::cerr << "    " << c.name << " = " << c.got << ", expected " << c.want << "\n";
            ok = false;
        }
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test 2: JSON partial override; invalid path throws; unknown key warns.
// ---------------------------------------------------------------------
bool test_json_partial_override_and_errors() {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path();
    std::error_code ec;
    bool ok = true;

    // Partial override: 2 fields, others must keep defaults.
    const fs::path good_path = dir / "mpc_unit_test_robot_spec_partial.json";
    {
        std::ofstream out(good_path);
        out << R"({
          "robot": { "max_steer": 0.5, "wheel_base": 0.31 },
          "mpc": { "dt": 0.1 }
        })";
    }
    {
        mpc::RobotSpec robot = mpc::RobotSpec::defaults();
        mpc::MpcParams params = mpc::MpcParams::defaults();
        try {
            mpc::load_spec_from_json(good_path.string(), robot, params);
        } catch (const std::exception& ex) {
            std::cerr << "    unexpected throw on valid partial JSON: " << ex.what() << "\n";
            ok = false;
        }
        if (!near_eq(robot.max_steer, 0.5, 1e-9)) {
            std::cerr << "    max_steer override failed: got " << robot.max_steer << "\n";
            ok = false;
        }
        if (!near_eq(robot.wheel_base, 0.31, 1e-9)) {
            std::cerr << "    wheel_base override failed: got " << robot.wheel_base << "\n";
            ok = false;
        }
        // Untouched fields must keep compiled defaults.
        if (!near_eq(robot.max_accel, 0.73, 1e-9)) {
            std::cerr << "    max_accel should be untouched default, got " << robot.max_accel << "\n";
            ok = false;
        }
        if (!near_eq(robot.max_v_push, 0.295, 1e-9)) {
            std::cerr << "    max_v_push should be untouched default, got " << robot.max_v_push
                       << "\n";
            ok = false;
        }
        if (!near_eq(params.dt, 0.1, 1e-9)) {
            std::cerr << "    dt override failed: got " << params.dt << "\n";
            ok = false;
        }
        if (!near_eq(params.w_dist, 5.0, 1e-9)) {
            std::cerr << "    w_dist should be untouched default, got " << params.w_dist << "\n";
            ok = false;
        }
    }
    fs::remove(good_path, ec);

    // Nonexistent path -> throw.
    {
        mpc::RobotSpec robot = mpc::RobotSpec::defaults();
        mpc::MpcParams params = mpc::MpcParams::defaults();
        bool threw = false;
        try {
            mpc::load_spec_from_json(
                (dir / "mpc_unit_test_this_path_should_not_exist_12345.json").string(), robot,
                params);
        } catch (const std::exception&) {
            threw = true;
        }
        if (!threw) {
            std::cerr << "    expected throw for nonexistent path\n";
            ok = false;
        }
    }

    // Invalid JSON content -> throw.
    const fs::path invalid_path = dir / "mpc_unit_test_robot_spec_invalid.json";
    {
        std::ofstream out(invalid_path);
        out << "{ this is not valid json ";
    }
    {
        mpc::RobotSpec robot = mpc::RobotSpec::defaults();
        mpc::MpcParams params = mpc::MpcParams::defaults();
        bool threw = false;
        try {
            mpc::load_spec_from_json(invalid_path.string(), robot, params);
        } catch (const std::exception&) {
            threw = true;
        }
        if (!threw) {
            std::cerr << "    expected throw for invalid JSON content\n";
            ok = false;
        }
    }
    fs::remove(invalid_path, ec);

    // Unknown key -> warns on stderr but still loads successfully.
    const fs::path unknown_key_path = dir / "mpc_unit_test_robot_spec_unknown_key.json";
    {
        std::ofstream out(unknown_key_path);
        out << R"({
          "robot": { "max_steer": 0.31, "not_a_real_field": 42 },
          "mpc": {}
        })";
    }
    {
        mpc::RobotSpec robot = mpc::RobotSpec::defaults();
        mpc::MpcParams params = mpc::MpcParams::defaults();
        try {
            mpc::load_spec_from_json(unknown_key_path.string(), robot, params);
        } catch (const std::exception& ex) {
            std::cerr << "    unexpected throw on unknown-key JSON: " << ex.what() << "\n";
            ok = false;
        }
        if (!near_eq(robot.max_steer, 0.31, 1e-9)) {
            std::cerr << "    max_steer override (unknown-key file) failed\n";
            ok = false;
        }
    }
    fs::remove(unknown_key_path, ec);

    return ok;
}

// ---------------------------------------------------------------------
// Test 3: validate().
// ---------------------------------------------------------------------
bool test_validate() {
    bool ok = true;

    mpc::RobotSpec good = mpc::RobotSpec::defaults();
    const std::string good_err = good.validate();
    if (!good_err.empty()) {
        std::cerr << "    default spec unexpectedly failed validate(): " << good_err << "\n";
        ok = false;
    }

    mpc::RobotSpec bad = mpc::RobotSpec::defaults();
    bad.min_turning_radius_transit = 0.5;  // atan(0.29/0.5) ~= 0.5258 rad > max_steer 0.33
    const std::string bad_err = bad.validate();
    if (bad_err.empty()) {
        std::cerr << "    expected validate() to fail for min_turning_radius_transit=0.5\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 4: get_ref_state_at_time.
// ---------------------------------------------------------------------
bool test_get_ref_state_at_time() {
    bool ok = true;

    // Midpoint interpolation.
    {
        ReloPush::trajectory traj = make_traj({
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false},
            {2.0f, 0.0f, 0.0f, 1.0f, 2.0f, false},
        });
        mpc::RefState rs = mpc::get_ref_state_at_time(1.0, 0.0, traj);
        if (!rs.ok || !near_eq(rs.x, 1.0) || !near_eq(rs.ref_vel, 0.5)) {
            std::cerr << "    midpoint interpolation failed: ok=" << rs.ok << " x=" << rs.x
                       << " ref_vel=" << rs.ref_vel << "\n";
            ok = false;
        }
    }

    // Clamp before-start / after-end. Past the end, ref_vel is forced to
    // 0.0 (the robot is PARKED there) even though the last waypoint's own
    // nominal ref_vel label is nonzero (1.0 here) -- see MpcCore.h's doc
    // comment on get_ref_state_at_time. The before-start clamp has no such
    // special case and returns the front waypoint's own ref_vel as-is
    // (0.0 here, but because that's what the front waypoint itself holds,
    // not because of any zeroing rule).
    {
        ReloPush::trajectory traj = make_traj({
            {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false},
            {2.0f, 0.0f, 0.0f, 1.0f, 2.0f, false},
        });
        mpc::RefState before = mpc::get_ref_state_at_time(-5.0, 0.0, traj);
        mpc::RefState after = mpc::get_ref_state_at_time(50.0, 0.0, traj);
        if (!before.ok || !near_eq(before.x, 0.0) || !near_eq(before.ref_vel, 0.0)) {
            std::cerr << "    before-start clamp failed: ok=" << before.ok << " x=" << before.x
                       << "\n";
            ok = false;
        }
        if (!after.ok || !near_eq(after.x, 2.0) || !near_eq(after.ref_vel, 0.0)) {
            std::cerr << "    after-end clamp failed: ok=" << after.ok << " x=" << after.x
                       << " ref_vel=" << after.ref_vel << "\n";
            ok = false;
        }
    }

    // Yaw interpolation across the +-pi wrap: 3.0 -> -3.0 must go the short way
    // (through +-pi, not back through 0).
    {
        ReloPush::trajectory traj = make_traj({
            {0.0f, 0.0f, 3.0f, 0.0f, 0.0f, false},
            {0.0f, 0.0f, -3.0f, 0.0f, 2.0f, false},
        });
        mpc::RefState rs = mpc::get_ref_state_at_time(1.0, 0.0, traj);
        if (!rs.ok || !near_eq(rs.yaw, M_PI, 1e-6)) {
            std::cerr << "    yaw wrap interpolation failed: yaw=" << rs.yaw << " expected ~pi\n";
            ok = false;
        }
    }

    // Hold segment: two waypoints, same pose, ref_vel 0, 2s apart -> held pose,
    // v=0 in between.
    {
        ReloPush::trajectory traj = make_traj({
            {1.0f, 2.0f, 0.5f, 0.0f, 0.0f, false},
            {1.0f, 2.0f, 0.5f, 0.0f, 2.0f, false},
        });
        mpc::RefState rs = mpc::get_ref_state_at_time(1.0, 0.0, traj);
        if (!rs.ok || !near_eq(rs.x, 1.0) || !near_eq(rs.y, 2.0) || !near_eq(rs.yaw, 0.5) ||
            !near_eq(rs.ref_vel, 0.0)) {
            std::cerr << "    hold segment failed: x=" << rs.x << " y=" << rs.y
                       << " yaw=" << rs.yaw << " v=" << rs.ref_vel << "\n";
            ok = false;
        }
    }

    // Empty trajectory returns safely.
    {
        ReloPush::trajectory empty_traj;
        mpc::RefState rs = mpc::get_ref_state_at_time(1.0, 0.0, empty_traj);
        if (rs.ok) {
            std::cerr << "    expected ok=false for empty trajectory\n";
            ok = false;
        }
        if (!near_eq(rs.x, 0.0) || !near_eq(rs.y, 0.0) || !near_eq(rs.yaw, 0.0) ||
            !near_eq(rs.ref_vel, 0.0) || rs.is_pushing) {
            std::cerr << "    expected zeroed fields for empty trajectory\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 4b: terminal ref_vel zeroing. Regression test for the "terminal
// ref_vel zeroing" fix: get_ref_state_at_time's past-the-end clamp branch
// must force ref_vel to 0.0 (the plan wants the robot PARKED once there is
// no more trajectory left), even though the last waypoint's own ref_vel is
// a NONZERO per-leg nominal speed label -- distinct from (and covering more
// than) Test 4's generic clamp/interpolation coverage, whose after-end case
// now also expects ref_vel==0.0, updated alongside this fix. Before this
// fix, a downstream consumer (e.g. the LAUNCH governor's
// ref_indicates_motion check) could see a stale nonzero velocity target
// fighting a frozen position target during the post-completion
// finish-grace window -- see MpcCore.h's doc comment on
// get_ref_state_at_time and MpcCore.cpp's implementation.
// ---------------------------------------------------------------------
bool test_ref_vel_zero_past_trajectory_end() {
    bool ok = true;

    // Trajectory whose FINAL leg has a substantial nonzero nominal ref_vel
    // (-0.2, mirroring the reversal case that originally exposed this bug)
    // so a nonzero after.ref_vel could not be mistaken for coincidence.
    ReloPush::trajectory traj = make_traj({
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false},
        {1.0f, 2.0f, 0.7f, -0.2f, 3.0f, true},
    });

    // Past the end: ref_vel forced to 0.0; pose/yaw/is_pushing still
    // clamped to the last waypoint exactly as before this fix.
    {
        mpc::RefState rs = mpc::get_ref_state_at_time(50.0, 0.0, traj);
        if (!rs.ok || !near_eq(rs.x, 1.0) || !near_eq(rs.y, 2.0) || !near_eq(rs.yaw, 0.7) ||
            !rs.is_pushing || !near_eq(rs.ref_vel, 0.0)) {
            std::cerr << "    past-end failed: ok=" << rs.ok << " x=" << rs.x << " y=" << rs.y
                       << " yaw=" << rs.yaw << " is_pushing=" << rs.is_pushing
                       << " ref_vel=" << rs.ref_vel << " (expected pose=(1,2,0.7) is_pushing=1 "
                          "ref_vel=0)\n";
            ok = false;
        }
    }

    // Exactly at the end (rel_time == points.back().time): the clamp
    // condition is inclusive, so this must behave identically to the
    // past-end case above, not fall through to interpolation.
    {
        mpc::RefState rs = mpc::get_ref_state_at_time(3.0, 0.0, traj);
        if (!rs.ok || !near_eq(rs.x, 1.0) || !near_eq(rs.y, 2.0) || !near_eq(rs.ref_vel, 0.0)) {
            std::cerr << "    exact-at-end boundary failed: ok=" << rs.ok << " x=" << rs.x
                       << " y=" << rs.y << " ref_vel=" << rs.ref_vel << " (expected x=1 y=2 "
                          "ref_vel=0)\n";
            ok = false;
        }
    }

    // Just BEFORE the end (still inside the last segment, ordinary
    // interpolation) must be UNCHANGED by this fix: ref_vel reflects the
    // normal interpolated blend toward the nonzero final-leg nominal
    // speed, not zeroed.
    {
        mpc::RefState rs = mpc::get_ref_state_at_time(2.9, 0.0, traj);
        if (!rs.ok || near_eq(rs.ref_vel, 0.0)) {
            std::cerr << "    just-before-end incorrectly zeroed: ref_vel=" << rs.ref_vel << "\n";
            ok = false;
        }
    }

    // Before-start clamp is untouched by this fix: still returns the FIRST
    // waypoint's own ref_vel as-is (0.0 here because that's what the first
    // waypoint itself holds, not because of any past-end zeroing rule).
    {
        mpc::RefState rs = mpc::get_ref_state_at_time(-5.0, 0.0, traj);
        if (!rs.ok || !near_eq(rs.x, 0.0) || !near_eq(rs.ref_vel, 0.0)) {
            std::cerr << "    before-start clamp failed: ok=" << rs.ok << " x=" << rs.x
                       << " ref_vel=" << rs.ref_vel << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 5: rollout (acceleration-formulation model). REPLACES the STAGE 1
// "rollout_step: straight-line exactness + curved radius" test, which
// exercised the old velocity-control rollout_step(state, v, delta, L, dt);
// that signature no longer exists now that v is integrated from a rather
// than commanded directly. Same two scenarios, adapted to State4, plus an
// exact velocity-integration check.
// ---------------------------------------------------------------------
bool test_rollout_accel_model() {
    bool ok = true;

    // Straight motion from rest, constant acceleration: delta=0 -> yaw
    // unchanged; v_t = a*t*dt exactly; x accumulates the exact discrete sum
    // (semi-implicit: each step's x uses that step's NEW v).
    {
        mpc::State4 s;
        const double a = 0.5, delta = 0.0, L = 0.29, dt = 0.05;
        double expected_x = 0.0;
        for (int i = 1; i <= 10; ++i) {
            s = mpc::rollout_step(s, a, delta, L, dt);
            const double expected_v = a * i * dt;
            expected_x += expected_v * dt;
            if (!near_eq(s.v, expected_v, 1e-9) || !near_eq(s.x, expected_x, 1e-9) ||
                !near_eq(s.y, 0.0, 1e-9) || !near_eq(s.yaw, 0.0, 1e-9)) {
                std::cerr << "    straight accel rollout mismatch at step " << i << ": v=" << s.v
                           << " expected " << expected_v << ", x=" << s.x << " expected "
                           << expected_x << "\n";
                ok = false;
            }
        }
    }

    // Curved motion: zero acceleration (constant v), fixed steering ->
    // after a quarter turn the position should sit at (R, R) with
    // R = L / tan(delta), within 2%; velocity must stay exactly constant
    // (a=0 every step).
    {
        const double v0 = 0.2, delta = 0.3, L = 0.29, dt = 0.001;
        mpc::State4 s;
        s.v = v0;
        const double omega = (v0 / L) * std::tan(delta);
        const int steps = static_cast<int>(std::lround((M_PI / 2.0) / (omega * dt)));

        for (int i = 0; i < steps; ++i) {
            s = mpc::rollout_step(s, /*a=*/0.0, delta, L, dt);
        }

        const double R = L / std::tan(delta);
        const double rel_err_x = std::fabs(s.x - R) / R;
        const double rel_err_y = std::fabs(s.y - R) / R;
        const double yaw_err = std::fabs(s.yaw - M_PI / 2.0);
        if (rel_err_x > 0.02 || rel_err_y > 0.02 || yaw_err > 0.02 || !near_eq(s.v, v0, 1e-9)) {
            std::cerr << "    curved accel rollout mismatch: x=" << s.x << " y=" << s.y
                       << " yaw=" << s.yaw << " v=" << s.v << " (expected x~=" << R
                       << " y~=" << R << " yaw~=" << (M_PI / 2.0) << " v=" << v0 << ")\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// predict_delay_compensated: the sole consumer of
// MpcParams::control_delay_steps (main.cpp calls this instead of a
// hardcoded single rollout_step). Verifies the knob actually has an
// effect: 0 steps is a no-op, N steps chains N rollout_step() calls, and
// different N values produce genuinely different predictions.
// ---------------------------------------------------------------------
bool test_predict_delay_compensated() {
    bool ok = true;
    const double a = 0.4, delta = 0.15, L = 0.29, dt = 0.05;
    mpc::State4 start{1.0, 2.0, 0.3, 0.1};

    // 0 steps -> unchanged.
    {
        mpc::State4 got = mpc::predict_delay_compensated(start, a, delta, L, dt, 0);
        if (!near_eq(got.x, start.x) || !near_eq(got.y, start.y) || !near_eq(got.yaw, start.yaw) ||
            !near_eq(got.v, start.v)) {
            std::cerr << "    control_delay_steps=0 should be a no-op\n";
            ok = false;
        }
    }

    // 1 step -> exactly one rollout_step call (this is the default
    // control_delay_steps=1, i.e. today's/previous behavior).
    {
        mpc::State4 expected = mpc::rollout_step(start, a, delta, L, dt);
        mpc::State4 got = mpc::predict_delay_compensated(start, a, delta, L, dt, 1);
        if (!near_eq(got.x, expected.x) || !near_eq(got.y, expected.y) ||
            !near_eq(got.yaw, expected.yaw) || !near_eq(got.v, expected.v)) {
            std::cerr << "    control_delay_steps=1 should match a single rollout_step\n";
            ok = false;
        }
    }

    // 3 steps -> exactly three chained rollout_step calls, and this must
    // differ from both the 0-step and 1-step results (proves the knob
    // actually changes behavior, not just that it compiles/is read).
    {
        mpc::State4 expected = start;
        for (int i = 0; i < 3; ++i) {
            expected = mpc::rollout_step(expected, a, delta, L, dt);
        }
        mpc::State4 got = mpc::predict_delay_compensated(start, a, delta, L, dt, 3);
        if (!near_eq(got.x, expected.x) || !near_eq(got.y, expected.y) ||
            !near_eq(got.yaw, expected.yaw) || !near_eq(got.v, expected.v)) {
            std::cerr << "    control_delay_steps=3 should match three chained rollout_steps\n";
            ok = false;
        }
        mpc::State4 one_step = mpc::rollout_step(start, a, delta, L, dt);
        if (near_eq(got.v, start.v) || near_eq(got.v, one_step.v)) {
            std::cerr << "    control_delay_steps=3 result should differ from 0-step/1-step\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 6: solve on-track. Straight const-v reference, start exactly on it
// -> |lateral| tiny, |v_0 - ref_v| small.
// ---------------------------------------------------------------------
bool test_solve_on_track() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double L = mpc::RobotSpec::defaults().wheel_base;
    const double v = 0.2;

    const auto refs = make_straight_refs(0.0, v, mpc::kHorizon, dt);
    mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, v}, refs);

    mpc::MpcSolver solver;
    mpc::MpcOutput out = solver.solve(in);

    if (!out.solved) {
        std::cerr << "    on-track solve did not report solved=true\n";
        ok = false;
    }
    if (std::fabs(out.v_cmd - v) > 0.05) {
        std::cerr << "    on-track |v_cmd - ref_v| too large: " << out.v_cmd << "\n";
        ok = false;
    }

    // Reconstruct stage-1 lateral error exactly as the cost functor defines
    // it (see MPCCostFunctor::operator(), residual 4): the rolled-forward
    // state using out.v_cmd (== v0 + accel*dt, already clamped -- see
    // MpcOutput's doc comment) as the stage's velocity.
    mpc::State4 next;
    next.x = in.x0 + out.v_cmd * std::cos(in.yaw0) * dt;
    next.y = in.y0 + out.v_cmd * std::sin(in.yaw0) * dt;
    next.yaw = in.yaw0 + (out.v_cmd / L) * std::tan(out.steer) * dt;
    double dx = refs[0].x - next.x;
    double dy = refs[0].y - next.y;
    double lateral = -std::sin(next.yaw) * dx + std::cos(next.yaw) * dy;
    if (std::fabs(lateral) > 0.01) {
        std::cerr << "    on-track |lateral| too large: " << lateral << "\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 7: solve catch-up. Start behind with v=0, ref v=0.2 -> v_cmd > 0,
// and accel (== (v_cmd - v0)/dt by construction -- see MpcOutput's doc
// comment) stays within [0, max_accel + eps] (the box bound on the
// acceleration decision variable never lets it exceed max_accel).
// ---------------------------------------------------------------------
bool test_solve_catch_up() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_accel = mpc::RobotSpec::defaults().max_accel;
    const double ref_v = 0.2;

    // Reference starts 1m ahead of the vehicle and is already moving.
    const auto refs = make_straight_refs(1.0, ref_v, mpc::kHorizon, dt);
    mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.0}, refs);

    mpc::MpcSolver solver;
    mpc::MpcOutput out = solver.solve(in);

    if (!(out.v_cmd > 0.0)) {
        std::cerr << "    catch-up expected v_cmd > 0, got " << out.v_cmd << "\n";
        ok = false;
    }
    if (!(out.accel > 0.0)) {
        std::cerr << "    catch-up expected accel > 0, got " << out.accel << "\n";
        ok = false;
    }
    if (out.accel > max_accel + 1e-9) {
        std::cerr << "    catch-up accel exceeds max_accel: " << out.accel << " > " << max_accel
                   << "\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 8: bounds. Large velocity/position error -> the rate-clamped v_cmd
// implies accel == max_accel (within 1e-6), never above; steering never
// exceeds max_steer on a curved ref; v_cmd itself never exceeds max_v.
// ---------------------------------------------------------------------
bool test_solve_bounds() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_accel = mpc::RobotSpec::defaults().max_accel;
    const double max_steer = mpc::RobotSpec::defaults().max_steer;

    // Large error case: vehicle is 100m behind a reference moving at 0.2
    // m/s -- the position/velocity pull is enormous, so the rate-clamped
    // output should saturate at max_accel.
    {
        const auto refs = make_straight_refs(100.0, 0.2, mpc::kHorizon, dt);
        mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.0}, refs);

        mpc::MpcSolver solver;
        mpc::MpcOutput out = solver.solve(in);

        if (!near_eq(out.accel, max_accel, 1e-6)) {
            std::cerr << "    expected accel == max_accel (" << max_accel << "), got "
                       << out.accel << "\n";
            ok = false;
        }
        if (std::fabs(out.v_cmd) > in.max_v_nonpush + 1e-6) {
            std::cerr << "    v_cmd exceeds max_v: " << out.v_cmd << " > " << in.max_v_nonpush
                       << "\n";
            ok = false;
        }
    }

    // Curved/off-to-the-side reference: strong pull toward a large
    // steering command; box bounds must never be exceeded regardless.
    {
        std::vector<mpc::RefState> refs;
        refs.reserve(mpc::kHorizon);
        for (int t = 0; t < mpc::kHorizon; ++t) {
            mpc::RefState r;
            r.x = 0.2 * (t + 1);
            r.y = 5.0;  // large, constant lateral offset
            r.yaw = 1.2;
            r.ref_vel = 0.2;
            r.is_pushing = false;
            r.ok = true;
            refs.push_back(r);
        }
        mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.2}, refs);

        mpc::MpcSolver solver;
        mpc::MpcOutput out = solver.solve(in);

        if (std::fabs(out.steer) > max_steer + 1e-9) {
            std::cerr << "    steer exceeds max_steer on curved ref: " << out.steer << " vs "
                       << max_steer << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9: v-cap. ref_vel above max_v_push with is_pushing=true -> the
// solver's own [-max_v, max_v] box bound holds |v_cmd| within max_v_push
// (plus the small rate-clamp slack from starting slightly above it).
// ---------------------------------------------------------------------
bool test_solve_vcap_push() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_v_push = mpc::RobotSpec::defaults().max_v_push;
    const double v0 = 0.32;  // just above max_v_push (0.295)

    const auto refs = make_straight_refs(0.0, 0.6, mpc::kHorizon, dt, /*is_pushing=*/true);
    mpc::SolveInputs in =
        make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, v0}, refs, /*is_pushing=*/true);

    mpc::MpcSolver solver;
    mpc::MpcOutput out = solver.solve(in);

    if (std::fabs(out.v_cmd) > max_v_push + 1e-6) {
        std::cerr << "    v_cmd exceeds push cap: " << out.v_cmd << " > " << max_v_push << "\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9b: v-cap transition. main.cpp resolves max_v freshly every loop
// iteration from a single-time-point ref.is_pushing check (solve_in.max_v =
// ref.is_pushing ? max_v_push : max_v_nonpush; see main.cpp), i.e. there is
// no smoothing/blending/statefulness across the push<->non-push boundary --
// the very next solve() call after is_pushing flips uses the new cap
// immediately. Verifies exactly that on the SAME MpcSolver instance: a
// push-phase solve caps v_cmd to max_v_push, and a back-to-back non-push
// solve (same physical start state) is free to command a v_cmd above
// max_v_push (up to max_v_nonpush) -- i.e. the tighter push-phase cap does
// NOT leak into the following non-push call.
// ---------------------------------------------------------------------
bool test_solve_vcap_transition() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_v_push = mpc::RobotSpec::defaults().max_v_push;
    const double max_v_nonpush = mpc::RobotSpec::defaults().max_v_nonpush;
    // Sanity: the two caps must actually differ, or this test proves nothing.
    if (!(max_v_nonpush > max_v_push + 1e-9)) {
        std::cerr << "    expected max_v_nonpush > max_v_push (defaults changed?)\n";
        return false;
    }
    // Start already just above the push cap (mirrors test_solve_vcap_push):
    // small enough that the +-max_accel*dt rate clamp (about 0.0365 m/s per
    // 0.05s tick at max_accel=0.73) is never the binding constraint here --
    // the box bound [-max_v, max_v] is what's actually under test, not the
    // rate limiter. Reference keeps pulling for more speed than either cap
    // allows, so each solve saturates at its own respective box bound.
    const double ref_v = 0.6;
    const mpc::State4 start{0.0, 0.0, 0.0, max_v_push + 0.01};

    mpc::MpcSolver solver;

    // Push phase: capped at max_v_push.
    const auto push_refs = make_straight_refs(0.0, ref_v, mpc::kHorizon, dt, /*is_pushing=*/true);
    mpc::SolveInputs push_in =
        make_default_solve_inputs(start, push_refs, /*is_pushing=*/true, max_v_push, max_v_nonpush);
    mpc::MpcOutput push_out = solver.solve(push_in);
    if (push_out.v_cmd > max_v_push + 1e-6) {
        std::cerr << "    push-phase v_cmd exceeds max_v_push: " << push_out.v_cmd << "\n";
        ok = false;
    }

    // Immediately after (same solver instance, same physical start state,
    // no intervening state carried forward): non-push phase can legitimately
    // exceed max_v_push, proving the tighter cap did not leak across calls.
    const auto nonpush_refs =
        make_straight_refs(0.0, ref_v, mpc::kHorizon, dt, /*is_pushing=*/false);
    mpc::SolveInputs nonpush_in = make_default_solve_inputs(start, nonpush_refs,
                                                              /*is_pushing=*/false, max_v_push,
                                                              max_v_nonpush);
    mpc::MpcOutput nonpush_out = solver.solve(nonpush_in);
    if (nonpush_out.v_cmd > max_v_nonpush + 1e-6) {
        std::cerr << "    non-push-phase v_cmd exceeds max_v_nonpush: " << nonpush_out.v_cmd
                   << "\n";
        ok = false;
    }
    if (!(nonpush_out.v_cmd > max_v_push + 1e-6)) {
        std::cerr << "    expected non-push v_cmd to exceed the push cap (" << max_v_push
                   << "), got " << nonpush_out.v_cmd << " -- push cap may be leaking across calls\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9c: stage-accel-bound. The solve() loop applies
// problem.SetParameterLower/UpperBound(u.data(), 2*t, +-max_accel) for EVERY
// stage t in [0, kHorizon), not just stage 0 -- i.e. the box bound on the
// acceleration decision variable is enforced across the whole horizon.
// Drives a reference that pulls hard for high velocity at every horizon
// stage and inspects the raw horizon_a the solver actually optimized,
// checking ALL kHorizon entries respect [-max_accel, max_accel] (and
// horizon_delta respects [-max_steer, max_steer]), not merely the exposed
// stage-0 accel/steer. (Per-stage velocity capping is a SOFT clamp inside
// MPCCostFunctor -- see MpcCore.h's clamp_sym -- applied to the rolled
// state, not a box bound on a raw decision variable, so it is not asserted
// on horizon_a directly here; test 9b covers the exposed stage-0 v_cmd cap.)
// ---------------------------------------------------------------------
bool test_solve_stage_vmax() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_steer = mpc::RobotSpec::defaults().max_steer;
    const double max_accel = mpc::RobotSpec::defaults().max_accel;
    const double max_v_nonpush = mpc::RobotSpec::defaults().max_v_nonpush;
    const double v0 = 0.2;
    (void)dt;

    // A reference far out of reach at every stage (huge ref_vel and a large,
    // constant lateral offset with a steep heading) so the optimizer is
    // pulled toward the bounds at every t, not just t=0.
    std::vector<mpc::RefState> refs;
    refs.reserve(mpc::kHorizon);
    for (int t = 0; t < mpc::kHorizon; ++t) {
        mpc::RefState r;
        r.x = 5.0 * (t + 1);
        r.y = 5.0;
        r.yaw = 1.2;
        r.ref_vel = 5.0;  // far above max_v_nonpush
        r.is_pushing = false;
        r.ok = true;
        refs.push_back(r);
    }
    mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, v0}, refs,
                                                      /*is_pushing=*/false, mpc::RobotSpec::defaults().max_v_push,
                                                      max_v_nonpush);

    mpc::MpcSolver solver;
    mpc::MpcOutput out = solver.solve(in);

    if (out.horizon_a.size() != static_cast<size_t>(mpc::kHorizon) ||
        out.horizon_delta.size() != static_cast<size_t>(mpc::kHorizon)) {
        std::cerr << "    expected horizon_a/horizon_delta of size kHorizon, got "
                   << out.horizon_a.size() << "/" << out.horizon_delta.size() << "\n";
        return false;
    }

    for (int t = 0; t < mpc::kHorizon; ++t) {
        if (std::fabs(out.horizon_a[t]) > max_accel + 1e-6) {
            std::cerr << "    stage " << t << " raw accel exceeds max_accel: " << out.horizon_a[t]
                       << " > " << max_accel << "\n";
            ok = false;
        }
        if (std::fabs(out.horizon_delta[t]) > max_steer + 1e-9) {
            std::cerr << "    stage " << t << " raw delta exceeds max_steer: "
                       << out.horizon_delta[t] << " > " << max_steer << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9d: solve() with SolveInputs::use_warm_start left at its default
// (false) is fully stateless across calls -- it reseeds u[] purely from
// inputs.last_cmd_a/last_cmd_delta every time (see MpcSolver::solve()'s
// first loop), never from a previous call's converged optimum. This is a
// property of the flag-OFF path specifically, not a blanket "MpcCore never
// warm-starts" design claim -- see FEATURE 2A test P2 for the opt-in
// (use_warm_start=true) shift-seeding behavior that main.cpp's live control
// loop actually uses. Verifies the flag-off property behaviorally on ONE
// solver instance: solving A, then a very different B, then A again (all
// with use_warm_start=false, the default) must reproduce A's original
// result exactly -- if state were leaking across calls, the second A-solve
// would land somewhere different (seeded near B's optimum instead of purely
// from A's last_cmd_a/last_cmd_delta).
// ---------------------------------------------------------------------
bool test_no_warm_start_across_calls() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;

    const auto refs_a = make_straight_refs(0.0, 0.2, mpc::kHorizon, dt);
    mpc::SolveInputs in_a = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.05}, refs_a);

    // Deliberately very different: opposite direction, different start pose.
    const auto refs_b = make_straight_refs(-2.0, -0.3, mpc::kHorizon, dt);
    mpc::SolveInputs in_b = make_default_solve_inputs(mpc::State4{-1.0, 0.5, M_PI, -0.1}, refs_b);

    mpc::MpcSolver solver;
    mpc::MpcOutput out_a1 = solver.solve(in_a);
    mpc::MpcOutput out_b = solver.solve(in_b);
    mpc::MpcOutput out_a2 = solver.solve(in_a);

    // Sanity: A and B must actually diverge, or the "no leakage" check below
    // is vacuous.
    if (near_eq(out_a1.v_cmd, out_b.v_cmd, 1e-6) && near_eq(out_a1.steer, out_b.steer, 1e-6)) {
        std::cerr << "    scenario A and B solves unexpectedly coincide; test is not exercising "
                      "distinct optima\n";
        ok = false;
    }

    if (!near_eq(out_a1.v_cmd, out_a2.v_cmd, 1e-9) || !near_eq(out_a1.steer, out_a2.steer, 1e-9)) {
        std::cerr << "    repeating scenario A after solving B gave a different result: first=("
                   << out_a1.v_cmd << "," << out_a1.steer << ") second=(" << out_a2.v_cmd << ","
                   << out_a2.steer << ") -- solver state may be leaking across calls (warm start)\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9e: stop-accel. Mirror of the catch-up test (Test 7) but for
// decelerating to a stop: vehicle at v0>0 (SolveInputs::v0, the real
// velocity estimate), reference is a stationary hold (ref_vel=0) at
// (approximately) the vehicle's own position -- i.e. "you have arrived,
// stop here". Verifies a single solve commands v_cmd < v0 with a negative,
// magnitude-bounded accel, and that iterating solve() (feeding v_cmd back
// as the next v0, as main.cpp's loop does every 20Hz tick via its velocity
// estimate) decreases |v| monotonically with no sign flip until the vehicle
// is at rest -- the same closed-loop stopping behavior main.cpp relies on
// before it takes over with the final hardcoded publish_ackermann(...,0,0,0)
// at trajectory completion.
// ---------------------------------------------------------------------
bool test_solve_stop_accel() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_accel = mpc::RobotSpec::defaults().max_accel;

    // Stationary hold reference exactly where the vehicle already is (no
    // positional pull at all -- isolates the pure velocity-error-driven
    // deceleration this test is about).
    const auto refs = make_straight_refs(/*base_x=*/0.0, /*v=*/0.0, mpc::kHorizon, dt);
    mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.25}, refs);

    mpc::MpcSolver solver;
    mpc::MpcOutput first = solver.solve(in);

    if (!(first.v_cmd < in.v0 - 1e-9)) {
        std::cerr << "    stop-accel expected v_cmd < v0 (" << in.v0 << "), got " << first.v_cmd
                   << "\n";
        ok = false;
    }
    if (!(first.accel < 0.0)) {
        std::cerr << "    stop-accel expected accel < 0, got " << first.accel << "\n";
        ok = false;
    }
    if (first.accel < -max_accel - 1e-9) {
        std::cerr << "    stop-accel accel exceeds -max_accel: " << first.accel << " < "
                   << -max_accel << "\n";
        ok = false;
    }

    // Iterate the closed loop: feed v_cmd back as next last_cmd_v, same
    // stationary reference each tick, and confirm monotonic |v| decrease to
    // (near) zero with no sign flip -- mirrors main.cpp's 20Hz loop.
    double v = first.v_cmd;
    bool reached_rest = false;
    for (int i = 0; i < 200; ++i) {
        mpc::SolveInputs step_in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, v}, refs);
        mpc::MpcOutput step_out = solver.solve(step_in);
        if (step_out.v_cmd > v + 1e-9) {
            std::cerr << "    |v| increased during stop approach at iter " << i << ": " << v
                       << " -> " << step_out.v_cmd << "\n";
            ok = false;
        }
        if (step_out.v_cmd < -1e-9) {
            std::cerr << "    sign flip during stop approach at iter " << i << ": v=" << v
                       << " -> " << step_out.v_cmd << "\n";
            ok = false;
        }
        v = step_out.v_cmd;
        if (std::fabs(v) < 1e-4) {
            reached_rest = true;
            break;
        }
    }
    if (!reached_rest) {
        std::cerr << "    stop approach never converged to rest, ended at v=" << v << "\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9f: acceleration-formulation payload identity. Regression test for
// the "acceleration-formulation Ceres MPC" claim: MpcOutput::accel * dt must
// equal v_cmd - v0 EXACTLY (by construction -- see MpcOutput's doc comment
// and MpcCore.cpp's solve(), which derives accel FROM the (possibly
// velocity-capped) v_cmd rather than reporting the solver's raw decision
// variable unconditionally), across a variety of start velocities/
// references -- not merely approximately backed out from two independently
// chosen speeds the way a velocity-formulation controller's post-hoc
// (opt_v - last_cmd_v)/dt would be.
// ---------------------------------------------------------------------
bool test_accel_formulation_payload_identity() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;

    struct Case {
        double v0;
        double ref_v;
        const char* label;
    };
    const std::vector<Case> cases = {
        {0.0, 0.2, "accelerate from rest"},
        {0.2, 0.2, "hold speed"},
        {0.25, 0.0, "decelerate to stop"},
        {-0.15, -0.15, "hold reverse speed"},
        {0.1, -0.1, "reversal (forward v0, reverse ref)"},
    };

    for (const auto& c : cases) {
        const auto refs = make_straight_refs(0.0, c.ref_v, mpc::kHorizon, dt);
        mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, c.v0}, refs);

        mpc::MpcSolver solver;
        mpc::MpcOutput out = solver.solve(in);

        const double expected_v_cmd = in.v0 + out.accel * dt;
        if (!near_eq(out.v_cmd, expected_v_cmd, 1e-9)) {
            std::cerr << "    [" << c.label << "] payload identity violated: v_cmd=" << out.v_cmd
                       << " but v0+accel*dt=" << expected_v_cmd << " (v0=" << in.v0
                       << ", accel=" << out.accel << ")\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 9g: reversal tracking with a SIGNED negative ref_vel. Mirrors what
// MARS/include/RobotTrajectoryBuilder.h now emits through a genuine
// reversing maneuver (dir_sign < 0): a horizon whose reference recedes
// BEHIND the vehicle with a negative ref_vel at every stage must make the
// solver command a NEGATIVE v_cmd. Before both this fix and the
// RobotTrajectoryBuilder dir_sign fix, ref_vel could never go negative for a
// real MARS trajectory, so the velocity-tracking residual (see
// MPCCostFunctor::operator(), residual 3) only ever pulled toward a
// positive speed even during an actual reversal.
// ---------------------------------------------------------------------
bool test_solve_tracks_signed_reversal() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;

    const auto refs = make_straight_refs(/*base_x=*/0.0, /*v=*/-0.2, mpc::kHorizon, dt);
    mpc::SolveInputs in = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.0}, refs);

    mpc::MpcSolver solver;
    mpc::MpcOutput out = solver.solve(in);

    if (!(out.v_cmd < -0.01)) {
        std::cerr << "    expected a negative v_cmd tracking the reversing reference, got "
                   << out.v_cmd << "\n";
        ok = false;
    }
    if (!(out.accel < 0.0)) {
        std::cerr << "    expected negative accel to initiate the reversal, got " << out.accel
                   << "\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 10: dir-change resolution. Replicates the pre-round main.cpp's
// inline check exactly, including its asymmetry: w_dist is bumped for
// EITHER sign flip, but change_dir only fires for last_cmd_v>0 && ref_vel<0.
// ---------------------------------------------------------------------
bool test_dir_change_resolution() {
    bool ok = true;
    const mpc::MpcParams p = mpc::MpcParams::defaults();

    // last_cmd_v>0, ref_vel<0: both the weight bump AND change_dir fire.
    {
        mpc::DirChangeResolution r = mpc::resolve_dir_change(0.2, -0.2, p.w_dist, p.w_dist_dirchange);
        if (!near_eq(r.w_dist, p.w_dist_dirchange, 1e-12) || !r.change_dir) {
            std::cerr << "    forward->reverse flip should bump w_dist AND set change_dir\n";
            ok = false;
        }
    }

    // last_cmd_v<0, ref_vel>0: weight bump fires, but change_dir does NOT
    // (asymmetric, matching the original literal condition).
    {
        mpc::DirChangeResolution r = mpc::resolve_dir_change(-0.2, 0.2, p.w_dist, p.w_dist_dirchange);
        if (!near_eq(r.w_dist, p.w_dist_dirchange, 1e-12) || r.change_dir) {
            std::cerr << "    reverse->forward flip should bump w_dist but NOT set change_dir\n";
            ok = false;
        }
    }

    // No flip: default weight, no change_dir.
    {
        mpc::DirChangeResolution r = mpc::resolve_dir_change(0.2, 0.2, p.w_dist, p.w_dist_dirchange);
        if (!near_eq(r.w_dist, p.w_dist, 1e-12) || r.change_dir) {
            std::cerr << "    same-sign case should keep w_dist_default and change_dir=false\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// normalize_angle test.
// ---------------------------------------------------------------------
bool test_normalize_angle() {
    bool ok = true;
    struct Case {
        double in;
        double want;
    };
    // Deliberately avoids the +-pi branch-cut boundary itself (ill-defined
    // to sub-ULP precision under floating point); covers small angles and
    // multi-revolution wraps in both directions instead.
    const std::vector<Case> cases = {
        {0.0, 0.0},
        {0.1, 0.1},
        {-0.2, -0.2},
        {2.0 * M_PI + 0.3, 0.3},
        {-2.0 * M_PI - 0.3, -0.3},
        {4.0, 4.0 - 2.0 * M_PI},
        {-6.0, -6.0 + 2.0 * M_PI},
    };
    for (const auto& c : cases) {
        const double got = mpc::normalize_angle<double>(c.in);
        if (!near_eq(got, c.want, 1e-6)) {
            std::cerr << "    normalize_angle(" << c.in << ") = " << got << ", expected "
                       << c.want << "\n";
            ok = false;
        }
    }
    return ok;
}

// ---------------------------------------------------------------------
// Test 11: build_payload identities. Matches the pre-round controller on the
// wire-scaling policy: vel_scale/vel_scale_back is ALWAYS applied to the
// published "speed" -- there is no hardware-calibration on/off switch.
// "accel" is now passed straight through UNSCALED from the caller (the
// solver already derived it as (v_cmd - v0)/dt -- see MpcOutput's doc
// comment), so build_payload's own contract is just "accel out == accel in"
// (no dt/last-command bookkeeping happens here anymore).
// ---------------------------------------------------------------------
bool test_build_payload_identities() {
    bool ok = true;
    const mpc::MpcParams p = mpc::MpcParams::defaults();

    // Positive v, no dir change: uses vel_scale.
    {
        const double v_cmd = 0.2, accel = 2.0;
        mpc::PayloadValues pv = mpc::build_payload(v_cmd, 0.1, accel, p.vel_scale,
                                                     p.vel_scale_back, p.dir_change_scale, false);
        const double expected_speed = v_cmd * p.vel_scale;
        if (!near_eq(pv.speed, expected_speed, 1e-9)) {
            std::cerr << "    positive-v speed scaling mismatch: got " << pv.speed << " expected "
                       << expected_speed << "\n";
            ok = false;
        }
        // accel passes through UNSCALED and UNCHANGED -- build_payload does
        // not recompute or rescale it.
        if (!near_eq(pv.accel, accel, 1e-9)) {
            std::cerr << "    accel should pass through unchanged: got " << pv.accel
                       << " expected " << accel << "\n";
            ok = false;
        }
        if (!near_eq(pv.steering, 0.1, 1e-12)) {
            std::cerr << "    steering should pass through unchanged\n";
            ok = false;
        }
    }

    // Negative v: uses vel_scale_back.
    {
        const double v_cmd = -0.2;
        mpc::PayloadValues pv = mpc::build_payload(v_cmd, 0.0, /*accel=*/-4.0, p.vel_scale,
                                                     p.vel_scale_back, p.dir_change_scale, false);
        const double expected_speed = v_cmd * p.vel_scale_back;
        if (!near_eq(pv.speed, expected_speed, 1e-9)) {
            std::cerr << "    negative-v scale_back mismatch: got " << pv.speed << " expected "
                       << expected_speed << "\n";
            ok = false;
        }
    }

    // Dir change flagged: dir_change_scale applied ONLY when flagged, on top
    // of vel_scale.
    {
        const double v_cmd = 0.2;
        mpc::PayloadValues pv_flagged = mpc::build_payload(
            v_cmd, 0.0, /*accel=*/4.0, p.vel_scale, p.vel_scale_back, p.dir_change_scale, true);
        mpc::PayloadValues pv_unflagged = mpc::build_payload(
            v_cmd, 0.0, /*accel=*/4.0, p.vel_scale, p.vel_scale_back, p.dir_change_scale, false);
        const double expected_flagged = v_cmd * p.vel_scale * p.dir_change_scale;

        if (!near_eq(pv_flagged.speed, expected_flagged, 1e-9)) {
            std::cerr << "    dir-change scale not applied when flagged: got " << pv_flagged.speed
                       << " expected " << expected_flagged << "\n";
            ok = false;
        }
        if (near_eq(pv_flagged.speed, pv_unflagged.speed, 1e-9)) {
            std::cerr << "    dir-change scale should differ from the unflagged result\n";
            ok = false;
        }
    }

    // Zero v_cmd/accel: speed and accel both zero regardless of scaling.
    {
        mpc::PayloadValues pv = mpc::build_payload(0.0, 0.0, 0.0, p.vel_scale, p.vel_scale_back,
                                                     p.dir_change_scale, false);
        if (!near_eq(pv.speed, 0.0) || !near_eq(pv.accel, 0.0)) {
            std::cerr << "    zero v_cmd should yield zero speed and accel\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// Test 12: empty-trajectory completion path (helper level).
// ---------------------------------------------------------------------
bool test_empty_trajectory_completion_helpers() {
    bool ok = true;

    ReloPush::trajectory empty_traj;
    mpc::RefState rs = mpc::get_ref_state_at_time(0.0, 0.0, empty_traj);
    // ok=false is exactly the "nothing to track" signal main.cpp's
    // traj_point_count==0 guard acts on to mark the trajectory complete
    // before ever entering the control loop.
    const bool complete = !rs.ok;
    if (!complete) {
        std::cerr << "    expected empty trajectory to be flagged complete (ok=false)\n";
        ok = false;
    }

    // Deriving a stop payload in this situation must not crash and must be
    // stationary.
    const mpc::MpcParams p = mpc::MpcParams::defaults();
    mpc::PayloadValues stop_payload = mpc::build_payload(0.0, 0.0, /*accel=*/0.0, p.vel_scale,
                                                            p.vel_scale_back, p.dir_change_scale,
                                                            false);
    if (!near_eq(stop_payload.speed, 0.0) || !near_eq(stop_payload.steering, 0.0) ||
        !near_eq(stop_payload.accel, 0.0)) {
        std::cerr << "    expected a zeroed stop payload for an empty trajectory\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// mpc_robot_sim (SimCore) test helpers.
//
// encode_field_like_controller() replicates MPC/src/main.cpp's
// (static, non-exported) encodeAscii() byte-for-byte: base64(ASCII-decimal
// via ostringstream << setprecision(16)). Used to build ackermann payloads
// exactly as mpc_controller would publish them, for the S4 round-trip test.
// ---------------------------------------------------------------------
std::string encode_field_like_controller(double value) {
    std::ostringstream oss;
    oss << std::setprecision(16) << value;
    return base64_encode(oss.str());
}

std::string build_ackermann_payload_like_controller(double speed, double steering, double accel) {
    nlohmann::json j;
    j["speed"] = encode_field_like_controller(speed);
    j["steering"] = encode_field_like_controller(steering);
    j["accel"] = encode_field_like_controller(accel);
    return j.dump();
}

// ---------------------------------------------------------------------
// S1. sim step parity: the sim's actual per-tick pipeline (resolve_command
// in the non-watchdog-engaged case, feeding straight into mpc::rollout_step
// -- see robot_sim.cpp) must produce results BITWISE-EQUAL to calling
// mpc::rollout_step directly with the same (state, a, delta, dt), across
// straight/curved/reverse/zero-v cases. This is the model-parity guarantee
// the design spec requires between the controller's own rollout and the
// simulator's plant model.
// ---------------------------------------------------------------------
bool test_sim_step_parity_with_rollout_step() {
    bool ok = true;
    const double L = 0.29;
    const double dt = 0.05;

    struct Case {
        mpc::State4 s;
        double a;
        double delta;
        const char* label;
    };
    const std::vector<Case> cases = {
        {mpc::State4{0.0, 0.0, 0.0, 0.0}, 0.5, 0.0, "straight from rest"},
        {mpc::State4{1.0, 2.0, 0.3, 0.2}, 0.3, 0.15, "curved forward"},
        {mpc::State4{0.5, -0.5, -1.2, -0.2}, -0.4, -0.1, "reverse"},
        {mpc::State4{0.0, 0.0, 1.0, 0.0}, 0.0, 0.2, "zero-v with steering"},
    };

    for (const auto& c : cases) {
        mpc::ResolvedCommand resolved =
            mpc::resolve_command(/*watchdog_engaged=*/false, c.s.v, c.a, c.delta,
                                  /*max_accel=*/10.0);
        mpc::State4 sim_next = mpc::rollout_step(c.s, resolved.accel, resolved.steering, L, dt);
        mpc::State4 direct_next = mpc::rollout_step(c.s, c.a, c.delta, L, dt);

        if (sim_next.x != direct_next.x || sim_next.y != direct_next.y ||
            sim_next.yaw != direct_next.yaw || sim_next.v != direct_next.v) {
            std::cerr << "    [" << c.label << "] sim step parity mismatch: sim=(" << sim_next.x
                       << "," << sim_next.y << "," << sim_next.yaw << "," << sim_next.v
                       << ") direct=(" << direct_next.x << "," << direct_next.y << ","
                       << direct_next.yaw << "," << direct_next.v << ")\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// S2. closed-form: through the sim's resolve_command -> rollout_step
// pipeline, from rest with a=0.5 straight for N steps, v/x match the exact
// discrete recurrence; constant v with fixed delta traces a radius within
// 2% of wheel_base/tan(delta).
// ---------------------------------------------------------------------
bool test_sim_closed_form_kinematics() {
    bool ok = true;
    const double L = 0.29;

    // Straight, constant acceleration from rest.
    {
        const double dt = 0.05;
        mpc::State4 s;
        double expected_x = 0.0;
        for (int i = 1; i <= 10; ++i) {
            mpc::ResolvedCommand r = mpc::resolve_command(false, s.v, 0.5, 0.0, 10.0);
            s = mpc::rollout_step(s, r.accel, r.steering, L, dt);
            const double expected_v = 0.5 * i * dt;
            expected_x += expected_v * dt;
            if (!near_eq(s.v, expected_v, 1e-9) || !near_eq(s.x, expected_x, 1e-9) ||
                !near_eq(s.y, 0.0, 1e-9) || !near_eq(s.yaw, 0.0, 1e-9)) {
                std::cerr << "    straight closed-form mismatch at step " << i << ": v=" << s.v
                           << " expected " << expected_v << ", x=" << s.x << " expected "
                           << expected_x << "\n";
                ok = false;
            }
        }
    }

    // Curved, constant velocity (a=0), fixed steering -> quarter-turn radius.
    {
        const double v0 = 0.2, delta = 0.3, dt = 0.001;
        mpc::State4 s;
        s.v = v0;
        const double omega = (v0 / L) * std::tan(delta);
        const int steps = static_cast<int>(std::lround((M_PI / 2.0) / (omega * dt)));

        for (int i = 0; i < steps; ++i) {
            mpc::ResolvedCommand r = mpc::resolve_command(false, s.v, 0.0, delta, 10.0);
            s = mpc::rollout_step(s, r.accel, r.steering, L, dt);
        }

        const double R = L / std::tan(delta);
        const double rel_err_x = std::fabs(s.x - R) / R;
        const double rel_err_y = std::fabs(s.y - R) / R;
        if (rel_err_x > 0.02 || rel_err_y > 0.02 || !near_eq(s.v, v0, 1e-9)) {
            std::cerr << "    curved closed-form mismatch: x=" << s.x << " y=" << s.y
                       << " v=" << s.v << " (expected x~=" << R << " y~=" << R << " v=" << v0
                       << ")\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// S3. watchdog: engages only after the configured timeout elapses; while
// engaged, brakes |v| monotonically to <0.005 with no sign flip and holds
// the last commanded steering; a new command disengages immediately.
// ---------------------------------------------------------------------
bool test_watchdog_engage_brake_recover() {
    bool ok = true;
    const double L = 0.29;
    const double dt = 0.01;
    const double max_accel = 0.73;
    const double watchdog_timeout_s = 0.25;
    const double last_cmd_a = 0.0;
    const double last_cmd_delta = 0.2;  // must be held throughout braking.

    mpc::Watchdog wd;
    mpc::State4 s;
    s.v = 0.3;

    // Before the timeout elapses, the watchdog must not engage.
    {
        bool engaged_early = wd.update(watchdog_timeout_s - 0.01, watchdog_timeout_s);
        if (engaged_early) {
            std::cerr << "    watchdog engaged before timeout elapsed\n";
            ok = false;
        }
    }

    bool engaged_seen = false;
    bool saw_just_engaged = false;
    double age = 0.0;
    int i = 0;
    for (; i < 10000; ++i) {
        age += dt;
        bool engaged = wd.update(age, watchdog_timeout_s);
        if (engaged) {
            engaged_seen = true;
            if (wd.just_engaged()) {
                saw_just_engaged = true;
            }
        }

        mpc::ResolvedCommand r =
            mpc::resolve_command(engaged, s.v, last_cmd_a, last_cmd_delta, max_accel);

        if (engaged) {
            if (!near_eq(r.steering, last_cmd_delta, 1e-12)) {
                std::cerr << "    steering not held during braking at step " << i << ": got "
                           << r.steering << " expected " << last_cmd_delta << "\n";
                ok = false;
            }
            const double prev_v = s.v;
            const double next_v_estimate = prev_v + r.accel * dt;
            if (std::fabs(next_v_estimate) > std::fabs(prev_v) + 1e-9) {
                std::cerr << "    |v| increased during braking at step " << i << ": " << prev_v
                           << " -> " << next_v_estimate << "\n";
                ok = false;
            }
            if ((prev_v > 0.0 && next_v_estimate < -1e-9) ||
                (prev_v < 0.0 && next_v_estimate > 1e-9)) {
                std::cerr << "    sign flip during braking at step " << i << ": " << prev_v
                           << " -> " << next_v_estimate << "\n";
                ok = false;
            }
        }

        s = mpc::rollout_step(s, r.accel, r.steering, L, dt);

        if (engaged && std::fabs(s.v) < 0.005) {
            break;
        }
    }

    if (!engaged_seen) {
        std::cerr << "    watchdog never engaged\n";
        ok = false;
    }
    if (!saw_just_engaged) {
        std::cerr << "    just_engaged() never fired\n";
        ok = false;
    }
    if (std::fabs(s.v) >= 0.005) {
        std::cerr << "    braking never reached |v|<0.005, got v=" << s.v << " after " << i
                   << " steps\n";
        ok = false;
    }

    // A new command disengages immediately, regardless of update()'s timing.
    wd.on_command_received();
    if (wd.engaged()) {
        std::cerr << "    expected immediate disengage on command received\n";
        ok = false;
    }
    if (!wd.just_recovered()) {
        std::cerr << "    expected just_recovered() true right after on_command_received()\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// S4. ackermann decode: round-trips against the controller's own
// base64(ASCII-decimal) encoding for positive/negative/zero values;
// malformed payload/empty frame/missing key/non-base64/non-numeric ->
// ok=false, never throws; an unrecognized EXTRA key is tolerated (ignored),
// matching nlohmann::json::at()'s lookup-by-name semantics and the
// "never crash on unexpected input" robustness requirement.
// ---------------------------------------------------------------------
bool test_ackermann_decode_roundtrip() {
    bool ok = true;

    struct Case {
        double speed, steering, accel;
    };
    const std::vector<Case> cases = {
        {0.2, 0.1, 0.3},
        {-0.2, -0.1, -0.3},
        {0.0, 0.0, 0.0},
    };
    for (const auto& c : cases) {
        const std::string payload =
            build_ackermann_payload_like_controller(c.speed, c.steering, c.accel);
        mpc::AckermannCommand cmd = mpc::decode_ackermann_payload(payload);
        if (!cmd.ok || !near_eq(cmd.speed, c.speed, 1e-9) ||
            !near_eq(cmd.steering, c.steering, 1e-9) || !near_eq(cmd.accel, c.accel, 1e-9)) {
            std::cerr << "    round-trip mismatch for (" << c.speed << "," << c.steering << ","
                       << c.accel << "): ok=" << cmd.ok << " got (" << cmd.speed << ","
                       << cmd.steering << "," << cmd.accel << ")\n";
            ok = false;
        }
    }

    // Extra unrecognized key ("current", as vesc_zmq_sender sometimes sends)
    // must not cause rejection.
    {
        nlohmann::json j;
        j["speed"] = encode_field_like_controller(0.5);
        j["steering"] = encode_field_like_controller(0.1);
        j["accel"] = encode_field_like_controller(0.2);
        j["current"] = encode_field_like_controller(1.0);
        mpc::AckermannCommand cmd = mpc::decode_ackermann_payload(j.dump());
        if (!cmd.ok || !near_eq(cmd.speed, 0.5, 1e-9)) {
            std::cerr << "    extra unrecognized key should be tolerated, got ok=" << cmd.ok
                       << "\n";
            ok = false;
        }
    }

    // Malformed inputs must never throw and must report ok=false.
    const std::vector<std::string> bad_payloads = {
        "",                                                    // empty frame
        "not json at all",                                     // malformed / non-JSON
        "{ this is not valid json",                             // malformed JSON, starts with '{'
        R"({"speed":"###notbase64###","steering":"AAA=","accel":"AAA="})",  // non-base64
        R"({"steering":"AAA=","accel":"AAA="})",                            // missing "speed"
        std::string(R"({"speed":"AAA=","steering":"AAA=","accel":")") +
            base64_encode(std::string("not_a_number")) + "\"}",  // non-numeric decoded value
    };
    for (const auto& p : bad_payloads) {
        mpc::AckermannCommand cmd;
        try {
            cmd = mpc::decode_ackermann_payload(p);
        } catch (const std::exception& ex) {
            std::cerr << "    decode_ackermann_payload threw on malformed payload '" << p
                       << "': " << ex.what() << "\n";
            ok = false;
            continue;
        }
        if (cmd.ok) {
            std::cerr << "    expected ok=false for malformed payload: '" << p << "'\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// S5. localization encode: the bytes produced by encode_localization_payload
// are accepted by main.cpp's ACTUAL parse path, mirrored here exactly:
//   if (last_payload.front() == '{') {
//       auto json_data = nlohmann::json::parse(last_payload);
//       current_pose.x = json_data.at("x").get<double>();
//       ... y, yaw ...
//   }
// Round-trips x/y/yaw including negative values and yaw outside [-pi, pi]
// (main.cpp never wraps yaw on the receive side either).
// ---------------------------------------------------------------------
bool test_localization_encode_matches_controller_parse() {
    bool ok = true;

    struct Case {
        double x, y, yaw;
    };
    const std::vector<Case> cases = {
        {0.0, 0.0, 0.0},
        {1.234567, -2.345678, 0.5},
        {-5.5, 3.3, -3.0},
        {0.1, 0.2, 4.5},    // yaw beyond +pi, unwrapped
        {-0.1, -0.2, -4.5}, // yaw beyond -pi, unwrapped
    };

    for (const auto& c : cases) {
        const std::string payload = mpc::encode_localization_payload(c.x, c.y, c.yaw);

        if (payload.empty() || payload.front() != '{') {
            std::cerr << "    payload rejected by controller's front()=='{' gate: '" << payload
                       << "'\n";
            ok = false;
            continue;
        }

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(payload);
        } catch (const std::exception& ex) {
            std::cerr << "    controller-style json::parse threw: " << ex.what() << "\n";
            ok = false;
            continue;
        }

        try {
            const double gx = j.at("x").get<double>();
            const double gy = j.at("y").get<double>();
            const double gyaw = j.at("yaw").get<double>();
            if (!near_eq(gx, c.x, 1e-9) || !near_eq(gy, c.y, 1e-9) || !near_eq(gyaw, c.yaw, 1e-9)) {
                std::cerr << "    round-trip mismatch: got (" << gx << "," << gy << "," << gyaw
                           << ") expected (" << c.x << "," << c.y << "," << c.yaw << ")\n";
                ok = false;
            }
        } catch (const std::exception& ex) {
            std::cerr << "    controller-style .at(\"x\"/\"y\"/\"yaw\") lookup threw: "
                       << ex.what() << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// S6. clamping: |a| > max_accel and |delta| > max_steer arrive clamped,
// both at the clamp_command() level and once integrated into the state
// update via mpc::rollout_step.
// ---------------------------------------------------------------------
bool test_clamp_command() {
    bool ok = true;
    const double max_accel = 0.73;
    const double max_steer = 0.33;

    // Within bounds: unchanged, not flagged.
    {
        mpc::ClampResult r = mpc::clamp_command(0.2, -0.1, max_accel, max_steer);
        if (r.accel_clamped || r.steering_clamped || !near_eq(r.accel, 0.2, 1e-12) ||
            !near_eq(r.steering, -0.1, 1e-12)) {
            std::cerr << "    in-bounds command should pass through unclamped\n";
            ok = false;
        }
    }

    // accel over/under.
    {
        mpc::ClampResult r = mpc::clamp_command(5.0, 0.1, max_accel, max_steer);
        if (!r.accel_clamped || !near_eq(r.accel, max_accel, 1e-12) || r.steering_clamped) {
            std::cerr << "    accel-over-limit clamp failed: accel=" << r.accel << "\n";
            ok = false;
        }
    }
    {
        mpc::ClampResult r = mpc::clamp_command(-5.0, 0.0, max_accel, max_steer);
        if (!r.accel_clamped || !near_eq(r.accel, -max_accel, 1e-12)) {
            std::cerr << "    accel-under-limit clamp failed: accel=" << r.accel << "\n";
            ok = false;
        }
    }

    // steering over/under.
    {
        mpc::ClampResult r = mpc::clamp_command(0.0, 5.0, max_accel, max_steer);
        if (!r.steering_clamped || !near_eq(r.steering, max_steer, 1e-12)) {
            std::cerr << "    steering-over-limit clamp failed: steering=" << r.steering << "\n";
            ok = false;
        }
    }
    {
        mpc::ClampResult r = mpc::clamp_command(0.0, -5.0, max_accel, max_steer);
        if (!r.steering_clamped || !near_eq(r.steering, -max_steer, 1e-12)) {
            std::cerr << "    steering-under-limit clamp failed: steering=" << r.steering << "\n";
            ok = false;
        }
    }

    // Integration effect: the clamped values, not the raw ones, are what
    // actually arrive in the state update.
    {
        const mpc::State4 s0{0.0, 0.0, 0.0, 0.2};
        const double L = 0.29, dt = 0.05;
        mpc::ClampResult r = mpc::clamp_command(10.0, 10.0, max_accel, max_steer);
        mpc::State4 clamped_next = mpc::rollout_step(s0, r.accel, r.steering, L, dt);

        const double expected_v = s0.v + max_accel * dt;
        if (!near_eq(clamped_next.v, expected_v, 1e-9)) {
            std::cerr << "    clamped state update used unclamped accel: v=" << clamped_next.v
                       << " expected " << expected_v << "\n";
            ok = false;
        }

        mpc::State4 unclamped_next = mpc::rollout_step(s0, 10.0, 10.0, L, dt);
        if (near_eq(clamped_next.v, unclamped_next.v, 1e-6) ||
            near_eq(clamped_next.yaw, unclamped_next.yaw, 1e-6)) {
            std::cerr << "    clamped state update should differ materially from an unclamped "
                          "rollout\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// FEATURE A (mpc_robot_sim actuation noise) test helpers/tests.
// ---------------------------------------------------------------------

double mean_of(const std::vector<double>& v) {
    double sum = 0.0;
    for (double x : v) sum += x;
    return v.empty() ? 0.0 : sum / static_cast<double>(v.size());
}

double stddev_of(const std::vector<double>& v, double mean) {
    if (v.size() < 2) return 0.0;
    double sq_sum = 0.0;
    for (double x : v) sq_sum += (x - mean) * (x - mean);
    return std::sqrt(sq_sum / static_cast<double>(v.size()));
}

// ---------------------------------------------------------------------
// N1. sigma=0 => applied == clamped command exactly (bitwise), for a sweep
// of commands including several that are out of actuator bounds (so
// clamp_command's own clamping is actually exercised, not just the trivial
// in-bounds path). Also covers the ONE-CHANNEL-ZERO case: with only ONE of
// the two independent channels' sigma at 0 (the other nonzero), THAT
// channel's perturbation must still come back exactly 0.0 (bitwise
// untouched), regardless of the other channel being actively drawn from the
// RNG -- proving the two channels are truly decoupled.
// ---------------------------------------------------------------------
bool test_noise_sigma_zero_bit_identical() {
    bool ok = true;
    const double max_accel = 0.73;
    const double max_steer = 0.33;

    // A fixed, nonzero seed -- both sigmas=0 must short-circuit before ever
    // touching the RNG, so the seed value must not matter.
    mpc::NoiseModel nm(/*accel_sigma_pct=*/0.0, /*steer_sigma_pct=*/0.0, /*seed=*/999);
    if (!near_eq(nm.accel_sigma_pct(), 0.0, 0.0) || !near_eq(nm.steer_sigma_pct(), 0.0, 0.0)) {
        std::cerr << "    NoiseModel(0.0, 0.0, seed) should report both sigma_pct()s ==0.0 "
                      "exactly\n";
        ok = false;
    }

    struct Case {
        double accel, steering;
    };
    const std::vector<Case> cases = {
        {0.0, 0.0},       {0.2, -0.1},      {-0.2, 0.1},     {max_accel, max_steer},
        {-max_accel, -max_steer}, {5.0, 5.0},  {-5.0, -5.0}, {5.0, -5.0},
    };
    for (const auto& c : cases) {
        mpc::NoiseModel::Perturbation pert = nm.sample_command(max_accel, max_steer);
        if (pert.d_accel != 0.0 || pert.d_steer != 0.0) {
            std::cerr << "    both-sigma=0 sample_command() returned nonzero perturbation ("
                       << pert.d_accel << "," << pert.d_steer << ")\n";
            ok = false;
        }
        mpc::ClampResult applied = mpc::clamp_command(c.accel + pert.d_accel, c.steering + pert.d_steer,
                                                        max_accel, max_steer);
        mpc::ClampResult clamped = mpc::clamp_command(c.accel, c.steering, max_accel, max_steer);
        // Bitwise (==, not near_eq) -- this is the whole point of N1.
        if (applied.accel != clamped.accel || applied.steering != clamped.steering) {
            std::cerr << "    (" << c.accel << "," << c.steering << "): applied=(" << applied.accel
                       << "," << applied.steering << ") != clamped=(" << clamped.accel << ","
                       << clamped.steering << ") at both sigma=0\n";
            ok = false;
        }
    }

    // One-channel-zero: accel_sigma=0, steer_sigma nonzero -- d_accel must
    // stay exactly 0.0 across many samples; d_steer should show real
    // variation (RNG genuinely advancing for that channel only).
    {
        mpc::NoiseModel nm_accel_zero(/*accel_sigma_pct=*/0.0, /*steer_sigma_pct=*/0.12,
                                        /*seed=*/1001);
        bool saw_nonzero_steer = false;
        bool steer_varied = false;
        double first_steer = 0.0;
        for (int i = 0; i < 50; ++i) {
            mpc::NoiseModel::Perturbation pert = nm_accel_zero.sample_command(max_accel, max_steer);
            if (pert.d_accel != 0.0) {
                std::cerr << "    accel_sigma=0 (steer_sigma=0.12): d_accel=" << pert.d_accel
                           << " expected exactly 0.0 at sample " << i << "\n";
                ok = false;
            }
            if (pert.d_steer != 0.0) saw_nonzero_steer = true;
            if (i == 0) {
                first_steer = pert.d_steer;
            } else if (!near_eq(pert.d_steer, first_steer, 1e-15)) {
                steer_varied = true;
            }
        }
        if (!saw_nonzero_steer || !steer_varied) {
            std::cerr << "    accel_sigma=0/steer_sigma=0.12: steer channel never varied -- RNG "
                          "may not be advancing independently of the zeroed accel channel\n";
            ok = false;
        }
    }

    // Symmetric case: steer_sigma=0, accel_sigma nonzero -- d_steer must
    // stay exactly 0.0 across many samples.
    {
        mpc::NoiseModel nm_steer_zero(/*accel_sigma_pct=*/0.15, /*steer_sigma_pct=*/0.0,
                                        /*seed=*/1002);
        bool saw_nonzero_accel = false;
        bool accel_varied = false;
        double first_accel = 0.0;
        for (int i = 0; i < 50; ++i) {
            mpc::NoiseModel::Perturbation pert = nm_steer_zero.sample_command(max_accel, max_steer);
            if (pert.d_steer != 0.0) {
                std::cerr << "    steer_sigma=0 (accel_sigma=0.15): d_steer=" << pert.d_steer
                           << " expected exactly 0.0 at sample " << i << "\n";
                ok = false;
            }
            if (pert.d_accel != 0.0) saw_nonzero_accel = true;
            if (i == 0) {
                first_accel = pert.d_accel;
            } else if (!near_eq(pert.d_accel, first_accel, 1e-15)) {
                accel_varied = true;
            }
        }
        if (!saw_nonzero_accel || !accel_varied) {
            std::cerr << "    steer_sigma=0/accel_sigma=0.15: accel channel never varied -- RNG "
                          "may not be advancing independently of the zeroed steer channel\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// N2. Fixed seed, >=2000 per-command perturbations. Sample mean within
// +-0.01*limit of 0 and sample std within 15% of THAT CHANNEL's own
// sigma_pct*limit, on BOTH channels independently. Accel and steer are
// deliberately given DIFFERENT sigmas here (rather than one shared value) so
// a cross-wired implementation (e.g. steer accidentally sampling with
// accel's sigma) would fail this check.
// ---------------------------------------------------------------------
bool test_noise_statistics() {
    bool ok = true;
    const double max_accel = 0.73;
    const double max_steer = 0.33;
    const double accel_sigma_pct = 0.10;
    const double steer_sigma_pct = 0.18;
    constexpr int kN = 2000;

    mpc::NoiseModel nm(accel_sigma_pct, steer_sigma_pct, /*seed=*/424242ULL);
    std::vector<double> accel_samples, steer_samples;
    accel_samples.reserve(kN);
    steer_samples.reserve(kN);
    for (int i = 0; i < kN; ++i) {
        mpc::NoiseModel::Perturbation pert = nm.sample_command(max_accel, max_steer);
        accel_samples.push_back(pert.d_accel);
        steer_samples.push_back(pert.d_steer);
    }

    const double expected_std_accel = accel_sigma_pct * max_accel;
    const double expected_std_steer = steer_sigma_pct * max_steer;

    const double mean_accel = mean_of(accel_samples);
    const double mean_steer = mean_of(steer_samples);
    const double std_accel = stddev_of(accel_samples, mean_accel);
    const double std_steer = stddev_of(steer_samples, mean_steer);

    if (std::fabs(mean_accel) > 0.01 * max_accel) {
        std::cerr << "    accel sample mean too far from 0: " << mean_accel << " (tol "
                   << 0.01 * max_accel << ")\n";
        ok = false;
    }
    if (std::fabs(mean_steer) > 0.01 * max_steer) {
        std::cerr << "    steer sample mean too far from 0: " << mean_steer << " (tol "
                   << 0.01 * max_steer << ")\n";
        ok = false;
    }
    if (std::fabs(std_accel - expected_std_accel) > 0.15 * expected_std_accel) {
        std::cerr << "    accel sample std " << std_accel << " not within 15% of expected "
                   << expected_std_accel << "\n";
        ok = false;
    }
    if (std::fabs(std_steer - expected_std_steer) > 0.15 * expected_std_steer) {
        std::cerr << "    steer sample std " << std_steer << " not within 15% of expected "
                   << expected_std_steer << "\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// N3. Perturbation held constant between commands (i.e. the applied value
// computed from ONE sample_command() call stays identical across repeated
// "ticks" that don't call it again), and resampled -> a (virtually certain,
// given a continuous distribution) DIFFERENT perturbation on the next
// explicit sample_command() call, mirroring robot_sim.cpp's
// "sample once per new command, hold via last_applied_*" pattern.
// ---------------------------------------------------------------------
bool test_noise_held_until_new_command() {
    bool ok = true;
    const double max_accel = 0.73;
    const double max_steer = 0.33;
    const double last_cmd_accel = 0.2, last_cmd_steering = -0.1;

    mpc::NoiseModel nm(/*accel_sigma_pct=*/0.15, /*steer_sigma_pct=*/0.20, /*seed=*/13579ULL);

    // "Command 1" arrives: sample once.
    mpc::NoiseModel::Perturbation pert1 = nm.sample_command(max_accel, max_steer);
    mpc::ClampResult applied1 = mpc::clamp_command(last_cmd_accel + pert1.d_accel,
                                                     last_cmd_steering + pert1.d_steer, max_accel,
                                                     max_steer);

    // Several "ticks" hold last_applied_* -- i.e. simply reusing applied1
    // without calling sample_command() again -- must reproduce the exact
    // same numbers every time (this is trivially true of re-reading a
    // stored value, but pins down that robot_sim.cpp's "only sample on a
    // NEW command" contract is meaningful: recomputing from the SAME
    // pert1/last_cmd_* must be idempotent).
    for (int tick = 0; tick < 5; ++tick) {
        mpc::ClampResult held = mpc::clamp_command(last_cmd_accel + pert1.d_accel,
                                                     last_cmd_steering + pert1.d_steer, max_accel,
                                                     max_steer);
        if (held.accel != applied1.accel || held.steering != applied1.steering) {
            std::cerr << "    held perturbation changed across tick " << tick << " without a new "
                          "command\n";
            ok = false;
        }
    }

    // "Command 2" arrives: resample.
    mpc::NoiseModel::Perturbation pert2 = nm.sample_command(max_accel, max_steer);
    if (near_eq(pert1.d_accel, pert2.d_accel, 1e-12) && near_eq(pert1.d_steer, pert2.d_steer, 1e-12)) {
        std::cerr << "    resampling on a new command produced an identical perturbation -- "
                      "sample_command() may not be drawing fresh randomness\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// N4. Clamp-after-noise: a command already AT the actuator limit, plus
// noise (sigma at its max allowed 0.25, so perturbations are large), can
// never exceed the limit once run through apply_command_noise() -- the
// SAME pure function robot_sim.cpp's ackermann-command handler calls (NOT a
// hand-rederivation of the add-then-clamp sequence), so this test exercises
// robot_sim.cpp's real code path bit-for-bit, not just an independent
// reimplementation of the intended ordering. So the test is not vacuous,
// at least one sampled perturbation must be nonzero and at least one clamp
// must actually trigger.
// ---------------------------------------------------------------------
bool test_noise_clamp_after() {
    bool ok = true;
    const double max_accel = 0.73;
    const double max_steer = 0.33;

    mpc::NoiseModel nm(mpc::NoiseModel::kMaxSigmaPct, mpc::NoiseModel::kMaxSigmaPct,
                        /*seed=*/2468ULL);

    bool saw_nonzero_perturbation = false;
    bool saw_clamp_trigger = false;
    constexpr int kN = 500;
    for (int i = 0; i < kN; ++i) {
        mpc::NoiseModel::Perturbation pert = nm.sample_command(max_accel, max_steer);
        if (pert.d_accel != 0.0 || pert.d_steer != 0.0) {
            saw_nonzero_perturbation = true;
        }
        mpc::ClampResult applied =
            mpc::apply_command_noise(max_accel, max_steer, pert, max_accel, max_steer);
        if (applied.accel_clamped || applied.steering_clamped) {
            saw_clamp_trigger = true;
        }
        if (applied.accel > max_accel + 1e-9 || applied.accel < -max_accel - 1e-9) {
            std::cerr << "    applied accel " << applied.accel << " exceeds +-max_accel ("
                       << max_accel << ") at sample " << i << "\n";
            ok = false;
        }
        if (applied.steering > max_steer + 1e-9 || applied.steering < -max_steer - 1e-9) {
            std::cerr << "    applied steering " << applied.steering << " exceeds +-max_steer ("
                       << max_steer << ") at sample " << i << "\n";
            ok = false;
        }
    }

    if (!saw_nonzero_perturbation) {
        std::cerr << "    never saw a nonzero perturbation across " << kN
                   << " samples -- test may not be exercising noise at all\n";
        ok = false;
    }
    // Starting exactly at the limit with sigma=0.25 (a large perturbation
    // scale), a positive-direction draw should clamp essentially every run;
    // require at least one to keep the test meaningful without being flaky
    // about the exact count.
    if (!saw_clamp_trigger) {
        std::cerr << "    never saw clamp_command() actually clamp across " << kN
                   << " samples starting at the actuator limit -- test may be vacuous\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// N5. Config-payload parsing (parse_sim_config_payload), pure-function
// level: valid value on EACH channel independently, both channels at once,
// out-of-range values (clamped, *_ok=true), cross-channel independence (a
// wrong-typed/missing field on one channel must not disturb the other in
// the SAME payload), and structurally malformed payloads (both channels
// retained, ok=false) -- plus unknown-field tolerance.
// ---------------------------------------------------------------------
bool test_sim_config_payload_parsing() {
    bool ok = true;

    // Valid, in-range, ACCEL field only -- steer channel must be untouched.
    {
        mpc::SimConfigParseResult r =
            mpc::parse_sim_config_payload(R"({"noise_sigma_pct": 0.15})", 0.05, 0.09);
        if (!r.accel_ok || r.accel_was_clamped || !near_eq(r.accel_sigma_pct, 0.15, 1e-9)) {
            std::cerr << "    accel-only payload: accel_ok=" << r.accel_ok
                       << " accel_was_clamped=" << r.accel_was_clamped << " accel_sigma_pct="
                       << r.accel_sigma_pct
                       << " (expected ok=true, was_clamped=false, sigma_pct=0.15)\n";
            ok = false;
        }
        if (r.steer_ok || !near_eq(r.steer_sigma_pct, 0.09, 1e-9)) {
            std::cerr << "    accel-only payload: steer channel should be retained untouched, got "
                          "steer_ok=" << r.steer_ok << " steer_sigma_pct=" << r.steer_sigma_pct
                       << " (expected ok=false, sigma_pct=0.09 retained)\n";
            ok = false;
        }
        if (!r.ok) {
            std::cerr << "    accel-only payload: overall ok should be true (accel_ok)\n";
            ok = false;
        }
    }

    // Valid, in-range, STEER field only -- accel channel must be untouched.
    {
        mpc::SimConfigParseResult r =
            mpc::parse_sim_config_payload(R"({"steer_noise_sigma_pct": 0.22})", 0.05, 0.09);
        if (!r.steer_ok || r.steer_was_clamped || !near_eq(r.steer_sigma_pct, 0.22, 1e-9)) {
            std::cerr << "    steer-only payload: steer_ok=" << r.steer_ok
                       << " steer_was_clamped=" << r.steer_was_clamped << " steer_sigma_pct="
                       << r.steer_sigma_pct
                       << " (expected ok=true, was_clamped=false, sigma_pct=0.22)\n";
            ok = false;
        }
        if (r.accel_ok || !near_eq(r.accel_sigma_pct, 0.05, 1e-9)) {
            std::cerr << "    steer-only payload: accel channel should be retained untouched, got "
                          "accel_ok=" << r.accel_ok << " accel_sigma_pct=" << r.accel_sigma_pct
                       << " (expected ok=false, sigma_pct=0.05 retained)\n";
            ok = false;
        }
    }

    // Both fields present at once, DIFFERENT values -- both channels update
    // independently within the same payload.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"noise_sigma_pct": 0.11, "steer_noise_sigma_pct": 0.21})", 0.0, 0.0);
        if (!r.accel_ok || !near_eq(r.accel_sigma_pct, 0.11, 1e-9) || !r.steer_ok ||
            !near_eq(r.steer_sigma_pct, 0.21, 1e-9)) {
            std::cerr << "    both-fields payload: accel_ok=" << r.accel_ok << " accel_sigma_pct="
                       << r.accel_sigma_pct << " steer_ok=" << r.steer_ok << " steer_sigma_pct="
                       << r.steer_sigma_pct << " (expected accel=0.11 steer=0.21, both ok)\n";
            ok = false;
        }
    }

    // Out-of-range high, per channel -> clamped to kMaxSigmaPct, *_ok still
    // true.
    {
        mpc::SimConfigParseResult r =
            mpc::parse_sim_config_payload(R"({"noise_sigma_pct": 0.9})", 0.05, 0.09);
        if (!r.accel_ok || !r.accel_was_clamped ||
            !near_eq(r.accel_sigma_pct, mpc::NoiseModel::kMaxSigmaPct, 1e-9)) {
            std::cerr << "    out-of-range-high accel payload: accel_ok=" << r.accel_ok
                       << " accel_was_clamped=" << r.accel_was_clamped << " accel_sigma_pct="
                       << r.accel_sigma_pct << " (expected ok=true, was_clamped=true, sigma_pct="
                       << mpc::NoiseModel::kMaxSigmaPct << ")\n";
            ok = false;
        }
    }
    {
        mpc::SimConfigParseResult r =
            mpc::parse_sim_config_payload(R"({"steer_noise_sigma_pct": 0.9})", 0.05, 0.09);
        if (!r.steer_ok || !r.steer_was_clamped ||
            !near_eq(r.steer_sigma_pct, mpc::NoiseModel::kMaxSigmaPct, 1e-9)) {
            std::cerr << "    out-of-range-high steer payload: steer_ok=" << r.steer_ok
                       << " steer_was_clamped=" << r.steer_was_clamped << " steer_sigma_pct="
                       << r.steer_sigma_pct << " (expected ok=true, was_clamped=true, sigma_pct="
                       << mpc::NoiseModel::kMaxSigmaPct << ")\n";
            ok = false;
        }
    }

    // Out-of-range low (negative), ACCEL channel -> clamped to kMinSigmaPct
    // (0), ok true.
    {
        mpc::SimConfigParseResult r =
            mpc::parse_sim_config_payload(R"({"noise_sigma_pct": -0.3})", 0.05, 0.09);
        if (!r.accel_ok || !r.accel_was_clamped ||
            !near_eq(r.accel_sigma_pct, mpc::NoiseModel::kMinSigmaPct, 1e-9)) {
            std::cerr << "    out-of-range-low accel payload: accel_ok=" << r.accel_ok
                       << " accel_was_clamped=" << r.accel_was_clamped << " accel_sigma_pct="
                       << r.accel_sigma_pct << " (expected ok=true, was_clamped=true, sigma_pct="
                       << mpc::NoiseModel::kMinSigmaPct << ")\n";
            ok = false;
        }
    }

    // Unknown extra field ignored.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"noise_sigma_pct": 0.1, "extra_field": 123})", 0.05, 0.09);
        if (!r.accel_ok || !near_eq(r.accel_sigma_pct, 0.1, 1e-9)) {
            std::cerr << "    payload with unknown extra field should still parse: accel_ok="
                       << r.accel_ok << " accel_sigma_pct=" << r.accel_sigma_pct << "\n";
            ok = false;
        }
    }

    // Cross-channel independence: a wrong-typed accel field must not
    // disturb a VALID steer field present in the SAME payload (proves
    // per-channel parsing, not an all-or-nothing payload rejection).
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"noise_sigma_pct": "not_a_number", "steer_noise_sigma_pct": 0.14})", 0.05, 0.09);
        if (r.accel_ok || !near_eq(r.accel_sigma_pct, 0.05, 1e-9)) {
            std::cerr << "    wrong-typed accel + valid steer: accel channel should be retained, "
                          "got accel_ok=" << r.accel_ok << " accel_sigma_pct=" << r.accel_sigma_pct
                       << "\n";
            ok = false;
        }
        if (!r.steer_ok || !near_eq(r.steer_sigma_pct, 0.14, 1e-9)) {
            std::cerr << "    wrong-typed accel + valid steer: steer channel should still parse, "
                          "got steer_ok=" << r.steer_ok << " steer_sigma_pct=" << r.steer_sigma_pct
                       << "\n";
            ok = false;
        }
        if (!r.ok) {
            std::cerr << "    wrong-typed accel + valid steer: overall ok should be true (steer "
                          "channel succeeded)\n";
            ok = false;
        }
    }

    // Structurally malformed / no-recognized-field payloads -> both
    // channels ok=false, both retained (current values echoed back
    // unchanged), never throws.
    const std::vector<std::string> malformed = {
        "",                                          // empty frame
        "not json at all",                           // non-JSON, doesn't start with '{'
        "{ this is not valid json",                  // malformed JSON, starts with '{'
        R"({"some_other_field": 1})",                 // missing both known keys
        R"({"noise_sigma_pct": "not_a_number"})",     // wrong type, accel only present
        R"({"steer_noise_sigma_pct": "not_a_number"})",  // wrong type, steer only present
    };
    for (const auto& payload : malformed) {
        const double current_accel = 0.22;
        const double current_steer = 0.13;
        mpc::SimConfigParseResult r;
        try {
            r = mpc::parse_sim_config_payload(payload, current_accel, current_steer);
        } catch (const std::exception& ex) {
            std::cerr << "    parse_sim_config_payload threw on malformed payload '" << payload
                       << "': " << ex.what() << "\n";
            ok = false;
            continue;
        }
        if (r.ok || r.accel_ok || r.steer_ok ||
            !near_eq(r.accel_sigma_pct, current_accel, 1e-12) ||
            !near_eq(r.steer_sigma_pct, current_steer, 1e-12)) {
            std::cerr << "    malformed payload '" << payload << "': expected ok=false, both "
                          "channels retained (accel=" << current_accel << ", steer="
                       << current_steer << "), got ok=" << r.ok << " accel_ok=" << r.accel_ok
                       << " accel_sigma_pct=" << r.accel_sigma_pct << " steer_ok=" << r.steer_ok
                       << " steer_sigma_pct=" << r.steer_sigma_pct << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// FEATURE 2A test P1: compute_effective_elapsed -- pure helper. Covers
// unpaused elapsed, frozen-during-pause (independent of `now`), and
// monotonic-non-decreasing behavior across multiple pause/resume cycles,
// mirroring exactly how main.cpp is expected to drive it: `pause_offset`
// only ever grows, by (resume_time - pause_start), on each RESUME.
// ---------------------------------------------------------------------
bool test_compute_effective_elapsed() {
    bool ok = true;
    const double traj_start = 1000.0;

    // Unpaused: elapsed == now - traj_start exactly (pause_offset=0,
    // paused=false, pause_start irrelevant).
    {
        for (double now : {1000.0, 1002.5, 1010.0}) {
            double got = mpc::compute_effective_elapsed(now, traj_start, /*pause_offset=*/0.0,
                                                          /*paused=*/false, /*pause_start=*/0.0);
            double want = now - traj_start;
            if (!near_eq(got, want, 1e-9)) {
                std::cerr << "    unpaused mismatch at now=" << now << ": got " << got
                           << " want " << want << "\n";
                ok = false;
            }
        }
    }

    // Frozen during pause: pause begins at pause_start=1002.0 (elapsed so far
    // = 2.0); for ANY later `now` while still paused, the result must stay
    // exactly 2.0 -- the clock does not advance.
    {
        const double pause_start = 1002.0;
        const double frozen_elapsed = pause_start - traj_start;  // 2.0
        for (double now : {1002.0, 1003.5, 1010.0, 1030.0}) {
            double got = mpc::compute_effective_elapsed(now, traj_start, /*pause_offset=*/0.0,
                                                          /*paused=*/true, pause_start);
            if (!near_eq(got, frozen_elapsed, 1e-9)) {
                std::cerr << "    frozen-during-pause mismatch at now=" << now << ": got " << got
                           << " want " << frozen_elapsed << "\n";
                ok = false;
            }
        }
    }

    // Multiple pause-resume cycles: simulate main.cpp's bookkeeping
    // (pause_offset += (resume_time - pause_start) on each RESUME) and check
    // the overall elapsed sequence is monotonic non-decreasing, with the
    // pause windows contributing zero addl elapsed.
    {
        double pause_offset = 0.0;
        std::vector<double> elapsed_trace;

        // Run 0..2s.
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1002.0, traj_start, pause_offset, false, 0.0));
        // Pause at t=2 (now=1002.0); frozen samples while paused.
        const double pause1_start = 1002.0;
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1003.0, traj_start, pause_offset, true, pause1_start));
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1005.0, traj_start, pause_offset, true, pause1_start));
        // Resume at now=1005.0 -> pause_offset += (1005-1002) = 3.0.
        pause_offset += (1005.0 - pause1_start);
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1006.0, traj_start, pause_offset, false, 0.0));
        // Pause again at now=1008.0.
        const double pause2_start = 1008.0;
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1008.0, traj_start, pause_offset, true, pause2_start));
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1012.0, traj_start, pause_offset, true, pause2_start));
        // Resume at now=1012.0 -> pause_offset += (1012-1008) = 4.0 (total 7.0).
        pause_offset += (1012.0 - pause2_start);
        elapsed_trace.push_back(
            mpc::compute_effective_elapsed(1013.0, traj_start, pause_offset, false, 0.0));

        for (size_t i = 1; i < elapsed_trace.size(); ++i) {
            if (elapsed_trace[i] < elapsed_trace[i - 1] - 1e-9) {
                std::cerr << "    elapsed not monotonic non-decreasing at index " << i << ": "
                           << elapsed_trace[i - 1] << " -> " << elapsed_trace[i] << "\n";
                ok = false;
            }
        }

        // Sanity on a couple of the concrete values: post-pause1-resume
        // elapsed at now=1006 should be (1006-1000) - 3.0 = 3.0 (i.e. the 1s
        // of paused wall time between now=1005 and now=1006 doesn't count,
        // but the pause itself, having ended, contributes its own offset).
        if (!near_eq(elapsed_trace[3], 3.0, 1e-9)) {
            std::cerr << "    post-resume-1 elapsed mismatch: got " << elapsed_trace[3]
                       << " want 3.0\n";
            ok = false;
        }
        // Final unpaused sample at now=1013: (1013-1000) - 7.0 = 6.0.
        if (!near_eq(elapsed_trace.back(), 6.0, 1e-9)) {
            std::cerr << "    final elapsed mismatch: got " << elapsed_trace.back()
                       << " want 6.0\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// FEATURE 2A test P2: MpcSolver::reset_warm_start() debug hook. Mirrors
// test_no_warm_start_across_calls's "reproduce exactly" pattern, but for the
// OPT-IN warm-started path (SolveInputs::use_warm_start=true): solving A
// (buffer empty -> falls back to the flat seed), then a very different B
// (buffer now seeded from A's raw horizon), then reset_warm_start(), then A
// again must reproduce the FIRST A-solve exactly -- proving reset really
// invalidates the stored horizon rather than leaving stale state from B to
// leak into the post-reset solve.
// ---------------------------------------------------------------------
bool test_warm_start_reset_hook() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;

    const auto refs_a = make_straight_refs(0.0, 0.2, mpc::kHorizon, dt);
    mpc::SolveInputs in_a = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, 0.05}, refs_a);
    in_a.use_warm_start = true;

    const auto refs_b = make_straight_refs(-2.0, -0.3, mpc::kHorizon, dt);
    mpc::SolveInputs in_b = make_default_solve_inputs(mpc::State4{-1.0, 0.5, M_PI, -0.1}, refs_b);
    in_b.use_warm_start = true;

    mpc::MpcSolver solver;

    // First A-solve: buffer starts empty, so this must equal a plain
    // (use_warm_start=false) solve of the same inputs -- the opt-in path is
    // behaviorally inert until a warm-started solve has actually happened.
    mpc::SolveInputs in_a_plain = in_a;
    in_a_plain.use_warm_start = false;
    mpc::MpcSolver baseline_solver;
    mpc::MpcOutput baseline = baseline_solver.solve(in_a_plain);

    mpc::MpcOutput out_a1 = solver.solve(in_a);
    if (!near_eq(out_a1.v_cmd, baseline.v_cmd, 1e-9) || !near_eq(out_a1.steer, baseline.steer, 1e-9)) {
        std::cerr << "    first warm-started A-solve (empty buffer) should match a plain solve: "
                      "got ("
                   << out_a1.v_cmd << "," << out_a1.steer << ") want (" << baseline.v_cmd << ","
                   << baseline.steer << ")\n";
        ok = false;
    }

    // Solve B (seeds the buffer from A's raw horizon), then reset, then A
    // again -- must reproduce out_a1 exactly, not something drifted by B.
    mpc::MpcOutput out_b = solver.solve(in_b);
    (void)out_b;
    solver.reset_warm_start();
    mpc::MpcOutput out_a2 = solver.solve(in_a);

    if (!near_eq(out_a1.v_cmd, out_a2.v_cmd, 1e-9) || !near_eq(out_a1.steer, out_a2.steer, 1e-9)) {
        std::cerr << "    repeating A after reset_warm_start() gave a different result: first=("
                   << out_a1.v_cmd << "," << out_a1.steer << ") second=(" << out_a2.v_cmd << ","
                   << out_a2.steer << ") -- reset_warm_start() may not be clearing state\n";
        ok = false;
    }

    // Negative control: WITHOUT an intervening reset, solving B then A again
    // (both warm-started) must actually diverge from the first A-solve --
    // otherwise this whole test would be vacuous (reset_warm_start() could
    // be a no-op and every assertion above would still pass by accident).
    mpc::MpcSolver solver2;
    mpc::MpcOutput ctrl_a1 = solver2.solve(in_a);
    mpc::MpcOutput ctrl_b = solver2.solve(in_b);
    (void)ctrl_b;
    mpc::MpcOutput ctrl_a2 = solver2.solve(in_a);  // no reset in between
    if (near_eq(ctrl_a1.v_cmd, ctrl_a2.v_cmd, 1e-9) && near_eq(ctrl_a1.steer, ctrl_a2.steer, 1e-9)) {
        std::cerr << "    expected warm-started A-solve to DIFFER after an unreset intervening "
                      "B-solve (buffer should have been seeded from B's horizon); got identical "
                      "results -- warm-start seeding may not be taking effect, making the "
                      "reset_warm_start() check above vacuous\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// FEATURE 2A test P3: opt-in warm start (SolveInputs::use_warm_start=true)
// across two CONSECUTIVE solves on a smoothly advancing reference -- the
// live control-loop pattern (main.cpp sets use_warm_start=true every tick
// and only calls reset_warm_start() on a new trajectory / RESUME). Mirrors
// the original "shifted seed + output continuity" intent by checking BOTH:
//   (a) output continuity: v_cmd/steer do not jump between the two solves
//       (consecutive ticks tracking the same straight-line reference one dt
//       apart);
//   (b) the SEED the second solve actually uses is exactly the first
//       solve's raw (pre-rate-clamp) horizon, shifted forward one stage
//       with the last stage duplicated -- i.e. the u[] populated from
//       warm_u_ at the top of MpcSolver::solve() (see MpcCore.cpp).
// (b) is verified without any MpcCore change or new instrumentation, by
// exploiting a legitimate solver property: driving every residual weight to
// 0 makes the Ceres cost (and its Jacobian) identically zero everywhere, so
// the optimizer cannot take a descent step and reports u UNCHANGED from
// wherever solve() seeded it -- turning the existing horizon_a/
// horizon_delta debug hook (see MpcOutput's doc comment) into a direct
// readout of the seed itself.
// ---------------------------------------------------------------------
bool test_warm_start_shifted_seed_and_continuity() {
    bool ok = true;
    const double dt = mpc::MpcParams::defaults().dt;
    const double L = mpc::RobotSpec::defaults().wheel_base;
    const double v = 0.2;

    // (a) Output continuity: one solver instance carries its warm-start
    // buffer across two ticks of a smoothly advancing straight-line
    // reference (tick 2's horizon starts exactly one dt later than tick
    // 1's) -- the live control-loop pattern.
    const auto refs_1 = make_straight_refs(/*base_x=*/0.0, v, mpc::kHorizon, dt);
    mpc::SolveInputs in_1 = make_default_solve_inputs(mpc::State4{0.0, 0.0, 0.0, v}, refs_1);
    in_1.use_warm_start = true;

    mpc::MpcSolver solver;
    mpc::MpcOutput out_1 = solver.solve(in_1);

    // Advance state one dt using the first solve's own commanded (accel,
    // steer), mirroring main.cpp's real dead-reckoning/localization state
    // update between ticks (acceleration formulation: a is a genuine
    // control input, not a fixed 0.0 placeholder).
    mpc::State4 s0{in_1.x0, in_1.y0, in_1.yaw0, in_1.v0};
    mpc::State4 s1 = mpc::rollout_step(s0, out_1.accel, out_1.steer, L, dt);

    const auto refs_2 = make_straight_refs(/*base_x=*/v * dt, v, mpc::kHorizon, dt);
    mpc::SolveInputs in_2 = make_default_solve_inputs(mpc::State4{s1.x, s1.y, s1.yaw, s1.v}, refs_2);
    in_2.last_cmd_a = out_1.accel;
    in_2.last_cmd_delta = out_1.steer;
    in_2.use_warm_start = true;

    mpc::MpcOutput out_2 = solver.solve(in_2);

    const double max_delta_v = mpc::RobotSpec::defaults().max_accel * dt;
    if (std::fabs(out_2.v_cmd - out_1.v_cmd) > max_delta_v + 1e-9) {
        std::cerr << "    v_cmd jumped more than the rate clamp allows across a smoothly "
                      "advancing reference: out_1="
                   << out_1.v_cmd << " out_2=" << out_2.v_cmd << " max_delta_v=" << max_delta_v
                   << "\n";
        ok = false;
    }
    if (std::fabs(out_2.steer - out_1.steer) > 0.05) {
        std::cerr << "    steer jumped implausibly far across a smoothly advancing straight-line "
                      "reference: out_1="
                   << out_1.steer << " out_2=" << out_2.steer << "\n";
        ok = false;
    }

    // (b) Exact seed check, on a FRESH solver so it is independent of (a)'s
    // solver's accumulated state. First, an identical warm-started solve of
    // in_1 -- deterministic, so this must reproduce out_1 exactly (same
    // assumption test P2's baseline check relies on).
    mpc::MpcSolver seed_solver;
    mpc::MpcOutput seed_out_1 = seed_solver.solve(in_1);
    if (!near_eq(seed_out_1.v_cmd, out_1.v_cmd, 1e-9) ||
        !near_eq(seed_out_1.steer, out_1.steer, 1e-9)) {
        std::cerr << "    re-solving in_1 on a fresh solver did not reproduce out_1 -- solve() is "
                      "not deterministic as assumed by this test\n";
        ok = false;
    }

    // Expected seed: seed_out_1's raw horizon, shifted forward one stage,
    // last stage duplicated -- mirrors MpcCore.cpp's solve() shift-seed loop
    // exactly.
    std::vector<double> expected_seed_a(mpc::kHorizon), expected_seed_delta(mpc::kHorizon);
    for (int t = 0; t < mpc::kHorizon - 1; ++t) {
        expected_seed_a[t] = seed_out_1.horizon_a[t + 1];
        expected_seed_delta[t] = seed_out_1.horizon_delta[t + 1];
    }
    expected_seed_a[mpc::kHorizon - 1] = seed_out_1.horizon_a[mpc::kHorizon - 1];
    expected_seed_delta[mpc::kHorizon - 1] = seed_out_1.horizon_delta[mpc::kHorizon - 1];

    // Zero-weight probe: same in_2 inputs, but with every residual weight
    // set to 0 (see comment above) so the returned raw horizon is exactly
    // whatever solve() seeded u[] with -- use_warm_start stays true, so
    // seed_solver's buffer (from seed_out_1) is what gets used.
    mpc::SolveInputs in_2_probe = in_2;
    in_2_probe.w_dist = 0.0;
    in_2_probe.w_yaw = 0.0;
    in_2_probe.w_vel = 0.0;
    in_2_probe.w_lat = 0.0;
    in_2_probe.w_control = 0.0;
    in_2_probe.w_delta_rate = 0.0;

    mpc::MpcOutput probe_out = seed_solver.solve(in_2_probe);

    for (int t = 0; t < mpc::kHorizon; ++t) {
        if (!near_eq(probe_out.horizon_a[t], expected_seed_a[t], 1e-6)) {
            std::cerr << "    seed mismatch at stage " << t
                       << " (a): probe=" << probe_out.horizon_a[t]
                       << " expected(shifted seed)=" << expected_seed_a[t] << "\n";
            ok = false;
        }
        if (!near_eq(probe_out.horizon_delta[t], expected_seed_delta[t], 1e-6)) {
            std::cerr << "    seed mismatch at stage " << t
                       << " (delta): probe=" << probe_out.horizon_delta[t]
                       << " expected(shifted seed)=" << expected_seed_delta[t] << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// FEATURE B (mpc_robot_sim deadband plant model) test helpers/tests.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// D1. Deadband state machine (mpc::step_deadband), pure-function level:
// frozen below breakaway, the JUMP at breakaway (effective_v == the raw
// v_cmd_equiv passed in, not eased toward it), hysteresis staying moving
// across the band between sustain and moving thresholds, the stall back to
// frozen below sustain, and boundary exactness (>= for breakaway, strict <
// for stall). Also covers the sustain<breakaway case that makes this a
// genuine hysteresis band (per RobotSpec::validate()'s 0 <= min_sustain_speed
// <= min_moving_speed requirement) and the pose-frozen consequence via
// mpc::integrate_pose_with_v.
// ---------------------------------------------------------------------
bool test_deadband_state_machine() {
    bool ok = true;
    const double min_moving = 0.15;
    const double min_sustain = 0.05;  // sustain < breakaway -- genuine hysteresis band.

    // From rest, below breakaway: stays not-moving, effective_v pinned at
    // exactly 0, regardless of how close to (but under) the threshold.
    {
        const std::vector<double> below = {0.0, 0.05, 0.1, 0.149, -0.149};
        for (double v : below) {
            mpc::DeadbandStepResult r =
                mpc::step_deadband(v, /*was_moving=*/false, min_moving, min_sustain);
            if (r.moving || r.effective_v != 0.0) {
                std::cerr << "    v=" << v << " below breakaway from rest: expected moving=false, "
                              "effective_v=0, got moving=" << r.moving
                           << " effective_v=" << r.effective_v << "\n";
                ok = false;
            }
        }
    }

    // Crossing breakaway (>=): moving flips true, effective_v == v EXACTLY
    // (bitwise -- the jump, not an eased-in fraction of v).
    {
        const std::vector<double> at_or_above = {0.15, 0.2, 0.5, -0.15, -0.3};
        for (double v : at_or_above) {
            mpc::DeadbandStepResult r =
                mpc::step_deadband(v, /*was_moving=*/false, min_moving, min_sustain);
            if (!r.moving || r.effective_v != v) {
                std::cerr << "    v=" << v << " at/above breakaway from rest: expected moving=true, "
                              "effective_v==v exactly, got moving=" << r.moving
                           << " effective_v=" << r.effective_v << "\n";
                ok = false;
            }
        }
    }

    // Hysteresis: once moving, STAYS moving (driven by the full v) for any
    // |v| down to min_sustain_speed -- specifically including the band BELOW
    // min_moving_speed but AT/ABOVE min_sustain_speed, which is exactly what
    // makes this hysteresis rather than a single threshold.
    {
        const std::vector<double> hysteresis_band = {0.14, 0.1, 0.05, -0.1, -0.05};
        for (double v : hysteresis_band) {
            mpc::DeadbandStepResult r =
                mpc::step_deadband(v, /*was_moving=*/true, min_moving, min_sustain);
            if (!r.moving || r.effective_v != v) {
                std::cerr << "    v=" << v << " in hysteresis band while moving: expected "
                              "moving=true, effective_v==v, got moving=" << r.moving
                           << " effective_v=" << r.effective_v << "\n";
                ok = false;
            }
        }
    }

    // Stall: while moving, dropping BELOW min_sustain_speed flips moving
    // false and snaps effective_v to exactly 0 (not eased out).
    {
        const std::vector<double> below_sustain = {0.049, 0.0, -0.049};
        for (double v : below_sustain) {
            mpc::DeadbandStepResult r =
                mpc::step_deadband(v, /*was_moving=*/true, min_moving, min_sustain);
            if (r.moving || r.effective_v != 0.0) {
                std::cerr << "    v=" << v << " below sustain while moving: expected moving=false, "
                              "effective_v=0, got moving=" << r.moving
                           << " effective_v=" << r.effective_v << "\n";
                ok = false;
            }
        }
    }

    // Boundary exactness: was_moving=false at EXACTLY min_moving_speed ->
    // already moving (>=, not >); was_moving=true at EXACTLY
    // min_sustain_speed -> STILL moving (< is strict, not <=).
    {
        mpc::DeadbandStepResult at_breakaway =
            mpc::step_deadband(min_moving, /*was_moving=*/false, min_moving, min_sustain);
        if (!at_breakaway.moving) {
            std::cerr << "    v==min_moving_speed exactly from rest should already be moving (>=)\n";
            ok = false;
        }
        mpc::DeadbandStepResult at_sustain =
            mpc::step_deadband(min_sustain, /*was_moving=*/true, min_moving, min_sustain);
        if (!at_sustain.moving) {
            std::cerr << "    v==min_sustain_speed exactly while moving should still be moving "
                          "(< is strict, not <=)\n";
            ok = false;
        }
    }

    // sustain == breakaway (no hysteresis band, degenerates to a simple
    // threshold) case, exercised separately for sanity: breakaway and stall
    // happen at the same |v|.
    {
        const double thresh = 0.1;
        mpc::DeadbandStepResult below =
            mpc::step_deadband(0.099, /*was_moving=*/true, thresh, thresh);
        if (below.moving) {
            std::cerr << "    sustain==moving: v just under threshold while moving should stall\n";
            ok = false;
        }
        mpc::DeadbandStepResult at =
            mpc::step_deadband(thresh, /*was_moving=*/false, thresh, thresh);
        if (!at.moving) {
            std::cerr << "    sustain==moving: v==threshold from rest should break away\n";
            ok = false;
        }
    }

    // Pose-frozen consequence (mpc::integrate_pose_with_v, exactly what
    // robot_sim.cpp feeds effective_v into): v_for_pose=0 must leave (x, y,
    // yaw) bit-identically unchanged regardless of steering/dt; a nonzero
    // v_for_pose must move it, and .v is set to v_for_pose exactly.
    {
        const mpc::State4 s{1.5, -2.5, 0.7, 0.0};
        mpc::State4 frozen = mpc::integrate_pose_with_v(s, /*v_for_pose=*/0.0, /*delta=*/0.3,
                                                          /*wheel_base=*/0.29, /*dt=*/0.05);
        if (frozen.x != s.x || frozen.y != s.y || frozen.yaw != s.yaw || frozen.v != 0.0) {
            std::cerr << "    integrate_pose_with_v(v_for_pose=0) should leave pose bit-identical "
                          "and report v=0\n";
            ok = false;
        }
        mpc::State4 moved =
            mpc::integrate_pose_with_v(s, /*v_for_pose=*/0.2, /*delta=*/0.3, 0.29, 0.05);
        if (near_eq(moved.x, s.x, 1e-9) && near_eq(moved.y, s.y, 1e-9) &&
            near_eq(moved.yaw, s.yaw, 1e-9)) {
            std::cerr << "    integrate_pose_with_v(v_for_pose=0.2) should move the pose\n";
            ok = false;
        }
        if (moved.v != 0.2) {
            std::cerr << "    integrate_pose_with_v should set .v to v_for_pose exactly\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// D2. Deadband DISABLED reduces to today's math, bitwise. robot_sim.cpp's
// actual --deadband-off code path calls the pre-existing mpc::rollout_step()
// directly and is therefore bit-identical BY CONSTRUCTION/inspection (see
// robot_sim.cpp's `else` branch -- unmodified by this feature); the
// process-level proof of that is the existing test_mpc_sim_loop.cpp/
// test_mpc_sim_noise.cpp binaries continuing to pass UNMODIFIED without ever
// passing --deadband (see this task's own report). What THIS test verifies
// at the pure-function level is the complementary claim: FEATURE B's new
// pose-integration path (step_deadband + integrate_pose_with_v), when driven
// in the trivially-always-moving regime (zero thresholds -- i.e. breakaway
// happens on tick 1 and it can never stall again), reproduces
// mpc::rollout_step's math EXACTLY, tick-for-tick, across a command sweep
// mixing positive/negative/zero accel and steering -- a regression guard
// against integrate_pose_with_v's formula ever silently drifting from
// rollout_step's.
// ---------------------------------------------------------------------
bool test_deadband_disabled_matches_rollout_step_sweep() {
    bool ok = true;
    const double L = 0.29;
    const double dt = 0.05;

    struct Cmd {
        double a;
        double delta;
    };
    const std::vector<Cmd> sweep = {
        {0.5, 0.0},  {0.5, 0.1},   {0.3, -0.2}, {0.0, 0.15},  {-0.4, 0.0},
        {-0.4, -0.1}, {0.2, 0.05}, {0.0, 0.0},  {0.73, 0.33}, {-0.73, -0.33},
    };

    mpc::State4 rollout_state;   // "today's" pipeline: mpc::rollout_step directly.
    mpc::State4 deadband_state;  // FEATURE B pipeline, thresholds=0 (trivially always moving).
    double v_cmd_equiv = 0.0;
    bool moving = false;

    for (size_t i = 0; i < sweep.size(); ++i) {
        const Cmd& c = sweep[i];
        rollout_state = mpc::rollout_step(rollout_state, c.a, c.delta, L, dt);

        const double v_cmd_equiv_next = v_cmd_equiv + c.a * dt;
        mpc::DeadbandStepResult db = mpc::step_deadband(v_cmd_equiv_next, moving,
                                                          /*min_moving_speed=*/0.0,
                                                          /*min_sustain_speed=*/0.0);
        moving = db.moving;
        deadband_state =
            mpc::integrate_pose_with_v(deadband_state, db.effective_v, c.delta, L, dt);
        v_cmd_equiv = v_cmd_equiv_next;

        if (rollout_state.x != deadband_state.x || rollout_state.y != deadband_state.y ||
            rollout_state.yaw != deadband_state.yaw || rollout_state.v != deadband_state.v) {
            std::cerr << "    tick " << i << ": rollout_step=(" << rollout_state.x << ","
                       << rollout_state.y << "," << rollout_state.yaw << "," << rollout_state.v
                       << ") != deadband(thresholds=0)=(" << deadband_state.x << ","
                       << deadband_state.y << "," << deadband_state.yaw << "," << deadband_state.v
                       << ")\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// RAII stderr capture, used by D3 to verify RobotSpec::validate()'s
// non-fatal pushing-margin WARNING (printed directly via std::cerr -- see
// RobotSpec.cpp) without depending on any return-value channel for it.
// Restores std::cerr's original streambuf even if a check below returns
// early or throws.
// ---------------------------------------------------------------------
class CerrCapture {
   public:
    CerrCapture() : old_buf_(std::cerr.rdbuf(buf_.rdbuf())) {}
    ~CerrCapture() { std::cerr.rdbuf(old_buf_); }
    std::string str() const { return buf_.str(); }

   private:
    std::ostringstream buf_;
    std::streambuf* old_buf_;
};

// ---------------------------------------------------------------------
// D3. RobotSpec deadband fields: defaults, JSON load/override (and no
// spurious "unknown key" warning), validate()'s new hard check
// (0 <= min_sustain_speed <= min_moving_speed) including the equal-bounds
// edge, and the independent non-fatal pushing-margin stderr WARNING
// (min_moving_speed >= speed_transfer) -- both when it should and should not
// fire, and that it still fires even alongside an unrelated fatal error.
// ---------------------------------------------------------------------
bool test_robot_spec_deadband_fields_and_validate() {
    bool ok = true;

    // Defaults: an IDEAL plant (0.0/0.0 -- no deadband, launch governor
    // inactive) is the compiled default; launch_margin (inert whenever
    // min_moving_speed is 0) still defaults to 0.1. Real hardware's 0.1/0.1
    // lives in config/robot_spec.json as an explicit --robot-spec override,
    // not in the compiled defaults -- see RobotSpec.h's doc comment.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        if (!near_eq(r.min_moving_speed, 0.0, 1e-9) || !near_eq(r.min_sustain_speed, 0.0, 1e-9) ||
            !near_eq(r.launch_margin, 0.1, 1e-9)) {
            std::cerr << "    defaults mismatch: min_moving_speed=" << r.min_moving_speed
                       << " min_sustain_speed=" << r.min_sustain_speed
                       << " launch_margin=" << r.launch_margin << " (expected 0.0/0.0/0.1)\n";
            ok = false;
        }
        // BREAKAWAY REALISM: max_breakaway_accel defaults to 3.0 regardless
        // of the ideal-plant deadband defaults above (it is inert whenever
        // --deadband/min_moving_speed is off, exactly like launch_margin).
        if (!near_eq(r.max_breakaway_accel, 3.0, 1e-9)) {
            std::cerr << "    max_breakaway_accel default mismatch: got " << r.max_breakaway_accel
                       << " (expected 3.0)\n";
            ok = false;
        }
        // Default (ideal-plant) spec must validate cleanly and print no
        // warning (0.0 < speed_transfer=0.15).
        CerrCapture cap;
        const std::string err = r.validate();
        if (!err.empty()) {
            std::cerr << "    default spec (incl. new fields) unexpectedly failed validate(): "
                       << err << "\n";
            ok = false;
        }
        if (cap.str().find("WARNING") != std::string::npos) {
            std::cerr << "    default spec unexpectedly printed a WARNING: " << cap.str() << "\n";
            ok = false;
        }
    }

    // config/robot_spec.json (the hardware truth, loaded exactly the way
    // mpc_controller/mpc_robot_sim do via --robot-spec=) must still resolve
    // to 0.1/0.1/0.1 -- i.e. this file is what OPTS a controller INTO real-
    // hardware deadband/governor behavior, on top of the ideal-plant
    // compiled defaults just checked above. Located relative to THIS source
    // file's own compiled-in path (__FILE__), not a hardcoded absolute
    // path, so this test is portable across checkouts (see this repo's own
    // CLAUDE.md: it has already moved machines once).
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        mpc::MpcParams p = mpc::MpcParams::defaults();
        namespace fs = std::filesystem;
        const fs::path config_path =
            fs::path(__FILE__).parent_path().parent_path() / "config" / "robot_spec.json";
        try {
            mpc::load_spec_from_json(config_path.string(), r, p);
        } catch (const std::exception& ex) {
            std::cerr << "    unexpected throw loading config/robot_spec.json: " << ex.what()
                       << " (path=" << config_path.string() << ")\n";
            ok = false;
        }
        if (!near_eq(r.min_moving_speed, 0.1, 1e-9) || !near_eq(r.min_sustain_speed, 0.1, 1e-9) ||
            !near_eq(r.launch_margin, 0.1, 1e-9)) {
            std::cerr << "    config/robot_spec.json should carry 0.1/0.1/0.1 as the hardware "
                          "truth, got min_moving_speed=" << r.min_moving_speed
                       << " min_sustain_speed=" << r.min_sustain_speed
                       << " launch_margin=" << r.launch_margin << "\n";
            ok = false;
        }
    }

    // JSON partial override of just the 3 new fields -- other fields keep
    // defaults, and no "unknown key" warning is printed for them.
    {
        namespace fs = std::filesystem;
        const fs::path path = fs::temp_directory_path() / "mpc_unit_test_robot_spec_deadband.json";
        {
            std::ofstream out(path);
            out << R"({
              "robot": { "min_moving_speed": 0.22, "min_sustain_speed": 0.07, "launch_margin": 0.3,
                         "max_breakaway_accel": 4.5 }
            })";
        }
        mpc::RobotSpec robot = mpc::RobotSpec::defaults();
        mpc::MpcParams params = mpc::MpcParams::defaults();
        CerrCapture cap;
        try {
            mpc::load_spec_from_json(path.string(), robot, params);
        } catch (const std::exception& ex) {
            std::cerr << "    unexpected throw loading deadband-field JSON: " << ex.what() << "\n";
            ok = false;
        }
        if (!near_eq(robot.min_moving_speed, 0.22, 1e-9) ||
            !near_eq(robot.min_sustain_speed, 0.07, 1e-9) ||
            !near_eq(robot.launch_margin, 0.3, 1e-9) ||
            !near_eq(robot.max_breakaway_accel, 4.5, 1e-9)) {
            std::cerr << "    deadband field override failed: min_moving_speed="
                       << robot.min_moving_speed << " min_sustain_speed=" << robot.min_sustain_speed
                       << " launch_margin=" << robot.launch_margin
                       << " max_breakaway_accel=" << robot.max_breakaway_accel
                       << " (expected 0.22/0.07/0.3/4.5)\n";
            ok = false;
        }
        if (!near_eq(robot.max_accel, 0.73, 1e-9)) {
            std::cerr << "    untouched field max_accel should keep its default, got "
                       << robot.max_accel << "\n";
            ok = false;
        }
        if (cap.str().find("unknown key") != std::string::npos) {
            std::cerr << "    loading the 3 new fields should not warn 'unknown key': "
                       << cap.str() << "\n";
            ok = false;
        }
        std::error_code ec;
        fs::remove(path, ec);
    }

    // validate(): min_sustain_speed > min_moving_speed -> fatal error
    // mentioning min_sustain_speed.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.min_moving_speed = 0.1;
        r.min_sustain_speed = 0.2;
        const std::string err = r.validate();
        if (err.empty() || err.find("min_sustain_speed") == std::string::npos) {
            std::cerr << "    expected a min_sustain_speed error for min_sustain_speed(0.2) > "
                          "min_moving_speed(0.1), got: '" << err << "'\n";
            ok = false;
        }
    }

    // validate(): negative min_sustain_speed -> fatal error.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.min_moving_speed = 0.1;
        r.min_sustain_speed = -0.01;
        const std::string err = r.validate();
        if (err.empty()) {
            std::cerr << "    expected an error for negative min_sustain_speed\n";
            ok = false;
        }
    }

    // validate(): equal bounds (min_sustain_speed == min_moving_speed) is
    // the inclusive edge of "0 <= min_sustain_speed <= min_moving_speed" and
    // must pass.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.min_moving_speed = 0.12;
        r.min_sustain_speed = 0.12;
        const std::string err = r.validate();
        if (!err.empty()) {
            std::cerr << "    equal min_sustain_speed==min_moving_speed should be valid, got: '"
                       << err << "'\n";
            ok = false;
        }
    }

    // validate(): max_breakaway_accel < max_accel -> fatal error mentioning
    // max_breakaway_accel (see RobotSpec::max_breakaway_accel's doc
    // comment: a breakaway ceiling below the nominal actuator ceiling is
    // not headroom at all).
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.max_breakaway_accel = r.max_accel - 0.01;
        const std::string err = r.validate();
        if (err.empty() || err.find("max_breakaway_accel") == std::string::npos) {
            std::cerr << "    expected a max_breakaway_accel error for max_breakaway_accel("
                       << r.max_breakaway_accel << ") < max_accel(" << r.max_accel << "), got: '"
                       << err << "'\n";
            ok = false;
        }
    }

    // validate(): equal bounds (max_breakaway_accel == max_accel) is the
    // inclusive edge of ">= max_accel" and must pass.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.max_breakaway_accel = r.max_accel;
        const std::string err = r.validate();
        if (!err.empty()) {
            std::cerr << "    equal max_breakaway_accel==max_accel should be valid, got: '" << err
                       << "'\n";
            ok = false;
        }
    }

    // Pushing-margin WARNING: min_moving_speed >= speed_transfer -> validate()
    // still returns "" (non-fatal) but prints a WARNING to stderr.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.min_moving_speed = r.speed_transfer;  // exactly at the >= boundary.
        r.min_sustain_speed = 0.05;
        CerrCapture cap;
        const std::string err = r.validate();
        if (!err.empty()) {
            std::cerr << "    pushing-margin hazard must be NON-FATAL, but validate() returned: '"
                       << err << "'\n";
            ok = false;
        }
        if (cap.str().find("WARNING") == std::string::npos) {
            std::cerr << "    expected a stderr WARNING for min_moving_speed(" << r.min_moving_speed
                       << ") >= speed_transfer(" << r.speed_transfer << "), got: '" << cap.str()
                       << "'\n";
            ok = false;
        }
    }

    // The warning fires independently of (i.e. even alongside) an unrelated
    // fatal problem -- proving it is not short-circuited by the earlier
    // turning-radius check's early return.
    {
        mpc::RobotSpec r = mpc::RobotSpec::defaults();
        r.min_turning_radius_transit = 0.5;  // fatal: implies steer > max_steer (see test_validate()).
        r.min_moving_speed = 0.5;            // also >= speed_transfer(0.15): hazard.
        CerrCapture cap;
        const std::string err = r.validate();
        if (err.empty()) {
            std::cerr << "    expected the turning-radius error to still be reported\n";
            ok = false;
        }
        if (cap.str().find("WARNING") == std::string::npos) {
            std::cerr << "    expected the pushing-margin WARNING even alongside an unrelated "
                          "fatal error, got: '" << cap.str() << "'\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// D4. Telemetry encode/parse round-trip, incl. moving/watchdog flags. Wire
// format check that moving/watchdog encode as bare JSON integers 0/1 (per
// the design spec), not JSON booleans; malformed inputs never throw and
// report ok=false, mirroring decode_ackermann_payload()'s contract.
// ---------------------------------------------------------------------
bool test_telemetry_encode_parse_roundtrip() {
    bool ok = true;

    struct Case {
        double t, v, v_cmd, steering, accel;
        bool moving, watchdog;
    };
    const std::vector<Case> cases = {
        {0.0, 0.0, 0.0, 0.0, 0.0, false, false},
        {12.345, 0.2, 0.35, -0.1, 0.5, true, false},
        {1.0, 0.0, 0.6, 0.05, -0.3, false, true},
        {5.5, -0.15, -0.15, 0.33, -0.73, true, true},
    };

    for (const auto& c : cases) {
        mpc::TelemetrySample s;
        s.t = c.t;
        s.v = c.v;
        s.v_cmd = c.v_cmd;
        s.steering = c.steering;
        s.accel = c.accel;
        s.moving = c.moving;
        s.watchdog = c.watchdog;

        const std::string payload = mpc::encode_telemetry_payload(s);

        nlohmann::json j;
        try {
            j = nlohmann::json::parse(payload);
        } catch (const std::exception& ex) {
            std::cerr << "    encode_telemetry_payload produced invalid JSON: " << ex.what()
                       << "\n";
            ok = false;
            continue;
        }
        if (!j.contains("moving") || !j.at("moving").is_number_integer() ||
            !j.contains("watchdog") || !j.at("watchdog").is_number_integer()) {
            std::cerr << "    moving/watchdog must be encoded as JSON integers (0/1), got: "
                       << payload << "\n";
            ok = false;
        } else {
            if (j.at("moving").get<int>() != (c.moving ? 1 : 0) ||
                j.at("watchdog").get<int>() != (c.watchdog ? 1 : 0)) {
                std::cerr << "    moving/watchdog integer value mismatch in: " << payload << "\n";
                ok = false;
            }
        }

        mpc::TelemetryParseResult parsed = mpc::parse_telemetry_payload(payload);
        if (!parsed.ok || !near_eq(parsed.sample.t, c.t, 1e-9) ||
            !near_eq(parsed.sample.v, c.v, 1e-9) || !near_eq(parsed.sample.v_cmd, c.v_cmd, 1e-9) ||
            !near_eq(parsed.sample.steering, c.steering, 1e-9) ||
            !near_eq(parsed.sample.accel, c.accel, 1e-9) || parsed.sample.moving != c.moving ||
            parsed.sample.watchdog != c.watchdog) {
            std::cerr << "    round-trip mismatch for payload: " << payload << " -> ok="
                       << parsed.ok << " t=" << parsed.sample.t << " v=" << parsed.sample.v
                       << " v_cmd=" << parsed.sample.v_cmd << " steering=" << parsed.sample.steering
                       << " accel=" << parsed.sample.accel << " moving=" << parsed.sample.moving
                       << " watchdog=" << parsed.sample.watchdog << "\n";
            ok = false;
        }
    }

    // Malformed inputs must never throw and must report ok=false.
    const std::vector<std::string> bad_payloads = {
        "",                                     // empty frame
        "not json at all",                      // malformed / non-JSON
        "{ this is not valid json",             // malformed JSON, starts with '{'
        R"({"t":1.0,"v":0.1})",                 // missing several required keys
        R"({"t":"not_a_number","v":0.1,"v_cmd":0.1,"steering":0,"accel":0,"moving":0,"watchdog":0})",
    };
    for (const auto& p : bad_payloads) {
        mpc::TelemetryParseResult parsed;
        try {
            parsed = mpc::parse_telemetry_payload(p);
        } catch (const std::exception& ex) {
            std::cerr << "    parse_telemetry_payload threw on malformed payload '" << p
                       << "': " << ex.what() << "\n";
            ok = false;
            continue;
        }
        if (parsed.ok) {
            std::cerr << "    expected ok=false for malformed payload: '" << p << "'\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// FEATURE B2 (live deadband reconfiguration via "/<robot>/sim_config") test
// helpers/tests.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// D5a. sim_config deadband fields: all-new-fields-together, deadband:0/1,
// partial (only one field touches, others retained), malformed-per-field
// (independent of other fields in the SAME payload succeeding), and
// clamping -- min_moving_speed to [0,0.5], min_sustain_speed to
// [0, APPLIED min_moving_speed] in BOTH the "only min_sustain_speed present"
// (clamps against the RETAINED min_moving_speed) and "both present together"
// (clamps against the NEW, same-payload min_moving_speed, not the stale
// retained one) forms. Also the pre-existing noise-field malformed-warning
// signal (*_bad) and the fully-malformed-payload retain-everything case.
// ---------------------------------------------------------------------
bool test_sim_config_deadband_fields_parsing() {
    bool ok = true;

    // All 5 fields present together, valid -- independent per-field parsing
    // within ONE payload.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"noise_sigma_pct":0.1,"steer_noise_sigma_pct":0.05,"deadband":1,)"
            R"("min_moving_speed":0.2,"min_sustain_speed":0.08})",
            /*current_accel=*/0.0, /*current_steer=*/0.0,
            /*current_deadband_enabled=*/false, /*current_min_moving=*/0.1,
            /*current_min_sustain=*/0.1);
        if (!r.accel_ok || !near_eq(r.accel_sigma_pct, 0.1, 1e-9)) {
            std::cerr << "    all-fields payload: accel channel failed to parse\n";
            ok = false;
        }
        if (!r.steer_ok || !near_eq(r.steer_sigma_pct, 0.05, 1e-9)) {
            std::cerr << "    all-fields payload: steer channel failed to parse\n";
            ok = false;
        }
        if (!r.deadband_ok || !r.deadband_enabled) {
            std::cerr << "    all-fields payload: deadband:1 should resolve to enabled=true\n";
            ok = false;
        }
        if (!r.min_moving_speed_ok || !near_eq(r.min_moving_speed, 0.2, 1e-9)) {
            std::cerr << "    all-fields payload: min_moving_speed failed to parse\n";
            ok = false;
        }
        if (!r.min_sustain_speed_ok || !near_eq(r.min_sustain_speed, 0.08, 1e-9)) {
            std::cerr << "    all-fields payload: min_sustain_speed failed to parse\n";
            ok = false;
        }
        if (!r.ok) {
            std::cerr << "    all-fields payload: overall ok should be true\n";
            ok = false;
        }
    }

    // "deadband":0 -> disabled; proves it is not just "presence == enable".
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"deadband":0})", 0.0, 0.0, /*current_deadband_enabled=*/true, 0.1, 0.1);
        if (!r.deadband_ok || r.deadband_enabled) {
            std::cerr << "    'deadband':0 should resolve to enabled=false, got ok=" << r.deadband_ok
                       << " enabled=" << r.deadband_enabled << "\n";
            ok = false;
        }
    }

    // Partial: ONLY "deadband" present -- noise fields AND the two threshold
    // fields all retained at their current values, untouched.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"deadband":1})", /*current_accel=*/0.12, /*current_steer=*/0.07,
            /*current_deadband_enabled=*/false, /*current_min_moving=*/0.15,
            /*current_min_sustain=*/0.09);
        if (!r.deadband_ok || !r.deadband_enabled) {
            std::cerr << "    partial payload: deadband should have parsed and enabled\n";
            ok = false;
        }
        if (r.accel_ok || !near_eq(r.accel_sigma_pct, 0.12, 1e-9)) {
            std::cerr << "    partial payload: accel channel should be retained untouched\n";
            ok = false;
        }
        if (r.steer_ok || !near_eq(r.steer_sigma_pct, 0.07, 1e-9)) {
            std::cerr << "    partial payload: steer channel should be retained untouched\n";
            ok = false;
        }
        if (r.min_moving_speed_ok || !near_eq(r.min_moving_speed, 0.15, 1e-9)) {
            std::cerr << "    partial payload: min_moving_speed should be retained untouched\n";
            ok = false;
        }
        if (r.min_sustain_speed_ok || !near_eq(r.min_sustain_speed, 0.09, 1e-9)) {
            std::cerr << "    partial payload: min_sustain_speed should be retained untouched\n";
            ok = false;
        }
    }

    // Malformed per-field, independent of valid fields in the SAME payload:
    // wrong-typed "deadband" alongside valid min_moving_speed/min_sustain_speed.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"deadband":"not_a_number","min_moving_speed":0.3,"min_sustain_speed":0.1})", 0.0,
            0.0, /*current_deadband_enabled=*/true, /*current_min_moving=*/0.1,
            /*current_min_sustain=*/0.1);
        if (!r.deadband_bad || r.deadband_ok || r.deadband_enabled != true) {
            std::cerr << "    wrong-typed deadband: expected bad=true, ok=false, retained=true, "
                          "got bad=" << r.deadband_bad << " ok=" << r.deadband_ok << " enabled="
                       << r.deadband_enabled << "\n";
            ok = false;
        }
        if (!r.min_moving_speed_ok || !near_eq(r.min_moving_speed, 0.3, 1e-9)) {
            std::cerr << "    wrong-typed deadband should not disturb a valid min_moving_speed in "
                          "the same payload\n";
            ok = false;
        }
        if (!r.min_sustain_speed_ok || !near_eq(r.min_sustain_speed, 0.1, 1e-9)) {
            std::cerr << "    wrong-typed deadband should not disturb a valid min_sustain_speed in "
                          "the same payload\n";
            ok = false;
        }
    }

    // Malformed per-field: wrong-typed "min_moving_speed" alongside a valid
    // "deadband".
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"min_moving_speed":"nope","deadband":1})", 0.0, 0.0, false,
            /*current_min_moving=*/0.22, /*current_min_sustain=*/0.05);
        if (!r.min_moving_speed_bad || r.min_moving_speed_ok ||
            !near_eq(r.min_moving_speed, 0.22, 1e-9)) {
            std::cerr << "    wrong-typed min_moving_speed: expected bad=true, retained=0.22, got "
                          "bad=" << r.min_moving_speed_bad << " value=" << r.min_moving_speed << "\n";
            ok = false;
        }
        if (!r.deadband_ok || !r.deadband_enabled) {
            std::cerr << "    wrong-typed min_moving_speed should not disturb a valid deadband in "
                          "the same payload\n";
            ok = false;
        }
    }

    // Malformed per-field: wrong-typed "min_sustain_speed" (a JSON array).
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"min_sustain_speed":[1,2,3]})", 0.0, 0.0, false, 0.2, 0.05);
        if (!r.min_sustain_speed_bad || r.min_sustain_speed_ok ||
            !near_eq(r.min_sustain_speed, 0.05, 1e-9)) {
            std::cerr << "    wrong-typed min_sustain_speed: expected bad=true, retained=0.05, got "
                          "bad=" << r.min_sustain_speed_bad << " value=" << r.min_sustain_speed
                       << "\n";
            ok = false;
        }
    }

    // Clamping: min_moving_speed to [0, 0.5], both directions.
    {
        mpc::SimConfigParseResult hi =
            mpc::parse_sim_config_payload(R"({"min_moving_speed":0.9})", 0.0, 0.0, false, 0.1, 0.1);
        if (!hi.min_moving_speed_ok || !hi.min_moving_speed_was_clamped ||
            !near_eq(hi.min_moving_speed, 0.5, 1e-9)) {
            std::cerr << "    min_moving_speed=0.9 should clamp to 0.5, got " << hi.min_moving_speed
                       << " (was_clamped=" << hi.min_moving_speed_was_clamped << ")\n";
            ok = false;
        }

        mpc::SimConfigParseResult lo =
            mpc::parse_sim_config_payload(R"({"min_moving_speed":-0.2})", 0.0, 0.0, false, 0.1, 0.1);
        if (!lo.min_moving_speed_ok || !lo.min_moving_speed_was_clamped ||
            !near_eq(lo.min_moving_speed, 0.0, 1e-9)) {
            std::cerr << "    min_moving_speed=-0.2 should clamp to 0.0, got " << lo.min_moving_speed
                       << " (was_clamped=" << lo.min_moving_speed_was_clamped << ")\n";
            ok = false;
        }
    }

    // Clamping: min_sustain_speed to [0, APPLIED min_moving_speed] -- case
    // (a): only min_sustain_speed present, clamps against the RETAINED
    // (current) min_moving_speed.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"min_sustain_speed":0.5})", 0.0, 0.0, false, /*current_min_moving=*/0.2,
            /*current_min_sustain=*/0.05);
        if (!r.min_sustain_speed_ok || !r.min_sustain_speed_was_clamped ||
            !near_eq(r.min_sustain_speed, 0.2, 1e-9)) {
            std::cerr << "    min_sustain_speed=0.5 (retained min_moving_speed=0.2) should clamp to "
                          "0.2, got " << r.min_sustain_speed << "\n";
            ok = false;
        }
    }

    // Clamping: case (b) -- BOTH fields present in the SAME payload;
    // min_sustain_speed must clamp against the NEW (this-payload)
    // min_moving_speed, NOT the stale retained one.
    {
        mpc::SimConfigParseResult r = mpc::parse_sim_config_payload(
            R"({"min_moving_speed":0.15,"min_sustain_speed":0.4})", 0.0, 0.0, false,
            /*current_min_moving=*/0.3, /*current_min_sustain=*/0.05);
        if (!r.min_moving_speed_ok || !near_eq(r.min_moving_speed, 0.15, 1e-9)) {
            std::cerr << "    same-payload case: min_moving_speed should resolve to 0.15\n";
            ok = false;
        }
        if (!r.min_sustain_speed_ok || !r.min_sustain_speed_was_clamped ||
            !near_eq(r.min_sustain_speed, 0.15, 1e-9)) {
            std::cerr << "    same-payload case: min_sustain_speed=0.4 should clamp to the NEW "
                          "min_moving_speed=0.15 (not the stale retained 0.3), got "
                       << r.min_sustain_speed << "\n";
            ok = false;
        }
    }

    // Negative min_sustain_speed clamps to 0.
    {
        mpc::SimConfigParseResult r =
            mpc::parse_sim_config_payload(R"({"min_sustain_speed":-0.1})", 0.0, 0.0, false, 0.2, 0.1);
        if (!r.min_sustain_speed_ok || !r.min_sustain_speed_was_clamped ||
            !near_eq(r.min_sustain_speed, 0.0, 1e-9)) {
            std::cerr << "    min_sustain_speed=-0.1 should clamp to 0.0, got "
                       << r.min_sustain_speed << "\n";
            ok = false;
        }
    }

    // Structurally malformed payload -> ALL fields retained, ok=false,
    // never throws.
    {
        mpc::SimConfigParseResult r;
        const std::string payload = "{ not valid json";
        try {
            r = mpc::parse_sim_config_payload(payload, /*current_accel=*/0.11,
                                               /*current_steer=*/0.22,
                                               /*current_deadband_enabled=*/true,
                                               /*current_min_moving=*/0.33,
                                               /*current_min_sustain=*/0.05);
        } catch (const std::exception& ex) {
            std::cerr << "    parse_sim_config_payload threw on malformed payload: " << ex.what()
                       << "\n";
            ok = false;
        }
        if (r.ok || r.accel_ok || r.steer_ok || r.deadband_ok || r.min_moving_speed_ok ||
            r.min_sustain_speed_ok || r.accel_bad || r.steer_bad || r.deadband_bad ||
            r.min_moving_speed_bad || r.min_sustain_speed_bad) {
            std::cerr << "    structurally malformed payload should yield ok=false and every "
                          "*_ok/*_bad flag false\n";
            ok = false;
        }
        if (!near_eq(r.accel_sigma_pct, 0.11, 1e-12) || !near_eq(r.steer_sigma_pct, 0.22, 1e-12) ||
            r.deadband_enabled != true || !near_eq(r.min_moving_speed, 0.33, 1e-12) ||
            !near_eq(r.min_sustain_speed, 0.05, 1e-12)) {
            std::cerr << "    structurally malformed payload should retain every field's current "
                          "value unchanged\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// D5b. Live enable/disable state-machine cases, at the pure-function level,
// exercising the SAME primitives (mpc::step_deadband/mpc::rollout_step)
// robot_sim.cpp's tick loop calls -- since a live sim_config toggle only
// ever changes WHICH branch/thresholds the tick loop consults next tick
// (see robot_sim.cpp's deadband_enabled doc comment: "no state reset"),
// these primitives fully characterize the live-toggle behavior without
// needing a process-level test.
// ---------------------------------------------------------------------
bool test_sim_config_deadband_live_toggle_state_machine() {
    bool ok = true;

    // Live enable while already moving: "was_moving=true" is exactly what
    // robot_sim.cpp's deadband_moving carries into the first post-enable
    // tick (it defaults true and is untouched while disabled -- see
    // robot_sim.cpp). Must KEEP moving=true, driven by the full v, even at a
    // v BELOW min_moving_speed but AT/ABOVE min_sustain_speed -- i.e. it
    // consults ONLY the sustain threshold on this transition, never
    // re-derives via the (higher) breakaway threshold.
    {
        const double moving_thresh = 0.2;
        const double sustain_thresh = 0.05;
        const double v_in_band = 0.1;  // < moving_thresh, >= sustain_thresh.

        mpc::DeadbandStepResult enabled_while_moving =
            mpc::step_deadband(v_in_band, /*was_moving=*/true, moving_thresh, sustain_thresh);
        if (!enabled_while_moving.moving || enabled_while_moving.effective_v != v_in_band) {
            std::cerr << "    live enable while moving (v=" << v_in_band
                       << ", in the hysteresis band) should keep moving=true with effective_v==v, "
                          "got moving=" << enabled_while_moving.moving << " effective_v="
                       << enabled_while_moving.effective_v << "\n";
            ok = false;
        }

        // Vacuousness guard: the SAME v from a genuine rest state (was_moving
        // =false) must NOT break away -- proving this is really exercising
        // the "inherited moving=true" path and not just a threshold that
        // happens to always pass.
        mpc::DeadbandStepResult from_rest =
            mpc::step_deadband(v_in_band, /*was_moving=*/false, moving_thresh, sustain_thresh);
        if (from_rest.moving) {
            std::cerr << "    vacuousness guard failed: v=" << v_in_band
                       << " should NOT break away from genuine rest (it's below min_moving_speed="
                       << moving_thresh << ")\n";
            ok = false;
        }
    }

    // Live disable while stalled releases the pose-freeze on the very next
    // tick. While enabled+stalled, step_deadband pins effective_v=0 even as
    // v_cmd_equiv keeps winding up from a nonzero accel; the disabled
    // pipeline (robot_sim.cpp calls mpc::rollout_step directly on `state`,
    // bypassing step_deadband/integrate_pose_with_v entirely) immediately
    // lets v (and pose) respond to accel again, with NO special-cased
    // "release" step -- it is simply what rollout_step already does when fed
    // the (currently zero) state.v.
    {
        const double moving_thresh = 0.2, sustain_thresh = 0.05, L = 0.29, dt = 0.05, accel = 0.3;
        mpc::State4 state;  // stalled at rest, v=0.
        double v_cmd_equiv = 0.0;
        bool moving = false;

        // A few enabled ticks while stalled: v_cmd_equiv winds up but pose
        // and reported v stay frozen (breakaway not yet reached).
        for (int i = 0; i < 3; ++i) {
            v_cmd_equiv += accel * dt;
            mpc::DeadbandStepResult r =
                mpc::step_deadband(v_cmd_equiv, moving, moving_thresh, sustain_thresh);
            moving = r.moving;
            state = mpc::integrate_pose_with_v(state, r.effective_v, /*delta=*/0.0, L, dt);
        }
        if (moving || state.v != 0.0 || state.x != 0.0) {
            std::cerr << "    setup invariant violated: still-enabled robot should be stalled and "
                          "frozen before the live disable (moving=" << moving << " v=" << state.v
                       << " x=" << state.x << ", v_cmd_equiv=" << v_cmd_equiv << ")\n";
            ok = false;
        }

        // Live disable: the VERY NEXT tick uses the normal (non-deadband)
        // pipeline directly on `state` (still v=0 from the freeze), exactly
        // as robot_sim.cpp's disabled branch does.
        mpc::State4 next = mpc::rollout_step(state, accel, /*delta=*/0.0, L, dt);
        if (!(next.v > 0.0) || !(next.x > 0.0)) {
            std::cerr << "    live disable while stalled should release the freeze on the very "
                          "next tick (pose/velocity should respond to accel immediately), got v="
                       << next.v << " x=" << next.x << "\n";
            ok = false;
        }
        const double expected_v = 0.0 + accel * dt;  // rollout_step's own v-update formula.
        if (!near_eq(next.v, expected_v, 1e-12)) {
            std::cerr << "    post-disable v should follow the plain rollout_step formula exactly, "
                          "expected " << expected_v << " got " << next.v << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// D6. BREAKAWAY REALISM: mpc::resolve_accel_clamp_ceiling (see SimCore.h /
// RobotSpec::max_breakaway_accel) -- the accel ceiling robot_sim.cpp's two
// received-command clamp sites (initial clamp_command + FEATURE A's
// apply_command_noise re-clamp) use. Ceiling is max_breakaway_accel ONLY
// while deadband is enabled AND the plant is currently STALLED (not
// moving); every other combination (deadband off regardless of moving
// state; deadband on but already moving) resolves to the nominal max_accel
// unchanged -- a pure no-op (bitwise max_accel) whenever deadband_enabled
// is false, preserving mpc_robot_sim's bit-identical-when-off invariant.
// ---------------------------------------------------------------------
bool test_breakaway_accel_clamp_ceiling() {
    bool ok = true;
    const double max_accel = 0.73;
    const double max_breakaway_accel = 3.0;

    struct Case {
        bool deadband_enabled;
        bool currently_moving;
        double expected;
        const char* label;
    };
    const std::vector<Case> cases = {
        {false, false, max_accel, "deadband off, stalled -- no-op (off wins)"},
        {false, true, max_accel, "deadband off, moving -- no-op (off wins)"},
        {true, false, max_breakaway_accel, "deadband on, STALLED -- breakaway ceiling"},
        {true, true, max_accel, "deadband on, moving -- nominal ceiling"},
    };
    for (const auto& c : cases) {
        const double got = mpc::resolve_accel_clamp_ceiling(c.deadband_enabled, c.currently_moving,
                                                              max_accel, max_breakaway_accel);
        if (!near_eq(got, c.expected, 1e-12)) {
            std::cerr << "    [" << c.label << "] expected " << c.expected << ", got " << got
                       << "\n";
            ok = false;
        }
    }

    // The deadband-off cases are bitwise identical to max_accel (not just
    // numerically close), matching the doc comment's "a pure no-op (returns
    // max_accel exactly)" contract.
    if (mpc::resolve_accel_clamp_ceiling(false, false, max_accel, max_breakaway_accel) !=
            max_accel ||
        mpc::resolve_accel_clamp_ceiling(false, true, max_accel, max_breakaway_accel) !=
            max_accel) {
        std::cerr << "    deadband-off no-op must return max_accel bitwise-exactly\n";
        ok = false;
    }

    // End-to-end sanity against a real RobotSpec: compiled default
    // max_breakaway_accel(3.0) is comfortably above compiled default
    // max_accel(0.73), so a stalled+deadband-on caller genuinely gets more
    // headroom, not an accidentally-inert clamp.
    {
        const mpc::RobotSpec spec = mpc::RobotSpec::defaults();
        const double ceiling =
            mpc::resolve_accel_clamp_ceiling(true, false, spec.max_accel, spec.max_breakaway_accel);
        if (!(ceiling > spec.max_accel)) {
            std::cerr << "    RobotSpec::defaults(): stalled+deadband-on ceiling(" << ceiling
                       << ") should exceed max_accel(" << spec.max_accel << ")\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// mpc_controller LAUNCH GOVERNOR (mpc::LaunchGovernor) test helpers/tests.
// ---------------------------------------------------------------------

// ---------------------------------------------------------------------
// G1. State transitions, pure-function level, all against
// RobotSpec::defaults() (min_moving_speed=min_sustain_speed=0.1,
// launch_margin=0.1 -> base launch speed 0.11): LAUNCH entry with correct
// magnitude/sign (both directions), exit at the 0.7 threshold (both sides of
// the boundary), a reversal crossing (exit then re-entry with the flipped
// sign), STOP-SNAP when the reference wants zero (both a genuine decel and
// the steady-state-idle case), and NO launch when the MPC's own v_des is
// already at/above breakaway.
// ---------------------------------------------------------------------
bool test_launch_governor_state_transitions() {
    bool ok = true;
    // Compiled default is now an ideal plant (min_moving_speed=
    // min_sustain_speed=0.0 -- see RobotSpec.h); explicitly set both to 0.1
    // here, as if a --robot-spec override for real hardware had been
    // applied (matching config/robot_spec.json's own 0.1/0.1).
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.1;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_delta = spec.max_accel * dt;                           // 0.73*0.05 = 0.0365
    const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);  // 0.11

    // -- Entry, positive direction: a single-tick KICK straight to base (NOT
    // a rate-limited ramp -- see LaunchGovernor.h's doc comment).
    // derive_accel_from_rest=true means the caller derives accel from 0.0
    // regardless of what last_published_v actually was (SUSTAINED kick --
    // see LaunchGovernorDecision's doc comment); last_published_v=0.0 here
    // is simply what a caller starting from a genuine rest would have. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/0.0, /*v_des=*/0.05,
                                                   /*ref_vel_now=*/0.05, /*last_published_v=*/0.0,
                                                   spec, dt, /*wall_elapsed_in_state=*/0.0);
        if (!d.override_active || gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    positive entry: expected override_active + kLaunch, got "
                          "override_active=" << d.override_active << " state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
        if (!near_eq(d.v_published, base, 1e-9)) {
            std::cerr << "    positive entry: expected a single-tick KICK straight to base="
                       << base << " (not a rate-limited max_accel*dt=" << max_delta
                       << " first step -- the OLD ramp design), got " << d.v_published << "\n";
            ok = false;
        }
        if (!d.derive_accel_from_rest) {
            std::cerr << "    positive entry: expected derive_accel_from_rest=true (SUSTAINED "
                          "kick -- see LaunchGovernorDecision's doc comment)\n";
            ok = false;
        }
        // The implied accel is the FULL one-tick step, deliberately allowed
        // to exceed spec.max_accel (see LaunchGovernorDecision's doc
        // comment / RobotSpec::max_breakaway_accel) -- base/dt = 0.11/0.05 =
        // 2.2, well above max_accel=0.73.
        const double implied_accel = (d.v_published - 0.0) / dt;
        if (implied_accel <= spec.max_accel) {
            std::cerr << "    positive entry: expected the KICK's implied accel (" << implied_accel
                       << ") to exceed spec.max_accel(" << spec.max_accel << ") -- that's the "
                          "whole point of the kick\n";
            ok = false;
        }
    }

    // -- Entry, negative direction: sign(v_des) is honored. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(0.0, /*v_des=*/-0.05, -0.05, 0.0, spec, dt, 0.0);
        if (!d.override_active || !near_eq(d.v_published, -base, 1e-9) ||
            !d.derive_accel_from_rest) {
            std::cerr << "    negative entry: expected v_published==" << -base
                       << " (KICK) with derive_accel_from_rest=true, got " << d.v_published
                       << " (override_active=" << d.override_active
                       << " derive_accel_from_rest=" << d.derive_accel_from_rest << ")\n";
            ok = false;
        }
    }

    // -- The KICK reaches base on the VERY FIRST tick (see G2 for the
    // exhaustive per-tick accel/hold check; this is a coarser end-to-end
    // sanity check within THIS test's own state-transition narrative). --
    double last_v = 0.0;
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(0.0, 0.05, 0.05, last_v, spec, dt, 0.0);
        last_v = d.v_published;
        if (!near_eq(last_v, base, 1e-9)) {
            std::cerr << "    KICK entry tick: expected v_published==base=" << base
                       << " immediately (tick 1, not eased in over several ticks), got " << last_v
                       << "\n";
            ok = false;
        }
    }

    // -- Exit boundary: strictly below 0.7*min_moving stays LAUNCH; at/above
    // exits to NORMAL (transparent that same tick). Fresh gov/v (the KICK
    // reaches base in one tick regardless, so there is no ramp-in-progress
    // state left to continue from the block above). --
    {
        mpc::LaunchGovernor gov;
        double v = 0.0;
        v = gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.0).v_published;  // enter LAUNCH.
        mpc::LaunchGovernorDecision below = gov.step(/*v_meas=*/0.069, 0.05, 0.05, v, spec, dt, 0.1);
        if (!below.override_active || gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    v_meas=0.069 (< 0.7*0.1) should still be LAUNCH, got "
                          "override_active=" << below.override_active << " state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
        v = below.v_published;
        mpc::LaunchGovernorDecision at = gov.step(/*v_meas=*/0.07, 0.05, 0.05, v, spec, dt, 0.15);
        if (at.override_active || gov.state() != mpc::LaunchGovernorState::kNormal) {
            std::cerr << "    v_meas=0.07 (== 0.7*0.1) should exit to NORMAL (transparent), got "
                          "override_active=" << at.override_active << " state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
    }

    // -- Reversal crossing: exit, then a new at-rest+wants-motion-negative
    // tick re-enters LAUNCH with the flipped sign, KICKING straight to -base
    // regardless of what last_published_v was (the KICK does not ramp from
    // it -- see LaunchGovernor.h's doc comment; last_published_v only feeds
    // the caller's own accel derivation, which main.cpp/G1's own positive-
    // entry sub-test above already exercises). --
    {
        mpc::LaunchGovernor gov;
        double v = 0.0;
        v = gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.0).v_published;   // enter LAUNCH, direction +1.
        gov.step(0.07, 0.05, 0.05, v, spec, dt, 0.1);                  // exit to NORMAL (motion confirmed).
        mpc::LaunchGovernorDecision reversed =
            gov.step(/*v_meas=*/0.0, /*v_des=*/-0.05, -0.05, /*last_published_v=*/0.0, spec, dt,
                      0.0);
        if (!reversed.override_active || gov.state() != mpc::LaunchGovernorState::kLaunch ||
            !near_eq(reversed.v_published, -base, 1e-9)) {
            std::cerr << "    reversal crossing: expected re-entry into LAUNCH with a single-tick "
                          "KICK to v_published==" << -base << ", got override_active="
                       << reversed.override_active << " v_published=" << reversed.v_published
                       << " state=" << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
    }

    // -- STOP-SNAP: v_des below min_sustain_speed AND reference wants ~zero
    // -> RAMP toward exactly 0 (rate-limited, NOT an instant jump -- see
    // LaunchGovernor.h's doc comment for why: an instant jump silently
    // strands an un-shed residual on the plant whenever the correction
    // needed exceeds one tick's max_accel*dt, which is EXACTLY what this
    // sub-test's last_published_v=0.09 (> max_delta=0.0365) exercises), from
    // NORMAL, regardless of a nonzero v_meas residual (a genuine
    // decel-to-stop tick). --
    {
        mpc::LaunchGovernor gov;  // fresh, NORMAL by construction.
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/0.05, /*v_des=*/0.02,
                                                   /*ref_vel_now=*/0.0, /*last_published_v=*/0.09,
                                                   spec, dt, 0.0);
        if (!d.override_active) {
            std::cerr << "    stop-snap (decel, far from 0): expected override_active\n";
            ok = false;
        }
        if (!near_eq(d.v_published, 0.09 - max_delta, 1e-9)) {
            std::cerr << "    stop-snap (decel, far from 0): expected a rate-limited first step of "
                          "0.09-max_delta=" << (0.09 - max_delta) << " (NOT an instant jump to 0.0 "
                          "-- see LaunchGovernor.h), got " << d.v_published << "\n";
            ok = false;
        }
    }
    // Regression case for the exact bug this fix addresses (see this task's
    // report): starting FAR from 0 (last_published_v=0.3, needing several
    // ticks), the ramp must take MULTIPLE ticks -- each moving by EXACTLY
    // max_delta -- to actually reach 0, snapping there exactly once within
    // reach, and holding (accel==0) once there. If STOP-SNAP instead jumped
    // straight to 0.0 in one tick, the CALLER's own last_published_v
    // bookkeeping would believe the plant arrived immediately even though
    // an accel-clamped real actuator physically could not have.
    {
        mpc::LaunchGovernor gov;
        double v = 0.3;
        int ticks = 0;
        bool reached_zero = false;
        for (int i = 0; i < 20; ++i) {
            const double prev_v = v;
            mpc::LaunchGovernorDecision d = gov.step(0.0, 0.02, 0.0, prev_v, spec, dt, 0.0);
            if (!d.override_active) {
                std::cerr << "    stop-snap ramp regression: expected override_active at tick " << i
                           << "\n";
                ok = false;
                break;
            }
            const double accel = (d.v_published - prev_v) / dt;
            if (near_eq(prev_v, 0.0, 1e-9)) {
                if (!near_eq(d.v_published, 0.0, 1e-9) || !near_eq(accel, 0.0, 1e-6)) {
                    std::cerr << "    stop-snap ramp regression: expected to hold at exactly 0 with "
                                  "accel==0 once there, got v_published=" << d.v_published
                               << " accel=" << accel << " at tick " << i << "\n";
                    ok = false;
                }
                reached_zero = true;
                v = d.v_published;
                break;
            }
            if (!near_eq(d.v_published, 0.0, 1e-9) && !near_eq(accel, -spec.max_accel, 1e-6)) {
                std::cerr << "    stop-snap ramp regression: expected accel saturated at exactly "
                              "-max_accel=" << -spec.max_accel << " while still above 0, got "
                           << accel << " (v_published " << prev_v << " -> " << d.v_published
                           << ") at tick " << i << "\n";
                ok = false;
            }
            v = d.v_published;
            ++ticks;
        }
        if (!reached_zero) {
            std::cerr << "    stop-snap ramp regression: never reached exactly 0 within 20 ticks, "
                          "stuck at " << v << "\n";
            ok = false;
        }
        // 0.3 / 0.0365 ~= 8.2, so ~8-9 ticks -- definitely NOT 1 (the old,
        // buggy instant-jump behavior).
        if (ticks < 5 || ticks > 12) {
            std::cerr << "    stop-snap ramp regression: took an implausible " << ticks
                       << " ticks to reach 0 -- expected ~8-9\n";
            ok = false;
        }
    }
    // Steady-state idle: v_meas/v_des both ~0, reference holding at zero ->
    // also snaps to exactly 0 (harmless no-op in effect, but exercises the
    // same code path as the decel case above with wants_motion=false).
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(0.0, 0.0, 0.0, 0.0, spec, dt, 0.0);
        if (!d.override_active || d.v_published != 0.0) {
            std::cerr << "    stop-snap (idle): expected override_active with v_published==0.0 "
                          "exactly, got override_active=" << d.override_active << " v_published="
                       << d.v_published << "\n";
            ok = false;
        }
    }

    // -- No LAUNCH when v_des is already at/above breakaway: transparent,
    // the MPC's own (already-sufficient) command is used unchanged. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.0, /*v_des=*/0.15, /*ref_vel_now=*/0.15, 0.0, spec, dt, 0.0);
        if (d.override_active || gov.state() != mpc::LaunchGovernorState::kNormal) {
            std::cerr << "    v_des=0.15 (>= min_moving_speed) should stay transparent, got "
                          "override_active=" << d.override_active << " state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
    }

    // -- No LAUNCH on a tiny residual-position-error v_des blip once the
    // reference has genuinely settled (ref_vel_now ~0) AND the residual
    // position error is AT/BELOW settle_position_tolerance: this must
    // resolve to STOP-SNAP instead, not a spurious relaunch. Regression
    // case for the "final approach" stall-relaunch cluster found
    // empirically in Part B/D (see LaunchGovernor.h's LAUNCH-entry doc
    // comment). at_rest+wants_motion+cannot_break_away all hold here (would
    // fire under the ORIGINAL 3-condition spec), but ref_vel_now is below
    // kStopSnapRefVelThreshold, so the plain-entry's 4th guard suppresses
    // it -- and position_error is explicitly passed as 0.0 here (<=
    // spec.settle_position_tolerance, the CORRECTION-LAUNCH entry's own
    // gate), so CORRECTION-LAUNCH does not pick up the slack either. THIS
    // IS ONE HALF OF A REGRESSION PAIR with the "mirror" case immediately
    // below (position_error=0.06, ABOVE tolerance, where LAUNCH now DOES
    // fire) -- both halves must stay green: the spurious-launch protection
    // must survive as the sub-tolerance case, exactly as it did before
    // CORRECTION-LAUNCH existed, while a genuinely uncorrected residual
    // still gets closed. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.0, /*v_des=*/0.03, /*ref_vel_now=*/0.005,
                      /*last_published_v=*/0.0, spec, dt, /*wall_elapsed_in_state=*/0.0,
                      /*v_raw_fresh=*/false, /*v_raw=*/0.0,
                      /*time_to_ref_stop=*/mpc::LaunchGovernor::kNoTimeToRefStopHint,
                      /*position_error=*/0.0);
        if (gov.state() == mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    tiny residual v_des=0.03 with a truly-settled ref_vel_now=0.005 AND "
                          "position_error=0.0<=tolerance should NOT enter LAUNCH (expected "
                          "STOP-SNAP instead), got LAUNCH with v_published=" << d.v_published << "\n";
            ok = false;
        }
        if (!d.override_active || d.v_published != 0.0) {
            std::cerr << "    tiny residual v_des=0.03 with ref_vel_now=0.005, position_error=0.0: "
                          "expected STOP-SNAP (override_active with v_published==0.0 exactly), got "
                          "override_active=" << d.override_active << " v_published=" << d.v_published
                       << "\n";
            ok = false;
        }
    }
    // -- MIRROR of the regression case immediately above (see its own
    // comment for the pairing): the IDENTICAL v_meas/v_des/ref_vel_now, but
    // position_error=0.06 (> spec.settle_position_tolerance=0.03, the
    // compiled default) -- CORRECTION-LAUNCH must now fire: state()==
    // kLaunch, a single-tick KICK to base (SAME fixed target the plain
    // entry uses, per LaunchGovernor.h's CORRECTION-LAUNCH paragraph --
    // deliberately not scaled by v_des), direction=sign(v_des)=+1. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.0, /*v_des=*/0.03, /*ref_vel_now=*/0.005,
                      /*last_published_v=*/0.0, spec, dt, /*wall_elapsed_in_state=*/0.0,
                      /*v_raw_fresh=*/false, /*v_raw=*/0.0,
                      /*time_to_ref_stop=*/mpc::LaunchGovernor::kNoTimeToRefStopHint,
                      /*position_error=*/0.06);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch || !d.override_active ||
            !near_eq(d.v_published, base, 1e-9) || !d.derive_accel_from_rest) {
            std::cerr << "    MIRROR: v_des=0.03, ref_vel_now=0.005 (stationary), position_error="
                          "0.06>tolerance should enter a CORRECTION LAUNCH (KICK to base=" << base
                       << "), got state=" << static_cast<int>(gov.state()) << " override_active="
                       << d.override_active << " v_published=" << d.v_published
                       << " derive_accel_from_rest=" << d.derive_accel_from_rest << "\n";
            ok = false;
        }
    }
    // Sanity companion: the SAME sub-threshold v_des, but with ref_vel_now
    // ALSO indicating genuine upcoming motion (above the threshold) -- must
    // still enter LAUNCH normally (proves the new guard does not just
    // globally suppress LAUNCH-entry). v_published is the single-tick KICK
    // straight to base -- same as the plain entry case.
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.0, /*v_des=*/0.03, /*ref_vel_now=*/0.03, 0.0, spec, dt, 0.0);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch || !d.override_active ||
            !near_eq(d.v_published, base, 1e-9)) {
            std::cerr << "    same v_des=0.03 WITH ref_vel_now=0.03 (above threshold) should still "
                          "enter LAUNCH normally, got state=" << static_cast<int>(gov.state())
                       << " override_active=" << d.override_active << " v_published="
                       << d.v_published << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G2. Two phases:
//   Phase 1 (the KICK regression case): with the escalation TARGET held
//     flat (elapsed stays well under the 0.5s interval), v_published must
//     JUMP straight to base on the entry tick (implied accel = base/dt,
//     deliberately far above max_accel -- the KICK, not a rate-limited
//     max_accel*dt-per-tick ramp, which is what an EARLIER version of this
//     class did), then HOLD there exactly (accel exactly 0) on every
//     following tick as long as the target does not change -- never
//     oscillating, never re-ramping.
//   Phase 2: escalation timing (0.5s steps, 1.1x compounding factor, cap at
//     1.5x the base launch speed) and the two independent stderr warning
//     cadences (once per step while still climbing; ~once/second, never
//     silently stopping, once capped) -- driven entirely by INJECTED
//     wall_elapsed_in_state values (no sleeps: this is exactly why
//     LaunchGovernor never touches the clock itself -- see
//     LaunchGovernor.h's class doc comment). last_v is threaded from phase
//     1 (already sitting exactly at base), mirroring a real 20Hz loop's own
//     last-published bookkeeping; the KICK publishes the target ladder
//     value EXACTLY every tick (escalated or not), so v_published tracks
//     c.expected_v below exactly regardless of step size.
// Uses an explicit min_moving_speed=min_sustain_speed=0.1 spec (as if a
// --robot-spec override for real hardware had been applied -- the compiled
// default is now 0.0, an ideal plant -- see RobotSpec.h) so base=0.11,
// cap=0.165 as before.
// ---------------------------------------------------------------------
bool test_launch_governor_escalation_timing() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.1;
    const double dt = mpc::MpcParams::defaults().dt;
    const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);  // 0.11
    const double cap = 1.5 * base;                                          // 0.165

    mpc::LaunchGovernor gov;

    // ---- Phase 1: KICK, elapsed pinned at 0.0 throughout (so the target
    // stays flat at `base`). ----
    double last_v = 0.0;
    mpc::LaunchGovernorDecision entry =
        gov.step(/*v_meas=*/0.0, /*v_des=*/0.05, /*ref_vel_now=*/0.05, last_v, spec, dt,
                  /*wall_elapsed_in_state=*/0.0);
    if (!entry.override_active || gov.state() != mpc::LaunchGovernorState::kLaunch) {
        std::cerr << "    phase 1 entry: expected override_active + kLaunch\n";
        ok = false;
    }
    if (!near_eq(entry.v_published, base, 1e-9)) {
        std::cerr << "    phase 1 entry: expected an IMMEDIATE KICK to base=" << base
                   << " (not a rate-limited first step -- the OLD ramp design), got "
                   << entry.v_published << "\n";
        ok = false;
    }
    if (!entry.derive_accel_from_rest) {
        std::cerr << "    phase 1 entry: expected derive_accel_from_rest=true\n";
        ok = false;
    }
    {
        // Caller derives accel from 0.0 (derive_accel_from_rest), NOT from
        // last_v -- on this entry tick last_v happens to be 0.0 already, so
        // this is also (entry.v_published - last_v)/dt; the distinction
        // only becomes observable starting the NEXT tick, below.
        const double entry_accel = (entry.v_published - 0.0) / dt;
        if (entry_accel <= spec.max_accel) {
            std::cerr << "    phase 1 entry: expected the KICK's implied accel (" << entry_accel
                       << ") to exceed spec.max_accel(" << spec.max_accel << ")\n";
            ok = false;
        }
    }
    last_v = entry.v_published;

    // Every following tick (target still flat at base): SUSTAINED kick --
    // v_published HOLDS exactly at base, but derive_accel_from_rest stays
    // true and the CALLER-derived accel (from 0.0, per
    // LaunchGovernorDecision's doc comment -- NOT from prev_v) stays the
    // FULL base/dt on every tick, never dropping to 0. This is the exact
    // regression this task's report describes: an earlier version of this
    // fix derived accel from prev_v (== base by tick 2, since v_published
    // itself was already holding), which correctly kept v_published steady
    // but silently zeroed the ACCEL published on the wire -- and since the
    // sim only integrates "accel" (never "speed"), that stranded any plant
    // that had not fully broken away within the single entry tick's own
    // impulse, with no further push ever again (observed to hang
    // indefinitely, escalating to its cap and no further, in a real
    // multi-robot run). Five ticks is arbitrary but plenty to catch either
    // that regression or the earlier rate-limited-ramp/v0-anchored
    // alternation regressions.
    for (int i = 0; i < 5; ++i) {
        const double prev_v = last_v;
        mpc::LaunchGovernorDecision d = gov.step(0.0, 0.05, 0.05, prev_v, spec, dt, 0.0);
        if (!d.derive_accel_from_rest) {
            std::cerr << "    phase 1 tick " << i << ": expected derive_accel_from_rest=true\n";
            ok = false;
        }
        const double accel_from_rest = (d.v_published - 0.0) / dt;
        if (!near_eq(d.v_published, base, 1e-9) || !near_eq(accel_from_rest, base / dt, 1e-6)) {
            std::cerr << "    phase 1 tick " << i << ": expected to hold v_published==base with a "
                          "SUSTAINED accel-from-rest==base/dt=" << (base / dt)
                       << ", got v_published=" << d.v_published << " accel_from_rest="
                       << accel_from_rest << "\n";
            ok = false;
        }
        last_v = d.v_published;
    }

    // ---- Phase 2: escalation ladder + warning cadence. ----
    struct Check {
        double elapsed;
        double expected_v;
        const char* label;
    };
    // Hand-computed: target(step) = base * 1.1^step, capped at 0.165.
    // step = floor(elapsed / 0.5). 1.1^4*base=0.161051 (<cap); 1.1^5*base=
    // 0.1771561 (>=cap -> clamped). The KICK publishes each step's target
    // EXACTLY, regardless of step size (unlike an earlier rate-limited-ramp
    // design, which would have needed the target increase to stay under
    // max_accel*dt to be absorbed in one tick -- irrelevant now).
    const std::vector<Check> checks = {
        {0.4, base, "step 0 (still < 0.5s)"},
        {0.5, base * 1.1, "step 1"},
        {0.5, base * 1.1, "step 1 (repeat call, same elapsed -- must be stable)"},
        {1.0, base * 1.1 * 1.1, "step 2"},
        {1.5, base * std::pow(1.1, 3), "step 3"},
        {2.0, base * std::pow(1.1, 4), "step 4 (last pre-cap step)"},
        {2.5, cap, "step 5 (first capped step)"},
        {2.6, cap, "step 5 (still capped, same step)"},
        {3.6, cap, "step 7 (capped, further step advance)"},
        {4.7, cap, "step 9 (capped, still holding, never gives up)"},
    };

    CerrCapture cap_output;
    for (const auto& c : checks) {
        mpc::LaunchGovernorDecision d = gov.step(0.0, 0.05, 0.05, last_v, spec, dt, c.elapsed);
        if (!d.override_active || gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    [" << c.label << "] expected to still be in an active LAUNCH\n";
            ok = false;
        }
        if (!near_eq(d.v_published, c.expected_v, 1e-9)) {
            std::cerr << "    [" << c.label << "] elapsed=" << c.elapsed << ": v_published="
                       << d.v_published << ", expected " << c.expected_v << "\n";
            ok = false;
        }
        last_v = d.v_published;
    }
    const std::string log = cap_output.str();

    // Warning cadence: exactly 5 "escalating" lines (steps 1,2,3,4, and the
    // step-5 transition-into-cap tick), then NONE thereafter (steps 7, 9
    // must not re-print it, even though escalation_step_ keeps advancing
    // internally) -- this is the "per-step warnings stop once capped"
    // requirement.
    auto count_substr = [](const std::string& s, const std::string& needle) {
        int n = 0;
        size_t pos = 0;
        while ((pos = s.find(needle, pos)) != std::string::npos) {
            ++n;
            pos += needle.size();
        }
        return n;
    };
    const int escalating_count = count_substr(log, "escalating launch target");
    const int capped_count = count_substr(log, "capped at");
    if (escalating_count != 5) {
        std::cerr << "    expected exactly 5 'escalating launch target' warnings (steps 1-4 + the "
                      "step-5 cap-transition tick), got " << escalating_count << ". Full log:\n"
                   << log;
        ok = false;
    }
    // Cap-cadence warnings: step 5 (elapsed 2.5, first hit), elapsed 3.6
    // (delta 1.1s >= 1.0s since 2.5), elapsed 4.7 (delta 1.1s >= 1.0s since
    // 3.6) = 3 total; elapsed 2.6 (delta 0.1s) must NOT add a 4th.
    if (capped_count != 3) {
        std::cerr << "    expected exactly 3 'capped at' warnings (elapsed 2.5/3.6/4.7 -- ~once/"
                      "second cadence while capped), got " << capped_count << ". Full log:\n"
                   << log;
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// G3. Governor is permanently transparent when spec.min_moving_speed == 0 --
// no LAUNCH, no STOP-SNAP, state() stays kNormal -- regardless of inputs
// that would otherwise trigger either transition under a nonzero threshold.
// Uses mpc::RobotSpec::defaults() DIRECTLY (no manual zeroing): the
// compiled default is now min_moving_speed=min_sustain_speed=0.0 (an ideal
// plant -- see RobotSpec.h and D3's own defaults check), so this test
// directly exercises the property the whole explicit-spec design relies on
// -- an unconfigured controller must never activate the governor.
// ---------------------------------------------------------------------
bool test_launch_governor_inactive_when_no_deadband() {
    bool ok = true;
    const mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    const double dt = mpc::MpcParams::defaults().dt;

    mpc::LaunchGovernor gov;

    // Would trigger LAUNCH entry under a nonzero threshold (at rest, wants
    // motion, "sub-breakaway" -- meaningless at threshold 0, but the inputs
    // themselves mimic that scenario).
    mpc::LaunchGovernorDecision d1 =
        gov.step(/*v_meas=*/0.0, /*v_des=*/0.05, 0.05, /*last_published_v=*/0.0, spec, dt, 0.0);
    if (d1.override_active || gov.state() != mpc::LaunchGovernorState::kNormal) {
        std::cerr << "    min_moving_speed=0: expected transparent/kNormal on a would-be-LAUNCH "
                      "input, got override_active=" << d1.override_active << " state="
                   << static_cast<int>(gov.state()) << "\n";
        ok = false;
    }

    // Repeated ticks (simulating an ongoing run) never latch into LAUNCH.
    for (int i = 0; i < 10; ++i) {
        mpc::LaunchGovernorDecision d =
            gov.step(0.0, 0.05, 0.05, 0.0, spec, dt, static_cast<double>(i) * 0.5);
        if (d.override_active || gov.state() != mpc::LaunchGovernorState::kNormal) {
            std::cerr << "    min_moving_speed=0: tick " << i << " unexpectedly active/non-kNormal\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G4. FAST EXIT: kRawExitDebounceCount (2) consecutive FRESH raw samples
// exit LAUNCH -- well before the (still intact, unchanged) filtered-
// estimate v_meas fallback would. Ticks with v_raw_fresh=false neither
// advance nor reset the streak; a fresh sample BELOW the bar resets it. A
// caller that never passes v_raw_fresh=true (default false) exits only via
// the pre-existing fallback -- byte-for-byte unchanged.
// ---------------------------------------------------------------------
bool test_launch_governor_raw_exit_debounce() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.1;
    const double dt = mpc::MpcParams::defaults().dt;
    const double exit_bar = mpc::LaunchGovernor::kExitFraction * spec.min_moving_speed;  // 0.07

    // -- 2 consecutive fresh samples at/above the bar exit fast, while
    // v_meas (the filtered estimate) stays well below its own fallback bar
    // throughout -- proves the FAST path, not the fallback, did it. --
    {
        mpc::LaunchGovernor gov;
        double v = gov.step(0.0, 0.05, 0.05, 0.0, spec, dt, 0.0).v_published;  // enter LAUNCH.

        // Tick 1: fresh raw sample AT the bar -- streak=1, still LAUNCH.
        mpc::LaunchGovernorDecision d1 = gov.step(/*v_meas=*/0.0, 0.05, 0.05, v, spec, dt, 0.05,
                                                    /*v_raw_fresh=*/true, /*v_raw=*/exit_bar);
        if (d1.just_exited_launch || gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    1 fresh sample at the bar should NOT exit yet (need 2), got "
                          "just_exited_launch=" << d1.just_exited_launch << " state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }

        // Tick 2: v_raw_fresh=false (no fresh sample this tick) -- streak
        // must be UNTOUCHED (neither advanced nor reset), still LAUNCH.
        mpc::LaunchGovernorDecision d2 =
            gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.1, /*v_raw_fresh=*/false, /*v_raw=*/0.0);
        if (d2.just_exited_launch || gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    a no-fresh-sample tick must not itself exit or reset progress\n";
            ok = false;
        }

        // Tick 3: SECOND consecutive fresh sample at/above the bar -- exits
        // NOW (streak reaches kRawExitDebounceCount=2), even though v_meas
        // (filtered) is still 0.0 -- far below exit_bar -- proving the FAST
        // path fired, not the fallback.
        mpc::LaunchGovernorDecision d3 = gov.step(/*v_meas=*/0.0, 0.05, 0.05, v, spec, dt, 0.15,
                                                    /*v_raw_fresh=*/true, /*v_raw=*/exit_bar);
        if (!d3.just_exited_launch || d3.override_active ||
            gov.state() != mpc::LaunchGovernorState::kNormal) {
            std::cerr << "    2nd consecutive fresh sample at the bar should exit LAUNCH: "
                          "just_exited_launch=" << d3.just_exited_launch << " override_active="
                       << d3.override_active << " state=" << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
    }

    // -- A fresh sample BELOW the bar resets the streak: 1 good, 1 bad, then
    // 2 MORE good samples are needed (not just 1) to exit. --
    {
        mpc::LaunchGovernor gov;
        double v = gov.step(0.0, 0.05, 0.05, 0.0, spec, dt, 0.0).v_published;

        gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.05, true, exit_bar);  // streak=1.
        mpc::LaunchGovernorDecision bad =
            gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.1, true, exit_bar - 0.01);  // below bar: reset.
        if (bad.just_exited_launch) {
            std::cerr << "    a below-bar fresh sample must not itself exit\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision after_reset_1 =
            gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.15, true, exit_bar);  // streak=1 (post-reset).
        if (after_reset_1.just_exited_launch) {
            std::cerr << "    only 1 good sample since the reset should NOT exit yet\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision after_reset_2 =
            gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.2, true, exit_bar);  // streak=2 (post-reset).
        if (!after_reset_2.just_exited_launch) {
            std::cerr << "    2nd good sample since the reset should exit now\n";
            ok = false;
        }
    }

    // -- Backward compatibility: a caller that never passes v_raw_fresh=true
    // (the default) exits ONLY via the pre-existing filtered-estimate
    // fallback, exactly as before this parameter was added. --
    {
        mpc::LaunchGovernor gov;
        double v = gov.step(0.0, 0.05, 0.05, 0.0, spec, dt, 0.0).v_published;
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/exit_bar, 0.05, 0.05, v, spec, dt, 0.05);
        if (!d.just_exited_launch || gov.state() != mpc::LaunchGovernorState::kNormal) {
            std::cerr << "    fallback-only exit (no v_raw_fresh ever passed) should still exit "
                          "and set just_exited_launch\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G5. OBSERVER INJECTION CONTRACT: LaunchGovernorDecision::just_exited_launch
// is true on EXACTLY the tick state_ transitions kLaunch->kNormal (by
// either exit path), and false on every other tick -- including ordinary
// LAUNCH ticks, ordinary NORMAL/transparent ticks, and STOP-SNAP ticks.
// main.cpp uses this flag (not directly testable here, main.cpp being a
// thin I/O shell with no pure entry point -- see main.cpp's own header
// comment) to one-shot inject its OWN velocity-observer state; this test
// verifies the governor-side contract that injection depends on.
// ---------------------------------------------------------------------
bool test_launch_governor_just_exited_launch_flag() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.1;
    const double dt = mpc::MpcParams::defaults().dt;
    const double exit_bar = mpc::LaunchGovernor::kExitFraction * spec.min_moving_speed;

    // -- Never true while still climbing in LAUNCH. --
    {
        mpc::LaunchGovernor gov;
        double v = 0.0;
        for (int i = 0; i < 3; ++i) {
            mpc::LaunchGovernorDecision d =
                gov.step(0.0, 0.05, 0.05, v, spec, dt, static_cast<double>(i) * 0.05);
            if (d.just_exited_launch) {
                std::cerr << "    tick " << i << ": just_exited_launch must be false while still "
                              "climbing in LAUNCH\n";
                ok = false;
            }
            v = d.override_active ? d.v_published : v;
        }
    }

    // -- True on exactly the FALLBACK exit tick, false before and after. --
    {
        mpc::LaunchGovernor gov;
        double v = gov.step(0.0, 0.05, 0.05, 0.0, spec, dt, 0.0).v_published;
        mpc::LaunchGovernorDecision still_launch =
            gov.step(/*v_meas=*/exit_bar - 0.01, 0.05, 0.05, v, spec, dt, 0.05);
        if (still_launch.just_exited_launch) {
            std::cerr << "    just below the fallback bar must not set just_exited_launch\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision exits =
            gov.step(/*v_meas=*/exit_bar, 0.05, 0.05, v, spec, dt, 0.1);
        if (!exits.just_exited_launch) {
            std::cerr << "    crossing the fallback bar must set just_exited_launch\n";
            ok = false;
        }
        // The NEXT tick (already NORMAL, ordinary transparent publishing --
        // v_des above breakaway now that motion is confirmed) must NOT
        // re-report an exit; it already happened.
        mpc::LaunchGovernorDecision after =
            gov.step(/*v_meas=*/0.15, /*v_des=*/0.15, 0.15, exits.v_published, spec, dt, 0.0);
        if (after.just_exited_launch) {
            std::cerr << "    just_exited_launch must not stay latched on the tick AFTER the exit\n";
            ok = false;
        }
    }

    // -- True on exactly the DEBOUNCE exit tick (see G4 for the full
    // debounce mechanics; this test only checks the flag). --
    {
        mpc::LaunchGovernor gov;
        double v = gov.step(0.0, 0.05, 0.05, 0.0, spec, dt, 0.0).v_published;
        mpc::LaunchGovernorDecision first =
            gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.05, true, exit_bar);
        if (first.just_exited_launch) {
            std::cerr << "    1st fresh sample must not set just_exited_launch (need 2)\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision second =
            gov.step(0.0, 0.05, 0.05, v, spec, dt, 0.1, true, exit_bar);
        if (!second.just_exited_launch) {
            std::cerr << "    2nd consecutive fresh sample must set just_exited_launch\n";
            ok = false;
        }
    }

    // -- Never true for a STOP-SNAP tick (NORMAL the whole time, never
    // entered LAUNCH at all). --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/0.05, /*v_des=*/0.02,
                                                   /*ref_vel_now=*/0.0, /*last_published_v=*/0.09,
                                                   spec, dt, 0.0);
        if (!d.override_active || d.just_exited_launch) {
            std::cerr << "    STOP-SNAP tick must override_active without setting "
                          "just_exited_launch (never entered LAUNCH)\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G6. ANTI-RE-STALL MOVING-floor: fires ONLY when ALL THREE of (plant
// believed moving via kExitFraction, |ref_vel_now| >=
// kMovingFloorRefVelThreshold, |v_des| < floor) hold, publishing
// floor*sign(ref_vel_now) via the SAME rate-limited-toward-target mechanism
// STOP-SNAP uses; never fires during LAUNCH (structurally unreachable -- the
// floor branch lives entirely inside the state_==kNormal path) or when any
// ONE of the three conditions is missing. Uses min_sustain_speed=0.07 (the
// D2 0.7*level hysteresis default) rather than ==min_moving_speed, so
// floor=0.099 sits inside the hysteresis band as it would at runtime.
// ---------------------------------------------------------------------
bool test_launch_governor_moving_floor() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.07;
    const double dt = mpc::MpcParams::defaults().dt;
    const double max_delta = spec.max_accel * dt;
    const double floor =
        spec.min_sustain_speed + 0.3 * (spec.min_moving_speed - spec.min_sustain_speed) + 0.02;
    const double moving_bar = mpc::LaunchGovernor::kExitFraction * spec.min_moving_speed;  // 0.07

    // -- Fires: all three conditions hold. last_published_v chosen exactly
    // at floor so it lands there trivially in one tick (isolating the VALUE
    // check from the separate rate-limiting check below). --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15,
                      /*last_published_v=*/floor, spec, dt, 0.0);
        if (!d.override_active || !near_eq(d.v_published, floor, 1e-9)) {
            std::cerr << "    moving-floor should fire and publish floor=" << floor
                       << ", got override_active=" << d.override_active << " v_published="
                       << d.v_published << "\n";
            ok = false;
        }
        if (gov.state() != mpc::LaunchGovernorState::kNormal || d.just_exited_launch) {
            std::cerr << "    moving-floor override must not touch state()/just_exited_launch\n";
            ok = false;
        }
    }

    // -- Direction follows sign(ref_vel_now), NOT sign(v_des) -- v_des here
    // is positive (0.02) while ref_vel_now is negative, and the published
    // floor must be NEGATIVE (ref_vel_now's sign). --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/-0.15, /*v_des=*/0.02, /*ref_vel_now=*/-0.15,
                      /*last_published_v=*/-floor, spec, dt, 0.0);
        if (!d.override_active || !near_eq(d.v_published, -floor, 1e-9)) {
            std::cerr << "    moving-floor direction should follow ref_vel_now's sign (negative), "
                          "got v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Rate-limited, not an instant jump: last_published_v far from the
    // target -- v_published must move by at most max_accel*dt this tick. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15,
                      /*last_published_v=*/0.0, spec, dt, 0.0);
        if (!d.override_active || !near_eq(d.v_published, max_delta, 1e-9)) {
            std::cerr << "    moving-floor should be rate-limited to max_accel*dt=" << max_delta
                       << " on the first tick from far away, got " << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Never fires: v_des already at/above floor -- transparent instead. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/0.15, /*v_des=*/floor + 0.01,
                                                   /*ref_vel_now=*/0.15, /*last_published_v=*/0.0,
                                                   spec, dt, 0.0);
        if (d.override_active) {
            std::cerr << "    v_des already >= floor should stay transparent, got v_published="
                       << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Never fires: plant NOT believed moving (v_meas below the
    // kExitFraction bar, even though ref wants motion and v_des is low) --
    // stays transparent (neither floor nor stop-snap: ref_vel_now is well
    // above kStopSnapRefVelThreshold here). --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/moving_bar - 0.01, /*v_des=*/0.02, /*ref_vel_now=*/0.15,
                      /*last_published_v=*/0.0, spec, dt, 0.0);
        if (d.override_active) {
            std::cerr << "    plant not believed moving should stay transparent (not floor), got "
                          "v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Never fires: |ref_vel_now| in the dead zone between
    // kStopSnapRefVelThreshold(0.02) and kMovingFloorRefVelThreshold(0.05)
    // -- neither STOP-SNAP nor the floor claims this band; transparent. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/0.15, /*v_des=*/0.02,
                                                   /*ref_vel_now=*/0.03, /*last_published_v=*/0.0,
                                                   spec, dt, 0.0);
        if (d.override_active) {
            std::cerr << "    ref_vel_now in the STOP-SNAP/floor dead zone should stay "
                          "transparent, got v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- STOP-SNAP still takes precedence in the region it claims
    // (ref_vel_now near zero): confirm STOP-SNAP's own ramp fires there,
    // not a (structurally unreachable, since the floor needs ref_vel_now >=
    // 0.05) floor value. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(/*v_meas=*/0.15, /*v_des=*/0.02,
                                                   /*ref_vel_now=*/0.0, /*last_published_v=*/0.09,
                                                   spec, dt, 0.0);
        if (!d.override_active || !near_eq(d.v_published, 0.09 - max_delta, 1e-9)) {
            std::cerr << "    ref_vel_now~0 should still take the STOP-SNAP ramp, not the floor\n";
            ok = false;
        }
    }

    // -- Never fires while LAUNCH is active: structurally unreachable -- a
    // tick that WOULD satisfy the floor's own bars, but is also a fresh
    // LAUNCH-entry (at_rest+wants_motion+cannot_break_away all ALSO true
    // here), is governed by the KICK instead. --
    {
        mpc::LaunchGovernor gov;
        const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.0, /*v_des=*/0.05, /*ref_vel_now=*/0.15,
                      /*last_published_v=*/0.0, spec, dt, 0.0);
        if (!d.override_active || !near_eq(d.v_published, base, 1e-9) ||
            gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    LAUNCH entry must win over any floor-like inputs -- expected the "
                          "KICK to base=" << base << ", got v_published=" << d.v_published
                       << " state=" << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G7. FLOOR YIELD NEAR REFERENCE END: the MOVING-floor's fourth condition
// (LaunchGovernor.h's kFloorYieldHorizonS doc comment has the full "why") --
// with all three of G6's original firing conditions held constant and
// satisfied, only time_to_ref_stop varies across these cases. Same spec as
// G6 (min_moving_speed=0.1, min_sustain_speed=0.07, floor=0.099).
// ---------------------------------------------------------------------
bool test_launch_governor_floor_yield_near_ref_end() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.07;
    const double dt = mpc::MpcParams::defaults().dt;
    const double floor =
        spec.min_sustain_speed + 0.3 * (spec.min_moving_speed - spec.min_sustain_speed) + 0.02;
    const double horizon = mpc::LaunchGovernor::kFloorYieldHorizonS;

    // -- Yields well within the horizon (0.1s from the reference's end):
    // stays transparent instead of publishing the floor. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            /*time_to_ref_stop=*/0.1);
        if (d.override_active) {
            std::cerr << "    floor should YIELD 0.1s from ref end (< horizon=" << horizon
                       << "), got override_active with v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Yields for a NEGATIVE time_to_ref_stop (reference already past its
    // last waypoint) -- same guard, treated as "within the horizon". --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            /*time_to_ref_stop=*/-0.2);
        if (d.override_active) {
            std::cerr << "    floor should YIELD for negative time_to_ref_stop, got "
                          "override_active with v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Boundary: exactly AT the horizon yields too (guard uses <=). --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            /*time_to_ref_stop=*/horizon);
        if (d.override_active) {
            std::cerr << "    floor should YIELD exactly AT the horizon (" << horizon
                       << "), got override_active with v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Just OUTSIDE the horizon: floor still fires normally (confirms the
    // guard's boundary is oriented correctly, not inverted). --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            /*time_to_ref_stop=*/horizon + 0.01);
        if (!d.override_active || !near_eq(d.v_published, floor, 1e-9)) {
            std::cerr << "    floor should still fire just outside the horizon ("
                       << (horizon + 0.01) << "s from end), got override_active="
                       << d.override_active << " v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Far from the end (5.0s, a typical mid-segment value, not just the
    // sentinel default): floor fires exactly as G6 describes. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            /*time_to_ref_stop=*/5.0);
        if (!d.override_active || !near_eq(d.v_published, floor, 1e-9)) {
            std::cerr << "    floor should fire 5.0s from ref end, got override_active="
                       << d.override_active << " v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Caller that never wires time_to_ref_stop up at all (fewer args, the
    // sentinel default) -- byte-for-byte the pre-existing G6 behavior. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15,
                      /*last_published_v=*/floor, spec, dt, /*wall_elapsed_in_state=*/0.0);
        if (!d.override_active || !near_eq(d.v_published, floor, 1e-9)) {
            std::cerr << "    floor should fire when time_to_ref_stop is omitted (sentinel "
                          "default), got override_active=" << d.override_active
                       << " v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Still fires mid-segment with the governor's OWN rate limiting intact
    // (not just the value check above) -- last_published_v far from target,
    // far from ref end. Mirrors G6's own rate-limit case exactly. --
    {
        mpc::LaunchGovernor gov;
        const double max_delta = spec.max_accel * dt;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/0.0,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            /*time_to_ref_stop=*/5.0);
        if (!d.override_active || !near_eq(d.v_published, max_delta, 1e-9)) {
            std::cerr << "    floor should still be rate-limited to max_accel*dt=" << max_delta
                       << " far from ref end, got " << d.v_published << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G8. FLOOR YIELD NEAR AN INTERMEDIATE HOLD: generalizes G7 to confirm the
// SAME time_to_ref_stop guard (LaunchGovernor.h's kFloorYieldHorizonS doc
// comment -- now describing STOP EVENTS broadly, not just the reference's
// absolute end) behaves correctly across a hold-shaped sequence of
// time_to_ref_stop values: an upcoming intermediate hold spanning
// [kHoldStart, kHoldEnd] (a 3.0s hold, mirroring this task's own synthetic
// test_mpc_deadband Part F scenario), approached, entered, sat through, and
// exited, with a later stop event (e.g. the trajectory's own end) far
// beyond it. time_to_ref_stop is hand-computed per case exactly as
// mpc::time_to_next_ref_stop would resolve it against a real
// extract_ref_stop_events() list containing this hold -- see the SEPARATE
// test_extract_ref_stop_events below for coverage of that computation
// itself, including the "back-to-back holds"/"trailing hold merging with
// end" edge cases. Same spec/floor as G6/G7 (min_moving_speed=0.1,
// min_sustain_speed=0.07, floor=0.099); only ref_time (and so
// time_to_ref_stop) varies across cases.
// ---------------------------------------------------------------------
bool test_launch_governor_floor_yield_near_hold() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.07;
    const double dt = mpc::MpcParams::defaults().dt;
    const double floor =
        spec.min_sustain_speed + 0.3 * (spec.min_moving_speed - spec.min_sustain_speed) + 0.02;
    const double horizon = mpc::LaunchGovernor::kFloorYieldHorizonS;

    constexpr double kHoldStart = 10.0;
    constexpr double kHoldEnd = 13.0;             // 3.0s hold.
    constexpr double kNextStopStart = kHoldEnd + 20.0;  // next event, well past the hold.

    // time_to_ref_stop exactly as mpc::time_to_next_ref_stop would resolve
    // it for a given ref_time against this hold plus a far-away next event
    // -- see that function's own contract (LaunchGovernor.h).
    auto time_to_ref_stop_for = [](double ref_time) {
        return (ref_time <= kHoldEnd) ? (kHoldStart - ref_time) : (kNextStopStart - ref_time);
    };

    // -- Far before the hold (5.0s out): floor fires normally. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            time_to_ref_stop_for(kHoldStart - 5.0));
        if (!d.override_active || !near_eq(d.v_published, floor, 1e-9)) {
            std::cerr << "    floor should fire 5.0s before the hold, got override_active="
                       << d.override_active << " v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Inside the horizon approaching the hold (0.1s before it, <
    // kFloorYieldHorizonS=0.4): floor yields. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            time_to_ref_stop_for(kHoldStart - 0.1));
        if (d.override_active) {
            std::cerr << "    floor should YIELD 0.1s before the hold (< horizon=" << horizon
                       << "), got override_active with v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Well inside the hold (1.5s in, time_to_ref_stop == -1.5): still
    // yields -- this is the case the pre-generalization mechanism (which
    // only ever compared against the trajectory's absolute end) got WRONG,
    // and is this fix's whole point. --
    {
        mpc::LaunchGovernor gov;
        const double t2s = time_to_ref_stop_for(kHoldStart + 1.5);
        if (!(t2s <= 0.0)) {
            std::cerr << "    test bug: expected time_to_ref_stop <= 0 inside the hold, got "
                       << t2s << "\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0, t2s);
        if (d.override_active) {
            std::cerr << "    floor should YIELD while inside the hold (time_to_ref_stop=" << t2s
                       << "), got override_active with v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Right at the hold's end (time_to_ref_stop == -3.0, inclusive
    // boundary): still yields. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0,
            time_to_ref_stop_for(kHoldEnd));
        if (d.override_active) {
            std::cerr << "    floor should YIELD exactly at the hold's end (inclusive), got "
                          "override_active with v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // -- Just past the hold's end (cruising resumes, next stop event far
    // away): floor RESUMES firing. --
    {
        mpc::LaunchGovernor gov;
        const double t2s = time_to_ref_stop_for(kHoldEnd + 0.5);
        if (!(t2s > horizon)) {
            std::cerr << "    test bug: expected time_to_ref_stop > horizon just past the hold, "
                          "got " << t2s << "\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision d = gov.step(
            /*v_meas=*/0.15, /*v_des=*/0.02, /*ref_vel_now=*/0.15, /*last_published_v=*/floor,
            spec, dt, /*wall_elapsed_in_state=*/0.0, /*v_raw_fresh=*/false, /*v_raw=*/0.0, t2s);
        if (!d.override_active || !near_eq(d.v_published, floor, 1e-9)) {
            std::cerr << "    floor should RESUME firing just past the hold (time_to_ref_stop="
                       << t2s << "), got override_active=" << d.override_active
                       << " v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// G9. CORRECTION-LAUNCH (see LaunchGovernor.h's CORRECTION-LAUNCH paragraph
// and RobotSpec::settle_position_tolerance/settle_cooldown_s/
// settle_max_attempts's own doc comments): the FINISH-AT-REST fix -- lets a
// robot close a residual position error at an otherwise-stationary
// reference instead of parking short forever. Covers, in order: (1) a
// truth table of each entry guard independently blocking (below-tolerance,
// cooldown active, attempts exhausted, ref moving -- plain-entry path
// unaffected); (2) all-satisfied entry in both directions; (3) escalation
// is skipped (flat repeated kick) and the give-up timeout fires cleanly;
// (4) attempt count + cooldown reset on a NEW stop event; (5) reset() (new
// trajectory/RESUME) clears everything. The G1 regression PAIR (sub-
// tolerance no-launch vs. its over-tolerance mirror) lives in G1 itself
// (test_launch_governor_state_transitions) -- not duplicated here.
// Uses the SAME explicit min_moving_speed=min_sustain_speed=0.1,
// launch_margin=0.1 (base=0.11) spec as G1/G2, plus explicit
// settle_position_tolerance=0.03/settle_cooldown_s=1.0/settle_max_attempts=3
// (== compiled defaults, set explicitly so this test stays correct even if
// those defaults change later).
// ---------------------------------------------------------------------
bool test_launch_governor_correction_launch() {
    bool ok = true;
    mpc::RobotSpec spec = mpc::RobotSpec::defaults();
    spec.min_moving_speed = 0.1;
    spec.min_sustain_speed = 0.1;
    spec.settle_position_tolerance = 0.03;
    spec.settle_cooldown_s = 1.0;
    spec.settle_max_attempts = 3;
    const double dt = mpc::MpcParams::defaults().dt;
    const double base = spec.min_moving_speed * (1.0 + spec.launch_margin);  // 0.11
    constexpr double kHint = mpc::LaunchGovernor::kNoTimeToRefStopHint;

    // ==== (1) Truth table: each guard independently blocking. ====

    // -- (1a) below-tolerance: position_error==spec.settle_position_tolerance
    // itself (boundary -- ">" required, so exactly-at-tolerance must NOT
    // fire either) blocks entry; falls through to STOP-SNAP instead. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint,
                      /*position_error=*/spec.settle_position_tolerance);
        if (gov.state() == mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    (1a) position_error==tolerance (boundary) should NOT enter "
                          "CORRECTION-LAUNCH, got LAUNCH\n";
            ok = false;
        }
    }

    // -- (1b) cooldown active: after one full correction-launch cycle
    // (enter -> confirmed breakaway exit), an immediate retry (well under
    // settle_cooldown_s=1.0 of self-elapsed time later) must NOT re-enter,
    // even though position_error is still over tolerance. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision entry =
            gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint, /*position_error=*/0.06);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    (1b) setup: expected CORRECTION-LAUNCH entry, got state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
        // Confirmed breakaway (fallback exit): v_meas >= kExitFraction*0.1=0.07.
        mpc::LaunchGovernorDecision exit_d = gov.step(0.08, 0.03, 0.0, entry.v_published, spec, dt,
                                                        0.05, false, 0.0, kHint, 0.06);
        if (gov.state() != mpc::LaunchGovernorState::kNormal || !exit_d.just_exited_launch ||
            gov.settle_attempts_this_stop_event() != 1) {
            std::cerr << "    (1b) setup: expected confirmed-breakaway exit to NORMAL with "
                          "settle_attempts_this_stop_event()==1, got state="
                       << static_cast<int>(gov.state()) << " just_exited_launch="
                       << exit_d.just_exited_launch << " attempts="
                       << gov.settle_attempts_this_stop_event() << "\n";
            ok = false;
        }
        mpc::LaunchGovernorDecision retry =
            gov.step(0.0, 0.03, 0.0, exit_d.v_published, spec, dt, 0.0, false, 0.0, kHint, 0.06);
        if (gov.state() == mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    (1b) immediate retry (cooldown not elapsed) should NOT re-enter "
                          "CORRECTION-LAUNCH, got LAUNCH\n";
            ok = false;
        }
    }

    // -- (1c) attempts exhausted: three give-up (timeout) cycles for the
    // SAME stop event exhaust settle_max_attempts==3; a fourth attempt,
    // even with cooldown fully elapsed (each cycle fast-forwards the
    // internal clock via a large-dt idle tick), must NOT enter. --
    {
        mpc::LaunchGovernor gov;
        for (int attempt = 0; attempt < spec.settle_max_attempts; ++attempt) {
            mpc::LaunchGovernorDecision entry = gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false,
                                                           0.0, kHint, 0.06);
            if (gov.state() != mpc::LaunchGovernorState::kLaunch) {
                std::cerr << "    (1c) attempt " << attempt << ": expected entry, got state="
                           << static_cast<int>(gov.state()) << "\n";
                ok = false;
            }
            // Give up: wall_elapsed_in_state at/above kCorrectionLaunchTimeoutS, v_meas never
            // confirms breakaway.
            mpc::LaunchGovernorDecision give_up =
                gov.step(0.0, 0.03, 0.0, entry.v_published, spec, dt,
                          mpc::LaunchGovernor::kCorrectionLaunchTimeoutS, false, 0.0, kHint, 0.06);
            if (gov.state() != mpc::LaunchGovernorState::kNormal || give_up.override_active) {
                std::cerr << "    (1c) attempt " << attempt << ": expected a transparent give-up "
                              "exit to NORMAL, got state=" << static_cast<int>(gov.state())
                           << " override_active=" << give_up.override_active << "\n";
                ok = false;
            }
            // Fast-forward the internal clock well past settle_cooldown_s before the next
            // attempt, via a single large-dt idle tick (position_error=0.0 default -> wants
            // no motion -> harmless STOP-SNAP no-op).
            gov.step(0.0, 0.0, 0.0, 0.0, spec, /*dt=*/1.5, 0.0);
        }
        if (gov.settle_attempts_this_stop_event() != spec.settle_max_attempts) {
            std::cerr << "    (1c) expected settle_attempts_this_stop_event()=="
                       << spec.settle_max_attempts << " after " << spec.settle_max_attempts
                       << " give-up cycles, got " << gov.settle_attempts_this_stop_event() << "\n";
            ok = false;
        }
        // Cooldown is fully elapsed here (each cycle above fast-forwarded 1.5s > 1.0s) -- the
        // ONLY remaining blocker is the exhausted attempt cap.
        mpc::LaunchGovernorDecision blocked =
            gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint, 0.06);
        if (gov.state() == mpc::LaunchGovernorState::kLaunch) {
            std::cerr << "    (1c) attempts exhausted (cooldown elapsed) should NOT re-enter "
                          "CORRECTION-LAUNCH, got LAUNCH\n";
            ok = false;
        }
        if (gov.settle_attempts_this_stop_event() != spec.settle_max_attempts) {
            std::cerr << "    (1c) blocked attempt must NOT itself increment the attempt count "
                          "(still expected " << spec.settle_max_attempts << "), got "
                       << gov.settle_attempts_this_stop_event() << "\n";
            ok = false;
        }
    }

    // -- (1d) ref moving: the PLAIN reference-motion entry path is
    // unaffected by position_error's value (even a huge one) -- it enters
    // via the ORIGINAL 4-condition gate exactly as G1 already covers,
    // regardless of what CORRECTION-LAUNCH's own gates would say. --
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(/*v_meas=*/0.0, /*v_des=*/0.05, /*ref_vel_now=*/0.05, 0.0, spec, dt, 0.0,
                      false, 0.0, kHint, /*position_error=*/0.5);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch || !near_eq(d.v_published, base, 1e-9)) {
            std::cerr << "    (1d) ref_vel_now=0.05 (moving) should take the PLAIN entry path "
                          "unchanged regardless of position_error=0.5, got state="
                       << static_cast<int>(gov.state()) << " v_published=" << d.v_published << "\n";
            ok = false;
        }
    }

    // ==== (2) All-satisfied entry, both directions. ====
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(0.0, /*v_des=*/0.03, /*ref_vel_now=*/0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint,
                      /*position_error=*/0.06);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch || !d.override_active ||
            !near_eq(d.v_published, base, 1e-9) || !d.derive_accel_from_rest) {
            std::cerr << "    (2) forward: expected CORRECTION-LAUNCH KICK to +base=" << base
                       << ", got state=" << static_cast<int>(gov.state()) << " v_published="
                       << d.v_published << "\n";
            ok = false;
        }
    }
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision d =
            gov.step(0.0, /*v_des=*/-0.03, /*ref_vel_now=*/0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint,
                      /*position_error=*/0.06);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch || !d.override_active ||
            !near_eq(d.v_published, -base, 1e-9) || !d.derive_accel_from_rest) {
            std::cerr << "    (2) reverse: expected CORRECTION-LAUNCH KICK to -base=" << -base
                       << ", got state=" << static_cast<int>(gov.state()) << " v_published="
                       << d.v_published << "\n";
            ok = false;
        }
    }

    // ==== (3) Escalation skipped (flat repeated kick), then a clean
    // give-up timeout. ====
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision entry =
            gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint, 0.06);
        if (!near_eq(entry.v_published, base, 1e-9)) {
            std::cerr << "    (3) entry: expected KICK to base=" << base << ", got "
                       << entry.v_published << "\n";
            ok = false;
        }
        // Probe several elapsed times a PLAIN launch would have escalated at
        // (kEscalationIntervalS=0.5s steps) -- v_meas stays 0 (never
        // confirms), so this sojourn stays in LAUNCH throughout, and
        // v_published must stay FLAT at base every time, never escalating.
        for (double elapsed : {0.1, 0.3, 0.6, 1.0, 1.4}) {
            mpc::LaunchGovernorDecision d =
                gov.step(0.0, 0.03, 0.0, base, spec, dt, elapsed, false, 0.0, kHint, 0.06);
            if (gov.state() != mpc::LaunchGovernorState::kLaunch || !near_eq(d.v_published, base, 1e-9)) {
                std::cerr << "    (3) at wall_elapsed_in_state=" << elapsed
                           << "s: expected a FLAT kick at base=" << base << " (no escalation), got "
                              "state=" << static_cast<int>(gov.state()) << " v_published="
                           << d.v_published << "\n";
                ok = false;
            }
        }
        // Now the give-up timeout: still no breakaway, elapsed >= kCorrectionLaunchTimeoutS.
        mpc::LaunchGovernorDecision give_up =
            gov.step(0.0, 0.03, 0.0, base, spec, dt, mpc::LaunchGovernor::kCorrectionLaunchTimeoutS,
                      false, 0.0, kHint, 0.06);
        if (gov.state() != mpc::LaunchGovernorState::kNormal || give_up.override_active ||
            !give_up.just_exited_launch || gov.settle_attempts_this_stop_event() != 1) {
            std::cerr << "    (3) give-up timeout: expected a transparent exit to NORMAL with "
                          "just_exited_launch=true and settle_attempts_this_stop_event()==1, got "
                          "state=" << static_cast<int>(gov.state()) << " override_active="
                       << give_up.override_active << " just_exited_launch="
                       << give_up.just_exited_launch << " attempts="
                       << gov.settle_attempts_this_stop_event() << "\n";
            ok = false;
        }
    }

    // ==== (4) Attempt count + cooldown reset on a NEW stop event. ====
    {
        mpc::LaunchGovernor gov;
        // Exhaust settle_max_attempts for "stop event A" (time_to_ref_stop
        // left at its sentinel default throughout -- no stop-event
        // transition detected yet).
        for (int attempt = 0; attempt < spec.settle_max_attempts; ++attempt) {
            mpc::LaunchGovernorDecision entry = gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false,
                                                           0.0, kHint, 0.06);
            gov.step(0.0, 0.03, 0.0, entry.v_published, spec, dt,
                      mpc::LaunchGovernor::kCorrectionLaunchTimeoutS, false, 0.0, kHint, 0.06);
            gov.step(0.0, 0.0, 0.0, 0.0, spec, /*dt=*/1.5, 0.0);
        }
        if (gov.settle_attempts_this_stop_event() != spec.settle_max_attempts) {
            std::cerr << "    (4) setup: expected attempts exhausted before the new stop event\n";
            ok = false;
        }
        // ONE call carrying BOTH the new-stop-event transition (time_to_ref_stop <=0, from the
        // sentinel default's implicit >0) AND a fresh entry attempt: the reset must apply BEFORE
        // this same tick's own entry check, so entry succeeds immediately -- no additional
        // cooldown wait needed, proving the cooldown baseline reset too (not just the count).
        mpc::LaunchGovernorDecision after_reset =
            gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0, /*time_to_ref_stop=*/-0.1, 0.06);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch || !near_eq(after_reset.v_published, base, 1e-9)) {
            std::cerr << "    (4) a NEW stop event (time_to_ref_stop<=0) should reset both the "
                          "attempt count and the cooldown baseline, allowing immediate re-entry; "
                          "got state=" << static_cast<int>(gov.state()) << " v_published="
                       << after_reset.v_published << "\n";
            ok = false;
        }
        if (gov.settle_attempts_this_stop_event() != 0) {
            std::cerr << "    (4) settle_attempts_this_stop_event() should read 0 immediately "
                          "after the reset (before this tick's own entry re-increments it on "
                          "exit), got " << gov.settle_attempts_this_stop_event() << "\n";
            ok = false;
        }
    }

    // ==== (5) reset() (new trajectory / RESUME) clears everything. ====
    {
        mpc::LaunchGovernor gov;
        mpc::LaunchGovernorDecision entry = gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0,
                                                       kHint, 0.06);
        gov.step(0.0, 0.03, 0.0, entry.v_published, spec, dt,
                  mpc::LaunchGovernor::kCorrectionLaunchTimeoutS, false, 0.0, kHint, 0.06);
        if (gov.settle_attempts_this_stop_event() != 1) {
            std::cerr << "    (5) setup: expected one attempt counted before reset()\n";
            ok = false;
        }
        gov.reset();
        if (gov.settle_attempts_this_stop_event() != 0 || gov.state() != mpc::LaunchGovernorState::kNormal) {
            std::cerr << "    (5) reset() should clear settle_attempts_this_stop_event() to 0 and "
                          "state() to kNormal, got attempts="
                       << gov.settle_attempts_this_stop_event() << " state="
                       << static_cast<int>(gov.state()) << "\n";
            ok = false;
        }
        // Immediate entry right after reset() must succeed (no stale cooldown baseline
        // surviving the reset).
        mpc::LaunchGovernorDecision after_reset_entry =
            gov.step(0.0, 0.03, 0.0, 0.0, spec, dt, 0.0, false, 0.0, kHint, 0.06);
        if (gov.state() != mpc::LaunchGovernorState::kLaunch ||
            !near_eq(after_reset_entry.v_published, base, 1e-9)) {
            std::cerr << "    (5) immediate entry after reset() should succeed (fresh cooldown "
                          "baseline), got state=" << static_cast<int>(gov.state()) << " v_published="
                       << after_reset_entry.v_published << "\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// EXTRACT_REF_STOP_EVENTS / TIME_TO_NEXT_REF_STOP: the pure, trajectory-
// agnostic hold-run scanner + per-tick lookup that main.cpp's arm-time
// precompute calls into (see LaunchGovernor.h's own doc comment on both
// functions for the full contract, and why they live there rather than in
// main.cpp -- a build-graph/testability choice, this test being exactly
// why). Operates purely on (time, effective-velocity) arrays; main.cpp's
// OWN derivation of that effective velocity from real waypoint (x,y)
// motion -- necessary because the raw trajectory_elem::ref_vel label never
// actually reaches zero across a real hold, see main.cpp's own comment at
// the arm-time call site -- is exercised end-to-end by test_mpc_deadband's
// Part F, not here.
// ---------------------------------------------------------------------
bool test_extract_ref_stop_events() {
    bool ok = true;

    // -- No holds: every segment cruises above threshold -- only the
    // trailing (unbounded) end-of-trajectory event is reported. --
    {
        const std::vector<double> times = {0.0, 1.0, 2.0, 3.0, 4.0};
        const std::vector<double> ref_vels = {0.2, 0.2, 0.2, 0.2, 0.0};
        auto events = mpc::extract_ref_stop_events(times, ref_vels);
        if (events.size() != 1 || !near_eq(events[0].start_time, 4.0, 1e-9) ||
            events[0].end_time < 1e6) {
            std::cerr << "    no-holds case: expected exactly one unbounded event at 4.0, got "
                       << events.size() << " events\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 1.5), 2.5, 1e-9)) {
            std::cerr << "    no-holds case: time_to_next_ref_stop(1.5) should be 2.5, got "
                       << mpc::time_to_next_ref_stop(events, 1.5) << "\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 4.0), 0.0, 1e-9)) {
            std::cerr << "    no-holds case: time_to_next_ref_stop(4.0) should be 0.0, got "
                       << mpc::time_to_next_ref_stop(events, 4.0) << "\n";
            ok = false;
        }
        if (mpc::time_to_next_ref_stop(events, 100.0) > 0.0) {
            std::cerr << "    no-holds case: time_to_next_ref_stop(100.0) (long past end) should "
                          "be <= 0, got " << mpc::time_to_next_ref_stop(events, 100.0) << "\n";
            ok = false;
        }
    }

    // -- Hold at start: the trajectory's very first segments are already
    // below threshold (e.g. a robot waiting before departing). --
    {
        const std::vector<double> times = {0.0, 0.5, 1.0, 4.0, 4.5};
        const std::vector<double> ref_vels = {0.0, 0.0, 0.2, 0.2, 0.0};
        auto events = mpc::extract_ref_stop_events(times, ref_vels);
        if (events.size() != 2 || !near_eq(events[0].start_time, 0.0, 1e-9) ||
            !near_eq(events[0].end_time, 1.0, 1e-9) ||
            !near_eq(events[1].start_time, 4.5, 1e-9) || events[1].end_time < 1e6) {
            std::cerr << "    hold-at-start case: expected events {0,1.0} then unbounded at 4.5, "
                          "got " << events.size() << " events\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 0.7), -0.7, 1e-9)) {
            std::cerr << "    hold-at-start case: time_to_next_ref_stop(0.7) (inside the hold) "
                          "should be -0.7, got " << mpc::time_to_next_ref_stop(events, 0.7) << "\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 2.0), 2.5, 1e-9)) {
            std::cerr << "    hold-at-start case: time_to_next_ref_stop(2.0) (cruising, between "
                          "the hold and the end) should be 2.5, got "
                       << mpc::time_to_next_ref_stop(events, 2.0) << "\n";
            ok = false;
        }
    }

    // -- Back-to-back holds: two logically-distinct low-velocity spans with
    // NO cruise segment between them merge into ONE event, since nothing
    // ever closes the run in between. --
    {
        const std::vector<double> times = {0.0, 1.0, 4.0, 7.0, 7.5, 10.0};
        const std::vector<double> ref_vels = {0.2, 0.0, 0.0, 0.2, 0.2, 0.0};
        auto events = mpc::extract_ref_stop_events(times, ref_vels);
        if (events.size() != 2 || !near_eq(events[0].start_time, 1.0, 1e-9) ||
            !near_eq(events[0].end_time, 7.0, 1e-9) ||
            !near_eq(events[1].start_time, 10.0, 1e-9) || events[1].end_time < 1e6) {
            std::cerr << "    back-to-back-holds case: expected a single MERGED event {1.0,7.0} "
                          "then unbounded at 10.0, got " << events.size() << " events\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 5.5), -4.5, 1e-9)) {
            std::cerr << "    back-to-back-holds case: time_to_next_ref_stop(5.5) should be "
                          "-4.5, got " << mpc::time_to_next_ref_stop(events, 5.5) << "\n";
            ok = false;
        }
    }

    // -- Trailing hold merging with the end: the low-velocity run reaches
    // all the way to the trajectory's own last waypoint (no cruise-out
    // afterward) -- NOT specially merged with the separately-appended final
    // event (see extract_ref_stop_events' doc comment for why that is
    // harmless), but coverage stays continuous (<= 0) across both and
    // beyond. --
    {
        const std::vector<double> times = {0.0, 1.0, 4.0, 7.0};
        const std::vector<double> ref_vels = {0.2, 0.0, 0.0, 0.0};
        auto events = mpc::extract_ref_stop_events(times, ref_vels);
        if (events.size() != 2 || !near_eq(events[0].start_time, 1.0, 1e-9) ||
            !near_eq(events[0].end_time, 7.0, 1e-9) ||
            !near_eq(events[1].start_time, 7.0, 1e-9) || events[1].end_time < 1e6) {
            std::cerr << "    trailing-hold case: expected {1.0,7.0} then unbounded at 7.0, got "
                       << events.size() << " events\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 5.0), -4.0, 1e-9)) {
            std::cerr << "    trailing-hold case: time_to_next_ref_stop(5.0) (inside the hold) "
                          "should be -4.0, got " << mpc::time_to_next_ref_stop(events, 5.0) << "\n";
            ok = false;
        }
        if (mpc::time_to_next_ref_stop(events, 7.0) > 0.0) {
            std::cerr << "    trailing-hold case: time_to_next_ref_stop(7.0) (exactly at the "
                          "merged end) should be <= 0, got "
                       << mpc::time_to_next_ref_stop(events, 7.0) << "\n";
            ok = false;
        }
        if (!near_eq(mpc::time_to_next_ref_stop(events, 8.0), -1.0, 1e-9)) {
            std::cerr << "    trailing-hold case: time_to_next_ref_stop(8.0) (past the "
                          "trajectory's own absolute end) should be -1.0 (still yielding via the "
                          "unbounded final event), got "
                       << mpc::time_to_next_ref_stop(events, 8.0) << "\n";
            ok = false;
        }
    }

    // -- Short dip below kRefStopMinHoldS (0.3s) is NOT reported as a hold
    // -- confirms the minimum-duration filter itself, not just that every
    // other case above happens to clear it comfortably. --
    {
        const std::vector<double> times = {0.0, 1.0, 1.2, 2.0};
        const std::vector<double> ref_vels = {0.2, 0.0, 0.2, 0.0};
        auto events = mpc::extract_ref_stop_events(times, ref_vels);
        if (events.size() != 1 || !near_eq(events[0].start_time, 2.0, 1e-9)) {
            std::cerr << "    short-dip case: a 0.2s dip (< kRefStopMinHoldS=0.3) should NOT be "
                          "reported as a hold -- expected only the unbounded end event at 2.0, "
                          "got " << events.size() << " events\n";
            ok = false;
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// SCHEDULE CATCH-UP: boosted_vr() unit tests. See MpcCore.h's doc comment
// on boosted_vr for the sign-convention derivation this exercises directly
// (in particular: NO extra sign(vr_nominal) factor, and clamping the SIGNED
// vr_prime rather than |vr_prime|).
// ---------------------------------------------------------------------
bool test_boosted_vr_catchup() {
    bool ok = true;
    const double vmax = 0.38;

    // Forward, behind: raised above nominal.
    {
        const double got = mpc::boosted_vr(0.2, 0.1, 1.0, vmax);
        if (!near_eq(got, 0.3, 1e-9)) {
            std::cerr << "    forward-behind: expected 0.3, got " << got << "\n";
            ok = false;
        }
    }

    // Forward, behind a lot: capped at vmax_stage.
    {
        const double got = mpc::boosted_vr(0.2, 1.0, 1.0, vmax);
        if (!near_eq(got, vmax, 1e-9)) {
            std::cerr << "    forward-behind-saturating: expected cap " << vmax << ", got " << got
                       << "\n";
            ok = false;
        }
    }

    // Forward, ahead: eased off, floored at half nominal.
    {
        const double got = mpc::boosted_vr(0.2, -0.1, 1.0, vmax);
        if (!near_eq(got, 0.1, 1e-9)) {
            std::cerr << "    forward-ahead: expected floor 0.1, got " << got << "\n";
            ok = false;
        }
    }

    // Forward, ahead EXTREME: still floored at half nominal, NOT snapped
    // back up to vmax_stage -- the clamp-order bug this case guards
    // against: clamping |vr_prime| and reapplying dir's sign afterward
    // would incorrectly produce +vmax here, since vr_prime = 0.2 +
    // 1.0*(-5.0) = -4.8 has |vr_prime| = 4.8 >> vmax.
    {
        const double got = mpc::boosted_vr(0.2, -5.0, 1.0, vmax);
        if (!near_eq(got, 0.1, 1e-9)) {
            std::cerr << "    forward-ahead-extreme: expected floor 0.1 (NOT snapped to vmax), "
                          "got " << got << "\n";
            ok = false;
        }
    }

    // Hold/stop stage: never boosted, regardless of e_lag (either sign).
    {
        const double got = mpc::boosted_vr(0.0, 100.0, 1.0, vmax);
        if (!near_eq(got, 0.0, 1e-12)) {
            std::cerr << "    hold-stage: expected exactly 0.0 regardless of e_lag, got " << got
                       << "\n";
            ok = false;
        }
    }
    {
        const double got = mpc::boosted_vr(0.0, -100.0, 1.0, vmax);
        if (!near_eq(got, 0.0, 1e-12)) {
            std::cerr << "    hold-stage (negative e_lag): expected exactly 0.0, got " << got
                       << "\n";
            ok = false;
        }
    }

    // On-schedule: e_lag=0 leaves the nominal target unchanged (the boost
    // is inert at near-zero lag -- relied on by the near-zero-lag
    // acceptance scenarios).
    {
        const double got = mpc::boosted_vr(0.2, 0.0, 1.0, vmax);
        if (!near_eq(got, 0.2, 1e-9)) {
            std::cerr << "    on-schedule: expected unchanged nominal 0.2, got " << got << "\n";
            ok = false;
        }
    }

    // REVERSE LEG, behind (e_lag < 0 -- see MpcCore.h's doc comment for the
    // derivation of why "behind" is NEGATIVE e_lag on a reverse leg):
    // catch-up means going FASTER in reverse, i.e. MORE negative, same
    // sign.
    {
        const double got = mpc::boosted_vr(-0.2, -0.1, 1.0, vmax);
        if (!near_eq(got, -0.3, 1e-9)) {
            std::cerr << "    reverse-behind: expected -0.3 (faster reverse), got " << got << "\n";
            ok = false;
        }
        if (!(got < -0.2)) {
            std::cerr << "    reverse-behind: expected MORE negative than nominal (-0.2), got "
                       << got << "\n";
            ok = false;
        }
    }

    // REVERSE LEG, behind a lot: capped at -vmax_stage (never exceeds the
    // magnitude cap, same as the forward case).
    {
        const double got = mpc::boosted_vr(-0.2, -1.0, 1.0, vmax);
        if (!near_eq(got, -vmax, 1e-9)) {
            std::cerr << "    reverse-behind-saturating: expected -" << vmax << ", got " << got
                       << "\n";
            ok = false;
        }
    }

    // REVERSE LEG, ahead (e_lag > 0 on a reverse leg): eases off toward
    // (but never past) half-nominal magnitude, and -- critically -- NEVER
    // FLIPS SIGN to positive/forward.
    {
        const double got = mpc::boosted_vr(-0.2, 0.1, 1.0, vmax);
        if (!near_eq(got, -0.1, 1e-9)) {
            std::cerr << "    reverse-ahead: expected floor -0.1, got " << got << "\n";
            ok = false;
        }
        if (!(got < 0.0)) {
            std::cerr << "    reverse-ahead: expected to STAY negative (reverse direction), got "
                       << got << "\n";
            ok = false;
        }
    }

    // REVERSE LEG, ahead EXTREME: the headline sign-correctness case. A
    // naive "vr' = vr + sign(vr)*K*e_lag" reading (see MpcCore.h's doc
    // comment for why that literal formula is WRONG) double-flips the
    // reverse direction; this case stresses the "ahead" side hard enough
    // (vr_prime = -0.2 + 1.0*5.0 = +4.8, a full sign flip to forward) to
    // catch that mistake: a correct implementation still reports a
    // NEGATIVE (reverse) result, floored at half nominal magnitude, never a
    // positive/forward one.
    {
        const double got = mpc::boosted_vr(-0.2, 5.0, 1.0, vmax);
        if (!near_eq(got, -0.1, 1e-9)) {
            std::cerr << "    reverse-ahead-extreme: expected floor -0.1 (NOT flipped to forward "
                          "or snapped to -vmax), got " << got << "\n";
            ok = false;
        }
        if (!(got < 0.0)) {
            std::cerr << "    reverse-ahead-extreme: expected to STAY negative, got " << got
                       << "\n";
            ok = false;
        }
    }

    // K scales the correction linearly.
    {
        const double got_k1 = mpc::boosted_vr(0.2, 0.05, 1.0, vmax);
        const double got_k2 = mpc::boosted_vr(0.2, 0.05, 2.0, vmax);
        if (!near_eq(got_k1, 0.25, 1e-9) || !near_eq(got_k2, 0.3, 1e-9)) {
            std::cerr << "    K scaling: expected 0.25 (K=1) and 0.30 (K=2), got " << got_k1
                       << " and " << got_k2 << "\n";
            ok = false;
        }
    }

    // Cap tracks the CALLER-supplied vmax_stage, not a hardcoded value --
    // main.cpp passes a different cap per stage depending on
    // RefState::is_pushing.
    {
        const double got = mpc::boosted_vr(0.08, 1.0, 1.0, /*vmax_stage=*/0.1);
        if (!near_eq(got, 0.1, 1e-9)) {
            std::cerr << "    per-stage vmax cap: expected cap 0.1, got " << got << "\n";
            ok = false;
        }
    }

    return ok;
}

}  // namespace

int main() {
    using TestFn = std::function<bool()>;
    const std::vector<std::pair<std::string, TestFn>> tests = {
        {"RobotSpec/MpcParams defaults match MARS values", test_defaults_match_mars_values},
        {"JSON partial override / invalid path throws / unknown key warns",
         test_json_partial_override_and_errors},
        {"RobotSpec::validate() default OK / bad turning radius fails", test_validate},
        {"get_ref_state_at_time: midpoint/clamp/yaw-wrap/hold/empty", test_get_ref_state_at_time},
        {"get_ref_state_at_time: terminal ref_vel zeroing past trajectory end (before/at/past "
         "boundary)",
         test_ref_vel_zero_past_trajectory_end},
        {"rollout_step (shared kinematics): straight-line exactness + curved radius",
         test_rollout_accel_model},
        {"normalize_angle wraps to (-pi, pi]", test_normalize_angle},
        {"predict_delay_compensated: control_delay_steps knob has an effect",
         test_predict_delay_compensated},
        {"solve on-track: tiny lateral error, v_cmd near ref_v", test_solve_on_track},
        {"solve catch-up: v_cmd > 0 and implied accel bounded by max_accel",
         test_solve_catch_up},
        {"solve bounds: accel saturates at max_accel; steer never exceeds max_steer",
         test_solve_bounds},
        {"solve v-cap push: v_cmd stays within max_v_push", test_solve_vcap_push},
        {"solve v-cap transition: push cap does not leak into the next non-push solve",
         test_solve_vcap_transition},
        {"solve stage-vmax: box bound enforced at every horizon stage, not just stage 0",
         test_solve_stage_vmax},
        {"solve no warm start (use_warm_start=false, default): repeating a scenario after a "
         "different one reproduces it exactly",
         test_no_warm_start_across_calls},
        {"solve stop-accel: decelerates to rest, monotonic and bounded by max_accel",
         test_solve_stop_accel},
        {"acceleration-formulation payload identity: accel*dt == v_cmd - v0 exactly",
         test_accel_formulation_payload_identity},
        {"solve tracks a signed-negative ref_vel (reversal) with v_cmd < 0",
         test_solve_tracks_signed_reversal},
        {"resolve_dir_change: asymmetric sign-flip weight bump + change_dir flag",
         test_dir_change_resolution},
        {"build_payload: always-scaled speed (no hw switch) + dir-change scaling",
         test_build_payload_identities},
        {"Empty-trajectory completion path (helper level)",
         test_empty_trajectory_completion_helpers},
        {"S1 sim step parity with mpc::rollout_step (straight/curved/reverse/zero-v)",
         test_sim_step_parity_with_rollout_step},
        {"S2 sim closed-form kinematics (straight recurrence + curved radius)",
         test_sim_closed_form_kinematics},
        {"S3 watchdog: engage/monotonic-brake/hold-steering/recover",
         test_watchdog_engage_brake_recover},
        {"S4 ackermann decode: round-trip + malformed/empty/unknown-key handling",
         test_ackermann_decode_roundtrip},
        {"S5 localization encode: accepted by controller's actual parse path",
         test_localization_encode_matches_controller_parse},
        {"S6 clamp_command: accel/steering clamped, effect visible in state update",
         test_clamp_command},
        {"N1 FEATURE A: sigma=0 -> applied == clamped command exactly (bitwise)",
         test_noise_sigma_zero_bit_identical},
        {"N2 FEATURE A: noise statistics (mean~0, std~sigma_pct*limit) over 2000 samples",
         test_noise_statistics},
        {"N3 FEATURE A: perturbation held across ticks, resampled on each new command",
         test_noise_held_until_new_command},
        {"N4 FEATURE A: clamp-after-noise never exceeds actuator limits",
         test_noise_clamp_after},
        {"N5 FEATURE A: sim_config payload parsing (valid/out-of-range-clamped/malformed-retained)",
         test_sim_config_payload_parsing},
        {"FEATURE 2A P1: compute_effective_elapsed (unpaused/frozen/monotonic across cycles)",
         test_compute_effective_elapsed},
        {"FEATURE 2A P2: MpcSolver::reset_warm_start() debug hook invalidates the stored horizon",
         test_warm_start_reset_hook},
        {"FEATURE 2A P3: opt-in warm start -- shifted-seed exactness + output continuity across "
         "consecutive live-loop-style solves",
         test_warm_start_shifted_seed_and_continuity},
        {"D1 FEATURE B: deadband state machine (frozen/jump/hysteresis/stall/pose-frozen)",
         test_deadband_state_machine},
        {"D2 FEATURE B: deadband-disabled-equivalent pipeline matches rollout_step bitwise over a "
         "command sweep",
         test_deadband_disabled_matches_rollout_step_sweep},
        {"D3 FEATURE B: RobotSpec deadband fields load/override + validate() incl. pushing-margin "
         "warning",
         test_robot_spec_deadband_fields_and_validate},
        {"D4 FEATURE B: telemetry payload encode/parse round-trip incl. moving/watchdog flags",
         test_telemetry_encode_parse_roundtrip},
        {"D5a FEATURE B2: live sim_config deadband fields -- parsing/partial/malformed-per-field/"
         "clamping",
         test_sim_config_deadband_fields_parsing},
        {"D5b FEATURE B2: live sim_config state machine -- enable-while-moving keeps moving, "
         "disable-while-stalled releases freeze",
         test_sim_config_deadband_live_toggle_state_machine},
        {"D6 BREAKAWAY REALISM: resolve_accel_clamp_ceiling (stalled vs moving, deadband on/off)",
         test_breakaway_accel_clamp_ceiling},
        {"G1 LAUNCH GOVERNOR: state transitions (entry magnitude/sign, exit threshold, reversal, "
         "stop-snap, no-launch-when-already-fast)",
         test_launch_governor_state_transitions},
        {"G2 LAUNCH GOVERNOR: escalation timing (0.5s steps, 1.1x, cap, warn cadence; injected "
         "elapsed times, no sleeps)",
         test_launch_governor_escalation_timing},
        {"G3 LAUNCH GOVERNOR: inactive (permanently transparent) when min_moving_speed == 0",
         test_launch_governor_inactive_when_no_deadband},
        {"G4 LAUNCH GOVERNOR: FAST EXIT raw-sample debounce (2-consecutive, reset-on-miss, "
         "fallback-compatible)",
         test_launch_governor_raw_exit_debounce},
        {"G5 LAUNCH GOVERNOR: just_exited_launch flag contract (observer injection trigger)",
         test_launch_governor_just_exited_launch_flag},
        {"G6 LAUNCH GOVERNOR: ANTI-RE-STALL moving-floor (fires/guards, never during LAUNCH/"
         "STOP-SNAP)",
         test_launch_governor_moving_floor},
        {"G7 LAUNCH GOVERNOR: moving-floor yields within kFloorYieldHorizonS of reference end "
         "(boundary, negative, far-away, sentinel-default, still-rate-limited)",
         test_launch_governor_floor_yield_near_ref_end},
        {"G8 LAUNCH GOVERNOR: moving-floor yields near an INTERMEDIATE HOLD (approach/enter/"
         "inside/boundary/resume)",
         test_launch_governor_floor_yield_near_hold},
        {"G9 LAUNCH GOVERNOR: CORRECTION-LAUNCH (finish-at-rest) -- guard truth table, both "
         "directions, escalation-skip + give-up timeout, per-stop-event reset, reset()",
         test_launch_governor_correction_launch},
        {"extract_ref_stop_events/time_to_next_ref_stop: hold-run scanner + lookup (no holds/"
         "hold-at-start/back-to-back/trailing-merges-with-end/short-dip-filtered)",
         test_extract_ref_stop_events},
        {"SCHEDULE CATCH-UP: boosted_vr (behind/ahead/hold/on-schedule/reverse-leg sign "
         "correctness/extreme-clamp/K-scaling/per-stage vmax cap)",
         test_boosted_vr_catchup},
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
