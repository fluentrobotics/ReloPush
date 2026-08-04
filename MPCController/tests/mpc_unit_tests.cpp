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

    // Clamp before-start / after-end.
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
        if (!after.ok || !near_eq(after.x, 2.0) || !near_eq(after.ref_vel, 1.0)) {
            std::cerr << "    after-end clamp failed: ok=" << after.ok << " x=" << after.x << "\n";
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
// encode_field_like_controller() replicates MPCController/src/main.cpp's
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

}  // namespace

int main() {
    using TestFn = std::function<bool()>;
    const std::vector<std::pair<std::string, TestFn>> tests = {
        {"RobotSpec/MpcParams defaults match MARS values", test_defaults_match_mars_values},
        {"JSON partial override / invalid path throws / unknown key warns",
         test_json_partial_override_and_errors},
        {"RobotSpec::validate() default OK / bad turning radius fails", test_validate},
        {"get_ref_state_at_time: midpoint/clamp/yaw-wrap/hold/empty", test_get_ref_state_at_time},
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
