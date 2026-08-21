// Lightweight unit tests for mpc:: MotorCalibCore (VelocityEstimator,
// LineFrame, the TrialRunner state machine, the fitters, export/parse of
// the frozen calibration schema, and the motor_calib_config.json loader),
// mirroring the harness style of MPC/tests/mpc_unit_tests.cpp: plain bool
// test_xxx() functions registered in main(), no gtest dependency, no
// sockets, no spawned processes.

#include "mpc/MotorCalibCore.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

bool near_eq(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

// ---------------------------------------------------------------------
// Fake IDriverClient: scriptable, records every call for assertions (raw()
// call timestamps/values, set_source()/servo()/stop() call counts).
// ---------------------------------------------------------------------
class FakeDriverClient : public mpc::IDriverClient {
public:
    bool fail_set_source = false;
    bool fail_raw = false;

    double now_ptr_value = 0.0;  // test sets this right before each step() call

    int set_source_call_count = 0;
    std::string last_set_source;
    int servo_call_count = 0;
    double last_servo_value = 0.0;
    int stop_call_count = 0;

    int raw_call_count = 0;
    std::vector<double> raw_call_times;
    std::vector<std::pair<mpc::RawMode, double>> raw_calls;

    mpc::DriverReply ping() override { return {true, "", 0.0}; }

    mpc::DriverReply set_source(const std::string& source) override {
        ++set_source_call_count;
        last_set_source = source;
        if (fail_set_source) return {false, "refused", 0.0};
        return {true, "", 0.0};
    }

    mpc::DriverReply raw(mpc::RawMode mode, double value, double ttl_ms) override {
        (void)ttl_ms;
        ++raw_call_count;
        raw_call_times.push_back(now_ptr_value);
        raw_calls.emplace_back(mode, value);
        if (fail_raw) return {false, "refused", 0.0};
        return {true, "", value};
    }

    mpc::DriverReply servo(double value) override {
        ++servo_call_count;
        last_servo_value = value;
        return {true, "", 0.0};
    }

    mpc::DriverReply stop() override {
        ++stop_call_count;
        return {true, "", 0.0};
    }
};

// ---------------------------------------------------------------------
// VelocityEstimator
// ---------------------------------------------------------------------

bool test_velocity_estimator_raw_straight_line_and_reversal() {
    bool ok = true;
    mpc::VelocityEstimator est;
    const double dt = 0.05;

    mpc::VelocityEstimate e0 = est.feed(0.0, 0.0, 0.0, 0.0);
    if (e0.v_raw_valid) {
        std::cerr << "    first sample should have v_raw_valid=false\n";
        ok = false;
    }

    // Constant-velocity straight line along +x, heading=0: v_raw == v_const
    // exactly (matches main.cpp's finite-difference formula).
    const double v_const = 0.3;
    double x = 0.0;
    for (int i = 1; i <= 10; ++i) {
        x += v_const * dt;
        mpc::VelocityEstimate e = est.feed(i * dt, x, 0.0, 0.0);
        if (!e.v_raw_valid) {
            std::cerr << "    v_raw_valid should be true from the 2nd sample onward\n";
            ok = false;
            break;
        }
        if (!near_eq(e.v_raw, v_const, 1e-9)) {
            std::cerr << "    v_raw=" << e.v_raw << " expected " << v_const << " at i=" << i << "\n";
            ok = false;
        }
    }

    // Too-close-together dt (<=1e-3): must retain the previous v_raw, even
    // with a huge dx, exactly like MPC/src/main.cpp's est_state.v retention.
    mpc::VelocityEstimate tiny_dt = est.feed(10 * dt + 0.0001, x + 100.0, 0.0, 0.0);
    if (!near_eq(tiny_dt.v_raw, v_const, 1e-9)) {
        std::cerr << "    tiny-dt sample should retain previous v_raw, got " << tiny_dt.v_raw << "\n";
        ok = false;
    }

    // Reversal: heading held at 0 while position reverses -> v_raw goes
    // negative (matches "a genuine reversal yields a negative estimate").
    mpc::VelocityEstimator est2;
    est2.feed(0.0, 0.0, 0.0, 0.0);
    est2.feed(0.1, 0.03, 0.0, 0.0);
    mpc::VelocityEstimate rev = est2.feed(0.2, 0.0, 0.0, 0.0);
    if (rev.v_raw >= 0.0) {
        std::cerr << "    expected negative v_raw on reversal, got " << rev.v_raw << "\n";
        ok = false;
    }

    return ok;
}

bool test_velocity_estimator_smoothing_window_and_accel() {
    bool ok = true;
    mpc::VelocityEstimator est;
    const double dt = 0.1;
    const double A = 0.2, w = 1.0;  // x(t) = A*sin(w*t), synthetic sinusoid trace

    std::vector<double> ts;
    std::vector<mpc::VelocityEstimate> outs;
    const int n = 12;
    for (int i = 0; i < n; ++i) {
        const double t = i * dt;
        const double x = A * std::sin(w * t);
        outs.push_back(est.feed(t, x, 0.0, 0.0));
        ts.push_back(t);
    }

    for (int i = 0; i < n; ++i) {
        const bool expect_smooth = (i >= 4);
        if (outs[i].smooth_valid != expect_smooth) {
            std::cerr << "    smooth_valid mismatch at i=" << i << ": got " << outs[i].smooth_valid
                       << " expected " << expect_smooth << "\n";
            ok = false;
        }
        const bool expect_a = (i >= 5);
        if (outs[i].a_valid != expect_a) {
            std::cerr << "    a_valid mismatch at i=" << i << ": got " << outs[i].a_valid
                       << " expected " << expect_a << "\n";
            ok = false;
        }
    }

    if (outs[4].smooth_valid) {
        double manual_mean = 0.0;
        for (int k = 0; k < 5; ++k) manual_mean += outs[k].v_raw;
        manual_mean /= 5.0;
        if (!near_eq(outs[4].v_smooth, manual_mean, 1e-9)) {
            std::cerr << "    v_smooth mismatch: got " << outs[4].v_smooth << " expected "
                       << manual_mean << "\n";
            ok = false;
        }
        if (!near_eq(outs[4].t_smooth, ts[2], 1e-9)) {
            std::cerr << "    t_smooth mismatch: got " << outs[4].t_smooth << " expected " << ts[2]
                       << "\n";
            ok = false;
        }
    } else {
        ok = false;
    }

    if (outs[5].a_valid) {
        const double expected_a =
            (outs[5].v_smooth - outs[4].v_smooth) / (outs[5].t_smooth - outs[4].t_smooth);
        if (!near_eq(outs[5].a_smooth, expected_a, 1e-9)) {
            std::cerr << "    a_smooth mismatch: got " << outs[5].a_smooth << " expected "
                       << expected_a << "\n";
            ok = false;
        }
    } else {
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// LineFrame
// ---------------------------------------------------------------------

bool test_line_frame() {
    bool ok = true;
    mpc::LineFrame lf;
    if (lf.captured()) {
        std::cerr << "    should not be captured initially\n";
        ok = false;
    }

    const double yaw = 0.7853981633974483;  // pi/4
    lf.capture(1.0, 2.0, yaw);
    if (!lf.captured()) ok = false;
    if (!near_eq(lf.x0(), 1.0, 1e-9) || !near_eq(lf.y0(), 2.0, 1e-9)) ok = false;
    if (!near_eq(lf.line_yaw(), yaw, 1e-9)) ok = false;

    if (!near_eq(lf.s(1.0, 2.0), 0.0, 1e-9) || !near_eq(lf.lateral(1.0, 2.0), 0.0, 1e-9)) {
        std::cerr << "    s/lateral at the origin itself should both be 0\n";
        ok = false;
    }

    const double c = std::cos(yaw), sn = std::sin(yaw);
    const double px = 1.0 + c, py = 2.0 + sn;
    if (!near_eq(lf.s(px, py), 1.0, 1e-6) || !near_eq(lf.lateral(px, py), 0.0, 1e-6)) {
        std::cerr << "    on-axis point: s=" << lf.s(px, py) << " lateral=" << lf.lateral(px, py)
                   << "\n";
        ok = false;
    }
    const double qx = 1.0 - sn, qy = 2.0 + c;
    if (!near_eq(lf.s(qx, qy), 0.0, 1e-6) || !near_eq(lf.lateral(qx, qy), 1.0, 1e-6)) {
        std::cerr << "    on-normal point: s=" << lf.s(qx, qy) << " lateral=" << lf.lateral(qx, qy)
                   << "\n";
        ok = false;
    }

    lf.capture(9.0, 9.0, 0.0);  // no force -> no-op
    if (!near_eq(lf.x0(), 1.0, 1e-9)) {
        std::cerr << "    capture() without force should be a no-op once already captured\n";
        ok = false;
    }
    lf.capture(9.0, 9.0, 0.0, std::nullopt, /*force=*/true);
    if (!near_eq(lf.x0(), 9.0, 1e-9) || !near_eq(lf.line_yaw(), 0.0, 1e-9)) {
        std::cerr << "    capture(force=true) should re-capture\n";
        ok = false;
    }

    mpc::LineFrame lf2;
    lf2.capture(0.0, 0.0, 0.5, 1.0);
    if (!near_eq(lf2.line_yaw(), 1.0, 1e-9)) {
        std::cerr << "    explicit yaw_override should win over initial_yaw\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// TrialRunner
// ---------------------------------------------------------------------

bool test_trial_runner_normal_completion_and_ttl_cadence() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;  // all compiled defaults
    mpc::TrialRunner runner(driver, cfg);

    const double dt = 0.02;
    const double k = 3.0, c0 = 0.03, tau = 0.2;  // synthetic first-order plant, see fit_grid tests too
    double now = 0.0;
    double x = 0.0;
    double v = 0.0;

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.06;
    spec.ttl_ms = 500.0;
    runner.start_trial(spec);

    bool reached_run = false, reached_stopping = false;
    int iterations = 0;
    const int max_iterations = 5000;
    while (runner.state() != mpc::TrialState::kDone && runner.state() != mpc::TrialState::kAborted &&
           iterations < max_iterations) {
        mpc::CalibSample pose{now, x, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});

        if (runner.state() == mpc::TrialState::kRun) reached_run = true;
        if (runner.state() == mpc::TrialState::kStopping) reached_stopping = true;

        double target = 0.0;
        if (runner.state() == mpc::TrialState::kRun) {
            const double mag = std::fabs(spec.value);
            if (mag > c0) target = (spec.value >= 0.0 ? 1.0 : -1.0) * k * (mag - c0);
        }
        v += (target - v) / tau * dt;
        x += v * dt;
        now += dt;
        ++iterations;
    }

    if (runner.state() != mpc::TrialState::kDone) {
        std::cerr << "    expected kDone, got state=" << static_cast<int>(runner.state())
                   << " abort_reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    if (!reached_run) {
        std::cerr << "    never reached kRun\n";
        ok = false;
    }
    if (!reached_stopping) {
        std::cerr << "    never reached kStopping\n";
        ok = false;
    }
    if (runner.rows().empty()) {
        std::cerr << "    expected non-empty rows()\n";
        ok = false;
    }
    for (const auto& r : runner.rows()) {
        if (!near_eq(r.cmd_value, spec.value, 1e-9) || r.cmd_mode != spec.mode) {
            std::cerr << "    logged row's cmd does not match the trial spec\n";
            ok = false;
            break;
        }
    }

    if (driver.raw_call_times.size() < 3) {
        std::cerr << "    expected several raw() TTL-keepalive calls, got "
                   << driver.raw_call_times.size() << "\n";
        ok = false;
    } else {
        for (std::size_t i = 1; i < driver.raw_call_times.size(); ++i) {
            const double gap = driver.raw_call_times[i] - driver.raw_call_times[i - 1];
            if (gap < cfg.reissue_interval_s - 1e-9 || gap > cfg.reissue_interval_s + 3 * dt) {
                std::cerr << "    raw() reissue gap out of expected ~" << cfg.reissue_interval_s
                           << "s range: got " << gap << "\n";
                ok = false;
            }
        }
    }
    for (const auto& c : driver.raw_calls) {
        if (c.first != spec.mode || !near_eq(c.second, spec.value, 1e-9)) {
            std::cerr << "    a raw() call did not carry the trial's constant command\n";
            ok = false;
            break;
        }
    }
    if (driver.set_source_call_count < 1 || driver.last_set_source != "calib") {
        std::cerr << "    expected set_source(\"calib\") to be called once at RUN entry\n";
        ok = false;
    }
    if (driver.servo_call_count < 1) {
        std::cerr << "    expected servo() to be called at RUN entry\n";
        ok = false;
    }
    if (driver.stop_call_count < 1) {
        std::cerr << "    expected stop() to be called entering STOPPING\n";
        ok = false;
    }

    return ok;
}

bool test_trial_runner_abort_mocap_stale() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;  // mocap_stale_abort_ms = 300 (default)
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.05;
    runner.start_trial(spec);

    mpc::CalibSample pose{0.0, 0.0, 0.0, 0.0};
    driver.now_ptr_value = 0.0;
    runner.step(0.0, true, pose, false, mpc::TelemetrySample{});  // seed one pose

    double now = 0.05;
    bool aborted = false;
    for (int i = 0; i < 20 && !aborted; ++i) {
        runner.step(now, false, pose, false, mpc::TelemetrySample{});  // no fresh pose from here on
        if (runner.state() == mpc::TrialState::kAborted) aborted = true;
        now += 0.05;
    }
    if (!aborted || runner.abort_reason() != mpc::TrialAbortReason::kMocapStale) {
        std::cerr << "    expected kMocapStale abort, got state=" << static_cast<int>(runner.state())
                   << " reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    return ok;
}

bool test_trial_runner_abort_over_speed() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kErpm;
    spec.value = 2000.0;
    runner.start_trial(spec);

    double now = 0.0;
    const double dt = 0.02;
    for (int i = 0; i < 80; ++i) {
        mpc::CalibSample pose{now, 0.0, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kRun) break;
    }
    if (runner.state() != mpc::TrialState::kRun) {
        std::cerr << "    failed to reach kRun before the over-speed probe\n";
        return false;
    }

    mpc::CalibSample jump_pose{now, 5.0, 0.0, 0.0};  // huge jump -> huge v_raw
    driver.now_ptr_value = now;
    runner.step(now, true, jump_pose, false, mpc::TelemetrySample{});
    if (runner.state() != mpc::TrialState::kAborted ||
        runner.abort_reason() != mpc::TrialAbortReason::kOverSpeed) {
        std::cerr << "    expected kOverSpeed abort, got state=" << static_cast<int>(runner.state())
                   << " reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    return ok;
}

bool test_trial_runner_abort_out_of_envelope() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;
    cfg.line_length_m = 1.0;
    cfg.end_margin_m = 0.1;  // envelope_max = 0.9 from s0=0
    cfg.settle_s = 100.0;    // never let normal STOPPING preempt this test
    cfg.max_duration_s = 100.0;
    cfg.max_speed_abort_mps = 5.0;  // generous -- isolate the envelope check
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.05;
    runner.start_trial(spec);

    double now = 0.0, x = 0.0;
    const double dt = 0.02;
    for (int i = 0; i < 80; ++i) {
        mpc::CalibSample pose{now, x, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kRun) break;
    }
    if (runner.state() != mpc::TrialState::kRun) {
        std::cerr << "    failed to reach kRun before the envelope probe\n";
        return false;
    }

    const double v_const = 0.5;  // well under max_speed_abort_mps=5.0
    bool aborted = false;
    for (int i = 0; i < 500 && !aborted; ++i) {
        x += v_const * dt;
        mpc::CalibSample pose{now, x, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kAborted) aborted = true;
    }
    if (!aborted || runner.abort_reason() != mpc::TrialAbortReason::kOutOfEnvelope) {
        std::cerr << "    expected kOutOfEnvelope abort, got state=" << static_cast<int>(runner.state())
                   << " reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    return ok;
}

// Regression (round-0 finding, MPC/src/MotorCalibCore.cpp TrialRunner::step()
// RUN-phase envelope check): motion OPPOSITE the commanded direction (e.g.
// reversed motor polarity, or rollback on a slope) must also trip
// kOutOfEnvelope once it exceeds the room available on THAT side
// (room_behind_) -- not just an excursion past room_ahead_ in the commanded
// direction (see test_trial_runner_abort_out_of_envelope above, which only
// covers that side). Arms at s0=0.4 (the MIDDLE of the [0.1,0.9] envelope,
// deliberately away from either edge) with a POSITIVE (forward) commanded
// value, then drives the pose BACKWARD -- the commanded-direction check
// (progress > room_ahead_) never fires here since progress stays negative
// throughout; only the wrong-direction check can catch this.
bool test_trial_runner_abort_out_of_envelope_wrong_direction() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;
    cfg.line_length_m = 1.0;
    cfg.end_margin_m = 0.1;  // envelope == [0.1, 0.9]
    cfg.settle_s = 100.0;    // never let normal STOPPING preempt this test
    cfg.max_duration_s = 100.0;
    cfg.max_speed_abort_mps = 5.0;  // generous -- isolate the envelope check
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.05;  // POSITIVE (forward) commanded direction.
    runner.start_trial(spec);

    double now = 0.0;
    const double x0 = 0.4;  // middle of [0.1,0.9]: room_ahead_=0.5, room_behind_=0.3.
    double x = x0;
    const double dt = 0.02;
    for (int i = 0; i < 80; ++i) {
        mpc::CalibSample pose{now, x, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kRun) break;
    }
    if (runner.state() != mpc::TrialState::kRun) {
        std::cerr << "    failed to reach kRun before the wrong-direction envelope probe\n";
        return false;
    }

    // Reversed motor polarity: the commanded direction is positive, but the
    // vehicle actually moves NEGATIVE. x0 - room_behind_ = 0.4 - 0.3 = 0.1
    // -- must abort once x drops below that, well before it could ever
    // reach the (irrelevant, on this side) forward envelope edge at 0.9.
    const double v_const = -0.5;  // well under max_speed_abort_mps=5.0 in magnitude.
    bool aborted = false;
    for (int i = 0; i < 500 && !aborted; ++i) {
        x += v_const * dt;
        mpc::CalibSample pose{now, x, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kAborted) aborted = true;
    }
    if (!aborted || runner.abort_reason() != mpc::TrialAbortReason::kOutOfEnvelope) {
        std::cerr << "    expected kOutOfEnvelope abort (wrong-direction excursion), got state="
                   << static_cast<int>(runner.state()) << " reason=" << mpc::to_string(runner.abort_reason())
                   << " (final x=" << x << ", started at x0=" << x0 << ")\n";
        ok = false;
    }
    // The vehicle must never have been allowed to travel anywhere near the
    // FORWARD edge (0.9) or even back past its own start -- it aborted
    // promptly once it exceeded the room available BEHIND the start point,
    // confirming this is genuinely the wrong-direction check firing (not
    // some other abort reason/timeout papering over an unbounded excursion).
    if (x > x0) {
        std::cerr << "    test assumption broken: x=" << x << " ended up forward of x0=" << x0 << "\n";
        ok = false;
    }
    if (x < -1.0) {
        std::cerr << "    excursion was NOT bounded close to the envelope: x=" << x
                   << " traveled far past the line's own physical extent -- the wrong-direction check "
                      "did not trip promptly\n";
        ok = false;
    }
    return ok;
}

bool test_trial_runner_abort_timeout() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;
    cfg.settle_s = 1000.0;     // never let normal STOPPING preempt this test
    cfg.max_duration_s = 0.5;  // short, so the test runs fast
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.05;
    runner.start_trial(spec);

    double now = 0.0;
    const double dt = 0.02;
    bool aborted = false;
    for (int i = 0; i < 200 && !aborted; ++i) {
        mpc::CalibSample pose{now, 0.0, 0.0, 0.0};  // stationary throughout -- isolates the timeout
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kAborted) aborted = true;
    }
    if (!aborted || runner.abort_reason() != mpc::TrialAbortReason::kTimeout) {
        std::cerr << "    expected kTimeout abort, got state=" << static_cast<int>(runner.state())
                   << " reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    return ok;
}

bool test_trial_runner_abort_insufficient_room_at_arm() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;
    cfg.line_length_m = 1.0;
    cfg.end_margin_m = 1.0;  // envelope collapses to zero/negative room from s0=0 in EITHER direction
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.05;
    runner.start_trial(spec);

    double now = 0.0;
    const double dt = 0.02;
    bool aborted = false;
    for (int i = 0; i < 80 && !aborted; ++i) {
        mpc::CalibSample pose{now, 0.0, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kAborted) aborted = true;
    }
    if (!aborted || runner.abort_reason() != mpc::TrialAbortReason::kInsufficientRoomAtArm) {
        std::cerr << "    expected kInsufficientRoomAtArm abort, got state="
                   << static_cast<int>(runner.state())
                   << " reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    if (driver.raw_call_count != 0) {
        std::cerr << "    expected zero raw() calls -- must abort at ARM, before RUN begins\n";
        ok = false;
    }
    return ok;
}

bool test_trial_runner_abort_driver_refused() {
    bool ok = true;
    FakeDriverClient driver;
    driver.fail_set_source = true;
    mpc::TrialRunnerConfig cfg;
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kDuty;
    spec.value = 0.05;
    runner.start_trial(spec);

    double now = 0.0;
    const double dt = 0.02;
    bool aborted = false;
    for (int i = 0; i < 80 && !aborted; ++i) {
        mpc::CalibSample pose{now, 0.0, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kAborted) aborted = true;
    }
    if (!aborted || runner.abort_reason() != mpc::TrialAbortReason::kDriverRefused) {
        std::cerr << "    expected kDriverRefused abort, got state=" << static_cast<int>(runner.state())
                   << " reason=" << mpc::to_string(runner.abort_reason()) << "\n";
        ok = false;
    }
    if (driver.raw_call_count != 0) {
        std::cerr << "    expected zero raw() calls when set_source(calib) is refused\n";
        ok = false;
    }
    return ok;
}

bool test_trial_runner_csv_writer() {
    bool ok = true;
    FakeDriverClient driver;
    mpc::TrialRunnerConfig cfg;
    cfg.settle_s = 1000.0;
    cfg.max_duration_s = 1000.0;
    mpc::TrialRunner runner(driver, cfg);

    mpc::TrialSpec spec;
    spec.mode = mpc::RawMode::kErpm;
    spec.value = 1500.0;
    runner.start_trial(spec);

    double now = 0.0;
    const double dt = 0.02;
    for (int i = 0; i < 80; ++i) {
        mpc::CalibSample pose{now, 0.0, 0.0, 0.0};
        driver.now_ptr_value = now;
        runner.step(now, true, pose, false, mpc::TelemetrySample{});
        now += dt;
        if (runner.state() == mpc::TrialState::kRun) break;
    }
    if (runner.state() != mpc::TrialState::kRun) {
        std::cerr << "    failed to reach kRun for the csv-writer test\n";
        return false;
    }

    // First logged row: no telemetry has arrived yet at all -> empty fields.
    // Second: a telemetry sample has now been seen -> nearest-timestamp join
    // succeeds (and keeps succeeding for every row after, once the buffer is
    // non-empty -- this deliberately logs the row-without-telemetry FIRST).
    mpc::CalibSample pose1{now, 0.01, 0.0, 0.0};
    driver.now_ptr_value = now;
    runner.step(now, true, pose1, false, mpc::TelemetrySample{});  // row WITHOUT telemetry
    now += dt;
    mpc::TelemetrySample telem{now, 1490.0, 0.29, 3.1, 12.4};
    mpc::CalibSample pose2{now, 0.02, 0.0, 0.0};
    driver.now_ptr_value = now;
    runner.step(now, true, pose2, true, telem);  // row WITH telemetry

    namespace fs = std::filesystem;
    const fs::path path = fs::temp_directory_path() / "motor_calib_test_trial.csv";
    if (!runner.write_csv(path.string())) {
        std::cerr << "    write_csv() failed\n";
        return false;
    }

    std::ifstream in(path);
    std::string header;
    std::getline(in, header);
    const std::string expected_header =
        "t,x,y,yaw,s,v_raw,v_smooth,cmd_mode,cmd_value,erpm,duty,current_motor,v_in";
    if (header != expected_header) {
        std::cerr << "    header mismatch: got '" << header << "'\n";
        ok = false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty()) lines.push_back(line);
    }
    if (lines.size() != runner.rows().size()) {
        std::cerr << "    row count mismatch: csv has " << lines.size() << ", rows() has "
                   << runner.rows().size() << "\n";
        ok = false;
    }
    bool found_empty_telemetry_row = false;
    for (const auto& l : lines) {
        const std::size_t comma_count = std::count(l.begin(), l.end(), ',');
        if (comma_count != 12) {
            std::cerr << "    row has wrong field count (" << comma_count << " commas): '" << l
                       << "'\n";
            ok = false;
        }
        if (l.size() >= 4 && l.substr(l.size() - 4) == ",,,,") found_empty_telemetry_row = true;
    }
    if (!found_empty_telemetry_row) {
        std::cerr << "    expected at least one row with empty telemetry fields\n";
        ok = false;
    }

    std::error_code ec;
    fs::remove(path, ec);
    return ok;
}

// ---------------------------------------------------------------------
// Fitters
// ---------------------------------------------------------------------

bool test_steady_state_v() {
    bool ok = true;
    const double v_ss_true = 0.18;
    const double cmd_true = 0.06;
    const double tau = 0.2;
    const double dt = 0.02;
    const int n = 150;  // 3.0s total

    std::vector<mpc::TrialRow> rows;
    for (int i = 0; i < n; ++i) {
        const double t = i * dt;
        mpc::TrialRow r;
        r.t = t;
        r.v_smooth = v_ss_true * (1.0 - std::exp(-t / tau));
        r.cmd_value = cmd_true;
        r.cmd_mode = mpc::RawMode::kDuty;
        rows.push_back(r);
    }

    mpc::SteadyStatePoint pt = mpc::steady_state_v(rows, 0.4);
    if (!pt.ok) {
        std::cerr << "    expected ok=true\n";
        return false;
    }
    if (std::fabs(pt.v_ss - v_ss_true) > 0.01) {
        std::cerr << "    v_ss=" << pt.v_ss << " expected ~" << v_ss_true << "\n";
        ok = false;
    }
    if (!near_eq(pt.cmd, cmd_true, 1e-9)) {
        std::cerr << "    cmd=" << pt.cmd << " expected " << cmd_true << "\n";
        ok = false;
    }

    mpc::SteadyStatePoint empty_pt = mpc::steady_state_v({}, 0.4);
    if (empty_pt.ok) {
        std::cerr << "    expected ok=false for empty rows\n";
        ok = false;
    }
    return ok;
}

bool test_fit_stall() {
    bool ok = true;
    const double c0 = 0.03, k = 3.0;
    const std::vector<double> cmds = {0.02, 0.03, 0.04, 0.05, 0.06, 0.08};
    std::vector<mpc::SteadyStatePoint> pts;
    for (double c : cmds) {
        mpc::SteadyStatePoint p;
        p.cmd = c;
        p.v_ss = (c > c0) ? k * (c - c0) : 0.0;
        p.ok = true;
        pts.push_back(p);
    }

    // min_moving deliberately NOT at an exact floating-point boundary of any
    // computed v_ss (0.03/0.04/0.06 land right on such boundaries and are
    // fragile to double-rounding) -- 0.025 sits cleanly between v_ss(0.03)=0
    // and v_ss(0.04)=0.03, so cmd=0.04 unambiguously clears it first.
    const double min_moving = 0.025;
    mpc::StallFit fit = mpc::fit_stall(pts, min_moving);
    if (!fit.ok) {
        std::cerr << "    expected ok=true\n";
        return false;
    }
    if (!near_eq(fit.min_cmd, 0.04, 1e-9)) {
        std::cerr << "    min_cmd=" << fit.min_cmd << " expected 0.04\n";
        ok = false;
    }
    if (!near_eq(fit.kick_cmd, 0.06, 1e-9)) {
        std::cerr << "    kick_cmd=" << fit.kick_cmd << " expected 0.06\n";
        ok = false;
    }
    if (!near_eq(fit.kick_ms, 150.0, 1e-9)) {
        std::cerr << "    kick_ms=" << fit.kick_ms << " expected 150\n";
        ok = false;
    }
    if (!near_eq(fit.min_moving_speed_mps, min_moving, 1e-9)) {
        std::cerr << "    min_moving_speed_mps=" << fit.min_moving_speed_mps << " expected "
                   << min_moving << "\n";
        ok = false;
    }

    std::vector<mpc::SteadyStatePoint> none_pts = {{0.02, 0.0, true}, {0.03, 0.0, true}};
    mpc::StallFit fit2 = mpc::fit_stall(none_pts, 0.03);
    if (fit2.ok) {
        std::cerr << "    expected ok=false when nothing clears min_moving\n";
        ok = false;
    }
    return ok;
}

bool test_fit_linear() {
    bool ok = true;
    const double true_offset = 0.03, true_slope = 0.2;
    std::vector<mpc::SteadyStatePoint> pts;
    for (double v : {0.05, 0.10, 0.15, 0.20, 0.25}) {
        mpc::SteadyStatePoint p;
        p.v_ss = v;
        p.cmd = true_offset + true_slope * v;
        p.ok = true;
        pts.push_back(p);
    }

    mpc::LinearFit fit = mpc::fit_linear(pts, /*stall_min_cmd=*/0.0);
    if (!fit.ok) {
        std::cerr << "    expected ok=true\n";
        return false;
    }
    const double slope_err = std::fabs(fit.cmd_per_mps - true_slope) / true_slope;
    if (slope_err > 0.05) {
        std::cerr << "    slope error " << slope_err << " exceeds 5% (got " << fit.cmd_per_mps
                   << ", true " << true_slope << ")\n";
        ok = false;
    }
    if (std::fabs(fit.cmd_offset - true_offset) > 0.01) {
        std::cerr << "    cmd_offset=" << fit.cmd_offset << " expected ~" << true_offset << "\n";
        ok = false;
    }
    if (fit.fit_rms > 1e-6) {
        std::cerr << "    fit_rms=" << fit.fit_rms << " expected ~0 on noiseless data\n";
        ok = false;
    }

    mpc::LinearFit fit2 = mpc::fit_linear({pts[0]}, 0.0);
    if (fit2.ok) {
        std::cerr << "    expected ok=false with fewer than 2 qualifying points\n";
        ok = false;
    }
    return ok;
}

bool test_fit_grid_accuracy() {
    bool ok = true;
    const double k = 3.0, c0 = 0.03, tau = 0.2;
    const std::vector<double> cmds = {0.05, 0.06, 0.07, 0.08, 0.10};
    const double dt = 0.02;

    std::vector<std::vector<mpc::TrialRow>> trials;
    for (double cmd : cmds) {
        const double v_ss = k * (cmd - c0);
        std::vector<mpc::TrialRow> rows;
        double v = 0.0;
        for (int i = 0; i < 150; ++i) {  // 3.0s
            mpc::TrialRow r;
            r.t = i * dt;
            r.v_smooth = v;
            r.cmd_value = cmd;
            r.cmd_mode = mpc::RawMode::kDuty;
            rows.push_back(r);
            v += (v_ss - v) / tau * dt;
        }
        trials.push_back(rows);
    }

    // A finer v-grid than the default (0.01 vs 0.05 step) keeps each bin's
    // own true-v spread small relative to da/dv=-1/tau -- with the default,
    // coarser bins mix samples whose ACTUAL v differs enough (across
    // different-cmd trials passing through the same nominal bin at
    // different times) to bias the per-bin linear fit beyond the 10%
    // target; this is a legitimate caller choice (fit_grid's grids are
    // parameters), not a change to the model being tested.
    std::vector<double> fine_v_grid;
    for (int i = 0; i <= 50; ++i) fine_v_grid.push_back(i * 0.01);

    mpc::GridFit fit = mpc::fit_grid(trials, fine_v_grid, mpc::default_grid_a_mps2());
    if (!fit.ok) {
        std::cerr << "    expected ok=true\n";
        return false;
    }

    int checked = 0;
    for (std::size_t i = 0; i < fit.v_mps.size(); ++i) {
        for (std::size_t j = 0; j < fit.a_mps2.size(); ++j) {
            if (!fit.supported[i][j]) continue;
            const double v = fit.v_mps[i];
            const double a = fit.a_mps2[j];
            const double expected_cmd = c0 + (v + a * tau) / k;
            const double got = fit.cmd[i][j];
            const double denom = std::max(0.01, std::fabs(expected_cmd));
            const double rel_err = std::fabs(got - expected_cmd) / denom;
            if (rel_err > 0.10) {
                std::cerr << "    cmd(v=" << v << ",a=" << a << ") got " << got << " expected "
                           << expected_cmd << " (rel err " << rel_err << ")\n";
                ok = false;
            }
            ++checked;
        }
    }
    if (checked == 0) {
        std::cerr << "    no supported cells were checked\n";
        ok = false;
    }
    return ok;
}

bool test_fit_grid_no_cross_trial_boundary() {
    bool ok = true;
    mpc::TrialRow a1, a2, b1, b2;
    a1.t = 0.0;
    a1.v_smooth = 0.04;
    a1.cmd_value = 0.05;
    a2.t = 0.1;
    a2.v_smooth = 0.06;
    a2.cmd_value = 0.05;
    // Trial B's own clock continues right where trial A's left off -- if
    // fit_grid ever (incorrectly) flattened `trials` into one sequence and
    // differentiated across the A/B boundary, this dt>0 gap would slip past
    // the "dt<=1e-6 skip" guard and inject a spurious 3rd sample.
    b1.t = 0.15;
    b1.v_smooth = 0.03;
    b1.cmd_value = 0.08;
    b2.t = 0.25;
    b2.v_smooth = 0.07;
    b2.cmd_value = 0.08;

    std::vector<std::vector<mpc::TrialRow>> trials = {{a1, a2}, {b1, b2}};
    mpc::GridFit fit = mpc::fit_grid(trials);
    if (!fit.ok) {
        std::cerr << "    expected ok=true\n";
        return false;
    }

    const std::size_t vb = 1;  // default grid's v_mps[1] == 0.05
    if (!near_eq(fit.v_mps[vb], 0.05, 1e-9)) {
        std::cerr << "    test assumption broken: default grid bin 1 is not 0.05\n";
        return false;
    }

    // The clean 2-point line through (cmd=0.05,a=0.2) [trial A] and
    // (cmd=0.08,a=0.4) [trial B] -- both diffs land in the v=0.05 bin.
    const double slope = 0.2 / 0.03;
    const double intercept = 0.2 - slope * 0.05;

    int checked = 0;
    for (std::size_t j = 0; j < fit.a_mps2.size(); ++j) {
        if (!fit.supported[vb][j]) continue;
        const double expected = (fit.a_mps2[j] - intercept) / slope;
        if (std::fabs(fit.cmd[vb][j] - expected) > 1e-6) {
            std::cerr << "    v-bin " << vb << " a=" << fit.a_mps2[j] << ": got " << fit.cmd[vb][j]
                       << " expected " << expected << " (cross-trial-boundary contamination?)\n";
            ok = false;
        }
        ++checked;
    }
    if (checked == 0) {
        std::cerr << "    v-bin " << vb << " was not supported at all\n";
        ok = false;
    }
    return ok;
}

// Regression (round-0 finding, MPC/src/MotorCalibCore.cpp fit_grid()):
// negative-direction (reverse) trial samples must be binned by SPEED
// MAGNITUDE |v_mid|, not raw signed v_mid, into the non-negative default
// v-grid. Two forward near-stall trials populate the v=0.00 bin; two
// reverse trials, cruising at steady v~=-0.40 (nowhere near stall), are
// added on top. With the fix, the deep-reverse samples land in the
// magnitude-matched v=0.40 bin and the v=0.00 bin's fit is byte-for-byte
// unaffected by their presence. Under the pre-fix bug (binning by signed
// v_mid), EVERY sample of the reverse trials would instead land in v=0.00
// (nearest_grid_index() of any negative value against an all-nonnegative
// ascending grid is always index 0) -- corrupting that bin's fit AND
// leaving the v=0.40 bin unsupported.
bool test_fit_grid_reverse_direction_binning() {
    bool ok = true;

    // Forward, near-zero-velocity samples -- two trials at different cmd,
    // both diffs landing in the v=0.00 bin (v_mid=0.01 and 0.015, both
    // nearer to grid point 0.00 than to 0.05).
    mpc::TrialRow a1, a2, b1, b2;
    a1.t = 0.0;  a1.v_smooth = 0.00;  a1.cmd_value = 0.05;  a1.cmd_mode = mpc::RawMode::kDuty;
    a2.t = 0.1;  a2.v_smooth = 0.02;  a2.cmd_value = 0.05;  a2.cmd_mode = mpc::RawMode::kDuty;
    b1.t = 0.0;  b1.v_smooth = 0.00;  b1.cmd_value = 0.08;  b1.cmd_mode = mpc::RawMode::kDuty;
    b2.t = 0.1;  b2.v_smooth = 0.03;  b2.cmd_value = 0.08;  b2.cmd_mode = mpc::RawMode::kDuty;

    // Reverse, deep-steady-cruise samples (v_mid=-0.40, |v_mid|=0.40 -- the
    // v=0.40 grid bin) -- two trials at different (negative) cmd, mirroring
    // the real-world "steady-state cmd-to-overcome-drag while cruising in
    // reverse" scenario the finding describes; deliberately far from stall.
    mpc::TrialRow c1, c2, d1, d2;
    c1.t = 0.0;  c1.v_smooth = -0.38;  c1.cmd_value = -0.20;  c1.cmd_mode = mpc::RawMode::kDuty;
    c2.t = 0.1;  c2.v_smooth = -0.42;  c2.cmd_value = -0.20;  c2.cmd_mode = mpc::RawMode::kDuty;
    d1.t = 0.0;  d1.v_smooth = -0.39;  d1.cmd_value = -0.30;  d1.cmd_mode = mpc::RawMode::kDuty;
    d2.t = 0.1;  d2.v_smooth = -0.41;  d2.cmd_value = -0.30;  d2.cmd_mode = mpc::RawMode::kDuty;

    const std::vector<std::vector<mpc::TrialRow>> forward_only = {{a1, a2}, {b1, b2}};
    const std::vector<std::vector<mpc::TrialRow>> combined = {{a1, a2}, {b1, b2}, {c1, c2}, {d1, d2}};

    const mpc::GridFit fit_fwd = mpc::fit_grid(forward_only);
    const mpc::GridFit fit_combined = mpc::fit_grid(combined);

    const std::size_t v0 = 0;  // default grid's v_mps[0] == 0.00
    const std::size_t v8 = 8;  // default grid's v_mps[8] == 0.40
    if (!near_eq(fit_combined.v_mps[v0], 0.00, 1e-9) || !near_eq(fit_combined.v_mps[v8], 0.40, 1e-9)) {
        std::cerr << "    test assumption broken: default grid bins 0/8 are not 0.00/0.40\n";
        return false;
    }
    // a_mps2 index 6 == 0.2 (the exact 'a' of trial A's own sample).
    const std::size_t a_pt2 = 6;
    if (!near_eq(fit_combined.a_mps2[a_pt2], 0.2, 1e-9)) {
        std::cerr << "    test assumption broken: default a-grid index 6 is not 0.2\n";
        return false;
    }

    // v=0.00 bin: forward-only fit at a=0.2 must exactly reproduce trial
    // A's own (cmd=0.05,a=0.2) sample (an exact 2-point line).
    if (!fit_fwd.supported[v0][a_pt2]) {
        std::cerr << "    forward-only: v=0.00,a=0.2 bin is not supported\n";
        ok = false;
    } else if (!near_eq(fit_fwd.cmd[v0][a_pt2], 0.05, 1e-6)) {
        std::cerr << "    forward-only: cmd(v=0.00,a=0.2) got " << fit_fwd.cmd[v0][a_pt2] << " expected 0.05\n";
        ok = false;
    }

    // Adding the two deep-reverse trials must NOT change the v=0.00 bin's
    // fit at all -- this is the core regression check. Under the pre-fix
    // bug (binning by signed v_mid) the reverse trials' samples would ALSO
    // land in this bin, changing both its supported-column set and its
    // fitted cmd(a=0.2) value away from 0.05.
    if (!fit_combined.supported[v0][a_pt2]) {
        std::cerr << "    combined: v=0.00,a=0.2 bin is not supported (reverse trials corrupted it)\n";
        ok = false;
    } else if (!near_eq(fit_combined.cmd[v0][a_pt2], 0.05, 1e-6)) {
        std::cerr << "    combined: cmd(v=0.00,a=0.2) got " << fit_combined.cmd[v0][a_pt2]
                   << " expected 0.05 (unchanged by the reverse trials) -- reverse-direction samples "
                      "leaked into the v=0.00 bin\n";
        ok = false;
    }

    // The reverse trials' own samples must land in the MAGNITUDE-matched
    // v=0.40 bin (positive proof they weren't silently dropped OR funneled
    // into v=0.00): exact 2-point line through (cmd=-0.20,a=-0.4) and
    // (cmd=-0.30,a=-0.2).
    const std::size_t a_neg04 = 3;  // -1.0 + 3*0.2 == -0.4
    const std::size_t a_neg02 = 4;  // -1.0 + 4*0.2 == -0.2
    if (!fit_combined.supported[v8][a_neg04]) {
        std::cerr << "    combined: v=0.40,a=-0.4 bin is not supported by the reverse trials\n";
        ok = false;
    } else if (!near_eq(fit_combined.cmd[v8][a_neg04], -0.20, 1e-6)) {
        std::cerr << "    combined: cmd(v=0.40,a=-0.4) got " << fit_combined.cmd[v8][a_neg04] << " expected -0.20\n";
        ok = false;
    }
    if (!fit_combined.supported[v8][a_neg02]) {
        std::cerr << "    combined: v=0.40,a=-0.2 bin is not supported by the reverse trials\n";
        ok = false;
    } else if (!near_eq(fit_combined.cmd[v8][a_neg02], -0.30, 1e-6)) {
        std::cerr << "    combined: cmd(v=0.40,a=-0.2) got " << fit_combined.cmd[v8][a_neg02] << " expected -0.30\n";
        ok = false;
    }

    return ok;
}

// ---------------------------------------------------------------------
// Export / import
// ---------------------------------------------------------------------

bool test_export_calibration_schema_shape() {
    bool ok = true;
    mpc::LinearFit linear;
    linear.cmd_offset = 0.02;
    linear.cmd_per_mps = 18.5;
    linear.fit_rms = 0.015;
    linear.ok = true;
    mpc::StallFit stall;
    stall.min_cmd = 0.03;
    stall.kick_cmd = 0.045;
    stall.kick_ms = 150.0;
    stall.min_moving_speed_mps = 0.03;
    stall.ok = true;
    std::vector<mpc::SteadyStatePoint> v_ss = {
        {0.08, 0.20, true},
        {0.03, 0.0, false},  // must be filtered out (ok==false)
        {0.05, 0.06, true},
    };

    nlohmann::json j = mpc::export_calibration("robot2", mpc::RawMode::kErpm, 4614.0, linear, stall,
                                                 v_ss, nullptr, 12, "unit test");

    if (j.at("version").get<int>() != 1) {
        std::cerr << "    version\n";
        ok = false;
    }
    if (j.at("mode").get<std::string>() != "erpm") {
        std::cerr << "    mode\n";
        ok = false;
    }
    if (j.at("robot_name").get<std::string>() != "robot2") {
        std::cerr << "    robot_name\n";
        ok = false;
    }
    if (!near_eq(j.at("erpm_per_mps").get<double>(), 4614.0, 1e-9)) ok = false;
    if (!near_eq(j.at("cmd_per_mps").get<double>(), 18.5, 1e-9)) ok = false;
    if (!near_eq(j.at("cmd_offset").get<double>(), 0.02, 1e-9)) ok = false;
    if (!near_eq(j.at("stall").at("min_cmd").get<double>(), 0.03, 1e-9) ||
        !near_eq(j.at("stall").at("kick_cmd").get<double>(), 0.045, 1e-9) ||
        !near_eq(j.at("stall").at("kick_ms").get<double>(), 150.0, 1e-9) ||
        !near_eq(j.at("stall").at("min_moving_speed_mps").get<double>(), 0.03, 1e-9)) {
        std::cerr << "    stall sub-object mismatch\n";
        ok = false;
    }

    const auto& vs = j.at("v_ss");
    if (vs.size() != 2) {
        std::cerr << "    v_ss size=" << vs.size() << " expected 2 (ok==false filtered out)\n";
        ok = false;
    } else if (!near_eq(vs[0].at("cmd").get<double>(), 0.05, 1e-9) ||
               !near_eq(vs[1].at("cmd").get<double>(), 0.08, 1e-9)) {
        std::cerr << "    v_ss not sorted ascending by cmd\n";
        ok = false;
    }
    if (!j.at("grid").is_null()) {
        std::cerr << "    expected grid:null when grid==nullptr\n";
        ok = false;
    }
    if (j.at("meta").at("n_trials").get<int>() != 12) ok = false;
    if (!near_eq(j.at("meta").at("fit_rms").get<double>(), 0.015, 1e-9)) ok = false;
    if (j.at("meta").at("notes").get<std::string>() != "unit test") ok = false;

    return ok;
}

bool test_export_parse_roundtrip() {
    bool ok = true;
    mpc::LinearFit linear;
    linear.cmd_offset = 1.5;
    linear.cmd_per_mps = 20.0;
    linear.fit_rms = 0.02;
    linear.ok = true;
    mpc::StallFit stall;
    stall.min_cmd = 0.03;
    stall.kick_cmd = 0.05;
    stall.kick_ms = 150.0;
    stall.min_moving_speed_mps = 0.03;
    stall.ok = true;
    std::vector<mpc::SteadyStatePoint> v_ss = {{0.05, 0.10, true}, {0.07, 0.20, true}};

    mpc::GridFit grid;
    grid.v_mps = {0.0, 0.1};
    grid.a_mps2 = {-0.5, 0.5};
    grid.cmd = {{0.02, 0.06}, {0.04, 0.08}};
    grid.ok = true;

    nlohmann::json j = mpc::export_calibration("mushr2", mpc::RawMode::kDuty, 0.0, linear, stall,
                                                 v_ss, &grid, 30, "roundtrip");
    mpc::ParsedCalibration p = mpc::parse_calibration(j);

    if (p.version != 1) ok = false;
    if (p.mode != "duty") ok = false;
    if (p.robot_name != "mushr2") ok = false;
    if (!near_eq(p.cmd_per_mps, 20.0, 1e-9)) ok = false;
    if (!near_eq(p.cmd_offset, 1.5, 1e-9)) ok = false;
    if (!near_eq(p.stall_min_cmd, 0.03, 1e-9) || !near_eq(p.stall_kick_cmd, 0.05, 1e-9) ||
        !near_eq(p.stall_kick_ms, 150.0, 1e-9) || !near_eq(p.stall_min_moving_speed_mps, 0.03, 1e-9)) {
        std::cerr << "    stall fields mismatch after roundtrip\n";
        ok = false;
    }
    if (p.v_ss.size() != 2) {
        std::cerr << "    v_ss size mismatch after roundtrip\n";
        ok = false;
    } else {
        if (!near_eq(p.v_ss[0].cmd, 0.05, 1e-9) || !near_eq(p.v_ss[0].v_ss, 0.10, 1e-9)) ok = false;
        if (!near_eq(p.v_ss[1].cmd, 0.07, 1e-9) || !near_eq(p.v_ss[1].v_ss, 0.20, 1e-9)) ok = false;
    }
    if (!p.has_grid) {
        std::cerr << "    expected has_grid=true\n";
        ok = false;
    } else {
        if (p.grid.v_mps != grid.v_mps || p.grid.a_mps2 != grid.a_mps2 || p.grid.cmd != grid.cmd) {
            std::cerr << "    grid mismatch after roundtrip\n";
            ok = false;
        }
    }
    if (p.n_trials != 30) ok = false;
    if (!near_eq(p.fit_rms, 0.02, 1e-9)) ok = false;
    if (p.notes != "roundtrip") ok = false;

    nlohmann::json j2 = mpc::export_calibration("mushr2", mpc::RawMode::kDuty, 0.0, linear, stall,
                                                  v_ss, nullptr, 30, "no grid");
    mpc::ParsedCalibration p2 = mpc::parse_calibration(j2);
    if (p2.has_grid) {
        std::cerr << "    expected has_grid=false for grid==nullptr\n";
        ok = false;
    }

    return ok;
}

bool test_export_calibration_current_mode_throws() {
    mpc::LinearFit linear;
    linear.ok = true;
    mpc::StallFit stall;
    stall.ok = true;
    bool threw = false;
    try {
        mpc::export_calibration("r", mpc::RawMode::kCurrent, 1.0, linear, stall, {}, nullptr, 0, "");
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "    expected std::invalid_argument for RawMode::kCurrent\n";
        return false;
    }
    return true;
}

bool test_parse_calibration_missing_key_throws() {
    bool ok = true;
    nlohmann::json j = {
        {"version", 1},
        {"mode", "erpm"},
        {"robot_name", "r"},
        {"erpm_per_mps", 1.0},
        {"cmd_per_mps", 1.0},
        // "cmd_offset" deliberately missing
        {"stall",
         {{"min_cmd", 0.0}, {"kick_cmd", 0.0}, {"kick_ms", 150.0}, {"min_moving_speed_mps", 0.0}}},
        {"v_ss", nlohmann::json::array()},
        {"grid", nullptr},
        {"meta", {{"n_trials", 0}, {"fit_rms", 0.0}, {"notes", ""}}},
    };
    bool threw = false;
    try {
        mpc::parse_calibration(j);
    } catch (const std::exception&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "    expected throw for missing 'cmd_offset'\n";
        ok = false;
    }

    nlohmann::json j2 = j;
    j2["cmd_offset"] = 0.0;
    j2["mode"] = "bogus";
    threw = false;
    try {
        mpc::parse_calibration(j2);
    } catch (const std::exception&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "    expected throw for invalid 'mode' string\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------

bool test_motor_calib_config_defaults_and_partial_override() {
    bool ok = true;
    mpc::MotorCalibConfig d = mpc::MotorCalibConfig::defaults();
    if (d.robot_ip != "192.168.1.PLACEHOLDER") ok = false;
    if (d.control_port != 3460) ok = false;
    if (d.telemetry_port != 3560) ok = false;
    if (d.localization_endpoint != "tcp://127.0.0.1:3260") ok = false;
    if (d.robot_topic_name != "robot2") ok = false;
    if (d.motive_body_name != "mushr2") ok = false;
    if (!near_eq(d.line.length_m, 4.5, 1e-9) || !near_eq(d.line.end_margin_m, 0.75, 1e-9)) ok = false;
    if (d.line.line_yaw_rad.has_value()) ok = false;
    if (!near_eq(d.safety.mocap_stale_abort_ms, 300.0, 1e-9) ||
        !near_eq(d.safety.max_speed_abort_mps, 0.8, 1e-9)) {
        ok = false;
    }
    if (d.sweeps.duty.size() != 6 || d.sweeps.erpm.size() != 7) ok = false;
    if (!near_eq(d.trial.max_duration_s, 6.0, 1e-9) || !near_eq(d.trial.settle_s, 1.5, 1e-9) ||
        !near_eq(d.trial.arm_settle_s, 1.0, 1e-9) || !near_eq(d.trial.steady_window_frac, 0.4, 1e-9)) {
        ok = false;
    }
    if (d.output_dir != "results/motor_calib") ok = false;
    if (!ok) {
        std::cerr << "    default field mismatch\n";
        return false;
    }

    namespace fs = std::filesystem;
    const fs::path path = fs::temp_directory_path() / "motor_calib_test_partial_config.json";
    {
        std::ofstream out(path);
        out << R"({
          "robot_ip": "10.0.0.5",
          "line": { "length_m": 3.0, "line_yaw_rad": 1.2 },
          "trial": { "settle_s": 2.0 },
          "not_a_real_top_key": 42
        })";
    }
    mpc::MotorCalibConfig cfg;
    try {
        cfg = mpc::load_motor_calib_config(path.string());
    } catch (const std::exception& ex) {
        std::cerr << "    unexpected throw on valid partial JSON: " << ex.what() << "\n";
        std::error_code ec;
        fs::remove(path, ec);
        return false;
    }
    std::error_code ec;
    fs::remove(path, ec);

    if (cfg.robot_ip != "10.0.0.5") {
        std::cerr << "    robot_ip override failed\n";
        ok = false;
    }
    if (!near_eq(cfg.line.length_m, 3.0, 1e-9)) {
        std::cerr << "    line.length_m override failed\n";
        ok = false;
    }
    if (!cfg.line.line_yaw_rad.has_value() || !near_eq(*cfg.line.line_yaw_rad, 1.2, 1e-9)) {
        std::cerr << "    line.line_yaw_rad override failed\n";
        ok = false;
    }
    if (!near_eq(cfg.trial.settle_s, 2.0, 1e-9)) {
        std::cerr << "    trial.settle_s override failed\n";
        ok = false;
    }
    if (!near_eq(cfg.line.end_margin_m, 0.75, 1e-9)) {
        std::cerr << "    line.end_margin_m should keep its compiled default\n";
        ok = false;
    }
    if (cfg.motive_body_name != "mushr2") {
        std::cerr << "    motive_body_name should keep its compiled default\n";
        ok = false;
    }
    if (!near_eq(cfg.trial.arm_settle_s, 1.0, 1e-9)) {
        std::cerr << "    trial.arm_settle_s should keep its compiled default\n";
        ok = false;
    }
    return ok;
}

bool test_motor_calib_config_invalid_path_and_json_throw() {
    bool ok = true;
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path();

    bool threw = false;
    try {
        mpc::load_motor_calib_config((dir / "motor_calib_test_nonexistent_12345.json").string());
    } catch (const std::exception&) {
        threw = true;
    }
    if (!threw) {
        std::cerr << "    expected throw for nonexistent path\n";
        ok = false;
    }

    const fs::path bad_path = dir / "motor_calib_test_invalid.json";
    {
        std::ofstream out(bad_path);
        out << "{ not valid json ";
    }
    threw = false;
    try {
        mpc::load_motor_calib_config(bad_path.string());
    } catch (const std::exception&) {
        threw = true;
    }
    std::error_code ec;
    fs::remove(bad_path, ec);
    if (!threw) {
        std::cerr << "    expected throw for invalid JSON content\n";
        ok = false;
    }

    return ok;
}

bool test_make_trial_runner_config_mapping() {
    bool ok = true;
    mpc::MotorCalibConfig cfg = mpc::MotorCalibConfig::defaults();
    cfg.line.length_m = 5.0;
    cfg.line.end_margin_m = 0.5;
    cfg.safety.mocap_stale_abort_ms = 250.0;
    cfg.safety.max_speed_abort_mps = 1.2;
    cfg.trial.max_duration_s = 8.0;
    cfg.trial.settle_s = 2.5;
    cfg.trial.arm_settle_s = 1.5;

    mpc::TrialRunnerConfig rc = mpc::make_trial_runner_config(cfg);
    if (!near_eq(rc.line_length_m, 5.0, 1e-9) || !near_eq(rc.end_margin_m, 0.5, 1e-9) ||
        !near_eq(rc.mocap_stale_abort_ms, 250.0, 1e-9) || !near_eq(rc.max_speed_abort_mps, 1.2, 1e-9) ||
        !near_eq(rc.max_duration_s, 8.0, 1e-9) || !near_eq(rc.settle_s, 2.5, 1e-9) ||
        !near_eq(rc.arm_settle_s, 1.5, 1e-9)) {
        std::cerr << "    make_trial_runner_config field mismatch\n";
        ok = false;
    }
    mpc::TrialRunnerConfig defaults;
    if (!near_eq(rc.arm_stationary_v_mps, defaults.arm_stationary_v_mps, 1e-9) ||
        !near_eq(rc.reissue_interval_s, defaults.reissue_interval_s, 1e-9) ||
        !near_eq(rc.center_servo_value, defaults.center_servo_value, 1e-9)) {
        std::cerr << "    non-config knobs should keep TrialRunnerConfig's own compiled defaults\n";
        ok = false;
    }
    return ok;
}

}  // namespace

int main() {
    using TestFn = std::function<bool()>;
    const std::vector<std::pair<std::string, TestFn>> tests = {
        {"VelocityEstimator: raw finite-difference velocity + reversal sign",
         test_velocity_estimator_raw_straight_line_and_reversal},
        {"VelocityEstimator: 5-sample centered smoothing window + d(v_smooth)/dt",
         test_velocity_estimator_smoothing_window_and_accel},
        {"LineFrame: capture (override vs initial yaw) + s()/lateral() projection",
         test_line_frame},
        {"TrialRunner: normal ARM->RUN->STOPPING->DONE completion + raw() TTL keepalive cadence",
         test_trial_runner_normal_completion_and_ttl_cadence},
        {"TrialRunner: SAFETY abort -- stale mocap", test_trial_runner_abort_mocap_stale},
        {"TrialRunner: SAFETY abort -- over speed", test_trial_runner_abort_over_speed},
        {"TrialRunner: SAFETY abort -- out of usable line envelope",
         test_trial_runner_abort_out_of_envelope},
        {"TrialRunner: SAFETY abort -- out of usable line envelope, WRONG-DIRECTION excursion "
         "(reversed polarity/rollback -- commanded-direction check alone never catches this)",
         test_trial_runner_abort_out_of_envelope_wrong_direction},
        {"TrialRunner: SAFETY abort -- trial wall-time timeout", test_trial_runner_abort_timeout},
        {"TrialRunner: ARM-time abort -- insufficient remaining line room",
         test_trial_runner_abort_insufficient_room_at_arm},
        {"TrialRunner: abort -- driver refuses set_source(calib)",
         test_trial_runner_abort_driver_refused},
        {"TrialRunner: write_csv() header/field-count/empty-telemetry formatting",
         test_trial_runner_csv_writer},
        {"steady_state_v: trailing-window mean converges to the true first-order asymptote",
         test_steady_state_v},
        {"fit_stall: min_cmd/kick_cmd/kick_ms recovered from a synthetic stall-clamped sweep",
         test_fit_stall},
        {"fit_linear: recovered slope within 5% of ground truth on noiseless synthetic data",
         test_fit_linear},
        {"fit_grid: recovered cmd(v,a) within 10% of the closed-form inverse where supported",
         test_fit_grid_accuracy},
        {"fit_grid: accel is never differentiated across a trial boundary",
         test_fit_grid_no_cross_trial_boundary},
        {"fit_grid: reverse-direction (negative v_mid) samples bin by SPEED MAGNITUDE, not raw "
         "signed v, so a deep-reverse steady-cruise trial lands in its own v-bin instead of "
         "corrupting the v=0.00 near-stall bin",
         test_fit_grid_reverse_direction_binning},
        {"export_calibration: frozen-schema key names/nesting/filtering/sorting",
         test_export_calibration_schema_shape},
        {"export_calibration/parse_calibration: round trip (with and without grid)",
         test_export_parse_roundtrip},
        {"export_calibration: RawMode::kCurrent throws std::invalid_argument",
         test_export_calibration_current_mode_throws},
        {"parse_calibration: missing required key / invalid mode string both throw",
         test_parse_calibration_missing_key_throws},
        {"MotorCalibConfig: defaults match motor_calib_config.json + partial JSON override",
         test_motor_calib_config_defaults_and_partial_override},
        {"MotorCalibConfig: nonexistent path / invalid JSON content both throw",
         test_motor_calib_config_invalid_path_and_json_throw},
        {"make_trial_runner_config: maps line/safety/trial fields, leaves the rest at defaults",
         test_make_trial_runner_config_mapping},
    };

    int failed = 0;
    for (const auto& [name, fn] : tests) {
        std::cout << "[ RUN      ] " << name << "\n";
        const bool result = fn();
        if (result) {
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
