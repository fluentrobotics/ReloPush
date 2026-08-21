// Lightweight unit tests for mpc::CalibrationCore (velocity/yaw-rate
// derivation, steadiness acceptance, coverage tracking, PAVA-pooled
// fitting, CSV round trip, export schema), mirroring the harness style of
// MPC/tests/motor_calib_tests.cpp: plain bool test_xxx() functions
// registered in main(), no gtest dependency, no sockets.

#include "mpc/CalibrationCore.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

bool near_eq(double a, double b, double eps) { return std::fabs(a - b) <= eps; }

// ---------------------------------------------------------------------
// Synthetic teleop driver: integrates a simple kinematic-bicycle pose
// trajectory (x,y,yaw) from a caller-chosen (v, omega) profile and feeds
// it through CalibrationCore's real feed_pose()/feed_telemetry()/
// feed_command() ingestion, exactly like a live Qt tab would. Ground-truth
// erpm/delta relationships are supplied by the caller (per test) so each
// test can assert the FIT recovers them.
// ---------------------------------------------------------------------
struct SynthState {
    double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0;
};

// Drives one constant-(v,omega) segment for `duration_s` at `dt`, holding
// a constant command (mode "duty", `command_value`, `servo`) throughout,
// and feeding telemetry erpm = `erpm_fn(v)` (+ noise, if `noise` is
// non-null) time-aligned to each pose.
void drive_segment(mpc::CalibrationCore& core, SynthState& st, double v, double omega, double duration_s,
                    double dt, double command_value, double servo,
                    const std::function<double(double)>& erpm_fn,
                    std::function<double()>* noise = nullptr) {
    const int n = static_cast<int>(duration_s / dt);
    for (int i = 0; i < n; ++i) {
        st.yaw += omega * dt;
        st.x += v * dt * std::cos(st.yaw);
        st.y += v * dt * std::sin(st.yaw);
        st.t += dt;
        core.feed_command(st.t, "duty", command_value, servo);
        double erpm = erpm_fn(v);
        if (noise != nullptr) erpm += (*noise)();
        core.feed_telemetry(st.t, erpm, 0.02, 1.0, 12.0);
        core.feed_pose(st.t, st.x, st.y, st.yaw);
    }
}

// ---------------------------------------------------------------------
// (a) NONLINEAR velocity map recovery.
// ---------------------------------------------------------------------
bool test_velocity_fit_nonlinear_recovery() {
    bool ok = true;
    auto erpm_truth = [](double v) { return 4000.0 * v + 2000.0 * v * v * v; };

    mpc::CalibrationConfig cfg;
    cfg.velocity.target_per_bin = 30;
    mpc::CalibrationCore core(cfg);

    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-15.0, 15.0);
    std::function<double()> noise = [&]() { return dist(rng); };

    SynthState st;
    const std::vector<double> v_values = {0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35};
    double command_value = 500.0;
    for (double v : v_values) {
        drive_segment(core, st, v, 0.0, 2.0, 0.02, command_value, 0.5, erpm_truth, &noise);
        command_value += 500.0;  // new command per segment -- restarts the hold timer
    }

    const mpc::VelocityFitResult fit = core.fit_velocity_map();
    if (!fit.ok) {
        std::cerr << "    fit failed: " << fit.error << "\n";
        return false;
    }
    if (fit.table.size() != v_values.size()) {
        std::cerr << "    expected " << v_values.size() << " table points (noise amplitude well below "
                      "the per-step erpm gap -- no pooling expected), got "
                  << fit.table.size() << "\n";
        ok = false;
    }
    for (const auto& p : fit.table) {
        const double truth = erpm_truth(p.v);
        const double tol = std::max(15.0, 0.06 * std::fabs(truth));
        if (!near_eq(p.erpm, truth, tol)) {
            std::cerr << "    table point v=" << p.v << " erpm=" << p.erpm << " truth=" << truth
                      << " tol=" << tol << "\n";
            ok = false;
        }
    }
    if (fit.min_reliable_erpm <= 0.0) {
        std::cerr << "    expected a positive min_reliable_erpm, got " << fit.min_reliable_erpm << "\n";
        ok = false;
    }
    // Linear fallback should also be in the right ballpark (positive slope).
    if (fit.erpm_per_mps <= 0.0) {
        std::cerr << "    expected positive linear-fallback erpm_per_mps, got " << fit.erpm_per_mps << "\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (b) PAVA pooling fixes an injected non-monotone bin.
// ---------------------------------------------------------------------
mpc::DerivedSample make_vsample(double v, double erpm) {
    mpc::DerivedSample s;
    s.v = v;
    s.erpm = erpm;
    s.accepted = true;
    return s;
}

bool test_pava_pooling_fixes_nonmonotone_bin() {
    bool ok = true;
    // Bin centers 0.1/0.2/0.3 -- bin1's median erpm (900) is LOWER than
    // bin0's (1000), an injected violation that must be pooled away.
    const std::vector<double> centers = {0.1, 0.2, 0.3};
    std::vector<std::vector<mpc::DerivedSample>> bins(3);
    bins[0] = {make_vsample(0.1, 990.0), make_vsample(0.1, 1000.0), make_vsample(0.1, 1010.0)};  // median 1000, n=3
    bins[1] = {make_vsample(0.2, 900.0), make_vsample(0.2, 900.0)};                                // median 900, n=2
    bins[2] = {make_vsample(0.3, 2000.0), make_vsample(0.3, 2000.0)};                              // median 2000, n=2

    const mpc::VelocityFitResult fit = mpc::fit_velocity_map(centers, bins);
    if (!fit.ok) {
        std::cerr << "    fit failed: " << fit.error << "\n";
        return false;
    }
    if (fit.table.size() != 2) {
        std::cerr << "    expected bins 0+1 to pool into 1 point (2 total), got " << fit.table.size() << "\n";
        return false;
    }
    // Pooled point: weighted mean of (0.1,1000,n=3) and (0.2,900,n=2).
    const double expect_v = (0.1 * 3 + 0.2 * 2) / 5.0;
    const double expect_erpm = (1000.0 * 3 + 900.0 * 2) / 5.0;
    if (!near_eq(fit.table[0].v, expect_v, 1e-9) || !near_eq(fit.table[0].erpm, expect_erpm, 1e-9)) {
        std::cerr << "    pooled point mismatch: v=" << fit.table[0].v << " erpm=" << fit.table[0].erpm
                  << " expected v=" << expect_v << " erpm=" << expect_erpm << "\n";
        ok = false;
    }
    if (!near_eq(fit.table[1].v, 0.3, 1e-9) || !near_eq(fit.table[1].erpm, 2000.0, 1e-9)) {
        std::cerr << "    bin2 point mismatch: v=" << fit.table[1].v << " erpm=" << fit.table[1].erpm << "\n";
        ok = false;
    }
    if (!(fit.table[0].v < fit.table[1].v) || !(fit.table[0].erpm < fit.table[1].erpm)) {
        std::cerr << "    table not strictly increasing after pooling\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (c) Steadiness filter rejects during command-churn and acceleration
// phases, with rejection counters recording why.
// ---------------------------------------------------------------------
bool test_steadiness_rejections_recorded() {
    bool ok = true;
    mpc::CalibrationCore core;
    SynthState st;
    const double dt = 0.02;

    // Phase 1: command changes EVERY tick (never held >= 0.5s) at a
    // constant, non-accelerating velocity.
    for (int i = 0; i < 150; ++i) {  // 3s
        st.x += 0.2 * dt;
        st.t += dt;
        core.feed_command(st.t, "duty", 500.0 + i, 0.5);  // distinct value every tick
        core.feed_telemetry(st.t, 1500.0, 0.02, 1.0, 12.0);
        core.feed_pose(st.t, st.x, st.y, st.yaw);
    }
    if (core.accepted_samples() != 0) {
        std::cerr << "    phase 1 (command churn) should accept nothing, got "
                  << core.accepted_samples() << " accepted\n";
        ok = false;
    }
    auto counts = core.rejection_counts();
    if (counts[mpc::RejectReason::kCommandNotHeld] == 0) {
        std::cerr << "    expected kCommandNotHeld rejections during command churn\n";
        ok = false;
    }

    // Phase 2: command held, but velocity ramps steeply (dv/dt well above
    // the 0.15 m/s^2 threshold) for the whole phase -- never settles.
    for (int i = 0; i < 100; ++i) {  // 2s, v: 0.05 -> 0.35 (dv/dt = 0.15/... ~0.3 m/s^2 avg step)
        const double v = 0.05 + (0.30 * i) / 100.0;
        st.x += v * dt;
        st.t += dt;
        core.feed_command(st.t, "duty", 9999.0, 0.5);
        core.feed_telemetry(st.t, 4000.0 * v, 0.02, 1.0, 12.0);
        core.feed_pose(st.t, st.x, st.y, st.yaw);
    }
    counts = core.rejection_counts();
    if (counts[mpc::RejectReason::kAccelerating] == 0) {
        std::cerr << "    expected kAccelerating rejections during the velocity ramp\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (d) Steering recovery, including reverse-driving arcs.
// ---------------------------------------------------------------------
bool test_steering_fit_recovery_with_reverse() {
    bool ok = true;
    const double wheel_base = 0.29;
    auto delta_truth = [](double servo) { return 1.2 * (servo - 0.5); };  // rad, linear in servo

    mpc::CalibrationConfig cfg;
    cfg.wheel_base = wheel_base;
    cfg.steering.notches = {0.3, 0.4, 0.5, 0.6, 0.7};
    cfg.steering.target_per_notch = 20;
    mpc::CalibrationCore core(cfg);

    SynthState st;
    double command_value = 100.0;
    for (double servo : cfg.steering.notches) {
        const double delta = delta_truth(servo);
        const double v = 0.2;
        const double omega = (v / wheel_base) * std::tan(delta);
        drive_segment(core, st, v, omega, 2.0, 0.02, command_value, servo,
                      [](double) { return 0.0; });
        command_value += 100.0;
    }
    // Extra reverse-driving arc at servo=0.6 (already tested forward above) --
    // same delta, v negative: proves "reverse driving counts" toward the SAME notch.
    {
        const double servo = 0.6;
        const double delta = delta_truth(servo);
        const double v = -0.2;
        const double omega = (v / wheel_base) * std::tan(delta);
        drive_segment(core, st, v, omega, 2.0, 0.02, command_value, servo, [](double) { return 0.0; });
    }

    const mpc::SteeringFitResult fit = core.fit_steering_map();
    if (!fit.ok) {
        std::cerr << "    fit failed: " << fit.error << "\n";
        return false;
    }
    if (fit.points.size() != cfg.steering.notches.size()) {
        std::cerr << "    expected " << cfg.steering.notches.size() << " table points, got "
                  << fit.points.size() << "\n";
        ok = false;
    }
    for (const auto& p : fit.points) {
        const double truth = delta_truth(p.servo);
        if (!near_eq(p.delta, truth, 0.01)) {
            std::cerr << "    servo=" << p.servo << " delta=" << p.delta << " truth=" << truth << "\n";
            ok = false;
        }
    }
    // The notch that saw BOTH forward and reverse driving should still recover
    // correctly (reverse samples fold into the same median without flipping sign).
    bool found_06 = false;
    for (const auto& p : fit.points) {
        if (near_eq(p.servo, 0.6, 1e-6)) {
            found_06 = true;
            if (!near_eq(p.delta, delta_truth(0.6), 0.01)) {
                std::cerr << "    mixed forward+reverse notch servo=0.6 delta=" << p.delta
                          << " truth=" << delta_truth(0.6) << "\n";
                ok = false;
            }
        }
    }
    if (!found_06) {
        std::cerr << "    servo=0.6 notch missing from fitted table\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (e) CSV write/load-resume roundtrip preserves coverage.
// ---------------------------------------------------------------------
bool test_csv_roundtrip_preserves_coverage() {
    bool ok = true;
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "calibration_core_test_roundtrip.csv";
    std::error_code rm_ec;
    std::filesystem::remove(path, rm_ec);

    mpc::CalibrationConfig cfg;
    mpc::CalibrationCore core1(cfg);
    core1.set_session_csv_path(path.string());

    SynthState st;
    auto erpm_fn = [](double v) { return 4000.0 * v; };
    drive_segment(core1, st, 0.10, 0.0, 2.0, 0.02, 500.0, 0.5, erpm_fn);
    drive_segment(core1, st, 0.20, 0.0, 2.0, 0.02, 800.0, 0.5, erpm_fn);

    const mpc::CoverageReport cov1 = core1.velocity_coverage();
    if (core1.accepted_samples() == 0) {
        std::cerr << "    setup: expected some accepted samples before roundtrip\n";
        return false;
    }

    mpc::CalibrationCore core2(cfg);
    std::string err;
    if (!core2.load_session_csv(path.string(), &err)) {
        std::cerr << "    load_session_csv failed: " << err << "\n";
        std::filesystem::remove(path, rm_ec);
        return false;
    }
    const mpc::CoverageReport cov2 = core2.velocity_coverage();

    if (cov1.bins.size() != cov2.bins.size()) {
        std::cerr << "    bin count mismatch after reload\n";
        ok = false;
    } else {
        for (std::size_t i = 0; i < cov1.bins.size(); ++i) {
            if (cov1.bins[i].count != cov2.bins[i].count) {
                std::cerr << "    bin " << i << " (center=" << cov1.bins[i].center
                          << ") count mismatch: before=" << cov1.bins[i].count
                          << " after=" << cov2.bins[i].count << "\n";
                ok = false;
            }
        }
    }
    if (core2.accepted_samples() != core1.accepted_samples()) {
        std::cerr << "    accepted_samples mismatch: before=" << core1.accepted_samples()
                  << " after=" << core2.accepted_samples() << "\n";
        ok = false;
    }

    std::filesystem::remove(path, rm_ec);
    return ok;
}

// ---------------------------------------------------------------------
// (f) Export schema contains every contract field; refuses <2 points.
// ---------------------------------------------------------------------
bool test_export_schema_fields_and_refusal() {
    bool ok = true;

    mpc::VelocityFitResult vfit;
    vfit.table = {{0.1, 1000.0}, {0.2, 1900.0}, {0.3, 2850.0}};
    vfit.min_reliable_erpm = 1000.0;
    vfit.erpm_per_mps = 9000.0;
    vfit.offset_erpm = 100.0;
    vfit.rms = 12.5;
    vfit.n_samples = 90;
    vfit.bins_filled = 3;
    vfit.bins_total = 14;
    vfit.ok = true;

    const nlohmann::json vj = mpc::velocity_calib_to_json(vfit, "robotA");
    const std::vector<std::string> v_top_keys = {"version",           "kind",   "robot_name", "table",
                                                   "min_reliable_erpm", "linear_fallback", "meta"};
    for (const auto& k : v_top_keys) {
        if (!vj.contains(k)) {
            std::cerr << "    velocity_calib.json missing top-level key '" << k << "'\n";
            ok = false;
        }
    }
    if (vj.value("kind", "") != "velocity_calib") { std::cerr << "    kind mismatch\n"; ok = false; }
    if (!vj.contains("table") || !vj["table"].is_array() || vj["table"].empty() ||
        !vj["table"][0].contains("v") || !vj["table"][0].contains("erpm")) {
        std::cerr << "    table entries missing v/erpm\n";
        ok = false;
    }
    for (const char* k : {"erpm_per_mps", "offset_erpm", "rms"}) {
        if (!vj["linear_fallback"].contains(k)) { std::cerr << "    linear_fallback missing " << k << "\n"; ok = false; }
    }
    for (const char* k : {"n_samples", "bins_filled", "bins_total", "created_unix_time", "note"}) {
        if (!vj["meta"].contains(k)) { std::cerr << "    velocity meta missing " << k << "\n"; ok = false; }
    }

    mpc::SteeringFitResult sfit;
    sfit.points = {{-0.2, 0.3}, {0.0, 0.5}, {0.2, 0.7}};
    sfit.delta_min = -0.2;
    sfit.delta_max = 0.2;
    sfit.residual_rms = 0.005;
    sfit.n_samples = 60;
    sfit.ok = true;

    const nlohmann::json sj = mpc::steering_map_to_json(sfit, "robotA", 0.29);
    const std::vector<std::string> s_top_keys = {"version",   "kind",       "robot_name", "wheel_base",
                                                   "points",    "delta_min",  "delta_max",  "residual_rms", "meta"};
    for (const auto& k : s_top_keys) {
        if (!sj.contains(k)) {
            std::cerr << "    steering_angle_map.json missing top-level key '" << k << "'\n";
            ok = false;
        }
    }
    if (sj.value("kind", "") != "steering_angle_map") { std::cerr << "    kind mismatch\n"; ok = false; }
    if (!sj.contains("points") || !sj["points"].is_array() || sj["points"].empty() ||
        !sj["points"][0].contains("delta") || !sj["points"][0].contains("servo")) {
        std::cerr << "    points entries missing delta/servo\n";
        ok = false;
    }
    for (const char* k : {"n_samples", "created_unix_time", "note"}) {
        if (!sj["meta"].contains(k)) { std::cerr << "    steering meta missing " << k << "\n"; ok = false; }
    }

    // Refusal on <2 points.
    const std::filesystem::path out_path =
        std::filesystem::temp_directory_path() / "calibration_core_test_export_refuse.json";
    std::error_code rm_ec;
    std::filesystem::remove(out_path, rm_ec);

    mpc::VelocityFitResult empty_vfit;  // table.size() == 0
    std::string err;
    if (mpc::export_velocity_calib(empty_vfit, "robotA", out_path.string(), &err)) {
        std::cerr << "    export_velocity_calib should refuse a <2-point table\n";
        ok = false;
    }
    if (std::filesystem::exists(out_path)) {
        std::cerr << "    export_velocity_calib should not have created a file on refusal\n";
        ok = false;
    }

    mpc::SteeringFitResult one_point_sfit;
    one_point_sfit.points = {{0.0, 0.5}};
    if (mpc::export_steering_map(one_point_sfit, "robotA", 0.29, out_path.string(), &err)) {
        std::cerr << "    export_steering_map should refuse a <2-point table\n";
        ok = false;
    }

    // Successful export round-trips through the real file (atomic tmp+rename).
    if (!mpc::export_velocity_calib(vfit, "robotA", out_path.string(), &err)) {
        std::cerr << "    export_velocity_calib unexpectedly failed: " << err << "\n";
        ok = false;
    } else {
        std::ifstream in(out_path);
        nlohmann::json reread;
        in >> reread;
        if (reread.value("kind", "") != "velocity_calib") {
            std::cerr << "    re-read exported file has wrong kind\n";
            ok = false;
        }
    }
    std::filesystem::remove(out_path, rm_ec);

    return ok;
}

// ---------------------------------------------------------------------
// (g) Velocity-bin coverage/instruction logic picks the emptiest bin.
// ---------------------------------------------------------------------
bool test_velocity_coverage_picks_emptiest_bin() {
    bool ok = true;
    mpc::CalibrationConfig cfg;
    cfg.velocity.target_per_bin = 30;
    mpc::CalibrationCore core(cfg);

    // Bin centers (v_min=0.05..v_max=0.40 step 0.05, both directions) are
    // ascending: -0.40..-0.05, 0.05..0.40. Give every bin (both directions)
    // some samples EXCEPT v=0.20, which stays at 0 (the emptiest) -- the
    // negative-direction bins must be populated too, or they'd tie v=0.20
    // at count=0 and the (arbitrary, lowest-index) tie-break would pick one
    // of them instead.
    std::vector<mpc::DerivedSample> rows;
    for (double v : {-0.40, -0.35, -0.30, -0.25, -0.20, -0.15, -0.10, -0.05,
                      0.05, 0.10, 0.15, 0.25, 0.30, 0.35, 0.40}) {
        for (int i = 0; i < 10; ++i) {
            mpc::DerivedSample s;
            s.v = v;
            s.accepted = true;
            rows.push_back(s);
        }
    }
    core.ingest_derived_samples(rows);

    const mpc::CoverageReport cov = core.velocity_coverage();
    bool found_020 = false;
    int idx_020 = -1;
    for (std::size_t i = 0; i < cov.bins.size(); ++i) {
        if (near_eq(cov.bins[i].center, 0.20, 1e-9)) {
            found_020 = true;
            idx_020 = static_cast<int>(i);
            if (cov.bins[i].count != 0) {
                std::cerr << "    expected v=0.20 bin count 0, got " << cov.bins[i].count << "\n";
                ok = false;
            }
        }
    }
    if (!found_020) {
        std::cerr << "    v=0.20 bin not found in coverage report (bin ladder mismatch)\n";
        return false;
    }

    const mpc::NextInstruction instr = core.next_instruction(mpc::Task::kVelocity);
    if (instr.target_bin != idx_020) {
        std::cerr << "    next_instruction picked bin " << instr.target_bin << " (center="
                  << (instr.target_bin >= 0 ? cov.bins[instr.target_bin].center : -999.0)
                  << "), expected the empty v=0.20 bin (index " << idx_020 << ")\n";
        ok = false;
    }
    if (instr.text.find("0.20") == std::string::npos) {
        std::cerr << "    instruction text should mention the target speed 0.20, got: " << instr.text << "\n";
        ok = false;
    }
    if (instr.text.find("forward") == std::string::npos) {
        std::cerr << "    instruction text for a positive-v bin should say 'forward', got: " << instr.text
                  << "\n";
        ok = false;
    }
    return ok;
}

}  // namespace

int main() {
    using TestFn = std::function<bool()>;
    const std::vector<std::pair<std::string, TestFn>> tests = {
        {"fit_velocity_map: recovers a NONLINEAR erpm(v) map within a few percent at bin centers",
         test_velocity_fit_nonlinear_recovery},
        {"fit_velocity_map: PAVA pooling fixes an injected non-monotone bin",
         test_pava_pooling_fixes_nonmonotone_bin},
        {"CalibrationCore: steadiness filter rejects command-churn and acceleration phases, "
         "rejection counters record why",
         test_steadiness_rejections_recorded},
        {"fit_steering_map: recovers delta(servo) within ~0.01 rad, including reverse-driving arcs",
         test_steering_fit_recovery_with_reverse},
        {"CalibrationCore: CSV write/load_session_csv roundtrip preserves coverage",
         test_csv_roundtrip_preserves_coverage},
        {"export_velocity_calib/export_steering_map: schema contains every contract field, "
         "refuses a <2-point table",
         test_export_schema_fields_and_refusal},
        {"CalibrationCore: velocity coverage/next_instruction picks the emptiest bin",
         test_velocity_coverage_picks_emptiest_bin},
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
    std::cout << "[  FAILED  ] " << failed << " of " << tests.size() << " tests.\n";
    return 1;
}
