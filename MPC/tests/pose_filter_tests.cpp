// Lightweight unit tests for mpc::PoseFilter (MPC/include/mpc/PoseFilter.h),
// mirroring the harness style of MPC/tests/calibration_core_tests.cpp: plain
// bool test_xxx() functions registered in main(), no gtest dependency, no
// sockets. Every test drives deterministic synthetic 120 Hz tracks (seeded
// RNG for reproducibility) through the real mpc::PoseFilter -- see
// doc/MOCAP_POSE_FILTER_PLAN.md for the design this verifies.

#include "mpc/PoseFilter.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHz = 120.0;
constexpr double kDt = 1.0 / kHz;

double wrap(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

double yaw_err(double a, double b) { return std::fabs(wrap(a - b)); }

mpc::PoseFilterConfig enabled_config() {
    mpc::PoseFilterConfig cfg;
    cfg.enabled = true;
    return cfg;
}

// ---------------------------------------------------------------------
// Deterministic Gaussian noise source (seeded, reproducible across runs).
// ---------------------------------------------------------------------
struct Noise {
    std::mt19937 rng;
    std::normal_distribution<double> pos_dist;
    std::normal_distribution<double> yaw_dist;
    explicit Noise(unsigned seed, double pos_sigma, double yaw_sigma)
        : rng(seed), pos_dist(0.0, pos_sigma), yaw_dist(0.0, yaw_sigma) {}
    double pos() { return pos_dist(rng); }
    double yaw() { return yaw_dist(rng); }
};

// ---------------------------------------------------------------------
// (a) Clean straight-line motion: v=0.3 m/s along +x, 0.5mm/0.1deg noise.
// Expect: zero rejects, posterior within 2mm/0.3deg of truth at steady state.
// ---------------------------------------------------------------------
bool test_clean_straight_motion() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(1, 0.0005, 0.1 * kPi / 180.0);
    const double v = 0.3;
    const int n = static_cast<int>(2.0 * kHz);  // 2 seconds.
    int rejects = 0;
    double last_x = 0.0, last_y = 0.0, last_yaw = 0.0, truth_x = 0.0;
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        truth_x = v * t;
        const double zx = truth_x + noise.pos();
        const double zy = 0.0 + noise.pos();
        const double zyaw = wrap(0.0 + noise.yaw());
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (!out.accepted) ++rejects;
        last_x = out.x;
        last_y = out.y;
        last_yaw = out.yaw;
    }
    bool ok = true;
    if (rejects != 0) {
        std::cerr << "  clean straight motion: expected 0 rejects, got " << rejects << "\n";
        ok = false;
    }
    if (std::fabs(last_x - truth_x) > 0.002 || std::fabs(last_y - 0.0) > 0.002) {
        std::cerr << "  clean straight motion: posterior position off by more than 2mm (x="
                  << last_x << " truth=" << truth_x << " y=" << last_y << ")\n";
        ok = false;
    }
    if (yaw_err(last_yaw, 0.0) > 0.3 * kPi / 180.0) {
        std::cerr << "  clean straight motion: posterior yaw off by more than 0.3deg\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (b) Clean cornering: yaw_rate=1 rad/s, v=0.3 m/s tangential. Zero rejects.
// ---------------------------------------------------------------------
bool test_clean_cornering() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(2, 0.0005, 0.1 * kPi / 180.0);
    const double v = 0.3, w = 1.0;
    const int n = static_cast<int>(1.0 * kHz);  // 1 second -> ~57 degrees of turn.
    double x = 0.0, y = 0.0, yaw = 0.0;
    int rejects = 0;
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        if (i > 0) {
            x += v * std::cos(yaw) * kDt;
            y += v * std::sin(yaw) * kDt;
            yaw = wrap(yaw + w * kDt);
        }
        const double zx = x + noise.pos();
        const double zy = y + noise.pos();
        const double zyaw = wrap(yaw + noise.yaw());
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (!out.accepted) ++rejects;
    }
    if (rejects != 0) {
        std::cerr << "  clean cornering: expected 0 rejects, got " << rejects << "\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------
// (c) Direction reversal: 1 m/s^2 decel from +0.3 m/s to -0.3 m/s and back.
// Zero rejects (process noise q_acc=3.0 comfortably covers 1 m/s^2).
// ---------------------------------------------------------------------
bool test_direction_reversal() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(3, 0.0005, 0.1 * kPi / 180.0);
    const double a = 1.0;  // m/s^2, decel then accel.
    const int n = static_cast<int>(1.5 * kHz);
    double x = 0.0, v = 0.3;
    int rejects = 0;
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        if (i > 0) {
            // Ramp down to -0.3 over the first ~0.6s, then hold.
            if (v > -0.3) v -= a * kDt;
            if (v < -0.3) v = -0.3;
            x += v * kDt;
        }
        const double zx = x + noise.pos();
        const double zy = 0.0 + noise.pos();
        const double zyaw = wrap(0.0 + noise.yaw());
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (!out.accepted) ++rejects;
    }
    if (rejects != 0) {
        std::cerr << "  direction reversal: expected 0 rejects, got " << rejects << "\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------
// (d) Single-frame 20cm translation spike: rejected, output stays within
// 1cm of truth, next frame accepted.
// ---------------------------------------------------------------------
bool test_single_frame_translation_spike() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(4, 0.0005, 0.1 * kPi / 180.0);
    const double v = 0.3;
    const int warmup = 60;
    const int spike_index = warmup;
    double truth_x_at_spike = 0.0;
    bool spike_rejected = false;
    bool spike_close_to_truth = false;
    bool next_accepted = false;
    for (int i = 0; i <= spike_index + 1; ++i) {
        const double t = i * kDt;
        const double truth_x = v * t;
        double zx = truth_x + noise.pos();
        const double zy = 0.0 + noise.pos();
        const double zyaw = wrap(0.0 + noise.yaw());
        if (i == spike_index) {
            zx += 0.20;
            truth_x_at_spike = truth_x;
        }
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (i == spike_index) {
            spike_rejected = !out.accepted;
            spike_close_to_truth = std::fabs(out.x - truth_x_at_spike) <= 0.01;
        } else if (i == spike_index + 1) {
            next_accepted = out.accepted;
        }
    }
    bool ok = true;
    if (!spike_rejected) {
        std::cerr << "  single-frame translation spike: spike frame was NOT rejected\n";
        ok = false;
    }
    if (!spike_close_to_truth) {
        std::cerr << "  single-frame translation spike: output departed >1cm from truth\n";
        ok = false;
    }
    if (!next_accepted) {
        std::cerr << "  single-frame translation spike: frame after spike was not accepted\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (e) Single-frame 90deg yaw flip: rejected via the yaw gate specifically.
// ---------------------------------------------------------------------
bool test_single_frame_yaw_flip() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(5, 0.0005, 0.1 * kPi / 180.0);
    const int warmup = 60;
    bool rejected = false;
    double d2_yaw_at_spike = 0.0, d2_pos_at_spike = 0.0;
    for (int i = 0; i <= warmup; ++i) {
        const double t = i * kDt;
        const double zx = 0.0 + noise.pos();
        const double zy = 0.0 + noise.pos();
        double zyaw = wrap(0.0 + noise.yaw());
        if (i == warmup) {
            zyaw = wrap(zyaw + kPi / 2.0);
        }
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (i == warmup) {
            rejected = !out.accepted;
            d2_yaw_at_spike = out.d2_yaw;
            d2_pos_at_spike = out.d2_pos;
        }
    }
    bool ok = true;
    if (!rejected) {
        std::cerr << "  single-frame yaw flip: not rejected\n";
        ok = false;
    }
    if (!(d2_yaw_at_spike > mpc::PoseFilterConfig().gate_chi2_yaw)) {
        std::cerr << "  single-frame yaw flip: d2_yaw (" << d2_yaw_at_spike
                  << ") did not exceed the yaw gate\n";
        ok = false;
    }
    if (!(d2_pos_at_spike <= mpc::PoseFilterConfig().gate_chi2_pos)) {
        std::cerr << "  single-frame yaw flip: d2_pos unexpectedly also exceeded its gate ("
                  << d2_pos_at_spike << ") -- test setup should isolate the yaw gate\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (f) 3-frame spike (fixed 20cm offset, stationary truth): all 3 rejected,
// then recovery on the frame back on the true track.
// ---------------------------------------------------------------------
bool test_three_frame_spike_recovery() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(6, 0.0005, 0.1 * kPi / 180.0);
    const int warmup = 60;
    std::vector<bool> spike_accepted;
    bool recovered = false;
    for (int i = 0; i <= warmup + 3; ++i) {
        const double t = i * kDt;
        double zx = 0.0 + noise.pos();
        const double zy = 0.0 + noise.pos();
        const double zyaw = wrap(0.0 + noise.yaw());
        const bool in_spike = (i >= warmup && i < warmup + 3);
        if (in_spike) zx += 0.20;
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (in_spike) spike_accepted.push_back(out.accepted);
        if (i == warmup + 3) recovered = out.accepted;
    }
    bool ok = true;
    if (spike_accepted.size() != 3) {
        std::cerr << "  three-frame spike: internal test error (wrong sample count)\n";
        return false;
    }
    for (size_t i = 0; i < spike_accepted.size(); ++i) {
        if (spike_accepted[i]) {
            std::cerr << "  three-frame spike: spike frame " << i << " was unexpectedly accepted\n";
            ok = false;
        }
    }
    if (!recovered) {
        std::cerr << "  three-frame spike: frame after the spike was not accepted\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (g) Genuine sustained 30cm step (teleport): rejected until reinit_after_s,
// then reinit=true and tracking resumes.
// ---------------------------------------------------------------------
bool test_sustained_step_reinit() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(7, 0.0005, 0.1 * kPi / 180.0);
    const int warmup = 60;
    const double step = 0.30;
    const int post_step_frames = static_cast<int>(0.5 * kHz);  // 0.5s after the step.
    bool reinit_seen = false;
    bool reinit_within_budget = false;
    int reinit_index_after_step = -1;
    bool accepted_after_reinit = true;
    for (int i = 0; i < warmup + post_step_frames; ++i) {
        const double t = i * kDt;
        double zx = 0.0 + noise.pos();
        const double zy = 0.0 + noise.pos();
        const double zyaw = wrap(0.0 + noise.yaw());
        if (i >= warmup) zx += step;  // genuinely, permanently at the new position.
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (i >= warmup && out.reinit && !reinit_seen) {
            reinit_seen = true;
            reinit_index_after_step = i - warmup;
        }
        if (reinit_seen && i > warmup + reinit_index_after_step) {
            if (!out.accepted) accepted_after_reinit = false;
        }
    }
    // reinit_after_s default 0.25s == 30 frames at 120Hz; allow a little
    // slack (self-consistency needs >=2 buffered rejects too).
    reinit_within_budget =
        reinit_seen && reinit_index_after_step <= static_cast<int>(0.25 * kHz) + 5;
    bool ok = true;
    if (!reinit_seen) {
        std::cerr << "  sustained step: no reinit ever fired\n";
        ok = false;
    } else if (!reinit_within_budget) {
        std::cerr << "  sustained step: reinit fired too late (frame " << reinit_index_after_step
                  << " after the step)\n";
        ok = false;
    }
    if (!accepted_after_reinit) {
        std::cerr << "  sustained step: frames after reinit were not all accepted\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (h) 1s gap then resume 40cm away: immediate reinit, no rejection.
// ---------------------------------------------------------------------
bool test_gap_then_resume_reinit() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(8, 0.0005, 0.1 * kPi / 180.0);
    mpc::PoseFilterOutput out0 = filt.step(0.0, 0.0, 0.0, 0.0);
    if (!out0.accepted || !out0.reinit) {
        std::cerr << "  gap then resume: internal test error (first sample not accepted/reinit)\n";
        return false;
    }
    mpc::PoseFilterOutput out1 = filt.step(1.05, 0.40, 0.0, 0.0);
    bool ok = true;
    if (!out1.accepted) {
        std::cerr << "  gap then resume: post-gap sample was rejected (should reinit immediately)\n";
        ok = false;
    }
    if (!out1.reinit) {
        std::cerr << "  gap then resume: post-gap sample did not report reinit=true\n";
        ok = false;
    }
    if (std::fabs(out1.x - 0.40) > 1e-9) {
        std::cerr << "  gap then resume: state not snapped to the new measurement\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (i) Yaw wrap across +-pi while cornering through it: zero rejects.
// ---------------------------------------------------------------------
bool test_yaw_wrap_across_pi() {
    mpc::PoseFilter filt(enabled_config());
    Noise noise(9, 0.0005, 0.1 * kPi / 180.0);
    const double v = 0.3, w = 1.0;
    // Start near +pi so the turn crosses the wrap boundary quickly.
    double x = 0.0, y = 0.0, yaw = kPi - 0.3;
    const int n = static_cast<int>(1.0 * kHz);
    int rejects = 0;
    bool crossed_wrap = false;
    double prev_yaw = yaw;
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        if (i > 0) {
            x += v * std::cos(yaw) * kDt;
            y += v * std::sin(yaw) * kDt;
            yaw = wrap(yaw + w * kDt);
        }
        if (prev_yaw > kPi / 2.0 && yaw < -kPi / 2.0) crossed_wrap = true;
        prev_yaw = yaw;
        const double zx = x + noise.pos();
        const double zy = y + noise.pos();
        const double zyaw = wrap(yaw + noise.yaw());
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (!out.accepted) ++rejects;
    }
    bool ok = true;
    if (!crossed_wrap) {
        std::cerr << "  yaw wrap: internal test error -- track never crossed +-pi\n";
        ok = false;
    }
    if (rejects != 0) {
        std::cerr << "  yaw wrap: expected 0 rejects, got " << rejects << "\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (j) Non-increasing timestamp is ignored.
// ---------------------------------------------------------------------
bool test_nonincreasing_timestamp_ignored() {
    mpc::PoseFilter filt(enabled_config());
    mpc::PoseFilterOutput out0 = filt.step(1.0, 0.0, 0.0, 0.0);
    mpc::PoseFilterOutput out_same_t = filt.step(1.0, 5.0, 5.0, 1.0);
    mpc::PoseFilterOutput out_earlier = filt.step(0.5, -5.0, -5.0, -1.0);
    bool ok = true;
    if (out_same_t.accepted) {
        std::cerr << "  non-increasing timestamp: equal-t sample was not ignored\n";
        ok = false;
    }
    if (out_earlier.accepted) {
        std::cerr << "  non-increasing timestamp: earlier-t sample was not ignored\n";
        ok = false;
    }
    if (std::fabs(out_same_t.x - out0.x) > 1e-12 || std::fabs(out_earlier.x - out0.x) > 1e-12) {
        std::cerr << "  non-increasing timestamp: ignored sample changed the published output\n";
        ok = false;
    }
    return ok;
}

// ---------------------------------------------------------------------
// (k) enabled=false: pure pass-through, output equals input exactly.
// ---------------------------------------------------------------------
bool test_disabled_is_passthrough() {
    mpc::PoseFilterConfig cfg;
    cfg.enabled = false;
    mpc::PoseFilter filt(cfg);
    Noise noise(11, 0.05, 0.2);  // large noise/spikes -- must still pass through verbatim.
    bool ok = true;
    for (int i = 0; i < 50; ++i) {
        const double t = i * kDt;
        const double zx = 0.3 * t + noise.pos() + (i == 25 ? 0.5 : 0.0);  // include a "spike".
        const double zy = noise.pos();
        const double zyaw = wrap(noise.yaw());
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (out.x != zx || out.y != zy || out.yaw != wrap(zyaw)) {
            std::cerr << "  disabled passthrough: output != input at i=" << i << "\n";
            ok = false;
        }
        if (!out.accepted) {
            std::cerr << "  disabled passthrough: accepted was false at i=" << i << "\n";
            ok = false;
        }
    }
    return ok;
}

// ---------------------------------------------------------------------
// (l) Plausibility rule alone (statistical gates disabled via huge chi^2
// thresholds) catches a 5cm single-frame spike at 120Hz.
// ---------------------------------------------------------------------
bool test_plausibility_rule_alone_catches_spike() {
    mpc::PoseFilterConfig cfg = enabled_config();
    cfg.gate_chi2_pos = 1e9;
    cfg.gate_chi2_yaw = 1e9;
    cfg.gate_chi2_all = 1e9;
    mpc::PoseFilter filt(cfg);
    Noise noise(12, 0.0005, 0.1 * kPi / 180.0);
    const int warmup = 60;
    bool rejected = false;
    for (int i = 0; i <= warmup; ++i) {
        const double t = i * kDt;
        double zx = 0.0 + noise.pos();
        const double zy = 0.0 + noise.pos();
        const double zyaw = wrap(noise.yaw());
        if (i == warmup) zx += 0.05;  // 5cm spike / 8.3ms => ~6 m/s implied speed.
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (i == warmup) rejected = !out.accepted;
    }
    if (!rejected) {
        std::cerr << "  plausibility-alone: 5cm/120Hz spike was NOT rejected despite disabled "
                     "statistical gates\n";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------
// (m) Regression fixture: MPC/tests/data/mocap_clean_excerpt.csv is a
// ~2000-frame excerpt of a REAL recorded robot2 session (optitrack_zmq_
// bridge --log-csv, 2026-08-28 -- see doc/MOCAP_POSE_FILTER_PLAN.md's
// "Implementation status"), confirmed via pose_filter_replay to produce
// zero rejects with the default config. Replaying it here through the core
// directly (not the CLI tool) keeps that guarantee under regression.
// ---------------------------------------------------------------------
bool test_clean_excerpt_fixture_replay_zero_rejects() {
    namespace fs = std::filesystem;
    const fs::path csv_path = fs::path(__FILE__).parent_path() / "data" / "mocap_clean_excerpt.csv";
    std::ifstream f(csv_path);
    if (!f.is_open()) {
        std::cerr << "  clean excerpt fixture: could not open " << csv_path << "\n";
        return false;
    }
    std::string header_line;
    std::getline(f, header_line);
    std::vector<std::string> header;
    {
        std::stringstream ss(header_line);
        std::string tok;
        while (std::getline(ss, tok, ',')) header.push_back(tok);
    }
    auto col_index = [&](const std::string& name) -> int {
        for (size_t i = 0; i < header.size(); ++i) {
            if (header[i] == name) return static_cast<int>(i);
        }
        return -1;
    };
    const int i_t = col_index("t_arrival");
    const int i_x = col_index("planar_x");
    const int i_y = col_index("planar_y");
    const int i_yaw = col_index("planar_yaw");
    if (i_t < 0 || i_x < 0 || i_y < 0 || i_yaw < 0) {
        std::cerr << "  clean excerpt fixture: missing an expected column in the header\n";
        return false;
    }

    mpc::PoseFilter filt(enabled_config());
    int rejects = 0, rows_seen = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) fields.push_back(tok);
        if (static_cast<int>(fields.size()) <= std::max({i_t, i_x, i_y, i_yaw})) continue;
        const double t = std::stod(fields[i_t]);
        const double zx = std::stod(fields[i_x]);
        const double zy = std::stod(fields[i_y]);
        const double zyaw = std::stod(fields[i_yaw]);
        mpc::PoseFilterOutput out = filt.step(t, zx, zy, zyaw);
        if (!out.accepted) ++rejects;
        ++rows_seen;
    }
    bool ok = true;
    if (rows_seen < 1900) {
        std::cerr << "  clean excerpt fixture: expected ~2000 rows, only read " << rows_seen << "\n";
        ok = false;
    }
    if (rejects != 0) {
        std::cerr << "  clean excerpt fixture: expected 0 rejects, got " << rejects << "\n";
        ok = false;
    }
    return ok;
}

}  // namespace

int main() {
    using TestFn = std::function<bool()>;
    const std::vector<std::pair<std::string, TestFn>> tests = {
        {"(a) clean straight motion: zero rejects, posterior within 2mm/0.3deg",
         test_clean_straight_motion},
        {"(b) clean cornering: zero rejects", test_clean_cornering},
        {"(c) direction reversal (1 m/s^2): zero rejects", test_direction_reversal},
        {"(d) single-frame 20cm translation spike: rejected, output stays near truth, recovers",
         test_single_frame_translation_spike},
        {"(e) single-frame 90deg yaw flip: rejected via the yaw gate", test_single_frame_yaw_flip},
        {"(f) 3-frame spike: all rejected, then recovery", test_three_frame_spike_recovery},
        {"(g) genuine sustained 30cm step: rejected until reinit_after_s, then reinit + resume",
         test_sustained_step_reinit},
        {"(h) 1s gap then resume 40cm away: immediate reinit, no rejection",
         test_gap_then_resume_reinit},
        {"(i) yaw wrap across +-pi while cornering: zero rejects", test_yaw_wrap_across_pi},
        {"(j) non-increasing timestamp is ignored", test_nonincreasing_timestamp_ignored},
        {"(k) enabled=false: pure pass-through", test_disabled_is_passthrough},
        {"(l) plausibility rule alone catches a 5cm/120Hz spike",
         test_plausibility_rule_alone_catches_spike},
        {"(m) MPC/tests/data/mocap_clean_excerpt.csv replays with zero rejects",
         test_clean_excerpt_fixture_replay_zero_rejects},
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
