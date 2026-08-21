#include "mpc/MotorCalibCore.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace mpc {

// =======================================================================
// VelocityEstimator
// =======================================================================

VelocityEstimate VelocityEstimator::feed(double t, double x, double y, double yaw) {
    VelocityEstimate out;

    if (have_prev_pose_) {
        const double dt = t - prev_t_;
        if (dt > 1e-3) {
            const double dx = x - prev_x_;
            const double dy = y - prev_y_;
            last_v_raw_ = (dx * std::cos(yaw) + dy * std::sin(yaw)) / dt;
            have_v_raw_ = true;
        }
        // else: samples too close together in time to trust a finite
        // difference -- keep last_v_raw_ as-is (matches MPC/src/main.cpp's
        // est_state.v retention, see this class's doc comment).
    }
    prev_t_ = t;
    prev_x_ = x;
    prev_y_ = y;
    have_prev_pose_ = true;

    out.v_raw = last_v_raw_;
    out.v_raw_valid = have_v_raw_;

    window_.push_back({t, last_v_raw_});
    if (window_.size() > kWindow) {
        window_.pop_front();
    }

    if (window_.size() == kWindow) {
        double sum = 0.0;
        for (const auto& p : window_) sum += p.v;
        const double v_smooth = sum / static_cast<double>(kWindow);
        const double t_center = window_[kWindow / 2].t;

        out.t_smooth = t_center;
        out.v_smooth = v_smooth;
        out.smooth_valid = true;

        if (have_prev_smooth_) {
            const double dts = t_center - prev_smooth_t_;
            if (dts > 1e-3) {
                out.a_smooth = (v_smooth - prev_smooth_v_) / dts;
                out.a_valid = true;
            }
        }
        prev_smooth_t_ = t_center;
        prev_smooth_v_ = v_smooth;
        have_prev_smooth_ = true;
    }

    return out;
}

void VelocityEstimator::reset() { *this = VelocityEstimator(); }

// =======================================================================
// LineFrame
// =======================================================================

void LineFrame::capture(double x0, double y0, double initial_yaw, std::optional<double> yaw_override,
                         bool force) {
    if (captured_ && !force) return;
    x0_ = x0;
    y0_ = y0;
    line_yaw_ = yaw_override.has_value() ? *yaw_override : initial_yaw;
    captured_ = true;
}

double LineFrame::s(double x, double y) const {
    return (x - x0_) * std::cos(line_yaw_) + (y - y0_) * std::sin(line_yaw_);
}

double LineFrame::lateral(double x, double y) const {
    return -(x - x0_) * std::sin(line_yaw_) + (y - y0_) * std::cos(line_yaw_);
}

// =======================================================================
// RawMode / TrialAbortReason string conversions
// =======================================================================

std::string to_string(RawMode mode) {
    switch (mode) {
        case RawMode::kDuty:
            return "duty";
        case RawMode::kErpm:
            return "erpm";
        case RawMode::kCurrent:
            return "current";
    }
    return "duty";
}

RawMode raw_mode_from_string(const std::string& s) {
    if (s == "duty") return RawMode::kDuty;
    if (s == "erpm") return RawMode::kErpm;
    if (s == "current") return RawMode::kCurrent;
    throw std::invalid_argument("mpc::raw_mode_from_string: unknown mode '" + s + "'");
}

std::string to_string(TrialAbortReason reason) {
    switch (reason) {
        case TrialAbortReason::kNone:
            return "none";
        case TrialAbortReason::kMocapStale:
            return "mocap_stale";
        case TrialAbortReason::kOutOfEnvelope:
            return "out_of_envelope";
        case TrialAbortReason::kOverSpeed:
            return "over_speed";
        case TrialAbortReason::kTimeout:
            return "timeout";
        case TrialAbortReason::kInsufficientRoomAtArm:
            return "insufficient_room_at_arm";
        case TrialAbortReason::kDriverRefused:
            return "driver_refused";
    }
    return "unknown";
}

// =======================================================================
// TrialRunner
// =======================================================================

TrialRunner::TrialRunner(IDriverClient& driver, TrialRunnerConfig cfg,
                          std::optional<double> line_yaw_override)
    : driver_(driver), cfg_(cfg), line_yaw_override_(line_yaw_override) {}

void TrialRunner::start_trial(TrialSpec spec) {
    spec_ = spec;
    rows_.clear();
    telemetry_buffer_.clear();
    state_ = TrialState::kArm;
    abort_reason_ = TrialAbortReason::kNone;
    abort_message_.clear();
    arm_stationary_since_ = -1.0;
    // NOTE: line_/vel_/last_pose_*/last_v_* are DELIBERATELY NOT reset here
    // -- they are session-level state that stays warm across trials (see
    // class doc comment), so the velocity-smoothing window never needs to
    // refill at the start of every single trial.
}

void TrialRunner::enter_run(double now) {
    const DriverReply src = driver_.set_source("calib");
    if (!src.ok) {
        abort(TrialAbortReason::kDriverRefused, "set_source(calib) refused: " + src.error);
        return;
    }
    driver_.servo(cfg_.center_servo_value);  // best-effort: straight-line run, no steering needed.
    state_ = TrialState::kRun;
    run_entered_time_ = now;
    // Forces the very first raw() issuance on the NEXT step() call
    // regardless of `now`'s value.
    last_raw_issue_time_ = -std::numeric_limits<double>::infinity();
}

void TrialRunner::log_row() {
    TrialRow row;
    row.t = last_pose_t_;
    row.x = last_pose_x_;
    row.y = last_pose_y_;
    row.yaw = last_pose_yaw_;
    row.s = line_.s(last_pose_x_, last_pose_y_);
    row.v_raw = last_v_raw_;
    row.v_smooth = last_v_smooth_;
    row.cmd_mode = spec_.mode;
    row.cmd_value = spec_.value;

    if (!telemetry_buffer_.empty()) {
        std::size_t best = 0;
        double best_d = std::fabs(telemetry_buffer_[0].t - row.t);
        for (std::size_t i = 1; i < telemetry_buffer_.size(); ++i) {
            const double d = std::fabs(telemetry_buffer_[i].t - row.t);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        row.has_telemetry = true;
        row.erpm = telemetry_buffer_[best].erpm;
        row.duty = telemetry_buffer_[best].duty;
        row.current_motor = telemetry_buffer_[best].current_motor;
        row.v_in = telemetry_buffer_[best].v_in;
    }

    rows_.push_back(row);
}

bool TrialRunner::abort(TrialAbortReason reason, const std::string& message) {
    driver_.stop();
    state_ = TrialState::kAborted;
    abort_reason_ = reason;
    abort_message_ = message;
    return false;
}

bool TrialRunner::step(double now, bool have_pose, const CalibSample& pose, bool have_telemetry,
                        const TelemetrySample& telemetry) {
    // Always-on bookkeeping, regardless of state() -- keeps the LineFrame
    // capture and the VelocityEstimator's smoothing window warm from the
    // very first pose this instance ever sees, even before start_trial()
    // is first called.
    if (have_pose) {
        const VelocityEstimate est = vel_.feed(pose.t, pose.x, pose.y, pose.yaw);
        if (est.v_raw_valid) last_v_raw_ = est.v_raw;
        if (est.smooth_valid) {
            last_v_smooth_ = est.v_smooth;
            have_valid_smooth_ = true;
        } else if (!have_valid_smooth_) {
            // Smoothing window not full yet -- fall back to raw so logged
            // rows never carry a meaningless zero.
            last_v_smooth_ = last_v_raw_;
        }
        if (!line_.captured()) {
            line_.capture(pose.x, pose.y, pose.yaw, line_yaw_override_);
        }
        last_pose_x_ = pose.x;
        last_pose_y_ = pose.y;
        last_pose_yaw_ = pose.yaw;
        last_pose_t_ = pose.t;
        last_pose_wall_t_ = now;
        have_seen_pose_ = true;
    }
    if (have_telemetry) {
        telemetry_buffer_.push_back(telemetry);
    }

    if (state_ == TrialState::kIdle || state_ == TrialState::kDone || state_ == TrialState::kAborted) {
        return false;
    }

    // From here on a trial is actively ARM/RUN/STOPPING -- stale/missing
    // mocap is always fatal (checked before any state-specific logic).
    const bool mocap_fresh =
        have_seen_pose_ && ((now - last_pose_wall_t_) * 1000.0 <= cfg_.mocap_stale_abort_ms);
    if (!mocap_fresh) {
        return abort(TrialAbortReason::kMocapStale,
                      "mocap sample age exceeded mocap_stale_abort_ms");
    }

    switch (state_) {
        case TrialState::kArm: {
            const bool stationary = std::fabs(last_v_raw_) < cfg_.arm_stationary_v_mps;
            if (!stationary) {
                arm_stationary_since_ = -1.0;
            } else if (arm_stationary_since_ < 0.0) {
                arm_stationary_since_ = now;
            }
            if (stationary && arm_stationary_since_ >= 0.0 &&
                (now - arm_stationary_since_) >= cfg_.arm_settle_s) {
                const double s0 = line_.s(last_pose_x_, last_pose_y_);
                const double envelope_min = cfg_.end_margin_m;
                const double envelope_max = cfg_.line_length_m - cfg_.end_margin_m;
                const int dir = (spec_.value >= 0.0) ? 1 : -1;
                // room_pos/room_neg: usable travel (m) from s0 to each physical envelope
                // edge, independent of commanded direction. room_ahead_/room_behind_ then
                // just relabel these by whichever edge the commanded direction is heading
                // toward vs. away from -- so the RUN-phase check below can catch BOTH an
                // overrun in the commanded direction and a wrong-direction excursion (e.g.
                // reversed motor polarity, or rollback on a slope) toward the other edge.
                const double room_pos = envelope_max - s0;
                const double room_neg = s0 - envelope_min;
                const double room = (dir > 0) ? room_pos : room_neg;
                if (room <= 0.0) {
                    return abort(TrialAbortReason::kInsufficientRoomAtArm,
                                 "no usable line room remains in the commanded direction");
                }
                trial_start_s_ = s0;
                room_ahead_ = room;
                room_behind_ = (dir > 0) ? room_neg : room_pos;
                enter_run(now);
                if (state_ == TrialState::kAborted) return false;
            }
            break;
        }
        case TrialState::kRun: {
            if (now - last_raw_issue_time_ >= cfg_.reissue_interval_s) {
                const DriverReply r = driver_.raw(spec_.mode, spec_.value, spec_.ttl_ms);
                last_raw_issue_time_ = now;
                if (!r.ok) {
                    return abort(TrialAbortReason::kDriverRefused,
                                 "raw() refused mid-run: " + r.error);
                }
            }
            if (have_pose) {
                log_row();
            }
            if (std::fabs(last_v_raw_) > cfg_.max_speed_abort_mps) {
                return abort(TrialAbortReason::kOverSpeed, "|v| exceeded max_speed_abort_mps");
            }
            {
                const double s_now = line_.s(last_pose_x_, last_pose_y_);
                const int dir = (spec_.value >= 0.0) ? 1 : -1;
                const double progress = dir * (s_now - trial_start_s_);
                if (progress > room_ahead_) {
                    return abort(TrialAbortReason::kOutOfEnvelope,
                                 "|s| beyond the usable envelope for this trial's direction");
                }
                // Wrong-direction excursion: motion AWAY from the commanded direction
                // (reversed motor polarity, or rollback on a slope -- exactly the kind of
                // fault a calibration campaign exists to catch) makes `progress` negative
                // and grows without bound there, since the check above only ever fires on
                // the positive (commanded-direction) side. Guard with `progress < 0.0` so a
                // correctly-tracking trial is never affected by this branch even when
                // room_behind_ is small/negative (e.g. trial_start_s_ itself sat close to
                // the opposite envelope edge) -- only an ACTUAL step backward past
                // trial_start_s_ can trip it.
                if (progress < 0.0 && -progress > room_behind_) {
                    return abort(TrialAbortReason::kOutOfEnvelope,
                                 "|s| beyond the usable envelope in the direction OPPOSITE the "
                                 "commanded one (wrong-direction motion)");
                }
            }
            if (now - run_entered_time_ >= cfg_.max_duration_s) {
                return abort(TrialAbortReason::kTimeout, "trial wall-time exceeded max_duration_s");
            }
            if (now - run_entered_time_ >= cfg_.settle_s) {
                driver_.stop();
                state_ = TrialState::kStopping;
            }
            break;
        }
        case TrialState::kStopping: {
            if (std::fabs(last_v_raw_) < cfg_.arm_stationary_v_mps) {
                state_ = TrialState::kDone;
                return false;
            }
            if (now - run_entered_time_ >= cfg_.max_duration_s) {
                return abort(TrialAbortReason::kTimeout, "trial wall-time exceeded max_duration_s");
            }
            break;
        }
        default:
            break;
    }
    return true;
}

bool TrialRunner::write_csv(const std::string& path) const {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.good()) return false;

    out << "t,x,y,yaw,s,v_raw,v_smooth,cmd_mode,cmd_value,erpm,duty,current_motor,v_in\n";
    out << std::setprecision(10);
    for (const auto& r : rows_) {
        out << r.t << ',' << r.x << ',' << r.y << ',' << r.yaw << ',' << r.s << ',' << r.v_raw << ','
            << r.v_smooth << ',' << to_string(r.cmd_mode) << ',' << r.cmd_value << ',';
        if (r.has_telemetry) {
            out << r.erpm << ',' << r.duty << ',' << r.current_motor << ',' << r.v_in;
        } else {
            out << ",,,";
        }
        out << '\n';
    }
    return true;
}

// =======================================================================
// Fitters
// =======================================================================

namespace {
// "small" local |dv/dt| threshold used by steady_state_v() to prefer
// already-converged samples within its trailing window -- see that
// function's doc comment. Chosen comfortably above sensor/finite-diff
// noise but well below a still-transient first-order response's typical
// slope near a tau~0.2s time constant.
constexpr double kSmallDvDtThresh = 0.15;  // m/s^2
}  // namespace

SteadyStatePoint steady_state_v(const std::vector<TrialRow>& rows, double steady_window_frac) {
    SteadyStatePoint out;
    if (rows.empty()) return out;

    double t_min = rows.front().t;
    double t_max = rows.front().t;
    for (const auto& r : rows) {
        t_min = std::min(t_min, r.t);
        t_max = std::max(t_max, r.t);
    }
    const double window_start = t_max - steady_window_frac * (t_max - t_min);

    std::vector<const TrialRow*> window;
    for (const auto& r : rows) {
        if (r.t >= window_start) window.push_back(&r);
    }
    if (window.empty()) {
        window.push_back(&rows.back());
    }
    std::sort(window.begin(), window.end(),
              [](const TrialRow* a, const TrialRow* b) { return a->t < b->t; });

    std::vector<const TrialRow*> filtered;
    for (std::size_t i = 0; i < window.size(); ++i) {
        double dv_dt = 0.0;
        bool have_dv_dt = false;
        if (i + 1 < window.size()) {
            const double dt = window[i + 1]->t - window[i]->t;
            if (dt > 1e-6) {
                dv_dt = (window[i + 1]->v_smooth - window[i]->v_smooth) / dt;
                have_dv_dt = true;
            }
        } else if (i > 0) {
            const double dt = window[i]->t - window[i - 1]->t;
            if (dt > 1e-6) {
                dv_dt = (window[i]->v_smooth - window[i - 1]->v_smooth) / dt;
                have_dv_dt = true;
            }
        }
        if (!have_dv_dt || std::fabs(dv_dt) <= kSmallDvDtThresh) {
            filtered.push_back(window[i]);
        }
    }
    if (filtered.empty()) filtered = window;

    double sum_v = 0.0;
    double sum_cmd = 0.0;
    for (const auto* r : filtered) {
        sum_v += r->v_smooth;
        sum_cmd += r->cmd_value;
    }
    out.v_ss = sum_v / static_cast<double>(filtered.size());
    out.cmd = sum_cmd / static_cast<double>(filtered.size());
    out.ok = true;
    return out;
}

StallFit fit_stall(const std::vector<SteadyStatePoint>& points, double min_moving) {
    StallFit out;
    out.min_moving_speed_mps = min_moving;

    std::vector<SteadyStatePoint> valid;
    double max_abs_cmd = 0.0;
    for (const auto& p : points) {
        if (!p.ok) continue;
        valid.push_back(p);
        max_abs_cmd = std::max(max_abs_cmd, std::fabs(p.cmd));
    }
    std::sort(valid.begin(), valid.end(), [](const SteadyStatePoint& a, const SteadyStatePoint& b) {
        return std::fabs(a.cmd) < std::fabs(b.cmd);
    });

    bool found = false;
    double best_min_cmd = 0.0;
    for (const auto& p : valid) {
        if (std::fabs(p.v_ss) > min_moving) {
            best_min_cmd = std::fabs(p.cmd);
            found = true;
            break;
        }
    }
    if (!found) {
        out.ok = false;
        return out;
    }

    out.min_cmd = best_min_cmd;
    out.kick_cmd = std::min(1.5 * best_min_cmd, max_abs_cmd);
    out.kick_ms = 150.0;
    out.ok = true;
    return out;
}

LinearFit fit_linear(const std::vector<SteadyStatePoint>& points, double stall_min_cmd) {
    LinearFit out;
    std::vector<double> xs;  // v
    std::vector<double> ys;  // cmd
    for (const auto& p : points) {
        if (!p.ok) continue;
        if (std::fabs(p.cmd) <= stall_min_cmd) continue;
        xs.push_back(p.v_ss);
        ys.push_back(p.cmd);
    }
    const std::size_t n = xs.size();
    if (n < 2) {
        out.ok = false;
        return out;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        sum_x += xs[i];
        sum_y += ys[i];
        sum_xx += xs[i] * xs[i];
        sum_xy += xs[i] * ys[i];
    }
    const double nd = static_cast<double>(n);
    const double denom = nd * sum_xx - sum_x * sum_x;
    if (std::fabs(denom) < 1e-12) {
        out.ok = false;
        return out;
    }

    out.cmd_per_mps = (nd * sum_xy - sum_x * sum_y) / denom;
    out.cmd_offset = (sum_y - out.cmd_per_mps * sum_x) / nd;

    double sse = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double pred = out.cmd_offset + out.cmd_per_mps * xs[i];
        const double err = pred - ys[i];
        sse += err * err;
    }
    out.fit_rms = std::sqrt(sse / nd);
    out.ok = true;
    return out;
}

const std::vector<double>& default_grid_v_mps() {
    static const std::vector<double> grid = [] {
        std::vector<double> g;
        for (int i = 0; i <= 10; ++i) g.push_back(i * 0.05);
        return g;
    }();
    return grid;
}

const std::vector<double>& default_grid_a_mps2() {
    static const std::vector<double> grid = [] {
        std::vector<double> g;
        for (int i = 0; i <= 10; ++i) g.push_back(-1.0 + i * 0.2);
        return g;
    }();
    return grid;
}

namespace {
std::size_t nearest_grid_index(double value, const std::vector<double>& grid) {
    std::size_t best = 0;
    double best_d = std::fabs(grid[0] - value);
    for (std::size_t i = 1; i < grid.size(); ++i) {
        const double d = std::fabs(grid[i] - value);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}
}  // namespace

GridFit fit_grid(const std::vector<std::vector<TrialRow>>& trials, const std::vector<double>& v_grid,
                  const std::vector<double>& a_grid) {
    GridFit out;
    out.v_mps = v_grid;
    out.a_mps2 = a_grid;
    const std::size_t nv = v_grid.size();
    const std::size_t na = a_grid.size();
    out.cmd.assign(nv, std::vector<double>(na, 0.0));
    out.supported.assign(nv, std::vector<bool>(na, false));
    if (nv == 0 || na == 0) return out;

    // Per-v-bin (cmd, a) samples, accel estimated from each trial's OWN
    // consecutive v_smooth samples (never differentiated across a trial
    // boundary -- see this function's doc comment).
    std::vector<std::vector<std::pair<double, double>>> bin_samples(nv);
    for (const auto& trial : trials) {
        for (std::size_t i = 0; i + 1 < trial.size(); ++i) {
            const double dt = trial[i + 1].t - trial[i].t;
            if (dt <= 1e-6) continue;
            const double a_est = (trial[i + 1].v_smooth - trial[i].v_smooth) / dt;
            const double v_mid = 0.5 * (trial[i].v_smooth + trial[i + 1].v_smooth);
            const double cmd = trial[i].cmd_value;  // constant within a trial
            // v_grid (default_grid_v_mps()) is a non-negative speed-MAGNITUDE
            // axis (0.00..0.50), matching the driver's own bilinear_lookup,
            // which clamps any negative v_now to that same v=0.00 edge (see
            // DriverCore.cpp's bracket_axis()). Binning by the RAW SIGNED
            // v_mid would send every sample of every negative-direction
            // (reverse) trial into the v=0.00 bin regardless of how far from
            // stall it actually was -- e.g. a reverse trial's own
            // steady-CRUISE samples (v_mid around -0.4, a~=0) would land in
            // the v=0.00 bin right alongside genuine near-stall/launch
            // samples, corrupting that bin's fitted a-vs-cmd line (the
            // steady-cruise cmd-to-overcome-drag point looks nothing like
            // the near-stall cmd-to-start-moving relationship the v=0.00 row
            // is actually meant to capture). Binning by |v_mid| instead
            // keeps a reverse trial's samples in the SAME magnitude-matched
            // row a forward trial's samples at that speed would use,
            // consistent with the a_mps2 axis already spanning both signs at
            // every v row (so the v=0.00 row still legitimately mixes small
            // positive- and negative-cmd/accel launch samples from BOTH
            // directions, which is exactly what a bidirectional near-stall
            // fit should do).
            const std::size_t vb = nearest_grid_index(std::fabs(v_mid), v_grid);
            bin_samples[vb].emplace_back(cmd, a_est);
        }
    }

    // Per-v-bin linear fit of a vs cmd, inverted to cmd(a) -- but ONLY
    // marked "supported" for a_grid columns within (a small margin beyond)
    // this bin's own OBSERVED accel range. A per-bin fit is a linear
    // MODEL, and evaluating/inverting it far past the accel range that bin
    // actually saw is an EXTRAPOLATION, not the "per-v-bin linear fit"
    // interpolation the task describes -- since real samples land at
    // slightly different true v within one nominal bin (the v-bin
    // approximation's own error), a compromise line fit through them can
    // be wildly wrong once evaluated outside the range it was actually
    // constrained by. a_grid columns outside the observed range instead
    // fall through to the neighbor interpolation/extrapolation pass below
    // (which reasons across v-bins, not by extrapolating one bin's own
    // possibly-unrepresentative line).
    for (std::size_t i = 0; i < nv; ++i) {
        const auto& samples = bin_samples[i];
        if (samples.size() < 2) continue;
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        double min_a = samples.front().second, max_a = samples.front().second;
        for (const auto& s : samples) {
            sum_x += s.first;
            sum_y += s.second;
            sum_xx += s.first * s.first;
            sum_xy += s.first * s.second;
            min_a = std::min(min_a, s.second);
            max_a = std::max(max_a, s.second);
        }
        const double n = static_cast<double>(samples.size());
        const double denom = n * sum_xx - sum_x * sum_x;
        if (std::fabs(denom) < 1e-9) continue;  // no cmd variation -- can't fit a line
        const double slope = (n * sum_xy - sum_x * sum_y) / denom;  // a = slope*cmd + intercept
        const double intercept = (sum_y - slope * sum_x) / n;
        if (std::fabs(slope) < 1e-9) continue;  // degenerate -- can't invert to cmd(a)

        const double margin = std::max(1e-6, 0.1 * (max_a - min_a));
        const double a_lo = min_a - margin, a_hi = max_a + margin;
        for (std::size_t j = 0; j < na; ++j) {
            if (a_grid[j] < a_lo || a_grid[j] > a_hi) continue;
            out.cmd[i][j] = (a_grid[j] - intercept) / slope;
            out.supported[i][j] = true;
        }
    }

    bool any_supported = false;
    for (std::size_t i = 0; i < nv; ++i) {
        for (std::size_t j = 0; j < na; ++j) {
            if (out.supported[i][j]) any_supported = true;
        }
    }

    if (any_supported) {
        // Column-wise fill: unsupported v-bins interpolate/extrapolate from
        // supported neighbors in the SAME a-column.
        for (std::size_t j = 0; j < na; ++j) {
            std::vector<std::size_t> supported_rows;
            for (std::size_t i = 0; i < nv; ++i) {
                if (out.supported[i][j]) supported_rows.push_back(i);
            }
            if (supported_rows.empty()) continue;  // handled by the global fallback pass below

            for (std::size_t i = 0; i < nv; ++i) {
                if (out.supported[i][j]) continue;
                std::size_t lo = supported_rows.size();  // sentinel: none found
                std::size_t hi = supported_rows.size();
                for (std::size_t k = 0; k < supported_rows.size(); ++k) {
                    if (supported_rows[k] <= i) lo = k;
                    if (supported_rows[k] >= i && hi == supported_rows.size()) hi = k;
                }
                if (lo < supported_rows.size() && hi < supported_rows.size() && lo != hi) {
                    const std::size_t ri = supported_rows[lo];
                    const std::size_t rj = supported_rows[hi];
                    const double v0 = v_grid[ri];
                    const double v1 = v_grid[rj];
                    const double frac = (v1 > v0) ? (v_grid[i] - v0) / (v1 - v0) : 0.0;
                    out.cmd[i][j] = out.cmd[ri][j] + frac * (out.cmd[rj][j] - out.cmd[ri][j]);
                } else if (lo < supported_rows.size()) {
                    out.cmd[i][j] = out.cmd[supported_rows[lo]][j];
                } else if (hi < supported_rows.size()) {
                    out.cmd[i][j] = out.cmd[supported_rows[hi]][j];
                }
            }
        }

        // Global fallback for any column with NO supported row at all
        // (rare -- only reachable with very sparse input): nearest
        // supported cell anywhere in the grid.
        for (std::size_t i = 0; i < nv; ++i) {
            for (std::size_t j = 0; j < na; ++j) {
                if (out.supported[i][j]) continue;
                bool col_has_support = false;
                for (std::size_t k = 0; k < nv; ++k) {
                    if (out.supported[k][j]) col_has_support = true;
                }
                if (col_has_support) continue;  // already filled above

                double best_d = -1.0;
                double best_val = 0.0;
                for (std::size_t ii = 0; ii < nv; ++ii) {
                    for (std::size_t jj = 0; jj < na; ++jj) {
                        if (!out.supported[ii][jj]) continue;
                        const double dv = v_grid[ii] - v_grid[i];
                        const double da = a_grid[jj] - a_grid[j];
                        const double d = dv * dv + da * da;
                        if (best_d < 0.0 || d < best_d) {
                            best_d = d;
                            best_val = out.cmd[ii][jj];
                        }
                    }
                }
                if (best_d >= 0.0) out.cmd[i][j] = best_val;
            }
        }
    }

    out.ok = any_supported;
    return out;
}

// =======================================================================
// Export / import
// =======================================================================

nlohmann::json export_calibration(const std::string& robot_name, RawMode mode, double erpm_per_mps,
                                   const LinearFit& linear, const StallFit& stall,
                                   const std::vector<SteadyStatePoint>& v_ss_points,
                                   const GridFit* grid, int n_trials, const std::string& notes) {
    if (mode == RawMode::kCurrent) {
        throw std::invalid_argument(
            "mpc::export_calibration: mode must be duty or erpm (the frozen schema's 'mode' field "
            "is never 'current')");
    }

    nlohmann::json j;
    j["version"] = 1;
    j["mode"] = to_string(mode);
    j["robot_name"] = robot_name;
    j["erpm_per_mps"] = erpm_per_mps;
    j["cmd_per_mps"] = linear.cmd_per_mps;
    j["cmd_offset"] = linear.cmd_offset;
    j["stall"] = {
        {"min_cmd", stall.min_cmd},
        {"kick_cmd", stall.kick_cmd},
        {"kick_ms", stall.kick_ms},
        {"min_moving_speed_mps", stall.min_moving_speed_mps},
    };

    std::vector<SteadyStatePoint> sorted_pts;
    for (const auto& p : v_ss_points) {
        if (p.ok) sorted_pts.push_back(p);
    }
    std::sort(sorted_pts.begin(), sorted_pts.end(),
              [](const SteadyStatePoint& a, const SteadyStatePoint& b) { return a.cmd < b.cmd; });
    nlohmann::json v_ss_json = nlohmann::json::array();
    for (const auto& p : sorted_pts) {
        v_ss_json.push_back({{"cmd", p.cmd}, {"v", p.v_ss}});
    }
    j["v_ss"] = v_ss_json;

    if (grid != nullptr && grid->ok) {
        j["grid"] = {
            {"v_mps", grid->v_mps},
            {"a_mps2", grid->a_mps2},
            {"cmd", grid->cmd},
        };
    } else {
        j["grid"] = nullptr;
    }

    j["meta"] = {
        {"n_trials", n_trials},
        {"fit_rms", linear.fit_rms},
        {"notes", notes},
    };

    return j;
}

ParsedCalibration parse_calibration(const nlohmann::json& j) {
    ParsedCalibration out;

    auto require = [&](const char* key) -> const nlohmann::json& {
        if (!j.contains(key)) {
            throw std::runtime_error(std::string("mpc::parse_calibration: missing required key '") +
                                      key + "'");
        }
        return j.at(key);
    };

    out.version = require("version").get<int>();
    out.mode = require("mode").get<std::string>();
    if (out.mode != "erpm" && out.mode != "duty") {
        throw std::runtime_error("mpc::parse_calibration: 'mode' must be 'erpm' or 'duty', got '" +
                                  out.mode + "'");
    }
    out.robot_name = require("robot_name").get<std::string>();
    out.erpm_per_mps = require("erpm_per_mps").get<double>();
    out.cmd_per_mps = require("cmd_per_mps").get<double>();
    out.cmd_offset = require("cmd_offset").get<double>();

    const auto& st = require("stall");
    if (!st.is_object()) {
        throw std::runtime_error("mpc::parse_calibration: 'stall' must be an object");
    }
    out.stall_min_cmd = st.at("min_cmd").get<double>();
    out.stall_kick_cmd = st.at("kick_cmd").get<double>();
    out.stall_kick_ms = st.at("kick_ms").get<double>();
    out.stall_min_moving_speed_mps = st.at("min_moving_speed_mps").get<double>();

    if (j.contains("v_ss") && j.at("v_ss").is_array()) {
        for (const auto& e : j.at("v_ss")) {
            SteadyStatePoint p;
            p.cmd = e.at("cmd").get<double>();
            p.v_ss = e.at("v").get<double>();
            p.ok = true;
            out.v_ss.push_back(p);
        }
    }

    if (j.contains("grid") && !j.at("grid").is_null()) {
        const auto& g = j.at("grid");
        out.has_grid = true;
        out.grid.v_mps = g.at("v_mps").get<std::vector<double>>();
        out.grid.a_mps2 = g.at("a_mps2").get<std::vector<double>>();
        out.grid.cmd = g.at("cmd").get<std::vector<std::vector<double>>>();
        out.grid.ok = true;
    }

    const auto& meta = require("meta");
    if (!meta.is_object()) {
        throw std::runtime_error("mpc::parse_calibration: 'meta' must be an object");
    }
    out.n_trials = meta.at("n_trials").get<int>();
    out.fit_rms = meta.at("fit_rms").get<double>();
    out.notes = meta.contains("notes") ? meta.at("notes").get<std::string>() : std::string();

    return out;
}

// =======================================================================
// Config
// =======================================================================

MotorCalibConfig MotorCalibConfig::defaults() { return MotorCalibConfig{}; }

namespace {
template <typename T>
void assign_if_present(const nlohmann::json& obj, const char* key, T& out) {
    if (obj.contains(key)) {
        out = obj.at(key).get<T>();
    }
}

void assign_optional_double_if_present(const nlohmann::json& obj, const char* key,
                                        std::optional<double>& out) {
    if (!obj.contains(key)) return;
    if (obj.at(key).is_null()) {
        out = std::nullopt;
    } else {
        out = obj.at(key).get<double>();
    }
}

const std::set<std::string>& motor_calib_top_keys() {
    static const std::set<std::string> keys = {
        "_doc",     "robot_ip",         "control_port",     "telemetry_port", "localization_endpoint",
        "robot_topic_name", "motive_body_name", "line",      "safety",         "sweeps",
        "trial",    "output_dir",
    };
    return keys;
}
}  // namespace

MotorCalibConfig load_motor_calib_config(const std::string& path) {
    MotorCalibConfig cfg;

    std::ifstream in(path);
    if (!in.good()) {
        throw std::runtime_error("mpc::load_motor_calib_config: cannot open file: " + path);
    }

    nlohmann::json root;
    try {
        in >> root;
    } catch (const std::exception& ex) {
        throw std::runtime_error("mpc::load_motor_calib_config: invalid JSON in '" + path +
                                  "': " + ex.what());
    }
    if (!root.is_object()) {
        throw std::runtime_error("mpc::load_motor_calib_config: top level of '" + path +
                                  "' must be a JSON object");
    }

    for (auto it = root.begin(); it != root.end(); ++it) {
        if (motor_calib_top_keys().find(it.key()) == motor_calib_top_keys().end()) {
            std::cerr << "[mpc::MotorCalibConfig] warning: unknown top-level key '" << it.key()
                      << "' in " << path << " (ignored)" << std::endl;
        }
    }

    assign_if_present(root, "robot_ip", cfg.robot_ip);
    assign_if_present(root, "control_port", cfg.control_port);
    assign_if_present(root, "telemetry_port", cfg.telemetry_port);
    assign_if_present(root, "localization_endpoint", cfg.localization_endpoint);
    assign_if_present(root, "robot_topic_name", cfg.robot_topic_name);
    assign_if_present(root, "motive_body_name", cfg.motive_body_name);

    if (root.contains("line")) {
        const auto& l = root.at("line");
        if (!l.is_object()) {
            throw std::runtime_error("mpc::load_motor_calib_config: 'line' must be an object in " +
                                      path);
        }
        assign_if_present(l, "length_m", cfg.line.length_m);
        assign_if_present(l, "end_margin_m", cfg.line.end_margin_m);
        assign_optional_double_if_present(l, "line_yaw_rad", cfg.line.line_yaw_rad);
    }
    if (root.contains("safety")) {
        const auto& s = root.at("safety");
        if (!s.is_object()) {
            throw std::runtime_error("mpc::load_motor_calib_config: 'safety' must be an object in " +
                                      path);
        }
        assign_if_present(s, "mocap_stale_abort_ms", cfg.safety.mocap_stale_abort_ms);
        assign_if_present(s, "max_speed_abort_mps", cfg.safety.max_speed_abort_mps);
    }
    if (root.contains("sweeps")) {
        const auto& sw = root.at("sweeps");
        if (!sw.is_object()) {
            throw std::runtime_error("mpc::load_motor_calib_config: 'sweeps' must be an object in " +
                                      path);
        }
        assign_if_present(sw, "duty", cfg.sweeps.duty);
        assign_if_present(sw, "erpm", cfg.sweeps.erpm);
    }
    if (root.contains("trial")) {
        const auto& t = root.at("trial");
        if (!t.is_object()) {
            throw std::runtime_error("mpc::load_motor_calib_config: 'trial' must be an object in " +
                                      path);
        }
        assign_if_present(t, "max_duration_s", cfg.trial.max_duration_s);
        assign_if_present(t, "settle_s", cfg.trial.settle_s);
        assign_if_present(t, "arm_settle_s", cfg.trial.arm_settle_s);
        assign_if_present(t, "steady_window_frac", cfg.trial.steady_window_frac);
    }
    assign_if_present(root, "output_dir", cfg.output_dir);

    return cfg;
}

TrialRunnerConfig make_trial_runner_config(const MotorCalibConfig& cfg) {
    TrialRunnerConfig out;
    out.line_length_m = cfg.line.length_m;
    out.end_margin_m = cfg.line.end_margin_m;
    out.mocap_stale_abort_ms = cfg.safety.mocap_stale_abort_ms;
    out.max_speed_abort_mps = cfg.safety.max_speed_abort_mps;
    out.max_duration_s = cfg.trial.max_duration_s;
    out.settle_s = cfg.trial.settle_s;
    out.arm_settle_s = cfg.trial.arm_settle_s;
    return out;
}

}  // namespace mpc
