#include "mpc/PoseFilter.h"

#include <cmath>

namespace mpc {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Wraps `angle` into (-pi, pi]. Deliberately a tiny local copy rather than a
// dependency on mpc::normalize_angle (OptiTrackCore.h) -- keeps PoseFilter a
// self-contained, dependency-light module (only Eigen), mirroring this
// codebase's precedent for small pure-math helpers (e.g. SustainSweepCore.h)
// not reaching across modules for a one-line function.
double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle <= -kPi) angle += 2.0 * kPi;
    return angle;
}

// dt values below this are treated as a duplicate/burst-arrival frame (same
// mocap sample delivered twice in immediate succession) rather than a real
// time step -- see PoseFilter::step()'s doc comment in the header and
// doc/MOCAP_POSE_FILTER_PLAN.md's "Implementation status" section for the
// live-data evidence (bridge --log-csv occasionally shows dt ~ 1e-5 s
// bursts) that made this necessary: naively dividing a genuine (even tiny)
// position delta by such a dt implies an absurd speed and would otherwise
// spuriously fail the plausibility gate on perfectly good data.
constexpr double kMinMeaningfulDt = 1e-4;

}  // namespace

PoseFilter::PoseFilter(PoseFilterConfig config) : config_(config) {}

void PoseFilter::reset() {
    initialized_ = false;
    state_.setZero();
    P_.setIdentity();
    last_time_ = 0.0;
    last_accepted_ = Sample{};
    last_accepted_time_ = 0.0;
    consecutive_rejects_ = 0;
    rejected_buffer_.clear();
    passthrough_has_last_ = false;
    passthrough_last_ = Sample{};
    last_output_ = PoseFilterOutput{};
    // reject_count_/reinit_count_ are cumulative lifetime counters -- NOT
    // cleared by reset() (see the header's doc comment on rejects()/
    // reinits()).
}

// ---------------------------------------------------------------------
// Predict / gate math (shared by step() and the const would_accept()).
// ---------------------------------------------------------------------

PoseFilter::Predicted PoseFilter::predict(const Eigen::Matrix<double, 6, 1>& state,
                                            const Eigen::Matrix<double, 6, 6>& P, double dt) const {
    Predicted out;
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 2) = dt;  // x += vx*dt
    F(1, 3) = dt;  // y += vy*dt
    F(4, 5) = dt;  // yaw += w*dt

    out.state = F * state;
    out.state(4) = wrap_angle(out.state(4));

    // Discrete white-noise-acceleration process noise (Bar-Shalom & Fortmann
    // ch. 6): for a 1D constant-velocity pair [p, v] driven by a piecewise-
    // constant acceleration with std `q`, Q_2x2 = q^2 * G * G^T with
    // G = [dt^2/2; dt]. Applied independently to the x/vx and y/vy blocks
    // (q_acc) and to the yaw/w block (q_yaw_acc) -- no cross-coupling
    // between x, y, and yaw.
    auto q_block = [](double dt_, double q) -> Eigen::Matrix2d {
        Eigen::Vector2d G(0.5 * dt_ * dt_, dt_);
        return (q * q) * (G * G.transpose());
    };

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    const Eigen::Matrix2d qx = q_block(dt, config_.q_acc);
    const Eigen::Matrix2d qy = q_block(dt, config_.q_acc);
    const Eigen::Matrix2d qyaw = q_block(dt, config_.q_yaw_acc);
    // x, vx are state indices 0, 2.
    Q(0, 0) = qx(0, 0);
    Q(0, 2) = qx(0, 1);
    Q(2, 0) = qx(1, 0);
    Q(2, 2) = qx(1, 1);
    // y, vy are state indices 1, 3.
    Q(1, 1) = qy(0, 0);
    Q(1, 3) = qy(0, 1);
    Q(3, 1) = qy(1, 0);
    Q(3, 3) = qy(1, 1);
    // yaw, w are state indices 4, 5.
    Q(4, 4) = qyaw(0, 0);
    Q(4, 5) = qyaw(0, 1);
    Q(5, 4) = qyaw(1, 0);
    Q(5, 5) = qyaw(1, 1);

    out.P = F * P * F.transpose() + Q;
    out.P = 0.5 * (out.P + out.P.transpose());  // keep symmetric under fp round-off.
    return out;
}

PoseFilter::Gate PoseFilter::compute_gate(const Eigen::Matrix<double, 6, 1>& state_pred,
                                            const Eigen::Matrix<double, 6, 6>& P_pred, double zx,
                                            double zy, double zyaw) const {
    Gate g;
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;  // x
    H(1, 1) = 1.0;  // y
    H(2, 4) = 1.0;  // yaw

    g.nu(0) = zx - state_pred(0);
    g.nu(1) = zy - state_pred(1);
    g.nu(2) = wrap_angle(zyaw - state_pred(4));  // shortest-arc yaw innovation.

    Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
    R(0, 0) = config_.r_pos * config_.r_pos;
    R(1, 1) = config_.r_pos * config_.r_pos;
    R(2, 2) = config_.r_yaw * config_.r_yaw;

    g.S = H * P_pred * H.transpose() + R;

    const Eigen::Matrix2d S_pos = g.S.topLeftCorner<2, 2>();
    const Eigen::Vector2d nu_pos = g.nu.head<2>();
    g.d2_pos = nu_pos.transpose() * S_pos.inverse() * nu_pos;

    const double s_yaw = g.S(2, 2);
    g.d2_yaw = (s_yaw > 0.0) ? (g.nu(2) * g.nu(2)) / s_yaw : 0.0;

    g.d2_all = g.nu.transpose() * g.S.inverse() * g.nu;

    return g;
}

bool PoseFilter::passes_gate(const Gate& g) const {
    return g.d2_pos <= config_.gate_chi2_pos && g.d2_yaw <= config_.gate_chi2_yaw &&
           g.d2_all <= config_.gate_chi2_all;
}

namespace {
// Several-sigma measurement-noise margin added to the raw max_speed*dt /
// max_yaw_rate*dt plausibility ceiling -- see motion_plausible()'s doc
// comment in the header for why this is needed (real bursty-arrival data
// otherwise trips the "hard" rule on pure sensor jitter at small dt).
constexpr double kPlausibilityNoiseSigma = 4.0;
}  // namespace

bool PoseFilter::motion_plausible(double dx, double dy, double dyaw, double dt) const {
    if (dt < kMinMeaningfulDt) return true;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double yaw_delta = std::fabs(wrap_angle(dyaw));
    const double allowed_dist = config_.max_speed * dt + kPlausibilityNoiseSigma * config_.r_pos;
    const double allowed_yaw = config_.max_yaw_rate * dt + kPlausibilityNoiseSigma * config_.r_yaw;
    return dist <= allowed_dist && yaw_delta <= allowed_yaw;
}

bool PoseFilter::plausible_vs_last_accepted(double t, double zx, double zy, double zyaw) const {
    const double dt = t - last_accepted_time_;
    const double dx = zx - last_accepted_.x;
    const double dy = zy - last_accepted_.y;
    const double dyaw = zyaw - last_accepted_.yaw;
    return motion_plausible(dx, dy, dyaw, dt);
}

bool PoseFilter::buffer_self_consistent(const std::deque<Sample>& buf) const {
    if (buf.size() < 2) return false;
    for (size_t i = 1; i < buf.size(); ++i) {
        const double dt = buf[i].t - buf[i - 1].t;
        const double dx = buf[i].x - buf[i - 1].x;
        const double dy = buf[i].y - buf[i - 1].y;
        const double dyaw = buf[i].yaw - buf[i - 1].yaw;
        if (!motion_plausible(dx, dy, dyaw, dt)) return false;
    }
    return true;
}

// ---------------------------------------------------------------------
// Init / re-init.
// ---------------------------------------------------------------------

namespace {
constexpr double kInitCovariance = 100.0;  // "large P" -- big enough that the very first real
                                              // update is dominated by the measurement, not the
                                              // (meaningless) initial guess.
}  // namespace

void PoseFilter::do_init(double t, double zx, double zy, double zyaw) {
    state_ << zx, zy, 0.0, 0.0, wrap_angle(zyaw), 0.0;
    P_.setIdentity();
    P_ *= kInitCovariance;
    initialized_ = true;
    last_time_ = t;
    last_accepted_ = Sample{t, zx, zy, wrap_angle(zyaw)};
    last_accepted_time_ = t;
    consecutive_rejects_ = 0;
    rejected_buffer_.clear();
}

void PoseFilter::do_reinit_from_gap(double t, double zx, double zy, double zyaw) {
    state_ << zx, zy, 0.0, 0.0, wrap_angle(zyaw), 0.0;
    P_.setIdentity();
    P_ *= kInitCovariance;
    last_time_ = t;
    last_accepted_ = Sample{t, zx, zy, wrap_angle(zyaw)};
    last_accepted_time_ = t;
    consecutive_rejects_ = 0;
    rejected_buffer_.clear();
    ++reinit_count_;
}

void PoseFilter::do_reinit_from_streak(double t, double zx, double zy, double zyaw) {
    // Velocity from the rejected buffer's last two samples (the buffer
    // already includes the current sample -- see step()'s call site).
    double vx = 0.0, vy = 0.0, w = 0.0;
    if (rejected_buffer_.size() >= 2) {
        const Sample& a = rejected_buffer_[rejected_buffer_.size() - 2];
        const Sample& b = rejected_buffer_.back();
        const double dt = b.t - a.t;
        if (dt >= kMinMeaningfulDt) {
            vx = (b.x - a.x) / dt;
            vy = (b.y - a.y) / dt;
            w = wrap_angle(b.yaw - a.yaw) / dt;
        }
    }
    state_ << zx, zy, vx, vy, wrap_angle(zyaw), w;
    P_.setIdentity();
    P_ *= kInitCovariance;
    last_time_ = t;
    last_accepted_ = Sample{t, zx, zy, wrap_angle(zyaw)};
    last_accepted_time_ = t;
    consecutive_rejects_ = 0;
    rejected_buffer_.clear();
    ++reinit_count_;
}

// ---------------------------------------------------------------------
// step().
// ---------------------------------------------------------------------

PoseFilterOutput PoseFilter::step(double t, double zx, double zy, double zyaw) {
    if (!config_.enabled) {
        PoseFilterOutput out;
        out.x = zx;
        out.y = zy;
        out.yaw = wrap_angle(zyaw);
        out.accepted = true;
        out.reinit = !passthrough_has_last_;
        if (passthrough_has_last_) {
            const double dt = t - passthrough_last_.t;
            if (dt >= kMinMeaningfulDt) {
                out.vx = (zx - passthrough_last_.x) / dt;
                out.vy = (zy - passthrough_last_.y) / dt;
                out.w = wrap_angle(zyaw - passthrough_last_.yaw) / dt;
            } else {
                // Duplicate/non-increasing sample: hold the previous velocity
                // estimate rather than dividing by ~0.
                out.vx = last_output_.vx;
                out.vy = last_output_.vy;
                out.w = last_output_.w;
            }
        }
        passthrough_last_ = Sample{t, zx, zy, out.yaw};
        passthrough_has_last_ = true;
        last_output_ = out;
        return out;
    }

    // Non-increasing timestamp: ignore outright (per doc/
    // MOCAP_POSE_FILTER_PLAN.md and the unit tests -- ignored samples never
    // touch state, buffers, or counters).
    if (initialized_ && t <= last_time_) {
        PoseFilterOutput out = last_output_;
        out.accepted = false;
        out.reinit = false;
        return out;
    }

    if (!initialized_) {
        do_init(t, zx, zy, zyaw);
        PoseFilterOutput out;
        out.x = state_(0);
        out.y = state_(1);
        out.yaw = state_(4);
        out.vx = state_(2);
        out.vy = state_(3);
        out.w = state_(5);
        out.accepted = true;
        out.reinit = true;
        last_output_ = out;
        return out;
    }

    // Gap re-init: checked BEFORE gating -- a long enough silence means the
    // robot may legitimately have moved (or stopped) by an arbitrary amount,
    // so this sample is accepted unconditionally rather than gated.
    if (t - last_accepted_time_ >= config_.gap_reinit_s) {
        do_reinit_from_gap(t, zx, zy, zyaw);
        PoseFilterOutput out;
        out.x = state_(0);
        out.y = state_(1);
        out.yaw = state_(4);
        out.vx = state_(2);
        out.vy = state_(3);
        out.w = state_(5);
        out.accepted = true;
        out.reinit = true;
        last_output_ = out;
        return out;
    }

    const double dt = t - last_time_;
    const double dt_eff = (dt < kMinMeaningfulDt) ? 0.0 : dt;

    const Predicted pred = predict(state_, P_, dt_eff);
    const Gate gate = compute_gate(pred.state, pred.P, zx, zy, zyaw);

    const bool plausible = plausible_vs_last_accepted(t, zx, zy, zyaw);
    const bool reject = !passes_gate(gate) || !plausible;

    PoseFilterOutput out;
    out.d2_pos = gate.d2_pos;
    out.d2_yaw = gate.d2_yaw;
    out.d2_all = gate.d2_all;

    if (!reject) {
        // Standard Kalman update (Joseph form for numerical robustness --
        // this filter runs unattended for hours at 120 Hz).
        Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 4) = 1.0;
        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
        R(0, 0) = config_.r_pos * config_.r_pos;
        R(1, 1) = config_.r_pos * config_.r_pos;
        R(2, 2) = config_.r_yaw * config_.r_yaw;

        const Eigen::Matrix<double, 6, 3> K = pred.P * H.transpose() * gate.S.inverse();
        state_ = pred.state + K * gate.nu;
        state_(4) = wrap_angle(state_(4));

        const Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 6> IKH = I6 - K * H;
        P_ = IKH * pred.P * IKH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        last_time_ = t;
        last_accepted_ = Sample{t, zx, zy, wrap_angle(zyaw)};
        last_accepted_time_ = t;
        consecutive_rejects_ = 0;
        rejected_buffer_.clear();

        out.x = state_(0);
        out.y = state_(1);
        out.yaw = state_(4);
        out.vx = state_(2);
        out.vy = state_(3);
        out.w = state_(5);
        out.accepted = true;
        out.reinit = false;
        out.consecutive_rejects = 0;
        last_output_ = out;
        return out;
    }

    // Reject: commit the PREDICTION only (coast) -- no measurement update.
    state_ = pred.state;
    P_ = pred.P;
    last_time_ = t;
    ++consecutive_rejects_;
    ++reject_count_;
    rejected_buffer_.push_back(Sample{t, zx, zy, wrap_angle(zyaw)});

    const bool streak_long_enough = (t - rejected_buffer_.front().t) >= config_.reinit_after_s;
    const bool self_consistent = buffer_self_consistent(rejected_buffer_);

    if (streak_long_enough && self_consistent) {
        do_reinit_from_streak(t, zx, zy, zyaw);
        out.x = state_(0);
        out.y = state_(1);
        out.yaw = state_(4);
        out.vx = state_(2);
        out.vy = state_(3);
        out.w = state_(5);
        out.accepted = true;
        out.reinit = true;
        out.consecutive_rejects = 0;
        last_output_ = out;
        return out;
    }

    out.x = state_(0);
    out.y = state_(1);
    out.yaw = state_(4);
    out.vx = state_(2);
    out.vy = state_(3);
    out.w = state_(5);
    out.accepted = false;
    out.reinit = false;
    out.consecutive_rejects = consecutive_rejects_;
    last_output_ = out;
    return out;
}

bool PoseFilter::would_accept(double t, double zx, double zy, double zyaw) const {
    if (!config_.enabled) return true;
    if (!initialized_) return true;   // no track yet -- nothing to reject against.
    if (t <= last_time_) return false;

    const double dt = t - last_time_;
    const double dt_eff = (dt < kMinMeaningfulDt) ? 0.0 : dt;
    const Predicted pred = predict(state_, P_, dt_eff);
    const Gate gate = compute_gate(pred.state, pred.P, zx, zy, zyaw);
    if (!passes_gate(gate)) return false;

    const double dx = zx - last_accepted_.x;
    const double dy = zy - last_accepted_.y;
    const double dyaw = zyaw - last_accepted_.yaw;
    return motion_plausible(dx, dy, dyaw, t - last_accepted_time_);
}

}  // namespace mpc
