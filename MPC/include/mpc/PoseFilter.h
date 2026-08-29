#pragma once

#include <cstdint>
#include <deque>

#include <Eigen/Dense>

// mpc::PoseFilter: a per-rigid-body gated constant-velocity Kalman filter
// that rejects single-(or few-)frame mocap pose jumps before they reach any
// consumer of optitrack_zmq_bridge's published localization. Pure logic, no
// sockets/ZMQ/JSON here -- see doc/MOCAP_POSE_FILTER_PLAN.md for the full
// design rationale (Bar-Shalom & Fortmann validation-gate tracking, ch. 2/3)
// and MPC/src/optitrack_zmq_bridge.cpp for the thin per-body integration
// (one PoseFilter instance per published body, stepped at the full mocap
// frame rate -- 120 Hz on this rig -- BEFORE the 30 Hz publish decimation).
//
// State: [x, y, vx, vy, yaw, w] (constant-velocity model). Measurement:
// [x, y, yaw] straight from Motive (already through the up-axis/world
// transform by the time the bridge calls step() -- this class never sees
// raw NatNet fields). Fixed-size 6x6/6x1/3x3/3x1 Eigen matrices throughout
// (stack-allocated -- step() never heap-allocates).
namespace mpc {

struct PoseFilterConfig {
    bool enabled = false;
    double gate_chi2_pos = 9.21;   // chi^2, 2 dof, 99% -- position-only gate.
    double gate_chi2_yaw = 6.63;   // chi^2, 1 dof, 99% -- yaw-only gate (catches a yaw-only flip
                                     // even when position is perfect -- common Motive symptom for
                                     // symmetric marker sets).
    double gate_chi2_all = 12.8;   // chi^2, 3 dof, 99.5% -- full joint gate.
    double max_speed = 1.0;        // m/s -- hard physical-plausibility ceiling (belt and braces
                                     // on top of the statistical gate above).
    double max_yaw_rate = 4.0;     // rad/s.
    double reinit_after_s = 0.25;  // consecutive-reject duration before a self-consistent streak
                                     // triggers a snap-to-measurement re-init.
    double gap_reinit_s = 0.5;     // any inter-sample gap this long (e.g. a mocap outage) forces
                                     // an immediate, unconditional re-init on the next sample --
                                     // the robot legitimately may have moved/stopped during it.
    double r_pos = 0.005;          // measurement std, m (position x and y, independently).
    double r_yaw = 0.01745;        // measurement std, rad (~1 degree).
    double q_acc = 3.0;            // process accel std, m/s^2 (drives x/y process noise).
    double q_yaw_acc = 15.0;       // process yaw-accel std, rad/s^2 (drives yaw process noise).
};

struct PoseFilterOutput {
    double x = 0.0, y = 0.0, yaw = 0.0;   // published pose: posterior on accept, coasted
                                            // prediction on reject, verbatim measurement on
                                            // enabled=false pass-through.
    double vx = 0.0, vy = 0.0, w = 0.0;   // filter's own velocity estimate (or a finite
                                            // difference, when enabled=false).
    bool accepted = true;   // false iff this sample was rejected by the gate (output is a
                              // coasted prediction, not a measurement update) OR the sample was
                              // ignored outright (non-increasing timestamp).
    bool reinit = false;    // true iff the state was just snapped to (at/near) this measurement
                              // -- either the very first sample ever, a persistent-reject
                              // re-init, or a post-gap re-init.
    double d2_pos = 0.0, d2_yaw = 0.0, d2_all = 0.0;  // Mahalanobis distances^2 used for gating
                              // this sample (0.0 when not evaluated, e.g. init/gap-reinit/
                              // ignored/disabled).
    int consecutive_rejects = 0;  // current reject-streak length (0 right after any acceptance,
                                    // including a re-init).
};

class PoseFilter {
   public:
    explicit PoseFilter(PoseFilterConfig config = PoseFilterConfig());

    // Feeds one measurement at time t (any monotonic clock, seconds; must be
    // the SAME clock across calls). Returns the filter's decision + output
    // pose for this sample. See the class doc comment / doc/
    // MOCAP_POSE_FILTER_PLAN.md for the full accept/reject/re-init state
    // machine.
    PoseFilterOutput step(double t, double zx, double zy, double zyaw);

    // Swap diagnostic: WITHOUT mutating any state, reports whether THIS
    // filter's current track would accept a measurement at (t, zx, zy,
    // zyaw) -- i.e. runs the identical predict+gate math step() would, but
    // never commits it. optitrack_zmq_bridge uses this to test a REJECTED
    // sample from body A against body B's filter (and vice versa): if B's
    // filter would accept A's rejected sample, that's evidence Motive may
    // have swapped the two bodies' identities (see doc/
    // MOCAP_POSE_FILTER_PLAN.md's "What it cannot fix" section).
    bool would_accept(double t, double zx, double zy, double zyaw) const;

    // Re-arms "the next step() call initializes the track from scratch",
    // exactly as if this PoseFilter had just been constructed (config is
    // kept). Also clears the reject/reinit streak state.
    void reset();

    const PoseFilterConfig& config() const { return config_; }

    // Cumulative counts over this instance's lifetime (survive reset()).
    uint64_t rejects() const { return reject_count_; }
    uint64_t reinits() const { return reinit_count_; }

   private:
    struct Sample {
        double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0;
    };

    struct Predicted {
        Eigen::Matrix<double, 6, 1> state;
        Eigen::Matrix<double, 6, 6> P;
    };

    struct Gate {
        Eigen::Matrix<double, 3, 1> nu;   // innovation (yaw component shortest-arc-wrapped).
        Eigen::Matrix<double, 3, 3> S;    // innovation covariance.
        double d2_pos = 0.0, d2_yaw = 0.0, d2_all = 0.0;
    };

    Predicted predict(const Eigen::Matrix<double, 6, 1>& state, const Eigen::Matrix<double, 6, 6>& P,
                       double dt) const;
    Gate compute_gate(const Eigen::Matrix<double, 6, 1>& state_pred,
                       const Eigen::Matrix<double, 6, 6>& P_pred, double zx, double zy,
                       double zyaw) const;
    bool passes_gate(const Gate& g) const;
    // Physical-plausibility check shared by plausible_vs_last_accepted(),
    // buffer_self_consistent(), and would_accept(): true iff a displacement
    // of (dx, dy) and a (signed, already shortest-arc) yaw change of dyaw
    // over `dt` seconds is consistent with max_speed/max_yaw_rate. dt below
    // kMinMeaningfulDt (see PoseFilter.cpp) is always plausible (nothing
    // meaningful to divide by). At larger dt, the allowed displacement is
    // max_speed*dt PLUS a several-sigma measurement-noise margin (r_pos/
    // r_yaw) -- WITHOUT this margin, real recorded data shows the raw
    // |delta|/dt rule spuriously rejecting perfectly clean samples whenever
    // two measurements arrive unusually close together (a few ms, well
    // above kMinMeaningfulDt but still small enough that ordinary
    // millimeter-scale sensor jitter divided by that dt implies an absurd
    // instantaneous speed) -- see doc/MOCAP_POSE_FILTER_PLAN.md's
    // "Implementation status" section for the live-data evidence. The
    // statistical gate (d2_pos/d2_yaw/d2_all) is naturally immune to this
    // (its innovation covariance already reflects the noise floor via R,
    // independent of dt) -- this margin brings the "hard" plausibility
    // floor in line with that same noise floor rather than weakening it.
    bool motion_plausible(double dx, double dy, double dyaw, double dt) const;
    bool plausible_vs_last_accepted(double t, double zx, double zy, double zyaw) const;
    // Every consecutive pair in `buf` (assumed time-ordered) individually
    // passes the max_speed/max_yaw_rate plausibility check -- used to decide
    // whether a persistent-reject streak is a genuine sustained motion
    // (teleport) worth re-initializing on, vs. noise. Requires >= 2 samples.
    bool buffer_self_consistent(const std::deque<Sample>& buf) const;

    void do_init(double t, double zx, double zy, double zyaw);
    void do_reinit_from_gap(double t, double zx, double zy, double zyaw);
    void do_reinit_from_streak(double t, double zx, double zy, double zyaw);

    PoseFilterConfig config_;

    bool initialized_ = false;
    Eigen::Matrix<double, 6, 1> state_ = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity();
    double last_time_ = 0.0;

    Sample last_accepted_{};
    double last_accepted_time_ = 0.0;

    int consecutive_rejects_ = 0;
    std::deque<Sample> rejected_buffer_;

    uint64_t reject_count_ = 0;
    uint64_t reinit_count_ = 0;

    // enabled=false pass-through bookkeeping (finite-difference velocity).
    bool passthrough_has_last_ = false;
    Sample passthrough_last_{};

    PoseFilterOutput last_output_{};
};

}  // namespace mpc
