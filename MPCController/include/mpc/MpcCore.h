#pragma once

#include <cmath>
#include <vector>

#include <ceres/ceres.h>

#include <ReloPush/trajectory.hpp>

#include "mpc/Kinematics.h"
#include "mpc/RobotSpec.h"

// Acceleration-formulation Ceres MPC: per-stage decision variables are
// [a_t, delta_t] (longitudinal acceleration + steering angle). Velocity is
// NOT a direct control input -- it is a state, integrated stage-by-stage
// from a real velocity estimate (SolveInputs::v0, the delay-compensated
// projection of the controller's own feedback-derived speed -- see
// main.cpp's finite-difference velocity estimate from consecutive
// localization samples) via the SAME semi-implicit rollout equations as
// mpc::rollout_step (Kinematics.h): v' = v + a*dt; x' = x + v'*cos(yaw)*dt;
// y' = y + v'*sin(yaw)*dt; yaw' = yaw + (v'/L)*tan(delta)*dt. This keeps the
// cost functor's internal model identical to the plant model shared with
// the standalone simulator (mpc_robot_sim / SimCore), which is what makes
// the outgoing "accel" field a genuine decision variable (not a post-hoc
// finite difference of two independently-chosen speeds) and gives
// v_cmd == v0 + accel*dt exactly (the payload identity).
namespace mpc {

// Prediction horizon. Stays a compile-time constant by design -- see
// config/robot_spec.json's "_doc" field. All Ceres AutoDiff template
// dimensions below are derived from it, never hardcoded literals.
constexpr int kHorizon = 8;
constexpr int kNumParams = 2 * kHorizon;
// 7 residuals per stage (x, y, yaw, vel, lateral, control-accel,
// control-delta) plus (kHorizon - 1) steering-rate residuals. 63 for
// kHorizon=8, matching the original AutoDiffCostFunction<MPCCostFunctor,
// 63, 16> literals -- residual COUNT is unchanged by the velocity->
// acceleration reformulation, only what channel 6 (control effort) and
// channel 4 (velocity error) are computed from.
constexpr int kNumResiduals = 7 * kHorizon + (kHorizon - 1);

// atan2(sin(angle), cos(angle)); wraps to (-pi, pi]. Works for both plain
// doubles and ceres::Jet<> -- Ceres brings `using std::sin/cos/atan2;` into
// its own namespace, so this compiles for both instantiations used by
// ceres::AutoDiffCostFunction.
template <typename T_num>
T_num normalize_angle(const T_num& angle) {
    return ceres::atan2(ceres::sin(angle), ceres::cos(angle));
}

// Symmetric clamp to [-limit, limit] using ceres::fmax/fmin (NOT a raw
// ternary): both are defined for {double,double} and {Jet,Jet} operand
// pairs and, unlike truncating to a bare T_num(limit) constant, propagate
// the FULL Jet (value + derivative) of whichever operand is selected -- so
// a stage that saturates the per-stage velocity cap still yields a useful
// (if partial) gradient to the solver instead of a dead flat region.
template <typename T_num>
T_num clamp_sym(const T_num& v, double limit) {
    return ceres::fmin(ceres::fmax(v, T_num(-limit)), T_num(limit));
}

// State4, rollout_step, and predict_delay_compensated live in
// mpc/Kinematics.h (included above): the ONE place shared between MpcCore's
// own delay-compensation prediction / main.cpp's dead-reckoning fallback
// (both now called with the REAL last-commanded acceleration, since a is
// this formulation's direct control input -- see main.cpp) and the
// standalone simulator's plant model (SimCore, which integrates the same
// received-acceleration model). MPCCostFunctor::operator() below inlines
// the identical rollout equations in templated (Jet-compatible) form rather
// than calling rollout_step() directly, since rollout_step operates on
// plain doubles only; keep the two in sync by construction if either
// changes.

struct RefState {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    // SIGNED reference velocity: positive when this leg of the trajectory
    // moves in the direction of `yaw`, negative during a genuine reversal
    // (see MARS/include/RobotTrajectoryBuilder.h's dir_sign). Consumed
    // directly by both the velocity-tracking residual below and
    // resolve_dir_change().
    double ref_vel = 0.0;
    bool is_pushing = false;
    // False only when the source trajectory had no waypoints at all; in
    // that case every other field is zeroed and MUST NOT be trusted.
    bool ok = true;
};

// Time-based linear interpolation over a ReloPush::trajectory, clamped to
// the first/last waypoint outside the trajectory's time span, with
// shortest-path (wrap-aware) yaw interpolation. Returns ok=false on an
// empty (or null) trajectory instead of dereferencing front()/back().
RefState get_ref_state_at_time(double abs_time, double traj_start_time,
                                const ReloPush::trajectory& ref_traj);

// Ceres cost functor for Ackermann trajectory tracking -- acceleration
// formulation. Per-stage decision variables are [a_t, delta_t]; velocity is
// a rolled-out state seeded from v0 (the caller's real, feedback-derived
// velocity estimate), not a decision variable.
struct MPCCostFunctor {
    double x0;
    double y0;
    double yaw0;
    double v0;  // Real (delay-compensated) velocity estimate -- rollout IC.
    double L;
    double dt;
    int T;

    double w_dist;
    double w_yaw;
    double w_vel;
    double w_lat;
    double w_control;
    double w_delta_rate;

    // Per-stage velocity caps (mpc::RefState::is_pushing selects which
    // applies at each stage), so a horizon that spans a push/non-push
    // transition uses the correct cap on each side of it rather than one
    // cap resolved once from the current-time reference.
    double max_v_push;
    double max_v_nonpush;

    std::vector<RefState> ref_states;  // size T

    MPCCostFunctor(double x0, double y0, double yaw0, double v0, double L, double dt, int T,
                   const std::vector<RefState>& ref_states, double w_dist, double w_yaw,
                   double w_vel, double w_lat, double w_control, double w_delta_rate,
                   double max_v_push, double max_v_nonpush);

    template <typename T_num>
    bool operator()(const T_num* const u, T_num* residuals) const {
        T_num x = T_num(x0);
        T_num y = T_num(y0);
        T_num yaw = T_num(yaw0);
        T_num v = T_num(v0);

        int res_idx = 0;

        for (int t = 0; t < T; ++t) {
            T_num a = u[2 * t];
            T_num delta = u[2 * t + 1];

            const auto& ref = ref_states[t];
            const double stage_max_v = ref.is_pushing ? max_v_push : max_v_nonpush;

            // Semi-implicit kinematic-bicycle rollout, acceleration control
            // -- IDENTICAL equations to mpc::rollout_step (Kinematics.h),
            // with the resulting velocity soft-clamped to this stage's
            // push/non-push cap (see clamp_sym doc comment above).
            v = v + a * T_num(dt);
            v = clamp_sym(v, stage_max_v);
            x += v * ceres::cos(yaw) * T_num(dt);
            y += v * ceres::sin(yaw) * T_num(dt);
            yaw += (v / T_num(L)) * ceres::tan(delta) * T_num(dt);

            // 1. Position error residuals (x and y)
            residuals[res_idx++] = T_num(std::sqrt(w_dist)) * (x - T_num(ref.x));
            residuals[res_idx++] = T_num(std::sqrt(w_dist)) * (y - T_num(ref.y));

            // 2. Heading error residual (shortest angle wrap)
            T_num dyaw = yaw - T_num(ref.yaw);
            residuals[res_idx++] = T_num(std::sqrt(w_yaw)) * normalize_angle(dyaw);

            // 3. Velocity error residual (both signed, so this fires
            // correctly through a reversal instead of only ever pulling
            // toward a positive reference).
            residuals[res_idx++] = T_num(std::sqrt(w_vel)) * (v - T_num(ref.ref_vel));

            // 4. Lateral error residual
            T_num dx = T_num(ref.x) - x;
            T_num dy = T_num(ref.y) - y;
            T_num e_y = -ceres::sin(yaw) * dx + ceres::cos(yaw) * dy;
            residuals[res_idx++] = T_num(std::sqrt(w_lat)) * e_y;

            // 5. Control effort residuals -- acceleration and steering
            // (previously v and delta; the velocity-formulation's "cost of
            // holding v away from 0" doesn't apply to a state, so this is
            // now a genuine actuator-effort penalty on the two real control
            // inputs).
            residuals[res_idx++] = T_num(std::sqrt(w_control)) * a;
            residuals[res_idx++] = T_num(std::sqrt(w_control)) * delta;
        }

        // 6. Steering rate change residuals
        for (int t = 1; t < T; ++t) {
            T_num delta_rate = u[2 * t + 1] - u[2 * (t - 1) + 1];
            residuals[res_idx++] = T_num(std::sqrt(w_delta_rate)) * delta_rate;
        }

        return true;
    }
};

// Inputs for one MPC solve.
struct SolveInputs {
    double x0 = 0.0;  // pred_state.x (delay-compensated)
    double y0 = 0.0;
    double yaw0 = 0.0;
    // Delay-compensated REAL velocity estimate -- the rollout's initial
    // condition. Traces back to a finite-difference of consecutive
    // localization samples (see main.cpp), not to the controller's own
    // last-commanded value, so the solve has true velocity feedback instead
    // of trusting its own open-loop belief.
    double v0 = 0.0;

    // Previous stage-0 commanded acceleration, used to flat-seed u[] when
    // use_warm_start is false (or no valid warm-start buffer exists yet).
    double last_cmd_a = 0.0;
    double last_cmd_delta = 0.0;

    std::vector<RefState> horizon_refs;  // size kHorizon, one per stage

    double wheel_base = 0.29;
    double dt = 0.05;

    double w_dist = 5.0;  // already resolved to w_dist or w_dist_dirchange
    double w_yaw = 1.0;
    double w_vel = 0.2;
    double w_lat = 40.0;
    double w_control = 0.001;
    double w_delta_rate = 50.0;

    // Per-stage velocity caps -- see MPCCostFunctor's doc comment. Replaces
    // the pre-round single `max_v` resolved once from the current-time
    // reference.
    double max_v_push = 0.295;
    double max_v_nonpush = 0.38;
    double max_steer = 0.33;
    double max_accel = 0.73;

    // FEATURE 2A (pause/resume) opt-in only -- defaults to false, which
    // reproduces the STAGE 1 solve() behavior EXACTLY (flat seed from
    // last_cmd_a/last_cmd_delta every call, no state carried across calls).
    // main.cpp's normal running-loop call site never sets this, so the wire
    // protocol / control behavior of a controller that is never paused is
    // byte-for-byte unchanged. When true (and MpcSolver holds a valid stored
    // horizon from a PRIOR use_warm_start=true call), solve() seeds u[] from
    // that previous raw horizon shifted forward one stage instead of the
    // flat seed -- see MpcSolver::reset_warm_start().
    bool use_warm_start = false;
};

// Outputs of one MPC solve.
struct MpcOutput {
    // Rolled-forward velocity after stage 0: v0 + accel*dt, clamped to
    // stage 0's push/non-push cap. This IS the "speed" the payload
    // publishes (before vel_scale/back/dir_change_scale) -- see
    // build_payload().
    double v_cmd = 0.0;
    double steer = 0.0;  // opt_delta (stage 0's steering decision variable).
    // Stage 0's commanded acceleration, DERIVED from v_cmd and the caller's
    // v0 as (v_cmd - v0) / dt so the identity accel*dt == v_cmd - v0 holds
    // exactly even when v_cmd was clamped to the velocity cap (rather than
    // reporting the solver's raw, possibly-unclamped u[0] decision value).
    double accel = 0.0;
    bool solved = false;
    int iterations = 0;

    // Full horizon of the RAW (pre-clamp) optimized controls, size
    // kHorizon: horizon_a[t]/horizon_delta[t] are the solver's u[2*t]/
    // u[2*t+1]. Not consumed by main.cpp (which only ever used stage 0's
    // v_cmd/steer) -- exposed purely for unit-test observability of the
    // solver's per-stage [-max_accel,max_accel]/[-max_steer,max_steer] box
    // bounds, which the solve() loop already applies to every stage (see
    // MpcCore.cpp). Does not change any existing behavior or the wire
    // protocol.
    std::vector<double> horizon_a;
    std::vector<double> horizon_delta;
};

// Acceleration-formulation MPC solver. By default (SolveInputs::
// use_warm_start left at its default of false) each solve seeds u[] with
// `last_cmd_a`/`last_cmd_delta` held constant across the horizon, with no
// state carried across calls. When a caller opts in (use_warm_start=true,
// FEATURE 2A -- this is what main.cpp's live control loop uses), solve()
// instead seeds u[] from THIS solver's own previous use_warm_start=true
// solve, shifted forward one stage with the last stage duplicated (see
// reset_warm_start() for invalidating that stored seed, e.g. across a
// PAUSE/RESUME or a new trajectory). Either way, solves via (DENSE_QR, 30
// max iterations, SILENT); v_cmd/accel are then derived from the resulting
// u[0] and the caller's v0 (see MpcOutput's doc comment).
class MpcSolver {
   public:
    MpcOutput solve(const SolveInputs& inputs) const;

    // FEATURE 2A debug/production hook: invalidates any stored warm-start
    // seed from a prior SolveInputs::use_warm_start=true call. A converged
    // solution goes stale once the vehicle has been braked to a stop (e.g.
    // across a PAUSE) -- the next resumed solve should NOT be seeded from a
    // horizon that no longer reflects where the reference/vehicle actually
    // are. No-op (and harmless) if no warm start has ever been requested.
    void reset_warm_start();

   private:
    // Raw (pre-clamp) horizon solution from the most recent
    // use_warm_start=true solve, only ever populated/consumed when that flag
    // is used -- callers who never opt in never observe any statefulness.
    mutable std::vector<double> warm_u_;
    mutable bool warm_valid_ = false;
};

// Replicates the pre-round main.cpp's inline direction-change check EXACTLY,
// including its asymmetry: w_dist is bumped to w_dist_dirchange for EITHER
// sign flip (current_v/ref_vel disagreeing in sign), but change_dir (which
// gates dir_change_scale in build_payload) is only set for the
// current_v>0 && ref_vel<0 case. `current_v` should be the controller's REAL
// (feedback-derived, delay-compensated) velocity estimate -- SolveInputs::v0
// -- not an open-loop last-commanded value, so this check actually reacts to
// the vehicle's true state through a reversal (see main.cpp).
struct DirChangeResolution {
    double w_dist;
    bool change_dir;
};
DirChangeResolution resolve_dir_change(double current_v, double ref_vel, double w_dist_default,
                                        double w_dist_dirchange);

// Outgoing VESC payload values (pre base64/ASCII encoding).
struct PayloadValues {
    double speed = 0.0;
    double steering = 0.0;
    double accel = 0.0;
};

// Builds the outgoing VESC payload. Matches the pre-round controller
// byte-for-byte on the wire-scaling policy: vel_scale/vel_scale_back is
// ALWAYS applied to the published "speed" (there is no hardware-calibration
// on/off switch -- every invocation of mpc_controller scales the wire
// "speed" field this way), and dir_change_scale is applied on top whenever
// dir_change_detected is set. "accel" is passed through UNSCALED from the
// caller (MpcOutput::accel, itself (v_cmd - v0)/dt), preserving the
// accel*dt == Δ(unscaled speed) identity on the wire.
PayloadValues build_payload(double opt_v, double opt_delta, double accel, double vel_scale,
                             double vel_scale_back, double dir_change_scale,
                             bool dir_change_detected);

// ---------------------------------------------------------------------
// FEATURE 2A (mpc_controller PAUSE/RESUME) helpers.
// ---------------------------------------------------------------------

// Output of compute_stop_accel(): a single rate-limited step of the current
// velocity toward zero.
struct StopAccelResult {
    double v_cmd = 0.0;  // current_v moved at most max_accel*dt toward 0.0
    double accel = 0.0;  // (v_cmd - current_v) / dt
};

// PAUSE-loop / shutdown-stop-burst braking step, used INSTEAD of
// MpcSolver::solve() (no Ceres solve is invoked). Pure function: moves
// current_v toward 0.0 by at most max_accel*dt (the same bound the solve's
// box constraint on `a` enforces), snapping exactly to 0.0 once within one
// step so the vehicle does not overshoot into reverse. `current_v` should be
// the controller's real, continuously-updated velocity estimate (not an
// open-loop last-commanded value), so braking during a pause tracks the
// vehicle's true speed. Feeding compute_stop_accel()'s output back through
// build_payload() preserves the accel == d(speed)/dt identity the same way
// the normal solve loop does.
StopAccelResult compute_stop_accel(double current_v, double max_accel, double dt);

// Pure helper computing the pause-adjusted (frozen-during-pause)
// trajectory-relative elapsed time:
//   effective_elapsed = now - traj_start_time - pause_offset
//                       - (paused ? (now - pause_start) : 0)
// `pause_offset` is the caller-accumulated total of all COMPLETED pause
// durations (updated by the caller on RESUME: pause_offset +=
// (resume_time - pause_start)); `pause_start` is only meaningful while
// `paused` is true. When paused, the result is independent of `now` (the
// clock is frozen at the value it had the instant the pause began). Used by
// main.cpp for BOTH the reference-trajectory lookup (added back onto
// traj_start_time to form an adjusted abs_time for get_ref_state_at_time)
// and the trajectory-completion check, so a paused controller neither
// advances its reference nor spuriously completes.
double compute_effective_elapsed(double now, double traj_start_time, double pause_offset,
                                  bool paused, double pause_start);

}  // namespace mpc
