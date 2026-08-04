#include "mpc/MpcCore.h"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace mpc {

// State4/rollout_step/predict_delay_compensated live in
// mpc/Kinematics.h/.cpp -- see MpcCore.h's include.

namespace {

RefState ref_state_from_point(const ReloPush::trajectory_elem& p) {
    RefState out;
    out.x = p.x;
    out.y = p.y;
    out.yaw = p.yaw;
    out.ref_vel = p.ref_vel;
    out.is_pushing = p.is_pushing;
    out.ok = true;
    return out;
}

} // namespace

RefState get_ref_state_at_time(double abs_time, double traj_start_time,
                                const ReloPush::trajectory& ref_traj) {
    RefState empty_out;
    empty_out.ok = false;

    if (!ref_traj.trajectory_points) {
        return empty_out;
    }
    const auto& points = *(ref_traj.trajectory_points);
    if (points.empty()) {
        return empty_out;
    }

    double rel_time = abs_time - traj_start_time;

    if (rel_time <= points.front().time) {
        return ref_state_from_point(points.front());
    }
    if (rel_time >= points.back().time) {
        return ref_state_from_point(points.back());
    }

    auto it = std::lower_bound(
        points.begin(), points.end(), rel_time,
        [](const ReloPush::trajectory_elem& elem, double val) { return elem.time < val; });

    int idx = static_cast<int>(std::distance(points.begin(), it));
    if (idx == 0) {
        return ref_state_from_point(points.front());
    }

    const auto& p0 = points[idx - 1];
    const auto& p1 = points[idx];

    double t0 = p0.time;
    double t1 = p1.time;
    double f = (rel_time - t0) / (t1 - t0);

    RefState out;
    out.x = (1.0 - f) * p0.x + f * p1.x;
    out.y = (1.0 - f) * p0.y + f * p1.y;
    out.ref_vel = (1.0 - f) * p0.ref_vel + f * p1.ref_vel;

    double yaw0 = p0.yaw;
    double yaw1 = p1.yaw;
    double dyaw = normalize_angle<double>(yaw1 - yaw0);
    out.yaw = yaw0 + f * dyaw;
    out.is_pushing = p1.is_pushing;
    out.ok = true;
    return out;
}

MPCCostFunctor::MPCCostFunctor(double x0_, double y0_, double yaw0_, double v0_, double L_,
                                double dt_, int T_, const std::vector<RefState>& ref_states_,
                                double w_dist_, double w_yaw_, double w_vel_, double w_lat_,
                                double w_control_, double w_delta_rate_, double max_v_push_,
                                double max_v_nonpush_)
    : x0(x0_),
      y0(y0_),
      yaw0(yaw0_),
      v0(v0_),
      L(L_),
      dt(dt_),
      T(T_),
      w_dist(w_dist_),
      w_yaw(w_yaw_),
      w_vel(w_vel_),
      w_lat(w_lat_),
      w_control(w_control_),
      w_delta_rate(w_delta_rate_),
      max_v_push(max_v_push_),
      max_v_nonpush(max_v_nonpush_),
      ref_states(ref_states_) {}

MpcOutput MpcSolver::solve(const SolveInputs& inputs) const {
    MpcOutput out;

    std::vector<double> u(kNumParams);
    if (inputs.use_warm_start && warm_valid_ && static_cast<int>(warm_u_.size()) == kNumParams) {
        // Shift the previously stored raw horizon forward by one stage (the
        // horizon has advanced one control period since that solve); the
        // final stage repeats its own last value.
        for (int t = 0; t < kHorizon - 1; ++t) {
            u[2 * t] = warm_u_[2 * (t + 1)];
            u[2 * t + 1] = warm_u_[2 * (t + 1) + 1];
        }
        u[2 * (kHorizon - 1)] = warm_u_[2 * (kHorizon - 1)];
        u[2 * (kHorizon - 1) + 1] = warm_u_[2 * (kHorizon - 1) + 1];
    } else {
        for (int t = 0; t < kHorizon; ++t) {
            u[2 * t] = inputs.last_cmd_a;
            u[2 * t + 1] = inputs.last_cmd_delta;
        }
    }

    // Defensive: the functor indexes ref_states[0..kHorizon-1]; pad a short
    // caller-supplied vector with its last value (or a default-constructed
    // RefState if empty, which should never happen in practice) rather than
    // reading OOB.
    std::vector<RefState> refs = inputs.horizon_refs;
    if (static_cast<int>(refs.size()) < kHorizon) {
        RefState fill = refs.empty() ? RefState{} : refs.back();
        refs.resize(kHorizon, fill);
    }

    ceres::Problem problem;
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<MPCCostFunctor, kNumResiduals, kNumParams>(
            new MPCCostFunctor(inputs.x0, inputs.y0, inputs.yaw0, inputs.v0, inputs.wheel_base,
                                inputs.dt, kHorizon, refs, inputs.w_dist, inputs.w_yaw,
                                inputs.w_vel, inputs.w_lat, inputs.w_control, inputs.w_delta_rate,
                                inputs.max_v_push, inputs.max_v_nonpush));

    problem.AddResidualBlock(cost_function, nullptr, u.data());

    for (int t = 0; t < kHorizon; ++t) {
        problem.SetParameterLowerBound(u.data(), 2 * t, -inputs.max_accel);
        problem.SetParameterUpperBound(u.data(), 2 * t, inputs.max_accel);
        problem.SetParameterLowerBound(u.data(), 2 * t + 1, -inputs.max_steer);
        problem.SetParameterUpperBound(u.data(), 2 * t + 1, inputs.max_steer);
    }

    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = 30;
    solver_options.logging_type = ceres::SILENT;
    solver_options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(solver_options, &problem, &summary);

    const double opt_a = u[0];
    const double opt_delta = u[1];

    // Roll stage 0 forward exactly as the cost functor did (same equations,
    // same per-stage velocity cap), then derive accel from the (possibly
    // clamped) result -- this is what guarantees the accel*dt == v_cmd - v0
    // identity holds even when the velocity cap was active, rather than
    // reporting the solver's raw (possibly out-of-cap) u[0] unconditionally.
    const bool stage0_pushing = !refs.empty() && refs[0].is_pushing;
    const double stage0_max_v = stage0_pushing ? inputs.max_v_push : inputs.max_v_nonpush;
    double v_cmd = inputs.v0 + opt_a * inputs.dt;
    if (v_cmd > stage0_max_v) {
        v_cmd = stage0_max_v;
    } else if (v_cmd < -stage0_max_v) {
        v_cmd = -stage0_max_v;
    }

    out.v_cmd = v_cmd;
    out.steer = opt_delta;
    out.accel = (inputs.dt > 0.0) ? (v_cmd - inputs.v0) / inputs.dt : 0.0;
    out.solved = summary.IsSolutionUsable();
    out.iterations = static_cast<int>(summary.iterations.size());

    out.horizon_a.resize(kHorizon);
    out.horizon_delta.resize(kHorizon);
    for (int t = 0; t < kHorizon; ++t) {
        out.horizon_a[t] = u[2 * t];
        out.horizon_delta[t] = u[2 * t + 1];
    }

    // Only ever populate/consult the warm-start buffer for callers that
    // opted in -- everyone else observes MpcSolver as fully stateless, exactly
    // as before this feature existed.
    if (inputs.use_warm_start) {
        warm_u_ = u;
        warm_valid_ = true;
    }

    return out;
}

void MpcSolver::reset_warm_start() {
    warm_u_.clear();
    warm_valid_ = false;
}

DirChangeResolution resolve_dir_change(double current_v, double ref_vel, double w_dist_default,
                                        double w_dist_dirchange) {
    DirChangeResolution r{w_dist_default, false};
    if ((current_v > 0.0 && ref_vel < 0.0) || (current_v < 0.0 && ref_vel > 0.0)) {
        r.w_dist = w_dist_dirchange;
        if (current_v > 0.0 && ref_vel < 0.0) {
            r.change_dir = true;
        }
    }
    return r;
}

PayloadValues build_payload(double opt_v, double opt_delta, double accel, double vel_scale,
                             double vel_scale_back, double dir_change_scale,
                             bool dir_change_detected) {
    PayloadValues out;
    out.steering = opt_delta;

    double scaled_speed = (opt_v < 0.0) ? vel_scale_back * opt_v : vel_scale * opt_v;
    if (dir_change_detected) {
        scaled_speed *= dir_change_scale;
    }
    out.speed = scaled_speed;
    out.accel = accel;
    return out;
}

StopAccelResult compute_stop_accel(double current_v, double max_accel, double dt) {
    StopAccelResult out;
    const double max_delta_v = max_accel * dt;
    double v = current_v;
    if (v > max_delta_v) {
        v -= max_delta_v;
    } else if (v < -max_delta_v) {
        v += max_delta_v;
    } else {
        v = 0.0;
    }
    out.v_cmd = v;
    out.accel = (dt > 0.0) ? (v - current_v) / dt : 0.0;
    return out;
}

double compute_effective_elapsed(double now, double traj_start_time, double pause_offset,
                                  bool paused, double pause_start) {
    double elapsed = now - traj_start_time - pause_offset;
    if (paused) {
        elapsed -= (now - pause_start);
    }
    return elapsed;
}

} // namespace mpc
