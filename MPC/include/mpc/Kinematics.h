#pragma once

// Shared kinematic-bicycle plant model, parameterized by acceleration.
// Factored out into its own header so it can be used by BOTH the MPC
// controller (MpcCore's own velocity-formulation solve never calls this
// directly, but main.cpp's delay-compensation prediction and dead-reckoning
// fallback do, always with a=0 since v is MpcCore's direct control input)
// and the standalone kinematic simulator (SimCore.cpp, which genuinely
// integrates a nonzero received acceleration -- see the simulator design
// spec) WITHOUT either side duplicating the equations or the simulator
// needing to link Ceres. This header has no Ceres dependency by design --
// only <cmath>.
//
// mpc::State4, mpc::rollout_step, and mpc::predict_delay_compensated are the
// ONE place these equations are written; every caller (main.cpp, the unit
// tests, SimCore) shares this single implementation.

namespace mpc {

// Full vehicle state used by the acceleration-formulation rollout: pose
// (x, y, yaw) plus longitudinal velocity v. Velocity is not a control input
// -- it is integrated from acceleration.
struct State4 {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
};

// One semi-implicit rollout step of the kinematic-bicycle,
// acceleration-control model:
//   v'   = v + a*dt
//   x'   = x + v'*cos(yaw)*dt
//   y'   = y + v'*sin(yaw)*dt
//   yaw' = yaw + (v'/L)*tan(delta)*dt
// (the NEW velocity drives the pose step, matching the old formulation
// where the commanded v applied immediately). Used for the MPC's own
// delay-compensation prediction, the controller's dead-reckoning fallback
// when no fresh localization arrives, AND the standalone simulator's plant
// model (SimCore) -- this is the ONE place the equations are written.
State4 rollout_step(const State4& state, double a, double delta, double wheel_base, double dt);

// Delay compensation: predicts `state` forward by `control_delay_steps`
// rollout_step() calls, all using the same last-commanded (a, delta) --
// i.e. assumes the last command stays in effect while it is in flight.
// control_delay_steps <= 0 means "no compensation" (returns state
// unchanged). This is the ONLY place mpc::MpcParams::control_delay_steps is
// meant to be consumed; main.cpp calls it instead of a hardcoded single
// rollout_step so the JSON-configurable knob actually has an effect.
State4 predict_delay_compensated(const State4& state, double a, double delta, double wheel_base,
                                  double dt, int control_delay_steps);

} // namespace mpc
