#include "mpc/Kinematics.h"

#include <cmath>

namespace mpc {

State4 rollout_step(const State4& state, double a, double delta, double wheel_base, double dt) {
    State4 next = state;
    next.v = state.v + a * dt;
    next.x = state.x + next.v * std::cos(state.yaw) * dt;
    next.y = state.y + next.v * std::sin(state.yaw) * dt;
    next.yaw = state.yaw + (next.v / wheel_base) * std::tan(delta) * dt;
    return next;
}

State4 predict_delay_compensated(const State4& state, double a, double delta, double wheel_base,
                                  double dt, int control_delay_steps) {
    State4 s = state;
    for (int i = 0; i < control_delay_steps; ++i) {
        s = rollout_step(s, a, delta, wheel_base, dt);
    }
    return s;
}

} // namespace mpc
