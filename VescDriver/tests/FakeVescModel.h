// FakeVescModel.h
//
// The fake VESC's motor simulation + command-application logic, factored
// out of tests/fake_vesc.cpp into its own header so it can be reused
// in-process by other tests later (per this part's task spec) without
// needing to fork/exec the fake_vesc binary. Pure simulation state -- no
// I/O of any kind lives here; fake_vesc.cpp owns the pty/serial plumbing
// and calls into this class.
//
// Header-only (small enough that a separate .cpp isn't worth the extra
// build-graph node); safe to #include from multiple translation units.
//
// Portability: C++14 only -- see VescDriver/CMakeLists.txt's HARD
// PORTABILITY RULES. Only depends on VescProtocol.h (for CommandId and the
// GET_VALUES reply layout) from the rest of VescDriver.

#ifndef VESC_DRIVER_TESTS_FAKE_VESC_MODEL_H_
#define VESC_DRIVER_TESTS_FAKE_VESC_MODEL_H_

#include "../src/VescProtocol.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace vesc_test {

struct FakeVescModelParams {
    double tau_s = 0.15;            // first-order erpm response time constant
    double erpm_per_duty = 30000.0; // SET_DUTY -> erpm_target scale
    double stall_duty = 0.03;       // |duty| below this -> erpm_target = 0
    double erpm_per_amp = 300.0;    // SET_CURRENT -> erpm_target scale
    double stall_amp = 1.0;         // |amps| below this -> erpm_target = 0
    int32_t stall_erpm = 800;       // SET_RPM: |erpm| below this -> erpm_target = 0
    double v_in = 12.0;             // reported in GET_VALUES
};

namespace detail {

inline int32_t read_be_i32_at(const std::vector<uint8_t>& payload, size_t idx) {
    const uint32_t u = (static_cast<uint32_t>(payload[idx]) << 24) |
                        (static_cast<uint32_t>(payload[idx + 1]) << 16) |
                        (static_cast<uint32_t>(payload[idx + 2]) << 8) |
                        static_cast<uint32_t>(payload[idx + 3]);
    return static_cast<int32_t>(u);
}

inline uint16_t read_be_u16_at(const std::vector<uint8_t>& payload, size_t idx) {
    return static_cast<uint16_t>((static_cast<uint16_t>(payload[idx]) << 8) |
                                  static_cast<uint16_t>(payload[idx + 1]));
}

inline void append_be_i32(std::vector<uint8_t>* out, int32_t value) {
    const uint32_t u = static_cast<uint32_t>(value);
    out->push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
    out->push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
    out->push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(u & 0xFF));
}

inline void append_be_i16(std::vector<uint8_t>* out, int16_t value) {
    const uint16_t u = static_cast<uint16_t>(value);
    out->push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(u & 0xFF));
}

inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }

}  // namespace detail

// A small first-order motor model: d(erpm)/dt = (erpm_target - erpm) / tau.
// Each SET_* command updates erpm_target (and, for SET_CURRENT_BRAKE, the
// active tau -- braking is modeled 4x faster than the configured tau).
// step() integrates the response exactly (closed-form exponential decay
// toward the target, so results don't depend on how finely the caller
// slices dt) and accumulates a tachometer from erpm.
class FakeVescModel {
public:
    explicit FakeVescModel(FakeVescModelParams params = FakeVescModelParams())
        : params_(params), tau_active_(params.tau_s) {}

    // Applies one decoded VESC command payload (id byte + big-endian args,
    // as produced by VescProtocol's command builders) to the model's
    // target state. Returns a short lowercase command name for logging
    // ("set_duty", "set_rpm", "set_current", "set_current_brake",
    // "set_servo_pos", "alive"), or an empty string if `payload` is empty,
    // too short for its own command's args, or not one of those commands
    // (e.g. FW_VERSION/GET_VALUES requests -- those are request/reply
    // commands the caller handles itself, not "apply a target" commands).
    // On success, *out_value is set to the command's decoded value
    // (physical units: duty as a fraction, amps, erpm, or servo position
    // in [0,1]; 0.0 for ALIVE, which carries no value).
    std::string apply_command(const std::vector<uint8_t>& payload, double* out_value) {
        if (out_value) *out_value = 0.0;
        if (payload.empty()) return "";
        const vesc::CommandId id = static_cast<vesc::CommandId>(payload[0]);

        switch (id) {
            case vesc::CommandId::SET_DUTY: {
                if (payload.size() < 5) return "";
                const double duty = detail::read_be_i32_at(payload, 1) / 100000.0;
                erpm_target_ = (std::fabs(duty) < params_.stall_duty) ? 0.0 : duty * params_.erpm_per_duty;
                tau_active_ = params_.tau_s;
                if (out_value) *out_value = duty;
                return "set_duty";
            }
            case vesc::CommandId::SET_CURRENT: {
                if (payload.size() < 5) return "";
                const double amps = detail::read_be_i32_at(payload, 1) / 1000.0;
                erpm_target_ = (std::fabs(amps) < params_.stall_amp) ? 0.0 : amps * params_.erpm_per_amp;
                tau_active_ = params_.tau_s;
                if (out_value) *out_value = amps;
                return "set_current";
            }
            case vesc::CommandId::SET_CURRENT_BRAKE: {
                if (payload.size() < 5) return "";
                const double amps = detail::read_be_i32_at(payload, 1) / 1000.0;
                erpm_target_ = 0.0;
                tau_active_ = params_.tau_s / 4.0;
                if (out_value) *out_value = amps;
                return "set_current_brake";
            }
            case vesc::CommandId::SET_RPM: {
                if (payload.size() < 5) return "";
                const int32_t erpm = detail::read_be_i32_at(payload, 1);
                erpm_target_ = (detail::iabs32(erpm) < params_.stall_erpm) ? 0.0 : static_cast<double>(erpm);
                tau_active_ = params_.tau_s;
                if (out_value) *out_value = static_cast<double>(erpm);
                return "set_rpm";
            }
            // Accepts SET_SERVO_POS under EITHER its modern FW5.x/6.x id
            // (12, vesc::CommandId::SET_SERVO_POS) or FW2.x's shifted id
            // (11, since FW2.x lacks COMM_SET_HANDBRAKE -- see
            // VescProtocol.h's resolve_servo_cmd_id() and its CAVEAT
            // comment) so tests can drive either id flexibly; the body
            // doesn't care which id it was called under.
            case static_cast<vesc::CommandId>(11):
            case vesc::CommandId::SET_SERVO_POS: {
                if (payload.size() < 3) return "";
                const double pos = detail::read_be_u16_at(payload, 1) / 1000.0;
                servo_pos_ = pos;
                if (out_value) *out_value = pos;
                return "set_servo_pos";
            }
            case vesc::CommandId::ALIVE:
                return "alive";
            default:
                return "";
        }
    }

    // Advances the first-order erpm response (closed-form exponential
    // toward erpm_target_ over dt_s seconds, using whichever tau was set
    // by the most recent apply_command()) and the tachometer integral.
    void step(double dt_s) {
        if (dt_s <= 0.0) return;
        if (tau_active_ > 1e-9) {
            const double decay = std::exp(-dt_s / tau_active_);
            erpm_ = erpm_target_ + (erpm_ - erpm_target_) * decay;
        } else {
            erpm_ = erpm_target_;
        }
        // erpm is electrical RPM -> revolutions/second = erpm/60; integrate
        // fractional revolutions into an int32 tachometer via an
        // accumulator so small dt*erpm slices aren't lost to truncation.
        tachometer_acc_ += (erpm_ / 60.0) * dt_s;
        tachometer_ = static_cast<int32_t>(tachometer_acc_);
    }

    double erpm() const { return erpm_; }
    double erpm_target() const { return erpm_target_; }
    int32_t tachometer() const { return tachometer_; }

    // Builds the current GET_VALUES reply payload. `legacy=false` (the
    // default) uses the exact big-endian layout VescProtocol::
    // parse_get_values() expects (modern FW 5.x/6.x -- see VescProtocol.h's
    // file-header table): command id, then scaled fields through `fault`.
    // `legacy=true` instead emits the FW 2.x layout that
    // VescProtocol::parse_get_values_legacy() expects (temp_mos1..6 +
    // temp_pcb in place of temp_fet/temp_motor, no avg_id/avg_iq fields --
    // see that function's own doc comment in VescProtocol.h for the full
    // field table) -- used by fake_vesc's --fw 2.x mode so the whole
    // vesc_teleop loop is testable against a fake FW-2.x unit without real
    // hardware.
    std::vector<uint8_t> build_get_values_reply(bool legacy = false) const {
        // Plausible currents: a small idle draw plus a term proportional to
        // how far erpm still has to move toward its target (i.e. higher
        // "torque demand" while accelerating/braking, settling toward the
        // idle draw once erpm has converged). Shared by both layouts below.
        const double current_motor = 0.5 + 0.00002 * std::fabs(erpm_target_ - erpm_);
        const double current_in = current_motor * 0.9;
        double duty = params_.erpm_per_duty > 1e-9 ? erpm_ / params_.erpm_per_duty : 0.0;
        duty = std::max(-1.0, std::min(1.0, duty));

        std::vector<uint8_t> out;
        out.push_back(static_cast<uint8_t>(vesc::CommandId::GET_VALUES));

        if (legacy) {
            for (int i = 0; i < 6; ++i) {
                detail::append_be_i16(&out, static_cast<int16_t>(std::lround(25.0 * 10)));  // temp_mos1..6 ~25C
            }
            detail::append_be_i16(&out, static_cast<int16_t>(std::lround(24.0 * 10)));  // temp_pcb ~24C
            detail::append_be_i32(&out, static_cast<int32_t>(std::lround(current_motor * 100)));
            detail::append_be_i32(&out, static_cast<int32_t>(std::lround(current_in * 100)));
            detail::append_be_i16(&out, static_cast<int16_t>(std::lround(duty * 1000)));
            detail::append_be_i32(&out, static_cast<int32_t>(std::lround(erpm_)));
            detail::append_be_i16(&out, static_cast<int16_t>(std::lround(params_.v_in * 10)));
            detail::append_be_i32(&out, 0);  // amp_hours
            detail::append_be_i32(&out, 0);  // amp_hours_charged
            detail::append_be_i32(&out, 0);  // watt_hours
            detail::append_be_i32(&out, 0);  // watt_hours_charged
            detail::append_be_i32(&out, tachometer_);
            detail::append_be_i32(&out, tachometer_);  // tachometer_abs -- reuse (no reversal tracking modeled)
            out.push_back(0);                          // fault = 0
            return out;
        }

        detail::append_be_i16(&out, static_cast<int16_t>(std::lround(25.0 * 10)));  // temp_fet ~25C
        detail::append_be_i16(&out, static_cast<int16_t>(std::lround(30.0 * 10)));  // temp_motor ~30C
        detail::append_be_i32(&out, static_cast<int32_t>(std::lround(current_motor * 100)));
        detail::append_be_i32(&out, static_cast<int32_t>(std::lround(current_in * 100)));
        detail::append_be_i32(&out, 0);  // avg_id (not modeled)
        detail::append_be_i32(&out, 0);  // avg_iq (not modeled)
        detail::append_be_i16(&out, static_cast<int16_t>(std::lround(duty * 1000)));
        detail::append_be_i32(&out, static_cast<int32_t>(std::lround(erpm_)));
        detail::append_be_i16(&out, static_cast<int16_t>(std::lround(params_.v_in * 10)));
        detail::append_be_i32(&out, 0);  // amp_hours
        detail::append_be_i32(&out, 0);  // amp_hours_charged
        detail::append_be_i32(&out, 0);  // watt_hours
        detail::append_be_i32(&out, 0);  // watt_hours_charged
        detail::append_be_i32(&out, tachometer_);
        detail::append_be_i32(&out, tachometer_);  // tachometer_abs -- reuse (no reversal tracking modeled)
        out.push_back(0);                          // fault = 0

        return out;
    }

private:
    FakeVescModelParams params_;
    double erpm_ = 0.0;
    double erpm_target_ = 0.0;
    double tau_active_;
    double servo_pos_ = 0.0;
    double tachometer_acc_ = 0.0;
    int32_t tachometer_ = 0;
};

}  // namespace vesc_test

#endif  // VESC_DRIVER_TESTS_FAKE_VESC_MODEL_H_
