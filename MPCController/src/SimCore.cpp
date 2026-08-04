#include "mpc/SimCore.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include <ReloPush/base64.h>

namespace mpc {

namespace {

// Decodes one base64(ASCII-decimal) field exactly as main.cpp's
// encodeAscii() produced it (base64_encode(ostringstream << setprecision(16)
// << value)). Throws (caught by the caller) on any malformed input --
// non-base64 characters, empty string, or a decoded string that isn't a
// valid decimal number.
double decode_ascii_field(const std::string& b64_value) {
    const std::string decoded = base64_decode(b64_value);
    if (decoded.empty()) {
        throw std::runtime_error("decode_ascii_field: empty decoded value");
    }
    size_t consumed = 0;
    const double value = std::stod(decoded, &consumed);
    // Reject trailing garbage (e.g. "0.2xyz") rather than silently accepting
    // a partially-parsed number.
    if (consumed != decoded.size()) {
        throw std::runtime_error("decode_ascii_field: trailing characters after number");
    }
    return value;
}

} // namespace

AckermannCommand decode_ackermann_payload(const std::string& payload) {
    AckermannCommand out;  // ok=false, all-zero -- the safe "didn't parse" default.

    if (payload.empty()) {
        return out;
    }

    try {
        // Guard against non-JSON payloads before handing them to the parser
        // (mirrors main.cpp's own `last_payload.front() == '{'` check).
        if (payload.front() != '{') {
            return out;
        }
        const nlohmann::json j = nlohmann::json::parse(payload);
        if (!j.contains("speed") || !j.contains("steering") || !j.contains("accel")) {
            return out;
        }

        const double speed = decode_ascii_field(j.at("speed").get<std::string>());
        const double steering = decode_ascii_field(j.at("steering").get<std::string>());
        const double accel = decode_ascii_field(j.at("accel").get<std::string>());

        out.speed = speed;
        out.steering = steering;
        out.accel = accel;
        out.ok = true;
    } catch (...) {
        // Malformed JSON, missing/wrong-typed keys, non-base64 values, or a
        // non-numeric decoded string all land here. Never propagate -- the
        // I/O layer must keep running and retain the last valid command.
        return AckermannCommand{};
    }

    return out;
}

std::string encode_localization_payload(double x, double y, double yaw) {
    nlohmann::json j;
    j["x"] = x;
    j["y"] = y;
    j["yaw"] = yaw;
    return j.dump();
}

ClampResult clamp_command(double accel, double steering, double max_accel, double max_steer) {
    ClampResult out;
    const double abs_max_accel = std::fabs(max_accel);
    const double abs_max_steer = std::fabs(max_steer);

    if (accel > abs_max_accel) {
        out.accel = abs_max_accel;
        out.accel_clamped = true;
    } else if (accel < -abs_max_accel) {
        out.accel = -abs_max_accel;
        out.accel_clamped = true;
    } else {
        out.accel = accel;
    }

    if (steering > abs_max_steer) {
        out.steering = abs_max_steer;
        out.steering_clamped = true;
    } else if (steering < -abs_max_steer) {
        out.steering = -abs_max_steer;
        out.steering_clamped = true;
    } else {
        out.steering = steering;
    }

    return out;
}

double watchdog_brake_accel(double v, double max_accel, double zero_v_eps) {
    const double abs_max_accel = std::fabs(max_accel);
    if (std::fabs(v) < zero_v_eps) {
        return 0.0;
    }
    // Oppose the current sign of v; braking to standstill never reverses
    // direction (see resolve_command/ResolvedCommand doc comment).
    return (v > 0.0) ? -abs_max_accel : abs_max_accel;
}

ResolvedCommand resolve_command(bool watchdog_engaged, double current_v, double last_cmd_accel,
                                 double last_cmd_steering, double max_accel, double zero_v_eps) {
    ResolvedCommand out;
    out.steering = last_cmd_steering;  // HOLD last steering, engaged or not.
    if (watchdog_engaged) {
        out.accel = watchdog_brake_accel(current_v, max_accel, zero_v_eps);
        // See ResolvedCommand::snap_v_to_zero doc comment: accel==0 here
        // means "already close enough to stopped", but that alone leaves
        // whatever sub-eps residual v currently is untouched -- flag it so
        // the caller can zero it explicitly instead of integrating a
        // constant-velocity drift indefinitely.
        out.snap_v_to_zero = std::fabs(current_v) < zero_v_eps;
    } else {
        out.accel = last_cmd_accel;
    }
    return out;
}

bool Watchdog::update(double last_cmd_age_s, double watchdog_timeout_s) {
    just_engaged_ = false;
    just_recovered_ = false;

    const bool should_be_engaged = last_cmd_age_s > watchdog_timeout_s;
    if (should_be_engaged && !engaged_) {
        engaged_ = true;
        just_engaged_ = true;
    } else if (!should_be_engaged && engaged_) {
        // update() alone never recovers based on age shrinking -- recovery
        // is driven exclusively by on_command_received() per the design
        // spec ("next received command disengages immediately"). This
        // branch is intentionally unreachable via update() alone since
        // last_cmd_age_s only grows between commands; kept only so the
        // state machine is self-consistent if ever queried with a
        // decreasing age.
        engaged_ = false;
        just_recovered_ = true;
    }
    return engaged_;
}

void Watchdog::on_command_received() {
    just_engaged_ = false;
    just_recovered_ = engaged_;  // only a real transition if we WERE engaged.
    engaged_ = false;
}

// ---------------------------------------------------------------------
// FEATURE A: actuation noise.
// ---------------------------------------------------------------------

double NoiseModel::clamp_sigma_pct(double sigma_pct) {
    if (!std::isfinite(sigma_pct)) {
        return kMinSigmaPct;
    }
    if (sigma_pct < kMinSigmaPct) {
        return kMinSigmaPct;
    }
    if (sigma_pct > kMaxSigmaPct) {
        return kMaxSigmaPct;
    }
    return sigma_pct;
}

NoiseModel::Perturbation NoiseModel::sample_command(double max_accel, double max_steer) {
    Perturbation p;
    // Each channel is evaluated independently -- a channel whose sigma is
    // exactly 0 returns an exact zero WITHOUT drawing from the RNG at all
    // (bitwise-untouched channel), regardless of what the OTHER channel is
    // doing. Draw order (accel then steer) matches the pre-split
    // single-sigma implementation when both channels are active.
    if (accel_sigma_pct_ > 0.0) {
        std::normal_distribution<double> dist_accel(0.0, accel_sigma_pct_ * std::fabs(max_accel));
        p.d_accel = dist_accel(rng_);
    }
    if (steer_sigma_pct_ > 0.0) {
        std::normal_distribution<double> dist_steer(0.0, steer_sigma_pct_ * std::fabs(max_steer));
        p.d_steer = dist_steer(rng_);
    }
    return p;
}

ClampResult apply_command_noise(double last_cmd_accel, double last_cmd_steering,
                                 const NoiseModel::Perturbation& pert, double max_accel,
                                 double max_steer) {
    // Noise FIRST, clamp AFTER -- reuses clamp_command() so the actuator
    // limit is enforced on the PERTURBED value, never on the raw sum.
    return clamp_command(last_cmd_accel + pert.d_accel, last_cmd_steering + pert.d_steer,
                          max_accel, max_steer);
}

SimConfigParseResult parse_sim_config_payload(const std::string& payload,
                                               double current_accel_sigma_pct,
                                               double current_steer_sigma_pct) {
    SimConfigParseResult out;
    // Safe defaults: retain both fields unless their OWN key parses.
    out.accel_sigma_pct = current_accel_sigma_pct;
    out.steer_sigma_pct = current_steer_sigma_pct;

    if (payload.empty() || payload.front() != '{') {
        return out;
    }

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(payload);
    } catch (...) {
        // Structurally malformed JSON -- never throw, retain both fields.
        return out;
    }

    if (j.contains("noise_sigma_pct")) {
        try {
            const double raw = j.at("noise_sigma_pct").get<double>();
            const double clamped = NoiseModel::clamp_sigma_pct(raw);
            out.accel_sigma_pct = clamped;
            out.accel_was_clamped = !(clamped == raw);
            out.accel_ok = true;
        } catch (...) {
            // Wrong-typed "noise_sigma_pct" -- leave accel_ok=false,
            // accel_sigma_pct retained; does not affect the other field.
            out.accel_bad = true;
        }
    }

    if (j.contains("steer_noise_sigma_pct")) {
        try {
            const double raw = j.at("steer_noise_sigma_pct").get<double>();
            const double clamped = NoiseModel::clamp_sigma_pct(raw);
            out.steer_sigma_pct = clamped;
            out.steer_was_clamped = !(clamped == raw);
            out.steer_ok = true;
        } catch (...) {
            // Wrong-typed "steer_noise_sigma_pct" -- leave steer_ok=false,
            // steer_sigma_pct retained; does not affect the other field.
            out.steer_bad = true;
        }
    }

    out.ok = out.accel_ok || out.steer_ok;
    return out;
}

} // namespace mpc
