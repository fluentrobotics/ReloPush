#include "AckermannCodec.h"

#include <stdexcept>
#include <string>

// Vendored copies (VescDriver/third_party/) -- deliberately NOT
// <nlohmann/json.hpp> / <ReloPush/base64.h> from outside VescDriver/, per
// this folder's "no includes of anything outside VescDriver/" rule. Quoted
// relative paths so resolution never depends on -I search order.
#include "../third_party/nlohmann/json.hpp"
#include "../third_party/base64.h"

namespace vesc {

namespace {

// Decodes one base64(ASCII-decimal) field exactly as mpc_controller's
// encodeAscii() produces it (base64_encode(ostringstream <<
// setprecision(16) << value)). Throws (caught by the caller) on any
// malformed input -- non-base64 characters, empty string, or a decoded
// string that isn't a valid decimal number (including trailing garbage).
double decode_ascii_field(const std::string& b64_value) {
    const std::string decoded = base64_decode(b64_value);
    if (decoded.empty()) {
        throw std::runtime_error("decode_ascii_field: empty decoded value");
    }
    size_t consumed = 0;
    const double value = std::stod(decoded, &consumed);
    if (consumed != decoded.size()) {
        throw std::runtime_error("decode_ascii_field: trailing characters after number");
    }
    return value;
}

}  // namespace

AckermannCommand decode_ackermann_payload(const std::string& payload) {
    AckermannCommand out;  // ok=false, all-zero -- the safe "didn't parse" default.

    if (payload.empty()) {
        return out;
    }

    try {
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
        return AckermannCommand{};
    }

    return out;
}

}  // namespace vesc
