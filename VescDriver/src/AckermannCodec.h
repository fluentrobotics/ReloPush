// AckermannCodec.h
//
// Decodes the ackermann command wire payload (the JSON blob mpc_controller
// sends: {"speed": base64(ascii-decimal), "steering": ..., "accel": ...}).
// This EXACTLY mirrors mpc::decode_ackermann_payload (MPC/src/SimCore.cpp)
// so a real VESC-driving Jetson decodes the identical wire format
// mpc_robot_sim already round-trips in simulation. Kept self-contained
// (own AckermannCommand type, own nlohmann/base64 vendored copies) per
// VescDriver's "no includes outside VescDriver/" rule.

#ifndef VESC_DRIVER_ACKERMANN_CODEC_H_
#define VESC_DRIVER_ACKERMANN_CODEC_H_

#include <string>

namespace vesc {

struct AckermannCommand {
    double speed = 0.0;
    double steering = 0.0;
    double accel = 0.0;
    bool ok = false;
};

// Never throws -- any parse failure (empty payload, non-'{' payload,
// invalid JSON, missing "speed"/"steering"/"accel" key, non-base64 value,
// non-numeric or trailing-garbage decoded string) yields
// AckermannCommand{ok=false} with all other fields at their default (0.0).
AckermannCommand decode_ackermann_payload(const std::string& payload);

}  // namespace vesc

#endif  // VESC_DRIVER_ACKERMANN_CODEC_H_
