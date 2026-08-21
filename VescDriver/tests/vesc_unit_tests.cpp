// FAST, pure in-process unit tests for VescDriver's codec/logic layers
// (VescProtocol/AckermannCodec/DriverCore/TeleopCore/SteeringCalib/
// McconfPatcher/PortDiscovery): plain bool test_xxx() functions
// registered in main(), no gtest dependency. Standalone manually-run
// binary -- NOT add_test()'d (see VescDriver/CMakeLists.txt's own
// comment). Split out of the former monolithic vesc_driver_tests.cpp so
// this half stays seconds-fast: nothing here forks/execs/spawns a
// process or touches fake_vesc -- see vesc_integration_tests.cpp for the
// pty/subprocess/ZMQ end-to-end tests.

#include "../src/AckermannCodec.h"
#include "../src/DriverCore.h"
#include "../src/McconfPatcher.h"
#include "../src/PortDiscovery.h"
#include "../src/SerialPort.h"
#include "../src/SpeedGovernor.h"
#include "../src/SteeringAngleMap.h"
#include "../src/SteeringCalib.h"
#include "../src/TeleopCore.h"
#include "../src/VelocityMap.h"
#include "../src/VescProtocol.h"
#include "FakeVescModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <zmq.h>

// Vendored base64 (used only to build a couple of test fixtures at
// runtime for the AckermannCodec malformation-class tests below; the
// primary valid-decode test uses a hand-computed literal instead -- see
// test_ackermann_valid_decode()) -- and, further down, to build wire-exact
// ackermann payloads for part 3's pty+ZMQ integration test.
#include "../third_party/base64.h"
// Vendored nlohmann::json -- used by part 3's DriverConfig/calibration
// loader tests and the pty+ZMQ integration test (building/parsing ZMQ
// JSON payloads and scanning fake_vesc's --log JSONL output).
#include "../third_party/nlohmann/json.hpp"

#include "test_support.h"

namespace {

// ---------------------------------------------------------------------
// (a) CRC16/XMODEM
// ---------------------------------------------------------------------

// Second, independently-written bitwise CRC-16/XMODEM implementation
// (poly 0x1021, init 0x0000), deliberately structured differently from
// VescProtocol.cpp's table-driven crc16() (this one XORs the incoming
// byte into the top 8 bits of the running CRC and shifts one bit at a
// time, rather than indexing a precomputed table) -- used to cross-check
// the production implementation rather than trust it blindly.
uint16_t reference_crc16_bitwise(const std::vector<uint8_t>& data) {
    uint16_t crc = 0x0000;
    for (uint8_t byte : data) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(byte) << 8));
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000u) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

std::vector<uint8_t> str_bytes(const std::string& s) { return std::vector<uint8_t>(s.begin(), s.end()); }

bool test_crc16_reference_vectors() {
    bool ok = true;

    // Vector 1: empty input. Hand-computed: with no bytes processed the
    // CRC never leaves its init value.
    ok &= check_true(vesc::crc16(std::vector<uint8_t>{}) == 0x0000, "crc16({}) == 0x0000");
    ok &= check_true(reference_crc16_bitwise({}) == 0x0000, "reference_crc16_bitwise({}) == 0x0000");

    // Vector 2: single byte 0x01. Hand-computed by tracing the bit
    // algorithm: crc starts 0x0000, XOR-ing 0x01 into the top byte gives
    // 0x0100; the single set bit walks up through 8 left-shifts (no
    // reduction fires until the 8th shift, when the bit reaches the MSB)
    // and on that final shift crc = (0x8000<<1 mod 2^16) ^ 0x1021 =
    // 0x0000 ^ 0x1021 = 0x1021 -- i.e. crc16({0x01}) == the generator
    // polynomial itself.
    ok &= check_true(vesc::crc16(std::vector<uint8_t>{0x01}) == 0x1021, "crc16({0x01}) == 0x1021");
    ok &= check_true(reference_crc16_bitwise({0x01}) == 0x1021, "reference_crc16_bitwise({0x01}) == 0x1021");

    // Vector 3: ASCII "123456789" -- the standard published CRC-16/XMODEM
    // check value (poly 0x1021, init 0x0000, no reflection, no xorout).
    const std::vector<uint8_t> check_str = str_bytes("123456789");
    ok &= check_true(vesc::crc16(check_str) == 0x31C3, "crc16(\"123456789\") == 0x31C3");
    ok &= check_true(reference_crc16_bitwise(check_str) == 0x31C3,
                      "reference_crc16_bitwise(\"123456789\") == 0x31C3");

    // Extra coverage vectors: no independently-known literal, but cross-
    // checked against the second (bitwise) implementation.
    const std::vector<std::vector<uint8_t>> extra_vectors = {
        str_bytes("A"),
        str_bytes("abc"),
        str_bytes("The quick brown fox jumps over the lazy dog"),
        {0xFF, 0x00, 0xAA, 0x55},
    };
    for (const auto& v : extra_vectors) {
        const uint16_t table_driven = vesc::crc16(v);
        const uint16_t bitwise = reference_crc16_bitwise(v);
        ok &= check_true(table_driven == bitwise, "crc16() matches reference_crc16_bitwise() for vector");
    }

    return ok;
}

// ---------------------------------------------------------------------
// (b) Frame encode -> streaming decode round trip
// ---------------------------------------------------------------------

bool test_frame_roundtrip_basic() {
    bool ok = true;
    const std::vector<uint8_t> payload = vesc::build_get_values();  // {0x04}
    const std::vector<uint8_t> frame = vesc::encode_frame(payload);

    // Short-form framing sanity: [0x02, len, payload..., crc_hi, crc_lo, 0x03]
    ok &= check_true(frame.size() == payload.size() + 5, "short frame size == payload+5");
    ok &= check_true(frame.front() == 0x02, "short frame starts with 0x02");
    ok &= check_true(frame.back() == 0x03, "short frame ends with 0x03");
    ok &= check_true(frame[1] == payload.size(), "short frame length byte == payload size");

    vesc::FrameDecoder decoder;
    decoder.feed(frame);
    ok &= check_true(decoder.pending_count() == 1, "one payload queued after feeding one full frame");

    std::vector<uint8_t> out;
    ok &= check_true(decoder.pop_payload(&out), "pop_payload succeeds");
    ok &= bytes_eq(out, payload, "round-tripped payload matches original");
    ok &= check_true(!decoder.pop_payload(&out), "no second payload queued");

    return ok;
}

bool test_frame_roundtrip_garbage_prefix() {
    bool ok = true;
    const std::vector<uint8_t> payload = vesc::build_set_rpm(1500);
    const std::vector<uint8_t> frame = vesc::encode_frame(payload);

    std::vector<uint8_t> stream = {0xAA, 0x55, 0x02, 0x00, 0x01, 0x02, 0x03};  // plausible-looking junk
    stream.insert(stream.end(), frame.begin(), frame.end());

    vesc::FrameDecoder decoder;
    decoder.feed(stream);
    ok &= check_true(decoder.pending_count() == 1, "garbage prefix resynced to exactly one valid frame");

    std::vector<uint8_t> out;
    ok &= check_true(decoder.pop_payload(&out), "pop_payload succeeds after garbage prefix");
    ok &= bytes_eq(out, payload, "payload after garbage prefix matches original");

    return ok;
}

bool test_frame_roundtrip_split_across_feeds() {
    bool ok = true;
    const std::vector<uint8_t> payload = vesc::build_set_duty(0.05);
    const std::vector<uint8_t> frame = vesc::encode_frame(payload);

    vesc::FrameDecoder decoder;
    // Feed one byte at a time; no payload should be ready until the very
    // last byte lands.
    for (size_t i = 0; i + 1 < frame.size(); ++i) {
        decoder.feed(&frame[i], 1);
        ok &= check_true(decoder.pending_count() == 0,
                          "no payload ready before the frame is fully fed (byte " + std::to_string(i) + ")");
    }
    decoder.feed(&frame.back(), 1);
    ok &= check_true(decoder.pending_count() == 1, "payload ready exactly when the last byte lands");

    std::vector<uint8_t> out;
    ok &= check_true(decoder.pop_payload(&out), "pop_payload succeeds");
    ok &= bytes_eq(out, payload, "split-feed payload matches original");

    return ok;
}

bool test_frame_roundtrip_corrupted_crc_rejected() {
    bool ok = true;
    const std::vector<uint8_t> good_payload = vesc::build_alive();
    std::vector<uint8_t> bad_frame = vesc::encode_frame(good_payload);
    // Flip a bit in the CRC high byte -- index 2 for a 1-byte payload
    // ([0x02, len=1, payload[0], crc_hi, crc_lo, 0x03] -> crc_hi at idx 3).
    bad_frame[3] ^= 0xFF;

    const std::vector<uint8_t> good_frame = vesc::encode_frame(vesc::build_fw_version());

    vesc::FrameDecoder decoder;
    decoder.feed(bad_frame);
    decoder.feed(good_frame);

    // Only the valid frame should have been emitted; the corrupted one is
    // dropped byte-by-byte during resync.
    ok &= check_true(decoder.pending_count() == 1, "corrupted-CRC frame dropped, only the good one queued");
    std::vector<uint8_t> out;
    ok &= check_true(decoder.pop_payload(&out), "pop_payload succeeds for the good frame");
    ok &= bytes_eq(out, vesc::build_fw_version(), "surviving payload is the good (post-corruption) one");
    ok &= check_true(!decoder.pop_payload(&out), "nothing else queued");

    return ok;
}

bool test_frame_roundtrip_long_form() {
    bool ok = true;
    // 300-byte payload (> 255) forces the long framing form.
    std::vector<uint8_t> payload;
    payload.reserve(300);
    for (int i = 0; i < 300; ++i) {
        payload.push_back(static_cast<uint8_t>(i % 256));
    }

    const std::vector<uint8_t> frame = vesc::encode_frame(payload);
    ok &= check_true(frame.front() == 0x03, "long frame starts with 0x03");
    ok &= check_true(frame[1] == 0x01 && frame[2] == 0x2C, "long frame length bytes == 300 big-endian (0x01,0x2C)");
    ok &= check_true(frame.back() == 0x03, "long frame ends with 0x03");
    ok &= check_true(frame.size() == payload.size() + 6, "long frame size == payload+6");

    vesc::FrameDecoder decoder;
    decoder.feed(frame);
    ok &= check_true(decoder.pending_count() == 1, "one payload queued after feeding one full long frame");

    std::vector<uint8_t> out;
    ok &= check_true(decoder.pop_payload(&out), "pop_payload succeeds for long frame");
    ok &= bytes_eq(out, payload, "long-form round-tripped payload matches original 300-byte payload");

    return ok;
}

// ---------------------------------------------------------------------
// (c) Command builders -- hand-computed exact byte sequences
// ---------------------------------------------------------------------

bool test_command_builders() {
    bool ok = true;

    ok &= bytes_eq(vesc::build_fw_version(), {0x00}, "build_fw_version()");
    ok &= bytes_eq(vesc::build_get_values(), {0x04}, "build_get_values()");
    ok &= bytes_eq(vesc::build_alive(), {0x1E}, "build_alive()");

    // SET_RPM(1500): id=8, int32 big-endian of 1500 = 0x000005DC.
    ok &= bytes_eq(vesc::build_set_rpm(1500), {0x08, 0x00, 0x00, 0x05, 0xDC}, "build_set_rpm(1500)");

    // SET_DUTY(0.05): id=5, int32 big-endian of round(0.05*100000)=5000=0x00001388.
    ok &= bytes_eq(vesc::build_set_duty(0.05), {0x05, 0x00, 0x00, 0x13, 0x88}, "build_set_duty(0.05)");

    // SET_SERVO_POS(0.5): id=12(0x0C), uint16 big-endian of round(0.5*1000)=500=0x01F4.
    ok &= bytes_eq(vesc::build_set_servo_pos(0.5), {0x0C, 0x01, 0xF4}, "build_set_servo_pos(0.5)");

    // Extra coverage: SET_CURRENT(2.5) -> id=6, int32(2500)=0x000009C4.
    ok &= bytes_eq(vesc::build_set_current(2.5), {0x06, 0x00, 0x00, 0x09, 0xC4}, "build_set_current(2.5)");

    // SET_CURRENT_BRAKE(1.234) -> id=7, int32(1234)=0x000004D2.
    ok &= bytes_eq(vesc::build_set_current_brake(1.234), {0x07, 0x00, 0x00, 0x04, 0xD2},
                   "build_set_current_brake(1.234)");

    // SET_RPM with a negative erpm: -1000 as int32 two's complement =
    // 0xFFFFFC18.
    ok &= bytes_eq(vesc::build_set_rpm(-1000), {0x08, 0xFF, 0xFF, 0xFC, 0x18}, "build_set_rpm(-1000)");

    // SET_SERVO_POS clamps out-of-range input to [0,1] before scaling.
    ok &= bytes_eq(vesc::build_set_servo_pos(-0.3), {0x0C, 0x00, 0x00}, "build_set_servo_pos(-0.3) clamps to 0");
    ok &= bytes_eq(vesc::build_set_servo_pos(1.7), {0x0C, 0x03, 0xE8}, "build_set_servo_pos(1.7) clamps to 1 (1000=0x03E8)");

    return ok;
}

// ---------------------------------------------------------------------
// (c2) servo command id resolution (FW2.x hazard fix) + id-aware encoding
// ---------------------------------------------------------------------

bool test_servo_cmd_id_resolution() {
    bool ok = true;

    // resolve_servo_cmd_id(): fw_major<=2 (legacy, no COMM_SET_HANDBRAKE)
    // -> 11; fw_major>=3 (assumed modern) -> 12. See VescProtocol.h's
    // CAVEAT comment above resolve_servo_cmd_id() for confidence level.
    ok &= check_true(vesc::resolve_servo_cmd_id(2) == 11, "resolve_servo_cmd_id(2) == 11");
    ok &= check_true(vesc::resolve_servo_cmd_id(1) == 11, "resolve_servo_cmd_id(1) == 11 (edge case, <=2)");
    ok &= check_true(vesc::resolve_servo_cmd_id(3) == 12, "resolve_servo_cmd_id(3) == 12");
    ok &= check_true(vesc::resolve_servo_cmd_id(5) == 12, "resolve_servo_cmd_id(5) == 12");
    ok &= check_true(vesc::resolve_servo_cmd_id(6) == 12, "resolve_servo_cmd_id(6) == 12");

    // build_set_servo_pos(0.5, 11): id=11(0x0B), uint16 big-endian of
    // round(0.5*1000)=500=0x01F4 -- differs from the fixed-id overload's
    // {0x0C, 0x01, 0xF4} (see test_command_builders) ONLY in the first byte.
    ok &= bytes_eq(vesc::build_set_servo_pos(0.5, 11), {0x0B, 0x01, 0xF4}, "build_set_servo_pos(0.5, 11)");

    // build_set_servo_pos(0.5, 12): same bytes as the existing fixed-id
    // overload -- confirms the two overloads agree when given the same id.
    ok &= bytes_eq(vesc::build_set_servo_pos(0.5, 12), {0x0C, 0x01, 0xF4}, "build_set_servo_pos(0.5, 12)");

    return ok;
}

// ---------------------------------------------------------------------
// (d) GET_VALUES parse from a hand-built canned byte vector
// ---------------------------------------------------------------------

// Hand-built GET_VALUES reply payload. Field values chosen so every raw
// scaled integer is easy to hand-verify (see inline comments): total 54
// bytes (id + 53 field bytes), matching VescProtocol.h's documented
// layout table through `fault`.
std::vector<uint8_t> canned_get_values_payload() {
    return {
        0x04,              // [0]      command id (GET_VALUES)
        0x00, 0xFD,        // [1..2]   temp_fet raw 253 -> 25.3
        0x01, 0x2C,        // [3..4]   temp_motor raw 300 -> 30.0
        0x00, 0x00, 0x04, 0xD2,  // [5..8]   avg_motor_current raw 1234 -> 12.34
        0x00, 0x00, 0x02, 0x37,  // [9..12]  avg_input_current raw 567 -> 5.67
        0x00, 0x00, 0x00, 0x6F,  // [13..16] avg_id raw 111 (consumed, not stored)
        0x00, 0x00, 0x00, 0xDE,  // [17..20] avg_iq raw 222 (consumed, not stored)
        0x00, 0x7B,        // [21..22] duty_now raw 123 -> 0.123
        0x00, 0x00, 0xAF, 0xC8,  // [23..26] rpm raw 45000 (erpm, no scaling)
        0x00, 0xF5,        // [27..28] v_in raw 245 -> 24.5
        0x00, 0x00, 0x3A, 0x98,  // [29..32] amp_hours raw 15000 (consumed, not stored)
        0x00, 0x00, 0x13, 0x88,  // [33..36] amp_hours_charged raw 5000 (consumed, not stored)
        0x00, 0x01, 0x86, 0xA0,  // [37..40] watt_hours raw 100000 (consumed, not stored)
        0x00, 0x00, 0x4E, 0x20,  // [41..44] watt_hours_charged raw 20000 (consumed, not stored)
        0x00, 0x01, 0xE2, 0x40,  // [45..48] tachometer raw 123456
        0x00, 0x09, 0xFB, 0xF1,  // [49..52] tachometer_abs raw 654321
        0x00,              // [53]     fault = 0
    };
}

bool test_get_values_parse() {
    bool ok = true;

    const std::vector<uint8_t> payload = canned_get_values_payload();
    ok &= check_true(payload.size() == 54, "canned GET_VALUES payload is 54 bytes");

    const vesc::VescValues v = vesc::parse_get_values(payload);
    ok &= check_true(v.ok, "parse_get_values() ok on well-formed canned payload");
    ok &= check_true(near_eq(v.temp_fet, 25.3), "temp_fet == 25.3");
    ok &= check_true(near_eq(v.temp_motor, 30.0), "temp_motor == 30.0");
    ok &= check_true(near_eq(v.current_motor, 12.34), "current_motor == 12.34");
    ok &= check_true(near_eq(v.current_in, 5.67), "current_in == 5.67");
    ok &= check_true(near_eq(v.duty, 0.123), "duty == 0.123");
    ok &= check_true(v.erpm == 45000, "erpm == 45000");
    ok &= check_true(near_eq(v.v_in, 24.5), "v_in == 24.5");
    ok &= check_true(v.tachometer == 123456, "tachometer == 123456");
    ok &= check_true(v.tachometer_abs == 654321, "tachometer_abs == 654321");
    ok &= check_true(v.fault == 0, "fault == 0");

    // Tolerate-longer: trailing bytes (as some firmware versions append)
    // must not change the parsed result or cause ok=false.
    std::vector<uint8_t> with_trailer = payload;
    with_trailer.insert(with_trailer.end(), {0xDE, 0xAD, 0xBE, 0xEF, 0x01});
    const vesc::VescValues v_trailer = vesc::parse_get_values(with_trailer);
    ok &= check_true(v_trailer.ok, "parse_get_values() tolerates trailing bytes");
    ok &= check_true(near_eq(v_trailer.temp_fet, 25.3) && v_trailer.tachometer_abs == 654321,
                      "trailing bytes do not change parsed fields");

    // Stop-cleanly: a short payload (cut off mid-field) must yield
    // ok=false rather than reading out of bounds.
    const std::vector<uint8_t> truncated(payload.begin(), payload.begin() + 10);
    const vesc::VescValues v_short = vesc::parse_get_values(truncated);
    ok &= check_true(!v_short.ok, "parse_get_values() reports ok=false on truncated payload");

    // Empty payload.
    const vesc::VescValues v_empty = vesc::parse_get_values({});
    ok &= check_true(!v_empty.ok, "parse_get_values() reports ok=false on empty payload");

    // Wrong command id.
    std::vector<uint8_t> wrong_id = payload;
    wrong_id[0] = 0x05;
    const vesc::VescValues v_wrong = vesc::parse_get_values(wrong_id);
    ok &= check_true(!v_wrong.ok, "parse_get_values() reports ok=false on wrong command id");

    return ok;
}

bool test_fw_version_parse() {
    bool ok = true;

    const std::vector<uint8_t> good = {0x00, 0x03, 0x62};  // id=0, major=3, minor=98
    const vesc::FwVersionReply reply = vesc::parse_fw_version(good);
    ok &= check_true(reply.ok, "parse_fw_version() ok on well-formed payload");
    ok &= check_true(reply.major == 3, "fw major == 3");
    ok &= check_true(reply.minor == 98, "fw minor == 98");

    // Tolerate trailing bytes.
    const std::vector<uint8_t> with_trailer = {0x00, 0x06, 0x00, 0xAA, 0xBB, 0xCC};
    const vesc::FwVersionReply reply2 = vesc::parse_fw_version(with_trailer);
    ok &= check_true(reply2.ok && reply2.major == 6 && reply2.minor == 0,
                      "parse_fw_version() tolerates trailing bytes");

    // Too short.
    const vesc::FwVersionReply reply3 = vesc::parse_fw_version({0x00, 0x03});
    ok &= check_true(!reply3.ok, "parse_fw_version() reports ok=false when too short");

    // Wrong id.
    const vesc::FwVersionReply reply4 = vesc::parse_fw_version({0x04, 0x03, 0x62});
    ok &= check_true(!reply4.ok, "parse_fw_version() reports ok=false on wrong command id");

    return ok;
}

// ---------------------------------------------------------------------
// (e) AckermannCodec
// ---------------------------------------------------------------------

bool test_ackermann_valid_decode() {
    bool ok = true;

    // Hand-computed: ASCII "0.25" is bytes {0x30,0x2E,0x32,0x35}; base64
    // of that (RFC 4648 standard alphabet, 3-byte group then 1 trailing
    // byte with "==" padding) is "MC4yNQ==". Traced by hand:
    //   group 1 (0x30,0x2E,0x32) -> 6-bit groups 001100/000010/111000/110010
    //                             -> 12,2,56,50 -> 'M','C','4','y'
    //   group 2 (0x35 alone)     -> (0x35&0xFC)>>2=13->'N',
    //                                (0x35&0x03)<<4=16->'Q', then "=="
    //   => "MC4y" + "NQ==" = "MC4yNQ=="
    const std::string payload =
        "{\"speed\":\"MC4yNQ==\",\"steering\":\"MC4yNQ==\",\"accel\":\"MC4yNQ==\"}";

    const vesc::AckermannCommand cmd = vesc::decode_ackermann_payload(payload);
    ok &= check_true(cmd.ok, "decode_ackermann_payload() ok on hand-computed valid payload");
    ok &= check_true(near_eq(cmd.speed, 0.25), "speed == 0.25");
    ok &= check_true(near_eq(cmd.steering, 0.25), "steering == 0.25");
    ok &= check_true(near_eq(cmd.accel, 0.25), "accel == 0.25");

    // Second valid-decode case with distinct per-field values, built at
    // runtime via the vendored base64_encode() (round-trip self-
    // consistency check, complementing the hand-verified literal above).
    auto b64_ascii = [](double x) {
        std::ostringstream oss;
        oss.precision(16);
        oss << x;
        return base64_encode(oss.str());
    };
    std::ostringstream json;
    json << "{\"speed\":\"" << b64_ascii(1.5) << "\",\"steering\":\"" << b64_ascii(-0.2)
         << "\",\"accel\":\"" << b64_ascii(0.75) << "\"}";
    const vesc::AckermannCommand cmd2 = vesc::decode_ackermann_payload(json.str());
    ok &= check_true(cmd2.ok, "decode_ackermann_payload() ok on runtime-encoded valid payload");
    ok &= check_true(near_eq(cmd2.speed, 1.5), "speed == 1.5");
    ok &= check_true(near_eq(cmd2.steering, -0.2), "steering == -0.2");
    ok &= check_true(near_eq(cmd2.accel, 0.75), "accel == 0.75");

    return ok;
}

bool test_ackermann_malformed_variants() {
    bool ok = true;

    auto expect_fail = [&](const std::string& payload, const std::string& what) {
        const vesc::AckermannCommand cmd = vesc::decode_ackermann_payload(payload);
        ok &= check_true(!cmd.ok, what);
        ok &= check_true(cmd.speed == 0.0 && cmd.steering == 0.0 && cmd.accel == 0.0,
                          what + " -- fields left at zero");
    };

    // Empty payload.
    expect_fail("", "empty payload -> ok=false");

    // Non-JSON payload (doesn't start with '{').
    expect_fail("not json at all", "non-JSON (no leading '{') payload -> ok=false");

    // Starts with '{' but is not valid JSON.
    expect_fail("{this is not valid json", "malformed JSON syntax -> ok=false");

    // Missing key ("accel" absent).
    expect_fail("{\"speed\":\"MC4yNQ==\",\"steering\":\"MC4yNQ==\"}", "missing \"accel\" key -> ok=false");

    // Non-base64 value.
    expect_fail("{\"speed\":\"!!!not-base64!!!\",\"steering\":\"MC4yNQ==\",\"accel\":\"MC4yNQ==\"}",
                "non-base64 \"speed\" value -> ok=false");

    // Trailing garbage in the decoded ASCII (decodes to "0.25xyz", not a
    // clean number) -- built at runtime via base64_encode() since the
    // point under test is the trailing-garbage rejection, not the
    // base64 encoding itself.
    const std::string trailing_garbage_b64 = base64_encode(std::string("0.25xyz"));
    expect_fail("{\"speed\":\"" + trailing_garbage_b64 + "\",\"steering\":\"MC4yNQ==\",\"accel\":\"MC4yNQ==\"}",
                "trailing garbage after decoded number -> ok=false");

    // Wrong JSON type for a required key (number instead of string).
    expect_fail("{\"speed\":123,\"steering\":\"MC4yNQ==\",\"accel\":\"MC4yNQ==\"}",
                "non-string \"speed\" value -> ok=false");

    return ok;
}

// ---------------------------------------------------------------------
// (f) PortDiscovery::rank_candidates() -- pure ranking logic, no
// filesystem/hardware access.
// ---------------------------------------------------------------------

bool test_rank_candidates_by_id_priority() {
    bool ok = true;

    const std::vector<vesc::ByIdEntry> by_id = {
        {"usb-Generic_USB_Serial-if00", "/dev/ttyUSB3"},
        {"usb-ChibiOS_ChibiOS_RT_VESC_12345-if00", "/dev/ttyACM2"},
    };
    const std::vector<std::string> fallbacks = {"/dev/ttyACM0", "/dev/ttyACM1"};

    const std::vector<vesc::PortCandidate> ranked = vesc::rank_candidates(by_id, fallbacks);
    ok &= check_true(ranked.size() == 4, "rank_candidates() returns one candidate per unique path");
    ok &= check_true(!ranked.empty() && ranked.front().path == "/dev/ttyACM2",
                      "ChibiOS/VESC-keyword by-id entry ranks first (got '" +
                          (ranked.empty() ? std::string("<empty>") : ranked.front().path) + "')");

    auto index_of = [&](const std::string& path) -> int {
        for (size_t i = 0; i < ranked.size(); ++i) {
            if (ranked[i].path == path) return static_cast<int>(i);
        }
        return -1;
    };
    const int generic_by_id_idx = index_of("/dev/ttyUSB3");
    const int fallback0_idx = index_of("/dev/ttyACM0");
    const int fallback1_idx = index_of("/dev/ttyACM1");
    ok &= check_true(generic_by_id_idx >= 0 && fallback0_idx >= 0 && generic_by_id_idx < fallback0_idx,
                      "a by-id entry with no recognized keyword still ranks above the ttyACM fallback tier");
    ok &= check_true(fallback0_idx >= 0 && fallback1_idx >= 0 && fallback0_idx < fallback1_idx,
                      "ttyACM fallback candidates keep ascending index order (ttyACM0 before ttyACM1)");

    return ok;
}

bool test_rank_candidates_stmicro_and_dedup() {
    bool ok = true;

    // This by-id entry resolves to a path that ALSO appears in the ttyACM
    // fallback list -- must be de-duplicated (probed once, at its
    // by-id-derived rank).
    const std::vector<vesc::ByIdEntry> by_id = {
        {"usb-STMicroelectronics_Virtual_COM_Port-if00", "/dev/ttyACM0"},
    };
    const std::vector<std::string> fallbacks = {"/dev/ttyACM0", "/dev/ttyACM1"};

    const std::vector<vesc::PortCandidate> ranked = vesc::rank_candidates(by_id, fallbacks);
    int count_acm0 = 0;
    for (const auto& c : ranked) {
        if (c.path == "/dev/ttyACM0") ++count_acm0;
    }
    ok &= check_true(count_acm0 == 1, "a path reachable via both by-id and ttyACM fallback appears exactly once");
    ok &= check_true(!ranked.empty() && ranked.front().path == "/dev/ttyACM0",
                      "STMicroelectronics-keyword by-id entry still ranks first despite dedup");
    ok &= check_true(ranked.size() == 2, "de-duplication drops exactly the one overlapping candidate");

    return ok;
}

bool test_rank_candidates_ttyacm_only() {
    bool ok = true;
    const std::vector<vesc::PortCandidate> ranked = vesc::rank_candidates({}, {"/dev/ttyACM0"});
    ok &= check_true(ranked.size() == 1 && ranked[0].path == "/dev/ttyACM0",
                      "with no by-id entries at all, the ttyACM fallback list alone is returned");
    const std::vector<vesc::PortCandidate> empty_ranked = vesc::rank_candidates({}, {});
    ok &= check_true(empty_ranked.empty(), "no candidates at all when both input lists are empty");
    return ok;
}

// ---------------------------------------------------------------------
// (f4) float32_auto -- round trip + independent bitwise reference +
// hand-computed literal cross-checks.
// ---------------------------------------------------------------------

// Second, INDEPENDENTLY-WRITTEN implementation of the same float32_auto
// algorithm (vesc-project bldc firmware's buffer.c
// buffer_append_float32_auto/buffer_get_float32_auto), deliberately
// structured differently from VescProtocol.cpp's version (ternary
// expressions instead of if-statements, different variable names/order,
// ints combined via +. instead of |= where equivalent) -- used to cross-
// check the production implementation bit-for-bit rather than trust it
// blindly. Implements the exact same specified math, since this is a
// custom, non-IEEE bit packing with no other "correct" alternative
// formula to independently derive from.
uint32_t reference_float32_auto_encode(float number) {
    int32_t exponent = 0;
    const float mantissa = frexpf(number, &exponent);
    const float mantissa_abs = fabsf(mantissa);
    const uint32_t mantissa_i =
        (mantissa_abs >= 0.5f) ? static_cast<uint32_t>((mantissa_abs - 0.5f) * 16777216.0f) : 0u;
    const int32_t biased_exponent = (mantissa_abs >= 0.5f) ? (exponent + 126) : exponent;
    const uint32_t sign_bit = (mantissa < 0.0f) ? (1u << 31) : 0u;
    return sign_bit + ((static_cast<uint32_t>(biased_exponent) << 23) & 0xFF800000u) + (mantissa_i & 0x7FFFFFu);
}

float reference_float32_auto_decode(uint32_t encoded) {
    const int32_t raw_exponent = static_cast<int32_t>((encoded >> 23) & 0xFFu);
    const uint32_t mantissa_i = encoded & 0x7FFFFFu;
    const bool is_negative = (encoded >> 31) & 0x1u;
    if (raw_exponent == 0 && mantissa_i == 0) {
        return 0.0f;
    }
    const float mantissa = static_cast<float>(mantissa_i) / 16777216.0f + 0.5f;
    const float result = ldexpf(is_negative ? -mantissa : mantissa, raw_exponent - 126);
    return result;
}

bool near_eq_rel(double a, double b, double rel_tol) {
    if (a == b) return true;
    const double denom = std::max(std::fabs(a), std::fabs(b));
    if (denom < 1e-30) return std::fabs(a - b) < 1e-30;  // both ~0
    return std::fabs(a - b) / denom <= rel_tol;
}

// Appends a raw big-endian uint16 -- used to hand-build mcconf test blobs
// containing the two u16 fields in the current-limits cluster and the
// 2-byte fixed-point kd_filter field (see McconfPatcher.h); these are
// NOT float32_auto, so vesc::append_be_f32_auto() doesn't apply to them.
void append_be_u16(std::vector<uint8_t>* out, uint16_t value) {
    out->push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(value & 0xFF));
}

bool test_float32_auto_roundtrip_and_reference() {
    bool ok = true;

    // Hand-computed literal 1: encode(60.0f). frexpf(60,&e) -> sig=0.9375,
    // e=6 (60 = 0.9375 * 2^6). sig_abs=0.9375 >= 0.5, so
    // sig_i = (0.9375-0.5)*2*8388608 = 0.4375*2*8388608 = 0.875*8388608
    //       = 7340032 = 0x700000 exactly (0.875 = 7/8 is an exact binary
    //       fraction); e += 126 -> e=132=0x84. res = (0x84<<23)|0x700000.
    // 0x84 = 132 = 128+4 -> 0x84<<23 = 128*0x800000 + 4*0x800000 =
    //   0x40000000 + 0x2000000 = 0x42000000. res = 0x42000000|0x700000 =
    //   0x42700000. sig>0 so no sign bit.
    ok &= check_true(vesc::float32_auto_encode(60.0f) == 0x42700000u,
                      "float32_auto_encode(60.0f) == 0x42700000 (hand-computed)");
    ok &= check_true(reference_float32_auto_encode(60.0f) == 0x42700000u,
                      "reference_float32_auto_encode(60.0f) == 0x42700000 (hand-computed)");

    // Hand-computed literal 2: encode(0.5f). frexpf(0.5,&e) -> sig=0.5,
    // e=0 (0.5 = 0.5 * 2^0). sig_abs=0.5 >= 0.5, so
    // sig_i = (0.5-0.5)*2*8388608 = 0; e += 126 -> e=126=0x7E.
    // 0x7E = 126 = 128-2 -> 0x7E<<23 = 0x40000000 - 0x1000000 =
    //   0x3F000000. res = 0x3F000000|0 = 0x3F000000.
    ok &= check_true(vesc::float32_auto_encode(0.5f) == 0x3F000000u,
                      "float32_auto_encode(0.5f) == 0x3F000000 (hand-computed)");
    ok &= check_true(reference_float32_auto_encode(0.5f) == 0x3F000000u,
                      "reference_float32_auto_encode(0.5f) == 0x3F000000 (hand-computed)");

    // Round trip + bit-for-bit cross-check against the independent
    // reference, across a broad value set.
    const std::vector<float> values = {0.0f,   0.002f, -0.002f, 0.004f, -0.004f, 0.2f,  900.0f,
                                        25.0f,  -25.0f, 60.0f,   1e6f,   1e-30f};
    for (float v : values) {
        const uint32_t prod_enc = vesc::float32_auto_encode(v);
        const uint32_t ref_enc = reference_float32_auto_encode(v);
        ok &= check_true(prod_enc == ref_enc, "float32_auto_encode(" + std::to_string(v) +
                                                   ") matches the independent reference bit-for-bit");

        const float prod_dec = vesc::float32_auto_decode(prod_enc);
        const float ref_dec = reference_float32_auto_decode(ref_enc);
        ok &= check_true(near_eq_rel(prod_dec, ref_dec, 1e-6),
                          "float32_auto_decode() matches the independent reference for " + std::to_string(v));

        if (v == 0.0f) {
            ok &= check_true(prod_dec == 0.0f, "float32_auto round trip of 0.0 == 0.0 exactly");
        } else {
            ok &= check_true(near_eq_rel(prod_dec, v, 1e-6),
                              "float32_auto round trip within 1e-6 relative tolerance for " + std::to_string(v));
        }
    }

    return ok;
}

// ---------------------------------------------------------------------
// (f5) McconfPatcher -- scan_speed_pid()/scan_current_limits()/patch()
// on hand-built synthetic blobs, including near-miss decoys.
// ---------------------------------------------------------------------

bool test_mcconf_patcher_scan_and_patch_known_blob() {
    bool ok = true;

    vesc::SyntheticMcconfLayout layout;
    const std::vector<uint8_t> blob = vesc::build_synthetic_mcconf_blob(&layout);

    const std::vector<vesc::SpeedPidMatch> pid_matches = vesc::scan_speed_pid(blob);
    ok &= check_true(vesc::is_unambiguous(pid_matches), "scan_speed_pid() on the shared synthetic blob is unambiguous");
    if (vesc::is_unambiguous(pid_matches)) {
        const auto& m = pid_matches.front();
        ok &= check_true(m.offset == layout.speed_pid_offset, "speed-PID match offset == the builder's known offset");
        ok &= check_true(near_eq(m.kp, vesc::SyntheticMcconfValues::kKp, 1e-6), "scanned kp matches");
        ok &= check_true(near_eq(m.ki, vesc::SyntheticMcconfValues::kKi, 1e-6), "scanned ki matches");
        ok &= check_true(near_eq(m.kd, vesc::SyntheticMcconfValues::kKd, 1e-6), "scanned kd matches");
        ok &= check_true(near_eq(m.kd_filter, vesc::SyntheticMcconfValues::kKdFilter, 1e-6), "scanned kd_filter matches");
        ok &= check_true(near_eq_rel(m.min_erpm, vesc::SyntheticMcconfValues::kMinErpm, 1e-5), "scanned min_erpm matches");
        ok &= check_true(m.allow_braking == vesc::SyntheticMcconfValues::kAllowBraking, "scanned allow_braking matches");
        ok &= check_true(near_eq_rel(m.ramp_erpms_s, vesc::SyntheticMcconfValues::kRampErpmsS, 1e-5), "scanned ramp matches");
    }

    const std::vector<vesc::CurrentLimitsMatch> current_matches = vesc::scan_current_limits(blob);
    ok &= check_true(vesc::is_unambiguous(current_matches),
                      "scan_current_limits() on the shared synthetic blob is unambiguous");
    if (vesc::is_unambiguous(current_matches)) {
        const auto& m = current_matches.front();
        ok &= check_true(m.offset == layout.current_limits_offset,
                          "current-limits match offset == the builder's known offset");
        ok &= check_true(near_eq_rel(m.current_max, vesc::SyntheticMcconfValues::kCurrentMax, 1e-5), "scanned current_max matches");
        ok &= check_true(near_eq_rel(m.current_min, vesc::SyntheticMcconfValues::kCurrentMin, 1e-5), "scanned current_min matches");
        ok &= check_true(near_eq_rel(m.in_current_max, vesc::SyntheticMcconfValues::kInCurrentMax, 1e-5),
                          "scanned in_current_max matches");
        ok &= check_true(near_eq_rel(m.in_current_min, vesc::SyntheticMcconfValues::kInCurrentMin, 1e-5),
                          "scanned in_current_min matches");
        ok &= check_true(m.unknown_field_a_raw == vesc::SyntheticMcconfValues::kUnknownFieldARaw,
                          "scanned unknown_field_a_raw matches (the u16 field between in_current_min and "
                          "abs_current_max that the old guessed layout was missing)");
        ok &= check_true(m.unknown_field_b_raw == vesc::SyntheticMcconfValues::kUnknownFieldBRaw,
                          "scanned unknown_field_b_raw matches");
        ok &= check_true(near_eq_rel(m.abs_current_max, vesc::SyntheticMcconfValues::kAbsCurrentMax, 1e-5),
                          "scanned abs_current_max matches");
    }

    // patch() changes EXACTLY 4 bytes -- diff before/after byte-by-byte.
    // NOTE: the new value is deliberately chosen to differ from the old
    // one in BOTH exponent and mantissa (an exact power-of-2 multiple
    // like 0.008 -> 0.016 only increments the exponent field by 1, which
    // -- since float32_auto is structurally a re-parameterized IEEE-754
    // layout -- can leave 3 of the 4 bytes bit-identical; that would
    // still be a fully correct patch(), just a bad choice of value for
    // testing "changes exactly 4 bytes"). 0.008 -> 3.5 changes both.
    if (vesc::is_unambiguous(pid_matches)) {
        const size_t ki_offset = pid_matches.front().offset + 4;
        const std::vector<uint8_t> patched = vesc::patch(blob, ki_offset, 3.5f);
        ok &= check_true(patched.size() == blob.size(), "patch() does not change blob size");
        int changed = 0;
        int changed_outside_field = 0;
        for (size_t i = 0; i < blob.size(); ++i) {
            if (blob[i] != patched[i]) {
                ++changed;
                if (i < ki_offset || i >= ki_offset + 4) ++changed_outside_field;
            }
        }
        ok &= check_true(changed == 4, "patch() changes exactly 4 bytes (got " + std::to_string(changed) + ")");
        ok &= check_true(changed_outside_field == 0,
                          "patch() touches nothing outside the targeted 4-byte field (got " +
                              std::to_string(changed_outside_field) + " changes outside)");
        const float new_ki = vesc::read_be_f32_auto(patched, ki_offset);
        ok &= check_true(near_eq_rel(new_ki, 3.5, 1e-5), "patched ki decodes back to the new value");

        // Re-scanning the patched blob still finds exactly the pid
        // cluster at the same offset, with only ki changed.
        const std::vector<vesc::SpeedPidMatch> after = vesc::scan_speed_pid(patched);
        ok &= check_true(vesc::is_unambiguous(after), "re-scan after patch is still unambiguous");
        if (vesc::is_unambiguous(after)) {
            ok &= check_true(near_eq(after.front().kp, vesc::SyntheticMcconfValues::kKp, 1e-6),
                              "kp unaffected by patching ki");
            ok &= check_true(near_eq_rel(after.front().ki, 3.5, 1e-5), "re-scanned ki reflects the patch");
        }
    }

    // patch_kd_filter() changes EXACTLY 2 bytes -- the field's whole
    // point is that it is NOT a 4-byte float32_auto field like its
    // neighbors (see McconfPatcher.h).
    if (vesc::is_unambiguous(pid_matches)) {
        const size_t kd_filter_offset = pid_matches.front().offset + 12;
        const std::vector<uint8_t> patched = vesc::patch_kd_filter(blob, kd_filter_offset, 0.35);
        ok &= check_true(patched.size() == blob.size(), "patch_kd_filter() does not change blob size");
        int changed = 0;
        int changed_outside_field = 0;
        for (size_t i = 0; i < blob.size(); ++i) {
            if (blob[i] != patched[i]) {
                ++changed;
                if (i < kd_filter_offset || i >= kd_filter_offset + 2) ++changed_outside_field;
            }
        }
        ok &= check_true(changed == 2, "patch_kd_filter() changes exactly 2 bytes (got " + std::to_string(changed) + ")");
        ok &= check_true(changed_outside_field == 0,
                          "patch_kd_filter() touches nothing outside the targeted 2-byte field (got " +
                              std::to_string(changed_outside_field) + " changes outside)");

        const std::vector<vesc::SpeedPidMatch> after = vesc::scan_speed_pid(patched);
        ok &= check_true(vesc::is_unambiguous(after), "re-scan after patch_kd_filter() is still unambiguous");
        if (vesc::is_unambiguous(after)) {
            ok &= check_true(near_eq(after.front().kd_filter, 0.35, 1e-4),
                              "re-scanned kd_filter reflects the patch (got " +
                                  std::to_string(after.front().kd_filter) + ")");
            ok &= check_true(near_eq(after.front().kp, vesc::SyntheticMcconfValues::kKp, 1e-6),
                              "kp unaffected by patching kd_filter");
        }
    }

    return ok;
}

bool test_mcconf_patcher_ambiguous_cases() {
    bool ok = true;

    // Empty / too-short blobs: no matches at all (not "exactly one").
    ok &= check_true(vesc::scan_speed_pid({}).empty(), "scan_speed_pid({}) finds nothing");
    ok &= check_true(vesc::scan_current_limits({}).empty(), "scan_current_limits({}) finds nothing");
    const std::vector<uint8_t> short_blob(10, 0x00);
    ok &= check_true(vesc::scan_speed_pid(short_blob).empty(), "scan_speed_pid() on a too-short blob finds nothing");
    ok &= check_true(vesc::scan_current_limits(short_blob).empty(),
                      "scan_current_limits() on a too-short blob finds nothing");
    ok &= check_true(!vesc::is_unambiguous(vesc::scan_speed_pid(short_blob)), "zero matches is NOT unambiguous");

    // NEAR-MISS decoy: a speed-PID-shaped cluster where every field is
    // in range EXCEPT ki (deliberately 100.0, outside the 1e-6..50
    // range) -- proves the scanner isn't too loose (it must reject this
    // whole cluster, not just flag one field).
    {
        std::vector<uint8_t> blob;
        blob.insert(blob.end(), {0x11, 0x22, 0x33});  // 3 bytes of leading filler (misaligns any accidental match)
        vesc::append_be_f32_auto(&blob, 0.008f);   // kp: in range
        vesc::append_be_f32_auto(&blob, 100.0f);   // ki: OUT of range (near-miss decoy)
        vesc::append_be_f32_auto(&blob, 0.0f);     // kd: in range
        append_be_u16(&blob, 2000);                  // kd_filter: raw 2000 -> 0.2, in range (2-byte field, not float32_auto)
        vesc::append_be_f32_auto(&blob, 900.0f);   // min_erpm: in range
        blob.push_back(1);                          // allow_braking: in range
        vesc::append_be_f32_auto(&blob, 25000.0f); // ramp: in range
        blob.insert(blob.end(), {0x44, 0x55, 0x66});
        const std::vector<vesc::SpeedPidMatch> matches = vesc::scan_speed_pid(blob);
        ok &= check_true(matches.empty(), "near-miss decoy (ki out of range) yields zero speed-PID matches");
        ok &= check_true(!vesc::is_unambiguous(matches), "near-miss decoy scan result is NOT unambiguous");
    }

    // NEAR-MISS decoy: a current-limits-shaped cluster where every field
    // is in range except abs_current_max, which is deliberately LESS
    // than current_max (violates the "abs_current_max >= current_max"
    // constraint).
    {
        std::vector<uint8_t> blob;
        blob.insert(blob.end(), {0x77, 0x88});
        vesc::append_be_f32_auto(&blob, 25.0f);   // current_max
        vesc::append_be_f32_auto(&blob, -25.0f);  // current_min
        vesc::append_be_f32_auto(&blob, 50.0f);   // in_current_max
        vesc::append_be_f32_auto(&blob, -20.0f);  // in_current_min
        append_be_u16(&blob, 10000);                // unknown_field_a_raw: in range (2-byte field, not float32_auto)
        append_be_u16(&blob, 50);                    // unknown_field_b_raw: in range
        vesc::append_be_f32_auto(&blob, 10.0f);   // abs_current_max: OUT of range (< current_max=25)
        blob.insert(blob.end(), {0x99, 0xAA});
        const std::vector<vesc::CurrentLimitsMatch> matches = vesc::scan_current_limits(blob);
        ok &= check_true(matches.empty(), "near-miss decoy (abs_current_max < current_max) yields zero matches");
        ok &= check_true(!vesc::is_unambiguous(matches), "near-miss decoy scan result is NOT unambiguous");
    }

    // AMBIGUOUS-by-duplication: two identical, non-overlapping
    // current-limits clusters in the same blob -- the scanner correctly
    // finds BOTH (2 matches), and the CLI-facing rule (is_unambiguous())
    // correctly treats 2 matches as "not usable", same as 0.
    {
        std::vector<uint8_t> blob;
        auto append_cluster = [&blob]() {
            vesc::append_be_f32_auto(&blob, 25.0f);
            vesc::append_be_f32_auto(&blob, -25.0f);
            vesc::append_be_f32_auto(&blob, 50.0f);
            vesc::append_be_f32_auto(&blob, -20.0f);
            append_be_u16(&blob, 10000);
            append_be_u16(&blob, 50);
            vesc::append_be_f32_auto(&blob, 30.0f);
        };
        append_cluster();
        // Filler long enough that the two clusters' scan windows cannot
        // overlap (cluster is 24 bytes).
        for (int i = 0; i < 40; ++i) blob.push_back(0xFF);
        append_cluster();
        const std::vector<vesc::CurrentLimitsMatch> matches = vesc::scan_current_limits(blob);
        ok &= check_true(matches.size() == 2, "two duplicated clusters yield exactly 2 matches (got " +
                                                   std::to_string(matches.size()) + ")");
        ok &= check_true(!vesc::is_unambiguous(matches), "2 matches is NOT unambiguous (CLI-facing 'not usable')");
    }

    return ok;
}

// ---------------------------------------------------------------------
// (f7) McconfPatcher against a REAL GET_MCCONF dump from a real FW 6.6
// VESC (tests/fixtures/mcconf_fw66_real.bin), hand-decoded ground truth.
// This is the test that actually validates the corrected layout, as
// opposed to the synthetic-blob tests above which only prove the scanner
// is internally self-consistent with whatever layout this file claims.
// ---------------------------------------------------------------------

bool test_mcconf_patcher_real_fw66_fixture() {
    bool ok = true;

    // Resolve tests/fixtures/mcconf_fw66_real.bin relative to THIS SOURCE
    // file's own compile-time path (__FILE__), not the test binary's
    // runtime location -- same convention as
    // test_driver_core_calibration_file_loading()'s resolution of
    // config/calibration_example.json above, robust across both build
    // modes (repo superbuild vs. VescDriver-standalone).
    const std::string source_file = __FILE__;
    const std::string marker = "/tests/vesc_unit_tests.cpp";
    const size_t marker_pos = source_file.rfind(marker);
    const std::string vescdriver_dir = (marker_pos != std::string::npos) ? source_file.substr(0, marker_pos) : ".";
    const std::string fixture_path = vescdriver_dir + "/tests/fixtures/mcconf_fw66_real.bin";

    std::ifstream f(fixture_path, std::ios::binary | std::ios::ate);
    if (!check_true(f.is_open(), "real FW 6.6 fixture opens at '" + fixture_path + "'")) {
        return ok;
    }
    const std::streamsize size = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> blob(static_cast<size_t>(size > 0 ? size : 0));
    if (size > 0) f.read(reinterpret_cast<char*>(blob.data()), size);
    ok &= check_true(blob.size() == 483, "real fixture is the expected 483 bytes (got " +
                                              std::to_string(blob.size()) + ")");

    // Current-limits cluster: ground truth hand-decoded from the real
    // dump, ALL at offset 8.
    const std::vector<vesc::CurrentLimitsMatch> current_matches = vesc::scan_current_limits(blob);
    ok &= check_true(vesc::is_unambiguous(current_matches),
                      "scan_current_limits() on the real fixture is unambiguous (" +
                          std::to_string(current_matches.size()) + " matches)");
    if (vesc::is_unambiguous(current_matches)) {
        const auto& m = current_matches.front();
        ok &= check_true(m.offset == 8, "current-limits cluster offset == 8 (got " + std::to_string(m.offset) + ")");
        ok &= check_true(near_eq_rel(m.current_max, 50.0, 1e-5), "l_current_max == 50.0");
        ok &= check_true(near_eq_rel(m.current_min, -50.0, 1e-5), "l_current_min == -50.0");
        ok &= check_true(near_eq_rel(m.in_current_max, 40.0, 1e-5), "l_in_current_max == 40.0");
        ok &= check_true(near_eq_rel(m.in_current_min, -40.0, 1e-5), "l_in_current_min == -40.0");
        ok &= check_true(m.unknown_field_a_raw == 10000, "unknown u16 field @ offset 24 == raw 10000 (got " +
                                                              std::to_string(m.unknown_field_a_raw) + ")");
        ok &= check_true(m.unknown_field_b_raw == 50, "unknown u16 field @ offset 26 == raw 50 (got " +
                                                            std::to_string(m.unknown_field_b_raw) + ")");
        ok &= check_true(near_eq_rel(m.abs_current_max, 65.0, 1e-5), "l_abs_current_max == 65.0 (at offset 28 -- "
                                                                      "this is the field the old guessed layout "
                                                                      "misread 4 bytes early)");
    }
    // l_min_erpm/l_max_erpm immediately follow the required cluster (not
    // part of the scanner's own fixed cluster, but ground-truth-checkable
    // directly at their known real offsets since we have the real blob
    // in hand here).
    ok &= check_true(near_eq_rel(vesc::read_be_f32_auto(blob, 32), -30000.0, 1e-5), "l_min_erpm @32 == -30000.0");
    ok &= check_true(near_eq_rel(vesc::read_be_f32_auto(blob, 36), 30000.0, 1e-5), "l_max_erpm @36 == 30000.0");

    // Speed-PID cluster: ground truth hand-decoded from the real dump,
    // ALL at offset 325.
    const std::vector<vesc::SpeedPidMatch> pid_matches = vesc::scan_speed_pid(blob);
    ok &= check_true(vesc::is_unambiguous(pid_matches), "scan_speed_pid() on the real fixture is unambiguous (" +
                                                             std::to_string(pid_matches.size()) + " matches)");
    if (vesc::is_unambiguous(pid_matches)) {
        const auto& m = pid_matches.front();
        ok &= check_true(m.offset == 325, "speed-PID cluster offset == 325 (got " + std::to_string(m.offset) + ")");
        ok &= check_true(near_eq_rel(m.kp, 5e-05, 1e-3), "s_pid_kp == 5e-05");
        ok &= check_true(near_eq_rel(m.ki, 0.00075, 1e-3), "s_pid_ki == 0.00075");
        ok &= check_true(near_eq(m.kd, 0.0, 1e-9), "s_pid_kd == 0.0");
        ok &= check_true(near_eq_rel(m.kd_filter, 0.2, 1e-6),
                          "s_pid_kd_filter == 0.2 (2-byte fixed-point @ offset 337, raw 0x07D0=2000 -- this is "
                          "the field the old guessed layout decoded as a 4-byte float32_auto and got completely "
                          "wrong)");
        ok &= check_true(near_eq_rel(m.min_erpm, 500.0, 1e-5), "s_pid_min_erpm == 500.0 (at offset 339)");
        ok &= check_true(m.allow_braking == 1, "s_pid_allow_braking == 1 (at offset 343)");
        ok &= check_true(near_eq_rel(m.ramp_erpms_s, 10000.0, 1e-5), "s_pid_ramp_erpms_s == 10000.0 (at offset 344)");
    }
    // s_pid_speed_source (optional 8th field, not part of the scanner's
    // fixed cluster -- checked directly here against the real blob).
    ok &= check_true(blob.size() > 348 && blob[348] == 0, "s_pid_speed_source @348 == 0");

    // pos-PID cluster immediately following (optional secondary anchor,
    // not used by the scanner -- ground-truth-checked directly here).
    ok &= check_true(near_eq_rel(vesc::read_be_f32_auto(blob, 349), 0.03, 1e-4), "p_pid_kp @349 == 0.03");
    ok &= check_true(near_eq(vesc::read_be_f32_auto(blob, 353), 0.0, 1e-9), "p_pid_ki @353 == 0.0");
    ok &= check_true(near_eq_rel(vesc::read_be_f32_auto(blob, 357), 0.0004, 1e-4), "p_pid_kd @357 == 0.0004");
    ok &= check_true(near_eq_rel(vesc::read_be_f32_auto(blob, 361), 0.00035, 1e-4), "p_pid_kd_proc @361 == 0.00035");

    return ok;
}

// ---------------------------------------------------------------------
// (h) DriverCore -- part 3. Pure logic, no ZMQ/serial: constructs
// DriverConfig/TickInputs by hand and drives DriverCore::tick() with
// synthetic monotonic time (no real sleeps).
// ---------------------------------------------------------------------

vesc::DriverConfig make_test_driver_config() {
    vesc::DriverConfig c;
    c.mode = "erpm";
    c.cmd_per_mps = 1000.0;
    c.cmd_offset = 0.0;
    c.erpm_per_mps = 1000.0;
    c.safety.max_erpm = 100000.0;
    c.safety.max_duty = 1.0;
    c.safety.max_current = 100.0;
    c.safety.safety_max_accel = 3.5;
    c.safety.safety_max_v = 0.6;
    c.watchdog_ms = 250.0;
    c.kick.enabled = false;  // most tests below want a clean, uninterrupted signal; kick is tested on its own.
    c.calibration_file.clear();  // LinearMap.
    return c;
}

vesc::TickInputs base_driver_inputs(double now_s) {
    vesc::TickInputs in;
    in.now_s = now_s;
    in.source = vesc::Source::kAckermann;
    return in;
}

bool test_driver_core_accel_integration_and_hold() {
    bool ok = true;
    const vesc::DriverConfig cfg = make_test_driver_config();
    vesc::DriverCore core(cfg);

    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.5;
    in.ackermann.steering = 0.0;
    in.ackermann.age_s = 0.0;

    // The very first tick() call on a fresh DriverCore always has dt==0
    // (there is no prior tick to difference against) -- v_target must be
    // unchanged.
    const vesc::TickResult r0 = core.tick(in);
    ok &= check_true(near_eq(r0.v_target, 0.0), "first-ever tick: dt==0 -> v_target unchanged (0.0)");

    const double dt = 0.02;  // 50 Hz.
    double expected_v = 0.0;
    for (int i = 1; i <= 25; ++i) {  // 25 * 0.02s = 0.5s.
        t += dt;
        in.now_s = t;
        in.ackermann.age_s = 0.0;  // a fresh valid command every tick.
        const vesc::TickResult r = core.tick(in);
        expected_v += 0.5 * dt;
        ok &= check_true(near_eq(r.v_target, expected_v, 1e-6),
                          "v_target integrates the held accel*dt, tick " + std::to_string(i));
        ok &= check_true(near_eq(r.motor.value, cfg.cmd_per_mps * expected_v, 1e-3),
                          "erpm cmd == cmd_per_mps*v_target, tick " + std::to_string(i));
        ok &= check_true(r.motor.type == vesc::MotorAction::Type::kRpm, "action type is kRpm while active");
    }

    // HOLD semantics: the held accel keeps being applied every tick even
    // while age_s creeps up (simulating no FRESH command arriving), as
    // long as it stays under watchdog_ms -- exactly like mpc_robot_sim
    // integrating a held accel field between packets.
    for (int i = 0; i < 3; ++i) {
        t += dt;
        in.now_s = t;
        in.ackermann.age_s += dt;  // no fresh command -- age keeps growing, but stays < 250ms.
        const vesc::TickResult r = core.tick(in);
        expected_v += 0.5 * dt;
        ok &= check_true(near_eq(r.v_target, expected_v, 1e-6),
                          "v_target keeps integrating the HELD accel while age_s < watchdog_ms");
    }
    ok &= check_true(core.state_string() == "active", "state stays \"active\" throughout (never went stale)");

    return ok;
}

bool test_driver_core_watchdog_lifecycle() {
    bool ok = true;
    vesc::DriverConfig cfg = make_test_driver_config();
    cfg.watchdog_ms = 100.0;  // shorter, for a snappier test.
    vesc::DriverCore core(cfg);

    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.accel = 1.0;
    in.ackermann.age_s = 0.0;

    const double dt = 0.02;
    double last_v = 0.0;
    for (int i = 0; i < 16; ++i) {
        t += dt;
        in.now_s = t;
        in.ackermann.age_s = 0.0;
        const vesc::TickResult r = core.tick(in);
        last_v = r.v_target;
    }
    ok &= check_true(last_v > 0.2, "built up positive v_target before the watchdog test (got " +
                                        std::to_string(last_v) + ")");
    ok &= check_true(core.state_string() == "active", "state is active before any staleness");

    // The command goes stale: keep calling tick() with age_s (caller-
    // computed "now - last valid time") growing past watchdog_ms=100ms.
    // NOTE: while age_s is still UNDER the watchdog timeout, the normal
    // ackermann branch keeps running (v_target legitimately keeps growing
    // from the still-held accel=1.0 -- exactly the HOLD semantics h1
    // covers) -- so prev_abs_v must track every tick, not just the
    // watchdog-engaged ones, or the very first engaged tick's comparison
    // would be against a stale, too-small pre-growth baseline.
    double age = 0.0;
    bool saw_watchdog = false;
    bool saw_zero_snap = false;
    double prev_abs_v = std::fabs(last_v);
    for (int i = 0; i < 150; ++i) {  // up to 3s of staleness.
        t += dt;
        age += dt;
        in.now_s = t;
        in.ackermann.age_s = age;
        const vesc::TickResult r = core.tick(in);
        if (core.state_string() == "watchdog_brake") {
            saw_watchdog = true;
            ok &= check_true(std::fabs(r.v_target) <= prev_abs_v + 1e-9,
                              "|v_target| never grows while the watchdog ramps it down");
            if (r.v_target == 0.0) {
                saw_zero_snap = true;
                ok &= check_true(r.motor.type == vesc::MotorAction::Type::kBrake,
                                  "once v_target snaps to exactly 0, the action is a direct brake command");
                prev_abs_v = std::fabs(r.v_target);
                break;
            }
        }
        prev_abs_v = std::fabs(r.v_target);
    }
    ok &= check_true(saw_watchdog, "watchdog engages (state -> watchdog_brake) once age_s exceeds watchdog_ms");
    ok &= check_true(saw_zero_snap, "v_target eventually snaps to exactly 0.0 while braking");

    // Malformed-does-not-refresh: one more tick with age_s continuing to
    // grow (simulating a malformed payload, which must never reset the
    // watchdog timer) -- must remain braked.
    t += dt;
    age += dt;
    in.now_s = t;
    in.ackermann.age_s = age;
    const vesc::TickResult r_stale = core.tick(in);
    ok &= check_true(core.state_string() == "watchdog_brake",
                      "watchdog stays engaged across a simulated malformed (non-refreshing) payload");
    ok &= check_true(r_stale.motor.type == vesc::MotorAction::Type::kBrake, "still braking");

    // Instant disengage: the very next VALID command (age_s resets to 0)
    // disengages immediately and resumes normal tracking from v_target==0.
    t += dt;
    in.now_s = t;
    in.ackermann.age_s = 0.0;
    in.ackermann.accel = 1.0;
    const vesc::TickResult r_recover = core.tick(in);
    ok &= check_true(core.state_string() == "active", "a fresh valid command disengages the watchdog immediately");
    ok &= check_true(near_eq(r_recover.v_target, 1.0 * dt, 1e-6),
                      "post-recovery v_target resumes integrating from 0 (it was held at 0 through the brake)");

    return ok;
}

bool test_driver_core_kick_lifecycle() {
    bool ok = true;
    vesc::DriverConfig cfg = make_test_driver_config();
    cfg.kick.enabled = true;
    cfg.kick.kick_cmd = 1500.0;
    cfg.kick.kick_ms = 100.0;
    cfg.kick.min_moving_speed_mps = 0.1;
    cfg.kick.kick_erpm_threshold = 200.0;
    cfg.cmd_per_mps = 1000.0;

    vesc::DriverCore core(cfg);
    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.5;  // small -> v_target rises slowly, well under min_moving_speed for a while.
    in.ackermann.age_s = 0.0;
    in.erpm_meas = 0.0;  // "at rest" (well under kick_erpm_threshold).

    core.tick(in);  // first-ever tick: dt==0, warms up has_ticked_.

    // Next tick: v_target becomes 0.5*0.02=0.01, entering (0, 0.1) with
    // erpm_meas=0 < 200 -> the kick must fire.
    t += 0.02;
    in.now_s = t;
    const vesc::TickResult r1 = core.tick(in);
    ok &= check_true(near_eq(r1.motor.value, cfg.kick.kick_cmd, 1e-6), "kick fires: motor.value == +kick_cmd");
    ok &= check_true(r1.motor.type == vesc::MotorAction::Type::kRpm, "kick output type matches the map's mode");

    // Sustained: every tick within kick_ms (100ms) republishes the SAME
    // kick command.
    for (int i = 0; i < 3; ++i) {
        t += 0.02;
        in.now_s = t;
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(near_eq(r.motor.value, cfg.kick.kick_cmd, 1e-6), "kick sustained, tick " + std::to_string(i));
    }

    // Push well past the kick_ms window -> falls through to normal mapping.
    t += 0.06;
    in.now_s = t;
    const vesc::TickResult r_after = core.tick(in);
    ok &= check_true(near_eq(r_after.motor.value, cfg.cmd_per_mps * r_after.v_target, 1e-3),
                      "post-kick output reverts to the normal cmd_per_mps*v_target mapping");

    // Direction follows sign(v_target): a fresh core with negative accel
    // from rest kicks negative.
    vesc::DriverCore core_neg(cfg);
    vesc::TickInputs in_neg = base_driver_inputs(0.0);
    in_neg.ackermann.valid = true;
    in_neg.ackermann.accel = -0.5;
    in_neg.ackermann.age_s = 0.0;
    core_neg.tick(in_neg);
    in_neg.now_s = 0.02;
    const vesc::TickResult r_neg = core_neg.tick(in_neg);
    ok &= check_true(near_eq(r_neg.motor.value, -cfg.kick.kick_cmd, 1e-6),
                      "kick direction follows sign(v_target) (negative)");

    return ok;
}

bool test_driver_core_linear_map() {
    bool ok = true;

    vesc::LinearMap m_erpm(vesc::MapMode::kErpm, /*cmd_offset*/ 10.0, /*cmd_per_mps*/ 2000.0);
    ok &= check_true(m_erpm.mode() == vesc::MapMode::kErpm, "LinearMap mode() == kErpm");
    ok &= check_true(near_eq(m_erpm.compute_cmd(/*v_now*/ 99.0, /*v_target*/ 0.3, /*a_desired*/ 7.0), 10.0 + 2000.0 * 0.3),
                      "LinearMap ignores v_now/a_desired: cmd == cmd_offset + cmd_per_mps*v_target");

    vesc::LinearMap m_duty(vesc::MapMode::kDuty, 0.0, 0.2);
    ok &= check_true(m_duty.mode() == vesc::MapMode::kDuty, "LinearMap mode() == kDuty");
    ok &= check_true(near_eq(m_duty.compute_cmd(0.0, -0.5, 0.0), -0.1), "LinearMap negative v_target");

    return ok;
}

bool test_driver_core_bilinear_lookup() {
    bool ok = true;

    // Mirrors VescDriver/config/calibration_example.json's own grid
    // exactly (see test_driver_core_calibration_file_loading() below for
    // the file-based counterpart of this same grid).
    vesc::CalibrationGrid grid;
    grid.v_mps = {0.0, 0.2, 0.4, 0.6};
    grid.a_mps2 = {-1.0, 0.0, 1.0, 2.5};
    grid.cmd = {
        {-400.0, 0.0, 900.0, 1900.0},
        {500.0, 950.0, 1450.0, 2350.0},
        {1400.0, 1850.0, 2300.0, 3100.0},
        {2200.0, 2650.0, 3050.0, 3600.0},
    };

    ok &= check_true(near_eq(vesc::bilinear_lookup(grid, 0.2, 0.0), 950.0), "bilinear: exact grid node");
    ok &= check_true(near_eq(vesc::bilinear_lookup(grid, 0.1, 0.0), 475.0, 1e-6),
                      "bilinear: interpolates along v (halfway between 0.0 and 950.0)");
    ok &= check_true(near_eq(vesc::bilinear_lookup(grid, 0.2, 0.5), 1200.0, 1e-6),
                      "bilinear: interpolates along a (halfway between 950.0 and 1450.0)");
    ok &= check_true(near_eq(vesc::bilinear_lookup(grid, -5.0, -50.0), -400.0),
                      "bilinear: clamps below both axis ranges to the low-edge node");
    ok &= check_true(near_eq(vesc::bilinear_lookup(grid, 50.0, 50.0), 3600.0),
                      "bilinear: clamps above both axis ranges to the high-edge node");

    return ok;
}

bool test_driver_core_calibration_parse_malformed() {
    bool ok = true;

    auto expect_fail = [&](const std::string& text, const std::string& what) {
        const vesc::CalibrationParseResult r = vesc::parse_calibration_json(text);
        ok &= check_true(!r.ok, what);
    };

    expect_fail("not json", "invalid JSON -> ok=false");
    expect_fail("[1,2,3]", "non-object top level -> ok=false");
    expect_fail("{\"mode\":\"erpm\"}", "missing erpm_per_mps/cmd_per_mps/cmd_offset -> ok=false");
    expect_fail("{\"mode\":\"weird\",\"erpm_per_mps\":1,\"cmd_per_mps\":1,\"cmd_offset\":0}",
                "unrecognized mode -> ok=false");
    expect_fail(
        "{\"mode\":\"erpm\",\"erpm_per_mps\":1,\"cmd_per_mps\":1,\"cmd_offset\":0,"
        "\"grid\":{\"v_mps\":[0.0,0.0],\"a_mps2\":[0.0,1.0],\"cmd\":[[1,2],[3,4]]}}",
        "non-strictly-ascending grid.v_mps axis -> ok=false");
    expect_fail(
        "{\"mode\":\"erpm\",\"erpm_per_mps\":1,\"cmd_per_mps\":1,\"cmd_offset\":0,"
        "\"grid\":{\"v_mps\":[0.0,1.0],\"a_mps2\":[0.0,1.0],\"cmd\":[[1,2,3],[4,5,6]]}}",
        "grid.cmd row shape mismatch -> ok=false");

    // "grid": null is explicitly fine (linear-fallback case, not a failure).
    const vesc::CalibrationParseResult ok_null =
        vesc::parse_calibration_json("{\"mode\":\"duty\",\"erpm_per_mps\":10,\"cmd_per_mps\":0.1,"
                                      "\"cmd_offset\":0.02,\"grid\":null}");
    ok &= check_true(ok_null.ok, "\"grid\":null parses ok");
    ok &= check_true(!ok_null.data.has_grid, "\"grid\":null -> has_grid=false");
    ok &= check_true(ok_null.data.mode == vesc::MapMode::kDuty, "mode==\"duty\" parsed correctly");

    // Absent "grid" key entirely -- same as null.
    const vesc::CalibrationParseResult ok_absent =
        vesc::parse_calibration_json("{\"mode\":\"erpm\",\"erpm_per_mps\":10,\"cmd_per_mps\":1,\"cmd_offset\":0}");
    ok &= check_true(ok_absent.ok && !ok_absent.data.has_grid, "absent \"grid\" key -> ok=true, has_grid=false");

    return ok;
}

bool test_driver_core_calibration_file_loading() {
    bool ok = true;

    // Resolve VescDriver/config/calibration_example.json relative to THIS
    // SOURCE file's own compile-time path (__FILE__), not the test
    // binary's runtime location -- robust across both build modes (repo
    // superbuild vs. VescDriver-standalone), which put the built binary in
    // different places but never move the source tree itself.
    const std::string source_file = __FILE__;
    const std::string marker = "/tests/vesc_unit_tests.cpp";
    const size_t marker_pos = source_file.rfind(marker);
    const std::string vescdriver_dir = (marker_pos != std::string::npos) ? source_file.substr(0, marker_pos) : ".";
    const std::string calib_path = vescdriver_dir + "/config/calibration_example.json";

    const vesc::CalibrationParseResult loaded = vesc::load_calibration_file(calib_path);
    ok &= check_true(loaded.ok, "load_calibration_file() succeeds on the canned example file (" + loaded.error + ")");
    if (loaded.ok) {
        ok &= check_true(loaded.data.mode == vesc::MapMode::kErpm, "example calibration mode == erpm");
        ok &= check_true(near_eq(loaded.data.erpm_per_mps, 4600.0), "example calibration erpm_per_mps");
        ok &= check_true(near_eq(loaded.data.cmd_per_mps, 4650.0), "example calibration cmd_per_mps");
        ok &= check_true(near_eq(loaded.data.cmd_offset, 15.0), "example calibration cmd_offset");
        ok &= check_true(loaded.data.has_grid, "example calibration has a grid");
        ok &= check_true(loaded.data.grid.v_mps.size() == 4 && loaded.data.grid.a_mps2.size() == 4,
                          "example calibration grid shape");
        ok &= check_true(near_eq(vesc::bilinear_lookup(loaded.data.grid, 0.2, 0.0), 950.0),
                          "example calibration grid content matches the expected node value");
    }

    // build_motor_map(): calibration_file set -> CalibratedMap, and its
    // effective erpm_per_mps comes from the FILE (4600.0) -- deliberately
    // different from the config's own (1234.0) to prove the override
    // actually happens.
    vesc::DriverConfig cfg_cal = make_test_driver_config();
    cfg_cal.calibration_file = calib_path;
    cfg_cal.erpm_per_mps = 1234.0;
    const vesc::MotorMapBuildResult built = vesc::build_motor_map(cfg_cal);
    ok &= check_true(built.used_calibration, "build_motor_map() uses the calibration file when configured");
    ok &= check_true(near_eq(built.effective_erpm_per_mps, 4600.0),
                      "build_motor_map() takes erpm_per_mps from the calibration file, overriding config's own");
    ok &= check_true(built.map && built.map->mode() == vesc::MapMode::kErpm, "built map mode");
    if (built.map) {
        ok &= check_true(near_eq(built.map->compute_cmd(0.2, 999.0, 0.0), 950.0),
                          "built CalibratedMap uses the grid keyed on v_now, ignoring v_target at a grid node");
    }

    // Absent file -> LinearMap fallback with config's OWN placeholder gains.
    vesc::DriverConfig cfg_missing = make_test_driver_config();
    cfg_missing.calibration_file.clear();
    const vesc::MotorMapBuildResult built_missing = vesc::build_motor_map(cfg_missing);
    ok &= check_true(!built_missing.used_calibration, "no calibration_file configured -> LinearMap fallback");
    ok &= check_true(near_eq(built_missing.effective_erpm_per_mps, cfg_missing.erpm_per_mps),
                      "LinearMap fallback uses config's own erpm_per_mps");

    // Present-but-broken (nonexistent) file -> ALSO falls back to
    // LinearMap, never a hard failure.
    vesc::DriverConfig cfg_bad = make_test_driver_config();
    cfg_bad.calibration_file = "/nonexistent/path/does_not_exist.json";
    const vesc::MotorMapBuildResult built_bad = vesc::build_motor_map(cfg_bad);
    ok &= check_true(!built_bad.used_calibration,
                      "a broken/missing calibration_file falls back to LinearMap, not a hard failure");
    ok &= check_true(built_bad.note.find("failed to load") != std::string::npos,
                      "note explains the fallback reason");

    return ok;
}

bool test_driver_core_safety_clamps() {
    bool ok = true;

    // clamp_raw_value(): all three calib modes.
    vesc::SafetyConfig safety;
    safety.max_duty = 0.15;
    safety.max_erpm = 3000.0;
    safety.max_current = 8.0;
    ok &= check_true(near_eq(vesc::DriverCore::clamp_raw_value(vesc::RawCalibMode::kDuty, 0.5, safety), 0.15),
                      "clamp_raw_value: duty clamps positive overflow");
    ok &= check_true(near_eq(vesc::DriverCore::clamp_raw_value(vesc::RawCalibMode::kDuty, -0.5, safety), -0.15),
                      "clamp_raw_value: duty clamps negative overflow");
    ok &= check_true(near_eq(vesc::DriverCore::clamp_raw_value(vesc::RawCalibMode::kDuty, 0.05, safety), 0.05),
                      "clamp_raw_value: an in-range duty value passes through unchanged");
    ok &= check_true(near_eq(vesc::DriverCore::clamp_raw_value(vesc::RawCalibMode::kErpm, 5000.0, safety), 3000.0),
                      "clamp_raw_value: erpm clamps overflow");
    ok &= check_true(near_eq(vesc::DriverCore::clamp_raw_value(vesc::RawCalibMode::kCurrent, -20.0, safety), -8.0),
                      "clamp_raw_value: current clamps negative overflow");

    // Ackermann accel clamp: 10.0 m/s^2 clamped to +-safety_max_accel=3.5,
    // isolated from the v_target clamp via a generous safety_max_v.
    {
        vesc::DriverConfig cfg = make_test_driver_config();
        cfg.safety.safety_max_accel = 3.5;
        cfg.safety.safety_max_v = 100.0;
        vesc::DriverCore core(cfg);
        vesc::TickInputs in = base_driver_inputs(0.0);
        in.ackermann.valid = true;
        in.ackermann.accel = 10.0;
        in.ackermann.age_s = 0.0;
        core.tick(in);  // dt==0 on the first call.
        in.now_s = 1.0;  // dt == 1.0s exactly.
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(near_eq(r.v_target, 3.5, 1e-6),
                          "accel=10.0 is clamped to +safety_max_accel=3.5 before integration (got v_target=" +
                              std::to_string(r.v_target) + ")");
    }

    // accel=2.2 (the LaunchGovernor's own documented ~2.2 m/s^2 launch-kick
    // magnitude -- see MPC/include/mpc/LaunchGovernor.h) passes through
    // UNCLAMPED: comfortably under safety_max_accel=3.5, and (the whole
    // point of the frozen contract's "do NOT clamp to 0.73" rule) well
    // over the nominal actuator max_accel=0.73.
    {
        vesc::DriverConfig cfg = make_test_driver_config();
        cfg.safety.safety_max_accel = 3.5;
        cfg.safety.safety_max_v = 100.0;
        vesc::DriverCore core(cfg);
        vesc::TickInputs in = base_driver_inputs(0.0);
        in.ackermann.valid = true;
        in.ackermann.accel = 2.2;
        in.ackermann.age_s = 0.0;
        core.tick(in);
        in.now_s = 1.0;
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(near_eq(r.v_target, 2.2, 1e-6),
                          "accel=2.2 passes through UNCLAMPED -- got v_target=" + std::to_string(r.v_target));
    }

    // v_target clamp: safety_max_v=0.6 with a large accel over a long dt.
    {
        vesc::DriverConfig cfg = make_test_driver_config();
        cfg.safety.safety_max_accel = 100.0;  // don't let the accel clamp interfere here.
        cfg.safety.safety_max_v = 0.6;
        vesc::DriverCore core(cfg);
        vesc::TickInputs in = base_driver_inputs(0.0);
        in.ackermann.valid = true;
        in.ackermann.accel = 50.0;
        in.ackermann.age_s = 0.0;
        core.tick(in);
        in.now_s = 1.0;
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(near_eq(r.v_target, 0.6, 1e-6), "v_target clamped to +safety_max_v=0.6");
    }

    // Mapped-cmd clamp: cmd_per_mps huge enough that the mapped erpm
    // command would exceed safety.max_erpm without the output-side clamp.
    {
        vesc::DriverConfig cfg = make_test_driver_config();
        cfg.cmd_per_mps = 1.0e6;
        cfg.safety.max_erpm = 3000.0;
        cfg.safety.safety_max_accel = 100.0;
        cfg.safety.safety_max_v = 100.0;
        vesc::DriverCore core(cfg);
        vesc::TickInputs in = base_driver_inputs(0.0);
        in.ackermann.valid = true;
        in.ackermann.accel = 1.0;
        in.ackermann.age_s = 0.0;
        core.tick(in);
        in.now_s = 1.0;
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(near_eq(std::fabs(r.motor.value), 3000.0, 1e-6),
                          "mapped erpm command clamped to safety.max_erpm regardless of a huge cmd_per_mps");
    }

    return ok;
}

bool test_driver_core_calib_path_and_source_switch() {
    bool ok = true;
    const vesc::DriverConfig cfg = make_test_driver_config();
    vesc::DriverCore core(cfg);

    vesc::TickInputs in = base_driver_inputs(0.0);
    in.source = vesc::Source::kCalib;
    in.calib.has_command = false;

    const vesc::TickResult r0 = core.tick(in);
    ok &= check_true(core.state_string() == "idle", "calib source, no command ever set -> idle");
    ok &= check_true(r0.motor.type == vesc::MotorAction::Type::kNone, "idle calib -> MotorAction::kNone");

    // A fresh raw duty command, within TTL.
    in.now_s = 0.1;
    in.calib.has_command = true;
    in.calib.mode = vesc::RawCalibMode::kDuty;
    in.calib.value = vesc::DriverCore::clamp_raw_value(vesc::RawCalibMode::kDuty, 0.05, cfg.safety);
    in.calib.age_s = 0.0;
    in.calib.ttl_ms = 500.0;
    const vesc::TickResult r1 = core.tick(in);
    ok &= check_true(core.state_string() == "active", "calib source with a fresh command -> active");
    ok &= check_true(r1.motor.type == vesc::MotorAction::Type::kDuty, "raw duty command -> kDuty action");
    ok &= check_true(near_eq(r1.motor.value, 0.05, 1e-9), "raw duty command value passed through (already clamped)");

    // TTL expiry.
    in.now_s = 0.9;
    in.calib.age_s = 0.8;  // > ttl_ms/1000 == 0.5s.
    const vesc::TickResult r2 = core.tick(in);
    ok &= check_true(core.state_string() == "watchdog_brake", "calib TTL expiry -> watchdog_brake state");
    ok &= check_true(r2.motor.type == vesc::MotorAction::Type::kBrake, "calib TTL expiry -> brake action");

    // Build a nonzero v_target via ackermann...
    vesc::TickInputs in_ack = base_driver_inputs(2.0);
    in_ack.source = vesc::Source::kAckermann;
    in_ack.ackermann.valid = true;
    in_ack.ackermann.accel = 1.0;
    in_ack.ackermann.age_s = 0.0;
    core.tick(in_ack);
    in_ack.now_s = 2.1;
    const vesc::TickResult r_ack = core.tick(in_ack);
    ok &= check_true(r_ack.v_target > 0.0, "built a nonzero v_target via ackermann before testing the calib-switch reset");

    // ...then switch to calib source -- v_target must reset to 0 immediately.
    vesc::TickInputs in_calib2 = base_driver_inputs(2.12);
    in_calib2.source = vesc::Source::kCalib;
    in_calib2.calib.has_command = false;
    core.tick(in_calib2);
    ok &= check_true(near_eq(core.v_target(), 0.0), "switching to calib source resets v_target to 0 immediately");

    return ok;
}

bool test_driver_core_stop() {
    bool ok = true;
    const vesc::DriverConfig cfg = make_test_driver_config();
    vesc::DriverCore core(cfg);

    vesc::TickInputs in = base_driver_inputs(0.0);
    in.ackermann.valid = true;
    in.ackermann.accel = 1.0;
    in.ackermann.age_s = 0.0;
    core.tick(in);
    in.now_s = 1.0;
    const vesc::TickResult r = core.tick(in);
    ok &= check_true(r.v_target > 0.0, "built up a nonzero v_target before calling stop()");

    const vesc::MotorAction stopped = core.stop();
    ok &= check_true(stopped.type == vesc::MotorAction::Type::kBrake, "stop() returns a brake MotorAction");
    ok &= check_true(near_eq(core.v_target(), 0.0), "stop() forces v_target to 0");
    ok &= check_true(core.state_string() == "idle", "stop() reports idle state immediately");

    return ok;
}

bool test_driver_config_loader() {
    bool ok = true;

    const vesc::ConfigLoadResult missing = vesc::load_driver_config("/nonexistent/path/driver_config.json");
    ok &= check_true(!missing.ok, "load_driver_config() on a missing file -> ok=false");

    // The real shipped config file must parse cleanly with no warnings.
    const std::string source_file = __FILE__;
    const std::string marker = "/tests/vesc_unit_tests.cpp";
    const size_t marker_pos = source_file.rfind(marker);
    const std::string vescdriver_dir = (marker_pos != std::string::npos) ? source_file.substr(0, marker_pos) : ".";
    const std::string real_config_path = vescdriver_dir + "/config/driver_config.json";
    const vesc::ConfigLoadResult real_cfg = vesc::load_driver_config(real_config_path);
    ok &= check_true(real_cfg.ok, "the shipped driver_config.json parses ok (" + real_cfg.error + ")");
    ok &= check_true(real_cfg.warnings.empty(), "the shipped driver_config.json produces no loader warnings");
    ok &= check_true(real_cfg.config.robot_name == "robot2", "shipped config robot_name");
    ok &= check_true(real_cfg.config.ackermann_port == 3160, "shipped config ackermann_port");
    ok &= check_true(near_eq(real_cfg.config.safety.safety_max_accel, 3.5), "shipped config safety.safety_max_accel");
    ok &= check_true(real_cfg.config.kick.enabled, "shipped config kick.enabled");

    // A small custom file exercising per-field override + a malformed
    // nested object falling back to that sub-struct's defaults with a
    // warning (the rest of the file still applies).
    const std::string tmp_dir = "/tmp/vesc_driver_tests_cfg_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string tmp_path = tmp_dir + "/driver_config.json";
    {
        std::ofstream f(tmp_path);
        f << "{\"robot_name\":\"robotX\",\"ackermann_port\":9999,\"safety\":123,"
             "\"kick\":{\"kick_cmd\":42.0}}";
    }
    const vesc::ConfigLoadResult custom = vesc::load_driver_config(tmp_path);
    ok &= check_true(custom.ok, "custom config parses ok overall despite one bad nested field");
    ok &= check_true(custom.config.robot_name == "robotX", "custom robot_name applied");
    ok &= check_true(custom.config.ackermann_port == 9999, "custom ackermann_port applied");
    ok &= check_true(near_eq(custom.config.safety.safety_max_accel, vesc::DriverConfig().safety.safety_max_accel),
                      "'safety':123 (non-object) -> safety sub-struct defaults retained");
    ok &= check_true(!custom.warnings.empty(), "a non-object 'safety' value produces a loader warning");
    ok &= check_true(near_eq(custom.config.kick.kick_cmd, 42.0), "custom kick.kick_cmd applied");
    ok &= check_true(near_eq(custom.config.kick.kick_ms, vesc::DriverConfig().kick.kick_ms),
                      "unspecified kick.kick_ms retains its default");

    return ok;
}

// Regression (round-0 finding, VescDriver/src/DriverCore.cpp): a NaN/Inf
// accel or steering value -- exactly what base64("nan") decodes to, which
// the reference AckermannCodec accepts as a successfully-decoded (not
// "malformed") payload -- must never defeat DriverCore's safety clamps or
// latch v_target_ into a permanent non-finite state. Every '>'/'<'
// comparison against NaN is false, so a naive clamp lets it straight
// through; DriverCore must explicitly reject/sanitize non-finite values
// instead.
bool test_driver_core_nonfinite_ackermann_rejected() {
    bool ok = true;
    const vesc::DriverConfig cfg = make_test_driver_config();
    vesc::DriverCore core(cfg);
    const double nan_val = std::numeric_limits<double>::quiet_NaN();

    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.5;
    in.ackermann.steering = 0.1;
    in.ackermann.age_s = 0.0;
    core.tick(in);  // first-ever tick: dt==0, warms up has_ticked_.

    t += 0.02;
    in.now_s = t;
    const vesc::TickResult r1 = core.tick(in);
    ok &= check_true(near_eq(r1.v_target, 0.01, 1e-6), "warm-up tick builds a nonzero v_target before the NaN probe");

    // A NaN accel + NaN steering arriving as a "valid" decoded payload
    // (age_s legitimately resets to 0.0, same as any other fresh command --
    // this is NOT the malformed-payload case, which never touches age_s).
    t += 0.02;
    in.now_s = t;
    in.ackermann.accel = nan_val;
    in.ackermann.steering = nan_val;
    in.ackermann.age_s = 0.0;
    const vesc::TickResult r2 = core.tick(in);

    ok &= check_true(std::isfinite(r2.v_target), "a NaN accel tick leaves v_target FINITE (does not latch NaN)");
    ok &= check_true(near_eq(r2.v_target, r1.v_target, 1e-9),
                      "a NaN accel this tick holds v_target at its previous value (treated as no accel this tick)");
    ok &= check_true(std::isfinite(r2.motor.value), "a NaN accel tick produces a FINITE motor command");
    ok &= check_true(std::fabs(r2.motor.value) <= std::fabs(cfg.safety.max_erpm) + 1e-6,
                      "a NaN accel tick's motor command stays within safety.max_erpm (not latched to +max_erpm)");
    ok &= check_true(std::isfinite(r2.servo_pos), "a NaN steering tick produces a FINITE servo_pos");
    ok &= check_true(r2.servo_pos >= 0.0 && r2.servo_pos <= 1.0, "servo_pos stays in [0,1] under NaN steering");
    ok &= check_true(core.state_string() == "active", "state stays \"active\" through a NaN-valued (not malformed) command");

    // Recovery: the very next VALID finite command resumes normal
    // integration from wherever v_target was left, proving the driver is
    // NOT permanently latched -- without the fix, v_target would have
    // become NaN forever (NaN + anything stays NaN) and this would never
    // recover short of a REP "stop" or a process restart.
    t += 0.02;
    in.now_s = t;
    in.ackermann.accel = 0.5;
    in.ackermann.steering = 0.1;
    in.ackermann.age_s = 0.0;
    const vesc::TickResult r3 = core.tick(in);
    ok &= check_true(std::isfinite(r3.v_target), "post-NaN recovery: v_target is finite again");
    ok &= check_true(near_eq(r3.v_target, r2.v_target + 0.5 * 0.02, 1e-6),
                      "post-NaN recovery: v_target resumes integrating the held accel normally from where it left off");
    ok &= check_true(std::isfinite(r3.motor.value), "post-NaN recovery: motor command is finite");
    ok &= check_true(near_eq(r3.motor.value, cfg.cmd_per_mps * r3.v_target, 1e-3),
                      "post-NaN recovery: motor command matches the normal cmd_per_mps*v_target mapping");

    // +Inf must be treated the same as NaN (also defeats a naive '>' clamp
    // that would otherwise let it through as the largest legal value).
    t += 0.02;
    in.now_s = t;
    in.ackermann.accel = std::numeric_limits<double>::infinity();
    in.ackermann.age_s = 0.0;
    const vesc::TickResult r4 = core.tick(in);
    ok &= check_true(std::isfinite(r4.v_target), "a +Inf accel tick also leaves v_target finite");
    ok &= check_true(std::isfinite(r4.motor.value), "a +Inf accel tick also produces a finite motor command");

    return ok;
}

// ---------------------------------------------------------------------
// SyntheticSpeedPlant is defined here (moved up from its original
// location in section (o) below, which still uses it) so the new
// SpeedGovernor (g) and DriverCore v2 (t) sections below can reuse the
// SAME plant model rather than duplicating it -- deliberately
// separate from FakeVescModel (this one is DUTY-actuated only, since
// that's the governor's only output, and needs a live, test-mutable
// v_in). Shape: a stall band (|duty| below some threshold ->
// erpm_target 0) plus a first-order lag toward erpm_target =
// plant_gain*duty*v_in.
struct SyntheticSpeedPlant {
    double plant_gain = 4400.0;  // erpm per (duty*volt) at steady state -- matches speed_ff_gain's own
                                  // real-log-derived default, so a well-tuned ff alone gets close.
    double stall_duty = 0.02;    // |duty| below this -> erpm_target 0 (mirrors FakeVescModel's stall_duty).
    double tau_s = 0.2;          // first-order lag time constant.
    double v_in = 8.0;           // caller-mutable mid-test (the battery-drop test below).
    double erpm = 0.0;

    double erpm_target(double duty) const {
        if (std::fabs(duty) < stall_duty) return 0.0;
        return plant_gain * duty * v_in;
    }
    void step(double duty, double dt) {
        if (dt <= 0.0) return;
        const double target = erpm_target(duty);
        const double decay = std::exp(-dt / tau_s);
        erpm = target + (erpm - target) * decay;
    }
};

// ---------------------------------------------------------------------
// (g) SpeedGovernor -- standalone (Driver v2): the class extracted out of
// TeleopCore's former inline "SPEED GOVERNOR MODE" math (see
// SpeedGovernor.h). Exercised directly here, independent of TeleopCore,
// since DriverCore's own actuation=="governor" backend uses the SAME
// class. SyntheticSpeedPlant (defined below, section (o)) is reused by
// test_speed_governor_saturation_freeze_anti_windup() -- this section is
// placed after it precisely so that struct is already in scope.
// ---------------------------------------------------------------------

bool test_speed_governor_pi_ff_formula_and_slew() {
    bool ok = true;
    vesc::SpeedGovernorConfig cfg;
    cfg.kp = 2e-6;
    cfg.ki = 1e-5;
    cfg.ff_gain = 4400.0;
    cfg.max_duty = 0.2;
    cfg.duty_slew_per_s = 100.0;  // fast enough that emitted tracks duty_cmd almost exactly within one tick.
    cfg.erpm_filter_tau_s = 0.1;
    vesc::SpeedGovernor gov(cfg);

    gov.feed_erpm(0.0, 0.0);  // first-ever sample -- seeds the filter directly, ignoring dt.
    ok &= check_true(near_eq(gov.erpm_filtered(), 0.0), "first feed_erpm() seeds the filter directly at the raw value");
    gov.feed_vin(8.0);
    gov.set_target_erpm(1000.0);

    const double dt = 0.02;
    const double duty = gov.step(dt);
    const double error = 1000.0 - 0.0;
    const double effective_v_in = std::max(8.0, 6.0);
    const double expected_ff = 1000.0 / (4400.0 * effective_v_in);
    const double expected = expected_ff + cfg.kp * error + cfg.ki * error * dt;
    ok &= check_true(near_eq(duty, expected, 1e-4), "step() matches the FF+P+(1 tick I) formula on a fresh governor");
    ok &= check_true(!gov.saturated(), "not saturated on this small, well-within-max_duty command");
    ok &= check_true(near_eq(gov.target_erpm(), 1000.0), "target_erpm() accessor reflects set_target_erpm()");
    ok &= check_true(near_eq(gov.v_in(), 8.0), "v_in() accessor reflects feed_vin()");

    return ok;
}

bool test_speed_governor_tau_based_ema_exact() {
    bool ok = true;
    vesc::SpeedGovernorConfig cfg;
    cfg.erpm_filter_tau_s = 0.1;
    vesc::SpeedGovernor gov(cfg);

    gov.feed_erpm(0.0, 0.0);
    ok &= check_true(near_eq(gov.erpm_filtered(), 0.0), "seed at 0");

    const double dt = 0.02;
    const double alpha = 1.0 - std::exp(-dt / 0.1);
    gov.feed_erpm(1000.0, dt);
    const double expected = alpha * 1000.0 + (1.0 - alpha) * 0.0;
    ok &= check_true(near_eq(gov.erpm_filtered(), expected, 1e-6),
                      "feed_erpm() applies alpha=1-exp(-dt/tau) exactly (Driver v2: replaces the fixed "
                      "per-sample alpha=0.3 TeleopCore used to hardcode)");

    const double before = gov.erpm_filtered();
    gov.feed_erpm(5000.0, 0.0);
    ok &= check_true(near_eq(gov.erpm_filtered(), before), "dt<=0 (repeated timestamp) leaves the filter unchanged");
    gov.feed_erpm(5000.0, -1.0);
    ok &= check_true(near_eq(gov.erpm_filtered(), before), "negative dt (backward clock) also leaves the filter unchanged");

    return ok;
}

bool test_speed_governor_saturation_freeze_anti_windup() {
    bool ok = true;
    vesc::SpeedGovernorConfig cfg;
    cfg.kp = 2e-6;
    cfg.ki = 1e-5;
    cfg.ff_gain = 4400.0;
    cfg.max_duty = 0.05;
    cfg.duty_slew_per_s = 10.0;
    cfg.erpm_filter_tau_s = 0.1;
    vesc::SpeedGovernor gov(cfg);
    SyntheticSpeedPlant plant;
    plant.v_in = 8.0;

    gov.feed_vin(plant.v_in);
    gov.feed_erpm(plant.erpm, 0.0);
    gov.set_target_erpm(1.0e5);  // absurd, unreachable target -- forces sustained saturation.

    const double dt = 0.02;
    for (int i = 0; i < 100; ++i) {  // 2s of sustained saturation.
        gov.feed_vin(plant.v_in);
        gov.feed_erpm(plant.erpm, dt);
        const double duty = gov.step(dt);
        plant.step(duty, dt);
    }
    ok &= check_true(gov.saturated(), "sustained huge error -> saturated() true");
    ok &= check_true(near_eq(gov.emitted_duty(), cfg.max_duty, 1e-6), "pinned at max_duty during sustained saturation");
    ok &= check_true(plant.erpm > 1500.0,
                      "plant settled near its own steady-state erpm at max_duty before the switch (got " +
                          std::to_string(plant.erpm) + ")");

    gov.set_target_erpm(500.0);  // achievable.
    bool converged = false;
    int ticks = -1;
    for (int i = 0; i < 500 && !converged; ++i) {  // up to 10 more seconds.
        gov.feed_vin(plant.v_in);
        gov.feed_erpm(plant.erpm, dt);
        const double duty = gov.step(dt);
        plant.step(duty, dt);
        if (std::fabs(plant.erpm - 500.0) <= 50.0) {
            converged = true;
            ticks = i;
        }
    }
    ok &= check_true(converged, "converges to the new achievable target after switching down from saturation");
    ok &= check_true(ticks >= 0 && ticks <= 250,
                      "converges within a BOUNDED number of ticks -- anti-windup, not a wound-up integrator "
                      "dragging it out (got " +
                          std::to_string(ticks) + " ticks)");

    return ok;
}

bool test_speed_governor_reset_clears_state() {
    bool ok = true;
    vesc::SpeedGovernorConfig cfg;
    cfg.kp = 2e-6;
    cfg.ki = 1e-5;
    cfg.ff_gain = 4400.0;
    cfg.max_duty = 0.15;
    cfg.duty_slew_per_s = 100.0;
    cfg.erpm_filter_tau_s = 0.1;
    vesc::SpeedGovernor gov(cfg);
    gov.feed_vin(8.0);
    gov.feed_erpm(0.0, 0.0);
    gov.set_target_erpm(1000.0);

    const double dt = 0.02;
    for (int i = 0; i < 50; ++i) {
        gov.feed_erpm(200.0, dt);
        gov.step(dt);
    }
    ok &= check_true(gov.emitted_duty() != 0.0, "built a nonzero emitted duty before reset");
    ok &= check_true(gov.erpm_filtered() > 1.0, "built a nonzero filtered erpm before reset");

    gov.reset();
    ok &= check_true(near_eq(gov.emitted_duty(), 0.0), "reset() zeroes the slewed output");
    ok &= check_true(!gov.saturated(), "reset() clears the saturation flag");
    ok &= check_true(near_eq(gov.v_in(), 8.0), "reset() does NOT touch v_in() -- kept warm, matches TeleopCore's former v_in_last_ contract");

    gov.feed_erpm(777.0, 999.0);  // huge dt on purpose -- must be ignored (seeding, not blending).
    ok &= check_true(near_eq(gov.erpm_filtered(), 777.0),
                      "post-reset(), the next feed_erpm() reseeds the filter directly regardless of dt");

    const double duty = gov.step(dt);
    const double error = 1000.0 - 777.0;
    const double expected = (1000.0 / (4400.0 * 8.0)) + cfg.kp * error + cfg.ki * error * dt;
    ok &= check_true(near_eq(duty, expected, 1e-4),
                      "step() right after reset() matches the fresh FF+P+(1 tick I) formula -- zero carried-over "
                      "integrator windup");

    return ok;
}

bool test_speed_governor_seed_output_for_kick_handoff() {
    bool ok = true;
    vesc::SpeedGovernorConfig cfg;
    cfg.duty_slew_per_s = 0.1;  // slow slew so the seeded starting point is directly observable.
    vesc::SpeedGovernor gov(cfg);
    gov.seed_output(0.05);
    gov.set_target_erpm(0.0);
    gov.feed_erpm(0.0, 0.0);
    gov.feed_vin(8.0);

    const double duty = gov.step(0.001);  // tiny dt -> slew barely moves from the seeded value.
    ok &= check_true(near_eq(duty, 0.05, 0.01),
                      "step() right after seed_output() starts slewing FROM the seeded value, not from 0 (got " +
                          std::to_string(duty) + ")");

    return ok;
}

// ---------------------------------------------------------------------
// (i) VelocityMap (Driver v2): monotone piecewise-linear v<->erpm map.
// ---------------------------------------------------------------------

bool test_velocity_map_forward_inverse_and_clamp() {
    bool ok = true;
    vesc::VelocityCalibData data;
    data.table = {{-0.6, -2768.4}, {0.0, 0.0}, {0.6, 2768.4}};
    vesc::VelocityMap map(data);

    ok &= check_true(map.has_table(), "3-point table -> has_table()==true");
    ok &= check_true(near_eq(map.erpm_for_velocity(0.0), 0.0), "exact node, forward");
    ok &= check_true(near_eq(map.erpm_for_velocity(0.3), 1384.2, 1e-6), "interpolated forward (halfway 0.0..0.6)");
    ok &= check_true(near_eq(map.velocity_for_erpm(1384.2), 0.3, 1e-6), "interpolated inverse");
    ok &= check_true(near_eq(map.erpm_for_velocity(-5.0), -2768.4), "forward clamps below the table's low end (never extrapolates)");
    ok &= check_true(near_eq(map.erpm_for_velocity(5.0), 2768.4), "forward clamps above the table's high end");
    ok &= check_true(near_eq(map.velocity_for_erpm(-999999.0), -0.6), "inverse clamps below the table's low end");
    ok &= check_true(near_eq(map.velocity_for_erpm(999999.0), 0.6), "inverse clamps above the table's high end");

    return ok;
}

bool test_velocity_map_linear_fallback() {
    bool ok = true;
    vesc::VelocityMap default_map;  // default-constructed -- linear mode, struct defaults.
    ok &= check_true(!default_map.has_table(), "default-constructed VelocityMap has no table");
    ok &= check_true(near_eq(default_map.erpm_for_velocity(0.5), 4614.0 * 0.5), "linear fallback forward uses the default erpm_per_mps");
    ok &= check_true(near_eq(default_map.velocity_for_erpm(4614.0 * 0.5), 0.5, 1e-9), "linear fallback inverse");

    vesc::VelocityCalibData data;
    data.linear_fallback.erpm_per_mps = 1000.0;
    data.linear_fallback.offset_erpm = 50.0;
    vesc::VelocityMap map2(data);
    ok &= check_true(!map2.has_table(), "empty table -> linear mode");
    ok &= check_true(near_eq(map2.erpm_for_velocity(1.0), 1050.0), "linear fallback honors offset_erpm/erpm_per_mps");
    ok &= check_true(near_eq(map2.velocity_for_erpm(1050.0), 1.0, 1e-9), "linear fallback inverse honors the offset");

    vesc::VelocityCalibData zero_gain;
    zero_gain.linear_fallback.erpm_per_mps = 0.0;
    vesc::VelocityMap map3(zero_gain);
    ok &= check_true(near_eq(map3.velocity_for_erpm(1000.0), 0.0), "velocity_for_erpm() returns 0 rather than dividing by ~zero erpm_per_mps");

    return ok;
}

bool test_velocity_map_parse_good_and_bad() {
    bool ok = true;
    auto expect_fail = [&](const std::string& text, const std::string& what) {
        const vesc::VelocityCalibParseResult r = vesc::parse_velocity_calib_json(text);
        ok &= check_true(!r.ok, what);
    };

    expect_fail("not json", "invalid JSON -> ok=false");
    expect_fail("[1,2,3]", "non-object top level -> ok=false");
    expect_fail("{\"table\":[{\"v\":0.0,\"erpm\":0.0}]}", "table with fewer than 2 points -> ok=false");
    expect_fail("{\"table\":[{\"v\":0.0,\"erpm\":0.0},{\"v\":0.0,\"erpm\":100.0}]}", "table not strictly ascending in v -> ok=false");
    expect_fail("{\"table\":[{\"v\":0.0,\"erpm\":100.0},{\"v\":0.5,\"erpm\":50.0}]}", "table not strictly ascending in erpm -> ok=false");
    expect_fail("{\"table\":[{\"v\":0.0},{\"v\":0.5,\"erpm\":50.0}]}", "table entry missing 'erpm' -> ok=false");
    expect_fail("{\"table\":\"nope\"}", "non-array 'table' -> ok=false");

    const vesc::VelocityCalibParseResult ok_absent = vesc::parse_velocity_calib_json("{\"min_reliable_erpm\":300.0}");
    ok &= check_true(ok_absent.ok && ok_absent.data.table.empty(), "absent 'table' key -> ok=true, empty table (the documented linear-fallback case)");
    ok &= check_true(near_eq(ok_absent.data.min_reliable_erpm, 300.0), "min_reliable_erpm parses independently");

    const vesc::VelocityCalibParseResult good = vesc::parse_velocity_calib_json(
        "{\"version\":2,\"table\":[{\"v\":-0.6,\"erpm\":-2768.4},{\"v\":0.6,\"erpm\":2768.4}],"
        "\"min_reliable_erpm\":250.0,\"linear_fallback\":{\"erpm_per_mps\":4614.0,\"offset_erpm\":0.0,\"rms\":0.0}}");
    ok &= check_true(good.ok, "well-formed 2-point table parses ok");
    ok &= check_true(good.data.table.size() == 2, "table size");
    ok &= check_true(near_eq(good.data.linear_fallback.erpm_per_mps, 4614.0), "linear_fallback sub-fields parse independently");

    return ok;
}

bool test_velocity_map_load_file_and_example() {
    bool ok = true;
    const std::string source_file = __FILE__;
    const std::string marker = "/tests/vesc_unit_tests.cpp";
    const size_t marker_pos = source_file.rfind(marker);
    const std::string vescdriver_dir = (marker_pos != std::string::npos) ? source_file.substr(0, marker_pos) : ".";
    const std::string example_path = vescdriver_dir + "/config/velocity_calib.example.json";

    const vesc::VelocityMapLoadResult loaded = vesc::load_velocity_map(example_path);
    ok &= check_true(loaded.used_table, "the shipped velocity_calib.example.json loads its table (" + loaded.note + ")");
    ok &= check_true(near_eq(loaded.map.erpm_for_velocity(0.6), 2768.4, 1e-6), "shipped example table content");

    const vesc::VelocityMapLoadResult missing = vesc::load_velocity_map("/nonexistent/velocity_calib.json");
    ok &= check_true(!missing.used_table, "a missing file falls back to linear, never a hard failure");
    ok &= check_true(near_eq(missing.map.erpm_for_velocity(1.0), 4614.0), "fallback uses the built-in linear defaults");

    const vesc::VelocityMapLoadResult empty_path = vesc::load_velocity_map("");
    ok &= check_true(!empty_path.used_table, "an empty path is the documented 'no file configured' case -- also linear");

    return ok;
}

// ---------------------------------------------------------------------
// (k) SteeringAngleMap (Driver v2): monotone piecewise-linear steering
// angle (delta, radians) -> servo position map. tests/fixtures/
// steering_angle_map_example.json is a TEST-ONLY fixture -- there is
// deliberately no shipped config/ example (absent means legacy servo
// path, per the task brief).
// ---------------------------------------------------------------------

bool test_steering_angle_map_interpolation_and_clamp() {
    bool ok = true;
    vesc::SteeringAngleMapData data;
    data.wheel_base = 0.29;
    data.delta_min = -0.33;
    data.delta_max = 0.30;
    data.points = {{-0.33, 0.62}, {0.0, 0.50}, {0.30, 0.40}};
    vesc::SteeringAngleMap map(data);

    ok &= check_true(near_eq(map.servo_for_delta(0.0), 0.50), "exact node");
    ok &= check_true(near_eq(map.servo_for_delta(-0.165), 0.56, 1e-6), "interpolates halfway between -0.33 and 0.0 (0.62->0.50)");
    ok &= check_true(near_eq(map.servo_for_delta(0.15), 0.45, 1e-6), "interpolates halfway between 0.0 and 0.30 (0.50->0.40)");
    ok &= check_true(near_eq(map.servo_for_delta(-5.0), 0.62), "delta below delta_min clamps to delta_min's own servo");
    ok &= check_true(near_eq(map.servo_for_delta(5.0), 0.40), "delta above delta_max clamps to delta_max's own servo");
    ok &= check_true(near_eq(map.wheel_base(), 0.29), "wheel_base() accessor");
    ok &= check_true(!map.empty(), "a 3-point map is not empty()");

    return ok;
}

bool test_steering_angle_map_parse_validation() {
    bool ok = true;
    auto expect_fail = [&](const std::string& text, const std::string& what) {
        const vesc::SteeringAngleMapParseResult r = vesc::parse_steering_angle_map_json(text);
        ok &= check_true(!r.ok, what);
    };

    expect_fail("not json", "invalid JSON -> ok=false");
    expect_fail("[1,2,3]", "non-object top level -> ok=false");
    expect_fail("{\"wheel_base\":0.29}", "missing 'points' -> ok=false");
    expect_fail("{\"points\":\"nope\"}", "non-array 'points' -> ok=false");
    expect_fail("{\"points\":[{\"delta\":-0.1,\"servo\":0.6}]}", "fewer than 2 points -> ok=false");
    expect_fail("{\"points\":[{\"delta\":0.0,\"servo\":0.5},{\"delta\":0.0,\"servo\":0.4}]}",
                "'points' not strictly ascending in delta -> ok=false");
    expect_fail(
        "{\"points\":[{\"delta\":-0.1,\"servo\":0.5},{\"delta\":0.0,\"servo\":0.5},{\"delta\":0.1,\"servo\":0.4}]}",
        "non-monotone servo column -> ok=false");
    expect_fail("{\"points\":[{\"delta\":-0.1,\"servo\":1.5},{\"delta\":0.1,\"servo\":0.4}]}",
                "servo outside [0,1] -> ok=false");

    const vesc::SteeringAngleMapParseResult good = vesc::parse_steering_angle_map_json(
        "{\"version\":1,\"wheel_base\":0.29,\"points\":[{\"delta\":-0.33,\"servo\":0.62},{\"delta\":0.30,\"servo\":0.40}],"
        "\"delta_min\":-0.33,\"delta_max\":0.30}");
    ok &= check_true(good.ok, "well-formed 2-point descending-servo map parses ok");
    ok &= check_true(near_eq(good.data.wheel_base, 0.29), "wheel_base parses");

    const vesc::SteeringAngleMapParseResult good_ascending =
        vesc::parse_steering_angle_map_json("{\"points\":[{\"delta\":-0.33,\"servo\":0.30},{\"delta\":0.30,\"servo\":0.70}]}");
    ok &= check_true(good_ascending.ok, "an ASCENDING servo column is also accepted -- either physical direction is legal");

    return ok;
}

bool test_steering_angle_map_load_file_and_wheelbase_check() {
    bool ok = true;
    const std::string source_file = __FILE__;
    const std::string marker = "/tests/vesc_unit_tests.cpp";
    const size_t marker_pos = source_file.rfind(marker);
    const std::string vescdriver_dir = (marker_pos != std::string::npos) ? source_file.substr(0, marker_pos) : ".";
    const std::string fixture_path = vescdriver_dir + "/tests/fixtures/steering_angle_map_example.json";

    const vesc::SteeringAngleMapLoadResult loaded = vesc::load_steering_angle_map(fixture_path);
    ok &= check_true(loaded.ok, "the test fixture steering_angle_map loads ok (" + loaded.note + ")");
    if (loaded.ok) {
        ok &= check_true(near_eq(loaded.map.wheel_base(), 0.29), "fixture wheel_base");
        ok &= check_true(near_eq(loaded.map.servo_for_delta(0.0), 0.50, 1e-6), "fixture midpoint node");

        // Wheel-base mismatch detection AT THE MAP LEVEL -- exactly the
        // comparison vesc_driver_main.cpp performs at startup (see that
        // file's own wheelbase-enforcement block, which exits nonzero on
        // a mismatch before ever touching the serial port).
        const double mismatched_wheel_base = 0.35;
        ok &= check_true(std::fabs(loaded.map.wheel_base() - mismatched_wheel_base) > 1e-3,
                          "a genuinely different configured wheel_base is detected as a mismatch");
        const double agreeing_wheel_base = 0.29;
        ok &= check_true(std::fabs(loaded.map.wheel_base() - agreeing_wheel_base) <= 1e-3,
                          "an agreeing configured wheel_base is NOT flagged as a mismatch");
    }

    const vesc::SteeringAngleMapLoadResult missing = vesc::load_steering_angle_map("/nonexistent/steering_angle_map.json");
    ok &= check_true(!missing.ok, "a missing file fails to load -- caller falls back to the legacy servo path");

    const vesc::SteeringAngleMapLoadResult empty_path = vesc::load_steering_angle_map("");
    ok &= check_true(!empty_path.ok, "an empty path is the documented 'no map configured' case -- legacy servo path");

    return ok;
}

// ---------------------------------------------------------------------
// (t) DriverCore Driver v2 additions -- command_semantics/actuation,
// SpeedGovernor/VelocityMap/SteeringAngleMap wiring, the raised watchdog
// default, and the steering-NaN-hold behavior change. Reuses
// make_test_driver_config()/base_driver_inputs() from section (h) above,
// and SyntheticSpeedPlant from section (o) below.
// ---------------------------------------------------------------------

bool test_driver_core_v2_struct_defaults() {
    bool ok = true;
    const vesc::DriverConfig cfg;  // struct defaults, no overrides.

    ok &= check_true(near_eq(cfg.watchdog_ms, 1500.0),
                      "DriverConfig{}'s own compiled-in watchdog_ms default is 1500 (Driver v2, raised from the pre-v2 250)");
    ok &= check_true(cfg.command_semantics == "accel",
                      "DriverConfig{}'s own compiled-in command_semantics default is \"accel\" (pre-v2-identical "
                      "behavior when unconfigured)");
    ok &= check_true(cfg.actuation == "map",
                      "DriverConfig{}'s own compiled-in actuation default is \"map\" -- see DriverCore.h's "
                      "ActuationMode comment for why this deliberately differs from the task brief's literal "
                      "\"governor\" default (a \"governor\" struct default would silently break every "
                      "pre-existing DriverCore test/caller that never sets this field)");
    ok &= check_true(near_eq(cfg.wheel_base, 0.29), "wheel_base default");
    ok &= check_true(near_eq(cfg.default_slew_mps2, 0.73), "default_slew_mps2 default");
    ok &= check_true(near_eq(cfg.kick.kick_duty, 0.05), "kick.kick_duty default");
    ok &= check_true(near_eq(cfg.governor.kp, 2e-6) && near_eq(cfg.governor.ki, 1e-5) &&
                          near_eq(cfg.governor.ff_gain, 4400.0) && near_eq(cfg.governor.duty_slew_per_s, 0.1) &&
                          near_eq(cfg.governor.erpm_filter_tau_s, 0.1),
                      "governor sub-config defaults match the task brief's own numbers");

    // The new 1500ms default is actually WIRED to watchdog timing (not
    // just a struct field) -- age just under it must not engage, just
    // over must.
    vesc::DriverCore core(cfg);
    vesc::TickInputs in = base_driver_inputs(0.0);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.1;
    in.ackermann.age_s = 1.499;
    core.tick(in);
    in.now_s = 0.02;
    core.tick(in);
    ok &= check_true(core.state_string() == "active", "age_s just under the new 1500ms default does NOT engage the watchdog");

    in.now_s = 0.04;
    in.ackermann.age_s = 1.501;
    core.tick(in);
    ok &= check_true(core.state_string() == "watchdog_brake", "age_s just over the new 1500ms default DOES engage the watchdog");

    return ok;
}

bool test_driver_core_velocity_semantics_nan_holds() {
    bool ok = true;
    vesc::DriverConfig cfg = make_test_driver_config();
    cfg.command_semantics = "velocity";
    cfg.actuation = "map";  // isolate semantics from actuation for this test.
    cfg.default_slew_mps2 = 0.5;
    cfg.safety.safety_max_v = 10.0;
    cfg.safety.safety_max_accel = 10.0;
    cfg.kick.enabled = false;
    vesc::DriverCore core(cfg);
    ok &= check_true(core.config().command_semantics == "velocity", "config round-trips command_semantics");

    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.speed = 2.0;  // target 2.0 m/s.
    in.ackermann.accel = 1.0;  // slew bound 1.0 m/s^2 (velocity semantics repurposes this field).
    in.ackermann.age_s = 0.0;
    core.tick(in);  // first-ever tick: dt==0.

    const double dt = 0.02;
    t += dt;
    in.now_s = t;
    const vesc::TickResult r1 = core.tick(in);
    ok &= check_true(near_eq(r1.v_target, 1.0 * dt, 1e-6),
                      "velocity semantics: internal setpoint slews toward the wire speed target at the wire "
                      "accel-as-bound rate");

    // NaN speed -> HOLD the last finite target (2.0), keep slewing toward it.
    t += dt;
    in.now_s = t;
    in.ackermann.speed = std::numeric_limits<double>::quiet_NaN();
    const vesc::TickResult r2 = core.tick(in);
    ok &= check_true(std::isfinite(r2.v_target), "NaN speed tick leaves v_target finite");
    ok &= check_true(near_eq(r2.v_target, r1.v_target + 1.0 * dt, 1e-6),
                      "NaN speed HOLDS the last finite target (2.0) -- setpoint keeps slewing toward it");

    // NaN accel(bound) -> HOLD the last finite bound (1.0).
    t += dt;
    in.now_s = t;
    in.ackermann.accel = std::numeric_limits<double>::quiet_NaN();
    const vesc::TickResult r3 = core.tick(in);
    ok &= check_true(std::isfinite(r3.v_target), "NaN accel(bound) tick leaves v_target finite");
    ok &= check_true(near_eq(r3.v_target, r2.v_target + 1.0 * dt, 1e-6), "NaN accel HOLDS the last finite slew bound (1.0)");

    // Regression: "accel" semantics NEVER reads wire speed at all.
    vesc::DriverConfig cfg_accel = make_test_driver_config();
    vesc::DriverCore core_accel(cfg_accel);
    vesc::TickInputs in_accel = base_driver_inputs(0.0);
    in_accel.ackermann.valid = true;
    in_accel.ackermann.accel = 0.0;    // no accel commanded.
    in_accel.ackermann.speed = 999.0;  // a huge wire speed that MUST be ignored in accel semantics.
    in_accel.ackermann.age_s = 0.0;
    core_accel.tick(in_accel);
    in_accel.now_s = 1.0;
    const vesc::TickResult r_accel = core_accel.tick(in_accel);
    ok &= check_true(near_eq(r_accel.v_target, 0.0, 1e-9), "accel semantics ignores wire speed entirely (v_target stays 0 with accel=0)");

    return ok;
}

bool test_driver_core_steering_nan_holds_last_finite() {
    bool ok = true;
    const vesc::DriverConfig cfg = make_test_driver_config();
    vesc::DriverCore core(cfg);

    vesc::TickInputs in = base_driver_inputs(0.0);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.0;
    in.ackermann.steering = 0.2;
    in.ackermann.age_s = 0.0;
    const vesc::TickResult r0 = core.tick(in);
    const double expected0 = cfg.servo.center + (cfg.servo.invert ? -1.0 : 1.0) * cfg.servo.gain_per_rad * 0.2;
    ok &= check_true(near_eq(r0.servo_pos, std::max(cfg.servo.min_pos, std::min(cfg.servo.max_pos, expected0)), 1e-9),
                      "servo_pos matches the affine formula for a finite steering value");

    // Driver v2 change: a NaN steering value now HOLDS 0.2 exactly --
    // NOT the pre-v2 "recenter to 0 rad" behavior.
    in.now_s = 0.02;
    in.ackermann.steering = std::numeric_limits<double>::quiet_NaN();
    const vesc::TickResult r1 = core.tick(in);
    ok &= check_true(near_eq(r1.servo_pos, r0.servo_pos, 1e-9),
                      "NaN steering HOLDS the last finite servo_pos exactly (not a snap to the 0-rad/center position)");

    in.now_s = 0.04;
    const vesc::TickResult r2 = core.tick(in);
    ok &= check_true(near_eq(r2.servo_pos, r0.servo_pos, 1e-9), "the hold persists across multiple consecutive NaN ticks");

    in.now_s = 0.06;
    in.ackermann.steering = -0.1;
    const vesc::TickResult r3 = core.tick(in);
    const double expected3 = cfg.servo.center + (cfg.servo.invert ? -1.0 : 1.0) * cfg.servo.gain_per_rad * (-0.1);
    ok &= check_true(near_eq(r3.servo_pos, std::max(cfg.servo.min_pos, std::min(cfg.servo.max_pos, expected3)), 1e-9),
                      "a fresh finite steering value immediately overrides the hold");

    return ok;
}

bool test_driver_core_steering_angle_map_wiring() {
    bool ok = true;
    const std::string source_file = __FILE__;
    const std::string marker = "/tests/vesc_unit_tests.cpp";
    const size_t marker_pos = source_file.rfind(marker);
    const std::string vescdriver_dir = (marker_pos != std::string::npos) ? source_file.substr(0, marker_pos) : ".";
    const std::string fixture_path = vescdriver_dir + "/tests/fixtures/steering_angle_map_example.json";

    vesc::DriverConfig cfg = make_test_driver_config();
    cfg.steering_angle_map_file = fixture_path;
    cfg.wheel_base = 0.29;  // agrees with the fixture.
    vesc::DriverCore core(cfg);
    ok &= check_true(core.has_steering_angle_map(),
                      "DriverCore loads the configured steering_angle_map_file (" + core.steering_angle_map_note() + ")");
    ok &= check_true(near_eq(core.steering_angle_map_wheel_base(), 0.29),
                      "DriverCore exposes the loaded map's own wheel_base for the caller's mismatch check");

    vesc::TickInputs in = base_driver_inputs(0.0);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.0;
    in.ackermann.steering = 0.0;  // exact node -> servo 0.50 per the fixture.
    in.ackermann.age_s = 0.0;
    const vesc::TickResult r = core.tick(in);
    ok &= check_true(near_eq(r.servo_pos, 0.50, 1e-6),
                      "servo_pos comes from the loaded SteeringAngleMap, REPLACING the legacy "
                      "center+gain_per_rad affine formula entirely");

    vesc::DriverConfig cfg_legacy = make_test_driver_config();
    vesc::DriverCore core_legacy(cfg_legacy);
    ok &= check_true(!core_legacy.has_steering_angle_map(), "no steering_angle_map_file configured -> legacy servo path (regression)");

    return ok;
}

bool test_driver_core_governor_actuation_converges() {
    bool ok = true;
    vesc::DriverConfig cfg = make_test_driver_config();
    cfg.actuation = "governor";
    cfg.command_semantics = "accel";
    cfg.kick.enabled = false;
    cfg.safety.max_duty = 0.2;
    cfg.governor.kp = 2e-6;
    cfg.governor.ki = 1e-5;
    cfg.governor.ff_gain = 4400.0;
    cfg.governor.duty_slew_per_s = 10.0;
    cfg.governor.erpm_filter_tau_s = 0.1;
    // velocity_calib_file left empty -> the default linear-fallback
    // VelocityMap (4614 erpm/mps), so the expected target erpm below is
    // exactly derivable from safety_max_v.
    vesc::DriverCore core(cfg);
    ok &= check_true(!core.velocity_map_used_table(), "no velocity_calib_file configured -> linear-fallback VelocityMap");

    SyntheticSpeedPlant plant;  // reused from section (o) below.
    plant.v_in = 8.0;
    plant.plant_gain = 4400.0;

    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.accel = 3.0;  // ramps v_target up quickly toward safety_max_v.
    in.ackermann.age_s = 0.0;
    core.tick(in);

    const double dt = 0.02;
    for (int i = 0; i < 500; ++i) {  // 10s.
        t += dt;
        in.now_s = t;
        in.ackermann.age_s = 0.0;
        in.erpm_meas = plant.erpm;
        in.v_in = plant.v_in;
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(r.motor.type == vesc::MotorAction::Type::kDuty,
                          "actuation==\"governor\" ALWAYS emits kDuty, tick " + std::to_string(i));
        plant.step(r.motor.value, dt);
    }
    const double expected_erpm = 4614.0 * cfg.safety.safety_max_v;
    ok &= check_true(near_eq(plant.erpm, expected_erpm, 150.0),
                      "governor actuation backend converges toward the VelocityMap-derived target erpm (got " +
                          std::to_string(plant.erpm) + ", expected ~" + std::to_string(expected_erpm) + ")");

    return ok;
}

bool test_driver_core_kick_in_governor_mode_seeds_slew() {
    bool ok = true;
    vesc::DriverConfig cfg = make_test_driver_config();
    cfg.actuation = "governor";
    cfg.kick.enabled = true;
    cfg.kick.kick_duty = 0.08;
    cfg.kick.kick_ms = 100.0;
    cfg.kick.min_moving_speed_mps = 0.1;
    cfg.kick.kick_erpm_threshold = 200.0;
    cfg.safety.max_duty = 0.5;           // generous, isolate this test from the safety clamp.
    cfg.governor.duty_slew_per_s = 0.5;  // slow enough that the seeded starting point is directly observable.
    vesc::DriverCore core(cfg);

    double t = 0.0;
    vesc::TickInputs in = base_driver_inputs(t);
    in.ackermann.valid = true;
    in.ackermann.accel = 0.5;  // small -> v_target rises slowly, entering the kick band.
    in.ackermann.age_s = 0.0;
    in.erpm_meas = 0.0;  // "at rest".
    core.tick(in);       // first-ever tick: dt==0.

    t += 0.02;
    in.now_s = t;
    const vesc::TickResult r1 = core.tick(in);
    ok &= check_true(r1.motor.type == vesc::MotorAction::Type::kDuty, "kick output type is kDuty under actuation==\"governor\"");
    ok &= check_true(near_eq(r1.motor.value, cfg.kick.kick_duty, 1e-9),
                      "kick fires: motor.value == +kick_duty (NOT kick_cmd, which is erpm-flavored)");

    for (int i = 0; i < 3; ++i) {
        t += 0.02;
        in.now_s = t;
        const vesc::TickResult r = core.tick(in);
        ok &= check_true(near_eq(r.motor.value, cfg.kick.kick_duty, 1e-9), "kick sustained at kick_duty, tick " + std::to_string(i));
    }

    // Push past kick_ms -> falls through to the governor. dt for this
    // specific tick() call is 0.06s (t was last at 0.08, now 0.14), so
    // the governor's own mandatory slew bounds this tick's movement to
    // at most duty_slew_per_s*0.06 away from the seeded kick_duty --
    // proving the seed handoff (an UN-seeded governor always starts its
    // slew from 0 instead).
    t += 0.06;
    in.now_s = t;
    const vesc::TickResult r_after = core.tick(in);
    ok &= check_true(r_after.motor.type == vesc::MotorAction::Type::kDuty, "post-kick output is still kDuty (governor actuation)");
    const double max_delta = cfg.governor.duty_slew_per_s * 0.06;
    ok &= check_true(std::fabs(r_after.motor.value - cfg.kick.kick_duty) <= max_delta + 1e-6,
                      "post-kick governor output starts its slew FROM the seeded kick_duty, within this tick's own "
                      "slew budget (got " +
                          std::to_string(r_after.motor.value) + ", seeded " + std::to_string(cfg.kick.kick_duty) + ")");

    return ok;
}

// (j) TeleopCore -- pure logic, no I/O: constructs a TeleopConfig by hand
// and drives handle_key()/step()/feed_telemetry() with synthetic
// monotonic time (no real sleeps), mirroring DriverCore's own unit-test
// style (section h above).
// ---------------------------------------------------------------------

vesc::TeleopConfig make_test_teleop_config() {
    vesc::TeleopConfig c;
    c.duty_mag_default = 0.02;
    c.erpm_mag_default = 1000.0;
    c.duty_step = 0.005;
    c.erpm_step = 100.0;
    c.max_duty = 0.2;
    c.max_erpm = 6000.0;
    c.deadman_ms = 2000.0;
    c.brake_hold_ms = 300.0;
    c.current_abort = 8.0;
    c.brake_amps = 3.0;
    return c;
}

bool test_teleop_core_mode_switch_separate_magnitudes() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    ok &= check_true(core.mode() == vesc::TeleopMode::kDuty, "TeleopCore starts in duty mode");
    ok &= check_true(near_eq(core.magnitude(), 0.02), "default duty magnitude");

    core.handle_key('+', 0.0);
    ok &= check_true(near_eq(core.duty_magnitude(), 0.025), "'+' bumps duty magnitude by duty_step");

    core.handle_key('E', 0.1);
    ok &= check_true(core.mode() == vesc::TeleopMode::kErpm, "'E' switches to erpm mode");
    ok &= check_true(near_eq(core.magnitude(), 1000.0),
                      "erpm magnitude starts at its own default, unaffected by duty's '+'");

    core.handle_key('+', 0.2);
    ok &= check_true(near_eq(core.erpm_magnitude(), 1100.0), "'+' in erpm mode bumps by erpm_step, independent of duty");

    core.handle_key('D', 0.3);
    ok &= check_true(near_eq(core.duty_magnitude(), 0.025),
                      "switching back to duty: duty magnitude retained its own earlier edit");
    ok &= check_true(near_eq(core.erpm_magnitude(), 1100.0),
                      "erpm magnitude retained its own edit while not the active mode");

    return ok;
}

bool test_teleop_core_step_adjust_and_clamp() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.max_duty = 0.03;  // tight ceiling to exercise the clamp quickly.
    vesc::TeleopCore core(cfg);

    core.handle_key('+', 0.0);  // -> 0.025
    core.handle_key('+', 0.1);  // -> 0.03 == max_duty
    ok &= check_true(near_eq(core.magnitude(), 0.03), "duty magnitude reaches max_duty");
    core.handle_key('+', 0.2);  // clamp: stays at max_duty.
    ok &= check_true(near_eq(core.magnitude(), 0.03), "'+' beyond max_duty is clamped, not exceeded");

    // Decrease repeatedly -- floors at exactly ONE step, never below it.
    for (int i = 0; i < 20; ++i) {
        core.handle_key('-', 0.3 + i * 0.01);
    }
    ok &= check_true(near_eq(core.magnitude(), cfg.duty_step),
                      "'-' repeated floors at exactly one duty_step, never reaches/crosses 0");

    // erpm side, using '=' and '_' aliases.
    core.handle_key('E', 1.0);
    core.handle_key('=', 1.1);
    ok &= check_true(near_eq(core.erpm_magnitude(), 1100.0), "'=' is an alias for '+' (erpm)");
    core.handle_key('_', 1.2);
    ok &= check_true(near_eq(core.erpm_magnitude(), 1000.0), "'_' is an alias for '-' (erpm)");

    return ok;
}

bool test_teleop_core_digit_entry_commit_and_clear() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    ok &= check_true(core.digit_buffer().empty(), "digit buffer starts empty");
    for (char c : std::string("0.075")) core.handle_key(c, 0.0);
    ok &= check_true(core.digit_buffer() == "0.075", "digits/'.' accumulate into the buffer verbatim");
    ok &= check_true(near_eq(core.magnitude(), 0.02), "buffer contents do not apply until ENTER");

    core.handle_key('\r', 0.1);
    ok &= check_true(near_eq(core.magnitude(), 0.075), "ENTER commits the buffer as the current mode's magnitude");
    ok &= check_true(core.digit_buffer().empty(), "buffer is cleared after commit");

    // ESC clears without committing.
    for (char c : std::string("999")) core.handle_key(c, 0.2);
    core.handle_key(0x1B, 0.3);
    ok &= check_true(core.digit_buffer().empty(), "ESC clears the buffer");
    ok &= check_true(near_eq(core.magnitude(), 0.075), "ESC leaves the prior magnitude untouched");

    // Commit is clamped to the mode's max.
    for (char c : std::string("50")) core.handle_key(c, 0.4);
    core.handle_key('\n', 0.5);  // '\n' also commits.
    ok &= check_true(near_eq(core.magnitude(), 0.2), "digit-entry commit clamps to max_duty (typed 50 > 0.2)");

    // Empty ENTER (nothing typed) is a no-op, not a crash/reset-to-0.
    core.handle_key('\r', 0.6);
    ok &= check_true(near_eq(core.magnitude(), 0.2), "ENTER with an empty buffer leaves the magnitude unchanged");

    return ok;
}

bool test_teleop_core_drive_and_deadman_refresh() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    const vesc::TeleopMotorAction idle = core.step(0.0);
    ok &= check_true(idle.type == vesc::TeleopMotorAction::Type::kNone, "before any key: step() returns kNone");

    core.handle_key('w', 0.0);
    ok &= check_true(core.is_driving() && core.direction() == vesc::DriveDirection::kForward,
                      "'w' engages driving forward");
    const vesc::TeleopMotorAction a1 = core.step(0.01);
    ok &= check_true(a1.type == vesc::TeleopMotorAction::Type::kDuty, "driving in duty mode -> kDuty action");
    ok &= check_true(near_eq(a1.value, 0.02), "forward duty action value == +magnitude");

    // Pressing 'w' again while already driving forward is a no-op state
    // change, but STILL refreshes the deadman clock.
    core.handle_key('w', 1.5);
    const vesc::TeleopMotorAction a2 = core.step(1.5);
    ok &= check_true(a2.type == vesc::TeleopMotorAction::Type::kDuty && near_eq(a2.value, 0.02),
                      "repeated 'w' keeps driving forward, same value");

    // Reverse.
    core.handle_key('s', 1.6);
    const vesc::TeleopMotorAction a3 = core.step(1.6);
    ok &= check_true(a3.type == vesc::TeleopMotorAction::Type::kDuty && near_eq(a3.value, -0.02),
                      "'s' switches to reverse -- signed value flips");

    return ok;
}

bool test_teleop_core_deadman_and_stop() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.deadman_ms = 500.0;
    cfg.brake_hold_ms = 200.0;
    vesc::TeleopCore core(cfg);

    core.handle_key('w', 0.0);
    core.step(0.0);
    // Just under the deadman window: still driving.
    const vesc::TeleopMotorAction a_before = core.step(0.499);
    ok &= check_true(a_before.type == vesc::TeleopMotorAction::Type::kDuty,
                      "still driving just under the deadman window (0.499s < 0.5s)");

    // Fires at EXACTLY last_key_time + deadman_ms.
    const vesc::TeleopMotorAction a_at = core.step(0.5);
    ok &= check_true(a_at.type == vesc::TeleopMotorAction::Type::kBrake,
                      "deadman fires at EXACTLY last_key_time+deadman_ms -> brake");
    ok &= check_true(core.is_braking() && core.stop_reason() == "deadman", "state/reason reflect the deadman trip");

    // Stays braking mid-hold.
    const vesc::TeleopMotorAction a_mid = core.step(0.6);
    ok &= check_true(a_mid.type == vesc::TeleopMotorAction::Type::kBrake, "still braking mid-hold");

    // Exactly at brake_started+brake_hold_ms -> none (idle).
    const vesc::TeleopMotorAction a_end = core.step(0.7);  // 0.5 + 0.2
    ok &= check_true(a_end.type == vesc::TeleopMotorAction::Type::kNone,
                      "brake hold elapses at exactly brake_started+brake_hold_ms -> kNone");
    ok &= check_true(!core.is_driving() && !core.is_braking(), "back to idle after the brake hold");

    // Explicit stop key: immediate brake-then-idle, same as deadman, but
    // triggered right away regardless of how recently a key arrived.
    vesc::TeleopCore core2(cfg);
    core2.handle_key('w', 0.0);
    core2.step(0.0);
    core2.handle_key(' ', 0.05);
    ok &= check_true(core2.is_braking() && core2.stop_reason() == "stop",
                      "space key -> immediate braking, reason \"stop\"");
    const vesc::TeleopMotorAction s1 = core2.step(0.05);
    ok &= check_true(s1.type == vesc::TeleopMotorAction::Type::kBrake, "stop key -> brake this tick");
    const vesc::TeleopMotorAction s2 = core2.step(0.05 + cfg.brake_hold_ms / 1000.0);
    ok &= check_true(s2.type == vesc::TeleopMotorAction::Type::kNone,
                      "stop key's brake hold also elapses after brake_hold_ms");

    // 'x' is the alias for space.
    vesc::TeleopCore core3(cfg);
    core3.handle_key('w', 0.0);
    core3.step(0.0);
    core3.handle_key('x', 0.02);
    ok &= check_true(core3.is_braking(), "'x' is an alias for the stop key");

    return ok;
}

bool test_teleop_core_current_abort_latches_and_clears() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    core.handle_key('w', 0.0);
    const vesc::TeleopMotorAction driving = core.step(0.0);
    ok &= check_true(driving.type == vesc::TeleopMotorAction::Type::kDuty, "driving before any telemetry arrives");

    vesc::VescValues v;
    v.current_motor = 3.0;  // under the 8.0A default abort -- no trip.
    core.feed_telemetry(v, 0.01);
    ok &= check_true(!core.is_aborted(), "current under the abort threshold does not trip");

    v.current_motor = 12.3;  // over -- trips, matches the example format in the task spec.
    core.feed_telemetry(v, 0.02);
    ok &= check_true(core.is_aborted(), "current over current_abort trips the latch");
    ok &= check_true(core.abort_reason() == "current 12.3A > 8.0A abort",
                      "abort reason string matches the documented format (got '" + core.abort_reason() + "')");

    // Latched: driving keys are ignored while aborted.
    core.handle_key('w', 0.03);
    ok &= check_true(!core.is_driving(), "'w' while aborted does not resume driving");
    const vesc::TeleopMotorAction a = core.step(0.03);
    ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kBrake, "step() keeps braking while aborted");

    // A further feed_telemetry() call (even a worse reading) does not
    // overwrite the already-latched reason.
    vesc::VescValues v2;
    v2.current_motor = 50.0;
    core.feed_telemetry(v2, 0.04);
    ok &= check_true(core.abort_reason() == "current 12.3A > 8.0A abort", "abort reason is not overwritten once latched");

    // 'c' clears it; driving can resume.
    core.handle_key('c', 0.05);
    ok &= check_true(!core.is_aborted(), "'c' clears the abort latch");
    ok &= check_true(core.abort_reason().empty(), "abort reason is cleared too");
    core.handle_key('w', 0.06);
    ok &= check_true(core.is_driving(), "driving resumes normally after clearing the abort");

    return ok;
}

bool test_teleop_core_fault_abort() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());
    core.handle_key('E', 0.0);
    core.handle_key('w', 0.0);
    core.step(0.0);

    vesc::VescValues v;
    v.current_motor = 0.1;  // well under the current-abort threshold.
    v.fault = 4;            // ABS_OVER_CURRENT
    core.feed_telemetry(v, 0.01);
    ok &= check_true(core.is_aborted(), "a nonzero fault code trips the abort latch even with low current");
    ok &= check_true(core.abort_reason() == "fault 4 (ABS_OVER_CURRENT)",
                      "fault abort reason names the fault (got '" + core.abort_reason() + "')");

    // Unrecognized fault code: raw number only, no parenthetical name.
    vesc::TeleopCore core2(make_test_teleop_config());
    vesc::VescValues v2;
    v2.fault = 42;
    core2.feed_telemetry(v2, 0.0);
    ok &= check_true(core2.abort_reason() == "fault 42",
                      "unrecognized fault code prints just the raw number (got '" + core2.abort_reason() + "')");

    // Every named fault code, per the spec's mapping.
    const std::vector<std::pair<int, std::string>> named = {
        {1, "OVER_VOLTAGE"}, {2, "UNDER_VOLTAGE"},     {3, "DRV"},
        {4, "ABS_OVER_CURRENT"}, {5, "OVER_TEMP_FET"}, {6, "OVER_TEMP_MOTOR"},
    };
    for (const auto& p : named) {
        ok &= check_true(vesc::fault_name(static_cast<uint8_t>(p.first)) == p.second,
                          "fault_name(" + std::to_string(p.first) + ") == \"" + p.second + "\"");
    }
    ok &= check_true(vesc::fault_name(0).empty(), "fault_name(0) is empty (0 means \"no fault\", never reached in practice)");

    return ok;
}

bool test_teleop_core_clamping_to_configured_max() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.max_duty = 0.1;
    cfg.max_erpm = 3000.0;
    cfg.duty_mag_default = 0.02;
    cfg.erpm_mag_default = 1000.0;
    vesc::TeleopCore core(cfg);

    for (int i = 0; i < 100; ++i) core.handle_key('+', i * 0.001);
    ok &= check_true(near_eq(core.duty_magnitude(), 0.1), "repeated '+' clamps duty magnitude to max_duty");

    core.handle_key('E', 1.0);
    for (int i = 0; i < 100; ++i) core.handle_key('+', 1.0 + i * 0.001);
    ok &= check_true(near_eq(core.erpm_magnitude(), 3000.0), "repeated '+' clamps erpm magnitude to max_erpm");

    // The clamp also holds for the actual commanded action while driving.
    core.handle_key('w', 2.0);
    const vesc::TeleopMotorAction a = core.step(2.0);
    ok &= check_true(near_eq(a.value, 3000.0), "driving action value stays at the clamped max_erpm");

    return ok;
}

// ---------------------------------------------------------------------
// (l) FW 2.x legacy GET_VALUES support: parse_get_values_legacy()/
// parse_get_values_for_fw() (VescProtocol), fake_vesc's --fw 2.x legacy
// reply layout, and vesc_teleop's end-to-end behavior against a fake
// FW-2.18 unit (legacy-layout startup notice, no ALIVE while idle, the
// deadman/drive path otherwise unchanged). Added after a real FW 2.18
// unit's replacement hardware turned out unable to be reflashed newer.
// ---------------------------------------------------------------------

// Hand-built legacy (FW 2.x) GET_VALUES reply payload. Reuses the SAME
// hand-verified numeric values as canned_get_values_payload() above for
// every field the two layouts share (current_motor/current_in/duty/rpm/
// v_in/amp_hours/watt_hours/tachometer/tachometer_abs), rearranged into
// the legacy field order (see VescProtocol.h's parse_get_values_legacy()
// doc comment) with SIX distinct temp_mos values (so the max() mapping
// rule is actually exercised, not just trivially true) and a NONZERO
// fault byte (the modern canned payload above uses fault=0; this one
// deliberately doesn't, to cover both).
std::vector<uint8_t> canned_get_values_payload_legacy() {
    return {
        0x04,        // [0]      command id (GET_VALUES)
        0x00, 0xC8,  // [1..2]   temp_mos1 raw 200 -> 20.0
        0x01, 0x63,  // [3..4]   temp_mos2 raw 355 -> 35.5  (the max -- becomes temp_fet)
        0x00, 0xDC,  // [5..6]   temp_mos3 raw 220 -> 22.0
        0x00, 0xB4,  // [7..8]   temp_mos4 raw 180 -> 18.0
        0x00, 0xFA,  // [9..10]  temp_mos5 raw 250 -> 25.0
        0x01, 0x2C,  // [11..12] temp_mos6 raw 300 -> 30.0
        0x00, 0xF0,  // [13..14] temp_pcb raw 240 -> 24.0 (consumed, not stored)
        0x00, 0x00, 0x04, 0xD2,  // [15..18] current_motor raw 1234 -> 12.34
        0x00, 0x00, 0x02, 0x37,  // [19..22] current_in raw 567 -> 5.67
        0x00, 0x7B,              // [23..24] duty_now raw 123 -> 0.123
        0x00, 0x00, 0xAF, 0xC8,  // [25..28] rpm raw 45000 (erpm, no scaling)
        0x00, 0xF5,              // [29..30] v_in raw 245 -> 24.5
        0x00, 0x00, 0x3A, 0x98,  // [31..34] amp_hours raw 15000 (consumed)
        0x00, 0x00, 0x13, 0x88,  // [35..38] amp_hours_charged raw 5000 (consumed)
        0x00, 0x01, 0x86, 0xA0,  // [39..42] watt_hours raw 100000 (consumed)
        0x00, 0x00, 0x4E, 0x20,  // [43..46] watt_hours_charged raw 20000 (consumed)
        0x00, 0x01, 0xE2, 0x40,  // [47..50] tachometer raw 123456
        0x00, 0x09, 0xFB, 0xF1,  // [51..54] tachometer_abs raw 654321
        0x04,        // [55]     fault = 4 (ABS_OVER_CURRENT)
    };
}

bool test_get_values_parse_legacy_fw2() {
    bool ok = true;

    const std::vector<uint8_t> payload = canned_get_values_payload_legacy();
    ok &= check_true(payload.size() == 56, "canned legacy GET_VALUES payload is 56 bytes");

    const vesc::VescValues v = vesc::parse_get_values_legacy(payload);
    ok &= check_true(v.ok, "parse_get_values_legacy() ok on well-formed canned payload");
    ok &= check_true(!v.has_temp_motor, "legacy parse reports has_temp_motor=false");
    ok &= check_true(near_eq(v.temp_motor, 0.0), "legacy parse leaves temp_motor at 0.0 (not NaN)");
    ok &= check_true(near_eq(v.temp_fet, 35.5), "temp_fet == max(temp_mos1..6) == 35.5 (temp_mos2)");
    ok &= check_true(near_eq(v.current_motor, 12.34), "current_motor == 12.34");
    ok &= check_true(near_eq(v.current_in, 5.67), "current_in == 5.67");
    ok &= check_true(near_eq(v.duty, 0.123), "duty == 0.123");
    ok &= check_true(v.erpm == 45000, "erpm == 45000");
    ok &= check_true(near_eq(v.v_in, 24.5), "v_in == 24.5");
    ok &= check_true(v.tachometer == 123456, "tachometer == 123456");
    ok &= check_true(v.tachometer_abs == 654321, "tachometer_abs == 654321");
    ok &= check_true(v.fault == 4, "fault == 4 (nonzero fault byte forwarded correctly)");

    // Tolerate-longer, same contract as the modern parser.
    std::vector<uint8_t> with_trailer = payload;
    with_trailer.insert(with_trailer.end(), {0xDE, 0xAD, 0xBE, 0xEF});
    const vesc::VescValues v_trailer = vesc::parse_get_values_legacy(with_trailer);
    ok &= check_true(v_trailer.ok, "parse_get_values_legacy() tolerates trailing bytes");
    ok &= check_true(near_eq(v_trailer.temp_fet, 35.5) && v_trailer.tachometer_abs == 654321,
                      "trailing bytes do not change parsed fields");

    // Stop-cleanly on a truncated payload.
    const std::vector<uint8_t> truncated(payload.begin(), payload.begin() + 10);
    const vesc::VescValues v_short = vesc::parse_get_values_legacy(truncated);
    ok &= check_true(!v_short.ok, "parse_get_values_legacy() reports ok=false on a truncated payload");

    // Empty payload / wrong command id.
    const vesc::VescValues v_empty = vesc::parse_get_values_legacy({});
    ok &= check_true(!v_empty.ok, "parse_get_values_legacy() reports ok=false on empty payload");
    std::vector<uint8_t> wrong_id = payload;
    wrong_id[0] = 0x05;
    const vesc::VescValues v_wrong = vesc::parse_get_values_legacy(wrong_id);
    ok &= check_true(!v_wrong.ok, "parse_get_values_legacy() reports ok=false on wrong command id");

    return ok;
}

bool test_get_values_parse_for_fw_dispatch() {
    bool ok = true;

    const std::vector<uint8_t> legacy_payload = canned_get_values_payload_legacy();
    const std::vector<uint8_t> modern_payload = canned_get_values_payload();

    const vesc::VescValues via_dispatch_legacy = vesc::parse_get_values_for_fw(legacy_payload, 2);
    const vesc::VescValues via_direct_legacy = vesc::parse_get_values_legacy(legacy_payload);
    ok &= check_true(via_dispatch_legacy.ok && near_eq(via_dispatch_legacy.temp_fet, via_direct_legacy.temp_fet) &&
                          via_dispatch_legacy.erpm == via_direct_legacy.erpm,
                      "parse_get_values_for_fw(payload, fw_major=2) dispatches to the legacy parser");
    ok &= check_true(!via_dispatch_legacy.has_temp_motor, "fw_major=2 dispatch carries has_temp_motor=false through");

    for (uint8_t fw_major : {3, 4, 5, 6}) {
        const vesc::VescValues via_dispatch_modern = vesc::parse_get_values_for_fw(modern_payload, fw_major);
        const vesc::VescValues via_direct_modern = vesc::parse_get_values(modern_payload);
        ok &= check_true(via_dispatch_modern.ok && near_eq(via_dispatch_modern.temp_fet, via_direct_modern.temp_fet) &&
                              via_dispatch_modern.erpm == via_direct_modern.erpm,
                          "parse_get_values_for_fw(payload, fw_major=" + std::to_string(fw_major) +
                              ") dispatches to the modern parser");
        ok &= check_true(via_dispatch_modern.has_temp_motor,
                          "fw_major=" + std::to_string(fw_major) + " dispatch carries has_temp_motor=true through");
    }

    // Feeding a LEGACY payload through the MODERN dispatch path is
    // exactly the real-world bug that motivated this whole feature: it
    // does NOT fail loudly (the legacy payload is long enough that the
    // modern parser's byte-count checks are all satisfied, so ok stays
    // true) -- it just silently produces WRONG values. That silent-
    // wrongness, not a crash/ok=false, is the actual hazard fw-aware
    // dispatch exists to prevent.
    const vesc::VescValues mismatched = vesc::parse_get_values_for_fw(legacy_payload, 6);
    ok &= check_true(mismatched.ok,
                      "parsing a (longer) legacy payload with the modern parser 'succeeds' (enough bytes) -- "
                      "silently wrong, not a hard failure");
    ok &= check_true(!near_eq(mismatched.temp_fet, via_direct_legacy.temp_fet, 1e-6),
                      "...and produces a DIFFERENT (wrong) temp_fet than the correct legacy-layout parse (got " +
                          std::to_string(mismatched.temp_fet) + " vs correct " +
                          std::to_string(via_direct_legacy.temp_fet) + ")");

    return ok;
}

// ---------------------------------------------------------------------
// (m) TeleopCore RAMP mode -- pure logic, no I/O: slews the emitted
// command toward direction()*magnitude() at a configurable per-mode
// rate instead of stepping instantly, while every safety path (stop,
// deadman, current-abort, fault-abort) still brakes IMMEDIATELY,
// bypassing the ramp entirely.
// ---------------------------------------------------------------------

bool test_teleop_core_ramp_off_instant_regression() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.ramp_enabled = false;  // explicit, though this is already the default.
    vesc::TeleopCore core(cfg);

    ok &= check_true(!core.ramp_enabled(), "ramp starts OFF per config/default");

    core.step(0.0);  // warm-up.
    core.handle_key('w', 0.0);
    const vesc::TeleopMotorAction a1 = core.step(0.001);  // an arbitrarily tiny dt.
    ok &= check_true(near_eq(a1.value, core.magnitude()),
                      "ramp OFF: instantly at the full target even on the very first (tiny-dt) tick");
    ok &= check_true(near_eq(core.emitted_value(), core.target_value()),
                      "emitted_value() == target_value() immediately, ramp OFF");

    core.handle_key('+', 0.001);  // bump magnitude -- instantly reflected too.
    const vesc::TeleopMotorAction a2 = core.step(0.002);
    ok &= check_true(near_eq(a2.value, core.magnitude()), "ramp OFF: a magnitude change is reflected instantly, no lag");

    return ok;
}

bool test_teleop_core_ramp_slew_progression_exact_arithmetic() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.ramp_enabled = true;
    cfg.erpm_ramp = 2000.0;
    cfg.erpm_mag_default = 1000.0;
    vesc::TeleopCore core(cfg);

    core.step(0.0);  // warm-up: has_stepped_=true, dt==0 on THIS call.
    core.handle_key('E', 0.0);
    core.handle_key('w', 0.0);
    ok &= check_true(near_eq(core.emitted_value(), 0.0), "emitted_value starts at 0 when driving begins fresh");

    const double dt = 0.02;  // 50 Hz, matches --rate-hz's typical value.
    double t = 0.0;
    double expected = 0.0;
    for (int i = 1; i <= 24; ++i) {
        t += dt;
        const vesc::TeleopMotorAction a = core.step(t);
        expected += 2000.0 * dt;  // 24*40 == 960, never reaches 1000 within this loop.
        ok &= check_true(near_eq(a.value, expected, 1e-6), "ramp tick " + std::to_string(i) + ": emitted == " +
                                                                 std::to_string(expected) + " (got " +
                                                                 std::to_string(a.value) + ")");
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kRpm, "action type stays kRpm while ramping");
    }
    ok &= check_true(near_eq(expected, 960.0, 1e-6) && !near_eq(core.emitted_value(), 1000.0),
                      "after 24 ticks at 40/tick, emitted is 960 -- not yet at the 1000 target");

    // Tick 25 reaches exactly the target (960 + 40 == 1000).
    t += dt;
    const vesc::TeleopMotorAction a25 = core.step(t);
    ok &= check_true(near_eq(a25.value, 1000.0, 1e-6), "tick 25 reaches exactly the target 1000");

    // Further ticks hold at the target -- clamped, no overshoot.
    t += dt;
    const vesc::TeleopMotorAction a26 = core.step(t);
    ok &= check_true(near_eq(a26.value, 1000.0, 1e-6), "ramp holds at the target on subsequent ticks, no overshoot");

    return ok;
}

bool test_teleop_core_ramp_through_zero_reversal() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.ramp_enabled = true;
    cfg.erpm_ramp = 1000.0;
    cfg.erpm_mag_default = 1000.0;
    vesc::TeleopCore core(cfg);

    core.step(0.0);
    core.handle_key('E', 0.0);
    core.handle_key('w', 0.0);

    core.step(0.1);                              // emitted: 0 -> 100.
    const vesc::TeleopMotorAction a1 = core.step(0.2);  // emitted: 100 -> 200.
    ok &= check_true(near_eq(a1.value, 200.0, 1e-6), "ramped up to 200 before reversing");

    core.handle_key('s', 0.2);  // reverse WHILE driving -- must NOT reset emitted_value_ to 0.
    ok &= check_true(near_eq(core.emitted_value(), 200.0),
                      "emitted_value is unchanged by the reversal keypress itself (only step() moves it)");
    ok &= check_true(core.is_driving() && core.direction() == vesc::DriveDirection::kReverse,
                      "direction flips to reverse, still driving");

    const vesc::TeleopMotorAction a2 = core.step(0.3);  // target now -1000; emitted: 200 -> 100.
    ok &= check_true(near_eq(a2.value, 100.0, 1e-6), "one tick after reversal: emitted decreases toward the new (negative) target");

    const vesc::TeleopMotorAction a3 = core.step(0.4);  // emitted: 100 -> 0.
    ok &= check_true(near_eq(a3.value, 0.0, 1e-6), "emitted reaches EXACTLY 0 while slewing through the reversal");

    const vesc::TeleopMotorAction a4 = core.step(0.5);  // emitted: 0 -> -100.
    ok &= check_true(near_eq(a4.value, -100.0, 1e-6),
                      "emitted continues past 0 into negative territory -- confirms it slews THROUGH zero rather "
                      "than resetting to 0 and ramping back up");
    ok &= check_true(a4.type == vesc::TeleopMotorAction::Type::kRpm, "action type stays kRpm throughout the reversal");

    // Continue to the new (reversed) target.
    double t = 0.5;
    double expected = -100.0;
    while (!near_eq(expected, -1000.0, 1e-6)) {
        t += 0.1;
        expected = std::max(-1000.0, expected - 100.0);
        const vesc::TeleopMotorAction a = core.step(t);
        ok &= check_true(near_eq(a.value, expected, 1e-6), "continues descending toward -1000 (t=" + std::to_string(t) + ")");
    }
    ok &= check_true(near_eq(core.emitted_value(), -1000.0, 1e-6), "eventually reaches exactly the reversed target -1000");

    return ok;
}

bool test_teleop_core_ramp_mid_ramp_target_change() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.ramp_enabled = true;
    cfg.duty_ramp = 1.0;
    cfg.duty_mag_default = 0.5;
    cfg.max_duty = 2.0;  // synthetic, generous ceiling so 0.8 below isn't clamped away.
    vesc::TeleopCore core(cfg);

    core.step(0.0);
    core.handle_key('w', 0.0);  // duty mode (the default), target 0.5.

    core.step(0.1);                              // emitted: 0 -> 0.1.
    const vesc::TeleopMotorAction a1 = core.step(0.2);  // emitted: 0.1 -> 0.2.
    ok &= check_true(near_eq(a1.value, 0.2, 1e-6), "ramping toward the original 0.5 target");

    // Change the magnitude (target) mid-ramp via direct digit entry --
    // the slew must continue from wherever emitted_value_ currently is
    // (0.2), NOT reset to 0 and restart.
    for (char c : std::string("0.8")) core.handle_key(c, 0.2);
    core.handle_key('\r', 0.2);
    ok &= check_true(near_eq(core.magnitude(), 0.8), "magnitude (target) updated to 0.8");
    ok &= check_true(near_eq(core.emitted_value(), 0.2), "emitted_value is UNCHANGED by the magnitude edit itself (only step() moves it)");

    const vesc::TeleopMotorAction a2 = core.step(0.3);  // dt=0.1; target 0.8; emitted: 0.2 -> 0.3.
    ok &= check_true(near_eq(a2.value, 0.3, 1e-6), "slew continues from 0.2 toward the NEW target 0.8, not reset to 0");

    const vesc::TeleopMotorAction a3 = core.step(0.4);  // emitted: 0.3 -> 0.4.
    ok &= check_true(near_eq(a3.value, 0.4, 1e-6), "still climbing toward 0.8");

    return ok;
}

bool test_teleop_core_ramp_safety_paths_bypass() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.ramp_enabled = true;
    cfg.erpm_ramp = 100.0;  // slow, so a mid-ramp (not-yet-at-target) tick is easy to catch.
    cfg.erpm_mag_default = 1000.0;
    cfg.deadman_ms = 200.0;
    cfg.brake_hold_ms = 300.0;

    // (a) stop key mid-ramp.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('E', 0.0);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction mid = core.step(0.1);  // dt=0.1, max_delta=10 -- well short of 1000.
        ok &= check_true(mid.type == vesc::TeleopMotorAction::Type::kRpm && mid.value < 1000.0 - 1e-6,
                          "(stop) confirmed mid-ramp (not yet at target) before pressing stop");
        core.handle_key(' ', 0.1);
        const vesc::TeleopMotorAction after = core.step(0.1);
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake,
                          "(stop) brakes IMMEDIATELY mid-ramp, not a ramped-down value");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(stop) brake value is brake_amps, unrelated to the ramp state");
    }

    // (b) deadman mid-ramp.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('E', 0.0);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction mid = core.step(0.1);
        ok &= check_true(mid.type == vesc::TeleopMotorAction::Type::kRpm && mid.value < 1000.0 - 1e-6,
                          "(deadman) confirmed mid-ramp before the deadman window elapses");
        const vesc::TeleopMotorAction after = core.step(0.2);  // now_s - last_key_time(0.0) == 0.2 >= deadman_ms/1000=0.2.
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake,
                          "(deadman) brakes IMMEDIATELY mid-ramp, not a further-ramped value");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(deadman) brake value is brake_amps");
    }

    // (c) current-abort mid-ramp.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('E', 0.0);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction mid = core.step(0.1);
        ok &= check_true(mid.type == vesc::TeleopMotorAction::Type::kRpm && mid.value < 1000.0 - 1e-6,
                          "(current-abort) confirmed mid-ramp before the abort trips");
        vesc::VescValues v;
        v.current_motor = 50.0;  // well over the default current_abort (8.0A).
        core.feed_telemetry(v, 0.11);
        const vesc::TeleopMotorAction after = core.step(0.12);
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake, "(current-abort) brakes IMMEDIATELY mid-ramp");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(current-abort) brake value is brake_amps");
    }

    // (d) fault-abort mid-ramp.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('E', 0.0);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction mid = core.step(0.1);
        ok &= check_true(mid.type == vesc::TeleopMotorAction::Type::kRpm && mid.value < 1000.0 - 1e-6,
                          "(fault-abort) confirmed mid-ramp before the abort trips");
        vesc::VescValues v;
        v.fault = 4;
        core.feed_telemetry(v, 0.11);
        const vesc::TeleopMotorAction after = core.step(0.12);
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake, "(fault-abort) brakes IMMEDIATELY mid-ramp");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(fault-abort) brake value is brake_amps");
    }

    return ok;
}

bool test_teleop_core_ramp_rate_entry_commit_clamp_cancel() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    // Plain digit entry (no 'Z') still targets magnitude, unaffected by
    // ramp-rate entry existing at all.
    for (char c : std::string("0.05")) core.handle_key(c, 0.0);
    core.handle_key('\r', 0.0);
    ok &= check_true(near_eq(core.magnitude(), 0.05), "plain digit-entry (no 'Z') still commits to magnitude, not ramp rate");
    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.1), "duty ramp rate is untouched by a plain magnitude entry (still its default)");

    // 'Z' arms ramp-rate entry (duty mode, the default).
    core.handle_key('Z', 0.1);
    ok &= check_true(core.digit_entry_is_ramp_rate(), "'Z' arms ramp-rate entry");
    for (char c : std::string("0.5")) core.handle_key(c, 0.1);
    core.handle_key('\r', 0.1);
    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.5), "'Z'+digits+ENTER commits to duty_ramp_rate()");
    ok &= check_true(!core.digit_entry_is_ramp_rate(), "ramp-rate-entry arming clears after a commit");
    ok &= check_true(near_eq(core.magnitude(), 0.05), "the earlier magnitude is untouched by the ramp-rate commit");

    // Clamp: above the duty ramp ceiling (1.0).
    core.handle_key('Z', 0.2);
    for (char c : std::string("5")) core.handle_key(c, 0.2);
    core.handle_key('\r', 0.2);
    ok &= check_true(near_eq(core.duty_ramp_rate(), 1.0), "duty ramp rate clamps to its ceiling (1.0) on an over-large entry");

    // Clamp: below the duty ramp floor (0.001) -- NOT floored at 0 like
    // magnitude entry; a 0 ramp rate would never reach its target.
    core.handle_key('Z', 0.3);
    for (char c : std::string("0.0000001")) core.handle_key(c, 0.3);
    core.handle_key('\r', 0.3);
    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.001), "duty ramp rate clamps to its floor (0.001) on an under-small entry");

    // ESC cancels an armed rate entry without touching the current rate.
    core.handle_key('Z', 0.4);
    for (char c : std::string("999")) core.handle_key(c, 0.4);
    core.handle_key(0x1B, 0.4);
    ok &= check_true(!core.digit_entry_is_ramp_rate(), "ESC cancels ramp-rate-entry arming");
    ok &= check_true(core.digit_buffer().empty(), "ESC clears the buffer");
    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.001), "ESC leaves the rate at its prior (clamped) value, uncommitted");

    // erpm mode has its own independent bounds ([10, 20000]).
    core.handle_key('E', 0.5);
    core.handle_key('Z', 0.5);
    for (char c : std::string("50000")) core.handle_key(c, 0.5);
    core.handle_key('\r', 0.5);
    ok &= check_true(near_eq(core.erpm_ramp_rate(), 20000.0), "erpm ramp rate clamps to its ceiling (20000) on an over-large entry");

    core.handle_key('Z', 0.6);
    for (char c : std::string("1")) core.handle_key(c, 0.6);
    core.handle_key('\r', 0.6);
    ok &= check_true(near_eq(core.erpm_ramp_rate(), 10.0), "erpm ramp rate clamps to its floor (10) on an under-small entry");

    return ok;
}

bool test_teleop_core_ramp_per_mode_rate_independence() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.1), "default duty ramp rate");
    ok &= check_true(near_eq(core.erpm_ramp_rate(), 500.0), "default erpm ramp rate");

    // Change duty's rate via 'Z' entry -- erpm's own rate must be untouched.
    core.handle_key('Z', 0.0);
    for (char c : std::string("0.1")) core.handle_key(c, 0.0);
    core.handle_key('\r', 0.0);
    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.1), "duty ramp rate updated");
    ok &= check_true(near_eq(core.erpm_ramp_rate(), 500.0), "erpm ramp rate unaffected by duty's own edit");

    // Switch to erpm mode and change ITS rate -- duty's edited rate must persist.
    core.handle_key('E', 0.1);
    core.handle_key('Z', 0.1);
    for (char c : std::string("2000")) core.handle_key(c, 0.1);
    core.handle_key('\r', 0.1);
    ok &= check_true(near_eq(core.erpm_ramp_rate(), 2000.0), "erpm ramp rate updated");
    ok &= check_true(near_eq(core.duty_ramp_rate(), 0.1), "duty ramp rate retains its own earlier edit, unaffected by erpm's own edit");

    // ramp_rate() reflects whichever mode is CURRENT.
    ok &= check_true(near_eq(core.ramp_rate(), 2000.0), "ramp_rate() reflects the current mode (erpm)");
    core.handle_key('D', 0.2);
    ok &= check_true(near_eq(core.ramp_rate(), 0.1), "ramp_rate() reflects the current mode after switching back to duty");

    return ok;
}

bool test_teleop_core_ramp_mode_switch_stops_first() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.ramp_enabled = true;
    cfg.erpm_ramp = 100.0;
    cfg.erpm_mag_default = 1000.0;
    vesc::TeleopCore core(cfg);

    core.step(0.0);
    core.handle_key('w', 0.0);  // driving forward in duty mode (the default).
    core.step(0.05);
    ok &= check_true(core.is_driving(), "driving before the mode switch");

    core.handle_key('E', 0.05);  // switch mode WHILE driving.
    ok &= check_true(core.is_braking(), "mode switch while driving triggers an immediate stop (kBraking)");
    ok &= check_true(core.stop_reason() == "mode_switch", "stop reason distinguishes a mode-switch stop from a manual stop key");
    ok &= check_true(core.mode() == vesc::TeleopMode::kErpm, "the mode itself DID switch, despite (and alongside) the stop");

    const vesc::TeleopMotorAction a = core.step(0.05);
    ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kBrake,
                      "next step() after a mode-switch stop brakes, exactly like a manual stop");

    // Driving must be explicitly re-engaged (w/s) -- it does not resume
    // on its own once the brake-hold elapses.
    const vesc::TeleopMotorAction after_hold = core.step(0.05 + cfg.brake_hold_ms / 1000.0);
    ok &= check_true(after_hold.type == vesc::TeleopMotorAction::Type::kNone,
                      "after the brake hold elapses, idle -- not automatically driving again");

    return ok;
}

// ---------------------------------------------------------------------
// (o) TeleopCore SPEED GOVERNOR mode -- pure logic, no I/O: drives
// TeleopCore's own key/mode wiring around SpeedGovernor
// (handle_key/step/feed_telemetry) against SyntheticSpeedPlant (now
// defined up in section (g), since the new SpeedGovernor/DriverCore-v2
// sections above also need it).
// ---------------------------------------------------------------------

bool test_teleop_core_speed_mode_separate_magnitude_and_step() {
    bool ok = true;
    vesc::TeleopCore core(make_test_teleop_config());

    ok &= check_true(near_eq(core.speed_magnitude(), 1000.0), "default speed-mode magnitude (target erpm)");

    core.handle_key('V', 0.0);
    ok &= check_true(core.mode() == vesc::TeleopMode::kSpeed, "'V' switches to speed mode");
    ok &= check_true(near_eq(core.magnitude(), 1000.0), "magnitude() reflects speed_magnitude() while in speed mode");

    core.handle_key('+', 0.0);  // erpm_step stepping, per spec (not a separate speed_step).
    ok &= check_true(near_eq(core.speed_magnitude(), 1100.0), "'+' in speed mode steps by erpm_step (100)");

    core.handle_key('E', 0.1);  // switch to erpm mode -- its own magnitude must be untouched.
    ok &= check_true(near_eq(core.magnitude(), 1000.0), "erpm mode's own magnitude is unaffected by speed mode's edit");

    core.handle_key('V', 0.2);  // back to speed mode -- retains its own earlier edit.
    ok &= check_true(near_eq(core.magnitude(), 1100.0), "speed mode retains its own earlier magnitude edit");

    // Digit entry clamps to max_erpm, same ceiling as erpm mode.
    for (char c : std::string("999999")) core.handle_key(c, 0.3);
    core.handle_key('\r', 0.3);
    ok &= check_true(near_eq(core.magnitude(), 6000.0),
                      "digit-entry commit clamps speed-mode magnitude to max_erpm (default 6000)");

    return ok;
}

bool test_teleop_core_speed_governor_converges_and_respects_slew() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    cfg.duty_ramp = 0.05;
    // speed_mag_default, NOT erpm_mag_default -- TeleopConfig has a
    // SEPARATE default field for speed mode's own target erpm (a real
    // bug caught by an earlier test run: setting the wrong field left
    // the target silently at speed_mag_default's own 1000.0 default,
    // which only failed to expose itself here because that happens to
    // be the value this test wants anyway).
    cfg.speed_mag_default = 1000.0;
    cfg.deadman_ms = 30000.0;  // long enough not to interfere with this test's ~10s run.
    vesc::TeleopCore core(cfg);
    SyntheticSpeedPlant plant;
    plant.v_in = 8.0;

    core.step(0.0);
    core.handle_key('V', 0.0);
    core.handle_key('w', 0.0);

    const double dt = 0.02;
    const double max_delta = cfg.duty_ramp * dt + 1e-9;
    double t = 0.0;
    double prev_duty = 0.0;
    for (int i = 0; i < 500; ++i) {  // 10s.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        ok &= check_true(std::fabs(a.value - prev_duty) <= max_delta,
                          "tick " + std::to_string(i) + ": duty change per tick never exceeds the configured "
                          "slew limit (delta=" + std::to_string(a.value - prev_duty) + ", limit=" +
                              std::to_string(max_delta) + ")");
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kDuty, "speed mode always emits kDuty");
        prev_duty = a.value;
        plant.step(a.value, dt);
    }
    ok &= check_true(near_eq(plant.erpm, 1000.0, 100.0),
                      "converges near the 1000 target within tolerance (got " + std::to_string(plant.erpm) + ")");

    return ok;
}

bool test_teleop_core_speed_governor_voltage_independence() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    // Fast (not the TeleopConfig default 0.1/s) so the governor's own
    // MANDATORY output slew (see step_speed_governor()) doesn't become
    // the dominant time constant relative to the plant's tau=0.2s --
    // with the default slow default here, the slew-limited ramp-up lags
    // far enough behind the PI's own error signal that the integrator
    // winds up well past what's needed, causing exactly the kind of
    // overshoot this test is trying to rule OUT (found via a real test
    // failure: converged to ~1152 instead of within 1000+-80).
    cfg.duty_ramp = 10.0;
    cfg.speed_mag_default = 1000.0;  // NOT erpm_mag_default -- see the other speed tests' own comment on this.
    cfg.deadman_ms = 30000.0;
    vesc::TeleopCore core(cfg);
    SyntheticSpeedPlant plant;
    plant.v_in = 8.2;

    core.step(0.0);
    core.handle_key('V', 0.0);
    core.handle_key('w', 0.0);

    const double dt = 0.02;
    double t = 0.0;
    for (int i = 0; i < 250; ++i) {  // 5s to converge at 8.2V.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
    }
    ok &= check_true(near_eq(plant.erpm, 1000.0, 80.0),
                      "converges near target at the initial 8.2V (got " + std::to_string(plant.erpm) + ")");

    // Battery droop mid-run: 8.2V -> 7.4V (a real, plausible sag under
    // load) -- this is the whole motivating claim: the governor's
    // feedback must absorb this, keeping erpm converged at the SAME
    // target despite the plant's own steady-state duty-to-erpm ratio
    // having just changed underneath it.
    plant.v_in = 7.4;
    for (int i = 0; i < 250; ++i) {  // another 5s to re-settle at the new voltage.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
    }
    ok &= check_true(near_eq(plant.erpm, 1000.0, 80.0),
                      "stays converged near the SAME target after the voltage drop -- feedback absorbs the "
                      "voltage change (got " + std::to_string(plant.erpm) + ")");

    return ok;
}

bool test_teleop_core_speed_governor_reversal_through_zero() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    cfg.duty_ramp = 10.0;  // see voltage-independence test's own comment -- avoids slew-driven integrator windup/overshoot.
    cfg.speed_mag_default = 800.0;  // NOT erpm_mag_default -- see the other speed tests' own comment on this.
    cfg.deadman_ms = 30000.0;
    vesc::TeleopCore core(cfg);
    SyntheticSpeedPlant plant;
    plant.v_in = 8.0;

    core.step(0.0);
    core.handle_key('V', 0.0);
    core.handle_key('w', 0.0);  // target +800.

    const double dt = 0.02;
    double t = 0.0;
    for (int i = 0; i < 250; ++i) {  // 5s to converge to +800.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
    }
    ok &= check_true(near_eq(plant.erpm, 800.0, 80.0), "converges near +800 before reversing (got " + std::to_string(plant.erpm) + ")");

    core.handle_key('s', t);  // reverse -- target now -800.
    for (int i = 0; i < 250; ++i) {  // another 5s to converge to -800.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kDuty, "speed mode ALWAYS emits kDuty, even mid-reversal");
    }
    ok &= check_true(near_eq(plant.erpm, -800.0, 80.0), "converges near -800 after reversing (got " + std::to_string(plant.erpm) + ")");

    return ok;
}

bool test_teleop_core_speed_governor_ff_disabled_still_converges() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 0.0;  // disabled -- pure PI, per spec "0 disables ff".
    cfg.duty_ramp = 10.0;     // see voltage-independence test's own comment -- avoids slew-driven integrator windup/overshoot.
    // 1200, NOT 600 (this test's own first draft): speed_mag_default is
    // the correct field (see the other speed tests' own comment on
    // that), but ALSO -- the duty needed for 600 erpm at this plant
    // (600/(4400*8) ~= 0.017) falls INSIDE the plant's own stall_duty
    // (0.02) dead zone, i.e. 600 erpm is not actually reachable at
    // steady state at all (the plant jumps from 0 to ~704 erpm right at
    // the stall boundary) -- a real modeling bug in this test caught by
    // an earlier run's wildly-off failure. 1200 needs duty ~= 0.034,
    // comfortably clear of the dead zone.
    cfg.speed_mag_default = 1200.0;
    cfg.deadman_ms = 30000.0;
    vesc::TeleopCore core(cfg);
    SyntheticSpeedPlant plant;
    plant.v_in = 8.0;

    core.step(0.0);
    core.handle_key('V', 0.0);
    core.handle_key('w', 0.0);

    const double dt = 0.02;
    double t = 0.0;
    for (int i = 0; i < 750; ++i) {  // more time budget -- pure PI (no ff) converges slower.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
    }
    ok &= check_true(near_eq(plant.erpm, 1200.0, 100.0),
                      "converges near target even with ff disabled (pure PI, slower) -- got " + std::to_string(plant.erpm));

    return ok;
}

bool test_teleop_core_speed_governor_anti_windup() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.max_duty = 0.05;
    cfg.duty_ramp = 10.0;  // fast slew so it doesn't mask the anti-windup effect under test.
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    cfg.deadman_ms = 30000.0;
    vesc::TeleopCore core(cfg);
    SyntheticSpeedPlant plant;
    plant.v_in = 8.0;

    core.step(0.0);
    core.handle_key('V', 0.0);
    for (char c : std::string("100000")) core.handle_key(c, 0.0);  // absurd, unreachable target.
    core.handle_key('\r', 0.0);
    core.handle_key('w', 0.0);

    const double dt = 0.02;
    double t = 0.0;
    for (int i = 0; i < 100; ++i) {  // 2s of sustained saturation.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
    }
    ok &= check_true(near_eq(core.emitted_value(), cfg.max_duty, 1e-6),
                      "pinned at max_duty during sustained saturation on an unreachable target");
    ok &= check_true(plant.erpm > 1500.0,
                      "plant has settled near its own steady-state erpm at max_duty before the switch (got " +
                          std::to_string(plant.erpm) + ")");

    // Switch to a modest, ACHIEVABLE target.
    for (char c : std::string("500")) core.handle_key(c, t);
    core.handle_key('\r', t);

    bool converged = false;
    int ticks_to_converge = -1;
    for (int i = 0; i < 500 && !converged; ++i) {  // up to 10 more seconds.
        t += dt;
        vesc::VescValues v;
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        plant.step(a.value, dt);
        if (std::fabs(plant.erpm - 500.0) <= 50.0) {
            converged = true;
            ticks_to_converge = i;
        }
    }
    ok &= check_true(converged, "converges near the new, achievable target (500) after switching down from saturation");
    // Anti-windup sanity bound: with the integrator correctly frozen
    // during the earlier saturation, this settles within a few seconds
    // (a handful of the plant's own tau=0.2s); a wound-up integrator
    // would drag this out MUCH longer (or never converge within this
    // budget at all) -- this is the "recovery time is bounded" proof.
    ok &= check_true(ticks_to_converge >= 0 && ticks_to_converge <= 250,
                      "converges within a plausible, BOUNDED number of ticks (anti-windup, not dragged out by a "
                      "wound-up integrator) -- got " + std::to_string(ticks_to_converge) + " ticks (~" +
                          std::to_string(ticks_to_converge * dt) + "s)");

    return ok;
}

// Checks that the VERY NEXT step() (right after re-engaging driving in
// speed mode with a freshly-seeded erpm/v_in reading) matches the
// FF+P+(exactly one tick of I) formula -- i.e. ZERO carried-over
// integrator contribution from any earlier windup. A stale, un-reset
// integrator would add a large extra term here (see the four call
// sites below, each of which first deliberately built up real windup
// via a sustained, NOT-saturated tracking error before triggering the
// reset path under test).
bool check_speed_governor_fresh_start(vesc::TeleopCore* core, double now_s, double dt, double target_erpm,
                                       double measured_erpm, double v_in, double kp, double ki, double ff_gain,
                                       const std::string& label) {
    bool ok = true;
    const vesc::TeleopMotorAction a = core->step(now_s);
    const double error = target_erpm - measured_erpm;
    const double effective_v_in = std::max(v_in, 6.0);
    const double expected_ff = (ff_gain > 1e-9) ? target_erpm / (ff_gain * effective_v_in) : 0.0;
    const double expected = expected_ff + kp * error + ki * error * dt;
    ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kDuty, label + ": still emits kDuty after re-engaging");
    ok &= check_true(near_eq(a.value, expected, 1e-4),
                      label + ": first tick after re-engaging matches FF+P+(1 tick I) with NO carried-over "
                              "integrator windup (expected " +
                          std::to_string(expected) + ", got " + std::to_string(a.value) + ")");
    return ok;
}

bool test_teleop_core_speed_governor_reset_on_stop_deadman_abort_modeswitch() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.duty_ramp = 100.0;  // fast enough that emitted_value_ tracks duty_cmd almost exactly within one tick.
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    cfg.speed_mag_default = 1000.0;  // NOT erpm_mag_default -- see the other speed tests' own comment on this.
    cfg.deadman_ms = 100000.0;  // effectively disabled for the non-deadman sub-cases below.
    cfg.brake_hold_ms = 50.0;

    // Persistent, NOT-saturated tracking error for a while, to
    // accumulate real integrator windup -- deliberately far from
    // max_duty so this isolates "does X reset the integrator" from the
    // separate anti-windup-under-saturation behavior tested above.
    auto build_windup = [&](vesc::TeleopCore* core, double* t) {
        for (int i = 0; i < 50; ++i) {
            *t += 0.02;
            vesc::VescValues v;
            v.erpm = 200.0;  // held well below the 1000 target.
            v.v_in = 8.0;
            core->feed_telemetry(v, *t);
            core->step(*t);
        }
    };

    // (a) stop key.
    {
        vesc::TeleopCore core(cfg);
        double t = 0.0;
        core.step(t);
        core.handle_key('V', t);
        core.handle_key('w', t);
        build_windup(&core, &t);

        core.handle_key(' ', t);
        t += 0.02;
        core.step(t);
        t += cfg.brake_hold_ms / 1000.0;
        core.step(t);  // idle.

        core.handle_key('w', t);
        vesc::VescValues v_fresh;
        v_fresh.erpm = 0.0;
        v_fresh.v_in = 8.0;
        core.feed_telemetry(v_fresh, t);
        t += 0.02;
        ok &= check_speed_governor_fresh_start(&core, t, 0.02, 1000.0, 0.0, 8.0, cfg.speed_kp, cfg.speed_ki,
                                                cfg.speed_ff_gain, "(stop key)");
    }

    // (b) deadman expiry -- its own short-deadman config, refreshed via
    // 'w' throughout the windup-building phase so ONLY the deliberate
    // silence afterward triggers it.
    {
        vesc::TeleopConfig cfg_dm = cfg;
        cfg_dm.deadman_ms = 300.0;
        vesc::TeleopCore core(cfg_dm);
        double t = 0.0;
        core.step(t);
        core.handle_key('V', t);
        core.handle_key('w', t);
        for (int i = 0; i < 50; ++i) {
            t += 0.02;
            core.handle_key('w', t);  // refresh the deadman (already driving forward -- a no-op state change).
            vesc::VescValues v;
            v.erpm = 200.0;
            v.v_in = 8.0;
            core.feed_telemetry(v, t);
            core.step(t);
        }

        t += cfg_dm.deadman_ms / 1000.0 + 0.05;  // go silent past the deadman.
        core.step(t);
        ok &= check_true(core.is_braking() && core.stop_reason() == "deadman", "(deadman) actually fired as expected");
        t += cfg_dm.brake_hold_ms / 1000.0;
        core.step(t);  // idle.

        core.handle_key('w', t);
        vesc::VescValues v_fresh;
        v_fresh.erpm = 0.0;
        v_fresh.v_in = 8.0;
        core.feed_telemetry(v_fresh, t);
        t += 0.02;
        ok &= check_speed_governor_fresh_start(&core, t, 0.02, 1000.0, 0.0, 8.0, cfg_dm.speed_kp, cfg_dm.speed_ki,
                                                cfg_dm.speed_ff_gain, "(deadman)");
    }

    // (c) current-abort.
    {
        vesc::TeleopCore core(cfg);
        double t = 0.0;
        core.step(t);
        core.handle_key('V', t);
        core.handle_key('w', t);
        build_windup(&core, &t);

        t += 0.02;
        vesc::VescValues v_abort;
        v_abort.current_motor = 50.0;
        v_abort.v_in = 8.0;
        core.feed_telemetry(v_abort, t);
        ok &= check_true(core.is_aborted(), "(abort) current-abort trips as expected before checking the reset");
        core.step(t);  // keeps last_step_time_s_ in sync with `t`, exactly like the other 3 sub-cases' own
                        // intermediate step() calls -- omitting this left a stale dt in the final check below
                        // (a real test bug caught by an earlier run: it silently doubled the expected dt).

        core.handle_key('c', t);
        core.handle_key('w', t);
        vesc::VescValues v_fresh;
        v_fresh.erpm = 0.0;
        v_fresh.v_in = 8.0;
        core.feed_telemetry(v_fresh, t);
        t += 0.02;
        ok &= check_speed_governor_fresh_start(&core, t, 0.02, 1000.0, 0.0, 8.0, cfg.speed_kp, cfg.speed_ki,
                                                cfg.speed_ff_gain, "(current-abort)");
    }

    // (d) mode switch (away and back).
    {
        vesc::TeleopCore core(cfg);
        double t = 0.0;
        core.step(t);
        core.handle_key('V', t);
        core.handle_key('w', t);
        build_windup(&core, &t);

        core.handle_key('D', t);  // switch away WHILE driving -- stops (mode_switch) AND resets.
        ok &= check_true(core.is_braking() && core.stop_reason() == "mode_switch",
                          "(mode-switch) switching away from speed mode while driving stops first");
        t += 0.02;
        core.step(t);
        t += cfg.brake_hold_ms / 1000.0;
        core.step(t);  // idle.

        core.handle_key('V', t);  // switch BACK to speed mode (idle -- no stop needed, but still resets).
        core.handle_key('w', t);
        vesc::VescValues v_fresh;
        v_fresh.erpm = 0.0;
        v_fresh.v_in = 8.0;
        core.feed_telemetry(v_fresh, t);
        t += 0.02;
        ok &= check_speed_governor_fresh_start(&core, t, 0.02, 1000.0, 0.0, 8.0, cfg.speed_kp, cfg.speed_ki,
                                                cfg.speed_ff_gain, "(mode-switch)");
    }

    return ok;
}

bool test_teleop_core_speed_governor_safety_paths_bypass() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    cfg.speed_mag_default = 1000.0;  // NOT erpm_mag_default -- see the other speed tests' own comment on this.
    cfg.deadman_ms = 200.0;
    cfg.brake_hold_ms = 300.0;

    // (a) stop key mid-governor.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('V', 0.0);
        core.handle_key('w', 0.0);
        vesc::VescValues v;
        v.erpm = 200.0;
        v.v_in = 8.0;
        core.feed_telemetry(v, 0.02);
        const vesc::TeleopMotorAction mid = core.step(0.02);
        ok &= check_true(mid.type == vesc::TeleopMotorAction::Type::kDuty, "(stop) governor is actively commanding duty before the stop");
        core.handle_key(' ', 0.02);
        const vesc::TeleopMotorAction after = core.step(0.02);
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake, "(stop) brakes IMMEDIATELY, not a governor wind-down");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(stop) brake value is brake_amps");
    }

    // (b) deadman mid-governor.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('V', 0.0);
        core.handle_key('w', 0.0);
        vesc::VescValues v;
        v.erpm = 200.0;
        v.v_in = 8.0;
        core.feed_telemetry(v, 0.02);
        core.step(0.02);
        const vesc::TeleopMotorAction after = core.step(0.2);  // now_s - last_key_time(0.0) == 0.2 >= deadman_ms/1000.
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake, "(deadman) brakes IMMEDIATELY mid-governor");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(deadman) brake value is brake_amps");
    }

    // (c) current-abort mid-governor.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('V', 0.0);
        core.handle_key('w', 0.0);
        vesc::VescValues v;
        v.erpm = 200.0;
        v.v_in = 8.0;
        core.feed_telemetry(v, 0.02);
        core.step(0.02);
        vesc::VescValues v_abort;
        v_abort.current_motor = 50.0;
        v_abort.v_in = 8.0;
        core.feed_telemetry(v_abort, 0.03);
        const vesc::TeleopMotorAction after = core.step(0.04);
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake, "(current-abort) brakes IMMEDIATELY mid-governor");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(current-abort) brake value is brake_amps");
    }

    // (d) fault-abort mid-governor.
    {
        vesc::TeleopCore core(cfg);
        core.step(0.0);
        core.handle_key('V', 0.0);
        core.handle_key('w', 0.0);
        vesc::VescValues v;
        v.erpm = 200.0;
        v.v_in = 8.0;
        core.feed_telemetry(v, 0.02);
        core.step(0.02);
        vesc::VescValues v_fault;
        v_fault.fault = 4;
        v_fault.v_in = 8.0;
        core.feed_telemetry(v_fault, 0.03);
        const vesc::TeleopMotorAction after = core.step(0.04);
        ok &= check_true(after.type == vesc::TeleopMotorAction::Type::kBrake, "(fault-abort) brakes IMMEDIATELY mid-governor");
        ok &= check_true(near_eq(after.value, cfg.brake_amps), "(fault-abort) brake value is brake_amps");
    }

    return ok;
}

// ---------------------------------------------------------------------
// (u) TeleopCore NEW KEY MAP + drive_invert (whole-vehicle sign) -- pure
// logic, no I/O. Added alongside the WASD key remap: (1) every new key
// does the documented thing and no key does two things, (2) every
// RETIRED (pre-WASD) key is inert and yields a non-empty retired_hint
// naming its replacement, (3) drive_invert=true negates the emitted
// command in duty/erpm modes, and (4) the speed governor still CONVERGES
// under drive_invert with correct feedback polarity -- see
// test_teleop_core_drive_invert_speed_governor_converges()'s own comment
// for why this must be a real closed-loop convergence proof, not a
// one-tick sign check (an incomplete wiring -- drive_sign() applied at
// only ONE of its two call sites -- would make this diverge instead).
// ---------------------------------------------------------------------

bool test_teleop_core_new_key_map_and_retired_keys() {
    bool ok = true;

    // Every RETIRED (pre-WASD) key is inert (no drive/steer/mode/quit
    // side effect) and yields a non-empty retired_hint naming its
    // replacement.
    struct RetiredCase {
        char key;
        std::string expect_hint_substr;  // the new key it points to, quoted.
    };
    const std::vector<RetiredCase> retired = {
        {'f', "'w'"}, {'b', "'s'"}, {'r', "'s'"}, {'j', "'a'"},
        {'l', "'d'"}, {'e', "'E'"}, {'v', "'V'"}, {'A', "'Z'"},
    };
    for (const auto& rc : retired) {
        vesc::TeleopCore core(make_test_teleop_config());
        const double pos_before = core.steering_position();
        const vesc::TeleopMode mode_before = core.mode();
        const vesc::KeyEvent ev = core.handle_key(rc.key, 0.0);
        const std::string label = std::string("retired key '") + rc.key + "'";
        ok &= check_true(!ev.retired_hint.empty(), label + " yields a non-empty retired_hint");
        ok &= check_true(ev.retired_hint.find(rc.expect_hint_substr) != std::string::npos,
                          label + " hint names its replacement " + rc.expect_hint_substr + " (got '" +
                              ev.retired_hint + "')");
        ok &= check_true(!core.is_driving(), label + " never starts driving");
        ok &= check_true(near_eq(core.steering_position(), pos_before), label + " never moves steering");
        ok &= check_true(core.mode() == mode_before, label + " never changes mode");
        ok &= check_true(ev.type == vesc::KeyEventType::kNone, label + " does not raise kQuit");
    }

    // SPACE and 'x' BOTH stop, via the exact same immediate brake path.
    {
        vesc::TeleopCore core(make_test_teleop_config());
        core.handle_key('w', 0.0);
        core.step(0.0);
        core.handle_key(' ', 0.1);
        ok &= check_true(core.is_braking() && core.stop_reason() == "stop", "SPACE stops (reason \"stop\")");
    }
    {
        vesc::TeleopCore core(make_test_teleop_config());
        core.handle_key('w', 0.0);
        core.step(0.0);
        core.handle_key('x', 0.1);
        ok &= check_true(core.is_braking() && core.stop_reason() == "stop", "'x' stops (reason \"stop\"), identically to SPACE");
    }

    // 'w'/'s' drive forward/backward, and touch ONLY drive state.
    {
        vesc::TeleopCore core(make_test_teleop_config());
        core.handle_key('w', 0.0);
        ok &= check_true(core.is_driving() && core.direction() == vesc::DriveDirection::kForward, "'w' drives forward");
        ok &= check_true(core.mode() == vesc::TeleopMode::kDuty, "'w' does not change mode");
        core.handle_key('s', 0.1);
        ok &= check_true(core.is_driving() && core.direction() == vesc::DriveDirection::kReverse, "'s' drives backward");
    }

    // 'a'/'d' steer, and touch ONLY steering state (no key does two things).
    {
        vesc::TeleopCore core(make_test_teleop_config());
        const double before = core.steering_position();
        core.handle_key('a', 0.0);
        ok &= check_true(core.steering_position() < before, "'a' steers left (decreases position, steer_invert=false)");
        ok &= check_true(!core.is_driving() && core.mode() == vesc::TeleopMode::kDuty,
                          "'a' does not touch drive state or mode -- steering only");
        core.handle_key('d', 0.1);
        ok &= check_true(near_eq(core.steering_position(), before), "'d' steers right (mirrors 'a')");
    }

    // 'D'/'E'/'V' select mode, and touch ONLY mode (not steering/driving,
    // while idle).
    {
        vesc::TeleopCore core(make_test_teleop_config());
        const double pos_before = core.steering_position();
        core.handle_key('D', 0.0);
        ok &= check_true(core.mode() == vesc::TeleopMode::kDuty, "'D' selects duty mode");
        core.handle_key('E', 0.1);
        ok &= check_true(core.mode() == vesc::TeleopMode::kErpm, "'E' selects erpm mode");
        core.handle_key('V', 0.2);
        ok &= check_true(core.mode() == vesc::TeleopMode::kSpeed, "'V' selects speed mode");
        ok &= check_true(near_eq(core.steering_position(), pos_before), "mode-switch keys never touch steering");
    }

    // 'z' toggles ramp; 'Z' arms ramp-rate entry (distinct from plain
    // digit entry, which targets magnitude).
    {
        vesc::TeleopCore core(make_test_teleop_config());
        ok &= check_true(!core.ramp_enabled(), "ramp starts off");
        core.handle_key('z', 0.0);
        ok &= check_true(core.ramp_enabled(), "'z' toggles ramp mode on");
        core.handle_key('z', 0.1);
        ok &= check_true(!core.ramp_enabled(), "'z' toggles ramp mode off again");

        core.handle_key('Z', 0.2);
        ok &= check_true(core.digit_entry_is_ramp_rate(), "'Z' arms ramp-rate entry");
    }

    // 'k'/'R' snap to center (synonyms).
    {
        vesc::TeleopCore core(make_test_teleop_config());
        const vesc::TeleopConfig cfg = core.config();
        core.handle_key('a', 0.0);
        ok &= check_true(!near_eq(core.steering_position(), cfg.steer_center), "moved off center first");
        core.handle_key('k', 0.1);
        ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'k' snaps to center");
        core.handle_key('a', 0.2);
        core.handle_key('R', 0.3);
        ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'R' also snaps to center");
    }

    // 'T' toggles trim mode; 'W' raises a pending center-save request.
    {
        vesc::TeleopCore core(make_test_teleop_config());
        ok &= check_true(!core.in_trim_mode(), "trim mode starts off");
        core.handle_key('T', 0.0);
        ok &= check_true(core.in_trim_mode(), "'T' toggles trim mode on");

        core.handle_key('a', 0.1);  // steering touched, so 'W' has a real position to raise.
        core.handle_key('W', 0.2);
        ok &= check_true(core.has_pending_center_save(), "'W' raises a pending center-save request");
    }

    // digits/'.'/ENTER/ESC/'+'/'-' value entry.
    {
        vesc::TeleopCore core(make_test_teleop_config());
        for (char c : std::string("0.05")) core.handle_key(c, 0.0);
        core.handle_key('\r', 0.0);
        ok &= check_true(near_eq(core.magnitude(), 0.05), "digits + ENTER commits magnitude");
        for (char c : std::string("99")) core.handle_key(c, 0.1);
        core.handle_key(0x1B, 0.1);
        ok &= check_true(core.digit_buffer().empty() && near_eq(core.magnitude(), 0.05),
                          "ESC cancels the buffer without committing");
        core.handle_key('+', 0.2);
        ok &= check_true(near_eq(core.magnitude(), 0.055), "'+' steps magnitude up");
        core.handle_key('-', 0.3);
        ok &= check_true(near_eq(core.magnitude(), 0.05), "'-' steps magnitude back down");
    }

    // 'c' clears a latched abort.
    {
        vesc::TeleopCore core(make_test_teleop_config());
        vesc::VescValues v;
        v.current_motor = 50.0;
        core.feed_telemetry(v, 0.0);
        ok &= check_true(core.is_aborted(), "abort latched, setting up the 'c' check");
        core.handle_key('c', 0.1);
        ok &= check_true(!core.is_aborted(), "'c' clears a latched abort");
    }

    // 'q' raises KeyEvent{kQuit}.
    {
        vesc::TeleopCore core(make_test_teleop_config());
        const vesc::KeyEvent ev = core.handle_key('q', 0.0);
        ok &= check_true(ev.type == vesc::KeyEventType::kQuit, "'q' raises a KeyEvent{kQuit}");
    }

    return ok;
}

bool test_teleop_core_drive_invert_duty_and_erpm_modes() {
    bool ok = true;

    // Duty mode: drive_invert=true negates the emitted command for both
    // 'w' (forward) and 's' (reverse).
    {
        vesc::TeleopConfig cfg = make_test_teleop_config();
        cfg.drive_invert = true;
        vesc::TeleopCore core(cfg);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction a = core.step(0.01);
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kDuty, "duty mode still emits kDuty under drive_invert");
        ok &= check_true(near_eq(a.value, -cfg.duty_mag_default),
                          "drive_invert=true negates 'w' duty command (vehicle +forward -> motor -)");

        core.handle_key('s', 0.1);
        const vesc::TeleopMotorAction b = core.step(0.11);
        ok &= check_true(near_eq(b.value, cfg.duty_mag_default),
                          "drive_invert=true negates 's' duty command too (vehicle -reverse -> motor +)");
    }

    // erpm mode: same negation, action type stays kRpm.
    {
        vesc::TeleopConfig cfg = make_test_teleop_config();
        cfg.drive_invert = true;
        vesc::TeleopCore core(cfg);
        core.handle_key('E', 0.0);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction a = core.step(0.01);
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kRpm, "erpm mode still emits kRpm under drive_invert");
        ok &= check_true(near_eq(a.value, -cfg.erpm_mag_default), "drive_invert=true negates 'w' erpm command");

        core.handle_key('s', 0.1);
        const vesc::TeleopMotorAction b = core.step(0.11);
        ok &= check_true(near_eq(b.value, cfg.erpm_mag_default), "drive_invert=true negates 's' erpm command too");
    }

    // Regression: drive_invert=false (the default) is byte-for-byte
    // unaffected -- 'w' stays the UN-negated positive command.
    {
        vesc::TeleopConfig cfg = make_test_teleop_config();
        ok &= check_true(!cfg.drive_invert, "drive_invert defaults to false");
        vesc::TeleopCore core(cfg);
        core.handle_key('w', 0.0);
        const vesc::TeleopMotorAction a = core.step(0.01);
        ok &= check_true(near_eq(a.value, cfg.duty_mag_default),
                          "drive_invert=false: 'w' emits the un-negated positive duty command");
    }

    // Brake magnitude is NEVER inverted -- brake_amps is a fixed,
    // sign-irrelevant magnitude regardless of drive_invert (mirrors
    // vesc_driver_main.cpp's own choice to leave kBrake/kNone untouched).
    {
        vesc::TeleopConfig cfg = make_test_teleop_config();
        cfg.drive_invert = true;
        cfg.deadman_ms = 100.0;
        cfg.brake_hold_ms = 50.0;
        vesc::TeleopCore core(cfg);
        core.handle_key('w', 0.0);
        core.step(0.0);
        core.handle_key(' ', 0.01);
        const vesc::TeleopMotorAction brake = core.step(0.01);
        ok &= check_true(brake.type == vesc::TeleopMotorAction::Type::kBrake, "stop key still brakes under drive_invert");
        ok &= check_true(near_eq(brake.value, cfg.brake_amps), "brake amps is NOT inverted (positive magnitude, sign-irrelevant)");
    }

    return ok;
}

// Real closed-loop convergence proof for drive_invert in SPEED GOVERNOR
// mode -- deliberately NOT a one-tick sign check. drive_sign() is applied
// at TWO places (TeleopCore.h's own comment on drive_invert): the emitted
// duty (vehicle->motor) and the incoming measured erpm (motor->vehicle).
// If either one were missing, the governor's feedback polarity would be
// WRONG and this closed loop would diverge (drive away from the target)
// instead of converging -- so a passing convergence result here is a
// genuine end-to-end proof both call sites are wired correctly, not just
// that each individually looks right in isolation.
bool test_teleop_core_drive_invert_speed_governor_converges() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.drive_invert = true;
    cfg.speed_kp = 2e-6;
    cfg.speed_ki = 1e-5;
    cfg.speed_ff_gain = 4400.0;
    cfg.duty_ramp = 0.05;
    cfg.speed_mag_default = 1000.0;
    cfg.deadman_ms = 30000.0;
    vesc::TeleopCore core(cfg);
    // Models the REAL, physically-inverted-wiring motor: fed whatever raw
    // MOTOR-frame duty is actually written to the wire, reports raw
    // MOTOR-frame erpm right back -- exactly the plant a real driver
    // talks to (see SyntheticSpeedPlant's own comment above, section (o)).
    SyntheticSpeedPlant plant;
    plant.v_in = 8.0;

    core.step(0.0);
    core.handle_key('V', 0.0);
    core.handle_key('w', 0.0);  // VEHICLE-frame target: +1000 (operator asked for forward).

    const double dt = 0.02;
    double t = 0.0;
    for (int i = 0; i < 500; ++i) {  // 10s.
        t += dt;
        vesc::VescValues v;
        // The wire only ever carries RAW MOTOR-frame erpm -- exactly what
        // a real driver forwards from GET_VALUES, never pre-converted to
        // vehicle frame by the caller (that conversion is TeleopCore's
        // own job, inside feed_telemetry()).
        v.erpm = plant.erpm;
        v.v_in = plant.v_in;
        core.feed_telemetry(v, t);
        const vesc::TeleopMotorAction a = core.step(t);
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kDuty, "speed mode always emits kDuty under drive_invert too");
        // a.value is already MOTOR-frame (drive_sign() applied inside
        // step_speed_governor()) -- feed it straight to the plant, exactly
        // as it would be written to the real (inverted-wiring) VESC.
        plant.step(a.value, dt);
    }

    // The governor's OWN vehicle-frame view (filtered_erpm(), what the
    // status line / any consumer of TeleopCore sees) converges to the
    // POSITIVE target the operator actually requested with 'w'.
    ok &= check_true(near_eq(core.filtered_erpm(), 1000.0, 100.0),
                      "vehicle-frame filtered_erpm() converges near the +1000 target the operator requested "
                      "(got " + std::to_string(core.filtered_erpm()) + ")");

    // The PHYSICAL plant (motor frame -- what the real inverted-wiring
    // motor actually does) converges to the NEGATIVE of that target: the
    // motor really does spin "backward" in raw sensor terms while the
    // reported vehicle-frame speed correctly reads forward -- this is the
    // whole point of drive_invert.
    ok &= check_true(near_eq(plant.erpm, -1000.0, 100.0),
                      "motor-frame plant.erpm converges near -1000 (physically-inverted motor spins backward "
                      "while the reported vehicle-frame speed reads forward) (got " + std::to_string(plant.erpm) +
                          ")");

    return ok;
}

// ---------------------------------------------------------------------
// (s) TeleopCore STEERING sub-state-machine (see TeleopCore.h's
// "STEERING SUB-STATE-MACHINE" section) -- an independent
// position-servo axis (a/d/k/R/T/W) layered onto the same class as the
// drive-side tests above, INDEPENDENT of drive mode/state/abort.
// ---------------------------------------------------------------------

bool test_teleop_core_steering_step_and_clamp() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "steering starts at steer_center");

    core.handle_key('a', 0.0);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center - cfg.steer_coarse_step),
                      "'a' steps left by steer_coarse_step");

    core.handle_key('d', 0.1);
    core.handle_key('d', 0.2);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center + cfg.steer_coarse_step),
                      "'d' steps right by steer_coarse_step (mirrors 'a')");

    // Drive toward and past the low clamp.
    for (int i = 0; i < 100; ++i) core.handle_key('a', 0.3 + i * 0.01);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_min_pos),
                      "'a' repeated clamps at steer_min_pos, never below/wraps");

    // Drive toward and past the high clamp.
    for (int i = 0; i < 200; ++i) core.handle_key('d', 2.0 + i * 0.01);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_max_pos),
                      "'d' repeated clamps at steer_max_pos, never exceeds/wraps");

    return ok;
}

bool test_teleop_core_steering_invert() {
    bool ok = true;

    // steer_invert=false (default): 'a' decreases, 'd' increases.
    vesc::TeleopConfig cfg_false = make_test_teleop_config();
    cfg_false.steer_invert = false;
    vesc::TeleopCore core_false(cfg_false);
    const double start_false = core_false.steering_position();
    core_false.handle_key('a', 0.0);
    const double after_a_false = core_false.steering_position();
    ok &= check_true(near_eq(after_a_false, start_false - cfg_false.steer_coarse_step),
                      "with steer_invert=false, 'a' DECREASES position");
    core_false.handle_key('d', 0.1);
    ok &= check_true(near_eq(core_false.steering_position(), start_false),
                      "with steer_invert=false, 'd' INCREASES position (mirrors 'a')");

    // steer_invert=true: EXACTLY the opposite polarity for the same keys.
    vesc::TeleopConfig cfg_true = make_test_teleop_config();
    cfg_true.steer_invert = true;
    vesc::TeleopCore core_true(cfg_true);
    const double start_true = core_true.steering_position();
    core_true.handle_key('a', 0.0);
    const double after_a_true = core_true.steering_position();
    ok &= check_true(near_eq(after_a_true, start_true + cfg_true.steer_coarse_step),
                      "with steer_invert=true, 'a' INCREASES position");
    core_true.handle_key('d', 0.1);
    ok &= check_true(near_eq(core_true.steering_position(), start_true),
                      "with steer_invert=true, 'd' DECREASES position (mirrors 'a')");

    // Direct cross-check: the same 'a' press moves the servo in OPPOSITE
    // directions depending on steer_invert -- not just each config
    // independently matching its own doc comment.
    ok &= check_true(((after_a_false - start_false) > 0.0) != ((after_a_true - start_true) > 0.0),
                      "'a' moves the servo in OPPOSITE directions under steer_invert=false vs. true");

    return ok;
}

bool test_teleop_core_steering_snap_to_center() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    // Idle case.
    core.handle_key('a', 0.0);
    core.handle_key('a', 0.1);
    ok &= check_true(!near_eq(core.steering_position(), cfg.steer_center), "moved away from center first (idle)");
    core.handle_key('k', 0.2);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'k' snaps exactly to steer_center (idle)");

    // Driving case.
    core.step(0.2);
    core.handle_key('w', 0.2);
    core.handle_key('d', 0.3);
    core.handle_key('d', 0.4);
    ok &= check_true(!near_eq(core.steering_position(), cfg.steer_center), "moved away from center (driving)");
    core.handle_key('k', 0.5);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'k' snaps exactly to steer_center (driving)");
    ok &= check_true(core.is_driving(), "'k' does not affect drive state");

    return ok;
}

bool test_teleop_core_steering_trim_mode_stops_drive_and_fine_step() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    core.handle_key('w', 0.0);
    core.step(0.0);
    ok &= check_true(core.is_driving(), "driving before entering trim mode");
    ok &= check_true(near_eq(core.steering_step(), cfg.steer_coarse_step), "steering_step() is coarse outside trim mode");

    core.handle_key('T', 0.1);
    ok &= check_true(core.in_trim_mode(), "'T' enters trim mode");
    ok &= check_true(core.is_braking(), "entering trim mode while driving stops immediately (kBraking)");
    ok &= check_true(core.stop_reason() == "trim_mode", "stop reason is \"trim_mode\"");

    const vesc::TeleopMotorAction a = core.step(0.1);
    ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kBrake, "next step() after entering trim mode brakes");

    ok &= check_true(near_eq(core.steering_step(), cfg.steer_fine_step), "steering_step() is fine while in trim mode");

    core.handle_key('T', 0.2);
    ok &= check_true(!core.in_trim_mode(), "'T' again exits trim mode");
    ok &= check_true(near_eq(core.steering_step(), cfg.steer_coarse_step),
                      "steering_step() reverts to coarse after exiting trim mode");

    return ok;
}

bool test_teleop_core_steering_trim_mode_ignores_drive_keys() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    core.handle_key('T', 0.0);
    ok &= check_true(core.in_trim_mode(), "trim mode entered");

    core.handle_key('w', 0.1);
    ok &= check_true(!core.is_driving(), "'w' is ignored while in_trim_mode()");
    core.handle_key('s', 0.2);
    ok &= check_true(!core.is_driving(), "'s' is ignored while in_trim_mode()");

    core.handle_key('T', 0.4);  // exit trim mode.
    core.handle_key('w', 0.5);
    ok &= check_true(core.is_driving(), "'w' works again once trim mode is exited");

    return ok;
}

bool test_teleop_core_steering_pending_center_save() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    ok &= check_true(!core.has_pending_center_save(), "no pending request initially");

    core.handle_key('d', 0.0);  // move off center so the pending value is distinctive.
    const double pos1 = core.steering_position();
    core.handle_key('W', 0.1);
    ok &= check_true(core.has_pending_center_save(), "'W' raises a pending request");
    ok &= check_true(near_eq(core.pending_center_save_value(), pos1), "pending value == current steering position");

    core.handle_key('d', 0.2);
    const double pos2 = core.steering_position();
    ok &= check_true(!near_eq(pos1, pos2), "position actually moved between the two 'W' presses");
    core.handle_key('W', 0.3);
    ok &= check_true(core.has_pending_center_save(), "still exactly one pending request after a second 'W'");
    ok &= check_true(near_eq(core.pending_center_save_value(), pos2),
                      "second 'W' overwrites the pending value with the NEW current position, not a queue");

    core.consume_pending_center_save();
    ok &= check_true(!core.has_pending_center_save(), "consume_pending_center_save() clears the pending flag");

    return ok;
}

bool test_teleop_core_steering_R_reverts_to_center() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    core.handle_key('a', 0.0);
    core.handle_key('a', 0.1);
    ok &= check_true(!near_eq(core.steering_position(), cfg.steer_center), "moved away from center");
    core.handle_key('R', 0.2);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'R' snaps exactly to steer_center, same as 'k'");

    // 'R' works identically outside AND inside trim mode -- deliberately
    // not gated, see the class header's "DELIBERATE SIMPLIFICATION".
    core.handle_key('T', 0.3);
    core.handle_key('d', 0.4);
    ok &= check_true(!near_eq(core.steering_position(), cfg.steer_center), "moved away from center while in trim mode");
    core.handle_key('R', 0.5);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'R' still snaps to center while in_trim_mode()");

    return ok;
}

bool test_teleop_core_steering_refreshes_deadman() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.deadman_ms = 1000.0;
    vesc::TeleopCore core(cfg);

    core.handle_key('w', 0.0);
    core.step(0.0);
    ok &= check_true(core.is_driving(), "driving started");

    const vesc::TeleopMotorAction before = core.step(0.9);
    ok &= check_true(before.type != vesc::TeleopMotorAction::Type::kBrake, "still driving just under the original deadline");

    // A steering-only key (no w/s) -- must still refresh the deadman.
    core.handle_key('a', 0.9);
    ok &= check_true(near_eq(core.deadman_remaining_s(0.9), 1.0), "steering key refreshed the deadman to the full window");

    // The ORIGINAL deadline (0.0 + 1.0s) must NOT fire the brake now.
    const vesc::TeleopMotorAction at_original_deadline = core.step(1.0);
    ok &= check_true(at_original_deadline.type != vesc::TeleopMotorAction::Type::kBrake,
                      "drive does NOT brake at the original deadline -- the steering key pushed it out");
    ok &= check_true(core.is_driving(), "still driving past the original deadline");

    return ok;
}

bool test_teleop_core_steering_unaffected_by_safety_paths() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    cfg.deadman_ms = 200.0;
    cfg.current_abort = 8.0;

    const double distinctive = 0.7;  // within [steer_min_pos, steer_max_pos] == [0.15, 0.85], off-center.

    // (a) stop key.
    {
        vesc::TeleopCore core(cfg);
        core.set_steer_center(distinctive);
        core.handle_key('k', 0.0);  // touch steering, land it exactly on the distinctive value.
        ok &= check_true(near_eq(core.steering_position(), distinctive), "(stop) steering set to the distinctive value");
        core.handle_key('w', 0.0);
        core.step(0.0);
        core.handle_key(' ', 0.1);  // stop key.
        ok &= check_true(core.is_braking(), "(stop) drive is braking");
        ok &= check_true(near_eq(core.steering_position(), distinctive), "(stop) steering position is COMPLETELY unchanged");
    }

    // (b) deadman expiry.
    {
        vesc::TeleopCore core(cfg);
        core.set_steer_center(distinctive);
        core.handle_key('k', 0.0);
        core.handle_key('w', 0.0);
        core.step(0.0);
        const vesc::TeleopMotorAction a = core.step(0.2);  // deadman_ms/1000 == 0.2.
        ok &= check_true(a.type == vesc::TeleopMotorAction::Type::kBrake, "(deadman) fired");
        ok &= check_true(near_eq(core.steering_position(), distinctive), "(deadman) steering position is COMPLETELY unchanged");
    }

    // (c) current-abort.
    {
        vesc::TeleopCore core(cfg);
        core.set_steer_center(distinctive);
        core.handle_key('k', 0.0);
        core.handle_key('w', 0.0);
        core.step(0.0);
        vesc::VescValues v;
        v.current_motor = 50.0;
        core.feed_telemetry(v, 0.1);
        ok &= check_true(core.is_aborted(), "(abort) current-abort latched");
        ok &= check_true(near_eq(core.steering_position(), distinctive), "(abort) steering position is COMPLETELY unchanged");
    }

    return ok;
}

bool test_teleop_core_steering_no_premature_emission() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    ok &= check_true(!core.steering_ever_touched(), "steering_ever_touched() starts false");
    ok &= check_true(!core.steering_changed_since_emit(), "steering_changed_since_emit() starts false");

    core.handle_key('T', 0.0);  // trim mode alone -- no position-affecting key.
    ok &= check_true(!core.steering_ever_touched(), "'T' alone does NOT set steering_ever_touched()");
    core.handle_key('T', 0.1);  // exit trim mode again, still untouched.
    ok &= check_true(!core.steering_ever_touched(), "toggling trim mode back off still does not touch steering");

    core.handle_key('a', 0.2);
    ok &= check_true(core.steering_ever_touched(), "'a' sets steering_ever_touched()");

    vesc::TeleopCore c_d(cfg);
    c_d.handle_key('d', 0.0);
    ok &= check_true(c_d.steering_ever_touched(), "'d' sets steering_ever_touched()");

    vesc::TeleopCore c_k(cfg);
    c_k.handle_key('k', 0.0);
    ok &= check_true(c_k.steering_ever_touched(), "'k' sets steering_ever_touched()");

    vesc::TeleopCore c_r(cfg);
    c_r.handle_key('R', 0.0);
    ok &= check_true(c_r.steering_ever_touched(), "'R' sets steering_ever_touched()");

    return ok;
}

bool test_teleop_core_steering_mark_emitted_clears_flag() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    ok &= check_true(!core.steering_changed_since_emit(), "starts unchanged");
    core.handle_key('a', 0.0);
    ok &= check_true(core.steering_changed_since_emit(), "'a' sets steering_changed_since_emit()");
    core.mark_steering_emitted();
    ok &= check_true(!core.steering_changed_since_emit(), "mark_steering_emitted() clears the flag");

    // Drive into the low clamp, then re-press 'a' once already there --
    // no ACTUAL position change, so the flag must stay clear.
    for (int i = 0; i < 200; ++i) core.handle_key('a', 0.1 + i * 0.01);
    core.mark_steering_emitted();
    core.handle_key('a', 5.0);  // already at steer_min_pos -- no real movement.
    ok &= check_true(!core.steering_changed_since_emit(),
                      "'a' pressed already at the clamp limit does not re-flag a change");

    core.handle_key('k', 5.1);  // moves back to center -- a real change.
    ok &= check_true(core.steering_changed_since_emit(), "a real subsequent change re-sets the flag");

    return ok;
}

bool test_teleop_core_steering_set_steer_center_updates_snap_target() {
    bool ok = true;
    vesc::TeleopConfig cfg = make_test_teleop_config();
    vesc::TeleopCore core(cfg);

    core.handle_key('k', 0.0);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_center), "'k' snaps to the original config steer_center");

    core.set_steer_center(0.3);
    core.handle_key('k', 0.1);
    ok &= check_true(near_eq(core.steering_position(), 0.3),
                      "'k' snaps to the NEW center after set_steer_center(), no new instance needed");

    core.handle_key('a', 0.2);
    core.handle_key('R', 0.3);
    ok &= check_true(near_eq(core.steering_position(), 0.3), "'R' also snaps to the updated center");

    // set_steer_center() itself is clamped into [steer_min_pos, steer_max_pos].
    core.set_steer_center(10.0);
    core.handle_key('k', 0.4);
    ok &= check_true(near_eq(core.steering_position(), cfg.steer_max_pos),
                      "set_steer_center() clamps an out-of-range value to steer_max_pos");

    return ok;
}

// ---------------------------------------------------------------------
// (r) SteeringCalib: shared steering-servo calibration (load/save, JSON
// validation/repair, atomic-write semantics, parent-dir creation, and
// path-precedence resolution). Pure logic + plain POSIX file I/O -- no
// serial/VESC access -- all fixtures below live under a PID-suffixed /tmp
// dir, never the real $HOME/.vesc or VescDriver/config/.
// ---------------------------------------------------------------------

std::string steering_calib_read_file(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool steering_calib_equal(const vesc::SteeringCalib& a, const vesc::SteeringCalib& b) {
    return a.version == b.version && near_eq(a.center, b.center) && near_eq(a.min_pos, b.min_pos) &&
           near_eq(a.max_pos, b.max_pos) && a.invert == b.invert && near_eq(a.rad_per_unit, b.rad_per_unit) &&
           a.note == b.note;
}

// (1) load/save round trip, (2) missing file, (3) malformed JSON, (4)
// validate_and_clamp()-driven load() outcomes (simple clamp vs. genuinely
// broken ordering).
bool test_steering_calib_load_save_and_validation() {
    bool ok = true;
    const std::string tmp_dir =
        "/tmp/vesc_driver_tests_steering_calib_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);

    // (1) Round trip: non-default values on every field, including a
    // nonzero rad_per_unit and a non-empty note, to prove those round-trip
    // too (not just the three clamped doubles).
    {
        vesc::SteeringCalib in;
        in.version = 3;
        in.center = 0.55;
        in.min_pos = 0.2;
        in.max_pos = 0.8;
        in.invert = true;
        in.rad_per_unit = 0.017;
        in.note = "round-trip fixture";

        const std::string path = tmp_dir + "/roundtrip.json";
        std::string save_err;
        ok &= check_true(vesc::save(path, in, &save_err), "save() succeeds on a fully-populated, valid SteeringCalib");
        ok &= check_true(save_err.empty(), "save() clears *err_msg on success");

        vesc::SteeringCalib out;
        std::string load_err;
        ok &= check_true(vesc::load(path, &out, &load_err), "load() succeeds on the file just saved");
        ok &= check_true(steering_calib_equal(in, out), "loaded SteeringCalib matches every saved field exactly");
        ok &= check_true(out.invert, "invert round-trips true");
        ok &= check_true(!near_eq(out.rad_per_unit, 0.0), "rad_per_unit round-trips nonzero");
        ok &= check_true(!out.note.empty(), "note round-trips non-empty");
    }

    // (2) Missing file.
    {
        const std::string path = tmp_dir + "/does_not_exist.json";
        vesc::SteeringCalib out;
        out.center = 0.99;  // pre-poison to prove load() resets it.
        std::string err;
        ok &= check_true(!vesc::load(path, &out, &err), "load() on a missing file returns false");
        ok &= check_true(steering_calib_equal(out, vesc::SteeringCalib{}),
                          "load() on a missing file resets *out to SteeringCalib{} defaults");
        ok &= check_true(!err.empty() && err.find(path) != std::string::npos,
                          "load() on a missing file sets a non-empty *err_msg mentioning the path");
    }

    // (3) Malformed JSON.
    {
        const std::string path = tmp_dir + "/malformed.json";
        {
            std::ofstream f(path);
            f << "{ this is not valid json !! ";
        }
        vesc::SteeringCalib out;
        out.center = 0.99;
        std::string err;
        ok &= check_true(!vesc::load(path, &out, &err), "load() on malformed JSON returns false");
        ok &= check_true(steering_calib_equal(out, vesc::SteeringCalib{}),
                          "load() on malformed JSON resets *out to defaults");
        ok &= check_true(!err.empty(), "load() on malformed JSON sets a non-empty *err_msg");
    }

    // (4a) A value slightly out of [0,1] that STILL validates (ordering
    // survives the clamp) -- max_pos=1.2 clamps to 1.0, and with
    // center=0.9/min_pos=0.15 the ordering 0<=0.15<0.9<1.0<=1 still holds,
    // so this is a pure kClamped repair, not kRepairedOrdering. (NOTE: a
    // center value itself pushed above 1.0 can NEVER survive as a clamp-
    // only repair -- clamping caps it at exactly 1.0, which can never be
    // STRICTLY LESS than max_pos<=1 afterward -- so this sub-case
    // necessarily exercises min_pos/max_pos instead of center; see
    // SteeringCalib.cpp's validate_and_clamp() for the exact rule.)
    {
        const std::string path = tmp_dir + "/clamp_only.json";
        {
            nlohmann::json j;
            j["center"] = 0.9;
            j["min_pos"] = 0.15;
            j["max_pos"] = 1.2;
            j["note"] = "clamp-only fixture";
            std::ofstream f(path);
            f << j.dump(2);
        }
        vesc::SteeringCalib out;
        std::string err;
        ok &= check_true(vesc::load(path, &out, &err), "load() on an out-of-[0,1] max_pos (ordering survives) returns true");
        ok &= check_true(near_eq(out.max_pos, 1.0), "max_pos=1.2 clamped to 1.0");
        ok &= check_true(out.min_pos >= 0.0 && out.min_pos < out.center && out.center < out.max_pos &&
                              out.max_pos <= 1.0,
                          "clamped result still satisfies 0<=min_pos<center<max_pos<=1");
    }

    // (4b) Genuinely broken ordering (min_pos >= center) -> load() returns
    // false, *out reset to full defaults, non-empty message.
    {
        const std::string path = tmp_dir + "/broken_ordering.json";
        {
            nlohmann::json j;
            j["center"] = 0.5;
            j["min_pos"] = 0.6;  // >= center: genuinely broken, not a simple clamp.
            j["max_pos"] = 0.9;
            std::ofstream f(path);
            f << j.dump(2);
        }
        vesc::SteeringCalib out;
        out.note = "poisoned";
        std::string err;
        ok &= check_true(!vesc::load(path, &out, &err), "load() on min_pos>=center returns false");
        ok &= check_true(steering_calib_equal(out, vesc::SteeringCalib{}),
                          "load() on a broken ordering resets *out to full compiled-in defaults");
        ok &= check_true(!err.empty(), "load() on a broken ordering sets a non-empty *err_msg");
    }

    return ok;
}

// (5) Atomic save semantics (no leftover .tmp on success; a failed save
// leaves any pre-existing file untouched) and (6) parent-directory
// creation.
bool test_steering_calib_atomic_save_and_parent_dirs() {
    bool ok = true;
    const std::string tmp_dir =
        "/tmp/vesc_driver_tests_steering_calib_atomic_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);

    // (5a) No leftover .tmp file after a successful save.
    {
        const std::string path = tmp_dir + "/no_leftover_tmp.json";
        vesc::SteeringCalib calib;
        std::string err;
        ok &= check_true(vesc::save(path, calib, &err), "save() succeeds for the no-leftover-tmp check");
        struct stat st;
        ok &= check_true(stat((path + ".tmp").c_str(), &st) != 0,
                          "no '<path>.tmp' file remains on disk after a successful save()");
    }

    // (5b) A failing save (unwritable parent dir) must leave a
    // pre-existing file at that exact path completely unchanged, and must
    // return false. Skipped (with a note, not a failure) when running as
    // root, since root ignores the write-permission bit and the premise
    // of this sub-case would not hold.
    if (geteuid() == 0) {
        std::cout << "    (skipping unwritable-dir save() check: running as root)\n";
    } else {
        const std::string ro_dir = tmp_dir + "/readonly_dir";
        mkdir(ro_dir.c_str(), 0755);
        const std::string path = ro_dir + "/steering_calib.json";

        vesc::SteeringCalib original;
        original.center = 0.6;
        original.note = "pre-existing, must survive the failed save";
        std::string err;
        ok &= check_true(vesc::save(path, original, &err), "initial save() (dir still writable) succeeds");
        const std::string original_contents = steering_calib_read_file(path);

        ok &= check_true(chmod(ro_dir.c_str(), 0500) == 0, "chmod(dir, 0500) (read+exec, no write) succeeds");

        vesc::SteeringCalib attempted;
        attempted.center = 0.7;
        attempted.note = "this save must fail and never land";
        std::string fail_err;
        ok &= check_true(!vesc::save(path, attempted, &fail_err), "save() into an unwritable directory returns false");
        ok &= check_true(!fail_err.empty(), "a failed save() sets a non-empty *err_msg");

        struct stat st;
        ok &= check_true(stat((path + ".tmp").c_str(), &st) != 0,
                          "a failed save() leaves no '<path>.tmp' behind either");

        const std::string contents_after = steering_calib_read_file(path);
        ok &= check_true(contents_after == original_contents,
                          "the pre-existing file's contents are byte-for-byte unchanged after the failed save()");

        // Restore write+exec so test cleanup (rmdir/unlink) can remove it.
        chmod(ro_dir.c_str(), 0700);
    }

    // (6) Parent-directory creation: save() to a path under nested,
    // not-yet-existing directories.
    {
        const std::string nested_path = tmp_dir + "/nested/subdir/steering_calib.json";
        vesc::SteeringCalib calib;
        calib.note = "nested-dir fixture";
        std::string err;
        ok &= check_true(vesc::save(nested_path, calib, &err),
                          "save() creates missing parent directories and succeeds (" + err + ")");
        struct stat st;
        ok &= check_true(stat(nested_path.c_str(), &st) == 0, "the file exists at the nested path after save()");

        vesc::SteeringCalib out;
        std::string load_err;
        ok &= check_true(vesc::load(nested_path, &out, &load_err), "the nested-path file loads back successfully");
        ok &= check_true(out.note == "nested-dir fixture", "the nested-path file's contents round-trip correctly");
    }

    return ok;
}

// (7) resolve_load_path()/resolve_save_path() precedence, exercised with
// injected fake home_dir/exe_dir fixtures under /tmp (never the real
// $HOME or /proc/self/exe). resolve_default_load_path()/
// resolve_default_save_path() are only smoke-called (compile/link/no
// crash), never asserted against the real machine's actual $HOME.
bool test_steering_calib_path_resolution() {
    bool ok = true;
    const std::string tmp_dir =
        "/tmp/vesc_driver_tests_steering_calib_paths_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);
    const std::string fake_home = tmp_dir + "/fake_home";
    const std::string fake_exe_dir = tmp_dir + "/fake_exe_dir";
    mkdir(fake_home.c_str(), 0755);
    mkdir(fake_exe_dir.c_str(), 0755);
    const std::string fake_home_vesc_dir = fake_home + "/.vesc";
    mkdir(fake_home_vesc_dir.c_str(), 0755);
    const std::string fake_exe_config_dir = tmp_dir + "/config";  // "<exe_dir>/../config"
    mkdir(fake_exe_config_dir.c_str(), 0755);

    const std::string home_file = fake_home_vesc_dir + "/steering_calib.json";
    const std::string exe_fallback_file = fake_exe_config_dir + "/steering_calib.json";
    // resolve_load_path() builds its tier-3 candidate as the LITERAL string
    // "<exe_dir>/../config/steering_calib.json" (never lexically
    // normalized) -- the OS resolves the ".." at stat()/open() time, so it
    // is filesystem-equivalent to exe_fallback_file above but NOT the same
    // std::string; this is the exact literal resolve_load_path() itself
    // returns, used below for string-equality assertions.
    const std::string exe_candidate_literal = fake_exe_dir + "/../config/steering_calib.json";

    // Explicit path wins over everything, regardless of what else exists.
    {
        const std::string explicit_path = tmp_dir + "/explicit.json";
        ok &= check_true(vesc::resolve_load_path(explicit_path, fake_home, fake_exe_dir) == explicit_path,
                          "resolve_load_path(): explicit_path wins over home/exe tiers");
        ok &= check_true(vesc::resolve_save_path(explicit_path, fake_home) == explicit_path,
                          "resolve_save_path(): explicit_path wins over home_dir");
    }

    // Neither home nor exe fallback file exists yet -> falls back to the
    // deterministic home-based tier-4 path.
    {
        const std::string got = vesc::resolve_load_path("", fake_home, fake_exe_dir);
        ok &= check_true(got == home_file,
                          "resolve_load_path(): with no files present, falls back to the deterministic "
                          "home-based path");
    }

    // home_dir empty and exe fallback absent too -> empty string.
    {
        const std::string got = vesc::resolve_load_path("", "", fake_exe_dir);
        ok &= check_true(got.empty(), "resolve_load_path(): empty home_dir + no exe fallback file -> empty string");
    }

    // exe fallback file present (home file still absent) -> tier 3 used.
    {
        std::ofstream f(exe_fallback_file);
        f << "{}";
    }
    {
        const std::string got = vesc::resolve_load_path("", fake_home, fake_exe_dir);
        ok &= check_true(got == exe_candidate_literal,
                          "resolve_load_path(): home file absent, exe fallback present -> tier 3 (exe fallback) used");
    }

    // Now create the home file too -> tier 2 must win over tier 3.
    {
        std::ofstream f(home_file);
        f << "{}";
    }
    {
        const std::string got = vesc::resolve_load_path("", fake_home, fake_exe_dir);
        ok &= check_true(got == home_file,
                          "resolve_load_path(): home file present -> tier 2 wins over tier 3 (exe fallback)");
    }

    // resolve_save_path() NEVER returns the exe_dir-derived path, even
    // though resolve_save_path() itself takes no exe_dir argument at all
    // (the point being verified: it always resolves to the home-based
    // path, never anything under fake_exe_dir/fake_exe_config_dir).
    {
        const std::string got = vesc::resolve_save_path("", fake_home);
        ok &= check_true(got == home_file, "resolve_save_path(): resolves to the home-based path");
        ok &= check_true(got.find(fake_exe_dir) == std::string::npos,
                          "resolve_save_path(): never returns anything under the exe_dir tree");
    }
    {
        const std::string got = vesc::resolve_save_path("", "");
        ok &= check_true(got.empty(), "resolve_save_path(): empty explicit_path + empty home_dir -> empty string");
    }

    // Smoke-call the real-environment wrappers -- just prove they
    // compile/link/return without crashing; never assert specifics about
    // this machine's actual $HOME.
    {
        const std::string default_load = vesc::resolve_default_load_path("");
        const std::string default_save = vesc::resolve_default_save_path("");
        (void)default_load;
        (void)default_save;
        ok &= check_true(true, "resolve_default_load_path()/resolve_default_save_path() callable without crashing");
    }

    return ok;
}

// (8) drive_invert field: defaults false on a fresh SteeringCalib{}, round-
// trips both true and false through save()/load() (proving it isn't just
// always coming back true or always defaulting regardless of what was
// saved); (9) SteeringCalib{}'s new compiled-in min_pos/max_pos defaults
// are exactly 0.05/0.95 (center unchanged at 0.5); (10) explicit non-
// default min_pos/max_pos survive load() rather than being silently
// replaced by the new 0.05/0.95 defaults; (11) validate_and_clamp()'s
// kRepairedOrdering path resets center/min_pos/max_pos to the new
// 0.5/0.05/0.95 defaults (pinned to exact literals, complementing the
// dynamic SteeringCalib{}-comparison already done in
// test_steering_calib_load_save_and_validation()'s broken-ordering
// sub-case above, which automatically tracks whatever the compiled-in
// defaults are).
bool test_steering_calib_drive_invert_and_new_defaults() {
    bool ok = true;
    const std::string tmp_dir =
        "/tmp/vesc_driver_tests_steering_calib_drive_invert_" + std::to_string(static_cast<long>(getpid()));
    mkdir(tmp_dir.c_str(), 0755);

    // (8a) drive_invert defaults to false on a fresh SteeringCalib{}.
    {
        vesc::SteeringCalib fresh;
        ok &= check_true(!fresh.drive_invert, "SteeringCalib{} defaults drive_invert to false");
    }

    // (8b) round trip true.
    {
        vesc::SteeringCalib in;
        in.center = 0.5;
        in.min_pos = 0.1;
        in.max_pos = 0.9;
        in.drive_invert = true;
        in.note = "drive_invert=true fixture";

        const std::string path = tmp_dir + "/drive_invert_true.json";
        std::string save_err;
        ok &= check_true(vesc::save(path, in, &save_err), "save() succeeds with drive_invert=true");

        vesc::SteeringCalib out;
        std::string load_err;
        ok &= check_true(vesc::load(path, &out, &load_err), "load() succeeds on the drive_invert=true file");
        ok &= check_true(out.drive_invert, "drive_invert round-trips true");
    }

    // (8c) round trip false, with other fields deliberately non-default so
    // this case can't be confused with "the field was just left at its
    // default" -- it proves an explicitly-saved false is actually read
    // back as false, not just coincidentally matching the default.
    {
        vesc::SteeringCalib in;
        in.center = 0.5;
        in.min_pos = 0.1;
        in.max_pos = 0.9;
        in.drive_invert = false;
        in.note = "drive_invert=false fixture, non-default limits";

        const std::string path = tmp_dir + "/drive_invert_false.json";
        std::string save_err;
        ok &= check_true(vesc::save(path, in, &save_err), "save() succeeds with drive_invert=false");

        vesc::SteeringCalib out;
        std::string load_err;
        ok &= check_true(vesc::load(path, &out, &load_err), "load() succeeds on the drive_invert=false file");
        ok &= check_true(!out.drive_invert, "drive_invert round-trips false (not just always true)");
        ok &= check_true(near_eq(out.min_pos, 0.1) && near_eq(out.max_pos, 0.9),
                          "the other explicit non-default fields also round-tripped, proving this file was "
                          "actually parsed and not just defaults");
    }

    // (9) SteeringCalib{}'s default min_pos/max_pos are exactly 0.05/0.95
    // (not the old 0.15/0.85); center is unchanged at 0.5.
    {
        vesc::SteeringCalib fresh;
        ok &= check_true(near_eq(fresh.min_pos, 0.05), "SteeringCalib{} default min_pos is 0.05");
        ok &= check_true(near_eq(fresh.max_pos, 0.95), "SteeringCalib{} default max_pos is 0.95");
        ok &= check_true(near_eq(fresh.center, 0.5), "SteeringCalib{} default center is still 0.5 (unchanged)");
    }

    // (10) Precedence regression check: explicit non-default min_pos/
    // max_pos survive load(), are NOT silently replaced by the new
    // compiled-in 0.05/0.95 defaults.
    {
        vesc::SteeringCalib in;
        in.center = 0.5;
        in.min_pos = 0.1;
        in.max_pos = 0.9;

        const std::string path = tmp_dir + "/explicit_non_default_limits.json";
        std::string save_err;
        ok &= check_true(vesc::save(path, in, &save_err), "save() succeeds with explicit min_pos=0.1/max_pos=0.9");

        vesc::SteeringCalib out;
        std::string load_err;
        ok &= check_true(vesc::load(path, &out, &load_err), "load() succeeds on the explicit-limits file");
        ok &= check_true(near_eq(out.min_pos, 0.1),
                          "loaded min_pos is the explicit 0.1, not silently replaced by the new 0.05 default");
        ok &= check_true(near_eq(out.max_pos, 0.9),
                          "loaded max_pos is the explicit 0.9, not silently replaced by the new 0.95 default");
    }

    // (11) validate_and_clamp()'s kRepairedOrdering path resets center/
    // min_pos/max_pos to the exact new 0.5/0.05/0.95 defaults.
    {
        vesc::SteeringCalib broken;
        broken.center = 0.5;
        broken.min_pos = 0.6;  // >= center: genuinely broken ordering, not a simple clamp.
        broken.max_pos = 0.9;
        std::string note;
        const vesc::ValidationOutcome outcome = vesc::validate_and_clamp(&broken, &note);
        ok &= check_true(outcome == vesc::ValidationOutcome::kRepairedOrdering,
                          "validate_and_clamp() on min_pos>=center reports kRepairedOrdering");
        ok &= check_true(near_eq(broken.center, 0.5), "kRepairedOrdering resets center to the new default 0.5");
        ok &= check_true(near_eq(broken.min_pos, 0.05), "kRepairedOrdering resets min_pos to the new default 0.05");
        ok &= check_true(near_eq(broken.max_pos, 0.95), "kRepairedOrdering resets max_pos to the new default 0.95");
    }

    return ok;
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, bool (*)()>> tests = {
        {"a) CRC16/XMODEM: table-driven vs. independent bitwise reference, hand-computed literals",
         test_crc16_reference_vectors},
        {"b1) frame encode -> streaming decode round trip (basic short frame)", test_frame_roundtrip_basic},
        {"b2) frame round trip with a garbage prefix (resync)", test_frame_roundtrip_garbage_prefix},
        {"b3) frame round trip split byte-by-byte across feed() calls", test_frame_roundtrip_split_across_feeds},
        {"b4) corrupted-CRC frame is rejected, decoder resyncs onto the next good frame",
         test_frame_roundtrip_corrupted_crc_rejected},
        {"b5) long-packet (payload > 255 bytes) framing form round trip", test_frame_roundtrip_long_form},
        {"c) command builders: exact hand-computed byte sequences", test_command_builders},
        {"c2) servo command id resolution (FW2.x hazard fix) + id-aware encoding",
         test_servo_cmd_id_resolution},
        {"d1) GET_VALUES parse from a hand-built canned payload", test_get_values_parse},
        {"d2) FW_VERSION parse", test_fw_version_parse},
        {"e1) AckermannCodec: valid payload decodes to expected doubles", test_ackermann_valid_decode},
        {"e2) AckermannCodec: malformation classes all yield ok=false", test_ackermann_malformed_variants},
        {"f1) rank_candidates(): ChibiOS/VESC-keyword by-id entries rank above generic by-id, which "
         "ranks above the ttyACM fallback tier",
         test_rank_candidates_by_id_priority},
        {"f2) rank_candidates(): STMicroelectronics keyword recognized; overlapping by-id/fallback "
         "paths de-duplicated",
         test_rank_candidates_stmicro_and_dedup},
        {"f3) rank_candidates(): ttyACM-only and fully-empty input edge cases", test_rank_candidates_ttyacm_only},
        {"f4) float32_auto: round trip + independent bitwise reference + hand-computed literals",
         test_float32_auto_roundtrip_and_reference},
        {"f5) McconfPatcher: scan_speed_pid/scan_current_limits/patch on the shared synthetic blob "
         "(known offsets, exact 4-byte diff)",
         test_mcconf_patcher_scan_and_patch_known_blob},
        {"f6) McconfPatcher: ambiguous/near-miss-decoy scan results correctly yield 0 or >1 matches",
         test_mcconf_patcher_ambiguous_cases},
        {"f7) McconfPatcher: scan_speed_pid/scan_current_limits against a REAL FW 6.6 GET_MCCONF dump, "
         "exact hand-decoded ground-truth values at exact offsets",
         test_mcconf_patcher_real_fw66_fixture},
        {"h1) DriverCore: accel integration + hold semantics", test_driver_core_accel_integration_and_hold},
        {"h2) DriverCore: watchdog engage/ramp/snap/brake + malformed-does-not-refresh + instant disengage",
         test_driver_core_watchdog_lifecycle},
        {"h3) DriverCore: kick-start lifecycle (fire, sustain, expire, direction)", test_driver_core_kick_lifecycle},
        {"h4) DriverCore: LinearMap", test_driver_core_linear_map},
        {"h5) DriverCore: bilinear grid lookup (exact node, interpolation, edge clamping)",
         test_driver_core_bilinear_lookup},
        {"h6) DriverCore: parse_calibration_json() malformation classes + grid:null/absent",
         test_driver_core_calibration_parse_malformed},
        {"h7) DriverCore: CalibratedMap loaded from the canned calibration_example.json + "
         "build_motor_map() fallback behavior",
         test_driver_core_calibration_file_loading},
        {"h8) DriverCore: safety clamps (raw calib, accel, v_target, mapped cmd) + accel=2.2 unclamped",
         test_driver_core_safety_clamps},
        {"h9) DriverCore: calib source path (idle/active/TTL-brake) + source-switch v_target reset",
         test_driver_core_calib_path_and_source_switch},
        {"h10) DriverCore: stop()", test_driver_core_stop},
        {"h11) load_driver_config(): missing file, the real shipped driver_config.json, and a custom "
         "file with a malformed nested object",
         test_driver_config_loader},
        {"h12) DriverCore: NaN/Inf accel+steering never latch v_target_/servo_pos/motor cmd non-finite, "
         "and the driver recovers on the next valid command",
         test_driver_core_nonfinite_ackermann_rejected},
        {"g1) SpeedGovernor: FF+P+I formula matches the pre-extraction inline math on a fresh governor",
         test_speed_governor_pi_ff_formula_and_slew},
        {"g2) SpeedGovernor: tau-based EMA alpha=1-exp(-dt/tau) exact arithmetic + dt<=0 no-op",
         test_speed_governor_tau_based_ema_exact},
        {"g3) SpeedGovernor: saturation freeze anti-windup + bounded recovery time on a synthetic plant",
         test_speed_governor_saturation_freeze_anti_windup},
        {"g4) SpeedGovernor: reset() clears integrator/filter/slew/saturation but NOT v_in(), fresh-start formula",
         test_speed_governor_reset_clears_state},
        {"g5) SpeedGovernor: seed_output() starts the next step()'s slew from the seeded value (kick handoff)",
         test_speed_governor_seed_output_for_kick_handoff},
        {"i1) VelocityMap: forward/inverse interpolation + end clamping (never extrapolates)",
         test_velocity_map_forward_inverse_and_clamp},
        {"i2) VelocityMap: linear-fallback mode (no table) + near-zero erpm_per_mps guard",
         test_velocity_map_linear_fallback},
        {"i3) VelocityMap: parse_velocity_calib_json() malformation classes + absent-table linear-fallback case",
         test_velocity_map_parse_good_and_bad},
        {"i4) VelocityMap: load_velocity_map() against the shipped velocity_calib.example.json + missing/empty-path fallback",
         test_velocity_map_load_file_and_example},
        {"k1) SteeringAngleMap: delta->servo interpolation + delta clamping to [delta_min,delta_max]",
         test_steering_angle_map_interpolation_and_clamp},
        {"k2) SteeringAngleMap: parse_steering_angle_map_json() validation (monotone delta, monotone-either-"
         "direction servo, servo range, point count)",
         test_steering_angle_map_parse_validation},
        {"k3) SteeringAngleMap: load_steering_angle_map_file() against a test fixture + wheel-base mismatch "
         "detection at the map level + missing/empty-path fallback",
         test_steering_angle_map_load_file_and_wheelbase_check},
        {"t1) DriverCore v2: DriverConfig{}'s own compiled-in defaults (watchdog_ms=1500, command_semantics="
         "\"accel\", actuation=\"map\", governor sub-config) + the new watchdog timing actually uses 1500ms",
         test_driver_core_v2_struct_defaults},
        {"t2) DriverCore v2: command_semantics==\"velocity\" per-field NaN-holds (speed NaN holds target, "
         "accel NaN holds the slew bound) + accel semantics never reads wire speed (regression)",
         test_driver_core_velocity_semantics_nan_holds},
        {"t3) DriverCore v2: NaN steering HOLDS the last finite servo_pos (replaces the pre-v2 recenter-to-0 "
         "behavior)",
         test_driver_core_steering_nan_holds_last_finite},
        {"t4) DriverCore v2: a loaded SteeringAngleMap REPLACES the legacy affine servo formula + no-map "
         "regression",
         test_driver_core_steering_angle_map_wiring},
        {"t5) DriverCore v2: actuation==\"governor\" backend converges to the VelocityMap-derived target erpm "
         "on a synthetic plant, always emitting kDuty",
         test_driver_core_governor_actuation_converges},
        {"t6) DriverCore v2: kick bypass emits kick_duty under actuation==\"governor\" and seeds the "
         "governor's slew state on kick end",
         test_driver_core_kick_in_governor_mode_seeds_slew},
        {"j1) TeleopCore: mode switching keeps separate per-mode magnitudes", test_teleop_core_mode_switch_separate_magnitudes},
        {"j2) TeleopCore: +/- step adjust, floor-at-one-step, and max clamp", test_teleop_core_step_adjust_and_clamp},
        {"j3) TeleopCore: digit-entry commit (ENTER, clamped) and clear (ESC)",
         test_teleop_core_digit_entry_commit_and_clear},
        {"j4) TeleopCore: drive engage + repeated-key refresh + direction sign", test_teleop_core_drive_and_deadman_refresh},
        {"j5) TeleopCore: deadman fires at exactly last_key+deadman_ms -> brake for brake_hold then "
         "none; stop key (space/'x') triggers the same path immediately",
         test_teleop_core_deadman_and_stop},
        {"j6) TeleopCore: current-abort latches (exact reason string), ignores w/s, requires 'c' to clear",
         test_teleop_core_current_abort_latches_and_clears},
        {"j7) TeleopCore: nonzero-fault abort path + fault_name() table (named + unrecognized codes)",
         test_teleop_core_fault_abort},
        {"j8) TeleopCore: magnitude clamping to configured max_duty/max_erpm, including the live "
         "driving action",
         test_teleop_core_clamping_to_configured_max},
        {"l1) VescProtocol: parse_get_values_legacy() (FW 2.x) on a hand-built canned payload -- "
         "temp_fet=max(temp_mos1..6), has_temp_motor=false, nonzero fault byte, defensive short/"
         "trailing/wrong-id handling",
         test_get_values_parse_legacy_fw2},
        {"l2) VescProtocol: parse_get_values_for_fw() dispatches fw_major==2 to legacy / 3-6 to modern, "
         "and demonstrates the silent-misparse hazard of the wrong dispatch",
         test_get_values_parse_for_fw_dispatch},
        {"m1) TeleopCore RAMP: ramp OFF is byte-for-byte the original instant behavior (regression lock)",
         test_teleop_core_ramp_off_instant_regression},
        {"m2) TeleopCore RAMP: slew progression at exact tick arithmetic (reaches target exactly, "
         "holds with no overshoot)",
         test_teleop_core_ramp_slew_progression_exact_arithmetic},
        {"m3) TeleopCore RAMP: direction reversal slews THROUGH zero (no reset-to-0 snap)",
         test_teleop_core_ramp_through_zero_reversal},
        {"m4) TeleopCore RAMP: a magnitude change mid-ramp moves the target only -- the slew "
         "continues from the current emitted value",
         test_teleop_core_ramp_mid_ramp_target_change},
        {"m5) TeleopCore RAMP: stop key / deadman / current-abort / fault-abort ALL brake "
         "immediately mid-ramp, bypassing the ramp entirely",
         test_teleop_core_ramp_safety_paths_bypass},
        {"m6) TeleopCore RAMP: 'Z'+digits+ENTER commits ramp-rate entry (clamped per-mode bounds), "
         "ESC cancels, plain digit-entry still targets magnitude",
         test_teleop_core_ramp_rate_entry_commit_clamp_cancel},
        {"m7) TeleopCore RAMP: duty_ramp/erpm_ramp rates are independent per mode",
         test_teleop_core_ramp_per_mode_rate_independence},
        {"m8) TeleopCore RAMP: switching mode (D/E) while driving stops first (reason \"mode_switch\"), "
         "never slews across incompatible units",
         test_teleop_core_ramp_mode_switch_stops_first},
        {"o1) TeleopCore SPEED: separate per-mode magnitude (target erpm), erpm_step stepping, clamp to max_erpm",
         test_teleop_core_speed_mode_separate_magnitude_and_step},
        {"o2) TeleopCore SPEED: governor converges to target within tolerance, never exceeding the "
         "configured slew limit per tick",
         test_teleop_core_speed_governor_converges_and_respects_slew},
        {"o3) TeleopCore SPEED: voltage independence -- stays converged on the same target erpm across "
         "a mid-run battery voltage drop (8.2V -> 7.4V)",
         test_teleop_core_speed_governor_voltage_independence},
        {"o4) TeleopCore SPEED: direction reversal converges from +target to -target, always emitting kDuty",
         test_teleop_core_speed_governor_reversal_through_zero},
        {"o5) TeleopCore SPEED: ff_gain=0 (pure PI) still converges, just slower",
         test_teleop_core_speed_governor_ff_disabled_still_converges},
        {"o6) TeleopCore SPEED: integrator anti-windup -- pinned at max_duty under sustained "
         "saturation, then converges to a new achievable target within a BOUNDED recovery time",
         test_teleop_core_speed_governor_anti_windup},
        {"o7) TeleopCore SPEED: governor state (integrator + erpm filter) fully resets on stop key, "
         "deadman expiry, current-abort, AND mode-switch -- no carried-over windup into the next drive",
         test_teleop_core_speed_governor_reset_on_stop_deadman_abort_modeswitch},
        {"o8) TeleopCore SPEED: stop/deadman/current-abort/fault-abort all brake IMMEDIATELY "
         "mid-governor, bypassing it entirely (no gradual wind-down)",
         test_teleop_core_speed_governor_safety_paths_bypass},
        {"u1) TeleopCore: new key map -- every key does its documented (and ONLY its documented) "
         "job, SPACE and 'x' both stop, every retired key (f/b/r/j/l/e/v/A) is inert and yields a "
         "non-empty retired_hint naming its replacement",
         test_teleop_core_new_key_map_and_retired_keys},
        {"u2) TeleopCore: drive_invert=true negates the emitted command in duty AND erpm modes for "
         "both 'w'/'s', drive_invert=false is unaffected, and brake magnitude is never inverted",
         test_teleop_core_drive_invert_duty_and_erpm_modes},
        {"u3) TeleopCore: drive_invert=true speed governor CONVERGES with correct feedback polarity "
         "-- real closed-loop proof feeding MOTOR-frame erpm in and asserting the VEHICLE-frame "
         "target is reached (not a one-tick sign check)",
         test_teleop_core_drive_invert_speed_governor_converges},
        {"s1) TeleopCore STEERING: a/d coarse stepping + clamping at steer_min_pos/steer_max_pos "
         "(no overshoot, no wrap)",
         test_teleop_core_steering_step_and_clamp},
        {"s2) TeleopCore STEERING: steer_invert=false vs. true flips which of a/d increases vs. "
         "decreases position -- direct opposite-polarity comparison",
         test_teleop_core_steering_invert},
        {"s3) TeleopCore STEERING: 'k' snaps exactly to config_.steer_center, in both idle and driving states",
         test_teleop_core_steering_snap_to_center},
        {"s4) TeleopCore STEERING: 'T' entering trim mode while driving stops first (reason \"trim_mode\") "
         "and switches a/d to the fine step; exiting reverts to coarse",
         test_teleop_core_steering_trim_mode_stops_drive_and_fine_step},
        {"s5) TeleopCore STEERING: w/s are ignored while in_trim_mode() (drive never starts), and work "
         "again once trim mode exits",
         test_teleop_core_steering_trim_mode_ignores_drive_keys},
        {"s6) TeleopCore STEERING: 'W' raises exactly one pending center-save request (current position); "
         "a second 'W' overwrites, not queues; consume_pending_center_save() clears it",
         test_teleop_core_steering_pending_center_save},
        {"s7) TeleopCore STEERING: 'R' reverts to config_.steer_center, identically to 'k', both outside "
         "and inside trim mode (deliberately not trim-gated)",
         test_teleop_core_steering_R_reverts_to_center},
        {"s8) TeleopCore STEERING: a/d/k refresh the deadman clock exactly like any other key, without "
         "themselves being a drive command",
         test_teleop_core_steering_refreshes_deadman},
        {"s9) TeleopCore STEERING: stop key / deadman expiry / current-abort NEVER move "
         "steering_position() -- COMPLETELY unchanged across each safety-path transition",
         test_teleop_core_steering_unaffected_by_safety_paths},
        {"s10) TeleopCore STEERING: no SET_SERVO_POS-readiness signal (steering_ever_touched()) until "
         "a/d/k/R actually runs; 'T' alone never sets it",
         test_teleop_core_steering_no_premature_emission},
        {"s11) TeleopCore STEERING: mark_steering_emitted() clears steering_changed_since_emit() until "
         "the next actual position change",
         test_teleop_core_steering_mark_emitted_clears_flag},
        {"s12) TeleopCore STEERING: set_steer_center() updates what k/R snap to in-place (no new "
         "instance needed), itself clamped",
         test_teleop_core_steering_set_steer_center_updates_snap_target},
        {"r1) SteeringCalib: load/save round trip (all fields incl. rad_per_unit/note), missing "
         "file, malformed JSON, and validate_and_clamp()-driven load() outcomes (clamp-only vs. "
         "genuinely broken ordering)",
         test_steering_calib_load_save_and_validation},
        {"r2) SteeringCalib: atomic save() (no leftover .tmp; a failed save leaves a pre-existing "
         "file byte-for-byte unchanged) + parent-directory auto-creation",
         test_steering_calib_atomic_save_and_parent_dirs},
        {"r3) SteeringCalib: resolve_load_path()/resolve_save_path() precedence (explicit > home > "
         "exe fallback > deterministic home fallback), injected fake home/exe dirs only",
         test_steering_calib_path_resolution},
        {"r4) SteeringCalib: drive_invert defaults false + round-trips true/false through save()/"
         "load(), new default min_pos/max_pos are exactly 0.05/0.95, explicit non-default limits "
         "still survive load(), and kRepairedOrdering resets to the new defaults",
         test_steering_calib_drive_invert_and_new_defaults},
    };

    return run_registered_tests(tests);
}
