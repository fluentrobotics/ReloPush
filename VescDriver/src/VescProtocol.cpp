#include "VescProtocol.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>

namespace vesc {

namespace {

// Lazily-built CRC-16/XMODEM table (poly 0x1021), computed once via the
// standard bit-by-bit method and cached in a function-local static (magic
// statics are thread-safe as of C++11).
const std::array<uint16_t, 256>& crc16_table() {
    static const std::array<uint16_t, 256> table = [] {
        std::array<uint16_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint16_t crc = static_cast<uint16_t>(i << 8);
            for (int bit = 0; bit < 8; ++bit) {
                if (crc & 0x8000u) {
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
                } else {
                    crc = static_cast<uint16_t>(crc << 1);
                }
            }
            t[i] = crc;
        }
        return t;
    }();
    return table;
}

// Upper bound on a long-form (2-byte length) payload. VESC GET_VALUES
// replies are well under 100 bytes, so this is already generous for
// every command this driver builds/parses today; it exists to guard the
// streaming decoder against a corrupted/garbage length field forcing an
// unbounded wait for bytes that may never arrive.
//
// This must stay LOW ENOUGH that a stray 0x03 byte (which is both the
// universal frame STOP byte and the long-form frame START byte, so it
// appears constantly in real traffic) immediately followed by two
// unrelated bytes is very unlikely to alias into a "plausible" declared
// length -- otherwise the decoder cannot tell that candidate apart from
// a genuine not-yet-fully-arrived long frame and has no choice but to
// wait, which is a real resync hazard (e.g. right after a CRC-corrupted
// short frame's own valid stop byte, immediately followed by the next
// real frame's start+length bytes).
//
// Raised from 400 to 512 to fit a GET_MCCONF/SET_MCCONF motor-config
// blob (~400-500 bytes on real FW 6.x firmware) through the long-packet
// (0x03) form -- this is the ONLY reason for the increase; nothing else
// this driver builds/parses needs more than a few hundred bytes.
//
// IMPORTANT: this is NOT just "same reasoning, bigger number" -- 600 was
// tried first and empirically REGRESSED two of this file's own existing
// resync tests (test_frame_roundtrip_garbage_prefix and
// test_frame_roundtrip_corrupted_crc_rejected in vesc_driver_tests.cpp).
// Tracing why: those tests' own garbage/corrupted bytes, after the
// decoder resyncs past them one byte at a time, happen to land on a
// stray 0x03 followed by two bytes that decode as declared length 517
// -- i.e. they are literal worked examples of the exact aliasing hazard
// this bound exists to guard against, per the comment above. 517 is
// comfortably above the real requirement (~500) but was ALSO now
// "plausible" under a 600 bound, so the decoder waited for 517 bytes
// that were never coming (that unit test does a single one-shot feed())
// instead of recognizing the garbage and resyncing onto the valid frame
// sitting right behind it. 512 keeps that same 517-aliasing case
// correctly rejected (512 < 517) while still comfortably covering the
// mcconf blob. A future change that needs a bound at or above 517 MUST
// NOT just raise this constant again without re-deriving (or
// re-generating) safe test fixtures -- re-run the full resync/garbage-
// byte test suite (vesc_driver_tests) and actually trace any failure
// the way this comment does, don't loosen or delete the assertions.
constexpr size_t kMaxPayloadLen = 512;

void append_be_i32(std::vector<uint8_t>* out, int32_t value) {
    const uint32_t u = static_cast<uint32_t>(value);
    out->push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
    out->push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
    out->push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(u & 0xFF));
}

void append_be_u16(std::vector<uint8_t>* out, uint16_t value) {
    out->push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(value & 0xFF));
}

// Reads a big-endian signed 16-bit value at payload[idx], advancing idx by
// 2. Caller must have already verified 2 bytes remain.
int16_t read_be_i16(const std::vector<uint8_t>& payload, size_t idx) {
    const uint16_t u = (static_cast<uint16_t>(payload[idx]) << 8) |
                        static_cast<uint16_t>(payload[idx + 1]);
    return static_cast<int16_t>(u);
}

// Reads a big-endian signed 32-bit value at payload[idx], advancing idx by
// 4. Caller must have already verified 4 bytes remain.
int32_t read_be_i32(const std::vector<uint8_t>& payload, size_t idx) {
    const uint32_t u = (static_cast<uint32_t>(payload[idx]) << 24) |
                        (static_cast<uint32_t>(payload[idx + 1]) << 16) |
                        (static_cast<uint32_t>(payload[idx + 2]) << 8) |
                        static_cast<uint32_t>(payload[idx + 3]);
    return static_cast<int32_t>(u);
}

// Shared clamp+encode logic for build_set_servo_pos()'s two overloads --
// only the command id byte pushed at the front differs between them.
std::vector<uint8_t> build_set_servo_pos_with_id(double pos, uint8_t cmd_id) {
    const double clamped = std::min(1.0, std::max(0.0, pos));
    std::vector<uint8_t> out;
    out.push_back(cmd_id);
    append_be_u16(&out, static_cast<uint16_t>(std::lround(clamped * 1000.0)));
    return out;
}

}  // namespace

uint16_t crc16(const uint8_t* data, size_t len) {
    const auto& table = crc16_table();
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; ++i) {
        const uint8_t idx = static_cast<uint8_t>((crc >> 8) ^ data[i]);
        crc = static_cast<uint16_t>((crc << 8) ^ table[idx]);
    }
    return crc;
}

std::vector<uint8_t> encode_frame(const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    const size_t len = payload.size();
    const uint16_t crc = crc16(payload);

    if (len <= 255) {
        frame.reserve(len + 5);
        frame.push_back(0x02);
        frame.push_back(static_cast<uint8_t>(len));
    } else {
        frame.reserve(len + 6);
        frame.push_back(0x03);
        frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(len & 0xFF));
    }
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(0x03);
    return frame;
}

void FrameDecoder::feed(const uint8_t* data, size_t len) {
    buffer_.insert(buffer_.end(), data, data + len);
    process();
}

bool FrameDecoder::pop_payload(std::vector<uint8_t>* out) {
    if (payloads_.empty()) {
        return false;
    }
    *out = std::move(payloads_.front());
    payloads_.pop_front();
    return true;
}

void FrameDecoder::process() {
    for (;;) {
        if (buffer_.empty()) {
            return;
        }
        const uint8_t start = buffer_[0];
        if (start != 0x02 && start != 0x03) {
            // Garbage byte -- resync by dropping it and rescanning.
            buffer_.erase(buffer_.begin());
            continue;
        }

        size_t header_len;
        size_t payload_len;
        if (start == 0x02) {
            if (buffer_.size() < 2) {
                return;  // wait for the length byte
            }
            header_len = 2;
            payload_len = buffer_[1];
        } else {
            if (buffer_.size() < 3) {
                return;  // wait for the 2-byte length
            }
            header_len = 3;
            payload_len = (static_cast<size_t>(buffer_[1]) << 8) | static_cast<size_t>(buffer_[2]);
            if (payload_len > kMaxPayloadLen) {
                // Implausible length -- treat the start byte as garbage
                // rather than waiting forever for bytes that may never come.
                buffer_.erase(buffer_.begin());
                continue;
            }
        }

        const size_t frame_len = header_len + payload_len + 2 /* crc */ + 1 /* stop */;
        if (buffer_.size() < frame_len) {
            return;  // wait for the rest of the frame
        }

        const size_t crc_hi_idx = header_len + payload_len;
        const uint8_t stop_byte = buffer_[frame_len - 1];
        const uint16_t crc_received = (static_cast<uint16_t>(buffer_[crc_hi_idx]) << 8) |
                                       static_cast<uint16_t>(buffer_[crc_hi_idx + 1]);
        const uint16_t crc_computed = crc16(buffer_.data() + header_len, payload_len);

        if (stop_byte != 0x03 || crc_received != crc_computed) {
            // Not actually a valid frame at this offset -- drop one byte
            // and keep scanning (resync), don't consume the whole
            // would-be frame.
            buffer_.erase(buffer_.begin());
            continue;
        }

        payloads_.emplace_back(buffer_.begin() + static_cast<long>(header_len),
                                buffer_.begin() + static_cast<long>(header_len + payload_len));
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(frame_len));
    }
}

// ---------------------------------------------------------------------
// Command builders
// ---------------------------------------------------------------------

std::vector<uint8_t> build_fw_version() {
    return {static_cast<uint8_t>(CommandId::FW_VERSION)};
}

std::vector<uint8_t> build_get_values() {
    return {static_cast<uint8_t>(CommandId::GET_VALUES)};
}

std::vector<uint8_t> build_set_duty(double duty) {
    std::vector<uint8_t> out;
    out.push_back(static_cast<uint8_t>(CommandId::SET_DUTY));
    append_be_i32(&out, static_cast<int32_t>(std::lround(duty * 100000.0)));
    return out;
}

std::vector<uint8_t> build_set_current(double amps) {
    std::vector<uint8_t> out;
    out.push_back(static_cast<uint8_t>(CommandId::SET_CURRENT));
    append_be_i32(&out, static_cast<int32_t>(std::lround(amps * 1000.0)));
    return out;
}

std::vector<uint8_t> build_set_current_brake(double amps) {
    std::vector<uint8_t> out;
    out.push_back(static_cast<uint8_t>(CommandId::SET_CURRENT_BRAKE));
    append_be_i32(&out, static_cast<int32_t>(std::lround(amps * 1000.0)));
    return out;
}

std::vector<uint8_t> build_set_rpm(int32_t erpm) {
    std::vector<uint8_t> out;
    out.push_back(static_cast<uint8_t>(CommandId::SET_RPM));
    append_be_i32(&out, erpm);
    return out;
}

std::vector<uint8_t> build_set_servo_pos(double pos) {
    return build_set_servo_pos_with_id(pos, static_cast<uint8_t>(CommandId::SET_SERVO_POS));
}

uint8_t resolve_servo_cmd_id(uint8_t fw_major) {
    return fw_major <= 2 ? 11 : 12;
}

std::vector<uint8_t> build_set_servo_pos(double pos, uint8_t cmd_id) {
    return build_set_servo_pos_with_id(pos, cmd_id);
}

std::vector<uint8_t> build_alive() {
    return {static_cast<uint8_t>(CommandId::ALIVE)};
}

std::vector<uint8_t> build_get_mcconf() {
    return {static_cast<uint8_t>(CommandId::GET_MCCONF)};
}

std::vector<uint8_t> build_get_mcconf_default() {
    return {static_cast<uint8_t>(CommandId::GET_MCCONF_DEFAULT)};
}

std::vector<uint8_t> build_set_mcconf(const std::vector<uint8_t>& blob) {
    std::vector<uint8_t> out;
    out.reserve(blob.size() + 1);
    out.push_back(static_cast<uint8_t>(CommandId::SET_MCCONF));
    out.insert(out.end(), blob.begin(), blob.end());
    return out;
}

// ---------------------------------------------------------------------
// Response parsers
// ---------------------------------------------------------------------

FwVersionReply parse_fw_version(const std::vector<uint8_t>& payload) {
    FwVersionReply out;
    if (payload.size() < 3 || payload[0] != static_cast<uint8_t>(CommandId::FW_VERSION)) {
        return out;  // ok=false
    }
    out.major = payload[1];
    out.minor = payload[2];
    out.ok = true;
    return out;
}

VescValues parse_get_values(const std::vector<uint8_t>& payload) {
    VescValues out;  // ok=false, all-zero default
    if (payload.empty() || payload[0] != static_cast<uint8_t>(CommandId::GET_VALUES)) {
        return out;
    }

    size_t idx = 1;
    const size_t n = payload.size();

    if (idx + 2 > n) return out;
    out.temp_fet = read_be_i16(payload, idx) / 10.0;
    idx += 2;

    if (idx + 2 > n) return out;
    out.temp_motor = read_be_i16(payload, idx) / 10.0;
    idx += 2;

    if (idx + 4 > n) return out;
    out.current_motor = read_be_i32(payload, idx) / 100.0;
    idx += 4;

    if (idx + 4 > n) return out;
    out.current_in = read_be_i32(payload, idx) / 100.0;
    idx += 4;

    if (idx + 4 > n) return out;  // avg_id -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // avg_iq -- consumed for offset, not stored
    idx += 4;

    if (idx + 2 > n) return out;
    out.duty = read_be_i16(payload, idx) / 1000.0;
    idx += 2;

    if (idx + 4 > n) return out;
    out.erpm = read_be_i32(payload, idx);
    idx += 4;

    if (idx + 2 > n) return out;
    out.v_in = read_be_i16(payload, idx) / 10.0;
    idx += 2;

    if (idx + 4 > n) return out;  // amp_hours -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // amp_hours_charged -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // watt_hours -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // watt_hours_charged -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;
    out.tachometer = read_be_i32(payload, idx);
    idx += 4;

    if (idx + 4 > n) return out;
    out.tachometer_abs = read_be_i32(payload, idx);
    idx += 4;

    if (idx + 1 > n) return out;
    out.fault = payload[idx];
    idx += 1;

    // Anything beyond this point (pid_pos_now/controller_id/extra temp
    // channels on some firmware versions) is tolerated and ignored.
    out.ok = true;
    return out;
}

VescValues parse_get_values_legacy(const std::vector<uint8_t>& payload) {
    VescValues out;  // ok=false, all-zero default
    out.has_temp_motor = false;
    if (payload.empty() || payload[0] != static_cast<uint8_t>(CommandId::GET_VALUES)) {
        return out;
    }

    size_t idx = 1;
    const size_t n = payload.size();

    double temp_mos[6];
    for (double& t : temp_mos) {
        if (idx + 2 > n) return out;
        t = read_be_i16(payload, idx) / 10.0;
        idx += 2;
    }
    out.temp_fet = *std::max_element(std::begin(temp_mos), std::end(temp_mos));

    if (idx + 2 > n) return out;  // temp_pcb -- consumed for offset, not stored (no VescValues field for it)
    idx += 2;

    if (idx + 4 > n) return out;
    out.current_motor = read_be_i32(payload, idx) / 100.0;
    idx += 4;

    if (idx + 4 > n) return out;
    out.current_in = read_be_i32(payload, idx) / 100.0;
    idx += 4;

    if (idx + 2 > n) return out;
    out.duty = read_be_i16(payload, idx) / 1000.0;
    idx += 2;

    if (idx + 4 > n) return out;
    out.erpm = read_be_i32(payload, idx);
    idx += 4;

    if (idx + 2 > n) return out;
    out.v_in = read_be_i16(payload, idx) / 10.0;
    idx += 2;

    if (idx + 4 > n) return out;  // amp_hours -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // amp_hours_charged -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // watt_hours -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;  // watt_hours_charged -- consumed for offset, not stored
    idx += 4;

    if (idx + 4 > n) return out;
    out.tachometer = read_be_i32(payload, idx);
    idx += 4;

    if (idx + 4 > n) return out;
    out.tachometer_abs = read_be_i32(payload, idx);
    idx += 4;

    if (idx + 1 > n) return out;
    out.fault = payload[idx];
    idx += 1;

    // Tolerate-longer, same as parse_get_values() above.
    out.ok = true;
    return out;
}

VescValues parse_get_values_for_fw(const std::vector<uint8_t>& payload, uint8_t fw_major) {
    if (fw_major == 2) return parse_get_values_legacy(payload);
    return parse_get_values(payload);
}

McconfReply parse_mcconf_reply(const std::vector<uint8_t>& payload, CommandId expected_id) {
    McconfReply out;
    if (payload.empty() || payload[0] != static_cast<uint8_t>(expected_id)) {
        return out;  // ok=false
    }
    out.blob.assign(payload.begin() + 1, payload.end());
    out.ok = true;
    return out;
}

bool is_set_mcconf_ack(const std::vector<uint8_t>& payload) {
    return !payload.empty() && payload[0] == static_cast<uint8_t>(CommandId::SET_MCCONF);
}

// ---------------------------------------------------------------------
// float32_auto (vesc-project bldc firmware's buffer.c encoding) -- this
// is a byte-for-byte port of buffer_append_float32_auto()/
// buffer_get_float32_auto(), including the encode side's (uint32_t)e
// cast (which relies on well-defined unsigned overflow when `e` is
// large/negative, exactly as the firmware itself does) rather than a
// "cleaned up" reimplementation, so this driver's encoded bytes are
// wire-compatible with real firmware and its decoder accepts real
// firmware's bytes.
// ---------------------------------------------------------------------

uint32_t float32_auto_encode(float value) {
    int32_t e = 0;
    const float sig = frexpf(value, &e);
    const float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;
    if (sig_abs >= 0.5f) {
        sig_i = static_cast<uint32_t>((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }
    uint32_t res = (static_cast<uint32_t>(e) << 23) | (sig_i & 0x7FFFFFu);
    if (sig < 0) {
        res |= (1u << 31);
    }
    return res;
}

float float32_auto_decode(uint32_t value) {
    int32_t e = static_cast<int32_t>((value >> 23) & 0xFFu);
    const uint32_t sig_i = value & 0x7FFFFFu;
    const bool neg = (value & (1u << 31)) != 0;
    float sig = 0.0f;
    if (e != 0 || sig_i != 0) {
        sig = static_cast<float>(sig_i) / (8388608.0f * 2.0f) + 0.5f;
        e -= 126;
    }
    if (neg) {
        sig = -sig;
    }
    return ldexpf(sig, e);
}

void append_be_f32_auto(std::vector<uint8_t>* out, float value) {
    const uint32_t u = float32_auto_encode(value);
    out->push_back(static_cast<uint8_t>((u >> 24) & 0xFF));
    out->push_back(static_cast<uint8_t>((u >> 16) & 0xFF));
    out->push_back(static_cast<uint8_t>((u >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(u & 0xFF));
}

float read_be_f32_auto(const std::vector<uint8_t>& buf, size_t idx) {
    const uint32_t u = (static_cast<uint32_t>(buf[idx]) << 24) |
                        (static_cast<uint32_t>(buf[idx + 1]) << 16) |
                        (static_cast<uint32_t>(buf[idx + 2]) << 8) |
                        static_cast<uint32_t>(buf[idx + 3]);
    return float32_auto_decode(u);
}

}  // namespace vesc
