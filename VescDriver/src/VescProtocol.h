// VescProtocol.h
//
// VESC UART packet codec: pure bytes-in/bytes-out. No serial I/O lives here
// -- this file only knows how to frame/unframe byte buffers and how to
// encode/decode the specific VESC commands this driver uses. The actual
// termios/tty read-write loop is a later part of this driver.
//
// Portability: C++14 only, no dependencies outside this file's own
// standard-library includes (see VescDriver/CMakeLists.txt / repo-root
// HARD PORTABILITY RULES for the Jetson Nano / Ubuntu 18.04 target).
//
// ---------------------------------------------------------------------
// FRAMING
// ---------------------------------------------------------------------
// Short packet (payload length <= 255):
//   [0x02, len:1B, payload..., crc_hi, crc_lo, 0x03]
// Long packet (payload length > 255):
//   [0x03, len_hi:1B, len_lo:1B, payload..., crc_hi, crc_lo, 0x03]
// (len is big-endian in the long form.)
//
// CRC16 is CRC-16/XMODEM: poly 0x1021, init 0x0000, no input/output
// reflection, no final XOR. It is computed over the payload bytes ONLY
// (never over the start/length/stop bytes).
//
// ---------------------------------------------------------------------
// COMMAND IDS / PAYLOAD LAYOUTS -- ALL MULTI-BYTE VALUES ARE BIG-ENDIAN
// ---------------------------------------------------------------------
// A VESC "payload" is [command_id:u8, args...]. Below is every command this
// driver builds, with its id and the exact byte layout of its args, cross-
// checked against the author's own knowledge of the vesc-project bldc
// firmware (commands.c / packet.c / confgenerator.c / datatypes.h). Firmware
// versions do drift (fields get added at the *end* of GET_VALUES over the
// years, and rare forks renumber commands) -- anywhere below marked with
// "CAVEAT" is a spot the author is not 100% certain of and a later
// integration pass against a real VESC + real firmware version should
// double check.
//
//   COMM_FW_VERSION      = 0   payload: [0]                         (empty args; request only)
//   COMM_GET_VALUES      = 4   payload: [4]                         (empty args; request only)
//   COMM_SET_DUTY        = 5   payload: [5, duty*100000  as i32]    (duty in roughly [-1,1])
//   COMM_SET_CURRENT     = 6   payload: [6, amps*1000    as i32]
//   COMM_SET_CURRENT_BRAKE = 7 payload: [7, amps*1000    as i32]
//   COMM_SET_RPM          = 8  payload: [8, erpm         as i32]    (electrical RPM, NOT mechanical)
//   COMM_SET_SERVO_POS    = 12 payload: [12, pos*1000    as u16]    (pos clamped to [0,1] before scaling)
//   COMM_SET_MCCONF        = 13 payload: [13, mcconf_blob...]        (full motor-config blob write; see
//                                                                     McconfPatcher.h for the blob's internal
//                                                                     field layout used by vesc_mcconf)
//   COMM_GET_MCCONF         = 14 payload: [14]                       (empty args; request only)
//   COMM_GET_MCCONF_DEFAULT = 15 payload: [15]                       (empty args; request only -- firmware's
//                                                                     built-in default mcconf, not currently
//                                                                     used by vesc_mcconf but exposed for
//                                                                     completeness)
//   COMM_ALIVE            = 30 payload: [30]                        (empty args; keepalive/watchdog pet)
//
// CAVEAT: the command id numbering above (0,4,5,6,7,8,12,13,14,15,30)
// matches this task's own specification and the author's recollection of
// datatypes.h's COMM_PACKET_ID enum ordering (COMM_FW_VERSION=0,
// COMM_JUMP_TO_BOOTLOADER=1, COMM_ERASE_NEW_APP=2, COMM_WRITE_NEW_APP_DATA=3,
// COMM_GET_VALUES=4, COMM_SET_DUTY=5, COMM_SET_CURRENT=6,
// COMM_SET_CURRENT_BRAKE=7, COMM_SET_RPM=8, COMM_SET_POS=9,
// COMM_SET_HANDBRAKE=10, COMM_SET_DETECT=11, COMM_SET_SERVO_POS=12,
// COMM_SET_MCCONF=13, COMM_GET_MCCONF=14, COMM_GET_MCCONF_DEFAULT=15, ...,
// COMM_ALIVE=30) -- this is believed correct for the mainline vesc-project
// bldc firmware, but has NOT been cross-checked against a live device in
// this task and should be verified against the actual firmware version
// flashed on the target VESC before field use. The mcconf blob's internal
// field layout (used to locate/patch speed-PID gains and current limits)
// is a further, separate CAVEAT documented in McconfPatcher.h -- it
// fingerprints the field order rather than trusting a fixed byte offset,
// specifically to detect (not silently misfire on) a firmware mismatch.
//
// CAVEAT (FW 2.x id shift): the ordering above (..., SET_POS=9,
// SET_HANDBRAKE=10, SET_DETECT=11, SET_SERVO_POS=12, SET_MCCONF=13, ...) is
// the MODERN (FW 5.x/6.x) ordering. The target hardware for this project is
// an old VESC running FW 2.18 that cannot be reflashed, and FW 2.x predates
// COMM_SET_HANDBRAKE entirely -- it was added in a later firmware revision
// -- so on FW 2.x every id from COMM_SET_DETECT onward is shifted down by
// one: SET_DETECT=10, SET_SERVO_POS=11, SET_MCCONF=12, GET_MCCONF=13, ...
// Sending SET_SERVO_POS's fixed id (12) to an FW 2.x unit therefore lands
// on COMM_SET_MCCONF instead -- a full motor-config-blob WRITE command --
// which is a real, live hazard, not a theoretical one. See
// resolve_servo_cmd_id()/build_set_servo_pos(pos, cmd_id) below for the
// fw-aware fix and that function's own CAVEAT comment for confidence level.
//
// ---------------------------------------------------------------------
// RESPONSES
// ---------------------------------------------------------------------
// FW_VERSION reply payload: [0, major:u8, minor:u8, ...trailing ignored]
//
// GET_VALUES reply payload (modern FW 5.x/6.x layout; all multi-byte
// fields big-endian):
//   [0]      = 4                              (echoed command id)
//   [1..2]   temp_fet            i16 / 10
//   [3..4]   temp_motor          i16 / 10
//   [5..8]   avg_motor_current   i32 / 100
//   [9..12]  avg_input_current   i32 / 100
//   [13..16] avg_id              i32 / 100     (consumed for offset, not stored)
//   [17..20] avg_iq              i32 / 100     (consumed for offset, not stored)
//   [21..22] duty_now            i16 / 1000
//   [23..26] rpm                 i32           (electrical RPM, no scaling)
//   [27..28] v_in                i16 / 10
//   [29..32] amp_hours           i32 / 1e4     (consumed for offset, not stored)
//   [33..36] amp_hours_charged   i32 / 1e4     (consumed for offset, not stored)
//   [37..40] watt_hours          i32 / 1e4     (consumed for offset, not stored)
//   [41..44] watt_hours_charged  i32 / 1e4     (consumed for offset, not stored)
//   [45..48] tachometer          i32
//   [49..52] tachometer_abs      i32
//   [53]     fault               u8
//   [54...]  CAVEAT: some firmware versions append pid_pos_now (i32/1e6),
//            controller_id (u8), and extra MOSFET/battery temp channels
//            here. This parser treats anything after `fault` as optional
//            trailing bytes and ignores them (tolerate-longer contract),
//            so it works whether or not those extra fields are present.
//
// Parsing is defensive in both directions: if the payload is SHORTER than
// needed, parsing stops cleanly at the point it runs out of bytes and
// reports failure (ok=false) rather than reading out of bounds; if the
// payload is LONGER than needed, the extra trailing bytes are ignored.

#ifndef VESC_DRIVER_VESC_PROTOCOL_H_
#define VESC_DRIVER_VESC_PROTOCOL_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace vesc {

// ---------------------------------------------------------------------
// CRC16 (CRC-16/XMODEM: poly 0x1021, init 0x0000)
// ---------------------------------------------------------------------

// Table-driven CRC16/XMODEM over `data[0..len)`. The table is built lazily
// once (function-local static, thread-safe init under C++14) rather than
// hand-written, but the algorithm it implements is the standard bit-by-bit
// CRC-16/XMODEM definition -- crc16(reinterpret_cast<const uint8_t*>("123456789"), 9) == 0x31C3.
uint16_t crc16(const uint8_t* data, size_t len);
inline uint16_t crc16(const std::vector<uint8_t>& data) {
    return crc16(data.data(), data.size());
}

// ---------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------

// Wraps `payload` in a short (len<=255) or long (len>255) VESC frame,
// including the CRC16 and start/stop bytes, ready to write to the UART.
std::vector<uint8_t> encode_frame(const std::vector<uint8_t>& payload);

// Streaming frame decoder: feed it raw bytes (however they arrive off the
// wire -- one at a time or in arbitrary chunks, possibly split mid-frame)
// and it emits complete, CRC-valid payloads. Garbage bytes (a stray byte
// that isn't a valid start byte, or a would-be frame whose stop byte/CRC
// doesn't check out) are silently dropped one byte at a time so the
// decoder resynchronizes onto the next real frame rather than getting
// stuck.
class FrameDecoder {
public:
    // Appends `len` bytes at `data` to the decoder's internal buffer and
    // extracts as many complete, valid payloads as are now available (they
    // are queued for pop_payload()).
    void feed(const uint8_t* data, size_t len);
    void feed(const std::vector<uint8_t>& data) { feed(data.data(), data.size()); }

    // Pops the oldest decoded payload into `*out` and returns true, or
    // returns false (leaving `*out` untouched) if no complete payload is
    // queued yet.
    bool pop_payload(std::vector<uint8_t>* out);

    // Number of complete payloads currently queued.
    size_t pending_count() const { return payloads_.size(); }

private:
    void process();

    std::vector<uint8_t> buffer_;
    std::deque<std::vector<uint8_t>> payloads_;
};

// ---------------------------------------------------------------------
// Command ids
// ---------------------------------------------------------------------

enum class CommandId : uint8_t {
    FW_VERSION = 0,
    GET_VALUES = 4,
    SET_DUTY = 5,
    SET_CURRENT = 6,
    SET_CURRENT_BRAKE = 7,
    SET_RPM = 8,
    SET_SERVO_POS = 12,
    SET_MCCONF = 13,
    GET_MCCONF = 14,
    GET_MCCONF_DEFAULT = 15,
    ALIVE = 30,
};

// ---------------------------------------------------------------------
// Command builders -- each returns the PAYLOAD bytes only (command id
// byte followed by big-endian-encoded args). Pass the result to
// encode_frame() to get the bytes actually written to the UART.
// ---------------------------------------------------------------------

std::vector<uint8_t> build_fw_version();
std::vector<uint8_t> build_get_values();
std::vector<uint8_t> build_set_duty(double duty);              // arg: int32 round(duty * 100000)
std::vector<uint8_t> build_set_current(double amps);            // arg: int32 round(amps * 1000)
std::vector<uint8_t> build_set_current_brake(double amps);      // arg: int32 round(amps * 1000)
std::vector<uint8_t> build_set_rpm(int32_t erpm);                // arg: int32 erpm (no scaling)
std::vector<uint8_t> build_set_servo_pos(double pos);            // arg: uint16 round(clamp(pos,0,1) * 1000)

// Resolves which raw command id SET_SERVO_POS should be encoded as, given
// the VESC's own reported firmware major version (see PortDiscovery.h's
// DiscoveryResult::fw_major / FwVersionReply::major). FW 2.x lacks
// COMM_SET_HANDBRAKE (added in a later firmware revision), so every id
// from COMM_SET_DETECT onward is shifted down by one relative to the
// modern (FW5.x/6.x) ordering this file's CommandId enum otherwise uses:
//   fw_major <= 2  -> 11  (FW2.x: ... SET_DETECT=10, SET_SERVO_POS=11, SET_MCCONF=12, ...)
//   fw_major >= 3  -> 12  (modern: ... SET_HANDBRAKE=10, SET_DETECT=11, SET_SERVO_POS=12, ...)
// CONFIDENCE: HIGH that the shift itself is real -- this is standard,
// well-documented vesc-project firmware history (COMM_SET_HANDBRAKE was
// added to datatypes.h's COMM_PACKET_ID enum after the original command
// set, after which every later id shifted down by one on unpatched old
// firmware). LOWER confidence on the fw_major>=3 boundary specifically:
// FW 3.x/4.x are ASSUMED to already be on the modern ordering, consistent
// with this file's own parse_get_values_for_fw() CAVEAT which makes the
// same fw_major==2-vs-not assumption for GET_VALUES's layout -- neither
// has been verified against a real 3.x/4.x unit, only a real FW 2.18 one.
// Re-verify against real hardware if a 3.x/4.x unit becomes available.
uint8_t resolve_servo_cmd_id(uint8_t fw_major);

// Builds a SET_SERVO_POS payload using an EXPLICIT command id (see
// resolve_servo_cmd_id() above) instead of the fixed CommandId::SET_SERVO_POS
// (12). Callers talking to firmware whose exact id is uncertain (i.e.
// anything that isn't confirmed modern FW5.x/6.x) MUST use this overload
// with a resolved (or explicitly user-overridden) id -- see this file's
// own CAVEAT comment above resolve_servo_cmd_id() -- never the fixed-id
// build_set_servo_pos(double) below, to avoid the COMM_SET_MCCONF hazard
// documented there.
std::vector<uint8_t> build_set_servo_pos(double pos, uint8_t cmd_id);

std::vector<uint8_t> build_alive();

// Motor-config (mcconf) commands -- see McconfPatcher.h for the blob's
// internal field layout. GET_MCCONF/GET_MCCONF_DEFAULT requests carry no
// args (just the command id byte); build_set_mcconf() carries the full
// blob to write as its args.
std::vector<uint8_t> build_get_mcconf();
std::vector<uint8_t> build_get_mcconf_default();
std::vector<uint8_t> build_set_mcconf(const std::vector<uint8_t>& blob);

// ---------------------------------------------------------------------
// Response parsers
// ---------------------------------------------------------------------

struct FwVersionReply {
    uint8_t major = 0;
    uint8_t minor = 0;
    bool ok = false;
};

// Parses an FW_VERSION reply payload. Requires payload[0] == 0 and at
// least 3 bytes total; trailing bytes beyond major/minor are ignored.
FwVersionReply parse_fw_version(const std::vector<uint8_t>& payload);

struct VescValues {
    double temp_fet = 0.0;
    double temp_motor = 0.0;
    bool has_temp_motor = true;  // false on FW 2.x (parse_get_values_legacy()) -- that layout
                                  // reports no motor temp sensor at all; temp_motor is left at
                                  // 0.0 (not NaN -- see this driver's general aversion to NaN
                                  // propagating through display/telemetry values) and callers
                                  // should treat it as "unavailable", not "0 degrees".
    double current_motor = 0.0;  // avg_motor_current
    double current_in = 0.0;     // avg_input_current
    double duty = 0.0;           // duty_now
    int32_t erpm = 0;            // rpm (electrical RPM)
    double v_in = 0.0;
    int32_t tachometer = 0;
    int32_t tachometer_abs = 0;
    uint8_t fault = 0;
    bool ok = false;
};

// Parses a GET_VALUES reply payload (see the file-header layout table
// above). Requires payload[0] == 4. Stops cleanly (ok=false, fields read
// so far left populated, remaining fields left at their struct defaults)
// if the payload runs out of bytes before `fault`; ignores any bytes
// after `fault` (tolerates newer-firmware trailing fields).
VescValues parse_get_values(const std::vector<uint8_t>& payload);

// ---------------------------------------------------------------------
// LEGACY (FW 2.x) GET_VALUES layout
// ---------------------------------------------------------------------
// Firmware 2.x (confirmed against a real unit reporting FW 2.18, whose
// hardware cannot be reflashed to a newer major version) predates the
// avg_id/avg_iq fields and the single temp_fet/temp_motor pair used by
// the modern (FW 5.x/6.x) layout above -- it instead reports up to 6
// per-MOSFET temperature sensors (temp_mos1..temp_mos6, matching VESC
// 4/6 hardware's multi-sensor FET temp wiring) plus a separate PCB temp,
// and no motor temperature sensor at all. All multi-byte fields are
// big-endian, same as the modern layout; "i16"/"i32" below denote a
// 2-byte/4-byte signed integer divided by the given scale (this
// protocol has no raw IEEE-754 floats on the wire outside the mcconf
// blob's own float32_auto encoding -- see that section further below).
//
// CAVEAT: this field order is reconstructed from the author's knowledge
// of the vesc-project bldc firmware's older commands.c COMM_GET_VALUES
// handler (pre the avg_id/avg_iq/consolidated-temp rework) and cross-
// checked against live evidence from a real FW 2.18 unit (a naive
// modern-layout parse of its reply read a "plausible" ~28.8C out of
// what the modern layout calls temp_fet at payload offset [1..2] --
// consistent with THIS layout's first field, temp_mos1, living at that
// same offset) -- it has NOT been byte-for-byte verified against the
// upstream bldc firmware source for exactly this version and should be
// re-checked against a real 2.x unit if downstream values look
// implausible.
//
//   [0]      = 4                              (echoed command id)
//   [1..2]   temp_mos1            i16 / 10
//   [3..4]   temp_mos2            i16 / 10
//   [5..6]   temp_mos3            i16 / 10
//   [7..8]   temp_mos4            i16 / 10
//   [9..10]  temp_mos5            i16 / 10
//   [11..12] temp_mos6            i16 / 10
//   [13..14] temp_pcb             i16 / 10
//   [15..18] current_motor        i32 / 100
//   [19..22] current_in           i32 / 100
//   [23..24] duty_now             i16 / 1000
//   [25..28] rpm                  i32           (electrical RPM, no scaling)
//   [29..30] v_in                 i16 / 10
//   [31..34] amp_hours            i32 / 1e4     (consumed for offset, not stored)
//   [35..38] amp_hours_charged    i32 / 1e4     (consumed for offset, not stored)
//   [39..42] watt_hours           i32 / 1e4     (consumed for offset, not stored)
//   [43..46] watt_hours_charged   i32 / 1e4     (consumed for offset, not stored)
//   [47..50] tachometer           i32
//   [51..54] tachometer_abs       i32
//   [55]     fault                u8
//
// Mapped into VescValues: temp_fet := max(temp_mos1..temp_mos6) (the
// modern layout's temp_fet is itself "the" FET temp on hardware with a
// single sensor, so max() is the closest single-value equivalent on
// hardware with several); temp_motor left at 0.0 with has_temp_motor set
// to false (this layout has no motor temp sensor); everything else maps
// directly. Same defensive short-payload/trailing-bytes handling as
// parse_get_values() above.
VescValues parse_get_values_legacy(const std::vector<uint8_t>& payload);

// Dispatches to parse_get_values_legacy() for fw_major==2, else to the
// modern parse_get_values() above. FW 3.x/4.x are ASSUMED to already use
// the modern layout (the avg_id/avg_iq/consolidated-temp rework predates
// this driver's own knowledge of exactly which minor version introduced
// it, and no FW 3.x/4.x unit has been available to confirm) -- treat a
// mismatch here the same way as any other CAVEAT in this file: verify
// against a real unit before trusting it if one becomes available.
VescValues parse_get_values_for_fw(const std::vector<uint8_t>& payload, uint8_t fw_major);

// Result of parsing a GET_MCCONF or GET_MCCONF_DEFAULT reply: the blob is
// everything after the echoed command id byte.
struct McconfReply {
    std::vector<uint8_t> blob;
    bool ok = false;
};

// Parses a GET_MCCONF (or, with expected_id=GET_MCCONF_DEFAULT, a
// GET_MCCONF_DEFAULT) reply payload: requires payload[0] == expected_id
// and returns everything after it as the blob. Empty blob (payload of
// exactly 1 byte) is accepted as ok=true with an empty blob -- callers
// that need a non-trivial blob should check blob.size() themselves.
McconfReply parse_mcconf_reply(const std::vector<uint8_t>& payload,
                                CommandId expected_id = CommandId::GET_MCCONF);

// True iff `payload` looks like a SET_MCCONF ack (payload[0] == 13).
// Per this driver's own contract, callers must NOT trust this ack alone
// as proof the write succeeded -- always re-GET_MCCONF and verify the
// blob byte-for-byte (see vesc_mcconf_main.cpp's patch/restore commands).
bool is_set_mcconf_ack(const std::vector<uint8_t>& payload);

// ---------------------------------------------------------------------
// float32_auto -- the vesc-project bldc firmware's compact float
// encoding used inside the mcconf blob (buffer.c's
// buffer_append_float32_auto/buffer_get_float32_auto). NOT IEEE-754: a
// custom sign/exponent/23-bit-significand packing chosen by the firmware
// (this driver reproduces its exact bit manipulation, including the
// firmware's own (uint32_t)e truncation-via-overflow behavior on the
// encode side, rather than a "cleaned up" reinterpretation, so encoded
// bytes are wire-compatible with real firmware).
// ---------------------------------------------------------------------

uint32_t float32_auto_encode(float value);
float float32_auto_decode(uint32_t value);

// Appends `value`'s float32_auto encoding as 4 big-endian bytes.
void append_be_f32_auto(std::vector<uint8_t>* out, float value);

// Reads 4 big-endian bytes at buf[idx..idx+4) as a float32_auto value.
// Caller must have already verified 4 bytes remain at idx.
float read_be_f32_auto(const std::vector<uint8_t>& buf, size_t idx);

}  // namespace vesc

#endif  // VESC_DRIVER_VESC_PROTOCOL_H_
