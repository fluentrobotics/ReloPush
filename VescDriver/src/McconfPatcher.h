// McconfPatcher.h
//
// Pure logic (no I/O) for locating and patching known field clusters
// inside a raw VESC motor-config (mcconf) blob, as returned by
// GET_MCCONF / written by SET_MCCONF (see VescProtocol.h).
//
// WHY SCAN INSTEAD OF USING A FIXED OFFSET: the mcconf blob's field
// layout is confgenerator.c's serialization of the firmware's internal
// mc_configuration struct, which drifts across firmware versions (fields
// get added/reordered/removed). Rather than hard-coding a byte offset
// (silently wrong -- and dangerous, since this writes motor-control
// gains -- on a firmware mismatch), this scans every offset in the blob
// for a cluster of consecutive fields whose *values* are all plausible
// for their known field order (FW 6.x mcconf layout, per this task's own
// specification). A firmware version whose field order/count differs
// enough will simply fail to produce a clean single match (zero matches,
// because no offset's values are all simultaneously plausible; or
// multiple matches, because plausible-looking noise happens to line up
// more than once) -- both are treated as AMBIGUOUS by the CLI and refuse
// to patch, rather than writing to the wrong bytes.
//
// GROUND TRUTH: the field order/offsets/ranges below WERE cross-checked
// against a real GET_MCCONF dump from a real FW 6.6 VESC (hand-decoded by
// a teammate, then independently re-derived byte-for-byte against
// VescProtocol.h's float32_auto codec while fixing this file -- see
// tests/fixtures/mcconf_fw66_real.bin and
// test_mcconf_patcher_real_fw66_fixture() in vesc_driver_tests.cpp, which
// asserts every field below at its exact byte offset). Earlier revisions
// of this file used a GUESSED layout that was WRONG in two ways the real
// dump exposed: (1) the current-limits cluster has two extra 16-bit
// fields between l_in_current_min and l_abs_current_max that were
// missing entirely, silently shifting every field read after them; (2)
// s_pid_kd_filter is NOT a 4-byte float32_auto field like its neighbors
// -- it is a 2-byte big-endian fixed-point field (raw_u16 / 1e4), which
// shifts every speed-PID field after it back by 2 bytes relative to the
// old (wrong) layout. Both are fixed below. Other firmware versions may
// still drift from this exact layout -- that is exactly why this scans
// for a plausible cluster instead of trusting a fixed byte offset; a
// firmware mismatch should still surface as 0 or >1 matches (AMBIGUOUS),
// never a silent wrong-offset write.
//
// Portability: C++14 only, no dependencies outside VescProtocol.h (for
// float32_auto) -- see VescDriver/CMakeLists.txt's HARD PORTABILITY
// RULES.

#ifndef VESC_DRIVER_MCCONF_PATCHER_H_
#define VESC_DRIVER_MCCONF_PATCHER_H_

#include "VescProtocol.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vesc {

// ---------------------------------------------------------------------
// Speed-PID cluster: s_pid_kp, s_pid_ki, s_pid_kd, s_pid_kd_filter,
// s_pid_min_erpm, s_pid_allow_braking, s_pid_ramp_erpms_s, in that exact
// order -- 5 float32_auto fields (4 bytes each) + 1 "float16-style"
// fixed-point field (kd_filter, 2 bytes, see below) + 1 raw byte
// (allow_braking) = 23 bytes total.
//
// s_pid_kd_filter is NOT float32_auto: it is 2 big-endian bytes decoded
// as raw_u16 / 1e4 (e.g. real-hardware raw 0x07D0 = 2000 -> 0.2). This
// was the single biggest layout error in the earlier guessed version of
// this file (which treated it as a 4-byte float32_auto field, silently
// misreading/miswriting every field after it).
// ---------------------------------------------------------------------

struct SpeedPidMatch {
    size_t offset = 0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double kd_filter = 0.0;  // decoded as raw_u16 / 1e4 -- see above, NOT float32_auto.
    double min_erpm = 0.0;
    uint8_t allow_braking = 0;
    double ramp_erpms_s = 0.0;
};

// Total byte size of one speed-PID cluster: kp/ki/kd (3*4 float32_auto)
// + kd_filter (2, fixed-point u16) + min_erpm (4, float32_auto) +
// allow_braking (1, raw byte) + ramp_erpms_s (4, float32_auto).
constexpr size_t kSpeedPidClusterBytes = 3 * 4 + 2 + 4 + 1 + 4;

// Slides over every byte offset in `blob` and decodes a candidate
// 7-field cluster in the fixed order documented above; an offset is a
// match only if EVERY field passes its plausible-range check (see
// McconfPatcher.cpp for the exact bounds). Returns every matching offset
// (usually 0, exactly 1, or -- on ambiguous/decoy-heavy input -- more
// than 1); callers must treat anything other than exactly 1 match as
// "not usable" (see is_unambiguous()).
std::vector<SpeedPidMatch> scan_speed_pid(const std::vector<uint8_t>& blob);

// ---------------------------------------------------------------------
// Current-limits cluster: l_current_max, l_current_min,
// l_in_current_max, l_in_current_min, [two 16-bit fields of currently
// unknown firmware name -- see unknown_field_*_raw below], l_abs_current_max,
// in that exact order -- 4 float32_auto fields (4 bytes each) + 2 raw
// big-endian u16 fields (2 bytes each) + 1 float32_auto field = 24 bytes
// total.
//
// The two u16 fields between l_in_current_min and l_abs_current_max were
// missing entirely from the earlier guessed version of this file, which
// silently misread l_abs_current_max (and anything scanned relative to
// it) 4 bytes too early. Their real firmware field names are not known
// with certainty (real-hardware raw values observed: 10000 and 50,
// plausibly scale/1e3 -> 10.0 and 0.05) -- they are validated with a
// generous scaled plausible range purely to strengthen the cluster match
// (reject obvious noise), not because this driver claims to know what
// they mean.
// ---------------------------------------------------------------------

struct CurrentLimitsMatch {
    size_t offset = 0;
    double current_max = 0.0;
    double current_min = 0.0;
    double in_current_max = 0.0;
    double in_current_min = 0.0;
    uint16_t unknown_field_a_raw = 0;  // raw big-endian u16 @ offset+16; real hardware observed 10000.
    uint16_t unknown_field_b_raw = 0;  // raw big-endian u16 @ offset+18; real hardware observed 50.
    double abs_current_max = 0.0;
};

// Total byte size of one current-limits cluster: current_max/current_min/
// in_current_max/in_current_min (4*4 float32_auto) + the two unknown u16
// fields (2*2) + abs_current_max (4, float32_auto).
constexpr size_t kCurrentLimitsClusterBytes = 4 * 4 + 2 * 2 + 4;

// Same contract as scan_speed_pid(), for the current-limits cluster.
std::vector<CurrentLimitsMatch> scan_current_limits(const std::vector<uint8_t>& blob);

// True iff `matches.size() == 1` -- the shared "is this scan result safe
// to act on" rule used by both the unit tests and the CLI (vesc_mcconf
// refuses to patch any field whose cluster scan is not unambiguous).
template <typename MatchVec>
inline bool is_unambiguous(const MatchVec& matches) {
    return matches.size() == 1;
}

// Returns a NEW blob (does not modify `blob`) with exactly the 4 bytes
// at `field_offset` replaced by the float32_auto encoding of
// `new_value`; every other byte is untouched. If `field_offset + 4`
// would run past the end of `blob`, returns an unmodified copy (no-op) --
// callers are expected to only call this with an offset that came from
// an unambiguous scan match (offset, or offset + a fixed intra-cluster
// byte delta), which is always in range by construction. Only for
// float32_auto fields (every field in both clusters above EXCEPT
// s_pid_kd_filter) -- see patch_kd_filter() for that one.
std::vector<uint8_t> patch(const std::vector<uint8_t>& blob, size_t field_offset, float new_value);

// Same contract as patch(), but for s_pid_kd_filter's 2-byte
// "float16-style" fixed-point encoding: writes exactly 2 bytes at
// `field_offset` as big-endian round(new_value * 1e4), NOT the
// float32_auto encoding used by every other field. If `field_offset + 2`
// would run past the end of `blob`, returns an unmodified copy (no-op),
// same as patch(). `new_value` is clamped into [0, 65535/1e4] before
// encoding so the round-trip never wraps around the u16 range.
std::vector<uint8_t> patch_kd_filter(const std::vector<uint8_t>& blob, size_t field_offset, double new_value);

// ---------------------------------------------------------------------
// Shared synthetic mcconf blob builder -- used identically by
// McconfPatcher's own unit tests and by tests/fake_vesc.cpp, so both
// exercise the SAME known offsets/values without duplicating the
// construction logic. NOT a real firmware dump -- a hand-built stand-in
// blob containing exactly one current-limits cluster and one speed-PID
// cluster (with the values documented in SyntheticMcconfValues below),
// surrounded by non-matching filler bytes.
// ---------------------------------------------------------------------

struct SyntheticMcconfValues {
    // Current-limits cluster values.
    static constexpr double kCurrentMax = 25.0;
    static constexpr double kCurrentMin = -25.0;
    static constexpr double kInCurrentMax = 50.0;
    static constexpr double kInCurrentMin = -20.0;
    // The two u16 fields between in_current_min and abs_current_max
    // (real firmware name unknown -- see CurrentLimitsMatch); values
    // mirror the raw values observed on the real FW 6.6 dump so the
    // synthetic blob exercises the scanner's positional logic the same
    // way real hardware does.
    static constexpr uint16_t kUnknownFieldARaw = 10000;
    static constexpr uint16_t kUnknownFieldBRaw = 50;
    static constexpr double kAbsCurrentMax = 30.0;

    // Speed-PID cluster values.
    static constexpr double kKp = 0.008;
    static constexpr double kKi = 0.008;
    static constexpr double kKd = 0.0;
    static constexpr double kKdFilter = 0.2;
    static constexpr double kMinErpm = 900.0;
    static constexpr uint8_t kAllowBraking = 1;
    static constexpr double kRampErpmsS = 25000.0;
};

// Byte offsets of the two embedded clusters within the blob returned by
// the most recent build_synthetic_mcconf_blob() call.
struct SyntheticMcconfLayout {
    size_t current_limits_offset = 0;
    size_t speed_pid_offset = 0;
};

// Builds a synthetic mcconf blob: 4 opaque "signature" bytes, filler,
// the current-limits cluster (SyntheticMcconfValues), filler, the
// speed-PID cluster (SyntheticMcconfValues), filler. If `out_layout` is
// non-null, it is filled in with the two clusters' byte offsets.
std::vector<uint8_t> build_synthetic_mcconf_blob(SyntheticMcconfLayout* out_layout = nullptr);

}  // namespace vesc

#endif  // VESC_DRIVER_MCCONF_PATCHER_H_
