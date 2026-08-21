#include "McconfPatcher.h"

#include <cmath>

namespace vesc {

namespace {

// Plausible-range checks. Every bound below is a "no real motor
// configuration could sanely have a value outside this" sanity check,
// not a tight validity check -- the goal is to reject obvious noise
// (including non-finite float32_auto decodes of unrelated bytes), not to
// validate a value is well-tuned.
bool in_range(double v, double lo, double hi) { return std::isfinite(v) && v >= lo && v <= hi; }

// Reads 2 big-endian bytes at buf[idx..idx+2) as a raw uint16. Caller
// must have already verified 2 bytes remain at idx. (VescProtocol.h only
// exposes a float32_auto u32 reader; the mcconf blob's kd_filter and the
// two unknown current-limit fields need a plain raw-u16 read instead.)
uint16_t read_be_u16(const std::vector<uint8_t>& buf, size_t idx) {
    return static_cast<uint16_t>((static_cast<uint16_t>(buf[idx]) << 8) | static_cast<uint16_t>(buf[idx + 1]));
}

void append_be_u16_raw(std::vector<uint8_t>* out, uint16_t value) {
    out->push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out->push_back(static_cast<uint8_t>(value & 0xFF));
}

}  // namespace

std::vector<SpeedPidMatch> scan_speed_pid(const std::vector<uint8_t>& blob) {
    std::vector<SpeedPidMatch> matches;
    if (blob.size() < kSpeedPidClusterBytes) return matches;

    for (size_t offset = 0; offset + kSpeedPidClusterBytes <= blob.size(); ++offset) {
        const double kp = read_be_f32_auto(blob, offset + 0);
        const double ki = read_be_f32_auto(blob, offset + 4);
        const double kd = read_be_f32_auto(blob, offset + 8);
        // kd_filter is a 2-byte fixed-point field (raw_u16 / 1e4), NOT
        // float32_auto -- see McconfPatcher.h. Every field below this
        // one is therefore offset 2 bytes earlier than the old (wrong)
        // 4-byte-float32_auto-assumed layout.
        const uint16_t kd_filter_raw = read_be_u16(blob, offset + 12);
        const double kd_filter = static_cast<double>(kd_filter_raw) / 1e4;
        const double min_erpm = read_be_f32_auto(blob, offset + 14);
        const uint8_t allow_braking = blob[offset + 18];
        const double ramp = read_be_f32_auto(blob, offset + 19);

        if (!in_range(kp, 1e-6, 1.0)) continue;
        if (!in_range(ki, 1e-6, 50.0)) continue;
        if (!in_range(kd, 0.0, 0.5)) continue;
        if (!in_range(kd_filter, 0.01, 0.9)) continue;
        if (!in_range(min_erpm, 10.0, 50000.0)) continue;
        if (allow_braking != 0 && allow_braking != 1) continue;
        if (!in_range(ramp, 10.0, 1e7)) continue;

        SpeedPidMatch m;
        m.offset = offset;
        m.kp = kp;
        m.ki = ki;
        m.kd = kd;
        m.kd_filter = kd_filter;
        m.min_erpm = min_erpm;
        m.allow_braking = allow_braking;
        m.ramp_erpms_s = ramp;
        matches.push_back(m);
    }
    return matches;
}

std::vector<CurrentLimitsMatch> scan_current_limits(const std::vector<uint8_t>& blob) {
    std::vector<CurrentLimitsMatch> matches;
    if (blob.size() < kCurrentLimitsClusterBytes) return matches;

    for (size_t offset = 0; offset + kCurrentLimitsClusterBytes <= blob.size(); ++offset) {
        const double current_max = read_be_f32_auto(blob, offset + 0);
        const double current_min = read_be_f32_auto(blob, offset + 4);
        const double in_current_max = read_be_f32_auto(blob, offset + 8);
        const double in_current_min = read_be_f32_auto(blob, offset + 12);
        // Two raw big-endian u16 fields of unknown firmware name -- see
        // McconfPatcher.h. Missing these entirely was the old (wrong)
        // layout's bug: it silently read abs_current_max 4 bytes too
        // early.
        const uint16_t field_a_raw = read_be_u16(blob, offset + 16);
        const uint16_t field_b_raw = read_be_u16(blob, offset + 18);
        const double abs_current_max = read_be_f32_auto(blob, offset + 20);

        if (!in_range(current_max, 1.0, 500.0)) continue;
        if (!in_range(current_min, -500.0, -0.1)) continue;
        if (!in_range(in_current_max, 0.1, 500.0)) continue;
        if (!in_range(in_current_min, -500.0, 0.0)) continue;
        // Loose scaled plausible ranges, purely to strengthen the
        // cluster match (reject obvious noise) -- see McconfPatcher.h.
        if (!in_range(static_cast<double>(field_a_raw) / 1000.0, 0.0, 100.0)) continue;
        if (!in_range(static_cast<double>(field_b_raw) / 1000.0, 0.0, 10.0)) continue;
        if (!in_range(abs_current_max, current_max, 1000.0)) continue;

        CurrentLimitsMatch m;
        m.offset = offset;
        m.current_max = current_max;
        m.current_min = current_min;
        m.in_current_max = in_current_max;
        m.in_current_min = in_current_min;
        m.unknown_field_a_raw = field_a_raw;
        m.unknown_field_b_raw = field_b_raw;
        m.abs_current_max = abs_current_max;
        matches.push_back(m);
    }
    return matches;
}

std::vector<uint8_t> patch(const std::vector<uint8_t>& blob, size_t field_offset, float new_value) {
    std::vector<uint8_t> out = blob;
    if (field_offset + 4 > out.size()) {
        return out;  // no-op: caller passed an out-of-range offset.
    }
    const uint32_t enc = float32_auto_encode(new_value);
    out[field_offset + 0] = static_cast<uint8_t>((enc >> 24) & 0xFF);
    out[field_offset + 1] = static_cast<uint8_t>((enc >> 16) & 0xFF);
    out[field_offset + 2] = static_cast<uint8_t>((enc >> 8) & 0xFF);
    out[field_offset + 3] = static_cast<uint8_t>(enc & 0xFF);
    return out;
}

std::vector<uint8_t> patch_kd_filter(const std::vector<uint8_t>& blob, size_t field_offset, double new_value) {
    std::vector<uint8_t> out = blob;
    if (field_offset + 2 > out.size()) {
        return out;  // no-op: caller passed an out-of-range offset.
    }
    double scaled = new_value * 1e4;
    if (scaled < 0.0) scaled = 0.0;
    if (scaled > 65535.0) scaled = 65535.0;
    const uint16_t raw = static_cast<uint16_t>(std::lround(scaled));
    out[field_offset + 0] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    out[field_offset + 1] = static_cast<uint8_t>(raw & 0xFF);
    return out;
}

std::vector<uint8_t> build_synthetic_mcconf_blob(SyntheticMcconfLayout* out_layout) {
    std::vector<uint8_t> blob;

    // 4 opaque "signature" bytes (stand-in for whatever real fields
    // precede the current-limits cluster in a real mcconf blob).
    blob.insert(blob.end(), {0xDE, 0xAD, 0xBE, 0xEF});

    // Filler: constant 0xFF. Deliberately chosen so ANY 4-byte-aligned
    // (or misaligned) float32_auto decode of a run of 0xFF bytes yields
    // its exponent field maxed out (0xFF), which decodes to a non-finite
    // (infinite) float -- guaranteed to fail every range check above, no
    // matter where a scan window happens to land inside a filler run.
    auto add_filler = [&blob](size_t n) {
        for (size_t i = 0; i < n; ++i) blob.push_back(0xFF);
    };

    add_filler(16);

    const size_t current_offset = blob.size();
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kCurrentMax));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kCurrentMin));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kInCurrentMax));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kInCurrentMin));
    append_be_u16_raw(&blob, SyntheticMcconfValues::kUnknownFieldARaw);
    append_be_u16_raw(&blob, SyntheticMcconfValues::kUnknownFieldBRaw);
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kAbsCurrentMax));

    add_filler(22);

    const size_t pid_offset = blob.size();
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kKp));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kKi));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kKd));
    // kd_filter: 2-byte fixed-point (raw_u16 = round(value*1e4)), NOT
    // float32_auto -- see McconfPatcher.h.
    append_be_u16_raw(&blob, static_cast<uint16_t>(std::lround(SyntheticMcconfValues::kKdFilter * 1e4)));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kMinErpm));
    blob.push_back(static_cast<uint8_t>(SyntheticMcconfValues::kAllowBraking));
    append_be_f32_auto(&blob, static_cast<float>(SyntheticMcconfValues::kRampErpmsS));

    add_filler(18);

    if (out_layout) {
        out_layout->current_limits_offset = current_offset;
        out_layout->speed_pid_offset = pid_offset;
    }
    return blob;
}

}  // namespace vesc
