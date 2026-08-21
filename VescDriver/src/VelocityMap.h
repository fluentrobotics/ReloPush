// VelocityMap.h
//
// Driver v2: monotone piecewise-linear v<->erpm map, loaded from a
// per-robot "velocity_calib.json" file (schema below) -- replaces the
// single fixed erpm_per_mps/cmd_per_mps ratio with a fitted curve when
// one is available, while still degrading gracefully to a linear
// conversion when it isn't. PURE logic + plain fstream/nlohmann::json
// file I/O, same "no sockets/serial" convention as DriverCore.h.
//
// Portability: C++14 only, no std::filesystem -- see
// VescDriver/CMakeLists.txt's HARD PORTABILITY RULES.

#ifndef VESC_DRIVER_VELOCITY_MAP_H_
#define VESC_DRIVER_VELOCITY_MAP_H_

#include <string>
#include <vector>

namespace vesc {

struct VelocityCalibPoint {
    double v = 0.0;
    double erpm = 0.0;
};

struct LinearVelocityFallback {
    double erpm_per_mps = 4614.0;
    double offset_erpm = 0.0;
    double rms = 0.0;
};

// Parsed contents of one "velocity_calib.json" file. Only the fields
// VelocityMap actually consumes are kept (table/min_reliable_erpm/
// linear_fallback's numeric fields) -- "version"/"kind"/"robot_name"/
// "meta" are accepted (not rejected) but not otherwise used.
struct VelocityCalibData {
    int version = 2;
    std::string robot_name;
    // Strictly increasing in BOTH v and erpm, or empty ("no table" ->
    // linear fallback). A non-empty table with fewer than 2 points, or
    // not strictly increasing in either column, is a PARSE FAILURE (see
    // parse_velocity_calib_json()) -- it is never silently accepted as
    // "no table".
    std::vector<VelocityCalibPoint> table;
    double min_reliable_erpm = 250.0;
    LinearVelocityFallback linear_fallback;
};

struct VelocityCalibParseResult {
    bool ok = false;
    VelocityCalibData data;
    std::string error;
};

// Parses one velocity-calib JSON document from an in-memory string (no
// file I/O -- see load_velocity_calib_file() below). ok=false (data left
// default) on: invalid JSON, a top level that isn't an object, or a
// "table" that is PRESENT but malformed (fewer than 2 points, a point
// missing/non-numeric "v"/"erpm", or not strictly ascending in v or in
// erpm). "table" absent entirely is NOT a failure (data.table left
// empty, ok=true) -- the documented "no table -> linear fallback" case.
// "min_reliable_erpm" and every "linear_fallback" sub-field fall back
// independently to their own struct defaults when missing/wrong-typed
// (same tolerant-per-key style as DriverCore.h's load_driver_config()).
VelocityCalibParseResult parse_velocity_calib_json(const std::string& json_text);

// Reads `path` and parses it via parse_velocity_calib_json(). ok=false
// (data left default) if the file cannot be opened for reading OR its
// contents fail to parse.
VelocityCalibParseResult load_velocity_calib_file(const std::string& path);

// Monotone piecewise-linear v<->erpm map. Two modes:
//   - TABLE mode (has_table()==true): erpm_for_velocity()/
//     velocity_for_erpm() linearly interpolate between the table's own
//     points, CLAMPING beyond either end (never extrapolating) -- e.g. a
//     query below table.front().v returns table.front().erpm exactly.
//   - LINEAR mode (has_table()==false, the default-constructed case, or
//     whenever the loaded file had no table): erpm = offset_erpm +
//     erpm_per_mps*v (and the algebraic inverse for velocity_for_erpm();
//     an erpm_per_mps too close to 0 makes velocity_for_erpm() return 0
//     rather than divide by (near) zero).
class VelocityMap {
   public:
    VelocityMap();  // linear mode, LinearVelocityFallback{}'s own defaults.
    explicit VelocityMap(const VelocityCalibData& data);

    bool has_table() const { return has_table_; }
    double min_reliable_erpm() const { return data_.min_reliable_erpm; }

    double erpm_for_velocity(double v) const;
    double velocity_for_erpm(double erpm) const;

   private:
    VelocityCalibData data_;
    bool has_table_ = false;
};

// Outcome of loading + building a VelocityMap from a file path, mirroring
// DriverCore.h's build_motor_map()/MotorMapBuildResult convention: `map`
// is ALWAYS a usable map (linear-fallback defaults on any failure --
// empty path, unreadable file, or a parse failure never a hard error),
// `used_table` distinguishes which, `note` explains what happened.
struct VelocityMapLoadResult {
    VelocityMap map;
    bool used_table = false;
    std::string note;
};
VelocityMapLoadResult load_velocity_map(const std::string& path);

}  // namespace vesc

#endif  // VESC_DRIVER_VELOCITY_MAP_H_
