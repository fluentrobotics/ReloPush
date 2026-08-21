// SteeringAngleMap.h
//
// Driver v2: monotone piecewise-linear STEERING-ANGLE (delta, radians) ->
// SERVO POSITION (0..1) map, loaded from a per-robot
// "steering_angle_map.json" file (schema below). Replaces the legacy
// center+gain_per_rad+invert affine servo computation with an absolute,
// fitted curve (the map itself already encodes any physical inversion --
// see servo_for_delta()'s own doc comment) when one is available; when
// none is loaded, DriverCore keeps using the legacy affine path
// unchanged (see DriverCore.h). PURE logic + plain fstream/nlohmann::json
// file I/O, same "no sockets/serial" convention as DriverCore.h.
//
// Portability: C++14 only, no std::filesystem -- see
// VescDriver/CMakeLists.txt's HARD PORTABILITY RULES.

#ifndef VESC_DRIVER_STEERING_ANGLE_MAP_H_
#define VESC_DRIVER_STEERING_ANGLE_MAP_H_

#include <string>
#include <vector>

namespace vesc {

struct SteeringAnglePoint {
    double delta = 0.0;  // steering angle, radians.
    double servo = 0.5;  // servo position, 0..1.
};

// Parsed contents of one "steering_angle_map.json" file. Only the fields
// SteeringAngleMap actually consumes are kept (wheel_base/points/
// delta_min/delta_max) -- "version"/"robot_name"/"residual_rms"/"meta"
// are accepted (not rejected) but not otherwise used by this class
// itself (wheel_base IS separately consumed by vesc_driver_main.cpp's
// wheelbase-mismatch check -- see that file).
struct SteeringAngleMapData {
    int version = 1;
    std::string robot_name;
    double wheel_base = 0.29;
    // Strictly monotone in delta (ascending); servo may move in EITHER
    // direction across the table (it just must itself be monotone,
    // matching whichever physical direction this vehicle's servo turns)
    // -- see parse_steering_angle_map_json() for the exact validation.
    std::vector<SteeringAnglePoint> points;
    double delta_min = 0.0;
    double delta_max = 0.0;
    double residual_rms = 0.0;
};

struct SteeringAngleMapParseResult {
    bool ok = false;
    SteeringAngleMapData data;
    std::string error;
};

// Parses one steering-angle-map JSON document from an in-memory string
// (no file I/O -- see load_steering_angle_map_file() below). ok=false
// (data left default) on: invalid JSON, a top level that isn't an
// object, a missing/wrong-typed required "points" key, fewer than 2
// points, a point with servo outside [0,1], a points.delta column that
// is not strictly ascending, or a points.servo column that is not
// monotone (strictly ascending OR strictly descending throughout -- both
// physical servo directions are legal, a NON-monotone column is not).
// "wheel_base"/"delta_min"/"delta_max"/"residual_rms" fall back
// independently to their own struct defaults when missing/wrong-typed.
SteeringAngleMapParseResult parse_steering_angle_map_json(const std::string& json_text);

// Reads `path` and parses it via parse_steering_angle_map_json().
// ok=false (data left default) if the file cannot be opened for reading
// OR its contents fail to parse.
SteeringAngleMapParseResult load_steering_angle_map_file(const std::string& path);

// Monotone piecewise-linear delta->servo map.
class SteeringAngleMap {
   public:
    SteeringAngleMap() = default;
    explicit SteeringAngleMap(const SteeringAngleMapData& data) : data_(data) {}

    // Clamps `delta` to [delta_min, delta_max] FIRST, then linearly
    // interpolates the clamped value between the table's bracketing
    // points (never extrapolates beyond the table's own delta range
    // either way -- the delta_min/delta_max clamp and the table's own
    // domain are expected to agree, but this method clamps to
    // delta_min/delta_max specifically, per the task's own contract).
    // The returned value already reflects the map's own physical
    // direction/inversion -- callers should NOT apply servo.invert or
    // any other sign flip on top of this (see DriverCore.h's own comment
    // on this at its call site).
    double servo_for_delta(double delta) const;

    double wheel_base() const { return data_.wheel_base; }
    double delta_min() const { return data_.delta_min; }
    double delta_max() const { return data_.delta_max; }
    bool empty() const { return data_.points.size() < 2; }

   private:
    SteeringAngleMapData data_;
};

// Outcome of loading a SteeringAngleMap from a file path. Unlike
// VelocityMap (which always has a usable linear fallback), there is NO
// meaningful fallback for an absolute servo curve -- `ok=false` means the
// caller must keep using the legacy servo path (see DriverCore.h's own
// "actuation"/steering-output comment); `map` is only meaningful when
// `ok` is true.
struct SteeringAngleMapLoadResult {
    bool ok = false;
    SteeringAngleMap map;
    std::string note;
};
SteeringAngleMapLoadResult load_steering_angle_map(const std::string& path);

}  // namespace vesc

#endif  // VESC_DRIVER_STEERING_ANGLE_MAP_H_
