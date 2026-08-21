#include "SteeringAngleMap.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

// Vendored copy (see VescDriver's "no includes outside VescDriver/" rule).
#include "../third_party/nlohmann/json.hpp"

namespace vesc {

namespace {

// Linear interpolation over a strictly-ascending `xs`, clamped to the
// table's own ends -- see VelocityMap.cpp's own copy of this same
// pattern (independently duplicated, not shared, per this folder's
// small-helper-duplication convention -- see DriverCore.cpp's
// rate_limited_toward() comment).
double interpolate_monotone(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        if (x >= xs[i] && x <= xs[i + 1]) {
            const double span = xs[i + 1] - xs[i];
            const double frac = (span > 1e-12) ? (x - xs[i]) / span : 0.0;
            return ys[i] + (ys[i + 1] - ys[i]) * frac;
        }
    }
    return ys.back();
}

template <typename T>
void assign_if_present(const nlohmann::json& j, const char* key, T* out) {
    if (!j.contains(key)) return;
    try {
        *out = j.at(key).get<T>();
    } catch (...) {
    }
}

}  // namespace

double SteeringAngleMap::servo_for_delta(double delta) const {
    if (empty()) return 0.5;
    double clamped = delta;
    if (clamped < data_.delta_min) clamped = data_.delta_min;
    if (clamped > data_.delta_max) clamped = data_.delta_max;

    std::vector<double> deltas, servos;
    deltas.reserve(data_.points.size());
    servos.reserve(data_.points.size());
    for (const auto& p : data_.points) {
        deltas.push_back(p.delta);
        servos.push_back(p.servo);
    }
    return interpolate_monotone(deltas, servos, clamped);
}

SteeringAngleMapParseResult parse_steering_angle_map_json(const std::string& json_text) {
    SteeringAngleMapParseResult out;

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(json_text);
    } catch (...) {
        out.error = "invalid JSON";
        return out;
    }
    if (!j.is_object()) {
        out.error = "top level is not a JSON object";
        return out;
    }

    try {
        SteeringAngleMapData data;
        assign_if_present(j, "version", &data.version);
        assign_if_present(j, "robot_name", &data.robot_name);
        assign_if_present(j, "wheel_base", &data.wheel_base);
        assign_if_present(j, "delta_min", &data.delta_min);
        assign_if_present(j, "delta_max", &data.delta_max);
        assign_if_present(j, "residual_rms", &data.residual_rms);

        if (!j.contains("points") || !j.at("points").is_array()) {
            out.error = "missing or non-array 'points'";
            return out;
        }
        const nlohmann::json& pts = j.at("points");
        std::vector<SteeringAnglePoint> points;
        points.reserve(pts.size());
        for (const auto& entry : pts) {
            if (!entry.is_object() || !entry.contains("delta") || !entry.contains("servo") ||
                !entry.at("delta").is_number() || !entry.at("servo").is_number()) {
                out.error = "malformed 'points' entry (expected {\"delta\":..,\"servo\":..})";
                return out;
            }
            SteeringAnglePoint p;
            p.delta = entry.at("delta").get<double>();
            p.servo = entry.at("servo").get<double>();
            if (p.servo < 0.0 || p.servo > 1.0) {
                out.error = "'points' entry has servo outside [0,1]";
                return out;
            }
            points.push_back(p);
        }
        if (points.size() < 2) {
            out.error = "'points' must have at least 2 entries";
            return out;
        }
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].delta <= points[i - 1].delta) {
                out.error = "'points' is not strictly ascending in delta";
                return out;
            }
        }
        bool all_ascending = true;
        bool all_descending = true;
        for (size_t i = 1; i < points.size(); ++i) {
            if (points[i].servo <= points[i - 1].servo) all_ascending = false;
            if (points[i].servo >= points[i - 1].servo) all_descending = false;
        }
        if (!all_ascending && !all_descending) {
            out.error = "'points' servo column is not monotone (must be strictly ascending or strictly descending)";
            return out;
        }
        data.points = std::move(points);

        out.data = std::move(data);
        out.ok = true;
    } catch (const std::exception& e) {
        out.ok = false;
        out.error = std::string("exception while parsing steering angle map JSON: ") + e.what();
    } catch (...) {
        out.ok = false;
        out.error = "unknown exception while parsing steering angle map JSON";
    }
    return out;
}

SteeringAngleMapParseResult load_steering_angle_map_file(const std::string& path) {
    SteeringAngleMapParseResult out;
    std::ifstream f(path);
    if (!f.is_open()) {
        out.error = "could not open '" + path + "'";
        return out;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parse_steering_angle_map_json(ss.str());
}

SteeringAngleMapLoadResult load_steering_angle_map(const std::string& path) {
    SteeringAngleMapLoadResult out;
    if (path.empty()) {
        out.note = "no steering_angle_map configured -- using legacy servo path";
        return out;
    }
    SteeringAngleMapParseResult parsed = load_steering_angle_map_file(path);
    if (!parsed.ok) {
        out.note = "steering_angle_map '" + path + "' failed to load (" + parsed.error +
                    ") -- using legacy servo path";
        return out;
    }
    out.ok = true;
    out.map = SteeringAngleMap(parsed.data);
    out.note = "loaded steering_angle_map '" + path + "'";
    return out;
}

}  // namespace vesc
