#include "VelocityMap.h"

#include <cmath>
#include <fstream>
#include <sstream>

// Vendored copy (see VescDriver's "no includes outside VescDriver/" rule).
#include "../third_party/nlohmann/json.hpp"

namespace vesc {

namespace {

// Linear interpolation over a strictly-ascending `xs`, clamped to the
// table's own ends (never extrapolates) -- independently written rather
// than shared with DriverCore.cpp's bracket_axis()/bilinear_lookup()
// (this is a 1D table, not a 2D grid, and VescDriver's own convention --
// see DriverCore.cpp's rate_limited_toward() comment -- is to duplicate
// small helpers rather than couple otherwise-independent modules).
// Precondition: xs.size()==ys.size()>=2, xs strictly ascending.
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
    return ys.back();  // unreachable given the range checks above; safe fallback.
}

template <typename T>
void assign_if_present(const nlohmann::json& j, const char* key, T* out) {
    if (!j.contains(key)) return;
    try {
        *out = j.at(key).get<T>();
    } catch (...) {
        // Wrong JSON type for this key -- silently retain whatever *out
        // already held (mirrors DriverCore.h's own load_driver_config()).
    }
}

}  // namespace

VelocityMap::VelocityMap() = default;

VelocityMap::VelocityMap(const VelocityCalibData& data) : data_(data), has_table_(data.table.size() >= 2) {}

double VelocityMap::erpm_for_velocity(double v) const {
    if (has_table_) {
        std::vector<double> vs, erpms;
        vs.reserve(data_.table.size());
        erpms.reserve(data_.table.size());
        for (const auto& p : data_.table) {
            vs.push_back(p.v);
            erpms.push_back(p.erpm);
        }
        return interpolate_monotone(vs, erpms, v);
    }
    return data_.linear_fallback.offset_erpm + data_.linear_fallback.erpm_per_mps * v;
}

double VelocityMap::velocity_for_erpm(double erpm) const {
    if (has_table_) {
        std::vector<double> vs, erpms;
        vs.reserve(data_.table.size());
        erpms.reserve(data_.table.size());
        for (const auto& p : data_.table) {
            vs.push_back(p.v);
            erpms.push_back(p.erpm);
        }
        return interpolate_monotone(erpms, vs, erpm);
    }
    if (std::fabs(data_.linear_fallback.erpm_per_mps) < 1e-9) return 0.0;
    return (erpm - data_.linear_fallback.offset_erpm) / data_.linear_fallback.erpm_per_mps;
}

VelocityCalibParseResult parse_velocity_calib_json(const std::string& json_text) {
    VelocityCalibParseResult out;

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
        VelocityCalibData data;
        assign_if_present(j, "version", &data.version);
        assign_if_present(j, "robot_name", &data.robot_name);
        assign_if_present(j, "min_reliable_erpm", &data.min_reliable_erpm);

        if (j.contains("linear_fallback") && j.at("linear_fallback").is_object()) {
            const nlohmann::json& lf = j.at("linear_fallback");
            assign_if_present(lf, "erpm_per_mps", &data.linear_fallback.erpm_per_mps);
            assign_if_present(lf, "offset_erpm", &data.linear_fallback.offset_erpm);
            assign_if_present(lf, "rms", &data.linear_fallback.rms);
        }

        if (j.contains("table")) {
            const nlohmann::json& t = j.at("table");
            if (!t.is_array()) {
                out.error = "'table' is present but not an array";
                return out;
            }
            std::vector<VelocityCalibPoint> points;
            points.reserve(t.size());
            for (const auto& entry : t) {
                if (!entry.is_object() || !entry.contains("v") || !entry.contains("erpm") ||
                    !entry.at("v").is_number() || !entry.at("erpm").is_number()) {
                    out.error = "malformed 'table' entry (expected {\"v\":..,\"erpm\":..})";
                    return out;
                }
                VelocityCalibPoint p;
                p.v = entry.at("v").get<double>();
                p.erpm = entry.at("erpm").get<double>();
                points.push_back(p);
            }
            if (points.size() < 2) {
                out.error = "'table' must have at least 2 points";
                return out;
            }
            for (size_t i = 1; i < points.size(); ++i) {
                if (points[i].v <= points[i - 1].v) {
                    out.error = "'table' is not strictly ascending in v";
                    return out;
                }
                if (points[i].erpm <= points[i - 1].erpm) {
                    out.error = "'table' is not strictly ascending in erpm";
                    return out;
                }
            }
            data.table = std::move(points);
        }

        out.data = std::move(data);
        out.ok = true;
    } catch (const std::exception& e) {
        out.ok = false;
        out.error = std::string("exception while parsing velocity calib JSON: ") + e.what();
    } catch (...) {
        out.ok = false;
        out.error = "unknown exception while parsing velocity calib JSON";
    }
    return out;
}

VelocityCalibParseResult load_velocity_calib_file(const std::string& path) {
    VelocityCalibParseResult out;
    std::ifstream f(path);
    if (!f.is_open()) {
        out.error = "could not open '" + path + "'";
        return out;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return parse_velocity_calib_json(ss.str());
}

VelocityMapLoadResult load_velocity_map(const std::string& path) {
    VelocityMapLoadResult out;

    if (path.empty()) {
        out.note = "no velocity_calib file configured -- using linear fallback";
        out.map = VelocityMap();
        out.used_table = false;
        return out;
    }

    VelocityCalibParseResult parsed = load_velocity_calib_file(path);
    if (!parsed.ok) {
        out.note = "velocity_calib '" + path + "' failed to load (" + parsed.error +
                    ") -- falling back to linear defaults";
        out.map = VelocityMap();
        out.used_table = false;
        return out;
    }

    out.map = VelocityMap(parsed.data);
    out.used_table = out.map.has_table();
    out.note = "loaded velocity_calib '" + path + "' (" + (out.used_table ? "table" : "linear-fallback") + ")";
    return out;
}

}  // namespace vesc
