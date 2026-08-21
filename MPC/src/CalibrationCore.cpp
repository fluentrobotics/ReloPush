#include "mpc/CalibrationCore.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

// NOTE on velocity bin range: the task spec describes the velocity bin
// ladder as "|v| in [0.05, 0.40] in 0.05 steps, both directions" with a
// "(14 bins)" parenthetical that undercounts by 2 -- 0.05..0.40 step 0.05
// is 8 point-center values/direction = 16 bins total, confirmed as the
// intended range (the "14" was the typo). VelocityCoverageConfig's default
// v_max=0.40 reflects this.

namespace mpc {

// =======================================================================
// to_string()
// =======================================================================

std::string to_string(RejectReason reason) {
    switch (reason) {
        case RejectReason::kNone:
            return "";
        case RejectReason::kCommandNotHeld:
            return "command_not_held";
        case RejectReason::kAccelerating:
            return "accelerating";
        case RejectReason::kPoseStale:
            return "pose_stale";
        case RejectReason::kTelemetryStale:
            return "telemetry_stale";
        case RejectReason::kInsufficientHistory:
            return "insufficient_history";
    }
    return "";
}

std::string to_string(Task task) { return task == Task::kVelocity ? "velocity" : "steering"; }

std::string to_string(BinState state) {
    switch (state) {
        case BinState::kEmpty:
            return "empty";
        case BinState::kPartial:
            return "partial";
        case BinState::kDone:
            return "done";
        case BinState::kNotApplicable:
            return "not_applicable";
    }
    return "empty";
}

namespace {
RejectReason reject_reason_from_string(const std::string& s) {
    if (s == "command_not_held") return RejectReason::kCommandNotHeld;
    if (s == "accelerating") return RejectReason::kAccelerating;
    if (s == "pose_stale") return RejectReason::kPoseStale;
    if (s == "telemetry_stale") return RejectReason::kTelemetryStale;
    if (s == "insufficient_history") return RejectReason::kInsufficientHistory;
    return RejectReason::kNone;
}
}  // namespace

// =======================================================================
// Session CSV read/write.
// =======================================================================

namespace {
constexpr const char* kSessionCsvHeader =
    "t,x,y,yaw,v,omega,erpm,duty_cmd,servo_cmd,v_in,current,accepted,reject_reason";

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',') {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r') {
            cur.push_back(c);
        }
    }
    out.push_back(cur);
    return out;
}
}  // namespace

bool write_derived_samples_csv(const std::string& path, const std::vector<DerivedSample>& rows,
                                bool append) {
    const bool file_exists = std::filesystem::exists(path);
    const std::ios::openmode mode = std::ios::out | (append ? std::ios::app : std::ios::trunc);
    std::ofstream out(path, mode);
    if (!out.good()) return false;

    const bool need_header = !append || !file_exists;
    if (need_header) {
        out << kSessionCsvHeader << "\n";
    }
    out << std::setprecision(10);
    for (const auto& r : rows) {
        out << r.t << ',' << r.x << ',' << r.y << ',' << r.yaw << ',' << r.v << ',' << r.omega << ','
            << r.erpm << ',' << r.duty_cmd << ',' << r.servo_cmd << ',' << r.v_in << ',' << r.current
            << ',' << (r.accepted ? 1 : 0) << ',' << to_string(r.reject_reason) << '\n';
    }
    return true;
}

bool load_derived_samples_csv(const std::string& path, std::vector<DerivedSample>* out,
                               std::string* error) {
    std::ifstream in(path);
    if (!in.good()) {
        if (error) *error = "mpc::load_derived_samples_csv: cannot open file: " + path;
        return false;
    }

    std::string header;
    if (!std::getline(in, header)) {
        if (error) *error = "mpc::load_derived_samples_csv: empty file: " + path;
        return false;
    }
    while (!header.empty() && (header.back() == '\r' || header.back() == '\n')) header.pop_back();
    if (header != kSessionCsvHeader) {
        if (error) {
            *error = "mpc::load_derived_samples_csv: unexpected header in " + path + ": '" + header + "'";
        }
        return false;
    }

    out->clear();
    std::string line;
    int line_no = 1;
    while (std::getline(in, line)) {
        ++line_no;
        if (line.empty()) continue;
        const auto fields = split_csv_line(line);
        if (fields.size() != 13) {
            if (error) {
                *error = "mpc::load_derived_samples_csv: " + path + ":" + std::to_string(line_no) +
                         " has " + std::to_string(fields.size()) + " fields, expected 13";
            }
            return false;
        }
        DerivedSample row;
        try {
            row.t = std::stod(fields[0]);
            row.x = std::stod(fields[1]);
            row.y = std::stod(fields[2]);
            row.yaw = std::stod(fields[3]);
            row.v = std::stod(fields[4]);
            row.omega = std::stod(fields[5]);
            row.erpm = std::stod(fields[6]);
            row.duty_cmd = std::stod(fields[7]);
            row.servo_cmd = std::stod(fields[8]);
            row.v_in = std::stod(fields[9]);
            row.current = std::stod(fields[10]);
            row.accepted = (fields[11] == "1");
            row.reject_reason = reject_reason_from_string(fields[12]);
        } catch (const std::exception& ex) {
            if (error) {
                *error = "mpc::load_derived_samples_csv: " + path + ":" + std::to_string(line_no) +
                         " parse error: " + ex.what();
            }
            return false;
        }
        out->push_back(row);
    }
    return true;
}

// =======================================================================
// CoverageReport::to_json()
// =======================================================================

nlohmann::json CoverageReport::to_json() const {
    nlohmann::json j;
    j["task"] = mpc::to_string(task);
    j["percent"] = percent;
    nlohmann::json bins_j = nlohmann::json::array();
    for (const auto& b : bins) {
        bins_j.push_back({
            {"center", b.center},
            {"count", b.count},
            {"target", b.target},
            {"state", mpc::to_string(b.state)},
        });
    }
    j["bins"] = bins_j;
    j["instruction"] = {{"text", instruction.text}, {"target_bin", instruction.target_bin}};
    return j;
}

// =======================================================================
// Small numeric helpers (least squares, median, stddev, PAVA).
// =======================================================================

namespace {

double least_squares_slope(const std::vector<double>& xs, const std::vector<double>& ys) {
    const double n = static_cast<double>(xs.size());
    double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        sum_x += xs[i];
        sum_y += ys[i];
        sum_xx += xs[i] * xs[i];
        sum_xy += xs[i] * ys[i];
    }
    const double denom = n * sum_xx - sum_x * sum_x;
    if (std::fabs(denom) < 1e-12) return 0.0;
    return (n * sum_xy - sum_x * sum_y) / denom;
}

double median_of(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    if (n % 2 == 1) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double stddev_of(const std::vector<double>& v) {
    if (v.size() < 2) return 0.0;
    double mean = 0.0;
    for (double x : v) mean += x;
    mean /= static_cast<double>(v.size());
    double ss = 0.0;
    for (double x : v) ss += (x - mean) * (x - mean);
    return std::sqrt(ss / static_cast<double>(v.size() - 1));
}

// One pooled point produced by PAVA (pool-adjacent-violators): the
// weighted mean of every original (x,y) pair merged into this block, plus
// the pooled weight (== total sample count across the merged bins).
struct PavaPoint {
    double x_sum = 0.0, y_sum = 0.0;
    double x_mean = 0.0, y_mean = 0.0;
    int weight = 0;
};

// Enforces y NON-DECREASING (with ties also merged, so the RESULT is
// strictly increasing) as a function of x, `xs` assumed already sorted
// ascending and distinct. Standard stack-based pool-adjacent-violators:
// push each point, then repeatedly merge (weighted-average) the top two
// blocks while the new top violates monotonicity relative to the block
// below it.
std::vector<PavaPoint> pava_pool_nondecreasing(const std::vector<double>& xs, const std::vector<double>& ys,
                                                const std::vector<int>& weights) {
    std::vector<PavaPoint> stack;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        PavaPoint p;
        p.weight = weights[i];
        p.x_sum = xs[i] * weights[i];
        p.y_sum = ys[i] * weights[i];
        p.x_mean = xs[i];
        p.y_mean = ys[i];
        stack.push_back(p);
        while (stack.size() >= 2) {
            PavaPoint& top = stack[stack.size() - 1];
            PavaPoint& prev = stack[stack.size() - 2];
            if (prev.y_mean < top.y_mean) break;  // strictly increasing already -- no violation
            PavaPoint merged;
            merged.weight = prev.weight + top.weight;
            merged.x_sum = prev.x_sum + top.x_sum;
            merged.y_sum = prev.y_sum + top.y_sum;
            merged.x_mean = merged.x_sum / merged.weight;
            merged.y_mean = merged.y_sum / merged.weight;
            stack.pop_back();
            stack.pop_back();
            stack.push_back(merged);
        }
    }
    return stack;
}

// Direction-aware wrapper: `increasing`=false enforces y non-increasing
// (strictly decreasing after tie-merging) instead, by negating y before/
// after pooling.
std::vector<PavaPoint> pava_pool_monotone(const std::vector<double>& xs, const std::vector<double>& ys,
                                           const std::vector<int>& weights, bool increasing) {
    if (increasing) return pava_pool_nondecreasing(xs, ys, weights);
    std::vector<double> neg_ys;
    neg_ys.reserve(ys.size());
    for (double y : ys) neg_ys.push_back(-y);
    auto pooled = pava_pool_nondecreasing(xs, neg_ys, weights);
    for (auto& p : pooled) {
        p.y_mean = -p.y_mean;
        p.y_sum = -p.y_sum;
    }
    return pooled;
}

// Piecewise-linear evaluation of a monotone table already sorted ascending
// by `.servo`; clamps outside the table's own servo range to the nearest
// endpoint's delta.
double interpolate_delta_at_servo(const std::vector<SteeringTablePoint>& pts_by_servo, double servo_query) {
    if (pts_by_servo.empty()) return 0.0;
    if (pts_by_servo.size() == 1 || servo_query <= pts_by_servo.front().servo) return pts_by_servo.front().delta;
    if (servo_query >= pts_by_servo.back().servo) return pts_by_servo.back().delta;
    for (std::size_t i = 0; i + 1 < pts_by_servo.size(); ++i) {
        const auto& a = pts_by_servo[i];
        const auto& b = pts_by_servo[i + 1];
        if (servo_query >= a.servo && servo_query <= b.servo) {
            const double frac = (b.servo > a.servo) ? (servo_query - a.servo) / (b.servo - a.servo) : 0.0;
            return a.delta + frac * (b.delta - a.delta);
        }
    }
    return pts_by_servo.back().delta;
}

}  // namespace

// =======================================================================
// infer_notches()
// =======================================================================

std::vector<double> infer_notches(const std::vector<double>& servo_values, double tolerance) {
    std::vector<double> sorted = servo_values;
    std::sort(sorted.begin(), sorted.end());
    std::vector<double> notches;
    std::vector<double> group;
    for (double v : sorted) {
        if (group.empty() || (v - group.back()) <= tolerance) {
            group.push_back(v);
        } else {
            double sum = 0.0;
            for (double g : group) sum += g;
            notches.push_back(sum / static_cast<double>(group.size()));
            group.clear();
            group.push_back(v);
        }
    }
    if (!group.empty()) {
        double sum = 0.0;
        for (double g : group) sum += g;
        notches.push_back(sum / static_cast<double>(group.size()));
    }
    return notches;
}

// =======================================================================
// fit_velocity_map()
// =======================================================================

VelocityFitResult fit_velocity_map(const std::vector<double>& bin_centers,
                                    const std::vector<std::vector<DerivedSample>>& bin_samples) {
    VelocityFitResult out;
    out.bins_total = static_cast<int>(bin_centers.size());

    std::vector<double> xs, ys;   // per-bin median v, median erpm
    std::vector<int> weights;     // per-bin sample count
    std::vector<double> spreads;  // per-bin erpm stddev, or -1 if <2 samples (can't assess)
    int n_samples_total = 0;
    int bins_filled = 0;

    const std::size_t n_bins = std::min(bin_centers.size(), bin_samples.size());
    for (std::size_t i = 0; i < n_bins; ++i) {
        const auto& samples = bin_samples[i];
        if (samples.empty()) continue;
        ++bins_filled;
        std::vector<double> vs, erpms;
        vs.reserve(samples.size());
        erpms.reserve(samples.size());
        for (const auto& s : samples) {
            vs.push_back(s.v);
            erpms.push_back(s.erpm);
        }
        n_samples_total += static_cast<int>(samples.size());
        xs.push_back(median_of(vs));
        ys.push_back(median_of(erpms));
        weights.push_back(static_cast<int>(samples.size()));
        spreads.push_back(samples.size() >= 2 ? stddev_of(erpms) : -1.0);
    }
    out.n_samples = n_samples_total;
    out.bins_filled = bins_filled;

    if (xs.size() < 2) {
        out.ok = false;
        out.error = "fit_velocity_map: need >=2 non-empty velocity bins, have " + std::to_string(xs.size());
        return out;
    }

    // xs/ys are already in ascending-v order (bin_centers is constructed
    // ascending and bin_samples is index-aligned with it) -- PAVA pools
    // adjacent bins whose median erpm doesn't strictly increase with v.
    const auto pooled = pava_pool_nondecreasing(xs, ys, weights);
    out.table.clear();
    out.table.reserve(pooled.size());
    for (const auto& p : pooled) out.table.push_back({p.x_mean, p.y_mean});

    // min_reliable_erpm: scan bins in order of increasing |v| (magnitude,
    // not signed v -- both directions approach stall from |v|=v_min
    // upward); the first bin with >=2 samples whose erpm spread clears a
    // scale-appropriate threshold (an absolute floor OR'd with a relative
    // fraction of that bin's own median erpm, so the check is meaningful
    // both near-zero and at highway speeds) marks reliability onset.
    std::vector<std::size_t> order(xs.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](std::size_t a, std::size_t b) { return std::fabs(xs[a]) < std::fabs(xs[b]); });

    bool found_reliable = false;
    for (std::size_t idx : order) {
        if (spreads[idx] < 0.0) continue;  // <2 samples -- can't assess consistency
        const double threshold = std::max(50.0, 0.15 * std::fabs(ys[idx]));
        if (spreads[idx] < threshold) {
            out.min_reliable_erpm = std::fabs(ys[idx]);
            found_reliable = true;
            break;
        }
    }
    if (!found_reliable) {
        // Fallback: lowest-|v| bin with any data at all, so the field is
        // still populated sensibly rather than left at 0.
        out.min_reliable_erpm = std::fabs(ys[order.front()]);
    }

    // Linear fallback/diagnostic: least squares over ALL accepted,
    // velocity-applicable samples directly (not bin medians), matching
    // MotorCalibCore::fit_linear's style.
    std::vector<double> all_v, all_erpm;
    for (const auto& bin : bin_samples) {
        for (const auto& s : bin) {
            all_v.push_back(s.v);
            all_erpm.push_back(s.erpm);
        }
    }
    if (all_v.size() >= 2) {
        const double n = static_cast<double>(all_v.size());
        double sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
        for (std::size_t i = 0; i < all_v.size(); ++i) {
            sum_x += all_v[i];
            sum_y += all_erpm[i];
            sum_xx += all_v[i] * all_v[i];
            sum_xy += all_v[i] * all_erpm[i];
        }
        const double denom = n * sum_xx - sum_x * sum_x;
        if (std::fabs(denom) > 1e-9) {
            out.erpm_per_mps = (n * sum_xy - sum_x * sum_y) / denom;
            out.offset_erpm = (sum_y - out.erpm_per_mps * sum_x) / n;
            double sse = 0.0;
            for (std::size_t i = 0; i < all_v.size(); ++i) {
                const double pred = out.offset_erpm + out.erpm_per_mps * all_v[i];
                const double err = pred - all_erpm[i];
                sse += err * err;
            }
            out.rms = std::sqrt(sse / n);
        }
    }

    out.ok = out.table.size() >= 2;
    if (!out.ok) out.error = "fit_velocity_map: PAVA pooling collapsed to <2 table points";
    return out;
}

// =======================================================================
// fit_steering_map()
// =======================================================================

SteeringFitResult fit_steering_map(const std::vector<double>& notches,
                                    const std::vector<std::vector<DerivedSample>>& notch_samples,
                                    double wheel_base) {
    SteeringFitResult out;

    std::vector<double> xs, ys;  // per-notch servo, median delta_eff
    std::vector<int> weights;
    int n_samples_total = 0;

    const std::size_t n_notches = std::min(notches.size(), notch_samples.size());
    for (std::size_t i = 0; i < n_notches; ++i) {
        const auto& samples = notch_samples[i];
        if (samples.empty()) continue;
        std::vector<double> deltas;
        deltas.reserve(samples.size());
        for (const auto& s : samples) {
            if (std::fabs(s.v) < 1e-6) continue;  // defensive -- coverage gate already requires |v|>min_speed_mps
            deltas.push_back(std::atan(wheel_base * s.omega / s.v));
        }
        if (deltas.empty()) continue;
        n_samples_total += static_cast<int>(deltas.size());
        xs.push_back(notches[i]);
        ys.push_back(median_of(deltas));
        weights.push_back(static_cast<int>(deltas.size()));
    }
    out.n_samples = n_samples_total;

    if (xs.size() < 2) {
        out.ok = false;
        out.error = "fit_steering_map: need >=2 non-empty steering notches, have " + std::to_string(xs.size());
        return out;
    }

    // xs (servo) is already ascending (notches is sorted at construction);
    // determine whether delta trends up or down with servo before pooling,
    // since PAVA enforces monotonicity in ONE direction only.
    const double slope = least_squares_slope(xs, ys);
    const bool increasing = slope >= 0.0;
    const auto pooled = pava_pool_monotone(xs, ys, weights, increasing);

    out.points.clear();
    out.points.reserve(pooled.size());
    for (const auto& p : pooled) out.points.push_back({p.y_mean, p.x_mean});  // {delta, servo}
    std::sort(out.points.begin(), out.points.end(),
              [](const SteeringTablePoint& a, const SteeringTablePoint& b) { return a.delta < b.delta; });

    if (!out.points.empty()) {
        out.delta_min = out.points.front().delta;
        out.delta_max = out.points.back().delta;
    }

    // residual_rms: compare each individual sample's own raw delta_eff
    // against the fitted table evaluated (piecewise-linear, servo->delta)
    // at that sample's own servo_cmd.
    std::vector<SteeringTablePoint> by_servo = out.points;
    std::sort(by_servo.begin(), by_servo.end(),
              [](const SteeringTablePoint& a, const SteeringTablePoint& b) { return a.servo < b.servo; });
    double sse = 0.0;
    int cnt = 0;
    for (std::size_t i = 0; i < n_notches; ++i) {
        for (const auto& s : notch_samples[i]) {
            if (std::fabs(s.v) < 1e-6) continue;
            const double raw_delta = std::atan(wheel_base * s.omega / s.v);
            const double pred = interpolate_delta_at_servo(by_servo, s.servo_cmd);
            const double err = pred - raw_delta;
            sse += err * err;
            ++cnt;
        }
    }
    out.residual_rms = cnt > 0 ? std::sqrt(sse / cnt) : 0.0;

    out.ok = out.points.size() >= 2;
    if (!out.ok) out.error = "fit_steering_map: PAVA pooling collapsed to <2 table points";
    return out;
}

// =======================================================================
// Export -- FROZEN OUTPUT SCHEMAS.
// =======================================================================

nlohmann::json velocity_calib_to_json(const VelocityFitResult& fit, const std::string& robot_name,
                                       const std::string& note) {
    nlohmann::json j;
    j["version"] = 2;
    j["kind"] = "velocity_calib";
    j["robot_name"] = robot_name;
    nlohmann::json table = nlohmann::json::array();
    for (const auto& p : fit.table) table.push_back({{"v", p.v}, {"erpm", p.erpm}});
    j["table"] = table;
    j["min_reliable_erpm"] = fit.min_reliable_erpm;
    j["linear_fallback"] = {
        {"erpm_per_mps", fit.erpm_per_mps},
        {"offset_erpm", fit.offset_erpm},
        {"rms", fit.rms},
    };
    j["meta"] = {
        {"n_samples", fit.n_samples},
        {"bins_filled", fit.bins_filled},
        {"bins_total", fit.bins_total},
        {"created_unix_time", static_cast<long long>(std::time(nullptr))},
        {"note", note},
    };
    return j;
}

namespace {
bool atomic_write_json(const nlohmann::json& j, const std::string& path, const char* fn_name,
                        std::string* error) {
    const std::string tmp_path = path + ".tmp";
    {
        std::ofstream out(tmp_path, std::ios::out | std::ios::trunc);
        if (!out.good()) {
            if (error) *error = std::string(fn_name) + ": cannot open " + tmp_path + " for writing";
            return false;
        }
        out << j.dump(2);
    }
    std::error_code ec;
    std::filesystem::rename(tmp_path, path, ec);
    if (ec) {
        if (error) *error = std::string(fn_name) + ": rename " + tmp_path + " -> " + path + " failed: " + ec.message();
        return false;
    }
    return true;
}
}  // namespace

bool export_velocity_calib(const VelocityFitResult& fit, const std::string& robot_name, const std::string& path,
                            std::string* error, const std::string& note) {
    if (fit.table.size() < 2) {
        if (error) *error = "mpc::export_velocity_calib: fit has fewer than 2 table points";
        return false;
    }
    return atomic_write_json(velocity_calib_to_json(fit, robot_name, note), path,
                              "mpc::export_velocity_calib", error);
}

nlohmann::json steering_map_to_json(const SteeringFitResult& fit, const std::string& robot_name,
                                     double wheel_base, const std::string& note) {
    nlohmann::json j;
    j["version"] = 1;
    j["kind"] = "steering_angle_map";
    j["robot_name"] = robot_name;
    j["wheel_base"] = wheel_base;
    nlohmann::json points = nlohmann::json::array();
    for (const auto& p : fit.points) points.push_back({{"delta", p.delta}, {"servo", p.servo}});
    j["points"] = points;
    j["delta_min"] = fit.delta_min;
    j["delta_max"] = fit.delta_max;
    j["residual_rms"] = fit.residual_rms;
    j["meta"] = {
        {"n_samples", fit.n_samples},
        {"created_unix_time", static_cast<long long>(std::time(nullptr))},
        {"note", note},
    };
    return j;
}

bool export_steering_map(const SteeringFitResult& fit, const std::string& robot_name, double wheel_base,
                          const std::string& path, std::string* error, const std::string& note) {
    if (fit.points.size() < 2) {
        if (error) *error = "mpc::export_steering_map: fit has fewer than 2 table points";
        return false;
    }
    return atomic_write_json(steering_map_to_json(fit, robot_name, wheel_base, note), path,
                              "mpc::export_steering_map", error);
}

// =======================================================================
// CalibrationCore
// =======================================================================

namespace {
std::vector<double> compute_velocity_bin_centers(const VelocityCoverageConfig& vc) {
    std::vector<double> positive;
    for (double v = vc.v_min; v <= vc.v_max + 1e-9; v += vc.v_step) positive.push_back(v);
    std::vector<double> centers;
    centers.reserve(positive.size() * 2);
    for (auto it = positive.rbegin(); it != positive.rend(); ++it) centers.push_back(-*it);
    for (double v : positive) centers.push_back(v);
    return centers;
}
}  // namespace

CalibrationCore::CalibrationCore(CalibrationConfig cfg) : cfg_(std::move(cfg)) {
    std::sort(cfg_.steering.notches.begin(), cfg_.steering.notches.end());
    velocity_bin_centers_ = compute_velocity_bin_centers(cfg_.velocity);
    velocity_bin_samples_.assign(velocity_bin_centers_.size(), {});
    steering_notch_samples_.assign(cfg_.steering.notches.size(), {});
}

bool CalibrationCore::set_steering_notches(const std::vector<double>& notches) {
    if (notches_locked_) return false;
    cfg_.steering.notches = notches;
    std::sort(cfg_.steering.notches.begin(), cfg_.steering.notches.end());
    steering_notch_samples_.assign(cfg_.steering.notches.size(), {});
    return true;
}

void CalibrationCore::set_session_csv_path(const std::string& path, bool append) {
    session_csv_path_ = path;
    if (path.empty()) return;
    if (!append || !std::filesystem::exists(path)) {
        write_derived_samples_csv(path, {}, /*append=*/false);
    }
}

std::vector<double> CalibrationCore::velocity_bin_centers() const { return velocity_bin_centers_; }

void CalibrationCore::feed_pose(double t, double x, double y, double yaw) {
    // BURST-DEBOUNCE: drop pose samples arriving <10ms after the previous
    // one -- mirrors CalibClient::LiveChannels::kMinPoseFeedGapS (see that
    // class's own doc comment for the receipt-jitter rationale). This is a
    // pure ingestion API with no socket receipt-timestamping of its own,
    // so the caller-supplied `t` itself plays the role CalibClient's
    // separately self-timestamped arrival_t plays there.
    constexpr double kMinPoseFeedGapS = 0.01;
    if (pose_feed_count_ > 0 && (t - last_pose_feed_t_) < kMinPoseFeedGapS) return;
    last_pose_feed_t_ = t;
    ++pose_feed_count_;
    notches_locked_ = true;

    // -- Raw heading-projected finite-difference velocity, identical
    // formula to MotorCalibCore::VelocityEstimator::feed(). --
    if (have_prev_pose_) {
        const double dt = t - prev_pose_t_;
        if (dt > 1e-3) {
            const double dx = x - prev_pose_x_;
            const double dy = y - prev_pose_y_;
            last_v_raw_ = (dx * std::cos(yaw) + dy * std::sin(yaw)) / dt;
            have_v_raw_ = true;
        }
        // else: too-close-together dt -- retain the previous v_raw_
        // (matches VelocityEstimator's own retention behavior).
    }
    prev_pose_t_ = t;
    prev_pose_x_ = x;
    prev_pose_y_ = y;
    have_prev_pose_ = true;

    // -- Light smoothing: single-pole low-pass (EMA). Deliberately lighter
    // (lower latency, no fixed sample-count warm-up) than
    // MotorCalibCore::VelocityEstimator's heavier centered 5-sample
    // average -- free teleop driving needs the steadiness gate to react
    // quickly once the human eases off the throttle, rather than lagging a
    // couple of samples behind. --
    constexpr double kVelSmoothAlpha = 0.35;
    if (have_v_raw_) {
        if (!have_v_smooth_) {
            v_smooth_ = last_v_raw_;
            have_v_smooth_ = true;
        } else {
            v_smooth_ = kVelSmoothAlpha * last_v_raw_ + (1.0 - kVelSmoothAlpha) * v_smooth_;
        }
    }

    double dv_dt = 0.0;
    bool have_dv_dt = false;
    if (have_v_smooth_ && have_prev_v_smooth_) {
        const double dt = t - prev_v_smooth_t_;
        if (dt > 1e-3) {
            dv_dt = (v_smooth_ - prev_v_smooth_) / dt;
            have_dv_dt = true;
        }
    }
    if (have_v_smooth_) {
        prev_v_smooth_ = v_smooth_;
        prev_v_smooth_t_ = t;
        have_prev_v_smooth_ = true;
    }

    // -- Yaw rate: least-squares slope of UNWRAPPED yaw over a sliding
    // ~0.3s window. --
    constexpr double kYawWindowS = 0.3;
    double yaw_unwrapped = yaw;
    if (have_yaw_) {
        double d = yaw - last_yaw_raw_;
        while (d > M_PI) d -= 2.0 * M_PI;
        while (d < -M_PI) d += 2.0 * M_PI;
        yaw_unwrapped = last_yaw_unwrapped_ + d;
    }
    last_yaw_raw_ = yaw;
    last_yaw_unwrapped_ = yaw_unwrapped;
    have_yaw_ = true;
    yaw_window_.push_back({t, yaw_unwrapped});
    while (yaw_window_.size() > 1 && (t - yaw_window_.front().t) > kYawWindowS) {
        yaw_window_.erase(yaw_window_.begin());
    }

    double omega = 0.0;
    bool have_omega = false;
    if (yaw_window_.size() >= 3 && (yaw_window_.back().t - yaw_window_.front().t) >= 0.5 * kYawWindowS) {
        std::vector<double> ts, ys;
        ts.reserve(yaw_window_.size());
        ys.reserve(yaw_window_.size());
        for (const auto& p : yaw_window_) {
            ts.push_back(p.t);
            ys.push_back(p.yaw_unwrapped);
        }
        omega = least_squares_slope(ts, ys);
        have_omega = true;
    }

    // -- Telemetry alignment: nearest-in-time within a 50ms window. In a
    // live, causally-ordered stream there is no future telemetry sample to
    // look ahead to when a pose arrives, so "nearest neighbor" reduces
    // exactly to "the most recently received telemetry sample" -- no
    // separate lookahead buffer is needed. --
    constexpr double kTelemetryAlignS = 0.05;
    const double telemetry_age = have_telemetry_ ? (t - last_telemetry_.t) : 1e18;
    (void)kTelemetryAlignS;  // documents the alignment window; the hard accept/reject gate below
                              // uses SteadinessConfig::max_telemetry_age_s (0.2s) -- see that
                              // field's doc comment for why the two aren't the same threshold.

    // -- Command hold duration. --
    const double command_held_s = have_command_ ? (t - command_held_since_) : 0.0;

    DerivedSample row;
    row.t = t;
    row.x = x;
    row.y = y;
    row.yaw = yaw;
    row.v = have_v_smooth_ ? v_smooth_ : last_v_raw_;
    row.omega = omega;
    row.erpm = have_telemetry_ ? last_telemetry_.erpm : 0.0;
    row.v_in = have_telemetry_ ? last_telemetry_.v_in : 0.0;
    row.current = have_telemetry_ ? last_telemetry_.current_motor : 0.0;
    row.duty_cmd = have_command_ ? current_command_.value : 0.0;
    row.servo_cmd = have_command_ ? current_command_.servo : 0.0;

    // -- Steadiness acceptance gate (see SteadinessConfig's doc comment). --
    const auto& sc = cfg_.steadiness;
    RejectReason reason = RejectReason::kNone;
    constexpr double kPoseAgeS = 0.0;  // always 0 -- derivation is synchronous with the pose just
                                        // fed (see RejectReason::kPoseStale's own doc comment).
    if (!have_v_smooth_ || !have_prev_v_smooth_ || !have_dv_dt || !have_omega) {
        reason = RejectReason::kInsufficientHistory;
    } else if (!have_command_ || command_held_s < sc.command_hold_s) {
        reason = RejectReason::kCommandNotHeld;
    } else if (std::fabs(dv_dt) >= sc.max_dv_dt_mps2) {
        reason = RejectReason::kAccelerating;
    } else if (kPoseAgeS >= sc.max_pose_age_s) {
        reason = RejectReason::kPoseStale;
    } else if (telemetry_age >= sc.max_telemetry_age_s) {
        reason = RejectReason::kTelemetryStale;
    }
    row.accepted = (reason == RejectReason::kNone);
    row.reject_reason = reason;

    ingest_one(row, /*also_log_csv=*/true);
}

void CalibrationCore::feed_telemetry(double t, double erpm, double duty, double current_motor, double v_in) {
    last_telemetry_ = {t, erpm, duty, current_motor, v_in};
    have_telemetry_ = true;
}

void CalibrationCore::feed_command(double t, const std::string& mode, double value, double servo) {
    const auto& sc = cfg_.steadiness;
    const bool is_new_command = !have_command_ || mode != current_command_.mode ||
                                 std::fabs(value - current_command_.value) > sc.value_eps ||
                                 std::fabs(servo - current_command_.servo) > sc.servo_eps;
    if (is_new_command) {
        command_held_since_ = t;
    }
    current_command_ = {t, mode, value, servo};
    have_command_ = true;
}

void CalibrationCore::ingest_one(const DerivedSample& row, bool also_log_csv) {
    ++total_samples_;
    if (row.accepted) {
        ++accepted_samples_;
    } else {
        ++rejection_counts_[row.reject_reason];
    }

    if (also_log_csv && !session_csv_path_.empty()) {
        write_derived_samples_csv(session_csv_path_, {row}, /*append=*/true);
    }

    if (!row.accepted) return;

    // -- Velocity task bin membership. --
    const auto& vc = cfg_.velocity;
    if (std::fabs(row.v) >= vc.v_min && !velocity_bin_centers_.empty()) {
        std::size_t best = 0;
        double best_d = std::fabs(velocity_bin_centers_[0] - row.v);
        for (std::size_t i = 1; i < velocity_bin_centers_.size(); ++i) {
            const double d = std::fabs(velocity_bin_centers_[i] - row.v);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        if (best_d <= 0.5 * vc.v_step + 1e-9) {
            velocity_bin_samples_[best].push_back(row);
        }
    }

    // -- Steering task notch membership. --
    const auto& stc = cfg_.steering;
    if (!stc.notches.empty() && std::fabs(row.v) > stc.min_speed_mps) {
        const double spacing = stc.notches.size() >= 2
                                    ? (stc.notches.back() - stc.notches.front()) /
                                          static_cast<double>(stc.notches.size() - 1)
                                    : 0.05;
        std::size_t best = 0;
        double best_d = std::fabs(stc.notches[0] - row.servo_cmd);
        for (std::size_t i = 1; i < stc.notches.size(); ++i) {
            const double d = std::fabs(stc.notches[i] - row.servo_cmd);
            if (d < best_d) {
                best_d = d;
                best = i;
            }
        }
        if (best_d <= 0.5 * spacing + 1e-9) {
            steering_notch_samples_[best].push_back(row);
        }
    }
}

bool CalibrationCore::load_session_csv(const std::string& path, std::string* error) {
    std::vector<DerivedSample> rows;
    std::string err;
    if (!load_derived_samples_csv(path, &rows, &err)) {
        if (error) *error = err;
        return false;
    }
    ingest_derived_samples(rows);
    return true;
}

void CalibrationCore::ingest_derived_samples(const std::vector<DerivedSample>& rows) {
    notches_locked_ = true;
    for (const auto& row : rows) ingest_one(row, /*also_log_csv=*/false);
}

CoverageReport CalibrationCore::velocity_coverage() const {
    CoverageReport rep;
    rep.task = Task::kVelocity;
    const auto& vc = cfg_.velocity;
    int total_target = 0, total_count = 0;
    for (std::size_t i = 0; i < velocity_bin_centers_.size(); ++i) {
        CoverageBin b;
        b.center = velocity_bin_centers_[i];
        b.count = static_cast<int>(i < velocity_bin_samples_.size() ? velocity_bin_samples_[i].size() : 0);
        b.target = vc.target_per_bin;
        b.state = b.target <= 0 ? BinState::kNotApplicable
                                 : (b.count == 0 ? BinState::kEmpty
                                                  : (b.count >= b.target ? BinState::kDone : BinState::kPartial));
        if (b.target > 0) {
            total_target += b.target;
            total_count += std::min(b.count, b.target);
        }
        rep.bins.push_back(b);
    }
    rep.percent = total_target > 0 ? (100.0 * total_count / total_target) : 0.0;
    rep.instruction = next_instruction(Task::kVelocity);
    return rep;
}

CoverageReport CalibrationCore::steering_coverage() const {
    CoverageReport rep;
    rep.task = Task::kSteering;
    const auto& stc = cfg_.steering;
    int total_target = 0, total_count = 0;
    for (std::size_t i = 0; i < stc.notches.size(); ++i) {
        CoverageBin b;
        b.center = stc.notches[i];
        b.count =
            static_cast<int>(i < steering_notch_samples_.size() ? steering_notch_samples_[i].size() : 0);
        b.target = stc.target_per_notch;
        b.state = b.target <= 0 ? BinState::kNotApplicable
                                 : (b.count == 0 ? BinState::kEmpty
                                                  : (b.count >= b.target ? BinState::kDone : BinState::kPartial));
        if (b.target > 0) {
            total_target += b.target;
            total_count += std::min(b.count, b.target);
        }
        rep.bins.push_back(b);
    }
    rep.percent = total_target > 0 ? (100.0 * total_count / total_target) : 0.0;
    rep.instruction = next_instruction(Task::kSteering);
    return rep;
}

NextInstruction CalibrationCore::next_instruction(Task task) const {
    NextInstruction out;
    if (task == Task::kVelocity) {
        const auto& vc = cfg_.velocity;
        int best_idx = -1, best_count = 0;
        for (std::size_t i = 0; i < velocity_bin_centers_.size(); ++i) {
            const int count =
                static_cast<int>(i < velocity_bin_samples_.size() ? velocity_bin_samples_[i].size() : 0);
            if (vc.target_per_bin <= 0 || count >= vc.target_per_bin) continue;
            if (best_idx < 0 || count < best_count) {
                best_idx = static_cast<int>(i);
                best_count = count;
            }
        }
        if (best_idx < 0) {
            out.text = "Velocity coverage complete.";
            out.target_bin = -1;
            return out;
        }
        const double center = velocity_bin_centers_[best_idx];
        std::ostringstream oss;
        oss << (center >= 0.0 ? "Drive forward" : "Drive backward") << " and hold ~" << std::fixed
            << std::setprecision(2) << std::fabs(center) << " m/s for 3 s (" << best_count << "/"
            << vc.target_per_bin << " samples)";
        out.text = oss.str();
        out.target_bin = best_idx;
        return out;
    }

    // Task::kSteering
    const auto& stc = cfg_.steering;
    int best_idx = -1, best_count = 0;
    for (std::size_t i = 0; i < stc.notches.size(); ++i) {
        const int count =
            static_cast<int>(i < steering_notch_samples_.size() ? steering_notch_samples_[i].size() : 0);
        if (stc.target_per_notch <= 0 || count >= stc.target_per_notch) continue;
        if (best_idx < 0 || count < best_count) {
            best_idx = static_cast<int>(i);
            best_count = count;
        }
    }
    if (best_idx < 0) {
        out.text = "Steering coverage complete.";
        out.target_bin = -1;
        return out;
    }
    // Notch label: 0 at the notch nearest the center servo value (0.5,
    // matching MotorCalibCore::TrialRunnerConfig::center_servo_value's own
    // default), negative below center, positive above.
    std::size_t center_idx = 0;
    double center_d = stc.notches.empty() ? 0.0 : std::fabs(stc.notches[0] - 0.5);
    for (std::size_t i = 1; i < stc.notches.size(); ++i) {
        const double d = std::fabs(stc.notches[i] - 0.5);
        if (d < center_d) {
            center_d = d;
            center_idx = i;
        }
    }
    const int label = static_cast<int>(best_idx) - static_cast<int>(center_idx);
    std::ostringstream oss;
    oss << "Steering notch " << label << " (servo " << std::fixed << std::setprecision(3)
        << stc.notches[best_idx] << "): drive a steady arc >= 2 s. Reverse driving counts. (" << best_count
        << "/" << stc.target_per_notch << " samples)";
    out.text = oss.str();
    out.target_bin = best_idx;
    return out;
}

VelocityFitResult CalibrationCore::fit_velocity_map() const {
    return mpc::fit_velocity_map(velocity_bin_centers_, velocity_bin_samples_);
}

SteeringFitResult CalibrationCore::fit_steering_map() const { return fit_steering_map(cfg_.wheel_base); }

SteeringFitResult CalibrationCore::fit_steering_map(double wheel_base) const {
    return mpc::fit_steering_map(cfg_.steering.notches, steering_notch_samples_, wheel_base);
}

}  // namespace mpc
