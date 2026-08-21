#include "mpc/RobotSpec.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace mpc {

RobotSpec RobotSpec::defaults() { return RobotSpec{}; }

std::string RobotSpec::validate() const {
    std::string error;

    if (min_turning_radius_transit > 1e-9) {
        double implied_steer = std::atan(wheel_base / min_turning_radius_transit);
        if (implied_steer > max_steer) {
            std::ostringstream oss;
            oss << "RobotSpec::validate: min_turning_radius_transit=" << min_turning_radius_transit
                << " with wheel_base=" << wheel_base << " implies a steering angle of "
                << implied_steer << " rad, which exceeds max_steer=" << max_steer << " rad";
            error = oss.str();
        }
    }

    if (error.empty() && (min_sustain_speed < 0.0 || min_sustain_speed > min_moving_speed)) {
        std::ostringstream oss;
        oss << "RobotSpec::validate: min_sustain_speed=" << min_sustain_speed
            << " must satisfy 0 <= min_sustain_speed <= min_moving_speed (min_moving_speed="
            << min_moving_speed << ")";
        error = oss.str();
    }

    if (error.empty() && max_breakaway_accel < max_accel) {
        std::ostringstream oss;
        oss << "RobotSpec::validate: max_breakaway_accel=" << max_breakaway_accel
            << " must satisfy max_breakaway_accel >= max_accel (max_accel=" << max_accel << ")";
        error = oss.str();
    }

    // FINISH-AT-REST CORRECTION LAUNCH: only meaningful while the LAUNCH
    // GOVERNOR is actually active (min_moving_speed > 0) -- see
    // settle_position_tolerance's own doc comment for the ~2-3cm
    // correctable-displacement rationale behind the 0.2m upper bound.
    if (error.empty() && min_moving_speed > 0.0 &&
        (settle_position_tolerance <= 0.0 || settle_position_tolerance > 0.2)) {
        std::ostringstream oss;
        oss << "RobotSpec::validate: settle_position_tolerance=" << settle_position_tolerance
            << " must satisfy 0 < settle_position_tolerance <= 0.2 (m) when min_moving_speed > 0 "
               "(min_moving_speed="
            << min_moving_speed << ")";
        error = oss.str();
    }

    // Non-fatal and independent of the checks above (evaluated even if one
    // of them already produced a fatal `error`), so operators never lose
    // visibility into the pushing-margin hazard just because an unrelated
    // problem is also present -- see this method's doc comment.
    if (min_moving_speed >= speed_transfer) {
        std::cerr << "RobotSpec::validate: WARNING: min_moving_speed=" << min_moving_speed
                  << " >= speed_transfer=" << speed_transfer
                  << " -- robot may never break away from the deadband while pushing at "
                     "speed_transfer (pushing-margin hazard)"
                  << std::endl;
    }

    return error;
}

MpcParams MpcParams::defaults() { return MpcParams{}; }

namespace {

template <typename T>
void assign_if_present(const nlohmann::json& obj, const char* key, T& out) {
    if (obj.contains(key)) {
        out = obj.at(key).get<T>();
    }
}

const std::set<std::string>& robot_spec_keys() {
    static const std::set<std::string> keys = {
        "wheel_base",  "front_length",
        "rear_length", "width",
        "min_turning_radius_transit", "min_turning_radius_transfer",
        "speed_transit", "speed_transfer",
        "max_steer", "max_accel",
        "max_v_push", "max_v_nonpush",
        "min_moving_speed", "min_sustain_speed", "launch_margin",
        "max_breakaway_accel",
        "settle_position_tolerance", "settle_cooldown_s", "settle_max_attempts",
    };
    return keys;
}

const std::set<std::string>& mpc_param_keys() {
    static const std::set<std::string> keys = {
        "dt", "control_delay_steps", "loop_hz",
        "w_dist", "w_dist_dirchange", "w_yaw", "w_vel", "w_lat",
        "w_control", "w_delta_rate",
        "vel_scale", "vel_scale_back", "dir_change_scale",
    };
    return keys;
}

void warn_unknown_keys(const nlohmann::json& obj, const std::set<std::string>& known,
                        const std::string& section) {
    for (auto it = obj.begin(); it != obj.end(); ++it) {
        if (known.find(it.key()) == known.end()) {
            std::cerr << "[mpc::RobotSpec] warning: unknown key '" << it.key() << "' in '"
                      << section << "' section of robot spec JSON (ignored)" << std::endl;
        }
    }
}

} // namespace

void load_spec_from_json(const std::string& path, RobotSpec& robot_out, MpcParams& mpc_out) {
    std::ifstream in(path);
    if (!in.good()) {
        throw std::runtime_error("mpc::load_spec_from_json: cannot open file: " + path);
    }

    nlohmann::json root;
    try {
        in >> root;
    } catch (const std::exception& ex) {
        throw std::runtime_error("mpc::load_spec_from_json: invalid JSON in '" + path +
                                  "': " + ex.what());
    }

    if (!root.is_object()) {
        throw std::runtime_error("mpc::load_spec_from_json: top level of '" + path +
                                  "' must be a JSON object");
    }

    if (root.contains("robot")) {
        const auto& r = root.at("robot");
        if (!r.is_object()) {
            throw std::runtime_error("mpc::load_spec_from_json: 'robot' must be an object in " +
                                      path);
        }
        warn_unknown_keys(r, robot_spec_keys(), "robot");
        assign_if_present(r, "wheel_base", robot_out.wheel_base);
        assign_if_present(r, "front_length", robot_out.front_length);
        assign_if_present(r, "rear_length", robot_out.rear_length);
        assign_if_present(r, "width", robot_out.width);
        assign_if_present(r, "min_turning_radius_transit", robot_out.min_turning_radius_transit);
        assign_if_present(r, "min_turning_radius_transfer", robot_out.min_turning_radius_transfer);
        assign_if_present(r, "speed_transit", robot_out.speed_transit);
        assign_if_present(r, "speed_transfer", robot_out.speed_transfer);
        assign_if_present(r, "max_steer", robot_out.max_steer);
        assign_if_present(r, "max_accel", robot_out.max_accel);
        assign_if_present(r, "max_v_push", robot_out.max_v_push);
        assign_if_present(r, "max_v_nonpush", robot_out.max_v_nonpush);
        assign_if_present(r, "min_moving_speed", robot_out.min_moving_speed);
        assign_if_present(r, "min_sustain_speed", robot_out.min_sustain_speed);
        assign_if_present(r, "launch_margin", robot_out.launch_margin);
        assign_if_present(r, "max_breakaway_accel", robot_out.max_breakaway_accel);
        assign_if_present(r, "settle_position_tolerance", robot_out.settle_position_tolerance);
        assign_if_present(r, "settle_cooldown_s", robot_out.settle_cooldown_s);
        assign_if_present(r, "settle_max_attempts", robot_out.settle_max_attempts);
    }

    if (root.contains("mpc")) {
        const auto& m = root.at("mpc");
        if (!m.is_object()) {
            throw std::runtime_error("mpc::load_spec_from_json: 'mpc' must be an object in " +
                                      path);
        }
        warn_unknown_keys(m, mpc_param_keys(), "mpc");
        assign_if_present(m, "dt", mpc_out.dt);
        assign_if_present(m, "control_delay_steps", mpc_out.control_delay_steps);
        assign_if_present(m, "loop_hz", mpc_out.loop_hz);
        assign_if_present(m, "w_dist", mpc_out.w_dist);
        assign_if_present(m, "w_dist_dirchange", mpc_out.w_dist_dirchange);
        assign_if_present(m, "w_yaw", mpc_out.w_yaw);
        assign_if_present(m, "w_vel", mpc_out.w_vel);
        assign_if_present(m, "w_lat", mpc_out.w_lat);
        assign_if_present(m, "w_control", mpc_out.w_control);
        assign_if_present(m, "w_delta_rate", mpc_out.w_delta_rate);
        assign_if_present(m, "vel_scale", mpc_out.vel_scale);
        assign_if_present(m, "vel_scale_back", mpc_out.vel_scale_back);
        assign_if_present(m, "dir_change_scale", mpc_out.dir_change_scale);
    }
}

} // namespace mpc
