// vesc_driver_main.cpp -> binary `vesc_driver`
//
// Part 3 of VescDriver: the real driver process. Thin I/O shell around
// DriverCore (control logic) + ZmqChannels (ZMQ endpoints) +
// VescProtocol/SerialPort/PortDiscovery (UART link to the VESC) +
// AckermannCodec (ackermann wire decode). See VescDriver/README.md for the
// full CLI/config/protocol reference.
//
// Portability: C++14 only, POSIX only -- see VescDriver/CMakeLists.txt's
// HARD PORTABILITY RULES.

#include "AckermannCodec.h"
#include "DriverCore.h"
#include "PortDiscovery.h"
#include "SerialPort.h"
#include "SteeringCalib.h"
#include "VescProtocol.h"
#include "ZmqChannels.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "../third_party/nlohmann/json.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;
void handle_stop_signal(int) { g_stop = 1; }

// ---------------------------------------------------------------------
// CLI parsing
// ---------------------------------------------------------------------

struct CliOptions {
    std::string config_path;  // empty -> resolved default (own-dir/config, then ./config).
    bool has_robot = false;
    std::string robot;
    bool has_ackermann_port = false;
    int ackermann_port = 0;
    bool has_control_port = false;
    int control_port = 0;
    bool has_telemetry_port = false;
    int telemetry_port = 0;
    bool has_serial_port = false;
    std::string serial_port;
    bool has_mode = false;
    std::string mode;
    bool has_calibration = false;
    std::string calibration;
    std::string steering_calib_path;  // empty -> auto-resolve via SteeringCalib::resolve_default_load_path().
    bool print_telemetry = false;

    // VEHICLE drive-direction sign override (see SteeringCalib.h's
    // drive_invert) -- has_drive_invert==false means "use whatever the
    // loaded steering calibration says" (false if no calibration file
    // loaded, matching SteeringCalib{}'s own default).
    bool has_drive_invert = false;
    bool drive_invert = false;
};

bool next_value(int argc, char** argv, int* i, std::string* out) {
    const std::string arg = argv[*i];
    const size_t eq = arg.find('=');
    if (eq != std::string::npos) {
        *out = arg.substr(eq + 1);
        return true;
    }
    if (*i + 1 < argc) {
        *out = argv[++(*i)];
        return true;
    }
    return false;
}

bool flag_matches(const std::string& arg, const char* flag) {
    const std::string f = flag;
    return arg == f || arg.rfind(f + "=", 0) == 0;
}

CliOptions parse_cli(int argc, char** argv) {
    CliOptions o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (flag_matches(arg, "--config") && next_value(argc, argv, &i, &value)) {
            o.config_path = value;
        } else if (flag_matches(arg, "--robot") && next_value(argc, argv, &i, &value)) {
            o.has_robot = true;
            o.robot = value;
        } else if (flag_matches(arg, "--ackermann-port") && next_value(argc, argv, &i, &value)) {
            o.has_ackermann_port = true;
            o.ackermann_port = std::atoi(value.c_str());
        } else if (flag_matches(arg, "--control-port") && next_value(argc, argv, &i, &value)) {
            o.has_control_port = true;
            o.control_port = std::atoi(value.c_str());
        } else if (flag_matches(arg, "--telemetry-port") && next_value(argc, argv, &i, &value)) {
            o.has_telemetry_port = true;
            o.telemetry_port = std::atoi(value.c_str());
        } else if (flag_matches(arg, "--serial-port") && next_value(argc, argv, &i, &value)) {
            o.has_serial_port = true;
            o.serial_port = value;
        } else if (flag_matches(arg, "--mode") && next_value(argc, argv, &i, &value)) {
            o.has_mode = true;
            o.mode = value;
        } else if (flag_matches(arg, "--calibration") && next_value(argc, argv, &i, &value)) {
            o.has_calibration = true;
            o.calibration = value;
        } else if (flag_matches(arg, "--steering-calib") && next_value(argc, argv, &i, &value)) {
            o.steering_calib_path = value;
        } else if (arg == "--print-telemetry") {
            o.print_telemetry = true;
        } else if (arg == "--drive-invert") {
            o.has_drive_invert = true;
            o.drive_invert = true;
        } else if (arg == "--no-drive-invert") {
            o.has_drive_invert = true;
            o.drive_invert = false;
        } else {
            std::fprintf(stderr, "vesc_driver: ignoring unrecognized argument '%s'\n", arg.c_str());
        }
    }
    return o;
}

std::string own_exe_dir() {
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "";
    buf[n] = '\0';
    const std::string path(buf);
    const size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string(".") : path.substr(0, slash);
}

bool path_exists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

// Default --config resolution: <executable's own dir>/config/driver_config.json
// if that exists, else ./config/driver_config.json (cwd-relative fallback).
std::string resolve_default_config_path() {
    const std::string exe_dir = own_exe_dir();
    if (!exe_dir.empty()) {
        const std::string candidate = exe_dir + "/config/driver_config.json";
        if (path_exists(candidate)) return candidate;
    }
    return "./config/driver_config.json";
}

void apply_cli_overrides(const CliOptions& cli, vesc::DriverConfig* config) {
    if (cli.has_robot) config->robot_name = cli.robot;
    if (cli.has_ackermann_port) config->ackermann_port = cli.ackermann_port;
    if (cli.has_control_port) config->control_port = cli.control_port;
    if (cli.has_telemetry_port) config->telemetry_port = cli.telemetry_port;
    if (cli.has_serial_port) config->serial_port = cli.serial_port;
    if (cli.has_mode) config->mode = cli.mode;
    if (cli.has_calibration) config->calibration_file = cli.calibration;
}

// ---------------------------------------------------------------------
// Runtime (mutable, caller-owned) state -- the "held command" bookkeeping
// DriverCore itself deliberately does not own (see DriverCore.h's file
// header / TickInputs's own doc comment).
// ---------------------------------------------------------------------

struct RuntimeState {
    vesc::Source source = vesc::Source::kAckermann;

    bool ackermann_ever_valid = false;
    double held_accel = 0.0;
    double held_steering = 0.0;
    double last_valid_ackermann_time_s = 0.0;

    bool calib_ever_set = false;
    vesc::RawCalibMode calib_mode = vesc::RawCalibMode::kDuty;
    double calib_value = 0.0;  // already safety-clamped (see DriverCore::clamp_raw_value()).
    double calib_ttl_ms = 500.0;
    double last_calib_time_s = 0.0;

    bool servo_override_active = false;
    double servo_override_value = 0.5;

    double erpm_meas = 0.0;
};

struct TelemetrySnapshot {
    double t = 0.0;
    double erpm = 0.0;
    double duty = 0.0;
    double current_motor = 0.0;
    double current_in = 0.0;
    double v_in = 0.0;
    int32_t tacho = 0;
    int fault = 0;
    std::string source = "ackermann";
    std::string mode = "erpm";
    double cmd_value = 0.0;
    double cmd_age_ms = -1.0;
    double v_est = 0.0;
};

nlohmann::json telemetry_to_json(const TelemetrySnapshot& s) {
    nlohmann::json j;
    j["t"] = s.t;
    j["erpm"] = s.erpm;
    j["duty"] = s.duty;
    j["current_motor"] = s.current_motor;
    j["current_in"] = s.current_in;
    j["v_in"] = s.v_in;
    j["tacho"] = s.tacho;
    j["fault"] = s.fault;
    j["source"] = s.source;
    j["mode"] = s.mode;
    j["cmd_value"] = s.cmd_value;
    j["cmd_age_ms"] = s.cmd_age_ms;
    j["v_est"] = s.v_est;
    return j;
}

void send_motor_action(vesc::SerialPort* port, const vesc::MotorAction& action) {
    std::vector<uint8_t> payload;
    switch (action.type) {
        case vesc::MotorAction::Type::kNone:
            payload = vesc::build_alive();
            break;
        case vesc::MotorAction::Type::kRpm:
            payload = vesc::build_set_rpm(static_cast<int32_t>(std::lround(action.value)));
            break;
        case vesc::MotorAction::Type::kDuty:
            payload = vesc::build_set_duty(action.value);
            break;
        case vesc::MotorAction::Type::kCurrent:
            payload = vesc::build_set_current(action.value);
            break;
        case vesc::MotorAction::Type::kBrake:
            payload = vesc::build_set_current_brake(action.value);
            break;
    }
    port->write_all(vesc::encode_frame(payload));
}

// ---------------------------------------------------------------------
// Control REP request handling.
// ---------------------------------------------------------------------

struct ControlOutcome {
    std::string reply_json;
    bool immediate_action = false;
    vesc::MotorAction action;
};

ControlOutcome handle_control_request(const std::string& req_text, double now_s, RuntimeState* state,
                                        vesc::DriverCore* core, const std::string& robot_name,
                                        const std::string& serial_port_path, const std::string& fw_string,
                                        const TelemetrySnapshot& snapshot) {
    ControlOutcome out;
    nlohmann::json reply;

    nlohmann::json req;
    try {
        req = nlohmann::json::parse(req_text);
    } catch (...) {
        reply["ok"] = false;
        reply["error"] = "malformed request";
        out.reply_json = reply.dump();
        return out;
    }
    if (!req.is_object() || !req.contains("cmd") || !req.at("cmd").is_string()) {
        reply["ok"] = false;
        reply["error"] = "unknown cmd";
        out.reply_json = reply.dump();
        return out;
    }
    const std::string cmd = req.at("cmd").get<std::string>();

    if (cmd == "ping") {
        reply["ok"] = true;
        reply["robot"] = robot_name;
        reply["state"] = core->state_string();
        reply["source"] = (state->source == vesc::Source::kAckermann) ? "ackermann" : "calib";
        reply["serial_port"] = serial_port_path;
        reply["fw"] = fw_string;
    } else if (cmd == "set_source") {
        const bool have = req.contains("source") && req.at("source").is_string();
        const std::string src_str = have ? req.at("source").get<std::string>() : std::string();
        if (have && src_str == "ackermann") {
            state->source = vesc::Source::kAckermann;
            reply["ok"] = true;
        } else if (have && src_str == "calib") {
            state->source = vesc::Source::kCalib;
            reply["ok"] = true;
        } else {
            reply["ok"] = false;
            reply["error"] = "invalid source (expected \"ackermann\" or \"calib\")";
        }
    } else if (cmd == "raw") {
        if (state->source != vesc::Source::kCalib) {
            reply["ok"] = false;
            reply["error"] = "raw only legal while source==\"calib\"";
        } else {
            const std::string mode_str = req.contains("mode") && req.at("mode").is_string()
                                              ? req.at("mode").get<std::string>()
                                              : std::string();
            bool mode_ok = false;
            const vesc::RawCalibMode rmode = vesc::raw_calib_mode_from_string(mode_str, &mode_ok);
            const bool has_value = req.contains("value") && req.at("value").is_number();
            if (!mode_ok || !has_value) {
                reply["ok"] = false;
                reply["error"] = "invalid/missing mode or value";
            } else {
                const double raw_value = req.at("value").get<double>();
                double ttl_ms = 500.0;
                if (req.contains("ttl_ms") && req.at("ttl_ms").is_number()) {
                    ttl_ms = req.at("ttl_ms").get<double>();
                }
                const double applied =
                    vesc::DriverCore::clamp_raw_value(rmode, raw_value, core->config().safety);
                state->calib_ever_set = true;
                state->calib_mode = rmode;
                state->calib_value = applied;
                state->calib_ttl_ms = ttl_ms;
                state->last_calib_time_s = now_s;
                reply["ok"] = true;
                reply["applied_value"] = applied;
            }
        }
    } else if (cmd == "servo") {
        const bool has_value = req.contains("value") && req.at("value").is_number();
        if (!has_value) {
            reply["ok"] = false;
            reply["error"] = "missing value";
        } else {
            double v = req.at("value").get<double>();
            if (v < 0.0) v = 0.0;
            if (v > 1.0) v = 1.0;
            state->servo_override_active = true;
            state->servo_override_value = v;
            reply["ok"] = true;
        }
    } else if (cmd == "stop") {
        out.action = core->stop();
        out.immediate_action = true;
        state->calib_ever_set = false;  // "raw command cleared" -- see the contract.
        reply["ok"] = true;
    } else if (cmd == "telemetry") {
        reply = telemetry_to_json(snapshot);
        reply["ok"] = true;
    } else {
        reply["ok"] = false;
        reply["error"] = "unknown cmd";
    }

    out.reply_json = reply.dump();
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    const CliOptions cli = parse_cli(argc, argv);

    const std::string config_path = !cli.config_path.empty() ? cli.config_path : resolve_default_config_path();
    vesc::ConfigLoadResult load_result = vesc::load_driver_config(config_path);
    vesc::DriverConfig config = load_result.config;
    if (!load_result.ok) {
        std::fprintf(stderr, "vesc_driver: could not load config '%s' (%s) -- using built-in defaults.\n",
                      config_path.c_str(), load_result.error.c_str());
    } else {
        std::fprintf(stderr, "vesc_driver: loaded config '%s'\n", config_path.c_str());
        for (const auto& w : load_result.warnings) {
            std::fprintf(stderr, "vesc_driver: config warning: %s\n", w.c_str());
        }
    }
    apply_cli_overrides(cli, &config);

    // Shared steering calibration (SteeringCalib.h): center/min_pos/max_pos/
    // invert are overridden from this file when it loads successfully --
    // gain_per_rad is DELIBERATELY left alone (rad_per_unit is reserved/
    // unused for now, not interchangeable with gain_per_rad's units). Not
    // finding a calibration file is normal (fresh install / no calibration
    // run yet), not an error -- driver_config.json's own servo.* values
    // stay in effect in that case. Mirrors vesc_teleop_main.cpp's own use
    // of this same module.
    vesc::SteeringCalib steering_calib;
    std::string steering_calib_err;
    const std::string steering_calib_path = vesc::resolve_default_load_path(cli.steering_calib_path);
    const bool steering_calib_loaded = vesc::load(steering_calib_path, &steering_calib, &steering_calib_err);
    std::string servo_center_source = "driver_config.json";
    if (steering_calib_loaded) {
        config.servo.center = steering_calib.center;
        config.servo.min_pos = steering_calib.min_pos;
        config.servo.max_pos = steering_calib.max_pos;
        config.servo.invert = steering_calib.invert;
        servo_center_source = "steering calibration (" + steering_calib_path + ")";
    }

    // VEHICLE drive-direction sign (see SteeringCalib.h's drive_invert):
    // from the shared calibration by default, CLI-overridable. Applied
    // below at the vehicle<->motor boundary -- NEVER inside DriverCore
    // itself (which stays wiring-agnostic, exactly like TeleopCore.h's
    // own drive_sign()/TeleopConfig::drive_invert design: both the
    // outgoing motor command AND the incoming measured erpm must be
    // converted, or the two would disagree on which frame v_target/v_now
    // are in).
    bool drive_invert = steering_calib_loaded ? steering_calib.drive_invert : false;
    if (cli.has_drive_invert) drive_invert = cli.drive_invert;
    const double drive_sign = drive_invert ? -1.0 : 1.0;

    // Serial discovery + FW handshake. An explicit --serial-port is passed
    // straight through as find_vesc_port()'s override (used directly, no
    // enumeration -- but STILL must answer the handshake, per
    // PortDiscovery.h's own contract) rather than trusted blindly. Probes
    // at config.baud so discovery and the real (re)open below always agree
    // on baud rate -- a configured non-default baud would otherwise make
    // every probe handshake at the wrong rate and discovery would never
    // find the VESC.
    const vesc::DiscoveryResult disc = vesc::find_vesc_port(config.serial_port, 400, config.baud);
    if (!disc.ok) {
        std::fprintf(stderr, "vesc_driver: could not find/handshake a VESC. Candidates tried:\n");
        for (const auto& e : disc.log) {
            std::fprintf(stderr, "  path=%-24s source=%-24s responded=%-3s note=%s\n", e.path.c_str(),
                          e.source.c_str(), e.responded ? "yes" : "no", e.note.c_str());
        }
        return 1;
    }

    vesc::SerialPort port;
    if (!port.open(disc.path, config.baud)) {
        std::fprintf(stderr,
                      "vesc_driver: found a VESC at '%s' during discovery but could not (re)open it for "
                      "normal use: %s\n",
                      disc.path.c_str(), port.last_error().c_str());
        return 1;
    }
    const std::string fw_string = std::to_string(static_cast<int>(disc.fw_major)) + "." +
                                   std::to_string(static_cast<int>(disc.fw_minor));

    // SET_SERVO_POS's raw command id shifts on FW 2.x (no COMM_SET_HANDBRAKE
    // -- see VescProtocol.h's resolve_servo_cmd_id() CAVEAT comment). Resolve
    // it once here from the VESC's own reported firmware, rather than ever
    // sending the fixed modern id (12) blind -- on FW 2.x that id is
    // COMM_SET_MCCONF (a motor-config-blob WRITE), not SET_SERVO_POS.
    const uint8_t servo_cmd_id = vesc::resolve_servo_cmd_id(disc.fw_major);

    vesc::DriverCore core(config);

    vesc::ZmqChannels channels;
    std::string zmq_error;
    if (!channels.init(config.robot_name, config.ackermann_port, config.control_port, config.telemetry_port,
                        &zmq_error)) {
        std::fprintf(stderr, "vesc_driver: ZMQ init failed: %s\n", zmq_error.c_str());
        return 1;
    }

    std::printf(
        "vesc_driver: robot=%s serial=%s fw=%s servo_cmd_id=%d map=%s(%s) ackermann_port=%d control_port=%d "
        "telemetry_port=%d watchdog_ms=%.0f control_rate_hz=%.0f\n",
        config.robot_name.c_str(), disc.path.c_str(), fw_string.c_str(), static_cast<int>(servo_cmd_id),
        vesc::to_string(core.map_mode()).c_str(), core.used_calibration() ? "calibrated" : "linear-placeholder",
        config.ackermann_port, config.control_port, config.telemetry_port, config.watchdog_ms,
        config.control_rate_hz);
    std::fprintf(stderr, "vesc_driver: motor map: %s\n", core.motor_map_note().c_str());
    // Steering calibration: which source actually supplied servo.center/
    // min_pos/max_pos/invert (see the load block above, before serial
    // discovery) -- an operator glancing at startup output should
    // immediately know whether it's driver_config.json's own servo.* or the
    // shared calibration file (and which path), same "not found is normal,
    // not alarming" tone vesc_teleop_main.cpp already uses for this exact
    // message.
    if (!steering_calib_err.empty()) {
        std::printf("vesc_driver: steering calibration: %s\n", steering_calib_err.c_str());
    }
    std::printf(
        "vesc_driver: steering: source=%s center=%.3f min=%.3f max=%.3f invert=%s\n",
        servo_center_source.c_str(), config.servo.center, config.servo.min_pos, config.servo.max_pos,
        config.servo.invert ? "true" : "false");
    std::printf("vesc_driver: drive_invert=%s\n", drive_invert ? "true" : "false");
    if (disc.fw_major <= 2) {
        std::fprintf(stderr,
                      "vesc_driver: WARNING: servo_cmd_id=%d is derived from firmware-version history "
                      "(FW 2.x lacks COMM_SET_HANDBRAKE), not confirmed against this exact unit -- "
                      "verify visually with small servo movements before trusting it.\n",
                      static_cast<int>(servo_cmd_id));
    }
    std::fflush(stdout);

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    RuntimeState state;
    TelemetrySnapshot snapshot;
    vesc::FrameDecoder decoder;

    const auto proc_start = std::chrono::steady_clock::now();
    auto now_seconds = [&proc_start]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - proc_start).count();
    };

    const double control_period_s = 1.0 / std::max(1.0, config.control_rate_hz);
    const double get_values_interval_s = 1.0 / 25.0;
    const double telemetry_interval_s = 1.0 / std::max(1.0, config.telemetry_rate_hz);
    double last_get_values_req_s = -1.0e9;
    double last_telemetry_pub_s = -1.0e9;

    auto next_tick = std::chrono::steady_clock::now();

    while (!g_stop) {
        const double now_s = now_seconds();

        // 1. Drain the ackermann SUB socket (newest payload wins).
        std::string ack_payload;
        if (channels.poll_ackermann(&ack_payload)) {
            const vesc::AckermannCommand cmd = vesc::decode_ackermann_payload(ack_payload);
            if (cmd.ok) {
                state.held_accel = cmd.accel;
                state.held_steering = cmd.steering;
                state.last_valid_ackermann_time_s = now_s;
                state.ackermann_ever_valid = true;
                if (state.source == vesc::Source::kAckermann) {
                    // "in ackermann source the ackermann steering stream
                    // overrides it on next command" -- see the frozen
                    // control-protocol contract's "servo" verb.
                    state.servo_override_active = false;
                }
            }
            // Malformed: silently retained -- per the frozen wire contract,
            // this must NOT refresh the watchdog timer (last_valid_*_s is
            // only touched on cmd.ok==true, above).
        }

        // 2. Serve at most one pending control REP request immediately.
        std::string req_json;
        if (channels.poll_control_request(&req_json)) {
            const ControlOutcome outcome = handle_control_request(req_json, now_s, &state, &core,
                                                                     config.robot_name, disc.path, fw_string,
                                                                     snapshot);
            channels.send_control_reply(outcome.reply_json);
            if (outcome.immediate_action) {
                send_motor_action(&port, outcome.action);
            }
        }

        // 3. Drain the serial link and parse any GET_VALUES replies.
        {
            std::vector<uint8_t> buf;
            if (port.read_available(&buf) > 0) {
                decoder.feed(buf);
            }
            std::vector<uint8_t> payload;
            while (decoder.pop_payload(&payload)) {
                if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::GET_VALUES)) {
                    const vesc::VescValues v = vesc::parse_get_values(payload);
                    if (v.ok) {
                        state.erpm_meas = v.erpm;
                        snapshot.erpm = v.erpm;
                        snapshot.duty = v.duty;
                        snapshot.current_motor = v.current_motor;
                        snapshot.current_in = v.current_in;
                        snapshot.v_in = v.v_in;
                        snapshot.tacho = v.tachometer;
                        snapshot.fault = static_cast<int>(v.fault);
                    }
                }
                // Anything else (e.g. a stray FW_VERSION reply) is ignored
                // post-startup.
            }
        }

        // 4. Step DriverCore.
        vesc::TickInputs in;
        in.now_s = now_s;
        in.source = state.source;
        in.ackermann.valid = state.ackermann_ever_valid;
        in.ackermann.accel = state.held_accel;
        in.ackermann.steering = state.held_steering;
        in.ackermann.age_s = state.ackermann_ever_valid ? (now_s - state.last_valid_ackermann_time_s) : 1.0e18;
        in.calib.has_command = state.calib_ever_set;
        in.calib.mode = state.calib_mode;
        in.calib.value = state.calib_value;
        in.calib.age_s = state.calib_ever_set ? (now_s - state.last_calib_time_s) : 1.0e18;
        in.calib.ttl_ms = state.calib_ttl_ms;
        in.servo_override_active = state.servo_override_active;
        in.servo_override_value = state.servo_override_value;
        // MOTOR -> VEHICLE frame: state.erpm_meas is the raw VESC-reported
        // erpm (motor frame); DriverCore's v_now/v_target both live in
        // VEHICLE frame ("+ is forward"), so this must be converted here,
        // matching TeleopCore's own feed_telemetry() conversion.
        in.erpm_meas = state.erpm_meas * drive_sign;

        vesc::TickResult result = core.tick(in);

        // VEHICLE -> MOTOR frame: DriverCore itself is wiring-agnostic and
        // returns result.motor.value in VEHICLE frame -- invert here,
        // right at the hardware boundary, exactly like TeleopCore's own
        // drive_sign() applied to its emitted action.value. kBrake/kNone
        // are magnitude-only/sign-irrelevant and are deliberately left
        // untouched (mirrors TeleopCore never inverting brake_amps).
        if (drive_invert && (result.motor.type == vesc::MotorAction::Type::kRpm ||
                              result.motor.type == vesc::MotorAction::Type::kDuty ||
                              result.motor.type == vesc::MotorAction::Type::kCurrent)) {
            result.motor.value = -result.motor.value;
        }

        // 5. Send to the VESC -- every tick, doubling as firmware keepalive
        // (COMM_ALIVE when the action is "none").
        send_motor_action(&port, result.motor);
        if (config.servo.enabled) {
            port.write_all(vesc::encode_frame(vesc::build_set_servo_pos(result.servo_pos, servo_cmd_id)));
        }

        snapshot.t = now_s;
        snapshot.source = (state.source == vesc::Source::kAckermann) ? "ackermann" : "calib";
        snapshot.mode = (state.source == vesc::Source::kAckermann) ? vesc::to_string(core.map_mode())
                                                                     : vesc::to_string(state.calib_mode);
        snapshot.cmd_value = result.motor.value;
        if (state.source == vesc::Source::kAckermann) {
            snapshot.cmd_age_ms =
                state.ackermann_ever_valid ? 1000.0 * (now_s - state.last_valid_ackermann_time_s) : -1.0;
        } else {
            snapshot.cmd_age_ms = state.calib_ever_set ? 1000.0 * (now_s - state.last_calib_time_s) : -1.0;
        }
        snapshot.v_est =
            (core.effective_erpm_per_mps() > 1e-9) ? (state.erpm_meas / core.effective_erpm_per_mps()) : 0.0;

        // 6. GET_VALUES poll cadence (~25 Hz), interleaved with the above.
        if (now_s - last_get_values_req_s >= get_values_interval_s) {
            port.write_all(vesc::encode_frame(vesc::build_get_values()));
            last_get_values_req_s = now_s;
        }

        // 7. Telemetry publish cadence.
        if (now_s - last_telemetry_pub_s >= telemetry_interval_s) {
            const std::string topic = "/" + config.robot_name + "/vesc_telemetry";
            const std::string payload = telemetry_to_json(snapshot).dump();
            channels.publish_telemetry(topic, payload);
            last_telemetry_pub_s = now_s;
            if (cli.print_telemetry) {
                std::printf("[telemetry] %s\n", payload.c_str());
                std::fflush(stdout);
            }
        }

        // 8. Sleep to the next control tick.
        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(control_period_s));
        const auto now_tp = std::chrono::steady_clock::now();
        if (next_tick < now_tp - std::chrono::milliseconds(200)) {
            // Fell far behind (e.g. a debugger pause, or an overloaded
            // host) -- resync rather than spinning to catch up forever.
            next_tick = now_tp;
        }
        std::this_thread::sleep_until(next_tick);
    }

    // SIGINT/SIGTERM: brake burst (SET_CURRENT_BRAKE x5 over ~100ms),
    // servo to center, clean exit.
    std::fprintf(stderr, "vesc_driver: shutting down (brake burst)...\n");
    vesc::MotorAction shutdown_brake;
    shutdown_brake.type = vesc::MotorAction::Type::kBrake;
    shutdown_brake.value = std::min(2.0, std::fabs(config.safety.max_current));
    for (int i = 0; i < 5; ++i) {
        send_motor_action(&port, shutdown_brake);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (config.servo.enabled) {
        port.write_all(vesc::encode_frame(vesc::build_set_servo_pos(config.servo.center, servo_cmd_id)));
    }
    channels.shutdown();
    return 0;
}
