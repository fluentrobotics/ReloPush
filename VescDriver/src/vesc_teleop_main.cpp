// vesc_teleop_main.cpp -> binary `vesc_teleop`
//
// Interactive motor teleoperation CLI: the FIRST, most cautious contact
// with a brand-new, unknown-firmware VESC driving a bare motor with no
// load (the previous unit was destroyed by cumulative high-current stall
// damage -- see README.md's "Teleoperation" section). This tool's job is
// to let an operator nudge the motor by hand, at low commanded
// magnitude, while watching live telemetry, with a deadman timer and a
// current/fault abort latch standing between a stuck key (or a runaway
// firmware surprise) and sustained high current.
//
// All the actual state-machine logic (mode/magnitude bookkeeping, drive/
// stop/deadman/abort transitions) lives in TeleopCore.h/.cpp -- PURE, no
// I/O -- this file is the thin shell: terminal input, the serial link to
// the VESC, and rendering the status line.
//
// Portability: C++14 only, POSIX only -- see VescDriver/CMakeLists.txt's
// HARD PORTABILITY RULES.

#include "PortDiscovery.h"
#include "SerialPort.h"
#include "SteeringCalib.h"
#include "TeleopCore.h"
#include "VescProtocol.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t g_stop = 0;
void handle_stop_signal(int) { g_stop = 1; }

// ---------------------------------------------------------------------
// CLI parsing (same "--flag value" / "--flag=value" convention as
// vesc_driver_main.cpp / vesc_mcconf_main.cpp / fake_vesc.cpp).
// ---------------------------------------------------------------------

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

struct CliOptions {
    std::string serial_port;  // empty -> auto-discover.
    int baud = 115200;
    double max_duty = 0.2;
    double max_erpm = 6000.0;
    double duty_step = 0.005;
    double erpm_step = 100.0;
    double current_abort = 8.0;
    double deadman_ms = 2000.0;
    double rate_hz = 20.0;
    double duty_ramp = 0.1;
    double erpm_ramp = 500.0;
    bool ramp_enabled = false;
    double speed_kp = 2e-6;
    double speed_ki = 1e-5;
    double speed_ff_gain = 4400.0;

    // Steering (see SteeringCalib.h / TeleopCore.h's "STEERING
    // SUB-STATE-MACHINE"). steering_calib_path empty -> auto-resolve via
    // SteeringCalib::resolve_default_load_path()/resolve_default_save_path().
    std::string steering_calib_path;
    bool has_servo_cmd_id = false;
    uint8_t servo_cmd_id = 0;
    double servo_step = 0.01;
    double servo_fine_step = 0.002;
    bool has_servo_min = false;
    double servo_min = 0.0;
    bool has_servo_max = false;
    double servo_max = 0.0;
    double servo_refresh_ms = 500.0;  // 0 = on-change-only, never periodic.
    bool no_steering = false;

    // VEHICLE drive-direction sign override (see SteeringCalib.h's
    // drive_invert / TeleopConfig::drive_invert's own comment) -- empty
    // (has_drive_invert==false) means "use whatever the loaded steering
    // calibration says" (false if --no-steering or the calibration
    // failed to load, matching calib's own default-constructed value).
    bool has_drive_invert = false;
    bool drive_invert = false;
};

CliOptions parse_cli(int argc, char** argv) {
    CliOptions o;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (flag_matches(arg, "--serial-port") && next_value(argc, argv, &i, &value)) {
            o.serial_port = value;
        } else if (flag_matches(arg, "--baud") && next_value(argc, argv, &i, &value)) {
            o.baud = std::atoi(value.c_str());
        } else if (flag_matches(arg, "--max-duty") && next_value(argc, argv, &i, &value)) {
            o.max_duty = std::atof(value.c_str());
        } else if (flag_matches(arg, "--max-erpm") && next_value(argc, argv, &i, &value)) {
            o.max_erpm = std::atof(value.c_str());
        } else if (flag_matches(arg, "--duty-step") && next_value(argc, argv, &i, &value)) {
            o.duty_step = std::atof(value.c_str());
        } else if (flag_matches(arg, "--erpm-step") && next_value(argc, argv, &i, &value)) {
            o.erpm_step = std::atof(value.c_str());
        } else if (flag_matches(arg, "--current-abort") && next_value(argc, argv, &i, &value)) {
            o.current_abort = std::atof(value.c_str());
        } else if (flag_matches(arg, "--deadman-ms") && next_value(argc, argv, &i, &value)) {
            o.deadman_ms = std::atof(value.c_str());
        } else if (flag_matches(arg, "--rate-hz") && next_value(argc, argv, &i, &value)) {
            o.rate_hz = std::atof(value.c_str());
        } else if (flag_matches(arg, "--duty-ramp") && next_value(argc, argv, &i, &value)) {
            o.duty_ramp = std::atof(value.c_str());
        } else if (flag_matches(arg, "--erpm-ramp") && next_value(argc, argv, &i, &value)) {
            o.erpm_ramp = std::atof(value.c_str());
        } else if (arg == "--ramp") {
            o.ramp_enabled = true;
        } else if (flag_matches(arg, "--speed-kp") && next_value(argc, argv, &i, &value)) {
            o.speed_kp = std::atof(value.c_str());
        } else if (flag_matches(arg, "--speed-ki") && next_value(argc, argv, &i, &value)) {
            o.speed_ki = std::atof(value.c_str());
        } else if (flag_matches(arg, "--speed-ff") && next_value(argc, argv, &i, &value)) {
            o.speed_ff_gain = std::atof(value.c_str());
        } else if (flag_matches(arg, "--steering-calib") && next_value(argc, argv, &i, &value)) {
            o.steering_calib_path = value;
        } else if (flag_matches(arg, "--servo-cmd-id") && next_value(argc, argv, &i, &value)) {
            o.has_servo_cmd_id = true;
            o.servo_cmd_id = static_cast<uint8_t>(std::atoi(value.c_str()));
        } else if (flag_matches(arg, "--servo-step") && next_value(argc, argv, &i, &value)) {
            o.servo_step = std::atof(value.c_str());
        } else if (flag_matches(arg, "--servo-fine-step") && next_value(argc, argv, &i, &value)) {
            o.servo_fine_step = std::atof(value.c_str());
        } else if (flag_matches(arg, "--servo-min") && next_value(argc, argv, &i, &value)) {
            o.has_servo_min = true;
            o.servo_min = std::atof(value.c_str());
        } else if (flag_matches(arg, "--servo-max") && next_value(argc, argv, &i, &value)) {
            o.has_servo_max = true;
            o.servo_max = std::atof(value.c_str());
        } else if (flag_matches(arg, "--servo-refresh-ms") && next_value(argc, argv, &i, &value)) {
            o.servo_refresh_ms = std::atof(value.c_str());
        } else if (arg == "--no-steering") {
            o.no_steering = true;
        } else if (arg == "--drive-invert") {
            o.has_drive_invert = true;
            o.drive_invert = true;
        } else if (arg == "--no-drive-invert") {
            o.has_drive_invert = true;
            o.drive_invert = false;
        } else {
            std::fprintf(stderr, "vesc_teleop: ignoring unrecognized argument '%s'\n", arg.c_str());
        }
    }
    return o;
}

// `calib` is the (already loaded, and CLI-min/max-overridden) steering
// calibration -- see main()'s "Steering calibration" block below. When
// cli.no_steering is set, main() passes a default-constructed
// SteeringCalib{} here: harmless, since main() never looks at/emits
// steering state in that case (see the "no_steering" gates around the
// main loop and shutdown below).
vesc::TeleopConfig teleop_config_from_cli(const CliOptions& cli, const vesc::SteeringCalib& calib) {
    vesc::TeleopConfig cfg;
    cfg.max_duty = cli.max_duty;
    cfg.max_erpm = cli.max_erpm;
    cfg.duty_step = cli.duty_step;
    cfg.erpm_step = cli.erpm_step;
    cfg.current_abort = cli.current_abort;
    cfg.deadman_ms = cli.deadman_ms;
    cfg.duty_ramp = cli.duty_ramp;
    cfg.erpm_ramp = cli.erpm_ramp;
    cfg.ramp_enabled = cli.ramp_enabled;
    cfg.speed_kp = cli.speed_kp;
    cfg.speed_ki = cli.speed_ki;
    cfg.speed_ff_gain = cli.speed_ff_gain;
    cfg.steer_center = calib.center;
    cfg.steer_min_pos = calib.min_pos;
    cfg.steer_max_pos = calib.max_pos;
    cfg.steer_invert = calib.invert;
    cfg.drive_invert = calib.drive_invert;
    cfg.steer_coarse_step = cli.servo_step;
    cfg.steer_fine_step = cli.servo_fine_step;
    return cfg;
}

// ---------------------------------------------------------------------
// Terminal handling: raw mode (with ISIG kept ON so Ctrl-C still raises
// SIGINT -- our own handler does the same graceful brake-then-exit as a
// 'q' keypress) when stdin is a real tty; non-blocking O_NONBLOCK reads
// either way (also applied to a piped stdin, which is how the automated
// integration test drives this binary without a real terminal).
// ---------------------------------------------------------------------

class StdinNonBlockGuard {
public:
    StdinNonBlockGuard() {
        orig_flags_ = fcntl(STDIN_FILENO, F_GETFL);
        if (orig_flags_ >= 0) {
            fcntl(STDIN_FILENO, F_SETFL, orig_flags_ | O_NONBLOCK);
            applied_ = true;
        }
    }
    ~StdinNonBlockGuard() {
        if (applied_) fcntl(STDIN_FILENO, F_SETFL, orig_flags_);
    }

private:
    int orig_flags_ = -1;
    bool applied_ = false;
};

class RawTerminalGuard {
public:
    RawTerminalGuard() {
        is_tty_ = (isatty(STDIN_FILENO) != 0);
        if (!is_tty_) return;
        if (tcgetattr(STDIN_FILENO, &orig_) != 0) {
            is_tty_ = false;  // treat "can't even query it" as non-tty -- degrade gracefully.
            return;
        }
        struct termios raw = orig_;
        cfmakeraw(&raw);
        raw.c_lflag |= ISIG;  // keep Ctrl-C/Ctrl-\ generating real signals for our own handler.
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
            applied_ = true;
        }
    }
    ~RawTerminalGuard() {
        if (applied_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }
    bool is_tty() const { return is_tty_; }

private:
    bool is_tty_ = false;
    bool applied_ = false;
    struct termios orig_ {};
};

// Drains every byte currently available on stdin (non-blocking, per
// StdinNonBlockGuard above) into `*out`. EOF (read() returning 0 -- e.g.
// a test's piped stdin writer closing early) just stops this tick's
// drain; it is NOT treated as a request to quit (only 'q' or a signal is).
void read_available_stdin_bytes(std::vector<char>* out) {
    for (;;) {
        char c = 0;
        const ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n == 1) {
            out->push_back(c);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        break;  // n==0 (EOF) or n<0 (EAGAIN/EWOULDBLOCK or a hard error): nothing more this tick.
    }
}

// ---------------------------------------------------------------------
// VESC link helpers.
// ---------------------------------------------------------------------

// `fw_is_v2`: FW 2.x has no COMM_ALIVE (id 30) at all -- unknown command
// ids are harmlessly ignored by that firmware, but this driver doesn't
// send them anyway, per the task's own "don't send it" contract. An idle
// motor needs no keepalive either way (nothing is being commanded), so
// this simply sends NOTHING for the kNone case on FW 2.x instead of
// build_alive().
void send_motor_action(vesc::SerialPort* port, const vesc::TeleopMotorAction& action, bool fw_is_v2) {
    std::vector<uint8_t> payload;
    switch (action.type) {
        case vesc::TeleopMotorAction::Type::kNone:
            if (fw_is_v2) return;
            payload = vesc::build_alive();
            break;
        case vesc::TeleopMotorAction::Type::kDuty:
            payload = vesc::build_set_duty(action.value);
            break;
        case vesc::TeleopMotorAction::Type::kRpm:
            payload = vesc::build_set_rpm(static_cast<int32_t>(std::lround(action.value)));
            break;
        case vesc::TeleopMotorAction::Type::kBrake:
            payload = vesc::build_set_current_brake(action.value);
            break;
    }
    port->write_all(vesc::encode_frame(payload));
}

// Sends a fresh GET_VALUES request and blocks (short, bounded) for its
// reply -- used only for the one-time startup sanity snapshot; the main
// loop below does its own non-blocking poll/parse instead. Parses via
// parse_get_values_for_fw() so the FW-2.x legacy layout is handled
// correctly from the very first snapshot, not just once the main loop
// starts.
bool blocking_get_values(vesc::SerialPort* port, vesc::FrameDecoder* decoder, uint8_t fw_major, vesc::VescValues* out,
                          double timeout_s) {
    port->write_all(vesc::encode_frame(vesc::build_get_values()));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<uint8_t> buf;
        if (port->read_available(&buf) > 0) decoder->feed(buf);
        std::vector<uint8_t> payload;
        while (decoder->pop_payload(&payload)) {
            if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::GET_VALUES)) {
                const vesc::VescValues v = vesc::parse_get_values_for_fw(payload, fw_major);
                if (v.ok) {
                    *out = v;
                    return true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// ---------------------------------------------------------------------
// Help / status text.
// ---------------------------------------------------------------------

// Prints the full CLI flag reference (name, default, meaning) to stdout.
// Deliberately takes no arguments and opens nothing -- called from
// --help/-h BEFORE any port discovery/serial connection is attempted, so
// it works with no VESC attached at all. Defaults shown here are
// CliOptions's own in-class defaults -- kept in sync with that struct by
// hand (mirrors README.md's "CLI flags" table under "Teleoperation").
void print_cli_flags_help() {
    std::printf(
        "Usage: vesc_teleop [flags]\n"
        "\n"
        "Interactive motor teleoperation CLI for first-contact testing of a VESC\n"
        "(see the key map below). --help/-h prints this and exits immediately --\n"
        "no serial port is opened and no hardware is required.\n"
        "\n"
        "Flags:\n"
        "  --serial-port <path>    (default: auto-discover) Skip auto-discovery, use\n"
        "                          this port directly (still handshaked).\n"
        "  --baud <n>              (default: 115200) Serial baud rate.\n"
        "  --max-duty <x>          (default: 0.2) Duty-mode magnitude ceiling.\n"
        "  --max-erpm <x>          (default: 6000) Erpm-mode magnitude ceiling.\n"
        "  --duty-step <x>         (default: 0.005) '+'/'-' step size in duty mode.\n"
        "  --erpm-step <x>         (default: 100) '+'/'-' step size in erpm mode.\n"
        "  --current-abort <x>     (default: 8.0) |motor current| (A) that latches\n"
        "                          the abort.\n"
        "  --deadman-ms <x>        (default: 2000) No-keypress timeout (ms) before\n"
        "                          auto-stopping.\n"
        "  --rate-hz <x>           (default: 20) Control-loop / telemetry-poll rate\n"
        "                          (Hz).\n"
        "  --duty-ramp <x>         (default: 0.1) Duty-mode ramp rate (duty\n"
        "                          units/second); also the speed governor's own\n"
        "                          (always-on) output slew rate.\n"
        "  --erpm-ramp <x>         (default: 500) Erpm-mode ramp rate (erpm/second).\n"
        "  --ramp                  (default: off) Start with RAMP mode already\n"
        "                          enabled.\n"
        "  --speed-kp <x>          (default: 2e-6) Speed governor proportional gain\n"
        "                          (duty per erpm of error).\n"
        "  --speed-ki <x>          (default: 1e-5) Speed governor integral gain\n"
        "                          (duty per erpm-second of error).\n"
        "  --speed-ff <x>          (default: 4400) Speed governor feedforward gain\n"
        "                          (erpm per duty*volt); 0 disables it (pure PI).\n"
        "  --steering-calib <path> (default: auto-resolve, see README) Explicit path\n"
        "                          to the steering calibration JSON -- also used as\n"
        "                          the save path for 'W' (trim-mode center-save).\n"
        "  --servo-cmd-id <n>      (default: derived from firmware version) Force\n"
        "                          the raw SET_SERVO_POS command id instead of\n"
        "                          auto-resolving it from the VESC's reported\n"
        "                          firmware major version.\n"
        "  --servo-step <x>        (default: 0.01) 'a'/'d' coarse steering step.\n"
        "  --servo-fine-step <x>   (default: 0.002) 'a'/'d' steering step while\n"
        "                          trim mode ('T') is active.\n"
        "  --servo-min <x>         (default: from calibration file) Override the\n"
        "                          steering travel lower bound (0..1).\n"
        "  --servo-max <x>         (default: from calibration file) Override the\n"
        "                          steering travel upper bound (0..1) -- keep it\n"
        "                          above --servo-min or steering will clamp\n"
        "                          unexpectedly.\n"
        "  --servo-refresh-ms <x>  (default: 500) Periodic SET_SERVO_POS refresh\n"
        "                          interval (ms) once steering has been touched;\n"
        "                          0 = on-change-only, never periodic.\n"
        "  --no-steering           (default: off) Disable steering entirely: no\n"
        "                          calibration load, no SET_SERVO_POS ever sent.\n"
        "  --drive-invert          (default: from steering calibration) Force the\n"
        "                          VEHICLE drive-direction sign inverted (a positive\n"
        "                          motor command physically drives backward) --\n"
        "                          overrides the loaded calibration's own\n"
        "                          drive_invert, applies even with --no-steering.\n"
        "  --no-drive-invert       Force drive_invert OFF, overriding the loaded\n"
        "                          calibration.\n"
        "  --help, -h              Print this help and exit (no hardware required).\n"
        "\n"
        "Key map (also printed after connecting to a real VESC):\n");
    std::fflush(stdout);
}

void print_help(const CliOptions& cli) {
    std::printf(
        "\n"
        "vesc_teleop -- interactive motor teleoperation (first-contact testing)\n"
        "\n"
        "  w          drive forward\n"
        "  s          drive backward\n"
        "  SPACE  x   immediate stop (brake, then idle)\n"
        "  a  d       steer left/right (coarse step; fine step while trim mode is\n"
        "             active -- see 'T' below)\n"
        "  k  R       snap steering to the calibrated center (synonyms; available\n"
        "             in ANY mode/state, not gated to trim mode)\n"
        "  T          toggle TRIM MODE -- stops driving first, blocks 'w'/'s' while\n"
        "             active, and switches 'a'/'d' to the fine step\n"
        "  W          save the current steering position as the new calibrated\n"
        "             center, persisted to the calibration file\n"
        "  D          select duty mode\n"
        "  E          select erpm mode\n"
        "  V          select speed mode (duty-actuated, erpm-feedback governor --\n"
        "             see below; the wire only ever carries SET_DUTY in this mode)\n"
        "  +  =       increase the current mode's magnitude by one step\n"
        "  -  _       decrease the current mode's magnitude by one step (floored)\n"
        "  <digits> . then ENTER   type a number, ENTER to set it as the current\n"
        "                          mode's magnitude directly (ESC cancels) -- in\n"
        "                          speed mode this sets the TARGET ERPM\n"
        "  c          clear a latched abort (see below) -- required before driving again\n"
        "  z          toggle RAMP mode on/off (slews toward the target instead of\n"
        "             stepping instantly -- see 'Z' below to set the rate)\n"
        "  Z  <digits> . then ENTER   arm ramp-rate entry: sets the CURRENT mode's\n"
        "             ramp rate instead of its magnitude (ESC cancels) -- in speed\n"
        "             mode this sets the governor's own output slew rate\n"
        "  q          quit (brakes first)\n"
        "\n"
        "RETIRED (inert -- pressing one prints a hint naming its replacement, does\n"
        "no drive/steer action): f, b, r, j, l, e, v, A. See the key map above for\n"
        "what replaced each one.\n"
        "\n"
        "SAFETY: deadman -- motor auto-stops after %.1f s without any keypress.\n"
        "        current-abort -- latches a brake if |motor current| exceeds %.1fA\n"
        "        or the VESC reports a nonzero fault code; press 'c' to clear.\n"
        "        RAMP mode / speed mode do NOT weaken any of the above --\n"
        "        stop/deadman/abort always brake immediately, never a gradual\n"
        "        ramp-down or governor wind-down.\n"
        "\n"
        "SPEED MODE: unlike this VESC's own native erpm mode (whose speed PID can\n"
        "be dead below a minimum setpoint, then engage violently once past it),\n"
        "speed mode drives via duty only and closes the loop on MEASURED erpm --\n"
        "this also makes the actual speed independent of battery voltage: as the\n"
        "battery sags under load, the feedback compensates automatically instead\n"
        "of the commanded speed drooping with it.\n"
        "\n"
        "STEERING: independent of drive mode -- 'a'/'d'/'k'/'R'/'T'/'W' work\n"
        "identically whether idle/driving/braking/aborted, and every steering key\n"
        "still refreshes the deadman like any other key. Trim mode ('T') stops\n"
        "the drive first and blocks 'w'/'s' while active, so steering can be tuned\n"
        "with the drivetrain guaranteed inert. The safety paths\n"
        "(stop/deadman/abort) never move steering -- only 'a'/'d'/'k'/'R' do.\n"
        "Nothing is sent to the VESC's servo output until the FIRST steering\n"
        "keypress of the session: this avoids a startup jump to whatever\n"
        "position the calibration file happens to hold.\n"
        "\n",
        cli.deadman_ms / 1000.0, cli.current_abort);
    std::fflush(stdout);
}

std::string mode_name(vesc::TeleopMode m) {
    if (m == vesc::TeleopMode::kDuty) return "duty";
    if (m == vesc::TeleopMode::kErpm) return "erpm";
    return "speed";
}

std::string drive_state_name(const vesc::TeleopCore& core) {
    if (core.is_aborted()) return "ABORTED";
    if (core.is_braking()) return "BRAKING(" + core.stop_reason() + ")";
    if (core.is_driving()) return (core.direction() == vesc::DriveDirection::kForward) ? "FWD" : "REV";
    return "IDLE";
}

std::string format_status_line(const vesc::TeleopCore& core, double now_s, bool have_values,
                                const vesc::VescValues& v, bool show_steering, double calib_center) {
    char buf[512];
    int n;
    if (core.mode() == vesc::TeleopMode::kSpeed) {
        // Speed mode's own dedicated line (see TeleopCore.h's "SPEED
        // GOVERNOR MODE"): TGT is the target erpm, meas is the filtered
        // measured erpm feeding the governor, duty_out is what's
        // actually on the wire (SET_DUTY only, never SET_RPM).
        n = std::snprintf(buf, sizeof(buf), "MODE=speed TGT=%.0f meas=%.0f duty_out=%.4f DRIVE=%s (gov duty_ramp=%.4f/s)",
                           core.target_value(), core.filtered_erpm(), core.emitted_value(),
                           drive_state_name(core).c_str(), core.duty_ramp_rate());
    } else {
        n = std::snprintf(buf, sizeof(buf), "MODE=%s SET=%.4f DRIVE=%s", mode_name(core.mode()).c_str(),
                           core.magnitude(), drive_state_name(core).c_str());
        if (core.ramp_enabled()) {
            n += std::snprintf(buf + n, sizeof(buf) - n, " RAMP=%g/s", core.ramp_rate());
        } else {
            n += std::snprintf(buf + n, sizeof(buf) - n, " RAMP=off");
        }
        // Only shown while actively slewing (ramp on, driving, not yet at
        // target) -- once emitted_value() reaches target_value() this stops
        // appearing, same info the DRIVE=FWD/REV value already conveys.
        if (core.ramp_enabled() && core.is_driving() && core.emitted_value() != core.target_value()) {
            n += std::snprintf(buf + n, sizeof(buf) - n, " cmd=%g->%g", core.emitted_value(), core.target_value());
        }
    }
    if (!core.digit_buffer().empty()) {
        n += std::snprintf(buf + n, sizeof(buf) - n, " entry%s=%s",
                            core.digit_entry_is_ramp_rate() ? "(rate)" : "", core.digit_buffer().c_str());
    }
    if (have_values) {
        std::string fault_str = "none";
        if (v.fault != 0) {
            const std::string name = vesc::fault_name(v.fault);
            fault_str = name.empty() ? std::to_string(static_cast<int>(v.fault)) : name;
        }
        n += std::snprintf(buf + n, sizeof(buf) - n, " | erpm=%ld duty=%.3f I=%.2fA Vin=%.1fV Tfet=%.1fC fault=%s",
                            static_cast<long>(v.erpm), v.duty, v.current_motor, v.v_in, v.temp_fet,
                            fault_str.c_str());
    } else {
        n += std::snprintf(buf + n, sizeof(buf) - n, " | (no telemetry yet)");
    }
    if (core.is_driving()) {
        n += std::snprintf(buf + n, sizeof(buf) - n, " deadman=%.1fs", core.deadman_remaining_s(now_s));
    }
    if (core.is_aborted()) {
        n += std::snprintf(buf + n, sizeof(buf) - n, " || ABORT: %s -- press 'c' to clear", core.abort_reason().c_str());
    }
    if (show_steering) {
        n += std::snprintf(buf + n, sizeof(buf) - n, " STEER=%.3f (center %.3f)", core.steering_position(), calib_center);
        if (core.in_trim_mode()) {
            n += std::snprintf(buf + n, sizeof(buf) - n, " TRIM");
        }
    }
    return std::string(buf);
}

}  // namespace

int main(int argc, char** argv) {
    // --help/-h: checked BEFORE parse_cli()/any port discovery, so this
    // works with zero hardware attached -- scans raw argv directly
    // (deliberately not folded into parse_cli()'s own loop, so it can
    // short-circuit main() immediately regardless of what other flags
    // are also present). Prints to stdout and exits 0, per the standing
    // convention that --help is a successful, informational invocation,
    // not a usage error.
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_cli_flags_help();
            print_help(CliOptions());  // key map + SAFETY/SPEED MODE text, using the tool's own defaults.
            return 0;
        }
    }

    const CliOptions cli = parse_cli(argc, argv);

    std::printf("vesc_teleop: opening the VESC (serial-port=%s baud=%d)...\n",
                cli.serial_port.empty() ? "<auto-discover>" : cli.serial_port.c_str(), cli.baud);
    std::fflush(stdout);

    const vesc::DiscoveryResult disc = vesc::find_vesc_port(cli.serial_port, 400, cli.baud);
    if (!disc.ok) {
        std::fprintf(stderr, "vesc_teleop: could not find/handshake a VESC. Candidates tried:\n");
        for (const auto& e : disc.log) {
            std::fprintf(stderr, "  path=%-24s source=%-24s responded=%-3s note=%s\n", e.path.c_str(),
                         e.source.c_str(), e.responded ? "yes" : "no", e.note.c_str());
        }
        return 1;
    }

    vesc::SerialPort port;
    if (!port.open(disc.path, cli.baud)) {
        std::fprintf(stderr, "vesc_teleop: found a VESC at '%s' but could not (re)open it: %s\n", disc.path.c_str(),
                     port.last_error().c_str());
        return 1;
    }

    // Firmware version banner -- printed PROMINENTLY, per this tool's
    // "first cautious contact with unknown hardware" job: GET_VALUES'
    // layout (VescProtocol.h) was only cross-checked against FW 5.x/6.x
    // documentation and (separately) a real FW 2.18 unit, never against
    // any other unknown unit -- warn loudly unless it is exactly a 6.x,
    // note (without alarm) when the legacy FW 2.x layout is in use, and
    // tell the operator to sanity-check v_in against the snapshot
    // printed below either way.
    std::printf(
        "\n"
        "================================================================\n"
        " Connected: %s\n"
        " Firmware:  %d.%d\n"
        "================================================================\n",
        disc.path.c_str(), static_cast<int>(disc.fw_major), static_cast<int>(disc.fw_minor));
    if (disc.fw_major == 2) {
        std::printf(
            " NOTE: FW 2.x legacy telemetry layout active (temp_mos1..6 + temp_pcb,\n"
            " no motor temp sensor -- see VescProtocol.h's parse_get_values_legacy()).\n"
            " Sanity-check v_in below against the VESC's actual supply voltage.\n"
            "================================================================\n");
    } else if (disc.fw_major != 6) {
        std::printf(
            " WARNING: GET_VALUES layout verified only for FW 5.x/6.x -- telemetry\n"
            " values may be misparsed on this firmware. Sanity-check v_in below\n"
            " against the VESC's actual supply voltage before trusting anything else.\n"
            "================================================================\n");
    }
    std::fflush(stdout);

    // Steering calibration: load + resolve servo_cmd_id, printed BEFORE
    // print_help() below so the operator sees exactly what position this
    // session's 'k'/'R'/'a'/'d' will use, and what raw command id
    // SET_SERVO_POS will be encoded as, before any keys are accepted. See
    // SteeringCalib.h for load()/save()/resolve_default_*_path()'s own
    // precedence rules, and VescProtocol.h's resolve_servo_cmd_id() for
    // the FW2.x COMM_SET_HANDBRAKE-shift rationale (mirrors
    // vesc_driver_main.cpp's own wiring of the same resolution).
    vesc::SteeringCalib calib;
    std::string steering_load_path;
    std::string steering_save_path;
    uint8_t servo_cmd_id = 0;
    if (cli.no_steering) {
        std::printf("vesc_teleop: steering disabled (--no-steering) -- no calibration loaded, no servo id resolved.\n");
    } else {
        steering_load_path = vesc::resolve_default_load_path(cli.steering_calib_path);
        std::string calib_err;
        vesc::load(steering_load_path, &calib, &calib_err);
        if (!calib_err.empty()) {
            std::printf("vesc_teleop: steering calibration: %s\n", calib_err.c_str());
        }
        if (cli.has_servo_min) calib.min_pos = cli.servo_min;
        if (cli.has_servo_max) calib.max_pos = cli.servo_max;
        steering_save_path = vesc::resolve_default_save_path(cli.steering_calib_path);

        servo_cmd_id = cli.has_servo_cmd_id ? cli.servo_cmd_id : vesc::resolve_servo_cmd_id(disc.fw_major);
        std::printf(
            "vesc_teleop: steering: load=%s save=%s center=%.3f min=%.3f max=%.3f invert=%s servo_cmd_id=%d\n",
            steering_load_path.c_str(), steering_save_path.c_str(), calib.center, calib.min_pos, calib.max_pos,
            calib.invert ? "true" : "false", static_cast<int>(servo_cmd_id));
        if (disc.fw_major <= 2 && !cli.has_servo_cmd_id) {
            std::printf(
                "vesc_teleop: WARNING: servo_cmd_id=%d is derived from firmware-version history "
                "(FW 2.x lacks COMM_SET_HANDBRAKE), not confirmed against this exact unit -- "
                "verify visually with small servo movements before trusting it.\n",
                static_cast<int>(servo_cmd_id));
        }
    }
    // drive_invert (see SteeringCalib.h / TeleopConfig::drive_invert): a
    // WHOLE-VEHICLE property, not a steering one, so the CLI override
    // applies regardless of --no-steering (calib stays default-
    // constructed, drive_invert=false, in that case unless overridden
    // here). Printed unconditionally so the operator always sees the
    // resolved value before any keys are accepted.
    if (cli.has_drive_invert) calib.drive_invert = cli.drive_invert;
    std::printf("vesc_teleop: drive_invert=%s\n", calib.drive_invert ? "true" : "false");
    std::fflush(stdout);

    // One GET_VALUES snapshot BEFORE accepting any keys -- lets the
    // operator sanity-check the telemetry layout on the new hardware
    // before commanding it to do anything.
    vesc::FrameDecoder decoder;
    vesc::VescValues startup_values;
    const bool got_startup_values = blocking_get_values(&port, &decoder, disc.fw_major, &startup_values, 1.5);
    if (got_startup_values) {
        char temp_motor_field[32];
        if (startup_values.has_temp_motor) {
            std::snprintf(temp_motor_field, sizeof(temp_motor_field), "%.1fC", startup_values.temp_motor);
        } else {
            std::snprintf(temp_motor_field, sizeof(temp_motor_field), "n/a");
        }
        std::printf(
            "Startup GET_VALUES snapshot: v_in=%.2fV temp_fet=%.1fC temp_motor=%s erpm=%ld duty=%.3f fault=%d\n",
            startup_values.v_in, startup_values.temp_fet, temp_motor_field,
            static_cast<long>(startup_values.erpm), startup_values.duty, static_cast<int>(startup_values.fault));
    } else {
        std::printf("Startup GET_VALUES snapshot: no reply within 1.5s (continuing anyway).\n");
    }
    std::fflush(stdout);

    print_help(cli);

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    StdinNonBlockGuard stdin_nonblock_guard;
    RawTerminalGuard terminal_guard;

    vesc::TeleopCore teleop(teleop_config_from_cli(cli, calib));

    const auto proc_start = std::chrono::steady_clock::now();
    auto now_seconds = [&proc_start]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - proc_start).count();
    };

    bool have_values = false;
    vesc::VescValues last_values;

    bool was_aborted = false;
    size_t last_line_len = 0;
    double last_pipe_print_s = -1.0e9;
    double last_servo_emit_s = -1.0e9;
    // Tracks the currently-persisted center (mirrors `calib.center`'s last
    // successfully-saved value) -- threaded into format_status_line()'s
    // "(center %.3f)" readout and used as the shutdown parting position.
    double current_calib_center = calib.center;

    const double period_s = 1.0 / std::max(1.0, cli.rate_hz);
    auto next_tick = std::chrono::steady_clock::now();

    bool quit_requested = false;
    while (!g_stop && !quit_requested) {
        const double now_s = now_seconds();

        std::vector<char> keys;
        read_available_stdin_bytes(&keys);
        for (char k : keys) {
            const vesc::KeyEvent ev = teleop.handle_key(k, now_s);
            if (!ev.retired_hint.empty()) {
                std::printf("\n%s\n", ev.retired_hint.c_str());
                std::fflush(stdout);
                last_line_len = 0;  // status line below is about to redraw from column 0.
            }
            if (ev.type == vesc::KeyEventType::kQuit) quit_requested = true;
        }

        // Drain the serial link; feed any fresh GET_VALUES reply to the
        // safety watch and the status line.
        {
            std::vector<uint8_t> buf;
            if (port.read_available(&buf) > 0) decoder.feed(buf);
            std::vector<uint8_t> payload;
            while (decoder.pop_payload(&payload)) {
                if (!payload.empty() && payload[0] == static_cast<uint8_t>(vesc::CommandId::GET_VALUES)) {
                    const vesc::VescValues v = vesc::parse_get_values_for_fw(payload, disc.fw_major);
                    if (v.ok) {
                        last_values = v;
                        have_values = true;
                        teleop.feed_telemetry(v, now_s);
                    }
                }
            }
        }

        const vesc::TeleopMotorAction action = teleop.step(now_s);
        send_motor_action(&port, action, disc.fw_major == 2);
        port.write_all(vesc::encode_frame(vesc::build_get_values()));

        // Steering: emit SET_SERVO_POS on change (or periodic refresh),
        // and handle a pending 'W' center-save request -- both entirely
        // skipped when --no-steering is set. NEVER sent before
        // steering_ever_touched() is true (see TeleopCore.h's "NO
        // PREMATURE EMISSION") -- no startup servo jump.
        if (!cli.no_steering) {
            if (teleop.steering_ever_touched()) {
                const bool refresh_due =
                    cli.servo_refresh_ms > 0.0 && (now_s - last_servo_emit_s) * 1000.0 >= cli.servo_refresh_ms;
                if (teleop.steering_changed_since_emit() || refresh_due) {
                    port.write_all(vesc::encode_frame(vesc::build_set_servo_pos(teleop.steering_position(), servo_cmd_id)));
                    teleop.mark_steering_emitted();
                    last_servo_emit_s = now_s;
                }
            }

            if (teleop.has_pending_center_save()) {
                const double new_center = teleop.pending_center_save_value();
                const double old_center = current_calib_center;
                calib.center = new_center;
                std::string save_err;
                const bool saved = vesc::save(steering_save_path, calib, &save_err);
                // Loud, multi-line, impossible-to-miss-scrolling-by banner --
                // matches the ABORT banner's visual weight below, printed
                // once per 'W' (not every tick).
                std::printf(
                    "\n\n"
                    "================================================================\n"
                    " STEERING CENTER %s: %.3f -> %.3f\n"
                    " file: %s\n"
                    " %s\n"
                    "================================================================\n\n",
                    saved ? "SAVED" : "SAVE FAILED", old_center, new_center, steering_save_path.c_str(),
                    saved ? "OK" : save_err.c_str());
                std::fflush(stdout);
                last_line_len = 0;
                if (saved) {
                    current_calib_center = new_center;
                    teleop.set_steer_center(new_center);
                } else {
                    // Roll back: don't leave `calib` holding an unpersisted
                    // center that a LATER successful save might otherwise
                    // silently carry forward as if THIS one had succeeded.
                    calib.center = old_center;
                }
                teleop.consume_pending_center_save();
            }
        }

        // Abort banner: printed once, on the transition into kAborted
        // (not every tick -- the carriage-return status line below
        // already reflects the abort continuously).
        if (teleop.is_aborted() && !was_aborted) {
            std::printf("\n\n!!! ABORT !!! %s\nPress 'c' to clear before driving again.\n\n",
                        teleop.abort_reason().c_str());
            std::fflush(stdout);
            last_line_len = 0;
        }
        was_aborted = teleop.is_aborted();

        const std::string line =
            format_status_line(teleop, now_s, have_values, last_values, !cli.no_steering, current_calib_center);
        if (terminal_guard.is_tty()) {
            std::string padded = line;
            if (padded.size() < last_line_len) padded += std::string(last_line_len - padded.size(), ' ');
            last_line_len = line.size();
            std::printf("\r%s", padded.c_str());
            std::fflush(stdout);
        } else if (now_s - last_pipe_print_s >= 0.5) {
            std::printf("%s\n", line.c_str());
            std::fflush(stdout);
            last_pipe_print_s = now_s;
        }

        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(period_s));
        const auto now_tp = std::chrono::steady_clock::now();
        if (next_tick < now_tp - std::chrono::milliseconds(200)) {
            next_tick = now_tp;  // fell far behind -- resync rather than spinning to catch up forever.
        }
        std::this_thread::sleep_until(next_tick);
    }

    if (terminal_guard.is_tty()) std::printf("\n");
    std::printf("vesc_teleop: shutting down (brake burst)...\n");
    std::fflush(stdout);

    vesc::TeleopMotorAction shutdown_brake;
    shutdown_brake.type = vesc::TeleopMotorAction::Type::kBrake;
    shutdown_brake.value = teleop.config().brake_amps;
    for (int i = 0; i < 5; ++i) {
        send_motor_action(&port, shutdown_brake, disc.fw_major == 2);  // kBrake ignores fw_is_v2 either way.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Parting steering action: deliberately snaps to the CALIBRATED
    // CENTER (current_calib_center), NOT wherever steering_position()
    // happens to be left -- a "leave the servo at a known-safe position"
    // safety action distinct from mid-session emission (which tracks the
    // live position), mirroring vesc_driver_main.cpp's own shutdown
    // behavior (which sends config.servo.center, not the last commanded
    // position).
    if (!cli.no_steering && teleop.steering_ever_touched()) {
        port.write_all(vesc::encode_frame(vesc::build_set_servo_pos(current_calib_center, servo_cmd_id)));
    }

    return 0;
}
