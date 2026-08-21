// robot_calibration.cpp -> binary `robot_calibration`
//
// Headless CLI for mpc::CalibrationCore (see MPC/include/mpc/CalibrationCore.h):
// fits the velocity/steering calibration maps from an already-recorded
// teleop session directory and exports the FROZEN OUTPUT SCHEMAS the
// robot-side driver loads (velocity_calib.json / steering_angle_map.json).
//
// This tool does NOT ingest live streams itself (no ZMQ/sockets here at
// all -- a later Qt tab does the live feed_pose()/feed_telemetry()/
// feed_command() wiring, mirroring how CalibClient.h wires
// MotorCalibCore's TrialRunner to real sockets). It only reads the session
// CSVs a live session already wrote, matching the workflow of running one
// teleop capture session per task (samples_velocity.csv from a "drive
// straight at various speeds" session, samples_steering.csv from a "drive
// arcs at various servo notches" session -- see
// MPC/include/mpc/CalibrationCore.h's file header comment for why
// CalibrationCore itself is agnostic to this file-per-task convention;
// it's purely a caller-side choice this CLI happens to also make).
//
// Output: one JSONL event per line on stdout ({"ev":"fit_start"|"fit_done"|
// "export"|"error", ...}) for machine consumption, human-readable progress
// on stderr. Exits nonzero if any requested task's fit or export fails.

#include "mpc/CalibrationCore.h"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

struct CliOptions {
    std::string fit_from;
    std::string task = "both";  // "velocity" | "steering" | "both"
    double wheel_base = 0.29;
    std::string robot_name;
    std::string out_dir;  // defaults to fit_from if empty
};

bool next_value(int argc, char** argv, int* i, std::string* out) {
    const std::string arg = argv[*i];
    const std::size_t eq = arg.find('=');
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

void print_usage() {
    std::cerr << "Usage: robot_calibration --fit-from <session_dir> [options]\n"
                 "  --fit-from <dir>       Session directory containing samples_velocity.csv/\n"
                 "                         samples_steering.csv (required)\n"
                 "  --task <t>             velocity|steering|both (default both)\n"
                 "  --wheel-base <m>       Bicycle-model wheel base for the steering fit (default 0.29)\n"
                 "  --robot-name <name>    robot_name field written into the exported JSON\n"
                 "  --out <dir>            Output directory for the exported JSON files\n"
                 "                         (default: same as --fit-from)\n"
                 "  --help, -h             Show this help\n";
}

bool parse_cli(int argc, char** argv, CliOptions* opt, std::string* error) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string val;
        if (arg == "--help" || arg == "-h") {
            print_usage();
            std::exit(0);
        } else if (flag_matches(arg, "--fit-from")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--fit-from needs a value"; return false; }
            opt->fit_from = val;
        } else if (flag_matches(arg, "--task")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--task needs a value"; return false; }
            opt->task = val;
        } else if (flag_matches(arg, "--wheel-base")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--wheel-base needs a value"; return false; }
            opt->wheel_base = std::stod(val);
        } else if (flag_matches(arg, "--robot-name")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--robot-name needs a value"; return false; }
            opt->robot_name = val;
        } else if (flag_matches(arg, "--out")) {
            if (!next_value(argc, argv, &i, &val)) { *error = "--out needs a value"; return false; }
            opt->out_dir = val;
        } else {
            *error = "unknown argument: " + arg;
            return false;
        }
    }
    if (opt->fit_from.empty()) {
        *error = "--fit-from is required";
        return false;
    }
    if (opt->task != "velocity" && opt->task != "steering" && opt->task != "both") {
        *error = "--task must be velocity|steering|both, got '" + opt->task + "'";
        return false;
    }
    if (opt->out_dir.empty()) opt->out_dir = opt->fit_from;
    return true;
}

void emit(const nlohmann::json& ev) { std::cout << ev.dump() << std::endl; }

// Reads `path`'s ACCEPTED rows' servo_cmd values -- used only to infer the
// steering notch ladder (CalibrationCore itself needs a notch config
// BEFORE ingesting a session, since notch bin membership is decided at
// ingest time -- see CalibrationCore::ingest_one()).
std::vector<double> collect_accepted_servo_cmds(const std::vector<mpc::DerivedSample>& rows) {
    std::vector<double> out;
    for (const auto& r : rows) {
        if (r.accepted) out.push_back(r.servo_cmd);
    }
    return out;
}

// Runs one task's fit + export against `csv_path`. Returns true on full
// success (fit ok AND export ok); emits fit_start/fit_done|error/export
// JSONL events as it goes.
bool run_velocity_task(const CliOptions& opt) {
    const std::string csv_path = (std::filesystem::path(opt.fit_from) / "samples_velocity.csv").string();
    std::cerr << "[robot_calibration] velocity: loading " << csv_path << "\n";

    std::vector<mpc::DerivedSample> rows;
    std::string err;
    if (!mpc::load_derived_samples_csv(csv_path, &rows, &err)) {
        emit({{"ev", "error"}, {"task", "velocity"}, {"message", err}});
        std::cerr << "[robot_calibration] velocity: " << err << "\n";
        return false;
    }

    mpc::CalibrationConfig cfg;
    cfg.robot_name = opt.robot_name;
    cfg.wheel_base = opt.wheel_base;
    mpc::CalibrationCore core(cfg);
    core.ingest_derived_samples(rows);

    emit({{"ev", "fit_start"}, {"task", "velocity"}, {"session_dir", opt.fit_from},
          {"n_samples", static_cast<int>(rows.size())}});

    const mpc::VelocityFitResult fit = core.fit_velocity_map();
    if (!fit.ok) {
        emit({{"ev", "error"}, {"task", "velocity"}, {"message", fit.error}});
        std::cerr << "[robot_calibration] velocity: fit failed: " << fit.error << "\n";
        return false;
    }
    emit({{"ev", "fit_done"},
          {"task", "velocity"},
          {"n_table_points", static_cast<int>(fit.table.size())},
          {"min_reliable_erpm", fit.min_reliable_erpm},
          {"erpm_per_mps", fit.erpm_per_mps},
          {"offset_erpm", fit.offset_erpm},
          {"linear_rms", fit.rms},
          {"bins_filled", fit.bins_filled},
          {"bins_total", fit.bins_total}});
    std::cerr << "[robot_calibration] velocity: fit ok, " << fit.table.size() << " table points, "
              << "min_reliable_erpm=" << fit.min_reliable_erpm << "\n";

    const std::string out_path = (std::filesystem::path(opt.out_dir) / "velocity_calib.json").string();
    std::string export_err;
    if (!mpc::export_velocity_calib(fit, opt.robot_name, out_path, &export_err)) {
        emit({{"ev", "error"}, {"task", "velocity"}, {"message", export_err}});
        std::cerr << "[robot_calibration] velocity: export failed: " << export_err << "\n";
        return false;
    }
    emit({{"ev", "export"}, {"task", "velocity"}, {"path", out_path}});
    std::cerr << "[robot_calibration] velocity: wrote " << out_path << "\n";
    return true;
}

bool run_steering_task(const CliOptions& opt) {
    const std::string csv_path = (std::filesystem::path(opt.fit_from) / "samples_steering.csv").string();
    std::cerr << "[robot_calibration] steering: loading " << csv_path << "\n";

    std::vector<mpc::DerivedSample> rows;
    std::string err;
    if (!mpc::load_derived_samples_csv(csv_path, &rows, &err)) {
        emit({{"ev", "error"}, {"task", "steering"}, {"message", err}});
        std::cerr << "[robot_calibration] steering: " << err << "\n";
        return false;
    }

    const std::vector<double> notches = mpc::infer_notches(collect_accepted_servo_cmds(rows), 0.005);
    std::cerr << "[robot_calibration] steering: inferred " << notches.size() << " notches from "
              << csv_path << "\n";

    mpc::CalibrationConfig cfg;
    cfg.robot_name = opt.robot_name;
    cfg.wheel_base = opt.wheel_base;
    cfg.steering.notches = notches;
    mpc::CalibrationCore core(cfg);
    core.ingest_derived_samples(rows);

    emit({{"ev", "fit_start"}, {"task", "steering"}, {"session_dir", opt.fit_from},
          {"n_samples", static_cast<int>(rows.size())}, {"n_notches", static_cast<int>(notches.size())}});

    const mpc::SteeringFitResult fit = core.fit_steering_map(opt.wheel_base);
    if (!fit.ok) {
        emit({{"ev", "error"}, {"task", "steering"}, {"message", fit.error}});
        std::cerr << "[robot_calibration] steering: fit failed: " << fit.error << "\n";
        return false;
    }
    emit({{"ev", "fit_done"},
          {"task", "steering"},
          {"n_points", static_cast<int>(fit.points.size())},
          {"delta_min", fit.delta_min},
          {"delta_max", fit.delta_max},
          {"residual_rms", fit.residual_rms}});
    std::cerr << "[robot_calibration] steering: fit ok, " << fit.points.size() << " table points, "
              << "residual_rms=" << fit.residual_rms << "\n";

    const std::string out_path = (std::filesystem::path(opt.out_dir) / "steering_angle_map.json").string();
    std::string export_err;
    if (!mpc::export_steering_map(fit, opt.robot_name, opt.wheel_base, out_path, &export_err)) {
        emit({{"ev", "error"}, {"task", "steering"}, {"message", export_err}});
        std::cerr << "[robot_calibration] steering: export failed: " << export_err << "\n";
        return false;
    }
    emit({{"ev", "export"}, {"task", "steering"}, {"path", out_path}});
    std::cerr << "[robot_calibration] steering: wrote " << out_path << "\n";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    CliOptions opt;
    std::string parse_error;
    if (!parse_cli(argc, argv, &opt, &parse_error)) {
        std::cerr << "[robot_calibration] " << parse_error << "\n";
        print_usage();
        return 1;
    }

    std::filesystem::create_directories(opt.out_dir);

    bool all_ok = true;
    if (opt.task == "velocity" || opt.task == "both") {
        all_ok = run_velocity_task(opt) && all_ok;
    }
    if (opt.task == "steering" || opt.task == "both") {
        all_ok = run_steering_task(opt) && all_ok;
    }

    return all_ok ? 0 : 1;
}
