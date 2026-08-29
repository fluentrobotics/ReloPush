// pose_filter_replay: offline tool that runs the SAME mpc::PoseFilter core
// the bridge uses over a previously-recorded optitrack_zmq_bridge --log-csv
// file, per rigid body, printing every rejection/reinit with its timestamp
// and d^2 values plus a run summary. This is how the gate thresholds get
// tuned/validated against real mocap data before --pose-filter is ever
// turned on live, and how a real captured jump becomes a regression fixture
// (see doc/MOCAP_POSE_FILTER_PLAN.md's "Deliverables" #3/#4).
//
// Pure CSV-in/stdout-out: no ZMQ, no sockets, no NatNet. Reads the CSV
// columns by NAME (t_arrival, robot, planar_x, planar_y, planar_yaw, and
// tracking_valid if present) so it tolerates the bridge's own --log-csv
// header growing new columns over time (this tool always appends new
// columns at the end too -- see optitrack_zmq_bridge.cpp's --log-csv doc
// comment).

#include "mpc/PoseFilter.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct CsvRow {
    double t_arrival = 0.0;
    std::string robot;
    double planar_x = 0.0, planar_y = 0.0, planar_yaw = 0.0;
    bool tracking_valid = true;
};

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) out.push_back(tok);
    // std::getline drops a trailing empty field after a final comma; the
    // bridge's own writer never emits one (every row ends with a real
    // value), so this is not a concern here.
    return out;
}

// Reads the whole CSV, keyed by column name so future --log-csv columns
// (appended at the end, per the bridge's own convention) never break this
// tool. Rows with tracking_valid==0 (if that column exists) are skipped
// entirely -- matches the spec's "skip rows with tracking_valid==0".
std::vector<CsvRow> read_csv(const std::string& path, bool* ok, std::string* error) {
    *ok = true;  // only ever set false below, on an actual failure.
    std::vector<CsvRow> rows;
    std::ifstream f(path);
    if (!f.is_open()) {
        *ok = false;
        *error = "could not open --csv '" + path + "'";
        return rows;
    }
    std::string header_line;
    if (!std::getline(f, header_line)) {
        *ok = false;
        *error = "--csv file is empty";
        return rows;
    }
    std::vector<std::string> header = split_csv_line(header_line);
    std::unordered_map<std::string, int> col;
    for (size_t i = 0; i < header.size(); ++i) col[header[i]] = static_cast<int>(i);

    auto require_col = [&](const std::string& name) -> int {
        auto it = col.find(name);
        if (it == col.end()) {
            *ok = false;
            *error = "--csv is missing required column '" + name + "'";
            return -1;
        }
        return it->second;
    };
    const int i_t = require_col("t_arrival");
    const int i_robot = require_col("robot");
    const int i_x = require_col("planar_x");
    const int i_y = require_col("planar_y");
    const int i_yaw = require_col("planar_yaw");
    if (!*ok) return rows;
    const auto tv_it = col.find("tracking_valid");
    const int i_valid = (tv_it != col.end()) ? tv_it->second : -1;

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::vector<std::string> fields = split_csv_line(line);
        const int needed = 1 + std::max({i_t, i_robot, i_x, i_y, i_yaw, i_valid});
        if (static_cast<int>(fields.size()) < needed) continue;  // malformed/truncated row -- skip.
        CsvRow row;
        try {
            row.t_arrival = std::stod(fields[i_t]);
            row.robot = fields[i_robot];
            row.planar_x = std::stod(fields[i_x]);
            row.planar_y = std::stod(fields[i_y]);
            row.planar_yaw = std::stod(fields[i_yaw]);
            row.tracking_valid = (i_valid < 0) || (std::stoi(fields[i_valid]) != 0);
        } catch (const std::exception&) {
            continue;  // malformed numeric field -- skip this row, keep going.
        }
        if (!row.tracking_valid) continue;
        rows.push_back(row);
    }
    *ok = true;
    return rows;
}

// Mirrors optitrack_zmq_bridge.cpp's own "pose_filter" JSON block reduction
// (see load_map_config() there) -- kept independently here rather than
// shared, matching this codebase's precedent of each binary owning its own
// config-loading (OptiTrackCore.h/.cpp stays JSON-free; see its file header
// comment) rather than introducing a new cross-module JSON dependency.
mpc::PoseFilterConfig load_pose_filter_config(const std::string& map_config_path, bool* found) {
    mpc::PoseFilterConfig cfg;
    *found = false;
    if (map_config_path.empty()) return cfg;
    std::ifstream f(map_config_path);
    if (!f.is_open()) {
        std::cerr << "[pose_filter_replay] Warning: could not open --map-config '"
                  << map_config_path << "', using defaults" << std::endl;
        return cfg;
    }
    try {
        nlohmann::json j;
        f >> j;
        if (!j.contains("pose_filter") || !j.at("pose_filter").is_object()) return cfg;
        const nlohmann::json& pf = j.at("pose_filter");
        *found = true;
        if (pf.contains("enabled")) cfg.enabled = pf.at("enabled").get<bool>();
        if (pf.contains("gate_chi2_pos")) cfg.gate_chi2_pos = pf.at("gate_chi2_pos").get<double>();
        if (pf.contains("gate_chi2_yaw")) cfg.gate_chi2_yaw = pf.at("gate_chi2_yaw").get<double>();
        if (pf.contains("gate_chi2_all")) cfg.gate_chi2_all = pf.at("gate_chi2_all").get<double>();
        if (pf.contains("max_speed")) cfg.max_speed = pf.at("max_speed").get<double>();
        if (pf.contains("max_yaw_rate")) cfg.max_yaw_rate = pf.at("max_yaw_rate").get<double>();
        if (pf.contains("reinit_after_s")) cfg.reinit_after_s = pf.at("reinit_after_s").get<double>();
        if (pf.contains("gap_reinit_s")) cfg.gap_reinit_s = pf.at("gap_reinit_s").get<double>();
        if (pf.contains("r_pos")) cfg.r_pos = pf.at("r_pos").get<double>();
        if (pf.contains("r_yaw")) cfg.r_yaw = pf.at("r_yaw").get<double>();
        if (pf.contains("q_acc")) cfg.q_acc = pf.at("q_acc").get<double>();
        if (pf.contains("q_yaw_acc")) cfg.q_yaw_acc = pf.at("q_yaw_acc").get<double>();
    } catch (const std::exception& ex) {
        std::cerr << "[pose_filter_replay] Warning: failed to parse --map-config ('" << ex.what()
                  << "'), using defaults" << std::endl;
    }
    return cfg;
}

struct InjectedSpike {
    bool set = false;
    double t = 0.0, dx = 0.0, dy = 0.0, dyaw = 0.0;
};

InjectedSpike parse_inject_spike(const std::string& arg) {
    InjectedSpike sp;
    std::stringstream ss(arg);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
        try {
            vals.push_back(std::stod(tok));
        } catch (const std::exception&) {
            std::cerr << "[pose_filter_replay] Warning: --inject-spike expects t,dx,dy,dyaw; "
                         "ignoring malformed value '"
                      << tok << "'" << std::endl;
            return sp;
        }
    }
    if (vals.size() != 4) {
        std::cerr << "[pose_filter_replay] Warning: --inject-spike expects exactly 4 comma-"
                     "separated values (t,dx,dy,dyaw), got "
                  << vals.size() << "; ignoring" << std::endl;
        return sp;
    }
    sp.set = true;
    sp.t = vals[0];
    sp.dx = vals[1];
    sp.dy = vals[2];
    sp.dyaw = vals[3];
    return sp;
}

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " --csv <bridge --log-csv file> [options]\n\n"
           "Replays optitrack_zmq_bridge's mpc::PoseFilter over a recorded CSV log (per body),\n"
           "printing every rejection/reinit and a summary. See doc/MOCAP_POSE_FILTER_PLAN.md.\n\n"
           "  --csv <path>              REQUIRED. A file produced by 'optitrack_zmq_bridge\n"
           "                            --log-csv <path>' (needs at least t_arrival,robot,\n"
           "                            planar_x,planar_y,planar_yaw; tracking_valid==0 rows are\n"
           "                            skipped if that column is present).\n"
           "  --map-config <path>       Load the base PoseFilterConfig from this JSON's\n"
           "                            \"pose_filter\" block (same schema/keys as\n"
           "                            MPC/config/mocap_map_config.json). Any --gate-*/--max-*/\n"
           "                            etc. flag below overrides the loaded value.\n"
           "  --body <name>             Only replay this one 'robot' column value (default: all\n"
           "                            bodies present in the CSV, each with its own filter\n"
           "                            instance and its own summary).\n"
           "  --gate-pos <chi2>         override gate_chi2_pos\n"
           "  --gate-yaw <chi2>         override gate_chi2_yaw\n"
           "  --gate-all <chi2>         override gate_chi2_all\n"
           "  --max-speed <m/s>         override max_speed\n"
           "  --max-yaw-rate <rad/s>    override max_yaw_rate\n"
           "  --reinit-after-s <s>      override reinit_after_s\n"
           "  --gap-reinit-s <s>        override gap_reinit_s\n"
           "  --r-pos <m>               override r_pos\n"
           "  --r-yaw <rad>             override r_yaw\n"
           "  --q-acc <m/s^2>           override q_acc\n"
           "  --q-yaw-acc <rad/s^2>     override q_yaw_acc\n"
           "  --inject-spike t,dx,dy,dyaw   For each replayed body, finds the CSV row nearest to\n"
           "                            time t and adds (dx,dy,dyaw) to its measurement before\n"
           "                            filtering -- proves the gate still catches a synthetic\n"
           "                            spike injected into an otherwise-clean recording.\n"
           "  --help, -h                print this message and exit\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string csv_path, map_config_path, body_filter;
    mpc::PoseFilterConfig cli_overrides;  // only fields actually touched below are meaningful.
    bool set_gate_pos = false, set_gate_yaw = false, set_gate_all = false, set_max_speed = false,
         set_max_yaw_rate = false, set_reinit_after = false, set_gap_reinit = false,
         set_r_pos = false, set_r_yaw = false, set_q_acc = false, set_q_yaw_acc = false;
    InjectedSpike spike;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (arg == "--csv") {
            csv_path = next();
        } else if (arg == "--map-config") {
            map_config_path = next();
        } else if (arg == "--body") {
            body_filter = next();
        } else if (arg == "--gate-pos") {
            cli_overrides.gate_chi2_pos = std::stod(next());
            set_gate_pos = true;
        } else if (arg == "--gate-yaw") {
            cli_overrides.gate_chi2_yaw = std::stod(next());
            set_gate_yaw = true;
        } else if (arg == "--gate-all") {
            cli_overrides.gate_chi2_all = std::stod(next());
            set_gate_all = true;
        } else if (arg == "--max-speed") {
            cli_overrides.max_speed = std::stod(next());
            set_max_speed = true;
        } else if (arg == "--max-yaw-rate") {
            cli_overrides.max_yaw_rate = std::stod(next());
            set_max_yaw_rate = true;
        } else if (arg == "--reinit-after-s") {
            cli_overrides.reinit_after_s = std::stod(next());
            set_reinit_after = true;
        } else if (arg == "--gap-reinit-s") {
            cli_overrides.gap_reinit_s = std::stod(next());
            set_gap_reinit = true;
        } else if (arg == "--r-pos") {
            cli_overrides.r_pos = std::stod(next());
            set_r_pos = true;
        } else if (arg == "--r-yaw") {
            cli_overrides.r_yaw = std::stod(next());
            set_r_yaw = true;
        } else if (arg == "--q-acc") {
            cli_overrides.q_acc = std::stod(next());
            set_q_acc = true;
        } else if (arg == "--q-yaw-acc") {
            cli_overrides.q_yaw_acc = std::stod(next());
            set_q_yaw_acc = true;
        } else if (arg == "--inject-spike") {
            spike = parse_inject_spike(next());
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "[pose_filter_replay] Warning: unrecognized argument '" << arg << "'"
                      << std::endl;
        }
    }

    if (csv_path.empty()) {
        std::cerr << "[pose_filter_replay] --csv is required\n\n";
        print_usage(argv[0]);
        return 1;
    }

    bool config_found = false;
    mpc::PoseFilterConfig cfg = load_pose_filter_config(map_config_path, &config_found);
    // Replaying always exercises the filter's real accept/reject logic --
    // "enabled" from a loaded config would otherwise silently turn this
    // tool into a no-op pass-through, which is never what a replay run
    // wants (the whole point is to see what the gate WOULD do).
    cfg.enabled = true;
    if (set_gate_pos) cfg.gate_chi2_pos = cli_overrides.gate_chi2_pos;
    if (set_gate_yaw) cfg.gate_chi2_yaw = cli_overrides.gate_chi2_yaw;
    if (set_gate_all) cfg.gate_chi2_all = cli_overrides.gate_chi2_all;
    if (set_max_speed) cfg.max_speed = cli_overrides.max_speed;
    if (set_max_yaw_rate) cfg.max_yaw_rate = cli_overrides.max_yaw_rate;
    if (set_reinit_after) cfg.reinit_after_s = cli_overrides.reinit_after_s;
    if (set_gap_reinit) cfg.gap_reinit_s = cli_overrides.gap_reinit_s;
    if (set_r_pos) cfg.r_pos = cli_overrides.r_pos;
    if (set_r_yaw) cfg.r_yaw = cli_overrides.r_yaw;
    if (set_q_acc) cfg.q_acc = cli_overrides.q_acc;
    if (set_q_yaw_acc) cfg.q_yaw_acc = cli_overrides.q_yaw_acc;

    std::cout << "[pose_filter_replay] csv=" << csv_path
              << (map_config_path.empty() ? "" : " map_config=" + map_config_path)
              << (config_found ? " (pose_filter block found)" : " (using default config)")
              << " gate_pos=" << cfg.gate_chi2_pos << " gate_yaw=" << cfg.gate_chi2_yaw
              << " gate_all=" << cfg.gate_chi2_all << " max_speed=" << cfg.max_speed
              << " max_yaw_rate=" << cfg.max_yaw_rate << " reinit_after_s=" << cfg.reinit_after_s
              << " gap_reinit_s=" << cfg.gap_reinit_s << " r_pos=" << cfg.r_pos
              << " r_yaw=" << cfg.r_yaw << " q_acc=" << cfg.q_acc << " q_yaw_acc=" << cfg.q_yaw_acc
              << std::endl;

    bool ok = false;
    std::string error;
    std::vector<CsvRow> all_rows = read_csv(csv_path, &ok, &error);
    if (!ok) {
        std::cerr << "[pose_filter_replay] Error: " << error << std::endl;
        return 1;
    }
    std::cout << "[pose_filter_replay] loaded " << all_rows.size() << " valid rows from " << csv_path
              << std::endl;

    // Group rows by body, preserving each body's own chronological
    // subsequence (the CSV is written in overall-arrival order, so
    // filtering to one robot automatically keeps that robot's own rows
    // time-ordered).
    std::map<std::string, std::vector<const CsvRow*>> by_body;
    for (const CsvRow& row : all_rows) {
        if (!body_filter.empty() && row.robot != body_filter) continue;
        by_body[row.robot].push_back(&row);
    }
    if (by_body.empty()) {
        std::cerr << "[pose_filter_replay] Error: no rows matched"
                  << (body_filter.empty() ? "" : " --body '" + body_filter + "'") << std::endl;
        return 1;
    }

    int total_rejects = 0;
    int total_reinits = 0;

    for (const auto& kv : by_body) {
        const std::string& body = kv.first;
        const std::vector<const CsvRow*>& rows = kv.second;

        // --inject-spike: find the row nearest to `spike.t` for THIS body.
        int spike_row_index = -1;
        if (spike.set) {
            double best_dt = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < rows.size(); ++i) {
                const double dt = std::fabs(rows[i]->t_arrival - spike.t);
                if (dt < best_dt) {
                    best_dt = dt;
                    spike_row_index = static_cast<int>(i);
                }
            }
        }

        mpc::PoseFilter filt(cfg);
        int rejects = 0, reinits = 0, ignored = 0;
        double max_d2_accepted = 0.0;
        double max_dpos_accepted = 0.0;
        double last_x = 0.0, last_y = 0.0;
        bool have_last_accepted = false;
        double last_t = 0.0;
        bool have_last_accepted_t = false;

        std::cout << "\n[pose_filter_replay] === body '" << body << "' (" << rows.size()
                  << " rows) ===" << std::endl;

        for (size_t i = 0; i < rows.size(); ++i) {
            double zx = rows[i]->planar_x;
            double zy = rows[i]->planar_y;
            double zyaw = rows[i]->planar_yaw;
            if (spike.set && static_cast<int>(i) == spike_row_index) {
                zx += spike.dx;
                zy += spike.dy;
                zyaw += spike.dyaw;
                std::cout << "[pose_filter_replay]   " << body << " t=" << rows[i]->t_arrival
                          << " INJECTED SPIKE (dx=" << spike.dx << " dy=" << spike.dy
                          << " dyaw=" << spike.dyaw << ")" << std::endl;
            }
            const double t_before = rows[i]->t_arrival;
            const bool would_be_nonincreasing = have_last_accepted_t && t_before <= last_t;
            mpc::PoseFilterOutput out = filt.step(t_before, zx, zy, zyaw);
            last_t = t_before;
            have_last_accepted_t = true;
            if (!out.accepted && !out.reinit && would_be_nonincreasing) {
                // The CSV's own t_arrival column is written in overall-arrival order and is
                // normally strictly increasing per body already; a non-increasing timestamp here
                // means step() took the "ignore" path (see PoseFilter.cpp) rather than a real
                // gate rejection -- counted separately so the summary stays meaningful.
                ++ignored;
                std::cout << "[pose_filter_replay]   " << body << " t=" << rows[i]->t_arrival
                          << " IGNORED (non-increasing timestamp)" << std::endl;
                continue;
            }
            if (out.reinit) {
                ++reinits;
                std::cout << "[pose_filter_replay]   " << body << " t=" << rows[i]->t_arrival
                          << " REINIT (consecutive_rejects_before=" << out.consecutive_rejects
                          << ")" << std::endl;
            } else if (!out.accepted) {
                ++rejects;
                std::cout << "[pose_filter_replay]   " << body << " t=" << rows[i]->t_arrival
                          << " REJECT d2_pos=" << out.d2_pos << " d2_yaw=" << out.d2_yaw
                          << " d2_all=" << out.d2_all
                          << " consecutive_rejects=" << out.consecutive_rejects << std::endl;
            }
            if (out.accepted) {
                max_d2_accepted = std::max({max_d2_accepted, out.d2_pos, out.d2_yaw, out.d2_all});
                if (have_last_accepted) {
                    const double dpos = std::hypot(out.x - last_x, out.y - last_y);
                    max_dpos_accepted = std::max(max_dpos_accepted, dpos);
                }
                last_x = out.x;
                last_y = out.y;
                have_last_accepted = true;
            }
        }

        total_rejects += rejects;
        total_reinits += reinits;

        std::cout << "[pose_filter_replay] --- body '" << body << "' summary ---\n"
                  << "  frames=" << rows.size() << " rejects=" << rejects
                  << " reinits=" << reinits << " ignored=" << ignored << "\n"
                  << "  max_d2_on_accepted=" << max_d2_accepted
                  << " max_delta_pos_between_accepted=" << max_dpos_accepted
                  << " (filter lifetime: rejects()=" << filt.rejects()
                  << " reinits()=" << filt.reinits() << ")" << std::endl;
    }

    std::cout << "\n[pose_filter_replay] TOTAL across " << by_body.size()
              << " body(ies): rejects=" << total_rejects << " reinits=" << total_reinits
              << std::endl;

    return 0;
}
