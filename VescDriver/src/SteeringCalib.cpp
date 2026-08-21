#include "SteeringCalib.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <sstream>

#include <sys/stat.h>
#include <unistd.h>

// Vendored copy (see VescDriver's "no includes outside VescDriver/" rule).
#include "../third_party/nlohmann/json.hpp"

namespace vesc {

namespace {

// Same tolerant per-key assign helper as DriverCore.cpp's own
// assign_if_present() (reproduced here rather than shared, per this
// folder's "no includes outside VescDriver/" rule applied file-to-file).
template <typename T>
void assign_if_present(const nlohmann::json& j, const char* key, T* out) {
    if (!j.contains(key)) {
        return;
    }
    try {
        *out = j.at(key).get<T>();
    } catch (...) {
        // Wrong JSON type for this key -- silently retain whatever *out
        // already held.
    }
}

bool path_exists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

// Creates every path component of `dir` that doesn't already exist, POSIX
// mkdir()-per-component (no std::filesystem -- see CMakeLists.txt's HARD
// PORTABILITY RULES). Returns true if `dir` exists (or was created) as a
// directory by the end; false on any real failure (e.g. permission
// denied), with *err_msg set.
bool ensure_dir_exists(const std::string& dir, std::string* err_msg) {
    if (dir.empty() || dir == "." || dir == "/") {
        return true;
    }
    if (path_exists(dir)) {
        return true;  // already there (file-vs-dir distinction left to the later open()/rename()).
    }

    // Walk component-by-component from the root so each intermediate
    // directory is created in order (mkdir() cannot create more than one
    // missing level at a time).
    std::string prefix;
    size_t pos = 0;
    if (!dir.empty() && dir[0] == '/') {
        prefix = "/";
        pos = 1;
    }
    while (pos <= dir.size()) {
        size_t next = dir.find('/', pos);
        if (next == std::string::npos) next = dir.size();
        const std::string component = dir.substr(pos, next - pos);
        if (!component.empty()) {
            prefix += component;
            if (!path_exists(prefix)) {
                if (mkdir(prefix.c_str(), 0755) != 0 && errno != EEXIST) {
                    if (err_msg) {
                        *err_msg = "could not create directory '" + prefix + "': " + std::strerror(errno);
                    }
                    return false;
                }
            }
            prefix += "/";
        }
        pos = next + 1;
    }
    return path_exists(dir);
}

// Directory portion of `path` (everything before the last '/'), or empty
// if `path` has no '/'.
std::string dirname_of(const std::string& path) {
    const size_t slash = path.find_last_of('/');
    return (slash == std::string::npos) ? std::string() : path.substr(0, slash);
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

}  // namespace

// ---------------------------------------------------------------------
// Validation.
// ---------------------------------------------------------------------

ValidationOutcome validate_and_clamp(SteeringCalib* calib, std::string* note) {
    if (!calib) {
        return ValidationOutcome::kOk;
    }

    const double orig_center = calib->center;
    const double orig_min = calib->min_pos;
    const double orig_max = calib->max_pos;

    calib->center = std::min(1.0, std::max(0.0, calib->center));
    calib->min_pos = std::min(1.0, std::max(0.0, calib->min_pos));
    calib->max_pos = std::min(1.0, std::max(0.0, calib->max_pos));

    const bool clamped = (calib->center != orig_center) || (calib->min_pos != orig_min) ||
                          (calib->max_pos != orig_max);

    const bool ordering_ok = (calib->min_pos >= 0.0) && (calib->min_pos < calib->center) &&
                              (calib->center < calib->max_pos) && (calib->max_pos <= 1.0);

    if (!ordering_ok) {
        const SteeringCalib defaults;
        calib->center = defaults.center;
        calib->min_pos = defaults.min_pos;
        calib->max_pos = defaults.max_pos;
        if (note) {
            std::ostringstream oss;
            oss << "steering calibration ordering invalid (min_pos=" << orig_min
                << ", center=" << orig_center << ", max_pos=" << orig_max
                << " does not satisfy 0<=min_pos<center<max_pos<=1) -- reset to defaults "
                   "(center="
                << defaults.center << ", min_pos=" << defaults.min_pos
                << ", max_pos=" << defaults.max_pos << ")";
            *note = oss.str();
        }
        return ValidationOutcome::kRepairedOrdering;
    }

    if (clamped) {
        if (note) {
            std::ostringstream oss;
            oss << "steering calibration values clamped into [0,1] (center " << orig_center << "->"
                << calib->center << ", min_pos " << orig_min << "->" << calib->min_pos
                << ", max_pos " << orig_max << "->" << calib->max_pos << ")";
            *note = oss.str();
        }
        return ValidationOutcome::kClamped;
    }

    return ValidationOutcome::kOk;
}

// ---------------------------------------------------------------------
// load() / save().
// ---------------------------------------------------------------------

bool load(const std::string& path, SteeringCalib* out, std::string* err_msg) {
    if (out) *out = SteeringCalib{};

    if (!path_exists(path)) {
        if (err_msg) *err_msg = "steering calibration file not found: '" + path + "'";
        return false;
    }

    std::ifstream f(path);
    if (!f.is_open()) {
        if (err_msg) *err_msg = "could not open steering calibration file '" + path + "'";
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(ss.str());
    } catch (const std::exception& e) {
        if (err_msg) *err_msg = std::string("invalid JSON in '") + path + "': " + e.what();
        return false;
    } catch (...) {
        if (err_msg) *err_msg = "invalid JSON in '" + path + "' (unknown exception)";
        return false;
    }
    if (!j.is_object()) {
        if (err_msg) *err_msg = "top level of '" + path + "' is not a JSON object";
        return false;
    }

    SteeringCalib parsed;  // starts from SteeringCalib{} defaults.
    assign_if_present(j, "version", &parsed.version);
    assign_if_present(j, "center", &parsed.center);
    assign_if_present(j, "min_pos", &parsed.min_pos);
    assign_if_present(j, "max_pos", &parsed.max_pos);
    assign_if_present(j, "invert", &parsed.invert);
    assign_if_present(j, "rad_per_unit", &parsed.rad_per_unit);
    assign_if_present(j, "drive_invert", &parsed.drive_invert);
    assign_if_present(j, "note", &parsed.note);

    std::string validation_note;
    const ValidationOutcome outcome = validate_and_clamp(&parsed, &validation_note);

    if (outcome == ValidationOutcome::kRepairedOrdering) {
        if (out) *out = SteeringCalib{};  // never trust file-derived values on a broken ordering.
        if (err_msg) *err_msg = "steering calibration file '" + path + "' rejected: " + validation_note;
        return false;
    }

    if (out) *out = parsed;
    if (err_msg) *err_msg = (outcome == ValidationOutcome::kClamped) ? validation_note : std::string();
    return true;
}

bool save(const std::string& path, const SteeringCalib& calib, std::string* err_msg) {
    if (err_msg) err_msg->clear();

    SteeringCalib clamped = calib;  // never persist invalid data -- see header doc comment.
    validate_and_clamp(&clamped, nullptr);

    const std::string dir = dirname_of(path);
    if (!ensure_dir_exists(dir, err_msg)) {
        return false;  // *err_msg already set by ensure_dir_exists(); path/.tmp untouched.
    }

    nlohmann::json j;
    j["_doc"] =
        "Shared vehicle calibration: steering servo (center position + travel limits + invert) AND "
        "drive-direction sign (drive_invert, true if the robot's 'forward' motor command sign is "
        "physically inverted by wiring), read/written by vesc_teleop's trim mode, vesc_driver, and "
        "future MPC-side tools. Deliberately lives OUTSIDE the VescDriver/ repo tree (see "
        "SteeringCalib.h's resolve_load_path()/resolve_save_path()) because that whole folder is "
        "rsync --delete'd on every deploy to the robot -- anything saved inside it would be "
        "destroyed on the next deploy.";
    j["version"] = clamped.version;
    j["center"] = clamped.center;
    j["min_pos"] = clamped.min_pos;
    j["max_pos"] = clamped.max_pos;
    j["invert"] = clamped.invert;
    j["rad_per_unit"] = clamped.rad_per_unit;
    j["drive_invert"] = clamped.drive_invert;
    j["note"] = clamped.note;
    j["saved_unix_time"] = static_cast<long long>(std::time(nullptr));

    const std::string tmp_path = path + ".tmp";
    {
        std::ofstream f(tmp_path, std::ios::trunc);
        if (!f.is_open()) {
            if (err_msg) *err_msg = "could not open '" + tmp_path + "' for writing";
            unlink(tmp_path.c_str());
            return false;
        }
        f << j.dump(2);
        f.flush();
        if (!f.good()) {
            if (err_msg) *err_msg = "write error writing '" + tmp_path + "'";
            f.close();
            unlink(tmp_path.c_str());
            return false;
        }
    }

    if (rename(tmp_path.c_str(), path.c_str()) != 0) {
        if (err_msg) {
            *err_msg = "could not rename '" + tmp_path + "' to '" + path + "': " + std::strerror(errno);
        }
        unlink(tmp_path.c_str());
        return false;
    }

    if (err_msg) err_msg->clear();
    return true;
}

// ---------------------------------------------------------------------
// Path resolution.
// ---------------------------------------------------------------------

std::string resolve_load_path(const std::string& explicit_path, const std::string& home_dir,
                               const std::string& exe_dir) {
    if (!explicit_path.empty()) {
        return explicit_path;
    }

    if (!home_dir.empty()) {
        const std::string home_candidate = home_dir + "/.vesc/steering_calib.json";
        if (path_exists(home_candidate)) {
            return home_candidate;
        }
    }

    if (!exe_dir.empty()) {
        const std::string exe_candidate = exe_dir + "/../config/steering_calib.json";
        if (path_exists(exe_candidate)) {
            return exe_candidate;
        }
    }

    if (!home_dir.empty()) {
        return home_dir + "/.vesc/steering_calib.json";
    }

    return "";
}

std::string resolve_save_path(const std::string& explicit_path, const std::string& home_dir) {
    if (!explicit_path.empty()) {
        return explicit_path;
    }
    if (!home_dir.empty()) {
        return home_dir + "/.vesc/steering_calib.json";
    }
    return "";
}

std::string resolve_default_load_path(const std::string& explicit_path) {
    const char* home_env = std::getenv("HOME");
    const std::string home_dir = home_env ? std::string(home_env) : std::string();
    const std::string exe_dir = own_exe_dir();
    return resolve_load_path(explicit_path, home_dir, exe_dir);
}

std::string resolve_default_save_path(const std::string& explicit_path) {
    const char* home_env = std::getenv("HOME");
    const std::string home_dir = home_env ? std::string(home_env) : std::string();
    return resolve_save_path(explicit_path, home_dir);
}

}  // namespace vesc
