// SteeringCalib.h
//
// Shared VEHICLE calibration -- steering-servo calibration (center position
// + travel limits + invert) AND drive-direction sign (drive_invert) -- read
// and written by multiple programs (vesc_teleop's trim mode, vesc_driver,
// and later MPC-side tools) -- this module is the ONE persistent, shared
// piece all of them agree on. Both pieces live in this same struct/file
// (rather than a separate one for drive_invert) because they are
// calibrated and persisted together the same way. PURE logic + plain POSIX
// file I/O: no serial/VESC/socket access of its own (mirrors DriverCore.h's
// own "pure, fully unit-testable" convention -- see that file's header
// comment).
//
// Portability: C++14 only, POSIX only, no includes outside VescDriver/ --
// see VescDriver/CMakeLists.txt's HARD PORTABILITY RULES.
//
// Deployment note (see resolve_load_path()/resolve_save_path() below for
// the full precedence rules): this whole VescDriver/ folder is rsync
// --delete'd to the robot, so the LIVE calibration file must never live
// inside the synced tree -- it defaults to "$HOME/.vesc/steering_calib.json"
// on the target machine, well outside VescDriver/. VescDriver/config/
// steering_calib.example.json is a repo-shipped, read-only example/
// fallback only (see that file's own "_doc").

#ifndef VESC_DRIVER_STEERING_CALIB_H_
#define VESC_DRIVER_STEERING_CALIB_H_

#include <string>

namespace vesc {

struct SteeringCalib {
    int version = 1;
    double center = 0.5;
    double min_pos = 0.05;
    double max_pos = 0.95;
    bool invert = false;
    double rad_per_unit = 0.0;  // 0 = unknown/uncalibrated -- reserved for later steering-angle
                                 // work, not consumed by anything yet.
    // Whole-VEHICLE property (not steering-specific): true means the
    // robot's "forward" motor command sign is physically inverted
    // (wiring-dependent), so DRIVE code (not this file) must negate the
    // sign of its motor command when this is true. Lives in this same
    // struct/file as the steering fields because it is calibrated and
    // persisted the same way, not because it's part of the steering
    // servo itself.
    bool drive_invert = false;
    std::string note;
};

// Outcome of validate_and_clamp() -- see that function's own doc comment.
enum class ValidationOutcome {
    kOk,               // nothing needed changing.
    kClamped,          // center/min_pos/max_pos were clamped into [0,1] (a normal, silent-ish
                        // numeric-safety repair, not an error), ordering already held afterward.
    kRepairedOrdering  // the file/caller's own numbers had a genuinely broken ordering
                       // (not just out-of-[0,1]) -- center/min_pos/max_pos were RESET to this
                       // struct's own compiled-in defaults.
};

// Validates/repairs calib->center/min_pos/max_pos in place; invert/
// rad_per_unit/note/version are never touched by this function, regardless
// of outcome.
//
//   1. First, center/min_pos/max_pos are each clamped into [0,1]
//      unconditionally (e.g. a stray 1.05 -> 1.0). If any of the three
//      actually changed here, that alone (assuming ordering then holds --
//      see step 2) is outcome kClamped.
//   2. Then the ordering 0<=min_pos < center < max_pos<=1 is checked
//      (strict '<' on both). If it does NOT hold (min_pos>=center,
//      center>=max_pos, or min_pos==max_pos -- a genuinely broken
//      relationship, not simple out-of-range), ALL THREE of
//      center/min_pos/max_pos are reset to SteeringCalib{}'s own defaults
//      (0.5/0.05/0.95) and the outcome is kRepairedOrdering, overriding
//      whatever step 1 alone would have reported.
//   3. If neither step changed anything, kOk.
//
// On return, *calib always satisfies 0<=min_pos<center<max_pos<=1,
// regardless of which outcome is returned. If `note` is non-null, it is set
// to a one-line explanation of what (if anything) was repaired and why;
// left untouched when the outcome is kOk.
ValidationOutcome validate_and_clamp(SteeringCalib* calib, std::string* note);

// Loads a SteeringCalib from `path` (nlohmann::json, tolerant per-key
// fallback to SteeringCalib{}'s own defaults -- same style as
// DriverCore.h's load_driver_config()).
//
//   - File doesn't exist: returns false, *out = SteeringCalib{} (defaults),
//     *err_msg = a message including `path` and "not found"-type wording.
//     NOT an error condition for callers (a fresh install has no
//     calibration file yet) -- treat false here as "using defaults," but
//     the bool is still false either way; only *err_msg's text
//     distinguishes "missing" from "malformed" below.
//   - File exists but isn't valid JSON, or its top level isn't a JSON
//     object: returns false, *out = SteeringCalib{} (defaults), *err_msg
//     describes the parse failure.
//   - File exists, parses, but validate_and_clamp() on the parsed values
//     returns kRepairedOrdering (the file's own numbers were genuinely
//     inconsistent, not just slightly out of [0,1]): returns false,
//     *out = SteeringCalib{} (full compiled-in defaults -- never the
//     repaired-but-still-file-derived values), *err_msg explains why
//     (reuses validate_and_clamp's note).
//   - File exists, parses, and validates with outcome kOk or kClamped:
//     returns true, *out = the (possibly range-clamped) parsed values,
//     *err_msg cleared on kOk, or holds the clamp note on kClamped.
//
// Missing individual JSON keys fall back to SteeringCalib{}'s own per-field
// defaults rather than failing the whole load (same tolerant-per-key style
// as DriverCore.h's load_driver_config()).
bool load(const std::string& path, SteeringCalib* out, std::string* err_msg);

// Saves `calib` to `path` as pretty-printed JSON (nlohmann::json .dump(2),
// matching this repo's existing config-file convention -- e.g.
// MARS/src/simviz/MocapCore.cpp, MPC/src/motor_calibration.cpp).
//
//   - Runs validate_and_clamp() on a COPY of `calib` first and persists the
//     clamped copy -- never writes invalid data to disk. This is an added
//     safety measure beyond the strict task spec (save() is documented
//     here as always producing a valid file, even if the caller handed it
//     something out of range).
//   - Creates the parent directory (POSIX mkdir()-per-path-component walk,
//     no std::filesystem; each component created with mode 0755, EEXIST
//     tolerated, any other errno fails the whole save with *err_msg set
//     and neither `path` nor a ".tmp" file touched).
//   - Writes atomically: the full JSON goes to "<path>.tmp" first; on any
//     failure writing that file, the partial ".tmp" is unlink()'d and
//     save() returns false with *err_msg set, leaving the ORIGINAL `path`
//     file (if any existed) completely untouched. On a successful ".tmp"
//     write, rename(tmp, path) (atomic same-filesystem replace); if that
//     rename() fails, the ".tmp" is unlink()'d and save() returns false
//     with *err_msg set -- again leaving the original `path` untouched.
//   - On success: true, *err_msg cleared.
//
// JSON shape: a top-level object with keys "version"/"center"/"min_pos"/
// "max_pos"/"invert"/"rad_per_unit"/"drive_invert"/"note" (1:1 with the
// struct fields), plus "_doc" (explains this file is the shared vehicle
// calibration -- steering servo (center/limits/invert) AND drive-direction
// sign (drive_invert) -- read by vesc_teleop/vesc_driver/future MPC-side
// tools, and that it lives outside the repo tree deliberately) and
// "saved_unix_time" (integer, time(nullptr) at save time).
bool save(const std::string& path, const SteeringCalib& calib, std::string* err_msg);

// ---------------------------------------------------------------------
// Path resolution.
// ---------------------------------------------------------------------

// Pure w.r.t. the real environment (callers inject home_dir/exe_dir rather
// than this function reading getenv()/exe-detection itself, so it is fully
// unit-testable against fake temp directories) but DOES touch the
// filesystem to check existence of each precedence candidate -- mirrors
// vesc_driver_main.cpp's own resolve_default_config_path() pattern. See
// resolve_default_load_path()/resolve_default_save_path() below for the
// real-environment convenience wrappers actual programs call.
//
// Load precedence:
//   1. explicit_path, if non-empty -- returned as-is (existence is NOT
//      checked here; load() itself reports "not found" if it turns out not
//      to exist).
//   2. "<home_dir>/.vesc/steering_calib.json", if home_dir is non-empty AND
//      that file exists.
//   3. "<exe_dir>/../config/steering_calib.json", if exe_dir is non-empty
//      AND that file exists -- the repo-shipped example/fallback (read-
//      only; see config/steering_calib.example.json).
//   4. Falls back to "<home_dir>/.vesc/steering_calib.json" regardless of
//      whether it exists yet, if home_dir is non-empty (a deterministic
//      "this is where load() will report not-found, and where save() will
//      write" path). Empty string only if home_dir is ALSO empty.
std::string resolve_load_path(const std::string& explicit_path, const std::string& home_dir,
                               const std::string& exe_dir);

// Save path: explicit_path if non-empty, else
// "<home_dir>/.vesc/steering_calib.json". NEVER the exe_dir/repo-tree
// fallback (that one is read-only, see resolve_load_path() above). Empty
// string if explicit_path is empty AND home_dir is empty.
std::string resolve_save_path(const std::string& explicit_path, const std::string& home_dir);

// Real-environment convenience wrappers: home_dir = getenv("HOME") (empty
// if unset), exe_dir = this process's own executable directory via
// /proc/self/exe (same technique as vesc_driver_main.cpp's own
// own_exe_dir() -- duplicated here rather than including
// vesc_driver_main.cpp, per this folder's "no includes outside
// VescDriver/" rule applied file-to-file as well).
std::string resolve_default_load_path(const std::string& explicit_path);
std::string resolve_default_save_path(const std::string& explicit_path);

}  // namespace vesc

#endif  // VESC_DRIVER_STEERING_CALIB_H_
