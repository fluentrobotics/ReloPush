#ifndef RUNTIME_OPTIONS_PARSING_H
#define RUNTIME_OPTIONS_PARSING_H

#include <PHAstarPushDemoOptions.h>
#include <string>
#include <vector>
#include <array>
#include <optional>

// Parse command-line arguments into RuntimeOptions
RuntimeOptions parse_runtime_options(int argc, char **argv);

// Get default sequence file path
std::string default_sequence_path();

// Parse a semicolon-separated list of poses (x,y,theta format).
// Returns a vector of std::array<double, 3> on success.
// Returns std::nullopt on error (prints error message to stderr).
// Accepts 1-4 poses; rejects more than 4 or malformed input.
std::optional<std::vector<std::array<double, 3>>> parse_robot_poses(const std::string &poses_str);

#endif // RUNTIME_OPTIONS_PARSING_H
