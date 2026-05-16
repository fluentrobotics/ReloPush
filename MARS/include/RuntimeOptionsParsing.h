#ifndef RUNTIME_OPTIONS_PARSING_H
#define RUNTIME_OPTIONS_PARSING_H

#include <PHAstarPushDemoOptions.h>
#include <string>

// Parse command-line arguments into RuntimeOptions
RuntimeOptions parse_runtime_options(int argc, char **argv);

// Get default sequence file path
std::string default_sequence_path();

#endif // RUNTIME_OPTIONS_PARSING_H
