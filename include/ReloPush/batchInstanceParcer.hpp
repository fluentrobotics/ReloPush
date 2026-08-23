#ifndef BATCH_INSTANCE_PARCER_HPP
#define BATCH_INSTANCE_PARCER_HPP

#include <ReloPush/State.h>
#include <ReloPush/ObjectInfo.hpp>

#include <ReloPush/PrintInColor.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <ReloPush/GraphBuilder.hpp>

#include <ReloPush/config.h>

std::vector<std::string> split(std::string& s, std::string delimiter);

std::string removeExtension(const std::string& filename);

/// Read a file and create a std::vector by lines
std::vector<std::string> read_file(std::string f_path);

bool parse_instance_from_file( std::string file_path, size_t data_ind,
                               ObjectMap& objects,
                               GoalMap& goals,
                               std::vector<ReloPush::State>& robots,
                               std::unordered_map<std::string, ObjectGoalPair> &objGoalPairs,
                               bool* has_ws = nullptr,
                               double* ws_x = nullptr,
                               double* ws_y = nullptr);

/*
 * argv[1]: filename, argv[2]: index, argv[3]: use_opt
 */
void handle_args(int argc, char **argv, std::string& data_file, int& data_ind, bool& use_opt, bool& no_init_guess, bool& use_dfs);

#endif
