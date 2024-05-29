#ifndef PARSE_TESTDATA_H
#define PARSE_TESTDATA_H

#include <vector>
#include <string>

#include "movableObject.h"
#include <omplTools/State.h>
#include <pathPlanTools/tf_tools.h>
#include <iostream>
#include <fstream>


std::vector<std::string> split(std::string& s, std::string delimiter);

std::string removeExtension(const std::string& filename);

/// Read a file and create a std::vector by lines
std::vector<std::string> read_file(std::string f_path);


std::unordered_map<std::string,std::string> parse_movableObjects_robots_from_file(std::vector<movableObject>& mo_list, 
                                            std::vector<State>& robots,
                                            std::vector<movableObject>& delivery_list, 
                                            size_t data_ind, std::string file_path);


#endif