#ifndef PARAMS_H
#define PARAMS_H

#include <string>

namespace params
{
    extern const std::string world_frame;

    // todo: parse map size as params
    extern float map_max_x; // m
    extern float map_max_y; // m

    extern const bool use_mocap; //todo: parse as a parameter
    extern const bool use_testdata;
    extern const bool reset_robot_pose;

    extern const bool print_graph;

    extern int leave_log;
    extern bool print_log;
    extern bool use_better_path;
}

#endif