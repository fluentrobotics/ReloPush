#ifndef PARAMS_H
#define PARAMS_H

#include <string>

namespace params
{
    extern const std::string world_frame;

    // todo: parse map size as params
    extern float map_max_x; // m
    extern float map_max_y; // m

    // index of post-push pose from last state of the path
    extern int post_push_ind;
    extern float interpolation_step;

    extern const bool use_mocap; //todo: parse as a parameter
    
    extern const bool reset_robot_pose;

    extern const bool print_graph;

    extern int leave_log;
    extern bool print_log;
    extern bool use_better_path;
    extern bool print_final_path;
    extern bool use_testdata;

    extern float grid_search_timeout;
}

#endif