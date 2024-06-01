#ifndef PARAMS_H
#define PARAMS_H

#include <string>

namespace params
{
    static const std::string world_frame = "map";

    // todo: parse map size as params
    static float map_max_x = 4; // m
    static float map_max_y = 5.4; // m

    static const bool use_mocap = false; //todo: parse as a parameter
    static const bool use_testdata = true;
    static const bool reset_robot_pose = true;

    static const bool print_graph = true;

    static int leave_log = 0;
    static bool print_log = true;
    static bool use_better_path = false;
}

#endif