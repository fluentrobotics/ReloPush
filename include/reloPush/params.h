#ifndef PARAMS_H
#define PARAMS_H

#include <string>

namespace params
{
    const std::string world_frame = "map";

    // todo: parse map size as params
    float map_max_x = 4; // m
    float map_max_y = 5.2; // m

    const bool use_mocap = false; //todo: parse as a parameter
    const bool use_testdata = true;
    const bool reset_robot_pose = false;

    const bool print_graph = false;
}

#endif