#include <reloPush/params.h>

namespace params
{
    const std::string world_frame = "map";

    // todo: parse map size as params
    float map_max_x = 4; // m
    float map_max_y = 5.4; // m

    int post_push_ind = -4; // ind

    const bool use_mocap = false; //todo: parse as a parameter
    const bool use_testdata = true;
    const bool reset_robot_pose = true;

    const bool print_graph = true;

    int leave_log = 0;
    bool print_log = true;
    bool use_better_path = true;
}