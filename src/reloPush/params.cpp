#include <reloPush/params.h>

namespace params
{
    const std::string world_frame = "map";

    // todo: parse map size as params
    float map_max_x = 4; // m
    float map_max_y = 5.4; // m

    float pre_push_dist = 0.6f;
    int post_push_ind = -2; // ind
    float interpolation_step = 0.2f;

    const bool use_mocap = false; //todo: parse as a parameter
    
    const bool reset_robot_pose = true;

    const bool print_graph = false;

    // baseline: MP only
    const bool use_mp_only = false;

    int leave_log = 0;
    bool print_log = false;
    bool use_better_path = false; // true for proposed, false for dubins only
    bool print_final_path = false;
    bool use_testdata = true; //use file to initialize planning

    int64_t grid_search_timeout = 0; //ms (0 for no timeout)
}