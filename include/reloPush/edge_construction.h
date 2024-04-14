#include <vector>
#include <graphTools/graph_info.h>
#include <reloPush/movableObject.h>
#include <omplTools/dubins_tools.h>
#include <pathPlanTools/path_planning_tools.h>

namespace reloPush{
    void construct_edges(std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float turning_radius, bool print_log = false);
}