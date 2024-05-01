#include <vector>
#include <graphTools/graph_info.h>
#include <reloPush/movableObject.h>
#include <omplTools/dubins_tools.h>
#include <pathPlanTools/path_planning_tools.h>
#include <graphTools/edge_path_info.h>

namespace reloPush{
    void construct_edges(std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float turning_radius, graphTools::EdgeMatcher& edgeMatcher, bool print_log = false);
}