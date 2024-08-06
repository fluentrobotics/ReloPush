#include <vector>
#include <graphTools/graph_info.h>
#include <reloPush/movableObject.h>
#include <omplTools/dubins_tools.h>
#include <pathPlanTools/path_planning_tools.h>
#include <graphTools/edge_path_info.h>
#include <reloPush/params.h>
#include <reloPush/push_pose_tools.h>
#include <reloPush/stopwatch.h>

#include <tuple>
#include <algorithm>

namespace reloPush{
    void construct_edges(std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float max_x, float max_y, float turning_radius, 
                        graphTools::EdgeMatcher& edgeMatcher, std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,std::vector<stopWatch>& time_watches);
    void add_deliveries(std::vector<movableObject>& delivery_list, std::vector<movableObject>& mo_list, GraphPtr gPtr, 
                        Environment& env, float max_x, float max_y, float turning_radius, graphTools::EdgeMatcher& edgeMatcher,
                        std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,std::vector<stopWatch>& time_watches, bool print_log = false);
}