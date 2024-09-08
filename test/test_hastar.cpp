#include <pathPlanTools/path_planning_tools.h>
#include <reloPush/params.h>


int main()
{
    // start pose
    State start_pose = State(1,1,0);

    // goal pose
    State goal_pose = State(4,4,0);

    // obstacles
    std::unordered_set<State> obs;

    // init env
    Environment env(5,5,obs,0.5,0.3, true, goal_pose);

    auto plan_res = planHybridAstar(start_pose, goal_pose, env, params::grid_search_timeout, false);

    auto path = plan_res->getPath(true);

    for(auto it = path.begin(); it != path.end(); it++)
    {
        std::cout << *it;
        if(std::next(it) != path.end())
            std::cout << ",";
        std::cout << std::endl;
    }

    Environment env2(5,5,env.get_obs(),Constants::r_push,Constants::LF_nonpush,false,goal_pose);

    plan_res = planHybridAstar(start_pose, goal_pose, env2, params::grid_search_timeout, false);

    path = plan_res->getPath(true);

    for(auto it = path.begin(); it != path.end(); it++)
    {
        std::cout << *it;
        if(std::next(it) != path.end())
            std::cout << ",";
        std::cout << std::endl;
    }

    return 0;
}