#include<PlanHybridAstar.hpp>

/*
 * The original SH_ASTAR needs start/goal yaws to be negated. The resulting yaws also need to be negated.
 * Collision checking needs each state to be back to non-negated yaw
 */
PathPlanResultPtr planHybridAstar(ReloPush::State start_in, ReloPush::State goal_in,
                                  Environment& env, bool allow_reverse, float turning_radius, float speed, int64_t timeout_ms,
                                  bool print_res,float car_width, float LF, float obs_rad)
{
    //auto time_start = std::chrono::high_resolution_clock::now();
    // make sure the angle range is in 0~2pi
    start_in.yaw = fromOMPL::mod2pi(start_in.yaw);
    goal_in.yaw = fromOMPL::mod2pi(goal_in.yaw);

#pragma region check_validity
    // check if states are valid
    auto start_valid = env.stateValid(start_in, car_width,obs_rad);
    auto goal_valid = env.stateValid(goal_in, car_width, obs_rad);

    // not valid start/target
    if(!start_valid)
    {
        PathPlanResult solution(start_in,goal_in,PlanValidity::start_inval);
        //std::cout << "\033[1m\033[31m Start not valid \033[0m\n";
        //std::cout << "start not valid: (" << start.x << ", " << start.y << ", " << start.yaw << ")\n";
        solution.cost = -1;
        return std::make_shared<PathPlanResult>(solution);
    }
    if(!goal_valid)
    {
        PathPlanResult solution(start_in,goal_in,PlanValidity::goal_inval);
        //std::cout << "\033[1m\033[31m Target not valid \033[0m\n";
        //std::cout << "target not valid: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
        solution.cost = -1;
        return std::make_shared<PathPlanResult>(solution);
    }

    // if start and goal are the same, return empty path with success
    if(start_in == goal_in)
    {
        PathPlanResult solution(start_in,goal_in,PlanValidity::success);
        solution.states.clear();
        solution.actions.clear();
        //std::cout << "\033[1m\033[31m Target not valid \033[0m\n";
        //std::cout << "target: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
        solution.cost = 0;
        return std::make_shared<PathPlanResult>(solution);
    }
#pragma endregion

    // negate yaw for hybrid astar
    ReloPush::State start_neg = ReloPush::State(start_in.x,start_in.y,fromOMPL::mod2pi(-1*start_in.yaw));
    ReloPush::State goal_neg = ReloPush::State(goal_in.x, goal_in.y, fromOMPL::mod2pi(-1*goal_in.yaw));

    // choose
    env.changeGoal(goal_neg);

    //if(allow_reverse)
    //    env.nonPushMode(turning_radius,speed,LF);
    //else
    //    env.pushMode(turning_radius,speed,LF);

    HybridAStar<ReloPush::State, Action, double, Environment> hybridAStar(env);
    PathPlanResult solution(start_neg, goal_neg);
    bool searchSuccess = hybridAStar.search(start_neg, solution, allow_reverse, 0, timeout_ms);

    //auto time_end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
    //if(!searchSuccess)
    //    std::cout << "= " << "\tFailed" << " =" << std::endl;
    //std::cout << "=== " << duration << " ===" << std::endl;

    if (searchSuccess) {
        if(print_res)
        {
            std::cout << "\033[1m\033[32m Succesfully found a path! \033[0m\n";

            for (auto iter = solution.states.begin(); iter != solution.states.end(); iter++)
                std::cout << iter->first << "," << iter->second << std::endl;

            std::cout << "Solution: gscore/cost:" << solution.cost
                      << "\t fmin:" << solution.fmin << "\n\rDiscover " << env.Dcount
                      << " Nodes and Expand " << env.Ecount << " Nodes." << std::endl;
        }

        solution.validity = PlanValidity::success;
    }
    else {
        //if(print_res)
        //std::cout << "\033[1m\033[31m Failed to find a path \033[0m\n";
        //std::cout << "start: (" << start.x << ", " << start.y << ", " << start.yaw << ") target: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
        solution.cost = -1;
        solution.validity = PlanValidity::no_sol;
    }

    return std::make_shared<PathPlanResult>(solution);
}


PathPlanResultPtr planHybridAstar(ReloPush::State start_in, ReloPush::State goal_in, PlanningContext& ctx, bool allow_reverse)
{

    float rho, speed;
    if(allow_reverse)
    {
        rho = ctx.parameters.turning_rad_pair.non_push;
        speed = ctx.parameters.speed_pair.non_push;
    }
    else
    {
        rho = ctx.parameters.turning_rad_pair.push;
        speed = ctx.parameters.speed_pair.push;
    }


    return planHybridAstar(start_in, goal_in, ctx.env_nonpush, allow_reverse, rho, speed, ctx.timeout_ms,ctx.print_res,ctx.parameters.car_width, ctx.parameters.LF_nonpush,ctx.parameters.obs_rad);
}

//PathPlanResultPtr planHybridAstar(ReloPush::State start_in, ReloPush::State goal_in, Environment& env, bool allow_reverse, int64_t timeout_ms ,bool print_res,float car_width, float obs_rad)
