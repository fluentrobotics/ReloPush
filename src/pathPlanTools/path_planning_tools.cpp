
#include<pathPlanTools/path_planning_tools.h>

std::shared_ptr<PlanResult<State, Action, double>> planHybridAstar(State start, State goal_in, Environment& env, bool print_res)
{
    // make sure the angle range is in 0~2pi
    start.yaw = jeeho::convertEulerRange_to_2pi(start.yaw);
    goal_in.yaw = jeeho::convertEulerRange_to_2pi(goal_in.yaw);

    // check if states are valid
    auto start_valid = env.stateValid(start);
    auto goal_valid = env.stateValid(goal_in);

    // not valid start/target
    if(!start_valid)
    {
        PlanResult<State, Action, double> solution;
        std::cout << "\033[1m\033[31m Start not valid \033[0m\n";
        std::cout << "start: (" << start.x << ", " << start.y << ", " << start.yaw << ")\n";
        solution.cost = -1;
        return std::make_shared<PlanResult<State, Action, double>>(solution); 
    }
    else if(!goal_valid)
    {
        PlanResult<State, Action, double> solution;
        std::cout << "\033[1m\033[31m Target not valid \033[0m\n";
        std::cout << "target: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
        solution.cost = -1;
        return std::make_shared<PlanResult<State, Action, double>>(solution); 

    }

    // negate yaw for hybrid astar
    State start_neg = State(start.x,start.y,-1*start.yaw);
    State goal_neg = State(goal_in.x, goal_in.y, -1*goal_in.yaw);
    
    env.changeGoal(goal_neg);
    
    HybridAStar<State, Action, double, Environment> hybridAStar(env);
    PlanResult<State, Action, double> solution;
    bool searchSuccess = hybridAStar.search(start_neg, solution);

    if (searchSuccess) {
        if(print_res)
        {
            std::cout << "\033[1m\033[32m Succesfully find a path! \033[0m\n";
    
            for (auto iter = solution.states.begin(); iter != solution.states.end(); iter++)
            std::cout << iter->first << "," << iter->second << std::endl;

            std::cout << "Solution: gscore/cost:" << solution.cost
                    << "\t fmin:" << solution.fmin << "\n\rDiscover " << env.Dcount
                    << " Nodes and Expand " << env.Ecount << " Nodes." << std::endl;
        } 
    }
    else {
        //if(print_res)
            std::cout << "\033[1m\033[31m Fail to find a path \033[0m\n";
            std::cout << "start: (" << start.x << ", " << start.y << ", " << start.yaw << ") target: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
            solution.cost = -1;
    }

    return std::make_shared<PlanResult<State, Action, double>>(solution);    
}
