
#include<pathPlanTools/path_planning_tools.h>
#include <chrono>
std::shared_ptr<PlanResult<State, Action, double>> planHybridAstar(State start, State goal_in, Environment& env, int64_t timeout_ms ,bool print_res)
{  
    //auto time_start = std::chrono::high_resolution_clock::now();
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
        //std::cout << "\033[1m\033[31m Start not valid \033[0m\n";
        //std::cout << "start not valid: (" << start.x << ", " << start.y << ", " << start.yaw << ")\n";
        solution.cost = -1;
        return std::make_shared<PlanResult<State, Action, double>>(solution); 
    }
    if(!goal_valid)
    {
        PlanResult<State, Action, double> solution;
        //std::cout << "\033[1m\033[31m Target not valid \033[0m\n";
        //std::cout << "target not valid: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
        solution.cost = -1;
        return std::make_shared<PlanResult<State, Action, double>>(solution); 

    }

    // if start and goal are the same, return empty path with success
    if(start == goal_in)
    {
        PlanResult<State, Action, double> solution;
        solution.states.clear();
        solution.actions.clear();
        //std::cout << "\033[1m\033[31m Target not valid \033[0m\n";
        //std::cout << "target: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
        solution.cost = 0;
        return std::make_shared<PlanResult<State, Action, double>>(solution); 
    }

    // negate yaw for hybrid astar
    State start_neg = State(start.x,start.y,-1*start.yaw);
    State goal_neg = State(goal_in.x, goal_in.y, -1*goal_in.yaw);
    
    env.changeGoal(goal_neg);
    
    HybridAStar<State, Action, double, Environment> hybridAStar(env);
    PlanResult<State, Action, double> solution;
    bool searchSuccess = hybridAStar.search(start_neg, solution, 0, timeout_ms);

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
    }
    else {
        //if(print_res)
            //std::cout << "\033[1m\033[31m Failed to find a path \033[0m\n";
            //std::cout << "start: (" << start.x << ", " << start.y << ", " << start.yaw << ") target: (" << goal_in.x << ", " << goal_in.y << ", " << goal_in.yaw << ")\n";
            solution.cost = -1;
    }

    return std::make_shared<PlanResult<State, Action, double>>(solution);    
}
