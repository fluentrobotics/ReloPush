
#include<pathPlanTools/path_planning_tools.h>
#include <chrono>

PlanningContext::PlanningContext()
{}

PlanningContext::PlanningContext(bool use_reverse, float turning_r, float speed_lim) : allow_reverse(use_reverse), turning_radius(turning_r), speed_limit(speed_lim)
{
    deltat = speed_limit / turning_r / 1.5;
    xyResolution = turning_r * deltat;
    yawResolution = deltat;

    dx.resize(6);
    dy.resize(6);
    dyaw.resize(6);

    update_dx();
    update_dy();
    update_dyaw();
}

void PlanningContext::update_dx()
{
    dx[0] = turning_radius * deltat;
    dx[1] = turning_radius * sin(deltat);
    dx[2] = turning_radius * sin(deltat);
    dx[3] = -turning_radius * deltat;
    dx[4] = -turning_radius * sin(deltat);
    dx[5] = -turning_radius * sin(deltat);
}
void PlanningContext::update_dy()
{
    double y_term = turning_radius * (1 - cos(deltat));
    dy[0] = 0;
    dy[1] = -y_term;
    dy[2] = y_term;
    dy[3] = 0;
    dy[4] = -y_term;
    dy[5] = y_term;
}
void PlanningContext::update_dyaw()
{
    dyaw[0] = 0;
    dyaw[1] = deltat;
    dyaw[2] = -deltat;
    dyaw[3] = 0;
    dyaw[4] = -deltat;
    dyaw[5] = deltat;
}


namespace Constants {

    /*
    float r = r_nonpush;
    float deltat = deltat_nonpush;
    bool allow_reverse = true;
    float xyResolution = xyResolution_nonpush;
    float yawResolution = yawResolution_nonpush;

    double dx[] = {r * deltat, r* sin(deltat),  r* sin(deltat),
                     -r* deltat, -r* sin(deltat), -r* sin(deltat)};
    double dy[] = {0, -r*(1 - cos(deltat)), r*(1 - cos(deltat)),
                        0, -r*(1 - cos(deltat)), r*(1 - cos(deltat))};
    double dyaw[] = {0, deltat, -deltat, 0, -deltat, deltat};
    */

    float normalizeHeadingRad(float t) {
        if (t < 0) {
            t = t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
            return 2.f * M_PI + t;
        }

        return t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
    }
/*
    void update_dx()
    {
        dx[0] = r * deltat;
        dx[1] = r * sin(deltat);
        dx[2] = r * sin(deltat);
        dx[3] = -r * deltat;
        dx[4] = -r * sin(deltat);
        dx[5] = -r * sin(deltat);
    }

    void update_dy()
    {
        double y_term = r * (1 - cos(deltat));
        dy[0] = 0;
        dy[1] = -y_term;
        dy[2] = y_term;
        dy[3] = 0;
        dy[4] = -y_term;
        dy[5] = y_term;
    }

    void update_dyaw()
    {
        dyaw[0] = 0;
        dyaw[1] = deltat;
        dyaw[2] = -deltat;
        dyaw[3] = 0;
        dyaw[4] = -deltat;
        dyaw[5] = deltat;
    }

    void update_dx_dy_dyaw()
    {
        update_dx();
        update_dy();
        update_dyaw;
    }

    void switch_to_pushing()
    {
        r = r_push;
        deltat = deltat_push;
        xyResolution = xyResolution_push;
        yawResolution = yawResolution_push;
        allow_reverse = false;

        params::is_pushing = true;

        update_dx_dy_dyaw();
    }

    void switch_to_nonpushing()
    {
        r = r_nonpush;
        deltat = deltat_nonpush;
        xyResolution = xyResolution_nonpush;
        yawResolution = yawResolution_nonpush;
        allow_reverse = true;

        params::is_pushing = false;

        update_dx_dy_dyaw();
    }
    */

}


PathPlanResultPtr planHybridAstar(State start_in, State goal_in, Environment& env, int64_t timeout_ms ,bool print_res,float car_width, float obs_rad)
{  
    //auto time_start = std::chrono::high_resolution_clock::now();
    // make sure the angle range is in 0~2pi
    start_in.yaw = jeeho::convertEulerRange_to_2pi(start_in.yaw);
    goal_in.yaw = jeeho::convertEulerRange_to_2pi(goal_in.yaw);

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
    State start_neg = State(start_in.x,start_in.y,-1*start_in.yaw);
    State goal_neg = State(goal_in.x, goal_in.y, -1*goal_in.yaw);
    
    // choose 
    env.changeGoal(goal_neg);
    
    HybridAStar<State, Action, double, Environment> hybridAStar(env);
    PathPlanResult solution(start_in, goal_in);
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
