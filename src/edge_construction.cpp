#include <reloPush/edge_construction.h>

// Assumes a rectangular workspace
bool is_within_boundary(OmplState* state, float min_x, float min_y, float max_x, float max_y)
{
    if (state->getX() < 0 || state->getX() >= max_x || state->getY() < 0 || state->getY() >= max_y)
    {
        if(params::print_log)
            std::cout << "Out of Boundary " << "x: "<< state->getX() << " y: " << state->getY() << " yaw: " << state->getYaw() << std::endl;
        return false;
    }
    else
        return true;
}

StateValidity check_validity_by_interpolation(OmplState* dubinsStart, OmplState* interState, std::pair<pathType,reloDubinsPath>& dubins_res,
                                                ompl::base::DubinsStateSpace& dubinsSpace,
                                                float& turning_radius, size_t num_pts, 
                                                Environment& env, float max_x, float max_y)
{
    StateValidity validity = StateValidity::valid;

    for (size_t np=0; np<num_pts; np++)
    {
        //auto start = std::chrono::steady_clock::now();
        //std::cout << "DubinsStart x: " << dubinsStart->getX() << " y: " << dubinsStart->getY() << std::endl;

        jeeho_interpolate(dubinsStart, dubins_res.second.omplDubins, (double)np / (double)num_pts, interState, &dubinsSpace,
                        turning_radius);

        //std::cout << "interStart x: " << interState->getX() << " y: " << interState->getY() << std::endl;
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        //std::cout << "Elapsed time: " << duration << " usec" << std::endl;

        if(params::print_log)
            std::cout << interState->getX() << " " << interState->getY() << " " << interState->getYaw() << std::endl;


        auto val = env.stateValid(State(interState->getX(),interState->getY(),interState->getYaw()),0.15,0,0.15,0.15);
        if(!val) //todo: set better values
        {
            if(val.get_validity()==StateValidity::collision){
                // for debug
                float x_debug = interState->getX();
                float y_debug = interState->getY();
                float yaw_debug = interState->getYaw();
                //collision found
                if(params::print_log)
                    std::cout << "Collision found " << np << "x: "<< interState->getX() << " y: " << interState->getY() << " yaw: " << interState->getYaw() << std::endl;
                validity = StateValidity::collision;

                break;
            }
            else if(val.get_validity()==StateValidity::out_of_boundary)
            {
                validity = StateValidity::out_of_boundary;
            }
        }                             
    }

    return validity;
}

// straight path
StateValidity check_validity_by_interpolation(State& from, State& to, float resolution, 
                                                Environment& env, float max_x, float max_y)
{
    StateValidity validity = StateValidity::valid;

    // interpolate the straight path
    auto interpolated_path = interpolateStraightPath(from,to,resolution);

    for (size_t p=0; p< interpolated_path.size(); p++)
    {
        auto v = env.stateValid(interpolated_path[p]);

        if(!v)
        {
            validity = v.data.second;
            break;
        }

    }

    return validity;
}

/*
** input: turning_radius, pivot_state(from_state), dubinsSpace
** output: dubinsStart, interState
*/
void init_dubins_state(float turning_radius, StatePtr pivot_state,
                       ompl::base::DubinsStateSpace& dubinsSpace, OmplState* dubinsStart)
{
    // Allocate and initialize the starting state
    
    dubinsStart->setXY(pivot_state->x, pivot_state->y);
    dubinsStart->setYaw(pivot_state->yaw);
    
    // Allocate and initialize the intermediate state
    //interState = (OmplState *)dubinsSpace.allocState();
}

StateValidity check_collision(std::vector<movableObject>& mo_list, StatePtr pivot_state, VertexPtr pivot_vertex, size_t state_ind,
                    size_t n, size_t m, std::pair<pathType,reloDubinsPath> dubins_res, Environment& env, float max_x, float max_y, float turning_radius, bool is_delivery = false)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius); OmplState* dubinsStart = (OmplState *)dubinsSpace.allocState(), *interState = (OmplState *)dubinsSpace.allocState();
    init_dubins_state(turning_radius, pivot_state, dubinsSpace, dubinsStart);
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = static_cast<int>(dubins_res.second.lengthCost()/(0.1*0.5)); //todo: get resolution as a param

    StateValidity validity = StateValidity::valid;
    
    std::vector<State> took_out(0);
    // takeout pivot object from obstacles
    took_out.push_back(State(mo_list[n].get_x(),mo_list[n].get_y(),0));
    env.remove_obs(State(mo_list[n].get_x(),mo_list[n].get_y(),0));

    // takeout target object from obstacles
    // only if this is not a delivery location
    if(!is_delivery){
        took_out.push_back(State(mo_list[m].get_x(),mo_list[m].get_y(),0));
        env.remove_obs(State(mo_list[m].get_x(),mo_list[m].get_y(),0));
    }

    // Interpolate dubins path to check for collision on grid map
    validity = check_validity_by_interpolation(dubinsStart, interState, dubins_res, dubinsSpace, turning_radius, num_pts, env, max_x, max_y);

    if(validity == StateValidity::valid && params::print_log)
        std::cout << "edge found" << std::endl;

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);
    
    return validity;
}

StateValidity check_collision(movableObject fromObj, movableObject toObj, StatePtr pivot_state, VertexPtr pivot_vertex, size_t state_ind,
                    std::pair<pathType,reloDubinsPath> dubins_res, Environment& env, float max_x, float max_y, float turning_radius, bool is_delivery = false)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius); OmplState* dubinsStart = (OmplState *)dubinsSpace.allocState(), *interState = (OmplState *)dubinsSpace.allocState();
    init_dubins_state(turning_radius, pivot_state, dubinsSpace, dubinsStart);
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = static_cast<int>(dubins_res.second.lengthCost()/(0.1*0.5)); //todo: get resolution as a param

    StateValidity validity = StateValidity::valid;
    
    std::vector<State> took_out(0);
    // takeout pivot object from obstacles
    took_out.push_back(State(fromObj.get_x(),fromObj.get_y(),0));
    env.remove_obs(State(fromObj.get_x(),fromObj.get_y(),0));

    // takeout target object from obstacles
    // only if this is not a delivery location
    if(!is_delivery){
        took_out.push_back(State(toObj.get_x(),toObj.get_y(),0));
        env.remove_obs(State(toObj.get_x(),toObj.get_y(),0));
    }

    // Interpolate dubins path to check for collision on grid map
    validity = check_validity_by_interpolation(dubinsStart, interState, dubins_res, dubinsSpace, turning_radius, num_pts, env, max_x, max_y);

    if(validity == StateValidity::valid && params::print_log)
        std::cout << "edge found" << std::endl;         

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);

    return validity;
}

StateValidity check_collision_straight(movableObject fromObj, movableObject toObj, StatePtr pivot_state, VertexPtr pivot_vertex, size_t state_ind,
                    std::pair<State,State> straight_path, Environment& env, float max_x, float max_y, float turning_radius, bool is_delivery = false)
{

    // path length
    auto dx = straight_path.second.x - straight_path.first.x;
    auto dy = straight_path.second.y - straight_path.first.y;
    auto path_length = sqrtf(dx*dx + dy*dy);

    size_t num_pts = static_cast<int>(path_length/(0.1*0.5)); //todo: get resolution as a param

    StateValidity validity = StateValidity::valid;
    
    std::vector<State> took_out(0);
    // takeout pivot object from obstacles
    took_out.push_back(State(fromObj.get_x(),fromObj.get_y(),0));
    env.remove_obs(State(fromObj.get_x(),fromObj.get_y(),0));

    // takeout target object from obstacles
    // only if this is not a delivery location
    if(!is_delivery){
        took_out.push_back(State(toObj.get_x(),toObj.get_y(),0));
        env.remove_obs(State(toObj.get_x(),toObj.get_y(),0));
    }

    // Interpolate dubins path to check for collision on grid map
    validity = check_validity_by_interpolation(straight_path.first,straight_path.second,Constants::mapResolution,env,max_x,max_y);;

    if(validity == StateValidity::valid && params::print_log)
        std::cout << "edge found" << std::endl;         

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);

    return validity;
}

StateValidity check_collision_straight(Environment& env, State& from_state, State& to_state, float path_length, float max_x, float max_y ,bool is_delivery = false)
{
    size_t num_pts = static_cast<int>(path_length/(0.1*0.5)); //todo: get resolution as a param

    StateValidity validity = StateValidity::valid;
    
    std::vector<State> took_out(0);
    // takeout pivot object from obstacles
    took_out.push_back(State(from_state));
    env.remove_obs(from_state);

    // takeout target object from obstacles
    // only if this is not a delivery location
    if(!is_delivery){
        took_out.push_back(State(to_state));
        env.remove_obs(to_state);
    }

    // Interpolate dubins path to check for collision on grid map
    validity = check_validity_by_interpolation(from_state,to_state,Constants::mapResolution,env,max_x,max_y);

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);

    return validity;
}

// todo: use better approximation <Pose,distance,uvec_index>
std::vector<std::tuple<State,float,int>> find_pre_relo_along_vector(Environment& env, State& pivot_state, State& target_state, float turning_rad, std::vector<Eigen::Vector2f> unit_vectors)
{
    // remove obs
    env.remove_obs(pivot_state);

    //float d_thres = get_longpath_d_thres(pivot_state,target_state,turning_rad);

    std::vector<std::tuple<State,float,int>> found_poses(0);

    for(size_t n=0; n<unit_vectors.size(); n++)
    {
        bool is_found = false;
        auto incre = unit_vectors[n] * Constants::mapResolution; // todo: there may be a better step size
        //float vec_yaw = atan2f(incre.y(),incre.x());
        State temp_state = State(pivot_state.x+incre.x(), pivot_state.y+incre.y(), pivot_state.yaw);
        
        //validity
        auto validity = env.stateValid(temp_state,Constants::obsRadius*2, Constants::obsRadius, Constants::obsRadius, Constants:: obsRadius);
        while(validity.get_validity() != StateValidity::out_of_boundary)
        {
            if(validity.get_validity() == StateValidity::valid)
            {
                float d_thres = get_longpath_d_thres(temp_state,target_state,turning_rad);
                // found if a long path
                float d_temp = get_current_longpath_d(temp_state,target_state);
                if(d_temp < d_thres)
                {
                    is_found = true;
                    break;
                }
                else
                {
                    temp_state.x += incre.x();
                    temp_state.y += incre.y();
                    validity = env.stateValid(temp_state,Constants::obsRadius*2, Constants::obsRadius, Constants::obsRadius, Constants:: obsRadius);
                }
            }
            else
            {
                temp_state.x += incre.x();
                temp_state.y += incre.y();
                validity = env.stateValid(temp_state,Constants::obsRadius*2, Constants::obsRadius, Constants::obsRadius, Constants:: obsRadius);
            }
        }

        if(is_found)
            found_poses.push_back(std::make_tuple(temp_state, StateDistance(temp_state,pivot_state),n));
    }

    //sort by pushing distance
    // Sort by the float value (second element of the pair) in ascending order
    std::sort(found_poses.begin(), found_poses.end(), [](const std::tuple<State, float, int>& a, const std::tuple<State, float, int>& b) {
        return std::get<1>(a) < std::get<1>(b);
    });

    //return obs
    env.add_obs(pivot_state);

    return found_poses;
}


// Comparison function to compare the float values in the tuples
bool compareTuples(const std::tuple<bool, float, int>& a, const std::tuple<bool, float, int>& b) {
    return std::get<1>(a) > std::get<1>(b); // Compare the second element (float)
}

void proposed_edge_construction(std::vector<movableObject>& mo_list, StatePtr pivot_state, StatePtr target_state, VertexPtr pivot_vertex, VertexPtr target_vertex,
                                size_t& state_ind, size_t& n, size_t& m, std::pair<pathType, reloDubinsPath>& dubins_res, Environment& env,
                                float& max_x, float& max_y, float& turning_radius, GraphPtr gPtr, preRelocList& preRelocs, graphTools::EdgeMatcher& edgeMatcher,
                                std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,
                                movableObject& pivot_mo, movableObject& target_mo, std::vector<stopWatch>& time_watches)
{

    // check collision
    stopWatch time_chk_col("chk_col", measurement_type::graphConst);
    auto validity = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, max_x, max_y, turning_radius);
    time_chk_col.stop();
    time_watches.push_back(time_chk_col);

    if(validity == StateValidity::valid)
    {
        Edge e;
        bool succ;

        // for debug only
        auto name1 = graphTools::getVertexName(*pivot_vertex,gPtr);
        auto name2 = graphTools::getVertexName(*target_vertex,gPtr);

        std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
        // add to edge-path matcher
        edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));
    }
    else if(validity == StateValidity::collision)
    {
        if(params::print_log)
            std::cout << " dubins collision" << std::endl;

        // store to failed paths
        failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
    }
    else if(validity == StateValidity::out_of_boundary)
    {
        // check if a long path is possible by relocating it
        stopWatch time_ofb1("ofb1", measurement_type::graphConst);
        auto unit_vecs = pivot_mo.get_push_unitvecs();

        float d_current_sq = powf(target_state->x - pivot_state->x,2) + powf(target_state->y - pivot_state->y,2);
        
        // find push indices that moves away from target
        std::vector<std::tuple<bool,float,size_t>> away_flags(pivot_mo.get_n_side());
        // MP
        for(size_t s=0; s<away_flags.size(); s++)
        {
            auto scaled_uvec = 0.1 * unit_vecs[s];
            float x_next = pivot_state->x + scaled_uvec.x();
            float y_next = pivot_state->y + scaled_uvec.y();

            float d_next_sq = powf(target_state->x - x_next,2) + powf(target_state->y - y_next,2);

            if(d_next_sq > d_current_sq)
                away_flags[s] = std::make_tuple(true,d_next_sq - d_current_sq,s);
            else
                away_flags[s] = std::make_tuple(false,-1,s); // todo: what if it can be pushed far
        }

        float d_current = sqrtf(d_current_sq);
        // find d_threshold to be a long path
        auto d_thres = get_longpath_d_thres(*pivot_state,*target_state);

        // if this is already a long path, no edge
        if(d_current > d_thres)
        {
            // store to failed paths
            failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
            return;
        }

        // distance needed (d_thres should be larger than d_current)
        float d_delta = d_thres - d_current;

        time_ofb1.stop();
        time_watches.push_back(time_ofb1);

        // vector with ture only
        std::vector<std::tuple<bool,float,size_t>> candidates_vec(0);
        /* it might be a good idea to try all directions */
        
        for(auto& it: away_flags)
        {
            if(std::get<0>(it))
                candidates_vec.push_back(it);
        }
        
        //candidates_vec = std::vector<std::tuple<bool,float,size_t>>(away_flags);

        // sort in decending order of distance difference
        std::sort(candidates_vec.begin(), candidates_vec.end(), compareTuples);

        bool push_found = false;
        Eigen::Vector2f push_thres;
        // check if pushing can make if feasible
        for(auto& it : candidates_vec)
        {
            stopWatch time_ofb2("ofb2", measurement_type::graphConst);
            // how much to push in this direction to meet d_delta
            /// rotate i.r.t. mo
            size_t push_ind = std::get<2>(it);
            float yaw = jeeho::convertEulerRange_to_pi(pivot_mo.get_pushing_poses()[push_ind]->yaw);
            Eigen::Matrix2f R;
            R << std::cos(yaw), -std::sin(yaw), std::sin(yaw),  std::cos(yaw);
            Eigen::Matrix2f R_inv = R.transpose();

            Eigen::Vector2f target_centered_at_pivot = {target_state->x - pivot_state->x, target_state->y - pivot_state->y};
            Eigen::Vector2f target_irt_pivot = R_inv * target_centered_at_pivot;

            float x_thres = sqrtf(powf(d_thres,2) - powf(target_irt_pivot.y(),2));
            float dist_to_push = x_thres - target_irt_pivot.x();

            Eigen::Vector2f push_vec = dist_to_push * unit_vecs[push_ind];

            State state_thres = State(pivot_state->x + push_vec.x(), pivot_state->y + push_vec.y(), pivot_state->yaw);

            // check if the state is valid
            auto is_valid = env.stateValid(state_thres);

            // check if prepush is valid
            State new_prepush = State(state_thres.x,state_thres.y,pivot_state->yaw);
            auto is_pre_push_valid = env.stateValid(find_pre_push(new_prepush, params::pre_push_dist));

            // check there is a path connecting preRelo to new pre-push
            // add/remove virtual obstacles

            // in previous version, motion was being planned here. this is modifed to be done in latter stage.
            //
            //bool is_path_to_prepush_valid = false;
            env.remove_obs(*pivot_state);
            // relocated
            env.add_obs(state_thres);

            time_ofb2.stop();
            time_watches.push_back(time_ofb2);

            State arrivalState(state_thres.x,state_thres.y,yaw);

            ///////////////////// change of order: don't plan motion here but leave it as a blank. cross it out when it failes during MP stage ///////////////////
            //stopWatch time_mp("pre_mp", measurement_type::pathPlan);
            //auto plan_res = planHybridAstar(find_pre_push(arrivalState), new_prepush, env, params::grid_search_timeout, false);
            //time_mp.stop();
            //time_watches.push_back(time_mp);
            //if(plan_res->success)
            //   is_path_to_prepush_valid = true;

            // revert obstacle
            env.add_obs(*pivot_state);
            env.remove_obs(state_thres);

            // if yes, go with it. if not, try next best
            //if(is_valid && is_pre_push_valid && is_path_to_prepush_valid)
            if(is_valid && is_pre_push_valid)
            {
                stopWatch time_pre_valid("pre_valid", measurement_type::graphConst);
                push_found = true;
                push_thres = push_vec;

                // note: not pre-push poses
                //preRelocs.push_back(std::make_pair(*pivot_mo.get_pushing_poses()[push_ind],state_thres));
                // find dubins
                auto dubins_pre = findDubins(*pivot_state,state_thres,turning_radius);

                State prePath_start = State(pivot_state->x,pivot_state->y,yaw);

                //leave motion plan as a blank placeholder for now. It will be handled later.
                //preReloPath prePath = preReloPath(prePath_start,state_thres,dubins_pre,plan_res);
                State arrivalStatePrepush = find_pre_push(arrivalState, params::pre_push_dist);
                preReloPath prePath = preReloPath(prePath_start,state_thres,dubins_pre,std::make_shared<PathPlanResult>(PathPlanResult(arrivalStatePrepush, new_prepush, *pivot_state, state_thres)));

                // cost for pre-relocations
                float pre_cost=0;
                for(auto& it : preRelocs)
                    pre_cost += sqrtf(powf(it.preReloDubins.targetState.x - it.preReloDubins.startState.x,2) + powf(it.preReloDubins.targetState.y - it.preReloDubins.startState.y,2));

                // find new dubins path
                auto new_dubins_res = is_good_path(state_thres,*target_state,turning_radius);

                Edge e;
                bool succ;
                std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, new_dubins_res.second.lengthCost() + pre_cost, *gPtr);
                // add to edge-path matcher
                edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,state_thres,*target_state,new_dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));

                time_pre_valid.stop();
                time_watches.push_back(time_pre_valid);

                break;
            }
        }   

        // if no candidates were possible, no edge
        if(!push_found)
        {
            if(params::print_log)
                std::cout << " no feasible path within boundary" << std::endl;
        }
    }
}





// newer version
// todo: merge with the older one
void proposed_edge_construction(movableObject& fromObj, movableObject& toObj, StatePtr pivot_state, StatePtr target_state, VertexPtr pivot_vertex, VertexPtr target_vertex,
                                size_t& state_ind, std::pair<pathType, reloDubinsPath>& dubins_res, Environment& env,
                                float& max_x, float& max_y, float& turning_radius, GraphPtr gPtr, preRelocList& preRelocs, graphTools::EdgeMatcher& edgeMatcher,
                                std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,
                                movableObject& pivot_mo, movableObject& target_mo,std::vector<stopWatch>& time_watches, bool is_delivery = false)
{
    // check collision
    stopWatch time_chk_col("chk_col", measurement_type::pathPlan);
    auto validity = check_collision(fromObj, toObj, pivot_state, pivot_vertex, state_ind, dubins_res, env, max_x, max_y, turning_radius, is_delivery);
    time_chk_col.stop();
    time_watches.push_back(time_chk_col);

    // for debug only
    auto name_pivot = graphTools::getVertexName(*pivot_vertex,gPtr);
    auto name_target = graphTools::getVertexName(*target_vertex,gPtr);

    // for debug only
    if(name_target == "d3_0")
        std::string dummy = ""; //conditional break doesn't work somehow 

    if(validity == StateValidity::valid) // todo: can it get better with pre-relocation?
    {
        Edge e;
        bool succ;

        std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
        // add to edge-path matcher
        edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));
    }
    else if(validity == StateValidity::collision)
    {
        if(params::print_log)
            std::cout << " dubins collision" << std::endl;

        // store to failed paths
        failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
    }
    //todo: add start/goal invalid
    else if(validity == StateValidity::out_of_boundary)
    {
        
        // check if a long path is possible by relocating the object it
        if(dubins_res.first == pathType::SP)
        {
            stopWatch time_ofb1("ofb1", measurement_type::graphConst);
            auto unit_vecs = pivot_mo.get_push_unitvecs();


            auto pre_relo_candidates = find_pre_relo_along_vector(env,*pivot_state,*target_state,turning_radius,unit_vecs);

            time_ofb1.stop();
            time_watches.push_back(time_ofb1);

            bool push_found = false;

            // check if pushing can make if feasible
            for(auto& it : pre_relo_candidates)
            {
                State pre_relo_state;
                float pre_relo_dist;
                int uvec_ind;
                std::tie(pre_relo_state,pre_relo_dist,uvec_ind) = it;

                stopWatch time_ofb2("ofb2", measurement_type::graphConst);

                // check collision of this pre-relocation
                auto pre_relo_validity = check_collision_straight(env,*pivot_state,pre_relo_state,pre_relo_dist,max_x,max_y,true);
                time_ofb2.stop();
                time_watches.push_back(time_ofb2);

                if(pre_relo_validity!=StateValidity::valid)
                {
                    // not an option
                    continue;
                }

                stopWatch time_dubins2("time_dubins2",measurement_type::pathPlan);
                // find new dubins path
                auto new_dubins_res = is_good_path(pre_relo_state,*target_state,turning_radius);

                //for test
                auto d_test_th = get_longpath_d_thres(pre_relo_state, *target_state);
                auto d_test = get_current_longpath_d(pre_relo_state,*target_state);

                // check if the path is valid
                //auto is_valid = env.stateValid(long_thres);
                //auto ofb_start = std::chrono::high_resolution_clock::now();
                auto chk = check_collision(fromObj, toObj, std::make_shared<State>(pre_relo_state), pivot_vertex, state_ind, new_dubins_res, 
                                            env, max_x, max_y, turning_radius, is_delivery);

                time_dubins2.stop();
                time_watches.push_back(time_dubins2);
                //auto ofb_end = std::chrono::high_resolution_clock::now();
                //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(ofb_end - ofb_start).count();
                //std::cout << "OFB: " <<  duration << " us" << std::endl;

                stopWatch time_valid_pre_push("time_validity", measurement_type::pathPlan);

                // check if prepush is valid
                State new_prepose = State(pre_relo_state.x,pre_relo_state.y,pivot_state->yaw);
                State new_prepush = find_pre_push(new_prepose, params::pre_push_dist); 

                //for test only. not part of the algorithm
                //auto test = env.stateValid(new_prepush);

                auto is_pre_push_valid = env.stateValid(new_prepush);                

                // check there is a path connecting relocation post-push to new pre-push
                // add/remove virtual obstacles
                //bool is_path_to_prepush_valid = false;
                env.remove_obs(*pivot_state);
                // relocated
                env.add_obs(pre_relo_state);

                float yaw = jeeho::convertEulerRange_to_pi(pivot_mo.get_pushing_poses()[uvec_ind]->yaw);
                State arrivalState(pre_relo_state.x, pre_relo_state.y, yaw);
                State relocation_postpush = find_pre_push(arrivalState, params::pre_push_dist);

                time_valid_pre_push.stop();
                time_watches.push_back(time_valid_pre_push);                

                // revert obstacle
                env.add_obs(*pivot_state);
                env.remove_obs(pre_relo_state);

                // if yes, go with it. if not, try next best
                
                //if(chk == StateValidity::valid && is_valid && is_pre_push_valid && is_path_to_prepush_valid)
                if(chk == StateValidity::valid && is_pre_push_valid)
                {
                    stopWatch time_pre_valid("pre_valid", measurement_type::graphConst);

                    push_found = true;

                    // note: not pre-push poses
                    // todo: handle multiple pre-relocations
                    //preRelocs.push_back(std::make_pair(*pivot_mo.get_pushing_poses()[push_ind],state_thres));

                    // find dubins
                    State preRelocStart(pivot_state->x, pivot_state->y, arrivalState.yaw);
                    //State preRelocStart_prepush = find_pre_push(preRelocStart);
                    auto dubins_pre = findDubins(preRelocStart,arrivalState,turning_radius);

                    // State prePath_start = State(pivot_state->x,pivot_state->y,yaw);
                    //preReloPath prePath = preReloPath(preRelocStart,arrivalState,dubins_pre,plan_res);
                    preReloPath prePath = preReloPath(preRelocStart,arrivalState,dubins_pre,std::make_shared<PathPlanResult>(PathPlanResult(relocation_postpush, new_prepush, *pivot_state, pre_relo_state)));
                    preRelocs.push_back(prePath);

                    // cost for pre-relocations
                    float pre_cost=0;
                    for(auto& it : preRelocs)
                        pre_cost += sqrtf(powf(it.preReloDubins.targetState.x - it.preReloDubins.startState.x,2) + powf(it.preReloDubins.targetState.y - it.preReloDubins.startState.y,2)); // assuming straight push per action

                    Edge e;
                    bool succ;
                    std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, new_dubins_res.second.lengthCost() + pre_cost, *gPtr);
                    // add to edge-path matcher
                    edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,new_dubins_res.second,
                                        preRelocs,new_dubins_res.first,e,gPtr));

                    time_pre_valid.stop();
                    time_watches.push_back(time_pre_valid);

                    break;
                }
            }   

            // if no candidates were possible, no edge
            if(!push_found)
            {
                if(params::print_log)
                    std::cout << " no feasible path within boundary" << std::endl;

                // todo: add to failed path
            }

            // if yes, add edge
            else
            {
                if(params::print_log)
                    std::cout << " make it feashible by pushing" << std::endl;
            }

        }
        else // not a short-path
        {
                // it is already a long-path
                
        }
        
    }
}


void reloPush::construct_edges(std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float max_x, float max_y, float turning_radius,
                             graphTools::EdgeMatcher& edgeMatcher,std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths, 
                             std::vector<stopWatch>& time_watches)
{
    edgeMatcher.reset();

    // todo: remove duplicate with proposed edge construction
    // only difference is from/to
    // construct edges
    for(size_t n=0; n<mo_list.size(); n++)
    {
        auto pivot_mo = mo_list[n];
        auto pivot_vslist = pivot_mo.get_vertex_state_list(); //the sides of an objects
        
        for(size_t piv_state_ind=0; piv_state_ind<pivot_vslist.size(); piv_state_ind++)
        {
            auto pivot_state = pivot_vslist[piv_state_ind].state;
            auto pivot_vertex = pivot_vslist[piv_state_ind].vertex;

            // for each movable object
            for(size_t m=0; m<mo_list.size(); m++)
            {
                if(n!=m) // not the same object
                {
                    // for each pushing side
                    for(size_t state_ind = 0; state_ind<mo_list[m].get_n_side(); state_ind++)
                    {
                        // checking if there needs an edge between pivot and target states
                        auto target_mo = mo_list[m];
                        auto target_state =  target_mo.get_vertex_state_list()[state_ind].state; // list of Vertex-State Pairs

                        if(params::print_log)
                            std::cout << "MO" << n << " dir" << piv_state_ind << " -> MO" << m << " dir" << state_ind << std::endl;

                        stopWatch time_dubins("time_dubins",measurement_type::pathPlan);
                        // check for good path
                        auto dubins_res = is_good_path(*pivot_state,*target_state,turning_radius);
                        time_dubins.stop();
                        time_watches.push_back(time_dubins);

                        // pre_reclocations
                        preRelocList preRelocs(0);

#pragma region proposed_path_classification
                        if(params::use_better_path)
                        {
                            auto target_vertex = mo_list[m].get_vertex_state_list()[state_ind].vertex;

                            proposed_edge_construction(pivot_mo,target_mo,pivot_state,target_state,pivot_vertex,target_vertex,
                                                            state_ind,dubins_res,env,max_x,max_y,turning_radius,
                                                            gPtr,preRelocs,edgeMatcher,failed_paths,pivot_mo,target_mo,time_watches,false);
                        }
                        
#pragma endregion proposed_path_classification

#pragma region normal_dubins_path
                        else // use ordinary dubins path
                        {
                            stopWatch time_coll("time_collision",measurement_type::pathPlan);
                            // check collision
                            auto validity = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, max_x, max_y, turning_radius);

                            time_coll.stop();
                            time_watches.push_back(time_coll);

                            if(validity == StateValidity::valid)
                            {
                                stopWatch time_edges("time_edges", measurement_type::graphConst);
                                auto target_vertex = mo_list[m].get_vertex_state_list()[state_ind].vertex;

                                Edge e;
                                bool succ;

                                // for debug only
                                auto pivot_name = graphTools::getVertexName(*pivot_vertex,gPtr);
                                auto target_name = graphTools::getVertexName(*target_vertex,gPtr);

                                std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
                                // add to edge-path matcher
                                edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,dubins_res.first,e,gPtr));
                                time_edges.stop();
                                time_watches.push_back(time_edges);
                            }
                            else
                            {
                                if(params::print_log)
                                    std::cout << " dubins collision" << std::endl;

                                // store to failed paths
                                failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
                            }
                            
                        }
#pragma endregion normal_dubins_path
                    }
                }
            }
        }
    }
}

void reloPush::add_deliveries(std::vector<movableObject>& delivery_list, std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env,
                                 float max_x, float max_y, float turning_radius, graphTools::EdgeMatcher& edgeMatcher,
                                 std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,std::vector<stopWatch>& time_watches, bool print_log)
{
    // for each delivery location
    for(size_t n=0; n<delivery_list.size(); n++)
    {
        movableObject target_mo = delivery_list[n];
        auto target_vslist = target_mo.get_vertex_state_list(); // all goal-poses and their vertices
        
        for(size_t t_state_ind=0; t_state_ind<target_vslist.size(); t_state_ind++) // for each goal pose
        {
            auto target_state = target_vslist[t_state_ind].state;
            auto target_vertex = target_vslist[t_state_ind].vertex;

            // for debug only
            auto target_name = graphTools::getVertexName(*target_vertex,gPtr);

            // for each movable object
            for(size_t m=0; m<mo_list.size(); m++)
            {
                // for each pushing side
                for(size_t state_ind = 0; state_ind<mo_list[m].get_n_side(); state_ind++)
                {
                    // checking if there needs an edge between pivot and target states
                    auto pivot_mo = mo_list[m];
                    auto pivot_state = pivot_mo.get_vertex_state_list()[state_ind].state; // list of Vertex-State Pairs
                    auto pivot_vertex = pivot_mo.get_vertex_state_list()[state_ind].vertex;

                    // for debug only
                    auto pivot_name = graphTools::getVertexName(*pivot_vertex,gPtr);

                    if(print_log)
                        std::cout << "MO" << n << " dir" << state_ind << " -> MO" << m << " dir" << t_state_ind << std::endl;

                    // check for good path
                    auto dubins_res = is_good_path(*pivot_state,*target_state,turning_radius);

                    // pre_reclocations
                    preRelocList preRelocs(0);

#pragma region proposed_path_classification
                    if(params::use_better_path)
                    {
                        //auto tstart = std::chrono::high_resolution_clock::now();
                        proposed_edge_construction(pivot_mo, target_mo, pivot_state, target_state, pivot_vertex, target_vertex, state_ind, dubins_res, env, max_x, max_y, turning_radius, gPtr,
                                                    preRelocs, edgeMatcher, failed_paths, pivot_mo, target_mo,time_watches, true);
                        //auto tend = std::chrono::high_resolution_clock::now();
                        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tend - tstart).count();
                        //std::cout << "propEdge: " <<  duration << " us" << std::endl;
                    }
#pragma endregion proposed_path_classification

#pragma region normal_dubins_path
                    else // use ordinary dubins path
                    {
                        stopWatch time_coll("time_collision",measurement_type::pathPlan);
                        // check collision
                        auto validity = check_collision(mo_list[m], delivery_list[n], pivot_state, pivot_vertex, state_ind, dubins_res, env, max_x, max_y, turning_radius, true);
                        time_coll.stop();
                        time_watches.push_back(time_coll);

                        if(validity == StateValidity::valid)
                        {
                            stopWatch time_edges("time_edges", measurement_type::graphConst);
                            //auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;

                            Edge e;
                            bool succ;

                            // test
                            auto tname = graphTools::getVertexName(*target_vertex, gPtr);

                            std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
                            // add to edge-path matcher
                            edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,dubins_res.first,e,gPtr));

                            time_edges.stop();
                            time_watches.push_back(time_edges);
                        }
                        else
                        {
                            if(print_log)
                                std::cout << " dubins collision" << std::endl;
                            
                            failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
                        }
                    }
#pragma endregion normal_dubins_path
                }
                
            }
        }
    }
}