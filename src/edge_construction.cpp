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

stateValidity check_validity_by_interpolation(OmplState* dubinsStart, OmplState* interState, std::pair<pathType,reloDubinsPath>& dubins_res,
                                                ompl::base::DubinsStateSpace& dubinsSpace,
                                                float& turning_radius, size_t num_pts, 
                                                Environment& env, float max_x, float max_y)
{
    stateValidity validity = stateValidity::valid;

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

        // check if within boundary
        if(!is_within_boundary(interState,0,0,max_x,max_y))
        {
            validity = stateValidity::out_of_boundary;
            break;
        }


        if(env.stateValid(State(interState->getX(),interState->getY(),interState->getYaw()),0.15,0,0.15,0.15) == false) //todo: set better values
        {
            //collision found
            if(params::print_log)
                std::cout << "Collision found " << np << "x: "<< interState->getX() << " y: " << interState->getY() << " yaw: " << interState->getYaw() << std::endl;
            validity = stateValidity::collision;

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
                       ompl::base::DubinsStateSpace& dubinsSpace, OmplState* dubinsStart, OmplState* interState)
{
    // Allocate and initialize the starting state
    dubinsStart = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(pivot_state->x, pivot_state->y);
    dubinsStart->setYaw(pivot_state->yaw);
    
    // Allocate and initialize the intermediate state
    interState = (OmplState *)dubinsSpace.allocState();
}

stateValidity check_collision(std::vector<movableObject>& mo_list, StatePtr pivot_state, VertexPtr pivot_vertex, size_t state_ind,
                    size_t n, size_t m, std::pair<pathType,reloDubinsPath> dubins_res, Environment& env, float max_x, float max_y, float turning_radius, bool is_delivery = false)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius); OmplState* dubinsStart, *interState = nullptr;
    init_dubins_state(turning_radius, pivot_state, dubinsSpace, dubinsStart, interState);
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = static_cast<int>(dubins_res.second.lengthCost()/(0.1*0.5)); //todo: get resolution as a param

    stateValidity validity = stateValidity::valid;
    
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

    if(validity == stateValidity::valid && params::print_log)
        std::cout << "edge found" << std::endl;

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);
    
    return validity;
}

stateValidity check_collision(movableObject fromObj, movableObject toObj, StatePtr pivot_state, VertexPtr pivot_vertex, size_t state_ind,
                    std::pair<pathType,reloDubinsPath> dubins_res, Environment& env, float max_x, float max_y, float turning_radius, bool is_delivery = false)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius); OmplState* dubinsStart, *interState = nullptr;
    init_dubins_state(turning_radius, pivot_state, dubinsSpace, dubinsStart, interState);
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = static_cast<int>(dubins_res.second.lengthCost()/(0.1*0.5)); //todo: get resolution as a param

    stateValidity validity = stateValidity::valid;
    
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

    if(validity == stateValidity::valid && params::print_log)
        std::cout << "edge found" << std::endl;         

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);

    return validity;
}

// Comparison function to compare the float values in the tuples
bool compareTuples(const std::tuple<bool, float, int>& a, const std::tuple<bool, float, int>& b) {
    return std::get<1>(a) > std::get<1>(b); // Compare the second element (float)
}

void proposed_edge_construction(std::vector<movableObject>& mo_list, StatePtr pivot_state, StatePtr target_state, VertexPtr pivot_vertex, VertexPtr target_vertex,
                                size_t& state_ind, size_t& n, size_t& m, std::pair<pathType, reloDubinsPath>& dubins_res, Environment& env,
                                float& max_x, float& max_y, float& turning_radius, GraphPtr gPtr, preRelocList& preRelocs, graphTools::EdgeMatcher& edgeMatcher,
                                std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,
                                movableObject& pivot_mo, movableObject& target_mo)
{
    // check collision
    auto validity = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, max_x, max_y, turning_radius);

    if(validity == stateValidity::valid)
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
    else if(validity == stateValidity::collision)
    {
        if(params::print_log)
            std::cout << " dubins collision" << std::endl;

        // store to failed paths
        failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
    }
    else if(validity == stateValidity::out_of_boundary)
    {
        // check if a long path is possible by relocating it
        
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
            auto is_pre_push_valid = env.stateValid(find_pre_push(new_prepush));

            // check there is a path connecting preRelo to new pre-push
            // add/remove virtual obstacles
            bool is_path_to_prepush_valid = false;
            env.remove_obs(*pivot_state);
            // relocated
            env.add_obs(state_thres);

            State arrivalState(state_thres.x,state_thres.y,yaw);
            auto plan_res = planHybridAstar(find_pre_push(arrivalState), new_prepush, env, false);
            if(plan_res->success)
               is_path_to_prepush_valid = true;

            // revert obstacle
            env.add_obs(*pivot_state);
            env.remove_obs(state_thres);

            // if yes, go with it. if not, try next best
            if(is_valid && is_pre_push_valid && is_path_to_prepush_valid)
            {
                push_found = true;
                push_thres = push_vec;

                // note: not pre-push poses
                //preRelocs.push_back(std::make_pair(*pivot_mo.get_pushing_poses()[push_ind],state_thres));
                // find dubins
                auto dubins_pre = findDubins(*pivot_state,state_thres,turning_radius);

                State prePath_start = State(pivot_state->x,pivot_state->y,yaw);
                preReloPath prePath = preReloPath(prePath_start,state_thres,dubins_pre,plan_res);

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
                                movableObject& pivot_mo, movableObject& target_mo, bool is_delivery = false)
{
    // check collision
    auto validity = check_collision(fromObj, toObj, pivot_state, pivot_vertex, state_ind, dubins_res, env, max_x, max_y, turning_radius, is_delivery);

    // for debug only
    auto name_pivot = graphTools::getVertexName(*pivot_vertex,gPtr);
    auto name_target = graphTools::getVertexName(*target_vertex,gPtr);

    // for debug only
    if(name_target == "d3_0")
        std::string dummy = ""; //conditional break doesn't work somehow 

    if(validity == stateValidity::valid) // todo: can it get better with pre-relocation?
    {
        Edge e;
        bool succ;

        std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
        // add to edge-path matcher
        edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));
    }
    else if(validity == stateValidity::collision)
    {
        if(params::print_log)
            std::cout << " dubins collision" << std::endl;

        // store to failed paths
        failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
    }
    else if(validity == stateValidity::out_of_boundary)
    {
        // check if a long path is possible by relocating the object it
        if(dubins_res.first == pathType::SP)
        {
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

            float d_current = sqrtf(d_current_sq); // normalized by turning radius
            // find d_threshold to be a long path
            auto d_thres = get_longpath_d_thres(*pivot_state,*target_state, turning_radius) * 1.001; // tie-break

            // if this is already a long path, no edge
            if(d_current > d_thres)
            {
                // store to failed paths
                failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
                return;
            }

            // distance needed (d_thres should be larger than d_current)
            float d_delta = d_thres - d_current;

            // vector with ture only
            std::vector<std::tuple<bool,float,size_t>> candidates_vec(0);
            /* todo: it might be a good idea to try all directions */
            
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
                // how much to push in this direction to meet d_delta
                /// rotate i.r.t. mo
                size_t push_ind = std::get<2>(it);
                float yaw = jeeho::convertEulerRange_to_pi(pivot_mo.get_pushing_poses()[push_ind]->yaw);
                Eigen::Matrix2f R;
                R << std::cos(yaw), -std::sin(yaw), std::sin(yaw),  std::cos(yaw);
                Eigen::Matrix2f R_inv = R.transpose();

                Eigen::Vector2f target_centered_at_pivot = {target_state->x - pivot_state->x, target_state->y - pivot_state->y};
                Eigen::Vector2f target_irt_pivot = R_inv * target_centered_at_pivot;

                // centered at target i.r.t. start
                Eigen::Vector2f start_c_target = -1*target_irt_pivot;

                float x_thres = sqrtf(powf(d_thres,2) - powf(start_c_target.y(),2));
                float dist_to_push = x_thres - start_c_target.x();

                Eigen::Vector2f push_vec = dist_to_push * unit_vecs[push_ind];

                // reloacted object pose
                State state_thres = State(pivot_state->x + push_vec.x(), pivot_state->y + push_vec.y(), pivot_state->yaw);

                // find new dubins path
                auto new_dubins_res = is_good_path(state_thres,*target_state,turning_radius);

                // check if the path is valid
                //auto is_valid = env.stateValid(state_thres);
                auto chk = check_collision(fromObj, toObj, std::make_shared<State>(state_thres), pivot_vertex, state_ind, new_dubins_res, 
                                            env, max_x, max_y, turning_radius, is_delivery);

                // check if the state is valid
                auto is_valid = env.stateValid(state_thres);

                // check if prepush is valid
                State new_prepush = State(state_thres.x,state_thres.y,pivot_state->yaw);
                auto is_pre_push_valid = env.stateValid(find_pre_push(new_prepush));

                // check there is a path connecting preRelo to new pre-push
                // add/remove virtual obstacles
                bool is_path_to_prepush_valid = false;
                env.remove_obs(*pivot_state);
                // relocated
                env.add_obs(state_thres);

                State arrivalState(state_thres.x,state_thres.y,yaw);
                auto plan_res =planHybridAstar(find_pre_push(arrivalState), new_prepush, env, false);
                if(plan_res->success)
                is_path_to_prepush_valid = true;

                // revert obstacle
                env.add_obs(*pivot_state);
                env.remove_obs(state_thres);

                // if yes, go with it. if not, try next best
                if(chk == stateValidity::valid && is_valid && is_pre_push_valid && is_path_to_prepush_valid)
                {
                    push_found = true;
                    push_thres = push_vec;

                    // note: not pre-push poses
                    // todo: handle multiple pre-relocations
                    //preRelocs.push_back(std::make_pair(*pivot_mo.get_pushing_poses()[push_ind],state_thres));

                    // find dubins
                    auto dubins_pre = findDubins(*pivot_state,state_thres,turning_radius);

                    State prePath_start = State(pivot_state->x,pivot_state->y,yaw);
                    preReloPath prePath = preReloPath(prePath_start,state_thres,dubins_pre,plan_res);

                    // cost for pre-relocations
                    float pre_cost=0;
                    for(auto& it : preRelocs)
                        pre_cost += sqrtf(powf(it.preReloDubins.targetState.x - it.preReloDubins.startState.x,2) + powf(it.preReloDubins.targetState.y - it.preReloDubins.startState.y,2));

                    Edge e;
                    bool succ;
                    std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, new_dubins_res.second.lengthCost() + pre_cost, *gPtr);
                    // add to edge-path matcher
                    edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,new_dubins_res.second,
                                        preRelocs,pathType::smallLP,e,gPtr));

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
                             graphTools::EdgeMatcher& edgeMatcher,std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths)
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

                        // check for good path
                        auto dubins_res = is_good_path(*pivot_state,*target_state,turning_radius);

                        // pre_reclocations
                        preRelocList preRelocs(0);

#pragma region proposed_path_classification
                        if(params::use_better_path)
                        {
                            /*
                            // check collision
                            auto validity = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, max_x, max_y, turning_radius);

                            if(validity == stateValidity::valid)
                            {
                                auto target_vertex = mo_list[m].get_vertex_state_list()[state_ind].vertex;

                                Edge e;
                                bool succ;

                                // for debug only
                                auto name1 = graphTools::getVertexName(*pivot_vertex,gPtr);
                                auto name2 = graphTools::getVertexName(*target_vertex,gPtr);

                                std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
                                // add to edge-path matcher
                                edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));
                                
                               
                            }
                            else
                            {
                                if(params::print_log)
                                    std::cout << " dubins collision" << std::endl;

                                // store to failed paths
                                failed_paths[graphTools::getVertexName(*pivot_vertex,gPtr)].push_back(std::make_pair(pivot_state,dubins_res.second));
                            }
                            */
                            auto target_vertex = mo_list[m].get_vertex_state_list()[state_ind].vertex;

                            proposed_edge_construction(pivot_mo,target_mo,pivot_state,target_state,pivot_vertex,target_vertex,
                                                            state_ind,dubins_res,env,max_x,max_y,turning_radius,
                                                            gPtr,preRelocs,edgeMatcher,failed_paths,pivot_mo,target_mo,false);
                        }
                        
#pragma endregion proposed_path_classification

#pragma region normal_dubins_path
                        else // use ordinary dubins path
                        {
                            // check collision
                            auto validity = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, max_x, max_y, turning_radius);

                            if(validity == stateValidity::valid)
                            {
                                auto target_vertex = mo_list[m].get_vertex_state_list()[state_ind].vertex;

                                Edge e;
                                bool succ;

                                // for debug only
                                auto name1 = graphTools::getVertexName(*pivot_vertex,gPtr);
                                auto name2 = graphTools::getVertexName(*target_vertex,gPtr);

                                std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
                                // add to edge-path matcher
                                edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));
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
                                 std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths, bool print_log)
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
                        proposed_edge_construction(pivot_mo, target_mo, pivot_state, target_state, pivot_vertex, target_vertex, state_ind, dubins_res, env, max_x, max_y, turning_radius, gPtr,
                                                    preRelocs, edgeMatcher, failed_paths, pivot_mo, target_mo, true);
                    }
#pragma endregion proposed_path_classification

#pragma region normal_dubins_path
                    else // use ordinary dubins path
                    {
                        // check collision
                        auto validity = check_collision(mo_list[m], delivery_list[n], pivot_state, pivot_vertex, state_ind, dubins_res, env, max_x, max_y, turning_radius, true);

                        if(validity == stateValidity::valid)
                        {
                            //auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;

                            Edge e;
                            bool succ;

                            // test
                            auto tname = graphTools::getVertexName(*target_vertex, gPtr);

                            std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.lengthCost(), *gPtr);
                            // add to edge-path matcher
                            edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,preRelocs,pathType::smallLP,e,gPtr));
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