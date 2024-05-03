#include <reloPush/edge_construction.h>


bool check_collision(std::vector<movableObject>& mo_list, StatePtr pivot_state, VertexPtr pivot_vertex, size_t state_ind,
                    size_t n, size_t m, std::pair<pathType,dubinsPath> dubins_res, Environment& env, float turning_radius, bool print_log)
{
    // check collision
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(pivot_state->x, pivot_state->y);
    dubinsStart->setYaw(pivot_state->yaw);
    OmplState *interState = (OmplState *)dubinsSpace.allocState();
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = static_cast<int>(dubins_res.second.length()/0.1); //todo: get resolution as a param

    bool collision_found = false;

    // remove collision at starting pose
    auto took_out = env.takeout_start_collision(*pivot_state);

    // takeout pivot object from obstacles
    took_out.push_back(State(mo_list[n].x,mo_list[n].y,0));
    env.remove_obs(State(mo_list[n].x,mo_list[n].y,0));

    // takeout target object from obstacles
    took_out.push_back(State(mo_list[m].x,mo_list[m].y,0));
    env.remove_obs(State(mo_list[m].x,mo_list[m].y,0));

    // Interpolate dubins path to check for collision on grid map
    for (size_t np=0; np<num_pts; np++)
    {
        //auto start = std::chrono::steady_clock::now();
        jeeho_interpolate(dubinsStart, dubins_res.second, (double)np / (double)num_pts, interState, &dubinsSpace,
                        turning_radius);
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        //std::cout << "Elapsed time: " << duration << " usec" << std::endl;
        std::cout << interState->getX() << " " << interState->getY() << " " << interState->getYaw() << std::endl;
        if(env.stateValid(State(interState->getX(),interState->getY(),interState->getYaw()),0.15,0,0.15,0.15) == false) //todo: set better values
        {
            //collision found
            std::cout << "Collision found" << std::endl;
            collision_found = true;

            break;
        }                             
    }

    if(!collision_found)
    {
        if(print_log)
            std::cout << "edge found" << std::endl;
        //auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;
        //Edge e;
        //bool succ;
        
        //std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.length(), *gPtr);
        // add to edge-path matcher
        //edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,dubins_res.second,pathType::smLP,e,gPtr));                               
    }       

    //put back took-out obstacles
    for(auto it : took_out)
        env.add_obs(it);

    return collision_found;
}


void reloPush::construct_edges(std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float turning_radius, graphTools::EdgeMatcher& edgeMatcher, bool print_log)
{
    //todo: parameterize
    bool use_better_path = false;

    // construct edges
    for(size_t n=0; n<mo_list.size(); n++)
    {
        auto pivot_vslist = mo_list[n].vertex_state_list;
        
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
                    for(size_t state_ind = 0; state_ind<mo_list[m].n_side; state_ind++)
                    {
                        // checking if there needs an edge between pivot and target states
                        auto target_state = mo_list[m].vertex_state_list[state_ind].state; // list of Vertex-State Pairs

                        if(print_log)
                            std::cout << "MO" << n << " dir" << piv_state_ind << " -> MO" << m << " dir" << state_ind << std::endl;

                        // check for good path
                        auto dubins_res = is_good_path(*pivot_state,*target_state,turning_radius);

#pragma region proposed_path_classification
                        if(use_better_path)
                        {
                            if(dubins_res.first == pathType::smallLP) // a good path is found. Check if there is a collision with other objects
                            {
                                // check collision
                                auto collision_found = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, turning_radius, print_log);

                                if(!collision_found)
                                {
                                    auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;

                                    Edge e;
                                    bool succ;

                                    std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.length(), *gPtr);
                                    // add to edge-path matcher
                                    edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,pathType::smallLP,e,gPtr));   
                                }
                                else // collision found
                                {
                                    std::cout << " Collision found on a good path " << std::endl;
                                }

                                /*
                                ompl::base::DubinsStateSpace dubinsSpace(turning_radius);
                                OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
                                dubinsStart->setXY(pivot_state->x, pivot_state->y);
                                dubinsStart->setYaw(pivot_state->yaw);
                                OmplState *interState = (OmplState *)dubinsSpace.allocState();
                                // auto path_g = generateSmoothPath(dubinsPath,0.1);

                                size_t num_pts = static_cast<int>(dubins_res.second.length()/0.1); //todo: get resolution as a param

                                bool collision_found = false;

                                // remove collision at starting pose
                                auto took_out = env.takeout_start_collision(*pivot_state);

                                // takeout pivot object from obstacles
                                took_out.push_back(State(mo_list[n].x,mo_list[n].y,0));
                                env.remove_obs(State(mo_list[n].x,mo_list[n].y,0));

                                // takeout target object from obstacles
                                took_out.push_back(State(mo_list[m].x,mo_list[m].y,0));
                                env.remove_obs(State(mo_list[m].x,mo_list[m].y,0));

                                // Interpolate dubins path to check for collision on grid map
                                for (size_t np=0; np<num_pts; np++)
                                {
                                    //auto start = std::chrono::steady_clock::now();
                                    jeeho_interpolate(dubinsStart, dubins_res.second, (double)np / (double)num_pts, interState, &dubinsSpace,
                                                    turning_radius);
                                    //auto end = std::chrono::steady_clock::now();
                                    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                                    //std::cout << "Elapsed time: " << duration << " usec" << std::endl;
                                    std::cout << interState->getX() << " " << interState->getY() << " " << interState->getYaw() << std::endl;
                                    if(env.stateValid(State(interState->getX(),interState->getY(),interState->getYaw()),0.15,0,0.15,0.15) == false) //todo: set better values
                                    {
                                        //collision found
                                        std::cout << "Collision found" << std::endl;
                                        collision_found = true;

                                        break;
                                        //todo: add splited path
                                    }                             
                                }

                                if(!collision_found)
                                {
                                    if(print_log)
                                        std::cout << "edge found" << std::endl;
                                    auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;
                                    Edge e;
                                    bool succ;
                                    
                                    std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.length(), *gPtr);
                                    // add to edge-path matcher
                                    edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,dubins_res.second,pathType::smLP,e,gPtr));                                
                                }       

                                //put back took-out obstacles
                                for(auto it : took_out)
                                    env.add_obs(it);
                                */
                                
                            }
                            // need to split
                            else if (dubins_res.first == pathType::largeLP || dubins_res.first == pathType::SP){ // either short-path or large-turn long-path
                                if(print_log)
                                    std::cout << "Short Path Case" << std::endl;
                                
                                //todo: add splited path
                            }
                            // not a path
                            else{
                                if(print_log)
                                    std::cout << "No Path Found" << std::endl;
                            }
                        }
#pragma endregion proposed_path_classification

#pragma region normal_dubins_path
                        else // use ordinary dubins path
                        {
                            // check collision
                            auto collision_found = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, turning_radius, print_log);

                            if(!collision_found)
                            {
                                auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;

                                Edge e;
                                bool succ;

                                std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.length(), *gPtr);
                                // add to edge-path matcher
                                edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,pathType::smallLP,e,gPtr));
                            }
                            else
                            {
                                if(print_log)
                                    std::cout << " dubins collision" << std::endl;
                            }
                        }
#pragma endregion normal_dubins_path
                    }
                }
            }
        }
    }
}

void reloPush::add_deliveries(std::vector<movableObject>& delivery_list, std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float turning_radius, graphTools::EdgeMatcher& edgeMatcher, bool print_log)
{
    //todo: parameterize
    bool use_better_path = false;

    // for each delivery location
    for(size_t n=0; n<delivery_list.size(); n++)
    {
        auto target_vslist = delivery_list[n].vertex_state_list; // all goal-poses and their vertices
        
        for(size_t t_state_ind=0; t_state_ind<target_vslist.size(); t_state_ind++) // for each goal pose
        {
            auto target_state = target_vslist[t_state_ind].state;
            auto target_vertex = target_vslist[t_state_ind].vertex;

            // for each movable object
            for(size_t m=0; m<mo_list.size(); m++)
            {
                // for each pushing side
                for(size_t state_ind = 0; state_ind<mo_list[m].n_side; state_ind++)
                {
                    // checking if there needs an edge between pivot and target states
                    auto pivot_state = mo_list[m].vertex_state_list[state_ind].state; // list of Vertex-State Pairs
                    auto pivot_vertex = mo_list[m].vertex_state_list[state_ind].vertex;

                    if(print_log)
                        std::cout << "MO" << n << " dir" << state_ind << " -> MO" << m << " dir" << t_state_ind << std::endl;

                    // check for good path
                    auto dubins_res = is_good_path(*pivot_state,*target_state,turning_radius);

#pragma region proposed_path_classification
                    if(use_better_path)
                    {
                        // todo
                    }
#pragma endregion proposed_path_classification

#pragma region normal_dubins_path
                    else // use ordinary dubins path
                    {
                        // check collision
                        auto collision_found = check_collision(mo_list, pivot_state, pivot_vertex, state_ind, n, m, dubins_res, env, turning_radius, print_log);

                        if(!collision_found)
                        {
                            auto target_vertex = mo_list[m].vertex_state_list[state_ind].vertex;

                            Edge e;
                            bool succ;

                            std::tie(e,succ) = boost::add_edge(*pivot_vertex, *target_vertex, dubins_res.second.length(), *gPtr);
                            // add to edge-path matcher
                            edgeMatcher.insert(e, graphTools::EdgePathInfo(*pivot_vertex,*target_vertex,*pivot_state,*target_state,dubins_res.second,pathType::smallLP,e,gPtr));
                        }
                        else
                        {
                            if(print_log)
                                std::cout << " dubins collision" << std::endl;
                        }
                    }
#pragma endregion normal_dubins_path
                }
                
            }
        }
    }
}