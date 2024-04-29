#include <reloPush/edge_construction.h>

void reloPush::construct_edges(std::vector<movableObject>& mo_list, GraphPtr gPtr, Environment& env, float turning_radius, bool print_log)
{
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
                        if(dubins_res.first) // a good path is found. Check if there is a collision with other objects
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
                                //todo: add to an edge container
                            }       

                            //put back took-out obstacles
                            for(auto it : took_out)
                                env.add_obs(it);
                        }
                        // not a good path
                        else{ // either short-path or large-turn long-path
                            if(print_log)
                                std::cout << "no edge" << std::endl;
                            
                            //todo: add splited path
                        }
                    }
                }
            }
        }
    }
}