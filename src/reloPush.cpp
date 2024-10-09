

#include <reloPush/reloPush_tools.h>
#include <numeric>
//#include <reloPush/reloPush_tools.h>

//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>

ros::Publisher* vertex_marker_pub_ptr = nullptr;
ros::Publisher* edge_marker_pub_ptr = nullptr;
ros::Publisher* object_marker_pub_ptr = nullptr;
ros::Publisher* dubins_path_pub_ptr = nullptr;
ros::Publisher* failed_path_pub_ptr = nullptr;
ros::Publisher* delivery_marker_pub_ptr = nullptr;
//test
ros::Publisher* test_path_pub_ptr = nullptr;
ros::Publisher* text_pub_ptr = nullptr;
ros::Publisher* rviz_path_pub_ptr;

ros::Publisher* boundary_pub_ptr;

ros::Publisher* robot_pose_reset_ptr;

ros::Publisher *path_info_pub_ptr;

ros::NodeHandle* nh_ptr = nullptr;

std::string get_mode_name()
{
    if(params::use_mp_only)
        return "mp_only";
    else
    {
        if(params::use_better_path)
            return "proposed";
        else
            return "dubins_only";
    }
}

void save_StatePath_to_file(StatePath& path_in, std::string file_name)
{
    // Open a file in write mode
    std::ofstream file(file_name);

    if (file.is_open()) {
        for (const auto& state : path_in) {
            file << state.x << " " << state.y << " " << state.yaw << "\n";
        }
        file.close();
        std::cout << "File saved successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file." << std::endl;
    }
}

void save_StrVec_to_file(StrVec& strvec_in, std::string file_name)
{
    // Open a file in write mode
    std::ofstream file(file_name);

    if (file.is_open()) {
        for (const auto& s : strvec_in) {
            file << s << "\n";
        }
        file.close();
        std::cout << "File saved successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file." << std::endl;
    }
}

reloPlanResult reloLoop(std::unordered_set<State>& obs, std::vector<movableObject>& mo_list, std::vector<movableObject>& delivered_obs,
                Environment& env, float map_max_x, float map_max_y, GraphPtr gPtr, graphTools::EdgeMatcher& edgeMatcher,
                stopWatchSet& time_watches, std::vector<movableObject>& delivery_list, 
                strMap& delivery_table, 
                std::vector<std::shared_ptr<deliveryContext>>& delivery_contexts, std::vector<State>& robots, std::unordered_map<std::string,
                std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths)
{    
    std::vector<std::string> deliv_seq(0);
    std::vector<int> vec_num_interm_relocs(0), vec_num_pre_relocs(0);
    PathInfoList out_pathinfo = PathInfoList();

    while(mo_list.size()>0)
    //while(delivery_table.size()>0)
    {
        // set static obstacles from movable objects
        init_static_obstacles(obs, mo_list, delivered_obs);

        // reset env
        stopWatch time_init_env("init_env",measurement_type::updateEnv);
        env = Environment(map_max_x, map_max_y, obs, Constants::r_push, Constants::LF_push, false);
        time_watches.stop_and_append(time_init_env);

        // update graph
        update_graph(mo_list, gPtr);

        // print vertex table (name - index)
        // Print the table of vertex names and their numbers
        if(params::print_graph)
        {
            typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
            std::cout << "\nVertex Number\tVertex Name\n";
            std::cout << "-------------\t-----------\n";

            VertexIterator vi, vi_end;
            for (boost::tie(vi, vi_end) = vertices(*gPtr); vi != vi_end; ++vi) {
                std::cout << *vi << "\t\t" << graphTools::getVertexName(*vi,gPtr) << "\n";
            }
            std::cout << std::endl;
        }

        // genearte name matcher
        NameMatcher nameMatcher(mo_list);
        // clear edge matcher
        edgeMatcher.reset();

        // Baseline: hybrid Astar only
        if(params::use_mp_only)
        {                 
            for(auto& it : delivery_list)
                it.add_to_graph(gPtr);
            nameMatcher.addVertices(delivery_list);
            PathMatListPtr pathMatListPtr(new PathMatList);

            //Constants::switch_to_pushing();   
            //Environment env_push(map_max_x, map_max_y, env.get_obs(), Constants::r_push, false); //todo: get size fro env
            stopWatch time_cost("cost_mat",measurement_type::pathPlan); 
            // generate cost matrix by motion planning
            // plan with pushing max radius
            auto costMat_vertices_pairs = get_cost_mat_vertices_pair(delivery_table, nameMatcher, gPtr, env, true, pathMatListPtr);
            time_watches.stop_and_append(time_cost);
            // switch back to nonpushing for future use
            //Constants::switch_to_nonpushing();
            
            while(true)
            {
                stopWatch time_find_min("find_min", measurement_type::graphPlan);
                // make list of minimum path of each delivery
                std::vector<graphPlanResultPtr> min_list = find_min_from_mat(costMat_vertices_pairs);
                auto min_list_ind = find_min_path(min_list);
                time_watches.stop_and_append(time_find_min);

                if(min_list_ind == -1) // no solution
                {
                    // return failure //////
                    return reloPlanResult(false); // instance failed
                }
                // print path on graph
                print_graph_path(min_list[min_list_ind]->path, gPtr);
                
                // check feasibility (robot to pre-push)
                auto min_row = min_list[min_list_ind]->min_row;
                auto min_col = min_list[min_list_ind]->min_col;
                auto plan_push = pathMatListPtr->at(min_list_ind)[min_row][min_col];
                
                auto push_path = plan_push->getPath(true);
                // remove last for arrival
                // omit last steps for arrival
                push_path.erase(push_path.end() - 2, push_path.end());

                auto pre_push_pose = find_pre_push(plan_push->start_pose, params::pre_push_dist);

                stopWatch time_plan_app("plan_app",measurement_type::pathPlan);
                Environment env_nonpush(params::map_max_x, params::map_max_y,env.get_obs(),Constants::r_nonpush, Constants::LF_nonpush,true);
                auto plan_app = planHybridAstar(robots[0], pre_push_pose, env_nonpush, params::grid_search_timeout, false);
                time_watches.stop_and_append(time_plan_app);
                if(!plan_app->success)
                {
                    if(plan_app->validity == PlanValidity::start_inval)
                        std::cout << "\tStart Invalid" << std::endl;
                    // cross out and replan
                    costMat_vertices_pairs[min_list_ind].first->operator()(min_list[min_list_ind]->min_row, min_list[min_list_ind]->min_col) = std::numeric_limits<float>::infinity();
                    // repeat
                    Color::println("REPLAN", Color::RED,Color::BG_YELLOW);
                    continue;
                }
                else // feasible
                {
                    stopWatch time_interp("interpolation", measurement_type::pathPlan);
                    //env.add_obs(plan_push->goal_pose);

                    auto app_path = plan_app->getPath(true);
                    // get driving actions
                    auto driving_actions = get_driving_actions(plan_app);
                    // add pose-push pose
                    // todo: do it better
                    State post_push_app;
                    auto post_ind_app = find_post_push_ind(app_path,env);
                    if(app_path.size()>=std::abs(post_ind_app))
                        post_push_app = app_path.end()[post_ind_app];
                    //else
                    //    post_push = push_path[0];
                    //app_path.push_back(post_push_app);

                    // add arrival offset (controller may not accurately lead the robot to the arrival pose)
                    State force_arrival_state = find_pre_push(plan_push->nominal_goal_pose, params::pre_push_dist - 0.16);
                    push_path.push_back(force_arrival_state);

                    // add post-push pose
                    // todo: do it better
                    //env.changeLF(Constants::LF_nonpush);
                    State post_push;
                    int post_push_ind = find_post_push_ind(push_path,env);
                    if(push_path.size()>=std::abs(post_push_ind))
                        post_push = push_path.end()[post_push_ind-1];
                    //else
                    //    post_push = push_path[0];
                    push_path.push_back(post_push);
                    //env.changeLF(Constants::LF_push);

                    time_watches.stop_and_append(time_interp);


                    auto path_app = PathInfo(min_list[min_list_ind]->targetVertexName,moveType::app,plan_app->start_pose,plan_app->goal_pose,app_path,driving_actions);
                    auto path_push = PathInfo(min_list[min_list_ind]->targetVertexName,moveType::final,plan_push->start_pose,plan_push->goal_pose,push_path, std::vector<bool>(push_path.size(),true));
                    out_pathinfo.push_back(path_app);
                    out_pathinfo.push_back(path_push);

                    // update robot
                    robots[0] = path_push.path.back();

                    // update movable objects list
                    for(size_t o = 0; o<mo_list.size(); o++)
                    {
                        //auto test_var = min_list[min_list_ind]->object_name;
                        if(mo_list[o].get_name() == min_list[min_list_ind]->object_name)
                        {
                            //delivered_obs.push_back(mo_list[o]);
                            auto delivery_location_str = delivery_table[mo_list[o].get_name()];
                            delivered_obs.push_back(*nameMatcher.getObject(delivery_location_str));
                            mo_list.erase(mo_list.begin() + o);
                        }
                    }

                    if(mo_list.size() > 0) // reconstruct graph when it is not done
                    {
                        // update nameMatcher
                        nameMatcher = NameMatcher(mo_list);

                        // update delivery list
                        /// remove previously assigned delivery
                        /// delivery map has object name as the key
                        delivery_table.erase(min_list[min_list_ind]->object_name);
                    }

                    break;
                } // end iteration for remaining objects
            }
        } // end mp-only
        else
        {
            // construct edges between objects
            reloPush::construct_edges(mo_list, gPtr, env, map_max_x, map_max_y, Constants::r_push, edgeMatcher, failed_paths, time_watches.watches);
        
            // print edges
            if(params::print_graph)
                print_edges(gPtr);

            draw_paths(edgeMatcher,env,failed_paths,dubins_path_pub_ptr,failed_path_pub_ptr,Constants::r_push);

            // add deliverries to graph
            add_delivery_to_graph(delivery_list, mo_list, env, map_max_x, map_max_y, edgeMatcher, nameMatcher, failed_paths, gPtr, time_watches.watches);
            
            // print edges
            if(params::print_graph)
                print_edges(gPtr);
            
            relocationPairList relocPair; // for updating movable objects
            StatePath push_path;

            ReloPathResult reloPush_path = iterate_remaining_deliveries(env, push_path, delivery_table, robots, relocPair, 
                                                                        nameMatcher, edgeMatcher, time_watches.watches, gPtr);

            // store delivery set
            if(reloPush_path.is_succ){
                deliveryContext dSet(mo_list, relocPair, reloPush_path.from_obj->get_name(), reloPush_path.state_path);
                delivery_contexts.push_back(std::make_shared<deliveryContext>(dSet));
                // save to path info
                out_pathinfo.append(reloPush_path.pathInfoList);
            }
            else{ // Instance failed
                return reloPlanResult(false);
            }

            // add to delivery sequence
            //deliv_seq.push_back(min_list[min_list_ind]->object_name);
            deliv_seq.push_back(reloPush_path.from_obj->get_name());
            
            stopWatch time_update("update",measurement_type::updateEnv);        
            // update movable objects list
            /// move and remove
            update_mo_list(mo_list,relocPair);
            //mo_list.erase(mo_list.begin() + min_list[0]->delivery_ind); // fix
            for(size_t o = 0; o<mo_list.size(); o++)
            {
                if(mo_list[o].get_name() == reloPush_path.from_obj->get_name())
                {
                    //delivered_obs.push_back(mo_list[o]);
                    auto delivery_location_str = delivery_table[mo_list[o].get_name()];
                    delivered_obs.push_back(*nameMatcher.getObject(delivery_location_str));
                    mo_list.erase(mo_list.begin() + o);
                }
            }

            if(mo_list.size() > 0) // reconstruct graph when it is not done
            {
                // update nameMatcher
                nameMatcher = NameMatcher(mo_list);

                // update delivery list
                /// remove previously assigned delivery
                /// delivery map has object name as the key
                delivery_table.erase(reloPush_path.from_obj->get_name());
            }

            // update robot
            robots[0] = push_path.back();
            //robots[0].yaw *= -1;
            time_watches.stop_and_append(time_update,false);

            // count relocations
            vec_num_interm_relocs.push_back(reloPush_path.num_interm_reloc);
            vec_num_pre_relocs.push_back(reloPush_path.num_pre_reloc);
        }
    }

    return reloPlanResult(true, std::accumulate(vec_num_interm_relocs.begin(), vec_num_interm_relocs.end(), 0),
                            std::accumulate(vec_num_pre_relocs.begin(), vec_num_pre_relocs.end(), 0), deliv_seq, out_pathinfo);
}


int main(int argc, char **argv) 
{
    //const char* args[] = {"reloPush", "data_demo.txt", "0", "1", "proposed"};
    //argv = const_cast<char**>(args);
    //argc = 5;

    std::string data_file=""; int data_ind=0;
    handle_args(argc, argv, data_file, data_ind); 
    // print initial messages
    init_prompt(data_file, data_ind);

    ros::init(argc, argv, "reloPush");
    ros::NodeHandle nh;
    nh_ptr = &nh;
    // initialize publishers
    initialize_publishers(nh,params::use_mocap);

    // wait for debug attach
    ros::Duration(0.1).sleep();
    // occasionally, the first message gets ignored if pub starts after sub
    // ros::spinOnce();
    ros::Duration(0.1).sleep();
    if(!params::leave_log)
        ros::Duration(1.0).sleep();

#pragma region initialize_essential_data
    ///////// Init essential data holders /////////
    int num_push_sides = 4; // todo: parse as a param
    std::vector<movableObject> mo_list(0);
    std::vector<State> robots(0);
    // init empty delivery locations
    std::vector<movableObject> delivery_list(0);
    std::unordered_map<std::string,std::string> delivery_table;

    //generate graph
    GraphPtr gPtr(new Graph);
    // edge to path matcher
    graphTools::EdgeMatcher edgeMatcher;
    // initialize grid map
    std::unordered_set<State> obs;
    //State goal(0,0,0); // arbitrary goal
    Environment env(params::map_max_x, params::map_max_y, obs, Constants::r_push, Constants::LF_push, false);

    if(!params::use_testdata){
        // init objects and add to graph
        init_movable_objects(mo_list, num_push_sides);
        // init robots
        init_robots(robots, nh, params::use_mocap);
        // make delivery table
        delivery_table = init_delivery_table(delivery_list,mo_list,env,edgeMatcher,gPtr,num_push_sides);
    }
    else{ // parse from input file
        parse_from_input_file(data_file, data_ind, mo_list, robots, delivery_table, delivery_list);
        if(params::use_mocap)
        {
            robots.clear();
            init_robots(robots, nh, params::use_mocap);
        }
    }

    // save initital mo_list
    std::vector<movableObject> initMOList(mo_list);
    // init nameMatcher
    //NameMatcher initNameMatcher(mo_list);
    // list of each delivery set
    deliveryContextSet deliverySets;
    std::vector<movableObject> delivered_obs;
    // store failed paths
    std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>> failed_paths;
    // init failed paths map
    for(auto& it : initMOList)
        failed_paths.insert({it.get_name(),{}});

    // time measurements
    stopWatchSet timeWatches;
    /////////////////////////////////////////////
    
    ///////////////////////////////////////////////
#pragma endregion initialize_essential_data

    // initial visualization on RViz
    init_visualization(initMOList, delivery_list);

   stopWatch time_plan("All-planning", measurement_type::allPlan);
    /////////////// loop starts ///////////////
    reloPlanResult reloResult = reloLoop(obs, mo_list, delivered_obs, env, params::map_max_x, params::map_max_y, gPtr, edgeMatcher, timeWatches, 
            delivery_list, delivery_table, deliverySets.delivery_contexts, robots, failed_paths);
    //////////// loop ends ////////////

    timeWatches.stop_and_append(time_plan);

    // print measurements
    //timeWatches.print();

#pragma region hanle_results
    // gather measurements
    DataCollector dcol(timeWatches,reloResult.pathInfoList);
    dcol.pathInfoList.print_seq();

        // print results
    if(reloResult.is_succ)
    {
        std::cout << "Pre-relocations: " << dcol.pathInfoList.count_pre_relocations() << std::endl;
        std::cout << "Temp-relocations: " << dcol.pathInfoList.count_temp_relocations() << std::endl;
        std::cout << "Total-relocations: " << dcol.pathInfoList.count_total_relocations() << std::endl;
    }
    else
    {
        std::cout << "Instance Failed" << std::endl;
    }

    // generate final navigation path
    StatePath final_path = deliverySets.serializePath();


    if(params::use_mp_only)
        final_path = *reloResult.pathInfoList.serializedPath();
    auto navPath_ptr = statePath_to_navPath(final_path); 


    // for dev
    //save_StatePath_to_file(final_path, "old.txt");

    //StatePath new_final_path = *dcol.pathInfoList.serializedPath();
    //save_StatePath_to_file(new_final_path, "new.txt");

    //auto mode_str = *dcol.pathInfoList.serializedPathWithMode().second;
    //save_StrVec_to_file(mode_str,"mode");


    if(params::print_final_path)
    {
        std::cout << "== Final Path ==" << std::endl;
        for(auto& p : final_path)
            std::cout << p.x << "," << p.y << "," << p.yaw << std::endl;

        // to verify
        std::cout << std::endl;
        dcol.pathInfoList.print_path();

        // for verification
        std::cout << "pathinfo length: " << dcol.pathInfoList.total_path_length() << std::endl;
        std::cout << "outpath length: " << path_length(final_path) << std::endl;
    }

    //visualization_loop(gPtr, mo_list, delivery_list, nameMatcher, edgeMatcher, env, navPath_ptr, failed_paths, 10);

    float exec_time_in = -1;

    rviz_path_pub_ptr->publish(*navPath_ptr);
    ros::spinOnce();
    ros::Duration(0.01).sleep();

#pragma region visualize_push_path
/*
    while(ros::ok())
    {

        // visualize push paths separately
        std::vector<std::shared_ptr<nav_msgs::Path>> push_paths, non_push_paths;
        std::vector<ros::Publisher> push_path_publishers, non_push_path_publishers;

        int i=0;
        int j=0;

        for(auto& pathInfo : dcol.pathInfoList.paths)
        {
            if(pathInfo.is_pushing())
            {
                // parse to nav_msgs path
                push_paths.push_back(statePath_to_navPath(pathInfo.path));
                // make a publisher
                std::string topic_name = "/car/push_path_" + std::to_string(i);  // Create a unique topic name for each path
                push_path_publishers.push_back(nh.advertise<nav_msgs::Path>(topic_name, 10));
                i++;
            }
            else
            {
                // parse to nav_msgs path
                non_push_paths.push_back(statePath_to_navPath(pathInfo.path));
                // make a publisher
                std::string topic_name = "/car/non_push_path_" + std::to_string(j);  // Create a unique topic name for each path
                non_push_path_publishers.push_back(nh.advertise<nav_msgs::Path>(topic_name, 10));
                j++;
            }
        }
        std::cout << "n push: " << dcol.pathInfoList.paths[1].pathLength() << std::endl;
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        // publish the path
        for (int i = 0; i < push_paths.size(); ++i) {
            push_path_publishers[i].publish(*push_paths[i]);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        for (int i = 0; i < non_push_paths.size(); ++i) {
            non_push_path_publishers[i].publish(*non_push_paths[i]);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    */
    #pragma endregion
    

    if(!params::leave_log && reloResult.is_succ)
    {
        
        vis_loop(initMOList, edgeMatcher, env, failed_paths, delivery_list, navPath_ptr);
    }

    else if(params::measure_exec_time)
    {
        if(reloResult.is_succ)
        {
            //test_path_pub_ptr->publish(*navPath_ptr); // previous way
            

            // path and mode
            auto final_nav_path_str = dcol.pathInfoList.serializeAllinStr();
            std_msgs::String strmsg;
            strmsg.data = final_nav_path_str;
            path_info_pub_ptr->publish(strmsg);

            ros::spinOnce();
            ros::Duration(0.05).sleep();
            // todo: check mushr_rhc is running
            std::cout << "\twaiting for execution time" << std::endl;
            // wait for exec time return
            std::string exec_time_topic_name = "/car/execution_time";
            if(params::use_mocap)
                exec_time_topic_name = "/mushr2/execution_time";

            std_msgs::Float32ConstPtr msg = ros::topic::waitForMessage<std_msgs::Float32>(exec_time_topic_name);
            exec_time_in = msg->data;
            std::cout << "\tExecution Time: " << exec_time_in << std::endl;
        }
        else
        {
            // failed. skip execution
            exec_time_in = -1;
        }
    }

    // log
    if (params::leave_log == 1)
    {
        // jeeho::logger records(reloResult.is_succ, reloResult.num_of_reloc, reloResult.delivery_sequence, timeWatches, removeExtension(data_file), data_ind);
        auto file_wo_ext = removeExtension(data_file);
        auto mode_name = get_mode_name();
        if(params::use_mocap)
            mode_name += "_real";

        // jeeho::logger records(reloResult.is_succ, dcol.pathInfoList.count_total_relocations(), reloResult.delivery_sequence, timeWatches, file_wo_ext, data_ind, mode_name);

        jeeho::logger records(reloResult.is_succ, dcol, file_wo_ext, data_ind, mode_name,exec_time_in);

        // write to file
        records.write_to_file(std::string(CMAKE_SOURCE_DIR) + "/log/raw" + "/log_" + file_wo_ext + "_" + mode_name);
    } 

    else
        ros::Duration(0.01).sleep();






#pragma endregion

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
