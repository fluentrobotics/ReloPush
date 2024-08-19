

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

ros::Publisher* boundary_pub_ptr;

ros::Publisher* robot_pose_reset_ptr;


ros::NodeHandle* nh_ptr = nullptr;

/*
void visualization_loop(GraphPtr gPtr, std::vector<movableObject>& mo_list, std::vector<movableObject>& delivery_list
                        , NameMatcher& nameMatcher, graphTools::EdgeMatcher edgeMatcher, Environment& env, std::shared_ptr<nav_msgs::Path> navPath_ptr,
                         std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths, double loop_rate=10)
{
    ros::Rate r(loop_rate);
    // visualize vertices
    auto graph_vis_pair = visualize_graph(*gPtr, nameMatcher, vertex_marker_pub_ptr,edge_marker_pub_ptr);
    // visualize movable obstacles
    auto mo_vis = draw_obstacles(mo_list, object_marker_pub_ptr);
    // visualize edge paths
    auto vis_path_msg = draw_paths(edgeMatcher,env,failed_paths,dubins_path_pub_ptr,failed_path_pub_ptr,Constants::r);
    // visualize delivery locations
    auto vis_deli_msg = draw_deliveries(delivery_list,delivery_marker_pub_ptr);
    // visualize object names
    auto vis_names_msg = draw_texts(mo_list,text_pub_ptr);
    // publish final path once
    test_path_pub_ptr->publish(*navPath_ptr); 

    while(ros::ok())
    {
        dubins_path_pub_ptr->publish(vis_path_msg);
        vertex_marker_pub_ptr->publish(graph_vis_pair.first);
        edge_marker_pub_ptr->publish(graph_vis_pair.second);
        delivery_marker_pub_ptr->publish(vis_deli_msg);
        object_marker_pub_ptr->publish(mo_vis);

        //test
        //test_path_pub_ptr->publish(*navPath_ptr);

        //object names
        text_pub_ptr->publish(vis_names_msg);

        //testing
        //auto robotRt = get_real_robotPose(*nh_ptr);
        //std::cout << robotRt.first.z() << std::endl;

        ros::spinOnce();
        r.sleep();
    }
}
*/



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
        env = Environment(map_max_x, map_max_y, obs);

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

        // Baseline: hybrid Astar only
        if(params::use_mp_only)
        {
            // generate cost matrix by motion planning

            // plan with pushing max radius
            Constants::switch_to_pushing();

            //for each entry in delivery table
            for (auto i = delivery_table.begin(); i != delivery_table.end(); i++)
            {
                std::cout << i->first << " -> " << i->second << std::endl;
                auto pivot_obj = nameMatcher.getObject(i->first);
                auto target_obj = nameMatcher.getObject(i->second);

                auto pivot_names = pivot_obj->get_vertex_names();
                auto target_names = target_obj->get_vertex_names();

                const int rows = pivot_obj->get_n_side();
                const int cols = target_obj->get_n_side();

                // init cost table
                MatType cost_mat(rows,cols);
                // paths
                VertexPath paths(rows);

                for(int row=0; row<rows; row++)
                {
                    paths[row].resize(cols);
                    for(int col=0; col<cols; col++)
                    {                
                        std::string start_name = pivot_names[row]; // by row
                        std::string target_name = target_names[col]; // by col

                        auto start_pose = pivot_obj->get_pushing_poses()[row];
                        auto goal_pose = target_obj->get_pushing_poses()[col];

                        auto plan_res = planHybridAstar(*start_pose,*goal_pose,env,0,false);

                        //paths[row][col] = res.first;
                        //cost_mat(row,col) = res.second;
                    }
                }

                //std::cout << cost_mat << std::endl;

                // store in matrix vector
                //ObjectCostMat mat_pair(std::make_shared<MatType>(cost_mat),ObjectPairPath(pivot_obj,target_obj,std::make_shared<VertexPath>(paths)));
                //out_mat_vec.push_back(mat_pair);
            }

            // switch back to nonpushing for future use
            Constants::switch_to_nonpushing();

            // find minimum cost delivery

            // update and repeat
        }

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
        
        relocationPair_list relocPair; // for updating movable objects
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
        
        stopWatch time_update("update");        
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

    return reloPlanResult(true, std::accumulate(vec_num_interm_relocs.begin(), vec_num_interm_relocs.end(), 0),
                            std::accumulate(vec_num_pre_relocs.begin(), vec_num_pre_relocs.end(), 0), deliv_seq, out_pathinfo);
}


int main(int argc, char **argv) 
{
    // const char* args[] = { "program_name", "data_3o_2.txt", "2", "0" };
    // argv = const_cast<char**>(args);
    // argc = 4;

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
    Environment env(params::map_max_x, params::map_max_y, obs);

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
        std::cout << "Instance Failed" << std::endl;

    // log
    if(params::leave_log == 1)
    {
        //jeeho::logger records(reloResult.is_succ, reloResult.num_of_reloc, reloResult.delivery_sequence, timeWatches, removeExtension(data_file), data_ind);
        jeeho::logger records(reloResult.is_succ, dcol.pathInfoList.count_total_relocations(), reloResult.delivery_sequence, timeWatches, removeExtension(data_file), data_ind);
        // write to file
        if(params::use_better_path)
            records.write_to_file(std::string(CMAKE_SOURCE_DIR) + "/log/raw" + "/log_" + removeExtension(data_file) + ".txt");
        else
            records.write_to_file(std::string(CMAKE_SOURCE_DIR) + "/log/raw" + "/log_" + removeExtension(data_file) + "_dubins_only.txt");
    }

    // generate final navigation path
    StatePath final_path = deliverySets.serializePath();
    auto navPath_ptr = statePath_to_navPath(final_path);

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

    if(!params::leave_log)
        vis_loop(initMOList, edgeMatcher, env, failed_paths, delivery_list, navPath_ptr);
    
    else
        ros::Duration(0.01).sleep();

#pragma endregion

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
