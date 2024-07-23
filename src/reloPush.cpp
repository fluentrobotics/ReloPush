

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
                std::vector<stopWatch>& time_watches, std::vector<movableObject>& delivery_list, 
                std::unordered_map<std::string,std::string>& delivery_table, 
                std::vector<std::shared_ptr<deliveryContext>>& delivery_contexts, std::vector<State>& robots, std::unordered_map<std::string,
                std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths)
{

    std::vector<size_t> temp_relocs(0);
    std::vector<std::string> deliv_seq(0);

    // count number of pre-relocations
    size_t count_pre_relocs = 0;

    while(mo_list.size()>0)
    {
        // set static obstacles from movable objects
        init_static_obstacles(obs, mo_list, delivered_obs);

        // reset env
        env = Environment(map_max_x, map_max_y, obs);

        // update graph
        update_graph(mo_list, gPtr);

        // print vertex table (name - index)
        // Print the table of vertex names and their numbers
        typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
        std::cout << "\nVertex Number\tVertex Name\n";
        std::cout << "-------------\t-----------\n";

        VertexIterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(*gPtr); vi != vi_end; ++vi) {
            std::cout << *vi << "\t\t" << graphTools::getVertexName(*vi,gPtr) << "\n";
        }
        std::cout << std::endl;

        // genearte name matcher
        NameMatcher nameMatcher(mo_list);

        stopWatch time_edge("edge", measurement_type::graphPlan);
        // construct edges between objects
        reloPush::construct_edges(mo_list, gPtr, env, map_max_x, map_max_y, Constants::r, edgeMatcher, failed_paths);
        time_edge.stop();
        time_edge.print_us();
        time_watches.push_back(time_edge);


        // test: verify objects graph first
        // print edges
        if(params::print_graph)
            print_edges(gPtr);

        draw_paths(edgeMatcher,env,failed_paths,dubins_path_pub_ptr,failed_path_pub_ptr,Constants::r);

        // add deliverries to graph
        stopWatch time_edge_d("delivery", measurement_type::graphPlan);
        add_delivery_to_graph(delivery_list, mo_list, env, map_max_x, map_max_y, edgeMatcher, nameMatcher, failed_paths, gPtr);
        time_edge_d.stop();
        time_edge_d.print_us();
        time_watches.push_back(time_edge_d);
        
        // print edges
        if(params::print_graph)
            print_edges(gPtr);

        // traverse on graph
        pathFinder pf; // todo: make it static
        // find best push traverse for all assignments
        stopWatch time_assign("assignment", measurement_type::assign);
        auto min_list = find_min_cost_seq(delivery_table,nameMatcher,gPtr);


        /// remaining ones ////

        // find non-inf ind
        int min_list_ind = -1;
        for(int i=0; i<min_list.size(); i++)
        {
            if(min_list[i]->cost != std::numeric_limits<float>::infinity())
            {
                // todo: find one with lowest cost
                min_list_ind = i;
                break;
            }
        }

        // at least one object cannot be delivered
        if(min_list_ind == -1)
        {
            // failed
            Color::println("At least one object cannot be delivered",Color::BG_RED,Color::BG_YELLOW);
            return reloPlanResult(false);
        }


        time_assign.stop();
        time_watches.push_back(time_assign);

        // choose delivery with lowest cost       
        pf.printPath(gPtr, min_list[min_list_ind]->path);
        //pf.printPath(gPtr, min_list[1]->path);
        auto reloc_objects = get_intermediate_objects(min_list[min_list_ind]->path, nameMatcher);
        // add to num of reloc
        temp_relocs.push_back(reloc_objects.size());

        // count number of pre-relocations
        size_t num_prereloc = 0;

        stopWatch time_path_gen_push_path("push-path", measurement_type::pathPlan);
        // path segments for relocation
        // final pushing
        auto push_path = get_push_path(min_list[min_list_ind]->path, edgeMatcher, gPtr, num_prereloc);
        time_path_gen_push_path.stop();
        time_watches.push_back(time_path_gen_push_path);

        // count
        count_pre_relocs += num_prereloc;

        // relocation paths
        pathsPtr relo_paths;
        relocationPair_list relocPair; // for updating movable objects

        stopWatch time_path_gen_relo_path("relocate", measurement_type::relocatePlan);
        std::tie(relo_paths, relocPair) = find_relo_path(push_path, reloc_objects, env); // pre-relocation path
        time_path_gen_push_path.stop();
        time_watches.push_back(time_path_gen_push_path);

        stopWatch time_path_gen_comb_path("combine", measurement_type::pathPlan);
        // combined path for the delivery
        auto reloPush_path = combine_relo_push(push_path, *relo_paths, robots[0], env, reloc_objects);
        time_path_gen_comb_path.stop();
        time_watches.push_back(time_path_gen_comb_path);
        
        if(!reloPush_path.second) // hybrid astar failed
        {
            Color::println("Hybrid Astar Failed", Color::RED, Color::BG_YELLOW);
            return reloPlanResult(false); // todo: don't return unless this is the only one left
        }

        ///////////////////

        //auto navPath_ptr = statePath_to_navPath(reloPush_path, use_mocap);

        // store delivery set
        deliveryContext dSet(mo_list, relocPair, min_list[min_list_ind]->object_name, std::make_shared<statePath>(reloPush_path.first));
        delivery_contexts.push_back(std::make_shared<deliveryContext>(dSet));

        // add to delivery sequence
        deliv_seq.push_back(min_list[min_list_ind]->object_name);
        
        stopWatch time_update("update");        
        // update movable objects list
        /// move and remove
        update_mo_list(mo_list,relocPair);
        //mo_list.erase(mo_list.begin() + min_list[0]->delivery_ind); // fix
        for(size_t o = 0; o<mo_list.size(); o++)
        {
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

        // update robot
        robots[0] = push_path.back();
        //robots[0].yaw *= -1;
        time_update.stop();
        time_watches.push_back(time_update);    
    }

    return reloPlanResult(true, std::accumulate(temp_relocs.begin(), temp_relocs.end(), 0),count_pre_relocs, deliv_seq);
}

void init_visualization(std::vector<movableObject>& initMOList, std::vector<movableObject>& delivery_list)
{
    draw_obstacles(initMOList, object_marker_pub_ptr);
    draw_texts(initMOList,text_pub_ptr);
    draw_deliveries(delivery_list,delivery_marker_pub_ptr);
    visualize_workspace_boundary(params::map_max_x, params::map_max_y, boundary_pub_ptr);
}

void parse_from_input_file(std::string& data_file, int& data_ind, std::vector<movableObject>& mo_list,
                            std::vector<State>& robots, std::unordered_map<std::string,std::string>& delivery_table,
                            std::vector<movableObject>& delivery_list)
{
    // parse from input file
    Color::println("=== Testdata " + data_file + " " + std::to_string(data_ind) + " ===",Color::CYAN, Color::BG_DEFAULT);
    delivery_table = parse_movableObjects_robots_from_file(mo_list,robots,delivery_list,data_ind,std::string(CMAKE_SOURCE_DIR) + "/testdata/" + data_file);
    
    if(params::reset_robot_pose)
    {
        // reset robot initpose on sim
        geometry_msgs::PoseWithCovarianceStamped init_pose;
        init_pose.pose.pose.position.x = robots[0].x;
        init_pose.pose.pose.position.y = robots[0].y;

        // angle in -pi ~ pi range
        float yaw = robots[0].yaw;
        yaw = jeeho::convertEulerRange_to_pi(yaw);
        //to quaternion
        auto yaw_euler = Eigen::Vector3f(0,0,yaw);
        auto yaw_quat = jeeho::euler_to_quaternion_zyx(yaw_euler);
        init_pose.pose.pose.orientation.w = yaw_quat.w();
        init_pose.pose.pose.orientation.x = yaw_quat.x();
        init_pose.pose.pose.orientation.y = yaw_quat.y();
        init_pose.pose.pose.orientation.z = yaw_quat.z();

        robot_pose_reset_ptr->publish(init_pose);
        ros::spinOnce();
        if(!params::leave_log)
            ros::Duration(0.5).sleep();
    }
}

void handle_args(int argc, char **argv, std::string& data_file, int& data_ind)
{
    data_ind = 21;
    //std::string data_file = "data_3o_2.txt";
    data_file = "data_3o_2.txt";
    
    //int leave_log = 1;
    if(argc==4)
    {
        data_file = std::string(argv[1]);
        data_ind = std::atoi(argv[2]);        
        params::leave_log = std::atoi(argv[3]);
    }
}

void init_prompt(std::string& data_file, int& data_ind)
{
    Color::println("== data: " + data_file + " ===",Color::BG_YELLOW);
    Color::println("== better path: " + std::to_string(params::use_better_path) + " ===",Color::BG_YELLOW);
    Color::println("== ind: " + std::to_string(data_ind) + " ===",Color::BG_YELLOW);
    Color::println("== log: " + std::to_string(params::leave_log) + " ===",Color::BG_YELLOW);
    Color::println("== post-push: " + std::to_string(params::post_push_ind) + " ===",Color::BG_YELLOW);
}

void vis_loop(std::vector<movableObject>& initMOList, graphTools::EdgeMatcher& edgeMatcher, Environment& env,
              std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths,
              std::vector<movableObject>& delivery_list, std::shared_ptr<nav_msgs::Path> navPath_ptr)
{
    // publish final path once
    test_path_pub_ptr->publish(*navPath_ptr);
    while(ros::ok())
    {
        // visualize movable obstacles
        auto mo_vis = draw_obstacles(initMOList, object_marker_pub_ptr);
        // visualize edge paths
        auto vis_path_msg = draw_paths(edgeMatcher,env,failed_paths,dubins_path_pub_ptr,failed_path_pub_ptr,Constants::r);
        // visualize delivery locations
        auto vis_deli_msg = draw_deliveries(delivery_list,delivery_marker_pub_ptr);
        // visualize object names
        auto vis_names_msg = draw_texts(initMOList,text_pub_ptr);

        ros::spinOnce();
        ros::Duration(0.5).sleep();                
    }
}

int main(int argc, char **argv) 
{
    /*
    int data_ind = 0;
    //std::string data_file = "data_3o_2.txt";
    std::string data_file = "data_4o.txt";
    
    //int leave_log = 1;
    if(argc==4)
    {
        data_file = std::string(argv[1]);
        data_ind = std::atoi(argv[2]);        
        params::leave_log = std::atoi(argv[3]);
    }
    */
    std::string data_file=""; int data_ind=0;
    handle_args(argc, argv, data_file, data_ind); 
    // print initial messages
    init_prompt(data_file, data_ind);

    ros::init(argc, argv, "reloPush");
    ros::NodeHandle nh;
    nh_ptr = &nh;
    // initialize publishers
    initialize_publishers(nh);

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
    stopWatch time_plan("All-planning", measurement_type::allPlan);
    ///////////////////////////////////////////////
#pragma endregion initialize_essential_data


    // initial visualization on RViz
    init_visualization(initMOList, delivery_list);
   
    /////////////// loop starts ///////////////
    reloPlanResult reloResult = reloLoop(obs, mo_list, delivered_obs, env, params::map_max_x, params::map_max_y, gPtr, edgeMatcher, timeWatches.watches, 
            delivery_list, delivery_table, deliverySets.delivery_contexts, robots, failed_paths);
    //////////// loop ends ////////////

    time_plan.stop();
    time_plan.print_us();
    timeWatches.watches.push_back(time_plan);

    // print measurements
    //timeWatches.print();

        // print results
    std::cout << "Pre-relocations: " << reloResult.num_of_prereloc << std::endl;

    // log
    if(params::leave_log == 1)
    {
        jeeho::logger records(reloResult.is_succ, reloResult.num_of_reloc, reloResult.delivery_sequence, timeWatches, removeExtension(data_file), data_ind);
        // write to file
        records.write_to_file(std::string(CMAKE_SOURCE_DIR) + "/log/raw" + "/log_" + removeExtension(data_file) + ".txt");
    }

    // generate final navigation path
    statePath final_path = deliverySets.serializePath();
    auto navPath_ptr = statePath_to_navPath(final_path);

    if(params::print_final_path)
    {
        std::cout << "== Final Path ==" << std::endl;
        for(auto& p : final_path)
            std::cout << p.x << "," << p.y << "," << p.yaw << std::endl;
    }

    //visualization_loop(gPtr, mo_list, delivery_list, nameMatcher, edgeMatcher, env, navPath_ptr, failed_paths, 10);

    if(!params::leave_log)
        vis_loop(initMOList, edgeMatcher, env, failed_paths, delivery_list, navPath_ptr);
    
    else
        ros::Duration(0.01).sleep();

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
