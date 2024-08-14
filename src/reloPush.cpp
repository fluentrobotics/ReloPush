

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

int find_min_path(std::vector<graphPlanResultPtr>& min_list)
{
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
        //return reloPlanResult(false);
    }
    return min_list_ind;
}

void print_graph_path(std::vector<std::size_t>& path, GraphPtr gPtr)
{
    // traverse on graph
    pathFinder pf; // todo: make it static
    // choose delivery with lowest cost       
    pf.printPath(gPtr, path);
}

ReloPathResult iterate_remaining_deliveries(Environment& env, StatePath& push_path, strMap& delivery_table,
                                                                std::vector<State>& robots, relocationPair_list& relocPair, NameMatcher& nameMatcher, 
                                                                graphTools::EdgeMatcher& edgeMatcher,std::vector<stopWatch>& time_watches, GraphPtr gPtr)
{
    stopWatch time_search("cost_mat", measurement_type::graphPlan);
    auto costMat_vertices_pairs = get_cost_mat_vertices_pair(delivery_table, nameMatcher, gPtr);
    time_search.stop();
    time_watches.push_back(time_search);

    int min_list_ind = -1;

    size_t min_cost_row, min_cost_col;

    StatePathPtr out_path = nullptr;

    // path info list
    PathInfoList final_pathinfo = PathInfoList();

    int num_temp_relocs=0, num_pre_relocs=0;

    while(true)
    {
        // find best push traverse for all assignments
        //stopWatch time_assign("assignment", measurement_type::assign);
        
        // minimum row/col for each remaining delivery
        //auto min_list = find_min_cost_seq(delivery_table,nameMatcher,gPtr);
        
        stopWatch time_search_min("min_search", measurement_type::graphPlan);
        // make list of minimum path of each delivery
        std::vector<graphPlanResultPtr> min_list = find_min_from_mat(costMat_vertices_pairs);
        
        // count number of pre-relocations
        //size_t count_pre_relocs = 0;

        // find first on-inf ind
        min_list_ind = find_min_path(min_list); // todo: handle -1
        time_search_min.stop();
        time_watches.push_back(time_search_min);

        if(min_list_ind == -1) // no solution
        {
            // return failure //////
            return ReloPathResult(); // false by default constructor
        }

        // print path on graph
        print_graph_path(min_list[min_list_ind]->path, gPtr);

        // find objects in the middle
        auto reloc_objects = get_intermediate_objects(min_list[min_list_ind]->path, nameMatcher);

        // add to num of reloc
        //std::vector<size_t> temp_relocs(0);
        //temp_relocs.push_back(reloc_objects.size()); // todo: fix this
        int num_tempreloc = static_cast<int>(reloc_objects.size());

        // count number of pre-relocations (of the target object)
        size_t num_prereloc = 0;

        //stopWatch time_path_gen_push_path("push-path", measurement_type::pathPlan);
        // path segments for relocation
        // final pushing
        std::cout << "1" << std::endl;
        auto push_path_pair = get_push_path(min_list[min_list_ind]->path, edgeMatcher, gPtr, num_prereloc);
        push_path = push_path_pair.first;
        PathInfoList push_pathinfo = push_path_pair.second;
        //time_path_gen_push_path.stop();
        //time_watches.push_back(time_path_gen_push_path);

        // count
        //count_pre_relocs += num_prereloc;
        std::cout << 2 << std::endl;
        // relocation paths
        StatePathsPtr relo_paths;
        ReloPathInfoList relo_pathinfo;
        stopWatch time_path_gen_relo_path("relocate", measurement_type::relocatePlan);
        // list of state paths for temporary relocation of intermediate objects
        std::tie(relo_paths, relocPair, relo_pathinfo) = find_relo_path(push_path, reloc_objects, env); // pre-relocation path
        time_path_gen_relo_path.stop();
        time_watches.push_back(time_path_gen_relo_path);

        std::cout << "3" << std::endl;
        stopWatch time_path_gen_comb_path("motion", measurement_type::pathPlan);

        PathInfoList tempFinalPathInfo = PathInfoList();
        // combined path for the delivery
        auto reloPush_path = combine_relo_push(push_path, *relo_paths, push_pathinfo, relo_pathinfo, tempFinalPathInfo, robots[0], env, reloc_objects);
        time_path_gen_comb_path.stop();
        time_watches.push_back(time_path_gen_comb_path);

        //iterate until a valid one is found. Fail if all of them are not valid
        
        if(!reloPush_path.second) // hybrid astar failed
        {
            Color::println("Hybrid Astar Failed", Color::RED, Color::BG_YELLOW);
            //return reloPlanResult(false); // todo: don't return unless this is the only one left

            // mark this path as not feasible by raising cost
            costMat_vertices_pairs[min_list_ind].first->operator()(min_list[min_list_ind]->min_row, min_list[min_list_ind]->min_col) = std::numeric_limits<float>::infinity();

            // repeat
            Color::println("REPLAN", Color::RED,Color::BG_YELLOW);
        }
        
        // all necessary paths are found
        else
        {
            min_cost_row = min_list[min_list_ind]->min_row;
            min_cost_col = min_list[min_list_ind]->min_col;
            out_path = std::make_shared<StatePath>(reloPush_path.first);

            num_pre_relocs = num_prereloc;
            num_temp_relocs = num_tempreloc;            

            // store in path info
            final_pathinfo = tempFinalPathInfo;

            break;
        }
    }

    //return reloPush_path;
    auto from_obj = costMat_vertices_pairs[min_list_ind].second.from_obj;
    auto to_obj = costMat_vertices_pairs[min_list_ind].second.to_obj;
    
    return ReloPathResult(true, from_obj, to_obj, min_cost_row, min_cost_col, out_path, num_temp_relocs, num_pre_relocs, final_pathinfo);
}

reloPlanResult reloLoop(std::unordered_set<State>& obs, std::vector<movableObject>& mo_list, std::vector<movableObject>& delivered_obs,
                Environment& env, float map_max_x, float map_max_y, GraphPtr gPtr, graphTools::EdgeMatcher& edgeMatcher,
                std::vector<stopWatch>& time_watches, std::vector<movableObject>& delivery_list, 
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

        //stopWatch time_edge("edge", measurement_type::graphConst);
        // construct edges between objects
        reloPush::construct_edges(mo_list, gPtr, env, map_max_x, map_max_y, Constants::r, edgeMatcher, failed_paths, time_watches);
        //time_edge.stop();
        //time_edge.print_us();
        //time_watches.push_back(time_edge);

        // test: verify objects graph first
        // print edges
        if(params::print_graph)
            print_edges(gPtr);

        draw_paths(edgeMatcher,env,failed_paths,dubins_path_pub_ptr,failed_path_pub_ptr,Constants::r);

        // add deliverries to graph
        //stopWatch time_edge_d("delivery", measurement_type::graphConst);
        add_delivery_to_graph(delivery_list, mo_list, env, map_max_x, map_max_y, edgeMatcher, nameMatcher, failed_paths, gPtr, time_watches);
        //time_edge_d.stop();
        //time_edge_d.print_us();
        //time_watches.push_back(time_edge_d);
        
        // print edges
        if(params::print_graph)
            print_edges(gPtr);
        
        relocationPair_list relocPair; // for updating movable objects
        StatePath push_path;

        stopWatch time_temp("temp"); 
        ReloPathResult reloPush_path = iterate_remaining_deliveries(env, push_path, delivery_table, robots, relocPair, 
                                                                    nameMatcher, edgeMatcher, time_watches, gPtr);
        time_temp.stop();
        time_temp.print_us();
        ///////////////////

        //auto navPath_ptr = statePath_to_navPath(reloPush_path, use_mocap);

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
        time_update.stop();
        time_watches.push_back(time_update);    

        // count relocations
        vec_num_interm_relocs.push_back(reloPush_path.num_interm_reloc);
        vec_num_pre_relocs.push_back(reloPush_path.num_pre_reloc);
    }

    return reloPlanResult(true, std::accumulate(vec_num_interm_relocs.begin(), vec_num_interm_relocs.end(), 0),
                            std::accumulate(vec_num_pre_relocs.begin(), vec_num_pre_relocs.end(), 0), deliv_seq, out_pathinfo);
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
    if(argc == 3) // triggered by one dummy arg
    {
        params::use_testdata = true;
        data_ind = 21;
        //std::string data_file = "data_3o_2.txt";
        data_file = "data_3o_2.txt";
        params::leave_log = false;
    }
    
    //int leave_log = 1;
    else if(argc==4)
    {
        data_file = std::string(argv[1]);
        data_ind = std::atoi(argv[2]);        
        params::leave_log = std::atoi(argv[3]);
    }

    else
    {
        params::use_testdata = false; // use hard-coded data
        params::leave_log = false;
    }
}

void init_prompt(std::string& data_file, int& data_ind)
{
    Color::println("== data: " + data_file + " ===",Color::BG_YELLOW);
    Color::println("== better path: " + std::to_string(params::use_better_path) + " ===",Color::BG_YELLOW);
    Color::println("== ind: " + std::to_string(data_ind) + " ===",Color::BG_YELLOW);
    Color::println("== log: " + std::to_string(params::leave_log) + " ===",Color::BG_YELLOW);
    Color::println("== post-push: " + std::to_string(params::post_push_ind) + " ===",Color::BG_YELLOW);
    Color::println("== timeout (ms): " + std::to_string(params::grid_search_timeout) + " ===",Color::BG_YELLOW);
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

    //const char* args[] = { "program_name", "data_6o.txt", "0", "0" };
    //argv = const_cast<char**>(args);
    //argc = 4;

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
        records.write_to_file(std::string(CMAKE_SOURCE_DIR) + "/log/raw" + "/log_" + removeExtension(data_file) + ".txt");
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

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
