
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <graphTools/dijkstra_tools.h>
#include <graphTools/graph_info.h>
#include <graphTools/visualizer.h>

#include <reloPush/movableObject.h>
#include <omplTools/dubins_tools.h>
#include <reloPush/edge_construction.h>
#include <pathPlanTools/path_planning_tools.h>
#include <reloPush/stopwatch.h>
#include <graphTools/edge_path_info.h>
#include <pathPlanTools/dubins_interpolation.h>

#include <reloPush/color_print.h>

#include <Eigen/Core>
#include <Eigen/Dense>

//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>

ros::Publisher* vertex_marker_pub_ptr;
ros::Publisher* edge_marker_pub_ptr;
ros::Publisher* object_marker_pub_ptr;
ros::Publisher* dubins_path_pub_ptr;
ros::Publisher* delivery_marker_pub_ptr;
//test
ros::Publisher* test_path_pub_ptr;
ros::Publisher* text_pub_ptr;

const std::string world_frame = "map";

typedef visualization_msgs::MarkerArray vMArray;

void initialize_publishers(ros::NodeHandle& nh)
{
    // init publishers and make pointers
    ros::Publisher vertex_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_nodes", 10);
    vertex_marker_pub_ptr = new ros::Publisher(vertex_marker_pub);

    ros::Publisher edge_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_edges", 2);
    edge_marker_pub_ptr = new ros::Publisher(edge_marker_pub);

    ros::Publisher object_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("movable_objects", 10);
    object_marker_pub_ptr = new ros::Publisher(object_marker_pub);

    ros::Publisher dubins_path_pub = nh.advertise<visualization_msgs::MarkerArray>("dubins_paths", 10);
    dubins_path_pub_ptr = new ros::Publisher(dubins_path_pub);

    ros::Publisher delivery_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("delivery_locations", 10);
    delivery_marker_pub_ptr = new ros::Publisher(delivery_marker_pub);

    //test
    ros::Publisher test_path_pub = nh.advertise<nav_msgs::Path>("/car/planned_trajectory", 10);
    test_path_pub_ptr = new ros::Publisher(test_path_pub);

    ros::Publisher text_pub = nh.advertise<visualization_msgs::MarkerArray>("object_names", 5);
    text_pub_ptr = new ros::Publisher(text_pub);
}

void free_publisher_pointers()
{
    delete(vertex_marker_pub_ptr);
    delete(edge_marker_pub_ptr);
    delete(object_marker_pub_ptr);
    delete(delivery_marker_pub_ptr);
    //test
    delete(test_path_pub_ptr);
}

std::pair<size_t,size_t> find_min_row_col(Eigen::MatrixXf& mat)
{
    /*
    //find lowest
    float minVal;
    int minRow;
    minVal = mat.rowwise().minCoeff().minCoeff(&minRow);

    // Find column with minimum value
    float minColVal;
    int minCol;
    minColVal = mat.colwise().minCoeff().minCoeff(&minCol);
    */
    size_t minRow, minCol;
    float minValue = mat.minCoeff(&minRow, &minCol);

    return std::make_pair(minRow, minCol);
}

void print_edges(GraphPtr g)
{
    auto edge_list = graphTools::getEdges(*g);
    std::cout << "got edges" << std::endl;

    //print all edges and connected vertices
    for (size_t n=0; n<edge_list->size(); n++)
    {
        auto vertPair = graphTools::getVertexPair(edge_list, n, g);
        //name of vert
        std::cout << "Edge " << n << std::endl;
        auto vSource = graphTools::getVertexName(vertPair.getSource(), g);
        std::cout << "Souce name: " << vSource << std::endl;
        auto vSink = graphTools::getVertexName(vertPair.getSink(), g);
        std::cout << "Sink name: " << vSink << std::endl;
        auto eWeight = vertPair.getEdgeWeight();
        std::cout << "Edge Weight: " << eWeight << "\n" << std::endl;
    }
}

class graphPlanResult
{
    public:
        std::string sourceVertexName;
        std::string targetVertexName;
        float cost;
        std::vector<Vertex> path;
        std::string object_name;

        graphPlanResult()
        {}
        graphPlanResult(std::string s_in, std::string t_in, float c_in, std::vector<Vertex> p_in, std::string name_in) : sourceVertexName(s_in), targetVertexName(t_in), cost(c_in), path(p_in), object_name(name_in)
        {}
};

typedef std::shared_ptr<graphPlanResult> graphPlanResultPtr;
std::vector<graphPlanResultPtr> find_min_cost_seq(std::unordered_map<std::string,std::string>& delivery_table, NameMatcher& nameMatcher,GraphPtr gPtr)
{
    pathFinder pf;
    std::vector<graphPlanResultPtr> out_vec(0);
    
    //for each entry in delivery table
    for (auto i = delivery_table.begin(); i != delivery_table.end(); i++)
    {
        //cout << i->first << "       " << i->second << endl;
        auto pivot_obj = nameMatcher.getObject(i->first);
        auto target_obj = nameMatcher.getObject(i->second);

        auto pivot_names = pivot_obj->get_vertex_names();
        auto target_names = target_obj->get_vertex_names();

        const int rows = pivot_obj->get_n_side();
        const int cols = target_obj->get_n_side();

        // init cost table
        Eigen::MatrixXf cost_mat(rows,cols);
        // paths
        std::vector<std::vector<std::vector<Vertex>>> paths(rows);

        for(int row=0; row<rows; row++)
        {
            paths[row].resize(cols);
            for(int col=0; col<cols; col++)
            {                
                std::string start_name = pivot_names[row]; // by row
                std::string target_name = target_names[col]; // by col

                auto res = pf.djikstra(gPtr,start_name,target_name);

                paths[row][col] = res.first;
                cost_mat(row,col) = res.second;
            }
        }

        std::cout << cost_mat << std::endl;

        // Find the indices of the smallest element
        size_t minRow, minCol;
        //float minValue = cost_mat.minCoeff(&minRow, &minCol);
        std::tie(minRow,minCol) = find_min_row_col(cost_mat);

        std::cout << "Smallest value: " << cost_mat(minRow,minCol) << std::endl;

        //std::cout << "row: " << minRow << " col: " << minCol << std::endl;

        Color::println("This may not be the only smallest value",Color::YELLOW);

        graphPlanResult gp(pivot_names[minRow],target_names[minCol],cost_mat(minRow,minCol),paths[minRow][minCol],i->first);
        out_vec.push_back(std::make_shared<graphPlanResult>(gp));
    }

    return out_vec;
}

std::shared_ptr<PlanResult<State, Action, double>> planHybridAstar(State start, State goal_in, Environment& env, bool print_res = false, bool negate_start_yaw = false, bool negate_goal_yaw = false)
{
    

    if(negate_start_yaw)
        start.yaw *= -1;

    if(negate_goal_yaw)
        goal_in.yaw *= -1;

    env.changeGoal(goal_in);
    
    HybridAStar<State, Action, double, Environment> hybridAStar(env);
    PlanResult<State, Action, double> solution;
    bool searchSuccess = hybridAStar.search(start, solution);

    if (searchSuccess) {
        if(print_res)
        {
            std::cout << "\033[1m\033[32m Succesfully find a path! \033[0m\n";
    
            for (auto iter = solution.states.begin(); iter != solution.states.end(); iter++)
            std::cout << iter->first << "," << iter->second << std::endl;

            std::cout << "Solution: gscore/cost:" << solution.cost
                    << "\t fmin:" << solution.fmin << "\n\rDiscover " << env.Dcount
                    << " Nodes and Expand " << env.Ecount << " Nodes." << std::endl;
        } 
    }
    else {
        //if(print_res)
            std::cout << "\033[1m\033[31m Fail to find a path \033[0m\n";
    }

    return std::make_shared<PlanResult<State, Action, double>>(solution);    
}

template<typename T>
std::shared_ptr<std::vector<std::vector<T>>> initDoubleVec(int rows, int cols)
{
    std::vector<std::vector<T>> outVec(rows, std::vector<T>(cols, 0));
    return std::make_shared<std::vector<std::vector<T>>>(outVec);
}

std::shared_ptr<nav_msgs::Path> statePath_to_navPath(std::vector<State>& path_in)
{
    nav_msgs::Path out_path;
    out_path.header.frame_id = world_frame;
    out_path.poses.resize(path_in.size());

    out_path.header.stamp = ros::Time::now();

    for(size_t n=0; n<path_in.size(); n++)
    {
        out_path.poses[n].pose.position.x = path_in[n].x;
        out_path.poses[n].pose.position.y = path_in[n].y;
        out_path.poses[n].pose.position.z = 0;
        out_path.poses[n].pose.orientation = jeeho::eigenQtoMsgQ(jeeho::euler_to_quaternion_xyz(0,0,path_in[n].yaw));

        //timing
        if(n>0)
        {
            //delta distance
            double dx = out_path.poses[n].pose.position.x - out_path.poses[n-1].pose.position.x;
            double dy = out_path.poses[n].pose.position.y - out_path.poses[n-1].pose.position.y;
            //double dz = out_path.poses[n].pose.position.z - out_path.poses[n-1].pose.position.z;
            auto dist = std::sqrt(dx*dx + dy*dy);

            //delta time
            auto dTime = (float)dist/Constants::speed_limit;

            out_path.poses[n].header.stamp = out_path.poses[n-1].header.stamp + ros::Duration(dTime);
        }
        else
            out_path.poses[n].header.stamp = ros::Time::now();
    }

    return std::make_shared<nav_msgs::Path>(out_path);
}

State find_pre_push(State& goalState, float distance = 0.6f)
{
    State outState(goalState);

    // Calculate the new x and y coordinates
    outState.x -= distance * cos(goalState.yaw);
    outState.y -= distance * sin(goalState.yaw);

    return outState;
}

State find_post_push(State& goalState, float distance = 0.6f)
{
    State outState(goalState);

    // Calculate the new x and y coordinates
    outState.x += distance * cos(goalState.yaw);
    outState.y += distance * sin(goalState.yaw);

    return outState;
}

void init_movable_objects(std::vector<movableObject>& mo_list, GraphPtr gPtr, int num_push_sides = 4)
{
    mo_list.push_back(movableObject(3,3.5,0,"b1",num_push_sides,gPtr));
    //mo_list.push_back(movableObject(1,3.5,0,"b2",num_push_sides,gPtr));
    mo_list.push_back(movableObject(1,1.5,0,"b2",num_push_sides,gPtr));
    mo_list.push_back(movableObject(1,2,0,"b3",num_push_sides,gPtr));
    mo_list.push_back(movableObject(0.5,3.5,0,"b4",num_push_sides,gPtr));
}

void init_static_obstacles(std::unordered_set<State>& obs, std::vector<movableObject>& mo_list)
{
    for(auto& it : mo_list)
    {
        obs.insert(State(it.get_x(),it.get_y(),0));
    }    
}

std::unordered_map<std::string,std::string> init_delivery(std::vector<movableObject>& delivery_list, std::vector<movableObject>& mo_list, Environment& env, 
                    graphTools::EdgeMatcher& edgeMatcher, NameMatcher& nameMatcher, GraphPtr gPtr, int num_push_sides = 4)
{
    // deliver b2 to 3,3.5
    //delivery_list.push_back(movableObject(3,3.5,0,"d2",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(0,0,0,"d1",num_push_sides,gPtr));
    delivery_list.push_back(movableObject(2,4,0,"d3",num_push_sides,gPtr));
    
    // assignment table. object -> delivery
    std::unordered_map<std::string,std::string> delivery_table;
    //delivery_table.insert({"b2","d2"});
    //delivery_table.insert({"b1","d1"});
    delivery_table.insert({"b3","d3"});

    // add to graph
    reloPush::add_deliveries(delivery_list,mo_list,gPtr,env,Constants::r,edgeMatcher,false);
    //add to namematcher
    nameMatcher.addVertices(delivery_list);

    return delivery_table;
}

void init_robots(std::vector<State>& robots)
{
    //robots.push_back(State(0.3, 1, -1*M_PI/2));
    //robots.push_back(State(2, 2.25, 0));
    robots.push_back(State(1.5, 2.5, 0));
    //robots.push_back(State(5, 3, M_PI/2));
}

int findFirstTrueIndex(const std::vector<bool>& vec) {
    for (size_t i = 0; i < vec.size(); ++i) {
        if (vec[i]) {
            return static_cast<int>(i); // Return the index of the first true element
        }
    }
    return -1; // Return -1 if no true element is found
}

std::vector<movableObjectPtr> get_intermediate_objects(std::vector<Vertex>& list_in, NameMatcher& nameMatcher)
{
    std::vector<movableObjectPtr> out_vec(0);

    std::vector<Vertex> temp_vec(list_in);
    if (temp_vec.size() >= 2) {
        // Remove the first element
        temp_vec.erase(temp_vec.begin());

        // Remove the last element
        temp_vec.erase(temp_vec.end() - 1);
    }
    else
    {
        // this should not happen
        Color::println("Unexpected graph plan result", Color::RED);
        temp_vec.clear();
    }

    for(auto& it : temp_vec)
        out_vec.push_back(nameMatcher.getObject(it));

    return out_vec;
}

void generate_temp_env(Environment& env_in, Environment& temp_env, std::vector<State>& push_path)
{
    // copy // todo: do better copy
    Environment out_env = env_in;

    //std::unordered_set<State> path_obs;

    // add path as obstacle
    for(size_t i=0; i<push_path.size(); i++)
    {
        // center point in cell coord
        out_env.add_obs(State(push_path[i].x,push_path[i].y,0));
    }

    temp_env = out_env;
}

// relocation path of a cube
// input: pushing path
//        object to relocate
//        map with obstacles
std::shared_ptr<std::vector<std::vector<State>>> find_relo_path(std::vector<State>& push_path, std::vector<movableObjectPtr>& relo_list, Environment& env)
{
    // find where to relocate
    // propagate along pushing directions until find one
    // increment by cell resolution (Constants::mapResolution)
    // n candidates

    // generate copied env with path points as obstacles
    Environment temp_env;
    generate_temp_env(env, temp_env, push_path);

    std::vector<std::vector<State>> out_paths(relo_list.size());

    // for each mo
    for(int m = 0; m<relo_list.size(); m++)
    {
        movableObjectPtr moPtr = relo_list[m];

        // take out this object from obstacles
        temp_env.remove_obs(State(moPtr->get_x(), moPtr->get_y(), moPtr->get_th()));

        auto init_pusing_poses = moPtr->get_pushing_poses();
        std::vector<State> candidates(init_pusing_poses.size());
        // copy
        for(size_t n=0; n<init_pusing_poses.size(); n++)
            candidates[n] = *(init_pusing_poses[n]);
        
        std::vector<bool> valid_vec(candidates.size());
        for(size_t i=0; i<valid_vec.size(); i++)
            valid_vec[i] = false;

        bool valid_position_found = false;
        while(true) //todo: add condition to quit
        {
            // increment by cell resolution along pushing direction
            for(size_t n=0; n<candidates.size(); n++){
                // unit vector of pushing direction
                // Calculate change in x and y coordinates
                float delta_x = Constants::mapResolution * cosf(candidates[n].yaw);
                float delta_y = Constants::mapResolution * sinf(candidates[n].yaw);

                candidates[n].x += delta_x;
                candidates[n].y += delta_y;

                // check if this is a valid position
                if(temp_env.stateValid(candidates[n],Constants::carWidth,0.2,Constants::LB,Constants::LF) == true) //todo: set better values
                {
                    valid_vec[n] = true;

                    // add break flag
                    valid_position_found = true;
                }
            }

            // break if a point is found
            if(valid_position_found)
                break;
        }

        // pick a valid pose if multiple // todo: find a better way to handle multiple candidates
        int valid_ind = findFirstTrueIndex(valid_vec);

        // path from init pre-push to target pre-push
        State start_pre_push = find_pre_push(*init_pusing_poses[valid_ind], 0.6f);
        State goal_pre_push = find_pre_push(candidates[valid_ind], 0.6f);
        //todo: interpolate the line

        // add new location as obstacle
        temp_env.add_obs(candidates[valid_ind]);

        // add this path
        out_paths[m] = std::vector<State>({start_pre_push,goal_pre_push});
    }

    return std::make_shared<std::vector<std::vector<State>>>(out_paths);
}

std::vector<State> get_push_path(std::vector<Vertex>& vertex_path, 
                                  graphTools::EdgeMatcher& edgeMatcher, GraphPtr gPtr)
{
    std::vector<State> push_path(0);

    // augment dubins path
    // don't do multi-processing
    for(size_t i=vertex_path.size()-1; i>0; i--)
    {
        //find edge //todo: handle multiple edges
        Vertex source = vertex_path[i];
        Vertex target = vertex_path[i-1];
        Edge edge = boost::edge(source, target, *gPtr).first;

        // corresponding path
        auto partial_path_info = edgeMatcher.getPath(edge);
        //auto pivot_state = nameMatcher.getVertexStatePair(min_list[mcol]->sourceVertexName)->state;

        // get interpolated list
        auto interp_list = interpolate_dubins(partial_path_info,Constants::r,0.2f);
        push_path.insert(push_path.end(), interp_list->begin(), interp_list->end());
    }    

    //return final_path;
    return push_path;
}

std::vector<State> combine_relo_push(std::vector<State>& push_path, std::vector<std::vector<State>>& relo_path, State& robot, Environment& env, std::vector<movableObjectPtr>& relo_list)
{
    std::vector<std::vector<State>> path_segments;
    if(relo_path.size()>0)
    {
        
        // start to first relo
        path_segments.push_back(planHybridAstar(robot, relo_path[0][0], env, false, true, true)->getPath(true));
        path_segments.push_back(relo_path[0]);
        // remove pushed object
        auto moPtr = relo_list[0];
        env.remove_obs(State(moPtr->get_x(), moPtr->get_y(), moPtr->get_th()));
        // relocated
        auto relocated_obs = find_post_push(relo_path[0].back());
        env.add_obs(relocated_obs);

        // relo
        // do not use multiprocessing
        for(size_t p=0; p<relo_path.size()-1; p++)
        {
            // last of this relo
            auto lastThisRelo = relo_path[p].back();
            // start of next relo
            auto firstNextRelo = relo_path[p+1].front();

            auto moPtr_loop = relo_list[p+1];
            env.remove_obs(State(moPtr_loop->get_x(), moPtr_loop->get_y(), moPtr_loop->get_th()));

            // plan hybrid astar path
            auto temp_path = planHybridAstar(lastThisRelo, firstNextRelo, env, false, true, true);
            path_segments.push_back(temp_path->getPath(true));
            path_segments.push_back(relo_path[p+1]);

            // put it back
            env.add_obs(find_post_push(relo_path[p+1].back()));
        }

        // relo to push
        auto lastLastRelo = relo_path.back().back();
        // first pre-push
        auto pre_push = find_pre_push(push_path.front(),0.6f);
        path_segments.push_back(planHybridAstar(lastLastRelo, pre_push, env, false, true, true)->getPath(true));

        path_segments.push_back(push_path);

    }

    else{
        // start to prepush
        auto pre_push = find_pre_push(push_path.front(),0.6f);
        path_segments.push_back(planHybridAstar(robot, pre_push, env, false)->getPath(true));
        path_segments.push_back(push_path);        
    }

    std::vector<State> combinedVector;
    for (const auto& subVector : path_segments) {
        combinedVector.insert(combinedVector.end(), subVector.begin(), subVector.end());
    }
    
    return combinedVector;
}


std::shared_ptr<nav_msgs::Path> generate_final_path(std::vector<State>& robots, std::vector<graphPlanResultPtr>& min_list, 
                                        NameMatcher& nameMatcher, graphTools::EdgeMatcher& edgeMatcher ,Environment& env, GraphPtr gPtr)
{
    // assign robot to a block
    // find assignment by shortest distacnce
    // for each min_list, calculate travel distance to approach
    // pick one with lowest cost
    // todo: use hungarian to find best assignment

    Eigen::MatrixXf robot_approach_mat = Eigen::MatrixXf(robots.size(),min_list.size());
    auto pathArr = initDoubleVec<std::shared_ptr<PlanResult<State, Action, double>>>(robots.size(),min_list.size());

    for(size_t r = 0; r<robots.size(); r++)
    {
        for(size_t min_it = 0; min_it < min_list.size(); min_it++)
        {
            auto approach_state = nameMatcher.getVertexStatePair(min_list[min_it]->sourceVertexName)->state;
            auto pre_push = find_pre_push(*approach_state,0.6f);
            //hybrid astar state yaw is negated
            pre_push.yaw*=-1;
            auto plan_res = planHybridAstar(robots[r], pre_push, env, false);

            // cost
            robot_approach_mat(r,min_it) = plan_res->cost; // todo: handle path plan failure
            // save to path array
            pathArr->at(r)[min_it] = plan_res;
        }
    }

    size_t mrow, mcol;
    std::tie(mrow,mcol) = find_min_row_col(robot_approach_mat);
    //min cost path
    auto minRobot = robots[mrow];
    auto minApproach = min_list[mcol];
    auto minPath = pathArr->at(mrow)[mcol];

    // combine approach + push path
    // minPath + dubins
    // final path as a list of states
    std::vector<State> final_path(minPath->getPath(true));

    // augment dubins path
    // don't do multi-processing
    for(size_t i=min_list[mcol]->path.size()-1; i>0; i--)
    {
        //find edge //todo: handle multiple edges
        Vertex source = min_list[mcol]->path[i];
        Vertex target = min_list[mcol]->path[i-1];
        Edge edge = boost::edge(source, target, *gPtr).first;

        // corresponding path
        auto partial_path_info = edgeMatcher.getPath(edge);
        //auto pivot_state = nameMatcher.getVertexStatePair(min_list[mcol]->sourceVertexName)->state;

        // get interpolated list
        auto interp_list = interpolate_dubins(partial_path_info,Constants::r,0.2f);
        final_path.insert(final_path.end(), interp_list->begin(), interp_list->end());
    }    

    //return final_path;
    return statePath_to_navPath(final_path);
}

void visualization_loop(GraphPtr gPtr, std::vector<movableObject>& mo_list, std::vector<movableObject>& delivery_list
                        , NameMatcher& nameMatcher, graphTools::EdgeMatcher edgeMatcher, Environment& env, std::shared_ptr<nav_msgs::Path> navPath_ptr, double loop_rate=10)
{
    ros::Rate r(loop_rate);
    // visualize vertices
    auto graph_vis_pair = visualize_graph(*gPtr, nameMatcher, vertex_marker_pub_ptr,edge_marker_pub_ptr);
    // visualize movable obstacles
    auto mo_vis = draw_obstacles(mo_list, object_marker_pub_ptr);
    // visualize edge paths
    auto vis_path_msg = draw_paths(edgeMatcher,env,dubins_path_pub_ptr,Constants::r);
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

        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "reloPush");
    ros::NodeHandle nh;
    // initialize publishers
    initialize_publishers(nh);
    // wait for debug attach
    ros::Duration(1.0).sleep();

    int num_push_sides = 4;
    std::vector<movableObject> mo_list(0);

    //generate graph
    GraphPtr gPtr(new Graph);

    // init objects and add to graph
    // todo: parse from input file
    init_movable_objects(mo_list, gPtr, num_push_sides);

    // genearte name matcher
    NameMatcher nameMatcher(mo_list);
    
    // edge to path matcher
    graphTools::EdgeMatcher edgeMatcher;

    // initialize grid map
    std::unordered_set<State> obs;
    init_static_obstacles(obs, mo_list);

    State goal(0,0,0); // arbitrary goal
    Environment env(20, 20, obs, goal);

    //stopWatch time_edge("edge_con");
    // construct edges
    reloPush::construct_edges(mo_list, gPtr, env, Constants::r, edgeMatcher ,false);
    //time_edge.stop();
    //time_edge.print_us();

    // add delivery poses
    std::vector<movableObject> delivery_list(0);
    std::unordered_map<std::string,std::string> delivery_table = init_delivery(delivery_list,mo_list,env,edgeMatcher,nameMatcher,gPtr,num_push_sides);

    // print edges
    print_edges(gPtr);
    // traverse on graph
    pathFinder pf;
    // find best push traverse for all assignments
    auto min_list = find_min_cost_seq(delivery_table,nameMatcher,gPtr);
    pf.printPath(gPtr, min_list[0]->path);
    //pf.printPath(gPtr, min_list[1]->path);
    auto reloc_objects = get_intermediate_objects(min_list[0]->path, nameMatcher);

    // hybrid astar from a robot
    std::vector<State> robots(0);
    init_robots(robots);

    // path segments for relocation
    auto push_path = get_push_path(min_list[0]->path, edgeMatcher, gPtr);
    auto relo_paths = find_relo_path(push_path, reloc_objects, env);
    auto reloPush_path = combine_relo_push(push_path, *relo_paths, robots[0], env, reloc_objects);

    auto navPath_ptr = statePath_to_navPath(reloPush_path);
    
    // combine all paths
    //auto navPath_ptr = statePath_to_navPath(final_path);
    //auto navPath_ptr = generate_final_path(robots, min_list, nameMatcher, edgeMatcher, env, gPtr);

    visualization_loop(gPtr, mo_list, delivery_list, nameMatcher, edgeMatcher, env, navPath_ptr, 10);

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
