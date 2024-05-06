
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

#include <Eigen/Core>
#include <Eigen/Dense>

//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>

ros::Publisher* vertex_marker_pub_ptr;
ros::Publisher* edge_marker_pub_ptr;
ros::Publisher* object_marker_pub_ptr;
ros::Publisher* dubins_path_pub_ptr;
ros::Publisher* delivery_marker_pub_ptr;

void initialize_publishers(ros::NodeHandle& nh)
{
    // init publishers and make pointers
    ros::Publisher vertex_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_nodes", 10);
    vertex_marker_pub_ptr = new ros::Publisher(vertex_marker_pub);

    ros::Publisher edge_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_edges", 2);
    edge_marker_pub_ptr = new ros::Publisher(edge_marker_pub);

    ros::Publisher object_marker_pub = nh.advertise<visualization_msgs::Marker>("movable_objects", 10);
    object_marker_pub_ptr = new ros::Publisher(object_marker_pub);

    ros::Publisher dubins_path_pub = nh.advertise<visualization_msgs::MarkerArray>("dubins_paths", 10);
    dubins_path_pub_ptr = new ros::Publisher(dubins_path_pub);

    ros::Publisher delivery_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("delivery_locations", 10);
    delivery_marker_pub_ptr = new ros::Publisher(delivery_marker_pub);
}

void free_publisher_pointers()
{
    delete(vertex_marker_pub_ptr);
    delete(edge_marker_pub_ptr);
    delete(object_marker_pub_ptr);
    delete(delivery_marker_pub_ptr);
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

        //find lowest
        float minVal;
        int minRow;
        minVal = cost_mat.rowwise().minCoeff().minCoeff(&minRow);

        // Find column with minimum value
        float minColVal;
        int minCol;
        minColVal = cost_mat.colwise().minCoeff().minCoeff(&minCol);

        std::cout << "row: " << minRow << " col: " << minCol << std::endl;

        graphPlanResult gp(pivot_names[minRow],target_names[minCol],cost_mat(minRow,minCol),paths[minRow][minCol],i->first);
        out_vec.push_back(std::make_shared<graphPlanResult>(gp));
    }

    return out_vec;
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "reloPush");
    ros::NodeHandle nh;

    // initialize publishers
    initialize_publishers(nh);

    // wait for debug attach
    ros::Duration(1.0).sleep();

    //test data
    //x = [2.5, 1  ,   1.5, 2]
    //y = [3, 3.5,   3.5,   3.5]

    std::vector<movableObject> mo_list(0);

    //generate graph
    GraphPtr gPtr(new Graph);

    // init objects and add to graph
    // todo: parse from input file
    int num_push_sides = 4;
    mo_list.push_back(movableObject(2.5,2,0,"b1",num_push_sides,gPtr));
    mo_list.push_back(movableObject(1,3.5,0,"b2",num_push_sides,gPtr));
    mo_list.push_back(movableObject(1.5,3.5,0,"b3",num_push_sides,gPtr));
    mo_list.push_back(movableObject(2,3.5,0,"b4",num_push_sides,gPtr));

    // genearte name matcher
    NameMatcher nameMatcher(mo_list);
    
    // edge to path matcher
    graphTools::EdgeMatcher edgeMatcher;

    // initialize grid map
    std::unordered_set<State> obs;
    for(auto& it : mo_list)
    {
        obs.insert(State(it.get_x(),it.get_y(),0));
    }
    State goal(0,0,0); // arbitrary goal
    Environment env(40, 40, obs, goal);

    stopWatch time_edge("edge_con");
    // construct edges
    reloPush::construct_edges(mo_list, gPtr, env, Constants::r, edgeMatcher ,false);
    time_edge.stop();
    time_edge.print_us();

    // add delivery poses
    std::vector<movableObject> delivery_list(0);
    
    // deliver b2 to 3,3.5
    delivery_list.push_back(movableObject(3,3.5,0,"d2",num_push_sides,gPtr));
    delivery_list.push_back(movableObject(0,0,0,"d1",num_push_sides,gPtr));
    
    // assignment table. object -> delivery
    std::unordered_map<std::string,std::string> delivery_table;
    delivery_table.insert({"b2","d2"});
    delivery_table.insert({"b1","d1"});

    // add to graph
    reloPush::add_deliveries(delivery_list,mo_list,gPtr,env,Constants::r,edgeMatcher,false);
    //add to namematcher
    nameMatcher.addVertices(delivery_list);

    // visualize vertices
    auto graph_vis_pair = visualize_graph(*gPtr, nameMatcher, vertex_marker_pub_ptr,edge_marker_pub_ptr);
    // visualize movable obstacles
    auto mo_vis = draw_obstacles(mo_list, object_marker_pub_ptr);
    // visualize edge paths
    auto vis_path_msg = draw_paths(edgeMatcher,env,dubins_path_pub_ptr,Constants::r);
    // visualize delivery locations
    auto vis_deli_msg = draw_deliveries(delivery_list,delivery_marker_pub_ptr);

    // print edges
    print_edges(gPtr);

    // traverse on graph
    pathFinder pf;
    //auto g = *gPtr;
    //std::string s = "b2_0";
    //std::string t = "d2_0";
    //auto trav_res = pf.djikstra(gPtr,s,t);
    //pf.printPath(gPtr, trav_res.first);
    //std::cout << "Cost: " << trav_res.second * Constants::r << std::endl;

    //test
    /*
    State a2(1,3.5,3.141592566167013);
    State a3(1.5,3.5,3.141592566167013);
    auto res = is_good_path(a3,a2,0.6);
    std::cout << "test " << res << std::endl;
    */

    //ros::spin();

    auto min_list = find_min_cost_seq(delivery_table,nameMatcher,gPtr);

    pf.printPath(gPtr, min_list[0]->path);
    pf.printPath(gPtr, min_list[1]->path);


    // hybrid astar from a robot
    State start(1, 1, -1*M_PI/2);
    State goal2(0.4, 3.5, 0);

    env.changeGoal(goal2);
    
    HybridAStar<State, Action, double, Environment> hybridAStar(env);
    PlanResult<State, Action, double> solution;
    bool searchSuccess = hybridAStar.search(start, solution);

    if (searchSuccess) {
    std::cout << "\033[1m\033[32m Succesfully find a path! \033[0m\n";
    
    for (auto iter = solution.states.begin(); iter != solution.states.end(); iter++)
       std::cout << iter->first << "," << iter->second << std::endl;

    std::cout << "Solution: gscore/cost:" << solution.cost
              << "\t fmin:" << solution.fmin << "\n\rDiscover " << env.Dcount
              << " Nodes and Expand " << env.Ecount << " Nodes." << std::endl;
    } else {
        std::cout << "\033[1m\033[31m Fail to find a path \033[0m\n";
    }


    ros::Rate r(10);

    while(ros::ok())
    {
        dubins_path_pub_ptr->publish(vis_path_msg);
        vertex_marker_pub_ptr->publish(graph_vis_pair.first);
        edge_marker_pub_ptr->publish(graph_vis_pair.second);
        delivery_marker_pub_ptr->publish(vis_deli_msg);
        object_marker_pub_ptr->publish(mo_vis);
        ros::spinOnce();
        r.sleep();
    }

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
