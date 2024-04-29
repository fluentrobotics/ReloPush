
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

ros::Publisher* vertex_marker_pub_ptr;
ros::Publisher* edge_marker_pub_ptr;

void initialize_publishers(ros::NodeHandle& nh)
{
    // init publishers and make pointers
    ros::Publisher vertex_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_nodes", 10);
    vertex_marker_pub_ptr = new ros::Publisher(vertex_marker_pub);
    ros::Publisher edge_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_edges", 2);
    edge_marker_pub_ptr = new ros::Publisher(edge_marker_pub);
}

void free_publisher_pointers()
{
    delete(vertex_marker_pub_ptr);
    delete(edge_marker_pub_ptr);
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
    

    // initialize grid map
    std::unordered_set<State> obs;
    for(auto& it : mo_list)
    {
        obs.insert(State(it.x,it.y,0));
    }
    State goal(0,0,0); // arbitrary goal
    Environment env(40, 40, obs, goal);

    stopWatch time_edge("edge_con");
    // construct edges
    reloPush::construct_edges(mo_list, gPtr, env, Constants::r,true);
    time_edge.stop();
    time_edge.print_us();
    // visualize vertices
    visualize_graph(*gPtr, nameMatcher, vertex_marker_pub_ptr,edge_marker_pub_ptr);

    //test
    /*
    State a2(1,3.5,3.141592566167013);
    State a3(1.5,3.5,3.141592566167013);
    auto res = is_good_path(a3,a2,0.6);
    std::cout << "test " << res << std::endl;
    */

    ros::spin();

    //remove publisher pointers
    free_publisher_pointers();

    return 0;
}
