
#include <ros/ros.h>
#include <stopwatch.h>
#include <string>
#include <unordered_map>
#include <graphTools/dijkstra_tools.h>
#include <graphTools/graph_info.h>
#include <graphTools/visualizer.h>

#include <reloPush/movableObject.h>
#include <omplTools/dubins_tools.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "reloPush");
    ros::NodeHandle nh;

    // wait for debug attach
    ros::Duration(0.5).sleep();

    //test data
    //x = [2.5, 1  ,   1.5, 2]
    //y = [3, 3.5,   3.5,   3.5]
    auto b1 = movableObject(2.5,3, 0, "b1");
    auto b2 = movableObject(1,  3.5,1.57,"b2");
    auto b3 = movableObject(1.5,3.5,0,"b3");
    auto b4 = movableObject(2,  3.5,0,"b4");

    std::vector<movableObject> moList = {b1,b2,b3,b4};

    auto r1 = movableObject(1,0,1.57,"r1");

    //map of vertex-object
    std::unordered_map<std::string,movableObject> objectNameMap;
    objectNameMap.insert({b1.name,b1});
    objectNameMap.insert({b2.name,b2});
    objectNameMap.insert({b3.name,b3});
    objectNameMap.insert({b4.name,b4});

    //graph to use
    GraphPtr gPtr(new Graph);
    Vertex v1 = add_vertex(b1.name, *gPtr);
    Vertex v2 = add_vertex(b2.name, *gPtr);
    Vertex v3 = add_vertex(b3.name, *gPtr);
    Vertex v4 = add_vertex(b4.name, *gPtr);

    //connect edges using dubins
    //auto res = is_good_path(r1,b2,0.6);

    for(auto& it : moList)
    {
        for(auto& it2 : moList)
        {
            auto res = is_good_path(it,it2,0.8);
            if(res)
                std::cout << "t" << std::endl;
        }
    }

    ros::spin();

    return 0;
}
