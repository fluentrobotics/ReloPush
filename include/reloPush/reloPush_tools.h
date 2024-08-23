#ifndef RELOPUSH_TOOLS_H
#define RELOPUSH_TOOLS_H

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <tuple>
#include <graphTools/dijkstra_tools.h>
#include <graphTools/graph_info.h>
#include <graphTools/visualizer.h>

#include <omplTools/dubins_tools.h>
#include <omplTools/State.h>
#include <reloPush/edge_construction.h>
#include <pathPlanTools/path_planning_tools.h>
#include <reloPush/logger.h>
#include <reloPush/data_collector.h> // logger will write file based on info on a data_collector
#include <graphTools/edge_path_info.h>
#include <pathPlanTools/dubins_interpolation.h>
#include <reloPush/deliverySet.h>

#include <reloPush/color_print.h>
#include <reloPush/params.h>
#include <reloPush/push_pose_tools.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

extern ros::Publisher* vertex_marker_pub_ptr;
extern ros::Publisher* edge_marker_pub_ptr;
extern ros::Publisher* object_marker_pub_ptr;
extern ros::Publisher* dubins_path_pub_ptr;
extern ros::Publisher* delivery_marker_pub_ptr;
//test
extern ros::Publisher* test_path_pub_ptr;
extern ros::Publisher* failed_path_pub_ptr;
extern ros::Publisher* text_pub_ptr;

extern ros::Publisher* boundary_pub_ptr;
extern ros::Publisher* robot_pose_reset_ptr;

extern ros::NodeHandle* nh_ptr;

typedef visualization_msgs::MarkerArray vMArray;
typedef std::unordered_map<std::string,std::string> strMap;
typedef std::vector<std::vector<std::vector<Vertex>>> VertexPath;
typedef std::shared_ptr<VertexPath> VertexPathPtr;

struct reloPlanResult{
    bool is_succ;
    size_t num_of_reloc; //number of temp relocation (of other objects blocking paths)
    size_t num_of_prereloc; //number of pre-relocation (of delivering object)
    std::vector<std::string> delivery_sequence;
    PathInfoList pathInfoList;

    reloPlanResult(bool is_success) : is_succ(is_success)
    {
        num_of_reloc = 0;
        num_of_prereloc = 0;
        delivery_sequence.clear();
        pathInfoList.paths.clear();
    }

    reloPlanResult(bool is_success, size_t num_reloc, size_t num_prereloc, std::vector<std::string>& d_seq, PathInfoList& pathInfo_in) 
    : is_succ(is_success), num_of_reloc(num_reloc), num_of_prereloc(num_prereloc), delivery_sequence(d_seq), pathInfoList(pathInfo_in)
    {}
};

struct ReloPathResult{
    bool is_succ;
    movableObjectPtr from_obj;
    movableObjectPtr to_obj;
    size_t from_push_ind; // index of pushing orientation of the object
    size_t to_arrival_ind; // index of arriving orientation of the object
    StatePathPtr state_path;
    int num_interm_reloc; // intermediate relocation of blocking objects
    int num_pre_reloc; // pre-relocation of the target object
    PathInfoList pathInfoList; // partial path info to be appended to the final result

    ReloPathResult()
    {
        is_succ = false;
        from_obj = nullptr;
        to_obj = nullptr;
        from_push_ind = 99;
        to_arrival_ind = 99;
        state_path = nullptr;
        num_interm_reloc = 0;
        num_pre_reloc = 0;
        pathInfoList = PathInfoList();
    }

    ReloPathResult(bool is_success, movableObjectPtr from_obj_ptr, movableObjectPtr to_obj_ptr, size_t push_ind,
                    size_t arrival_ind, StatePathPtr path_in, int temp_reloc, int pre_reloc, PathInfoList& list_in) 
                    : is_succ(is_success), from_obj(from_obj_ptr), to_obj(to_obj_ptr), 
                    from_push_ind(push_ind), to_arrival_ind(arrival_ind), 
                    state_path(path_in), num_interm_reloc(temp_reloc), num_pre_reloc(pre_reloc), pathInfoList(list_in)
    {}
};

struct ObjectPairPath{
    movableObjectPtr from_obj = nullptr;
    movableObjectPtr to_obj = nullptr;
    VertexPathPtr path = nullptr;

    ObjectPairPath(movableObjectPtr from_obj_ptr, movableObjectPtr to_obj_ptr, VertexPathPtr path_ptr) : from_obj(from_obj_ptr), to_obj(to_obj_ptr), path(path_ptr)
    {
    }
};

void initialize_publishers(ros::NodeHandle& nh, bool use_mocap = false)
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

    ros::Publisher failed_path_pub = nh.advertise<visualization_msgs::MarkerArray>("failed_paths", 10);
    failed_path_pub_ptr = new ros::Publisher(failed_path_pub);

    ros::Publisher delivery_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("delivery_locations", 10);
    delivery_marker_pub_ptr = new ros::Publisher(delivery_marker_pub);

    //test
    ros::Publisher test_path_pub;
    if(!use_mocap) // todo: parse as param
        test_path_pub = nh.advertise<nav_msgs::Path>("/car/planned_trajectory", 10);
    else
        test_path_pub = nh.advertise<nav_msgs::Path>("/mushr2/sim_trajectory", 10);

    test_path_pub_ptr = new ros::Publisher(test_path_pub);



    ros::Publisher text_pub = nh.advertise<visualization_msgs::MarkerArray>("object_names", 5);
    text_pub_ptr = new ros::Publisher(text_pub);

    // workspace boundary
    ros::Publisher boundary_pub = nh.advertise<visualization_msgs::Marker>("/workspace_boundary", 10);
    boundary_pub_ptr = new ros::Publisher(boundary_pub);

    // robot pose reset
    ros::Publisher robot_pose_reset = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    robot_pose_reset_ptr = new ros::Publisher(robot_pose_reset);
}

void free_publisher_pointers()
{
    delete(vertex_marker_pub_ptr);
    delete(edge_marker_pub_ptr);
    delete(object_marker_pub_ptr);
    delete(dubins_path_pub_ptr);
    delete(failed_path_pub_ptr);
    delete(delivery_marker_pub_ptr);
    //test
    delete(test_path_pub_ptr);
    delete(text_pub_ptr);
    delete(boundary_pub_ptr);
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

tf::StampedTransform listen_tf(std::string from_tf, std::string to_tf)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try{
      listener.waitForTransform(from_tf, to_tf,
                                    ros::Time::now(), ros::Duration(1.0));
    listener.lookupTransform(from_tf, to_tf,
                             ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    //ros::Duration(0.1).sleep();
    //continue;Quaternion XYZW: -0.0396948 0.953661 -0.258644 -0.148522
    transform.setData(tf::Transform::getIdentity());
  }
  return transform;
}

nav_msgs::Path transformPath(const nav_msgs::Path& input_path, const tf::Transform& transform) {
    nav_msgs::Path output_path = input_path; // Copy header and other metadata
    output_path.poses.clear();

    output_path.header.frame_id = "map_mocap";

    for (const auto& pose : input_path.poses) {
        // Convert geometry_msgs::Pose to tf::Pose
        tf::Pose tf_pose;
        tf::poseMsgToTF(pose.pose, tf_pose);

        // Apply the transformation
        tf::Pose transformed_pose = transform * tf_pose;

        // Convert tf::Pose back to geometry_msgs::Pose
        geometry_msgs::PoseStamped transformed_pose_stamped = pose;
        tf::poseTFToMsg(transformed_pose, transformed_pose_stamped.pose);

        // Add the transformed pose to the new path
        output_path.poses.push_back(transformed_pose_stamped);
    }

    return output_path;
}

int find_post_push_ind(StatePath& s, Environment& env){

    int pivot_ind = params::post_push_ind;
    if(env.stateValid(s.end()[params::post_push_ind]))
        return pivot_ind;

    while(abs(pivot_ind) < s.size())
    {
        pivot_ind--;
        if(env.stateValid(s.end()[pivot_ind]))
            return pivot_ind;
    }
    // todo: handle all not valid
}

/// @brief result of dijkstra on graph
/// Vertices and paht with cost
typedef std::shared_ptr<graphPlanResult> graphPlanResultPtr;

typedef Eigen::Matrix<float, -1, -1> MatType;
typedef std::shared_ptr<MatType> MatPtr;

typedef std::pair<MatPtr,ObjectPairPath> ObjectCostMat;

typedef std::vector<std::vector<PathPlanResultPtr>> PathMat;
typedef std::vector<PathMat> PathMatList;
typedef std::shared_ptr<PathMatList> PathMatListPtr;

// generate cost matrix of each delivery
// return: vector of pairs of costmat and vertices pair
std::vector<ObjectCostMat> get_cost_mat_vertices_pair(strMap& delivery_table, NameMatcher& nameMatcher,GraphPtr gPtr, 
                                                        Environment& env, bool mp_only = false, PathMatListPtr pathMatListPtr = nullptr)
{
    std::vector<ObjectCostMat> out_mat_vec;
    pathFinder pf;

    if(mp_only)
        pathMatListPtr->clear();

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

        //for mp_only
        PathMat pathMat(rows);

        for(int row=0; row<rows; row++)
        {
            paths[row].resize(cols);
            if(mp_only)
                pathMat[row].resize(cols);

            for(int col=0; col<cols; col++)
            {                
                std::string start_name = pivot_names[row]; // by row
                std::string target_name = target_names[col]; // by col

                if(!mp_only)
                {
                    auto res = pf.djikstra(gPtr,start_name,target_name);

                    paths[row][col] = res.first;
                    cost_mat(row,col) = res.second;
                }
                else // motion plan only version
                {
                    auto start_pose = pivot_obj->get_pushing_poses()[row];
                    auto goal_pose = target_obj->get_pushing_poses()[col];

                    // find arrival pre-push pose
                    auto goal_pre_push = find_pre_push(*goal_pose, params::pre_push_dist);

                    // temporarily remove from env
                    env.remove_obs(*start_pose);
                    auto plan_res = planHybridAstar(*start_pose,goal_pre_push,env,0,false);
                    // restore starting obj
                    env.add_obs(*start_pose);

                    if(!plan_res->success) // plan failed
                    {
                        cost_mat(row,col) = std::numeric_limits<float>::infinity();
                    }
                    else
                    {
                        paths[row][col] = {graphTools::getVertex(gPtr,start_name),graphTools::getVertex(gPtr,target_name)};
                        cost_mat(row,col) = (float)plan_res->cost;
                        pathMat[row][col] = plan_res;
                    }
                } // end motion plan only
            }
        }

        std::cout << cost_mat << std::endl;

        // store in matrix vector
        ObjectCostMat mat_pair(std::make_shared<MatType>(cost_mat),ObjectPairPath(pivot_obj,target_obj,std::make_shared<VertexPath>(paths)));
        out_mat_vec.push_back(mat_pair);
        if(mp_only)
            pathMatListPtr->push_back(pathMat);
    }
    
    return out_mat_vec;
}

// returns a list of min row/col pairs
std::vector<graphPlanResultPtr> find_min_from_mat(std::vector<ObjectCostMat>& mat_vec)
{
    std::vector<graphPlanResultPtr> out_vec(0);

    for(auto& it : mat_vec)
    {
        // Find the indices of the smallest element
        size_t minRow, minCol;

        // for naming purpose
        auto cost_mat_ptr = it.first;
        //float minValue = cost_mat.minCoeff(&minRow, &minCol);
        std::tie(minRow,minCol) = find_min_row_col(*cost_mat_ptr);

        std::cout << "Smallest value: " << (*cost_mat_ptr)(minRow,minCol) << std::endl;
        if((*cost_mat_ptr)(minRow,minCol)==std::numeric_limits<float>::infinity())
            Color::println("No path on graph", Color::WARNING, Color::BG_MAGENTA);

        //std::cout << "row: " << minRow << " col: " << minCol << std::endl;
        else
            Color::println("This may not be the only smallest value",Color::YELLOW);

        auto pivot_names = it.second.from_obj->get_vertex_names();
        auto target_names = it.second.to_obj->get_vertex_names();

        graphPlanResult gp(pivot_names[minRow],target_names[minCol],(*cost_mat_ptr)(minRow,minCol),(*it.second.path)[minRow][minCol],it.second.from_obj->get_name(), minRow, minCol);
        out_vec.push_back(std::make_shared<graphPlanResult>(gp));
    }

    return out_vec;
}

// finds the row/col combination with the minimum cost for each delivery
std::vector<graphPlanResultPtr> find_min_cost_seq(std::unordered_map<std::string,std::string>& delivery_table, NameMatcher& nameMatcher,GraphPtr gPtr)
{
    pathFinder pf;
    std::vector<graphPlanResultPtr> out_vec(0);
    
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
        if(cost_mat(minRow,minCol)==std::numeric_limits<float>::infinity())
            Color::println("No path on graph", Color::WARNING, Color::BG_MAGENTA);

        //std::cout << "row: " << minRow << " col: " << minCol << std::endl;
        else
            Color::println("This may not be the only smallest value",Color::YELLOW);

        graphPlanResult gp(pivot_names[minRow],target_names[minCol],cost_mat(minRow,minCol),paths[minRow][minCol],i->first, minRow, minCol);
        out_vec.push_back(std::make_shared<graphPlanResult>(gp));
    }

    return out_vec;
}

template<typename T>
std::shared_ptr<std::vector<std::vector<T>>> initDoubleVec(int rows, int cols)
{
    std::vector<std::vector<T>> outVec(rows, std::vector<T>(cols, 0));
    return std::make_shared<std::vector<std::vector<T>>>(outVec);
}

std::shared_ptr<nav_msgs::Path> statePath_to_navPath(std::vector<State>& path_in, bool use_mocap = false)
{
    nav_msgs::Path out_path;
    out_path.header.frame_id = params::world_frame;
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

    std::shared_ptr<nav_msgs::Path> out_ptr;

    if(!use_mocap)
        out_ptr = std::make_shared<nav_msgs::Path>(out_path);

    else
    {
        tf::Transform transform = listen_tf("map","map_mocap");

        // Apply the transformation to the path
        nav_msgs::Path transformed_path = transformPath(out_path, transform);
        out_ptr = std::make_shared<nav_msgs::Path>(transformed_path);
    }

    return out_ptr;
}



void init_movable_objects(std::vector<movableObject>& mo_list, int num_push_sides = 4)
{
    /*sim
    mo_list.push_back(movableObject(3,3.5,0,"b1",num_push_sides,gPtr));
    //mo_list.push_back(movableObject(1,3.5,0,"b2",num_push_sides,gPtr));
    mo_list.push_back(movableObject(1,1.5,0,"b2",num_push_sides,gPtr));
    mo_list.push_back(movableObject(1,2,0,"b3",num_push_sides,gPtr));
    mo_list.push_back(movableObject(0.5,3.5,0,"b4",num_push_sides,gPtr));
    */

    /*
    mo_list.push_back(movableObject(3.5,3.5,0,"b1",num_push_sides));
    //mo_list.push_back(movableObject(1,3.5,0,"b2",num_push_sides,gPtr));
    //mo_list.push_back(movableObject(1,1.5,0,"b2",num_push_sides));
    mo_list.push_back(movableObject(1.5,2,0,"b3",num_push_sides));
    //mo_list.push_back(movableObject(1,3.5,0,"b4",num_push_sides));
    */

    /* 2 obs 1 relo case */
    //mo_list.push_back(movableObject(2.2,3.5,0,"b1",num_push_sides));
    //mo_list.push_back(movableObject(1,3.5,0,"b3",num_push_sides));

    /* 3 obs 0 relo case */
    //mo_list.push_back(movableObject(1,1.5,0,"b1",num_push_sides));
    //mo_list.push_back(movableObject(1.4,2.3,0,"b2",num_push_sides));
    //mo_list.push_back(movableObject(1.2,3.1,0,"b3",num_push_sides));

    // pre-reloc
    //mo_list.push_back(movableObject(2,2.5,0,"b3",num_push_sides));

    /* 5 obs */
    mo_list.push_back(movableObject(1,1.5,0,"b1",num_push_sides));
    mo_list.push_back(movableObject(1.4,2.3,0,"b2",num_push_sides));
    mo_list.push_back(movableObject(2,4,0,"b3",num_push_sides));
    mo_list.push_back(movableObject(3.4,2.3,0,"b4",num_push_sides));
    mo_list.push_back(movableObject(1.58,4.7,0,"b5",num_push_sides));

    /* M */
    //mo_list.push_back(movableObject(0.8,4,0,"b1",num_push_sides));
    //mo_list.push_back(movableObject(1.8,4,0,"b2",num_push_sides));
    //mo_list.push_back(movableObject(2.8,4,0,"b3",num_push_sides));
    //mo_list.push_back(movableObject(3.8,4,0,"b4",num_push_sides));
    //mo_list.push_back(movableObject(0.8,0.8,0,"b5",num_push_sides));
    //mo_list.push_back(movableObject(1.8,0.8,0,"b6",num_push_sides));
    //mo_list.push_back(movableObject(2.8,0.8,0,"b7",num_push_sides));
    //mo_list.push_back(movableObject(3.8,0.8,0,"b8",num_push_sides));
    //mo_list.push_back(movableObject(4.5,3.2,0,"b9",num_push_sides));

}

std::unordered_map<std::string,std::string> init_delivery_table(std::vector<movableObject>& delivery_list, std::vector<movableObject>& mo_list, Environment& env, 
                    graphTools::EdgeMatcher& edgeMatcher, GraphPtr gPtr, int num_push_sides = 4)
{
    /* 2 obs 1 relo case */
    //delivery_list.push_back(movableObject(0,0,0,"d1",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(3,3.5,0,"d3",num_push_sides,gPtr));

    /* 3 obs 0 relo case */
    //delivery_list.push_back(movableObject(3.6,1.5,0,"d1",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(3.8,2.3,0,"d2",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(3.4,3.1,0,"d3",num_push_sides,gPtr));

    /* M */
    //delivery_list.push_back(movableObject(0.8,1.6,0,"d1",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(0.8,2,2,"d2",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(0.8,2.8,0,"d3",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(1.6,2.5,0,"d4",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(2.4,2,2,"d5",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(3.2,2.5,0,"d6",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(4.0,2.8,0,"d7",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(4.0,2,2,"d8",num_push_sides,gPtr));
    //delivery_list.push_back(movableObject(4.0,1.4,0,"d9",num_push_sides,gPtr));

    /* 5 obs */
    delivery_list.push_back(movableObject(3.6,1.5,0,"d1",num_push_sides,gPtr));
    delivery_list.push_back(movableObject(3.8,2.3,0,"d2",num_push_sides,gPtr));
    delivery_list.push_back(movableObject(3.4,3.1,0,"d3",num_push_sides,gPtr));
    delivery_list.push_back(movableObject(2.9,4,3,"d4",num_push_sides,gPtr));
    delivery_list.push_back(movableObject(3.8,5,0,"d5",num_push_sides,gPtr));
    
    // assignment table. object -> delivery
    std::unordered_map<std::string,std::string> delivery_table;
    //delivery_table.insert({"b2","d2"});
    delivery_table.insert({"b1","d1"});
    delivery_table.insert({"b2","d2"});
    delivery_table.insert({"b3","d3"});
    delivery_table.insert({"b4","d4"});
    delivery_table.insert({"b5","d5"});
    //delivery_table.insert({"b6","d6"});
    //delivery_table.insert({"b7","d7"});
    //delivery_table.insert({"b8","d8"});
    //delivery_table.insert({"b9","d9"});

    return delivery_table;
}

void update_graph(std::vector<movableObject>& mo_list, GraphPtr gPtr, bool reset = true)
{
    if(reset)
    {
        gPtr->clear();
    }

    for(auto& it : mo_list)
    {
        it.add_to_graph(gPtr);
    }    
}


void init_static_obstacles(std::unordered_set<State>& obs, std::vector<movableObject>& mo_list,std::vector<movableObject>& delivered_list, bool reset = true)
{
    if(reset)
    {
        obs.clear();
    }

    for(auto& it : mo_list)
    {
        obs.insert(State(it.get_x(),it.get_y(),0));
    }    

    for(auto& it : delivered_list)
    {
        obs.insert(State(it.get_x(),it.get_y(),0));
    }    
}



void add_delivery_to_graph(std::vector<movableObject>& delivery_list, std::vector<movableObject>& mo_list, Environment& env, float max_x, float max_y,
                    graphTools::EdgeMatcher& edgeMatcher, NameMatcher& nameMatcher, std::unordered_map<std::string, std::vector<std::pair<StatePtr,reloDubinsPath>>>& failed_paths, GraphPtr gPtr, std::vector<stopWatch>& time_watches)
{
    // add delivery verteices
    for(auto& it : delivery_list)
        it.add_to_graph(gPtr);
    // add to graph
    reloPush::add_deliveries(delivery_list,mo_list,gPtr,env, max_x, max_y ,Constants::r_push,edgeMatcher, failed_paths, time_watches);
    //add to namematcher
    nameMatcher.addVertices(delivery_list);
}



// euler_zyx, translation
std::pair<Eigen::Vector3f,Eigen::Vector3f> get_real_robotPose(ros::NodeHandle& nh)
{
        // todo: parse frames as params
        std::string from_tf = "map_mocap";
        std::string to_tf = "map";

        // get map tf
        // todo: move it to the beginning of the program to do it only once
        auto transform = listen_tf(from_tf, to_tf);
        auto robotPose_mocap = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/natnet_ros/mushr2/pose", nh);
        std::cout << "Real Robot Pose: " << robotPose_mocap->pose.position.x << ", " << robotPose_mocap->pose.position.y << std::endl;

        if (robotPose_mocap != nullptr) {
            // Robot pose on mocap frame
            Eigen::Vector3f robotPos_mocap(robotPose_mocap->pose.position.x, 
                                            robotPose_mocap->pose.position.y, 
                                            robotPose_mocap->pose.position.z);
            
            Eigen::Quaternionf robotQ_mocap(robotPose_mocap->pose.orientation.w,robotPose_mocap->pose.orientation.x,
                                            robotPose_mocap->pose.orientation.y,robotPose_mocap->pose.orientation.z);

            // to Rotation Matrix
            auto robotR_mocap = jeeho::quaternion_to_rotation_matrix<float>(robotQ_mocap);
            // robot transformation matrix
            auto robotH_mocap = jeeho::homo_matrix_from_R_t<float>(robotR_mocap, robotPos_mocap);

            // TF world to mocap
            Eigen::Vector3f t_w2mocap;
            Eigen::Matrix3f R_w2mocap;
            std::tie(R_w2mocap, t_w2mocap) = jeeho::tf_to_Rt<float>(transform);
            // frame transformation matrix
            auto mocapH = jeeho::homo_matrix_from_R_t(R_w2mocap, t_w2mocap);

            // transform
            auto robotH_w = mocapH * robotH_mocap;
            Eigen::Vector3f t_w;
            Eigen::Matrix3f R_w;
            std::tie(R_w,t_w) = jeeho::R_t_from_homo_matrix<float>(robotH_w);

            // get yaw
            auto euler_zyx = jeeho::rot_matrix_to_euler_ZYX<float>(R_w);

            return std::make_pair(euler_zyx, t_w);
        }

        else {
            ROS_WARN("No message received within the timeout period.");
        }


}

void init_robots(std::vector<State>& robots, ros::NodeHandle& nh,bool use_mocap = false)
{
    if(!use_mocap)
    {
        //robots.push_back(State(0.3, 1, -1*M_PI/2));
        //robots.push_back(State(2, 2.25, 0));
        
        robots.push_back(State(2, 2.5, M_PI/2));
        
        //robots.push_back(State(5, 3, M_PI/2));

        //robots.push_back(State(1,2.5,M_PI/2));
    }
    else
    {
        /*
        // get map tf
        tf::TransformListener listener;

        // todo: parse frames as params
        std::string from_tf = "map_mocap";
        std::string to_tf = "map";

        auto transform = listen_tf(from_tf, to_tf);

        // subscribe robot poses
        boost::shared_ptr<geometry_msgs::PoseStamped const> robotPose_mocap;
        robotPose_mocap = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/natnet_ros/mushr2/pose", nh);

        if (robotPose_mocap != nullptr) {

            // Robot pose on mocap frame
            Eigen::Vector3f robotPos_mocap(robotPose_mocap->pose.position.x, 
                                            robotPose_mocap->pose.position.y, 
                                            robotPose_mocap->pose.position.z);
            
            Eigen::Quaternionf robotQ_mocap(robotPose_mocap->pose.orientation.w,robotPose_mocap->pose.orientation.x,
                                            robotPose_mocap->pose.orientation.y,robotPose_mocap->pose.orientation.z);

            // to Rotation Matrix
            auto robotR_mocap = jeeho::quaternion_to_rotation_matrix<float>(robotQ_mocap);
            // robot transformation matrix
            auto robotH_mocap = jeeho::homo_matrix_from_R_t<float>(robotR_mocap, robotPos_mocap);

            // TF world to mocap
            Eigen::Vector3f t_w2mocap;
            Eigen::Matrix3f R_w2mocap;
            std::tie(R_w2mocap, t_w2mocap) = jeeho::tf_to_Rt<float>(transform);
            // frame transformation matrix
            auto mocapH = jeeho::homo_matrix_from_R_t(R_w2mocap, t_w2mocap);

            // transform
            auto robotH_w = mocapH * robotH_mocap;
            Eigen::Vector3f t_w;
            Eigen::Matrix3f R_w;
            std::tie(R_w,t_w) = jeeho::R_t_from_homo_matrix<float>(robotH_w);

            // get yaw
            auto euler_zyx = jeeho::rot_matrix_to_euler_ZYX<float>(R_w);
            */

            Eigen::Vector3f t_w, euler_zyx;
            std::tie(euler_zyx, t_w) = get_real_robotPose(nh);


            // change yaw to 0~2pi range
            auto yaw_2pi = jeeho::convertEulerRange_to_2pi(euler_zyx.z());

            /*
            // Extract the position
            double x = robotPose_mocap->pose.position.x;
            double y = robotPose_mocap->pose.position.y;
            double z = robotPose_mocap->pose.position.z;

            // Extract the orientation (quaternion)
            double qx = robotPose_mocap->pose.orientation.x;
            double qy = robotPose_mocap->pose.orientation.y;
            double qz = robotPose_mocap->pose.orientation.z;
            double qw = robotPose_mocap->pose.orientation.w;


            ROS_INFO("Received PoseStamped message:");
            ROS_INFO("Position - x: [%f], y: [%f], z: [%f]", x, y, z);
            ROS_INFO("Orientation - x: [%f], y: [%f], z: [%f], w: [%f]", qx, qy, qz, qw);


            // Convert geometry_msgs::PoseStamped to tf::Pose
            tf::Pose tf_pose;
            tf::poseMsgToTF(robotPose_mocap->pose, tf_pose);

            // Apply the transformation
            tf::Pose transformed_pose = transform * tf_pose;

            // Convert tf::Pose back to geometry_msgs::Pose
            geometry_msgs::PoseStamped transformed_pose_stamped = *robotPose_mocap;
            tf::poseTFToMsg(transformed_pose, transformed_pose_stamped.pose);



            Eigen::Quaterniond quat(transformed_pose_stamped.pose.orientation.w,
                            transformed_pose_stamped.pose.orientation.x,
                            transformed_pose_stamped.pose.orientation.y,
                            transformed_pose_stamped.pose.orientation.z);

            Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

            double yaw = euler.z(); // or euler[2]
            */


            //push
            robots.push_back(State(t_w.x(),t_w.y(),yaw_2pi));

    } 
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



// relocation path of an intermediate object
// input: pushing path
//        object to relocate
//        map with obstacles
std::tuple<StatePathsPtr, relocationPair_list, ReloPathInfoList> find_relo_path(std::vector<State>& push_path, std::vector<movableObjectPtr>& relo_list, Environment& env)
{
    // find where to relocate
    // propagate along pushing directions until find one
    // increment by cell resolution (Constants::mapResolution)
    // n candidates

    // generate copied env with path points as obstacles
    Environment temp_env;
    generate_temp_env(env, temp_env, push_path);

    std::vector<StatePath> out_paths(relo_list.size());
    ReloPathInfoList outInfo;
    outInfo.reloPathInfoList.resize(relo_list.size());

    // for movable object update
    relocationPair_list object_relocation;

    // for each movable object
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

        std::vector<bool> out_of_boundary_vec(candidates.size());
        for(size_t i=0; i<out_of_boundary_vec.size(); i++)
            out_of_boundary_vec[i] = false;

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
                auto validity_check = temp_env.stateValid(candidates[n],Constants::carWidth,0.3,Constants::LB,Constants::LF);
                if(validity_check == true) //todo: set better values
                {
                    valid_vec[n] = true;
                    // add break flag
                    valid_position_found = true;
                }
                else
                {
                    if(validity_check.data.second == StateValidity::out_of_boundary)
                    {
                        out_of_boundary_vec[n] = true;
                    }
                }
            }

            // break if a point is found
            if(valid_position_found)
                break;

            if(std::all_of(out_of_boundary_vec.begin(), out_of_boundary_vec.end(), [](bool v) { return v; }))
            {
                // all candidates expanded out of boundary. search failed
                return std::make_tuple(nullptr,relocationPair_list(), ReloPathInfoList());
            }
        }

        // pick a valid pose if multiple // todo: find a better way to handle multiple candidates
        int valid_ind = findFirstTrueIndex(valid_vec);

        // path from init pre-push to target pre-push
        State start_pre_push = find_pre_push(*init_pusing_poses[valid_ind], params::pre_push_dist);
        State goal_pre_push = find_pre_push(candidates[valid_ind], params::pre_push_dist);

        // angle range to 0 ~2 pi
        start_pre_push.yaw = jeeho::convertEulerRange_to_2pi(start_pre_push.yaw);
        goal_pre_push.yaw = jeeho::convertEulerRange_to_2pi(goal_pre_push.yaw);
        
        //
        object_relocation.push_back(std::make_pair(relo_list[m]->get_name(), candidates[valid_ind]));

        // add new location as obstacle
        temp_env.add_obs(candidates[valid_ind]);

        // add this path
        out_paths[m] = std::vector<State>({start_pre_push,goal_pre_push});
        // add to path info
        auto path_to_reloc = std::vector<State>({start_pre_push,goal_pre_push});
        ReloPathInfo p(std::make_shared<std::vector<State>>(path_to_reloc),relo_list[m]->get_name(),start_pre_push,goal_pre_push);
        outInfo.reloPathInfoList[m] = p;
    }

    //return std::make_pair(std::make_shared<std::vector<StatePath>>(out_paths), object_relocation);
    return std::make_tuple(std::make_shared<std::vector<std::vector<State>>>(out_paths), object_relocation, outInfo);
}

std::pair<StatePathPtr,PathInfoList> get_push_path(std::vector<Vertex>& vertex_path, 
                                  graphTools::EdgeMatcher& edgeMatcher, GraphPtr gPtr, size_t& num_prereloc, Environment& env, std::vector<stopWatch>& time_watches)
{
    std::vector<State> push_path(0);
    PathInfoList plist;

    num_prereloc = 0;

    // augment dubins path
    // don't do multi-processing
    for(size_t i=vertex_path.size()-1; i>0; i--)
    {
        //find edge //todo: handle multiple edges
        Vertex source = vertex_path[i];
        Vertex target = vertex_path[i-1];
        Edge edge = boost::edge(source, target, *gPtr).first;
        auto test_ = boost::edge(source, target, *gPtr).second;

        // delivering object
        std::string vName = graphTools::getVertexName(vertex_path[0],gPtr);

        //print vetex names
        //std::cout << graphTools::getVertexName(source, gPtr) << " -> " << graphTools::getVertexName(target,gPtr) << std::endl; 

        // corresponding path
        auto partial_path_info = edgeMatcher.getPath(edge);
        // count pre-relocation
        num_prereloc += partial_path_info.pre_relocations.size();
        //auto pivot_state = nameMatcher.getVertexStatePair(min_list[mcol]->sourceVertexName)->state;

        /* hold this modification*/
        /*
        // find dubins with pre-push (arrival) as goal
        // todo: check validity of this arrival pre-push
        auto arrival_prepush = find_pre_push(partial_path_info.targetState);
        auto dubins_with_arrival_prepush = findDubins(partial_path_info.sourceState,arrival_prepush,partial_path_info.path.get_turning_radius());

        // update with new dubins path
        partial_path_info.update_dubins(dubins_with_arrival_prepush);
        */

       ///////// Check if Motion is feasible (i.e. is new pose accessible)
        for(auto& it : partial_path_info.pre_relocations) //first: source second: target
        {
            if(it.pathToNextPush->states.size() == 0) // todo: this could be set more explicitly on earlier stage
            {
                auto obs_rm = it.pathToNextPush->obs_rm;
                auto obs_add = it.pathToNextPush->obs_add;
                // temporarily modify env
                env.remove_obs(obs_rm);
                env.add_obs(obs_add);

                stopWatch time_mp("preReloc", measurement_type::pathPlan);
                it.pathToNextPush = planHybridAstar(it.pathToNextPush->start_pose, it.pathToNextPush->goal_pose, env, params::grid_search_timeout, false);
                time_mp.stop();
                time_watches.push_back(time_mp);

                // restore env
                env.remove_obs(obs_add);
                env.add_obs(obs_rm);
                if(!it.pathToNextPush->success)
                {
                    // this plan is infeasible
                    return std::make_pair(nullptr, plist);
                }

                it.pathToNextPush->obs_add = obs_add;
                it.pathToNextPush->obs_rm = obs_rm;
            }
        }

        // get interpolated list (prepath + final path)
        auto interp_path_pair = interpolate_dubins(partial_path_info, params::interpolation_step, vName);
        auto interp_path = interp_path_pair.first;
        auto interp_pathinfo = interp_path_pair.second;
        
        // omit last steps for arrival
        auto last_pose = interp_path->end();
        if(interp_path->size() > 1) // todo: find this better (i.e. slightly change last pose)
            last_pose = interp_path->end()-1;

        push_path.insert(push_path.end(), interp_path->begin(), last_pose);
        plist.append(interp_pathinfo);
    }    

    // add pose-push pose
    // todo: do it better
    State post_push;
    int post_ind = find_post_push_ind(push_path,env);
    if(push_path.size()>=std::abs(post_ind))
        post_push = push_path.end()[post_ind];
    //else
    //    post_push = push_path[0];
    push_path.push_back(post_push);
    plist.paths.back().path.push_back(post_push); // add to path info as well

    //return final_path, pathinfo list;
    return std::make_pair(std::make_shared<StatePath>(push_path),plist);
}

// path info as input, final list as output
std::pair<std::vector<State>,bool> combine_relo_push(std::vector<State>& push_path, std::vector<std::vector<State>>& relo_path,
                                                    PathInfoList& push_pathinfo, ReloPathInfoList& relo_pathinfo, PathInfoList& final_pathinfo,
                                                    State& robot, Environment& env, std::vector<movableObjectPtr>& relo_list)
{
    std::vector<std::vector<State>> path_segments;
    // one or more temp relocations
    if(relo_path.size()>0)
    {
        // plan approach to first temp relocation
        auto plan_res =planHybridAstar(robot, relo_path[0][0], env, params::grid_search_timeout,false);
        if(!plan_res->success)
            return std::make_pair(std::vector<State>(0),false); // return false

        auto planned_path = plan_res->getPath(true);        
        // start to first relo
        path_segments.push_back(planned_path);
        path_segments.push_back(relo_path[0]);
        // save to path info
        auto rpath = relo_pathinfo.reloPathInfoList[0];
        PathInfo p_first_app(rpath.vertexName,moveType::app,robot,relo_path[0][0],planned_path);
        final_pathinfo.push_back(p_first_app);        
        PathInfo p_first_relo(rpath.vertexName, moveType::temp, rpath.fromPose, rpath.toPose, *rpath.statePathPtr);
        final_pathinfo.push_back(p_first_relo);

#pragma region modify_env_rm
        // remove pushed object
        auto moPtr = relo_list[0];
        env.remove_obs(State(moPtr->get_x(), moPtr->get_y(), moPtr->get_th()));
        // relocated
        auto relocated_obs = find_post_push(relo_path[0].back());
        env.add_obs(relocated_obs);
#pragma endregion

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
            plan_res = planHybridAstar(lastThisRelo, firstNextRelo, env, params::grid_search_timeout, false);
            if(!plan_res->success)
                return std::make_pair(std::vector<State>(0),false); // return false

            auto _path = plan_res->getPath(true);
            path_segments.push_back(_path);
            path_segments.push_back(relo_path[p+1]);
            // save to path info
            auto rpath = relo_pathinfo.reloPathInfoList[p+1];
            PathInfo p_app(rpath.vertexName,moveType::app,lastThisRelo, firstNextRelo,_path);
            final_pathinfo.push_back(p_app);        
            PathInfo p_relo(rpath.vertexName, moveType::temp, rpath.fromPose, rpath.toPose, *rpath.statePathPtr);
            final_pathinfo.push_back(p_relo);

#pragma region modify_env_restore
            // put it back
            env.add_obs(find_post_push(relo_path[p+1].back()));
#pragma endregion
        }

        // relo to push
        auto lastLastRelo = relo_path.back().back();
        // first pre-push
        auto pre_push = find_pre_push(push_path.front(), params::pre_push_dist);

        // plan hybrid astar
        plan_res = planHybridAstar(lastLastRelo, pre_push, env, params::grid_search_timeout, false);
        if(!plan_res->success)
            return std::make_pair(std::vector<State>(0),false); // return false

        auto final_app_path = plan_res->getPath(true);

        path_segments.push_back(final_app_path);
        path_segments.push_back(push_path);
        // save to path info
        PathInfo p_final_app(push_pathinfo.paths.back().vertexName,moveType::app,lastLastRelo, pre_push,final_app_path);
        final_pathinfo.push_back(p_final_app); // add final approach path

        final_pathinfo.append(push_pathinfo); // final delivery path
    }

    // no temp relocation
    else{
        // start to prepush
        auto pre_push = find_pre_push(push_path.front(),params::pre_push_dist);
        //stopWatch hb("hyb");
        auto plan_res = planHybridAstar(robot, pre_push, env, params::grid_search_timeout, false);
        if(!plan_res->success)
                return std::make_pair(std::vector<State>(0),false); // return false

        auto final_app_path = plan_res->getPath(true);
        path_segments.push_back(final_app_path);
        path_segments.push_back(push_path);
        // save to path info
        PathInfo p_final_app(push_pathinfo.paths.back().vertexName, moveType::app, robot, pre_push, final_app_path);
        final_pathinfo.push_back(p_final_app); // approach to delivering object

        final_pathinfo.append(push_pathinfo);
    }

    std::vector<State> combinedVector;
    for (const auto& subVector : path_segments) {
        combinedVector.insert(combinedVector.end(), subVector.begin(), subVector.end());
    }
    
    return std::make_pair(combinedVector,true);
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
            auto pre_push = find_pre_push(*approach_state, params::pre_push_dist);

            auto plan_res = planHybridAstar(robots[r], pre_push, env, params::grid_search_timeout, false);

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
        auto interp_list = interpolate_dubins(partial_path_info, params::interpolation_step);
        StatePath sp = *interp_list.first;
        final_path.insert(final_path.end(), sp.begin(), sp.end());
    }    

    //return final_path;
    return statePath_to_navPath(final_path);
}

void update_mo_list(std::vector<movableObject>& mo_list, relocationPair_list& relocPair)
{
    for(size_t n=0; n<relocPair.size(); n++)
    {
        auto objName = relocPair[n].first;

        for(auto& it: mo_list)
        {
            if(it.get_name() == objName)
            {
                it.set_x(relocPair[n].second.x);
                it.set_y(relocPair[n].second.y);
                it.update_pushing_poses();
                break;
            }
        }
        //mo_list[relocPair[n].first].set_x(relocPair[n].second.x);
        //mo_list[relocPair[n].first].set_y(relocPair[n].second.y);
        //relocPair[n].first->set_th(relocPair[n].second.yaw);
    }
}

float path_length(StatePath path)
{
    float totalLength = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x - path[i-1].x;
        float dy = path[i].y - path[i-1].y;
        totalLength += std::sqrt(dx * dx + dy * dy);
    }

    return totalLength;
}

int find_min_path(std::vector<graphPlanResultPtr>& min_list)
{
    int min_list_ind = -1;
    float pivot_cost = std::numeric_limits<float>::infinity();

    for(int i=0; i<min_list.size(); i++)
    {
        //if(min_list[i]->cost != std::numeric_limits<float>::infinity())
        //{
            if(min_list[i]->cost < pivot_cost)
            {
                min_list_ind = i;
                pivot_cost = min_list[i]->cost;
            }
        //}
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

    else if(argc==5)
    {
        data_file = std::string(argv[1]);
        data_ind = std::atoi(argv[2]);        
        params::leave_log = std::atoi(argv[3]);
        //params::use_better_path = std::atoi(argv[4]);
        std::string mode = std::string(argv[4]);
        params::use_mp_only = false;
        if(mode == "proposed")
            params::use_better_path = true;

        else if(mode == "dubins_only")
            params::use_better_path = false;

        else if(mode == "mp_only")
            params::use_mp_only = true;
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
    Color::println("== use real robot: " + std::to_string(params::use_mocap) + " ===", Color::BG_YELLOW);
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
        auto vis_path_msg = draw_paths(edgeMatcher,env,failed_paths,dubins_path_pub_ptr,failed_path_pub_ptr,Constants::r_push);
        // visualize delivery locations
        auto vis_deli_msg = draw_deliveries(delivery_list,delivery_marker_pub_ptr);
        // visualize object names
        auto vis_names_msg = draw_texts(initMOList,text_pub_ptr);

        ros::spinOnce();
        ros::Duration(0.5).sleep();                
    }
}



ReloPathResult iterate_remaining_deliveries(Environment& env, StatePath& push_path, strMap& delivery_table,
                                                                std::vector<State>& robots, relocationPair_list& relocPair, NameMatcher& nameMatcher, 
                                                                graphTools::EdgeMatcher& edgeMatcher, std::vector<stopWatch>& time_watches, GraphPtr gPtr)
{
    stopWatch time_search("cost_mat", measurement_type::graphPlan);
    auto costMat_vertices_pairs = get_cost_mat_vertices_pair(delivery_table, nameMatcher, gPtr, env);
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

        // find first non-inf ind
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
   
        auto push_path_pair = get_push_path(min_list[min_list_ind]->path, edgeMatcher, gPtr, num_prereloc, env, time_watches);
        if(push_path_pair.first == nullptr)
        {
            // relo path search failed
            Color::println("PrePath Failed", Color::RED, Color::BG_YELLOW);
            //return reloPlanResult(false); // todo: don't return unless this is the only one left

            // mark this path as not feasible by raising cost
            costMat_vertices_pairs[min_list_ind].first->operator()(min_list[min_list_ind]->min_row, min_list[min_list_ind]->min_col) = std::numeric_limits<float>::infinity();

            // repeat
            Color::println("REPLAN", Color::RED,Color::BG_YELLOW);
            continue;
        }
        push_path = *push_path_pair.first;
        PathInfoList push_pathinfo = push_path_pair.second;
        //time_path_gen_push_path.stop();
        //time_watches.push_back(time_path_gen_push_path);

        // count
        //count_pre_relocs += num_prereloc;

        // relocation paths
        StatePathsPtr relo_paths;
        ReloPathInfoList relo_pathinfo;
        stopWatch time_path_gen_relo_path("relocate", measurement_type::relocatePlan);
        // list of state paths for temporary relocation of intermediate objects
        std::tie(relo_paths, relocPair, relo_pathinfo) = find_relo_path(push_path, reloc_objects, env); // pre-relocation path
        time_path_gen_relo_path.stop();
        time_watches.push_back(time_path_gen_relo_path);

        if(relo_paths == nullptr)
        {
            // relo path search failed
            Color::println("ReloPath Failed", Color::RED, Color::BG_YELLOW);
            //return reloPlanResult(false); // todo: don't return unless this is the only one left

            // mark this path as not feasible by raising cost
            costMat_vertices_pairs[min_list_ind].first->operator()(min_list[min_list_ind]->min_row, min_list[min_list_ind]->min_col) = std::numeric_limits<float>::infinity();

            // repeat
            Color::println("REPLAN", Color::RED,Color::BG_YELLOW);
            continue;
        }

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
            continue;
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

#endif