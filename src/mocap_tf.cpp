#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/String.h>
#include <reloPush/hexNfloat.h>
//#include <reloPush/reloPush_tools.h>
#include <reloPush/params.h>
#include <pathPlanTools/path_planning_tools.h>
#include <omplTools/State.h>
#include <memory>

#include <pathPlanTools/tf_tools.h>

class PathAndPoseTransformer
{
public:
    PathAndPoseTransformer()
    {
        // Initialize ROS node
        ros::NodeHandle nh;

        // Subscribe to the original path
        path_sub_ = nh.subscribe("/mushr2/sim_trajectory", 10, &PathAndPoseTransformer::pathCallback, this);

        // Publisher for the transformed path
        path_pub_ = nh.advertise<nav_msgs::Path>("/mushr2/planned_trajectory", 10);

        // Subscribe to the PoseStamped in map_mocap
        pose_sub_ = nh.subscribe("/natnet_ros/mushr2/pose", 10, &PathAndPoseTransformer::poseCallback, this);

        // Publisher for the transformed PoseStamped
        pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mushr2/mocap_pose", 10);

        // Subscriber for serialized path info
        path_str_sub_ = nh.subscribe("/mocap/mushr2/planned_path_serialized", 10, &PathAndPoseTransformer::pathStrCallback, this);
        path_str_pub_ = nh.advertise<std_msgs::String>("/mushr2/planned_path_serialized",10);

        // Define the static transformation from map to map_mocap
        tf2::Transform transform;
        transform.setOrigin(tf2::Vector3(2.1, 2.65, 0.0));
        tf2::Quaternion q;
        q.setRPY(0, 0, -180/180*3.14159);
        transform.setRotation(q);

        // Convert tf2::Transform to geometry_msgs::TransformStamped
        transform_.header.frame_id = "map";
        transform_.child_frame_id = "map_mocap";
        transform_.transform.translation.x = transform.getOrigin().x();
        transform_.transform.translation.y = transform.getOrigin().y();
        transform_.transform.translation.z = transform.getOrigin().z();
        transform_.transform.rotation.x = transform.getRotation().x();
        transform_.transform.rotation.y = transform.getRotation().y();
        transform_.transform.rotation.z = transform.getRotation().z();
        transform_.transform.rotation.w = transform.getRotation().w();

        // Inverse transform for map_mocap to map
        tf2::Transform inverse_transform = transform.inverse();

        // Convert tf2::Transform to geometry_msgs::TransformStamped for inverse transform
        inverse_transform_.header.frame_id = "map_mocap";
        inverse_transform_.child_frame_id = "map";
        inverse_transform_.transform.translation.x = inverse_transform.getOrigin().x();
        inverse_transform_.transform.translation.y = inverse_transform.getOrigin().y();
        inverse_transform_.transform.translation.z = inverse_transform.getOrigin().z();
        inverse_transform_.transform.rotation.x = inverse_transform.getRotation().x();
        inverse_transform_.transform.rotation.y = inverse_transform.getRotation().y();
        inverse_transform_.transform.rotation.z = inverse_transform.getRotation().z();
        inverse_transform_.transform.rotation.w = inverse_transform.getRotation().w();

        // Publish the static transform
        static_broadcaster_.sendTransform(transform_);
    }

private:

    std::shared_ptr<nav_msgs::Path> TransformPath(const nav_msgs::Path& path_in)
    {
        nav_msgs::Path transformed_path;
        transformed_path.header.frame_id = "map_mocap";
        transformed_path.header.stamp = ros::Time::now();

        for (const auto &pose : path_in.poses)
        {
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(pose, transformed_pose, transform_);
            transformed_pose.header = pose.header;
            transformed_path.poses.push_back(transformed_pose);
        }

        return std::make_shared<nav_msgs::Path>(transformed_path);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        std::cout << "Received Sim Path: " << msg->poses.size() << std::endl;
        
        auto tfed_path = TransformPath(*msg);

        /*
        nav_msgs::Path transformed_path;
        transformed_path.header.frame_id = "map_mocap";
        transformed_path.header.stamp = ros::Time::now();

        for (const auto& pose : msg->poses)
        {
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(pose, transformed_pose, transform_);
            transformed_pose.header = pose.header;
            transformed_path.poses.push_back(transformed_pose);
        }
        */

        // Publish the transformed path
        path_pub_.publish(*tfed_path);
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped transformed_pose;
        tf2::doTransform(*msg, transformed_pose, inverse_transform_);
        transformed_pose.header.frame_id = "map";
        transformed_pose.header.stamp = ros::Time::now();

        // Publish the transformed pose
        pose_pub_.publish(transformed_pose);
    }

    std::shared_ptr<nav_msgs::Path> statePath_to_navPath(std::vector<State> &path_in, bool use_mocap = false)
    {
        nav_msgs::Path out_path;
        out_path.header.frame_id = params::world_frame;
        out_path.poses.resize(path_in.size());

        out_path.header.stamp = ros::Time::now();

        for (size_t n = 0; n < path_in.size(); n++)
        {
            out_path.poses[n].pose.position.x = path_in[n].x;
            out_path.poses[n].pose.position.y = path_in[n].y;
            out_path.poses[n].pose.position.z = 0;
            out_path.poses[n].pose.orientation = jeeho::eigenQtoMsgQ(jeeho::euler_to_quaternion_xyz(0, 0, path_in[n].yaw));

            // timing
            if (n > 0)
            {
                // delta distance
                double dx = out_path.poses[n].pose.position.x - out_path.poses[n - 1].pose.position.x;
                double dy = out_path.poses[n].pose.position.y - out_path.poses[n - 1].pose.position.y;
                // double dz = out_path.poses[n].pose.position.z - out_path.poses[n-1].pose.position.z;
                auto dist = std::sqrt(dx * dx + dy * dy);

                // delta time
                auto dTime = (float)dist / Constants::speed_limit;

                out_path.poses[n].header.stamp = out_path.poses[n - 1].header.stamp + ros::Duration(dTime);
            }
            else
                out_path.poses[n].header.stamp = ros::Time::now();
        }

        std::shared_ptr<nav_msgs::Path> out_ptr;
        out_ptr = std::make_shared<nav_msgs::Path>(out_path);

        /*
        if (!use_mocap)
            out_ptr = std::make_shared<nav_msgs::Path>(out_path);

        else
        {
            tf::Transform transform = listen_tf("map", "map_mocap");

            // Apply the transformation to the path
            nav_msgs::Path transformed_path = transformPath(out_path, transform);
            out_ptr = std::make_shared<nav_msgs::Path>(transformed_path);
        }
        */

        return out_ptr;
    }

    std::tuple<std::shared_ptr<nav_msgs::Path>, std::shared_ptr<std::vector<std::string>>,std::shared_ptr<std::string>> parse_path_str(const std::string &str_in)
    {
        auto group_sp_str = jeeho::split(str_in, jeeho::group_delim);
        std::string& path_str = group_sp_str[0];
        std::string& mode_str = group_sp_str[1];
        std::string& timing_str = group_sp_str[2];

        // parse path
        auto path_str_vec = jeeho::split(path_str,jeeho::data_delim);

        std::vector<State> stateVec(path_str_vec.size());
        for (size_t n = 0; n < path_str_vec.size(); n++)
        {
            auto temp_str = jeeho::split(path_str_vec[n],jeeho::elem_delim);
            float parsed_x = jeeho::hexstr2float(temp_str[0]);
            float parsed_y = jeeho::hexstr2float(temp_str[1]);
            float parsed_yaw = jeeho::hexstr2float(temp_str[2]);
            stateVec[n] = State(parsed_x, parsed_y, parsed_yaw);
        }
        // convert to nav_path
        auto parsedPathPtr = statePath_to_navPath(stateVec,false); // not transformed


        // parse mode str
        std::vector<std::string> modeVec = jeeho::split(mode_str,jeeho::data_delim);

        return std::make_tuple(parsedPathPtr, std::make_shared<std::vector<std::string>>(modeVec),std::make_shared<std::string>(timing_str));
    }

    std::string serializePathMode(const nav_msgs::Path& p_in, const std::vector<std::string>& m_in, std::shared_ptr<std::string> timing_str_ptr)
    {
        std::string out_str = "";

        for (size_t n = 0; n < p_in.poses.size(); n++)
        {
            auto &it = p_in.poses[n];
            auto x_hex_str = jeeho::float2hexstr(it.pose.position.x);
            auto y_hex_str = jeeho::float2hexstr(it.pose.position.y);

            auto q_eig = Eigen::Quaternionf(it.pose.orientation.w, it.pose.orientation.x,
                                            it.pose.orientation.y, it.pose.orientation.z);
            auto euler_zyx = jeeho::quaternion_to_euler_zyx<float>(q_eig);

            auto yaw_hex_str = jeeho::float2hexstr(euler_zyx[2]); // quaternion to eular

            std::string temp_str = x_hex_str + jeeho::elem_delim + y_hex_str + jeeho::elem_delim + yaw_hex_str; // x,y,yaw
            out_str += temp_str;
            if (n != p_in.poses.size() - 1)
                out_str += jeeho::data_delim;
        }

        out_str += jeeho::group_delim;

        for (size_t n = 0; n < m_in.size(); n++)
        {
            std::string it = m_in[n];

            out_str += it;
            if (n != m_in.size() - 1)
                out_str += jeeho::data_delim;
        }

        /*
        // add timing. First pose is 0
        out_str += jeeho::group_delim + jeeho::float2hexstr(0) + jeeho::data_delim;
        float last_t = 0;
        // do not use MP
        for (size_t n = 1; n < p_in.poses.size(); n++)
        {
            // delta distance
            float dx = p_in.poses[n].pose.position.x - p_in.poses[n-1].pose.position.x;
            float dy = p_in.poses[n].pose.position.y - p_in.poses[n-1].pose.position.y;
            // double dz = out_path.poses[n].pose.position.z - out_path.poses[n-1].pose.position.z;
            auto dist = std::sqrt(dx * dx + dy * dy);

            // delta time
            auto dTime = dist / Constants::speed_limit;
            last_t += dTime;
            out_str += jeeho::float2hexstr(last_t);1
            if (n != p_in.poses.size() - 1)
                out_str += jeeho::data_delim;
        }
        */

       // use original timing
       out_str += jeeho::group_delim + *timing_str_ptr;

        // add frame name
        out_str += jeeho::group_delim + "map_mocap";

        return out_str;
    }

    void pathStrCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::cout << "path info in" << std::endl;
        // parse data
        std::shared_ptr<nav_msgs::Path> path_ptr;
        std::shared_ptr<ros::V_string> mode_ptr;
        std::shared_ptr<std::string> timing_str;
        std::tie(path_ptr,mode_ptr,timing_str)= parse_path_str(msg->data);

        // transform
        auto tfed_path = TransformPath(*path_ptr);

        // re-serialize
        std::string out_str = serializePathMode(*tfed_path,*mode_ptr,timing_str);
        std_msgs::String out_str_msg;
        out_str_msg.data = out_str;

        // publish
        path_str_pub_.publish(out_str_msg);
    }

    ros::Subscriber path_sub_;
    ros::Publisher path_pub_;
    ros::Subscriber pose_sub_;
    ros::Publisher pose_pub_;
    ros::Subscriber path_str_sub_;
    ros::Publisher path_str_pub_;
    geometry_msgs::TransformStamped transform_;
    geometry_msgs::TransformStamped inverse_transform_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_and_pose_transformer");

    PathAndPoseTransformer ppt;


    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    //ros::spin();

    return 0;
}
