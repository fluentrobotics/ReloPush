#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        std::cout << "Received Sim Path: " << msg->poses.size() << std::endl;
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

        // Publish the transformed path
        path_pub_.publish(transformed_path);
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

    ros::Subscriber path_sub_;
    ros::Publisher path_pub_;
    ros::Subscriber pose_sub_;
    ros::Publisher pose_pub_;
    geometry_msgs::TransformStamped transform_;
    geometry_msgs::TransformStamped inverse_transform_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_and_pose_transformer");

    PathAndPoseTransformer ppt;

    ros::spin();

    return 0;
}
