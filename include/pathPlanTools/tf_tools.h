// for ARM-based systems such as Macbook (on VM)
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#ifndef TF_TOOLS_H
#define TF_TOOLS_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>


// Define the operator<< function for Eigen::QuaternionBase
template<typename Derived>
std::ostream& operator<<(std::ostream& s, const Eigen::QuaternionBase<Derived>& q) {
    s << q.x() << "i + " << q.y() << "j + " << q.z() << "k" << " + " << q.w();
    return s;
}

namespace jeeho
{
    Eigen::Quaternionf msgQtoEigenQ(geometry_msgs::Quaternion q);

    geometry_msgs::Quaternion eigenQtoMsgQ(Eigen::Quaternionf q);

    Eigen::Matrix4f geometryPose_to_Eigen(geometry_msgs::Pose p);

    Eigen::Quaternionf euler_to_quaternion_xyz(float roll, float pitch, float yaw);

    float convertEulerRange_to_2pi(float angle);

    float convertEulerRange_to_pi(float yaw);

    // ROS TF matrix to Eigen matrix
    template<typename T>
    void matrixTFToEigen(const tf::Matrix3x3& tf_matrix, Eigen::Matrix<T,3,3>& eigen_matrix);

    // quaternion to rotation matrix
    template<typename T>
    Eigen::Matrix<T, 3, 3> quaternion_to_rotation_matrix(Eigen::Quaternion<T>& q_in);

    // rotation matrix to quaternion
    template<typename T>
    Eigen::Quaternion<T> rotation_matrix_to_quaternion(Eigen::Matrix<T, 3, 3>& rmat);

    // euler to quaternion
    // input vector: {roll, pitch, yaw}
    template<typename T>
    Eigen::Quaternion<T> euler_to_quaternion_zyx(Eigen::Matrix<T, 3, 1>& euler_in);

    // quaternion to euler
    // out: {roll, pitch, yaw} -pi ~ pi
    template<typename T>
    Eigen::Matrix<T, 3, 1> quaternion_to_euler_zyx(Eigen::Quaternion<T>& q_in);

    // ROS tf transform to Tranformation Matrix
    template<typename T>
    std::pair<Eigen::Matrix<T,3,3>, Eigen::Matrix<T,3,1>> tf_to_Rt(tf::StampedTransform trans);

    template<typename T>
    std::pair<Eigen::Quaternion<T>, Eigen::Matrix<T,3,1>> tf_to_Qt(tf::StampedTransform trans);

    template<typename T>
    Eigen::Matrix<T,4,4> homo_matrix_from_R_t(Eigen::Matrix<T,3,3> R, Eigen::Matrix<T,3,1> t);

    template<typename T>
    std::pair<Eigen::Matrix<T,3,3>, Eigen::Matrix<T,3,1>> R_t_from_homo_matrix(Eigen::Matrix<T,4,4> H);

    template<typename T>
    Eigen::Matrix<T,3,1> rot_matrix_to_euler_ZYX(Eigen::Matrix<T,3,3> rmat);
    

} // namespace jeeho
#include <tf_tools.inl>

#endif