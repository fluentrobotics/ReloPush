#include <pathPlanTools/tf_tools.h>

namespace jeeho{
    // TF matrix to Eigen matrix
    template<typename T>
    void matrixTFToEigen(const tf::Matrix3x3& tf_matrix, Eigen::Matrix<T,3,3>& eigen_matrix) {
        // Populate the Eigen matrix with values from the tf::Matrix3x3
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                eigen_matrix(i, j) = static_cast<T>(tf_matrix[i][j]);
            }
        }
    }

    // quaternion to rotation matrix
    template<typename T>
    Eigen::Matrix<T, 3, 3> quaternion_to_rotation_matrix(Eigen::Quaternion<T>& q_in) {
        return q_in.normalized().toRotationMatrix();
    }

    // rotation matrix to quaternion
    template<typename T>
    Eigen::Quaternion<T> rotation_matrix_to_quaternion(Eigen::Matrix<T, 3, 3>& rmat)
    {
        return Eigen::Quaternion<T>(rmat);
    }


    // euler to quaternion
    // input vector: {roll, pitch, yaw}
    template<typename T>
    Eigen::Quaternion<T> euler_to_quaternion_zyx(Eigen::Matrix<T, 3, 1>& euler_in)
    {
        return Eigen::AngleAxis<T>(euler_in.z(), Eigen::Matrix<T, 3, 1>::UnitZ())
            * Eigen::AngleAxis<T>(euler_in.y(), Eigen::Matrix<T, 3, 1>::UnitY())
            * Eigen::AngleAxis<T>(euler_in.x(), Eigen::Matrix<T, 3, 1>::UnitX());
    }

    // quaternion to euler
    // out: {roll, pitch, yaw} -pi ~ pi
    template<typename T>
    Eigen::Matrix<T, 3, 1> quaternion_to_euler_zyx(Eigen::Quaternion<T>& q_in)
    {
        Eigen::Matrix<T,3,1> angles;    //yaw pitch roll
        const auto x = q_in.x();
        const auto y = q_in.y();
        const auto z = q_in.z();
        const auto w = q_in.w();

        // roll (x-axis rotation)
        T sinr_cosp = 2 * (w * x + y * z);
        T cosr_cosp = 1 - 2 * (x * x + y * y);
        angles[0] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        T sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            angles[1] = std::copysign(3.1415926 / 2, sinp); // use 90 degrees if out of range
        else
            angles[1] = std::asin(sinp);

        // yaw (z-axis rotation)
        T siny_cosp = 2 * (w * z + x * y);
        T cosy_cosp = 1 - 2 * (y * y + z * z);
        angles[2] = std::atan2(siny_cosp, cosy_cosp);
        return angles;
    }

    // ROS tf transform to Tranformation Matrix
    template<typename T>
    std::pair<Eigen::Matrix<T,3,3>, Eigen::Matrix<T,3,1>> tf_to_Rt(tf::StampedTransform trans)
    {
        tf::Quaternion trans_ori = trans.getRotation();
        tf::Vector3 trans_trans = trans.getOrigin();

        tf::Matrix3x3 rot_m(trans_ori);
        Eigen::Matrix<T,3,3> rot_m_eigen;
        //tf::matrixTFToEigen(rot_m, rot_m_eigen);
        jeeho::matrixTFToEigen<float>(rot_m, rot_m_eigen);

        Eigen::Matrix<T,3,1> trans_v(trans_trans.getX(), trans_trans.getY(), trans_trans.getZ());
        //tf::vectorTFToEigen(trans_trans, trans_v);


        return std::make_pair(rot_m_eigen, trans_v);
    }

    template<typename T>
    std::pair<Eigen::Quaternion<T>, Eigen::Matrix<T,3,1>> tf_to_Qt(tf::StampedTransform trans)
    {
        tf::Quaternion trans_ori = trans.getRotation();
        tf::Vector3 trans_trans = trans.getOrigin();

        Eigen::Quaternion<T> out_q(trans_ori.w(),trans_ori.getX(),trans_ori.getY(),trans_ori.getZ());

        Eigen::Matrix<T,3,1> trans_v;
        tf::vectorTFToEigen(trans_trans, trans_v);

        return std::make_pair(out_q, trans_v);
    }

    template<typename T>
    Eigen::Matrix<T,4,4> homo_matrix_from_R_t(Eigen::Matrix<T,3,3> R, Eigen::Matrix<T,3,1> t)
    {
        Eigen::Matrix<T,4,4> H = Eigen::Matrix<T,4,4>::Identity();
        H.block(0,0,3,3) = R;
        H.block(0,3,3,1) = t;
        return H;
    }

    template<typename T>
    std::pair<Eigen::Matrix<T,3,3>, Eigen::Matrix<T,3,1>> R_t_from_homo_matrix(Eigen::Matrix<T,4,4> H)
    {
        Eigen::Matrix<T,3,3> R;
        Eigen::Matrix<T,3,1> t; 
        R = H.block(0,0,3,3);
        t = H.block(0,3,3,1);
        return std::make_pair(R,t);
    }

    /*
    * out: roll, pitch, yaw
    */
    template<typename T>
    Eigen::Matrix<T,3,1> rot_matrix_to_euler_ZYX(Eigen::Matrix<T,3,3> rmat)
    {
        struct Euler
        {
            double yaw;
            double pitch;
            double roll;
        };
        const double pi = 3.14159265;
        Euler euler_out;
        //check singularity
        if(fabs(rmat(2,0) >= 1))
        {
            euler_out.yaw = 0;
            // From difference of angles formula
            if (rmat(2,0) < 0)  //gimbal locked down
            {
            double delta = atan2(rmat(0,1),rmat(0,2));
                euler_out.pitch = pi / double(2.0);
                euler_out.roll = delta;
            }
            else // gimbal locked up
            {
            double delta = atan2(-rmat(0,1),-rmat(0,2));
                euler_out.pitch = -1 * pi / double(2.0);
                euler_out.roll = delta;
            }
        }

        else
        {
            euler_out.pitch = -1 * asin(rmat(2,0));

            euler_out.roll = atan2(rmat(2,1)/cos(euler_out.pitch),
                rmat(2,2)/cos(euler_out.pitch));

            euler_out.yaw = atan2(rmat(1,0)/cos(euler_out.pitch),
                rmat(0,0)/cos(euler_out.pitch));
        }

        return Eigen::Matrix<T,3,1>(euler_out.roll, euler_out.pitch, euler_out.yaw);
    }
}