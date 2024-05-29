#include <pathPlanTools/tf_tools.h>

namespace jeeho
{
    Eigen::Quaternionf msgQtoEigenQ(geometry_msgs::Quaternion q)
    {
        // w x y z
        Eigen::Quaternionf outQ(q.w, q.x, q.y, q.z);

        return outQ;
    }

    geometry_msgs::Quaternion eigenQtoMsgQ(Eigen::Quaternionf q)
    {
        geometry_msgs::Quaternion outQ;
        outQ.w = q.w();
        outQ.x = q.x();
        outQ.y = q.y();
        outQ.z = q.z();

        return outQ;
    }

    Eigen::Matrix4f geometryPose_to_Eigen(geometry_msgs::Pose p)
    {
        Eigen::Affine3d out_aff;
        tf::poseMsgToEigen(p,out_aff);
        return out_aff.matrix().cast<float>();
    }

    Eigen::Quaternionf euler_to_quaternion_xyz(float roll, float pitch, float yaw)
    {
        // Convert roll, pitch, and yaw angles to quaternion
        Eigen::Quaternionf q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

        // Print the resulting quaternion
        //std::cout << "Quaternion: " << q.coeffs().transpose() << std::endl;
        return q;
    }

    float convertEulerRange_to_2pi(float angle) {
        if (angle < 0) {
            return angle + 2 * M_PI;
        } else {
            return angle;
        }
    }

    float convertEulerRange_to_pi(float yaw) {
        if (yaw > M_PI) {
            yaw -= 2 * M_PI;
        }
        return yaw;
    }
}