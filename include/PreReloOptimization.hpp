#ifndef PRERELOOPTIMIZATION_HPP
#define PRERELOOPTIMIZATION_HPP

#include <iostream>
#include <cmath>
#include "ceres/ceres.h"
#include "glog/logging.h"

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>

#include <ompl/geometric/planners/rrt/RRT.h>

#include <FromOMPL.h>
#include <FromOMPL_Ceres.hpp>
#include <GraphData.hpp>

/// For finding initial guess
#include <utility>
#include <Eigen/Dense>
///

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;

namespace ReloPush{

    // Object from pre-push
    ReloPush::State revert_pre_push(ReloPush::State& prePushState, float distance);

    struct OptResult
    {
        double x;
        double y;
        double landing_yaw;
        double cost;
        double change_in_yaw;

        OptResult(double x_in, double y_in, double yaw_in, double cost_in, double delta_yaw)
            : x(x_in), y(y_in), landing_yaw(yaw_in), cost(cost_in), change_in_yaw(delta_yaw)
        {}
    };

    /**
     * The cost functor.  We'll store all "constants" from your code
     * as data members.  The parameter block is param[0..1] = (x1, y1).
     *
     * We output a single residual = cost.
     */
    struct CostFunctor {

        CostFunctor(double x_i, double y_i, double th_i,
                    double x2,  double y2,  double th2,
                    double th_ip, double turning_radius, double pre_push_distance,
                    WorkspaceBoundary ws_in)
            : x_i_(x_i), y_i_(y_i), th_i_(th_i),
            x2_(x2),   y2_(y2),   th2_(th2),
            th_ip_(th_ip), R_(turning_radius), ws(ws_in), pre_push_dist(pre_push_distance)
        {


            // Starting pre-push
            x_i_pre_ = x_i_ - pre_push_dist * cos(th_ip);
            y_i_pre_ = y_i_ - pre_push_dist * sin(th_ip);

            // Precompute the left/right circle centers.
            // We'll store them as doubles, but they get cast to T automatically inside Evaluate().
            cx_left_  = x_i_pre_ + R_ * std::cos(th_ip_ + M_PI/2.0);
            cy_left_  = y_i_pre_ + R_ * std::sin(th_ip_ + M_PI/2.0);
            cx_right_ = x_i_pre_ + R_ * std::cos(th_ip_ - M_PI/2.0);
            cy_right_ = y_i_pre_ + R_ * std::sin(th_ip_ - M_PI/2.0);

            // Goal pre-push
            goal_x_pre_ = x2 - pre_push_dist * cos(th2);
            goal_y_pre_ = y2 - pre_push_dist * sin(th2);
        }

        template <typename T>
        bool isInBoundary(const T& x, const T& y) const
        {
            if(x < T(ws.xMin) || x > T(ws.xMax) || y < T(ws.yMin) || y > T(ws.yMax))
            {
                return false;
            }

            return true;
        }

        template <typename T>
        bool operator()(const T* const param, T* residual) const {
            // param = [x1, y1]
            T x1w = param[0];
            T y1w = param[1];

            // limit optimization range (soft constraint)
            //if(x1w < T(ws.xMin) || x1w > T(ws.xMax) || y1w < T(ws.yMin) || y1w > T(ws.yMax))
            //{
            //    residual[0] = T(200.0);
                //return false;
            //    return true;
            //}

            if(this->isInBoundary<T>(x1w, y1w)==false)
            {
                residual[0] = T(200.0);
                return true;
            }

            //---------------------------------------------------------
            // (1) Transform (x1w, y1w) into local coords
            //     using (x_i_, y_i_, th_ip_)
            //---------------------------------------------------------
            T xc, yc;
            //worldToLocal(x1w, y1w, T(x_i_), T(y_i_), T(th_ip_), &xc, &yc);
            worldToLocal(x1w, y1w, T(x_i_pre_), T(y_i_pre_), T(th_ip_), &xc, &yc);

            //---------------------------------------------------------
            // (2) Compute local landing orientation th1pc
            //---------------------------------------------------------
            auto orientation_length = computeLocalOrientation(xc, yc, T(R_));
            T th1pc = orientation_length.th1pc;
            T straight_arc_length = orientation_length.path_length;
            // If th1pc is NaN => cost = 10 (like your MATLAB code).
            // Ceres doesn't gracefully handle comparisons to NaN, so we do a check:
            if (ceres::isnan(th1pc)) {
                residual[0] = T(100.0);
                //return false;
                return true;
            }

            //---------------------------------------------------------
            // (3) Compute world orientation: th1p = th_ip + th1pc
            //     Then final heading: th1 = (th1p - th_ip) + th_i
            //     (which is effectively th_i + th1pc)
            //---------------------------------------------------------
            T th1p = mod2pi<T>(T(th_ip_) + th1pc);

            // Find the shortest path using Dubins
            T path_a_length = Dubins_length_ceres<T>(T(x_i_pre_), T(y_i_pre_), T(th_ip_),
                                                   x1w, y1w, th1p, T(R_));

            T obj_pre_relo_x = x1w + T(pre_push_dist) * ceres::cos(th1p);
            T obj_pre_relo_y = y1w + T(pre_push_dist) * ceres::sin(th1p);

            if(this->isInBoundary(obj_pre_relo_x,obj_pre_relo_y)==false)
            {
                // Pre-relocation is out-of-boundary
                residual[0] = T(200.0);
                return true;
            }

            T th1  = mod2pi<T>((th1p - T(th_ip_)) + T(th_i_));

            //---------------------------------------------------------
            // (4) Long-path threshold logic
            //---------------------------------------------------------
            //T alpha, beta;
            //find_alpha_beta(x1w, y1w, th1, T(x2_), T(y2_), T(th2_), alpha, beta);
            //T d_thres = longpath_thres_dist(alpha, beta);

            // d = Euclidean((x2,y2),(x1w,y1w)) / turning_radius
            //T dx = (T(x2_) - x1w);
            //T dy = (T(y2_) - y1w);
           // T dist_xy = ceres::sqrt(dx*dx + dy*dy);
            //T d = dist_xy / T(R_);

            // final pre_push before goal
            T final_prepush_x = obj_pre_relo_x - T(pre_push_dist) * ceres::cos(th1);
            T final_prepush_y = obj_pre_relo_y - T(pre_push_dist) * ceres::sin(th1);

            if(this->isInBoundary(final_prepush_x,final_prepush_y)==false)
            {
                // final prepush is out-of-boundary
                residual[0] = T(200.0);
                return true;
            }

            //T path_length = Dubins_length_ceres<T>(x1w, y1w, th1, T(x2_), T(y2_), T(th2_), T(R_));
            T path_length = Dubins_length_ceres<T>(final_prepush_x, final_prepush_y, th1,
                                                   T(goal_x_pre_), T(goal_y_pre_), T(th2_), T(R_));

            //T delta_d = path_length + straight_arc_length;
            T delta_d = path_length + path_a_length;
            //if (d_thres > d) {
            //    delta_d = T(10.0);
            //} else {
            //    delta_d = d;
            //}

            //---------------------------------------------------------
            // (5) Two-circle check => cost = 6 if inside either circle
            //---------------------------------------------------------
            T dist_left  = ceres::sqrt( ceres::pow(x1w - T(cx_left_),  T(2.0)) +
                                      ceres::pow(y1w - T(cy_left_),  T(2.0)) );

            T dist_right = ceres::sqrt( ceres::pow(x1w - T(cx_right_), T(2.0)) +
                                       ceres::pow(y1w - T(cy_right_), T(2.0)) );
            //T total_cost = delta_d;

            T total_cost = delta_d;

            /*
            if ((dist_left <= T(R_)) || (dist_right <= T(R_))) {
                total_cost = T(100.0);
                //return false;
            }
            */

            if (dist_left <= T(R_))
            {
                T A=T(5);
                T k=-T(0.5);
                total_cost = A * ceres::exp(k * ceres::abs(dist_left)) + T(10.0);
            }
            else if(dist_right <= T(R_))
            {
                T A=T(5);
                T k=-T(0.5);
                total_cost = A * ceres::exp(k * ceres::abs(dist_right)) + T(10.0);
            }


            // todo: handle negative residue
            if(total_cost < T(0))
            {
                total_cost = T(100.0);
                //return false;
            }

            if(straight_arc_length < T(0))
            {
                total_cost = T(100.0);
                //return false;
            }

            //---------------------------------------------------------
            // (6) Output the cost as a single residual
            //---------------------------------------------------------
            residual[0] = total_cost;

            //std::cout << "COST: " << total_cost << std::endl;
            return true;
        }

        // Data members (constants from your MATLAB code):
        double x_i_, y_i_, th_i_;
        double x2_,  y2_,  th2_;
        double th_ip_, R_;
        WorkspaceBoundary ws; // workspace boundary

        // Precomputed circle centers (in world coords):
        double cx_left_,  cy_left_;
        double cx_right_, cy_right_;

        // For finding pre-push
        double pre_push_dist;
        double x_i_pre_, y_i_pre_; // pre-push for pre-relo start pose

        // Goal pre-push
        double goal_x_pre_, goal_y_pre_;
    };


    /**
     * x, y, th
     */
    struct CostFunctorSE2 {

        CostFunctorSE2(double x_i, double y_i, double th_i,
                    double x2,  double y2,  double th2,
                    double th_ip, double turning_radius, double pre_push_distance,
                    WorkspaceBoundary ws_in)
            : x_i_(x_i), y_i_(y_i), th_i_(th_i),
            x2_(x2),   y2_(y2),   th2_(th2),
            th_ip_(th_ip), R_(turning_radius), ws(ws_in), pre_push_dist(pre_push_distance)
        {


            // Starting pre-push
            x_i_pre_ = x_i_ - pre_push_dist * cos(th_ip);
            y_i_pre_ = y_i_ - pre_push_dist * sin(th_ip);

            // Precompute the left/right circle centers.
            // We'll store them as doubles, but they get cast to T automatically inside Evaluate().
            cx_left_  = x_i_pre_ + R_ * std::cos(th_ip_ + M_PI/2.0);
            cy_left_  = y_i_pre_ + R_ * std::sin(th_ip_ + M_PI/2.0);
            cx_right_ = x_i_pre_ + R_ * std::cos(th_ip_ - M_PI/2.0);
            cy_right_ = y_i_pre_ + R_ * std::sin(th_ip_ - M_PI/2.0);

            // Goal pre-push
            goal_x_pre_ = x2 - pre_push_dist * cos(th2);
            goal_y_pre_ = y2 - pre_push_dist * sin(th2);
        }

        template <typename T>
        bool isInBoundary(const T& x, const T& y) const
        {
            if(x < T(ws.xMin) || x > T(ws.xMax) || y < T(ws.yMin) || y > T(ws.yMax))
            {
                return false;
            }

            return true;
        }

        template <typename T>
        bool operator()(const T* const param, T* residual) const {
            // param = [x1, y1]
            T x1w = param[0];  // prerelocation x
            T y1w = param[1]; // prerelocation y
            T th1p = mod2pi<T>(param[2]); // prerelocation th

            // limit optimization range (soft constraint)
            //if(x1w < T(ws.xMin) || x1w > T(ws.xMax) || y1w < T(ws.yMin) || y1w > T(ws.yMax))
            //{
            //    residual[0] = T(200.0);
            //return false;
            //    return true;
            //}

            if(this->isInBoundary<T>(x1w, y1w)==false)
            {
                residual[0] = T(200.0);
                return true;
            }

            // Find the shortest path using Dubins
            T path_a_length = Dubins_length_ceres<T>(T(x_i_pre_), T(y_i_pre_), T(th_ip_),
                                                     x1w, y1w, th1p, T(R_));

            T obj_pre_relo_x = x1w + T(pre_push_dist) * ceres::cos(th1p);
            T obj_pre_relo_y = y1w + T(pre_push_dist) * ceres::sin(th1p);

            if(this->isInBoundary(obj_pre_relo_x,obj_pre_relo_y)==false)
            {
                // Pre-relocation is out-of-boundary
                residual[0] = T(200.0);
                return true;
            }

            T th1  = mod2pi<T>((th1p - T(th_ip_)) + T(th_i_)); // final pushing orientation

            // final pre_push before goal
            T final_prepush_x = obj_pre_relo_x - T(pre_push_dist) * ceres::cos(th1);
            T final_prepush_y = obj_pre_relo_y - T(pre_push_dist) * ceres::sin(th1);

            if(this->isInBoundary(final_prepush_x,final_prepush_y)==false)
            {
                // final prepush is out-of-boundary
                residual[0] = T(200.0);
                return true;
            }

            //T path_length = Dubins_length_ceres<T>(x1w, y1w, th1, T(x2_), T(y2_), T(th2_), T(R_));
            T path_length = Dubins_length_ceres<T>(final_prepush_x, final_prepush_y, th1,
                                                   T(goal_x_pre_), T(goal_y_pre_), T(th2_), T(R_));

            //T delta_d = path_length + straight_arc_length;
            T delta_d = path_length + path_a_length;

            //---------------------------------------------------------
            // (5) Two-circle check => cost = 6 if inside either circle
            //---------------------------------------------------------
            T dist_left  = ceres::sqrt( ceres::pow(x1w - T(cx_left_),  T(2.0)) +
                                      ceres::pow(y1w - T(cy_left_),  T(2.0)) );

            T dist_right = ceres::sqrt( ceres::pow(x1w - T(cx_right_), T(2.0)) +
                                       ceres::pow(y1w - T(cy_right_), T(2.0)) );
            //T total_cost = delta_d;

            T total_cost = delta_d;

            if (dist_left <= T(R_))
            {
                T A=T(5);
                T k=-T(0.5);
                total_cost = A * ceres::exp(k * ceres::abs(dist_left)) + T(10.0);
            }
            else if(dist_right <= T(R_))
            {
                T A=T(5);
                T k=-T(0.5);
                total_cost = A * ceres::exp(k * ceres::abs(dist_right)) + T(10.0);
            }


            // todo: handle negative residue
            if(total_cost < T(0))
            {
                total_cost = T(100.0);
                //return false;
            }

            //---------------------------------------------------------
            // (6) Output the cost as a single residual
            //---------------------------------------------------------
            residual[0] = total_cost;

            //std::cout << "COST: " << total_cost << std::endl;
            return true;
        }

        // Data members (constants from your MATLAB code):
        double x_i_, y_i_, th_i_;
        double x2_,  y2_,  th2_;
        double th_ip_, R_;
        WorkspaceBoundary ws; // workspace boundary

        // Precomputed circle centers (in world coords):
        double cx_left_,  cy_left_;
        double cx_right_, cy_right_;

        // For finding pre-push
        double pre_push_dist;
        double x_i_pre_, y_i_pre_; // pre-push for pre-relo start pose

        // Goal pre-push
        double goal_x_pre_, goal_y_pre_;
    };



    /*! \brief Find a Pre-Relocation by Optimization
        returns a OptResult

        \tparam x_g, y_g, yaw_g: Goal Pose
        \tparam x_r, y_r: Initial Position
        \tparam yaw_r: Pusing Orientation for PreRelocation
        \tparam min_turn_radius: turning radius
        \tparam th_delta: pushing_orientation minus final_push_orientation (orientation of the last push)
    */
    std::pair<double, double> find_init_guess_intersection(double x_g, double y_g, double yaw_g,
                                                           double x_r, double y_r, double yaw_r,
                                                           double min_turn_radius, double th_delta);

    /*! planTurnAndOffsetBumper_R_givenBumperTheta computes the final bumper position
    and the robot's final position given the car's turning maneuver constraints.
    Inputs:
    \tparam carPose      : Eigen::Vector3d {x, y, theta} representing the car's initial pose.
    \tparam goalPose     : Eigen::Vector3d {x, y, theta} representing the goal pose.
    \tparam bumperOffset : Distance from the turning finish point to the bumper (along the bumper's direction).
    \tparam R            : The minimum turning radius of the car.
    \tparam bumperTheta  : The given orientation (in radians) of the bumper.

    Returns a std::pair where:
    \tparam first  : pBumper_final (Eigen::Vector2d) is the final bumper position.
    \tparam second : robot_final (Eigen::Vector2d) is the final robot (car) position.
    */
    std::pair<Eigen::Vector2d, Eigen::Vector2d>
    FindInitialGuess(const Eigen::Vector3d& carPose,
                     const Eigen::Vector3d& goalPose,
                     double bumperOffset,
                     double R,
                     double bumperTheta);

    /*! \brief Find a Pre-Relocation by Optimization
        returns a OptResult

        \tparam x_i, y_i, th_i Pose of initial push
        \tparam th_ip Pre-Relocation push direction
        \tparam x2, y2, th2 Goal pose
        \tparam R Turning Radius
    */
    OptResult FindPreRelocationOptimization(double x_i, double y_i, double th_i,
                                                    double x2, double y2, double th2,
                                            double th_ip, double R, double x_init_guess, double y_init_guess, PlanningContext& ctx);
}

#endif // PRERELOOPTIMIZATION_HPP
