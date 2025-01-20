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

#include <FromOMPL_Ceres.hpp>
#include <GraphData.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;

struct OptResult
{
    double x;
    double y;
    double yaw;
    double cost;

    OptResult(double x_in, double y_in, double yaw_in, double cost_in)
        : x(x_in), y(y_in), yaw(yaw_in), cost(cost_in)
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
                double th_ip, double turning_radius)
        : x_i_(x_i), y_i_(y_i), th_i_(th_i),
        x2_(x2),   y2_(y2),   th2_(th2),
        th_ip_(th_ip), R_(turning_radius)
    {
        // Precompute the left/right circle centers.
        // We'll store them as doubles, but they get cast to T automatically inside Evaluate().
        cx_left_  = x_i_ + R_ * std::cos(th_ip_ + M_PI/2.0);
        cy_left_  = y_i_ + R_ * std::sin(th_ip_ + M_PI/2.0);
        cx_right_ = x_i_ + R_ * std::cos(th_ip_ - M_PI/2.0);
        cy_right_ = y_i_ + R_ * std::sin(th_ip_ - M_PI/2.0);
    }

    template <typename T>
    bool operator()(const T* const param, T* residual) const {
        // param = [x1, y1]
        T x1w = param[0];
        T y1w = param[1];

        //---------------------------------------------------------
        // (1) Transform (x1w, y1w) into local coords
        //     using (x_i_, y_i_, th_ip_)
        //---------------------------------------------------------
        T xc, yc;
        worldToLocal(x1w, y1w, T(x_i_), T(y_i_), T(th_ip_), &xc, &yc);

        //---------------------------------------------------------
        // (2) Compute local landing orientation th1pc
        //---------------------------------------------------------
        auto orientation_length = computeLocalOrientation(xc, yc, T(R_));
        T th1pc = orientation_length.th1pc;
        T straight_arc_length = orientation_length.path_length;
        // If th1pc is NaN => cost = 10 (like your MATLAB code).
        // Ceres doesn't gracefully handle comparisons to NaN, so we do a check:
        if (ceres::isnan(th1pc)) {
            residual[0] = T(10.0);
            return true;
        }

        //---------------------------------------------------------
        // (3) Compute world orientation: th1p = th_ip + th1pc
        //     Then final heading: th1 = (th1p - th_ip) + th_i
        //     (which is effectively th_i + th1pc)
        //---------------------------------------------------------
        T th1p = mod2pi<T>(T(th_ip_) + th1pc);
        T th1  = mod2pi<T>((th1p - T(th_ip_)) + T(th_i_));

        //---------------------------------------------------------
        // (4) Long-path threshold logic
        //---------------------------------------------------------
        T alpha, beta;
        find_alpha_beta(x1w, y1w, th1, T(x2_), T(y2_), T(th2_), alpha, beta);
        T d_thres = longpath_thres_dist(alpha, beta);

        // d = Euclidean((x2,y2),(x1w,y1w)) / turning_radius
        T dx = (T(x2_) - x1w);
        T dy = (T(y2_) - y1w);
        T dist_xy = ceres::sqrt(dx*dx + dy*dy);
        T d = dist_xy / T(R_);

        T path_length = Dubins_length_ceres<T>(x1w, y1w, th1, T(x2_), T(y2_), T(th2_), T(R_));


        T delta_d = path_length + straight_arc_length;
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

        if ((dist_left <= T(R_)) || (dist_right <= T(R_))) {
            total_cost = T(10.0);
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

    // Precomputed circle centers (in world coords):
    double cx_left_,  cy_left_;
    double cx_right_, cy_right_;
};

/*! \brief Find a Pre-Relocation by Optimization
    returns a OptResult

    \tparam x_i, y_i, th_i Pose of initial push
    \tparam th_ip Pre-Relocation push direction
    \tparam x2, y2, th2 Goal pose
    \tparam R Turning Radius
*/
OptResult FindPreRelocationOptimization(double x_i, double y_i, double th_i,
                                                double x2, double y2, double th2,
                                        double th_ip, double R, double x_init_guess, double y_init_guess);

#endif // PRERELOOPTIMIZATION_HPP
