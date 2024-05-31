#ifndef FROM_OMPL_H
#define FROM_OMPL_H

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>

#include <ompl/geometric/planners/rrt/RRT.h>

namespace fromOMPL{
    extern double twopi;
    extern const double DUBINS_EPS;
    extern const double DUBINS_ZERO;
    double mod2pi(double x);

    double longpath_thres_dist(double& alpha, double& beta);
    bool is_longpath_case(double d, double alpha, double beta);
    ompl::base::DubinsStateSpace::DubinsPath dubins_classification(const double d, const double alpha, const double beta);
    ompl::base::DubinsStateSpace::DubinsPath dubins_exhaustive(const double d, const double alpha, const double beta);
}

#endif