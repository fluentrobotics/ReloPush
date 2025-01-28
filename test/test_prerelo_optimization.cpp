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
#include <PreReloOptimization.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;


////////////////////////////////////////////////////////////////////////////
// A simple "main()" to demonstrate building and running the optimizer.
////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 1) Our "constants" from your example:
    /*
    double x_i   = 1.0;       // Starting x
    double y_i   = 1.0;       // Starting y
    double th_i  = 4.7123;    // Starting orientation
    double th_ip = M_PI/2.0;  // Secondary heading
    double x2    = 1.5;       // Goal x
    double y2    = 1.5;       // Goal y
    double th2   = 5.71239;   // Goal orientation
    double R     = 1.9188; // turning radius
    */

    double x_i   = 2.5;       // Starting x
    double y_i   = 3.9;       // Starting y
    double th_i  = 4.7123;    // Starting orientation
    double th_ip = 0;  // Secondary heading
    double x2    = 3;       // Goal x
    double y2    = 3.2;       // Goal y
    double th2   = 4.7123;   // Goal orientation
    double R     = 1.9188; // turning radius

    WorkspaceBoundary ws(4,5.2); // workspace boundary (x_max, y_max)

    // 2) The parameter block: [x1, y1]
    //    We'll let Ceres find an optimal x1,y1 that minimize the cost.
    double param[2];
    // Suppose we start at an initial guess:
    //param[0] = 0.871872;  // x1 init
    //param[1] = 2.23724;  // y1 init

    // Find a good initial guess for this optimization
    // try intersection

    // Calculate the initial guess for the intersection
    std::pair<double, double> intersection = ReloPush::find_init_guess_intersection(
        x2, y2, th2,
        x_i, y_i, th_ip,
        R, (th_ip-th_i)
        );


    param[0] = intersection.first;
    param[1] = intersection.second;

    //param[0] = 0.618747;
    //param[1] = 2.59334;

    // 3) Build the problem
    ceres::Problem problem;

    // Create a cost function (AutoDiff or NumericDiff).
    // We'll use AutoDiffCostFunction, which needs a functor, the #residuals,
    // and the size of each parameter block.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ReloPush::CostFunctor, 1, 2>(
        new ReloPush::CostFunctor(x_i, y_i, th_i, x2, y2, th2, th_ip, R,ws));

    // Add residual block
    problem.AddResidualBlock(cost_function, nullptr, param);

    // 4) Configure the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;

    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;


    //options.check_gradients = true;

    options.initial_trust_region_radius = 0.1;  // Larger initial step size
    //options.max_trust_region_radius = 1e4;     // Allow large step sizes
    //options.use_nonmonotonic_steps = true;
    //options.minimizer_type = ceres::LINE_SEARCH;
    //options.function_tolerance = 1e-12;    // Allow lower precision in function evaluation
    //options.gradient_tolerance = 1e-12;   // Allow larger gradients near the solution
    //options.parameter_tolerance = 1e-12;

    // 5) Run the solver
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 6) Print results
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final x1,y1: " << param[0] << ", " << param[1] << "\n";

    // If you want, we can evaluate the final cost:
    double cost_eval[1];
    double* parameters = &param[0];
    cost_function->Evaluate(&parameters, cost_eval, nullptr);
    std::cout << "Final cost = " << cost_eval[0] << "\n";

    return 0;
}
