#include <iostream>
#include <cmath>
//#include <GraphBuilder.hpp>
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

#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;


////////////////////////////////////////////////////////////////////////////
// A simple "main()" to demonstrate building and running the optimizer.
////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    //google::InitGoogleLogging(argv[0]);

    // 1) Our "constants" from your example:

/*
    double x_i   = 1.2;       // Starting x
    double y_i   = 3.1;       // Starting y
    double th_i  = 0;    // Starting orientation
    double th_ip = 1.5*M_PI;  // Secondary heading
    double x2    = 3.6;       // Goal x
    double y2    = 1.5;       // Goal y
    double th2   = 0;   // Goal orientation
    double R     = 1.9188; // turning radius
*/

    double x_i   = 1.053499816410735;       // Starting x
    double y_i   = 1.5889540125669037;       // Starting y
    double th_i  = 4.70575;    // Starting orientation
    double th_ip = 1.564099651924602;  // Secondary heading
    double x2    = 0.557636022567749;       // Goal x
    double y2    = 2.989194631576538;       // Goal y
    double th2   = 3.995207281904765;   // Goal orientation
    double R     = 1.5496432781219482; // turning radius


/*
    double x_i   = 1.2;       // Starting x
    double y_i   = 1;       // Starting y
    double th_i  = 4.7123;    // Starting orientation
    double th_ip = M_PI/2;  // Secondary heading
    double x2    = 1.7;       // Goal x
    double y2    = 1.5;       // Goal y
    double th2   = 5.4124;   // Goal orientation
    double R     = 1.9188; // turning radius
*/

/*
    double x_i = 2.281902189811266;
    double y_i = 2.550387669539164;
    double th_i =  4.707587558373694;
    double x2 = 0.6486319899559021;
    double y2 = 3.4546070098876953;
    double th2 = 5.4250272989233554;
    double th_ip = 3.136791231578797;
    double R = 1.5496432781219482;
        //x_init_guess: 2.4745738059108167
        //y_init_guess: 2.167457858022773
*/

    /*
    double x_i   = 2.5;       // Starting x
    double y_i   = 3.9;       // Starting y
    double th_i  = 4.7123;    // Starting orientation
    double th_ip = 0;  // Secondary heading
    double x2    = 3;       // Goal x
    double y2    = 3.2;       // Goal y
    double th2   = 4.7123;   // Goal orientation
    double R     = 1.9188; // turning radius
    */

    /*
    double x_i = 1.2;
    double y_i = 3.1;
    double th_i = 0;
    double th_ip = 1.5*M_PI;
    double x2  = 3.6;
    double y2  = 1.5;
    double th2 = 0;
    double R     = 1.9188; // turning radius
    */

    /*
    double x_i = 1.4;
    double y_i = 3.3;
    double th_i = 0;
    double th_ip = 4.71239;
    double x2  = 3.6;
    double y2  = 1.5;
    double th2 = 1.57;
    double R     = 1.9188; // turning radius
    */

    WorkspaceBoundary ws(4,5.2); // workspace boundary (x_max, y_max)

    // 2) The parameter block: [x1, y1]
    //    We'll let Ceres find an optimal x1,y1 that minimize the cost.
    double param[2];
    // Suppose we start at an initial guess:
    //param[0] = 0.871872;  // x1 init
    //param[1] = 2.23724;  // y1 init

    // Find a good initial guess for this optimization
    // try intersection

    double pre_push_dist = 0.54;// 0.38+0.075;
    // Get pre-push
    auto pushPose = ReloPush::State(x_i,y_i,th_ip);
    //auto Start_prepush = find_pre_push(pushPose, pre_push_dist);

    // Pre-push
    double start_prepush_x = pushPose.x - pre_push_dist * cos(pushPose.yaw);
    double start_prepush_y = pushPose.y - pre_push_dist * sin(pushPose.yaw);
    std::cout << "Start Prepush: " << start_prepush_x << ", " << start_prepush_y << std::endl;

    // Calculate the initial guess for the intersection

    std::pair<double, double> intersection = ReloPush::find_init_guess_intersection(
        x2, y2, th2,
        start_prepush_x, start_prepush_y, th_ip,
        R, (th_ip-th_i)
        );


    auto init_guess = ReloPush::FindInitialGuess(Eigen::Vector3d(start_prepush_x,start_prepush_y,th_ip),Eigen::Vector3d(x2,y2,th2),pre_push_dist,R,th_i);


    // Find intersection prepush (to be discussed further)
    //double x1c,y1c;
    //worldToLocal<double>(intersection.first, intersection.second, start_prepush_x, start_prepush_y, th_ip, &x1c, &y1c);
    //auto locOriRes = computeLocalOrientation<double>(x1c, y1c, R);

    //auto init_guess_prepush = find_pre_push(ReloPush::State(intersection.first, intersection.second,th_ip + locOriRes.th1pc), pre_push_dist);

    //double init_x = intersection.first - pre_push_dist * cos(th_ip + locOriRes.th1pc);
    //double init_y = intersection.second - pre_push_dist * sin(th_ip + locOriRes.th1pc);
    double init_x = init_guess.first.x();
    double init_y = init_guess.first.y();

    std::cout << "Initial Guess: " << init_x << ", " << init_y << std::endl;

    double goal_prepush_x = x2 - pre_push_dist * cos(th2);
    double goal_prepush_y = y2 - pre_push_dist * sin(th2);

    //param[0] = init_x;
    //param[1] = init_y;

    //param[0] = 1.16;
    //param[1] = 3.17;

    // 3) Build the problem
    ceres::Problem problem;


    /*
    // Create a cost function (AutoDiff or NumericDiff).
    // We'll use AutoDiffCostFunction, which needs a functor, the #residuals,
    // and the size of each parameter block.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ReloPush::CostFunctor, 1, 2>(
        new ReloPush::CostFunctor(x_i, y_i, th_i, x2, y2, th2, th_ip, R, pre_push_dist,ws));

    // Add residual block
    problem.AddResidualBlock(cost_function, nullptr, param);

    // 4) Configure the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-4;  // Ensure convergence
    options.gradient_tolerance = 1e-4;
    options.parameter_tolerance = 1e-4;
    options.minimizer_progress_to_stdout = true;
    options.use_nonmonotonic_steps = true;
    options.num_threads = 4;

    //options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    //options.trust_region_strategy_type = ceres::DOGLEG;
    //options.max_num_iterations = 100;
    //options.use_inner_iterations = true;
    //options.min_trust_region_radius = 1e-8;
    //options.initial_trust_region_radius = 0.001;
    //options.max_trust_region_radius = 1e24;



    //options.check_gradients = true;

    options.initial_trust_region_radius = 1;  // Larger initial step size
    //options.max_trust_region_radius = 1e4;     // Allow large step sizes
    //options.use_nonmonotonic_steps = true;
   // options.minimizer_type = ceres::LINE_SEARCH;
    //options.function_tolerance = 1e-12;    // Allow lower precision in function evaluation
    //options.gradient_tolerance = 1e-12;   // Allow larger gradients near the solution
    //options.parameter_tolerance = 1e-12;

    // 5) Run the solver
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 6) Print results
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final x1,y1 (Robot): " << param[0] << ", " << param[1] << "\n";

    // If you want, we can evaluate the final cost:
    double cost_eval[1];
    double* parameters = &param[0];
    cost_function->Evaluate(&parameters, cost_eval, nullptr);
    std::cout << "Final cost = " << cost_eval[0] << "\n";

    options.minimizer_type = ceres::LINE_SEARCH;
    options.max_num_line_search_step_size_iterations = 5;
    options.line_search_direction_type = ceres::BFGS;
    //options.min_line_search_step_size = 1e-8;
    //options.line_search_sufficient_function_decrease = 1e-4;
    //options.line_search_sufficient_curvature_decrease = 0.9;

    auto start = std::chrono::high_resolution_clock::now();
    ceres::Solve(options, &problem, &summary);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Elapsed time: " << duration.count() << " ms" << std::endl;


    // Start Prepush
    double x_i_prepush = x_i - pre_push_dist * cos(th_ip);
    double y_i_prepush = y_i - pre_push_dist * sin(th_ip);
    auto yaw_l = findLandingYaw<double>(x_i_prepush,y_i_prepush,th_i,param[0],param[1],th_ip,R);

    // Optimized robot relo
    ReloPush::State robotRelo(param[0],param[1],yaw_l);

    // 6) Print results
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Final x1,y1 (Robot): " << param[0] << ", " << param[1] << ", landing_yaw: " << yaw_l << "\n";
    parameters = &param[0];
    cost_function->Evaluate(&parameters, cost_eval, nullptr);
    std::cout << "Final cost = " << cost_eval[0] << "\n\n";

*/


    double init_guess_th = th2 + (th_ip-th_i);
    std::cout << "=== SE2 Optimization ===" << std::endl;
    // run SE2 optimization
    double paramSE2[3] = {init_x,init_y,init_guess_th}; // init from guess
    //double paramSE2[3] = {goal_prepush_x,goal_prepush_y,th2}; // init from goal
    //double paramSE2[3] = {start_prepush_x,start_prepush_y,th_ip}; // init from start
    //double paramSE2[3] = {param[0],param[1],yaw_l};
    double robot_prerelo_x_init = paramSE2[0] - pre_push_dist * cos(paramSE2[2]);
    double robot_prerelo_y_init = paramSE2[1] - pre_push_dist * sin(paramSE2[2]);


    ceres::Solver::Summary summarySE2;
    ceres::Problem problemSE2;
    ceres::CostFunction* cost_function_se2 =
        new ceres::AutoDiffCostFunction<ReloPush::CostFunctorSE2, 1,3>(
        new ReloPush::CostFunctorSE2(x_i,y_i,th_i,x2,y2,th2,th_ip,R,pre_push_dist,ws));

    problemSE2.AddResidualBlock(cost_function_se2, nullptr, paramSE2);
    ceres::Solver::Options optionsSE2;
    optionsSE2.linear_solver_type = ceres::DENSE_QR;
    optionsSE2.function_tolerance = 1e-4;  // Ensure convergence
    optionsSE2.gradient_tolerance = 1e-4;
    optionsSE2.parameter_tolerance = 1e-4;
    optionsSE2.minimizer_progress_to_stdout = true;
    optionsSE2.use_nonmonotonic_steps = true;
    optionsSE2.num_threads = 4;
    optionsSE2.initial_trust_region_radius = 1;

    ceres::Solve(optionsSE2, &problemSE2, &summarySE2);

    double robot_prerelo_x_1 = paramSE2[0] - pre_push_dist * cos(paramSE2[2]);
    double robot_prerelo_y_1 = paramSE2[1] - pre_push_dist * sin(paramSE2[2]);

    optionsSE2.minimizer_type = ceres::LINE_SEARCH;
    optionsSE2.max_num_line_search_step_size_iterations = 5;
    optionsSE2.line_search_direction_type = ceres::BFGS;
    optionsSE2.function_tolerance = 1e-8;  // Ensure convergence
    optionsSE2.gradient_tolerance = 1e-8;
    optionsSE2.parameter_tolerance = 1e-8;

    ceres::Solve(optionsSE2, &problemSE2, &summarySE2);


    std::cout << summarySE2.FullReport() << "\n";
    std::cout << "Final SE2 x1,y1,th1 (obj): " << paramSE2[0] << ", " << paramSE2[1] << ", " << paramSE2[2] << "\n";
    double cost_eval_se2[1];
    double* parametersSE2 = &paramSE2[0];
    cost_function_se2->Evaluate(&parametersSE2, cost_eval_se2, nullptr);
    std::cout << "Final cost SE2 = " << cost_eval_se2[0] << "\n";



    // object prerelo
    double robot_prerelo_x = paramSE2[0] - pre_push_dist * cos(paramSE2[2]);
    double robot_prerelo_y = paramSE2[1] - pre_push_dist * sin(paramSE2[2]);
    // final prepush
    double final_push_th = th_i + (paramSE2[2]-th_ip);
    double final_prepush_x = paramSE2[0] - pre_push_dist * cos(final_push_th);
    double final_prepush_y = paramSE2[1] - pre_push_dist * sin(final_push_th);

    return 0;
}
