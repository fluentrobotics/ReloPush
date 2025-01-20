
#include <PreReloOptimization.hpp>


/*! \brief Find a Pre-Relocation by Optimization
    returns a OptResult

    \tparam x_i, y_i, th_i Pose of initial push
    \tparam th_ip Pre-Relocation push direction
    \tparam x2, y2, th2 Goal pose
    \tparam R Turning Radius
*/
OptResult FindPreRelocationOptimization(double x_i, double y_i, double th_i,
                                        double x2, double y2, double th2,
                                        double th_ip, double R, double x_init_guess, double y_init_guess)
{
    double param[2];
    // Suppose we start at an initial guess:
    param[0] = x_init_guess;  // x1 init
    param[1] = y_init_guess;  // y1 init

    // 3) Build the problem
    ceres::Problem problem;

    // Create a cost function (AutoDiff or NumericDiff).
    // We'll use AutoDiffCostFunction, which needs a functor, the #residuals,
    // and the size of each parameter block.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 2>(
            new CostFunctor(x_i, y_i, th_i, x2, y2, th2, th_ip, R));

    // Add residual block
    problem.AddResidualBlock(cost_function, nullptr, param);

    // 4) Configure the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;


    // 5) Run the solver
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 6) Print results
    //std::cout << summary.BriefReport() << "\n";
    //std::cout << "Final x1,y1: " << param[0] << ", " << param[1] << "\n";

    // If you want, we can evaluate the final cost:
    double cost_eval[1];
    double* parameters = &param[0];
    cost_function->Evaluate(&parameters, cost_eval, nullptr);
    //std::cout << "Final cost = " << cost_eval[0] << "\n";

    auto yaw_l = findLandingYaw(x_i,y_i,th_i,param[0],param[1],th_ip,R);

    return OptResult(param[0],param[1],yaw_l,cost_eval[0]);
}


