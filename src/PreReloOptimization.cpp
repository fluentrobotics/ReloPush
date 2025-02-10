
#include <PreReloOptimization.hpp>


namespace ReloPush
{
    ReloPush::State revert_pre_push(ReloPush::State& prePushState, float distance)
    {
        ReloPush::State outState(prePushState);

        // Calculate the original x and y coordinates by adding the distance along the heading
        outState.x += distance * cos(prePushState.yaw);
        outState.y += distance * sin(prePushState.yaw);

        // Wrap the yaw angle into the [0, 2*pi) range
        outState.yaw = fromOMPL::mod2pi(outState.yaw);

        return outState;
    }

    /// Finding an initial guess for this optimization
    // Function to wrap an angle to the range [-pi, pi]
    double wrap_to_pi(double angle) {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    // Function to find the intersection point of a line with slope 's' and a horizontal line at y = y_int
    std::pair<double, double> find_y_Intersection(double s, double x_s, double y_s, double y_int) {
        if (std::abs(s) < 1e-12) { // Using a small epsilon instead of exact zero
            if (std::abs(y_s - y_int) < 1e-12) {
                //throw std::runtime_error("The lines are coincident (infinite intersections).");
                return std::make_pair(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
            } else {
                //throw std::runtime_error("The lines are parallel and do not intersect.");
                return std::make_pair(std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
            }
        }

        // Calculate x-coordinate of the intersection
        double x_intersect = (y_int - y_s) / s + x_s;

        // y-coordinate is the y_int
        double y_intersect = y_int;

        return std::make_pair(x_intersect, y_intersect);
    }

    // Function to find the intersection points on the left and right turn circles
    // todo: remove x,y_robot
    std::pair<double, double> find_turn_circle_intersections(double yaw_g_ip, double x_g, double y_g, double th_delta, double min_turn_radius)
    {
        // orthogonal cases (to be developed)
        if(abs(yaw_g_ip - M_PI/2) < 0.01 || abs(mod2pi(yaw_g_ip) - (1.5)*M_PI) < 0.01)
        {
            return std::make_pair(x_g,0);
        }

        double th_g_ip = yaw_g_ip;
        th_g_ip = fromOMPL::mod2pi(th_g_ip);

        double R = min_turn_radius;

        // Find target left and right polar angle
        double phi_L = fromOMPL::mod2pi(th_g_ip - th_delta);
        double phi_R = fromOMPL::mod2pi(-1*th_g_ip - th_delta);


        double x_l = R * sin(phi_L);
        double y_l = R - R*cos(phi_L);

        double x_r = R * sin(phi_R);
        double y_r = -1*(R - R*cos(phi_R));

        double x_out, y_out;

        if(phi_L <= phi_R)
        {
            x_out = x_l;
            y_out = y_l;
        }
        else
        {
            x_out = x_r;
            y_out = y_r;
        }

        return std::make_pair(x_out, y_out);

        /*

        // 2) Compute intersection in the robot frame for the left turn
        double x_left_r = R * sin(psi);
        double y_left_r = R - R * cos(psi);

        // 3) Compute intersection in the robot frame for the right turn
        double x_right_r = -R * sin(psi);
        double y_right_r = -R + R * cos(psi);

        // 4) Transform these points back into the world frame
        double cos_theta = cos(theta_r);
        double sin_theta = sin(theta_r);

        // Left-turn circle intersection in world frame
        double x_left_w = x_r + cos_theta * x_left_r - sin_theta * y_left_r;
        double y_left_w = y_r + sin_theta * x_left_r + cos_theta * y_left_r;

        // Right-turn circle intersection in world frame
        double x_right_w = x_r + cos_theta * x_right_r - sin_theta * y_right_r;
        double y_right_w = y_r + sin_theta * x_right_r + cos_theta * y_right_r;

        // 5) Select the appropriate intersection based on y-coordinate
        double x_out, y_out;
        if (y_left_w >= 0) {
            x_out = x_left_w;
            y_out = y_left_w;
        } else {
            x_out = x_right_w;
            y_out = y_right_w;
        }

        return std::make_pair(x_out, y_out);
        */
    }

    // Function to find the initial guess for the intersection point in the world frame
    std::pair<double, double> find_init_guess_intersection(double x_g, double y_g, double yaw_g,
                                                           double x_r, double y_r, double yaw_ip,
                                                           double min_turn_radius, double th_delta) {
        // ------------------------------
        // 1) Define final orientation in the robot frame
        // double th_final_robot = wrap_to_pi(yaw_g - yaw_ip);

        // ------------------------------
        // 2) Transform (x_g, y_g) into the robot frame
        double dx = x_g - x_r;
        double dy = y_g - y_r;

        double cos_yaw_ip = cos(yaw_ip);
        double sin_yaw_ip = sin(yaw_ip);

        // Robot-frame coordinates of the goal
        double x_g_r = cos_yaw_ip * dx + sin_yaw_ip * dy;
        double y_g_r = -sin_yaw_ip * dx + cos_yaw_ip * dy;

        // Transform the orientation
        double yaw_g_ip = wrap_to_pi(yaw_g - yaw_ip);

        // ------------------------------
        // 3) Find a point in the robot frame for which the orientation is 'th_final_robot'
        double xP_r, yP_r;
        //try {
            std::pair<double, double> turn_circle = find_turn_circle_intersections(yaw_g_ip,
                                                                                   x_g, y_g, th_delta,
                                                                                   min_turn_radius);
            xP_r = turn_circle.first;
            yP_r = turn_circle.second;

            // Check for NaN
            if (std::isnan(xP_r) || std::isnan(yP_r)) {
                //throw std::runtime_error("No feasible point found that matches the final orientation!");
                return std::make_pair(std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN());
            }
        //}
        //catch (const std::exception& e) {
            //std::cerr << "Warning: " << e.what() << std::endl;
            //return std::make_pair(std::numeric_limits<double>::quiet_NaN(),
            //                      std::numeric_limits<double>::quiet_NaN());
        //}

        // ------------------------------
        // 4) The line of all such points in the robot frame that yield that same orientation
        //    is perpendicular to the local y-axis, i.e., parallel to the local x-axis.

        // Parameterize the line: X_line(t) = xP_r + t, Y_line(t) = yP_r

        // ------------------------------
        // 5) Find the intersection of that line with the infinite line from the robot’s origin
        //    heading toward (x_g_r, y_g_r).

        double xI_r, yI_r;
        if (std::abs(y_g_r) < 1e-9) { // Degenerate case
            // The goal line is horizontal in the robot frame.
            // Handle this by setting intersection to NaN
            xI_r = std::numeric_limits<double>::quiet_NaN();
            yI_r = std::numeric_limits<double>::quiet_NaN();
        }
        else {
            // Slope of the goal line
            double s;
            if (std::abs(x_g_r) < 1e-9) { // Avoid division by zero for vertical line
                s = std::numeric_limits<double>::infinity();
            }
            else {
                s = std::tan(yaw_g_ip);
            }

            if (std::isinf(s)) { // Vertical line
                xI_r = 0.0; // Intersection at x = 0
                yI_r = yP_r;
            }
            else {
                //try {
                    std::pair<double, double> intersection = find_y_Intersection(s, x_g_r, y_g_r, yP_r);
                    xI_r = intersection.first;
                    yI_r = intersection.second;
                    //if(xI_r == std::numeric_limits<double>::quiet_NaN() || yI_r == std::numeric_limits<double>::quiet_NaN())
                    //{
                        // parallel. skip

                    //}
                //}
                //catch (const std::exception& e) {
                //    std::cerr << "Warning: " << e.what() << std::endl;
                //    xI_r = std::numeric_limits<double>::quiet_NaN();
                //    yI_r = std::numeric_limits<double>::quiet_NaN();
                //}
            }
        }

        // ------------------------------
        // 6) Transform intersection back to the world frame
        double xI_world, yI_world;
        if (!std::isnan(xI_r) && !std::isnan(yI_r)) {
            xI_world = x_r + cos_yaw_ip * xI_r - sin_yaw_ip * yI_r;
            yI_world = y_r + sin_yaw_ip * xI_r + cos_yaw_ip * yI_r;
        }
        else {
            xI_world = std::numeric_limits<double>::quiet_NaN();
            yI_world = std::numeric_limits<double>::quiet_NaN();
        }

        return std::make_pair(xI_world, yI_world);
    }



    /*! \brief Find a Pre-Relocation by Optimization
        returns a OptResult

        \tparam x_i, y_i, th_i Pose of initial push
        \tparam th_ip Pre-Relocation push direction
        \tparam x2, y2, th2 Goal pose
        \tparam R Turning Radius
    */
    OptResult FindPreRelocationOptimization(double x_i, double y_i, double th_i,
                                            double x2, double y2, double th2,
                                            double th_ip, double R, double x_init_guess, double y_init_guess, PlanningContext& ctx)
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
                new CostFunctor(x_i, y_i, th_i, x2, y2, th2, th_ip, R,ctx.parameters.PrePush_dist, ctx.parameters.boundary));

        // Add residual block
        problem.AddResidualBlock(cost_function, nullptr, param);

        // 4) Configure the solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        //options.minimizer_progress_to_stdout = true;
        options.use_nonmonotonic_steps = true;
        //options.minimizer_type = ceres::LINE_SEARCH;
        /*
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.use_nonmonotonic_steps = true;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.initial_trust_region_radius = 0.1; //initial step size
        options.max_num_iterations = 100;
        */


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


        // second optimization: line search
        options.minimizer_type = ceres::LINE_SEARCH;
        ceres::Solve(options, &problem, &summary);
        parameters = &param[0];
        cost_function->Evaluate(&parameters, cost_eval, nullptr);


        // Start Prepush
        double x_i_prepush = x_i - ctx.parameters.PrePush_dist * cos(th_ip);
        double y_i_prepush = y_i - ctx.parameters.PrePush_dist * sin(th_ip);
        auto yaw_l = findLandingYaw(x_i_prepush,y_i_prepush,th_i,param[0],param[1],th_ip,R);

        // Optimized robot relo
        ReloPush::State robotRelo(param[0],param[1],yaw_l);

        // pre-relocation from optimized car prepush for pre-relo
        //ReloPush::State PreRelo = revert_pre_push(robotRelo,ctx.parameters.PrePush_dist);

        return OptResult(robotRelo.x,robotRelo.y,yaw_l,cost_eval[0], yaw_l-th_ip);
    }
}


