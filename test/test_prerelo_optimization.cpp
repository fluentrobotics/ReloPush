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


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;


template <typename T>
void find_alpha_beta(T& x1, T& y1, T& th1,
                     T x2, T y2, T th2,
                     T& alpha_out, T& beta_out) {
    // alpha = mod2pi(th1 - atan2((y2 - y1),(x2 - x1)));
    // beta  = mod2pi(th2 - atan2((y2 - y1),(x2 - x1)));
    T dx = (x2 - x1);
    T dy = (y2 - y1);
    T angle = ceres::atan2(dy, dx);

    T alpha = mod2pi(th1 - angle);
    T beta  = mod2pi(th2 - angle);

    alpha_out = alpha;
    beta_out  = beta;
}

template <typename T>
T longpath_thres_dist(const T& alpha, const T& beta) {
    // d_thres = abs(sin(alpha)) + abs(sin(beta))
    //         + sqrt(4 - (cos(alpha) + cos(beta))^2);
    T sA = ceres::sin(alpha);
    T sB = ceres::sin(beta);
    T cA = ceres::cos(alpha);
    T cB = ceres::cos(beta);

    T val = ceres::abs(sA) + ceres::abs(sB) +
            ceres::sqrt(T(4.0) - ceres::pow((cA + cB), T(2.0)));
    return val;
}

template <typename T>
void worldToLocal(const T& xw, const T& yw,
                  const T& x0, const T& y0, const T& th0,
                  T* xc_out, T* yc_out) {
    // (xw - x0, yw - y0) rotated by -th0
    T dx = xw - x0;
    T dy = yw - y0;

    // xc = dx*cos(th0) + dy*sin(th0)
    // yc = -dx*sin(th0) + dy*cos(th0)
    T xc = dx * ceres::cos(th0) + dy * ceres::sin(th0);
    T yc = -dx * ceres::sin(th0) + dy * ceres::cos(th0);

    *xc_out = xc;
    *yc_out = yc;
}

// Struct to hold the results
template <typename T>
struct OrientationResult {
    T th1pc;
    T path_length;
};

/**
 * Compute local orientation th1pc and path_length for traveling
 * from (0,0,0) to (x1c,y1c) with minimal-turn path.
 */
template <typename T>
OrientationResult<T> computeLocalOrientation(const T& x1c, const T& y1c, const T& R) {
    OrientationResult<T> result;

    // 1) Constraints: if y1c > 2R or y1c < -2R => invalid
    if (y1c > T(2.0) * R || y1c < -T(2.0) * R) {
        result.th1pc = std::numeric_limits<T>::quiet_NaN();
        result.path_length = std::numeric_limits<T>::quiet_NaN();
        return result;
    }

    // 2) Straight-line case
    if (ceres::abs(y1c) < T(1e-12)) {
        result.th1pc = T(0.0);
        result.path_length = x1c;
        return result;
    }

    T rad;
    T center_x, center_y;
    T mr, mt;
    T th1pc;
    T h;
    T arc_length;

    if (y1c > T(0.0)) {
        // 3) y1c > 0
        rad = T(2.0) * R * y1c - y1c * y1c;
        if (rad < T(0.0)) {
            result.th1pc = std::numeric_limits<T>::quiet_NaN();
            result.path_length = std::numeric_limits<T>::quiet_NaN();
            return result;
        }
        h = x1c - ceres::sqrt(rad);
        center_x = h;
        center_y = R;

        // Slope calculations
        mr = (y1c - center_y) / (x1c - center_x);
        mt = -T(1.0) / mr;
        th1pc = mod2pi(ceres::atan(mt));

        // Compute arc length
        arc_length = R * ceres::acos(( -y1c + R ) / R); // Equivalent to acos(1 - y1c/R)
        result.path_length = h + arc_length;
    }
    else {
        // 4) y1c < 0
        rad = -T(2.0) * R * y1c - y1c * y1c;
        if (rad < T(0.0)) {
            result.th1pc = std::numeric_limits<T>::quiet_NaN();
            result.path_length = std::numeric_limits<T>::quiet_NaN();
            return result;
        }
        h = x1c - ceres::sqrt(rad);
        center_x = h;
        center_y = -R;

        // Slope calculations
        mr = (y1c - center_y) / (x1c - center_x);
        mt = -T(1.0) / mr;
        th1pc = mod2pi(ceres::atan(mt));

        // Compute arc length
        arc_length = R * ceres::acos(( y1c + R ) / R);
        result.path_length = h + arc_length;
    }

    result.th1pc = th1pc;
    return result;
}


template<typename T>
T Dubins_length_ceres(T start_x, T start_y, T start_yaw,
                      T goal_x, T goal_y, T goal_yaw,
                      T turning_radius = 1.0)
{
    T alpha_ceres, beta_ceres, d_ceres;
    find_alpha_beta<T>(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, alpha_ceres, beta_ceres);

    d_ceres = ceres::sqrt(ceres::pow(goal_x-start_x,2) + ceres::pow(goal_y-start_y,2))/turning_radius;

    bool is_long = fromOMPLCeres::is_longpath_case(d_ceres,alpha_ceres,beta_ceres);

    T path_length = T(100);

    if(is_long)
        path_length = fromOMPLCeres::dubins_classification(d_ceres, alpha_ceres, beta_ceres, turning_radius).lengthCost();

    else
        path_length = fromOMPLCeres::dubins_exhaustive(d_ceres, alpha_ceres, beta_ceres, turning_radius).lengthCost();

    return path_length;
}

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

////////////////////////////////////////////////////////////////////////////
// A simple "main()" to demonstrate building and running the optimizer.
////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 1) Our "constants" from your example:
    double x_i   = 1.0;       // Starting x
    double y_i   = 1.0;       // Starting y
    double th_i  = 4.7123;    // Starting orientation
    double th_ip = M_PI/2.0;  // Secondary heading
    double x2    = 1.5;       // Goal x
    double y2    = 1.5;       // Goal y
    double th2   = 5.41239;   // Goal orientation
    double R     = 1.43061495; // turning radius

    // 2) The parameter block: [x1, y1]
    //    We'll let Ceres find an optimal x1,y1 that minimize the cost.
    double param[2];
    // Suppose we start at an initial guess:
    param[0] = 0.871872;  // x1 init
    param[1] = 2.23724;  // y1 init

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
    //options.initial_trust_region_radius = 1.0;  // Larger initial step size
    //options.max_trust_region_radius = 1e4;     // Allow large step sizes
    //options.use_nonmonotonic_steps = true;
    //options.minimizer_type = ceres::LINE_SEARCH;
    //options.function_tolerance = 1e-6;    // Allow lower precision in function evaluation
    //options.gradient_tolerance = 1e-6;   // Allow larger gradients near the solution
    //options.parameter_tolerance = 1e-6;

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
