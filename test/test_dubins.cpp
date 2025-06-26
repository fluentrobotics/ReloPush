#include<iostream>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <boost/program_options.hpp>

#include <ompl/geometric/planners/rrt/RRT.h>

#include <cmath>
#include <chrono>

#define M_PI 3.14159265358979323846 /* pi */

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;

struct State
{
    State(double x, double y, double yaw, int time = 0) : x(x), y(y), yaw(yaw), time(time)
    {
    }

    State() = default;

    bool operator==(const State &s) const
    {
        return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
    }

    State(const State &) = default;
    State(State &&) = default;
    State &operator=(const State &) = default;
    State &operator=(State &&) = default;

    friend std::ostream &operator<<(std::ostream &os, const State &s)
    {
        return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")";
    }

    double x;
    double y;
    double yaw;
    int time;
};

void jeeho_interpolate(const OmplState *from, const ompl::base::DubinsStateSpace::DubinsPath &path, double t,
                       OmplState *state, ompl::base::DubinsStateSpace* space, double turning_radius)
{
    OmplState *s = space->allocState()->as<OmplState>();
    double seg = t * path.length(), phi, v;

    s->setXY(0,0);
    s->setYaw(from->getYaw());
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[i])
            {
            case ompl::base::DubinsStateSpace::DUBINS_LEFT:
                s->setXY(s->getX() + sin(phi + v) - sin(phi), s->getY() - cos(phi + v) + cos(phi));
                s->setYaw(phi + v);
                break;
            case ompl::base::DubinsStateSpace::DUBINS_RIGHT:
                s->setXY(s->getX() - sin(phi - v) + sin(phi), s->getY() + cos(phi - v) - cos(phi));
                s->setYaw(phi - v);
                break;
            case ompl::base::DubinsStateSpace::DUBINS_STRAIGHT:
                s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                break;
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[2 - i])
            {
            case ompl::base::DubinsStateSpace::DUBINS_LEFT:  // DUBINS_LEFT
                s->setXY(s->getX() + sin(phi - v) - sin(phi), s->getY() - cos(phi - v) + cos(phi));
                s->setYaw(phi - v);
                break;
            case ompl::base::DubinsStateSpace::DUBINS_RIGHT:  // DUBINS_RIGHT
                s->setXY(s->getX() - sin(phi + v) + sin(phi), s->getY() + cos(phi + v) - cos(phi));
                s->setYaw(phi + v);
                break;
            case ompl::base::DubinsStateSpace::DUBINS_STRAIGHT:  // DUBINS_STRAIGHT
                s->setXY(s->getX() - v * cos(phi), s->getY() - v * sin(phi));
                break;
            }
        }
    }


    state->setX(s->getX() * turning_radius + from->getX());
    state->setY(s->getY() * turning_radius + from->getY());
    space->getSubspace(1)->enforceBounds(s->as<OmplState>(1));
    state->setYaw(s->getYaw());
    space->freeState(s);
}


// from OMPL
double twopi = 2. * boost::math::constants::pi<double>();
const double DUBINS_EPS = 1e-6;
const double DUBINS_ZERO = -1e-7;

double mod2pi(double x)
{
    if (x < 0 && x > DUBINS_ZERO) //DUBINS_ZERO
        return 0;
    double xm = x - twopi * floor(x / twopi);
    if (twopi - xm < .5 * DUBINS_EPS) //DUBINS_EPS
        xm = 0.;
    return xm;
}

void find_alpha_beta_ompl(State& s1, State& s2, double& alpha_out, double& beta_out)
{
    double x1 = s1.x, y1 = s1.y, th1 = s1.yaw;
    double x2 = s2.x, y2 = s2.y, th2 = s2.yaw;
    double dx = x2 - x1, dy = y2 - y1, th = atan2(dy, dx);
    alpha_out = mod2pi(th1 - th), beta_out = mod2pi(th2 - th);
}

// determine if it is a long-path
double longpath_thres_dist(double& alpha, double& beta)
{
    return std::abs(std::sin(alpha)) + std::abs(std::sin(beta)) +
           std::sqrt(4 - std::pow(std::cos(alpha) + std::cos(beta), 2));
}

// from OMPL
bool is_longpath_case(double d, double alpha, double beta)
{
    return (longpath_thres_dist(alpha,beta) - d) < 0;
}

bool is_longpath_case(State& s1, State& s2, double turning_rad)
{
    double alpha, beta;
    find_alpha_beta_ompl(s1,s2,alpha,beta); //todo: investigate if OMPL's alpha and beta is needed
    double dx = s2.x - s1.x, dy = s2.y - s1.y, d = sqrt(dx * dx + dy * dy) / turning_rad;
    return is_longpath_case(d,alpha,beta);
}


ompl::base::DubinsStateSpace::DubinsPath findDubins(State &start, State &goal, double turning_radius = 1.0)
{
    // is long_path?
    std::cout << "Is it Long-Path case? " << is_longpath_case(start,goal,turning_radius) << std::endl;
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(start.x, start.y);
    //dubinsStart->setYaw(-start.yaw);
    dubinsStart->setYaw(mod2pi(start.yaw));
    dubinsEnd->setXY(goal.x, goal.y);
    //dubinsEnd->setYaw(-goal.yaw);
    dubinsEnd->setYaw(mod2pi(goal.yaw));
    ompl::base::DubinsStateSpace::DubinsPath dubinsPath = dubinsSpace.dubins(dubinsStart, dubinsEnd);
    //dubinsStart->setXY(start.x, start.y);
    //dubinsStart->setYaw(-start.yaw);

    for (auto pathidx = 0; pathidx < 3; pathidx++)
    {
        switch (dubinsPath.type_[pathidx])
        {
        case 0:  // DUBINS_LEFT
            std::cout << "Left" << std::endl;
            break;
        case 1:  // DUBINS_STRAIGHT
            std::cout << "Straight" << std::endl;
            break;
        case 2:  // DUBINS_RIGHT
            std::cout << "Right" << std::endl;
            break;
        default:
            std::cout << "\033[1m\033[31m"
                      << "Warning: Receive unknown DubinsPath type"
                      << "\033[0m\n";
            break;
        }
        std::cout << fabs(dubinsPath.length_[pathidx]) << std::endl;
    }

    std::cout << "Total Length: " << dubinsPath.length() * turning_radius << std::endl;

    OmplState *interState = (OmplState *)dubinsSpace.allocState();
    // auto path_g = generateSmoothPath(dubinsPath,0.1);

    size_t num_pts = 100;

    for (size_t n=0; n<num_pts; n++)
    {
        //auto start = std::chrono::steady_clock::now();
        jeeho_interpolate(dubinsStart, dubinsPath, (double)n / (double)num_pts, interState, &dubinsSpace,
                          turning_radius);
        //auto end = std::chrono::steady_clock::now();
        //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        //std::cout << "Elapsed time: " << duration << " usec" << std::endl;

        std::cout << "[" << interState->getX() << ", " << interState->getY() << ", " << interState->getYaw() << "]";
        if(n != num_pts-1)
            std::cout << "," << std::endl;
        else {
            std::cout << std::endl;
        }
    }

    return dubinsPath;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "hi bmo" << std::endl;

    //State start(-0.256899,2.79287, 0);
    //State goal(1.3815808296203613,2.2772977352142334, 0);


    ///State start(0.906355,2.29097, 0.2562);
    ///State goal(1.3815808296203613,2.2772977352142334, 0);


    //New X: 1.38189
    //New Y: 2.40132
    //New X: -0.301736
    //New Y: 7.4522
    //New X: 0.538392`
    //New Y: 1.80054
    //New X: 0.273708
    //New Y: 2.50049
    //State start(0.223708,2.50049, 0);

    State start(1.11157628, 1.136766, 1.5395157545608618);
    State goal(0.9260568763535091, 2.612194685829129, 2.3200068632748465);

    //State start(1.126264805132916, 1.6061862941057266, 1.5395157545608618);
    //State goal(0.6061966419219971, 2.956084966659546, 2.3200068632748465);

    // New X: 0.387293
    // New Y: 2.38971
    // New th2: 0.117008
    // State start(0.387293,2.38971, 0);
    // State goal(1.3815808296203613,2.2772977352142334, -0.117008);

    findDubins(start, goal, 1.41);
    //  findDubins(start, goal,1);


    return 0;
}
