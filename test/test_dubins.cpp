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


ompl::base::DubinsStateSpace::DubinsPath findDubins(State &start, State &goal, double turning_radius = 1.0)
{
    ompl::base::DubinsStateSpace dubinsSpace(turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);
    dubinsEnd->setXY(goal.x, goal.y);
    dubinsEnd->setYaw(-goal.yaw);
    ompl::base::DubinsStateSpace::DubinsPath dubinsPath = dubinsSpace.dubins(dubinsStart, dubinsEnd);
    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);

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

        std::cout << interState->getX() << " " << interState->getY() << " " << interState->getYaw() << std::endl;
    }

    return dubinsPath;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "hi bmo" << std::endl;
    
    State start(1, 1, 0);
    State goal(17, 17, -1 * M_PI);

    findDubins(start, goal);


    return 0;
}