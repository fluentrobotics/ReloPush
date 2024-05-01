#ifndef DUBINS_TOOLS_H
#define DUBINS_TOOLS_H

#include <cmath>
#include <chrono>
//#include <graphTools/graph_info.h>
//#include <reloPush/movableObject.h>
#include <omplTools/fromOMPL.h>
#include <omplTools/State.h>

#include <Eigen/Core>
#include <Eigen/Dense>

//#define M_PI 3.14159265358979323846 /* pi */

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;
typedef ompl::base::DubinsStateSpace::DubinsPath dubinsPath;

enum pathType 
    {   smallLP = 0, // small-turn long-path 
        largeLP = 1, // large-turn long-path
        SP = 2, // short-path
        none = -1 //not a path
    };

void jeeho_interpolate(const OmplState *from, const ompl::base::DubinsStateSpace::DubinsPath &path, double t,
                       OmplState *state, ompl::base::DubinsStateSpace* space, double turning_radius);


ompl::base::DubinsStateSpace::DubinsPath findDubins(State &start, State &goal, double turning_radius = 1.0);

// Function to transform a point from the global frame to the robot's frame
Eigen::Vector2d worldToRobot(double x, double y, double theta, double robot_x, double robot_y);

std::pair<pathType,dubinsPath> is_good_path(State& s1, State& s2, float turning_rad);

#endif
