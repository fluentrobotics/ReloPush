#ifndef DUBINS_TOOLS_H
#define DUBINS_TOOLS_H

#include <cmath>
#include <chrono>
//#include <graphTools/graph_info.h>
//#include <reloPush/movableObject.h>
#include <omplTools/fromOMPL.h>
#include <omplTools/State.h>
#include <reloPush/push_pose_tools.h>
#include <pathPlanTools/path_planning_tools.h>

#include <Eigen/Core>
#include <Eigen/Dense>

//#define M_PI 3.14159265358979323846 /* pi */

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;
typedef ompl::base::DubinsStateSpace::DubinsPath dubinsPath;

class reloDubinsPath{

public:
    dubinsPath omplDubins;
    State startState;
    State targetState;

    reloDubinsPath(int i)
    {}

    reloDubinsPath(dubinsPath& omplDubinsPath) : omplDubins(omplDubinsPath)
    {}

    reloDubinsPath(const ompl::base::DubinsStateSpace::DubinsPathSegmentType *type = ompl::base::DubinsStateSpace::dubinsPathType[0],
                         double t = 0., double p = std::numeric_limits<double>::max(), double q = 0., float r=1.0)
    {
        omplDubins = dubinsPath(type,t,p,q);
        turning_rad = r;
    }

    reloDubinsPath(State& start, State& target, dubinsPath& dubins_in) : startState(start), targetState(target)
    {
        omplDubins = dubins_in;
    }

    float lengthCost() {
        return static_cast<float>(omplDubins.length()) * turning_rad;
    }

private:
    float turning_rad =1.0;
    void set_r(float r){
        turning_rad = r;
    }
};

class preReloPath{
    public:
    reloDubinsPath preReloDubins;
    std::shared_ptr<PlanResult<State, Action, double>> pathToNextPush; // to next pre-push

    preReloPath()
    {
        preReloDubins = reloDubinsPath(0);
    }

    preReloPath(State start, State target, reloDubinsPath& dubins_in, std::shared_ptr<PlanResult<State, Action, double>> path_to_next_prePush)
    {
        preReloDubins = dubins_in;
        preReloDubins.startState = start;
        preReloDubins.targetState = target;
        pathToNextPush = path_to_next_prePush;
    }
};

enum pathType 
    {   smallLP = 0, // small-turn long-path 
        largeLP = 1, // large-turn long-path
        SP = 2, // short-path
        none = -1 //not a path
    };

void jeeho_interpolate(const OmplState *from, const ompl::base::DubinsStateSpace::DubinsPath &path, double t,
                       OmplState *state, ompl::base::DubinsStateSpace* space, double turning_radius);


reloDubinsPath findDubins(State &start, State &goal, double turning_radius = 1.0);

// Function to transform a point from the global frame to the robot's frame
Eigen::Vector2d worldToRobot(double x, double y, double theta, double robot_x, double robot_y);

float get_longpath_d_thres(State& s1, State& s2, float turning_rad = 1.0f);
std::pair<pathType,reloDubinsPath> is_good_path(State& s1, State& s2, float turning_rad, bool use_pre_push_pose = true);



#endif
