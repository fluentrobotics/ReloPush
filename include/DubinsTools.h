#ifndef DUBINS_TOOLS_H
#define DUBINS_TOOLS_H

#include <cmath>
#include <chrono>
//#include <graphTools/graph_info.h>
//#include <reloPush/movableObject.h>
#include <FromOMPL.h>
#include <State.h>
//#include <PushPoseTools.h>
#include <PathPlanningTools.h>
#include <PlanningContext.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

//#define M_PI 3.14159265358979323846 /* pi */

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace po = boost::program_options;

typedef ompl::base::SE2StateSpace::StateType OmplState;
typedef ompl::base::DubinsStateSpace::DubinsPath dubinsPath;

void jeeho_interpolate(const OmplState *from, const ompl::base::DubinsStateSpace::DubinsPath &path, double t,
                       OmplState *state, ompl::base::DubinsStateSpace* space, double turning_radius);

class reloDubinsPath{

public:
    dubinsPath omplDubins;
    ReloPush::State startState;
    ReloPush::State targetState;

    reloDubinsPath(int i)
    {}

    reloDubinsPath(ReloPush::State& start, ReloPush::State& target, dubinsPath& omplDubinsPath, float r)
        : startState(start), targetState(target), turning_rad(r) ,omplDubins(omplDubinsPath)
    {}

    reloDubinsPath(const ompl::base::DubinsStateSpace::DubinsPathSegmentType *type = ompl::base::DubinsStateSpace::dubinsPathType[0],
                   double t = 0., double p = std::numeric_limits<double>::max(), double q = 0., float r=1.0)
    {
        omplDubins = dubinsPath(type,t,p,q);
        turning_rad = r;
    }

    reloDubinsPath(ReloPush::State& start, ReloPush::State& target, const ompl::base::DubinsStateSpace::DubinsPathSegmentType *type = ompl::base::DubinsStateSpace::dubinsPathType[0],
                   double t = 0., double p = std::numeric_limits<double>::max(), double q = 0., float r=1.0): startState(start), targetState(target)
    {
        omplDubins = dubinsPath(type,t,p,q);
        turning_rad = r;
    }

    reloDubinsPath(ReloPush::State& start, ReloPush::State& target, dubinsPath& dubins_in) : startState(start), targetState(target)
    {
        omplDubins = dubins_in;
    }

    float lengthCost() {
        return static_cast<float>(omplDubins.length()) * turning_rad;
    }

    float get_turning_radius()
    {
        return turning_rad;
    }

    ReloPush::StatePathPtr interpolate(float resolution)
    {
        auto l = lengthCost(); // unit cost * turning rad
        auto num_pts = static_cast<size_t>(l/resolution);

        ompl::base::DubinsStateSpace dubinsSpace(turning_rad);
        OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
        dubinsStart->setXY(startState.x, startState.y);
        dubinsStart->setYaw(startState.yaw);
        OmplState *interState = (OmplState *)dubinsSpace.allocState();

        std::vector<ReloPush::State> waypoints(num_pts);

        // interpolate dubins path
        // Interpolate dubins path to check for collision on grid map
        //nav_msgs::Path single_path;
        //single_path.poses.resize(num_pts);
        if(num_pts>0){
            for (size_t np=0; np<num_pts; np++)
            {
                //auto start = std::chrono::steady_clock::now();
                jeeho_interpolate(dubinsStart, omplDubins, (double)np / (double)num_pts, interState, &dubinsSpace,
                                  turning_rad);

                ReloPush::State tempState(interState->getX(), interState->getY(),interState->getYaw());
                waypoints[np] = tempState;
            }
        }
        else{
            //std::cout << "Path too short to interpolate" << std::endl;
            waypoints.resize(1);
            waypoints[0] = targetState;
        } // path is too short there is nothing to interpolate


        return std::make_shared<ReloPush::StatePath>(waypoints);
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
    PathPlanResultPtr pathToNextPush; // to next pre-push
    ReloPush::State nextPushState;
    ReloPush::StatePathPtr manual_path; // if another path is preffered for pre-relo
    bool use_dubins;
    ReloPush::State originalState; // from
    ReloPush::State preReloState; // to

    preReloPath()
    {
        preReloDubins = reloDubinsPath(0);
        use_dubins = true;
    }

    preReloPath(ReloPush::State start, ReloPush::State target, reloDubinsPath& dubins_in, PathPlanResultPtr path_to_next_prePush,
                ReloPush::State& next_push, ReloPush::State& prev_state, ReloPush::State& preRelo_state)
    {
        preReloDubins = dubins_in;
        preReloDubins.startState = start;
        preReloDubins.targetState = target;
        pathToNextPush = path_to_next_prePush; // approach path
        nextPushState = next_push; // push pose to next target
        manual_path = nullptr;
        use_dubins = true;
        originalState = prev_state;
        preReloState = preRelo_state;
    }
    preReloPath(ReloPush::State start, ReloPush::State target, ReloPush::StatePathPtr path_in,
                reloDubinsPath& dubins_in, PathPlanResultPtr path_to_next_prePush, ReloPush::State& next_push,
                ReloPush::State& prev_state, ReloPush::State& preRelo_state) // dubins for start and target info
    {
        manual_path = path_in;
        preReloDubins = dubins_in;
        preReloDubins.startState = start;
        preReloDubins.targetState = target;
        pathToNextPush = path_to_next_prePush; // approach path
        nextPushState = next_push; // push pose to next target
        use_dubins = false;
        originalState = prev_state;
        preReloState = preRelo_state;
    }

};

enum pathType
{
    //smallLP = 0, // small-turn long-path
    LP = 0, // large-turn long-path
    SP = 1, // short-path
    none = -1 //not a path
};

ReloPush::StatePathPtr interpolateDubins(reloDubinsPath& dubins_in, PlanningContext& ctx);

StateValidity isDubinsValid(reloDubinsPath& dubins_in, PlanningContext& ctx);

std::vector<ReloPush::State> interpolateStraightPath(const ReloPush::State& start, const ReloPush::State& goal, float resolution);

reloDubinsPath findDubins(ReloPush::State start, ReloPush::State goal, double turning_radius = 1.0, bool print_type = false);

// Function to transform a point from the global frame to the robot's frame
Eigen::Vector2d worldToRobot(double x, double y, double theta, double robot_x, double robot_y);

float get_current_longpath_d(ReloPush::State& s1, ReloPush::State& s2);
float get_longpath_d_thres(ReloPush::State& s1, ReloPush::State& s2, float turning_rad = 1.0f);
bool is_longpath_case(ReloPush::State& s1, ReloPush::State& s2, double turning_rad);
//std::pair<pathType,reloDubinsPath> is_good_path(State& s1, State& s2, float turning_rad, bool use_pre_push_pose = true);

std::pair<pathType,reloDubinsPath> PlanDubins(ReloPush::State s1, ReloPush::State& s2, PlanningContext& ctx, bool use_pre_push_pose = true);



#endif
