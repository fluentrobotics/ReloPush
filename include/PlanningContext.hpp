#ifndef PLANNERCONTEXT_HPP
#define PLANNERCONTEXT_HPP

#include <PathPlanningTools.h>
#include <Parameters.hpp> // struct with boundary, thresholds, etc.
#include <ObjectInfo.hpp>
#include <unordered_map>

typedef std::vector<ObjectInfo> ObjectList;
typedef std::vector<GoalInfo> GoalList;

typedef std::unordered_map<std::string, ObjectInfo> ObjectMap;
typedef std::unordered_map<std::string, GoalInfo> GoalMap;

State object_to_state(ObjectInfo& obj);

State goal_to_state(GoalInfo& goal);

// Function to convert ObjectMap to std::vector<State>
std::vector<State> convert_to_states(ObjectMap &objects);

// Function to convert GoalMap to std::vector<State>
std::vector<State> convert_to_states(GoalMap &goals);

/**
 * @brief A single struct containing everything needed for planning:
 *        - The environment (collision checks, boundary).
 *        - The planning parameters (turning radius, thresholds, etc.).
 */
struct PlanningContext
{
    Environment env;         ///< The grid map, collision queries, etc.
    PlanningParameters parameters;///< Example: boundary, thresholds, etc.
    //ObjectList mo_list; //movable objects
    //ObjectList delivered_list; //delivered objects

    ObjectMap mo_list;
    GoalMap delivered_list;

    PlanningContext(PlanningParameters params_in, ObjectMap& obs_in) : parameters(params_in), mo_list(obs_in)
    {
        ObjectList static_in = {};
        std::unordered_set<State> obs;
        env = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_push, Constants::LF_push, false); //todo: params:: -> parameters

        parameters.turning_rad_pair.push = Constants::r_push;
        parameters.turning_rad_pair.non_push = Constants::r_nonpush;

        parameters.map_resolution = Constants::mapResolution;
        parameters.car_width = Constants::carWidth;
        parameters.obs_rad = Constants::obsRadius;
        parameters.LF = Constants::LF_push;
        parameters.LB = Constants::LB;
    }

    PlanningContext(PlanningParameters params_in, ObjectMap& obs_in, GoalMap& static_in) : parameters(params_in), mo_list(obs_in), delivered_list(static_in)
    {
        std::unordered_set<State> obs;
        env = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_push, Constants::LF_push, false); //todo: params:: -> parameters

        parameters.turning_rad_pair.push = Constants::r_push;
        parameters.turning_rad_pair.non_push = Constants::r_nonpush;
        //parameters.boundary.xMax = params_in.boundary.xMax;
        //parameters.boundary.yMax = params_in.boundary.yMax;
        //parameters.boundary.xMin = 0; // strong
        //parameters.boundary.yMin = 0; // strong

        parameters.map_resolution = Constants::mapResolution;
        parameters.car_width = Constants::carWidth;
        parameters.obs_rad = Constants::obsRadius;
        parameters.LF = Constants::LF_push;
        parameters.LB = Constants::LB;

        updateObs(mo_list, delivered_list);
    }

    void updateObs()
    {
        std::unordered_set<State> obs;
        auto mo_list_states = convert_to_states(mo_list);
        obs.insert(mo_list_states.begin(), mo_list_states.end());

        auto delivered_list_states = convert_to_states(delivered_list);
        obs.insert(delivered_list_states.begin(), delivered_list_states.end());

        env = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs, parameters.turning_rad_pair.push, parameters.LF, false);
    }

    void updateObs(std::unordered_map<std::string, ObjectInfo>& mo_list_in, std::unordered_map<std::string, GoalInfo>& delivered_list_in)
    {
        std::unordered_set<State> obs;
        auto mo_list_states = convert_to_states(mo_list_in);
        obs.insert(mo_list_states.begin(), mo_list_states.end());

        auto delivered_list_states = convert_to_states(delivered_list_in);
        obs.insert(delivered_list_states.begin(), delivered_list_states.end());

        env = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs, parameters.turning_rad_pair.push, parameters.LF, false);
    }

    void updateObs(std::unordered_set<State>& obs_in)
    {
        env = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs_in, parameters.turning_rad_pair.push, parameters.LF, false);
    }

    // You can add more fields if needed (e.g. special caches or extra data).
};

#endif // PLANNERCONTEXT_HPP
