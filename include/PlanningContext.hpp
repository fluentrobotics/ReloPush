#ifndef PLANNERCONTEXT_HPP
#define PLANNERCONTEXT_HPP

#include <PathPlanningTools.h>
#include <Parameters.hpp> // struct with boundary, thresholds, etc.
#include <ObjectInfo.hpp>
#include <unordered_map>

typedef std::vector<ObjectInfo> ObjectList;
typedef std::vector<GoalInfo> GoalList;

ReloPush::State object_to_state(ObjectInfo& obj);

ReloPush::State goal_to_state(GoalInfo& goal);

// Function to convert ObjectMap to std::vector<State>
std::vector<ReloPush::State> convert_to_states(ObjectMap &objects);

// Function to convert GoalMap to std::vector<State>
std::vector<ReloPush::State> convert_to_states(GoalMap &goals);

/**
 * @brief A single struct containing everything needed for planning:
 *        - The environment (collision checks, boundary).
 *        - The planning parameters (turning radius, thresholds, etc.).
 */
struct PlanningContext
{
    Environment env_push, env_nonpush;         ///< The grid map, collision queries, etc.
    PlanningParameters parameters;///< Example: boundary, thresholds, etc.
    //ObjectList mo_list; //movable objects
    //ObjectList delivered_list; //delivered objects

    ObjectMap mo_list;
    GoalMap delivered_list;
    int sample_N = 25;
    int64_t timeout_ms = 0; // 0: no timeout for hybrid-astar
    bool print_res = false; // print result for hybrid-astar
    bool use_prelo_optimization = false;


    // Evenly sampled positions for optimizations
    std::vector<ReloPush::State> sampledPositions;

    PlanningContext()
    {

    }

    PlanningContext(PlanningParameters params_in, ObjectMap& obs_in) : parameters(params_in), mo_list(obs_in)
    {
        ObjectList static_in = {};
        std::unordered_set<ReloPush::State> obs;
        env_push = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_push, Constants::LF_push, false); //todo: params:: -> parameters
        env_nonpush = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_nonpush, Constants::LF_nonpush, true);

        parameters.turning_rad_pair.push = Constants::r_push;
        parameters.turning_rad_pair.non_push = Constants::r_nonpush;

        parameters.map_resolution = Constants::mapResolution;
        parameters.car_width = Constants::carWidth;
        parameters.obs_rad = Constants::obsRadius;
        parameters.LF_push = Constants::LF_push;
        parameters.LF_nonpush = Constants::LF_nonpush;
        parameters.LB = Constants::LB;

        updateObs(mo_list, delivered_list);
        //uniformSampleMap(sample_N);
    }

    // todo: combine the constructors
    PlanningContext(PlanningParameters params_in, ObjectMap& obs_in, GoalMap& static_in) : parameters(params_in), mo_list(obs_in), delivered_list(static_in)
    {
        std::unordered_set<ReloPush::State> obs;
        env_push = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_push, Constants::LF_push, false); //todo: params:: -> parameters
        env_nonpush = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_nonpush, Constants::LF_nonpush, true);

        parameters.turning_rad_pair.push = Constants::r_push;
        parameters.turning_rad_pair.non_push = Constants::r_nonpush;

        parameters.map_resolution = Constants::mapResolution;
        parameters.car_width = Constants::carWidth;
        parameters.obs_rad = Constants::obsRadius;
        parameters.LF_push = Constants::LF_push;
        parameters.LF_nonpush = Constants::LF_nonpush;
        parameters.LB = Constants::LB;

        updateObs(mo_list, delivered_list);
        //uniformSampleMap(sample_N);
    }

    PlanningContext(PlanningParameters params_in, ObjectMap& obs_in, GoalMap& static_in, bool& use_opt)
        : parameters(params_in), mo_list(obs_in), delivered_list(static_in), use_prelo_optimization(use_opt)
    {
        std::unordered_set<ReloPush::State> obs;
        env_push = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_push, Constants::LF_push, false); //todo: params:: -> parameters
        env_nonpush = Environment(params_in.boundary.xMax, params_in.boundary.yMax, obs, Constants::r_nonpush, Constants::LF_nonpush, true);

        parameters.turning_rad_pair.push = Constants::r_push;
        parameters.turning_rad_pair.non_push = Constants::r_nonpush;

        parameters.map_resolution = Constants::mapResolution;
        parameters.car_width = Constants::carWidth;
        parameters.obs_rad = Constants::obsRadius;
        parameters.LF_push = Constants::LF_push;
        parameters.LF_nonpush = Constants::LF_nonpush;
        parameters.LB = Constants::LB;

        updateObs(mo_list, delivered_list);
        //uniformSampleMap(sample_N);
    }

    void updateObs()
    {
        std::unordered_set<ReloPush::State> obs;
        auto mo_list_states = convert_to_states(mo_list);
        obs.insert(mo_list_states.begin(), mo_list_states.end());

        auto delivered_list_states = convert_to_states(delivered_list);
        obs.insert(delivered_list_states.begin(), delivered_list_states.end());

        env_push = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs, parameters.turning_rad_pair.push, parameters.LF_push, false);
        env_nonpush = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs, parameters.turning_rad_pair.non_push, parameters.LF_nonpush, true);
    }

    void updateObs(std::unordered_map<std::string, ObjectInfo>& mo_list_in, std::unordered_map<std::string, GoalInfo>& delivered_list_in)
    {
        std::unordered_set<ReloPush::State> obs;
        auto mo_list_states = convert_to_states(mo_list_in);
        obs.insert(mo_list_states.begin(), mo_list_states.end());

        auto delivered_list_states = convert_to_states(delivered_list_in);
        obs.insert(delivered_list_states.begin(), delivered_list_states.end());

        env_push = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs, parameters.turning_rad_pair.push, parameters.LF_push, false);
        env_nonpush = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs, parameters.turning_rad_pair.non_push, parameters.LF_nonpush, true);
    }

    void updateObs(std::unordered_set<ReloPush::State>& obs_in)
    {
        env_push = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs_in, parameters.turning_rad_pair.push, parameters.LF_push, false);
        env_nonpush = Environment(parameters.boundary.xMax, parameters.boundary.yMax, obs_in, parameters.turning_rad_pair.non_push, parameters.LF_nonpush, true);
    }

    void removeObs(ReloPush::State obs_in)
    {
        env_push.remove_obs(obs_in);
        env_nonpush.remove_obs(obs_in);
    }

    void addObs(ReloPush::State obs_in)
    {
        env_push.add_obs(obs_in);
        env_nonpush.add_obs(obs_in);
    }

    // Function to compute grid dimensions (rows and columns) based on N and aspect ratio
    std::pair<int, int> computeGridDimensions(int N, double width, double height) {
        // Start with the square root to get an initial estimate
        int cols = static_cast<int>(std::ceil(std::sqrt(N * (width / height))));
        int rows = static_cast<int>(std::ceil(static_cast<double>(N) / cols));
        return {rows, cols};
    }

    void uniformSampleMap(int N)
    {
        std::cout << "Uniform Sample Map (" << N << ")" << std::endl;
        if (N <= 0) {
            std::cerr << "Number of samples must be positive." << std::endl;
            //return samples;
        }

        sampledPositions.reserve(N);

        double min_x = parameters.boundary.xMin;
        double min_y = parameters.boundary.yMin;
        double max_x = parameters.boundary.xMax;
        double max_y = parameters.boundary.yMax;

        double width = max_x - min_x;
        double height = max_y - min_y;

        // Compute grid dimensions
        auto [rows, cols] = computeGridDimensions(N, width, height);

        // Compute spacing between samples
        double dx = width / cols;
        double dy = height / rows;

        // Generate sample positions
        for (int row = 0; row < rows && sampledPositions.size() < static_cast<size_t>(N); ++row) {
            for (int col = 0; col < cols && sampledPositions.size() < static_cast<size_t>(N); ++col) {
                // Compute the center of the current grid cell
                double x = min_x + (col + 0.5) * dx;
                double y = min_y + (row + 0.5) * dy;
                sampledPositions.emplace_back(ReloPush::State(x, y,0));
            }
        }

        //return samples;
    }

    // You can add more fields if needed (e.g. special caches or extra data).
};

#endif // PLANNERCONTEXT_HPP
