/*************************************
 * Prioritized Hybrid Astar Demo
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/

#include <PHAstar/Reeds_Shepp.h>
#include <PHAstar/Visualization.h>

#include <PHAstar/PHAstar.h>
#include <iomanip>

const bool print_path = false;

Params initialize_params() {
    Params params;
    //params.analytic_threshold = 5.0 * params.max_steer;  // Adjust if needed
    params.analytic_threshold = std::hypot(params.max_x - params.min_x, params.max_y - params.min_y) * 2.0;
    return params;
}

std::unordered_map<std::string, EntityMeta*> initialize_entities() {
    static RobotMeta robot1;
    robot1.name = "robot1";
    robot1.type = EntityType::ROBOT;
    robot1.initial_pose = {1.0, 1.0, 0.0};
    robot1.size = {0.4, 0.2, 0.3};
    robot1.min_turning_radius = 1.0;  // Example
    robot1.wheel_base = 0.4;
    robot1.speed_transit = 0.2;
    robot1.speed_transfer = 0.15;

    static RobotMeta robot2;
    robot2.name = "robot2";
    robot2.type = EntityType::ROBOT;
    robot2.initial_pose = {1.0, 2.0, 0.0};
    robot2.size = {0.4, 0.2, 0.3};
    robot2.min_turning_radius = 1.0;
    robot2.wheel_base = 0.4;
    robot2.speed_transit = 0.2;
    robot2.speed_transfer = 0.15;

    static ObjectMeta obj1;
    obj1.name = "obj1";
    obj1.type = EntityType::OBJECT;
    obj1.initial_pose = {1.6, 1.8, 0};
    obj1.size = {0.075, 0.075, 0.15};
    obj1.goal_pose = {}; // don't care for static obstacles

    static ObjectMeta obj2;
    obj2.name = "obj2";
    obj2.type = EntityType::OBJECT;
    obj2.initial_pose = {3.5, 3.0, 0.0};
    obj2.size = {0.075, 0.075, 0.15};
    obj2.goal_pose = {}; // don't care for static obstacles

    return {
        {"robot1", &robot1}, {"robot2", &robot2}, {"obj1", &obj1}, {"obj2", &obj2}
    };
}

std::vector<std::tuple<std::string, Pose, bool, std::string, double>> initialize_plans() {
    return {
        //{"robot1", {4.0, 4.0, M_PI / 2}, false, "", 0.0}, // agent name, goal pose, is it transferring item, ignore collision with, starting time
        {"robot1", {4.2, 4.2, M_PI / 2}, false, "", 0.0}, // testing close to the edge
        {"robot2", {4.5, 1.0, M_PI / 2}, false, "", 0.0},
        {"robot1", {3, 2, M_PI}, false, "", 29.0},
        {"robot2", {2.0, 3.0, 0.0}, false, "", 32.0}
    };
}

int main(int argc, char** argv) {
    Params params = initialize_params(); // planning parameters

    auto entities = initialize_entities(); // information on all entities (robot and objects)

    auto plans = initialize_plans(); // list of tasks

    TimeTable timetable(0.5);
    timetable.add_initial(entities); // space-time reservation table

    auto all_trajectories = perform_planning(entities, plans, timetable, params, print_path); // plan all paths

    if(print_path)
        print_timetable_poses(entities, all_trajectories, timetable);

    show_results(argc, argv, timetable, entities, all_trajectories, params);

    return 0;
}
