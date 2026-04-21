#include <PHAstar/Entities.h>
#include <PHAstar/PHAstar.h>
#include <PHAstar/Visualization.h>
#include <ReloPush/PlanHybridAstar.hpp>

#include <QString>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

bool DEBUG_VIS = false;

namespace
{

struct OwnedEntities
{
    std::vector<std::unique_ptr<EntityMeta>> owned;
    std::unordered_map<std::string, EntityMeta *> entities;

    RobotMeta *add_robot(const std::string &name, const Pose &initial_pose)
    {
        auto robot = std::make_unique<RobotMeta>();
        robot->name = name;
        robot->type = EntityType::ROBOT;
        robot->initial_pose = initial_pose;
        robot->size.front_length = Constants::LF_nonpush;
        robot->size.rear_length = Constants::LB;
        robot->size.width = Constants::carWidth;
        robot->min_turning_radius = Constants::r_nonpush;
        robot->wheel_base = Constants::L;
        robot->speed_transit = 0.2;
        robot->speed_transfer = 0.15;

        RobotMeta *raw = robot.get();
        entities[name] = raw;
        owned.push_back(std::move(robot));
        return raw;
    }

    ObjectMeta *add_obstacle(const std::string &name, const Pose &initial_pose)
    {
        auto object = std::make_unique<ObjectMeta>();
        object->name = name;
        object->type = EntityType::OBJECT;
        object->initial_pose = initial_pose;
        object->size.front_length = Constants::obsRadius;
        object->size.rear_length = Constants::obsRadius;
        object->size.width = Constants::obsRadius * 2.0;

        ObjectMeta *raw = object.get();
        entities[name] = raw;
        owned.push_back(std::move(object));
        return raw;
    }
};

double path_length(const std::vector<ReloPush::State> &path)
{
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
    return total;
}

double path_length(const std::vector<Waypoint> &path)
{
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
    {
        total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    }
    return total;
}

Params make_phastar_params()
{
    Params params;
    params.min_x = 0.0;
    params.max_x = 4.0;
    params.min_y = 0.0;
    params.max_y = 5.2;
    params.xy_resolution = Constants::mapResolution;
    params.robot_boundary_origin_only = true;
    params.analytic_threshold = 5.0 * params.max_steer;
    return params;
}

PlanningContext make_hybrid_context()
{
    PlanningParameters params;
    params.boundary = WorkspaceBoundary(4.0, 5.2);

    ObjectMap objects;
    objects["obj_block"] = ObjectInfo("obj_block", 2.0, 0.45, 0.0, 4, Constants::obsRadius);

    PlanningContext ctx(params, objects);
    ctx.timeout_ms = 0;
    ctx.print_res = false;
    return ctx;
}

Trajectory hybrid_path_to_trajectory(RobotMeta *robot,
                                     const std::vector<ReloPush::State> &path,
                                     double speed)
{
    Trajectory traj;
    traj.entity = robot;
    traj.start_time = 0.0;
    traj.is_transfer = false;
    traj.waypoints.reserve(path.size());

    double elapsed = 0.0;
    for (size_t i = 0; i < path.size(); ++i)
    {
        if (i > 0)
        {
            elapsed += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) /
                       std::max(speed, 1e-6);
        }

        Waypoint wp;
        wp.x = path[i].x;
        wp.y = path[i].y;
        wp.yaw = path[i].yaw;
        wp.time = elapsed;
        wp.linear_velocity = speed;
        wp.steering_angle = 0.0;
        traj.waypoints.push_back(wp);
    }

    return traj;
}

Trajectory phastar_waypoints_to_trajectory(RobotMeta *robot,
                                           const std::vector<Waypoint> &path)
{
    Trajectory traj;
    traj.entity = robot;
    traj.start_time = 0.0;
    traj.is_transfer = false;
    traj.waypoints = path;
    return traj;
}

} // namespace

int main(int argc, char **argv)
{
    const ReloPush::State hybrid_start(0.5, 0.45, 0.0);
    const ReloPush::State hybrid_goal(3.5, 0.45, 0.0);
    const Pose phastar_start(hybrid_start.x, hybrid_start.y, hybrid_start.yaw);
    const Pose phastar_goal(hybrid_goal.x, hybrid_goal.y, hybrid_goal.yaw);

    PlanningContext hybrid_ctx = make_hybrid_context();
    auto hybrid_result = planHybridAstar(hybrid_start, hybrid_goal, hybrid_ctx, true);
    if (!hybrid_result || hybrid_result->validity != PlanValidity::success)
    {
        std::cerr << "HybridAstar failed to find a path." << std::endl;
        return 1;
    }
    const auto hybrid_path = hybrid_result->getPath();
    if (hybrid_path.empty())
    {
        std::cerr << "HybridAstar returned an empty path." << std::endl;
        return 1;
    }

    OwnedEntities store;
    RobotMeta *robot = store.add_robot("robot1", phastar_start);
    store.add_obstacle("obj_block", Pose(2.0, 0.45, 0.0));

    Params phastar_params = make_phastar_params();
    TimeTable hybrid_timetable(0.05);
    hybrid_timetable.add_initial(store.entities);

    robot->initial_pose = phastar_start;
    auto hybrid_traj = hybrid_path_to_trajectory(robot, hybrid_path, robot->speed_transit);
    hybrid_timetable.add_trajectory(hybrid_traj);

    TimeTable phastar_timetable(0.05);
    phastar_timetable.add_initial(store.entities);

    robot->initial_pose = phastar_timetable.get_pose(robot, 0.0);
    PHAStar phastar_planner(robot, phastar_goal, &phastar_timetable, &store.entities,
                            phastar_params, false, "", 0.0);
    phastar_planner.max_search_iterations = 10000;
    const auto phastar_result = phastar_planner.Planning_with_res(0.0);
    if (phastar_result.status != PlanningStatus::SUCCESS)
    {
        std::cerr << "PHAstar failed to find a path: " << phastar_result.failure_detail
                  << std::endl;
        return 1;
    }
    if (phastar_result.waypoints.empty())
    {
        std::cerr << "PHAstar returned an empty path." << std::endl;
        return 1;
    }

    auto phastar_traj = phastar_waypoints_to_trajectory(robot, phastar_result.waypoints);
    phastar_timetable.add_trajectory(phastar_traj);

    const QString left_title = QString("HybridAstar\nlength=%1, states=%2")
                                   .arg(path_length(hybrid_path), 0, 'f', 2)
                                   .arg(static_cast<int>(hybrid_path.size()));
    const QString right_title = QString("PHAstar\nlength=%1, waypoints=%2")
                                    .arg(path_length(phastar_result.waypoints), 0, 'f', 2)
                                    .arg(static_cast<int>(phastar_result.waypoints.size()));

    show_results_comparison(argc, argv,
                            left_title,
                            hybrid_timetable,
                            store.entities,
                            phastar_params,
                            right_title,
                            phastar_timetable,
                            store.entities,
                            phastar_params);
    return 0;
}
