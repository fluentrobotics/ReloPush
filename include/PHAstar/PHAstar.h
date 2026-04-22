/*************************************
 * Prioritized Hybrid Astar
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
 *************************************/

#ifndef PHASTAR_H
#define PHASTAR_H

#include <PHAstar/Node.h>
#include <PHAstar/Point.h>
#include <PHAstar/Entities.h>
#include <PHAstar/Params.h>
#include <PHAstar/Reeds_Shepp.h>
#include <PHAstar/TimeTable.h>
#include <PHAstar/Visualization.h>
#include <PHAstar/PlanningResult.h>
#include <PHAstar/CollisionUtils.h>

// PHA* Implementation

extern bool DEBUG_VIS;

bool is_in_bounds(const Corners &corners, double min_x, double max_x, double min_y, double max_y)
{
    for (const auto &c : corners)
    {
        if (!(min_x <= c.x && c.x <= max_x && min_y <= c.y && c.y <= max_y))
        {
            // std::cout << "Out of bounds for corner: x=" << c.x << ", y=" << c.y << std::endl;
            return false;
        }
    }
    return true;
}

/*
struct Node {
    double x, y, yaw, t, cost, steer;
    Node* parent;
    int direction;
    Node(double x_ = 0, double y_ = 0, double yaw_ = 0, double t_ = 0, double cost_ = 0, double steer_ = 0, Node* p = nullptr, int dir = 1)
        : x(x_), y(y_), yaw(yaw_), t(t_), cost(cost_), steer(steer_), parent(p), direction(dir) {}

};
*/

class PHAStar
{
public:
    RobotMeta *robot;
    std::unique_ptr<Node> start, goal;
    TimeTable *timetable;
    const std::unordered_map<std::string, EntityMeta *> *entities;
    ObjectMeta *transferred = nullptr;
    EntityMeta *ignored_entity = nullptr;
    bool is_transfer;
    bool ignore_other_robots = false; // New flag for ghost planning
    bool suppress_debug_popup = false;
    std::string debug_plan_kind;
    Params params;
    int x_width, y_width, theta_width, time_width;
    std::vector<double> entity_diags;

    // Backup path (valid geometry but blocked by relocatable robot)
    PlanningResult backup_result;

    // Search limits
    int max_search_iterations = 20000;

    // For diagnostics
    double max_planning_time = 10.0; // [s] timeout threshold (tune via params if needed)
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock;

    size_t calc_grid_index(const Node *node)
    {
        int ix = static_cast<int>((node->x - params.min_x) / params.xy_resolution);
        int iy = static_cast<int>((node->y - params.min_y) / params.xy_resolution);
        int itheta = static_cast<int>(node->yaw / params.yaw_resolution) % theta_width;
        int itime = static_cast<int>(node->t / params.time_resolution);
        return (((static_cast<size_t>(iy) * x_width + ix) * theta_width + itheta) * time_width) + itime;
    }

    Node *new_node(double x, double y, double yaw, double t, double cost, double steer, Node *parent, int direction)
    {
        all_nodes.emplace_back(std::make_unique<Node>(x, y, yaw, t, cost, steer, parent, direction));
        return all_nodes.back().get();
    }

    std::vector<std::unique_ptr<Node>> all_nodes; // To manage memory

    double speed;
    double max_steer;
    double max_curvature;
    double wheel_base;
    double min_turn_radius;
    std::vector<std::pair<int, double>> motion_primitives;

    Node *generate_node(Node *current, std::pair<int, double> prim)
    {
        int direction = prim.first;
        double steer = prim.second;
        double x = current->x, y = current->y, yaw = current->yaw;
        double additional_cost = 0.0;
        double distance = 0.0;
        if (direction == 0)
        { // Wait
            additional_cost = params.wait_penalty;
        }
        else
        {
            additional_cost += (direction != current->direction) ? params.switch_penalty : 0.0;
            additional_cost += (direction < 0) ? params.reverse_penalty : 0.0;
            additional_cost += (std::abs(steer) > 1e-5) ? params.turn_penalty : 0.0;
            double d = direction * params.movement_length;
            distance = std::abs(d);
            double steer_adjusted = (direction > 0) ? steer : -steer;
            if (std::abs(steer_adjusted) < 1e-5)
            {
                x += d * std::cos(yaw);
                y += d * std::sin(yaw);
            }
            else
            {
                double R = wheel_base / std::tan(steer_adjusted);
                double beta = d / R;
                x += R * (std::sin(yaw + beta) - std::sin(yaw));
                y += R * (std::cos(yaw) - std::cos(yaw + beta));
                yaw += beta;
            }
        }
        double t = current->t + params.time_step;
        if (t > params.max_time)
            return nullptr;
        double cost = current->cost + additional_cost + distance;
        return new_node(x, y, yaw, t, cost, steer, current, direction);
    }

    CollisionInfo check_collision_at(const Node *node)
    {
        CollisionInfo result;
        result.time = node->t;

        double t = node->t;
        Pose robot_pose = {node->x, node->y, node->yaw};
        auto poses = timetable->get_poses(t);

        // Setup robot collision geometry
        CollisionGeometry robot_geom = setup_collision_geometry_for_type(
            robot_pose, EntityType::ROBOT, robot->size, params);
        CollisionGeometry robot_geom_bounds =
            setup_collision_geometry(robot_pose, robot->size, 1.0);

        // Check robot bounds
        if (check_robot_bounds_collision(robot_pose, robot_geom_bounds.corners, params))
        {
            return {false, "Robot Out of Bounds", "Boundary", t};
        }

        // Setup object geometry (if transferring)
        CollisionGeometry obj_geom;
        Pose obj_pose;
        const CollisionGeometry *obj_geom_ptr = nullptr;
        const Pose *obj_pose_ptr = nullptr;

        if (is_transfer && transferred)
        {
            obj_pose = TimeTable::compute_object_pose(robot_pose, robot->size, transferred->size);
            obj_geom = setup_collision_geometry_for_type(
                obj_pose, EntityType::OBJECT, transferred->size, params);
            CollisionGeometry obj_geom_bounds =
                setup_collision_geometry(obj_pose, transferred->size, 1.0);

            // Check object bounds
            if (check_object_bounds_collision(obj_pose, obj_geom_bounds.corners, params))
            {
                return {false, "Object Out of Bounds", "Boundary", t};
            }

            obj_geom_ptr = &obj_geom;
            obj_pose_ptr = &obj_pose;
        }

        // Check collisions against all entities
        // Filter out robots if ignore_other_robots is true
        std::unordered_map<EntityMeta *, Pose> filtered_poses;
        if (ignore_other_robots)
        {
            for (const auto &[ent, p] : poses)
            {
                if (ent->type != EntityType::ROBOT)
                {
                    filtered_poses[ent] = p;
                }
            }
        }

        auto collision_result = check_multiple_entities_collision(
            robot_geom, robot_pose,
            obj_geom_ptr, obj_pose_ptr,
            ignore_other_robots ? filtered_poses : poses, params,
            robot, transferred, ignored_entity);

        if (collision_result.has_collision)
        {
            if (!is_transfer && collision_result.colliding_entity &&
                collision_result.colliding_entity->type == EntityType::OBJECT)
            {
                auto it_pose = poses.find(collision_result.colliding_entity);
                bool near_start = start && (t <= start->t + 0.3 + 1e-9);
                if (near_start && it_pose != poses.end() &&
                    is_valid_transfer_contact_pose(robot_pose,
                                                   dynamic_cast<ObjectMeta *>(collision_result.colliding_entity),
                                                   it_pose->second))
                {
                    return {true, "Valid", "", t};
                }
            }

            return {false, collision_result.collision_type + " Collision",
                    collision_result.colliding_entity->name, t};
        }

        return {true, "Valid", "", t};
    }

    bool check_collision_along_path(Node *current, Node *new_node, std::pair<int, double> prim)
    {
        int direction = prim.first;
        double steer = prim.second;
        double time_inc = params.time_step;
        double primitive_length = std::abs(speed * params.time_step);

        for (int i = 1; i <= params.collision_steps; ++i)
        {
            double frac = static_cast<double>(i) / params.collision_steps;
            double x_i = current->x, y_i = current->y, yaw_i = current->yaw;

            // Compute intermediate pose
            if (direction != 0)
            {
                double steer_adjusted = (direction > 0) ? steer : -steer;
                double d = direction * (frac * primitive_length);
                if (std::abs(steer_adjusted) < 1e-5)
                {
                    x_i += d * std::cos(yaw_i);
                    y_i += d * std::sin(yaw_i);
                }
                else
                {
                    double R = wheel_base / std::tan(steer_adjusted);
                    double beta = d / R;
                    x_i += R * (std::sin(yaw_i + beta) - std::sin(yaw_i));
                    y_i += R * (std::cos(yaw_i) - std::cos(yaw_i + beta));
                    yaw_i += beta;
                }
            }

            double t_i = current->t + frac * time_inc;
            Node temp_node(x_i, y_i, yaw_i, t_i, 0, steer, nullptr, direction);
            CollisionInfo info = check_collision_at(&temp_node);
            if (!info.is_valid)
            {
                return false;
            }
        }
        return true;
    }

    std::pair<std::vector<std::tuple<double, double, double>>, double> analytic_expand(Node *node)
    {
        double dx = goal->x - node->x;
        double dy = goal->y - node->y;
        double dist = std::hypot(dx, dy);
        if (dist > params.analytic_threshold)
            return {{}, 0.0};
        auto [rs_x, rs_y, rs_yaw, ctypes, rs_lengths, steers, dirs] = ReedShepp::reeds_shepp_path_planning(node->x, node->y, node->yaw, goal->x, goal->y, goal->yaw, max_curvature, params.rs_step_size, wheel_base);
        if (rs_x.empty())
            return {{}, 0.0};
        double rs_length = 0.0;
        for (double l : rs_lengths)
            rs_length += std::abs(l);
        std::vector<std::tuple<double, double, double>> rs_path;
        for (size_t i = 0; i < rs_x.size(); ++i)
        {
            rs_path.emplace_back(rs_x[i], rs_y[i], rs_yaw[i]);
        }
        return {rs_path, rs_length};
    }
    /*
        bool check_collision_along_rs(const std::vector<std::tuple<double, double, double>>& rs_path, double current_t) {
            double front_inf = robot->size.front_length * params.inflation;
            double rear_inf = robot->size.rear_length * params.inflation;
            double width_inf = robot->size.width * params.inflation;
            double diag_r = std::sqrt((front_inf + rear_inf) * (front_inf + rear_inf) + width_inf * width_inf) / 2;
            for (size_t i = 0; i < rs_path.size() - 1; ++i) {
                auto [x1, y1, yaw1] = rs_path[i];
                auto [x2, y2, yaw2] = rs_path[i + 1];
                double dx = x2 - x1;
                double dy = y2 - y1;
                double dist = std::hypot(dx, dy);
                if (dist > 0) {
                    double time_inc = dist / speed;
                    for (int j = 1; j <= params.collision_steps; ++j) {
                        double frac = static_cast<double>(j) / params.collision_steps;
                        double x_i = x1 + frac * dx;
                        double y_i = y1 + frac * dy;
                        double yaw_i = yaw1 + frac * (yaw2 - yaw1);
                        double t_i = current_t + frac * time_inc;
                        auto corners_r = get_corners(x_i, y_i, yaw_i, front_inf, rear_inf, width_inf);
                        if (!is_in_bounds(corners_r, params.min_x, params.max_x, params.min_y, params.max_y)) return false;
                        auto poses = timetable->get_poses(t_i);
                        size_t idx = 0;
                        for (const auto& [ent, pose] : poses) {
                            if (ent == robot || ent == transferred || ent == ignored_entity) continue;
                            double dist_o = std::hypot(x_i - pose.x, y_i - pose.y);
                            if (dist_o > diag_r + entity_diags[idx] + params.safety_margin) {
                                ++idx;
                                continue;
                            }
                            auto corners_o = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
                            if (rectangles_intersect(corners_r, corners_o)) return false;
                            ++idx;
                        }
                    }
                    current_t += time_inc;
                }
            }
            return true;
        }
        */
    CollisionInfo check_collision_along_rs(const std::vector<std::tuple<double, double, double>> &rs_path, double current_t)
    {
        // We already have detailed logic in check_collision_at, but RS checks intermediate points.
        // Let's reimplement this loop to be consistent.

        double speed_val = speed;
        if (speed_val <= 1e-6)
            speed_val = 0.1; // Prevent division by zero

        bool robot_collision_found = false;
        CollisionInfo robot_col_info;

        for (size_t i = 0; i < rs_path.size() - 1; ++i)
        {
            auto [x1, y1, yaw1] = rs_path[i];
            auto [x2, y2, yaw2] = rs_path[i + 1];

            double dx = x2 - x1;
            double dy = y2 - y1;
            double dist = std::hypot(dx, dy);

            if (dist > 1e-5)
            {
                double time_inc = dist / speed_val;

                // Interpolate along this segment
                for (int j = 1; j <= params.collision_steps; ++j)
                {
                    double frac = static_cast<double>(j) / params.collision_steps;
                    double x_i = x1 + frac * dx;
                    double y_i = y1 + frac * dy;

                    // Interpolate Angle correctly
                    double dyaw = yaw2 - yaw1;
                    while (dyaw > M_PI)
                        dyaw -= 2 * M_PI;
                    while (dyaw < -M_PI)
                        dyaw += 2 * M_PI;
                    double yaw_i = yaw1 + frac * dyaw;

                    double t_i = current_t + frac * time_inc;

                    // Create a temporary Node to reuse our robust check_collision_at function
                    Node temp_node(x_i, y_i, yaw_i, t_i, 0, 0, nullptr, 1); // Cost/Steer irrelevant for collision

                    CollisionInfo info = check_collision_at(&temp_node);
                    if (!info.is_valid)
                    {
                        // Only treat as Soft Collision if it's a Robot AND not the Boundary
                        if (info.reason.find("Robot") != std::string::npos && info.entity_name != "Boundary")
                        {
                            // Soft Collision: Record and Continue
                            if (!robot_collision_found)
                            {
                                robot_collision_found = true;
                                robot_col_info = info;
                            }
                        }
                        else
                        {
                            // Hard Collision: Return Failure Immediately
                            return info;
                        }
                    }
                }
                current_t += time_inc;
            }
        }

        if (robot_collision_found)
        {
            return robot_col_info; // Allows caller to identify it was blocked by robot
        }

        return {true, "Valid", "", 0.0};
    }

    double calc_f(Node *node)
    {
        return node->cost + calc_heuristic(node);
    }

    double calc_heuristic(Node *node)
    {
        auto [_, __, ___, ____, lengths, _____, ______] = ReedShepp::reeds_shepp_path_planning(
            node->x, node->y, node->yaw, goal->x, goal->y, goal->yaw, max_curvature, params.rs_step_size * 10, wheel_base);
        if (lengths.empty())
            return std::numeric_limits<double>::infinity();
        double h = 0.0;
        for (double l : lengths)
            h += std::abs(l);
        return h;
    }

    std::vector<Waypoint> extract_path(Node *node, const std::vector<std::tuple<double, double, double>> &rs_path)
    {
        std::vector<Waypoint> waypoints;
        Node *current = node;
        while (current)
        {
            Waypoint w;
            w.time = current->t;
            w.x = current->x;
            w.y = current->y;
            w.yaw = current->yaw;
            w.linear_velocity = current->direction * speed;
            w.steering_angle = current->steer;
            waypoints.push_back(w);
            current = current->parent;
        }
        std::reverse(waypoints.begin(), waypoints.end());
        if (!rs_path.empty())
        {
            auto [rs_x, rs_y, rs_yaw, rs_ctypes, rs_lengths, rs_steers, rs_directions] = ReedShepp::reeds_shepp_path_planning(
                node->x, node->y, node->yaw, goal->x, goal->y, goal->yaw, max_curvature, params.rs_step_size, wheel_base);
            double rs_t = waypoints.back().time;
            for (size_t i = 1; i < rs_x.size(); ++i)
            {
                double dist = std::hypot(rs_x[i] - rs_x[i - 1], rs_y[i] - rs_y[i - 1]);
                rs_t += dist / speed;
                Waypoint w;
                w.time = rs_t;
                w.x = rs_x[i];
                w.y = rs_y[i];
                w.yaw = rs_yaw[i];
                w.linear_velocity = rs_directions[i] * speed;
                w.steering_angle = rs_steers[i];
                waypoints.push_back(w);
            }
        }
        return waypoints;
    }

public:
    // robot, goal, timetable, entities, params, is_transfer, obj_name, start_time
    PHAStar(RobotMeta *r, const Pose &goal_pose, TimeTable *tt, const std::unordered_map<std::string, EntityMeta *> *ents,
            const Params &p, bool trans = false, const std::string &obj_name = "", double start_t = 0.0,
            const std::string &debug_kind = "")
        : robot(r), timetable(tt), entities(ents), is_transfer(trans), debug_plan_kind(debug_kind), params(p)
    {
        // Update initial pose if chaining (but for now, assume caller updates r->initial_pose if needed)
        start = std::make_unique<Node>(r->initial_pose.x, r->initial_pose.y, r->initial_pose.yaw, start_t, 0.0, 0.0, nullptr, 1);
        goal = std::make_unique<Node>(goal_pose.x, goal_pose.y, goal_pose.yaw, 0.0, 0.0, 0.0, nullptr, 1);

        auto it = entities->find(obj_name);
        if (is_transfer)
        {
            transferred = (it != entities->end()) ? dynamic_cast<ObjectMeta *>(it->second) : nullptr;
        }
        else
        {
            // Handle error
            transferred = nullptr;

            if (!obj_name.empty() && it != entities->end())
            {
                ignored_entity = it->second;
            }
        }

        wheel_base = robot->wheel_base;
        min_turn_radius = robot->min_turning_radius;
        max_curvature = 1.0 / min_turn_radius;
        max_steer = std::atan(wheel_base * max_curvature);
        speed = is_transfer ? robot->speed_transfer : robot->speed_transit;
        params.movement_length = speed * params.time_step;

        if (is_transfer)
        {
            motion_primitives = {
                {1, 0.0}, {1, max_steer}, {1, -max_steer}, {1, max_steer / 2}, {1, -max_steer / 2}, {0, 0.0}};
        }
        else
        {
            motion_primitives = {
                {1, 0.0}, {1, max_steer}, {1, -max_steer}, {1, max_steer / 2}, {1, -max_steer / 2}, {-1, 0.0}, {-1, max_steer}, {-1, -max_steer}, {-1, max_steer / 2}, {-1, -max_steer / 2}, {0, 0.0}};
        }

        x_width = static_cast<int>((params.max_x - params.min_x) / params.xy_resolution) + 1;
        y_width = static_cast<int>((params.max_y - params.min_y) / params.xy_resolution) + 1;
        theta_width = static_cast<int>(2 * M_PI / params.yaw_resolution);
        time_width = static_cast<int>(params.max_time / params.time_resolution) + 1;

        entity_diags.reserve(entities->size());
        for (const auto &[name, ent] : *entities)
        {
            if (ent == robot || ent == transferred || ent == ignored_entity)
                continue;
            double diag = collision_origin_radius(ent->size);
            entity_diags.push_back(diag);
        }

        // params.analytic_threshold = 5.0 * min_turn_radius;
        start_clock = std::chrono::high_resolution_clock::now();
    }

    // Check if a pose collides with any entity at given time (returns colliding entity name or "")
    std::string check_pose_collision(const Pose &pose, double check_time, bool include_transferred = false)
    {
        Corners pose_bounds_corners = get_corners(
            pose.x, pose.y, pose.yaw,
            robot->size.front_length, robot->size.rear_length, robot->size.width);

        // Check bounds first
        if (!is_in_bounds(pose_bounds_corners, params.min_x, params.max_x, params.min_y, params.max_y))
        {
            return "map_bounds";
        }

        Corners pose_corners = get_collision_corners_for_type(
            pose, EntityType::ROBOT, robot->size, params);

        // Check against timetable entities
        auto current_poses = timetable->get_poses(check_time);
        for (const auto &[ent, ent_pose] : current_poses)
        {
            if (ent == robot)
                continue; // Skip self
            if (!include_transferred && ent == transferred)
                continue; // Skip object if pushing

            Corners ent_corners = get_collision_corners_for_type(
                ent_pose, ent->type, ent->size, params);
            if (rectangles_intersect(pose_corners, ent_corners))
            {
                return ent->name; // Return colliding entity name
            }
        }
        return ""; // No collision
    }

    bool is_valid_transfer_contact_pose(const Pose &robot_pose,
                                        const ObjectMeta *obj,
                                        const Pose &obj_pose) const
    {
        if (!obj)
            return false;

        double dx = obj_pose.x - robot_pose.x;
        double dy = obj_pose.y - robot_pose.y;

        double fx = std::cos(robot_pose.yaw);
        double fy = std::sin(robot_pose.yaw);
        double forward = dx * fx + dy * fy;
        double lateral = -dx * fy + dy * fx;

        double expected = robot->size.front_length + obj->size.rear_length;
        if (std::abs(forward - expected) > 0.2)
            return false;
        if (std::abs(lateral) > 0.2)
            return false;

        return true;
    }

    bool is_valid_start_contact_exception(const std::string &collider_name,
                                          double start_t) const
    {
        if (is_transfer || collider_name.empty())
            return false;

        auto ent_it = entities->find(collider_name);
        if (ent_it == entities->end())
            return false;

        EntityMeta *ent = ent_it->second;
        if (!ent || ent->type != EntityType::OBJECT)
            return false;

        auto poses = timetable->get_poses(start_t);
        auto pose_it = poses.find(ent);
        if (pose_it == poses.end())
            return false;

        return is_valid_transfer_contact_pose(robot->initial_pose,
                                              dynamic_cast<ObjectMeta *>(ent),
                                              pose_it->second);
    }

    // Validate start/goal and return detailed result
    PlanningResult validate_start_goal(double start_t, const Pose &goal_pose)
    {
        PlanningResult res;

        // Start validation (at current time)
        std::string start_collider = check_pose_collision(robot->initial_pose, start_t, is_transfer);
        if (start_collider == "map_bounds")
        {
            res.status = PlanningStatus::START_OUT_OF_BOUNDS;
            res.failure_detail = "Start pose out of map bounds";
            return res;
        }
        if (!start_collider.empty())
        {
            if (is_valid_start_contact_exception(start_collider, start_t))
            {
                // Allow immediate post-transfer contact at start for non-transfer transit.
            }
            else
            {
                res.status = PlanningStatus::START_INVALID_COLLISION;
                res.failure_detail = "Start pose collides with " + start_collider;
                res.colliding_entity = start_collider;
                res.failure_time = start_t;
                return res;
            }
        }

        // Goal validation (dynamic-aware): enforce bounds, reject only static occupancy.
        Corners goal_corners = get_corners(goal_pose.x, goal_pose.y, goal_pose.yaw,
                                           robot->size.front_length, robot->size.rear_length, robot->size.width);
        if (!is_in_bounds(goal_corners, params.min_x, params.max_x, params.min_y, params.max_y))
        {
            res.status = PlanningStatus::GOAL_OUT_OF_BOUNDS;
            res.failure_detail = "Goal pose out of map bounds";
            return res;
        }

        std::string goal_collider_now = check_pose_collision(goal_pose, start_t, is_transfer);
        if (!goal_collider_now.empty() && goal_collider_now != "map_bounds")
        {
            auto ent_it = entities->find(goal_collider_now);
            if (ent_it != entities->end())
            {
                EntityMeta *coll_ent = ent_it->second;
                // If the blocker is dynamic, let conflict resolution handle it (wait/relocate/detour).
                if (timetable->is_entity_static_after(start_t, coll_ent))
                {
                    res.status = PlanningStatus::GOAL_INVALID_COLLISION;
                    res.failure_detail = "Goal pose statically occupied by " + goal_collider_now;
                    res.colliding_entity = goal_collider_now;
                    res.failure_time = start_t;
                    return res;
                }
            }
        }

        res.status = PlanningStatus::SUCCESS; // Valid
        return res;
    }

    PlanningResult Planning_with_res(double check_time = 0.0)
    {
        PlanningResult res;
        auto is_soft_robot_block = [](const CollisionInfo &info)
        {
            return (info.reason.find("Robot") != std::string::npos) &&
                   (info.entity_name != "Boundary");
        };

        // Step 1: Quick start/goal validation
        PlanningResult validation = validate_start_goal(check_time, Pose(goal->x, goal->y, goal->yaw));
        if (validation.status != PlanningStatus::SUCCESS)
        {
            return validation;
        }

        std::vector<Node> local_explored; // for debug

        using PQElem = std::tuple<double, uint64_t, Node *>;
        auto cmp = [](const PQElem &a, const PQElem &b)
        { return std::get<0>(a) > std::get<0>(b) || (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) > std::get<1>(b)); };
        std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
        uint64_t node_id = 0;
        size_t start_id = calc_grid_index(start.get());
        std::unordered_map<size_t, double> g_costs;
        g_costs[start_id] = 0.0;
        open_set.emplace(calc_f(start.get()), node_id++, start.get());
        std::unordered_set<size_t> closed_set;

        size_t iteration = 0; // for debug
        static int debug_counter = 0;

        // Clear previous backup
        backup_result.status = PlanningStatus::INTERNAL_ERROR;
        backup_result.waypoints.clear();

        while (!open_set.empty())
        {
            iteration++;
            if (iteration % 1000 == 0)
            {
                std::cout << "A* iteration: " << iteration << ", open_set size: " << open_set.size()
                          << ", current f-cost: " << std::get<0>(open_set.top()) << std::endl;
            }

            // Early exit if search is taking too long
            if (iteration > (size_t)max_search_iterations)
            {
                if (backup_result.status == PlanningStatus::BLOCKED_BY_ROBOT)
                {
                    std::cout << "Hit iteration limit (" << max_search_iterations << ") with backup path. Returning backup." << std::endl;
                    return backup_result;
                }
                else
                {
                    std::cout << "Hit iteration limit (" << max_search_iterations << ") with NO backup. Aborting." << std::endl;
                    PlanningResult fail_res;
                    fail_res.status = PlanningStatus::NO_PATH_FOUND;
                    fail_res.failure_detail = "Search iteration limit exceeded";
                    return fail_res;
                }
            }

            auto [f, _, current] = open_set.top();
            open_set.pop();
            // for debug
            local_explored.push_back(*current);

            // for debug
            // std::cout << "DEBUG_VISITED," << current->x << "," << current->y << ","
            //         << current->yaw << "," << current->t << "," << current->cost << std::endl;

            size_t n_id = calc_grid_index(current);
            if (closed_set.count(n_id))
                continue;
            if (current->cost > g_costs[n_id])
                continue; // Outdated entry
            closed_set.insert(n_id);

            // for debug
            if (iteration % 5000 == 0)
            {
                std::cout << "  Current node: t=" << current->t << ", x=" << current->x << ", y=" << current->y << ", yaw=" << current->yaw
                          << ", cost=" << current->cost << std::endl;
            }

            auto is_collision_free = check_collision_at(current);
            if (!is_collision_free.is_valid)
                continue;

            auto [rs_path, rs_length] = analytic_expand(current);
            if (!rs_path.empty())
            {
                CollisionInfo rs_info = check_collision_along_rs(rs_path, current->t);
                if (rs_info.is_valid)
                {
                    auto waypoints = extract_path(current, rs_path);
                    double arrival_t = waypoints.back().time;

                    // Post-arrival check
                    bool post_safe = true;
                    bool post_blocked_by_robot = false;
                    CollisionInfo post_arrival_collision;
                    double last_timetable_time = timetable->get_max_time();
                    for (double future_t = arrival_t + params.time_step; future_t <= last_timetable_time; future_t += params.time_step)
                    {
                        Node dummy(goal->x, goal->y, goal->yaw, future_t, 0.0, 0.0, nullptr, 0);
                        auto collision_result = check_collision_at(&dummy);
                        if (!collision_result.is_valid)
                        {
                            post_safe = false;
                            post_arrival_collision = collision_result;
                            if (is_soft_robot_block(collision_result))
                            {
                                post_blocked_by_robot = true;
                            }
                            break;
                        }
                    }

                    if (post_safe)
                    {
                        std::cout << "Goal found with analytic expansion!" << std::endl;
                        return {waypoints, PlanningStatus::SUCCESS, ""};
                    }
                    else if (post_blocked_by_robot)
                    {
                        // Blocked at goal by a robot. Save as backup.
                        if (backup_result.status != PlanningStatus::BLOCKED_BY_ROBOT)
                        {
                            backup_result.waypoints = waypoints;
                            backup_result.status = PlanningStatus::BLOCKED_BY_ROBOT;
                            backup_result.colliding_entity = post_arrival_collision.entity_name;
                            backup_result.failure_detail = "Blocked at goal by " + post_arrival_collision.entity_name;
                            backup_result.failure_time = post_arrival_collision.time;
                        }
                    }
                    else // Hard collision in post-safe check
                    {
                        static int post_fail_counter = 0;
                        if (DEBUG_VIS && post_fail_counter++ < 2)
                        {
                            std::cout << "\n[DEBUG TRAP] Analytic Post-Arrival Rejected (Hard Collision)" << std::endl;
                            std::cout << "  > Reason: " << post_arrival_collision.reason << std::endl;
                            std::cout << "  > Entity: " << post_arrival_collision.entity_name << std::endl;
                        }

                        // Construct a temporary Failed Result to visualize
                        PlanningResult failed_res;
                        failed_res.waypoints = waypoints;
                        failed_res.status = PlanningStatus::NO_PATH_FOUND;
                        failed_res.failure_detail =
                            "Analytic post-arrival collision with " +
                            post_arrival_collision.entity_name + " (" +
                            post_arrival_collision.reason + ")";
                        failed_res.colliding_entity =
                            post_arrival_collision.entity_name;
                        failed_res.failure_time = post_arrival_collision.time;

                        auto current_pose = Pose(current->x, current->y, current->yaw);
                        auto goal_pose = Pose(goal->x, goal->y, goal->yaw);

                        if (DEBUG_VIS && !suppress_debug_popup)
                        {
                            // Call the visualizer immediately
                            visualize_planning_debug(
                                *timetable,
                                robot,
                                failed_res,
                                post_arrival_collision.time,
                                current_pose,
                                goal_pose,
                                params,
                                "",
                                debug_plan_kind.empty() ? (is_transfer ? "Transfer" : "Transit")
                                                        : debug_plan_kind);
                        }
                    }
                }
                else // CollisionInfo (rs_info) not valid
                {
                    if (is_soft_robot_block(rs_info))
                    {
                        // This path is blocked by a robot, but geometerically valid.
                        if (backup_result.status != PlanningStatus::BLOCKED_BY_ROBOT)
                        {
                            backup_result.waypoints = extract_path(current, rs_path);
                            backup_result.status = PlanningStatus::BLOCKED_BY_ROBOT;
                            backup_result.colliding_entity = rs_info.entity_name;
                            backup_result.failure_detail = "Blocked by " + rs_info.entity_name;
                            backup_result.failure_time = rs_info.time;
                        }
                    }
                    else
                    {
                        // Hard collision logic
                        static int rs_fail_counter = 0;
                        if (DEBUG_VIS && rs_fail_counter++ < 2)
                        {
                            std::cout << "\n[DEBUG TRAP] Analytic Path Rejected (Hard Collision) at t=" << rs_info.time << " due to " << rs_info.entity_name << std::endl;
                            // ... can add visualization here if needed, keeping it simple for now
                        }
                    }
                }
            }

            // Goal check (assuming near-goal threshold)
            double dist_to_goal = std::hypot(current->x - goal->x, current->y - goal->y);
            double dyaw_to_goal = std::fabs(mod2pi(current->yaw - goal->yaw));
            if (dist_to_goal < params.xy_resolution && dyaw_to_goal < params.yaw_resolution)
            {
                auto waypoints = extract_path(current, {});
                double arrival_t = waypoints.back().time;

                bool post_safe = true;
                double last_timetable_time = timetable->get_max_time();
                for (double future_t = arrival_t + params.time_step; future_t <= last_timetable_time; future_t += params.time_step)
                {
                    Node dummy(goal->x, goal->y, goal->yaw, future_t, 0.0, 0.0, nullptr, 0);
                    auto is_collision_free = check_collision_at(&dummy);
                    if (!is_collision_free.is_valid)
                    {
                        post_safe = false;
                        break;
                    }
                }

                if (post_safe)
                {
                    std::cout << "Goal found!" << std::endl;
                    // return {waypoints, PlanningStatus::SUCCESS, ""};
                    res.waypoints = waypoints;
                    res.status = PlanningStatus::SUCCESS;
                    return res;
                }
                // Else continue
            }

            for (auto prim : motion_primitives)
            {
                Node *new_node = generate_node(current, prim);
                if (!new_node)
                    continue;
                if (!check_collision_along_path(current, new_node, prim))
                    continue;
                size_t new_id = calc_grid_index(new_node);
                if (closed_set.count(new_id))
                    continue;
                double new_g = new_node->cost;
                auto it = g_costs.find(new_id);
                if (it != g_costs.end() && new_g >= it->second)
                    continue; // Worse or equal
                g_costs[new_id] = new_g;
                open_set.emplace(new_g + calc_heuristic(new_node), node_id++, new_node);
            }
        }

        if (backup_result.status == PlanningStatus::BLOCKED_BY_ROBOT)
        {
            std::cout << "Returning backup path blocked by " << backup_result.colliding_entity << std::endl;
            return backup_result;
        }

        std::cout << "No path found" << std::endl;
        res.status = PlanningStatus::NO_PATH_FOUND;
        res.failure_detail = "Search exhausted without reaching goal.";
        res.explored_nodes = std::move(local_explored); // Move to result on failure
        // return {};
        return res;
    }

    void set_ignore_other_robots(bool ignore)
    {
        ignore_other_robots = ignore;
    }

    void set_debug_popup_enabled(bool enabled)
    {
        suppress_debug_popup = !enabled;
    }

    // this version is to be deprecated
    std::vector<Waypoint> planning()
    {
        using PQElem = std::tuple<double, uint64_t, Node *>;
        auto cmp = [](const PQElem &a, const PQElem &b)
        { return std::get<0>(a) > std::get<0>(b) || (std::get<0>(a) == std::get<0>(b) && std::get<1>(a) > std::get<1>(b)); };
        std::priority_queue<PQElem, std::vector<PQElem>, decltype(cmp)> open_set(cmp);
        uint64_t node_id = 0;
        size_t start_id = calc_grid_index(start.get());
        std::unordered_map<size_t, double> g_costs;
        g_costs[start_id] = 0.0;
        open_set.emplace(calc_f(start.get()), node_id++, start.get());
        std::unordered_set<size_t> closed_set;

        size_t iteration = 0; // for debug
        while (!open_set.empty())
        {
            iteration++;
            if (iteration % 1000 == 0)
            {
                std::cout << "A* iteration: " << iteration << ", open_set size: " << open_set.size()
                          << ", current f-cost: " << std::get<0>(open_set.top()) << std::endl;
            }

            auto [f, _, current] = open_set.top();
            open_set.pop();

            size_t n_id = calc_grid_index(current);
            if (closed_set.count(n_id))
                continue;
            if (current->cost > g_costs[n_id])
                continue; // Outdated entry
            closed_set.insert(n_id);

            // for debug
            if (iteration % 5000 == 0)
            {
                std::cout << "  Current node: t=" << current->t << ", x=" << current->x << ", y=" << current->y << ", yaw=" << current->yaw
                          << ", cost=" << current->cost << std::endl;
            }

            auto is_collision_free = check_collision_at(current);
            if (!is_collision_free.is_valid)
                continue;

            auto [rs_path, rs_length] = analytic_expand(current);
            if (!rs_path.empty())
            {
                CollisionInfo rs_info = check_collision_along_rs(rs_path, current->t);
                if (rs_info.is_valid)
                {
                    auto waypoints = extract_path(current, rs_path);
                    double arrival_t = waypoints.back().time;

                    // Post-arrival check
                    bool post_safe = true;
                    for (double future_t = arrival_t + params.time_step; future_t <= params.max_time; future_t += params.time_step)
                    {
                        Node dummy(goal->x, goal->y, goal->yaw, future_t, 0.0, 0.0, nullptr, 0);
                        auto is_collision_free = check_collision_at(&dummy);
                        if (!is_collision_free.is_valid)
                        {
                            post_safe = false;
                            break;
                        }
                    }

                    if (post_safe)
                    {
                        std::cout << "Goal found with analytic expansion!" << std::endl;
                        return waypoints;
                    }
                    // Else continue searching
                    else
                    {
                        std::cout << "Analytic (Reeds-Shepp) path generated but rejected due to collision or bounds violation" << std::endl;
                    }
                }
            }

            // Goal check (assuming near-goal threshold)
            double dist_to_goal = std::hypot(current->x - goal->x, current->y - goal->y);
            double dyaw_to_goal = std::fabs(mod2pi(current->yaw - goal->yaw));
            if (dist_to_goal < params.xy_resolution && dyaw_to_goal < params.yaw_resolution)
            {
                auto waypoints = extract_path(current, {});
                double arrival_t = waypoints.back().time;

                bool post_safe = true;
                for (double future_t = arrival_t + params.time_step; future_t <= params.max_time; future_t += params.time_step)
                {
                    Node dummy(goal->x, goal->y, goal->yaw, future_t, 0.0, 0.0, nullptr, 0);
                    auto is_collision_free = check_collision_at(&dummy);
                    if (!is_collision_free.is_valid)
                    {
                        post_safe = false;
                        break;
                    }
                }

                if (post_safe)
                {
                    std::cout << "Goal found!" << std::endl;
                    return waypoints;
                }
                // Else continue
            }

            for (auto prim : motion_primitives)
            {
                Node *new_node = generate_node(current, prim);
                if (!new_node)
                    continue;
                if (!check_collision_along_path(current, new_node, prim))
                    continue;
                size_t new_id = calc_grid_index(new_node);
                if (closed_set.count(new_id))
                    continue;
                double new_g = new_node->cost;
                auto it = g_costs.find(new_id);
                if (it != g_costs.end() && new_g >= it->second)
                    continue; // Worse or equal
                g_costs[new_id] = new_g;
                open_set.emplace(new_g + calc_heuristic(new_node), node_id++, new_node);
            }
        }

        std::cout << "No path found" << std::endl;
        return {};
    }
};

std::vector<Trajectory> perform_planning(
    const std::unordered_map<std::string, EntityMeta *> &entities,
    const std::vector<std::tuple<std::string, Pose, bool, std::string, double>> &robot_plans,
    TimeTable &timetable,
    const Params &params,
    const bool print_path = false)
{
    std::vector<Trajectory> all_trajectories;
    std::unordered_map<std::string, double> robot_current_times; // Track last arrival per robot for chaining
    std::unordered_map<std::string, Pose> robot_current_poses;   // Track last pose per robot

    for (auto &plan : robot_plans)
    {
        auto [r_name, goal_pose, trans, obj_name, provided_start_t] = plan;
        RobotMeta *r = dynamic_cast<RobotMeta *>(entities.at(r_name));
        double current_start_t = (provided_start_t > 0.0) ? provided_start_t : robot_current_times[r_name]; // Use provided if >0, else dynamic
        Pose start_pose = robot_current_poses.count(r_name) ? robot_current_poses[r_name] : r->initial_pose;
        r->initial_pose = start_pose; // Temporarily set for planner

        std::cout << "Starting planning for " << r_name << (trans ? " transfer with " + obj_name : " transit")
                  << ": start_t=" << current_start_t
                  << ", start_pose=(x=" << start_pose.x << ", y=" << start_pose.y << ", yaw=" << start_pose.yaw << ")"
                  << ", goal_pose=(x=" << goal_pose.x << ", y=" << goal_pose.y << ", yaw=" << goal_pose.yaw << ")" << std::endl;

        PHAStar planner(r, goal_pose, &timetable, &entities, params, trans, obj_name, current_start_t);
        auto start_time = std::chrono::high_resolution_clock::now();
        auto waypoints = planner.planning();
        auto end_time = std::chrono::high_resolution_clock::now();

        // add final push
        double delta_t = params.final_push_distance / r->speed_transit;
        auto final_push_pose = offsetPose(waypoints.back(), params.final_push_distance);
        auto final_push_wpt = Waypoint(final_push_pose);
        final_push_wpt.time = waypoints.back().time + delta_t;
        final_push_wpt.linear_velocity = waypoints.back().linear_velocity;
        waypoints.push_back(final_push_wpt);

        // Fix double time offset: make waypoint times relative to trajectory start
        for (auto &wp : waypoints)
        {
            wp.time -= current_start_t;
        }
        std::chrono::duration<double> planning_time = end_time - start_time;
        std::cout << "Planning time for " << r_name << ": " << planning_time.count() << " seconds" << std::endl;

        if (!waypoints.empty())
        {
            std::cout << "Last waypoint for " << r_name << ": time=" << waypoints.back().time << ", x=" << waypoints.back().x << ", y=" << waypoints.back().y << ", yaw=" << waypoints.back().yaw << std::endl;

            if (print_path)
            {
                for (const auto &wp : waypoints)
                {
                    std::cout << "time=" << wp.time << ", x=" << wp.x << ", y=" << wp.y << ", yaw=" << wp.yaw << std::endl;
                }
            }

            Trajectory traj;
            traj.entity = r;
            traj.start_time = current_start_t;
            traj.waypoints = waypoints;
            traj.is_transfer = trans;
            traj.transferred_object = trans ? entities.at(obj_name) : nullptr;
            all_trajectories.push_back(traj);
            timetable.add_trajectory(traj);

            // Update for potential next plan for this robot
            robot_current_times[r_name] = current_start_t + waypoints.back().time;
            robot_current_poses[r_name] = {waypoints.back().x, waypoints.back().y, waypoints.back().yaw};
        }
        else
        {
            std::cout << "Failed to find a path for " << r_name << std::endl;
            // Continue to next plan, but note failure
        }
    }

    return all_trajectories;
}

#endif // PHASTAR_H
